#include <zephyr.h>
#include <sys/crc.h>
#include <sys/printk.h>
#include <sys/ring_buffer.h>
#include <string.h>
#include <dfu/mcuboot.h>
#include <uart_dfu.h>
#include <uart_dfu_target_server.h>
#include <dfu/dfu_target.h>
#include <dfu/mcuboot.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include "sfts.h"

#define RETURN_ON_ERROR(_err, _msg)		\
if (_err) {					\
	printk("%s (err %d)\n", _msg, _err);	\
	return;					\
}

/* Taken from dfu_target.c */
#define IDENTIFY_BUF_SIZE 32

/* Data buffer for identifying DFU image type. */
static u8_t id_buf[IDENTIFY_BUF_SIZE];
static u16_t id_buf_len;

/* Context for DFU over BLE using SFTS. */
static struct {
	enum {
		SFTS_DFU_STATE_IDLE,
		SFTS_DFU_STATE_WAITING_FOR_FIRST_SEGMENT,
		SFTS_DFU_STATE_TRANSFER_IN_PROGRESS,
#if CONFIG_DFU_THREAD
		SFTS_DFU_STATE_TRANSFER_ABORTING,
#endif
	} state;
	struct bt_conn *conn;
	u32_t file_size;
	u32_t current_crc;
#if CONFIG_DFU_THREAD
	struct ring_buf rbuf;
	u8_t rbuf_internal[CONFIG_DFU_THREAD_BUF_SIZE];
	struct k_sem sem_rbuf;
	struct k_sem sem_exit;
	struct k_thread thread;
	bool thread_alive;
	bool thread_exit;
#endif
} sfts_dfu_ctx;

#if CONFIG_DFU_THREAD
K_THREAD_STACK_DEFINE(dfu_thread_stack, CONFIG_DFU_THREAD_STACK_SIZE);
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SFTS_UUID_SERVICE),
};

static struct uart_dfu_target_server target_server;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void identify_buf_append(const u8_t *data, u16_t len)
{
	len = MIN(len, IDENTIFY_BUF_SIZE - id_buf_len);
	memcpy(&id_buf[id_buf_len], data, len);
}

static int image_identify(const u8_t *data, u16_t len)
{
	int img_type;

	/* Get DFU image type of the first segment.
	 * Use identify buf if dfu_target_img_type returned -EAGAIN before.
	 */
	if (!id_buf_len) {
		img_type = dfu_target_img_type(data, len);
	} else {
		identify_buf_append(data, len);
		img_type = dfu_target_img_type(id_buf, id_buf_len);
	}

	if (img_type == -EAGAIN) {
		/* More data is needed before the image can be identified.
		 * Temporarily store first segment in identify buf.
		 */
		if (!id_buf_len) {
			identify_buf_append(data, len);
		}
	}

	id_buf_len = 0;
	return img_type;
}

static int sfts_dfu_start(int img_type)
{
	int err;
	bool reset;
	size_t offset;

	for (reset = false; ; reset = true) {
		err = dfu_target_init(img_type, sfts_dfu_ctx.file_size, NULL);
		if ((err < 0) && (err != -EBUSY)) {
			(void) dfu_target_reset();
			return err;
		}

		err = dfu_target_offset_get(&offset);
		if (err) {
			return err;
		}

		if (offset > 0) {
			/* SFTS does not support starting download from offset.
			 * Reset and reinitialize target on next iteration.
			 */
			if (reset) {
				/* Target was reset on previous iteration. */
				__ASSERT(0, "DFU target offset is "
					    "nonzero after reset");
				return -EFAULT;
			}

			err = dfu_target_reset();
			if (err) {
				return err;
			}
		} else {
			/* Ready for transfer. */
			return 0;
		}
	}
}

static int sfts_dfu_stop(bool successful)
{
	int err = dfu_target_done(successful);

	/* Context reset. */
	bt_conn_unref(sfts_dfu_ctx.conn);
	sfts_dfu_ctx.conn = NULL;
	sfts_dfu_ctx.state = SFTS_DFU_STATE_IDLE;

	return err;
}

#if CONFIG_DFU_THREAD
/* This function is to be called within DFU thread. */
static void sfts_dfu_thread_cleanup(void)
{
	/* Wake up the thread from which 'sfts_dfu_thread_exit' was called. */
	k_sem_give(&sfts_dfu_ctx.sem_exit);
	k_yield();

	/* Reset rbuf in this thread because memset is costly. */
	ring_buf_reset(&sfts_dfu_ctx.rbuf);
	sfts_dfu_ctx.thread_alive = false;
}

/**
 * @brief DFU thread entry function.
 * @details Used for calling dfu_target API outside of BT RX thread.
 *
 * @param p1	Image type passed from @a first_image_segment_in.
 * @param p2	Unused.
 * @param p3	Unused.
 */
static void sfts_dfu_thread(void *p1, void *p2, void *p3)
{
	int err;
	u8_t *data;
	u32_t len;

	err = sfts_dfu_start((int) p1);
	if (err) {
		sfts_dfu_ctx.state = SFTS_DFU_STATE_TRANSFER_ABORTING;
		sfts_dfu_thread_cleanup();
		return;
	}

	while (!sfts_dfu_ctx.thread_exit) {
		/* Wait until more data is received and copied. */
		k_sem_take(&sfts_dfu_ctx.sem_rbuf, K_FOREVER);

		for (;;) {
			/* Fetch as much contiguous data as possible. */
			len = ring_buf_get_claim(&sfts_dfu_ctx.rbuf, &data,
						 CONFIG_DFU_THREAD_BUF_SIZE);
			if (!len) {
				break;
			}

			err = dfu_target_write(data, len);
			if (!err) {
				err = ring_buf_get_finish(&sfts_dfu_ctx.rbuf,
							  len);
				if (!err) {
					/* Success state. */
					k_yield();
					continue;
				}
			}

			/* Failure state. */
			sfts_dfu_ctx.state = SFTS_DFU_STATE_TRANSFER_ABORTING;
			sfts_dfu_thread_cleanup();
			return;
		}
	}

	/* Exit signaled from BT RX thread. */
	sfts_dfu_thread_cleanup();
}

/* This function is to be called outside of DFU thread. */
static void sfts_dfu_thread_exit(void)
{
	printk("Terminating DFU thread\n");

	/* Set flag to signal a graceful exit out of thread loop. */
	sfts_dfu_ctx.thread_exit = true;

	/* Wake up thread in case it is stuck waiting for semaphore. */
	k_sem_give(&sfts_dfu_ctx.sem_rbuf);

	/* Wait until thread exits loop. */
	k_sem_take(&sfts_dfu_ctx.sem_exit, K_FOREVER);
}
#endif /* CONFIG_DFU_THREAD */

static int next_image_segment_in(const u8_t *data, u16_t len)
{
#if CONFIG_DFU_THREAD
	/* Pipe data to DFU thread. */
	u32_t put_len = ring_buf_put(&sfts_dfu_ctx.rbuf, data, len);
	if (put_len < len) {
		/* DFU thread is not keeping up. Cancel transfer. */
		sfts_dfu_thread_exit();
		return -ECANCELED;
	} else {
		k_sem_give(&sfts_dfu_ctx.sem_rbuf);
	}
#else
	/* Process data in this thread. */
	int err = dfu_target_write(data, len);
	if (err) {
		return -ECANCELED;
	}
#endif

	sfts_dfu_ctx.current_crc = crc32_ieee_update(sfts_dfu_ctx.current_crc,
						     data, len);
	return 0;
}

static int first_image_segment_in(const u8_t *data, u16_t len)
{
	int img_type = image_identify(data, len);
	if (img_type == -EAGAIN) {
		/* Wait for more data. */
		return 0;
	} else if (img_type < 0) {
		printk("DFU image type unknown\n");
		return -ECANCELED;
	}

	printk("DFU image type: %d\n", img_type);

#if CONFIG_DFU_THREAD
	/* Set up a separate thread for starting DFU. */
	__ASSERT(!sfts_dfu_ctx.thread_alive,
		 "Old DFU thread was not terminated properly");
	__ASSERT(ring_buf_is_empty(&sfts_dfu_ctx.rbuf),
		 "sfts_dfu_ctx.rbuf was not init/reset before transfer");

	k_sem_reset(&sfts_dfu_ctx.sem_rbuf);
	k_sem_reset(&sfts_dfu_ctx.sem_exit);

	sfts_dfu_ctx.thread_alive = true;
	sfts_dfu_ctx.thread_exit = false;

	printk("Starting DFU thread\n");
	k_thread_create(&sfts_dfu_ctx.thread, dfu_thread_stack,
			K_THREAD_STACK_SIZEOF(dfu_thread_stack),
			sfts_dfu_thread, (void *) img_type, NULL, NULL,
			CONFIG_DFU_THREAD_PRIO, 0, K_NO_WAIT);
#else
	/* Start DFU in this thread. */
	int err = sfts_dfu_start(img_type);
	if (err) {
		return -ECANCELED;
	}
#endif

	sfts_dfu_ctx.state = SFTS_DFU_STATE_TRANSFER_IN_PROGRESS;
	return next_image_segment_in(data, len);
}

/*****************************************************************************
 * SFTS event callbacks
 *****************************************************************************/

static int on_sfts_new(struct bt_conn *conn, const u32_t file_size)
{
	__ASSERT_NO_MSG(sfts_dfu_ctx.state == SFTS_DFU_STATE_IDLE);
	__ASSERT_NO_MSG(sfts_dfu_ctx.conn != conn);

	if (sfts_dfu_ctx.conn == NULL) {
#if CONFIG_DFU_THREAD
		if (sfts_dfu_ctx.thread_alive) {
			printk("Old DFU thread appears to be still running\n");
			return -ECANCELED;
		}
#endif
		sfts_dfu_ctx.state = SFTS_DFU_STATE_WAITING_FOR_FIRST_SEGMENT;
		sfts_dfu_ctx.conn = bt_conn_ref(conn);
		sfts_dfu_ctx.file_size = file_size;
		sfts_dfu_ctx.current_crc = 0;

		printk("DFU transfer started (file size: %d)\n", file_size);
		return 0;
	}
	return -ECANCELED;
}

static int on_sfts_data(struct bt_conn *conn, const u8_t *data, u16_t len)
{
	__ASSERT_NO_MSG(sfts_dfu_ctx.conn == conn);

	switch (sfts_dfu_ctx.state) {
	case SFTS_DFU_STATE_WAITING_FOR_FIRST_SEGMENT:
		return first_image_segment_in(data, len);
	case SFTS_DFU_STATE_TRANSFER_IN_PROGRESS:
		return next_image_segment_in(data, len);
#if CONFIG_DFU_THREAD
	case SFTS_DFU_STATE_TRANSFER_ABORTING:
		return -ECANCELED;
#endif
	case SFTS_DFU_STATE_IDLE:
	default:
		/* Should not happen. */
		__ASSERT_NO_MSG(0);
		return -ECANCELED;
	}
}

static int on_sfts_complete(struct bt_conn *conn, const u32_t crc)
{
	__ASSERT_NO_MSG(sfts_dfu_ctx.state != SFTS_DFU_STATE_IDLE);
	__ASSERT_NO_MSG(sfts_dfu_ctx.conn == conn);

#if CONFIG_DFU_THREAD
	sfts_dfu_thread_exit();
#endif

	if (sfts_dfu_ctx.state == SFTS_DFU_STATE_TRANSFER_IN_PROGRESS &&
	    sfts_dfu_ctx.current_crc == crc) {
		int err = sfts_dfu_stop(true);
		if (err) {
			return -ECANCELED;
		} else {
			return 0;
		}
	} else {
		(void) sfts_dfu_stop(false);
		return -ECANCELED;
	}
}

static void on_sfts_abort(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(sfts_dfu_ctx.state != SFTS_DFU_STATE_IDLE);
	__ASSERT_NO_MSG(sfts_dfu_ctx.conn == conn);

#if CONFIG_DFU_THREAD
	sfts_dfu_thread_exit();
#endif

	(void) sfts_dfu_stop(false);
	printk("DFU transfer aborted\n");
}

static const struct bt_gatt_stfs_cb stfs_callbacks = {
	.new_cb = on_sfts_new,
	.data_cb = on_sfts_data,
	.complete_cb = on_sfts_complete,
	.abort_cb = on_sfts_abort,
};

/*****************************************************************************
 * Entry point
 *****************************************************************************/

void main(void)
{
	printk("Starting Peripheral UART DFU example\n");

#if CONFIG_DFU_THREAD
	ring_buf_init(&sfts_dfu_ctx.rbuf, CONFIG_DFU_THREAD_BUF_SIZE,
		      sfts_dfu_ctx.rbuf_internal);

	k_sem_init(&sfts_dfu_ctx.sem_rbuf, 0, 1);
	k_sem_init(&sfts_dfu_ctx.sem_exit, 0, 1);
#endif

	RETURN_ON_ERROR(bt_enable(NULL),
			"Bluetooth enabling failed");
	RETURN_ON_ERROR(bt_gatt_stfs_init(&stfs_callbacks),
			"Service initialization failed");

	printk("Bluetooth initialized\n");

	RETURN_ON_ERROR(uart_dfu_target_server_init(&target_server, 0),
			"Failed to initialize UART DFU server");
	RETURN_ON_ERROR(uart_dfu_target_server_enable(&target_server),
			"Failed to enable UART DFU server");

	printk("UART DFU server enabled\n");

	RETURN_ON_ERROR(bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad,
					ARRAY_SIZE(ad), NULL, 0),
			"Advertising failed to start");

	printk("Advertising successfully started\n");

	boot_write_img_confirmed();
}
