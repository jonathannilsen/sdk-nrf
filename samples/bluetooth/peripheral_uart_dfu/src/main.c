#include <zephyr.h>
#include <sys/crc.h>
#include <sys/printk.h>
#include <string.h>
#include <uart_dfu.h>
#include <uart_dfu_target_server.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>

#include "sfts.h"

#define RETURN_ON_ERROR(_err, _msg)		\
if (_err) {					\
	printk("%s (err %d)\n", _msg, _err);	\
	return;					\
}


static u32_t current_crc;
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SFTS_UUID_SERVICE),
};

int on_sfts_new(struct bt_conn *conn, const u32_t file_size)
{
	printk("New file with size: %u\n", file_size);

	current_crc = 0;
	return 0;
}

int on_sfts_data(struct bt_conn *conn, const u8_t *data, u16_t len)
{
	printk("Received %u bytes: ", len);
	for (u16_t i = 0; i < len; i++) {
		printk("%02X", data[i]);
	}
	printk("\n");

	current_crc = crc32_ieee_update(current_crc, data, len);
	return 0;
}

int on_sfts_complete(struct bt_conn *conn, const u32_t crc)
{
	/* TODO should compute CRC based on what's written to flash */
	if (current_crc == crc) {
		printk("CRC OK - Transfer complete\n");
		return 0;
	}
	printk("CRC error: received %08x, computed %08x\n", crc, current_crc);
	return -EFAULT;
}

void on_sfts_abort(struct bt_conn *conn)
{
	printk("Transfer aborted\n");
}

static const struct bt_gatt_stfs_cb stfs_callbacks = {
	.new_cb = on_sfts_new,
	.data_cb = on_sfts_data,
	.complete_cb = on_sfts_complete,
	.abort_cb = on_sfts_abort,
};


void main(void)
{
	printk("Starting Peripheral UART DFU example\n");

	RETURN_ON_ERROR(bt_enable(NULL),
			"Bluetooth enabling failed");
	RETURN_ON_ERROR(bt_gatt_stfs_init(&stfs_callbacks),
			"Service initialization failed");

	printk("Bluetooth initialized\n");

	RETURN_ON_ERROR(uart_dfu_init(),
			"Failed to initialize UART DFU");
	RETURN_ON_ERROR(uart_dfu_target_server_init(),
			"Failed to initialize UART DFU server");
	RETURN_ON_ERROR(uart_dfu_target_server_enable(),
			"Failed to enable UART DFU server");

	printk("UART DFU server enabled\n");

	RETURN_ON_ERROR(bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad,
					ARRAY_SIZE(ad), NULL, 0),
			"Advertising failed to start");

	printk("Advertising successfully started\n");
}
