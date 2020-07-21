#include <zephyr.h>
#include <sys/crc.h>
#include <sys/printk.h>
#include <string.h>
#include <lte_uart_dfu.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>

#include "sfts.h"

static u32_t current_crc;

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

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SFTS_UUID_SERVICE),
};

void main(void)
{
	int err;

	printk("test_lte_uart_dfu_lib sample started\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth enabling failed (err %d)\n", err);
		return;
	}

	err = bt_gatt_stfs_init(&stfs_callbacks);
	if (err) {
		printk("SFTS initialization failed (err %d)\n", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	lte_uart_dfu_init();
	lte_uart_dfu_start();
}
