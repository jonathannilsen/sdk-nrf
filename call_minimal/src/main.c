/*
 * Copyright (c) 2026 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdint.h>
#include <zephyr/ztest.h>
#include <ironside/se/api.h>

ZTEST(call_minimal, test_call_minimal_driver_works)
{
	struct ironside_se_periphconf_status status;
	struct periphconf_entry entries[] = {
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(200) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(201) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(202) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(203) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(204) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(205) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(206) },
		{ PERIPHCONF_IRQMAP_IRQ_SINK_REGPTR(207) },
	};

	BUILD_ASSERT(sizeof(entries) > IRONSIDE_SE_PERIPHCONF_INLINE_READ_MAX_COUNT);

	status = ironside_se_periphconf_read(entries, IRONSIDE_SE_PERIPHCONF_INLINE_READ_MAX_COUNT);
	printk("inline read status: %d\n", status.status);
	zassert_equal(status.status, 0);

	printk("entries:\n");
	for (int i = 0; i < IRONSIDE_SE_PERIPHCONF_INLINE_READ_MAX_COUNT; i++) {
		printk("  %d | 0x%08x: 0x%08x\n", i, entries[i].regptr, entries[i].value);
		entries[i].value = 0;
	}

	status = ironside_se_periphconf_read(entries, ARRAY_SIZE(entries));
	printk("buffer read status: %d\n", status.status);
	zassert_equal(status.status, 0);

	printk("entries:\n");
	for (int i = 0; i < ARRAY_SIZE(entries); i++) {
		printk("  %d | 0x%08x: 0x%08x\n", i, entries[i].regptr, entries[i].value);
		entries[i].value = 0;
	}
}

ZTEST_SUITE(call_minimal, NULL, NULL, NULL, NULL, NULL);
