/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ironside_se_smp_svr, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("IronSide SE SMP server ready (build: " __DATE__ " " __TIME__ ")");
	return 0;
}
