#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_LTE_UART_DFU lte_uart_dfu.c)

zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/dfu/include)