#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_library()
zephyr_library_sources(uart_dfu.c)
zephyr_library_sources_ifdef(CONFIG_UART_DFU_CLI uart_dfu_cli.c)
zephyr_library_sources_ifdef(CONFIG_UART_DFU_SRV uart_dfu_srv.c)
zephyr_library_sources_ifdef(CONFIG_UART_DFU_HOST uart_dfu_host.c)
zephyr_include_directories(.)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/dfu/include)
