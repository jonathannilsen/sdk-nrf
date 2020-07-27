#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_library()
zephyr_library_sources(uart_dfu.c)
zephyr_library_sources_ifdef(CONFIG_UART_DFU_TARGET_SERVER uart_dfu_target_server.c)
# FIXME: remove
zephyr_library_compile_options("-fdump-rtl-expand")

zephyr_include_directories(.)
zephyr_include_directories(${ZEPHYR_BASE}/../nrf/subsys/dfu/include)