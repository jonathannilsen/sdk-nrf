#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UART_BLOB_TX uart_blob_tx.c)
zephyr_library_sources_ifdef(CONFIG_UART_BLOB_RX uart_blob_rx.c)
