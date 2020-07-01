#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lib_uart_dfu_test)

# test_runner_generate(src/main.c)

# cmock_handle(${ZEPHYR_BASE}/include/drivers/uart.h uart)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/include
  ${ZEPHYR_BASE}/../nrf/lib/lte_uart_dfu
  )

target_compile_options(app
  PRIVATE
  -DCONFIG_LTE_UART_DFU_UART_LABEL="UART_1"
  -DCONFIG_UART_ASYNC_API=1
  )
