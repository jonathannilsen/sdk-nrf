#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dfu_target_test)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/subsys/dfu/src/dfu_target_uart.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/include
  ${ZEPHYR_BASE}/../nrf/include/dfu
  ${ZEPHYR_BASE}/../nrf/subsys/dfu/include
  )

target_compile_options(app
  PRIVATE
  -DCONFIG_DFU_TARGET_LOG_LEVEL=2
  -DCONFIG_DFU_TARGET_UART=1
  -DCONFIG_DFU_TARGET_UART_INSTANCE=0
  )
