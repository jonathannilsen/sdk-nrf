#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lib_uart_dfu_host_test)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/lib/uart_dfu/uart_dfu_host.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/include
  ${ZEPHYR_BASE}/../nrf/include/dfu
  ${ZEPHYR_BASE}/../nrf/lib/uart_dfu
  )
