#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(json)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/subsys/dfu/src/dfu_target_uart.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/subsys/dfu/include
  ${ZEPHYR_BASE}/../nrf/include/dfu
  . # To get pm_config.h
  ${ZEPHYR_BASE}/../nrf/subsys/dfu/src
  )

target_compile_options(app
  PRIVATE
  -DCONFIG_IMG_BLOCK_BUF_SIZE=4096
  -DCONFIG_DFU_TARGET_LOG_LEVEL=2
  )
