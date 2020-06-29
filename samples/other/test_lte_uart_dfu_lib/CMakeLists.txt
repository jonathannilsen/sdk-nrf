# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(test_lte_uart_dfu_lib)

zephyr_library_include_directories($ENV{ZEPHYR_BASE}/../nrf/subsys/dfu/include)

set(CONF_FILE "prj_${BOARD}.conf")

target_sources(app PRIVATE src/main.c)
