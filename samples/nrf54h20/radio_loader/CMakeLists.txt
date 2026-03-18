#
# Copyright (c) 2025 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(radio_loader)

target_sources(app PRIVATE src/main.c)

# Generate PERIPHCONF entries C file for the RAM image in this one instead.
# Taken from the CMake in zephyr/modules/hal_nordic/ironside/se/CMakeLists.txt.
if(CONFIG_NRF_PERIPHCONF_GENERATE_ENTRIES)
  set(remote_EDT_PICKLE "${ZEPHYR_BINARY_DIR}/../../remote/zephyr/edt.pickle")
  set(remote_periphconf_entries_c_file ${ZEPHYR_BINARY_DIR}/remote_periphconf_entries_generated.c)
  set(optional_args)
  if(CONFIG_NRF_PERIPHCONF_GENERATE_ENTRIES_LOCK)
    list(APPEND optional_args --lock)
  else()
    list(APPEND optional_args --no-lock)
  endif()
  execute_process(
    COMMAND
      ${CMAKE_COMMAND} -E env ZEPHYR_BASE=${ZEPHYR_BASE}
      ${PYTHON_EXECUTABLE} ${ZEPHYR_BASE}/modules/hal_nordic/ironside/se/scripts/gen_periphconf_entries.py
      --soc ${CONFIG_SOC}
      --in-edt-pickle ${remote_EDT_PICKLE}
      --out-periphconf-source ${remote_periphconf_entries_c_file}
      ${optional_args}
    WORKING_DIRECTORY ${ZEPHYR_BINARY_DIR}
    COMMAND_ERROR_IS_FATAL ANY
  )
  target_sources(app PRIVATE ${remote_periphconf_entries_c_file})
  message(STATUS "Generated ${remote_periphconf_entries_c_file} from ${remote_EDT_PICKLE}")
endif()
