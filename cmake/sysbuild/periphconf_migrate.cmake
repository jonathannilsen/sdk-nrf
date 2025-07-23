# Copyright (c) 2025 Nordic Semiconductor ASA
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

# This file includes extra sysbuild POST_CMAKE logic to assist with
# autogenerating an IronSide SE compatible UICR.
#
# It is enabled when SB_CONFIG_NRF_PERIPHCONF_MIGRATE=y is set and
# one of the sysbuild images has CONFIG_NRF_HALTIUM_GENERATE_UICR=y.
# It uses nrf-regtool to produce an extra source file for that image
# (periphconf_migrated.c) based on multiple image devicetrees.
#
# NOTE: This is a temporary solution for NCS. The planned solution
# for upstream Zephyr will not require nrf-regtool.

set(ironside_uicr_images "")
set(ironside_uicr_main_image)
foreach(image ${POST_CMAKE_IMAGES})
  sysbuild_get(${image}_54h20_app IMAGE ${image} VAR CONFIG_SOC_NRF54H20_CPUAPP KCONFIG)
  sysbuild_get(${image}_54h20_rad IMAGE ${image} VAR CONFIG_SOC_NRF54H20_CPURAD KCONFIG)
  if((${image}_54h20_app OR ${image}_54h20_rad))
    sysbuild_get(${image}_periphconf_section
      IMAGE ${image}
      VAR CONFIG_NRF_PERIPHCONF_SECTION
      KCONFIG
    )
    if(${image}_periphconf_section)
      list(APPEND ironside_uicr_images ${image})
    endif()
    if(NOT DEFINED ironside_uicr_main_image)
      sysbuild_get(image_generates_uicr IMAGE ${image} VAR CONFIG_NRF_HALTIUM_GENERATE_UICR KCONFIG)
      if(image_generates_uicr)
        set(ironside_uicr_main_image ${image})
      endif()
    endif()
  endif()
endforeach()
if(DEFINED ironside_uicr_main_image)
  find_program(NRF_REGTOOL nrf-regtool REQUIRED)
  set(nrf_regtool_cmd
    COMMAND
    ${CMAKE_COMMAND} -E env PYTHONPATH=${ZEPHYR_BASE}/scripts/dts/python-devicetree/src
    ${NRF_REGTOOL}
  )
  foreach(image ${ironside_uicr_images})
    ExternalProject_Get_Property(${image} BINARY_DIR)
    sysbuild_get(image_soc IMAGE ${image} VAR CONFIG_SOC KCONFIG)
    set(image_uicr_hex ${BINARY_DIR}/zephyr/uicr_to_migrate.hex)
    message(STATUS "Running nrf-regtool uicr-compile for ${image}")
    execute_process(
      ${nrf_regtool_cmd}
      uicr-compile
      --edt-pickle-file ${BINARY_DIR}/zephyr/edt.pickle
      --product-name ${image_soc}
      --output-file ${image_uicr_hex}
      WORKING_DIRECTORY ${APPLICATION_SOURCE_DIR}
      COMMAND_ERROR_IS_FATAL ANY
    )
    message(STATUS "Running nrf-regtool uicr-migrate for ${image}")
    execute_process(
      ${nrf_regtool_cmd}
      uicr-migrate
      --edt-pickle-file ${BINARY_DIR}/zephyr/edt.pickle
      --uicr-hex-file ${image_uicr_hex}
      --output-periphconf-file ${BINARY_DIR}/zephyr/periphconf_migrated.c
      WORKING_DIRECTORY ${APPLICATION_SOURCE_DIR}
      COMMAND_ERROR_IS_FATAL ANY
    )
    sysbuild_get(${image}_elf_name IMAGE ${image} VAR BYPRODUCT_KERNEL_ELF_NAME CACHE)
    list(APPEND periphconf_elf_files ${${image}_elf_name})
  endforeach()

  message("Have the following elfs: ${periphconf_elf_files}")

  ExternalProject_Get_Property(${ironside_uicr_main_image} BINARY_DIR)

  list(TRANSFORM periphconf_elf_files PREPEND "--in-periphconf-elf;" OUTPUT_VARIABLE in_periphconf_elf_arg)

  set(optional_byproducts)

  set(out_periphconf_hex_arg)
  sysbuild_get(gen_periphconf IMAGE ${ironside_uicr_main_image} VAR CONFIG_NRF_HALTIUM_UICR_PERIPHCONF KCONFIG)
  if(gen_periphconf)
    set(periphconf_hex_file ${BINARY_DIR}/periphconf.hex)
    set(out_periphconf_hex_arg
      --out-periphconf-hex ${periphconf_hex_file}
    )
    list(APPEND optional_byproducts ${periphconf_hex_file})
  endif()

  set(out_periphconf_recovery_hex_arg)
  sysbuild_get(gen_recovery_periphconf IMAGE ${ironside_uicr_main_image} VAR CONFIG_NRF_HALTIUM_UICR_RECOVERY_PERIPHCONF KCONFIG)
  if(gen_recovery_periphconf)
    set(periphconf_recovery_hex_file ${BINARY_DIR}/periphconf_recovery.hex)
    set(out_periphconf_recovery_hex_arg
      --out-periphconf-recovery-hex ${periphconf_recovery_hex_file}
    )
    list(APPEND optional_byproducts ${periphconf_recovery_hex_file})
  endif()

  set(uicr_hex_file ${BINARY_DIR}/uicr.hex)
  add_custom_target(
    gen_uicr ALL
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${ZEPHYR_BASE}/scripts/dts/python-devicetree/src
    ${PYTHON_EXECUTABLE} ${ZEPHYR_BASE}/soc/nordic/common/uicr/gen_uicr.py
    --in-config ${BINARY_DIR}/zephyr/.config # ${DOTCONFIG}
    --in-edt-pickle ${BINARY_DIR}/zephyr/edt.pickle # ${EDT_PICKLE}
    ${in_periphconf_elf_arg}
    ${out_periphconf_hex_arg}
    ${out_periphconf_recovery_hex_arg}
    --out-uicr-hex ${uicr_hex_file}

    DEPENDS ${periphconf_elf_files} ${ironside_uicr_main_image}

    BYPRODUCTS
    ${uicr_hex_file} ${optional_byproducts}

    COMMENT "Generating UICR artifacts"
  )
endif()
