function(add_file_size_validation target file_name max_size)
  # Add a command to run the bottom part of this file after the build is done.
  add_custom_command(TARGET ${target}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
    -DVALIDATE_FILE_NAME="${file_name}"
    -DVALIDATE_MAX_SIZE="${max_size}"
    -P ${ZEPHYR_BASE}/../nrf/cmake/validate_file_size.cmake
    COMMENT "Checking size of ${file_name}"
    )
endfunction()

if ((DEFINED VALIDATE_FILE_NAME) AND (DEFINED VALIDATE_MAX_SIZE))
  file(SIZE ${VALIDATE_FILE_NAME} file_size)
  math(EXPR file_size "${file_size}" OUTPUT_FORMAT HEXADECIMAL)
  math(EXPR max_size "${VALIDATE_MAX_SIZE}" OUTPUT_FORMAT HEXADECIMAL)

  if (file_size GREATER max_size)
    message(STATUS "Partition size is less than binary size")
  endif()
endif()