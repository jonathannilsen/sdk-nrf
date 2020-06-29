/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef DFU_TARGET_UART_H__
#define DFU_TARGET_UART_H__

#include <stddef.h>

/** @file dfu_target_uart.h
 * @defgroup dfu_target_uart UART DFU Target
 * @{
 * @brief DFU Target for upgrades performed over UART
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief See if data in buf indicates UART style upgrade.
 *
 * @retval true if data matches, false otherwise.
 */
bool dfu_target_uart_identify(const void *const buf);

/**
 * @brief Initialize dfu target, perform steps necessary to receive firmware.
 *
 * @param[in] file_size Size of the current file being downloaded.
 * @param[in] cb Callback for signaling events(unused).
 *
 * @return 0 on success, negative errno otherwise.
 */
int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb);

/**
 * @brief Get offset of firmware
 *
 * @param[out] offset Returns the offset of the firmware upgrade.
 *
 * @return 0 if success, otherwise negative value if unable to get the offset
 */
int dfu_target_uart_offset_get(size_t *offset);

/**
 * @brief Write firmware data.
 *
 * @param[in] buf Pointer to data that should be written.
 * @param[in] len Length of data to write.
 *
 * @return 0 on success, negative errno otherwise.
 */
int dfu_target_uart_write(const void *const buf, size_t len);

/**
 * @brief Deinitialize resources and finalize firmware upgrade if successful.

 * @param[in] successful Indicate whether the firmware was successfully recived.
 *
 * @return 0 on success, negative errno otherwise.
 */
int dfu_target_uart_done(bool successful);


/**@} */

#endif /* DFU_TARGET_UART_H__ */
