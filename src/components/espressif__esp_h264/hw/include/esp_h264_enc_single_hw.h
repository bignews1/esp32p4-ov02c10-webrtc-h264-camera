/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_h264_enc_single.h"
#include "esp_h264_enc_param_hw.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration type for hardware-based H.264 encoder
 */
typedef esp_h264_enc_cfg_t esp_h264_enc_cfg_hw_t;

/**
 * @brief  This function is used to create a new instance of the `esp_h264_enc_t` data structure,
 *         which represents a single-streams H.264 encoder in hardware
 *
 * @note  The group of picture(GOP) will be updated in intra frame
 *
 * @param[in]   cfg      It is a pointer to the `esp_h264_enc_cfg_hw_t` structure, which contains the configuration settings for the encoder
 * @param[out]  out_enc  It is a double pointer to the `esp_h264_enc_t` structure, which will store the created encoder instance
 *
 * @return
 *       - ESP_H264_ERR_OK   Succeeded
 *       - ESP_H264_ERR_ARG  Invalid arguments passed
 *       - ESP_H264_ERR_MEM  Insufficient memory, the `*out_enc` will be set NULL
 */
esp_h264_err_t esp_h264_enc_hw_new(const esp_h264_enc_cfg_hw_t *cfg, esp_h264_enc_handle_t *out_enc);

/**
 * @brief  This function returns a pointer to the hardware-encoded parameter structure associated with the given `esp_h264_enc_t` encoder
 *
 * @param[in]   enc        The encoder instance that is from `esp_h264_enc_hw_new`
 * @param[out]  out_param  The parameter set handle
 *
 * @return
 *       - ESP_H264_ERR_OK   Succeeded
 *       - ESP_H264_ERR_ARG  Invalid arguments passed
 */
esp_h264_err_t esp_h264_enc_hw_get_param_hd(esp_h264_enc_handle_t enc, esp_h264_enc_param_hw_handle_t *out_param);

/**
 * @brief Update GOP on the currently opened single HW encoder without recreating it.
 *
 * [H264-LIVE-GOP-R1-20260913]
 * The HW encoder already supports parameter GOP changes at the next IDR boundary.
 * This helper exposes that path to the V4L2 live camera owner without allocating,
 * freeing, closing, or reopening any encoder/DMA object.
 *
 * @param[in] gop New GOP value. Must be greater than zero.
 *
 * @return
 *       - ESP_H264_ERR_OK    Request accepted by the active encoder parameter set
 *       - ESP_H264_ERR_ARG   Invalid GOP
 *       - ESP_H264_ERR_FAIL  No HW encoder is currently opened
 */
esp_h264_err_t esp_h264_enc_hw_active_set_gop(uint8_t gop);

/**
 * @brief Reserve the shared PSRAM reference-frame pool for HW H.264.
 *
 * This is the fail-safe reference path and remains available even when the
 * persistent INTERNAL A/B pool cannot be reserved.
 */
esp_h264_err_t esp_h264_enc_hw_ref_pool_reserve(uint16_t max_width);

/**
 * @brief Release the shared PSRAM H.264 reference-frame pool when idle.
 *
 * Safe handover rule: this returns false while an encoder still owns the pool.
 * This does not release the R2 persistent INTERNAL block.
 */
bool esp_h264_enc_hw_ref_pool_release(void);

/** @brief Return true when the persistent reference-frame pool is ready. */
bool esp_h264_enc_hw_ref_pool_ready(void);

/**
 * @brief Reserve one persistent 1080-capable INTERNAL H.264 reference buffer.
 *
 * Test-only R2: the ~138KB block is retained until reboot and reused by both
 * 720p and 1080p encoder instances. The PSRAM ref pool remains the fallback.
 */
esp_h264_err_t esp_h264_enc_hw_ref_persist_reserve(void);

#ifdef __cplusplus
}
#endif
