/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_h264_alloc.h"
#include "esp_private/esp_cache_private.h"
#include "esp_h264_enc_hw_param.h"

/* [REF-PERSIST-R2 2026-09-12]
 * TEST-ONLY: reserve one 1920-wide H.264 reference buffer in INTERNAL memory
 * at network GOT_IP, after ESP-Hosted/SDIO is alive but before camera services
 * start. The block is large enough for both 1920x1080 (~138KB) and
 * 1288x728 (~93KB), is reused across camera stop/start and profile switches,
 * and is never returned until reboot.
 *
 * Both 720p and 1080p GOP5 may use this persistent INTERNAL reference. The
 * existing PSRAM reference pool remains the fail-safe fallback if reservation
 * is unavailable or the persistent block is unexpectedly busy.
 *
 * Reservation is fail-safe. Both DMA total free and one DMA-capable contiguous
 * block must cover 1080 ref_need + the protected reserve, and post-allocation
 * DMA headroom is rechecked.
 */
#ifndef HA_H264_REF_720_INTERNAL_ABTEST
#define HA_H264_REF_720_INTERNAL_ABTEST 1
#endif
#ifndef HA_H264_REF_720_DMA_RESERVE_BYTES
#define HA_H264_REF_720_DMA_RESERVE_BYTES (40U * 1024U)
#endif

static const char *TAG = "H264_ENC.HW.SET";

#define ESP_H264_ROI_SUP_NUM    (8)
#define ESP_H264_REDUNDANT_BYTE (8 + 64)
#define SPS_PPS_BUF_SIZE        (100)

typedef struct esp_h264_param {
    esp_h264_enc_param_hw_t    hw_base;
    esp_h264_set_dev_t         device;
    uint8_t                    fps;
    uint8_t                    gop;
    esp_h264_rc_hd_t           rc_hd;
    uint8_t                    qp_init;
    uint32_t                   bitrate;
    uint16_t                   width;
    uint16_t                   height;
    uint8_t                    mb_width;
    uint8_t                    mb_height;
    uint8_t                   *nal_buf;
    uint8_t                    nal_buf_len;
    uint16_t                   nal_bit_len;
    uint8_t                   *mvm_buf;
    uint32_t                   mvm_buf_len;
    uint8_t                   *db;
    uint8_t                   *ref;
    bool                       ref_from_pool; /* PSRAM shared ref pool */
    bool                       ref_from_internal_persist; /* [REF-PERSIST-R2] never free per encoder */
    h264_dma_desc_t           *dsc_ref;
    h264_dma_desc_t           *dsc_db[4];
    h264_dma_desc_t           *dsc_mvm;
    esp_h264_mutex_t           mutex;
#if HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300
    h264_dma_pbyte_t           pbyte;
#endif
    float                      bpp;
} esp_h264_param_t;

#if HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300

static h264_dma_pbyte_t esp_h264_get_pbyte_by_pic_type(esp_h264_raw_format_t pic_type)
{
    switch (pic_type) {
    case ESP_H264_RAW_FMT_O_UYY_E_VYY:
        return H264_DMA_PBYTE_1_5;
    case ESP_H264_RAW_FMT_RGB565_LE:
    case ESP_H264_RAW_FMT_UYVY:
        return H264_DMA_PBYTE_2;
    case ESP_H264_RAW_FMT_VUY:
    case ESP_H264_RAW_FMT_BGR888:
    default:
        return H264_DMA_PBYTE_3;
    }
}

static h264_ori_color_space_t esp_h264_get_color_space_by_pic_type(esp_h264_raw_format_t pic_type)
{
    switch (pic_type) {
    case ESP_H264_RAW_FMT_BGR888:
        return H264_ORI_COLOR_SPACE_RGB888;
    case ESP_H264_RAW_FMT_RGB565_LE:
        return H264_ORI_COLOR_SPACE_RGB565;
    case ESP_H264_RAW_FMT_VUY:
        return H264_ORI_COLOR_SPACE_YUV444;
    case ESP_H264_RAW_FMT_UYVY:
        return H264_ORI_COLOR_SPACE_YUV422;
    case ESP_H264_RAW_FMT_O_UYY_E_VYY:
        return H264_ORI_COLOR_SPACE_YUV420;
    default:
        return H264_ORI_COLOR_SPACE_INVALID;
    }
}

#endif  /* HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300 */

/** Basic parameter configure */
static esp_h264_err_t get_res(esp_h264_enc_param_handle_t handle, esp_h264_resolution_t *res)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    res->width = param->width;
    res->height = param->height;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t set_fps(esp_h264_enc_param_handle_t handle, uint8_t fps)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    param->fps = fps;
    esp_h264_mutex_lock(param->mutex, ESP_H264_MAX_DELAY);
    if (param->rc_hd) {
        esp_h264_enc_hw_rc_set_bt_fps(param->rc_hd, param->bitrate, param->fps);
    }
    esp_h264_enc_set_sps(param->nal_buf, param->nal_buf_len, param->height, param->width, param->fps);
    esp_h264_mutex_unlock(param->mutex);
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_fps(esp_h264_enc_param_handle_t handle, uint8_t *fps)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    *fps = param->fps;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t set_gop(esp_h264_enc_param_handle_t handle, uint8_t gop)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    param->gop = gop;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_gop(esp_h264_enc_param_handle_t handle, uint8_t *gop)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    *gop = param->gop;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t set_bitrate(esp_h264_enc_param_handle_t handle, uint32_t bitrate)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    param->bitrate = bitrate;
    if (param->rc_hd) {
        esp_h264_mutex_lock(param->mutex, ESP_H264_MAX_DELAY);
        esp_h264_enc_hw_rc_set_bt_fps(param->rc_hd, bitrate, param->fps);
        esp_h264_mutex_unlock(param->mutex);
    }
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_bitrate(esp_h264_enc_param_handle_t handle, uint32_t *bitrate)
{
    esp_h264_enc_param_hw_handle_t param_base = __containerof(handle, esp_h264_enc_param_hw_t, base);
    esp_h264_param_t *param = __containerof(param_base, esp_h264_param_t, hw_base);
    *bitrate = param->bitrate;
    return ESP_H264_ERR_OK;
}

/** HW special parameter configure */
static esp_h264_err_t cfg_roi(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_roi_cfg_t cfg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    esp_h264_mutex_lock(param->mutex, ESP_H264_MAX_DELAY);
    h264_hal_set_roi_mode(param->device, (int8_t)cfg.roi_mode, cfg.none_roi_delta_qp);
    esp_h264_mutex_unlock(param->mutex);
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_roi_cfg_info(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_roi_cfg_t *cfg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    cfg->roi_mode = ESP_H264_ROI_MODE_DISABLE;
    uint8_t roi_mode = 0;
    if (h264_hal_get_roi_mode(param->device, &roi_mode, &cfg->none_roi_delta_qp) == true) {
        cfg->roi_mode = roi_mode;
    }
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t set_roi_reg(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_roi_reg_t roi_reg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    ESP_H264_RET_ON_FALSE(roi_reg.reg_idx < ESP_H264_ROI_SUP_NUM, ESP_H264_ERR_ARG, TAG, "Invalid `reg_idx` parameter");
    int16_t x_size = roi_reg.len_x + roi_reg.x;
    int16_t y_size = roi_reg.len_y + roi_reg.y;
    ESP_H264_RET_ON_FALSE(x_size <= param->mb_width, ESP_H264_ERR_ARG, TAG, "The sum of `len_x` and `x` is greater than `mb_width` ");
    ESP_H264_RET_ON_FALSE(y_size <= param->mb_height, ESP_H264_ERR_ARG, TAG, "The sum of `len_y` and `y` is greater than `mb_height` ");
    uint8_t roi_mode = 0;
    int8_t none_roi_delta_qp = 0;
    /* Check ROI function enable*/
    if (h264_hal_get_roi_mode(param->device, &roi_mode, &none_roi_delta_qp) == false) {
        /* When ROI function disable, the ROI region is meaningless. */
        ESP_H264_LOGE(TAG, "ROI mode is disable. Please enable ROI mode first.");
        return ESP_H264_ERR_ARG;
    }
    switch (roi_mode) {
    case ESP_H264_ROI_MODE_DELTA_QP:
        ESP_H264_RET_ON_FALSE((roi_reg.qp >= -ESP_H264_QP_MAX), ESP_H264_ERR_ARG, TAG, "In ESP_H264_ROI_MODE_DELTA_QP, ROI region QP less than -51");
        break;
    case ESP_H264_ROI_MODE_FIX_QP:
        ESP_H264_RET_ON_FALSE((roi_reg.qp >= 0), ESP_H264_ERR_ARG, TAG, "In ESP_H264_ROI_MODE_FIX_QP, ROI region QP less than 0");
        break;
    default:
        break;
    }
    /* Open the ROI region */
    bool ena_reg = true;
    if (y_size == 0
            || x_size == 0
            || roi_reg.len_x == 0
            || roi_reg.len_y == 0) {
        /* Close the ROI region */
        ena_reg = false;
    }
    esp_h264_mutex_lock(param->mutex, ESP_H264_MAX_DELAY);
    h264_hal_set_roi_reg(param->device, ena_reg, roi_reg.x, roi_reg.y, roi_reg.len_x, roi_reg.len_y, roi_reg.qp, roi_reg.reg_idx);
    esp_h264_mutex_unlock(param->mutex);
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_roi_reg(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_roi_reg_t *roi_reg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    ESP_H264_RET_ON_FALSE(roi_reg->reg_idx < ESP_H264_ROI_SUP_NUM, ESP_H264_ERR_ARG, TAG, "Invalid `reg_idx` parameter");
    h264_hal_get_roi_reg(param->device, &roi_reg->x, &roi_reg->y, &roi_reg->len_x, &roi_reg->len_y, &roi_reg->qp, roi_reg->reg_idx);
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t cfg_mv(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_mv_cfg_t cfg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    if (cfg.mv_fmt >= ESP_H264_MVM_FMT_INVALID || cfg.mv_fmt < ESP_H264_MVM_FMT_ALL
            || cfg.mv_mode >= ESP_H264_MVM_MODE_INVALID || cfg.mv_mode < ESP_H264_MVM_MODE_DISABLE) {
        ESP_H264_LOGE(TAG, "Configure is vaild. mv_mode %d mv_fmt %d", cfg.mv_fmt, cfg.mv_mode);
        return ESP_H264_ERR_ARG;
    }
    esp_h264_mutex_lock(param->mutex, ESP_H264_MAX_DELAY);
    h264_hal_set_mv_mode(param->device, (int8_t)cfg.mv_mode, (uint8_t)cfg.mv_fmt);
    esp_h264_mutex_unlock(param->mutex);
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_mv_cfg_info(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_mv_cfg_t *cfg)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    int8_t mv_mode = 0;
    uint8_t mv_fmt = 0;
    h264_hal_get_mv_mode(param->device, &mv_mode, &mv_fmt);
    cfg->mv_mode = mv_mode;
    cfg->mv_fmt = mv_fmt;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t set_mv_pkt(esp_h264_enc_param_hw_handle_t handle, esp_h264_enc_mvm_pkt_t mv_pkt)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    param->mvm_buf = (uint8_t *)mv_pkt.data;
    param->mvm_buf_len = mv_pkt.len;
    return ESP_H264_ERR_OK;
}

static esp_h264_err_t get_mv_data_len(esp_h264_enc_param_hw_handle_t handle, uint32_t *length)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    *length = h264_hal_get_mvm_data_len(param->device);
    esp_h264_cache_check_and_invalidate(param->mvm_buf, param->mvm_buf_len);
    return ESP_H264_ERR_OK;
}

static int max_refame_buffer_size(int16_t mb_width)
{
    /** H264_DMA_MACRO_SIZE + H264_DMA_HALF_MACRO_SIZE : Y(16) + U(4) + V(4) */
    return H264_DMA_3_LINES * H264_DMA_MACRO_SIZE * (H264_DMA_MACRO_SIZE + H264_DMA_HALF_MACRO_SIZE) * mb_width + HW_DMA_BUF_ALIGN_SIZE;
}

/* [REF-POOL-1.3.0][REF-PSRAM AB1] Shared PSRAM reference-frame pool.
 * Reserved lazily for the maximum supported width and reused by each encoder
 * instance. R2 keeps it as the fail-safe path if INTERNAL persistence is not
 * available.
 */
static uint8_t  *s_ref_pool = NULL;
static uint32_t  s_ref_pool_size = 0;
static bool      s_ref_pool_busy = false;

/* [REF-PERSIST-R2] Dedicated 1080-capable INTERNAL reference buffer.
 * Lifetime is boot -> reboot. One camera encoder at a time may use it at either
 * 720p or 1080p; the PSRAM pool above remains the fail-safe fallback. */
static uint8_t  *s_ref_internal_persist = NULL;
static uint32_t  s_ref_internal_persist_size = 0;
static bool      s_ref_internal_persist_busy = false;


/* [H264-WEB20-PERSIST-R3B] Separate 20KiB INTERNAL/DMA pool for H.264's
 * short-lived db_tmp and DMA descriptor objects.  This is intentionally a
 * separate allocation from the ~138KiB reference frame: the system never
 * needs one 160KiB contiguous hole.  The pool is boot-persistent and is reset
 * by offset between encoder sessions, so H.264 small-object churn does not
 * fragment the global DMA heap. */
#define H264_SMALL_PERSIST_BYTES        (20U * 1024U)
#define H264_SMALL_PERSIST_ALIGN        16U
#define H264_PERSIST_PROTECT_BYTES      ((uint32_t)HA_H264_REF_720_DMA_RESERVE_BYTES)

static uint8_t  *s_h264_small_persist = NULL;
static uint32_t  s_h264_small_persist_size = 0;
static uint32_t  s_h264_small_persist_off = 0;
static bool      s_h264_small_persist_busy = false;
static bool      s_h264_small_reserve_attempted = false;
static bool      s_h264_persist_disabled_until_reboot = false;

static uint32_t h264_align_up_u32(uint32_t v, uint32_t a)
{
    return (v + (a - 1U)) & ~(a - 1U);
}

static bool h264_small_persist_reserve_once(void)
{
    if (s_h264_small_persist != NULL) {
        return true;
    }
    if (s_h264_small_reserve_attempted || s_h264_persist_disabled_until_reboot) {
        return false;
    }
    s_h264_small_reserve_attempted = true;

    const uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    const size_t pre_free = heap_caps_get_free_size(dma_caps);
    const size_t pre_largest = heap_caps_get_largest_free_block(dma_caps);
    const uint32_t need = H264_SMALL_PERSIST_BYTES;
    const uint32_t protect = H264_PERSIST_PROTECT_BYTES;

    /* Separate block: only this 20KiB request itself must be contiguous. Keep
     * the same 40KiB global DMA safety reserve after it is carved out. */
    if ((uint64_t)pre_free < (uint64_t)need + (uint64_t)protect ||
        (uint64_t)pre_largest < (uint64_t)need) {
        ESP_H264_LOGI(TAG,
                      "[H264-WEB20-PERSIST-R3B] reserve need=%u protect=%u dma_pre=%u/%u decision=SKIP reason=headroom",
                      (unsigned)need, (unsigned)protect,
                      (unsigned)pre_free, (unsigned)pre_largest);
        return false;
    }

    uint32_t actual_size = 0;
    uint8_t *pool = (uint8_t *)esp_h264_aligned_calloc(
        H264_SMALL_PERSIST_ALIGN, 1, need, &actual_size, ESP_H264_MEM_INTERNAL);
    if (pool == NULL) {
        ESP_H264_LOGI(TAG,
                      "[H264-WEB20-PERSIST-R3B] reserve need=%u dma_pre=%u/%u decision=SKIP reason=alloc-failed",
                      (unsigned)need, (unsigned)pre_free, (unsigned)pre_largest);
        return false;
    }

    const size_t post_free = heap_caps_get_free_size(dma_caps);
    const size_t post_largest = heap_caps_get_largest_free_block(dma_caps);
    if (post_free < protect || post_largest < protect) {
        esp_h264_free(pool);
        ESP_H264_LOGI(TAG,
                      "[H264-WEB20-PERSIST-R3B] reserve need=%u actual=%u protect=%u dma_pre=%u/%u dma_post=%u/%u decision=SKIP reason=post-headroom",
                      (unsigned)need, (unsigned)actual_size, (unsigned)protect,
                      (unsigned)pre_free, (unsigned)pre_largest,
                      (unsigned)post_free, (unsigned)post_largest);
        return false;
    }

    s_h264_small_persist = pool;
    s_h264_small_persist_size = actual_size;
    s_h264_small_persist_off = 0;
    s_h264_small_persist_busy = false;
    ESP_H264_LOGI(TAG,
                  "[H264-WEB20-PERSIST-R3B] reserve need=%u actual=%u protect=%u dma_pre=%u/%u dma_post=%u/%u ptr=%p decision=RESERVED",
                  (unsigned)need, (unsigned)actual_size, (unsigned)protect,
                  (unsigned)pre_free, (unsigned)pre_largest,
                  (unsigned)post_free, (unsigned)post_largest, (void *)pool);
    return true;
}

static uint32_t h264_small_persist_cache_align(void);

bool esp_h264_enc_hw_small_persist_begin(void)
{
    if (s_h264_persist_disabled_until_reboot || s_h264_small_persist == NULL ||
        s_h264_small_persist_busy) {
        return false;
    }
    s_h264_small_persist_off = 0;
    s_h264_small_persist_busy = true;
    memset(s_h264_small_persist, 0, s_h264_small_persist_size);
    esp_h264_cache_check_and_writeback(s_h264_small_persist, s_h264_small_persist_size);
    ESP_H264_LOGI(TAG,
                  "[H264-WEB20-PERSIST-R3B] session begin pool=%u cache_align=%u",
                  (unsigned)s_h264_small_persist_size,
                  (unsigned)h264_small_persist_cache_align());
    return true;
}

static uint32_t h264_small_persist_cache_align(void)
{
    size_t cache_align = H264_SMALL_PERSIST_ALIGN;
    esp_cache_get_alignment(ESP_H264_MEM_INTERNAL, &cache_align);
    if (cache_align < H264_SMALL_PERSIST_ALIGN ||
        (cache_align & (cache_align - 1U)) != 0U) {
        cache_align = H264_SMALL_PERSIST_ALIGN;
    }
    return (uint32_t)cache_align;
}

void *esp_h264_enc_hw_small_persist_calloc(uint32_t alignment, uint32_t size, uint32_t *actual_size)
{
    if (!s_h264_small_persist_busy || s_h264_small_persist == NULL || size == 0) {
        return NULL;
    }
    const uint32_t cache_align = h264_small_persist_cache_align();
    if (alignment < cache_align) {
        alignment = cache_align;
    }
    if ((alignment & (alignment - 1U)) != 0U) {
        return NULL;
    }
    const uint32_t alloc_size = h264_align_up_u32(size, cache_align);
    const uint32_t off = h264_align_up_u32(s_h264_small_persist_off, alignment);
    if ((uint64_t)off + (uint64_t)alloc_size > (uint64_t)s_h264_small_persist_size) {
        ESP_H264_LOGI(TAG,
                      "[H264-WEB20-PERSIST-R3B] session alloc req=%u actual=%u align=%u used=%u/%u decision=GLOBAL-FALLBACK",
                      (unsigned)size, (unsigned)alloc_size, (unsigned)alignment,
                      (unsigned)s_h264_small_persist_off,
                      (unsigned)s_h264_small_persist_size);
        return NULL;
    }
    uint8_t *ptr = s_h264_small_persist + off;
    memset(ptr, 0, alloc_size);
    esp_h264_cache_check_and_writeback(ptr, alloc_size);
    s_h264_small_persist_off = off + alloc_size;
    if (actual_size) {
        *actual_size = alloc_size;
    }
    return ptr;
}

bool esp_h264_enc_hw_small_persist_owns(const void *ptr)
{
    if (ptr == NULL || s_h264_small_persist == NULL) {
        return false;
    }
    const uintptr_t p = (uintptr_t)ptr;
    const uintptr_t lo = (uintptr_t)s_h264_small_persist;
    const uintptr_t hi = lo + (uintptr_t)s_h264_small_persist_size;
    return p >= lo && p < hi;
}

void esp_h264_enc_hw_small_persist_end(void)
{
    if (!s_h264_small_persist_busy) {
        return;
    }
    ESP_H264_LOGI(TAG,
                  "[H264-WEB20-PERSIST-R3B] session end used=%u/%u reset=1",
                  (unsigned)s_h264_small_persist_off,
                  (unsigned)s_h264_small_persist_size);
    s_h264_small_persist_off = 0;
    s_h264_small_persist_busy = false;
}

esp_h264_err_t esp_h264_enc_hw_ref_pool_reserve(uint16_t max_width)
{
    if (s_ref_pool != NULL) {
        return ESP_H264_ERR_OK;
    }
    if (max_width == 0) {
        return ESP_H264_ERR_ARG;
    }

    const int16_t mb_width = (int16_t)((max_width + 15U) >> 4);
    const uint32_t need = (uint32_t)max_refame_buffer_size(mb_width);
    uint32_t actual_size = 0;

    /* v1.3.0 uses aligned_calloc for param->ref. Keep identical zero-init
     * semantics for the persistent pool.
     */
    /* [REF-PSRAM AB1] Keep the ~138KB persistent reference frame in PSRAM.
     * ESP32-P4 H.264 has a dedicated external-memory reference DMA path; keep
     * all DMA descriptors and db_tmp allocations unchanged in INTERNAL. */
    uint8_t *pool = (uint8_t *)esp_h264_aligned_calloc(16, 1, need, &actual_size, ESP_H264_MEM_SPIRAM);
    if (pool == NULL) {
        ESP_H264_LOGE(TAG, "[REF-POOL] reserve failed max_width=%u need=%u",
                      (unsigned)max_width, (unsigned)need);
        return ESP_H264_ERR_MEM;
    }

    s_ref_pool = pool;
    s_ref_pool_size = actual_size;
    s_ref_pool_busy = false;
    ESP_H264_LOGI(TAG, "[REF-POOL][REF-PSRAM AB1] reserved max_width=%u need=%u actual=%u ptr=%p",
                  (unsigned)max_width, (unsigned)need, (unsigned)actual_size, (void *)pool);
    return ESP_H264_ERR_OK;
}

/* [REF-PERSIST-R2] Reserve one 1920-wide reference buffer once. */
esp_h264_err_t esp_h264_enc_hw_ref_persist_reserve(void)
{
#if !HA_H264_REF_720_INTERNAL_ABTEST
    return ESP_H264_ERR_FAIL;
#else
    if (s_ref_internal_persist != NULL) {
        (void)h264_small_persist_reserve_once();
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R3B] reserve already-ready ref=%u small=%u ptr=%p",
                      (unsigned)s_ref_internal_persist_size,
                      (unsigned)s_h264_small_persist_size, (void *)s_ref_internal_persist);
        return ESP_H264_ERR_OK;
    }

    const uint16_t width = 1920U;
    const int16_t mb_width = (int16_t)((width + 15U) >> 4);
    const uint32_t ref_need = (uint32_t)max_refame_buffer_size(mb_width);
    const uint32_t reserve = (uint32_t)HA_H264_REF_720_DMA_RESERVE_BYTES;
    const uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    const size_t pre_free = heap_caps_get_free_size(dma_caps);
    const size_t pre_largest = heap_caps_get_largest_free_block(dma_caps);
    const size_t int_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    if ((uint64_t)pre_free < (uint64_t)ref_need + (uint64_t)reserve ||
        (uint64_t)pre_largest < (uint64_t)ref_need + (uint64_t)reserve ||
        (uint64_t)int_largest < (uint64_t)ref_need) {
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R2] reserve need=%u protect=%u dma_pre=%u/%u int_largest=%u decision=SKIP reason=headroom",
                      (unsigned)ref_need, (unsigned)reserve,
                      (unsigned)pre_free, (unsigned)pre_largest, (unsigned)int_largest);
        return ESP_H264_ERR_MEM;
    }

    uint32_t actual_size = 0;
    uint8_t *pool = (uint8_t *)esp_h264_aligned_calloc(
        16, 1, ref_need, &actual_size, ESP_H264_MEM_INTERNAL);
    if (pool == NULL) {
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R2] reserve need=%u protect=%u dma_pre=%u/%u decision=SKIP reason=alloc-failed",
                      (unsigned)ref_need, (unsigned)reserve,
                      (unsigned)pre_free, (unsigned)pre_largest);
        return ESP_H264_ERR_MEM;
    }

    const size_t post_free = heap_caps_get_free_size(dma_caps);
    const size_t post_largest = heap_caps_get_largest_free_block(dma_caps);
    if (post_free < reserve || post_largest < reserve) {
        esp_h264_free(pool);
        const size_t recovered_free = heap_caps_get_free_size(dma_caps);
        const size_t recovered_largest = heap_caps_get_largest_free_block(dma_caps);
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R2] reserve need=%u actual=%u protect=%u dma_pre=%u/%u dma_post=%u/%u recovered=%u/%u decision=SKIP reason=post-headroom",
                      (unsigned)ref_need, (unsigned)actual_size, (unsigned)reserve,
                      (unsigned)pre_free, (unsigned)pre_largest,
                      (unsigned)post_free, (unsigned)post_largest,
                      (unsigned)recovered_free, (unsigned)recovered_largest);
        return ESP_H264_ERR_MEM;
    }

    s_ref_internal_persist = pool;
    s_ref_internal_persist_size = actual_size;
    s_ref_internal_persist_busy = false;
    const bool small_ok = h264_small_persist_reserve_once();
    const size_t final_free = heap_caps_get_free_size(dma_caps);
    const size_t final_largest = heap_caps_get_largest_free_block(dma_caps);
    ESP_H264_LOGI(TAG,
                  "[REF-PERSIST-R3B] reserve ref_need=%u ref_actual=%u small=%u small_ok=%u protect=%u dma_pre=%u/%u dma_final=%u/%u ptr=%p decision=RESERVED",
                  (unsigned)ref_need, (unsigned)actual_size,
                  (unsigned)s_h264_small_persist_size, small_ok ? 1U : 0U,
                  (unsigned)reserve,
                  (unsigned)pre_free, (unsigned)pre_largest,
                  (unsigned)final_free, (unsigned)final_largest, (void *)pool);
    return ESP_H264_ERR_OK;
#endif
}

/* [REF-POOL-HANDOVER AB1 2026-09-06]
 * Temporarily return the persistent ~138KB ref pool while no encoder
 * owns it.  The existing lazy reserve path reacquires it on the next H.264
 * encoder creation, after the new WebRTC transport has become ready.
 * Never free while s_ref_pool_busy is true. */
bool esp_h264_enc_hw_ref_pool_release(void)
{
    if (s_ref_pool == NULL) {
        return true;
    }
    if (s_ref_pool_busy) {
        ESP_H264_LOGE(TAG, "[REF-POOL-HANDOVER] release rejected: busy pool=%u",
                      (unsigned)s_ref_pool_size);
        return false;
    }

    uint8_t *pool = s_ref_pool;
    const uint32_t pool_size = s_ref_pool_size;
    esp_h264_free(pool);
    s_ref_pool = NULL;
    s_ref_pool_size = 0;
    s_ref_pool_busy = false;
    ESP_H264_LOGI(TAG, "[REF-POOL-HANDOVER] released bytes=%u",
                  (unsigned)pool_size);
    return true;
}

bool esp_h264_enc_hw_ref_pool_ready(void)
{
    return s_ref_pool != NULL;
}

static int max_db_buffer_size(int16_t mb_width, int16_t mb_height)
{
    int buffer_size = mb_width * (mb_height - 1) * H264_DMA_DB_12_LINES_ROW_LENGTH
                      + mb_width * (H264_DMA_DB_12_LINES_ROW_LENGTH + H264_DMA_DB_4_LINES_ROW_LENGTH)
                      + mb_width * (mb_height - 1) * H264_DMA_DB_4_LINES_ROW_LENGTH;
    return buffer_size + ((HW_DMA_BUF_ALIGN_SIZE + 1) << 1);
}

static void cfg_dsc(h264_dma_desc_t *dsc, uint8_t en_2d, uint8_t mode, uint16_t vb, uint16_t hb, uint8_t eof, uint8_t owner, uint16_t va, uint16_t ha, uint8_t *buf, h264_dma_desc_t *next_dsc)
{
    dsc->dma_2d_en = en_2d;
    dsc->mode = mode;
    dsc->vb = vb;
    dsc->hb = hb;
    dsc->eof = eof;
    dsc->owner = owner;
    dsc->va = va;
    dsc->ha = ha;
    dsc->buf = buf;
    dsc->next_desc = next_dsc;
    esp_h264_cache_check_and_writeback((uint8_t *)dsc, sizeof(h264_dma_desc_t));
}

esp_h264_err_t esp_h264_enc_hw_del_param(esp_h264_enc_param_hw_handle_t handle)
{
    if (handle) {
        esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
        if (param->nal_buf && !esp_h264_enc_hw_small_persist_owns(param->nal_buf)) {
            esp_h264_free(param->nal_buf);
        }
        if (param->ref) {
            if (param->ref_from_internal_persist) {
                s_ref_internal_persist_busy = false; /* persistent until reboot */
            } else if (param->ref_from_pool) {
                s_ref_pool_busy = false; /* PSRAM shared pool: release ownership only */
            } else {
                esp_h264_free(param->ref);
            }
            param->ref = NULL;
            param->ref_from_pool = false;
            param->ref_from_internal_persist = false;
        }
        if (param->db) {
            esp_h264_free(param->db);
        }
        if (param->dsc_ref && !esp_h264_enc_hw_small_persist_owns(param->dsc_ref)) {
            esp_h264_free(param->dsc_ref);
        }
        for (size_t i = 0; i < 4; i++) {
            if (param->dsc_db[i] && !esp_h264_enc_hw_small_persist_owns(param->dsc_db[i])) {
                esp_h264_free(param->dsc_db[i]);
            }
        }
        if (param->dsc_mvm && !esp_h264_enc_hw_small_persist_owns(param->dsc_mvm)) {
            esp_h264_free(param->dsc_mvm);
        }
        esp_h264_enc_hw_rc_del(param->rc_hd);
        if (param->mutex) {
            esp_h264_mutex_delete(param->mutex);
        }
        esp_h264_free(param);
    }
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_new_param(esp_h264_enc_hw_param_cfg_t *cfg, esp_h264_enc_param_hw_handle_t *out_handle)
{
    /* Parameter check */
    ESP_H264_RET_ON_FALSE(cfg && out_handle, ESP_H264_ERR_ARG, TAG, "Invalid parameter");

    *out_handle = NULL;
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    uint32_t actual_size;
    /** Create a new parameter handle */
    esp_h264_param_t *param = (esp_h264_param_t *)esp_h264_calloc_prefer(1, sizeof(esp_h264_param_t), &actual_size, ESP_H264_MEM_SPIRAM, ESP_H264_MEM_INTERNAL);
    ESP_H264_RET_ON_FALSE(param, ESP_H264_ERR_ARG, TAG, "No memory for handle");

    /* Parameter initalization */
    param->device = cfg->device;
    param->width = cfg->width;
    param->height = cfg->height;
    param->qp_init = (cfg->qp_min + cfg->qp_max) >> 1;
    param->fps = cfg->fps;
    param->bitrate = cfg->bitrate;
    h264_hal_set_qp(param->device, param->qp_init);
    h264_hal_get_mbres(param->device, &param->mb_width, &param->mb_height);

    /** Create RC handle */
    if (cfg->qp_min < cfg->qp_max) {
        param->rc_hd = esp_h264_enc_hw_rc_new(cfg->qp_max, cfg->qp_min, param->bitrate, param->fps, param->mb_width, param->mb_height);
        ESP_H264_GOTO_ON_FALSE(param->rc_hd, ret, __exit__, TAG, "No memory for RC");
    }
    /** Disable MV */
    h264_hal_set_mv_mode(param->device, (int8_t)ESP_H264_MVM_MODE_DISABLE, 0);

    /** SPS + PPS */
    param->nal_buf_len = SPS_PPS_BUF_SIZE;
    param->nal_buf = (uint8_t *)esp_h264_enc_hw_small_persist_calloc(4, param->nal_buf_len, &actual_size);
    if (param->nal_buf == NULL) {
        param->nal_buf = (uint8_t *)esp_h264_calloc_prefer(1, param->nal_buf_len, &actual_size, ESP_H264_MEM_INTERNAL, ESP_H264_MEM_SPIRAM);
    }
    ESP_H264_GOTO_ON_FALSE(param->nal_buf, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for NAL");
    param->nal_bit_len = esp_h264_enc_set_sps(param->nal_buf, param->nal_buf_len, param->height, param->width, param->fps);
    param->nal_bit_len += esp_h264_enc_set_pps(param->nal_buf + (param->nal_bit_len >> 3), param->nal_buf_len - (param->nal_bit_len >> 3), param->qp_init, true);

    /** Allocated reference frame and DB memory */
    const uint32_t ref_need = (uint32_t)max_refame_buffer_size(param->mb_width);
    /* [REF-POOL-LAZY] ISP/IPA has initialized before this natural ref allocation point. */
    if (s_ref_pool == NULL) {
        esp_h264_err_t lazy_ret = esp_h264_enc_hw_ref_pool_reserve(1920);
        ESP_H264_LOGI(TAG, "[REF-POOL-LAZY] first encoder reserve ret=%d ref_need=%u pool=%u",
                      (int)lazy_ret, (unsigned)ref_need, (unsigned)s_ref_pool_size);
    }
    if (s_ref_pool != NULL && !s_ref_pool_busy && ref_need <= s_ref_pool_size) {
        /* Stock v1.3.0 uses calloc for each new reference frame. Preserve that
         * behavior when reusing the persistent block so a new session never
         * inherits stale reference contents from the previous resolution.
         */
        memset(s_ref_pool, 0, ref_need);
        esp_h264_cache_check_and_writeback(s_ref_pool, ref_need);
        param->ref = s_ref_pool;
        param->ref_from_pool = true;
        s_ref_pool_busy = true;
        actual_size = s_ref_pool_size;
        ESP_H264_LOGI(TAG, "[REF-POOL] acquire need=%u pool=%u",
                      (unsigned)ref_need, (unsigned)s_ref_pool_size);
    } else {
        param->ref = (uint8_t *)esp_h264_aligned_calloc(16, 1, ref_need, &actual_size, ESP_H264_MEM_INTERNAL);
        param->ref_from_pool = false;
    }
    ESP_H264_GOTO_ON_FALSE(param->ref, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for reference frame");
    param->db = (uint8_t *)esp_h264_calloc_prefer(1, max_db_buffer_size(param->mb_width, param->mb_height), &actual_size, ESP_H264_MEM_INTERNAL, ESP_H264_MEM_SPIRAM);
    ESP_H264_GOTO_ON_FALSE(param->db, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for data");

    /** Allocated descriptor memory. R3 takes these tiny DMA objects from the
     * separate 20KiB persistent pool when the caller opened a pool session. */
    param->dsc_ref = (h264_dma_desc_t *)esp_h264_enc_hw_small_persist_calloc(16, sizeof(h264_dma_desc_t), &actual_size);
    if (param->dsc_ref == NULL) {
        param->dsc_ref = (h264_dma_desc_t *)esp_h264_aligned_calloc(16, 1, sizeof(h264_dma_desc_t), &actual_size, ESP_H264_MEM_INTERNAL);
    }
    ESP_H264_GOTO_ON_FALSE(param->dsc_ref, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for reference descriptor");
    for (size_t i = 0; i < 4; i++) {
        param->dsc_db[i] = (h264_dma_desc_t *)esp_h264_enc_hw_small_persist_calloc(16, sizeof(h264_dma_desc_t), &actual_size);
        if (param->dsc_db[i] == NULL) {
            param->dsc_db[i] = (h264_dma_desc_t *)esp_h264_aligned_calloc(16, 1, sizeof(h264_dma_desc_t), &actual_size, ESP_H264_MEM_INTERNAL);
        }
        ESP_H264_GOTO_ON_FALSE(param->dsc_db[i], ESP_H264_ERR_MEM, __exit__, TAG, "No memory for DB descriptor");
    }
    param->dsc_mvm = (h264_dma_desc_t *)esp_h264_enc_hw_small_persist_calloc(16, sizeof(h264_dma_desc_t), &actual_size);
    if (param->dsc_mvm == NULL) {
        param->dsc_mvm = (h264_dma_desc_t *)esp_h264_aligned_calloc(16, 1, sizeof(h264_dma_desc_t), &actual_size, ESP_H264_MEM_INTERNAL);
    }
    ESP_H264_GOTO_ON_FALSE(param->dsc_mvm, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for MVM descriptor");

    /** Create MUTEX */
    param->mutex = xSemaphoreCreateMutex();
    ESP_H264_GOTO_ON_FALSE(param->mutex, ESP_H264_ERR_MEM, __exit__, TAG, "No memory for mutex semaphore");

#if HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300
    /** Set the picture type */
    h264_hal_set_ori_color_space(param->device, esp_h264_get_color_space_by_pic_type(cfg->pic_type));
    param->pbyte = esp_h264_get_pbyte_by_pic_type(cfg->pic_type);
#endif
    /** Get the bits per pixel */
    param->bpp = ESP_H264_GET_BPP_BY_PIC_TYPE(cfg->pic_type);

    /** Encoder handle configure */
    param->hw_base.base.get_res = get_res;
    param->hw_base.base.set_fps = set_fps;
    param->hw_base.base.get_fps = get_fps;
    param->hw_base.base.set_gop = set_gop;
    param->hw_base.base.get_gop = get_gop;
    param->hw_base.base.set_bitrate = set_bitrate;
    param->hw_base.base.get_bitrate = get_bitrate;
    param->hw_base.cfg_mv = cfg_mv;
    param->hw_base.get_mv_cfg_info = get_mv_cfg_info;
    param->hw_base.set_mv_pkt = set_mv_pkt;
    param->hw_base.get_mv_data_len = get_mv_data_len;
    param->hw_base.cfg_roi = cfg_roi;
    param->hw_base.get_roi_cfg_info = get_roi_cfg_info;
    param->hw_base.set_roi_reg = set_roi_reg;
    param->hw_base.get_roi_reg = get_roi_reg;
    *out_handle = &param->hw_base;
    return ret;
__exit__:
    /** Delete the handle */
    esp_h264_enc_hw_del_param(&param->hw_base);
    return ret;
}

/* [REF-PERSIST-R2]
 * Called after the encoder's normal INTERNAL buffers/descriptors are ready.
 * The 1080-capable reference was reserved earlier at GOT_IP, so this function
 * only swaps ownership from the temporary PSRAM pool to the persistent block;
 * it performs no large allocation on normal camera start.
 */
bool esp_h264_enc_hw_ref_720_internal_abtest(esp_h264_enc_param_hw_handle_t handle)
{
#if !HA_H264_REF_720_INTERNAL_ABTEST
    (void)handle;
    return false;
#else
    if (handle == NULL) {
        return false;
    }

    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    const bool is_720_class = ((param->width == 1288U && param->height == 728U) ||
                               (param->width == 1280U && param->height == 720U));
    const bool is_1080 = (param->width == 1920U && param->height == 1080U);
    if (!is_720_class && !is_1080) {
        return false;
    }

    const uint32_t ref_need = (uint32_t)max_refame_buffer_size(param->mb_width);

    /* Only replace the PSRAM ref already acquired by this parameter object. */
    if (param->ref == NULL || !param->ref_from_pool || s_ref_pool == NULL ||
        param->ref != s_ref_pool || !s_ref_pool_busy || ref_need > s_ref_pool_size) {
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R2] %ux%u need=%u decision=PSRAM reason=pool-state",
                      (unsigned)param->width, (unsigned)param->height, (unsigned)ref_need);
        return false;
    }

    /* GOT_IP normally reserves this before camera services become usable.
     * Keep one lazy retry for non-network diagnostics / unusual boot ordering;
     * if it cannot reserve safely, PSRAM remains active. */
    if (s_ref_internal_persist == NULL) {
        const esp_h264_err_t r = esp_h264_enc_hw_ref_persist_reserve();
        if (r != ESP_H264_ERR_OK || s_ref_internal_persist == NULL) {
            ESP_H264_LOGI(TAG,
                          "[REF-PERSIST-R2] %ux%u need=%u decision=PSRAM reason=reserve-unavailable ret=%d",
                          (unsigned)param->width, (unsigned)param->height,
                          (unsigned)ref_need, (int)r);
            return false;
        }
    }

    if (s_ref_internal_persist_busy || ref_need > s_ref_internal_persist_size) {
        ESP_H264_LOGI(TAG,
                      "[REF-PERSIST-R2] %ux%u need=%u persistent=%u busy=%u decision=PSRAM reason=persistent-state",
                      (unsigned)param->width, (unsigned)param->height,
                      (unsigned)ref_need, (unsigned)s_ref_internal_persist_size,
                      s_ref_internal_persist_busy ? 1U : 0U);
        return false;
    }

    /* No allocation here. The ~138KB block is already outside the heap, so
     * using it does not consume additional DMA headroom at camera start. */
    memset(s_ref_internal_persist, 0, ref_need);
    esp_h264_cache_check_and_writeback(s_ref_internal_persist, ref_need);

    param->ref = s_ref_internal_persist;
    param->ref_from_pool = false;
    param->ref_from_internal_persist = true;
    s_ref_internal_persist_busy = true;
    s_ref_pool_busy = false;

    const uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    const size_t dma_free_now = heap_caps_get_free_size(dma_caps);
    const size_t dma_largest_now = heap_caps_get_largest_free_block(dma_caps);
    ESP_H264_LOGI(TAG,
                  "[REF-PERSIST-R3B] %ux%u need=%u persistent=%u small=%u dma_now=%u/%u decision=INTERNAL-PERSIST",
                  (unsigned)param->width, (unsigned)param->height,
                  (unsigned)ref_need, (unsigned)s_ref_internal_persist_size,
                  (unsigned)s_h264_small_persist_size,
                  (unsigned)dma_free_now, (unsigned)dma_largest_now);
    return true;
#endif
}

esp_h264_err_t esp_h264_enc_hw_get_mutex(esp_h264_enc_param_hw_handle_t handle, esp_h264_mutex_t *out_mutex)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    *out_mutex = param->mutex;
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_get_qp_init(esp_h264_enc_param_hw_handle_t handle, uint8_t *out_qp_init)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    *out_qp_init = param->qp_init;
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_get_nal(esp_h264_enc_param_hw_handle_t handle, uint8_t *out_nal_buf, uint16_t *out_nal_bit_len)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    memcpy(out_nal_buf, param->nal_buf, param->nal_bit_len >> 3);
    *out_nal_bit_len = param->nal_bit_len;
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_cfg_dma_db_ref(esp_h264_enc_param_hw_handle_t handle, h264_dma_hal_context_t *dma2d_hal)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    uint32_t buff_addr = (uint32_t)ALIGN_UP((uint32_t)param->ref, 8);
    cfg_dsc(param->dsc_ref, H264_DMA_2D_ENABLE, H264_DMA_MODE1, H264_DMA_3_LINES, H264_DMA_MACRO_SIZE * H264_DMA_MACRO_SIZE, H264_DMA_EOF_END,
            H264_DMA_OWNER_H264, H264_DMA_3_LINES, H264_DMA_MACRO_SIZE * H264_DMA_MACRO_SIZE * param->mb_width, (uint8_t *)buff_addr, param->dsc_ref);
    h264_dma_hal_cfg_ref_dsc(dma2d_hal, (uint32_t)param->dsc_ref, buff_addr, param->mb_width);
    uint32_t size = H264_DMA_DB_12_LINES_ROW_LENGTH * param->mb_width * (param->mb_height - 1)
                    + (H264_DMA_DB_12_LINES_ROW_LENGTH + H264_DMA_DB_4_LINES_ROW_LENGTH) * param->mb_width;
    buff_addr = (uint32_t)ALIGN_UP((uint32_t)param->db, 8);
    cfg_dsc(param->dsc_db[0], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), (uint8_t *)buff_addr, param->dsc_db[0]);
    cfg_dsc(param->dsc_db[1], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), (uint8_t *)buff_addr, param->dsc_db[1]);
    buff_addr = (uint32_t)ALIGN_UP((uint32_t)buff_addr + size, 8);
    size = H264_DMA_DB_4_LINES_ROW_LENGTH * param->mb_width * (param->mb_height - 1);
    cfg_dsc(param->dsc_db[2], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), (uint8_t *)buff_addr, param->dsc_db[2]);
    cfg_dsc(param->dsc_db[3], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), (uint8_t *)buff_addr, param->dsc_db[3]);
    h264_dma_hal_cfg_db12_4_dsc(dma2d_hal, (uint32_t *)param->dsc_db);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_cfg_dma_yuv_bs(esp_h264_enc_param_hw_handle_t handle, h264_dma_hal_context_t *dma2d_hal, h264_dma_desc_t *dsc_yuv, uint8_t *buf_yuv, h264_dma_desc_t *dsc_bs, uint8_t *buf_bs, uint32_t buf_bs_len)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    float hb = 96  / param->bpp;
    cfg_dsc(dsc_yuv, H264_DMA_2D_ENABLE, H264_DMA_MODE1, H264_DMA_MACRO_SIZE, (int)hb, H264_DMA_EOF_CONTINUE, H264_DMA_OWNER_H264,
            param->height, param->width, buf_yuv, NULL);
    h264_dma_hal_cfg_yuv_dsc(dma2d_hal, (uint32_t)dsc_yuv);
    cfg_dsc(dsc_bs, H264_DMA_2D_DISABLE, H264_DMA_MODE0, buf_bs_len & H264_DMA_MAX_SIZE, 0, H264_DMA_EOF_END, H264_DMA_OWNER_H264, (buf_bs_len >> H264_DMA_SIZE_BIT),
            0, buf_bs, NULL);
    h264_dma_hal_cfg_bs_dsc(dma2d_hal, (uint32_t)dsc_bs);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_cfg_dma_dbtmp(esp_h264_enc_param_hw_handle_t handle, h264_dma_hal_context_t *dma2d_hal, h264_dma_desc_t *dsc[2], uint8_t *buf)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    uint32_t size = H264_DMA_DB_4_LINES_ROW_LENGTH * param->mb_width;
    cfg_dsc(dsc[0], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), buf, dsc[0]);
    cfg_dsc(dsc[1], H264_DMA_2D_DISABLE, H264_DMA_MODE0, size & H264_DMA_MAX_SIZE, size & H264_DMA_MAX_SIZE, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (size >> H264_DMA_SIZE_BIT), (size >> H264_DMA_SIZE_BIT), buf, dsc[1]);
    h264_dma_hal_cfg_dbtmp_dsc(dma2d_hal, (uint32_t *)dsc);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_cfg_dma_mvm(esp_h264_enc_param_hw_handle_t handle, h264_dma_hal_context_t *dma2d_hal)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    int8_t mv_mode = 0;
    uint8_t mv_fmt = 0;
    h264_hal_get_mv_mode(param->device, &mv_mode, &mv_fmt);
    if (mv_mode == ESP_H264_MVM_MODE_DISABLE) {
        return ESP_H264_ERR_OK;
    }
    if (param->mvm_buf == NULL) {
        return ESP_H264_ERR_FAIL;
    }
    cfg_dsc(param->dsc_mvm, H264_DMA_2D_DISABLE, H264_DMA_MODE0, param->mvm_buf_len & H264_DMA_MAX_SIZE, 0, H264_DMA_EOF_END, H264_DMA_OWNER_H264,
            (param->mvm_buf_len >> H264_DMA_SIZE_BIT), 0, param->mvm_buf, NULL);
    h264_dma_hal_cfg_mvm_dsc(dma2d_hal, (uint32_t)param->dsc_mvm);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_set_qp(esp_h264_enc_param_hw_handle_t handle, uint8_t qp)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    h264_hal_set_qp(param->device, qp);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_get_rc_hd(esp_h264_enc_param_hw_handle_t handle, esp_h264_rc_hd_t *out_rc_hd)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    *out_rc_hd = param->rc_hd;
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_set_rc_rate_pred(esp_h264_enc_param_hw_handle_t handle, int32_t u, int32_t pred)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    h264_hal_set_rc_rate_pred(param->device, u, pred);
    return ESP_H264_ERR_OK;
}

esp_h264_err_t esp_h264_enc_hw_get_mbres(esp_h264_enc_param_hw_handle_t handle, uint8_t *out_mb_width, uint8_t *out_mb_height)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    h264_hal_get_mbres(param->device, out_mb_width, out_mb_height);
    return ESP_H264_ERR_OK;
}

#if HAL_CONFIG(CHIP_SUPPORT_MIN_REV) >= 300
void esp_h264_enc_hw_set_pbyte(esp_h264_enc_param_hw_handle_t handle, h264_dma_hal_context_t *dma2d_hal)
{
    esp_h264_param_t *param = __containerof(handle, esp_h264_param_t, hw_base);
    h264_dma_hal_set_pbyte(dma2d_hal, param->pbyte);
}
#endif

esp_h264_err_t esp_h264_enc_hw_res_check(uint16_t width, uint16_t height)
{
    if ((width >= ESP_H264_MIN_WIDTH)
            && (width <= ESP_H264_MAX_WIDTH)
            && (height >= ESP_H264_MIN_HEIGHT)
            && (height <= ESP_H264_MAX_HEIGHT)) {
        return ESP_H264_ERR_OK;
    }
    return ESP_H264_ERR_FAIL;
}

/*  Dual stream mode, DB tmp  buffer can be resused. */
uint16_t esp_h264_enc_hw_max_db_tmp_buffer_size(uint16_t width)
{
    uint16_t mb_xsize = (width + 15) >> 4;
    return mb_xsize * H264_DMA_DB_4_LINES_ROW_LENGTH + HW_DMA_BUF_ALIGN_SIZE + ESP_H264_REDUNDANT_BYTE;
}

uint8_t *esp_h264_enc_hw_slice_header_align8(uint8_t *bs, int32_t bit_len, h264_hal_context_t *h264_hal)
{
    int32_t blen = bit_len >> 3;
    int32_t unaligned_blen = (blen & HW_DMA_BUF_ALIGN_SIZE);
    uint32_t header[3];
    uint8_t *src = bs + (blen & (~HW_DMA_BUF_ALIGN_SIZE));
    uint8_t *dst = (uint8_t *)(&header[0]);
    for (int32_t i = 0; i < unaligned_blen; i++) {
        dst[7 - i] = src[i];
    }
    header[2] = bs[((bit_len + HW_DMA_BUF_ALIGN_SIZE)  >> 3) - 1];
    int32_t slice_header_bits = (unaligned_blen << 3) + (bit_len & HW_DMA_BUF_ALIGN_SIZE);
    h264_hal_set_slice_header(h264_hal, header, slice_header_bits);
    return src;
}
