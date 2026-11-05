/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Modified in this project: timing/profile selection and gain/AE diagnostics.
 * Public example snapshot 2026-11-05; see ../../NOTICE.md. */
 #include <string.h>
 #include <freertos/FreeRTOS.h>
 #include <freertos/task.h>
 #include "driver/gpio.h"
 #include "esp_err.h"
 #include "esp_log.h"
#include "esp_timer.h"
 
 #include "esp_cam_sensor.h"
 #include "esp_cam_sensor_detect.h"
 #include "ov02c10_settings.h"
 #include "ov02c10.h"
#include "ov02c10_webrtc_profile.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif

#define OV02C10_SENSOR_NAME "OV02C10"

 typedef struct {
    uint8_t dgain_fine; // digital gain fine
    uint8_t dgain_coarse; // digital gain coarse
    uint8_t analog_gain;
} ov02c10_gain_t;

typedef struct {
    uint32_t exposure_val;
    uint32_t exposure_max;
    uint32_t gain_index; // current gain index

    uint32_t vflip_en : 1;
    uint32_t hmirror_en : 1;
} ov02c10_para_t;

struct ov02c10_cam {
    ov02c10_para_t ov02c10_para;
};

#define OV02C10_VTS_MAX          0x46c // Max exposure is VTS-15
#define OV02C10_EXP_MAX_OFFSET   0x0f

// #define OV02C10_FETCH_EXP_M(val)     (((val) >> 4) & 0xFF)
// #define OV02C10_FETCH_EXP_L(val)     (((val) & 0xF) << 4)

#define OV02C10_FETCH_EXP_M(val)     (((val) & 0xFF00) >> 8)
#define OV02C10_FETCH_EXP_L(val)     (((val) & 0xFF))

#define OV02C10_FETCH_DGAIN_FINE_H(val)  (val >> 2)
#define OV02C10_FETCH_DGAIN_FINE_L(val)  (((val) & 0x01) << 6)

#define OV02C10_GROUP_HOLD_START        0x00
#define OV02C10_GROUP_HOLD_END          0x10
#define OV02C10_GROUP_HOLD_DELAY_FRAMES 0x11
 
 #define OV02C10_IO_MUX_LOCK(mux)
 #define OV02C10_IO_MUX_UNLOCK(mux)
 #define OV02C10_ENABLE_OUT_CLOCK(pin,clk)
 #define OV02C10_DISABLE_OUT_CLOCK(pin)

 #define EXPOSURE_V4L2_UNIT_US                   100
#define EXPOSURE_V4L2_TO_OV02C10(v, sf)          \
    ((uint32_t)(((double)v) * (sf)->fps * (sf)->isp_info->isp_v1_info.vts / (1000000 / EXPOSURE_V4L2_UNIT_US) + 0.5))
#define EXPOSURE_OV02C10_TO_V4L2(v, sf)          \
    ((int32_t)(((double)v) * 1000000 / (sf)->fps / (sf)->isp_info->isp_v1_info.vts / EXPOSURE_V4L2_UNIT_US + 0.5))
 
 #define OV02C10_PID         0x5602
 #define OV02C10_AE_TARGET_DEFAULT (0x50)
 
 #ifndef portTICK_RATE_MS
 #define portTICK_RATE_MS portTICK_PERIOD_MS
 #endif
 #define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
 #define OV02C10_SUPPORT_NUM CONFIG_CAMERA_OV02C10_MAX_SUPPORT


 static const uint8_t s_ov02c10_exp_min = 0x08;
 static const uint32_t s_limited_gain = CONFIG_CAMERA_OV02C10_ABSOLUTE_GAIN_LIMIT;
 static size_t s_limited_gain_index;

 static const char *TAG = "ov02c10";

/* [OV02C10-AE-ALLPATH-DIAG-R2-20260922]
 * R1 only observed ESP_CAM_SENSOR_GROUP_EXP_GAIN. Espressif's ISP pipeline can
 * also emit EXPOSURE_VAL / EXPOSURE_US and GAIN independently when ET/GN
 * metadata do not arrive together. R2 instruments ALL four command paths so a
 * flat R1 counter cannot be mistaken for a frozen AE loop.
 *
 * HOLD is also applied consistently to grouped and split commands. Nothing is
 * persisted and normal operation resumes automatically when the timer expires.
 */
#define OV02C10_AE_DIAG_MAX_MS       30000U
#define OV02C10_GAIN_FORCE_MAX_MS      60000U
#define OV02C10_AE_DIAG_LOG_MS         250U
#define OV02C10_AE_PATH_NONE              0U
#define OV02C10_AE_PATH_EXP_VAL           1U
#define OV02C10_AE_PATH_EXP_US            2U
#define OV02C10_AE_PATH_GAIN              3U
#define OV02C10_AE_PATH_GROUP             4U

typedef struct {
    volatile uint32_t watch_until_ms;
    volatile uint32_t hold_until_ms;
    volatile uint32_t hold_armed;
    volatile uint32_t hold_active;
    volatile uint32_t hold_exp;
    volatile uint32_t hold_gain;
    volatile uint32_t gain_force_until_ms;
    volatile uint32_t gain_force_value;
    volatile uint32_t gain_force_enabled;
    volatile uint32_t gain_force_suppressed;
    /* [CAM-USER-GAIN-R23-20260923]
     * Production manual sensor gain. Unlike the old diagnostic gain_force,
     * this has no lease and stays in RAM until the backend applies another
     * profile preference. NVS ownership lives in web_smoke.c. */
    volatile uint32_t user_gain_enabled;
    volatile uint32_t user_gain_value;
    volatile uint32_t last_req_exp;
    volatile uint32_t last_req_gain;
    volatile uint32_t last_applied_exp;
    volatile uint32_t last_applied_gain;
    volatile uint32_t write_count;
    volatile uint32_t group_count;
    volatile uint32_t exp_count;
    volatile uint32_t gain_count;
    volatile uint32_t last_path;
    volatile uint32_t suppressed_count;
    volatile uint32_t last_log_ms;
} ov02c10_ae_diag_t;

static ov02c10_ae_diag_t s_ae_diag;
static esp_cam_sensor_device_t *s_ae_diag_dev;



/* [CAM-AUTOCAL-GAIN-SWEEP-R15-20260922]
 * Experimental 720p60-only gain force. This is separate from the short full
 * AE/AGC HOLD lease: exposure remains live until R14/R15 starts its normal
 * candidate-search HOLD, while sensor gain is clamped to the user-selected
 * test index. Nothing is persisted. */
static inline bool ov02c10_ae_diag_is_720p60(const esp_cam_sensor_device_t *dev)
{
    return dev && dev->cur_format &&
           dev->cur_format->width == 1288U &&
           dev->cur_format->height == 728U &&
           dev->cur_format->fps >= 50U;
}

static inline uint32_t ov02c10_ae_diag_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline bool ov02c10_ae_diag_deadline_active(uint32_t deadline, uint32_t now)
{
    return deadline != 0U && (int32_t)(deadline - now) > 0;
}

static const char *ov02c10_ae_diag_path_name(uint32_t path)
{
    switch (path) {
    case OV02C10_AE_PATH_EXP_VAL: return "EXP_VAL";
    case OV02C10_AE_PATH_EXP_US:  return "EXP_US";
    case OV02C10_AE_PATH_GAIN:    return "GAIN";
    case OV02C10_AE_PATH_GROUP:   return "GROUP";
    default:                      return "NONE";
    }
}

static bool ov02c10_ae_diag_gain_force_active_for(const esp_cam_sensor_device_t *dev,
                                                   uint32_t now_ms,
                                                   uint32_t *gain_out)
{
    const uint32_t until = __atomic_load_n(&s_ae_diag.gain_force_until_ms, __ATOMIC_ACQUIRE);
    const bool active = ov02c10_ae_diag_is_720p60(dev) &&
                        __atomic_load_n(&s_ae_diag.gain_force_enabled, __ATOMIC_ACQUIRE) != 0U &&
                        ov02c10_ae_diag_deadline_active(until, now_ms);
    if (active && gain_out) {
        *gain_out = __atomic_load_n(&s_ae_diag.gain_force_value, __ATOMIC_ACQUIRE);
    }
    if (!ov02c10_ae_diag_deadline_active(until, now_ms) &&
        __atomic_load_n(&s_ae_diag.gain_force_enabled, __ATOMIC_ACQUIRE) != 0U) {
        __atomic_store_n(&s_ae_diag.gain_force_enabled, 0U, __ATOMIC_RELEASE);
    }
    return active;
}

void ov02c10_ae_diag_gain_force_get(uint32_t *gain_value,
                                    uint32_t *left_ms,
                                    uint32_t *suppressed_count,
                                    bool *active)
{
    const uint32_t now = ov02c10_ae_diag_now_ms();
    const uint32_t until = __atomic_load_n(&s_ae_diag.gain_force_until_ms, __ATOMIC_ACQUIRE);
    const bool deadline = ov02c10_ae_diag_deadline_active(until, now);
    const bool en = deadline &&
                    __atomic_load_n(&s_ae_diag.gain_force_enabled, __ATOMIC_ACQUIRE) != 0U;
    if (gain_value) *gain_value = __atomic_load_n(&s_ae_diag.gain_force_value, __ATOMIC_ACQUIRE);
    if (left_ms) *left_ms = en ? (until - now) : 0U;
    if (suppressed_count) {
        *suppressed_count = __atomic_load_n(&s_ae_diag.gain_force_suppressed, __ATOMIC_ACQUIRE);
    }
    if (active) *active = en;
}

esp_err_t ov02c10_ae_diag_watch_ms(uint32_t ms)
{
    if (ms == 0U || ms > OV02C10_AE_DIAG_MAX_MS) return ESP_ERR_INVALID_ARG;
    const uint32_t now = ov02c10_ae_diag_now_ms();
    __atomic_store_n(&s_ae_diag.watch_until_ms, now + ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.last_log_ms, 0U, __ATOMIC_RELEASE);
    ESP_LOGW(TAG, "[AE-DIAG-R2] WATCH start ms=%"PRIu32, ms);
    return ESP_OK;
}

esp_err_t ov02c10_ae_diag_hold_ms(uint32_t ms)
{
    if (ms == 0U || ms > OV02C10_AE_DIAG_MAX_MS) return ESP_ERR_INVALID_ARG;
    const uint32_t now = ov02c10_ae_diag_now_ms();
    __atomic_store_n(&s_ae_diag.hold_until_ms, now + ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.hold_active, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.hold_armed, 1U, __ATOMIC_RELEASE);
    const uint32_t watch_ms = MIN(ms + 3000U, OV02C10_AE_DIAG_MAX_MS);
    __atomic_store_n(&s_ae_diag.watch_until_ms, now + watch_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.last_log_ms, 0U, __ATOMIC_RELEASE);
    ESP_LOGW(TAG, "[AE-DIAG-R2] HOLD armed ms=%"PRIu32, ms);
    return ESP_OK;
}

void ov02c10_ae_diag_release(void)
{
    __atomic_store_n(&s_ae_diag.hold_until_ms, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.hold_armed, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.hold_active, 0U, __ATOMIC_RELEASE);
    ESP_LOGW(TAG, "[AE-DIAG-R2] HOLD release requested");
}

void ov02c10_ae_diag_get(uint32_t *req_exp, uint32_t *req_gain,
                         uint32_t *applied_exp, uint32_t *applied_gain,
                         uint32_t *hold_left_ms, uint32_t *write_count,
                         uint32_t *group_count, uint32_t *exp_count,
                         uint32_t *gain_count, uint32_t *last_path,
                         uint32_t *suppressed_count, bool *hold_active)
{
    const uint32_t now = ov02c10_ae_diag_now_ms();
    const uint32_t until = __atomic_load_n(&s_ae_diag.hold_until_ms, __ATOMIC_ACQUIRE);
    if (req_exp) *req_exp = __atomic_load_n(&s_ae_diag.last_req_exp, __ATOMIC_ACQUIRE);
    if (req_gain) *req_gain = __atomic_load_n(&s_ae_diag.last_req_gain, __ATOMIC_ACQUIRE);
    if (applied_exp) *applied_exp = __atomic_load_n(&s_ae_diag.last_applied_exp, __ATOMIC_ACQUIRE);
    if (applied_gain) *applied_gain = __atomic_load_n(&s_ae_diag.last_applied_gain, __ATOMIC_ACQUIRE);
    if (write_count) *write_count = __atomic_load_n(&s_ae_diag.write_count, __ATOMIC_ACQUIRE);
    if (group_count) *group_count = __atomic_load_n(&s_ae_diag.group_count, __ATOMIC_ACQUIRE);
    if (exp_count) *exp_count = __atomic_load_n(&s_ae_diag.exp_count, __ATOMIC_ACQUIRE);
    if (gain_count) *gain_count = __atomic_load_n(&s_ae_diag.gain_count, __ATOMIC_ACQUIRE);
    if (last_path) *last_path = __atomic_load_n(&s_ae_diag.last_path, __ATOMIC_ACQUIRE);
    if (suppressed_count) *suppressed_count = __atomic_load_n(&s_ae_diag.suppressed_count, __ATOMIC_ACQUIRE);
    const bool active = ov02c10_ae_diag_deadline_active(until, now) &&
                        (__atomic_load_n(&s_ae_diag.hold_armed, __ATOMIC_ACQUIRE) ||
                         __atomic_load_n(&s_ae_diag.hold_active, __ATOMIC_ACQUIRE));
    if (hold_active) *hold_active = active;
    if (hold_left_ms) *hold_left_ms = active ? (until - now) : 0U;
}

static bool ov02c10_ae_diag_prepare_hold(struct ov02c10_cam *cam, uint32_t now_ms,
                                         uint32_t *hold_until_out)
{
    const uint32_t hold_until = __atomic_load_n(&s_ae_diag.hold_until_ms, __ATOMIC_ACQUIRE);
    if (hold_until_out) *hold_until_out = hold_until;
    const bool hold_window = ov02c10_ae_diag_deadline_active(hold_until, now_ms);
    if (hold_window) {
        if (__atomic_exchange_n(&s_ae_diag.hold_armed, 0U, __ATOMIC_ACQ_REL) != 0U) {
            __atomic_store_n(&s_ae_diag.hold_exp, cam->ov02c10_para.exposure_val, __ATOMIC_RELEASE);
            __atomic_store_n(&s_ae_diag.hold_gain, cam->ov02c10_para.gain_index, __ATOMIC_RELEASE);
            __atomic_store_n(&s_ae_diag.hold_active, 1U, __ATOMIC_RELEASE);
            ESP_LOGW(TAG, "[AE-DIAG-R2] HOLD start exp=%"PRIu32" gain=%"PRIu32,
                     cam->ov02c10_para.exposure_val, cam->ov02c10_para.gain_index);
        }
        return __atomic_load_n(&s_ae_diag.hold_active, __ATOMIC_ACQUIRE) != 0U;
    }
    const uint32_t was_active = __atomic_exchange_n(&s_ae_diag.hold_active, 0U, __ATOMIC_ACQ_REL);
    __atomic_store_n(&s_ae_diag.hold_armed, 0U, __ATOMIC_RELEASE);
    if (was_active != 0U) ESP_LOGW(TAG, "[AE-DIAG-R2] HOLD end -> live control resumed");
    return false;
}

static void ov02c10_ae_diag_record(uint32_t path, uint32_t now_ms, uint32_t hold_until,
                                   bool hold_active)
{
    __atomic_store_n(&s_ae_diag.last_path, path, __ATOMIC_RELEASE);
    const uint32_t writes = __atomic_add_fetch(&s_ae_diag.write_count, 1U, __ATOMIC_RELAXED);
    if (path == OV02C10_AE_PATH_GROUP) __atomic_add_fetch(&s_ae_diag.group_count, 1U, __ATOMIC_RELAXED);
    else if (path == OV02C10_AE_PATH_EXP_VAL || path == OV02C10_AE_PATH_EXP_US) __atomic_add_fetch(&s_ae_diag.exp_count, 1U, __ATOMIC_RELAXED);
    else if (path == OV02C10_AE_PATH_GAIN) __atomic_add_fetch(&s_ae_diag.gain_count, 1U, __ATOMIC_RELAXED);

    const uint32_t watch_until = __atomic_load_n(&s_ae_diag.watch_until_ms, __ATOMIC_ACQUIRE);
    const bool watch = ov02c10_ae_diag_deadline_active(watch_until, now_ms) || hold_active;
    if (watch) {
        const uint32_t last_log = __atomic_load_n(&s_ae_diag.last_log_ms, __ATOMIC_ACQUIRE);
        if (last_log == 0U || (uint32_t)(now_ms - last_log) >= OV02C10_AE_DIAG_LOG_MS) {
            __atomic_store_n(&s_ae_diag.last_log_ms, now_ms, __ATOMIC_RELEASE);
            ESP_LOGW(TAG,
                     "[AE-DIAG-R2] n=%"PRIu32" path=%s hold=%u left=%"PRIu32
                     " req{exp=%"PRIu32" gain=%"PRIu32"}"
                     " apply{exp=%"PRIu32" gain=%"PRIu32"}"
                     " cnt{grp=%"PRIu32" exp=%"PRIu32" gain=%"PRIu32"} suppressed=%"PRIu32,
                     writes, ov02c10_ae_diag_path_name(path), hold_active ? 1U : 0U,
                     hold_active ? (hold_until - now_ms) : 0U,
                     __atomic_load_n(&s_ae_diag.last_req_exp, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.last_req_gain, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.last_applied_exp, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.last_applied_gain, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.group_count, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.exp_count, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.gain_count, __ATOMIC_ACQUIRE),
                     __atomic_load_n(&s_ae_diag.suppressed_count, __ATOMIC_ACQUIRE));
        }
    }
}

/* [CAM-AUTOCAL-720-AUTO-GAIN223-R17-20260922]
 * 720p60 is exposure-limited in dark scenes, so allow normal IPA/AGC to use
 * the already-validated R16 64x..126x register combinations automatically.
 * 1080 keeps the original 0..191 AGC range. 720 gets 0..223. */
#if CONFIG_CAMERA_OV02C10_ANA_GAIN_PRIORITY
#define OV02C10_R17_NORMAL_GAIN_COUNT   192U
#define OV02C10_R17_720_GAIN_COUNT      224U
#define OV02C10_R17_720_GAIN_MAX_INDEX  223U
#endif
 
#if CONFIG_CAMERA_OV02C10_ANA_GAIN_PRIORITY
// total gain = analog_gain x digital_gain x 1000(To avoid decimal points, the final abs_gain is multiplied by 1000.)
static const uint32_t ov02c10_total_gain_val_map[] = {
    1000,
    1031,
    1063,
    1094,
    1125,
    1156,
    1188,
    1219,
    1250,
    1281,
    1313,
    1344,
    1375,
    1406,
    1438,
    1469,
    1500,
    1531,
    1563,
    1594,
    1625,
    1656,
    1688,
    1719,
    1750,
    1781,
    1813,
    1844,
    1875,
    1906,
    1938,
    1969,
    // 2X
    2000,
    2062,
    2126,
    2188,
    2250,
    2312,
    2376,
    2438,
    2500,
    2562,
    2626,
    2688,
    2750,
    2812,
    2876,
    2938,
    3000,
    3062,
    3126,
    3188,
    3250,
    3312,
    3376,
    3438,
    3500,
    3562,
    3626,
    3688,
    3750,
    3812,
    3876,
    3938,
    // 4X
    4000,
    4124,
    4252,
    4376,
    4500,
    4624,
    4752,
    4876,
    5000,
    5124,
    5252,
    5376,
    5500,
    5624,
    5752,
    5876,
    6000,
    6124,
    6252,
    6376,
    6500,
    6624,
    6752,
    6876,
    7000,
    7124,
    7252,
    7376,
    7500,
    7624,
    7752,
    7876,
    // 8X
    8000,
    8248,
    8504,
    8752,
    9000,
    9248,
    9504,
    9752,
    10000,
    10248,
    10504,
    10752,
    11000,
    11248,
    11504,
    11752,
    12000,
    12248,
    12504,
    12752,
    13000,
    13248,
    13504,
    13752,
    14000,
    14248,
    14504,
    14752,
    15000,
    15248,
    15504,
    15752,
    // 16X
    16000,
    16496,
    17008,
    17504,
    18000,
    18496,
    19008,
    19504,
    20000,
    20496,
    21008,
    21504,
    22000,
    22496,
    23008,
    23504,
    24000,
    24496,
    25008,
    25504,
    26000,
    26496,
    27008,
    27504,
    28000,
    28496,
    29008,
    29504,
    30000,
    30496,
    31008,
    31504,
    // 32X
    32000,
    32992,
    34016,
    35008,
    36000,
    36992,
    38016,
    39008,
    40000,
    40992,
    42016,
    43008,
    44000,
    44992,
    46016,
    47008,
    48000,
    48992,
    50016,
    51008,
    52000,
    52992,
    54016,
    55008,
    56000,
    56992,
    58016,
    59008,
    60000,
    60992,
    62016,
    63008,
    // R17 720-only AUTO extension: 64X .. ~126X
    64000,
    66016,
    68000,
    70016,
    72000,
    74016,
    76000,
    78016,
    80000,
    82016,
    84000,
    86016,
    88000,
    90016,
    92000,
    94016,
    96000,
    98016,
    100000,
    102016,
    104000,
    106016,
    108000,
    110016,
    112000,
    114016,
    116000,
    118016,
    120000,
    122016,
    124000,
    126016,
};

// OV02C10 Gain map format: [DIG_FINE, DIG_COARSE, ANG]
static const ov02c10_gain_t ov02c10_gain_map[] = {
    {0x80, 0x01, 0x01},
    {0x84, 0x01, 0x01},
    {0x88, 0x01, 0x01},
    {0x8c, 0x01, 0x01},
    {0x90, 0x01, 0x01},
    {0x94, 0x01, 0x01},
    {0x98, 0x01, 0x01},
    {0x9c, 0x01, 0x01},
    {0xa0, 0x01, 0x01},
    {0xa4, 0x01, 0x01},
    {0xa8, 0x01, 0x01},
    {0xac, 0x01, 0x01},
    {0xb0, 0x01, 0x01},
    {0xb4, 0x01, 0x01},
    {0xb8, 0x01, 0x01},
    {0xbc, 0x01, 0x01},
    {0xc0, 0x01, 0x01},
    {0xc4, 0x01, 0x01},
    {0xc8, 0x01, 0x01},
    {0xcc, 0x01, 0x01},
    {0xd0, 0x01, 0x01},
    {0xd4, 0x01, 0x01},
    {0xd8, 0x01, 0x01},
    {0xdc, 0x01, 0x01},
    {0xe0, 0x01, 0x01},
    {0xe4, 0x01, 0x01},
    {0xe8, 0x01, 0x01},
    {0xec, 0x01, 0x01},
    {0xf0, 0x01, 0x01},
    {0xf4, 0x01, 0x01},
    {0xf8, 0x01, 0x01},
    {0xfc, 0x01, 0x01},
    // 2X
    {0x80, 0x01, 0x02},
    {0x84, 0x01, 0x02},
    {0x88, 0x01, 0x02},
    {0x8c, 0x01, 0x02},
    {0x90, 0x01, 0x02},
    {0x94, 0x01, 0x02},
    {0x98, 0x01, 0x02},
    {0x9c, 0x01, 0x02},
    {0xa0, 0x01, 0x02},
    {0xa4, 0x01, 0x02},
    {0xa8, 0x01, 0x02},
    {0xac, 0x01, 0x02},
    {0xb0, 0x01, 0x02},
    {0xb4, 0x01, 0x02},
    {0xb8, 0x01, 0x02},
    {0xbc, 0x01, 0x02},
    {0xc0, 0x01, 0x02},
    {0xc4, 0x01, 0x02},
    {0xc8, 0x01, 0x02},
    {0xcc, 0x01, 0x02},
    {0xd0, 0x01, 0x02},
    {0xd4, 0x01, 0x02},
    {0xd8, 0x01, 0x02},
    {0xdc, 0x01, 0x02},
    {0xe0, 0x01, 0x02},
    {0xe4, 0x01, 0x02},
    {0xe8, 0x01, 0x02},
    {0xec, 0x01, 0x02},
    {0xf0, 0x01, 0x02},
    {0xf4, 0x01, 0x02},
    {0xf8, 0x01, 0x02},
    {0xfc, 0x01, 0x02},
    // 4X
    {0x80, 0x01, 0x03},
    {0x84, 0x01, 0x03},
    {0x88, 0x01, 0x03},
    {0x8c, 0x01, 0x03},
    {0x90, 0x01, 0x03},
    {0x94, 0x01, 0x03},
    {0x98, 0x01, 0x03},
    {0x9c, 0x01, 0x03},
    {0xa0, 0x01, 0x03},
    {0xa4, 0x01, 0x03},
    {0xa8, 0x01, 0x03},
    {0xac, 0x01, 0x03},
    {0xb0, 0x01, 0x03},
    {0xb4, 0x01, 0x03},
    {0xb8, 0x01, 0x03},
    {0xbc, 0x01, 0x03},
    {0xc0, 0x01, 0x03},
    {0xc4, 0x01, 0x03},
    {0xc8, 0x01, 0x03},
    {0xcc, 0x01, 0x03},
    {0xd0, 0x01, 0x03},
    {0xd4, 0x01, 0x03},
    {0xd8, 0x01, 0x03},
    {0xdc, 0x01, 0x03},
    {0xe0, 0x01, 0x03},
    {0xe4, 0x01, 0x03},
    {0xe8, 0x01, 0x03},
    {0xec, 0x01, 0x03},
    {0xf0, 0x01, 0x03},
    {0xf4, 0x01, 0x03},
    {0xf8, 0x01, 0x03},
    {0xfc, 0x01, 0x03},
    // 8X
    {0x80, 0x01, 0x04},
    {0x84, 0x01, 0x04},
    {0x88, 0x01, 0x04},
    {0x8c, 0x01, 0x04},
    {0x90, 0x01, 0x04},
    {0x94, 0x01, 0x04},
    {0x98, 0x01, 0x04},
    {0x9c, 0x01, 0x04},
    {0xa0, 0x01, 0x04},
    {0xa4, 0x01, 0x04},
    {0xa8, 0x01, 0x04},
    {0xac, 0x01, 0x04},
    {0xb0, 0x01, 0x04},
    {0xb4, 0x01, 0x04},
    {0xb8, 0x01, 0x04},
    {0xbc, 0x01, 0x04},
    {0xc0, 0x01, 0x04},
    {0xc4, 0x01, 0x04},
    {0xc8, 0x01, 0x04},
    {0xcc, 0x01, 0x04},
    {0xd0, 0x01, 0x04},
    {0xd4, 0x01, 0x04},
    {0xd8, 0x01, 0x04},
    {0xdc, 0x01, 0x04},
    {0xe0, 0x01, 0x04},
    {0xe4, 0x01, 0x04},
    {0xe8, 0x01, 0x04},
    {0xec, 0x01, 0x04},
    {0xf0, 0x01, 0x04},
    {0xf4, 0x01, 0x04},
    {0xf8, 0x01, 0x04},
    {0xfc, 0x01, 0x04},
    // 16X
    {0x80, 0x01, 0x05},
    {0x84, 0x01, 0x05},
    {0x88, 0x01, 0x05},
    {0x8c, 0x01, 0x05},
    {0x90, 0x01, 0x05},
    {0x94, 0x01, 0x05},
    {0x98, 0x01, 0x05},
    {0x9c, 0x01, 0x05},
    {0xa0, 0x01, 0x05},
    {0xa4, 0x01, 0x05},
    {0xa8, 0x01, 0x05},
    {0xac, 0x01, 0x05},
    {0xb0, 0x01, 0x05},
    {0xb4, 0x01, 0x05},
    {0xb8, 0x01, 0x05},
    {0xbc, 0x01, 0x05},
    {0xc0, 0x01, 0x05},
    {0xc4, 0x01, 0x05},
    {0xc8, 0x01, 0x05},
    {0xcc, 0x01, 0x05},
    {0xd0, 0x01, 0x05},
    {0xd4, 0x01, 0x05},
    {0xd8, 0x01, 0x05},
    {0xdc, 0x01, 0x05},
    {0xe0, 0x01, 0x05},
    {0xe4, 0x01, 0x05},
    {0xe8, 0x01, 0x05},
    {0xec, 0x01, 0x05},
    {0xf0, 0x01, 0x05},
    {0xf4, 0x01, 0x05},
    {0xf8, 0x01, 0x05},
    {0xfc, 0x01, 0x05},
    //32x
    {0x80, 0x01, 0x06},
    {0x84, 0x01, 0x06},
    {0x88, 0x01, 0x06},
    {0x8c, 0x01, 0x06},
    {0x90, 0x01, 0x06},
    {0x94, 0x01, 0x06},
    {0x98, 0x01, 0x06},
    {0x9c, 0x01, 0x06},
    {0xa0, 0x01, 0x06},
    {0xa4, 0x01, 0x06},
    {0xa8, 0x01, 0x06},
    {0xac, 0x01, 0x06},
    {0xb0, 0x01, 0x06},
    {0xb4, 0x01, 0x06},
    {0xb8, 0x01, 0x06},
    {0xbc, 0x01, 0x06},
    {0xc0, 0x01, 0x06},
    {0xc4, 0x01, 0x06},
    {0xc8, 0x01, 0x06},
    {0xcc, 0x01, 0x06},
    {0xd0, 0x01, 0x06},
    {0xd4, 0x01, 0x06},
    {0xd8, 0x01, 0x06},
    {0xdc, 0x01, 0x06},
    {0xe0, 0x01, 0x06},
    {0xe4, 0x01, 0x06},
    {0xe8, 0x01, 0x06},
    {0xec, 0x01, 0x06},
    {0xf0, 0x01, 0x06},
    {0xf4, 0x01, 0x06},
    {0xf8, 0x01, 0x06},
    {0xfc, 0x01, 0x06},
    // R59 FINAL: 720-only AUTO extension, indices 192..223.
    // Measured candidate sweep selected ANG high byte 0x08:
    // index 191 baseline -> index 192 delta ~= +0.65%, eliminating the old jump.
    {0x80, 0x01, 0x08}, {0x84, 0x01, 0x08}, {0x88, 0x01, 0x08}, {0x8c, 0x01, 0x08},
    {0x90, 0x01, 0x08}, {0x94, 0x01, 0x08}, {0x98, 0x01, 0x08}, {0x9c, 0x01, 0x08},
    {0xa0, 0x01, 0x08}, {0xa4, 0x01, 0x08}, {0xa8, 0x01, 0x08}, {0xac, 0x01, 0x08},
    {0xb0, 0x01, 0x08}, {0xb4, 0x01, 0x08}, {0xb8, 0x01, 0x08}, {0xbc, 0x01, 0x08},
    {0xc0, 0x01, 0x08}, {0xc4, 0x01, 0x08}, {0xc8, 0x01, 0x08}, {0xcc, 0x01, 0x08},
    {0xd0, 0x01, 0x08}, {0xd4, 0x01, 0x08}, {0xd8, 0x01, 0x08}, {0xdc, 0x01, 0x08},
    {0xe0, 0x01, 0x08}, {0xe4, 0x01, 0x08}, {0xe8, 0x01, 0x08}, {0xec, 0x01, 0x08},
    {0xf0, 0x01, 0x08}, {0xf4, 0x01, 0x08}, {0xf8, 0x01, 0x08}, {0xfc, 0x01, 0x08},
};
_Static_assert(ARRAY_SIZE(ov02c10_total_gain_val_map) == OV02C10_R17_720_GAIN_COUNT, "R17 total gain count");
_Static_assert(ARRAY_SIZE(ov02c10_gain_map) == OV02C10_R17_720_GAIN_COUNT, "R17 register gain count");
#elif CONFIG_CAMERA_OV02C10_DIG_GAIN_PRIORITY
// total gain = analog_gain x digital_gain x 1000(To avoid decimal points, the final total_gain is multiplied by 1000.)
static const uint32_t ov02c10_total_gain_val_map[] = {
    1000,
    1031,
    1063,
    1094,
    1125,
    1156,
    1188,
    1219,
    1250,
    1281,
    1313,
    1344,
    1375,
    1406,
    1438,
    1469,
    1500,
    1531,
    1563,
    1594,
    1625,
    1656,
    1688,
    1719,
    1750,
    1781,
    1813,
    1844,
    1875,
    1906,
    1938,
    1969,
    // 2X
    2000,
    2063,
    2125,
    2188,
    2250,
    2313,
    2375,
    2438,
    2500,
    2563,
    2625,
    2688,
    2750,
    2813,
    2875,
    2938,
    3000,
    3063,
    3125,
    3188,
    3250,
    3313,
    3375,
    3438,
    3500,
    3563,
    3625,
    3688,
    3750,
    3813,
    3875,
    3938,
    // 4X
    4000,
    4126,
    4250,
    4376,
    4500,
    4626,
    4750,
    4876,
    5000,
    5126,
    5250,
    5376,
    5500,
    5626,
    5750,
    5876,
    6000,
    6126,
    6250,
    6376,
    6500,
    6626,
    6750,
    6876,
    7000,
    7126,
    7250,
    7376,
    7500,
    7626,
    7750,
    7876,
    // 8X
    8000,
    8252,
    8500,
    8752,
    9000,
    9252,
    9500,
    9752,
    10000,
    10252,
    10500,
    10752,
    11000,
    11252,
    11500,
    11752,
    12000,
    12252,
    12500,
    12752,
    13000,
    13252,
    13500,
    13752,
    14000,
    14252,
    14500,
    14752,
    15000,
    15252,
    15500,
    15752,
    // 16X
    16000,
    16504,
    17000,
    17504,
    18000,
    18504,
    19000,
    19504,
    20000,
    20504,
    21000,
    21504,
    22000,
    22504,
    23000,
    23504,
    24000,
    24504,
    25000,
    25504,
    26000,
    26504,
    27000,
    27504,
    28000,
    28504,
    29000,
    29504,
    30000,
    30504,
    31000,
    31504,
    // 32X
    32000,
    33008,
    34000,
    35008,
    36000,
    37008,
    38000,
    39008,
    40000,
    41008,
    42000,
    43008,
    44000,
    45008,
    46000,
    47008,
    48000,
    49008,
    50000,
    51008,
    52000,
    53008,
    54000,
    55008,
    56000,
    57008,
    58000,
    59008,
    60000,
    61008,
    62000,
    63008,
    // 64X
    64000,
    66016,
    68000,
    70016,
    72000,
    74016,
    76000,
    78016,
    80000,
    82016,
    84000,
    86016,
    88000,
    90016,
    92000,
    94016,
    96000,
    98016,
    100000,
    102016,
    104000,
    106016,
    108000,
    110016,
    112000,
    114016,
    116000,
    118016,
    120000,
    122016,
    124000,
    126016,
};

// OV02C10 Gain map format: [DIG_FINE, DIG_COARSE, ANG]
static const ov02c10_gain_t ov02c10_gain_map[] = {
    {0x80, 0x00, 0x01},
    {0x84, 0x00, 0x01},
    {0x88, 0x00, 0x01},
    {0x8c, 0x00, 0x01},
    {0x90, 0x00, 0x01},
    {0x94, 0x00, 0x01},
    {0x98, 0x00, 0x01},
    {0x9c, 0x00, 0x01},
    {0xa0, 0x00, 0x01},
    {0xa4, 0x00, 0x01},
    {0xa8, 0x00, 0x01},
    {0xac, 0x00, 0x01},
    {0xb0, 0x00, 0x01},
    {0xb4, 0x00, 0x01},
    {0xb8, 0x00, 0x01},
    {0xbc, 0x00, 0x01},
    {0xc0, 0x00, 0x01},
    {0xc4, 0x00, 0x01},
    {0xc8, 0x00, 0x01},
    {0xcc, 0x00, 0x01},
    {0xd0, 0x00, 0x01},
    {0xd4, 0x00, 0x01},
    {0xd8, 0x00, 0x01},
    {0xdc, 0x00, 0x01},
    {0xe0, 0x00, 0x01},
    {0xe4, 0x00, 0x01},
    {0xe8, 0x00, 0x01},
    {0xec, 0x00, 0x01},
    {0xf0, 0x00, 0x01},
    {0xf4, 0x00, 0x01},
    {0xf8, 0x00, 0x01},
    {0xfc, 0x00, 0x01},
    // 2X
    {0x80, 0x01, 0x01},
    {0x84, 0x01, 0x01},
    {0x88, 0x01, 0x01},
    {0x8c, 0x01, 0x01},
    {0x90, 0x01, 0x01},
    {0x94, 0x01, 0x01},
    {0x98, 0x01, 0x01},
    {0x9c, 0x01, 0x01},
    {0xa0, 0x01, 0x01},
    {0xa4, 0x01, 0x01},
    {0xa8, 0x01, 0x01},
    {0xac, 0x01, 0x01},
    {0xb0, 0x01, 0x01},
    {0xb4, 0x01, 0x01},
    {0xb8, 0x01, 0x01},
    {0xbc, 0x01, 0x01},
    {0xc0, 0x01, 0x01},
    {0xc4, 0x01, 0x01},
    {0xc8, 0x01, 0x01},
    {0xcc, 0x01, 0x01},
    {0xd0, 0x01, 0x01},
    {0xd4, 0x01, 0x01},
    {0xd8, 0x01, 0x01},
    {0xdc, 0x01, 0x01},
    {0xe0, 0x01, 0x01},
    {0xe4, 0x01, 0x01},
    {0xe8, 0x01, 0x01},
    {0xec, 0x01, 0x01},
    {0xf0, 0x01, 0x01},
    {0xf4, 0x01, 0x01},
    {0xf8, 0x01, 0x01},
    {0xfc, 0x01, 0x01},
    // 4X
    {0x80, 0x01, 0x01},
    {0x84, 0x01, 0x01},
    {0x88, 0x01, 0x01},
    {0x8c, 0x01, 0x01},
    {0x90, 0x01, 0x01},
    {0x94, 0x01, 0x01},
    {0x98, 0x01, 0x01},
    {0x9c, 0x01, 0x01},
    {0xa0, 0x01, 0x01},
    {0xa4, 0x01, 0x01},
    {0xa8, 0x01, 0x01},
    {0xac, 0x01, 0x01},
    {0xb0, 0x01, 0x01},
    {0xb4, 0x01, 0x01},
    {0xb8, 0x01, 0x01},
    {0xbc, 0x01, 0x01},
    {0xc0, 0x01, 0x01},
    {0xc4, 0x01, 0x01},
    {0xc8, 0x01, 0x01},
    {0xcc, 0x01, 0x01},
    {0xd0, 0x01, 0x01},
    {0xd4, 0x01, 0x01},
    {0xd8, 0x01, 0x01},
    {0xdc, 0x01, 0x01},
    {0xe0, 0x01, 0x01},
    {0xe4, 0x01, 0x01},
    {0xe8, 0x01, 0x01},
    {0xec, 0x01, 0x01},
    {0xf0, 0x01, 0x01},
    {0xf4, 0x01, 0x01},
    {0xf8, 0x01, 0x01},
    {0xfc, 0x01, 0x01},
    // 8X
    {0x80, 0x01, 0x04},
    {0x84, 0x01, 0x04},
    {0x88, 0x01, 0x04},
    {0x8c, 0x01, 0x04},
    {0x90, 0x01, 0x04},
    {0x94, 0x01, 0x04},
    {0x98, 0x01, 0x04},
    {0x9c, 0x01, 0x04},
    {0xa0, 0x01, 0x04},
    {0xa4, 0x01, 0x04},
    {0xa8, 0x01, 0x04},
    {0xac, 0x01, 0x04},
    {0xb0, 0x01, 0x04},
    {0xb4, 0x01, 0x04},
    {0xb8, 0x01, 0x04},
    {0xbc, 0x01, 0x04},
    {0xc0, 0x01, 0x04},
    {0xc4, 0x01, 0x04},
    {0xc8, 0x01, 0x04},
    {0xcc, 0x01, 0x04},
    {0xd0, 0x01, 0x04},
    {0xd4, 0x01, 0x04},
    {0xd8, 0x01, 0x04},
    {0xdc, 0x01, 0x04},
    {0xe0, 0x01, 0x04},
    {0xe4, 0x01, 0x04},
    {0xe8, 0x01, 0x04},
    {0xec, 0x01, 0x04},
    {0xf0, 0x01, 0x04},
    {0xf4, 0x01, 0x04},
    {0xf8, 0x01, 0x04},
    {0xfc, 0x01, 0x04},
    // 16X
    {0x80, 0x01, 0x08},
    {0x84, 0x01, 0x08},
    {0x88, 0x01, 0x08},
    {0x8c, 0x01, 0x08},
    {0x90, 0x01, 0x08},
    {0x94, 0x01, 0x08},
    {0x98, 0x01, 0x08},
    {0x9c, 0x01, 0x08},
    {0xa0, 0x01, 0x08},
    {0xa4, 0x01, 0x08},
    {0xa8, 0x01, 0x08},
    {0xac, 0x01, 0x08},
    {0xb0, 0x01, 0x08},
    {0xb4, 0x01, 0x08},
    {0xb8, 0x01, 0x08},
    {0xbc, 0x01, 0x08},
    {0xc0, 0x01, 0x08},
    {0xc4, 0x01, 0x08},
    {0xc8, 0x01, 0x08},
    {0xcc, 0x01, 0x08},
    {0xd0, 0x01, 0x08},
    {0xd4, 0x01, 0x08},
    {0xd8, 0x01, 0x08},
    {0xdc, 0x01, 0x08},
    {0xe0, 0x01, 0x08},
    {0xe4, 0x01, 0x08},
    {0xe8, 0x01, 0x08},
    {0xec, 0x01, 0x08},
    {0xf0, 0x01, 0x08},
    {0xf4, 0x01, 0x08},
    {0xf8, 0x01, 0x08},
    {0xfc, 0x01, 0x08},
    //32x
    {0x80, 0x01, 0x0f},
    {0x84, 0x01, 0x0f},
    {0x88, 0x01, 0x0f},
    {0x8c, 0x01, 0x0f},
    {0x90, 0x01, 0x0f},
    {0x94, 0x01, 0x0f},
    {0x98, 0x01, 0x0f},
    {0x9c, 0x01, 0x0f},
    {0xa0, 0x01, 0x0f},
    {0xa4, 0x01, 0x0f},
    {0xa8, 0x01, 0x0f},
    {0xac, 0x01, 0x0f},
    {0xb0, 0x01, 0x0f},
    {0xb4, 0x01, 0x0f},
    {0xb8, 0x01, 0x0f},
    {0xbc, 0x01, 0x0f},
    {0xc0, 0x01, 0x0f},
    {0xc4, 0x01, 0x0f},
    {0xc8, 0x01, 0x0f},
    {0xcc, 0x01, 0x0f},
    {0xd0, 0x01, 0x0f},
    {0xd4, 0x01, 0x0f},
    {0xd8, 0x01, 0x0f},
    {0xdc, 0x01, 0x0f},
    {0xe0, 0x01, 0x0f},
    {0xe4, 0x01, 0x0f},
    {0xe8, 0x01, 0x0f},
    {0xec, 0x01, 0x0f},
    {0xf0, 0x01, 0x0f},
    {0xf4, 0x01, 0x0f},
    {0xf8, 0x01, 0x0f},
    {0xfc, 0x01, 0x0f},
    //64x
    {0x80, 0x01, 0x1f},
    {0x84, 0x01, 0x1f},
    {0x88, 0x01, 0x1f},
    {0x8c, 0x01, 0x1f},
    {0x90, 0x01, 0x1f},
    {0x94, 0x01, 0x1f},
    {0x98, 0x01, 0x1f},
    {0x9c, 0x01, 0x1f},
    {0xa0, 0x01, 0x1f},
    {0xa4, 0x01, 0x1f},
    {0xa8, 0x01, 0x1f},
    {0xac, 0x01, 0x1f},
    {0xb0, 0x01, 0x1f},
    {0xb4, 0x01, 0x1f},
    {0xb8, 0x01, 0x1f},
    {0xbc, 0x01, 0x1f},
    {0xc0, 0x01, 0x1f},
    {0xc4, 0x01, 0x1f},
    {0xc8, 0x01, 0x1f},
    {0xcc, 0x01, 0x1f},
    {0xd0, 0x01, 0x1f},
    {0xd4, 0x01, 0x1f},
    {0xd8, 0x01, 0x1f},
    {0xdc, 0x01, 0x1f},
    {0xe0, 0x01, 0x1f},
    {0xe4, 0x01, 0x1f},
    {0xe8, 0x01, 0x1f},
    {0xec, 0x01, 0x1f},
    {0xf0, 0x01, 0x1f},
    {0xf4, 0x01, 0x1f},
    {0xf8, 0x01, 0x1f},
    {0xfc, 0x01, 0x1f},
};
#endif

 static const esp_cam_sensor_isp_info_t ov02c10_isp_info[] = {
     {
         .isp_v1_info = {
             .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
             .pclk = 81666700,
             .vts = 1164,
             .hts = 1140,
             .tline_ns = 14316,   /* Native 1288x728@60 tested ISP timing metadata. */
             .gain_def = 0x01,
             .exp_def = 0x46c,
             .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
         }
     },
     {
         .isp_v1_info = {
             .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
             .pclk = 81666700,
            // .pclk = 88333333,
             .vts = 1164,
             .hts = 2280,
             .tline_ns = 28636,
             .gain_def = 0x01,
             .exp_def = 0x46c,
             .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
         }
     },
     {
         .isp_v1_info = {
             .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            //  .pclk = 80000000,
             .pclk = 81666700,
             .vts = 1164,
             .hts = 2280,
             .tline_ns = 28636,   /* 1e9/(30*1164); matches active 0x380e/0x380f */
             .gain_def = 0x01,
             .exp_def = 0x46c,
             .bayer_type = ESP_CAM_SENSOR_BAYER_GBRG,
         }
     },
 };
 
/* [OV02C10-WEBRTC-DUAL-PROFILE-20260824]
 * Runtime-selectable WebRTC sensor profiles.  The selector is changed only
 * while the camera is stopped, before esp_video_init() detects the sensor.
 *   index 2 = production 1080p: 1920x1080 RAW10, 30fps, 2 CSI lanes
 *   index 0 = native high-speed: 1288x728 RAW10, 60fps, 2 CSI lanes
 * Default remains 1080p so every non-WebRTC caller keeps the proven behavior.
 */
#define OV02C10_WEBRTC_FORMAT_720P_INDEX   0U
#define OV02C10_WEBRTC_FORMAT_1080P_INDEX  2U
static volatile uint32_t s_ov02c10_webrtc_format_index = OV02C10_WEBRTC_FORMAT_1080P_INDEX;

/* Application-facing selector used by camera_csi_diag.c before esp_video_init().
 * Keep this tiny API here rather than duplicating the vendor register tables in
 * the application component. */
esp_err_t ov02c10_webrtc_select_profile(uint32_t profile)
{
    uint32_t idx;
    if (profile == 0U) {
        idx = OV02C10_WEBRTC_FORMAT_1080P_INDEX;
    } else if (profile == 1U) {
        idx = OV02C10_WEBRTC_FORMAT_720P_INDEX;
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    s_ov02c10_webrtc_format_index = idx;
    ESP_LOGI(TAG, "[WEBRTC-PROFILE] selected mode=%s format[%u]",
             profile == 1U ? "720p60-native" : "1080p", (unsigned)idx);
    return ESP_OK;
}

 static const esp_cam_sensor_format_t ov02c10_format_info[] = {
     {
         .name = "MIPI_2lane_24Minput_RAW10_1288x728_60fps",
         .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
         .port = ESP_CAM_SENSOR_MIPI_CSI,
         .xclk = 24000000,
         .width = 1288,
         .height = 728,
         .regs = ov02c10_input_24M_MIPI_2lane_raw10_1288x728_60fps,
         .regs_size = ARRAY_SIZE(ov02c10_input_24M_MIPI_2lane_raw10_1288x728_60fps),
         .fps = 60,
         .isp_info = &ov02c10_isp_info[0],
         .mipi_info = {
             .mipi_clk = OV02C10_MIPI_CSI_LINE_RATE_1920x1080_30FPS,
             .lane_num = 2,
             .line_sync_en = CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE ? true : false,
         },
         .reserved = NULL,
     },
     {
         .name = "MIPI_1lane_24Minput_RAW10_1920x1080_30fps",
         .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
         .port = ESP_CAM_SENSOR_MIPI_CSI,
         .xclk = 24000000,
         .width = 1920,
         .height = 1080,
         .regs = ov02c10_input_24M_MIPI_1lane_raw10_1920x1080_30fps,
         .regs_size = ARRAY_SIZE(ov02c10_input_24M_MIPI_1lane_raw10_1920x1080_30fps),
         .fps = 30,
         .isp_info = &ov02c10_isp_info[1],
         .mipi_info = {
             .mipi_clk = OV02C10_MIPI_CSI_LINE_RATE_1920x1080_30FPS,
             .lane_num = 1,
             .line_sync_en = CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE ? true : false,
         },
         .reserved = NULL,
     },
     {
         .name = "MIPI_2lane_24Minput_RAW10_1920x1080_30fps",
         .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
         .port = ESP_CAM_SENSOR_MIPI_CSI,
         .xclk = 24000000,
         .width = 1920,
         .height = 1080,
         .regs = ov02c10_input_24M_MIPI_2lane_raw10_1920x1080_30fps,
         .regs_size = ARRAY_SIZE(ov02c10_input_24M_MIPI_2lane_raw10_1920x1080_30fps),
         .fps = 30,
         .isp_info = &ov02c10_isp_info[2],
         .mipi_info = {
             .mipi_clk = OV02C10_MIPI_CSI_LINE_RATE_1920x1080_30FPS,
             .lane_num = 2,
             .line_sync_en = CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE ? true : false,
         },
         .reserved = NULL,
     },
 };
 
const esp_cam_sensor_format_t *ov02c10_webrtc_get_profile_format(uint32_t profile)
{
    if (profile == 0U) return &ov02c10_format_info[OV02C10_WEBRTC_FORMAT_1080P_INDEX];
    if (profile == 1U) return &ov02c10_format_info[OV02C10_WEBRTC_FORMAT_720P_INDEX];
    return NULL;
}

 static esp_err_t ov02c10_read(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t *read_buf)
 {
     return esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, read_buf);
 }
 
 static esp_err_t ov02c10_write(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t data)
 {
     return esp_sccb_transmit_reg_a16v8(sccb_handle, reg, data);
 }
 
 /* write a array of registers */
 static esp_err_t ov02c10_write_array(esp_sccb_io_handle_t sccb_handle, const ov02c10_reginfo_t *regarray)
 {
     int i = 0;
     esp_err_t ret = ESP_OK;
     while ((ret == ESP_OK) && regarray[i].reg != OV02C10_REG_END) {
         if (regarray[i].reg != OV02C10_REG_DELAY) {
             ret = ov02c10_write(sccb_handle, regarray[i].reg, regarray[i].val);
         } else {
             delay_ms(regarray[i].val);
         }
         i++;
     }
     ESP_LOGD(TAG, "count=%d", i);
     return ret;
 }
 
 static esp_err_t ov02c10_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
 {
     esp_err_t ret = ESP_OK;
     uint8_t reg_data = 0;
 
     ret = ov02c10_read(sccb_handle, reg, &reg_data);
     if (ret != ESP_OK) {
         return ret;
     }
     uint8_t mask = ((1 << length) - 1) << offset;
     value = (reg_data & ~mask) | ((value << offset) & mask);
     ret = ov02c10_write(sccb_handle, reg, value);
     return ret;
 }
 
 static esp_err_t ov02c10_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
 {
     ESP_LOGI(TAG,"test color = %d",enable);
     return ov02c10_set_reg_bits(dev->sccb_handle, 0x4503, 7, 1, enable ? 0x01 : 0x00);
 }
 
 static esp_err_t ov02c10_hw_reset(esp_cam_sensor_device_t *dev)
 {
     if (dev->reset_pin >= 0) {
         gpio_set_level(dev->reset_pin, 0);
         delay_ms(10);
         gpio_set_level(dev->reset_pin, 1);
         delay_ms(10);
     }
     return 0;
 }
 
 static esp_err_t ov02c10_soft_reset(esp_cam_sensor_device_t *dev)
 {
     esp_err_t ret = ov02c10_set_reg_bits(dev->sccb_handle, 0x0103, 0, 1, 0x01);
     delay_ms(5);
     return ret;
 }
 
 static esp_err_t ov02c10_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
 {
     uint8_t pid_h, pid_l;
     esp_err_t ret = ov02c10_read(dev->sccb_handle, OV02C10_REG_SENSOR_ID_H, &pid_h);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_h failed");
 
     ret = ov02c10_read(dev->sccb_handle, OV02C10_REG_SENSOR_ID_L, &pid_l);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_l failed");
 
     uint16_t pid = (pid_h << 8) | pid_l;
     if (pid) {
         id->pid = pid;
     }
     return ret;
 }
 
 static esp_err_t ov02c10_set_stream(esp_cam_sensor_device_t *dev, int enable)
 {
     esp_err_t ret;
     uint8_t val = OV02C10_MIPI_CTRL00_BUS_IDLE;
     if (enable) {
 #if CSI2_NONCONTINUOUS_CLOCK
         val |= OV02C10_MIPI_CTRL00_CLOCK_LANE_GATE | OV02C10_MIPI_CTRL00_LINE_SYNC_ENABLE;
 #endif
     } else {
         val |= OV02C10_MIPI_CTRL00_CLOCK_LANE_GATE | OV02C10_MIPI_CTRL00_CLOCK_LANE_DISABLE;
     }
 
     ret = ov02c10_write(dev->sccb_handle, 0x4800, CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE ? 0x64 : 0x00);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write pad out failed");
 
 #if CONFIG_CAMERA_OV02C10_ISP_AF_ENABLE
     ret = ov02c10_write(dev->sccb_handle, 0x3002, enable ? 0x01 : 0x00);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write pad out failed");
 
     ret = ov02c10_write(dev->sccb_handle, 0x3010, enable ? 0x01 : 0x00);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write pad out failed");
 
     ret = ov02c10_write(dev->sccb_handle, 0x300D, enable ? 0x01 : 0x00);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write pad out failed");
 #endif
 
     ret = ov02c10_write(dev->sccb_handle, 0x0100, enable ? 0x01 : 0x00);
     ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "write pad out failed");
 
     /* [OV02C10-TIMING-RB-R1-20260927]
      * Read-only, one-shot-at-STREAMON timing proof for native 1080p.
      * No sensor register is modified here. The goal is to compare the active
      * HTS/VTS/exposure registers with the driver ISP metadata.
      */
     if (enable && dev->cur_format && dev->cur_format->isp_info &&
             dev->cur_format->width == 1920 && dev->cur_format->height == 1080) {
         uint8_t hts_h = 0, hts_l = 0, vts_h = 0, vts_l = 0;
         uint8_t exp_h = 0, exp_l = 0;
         uint32_t read_mask = 0;
 
         if (ov02c10_read(dev->sccb_handle, 0x380c, &hts_h) == ESP_OK) read_mask |= (1U << 0);
         if (ov02c10_read(dev->sccb_handle, 0x380d, &hts_l) == ESP_OK) read_mask |= (1U << 1);
         if (ov02c10_read(dev->sccb_handle, 0x380e, &vts_h) == ESP_OK) read_mask |= (1U << 2);
         if (ov02c10_read(dev->sccb_handle, 0x380f, &vts_l) == ESP_OK) read_mask |= (1U << 3);
         if (ov02c10_read(dev->sccb_handle, 0x3501, &exp_h) == ESP_OK) read_mask |= (1U << 4);
         if (ov02c10_read(dev->sccb_handle, 0x3502, &exp_l) == ESP_OK) read_mask |= (1U << 5);
 
         const uint32_t actual_hts = ((uint32_t)hts_h << 8) | (uint32_t)hts_l;
         const uint32_t actual_vts = ((uint32_t)vts_h << 8) | (uint32_t)vts_l;
         const uint32_t actual_exp = ((uint32_t)exp_h << 8) | (uint32_t)exp_l;
 
         ESP_LOGW(TAG,
                  "[OV02C10-TIMING-RB-R1] mask=0x%02x profile=%ux%u@%u "
                  "meta{hts=%u vts=%u tline_ns=%u} "
                  "reg{380c=%02x 380d=%02x 380e=%02x 380f=%02x 3501=%02x 3502=%02x} "
                  "actual{hts=%u vts=%u exp=%u}",
                  (unsigned)read_mask,
                  (unsigned)dev->cur_format->width,
                  (unsigned)dev->cur_format->height,
                  (unsigned)dev->cur_format->fps,
                  (unsigned)dev->cur_format->isp_info->isp_v1_info.hts,
                  (unsigned)dev->cur_format->isp_info->isp_v1_info.vts,
                  (unsigned)dev->cur_format->isp_info->isp_v1_info.tline_ns,
                  (unsigned)hts_h, (unsigned)hts_l,
                  (unsigned)vts_h, (unsigned)vts_l,
                  (unsigned)exp_h, (unsigned)exp_l,
                  (unsigned)actual_hts,
                  (unsigned)actual_vts,
                  (unsigned)actual_exp);
     }
 
     dev->stream_status = enable;
 
     ESP_LOGD(TAG, "Stream=%d", enable);
     return ret;
 }
 
 static esp_err_t ov02c10_set_mirror(esp_cam_sensor_device_t *dev, int enable)
 {
     return ov02c10_set_reg_bits(dev->sccb_handle, 0x3821, 1, 1, enable ? 0x01 : 0x00);
 }
 
 static esp_err_t ov02c10_set_vflip(esp_cam_sensor_device_t *dev, int enable)
 {
     return ov02c10_set_reg_bits(dev->sccb_handle, 0x3820, 1, 1, enable ? 0x01 : 0x00);
 }
 
//  static esp_err_t ov02c10_set_AE_target(esp_cam_sensor_device_t *dev, int target)
//  {
//      esp_err_t ret = ESP_OK;
//      /* stable in high */
//      int fast_high, fast_low;
//      int AE_low = target * 23 / 25;  /* 0.92 */
//      int AE_high = target * 27 / 25; /* 1.08 */
 
//      fast_high = AE_high << 1;
//      if (fast_high > 255) {
//          fast_high = 255;
//      }
 
//      fast_low = AE_low >> 1;
 
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a0f, AE_high);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a10, AE_low);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a1b, AE_high);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a1e, AE_low);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a11, fast_high);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a1f, fast_low);
 
//      return ret;
//  }

 static esp_err_t ov02c10_set_exp_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct ov02c10_cam *cam_ov02c10 = (struct ov02c10_cam *)dev->priv;
    uint32_t value_buf = MAX(u32_val, s_ov02c10_exp_min);
    value_buf = MIN(value_buf, cam_ov02c10->ov02c10_para.exposure_max);

    if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG, "set exposure 0x%" PRIx32, value_buf);
    /* 4 least significant bits of expsoure are fractional part */
    // ret = ov02c10_write(dev->sccb_handle,
    //                    OV02C10_REG_SHUTTER_TIME_H,
    //                    OV02C10_FETCH_EXP_H(value_buf));

    if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,"OV02C10_FETCH_EXP_M(value_buf) = 0x%"PRIx32,OV02C10_FETCH_EXP_M(value_buf));
    ret = ov02c10_write(dev->sccb_handle,
                        OV02C10_REG_SHUTTER_TIME_M,
                        OV02C10_FETCH_EXP_M(value_buf));
    if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,"OV02C10_FETCH_EXP_M(value_buf) = 0x%"PRIx32,OV02C10_FETCH_EXP_L(value_buf));
    ret |= ov02c10_write(dev->sccb_handle,
                        OV02C10_REG_SHUTTER_TIME_L,
                        OV02C10_FETCH_EXP_L(value_buf));
    if (ret == ESP_OK) {
        cam_ov02c10->ov02c10_para.exposure_val = value_buf;
    }
    return ret;
}

 static esp_err_t ov02c10_set_total_gain_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret;
    struct ov02c10_cam *cam_ov02c10 = (struct ov02c10_cam *)dev->priv;

    /* [CAM-GAIN-720-MONOTONIC-FINAL-R63-20260929]
     * 720p60 only. R62 measured 5 x 224 points.
     * Refined continuity starts: {1536,1038,1367,1524,1645,1696,1524}.
     * Common block end: 2032. 1080 keeps the exact legacy/R59 path.
     */
    if (ov02c10_ae_diag_is_720p60(dev) &&
            u32_val < ARRAY_SIZE(ov02c10_gain_map)) {
        static const uint8_t analog_hi[7] = {1U,2U,3U,4U,5U,6U,8U};
        static const uint16_t digital_start[7] = {1536U,1038U,1367U,1524U,1645U,1696U,1524U};
        const uint32_t block = MIN(u32_val >> 5, 6U);
        const uint32_t pos = u32_val & 31U;
        const uint32_t dend = 2032U;
        const int32_t span = (int32_t)dend - (int32_t)digital_start[block];
        uint32_t dctrl = (uint32_t)((int32_t)digital_start[block] +
            (span * (int32_t)pos + (span >= 0 ? 15 : -15)) / 31);
        if (dctrl < 0x400U) { dctrl = 0x400U; }
        if (dctrl > 0x3fffU) { dctrl = 0x3fffU; }
        const uint32_t analog16 = ((uint32_t)analog_hi[block]) << 8;
        const uint32_t digital24 = dctrl << 6;
        ret = ov02c10_write(dev->sccb_handle, 0x3508, (uint8_t)(analog16 >> 8));
        ret |= ov02c10_write(dev->sccb_handle, 0x3509, (uint8_t)analog16);
        ret |= ov02c10_write(dev->sccb_handle, 0x350a, (uint8_t)(digital24 >> 16));
        ret |= ov02c10_write(dev->sccb_handle, 0x350b, (uint8_t)(digital24 >> 8));
        ret |= ov02c10_write(dev->sccb_handle, 0x350c, (uint8_t)digital24);
        if (ret == ESP_OK) { cam_ov02c10->ov02c10_para.gain_index = u32_val; }
        return ret;
    }

    ret = ov02c10_write(dev->sccb_handle,
                       OV02C10_REG_DIG_FINE_GAIN_H,
                       ov02c10_gain_map[u32_val].dgain_fine);
    ret |= ov02c10_write(dev->sccb_handle,
                         OV02C10_REG_DIG_COARSE_GAIN,
                         ov02c10_gain_map[u32_val].dgain_coarse);
    ret |= ov02c10_write(dev->sccb_handle,
                         OV02C10_REG_ANG_COARSE_GAIN,
                         ov02c10_gain_map[u32_val].analog_gain);
    if (ret == ESP_OK) { cam_ov02c10->ov02c10_para.gain_index = u32_val; }
    return ret;
}

static esp_err_t ov02c10_ae_diag_set_gain_val(esp_cam_sensor_device_t *dev, uint32_t gain_index)
{
    if (gain_index >= ARRAY_SIZE(ov02c10_gain_map)) return ESP_ERR_INVALID_ARG;
    return ov02c10_set_total_gain_val(dev, gain_index);
}

esp_err_t ov02c10_ae_diag_gain_force_ms(uint32_t ms, uint32_t gain_index)
{
    if (ms == 0U || ms > OV02C10_GAIN_FORCE_MAX_MS) return ESP_ERR_INVALID_ARG;
#if CONFIG_CAMERA_OV02C10_ANA_GAIN_PRIORITY
    if (gain_index > OV02C10_R17_720_GAIN_MAX_INDEX) return ESP_ERR_INVALID_ARG;
#else
    if (gain_index >= ARRAY_SIZE(ov02c10_gain_map)) return ESP_ERR_INVALID_ARG;
    if (s_limited_gain_index < ARRAY_SIZE(ov02c10_gain_map) &&
        gain_index > s_limited_gain_index) return ESP_ERR_INVALID_ARG;
#endif

    esp_cam_sensor_device_t *dev = s_ae_diag_dev;
    if (!ov02c10_ae_diag_is_720p60(dev)) return ESP_ERR_INVALID_STATE;

    const uint32_t now = ov02c10_ae_diag_now_ms();
    __atomic_store_n(&s_ae_diag.gain_force_value, gain_index, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.gain_force_until_ms, now + ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.gain_force_enabled, 1U, __ATOMIC_RELEASE);

    /* One immediate sensor write makes the slider deterministic even when the
     * IPA is already converged and would otherwise emit no new GAIN command. */
    esp_err_t ret = ov02c10_ae_diag_set_gain_val(dev, gain_index);
    if (ret != ESP_OK) {
        __atomic_store_n(&s_ae_diag.gain_force_enabled, 0U, __ATOMIC_RELEASE);
        __atomic_store_n(&s_ae_diag.gain_force_until_ms, 0U, __ATOMIC_RELEASE);
        return ret;
    }
    struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
    __atomic_store_n(&s_ae_diag.last_applied_gain, cam->ov02c10_para.gain_index, __ATOMIC_RELEASE);
    ESP_LOGW(TAG, "[AE-DIAG-R17] 720 GAIN FORCE start gain=%"PRIu32" ms=%"PRIu32,
             gain_index, ms);
    return ESP_OK;
}

void ov02c10_ae_diag_gain_force_release(void)
{
    __atomic_store_n(&s_ae_diag.gain_force_until_ms, 0U, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.gain_force_enabled, 0U, __ATOMIC_RELEASE);

    /* [CAM-AE-GOVERNOR-R24-20260923]
     * Do not replay last_req_gain here. R23 showed that a cached request can
     * be radically stale after manual/hold operation and can create an
     * immediate dark flash. Clearing the diagnostic force is enough; the ISP
     * governor resumes from the actual current sensor state on fresh frames. */
    ESP_LOGW(TAG, "[AE-DIAG-R24] GAIN FORCE release -> fresh AGC handoff (no stale write)");
}


/* [CAM-USER-GAIN-R23-20260923]
 * Normal 1080 exposes indices 0..191. R17 intentionally extends only the
 * native 720p60 mode to 0..223. Keep that distinction for manual control too. */
static uint32_t ov02c10_user_gain_max_for(const esp_cam_sensor_device_t *dev)
{
    return ov02c10_ae_diag_is_720p60(dev) ?
           OV02C10_R17_720_GAIN_MAX_INDEX :
           (OV02C10_R17_NORMAL_GAIN_COUNT - 1U);
}

static bool ov02c10_user_gain_active_for(const esp_cam_sensor_device_t *dev,
                                         uint32_t *gain_out)
{
    if (__atomic_load_n(&s_ae_diag.user_gain_enabled, __ATOMIC_ACQUIRE) == 0U) {
        return false;
    }

    uint32_t gain = __atomic_load_n(&s_ae_diag.user_gain_value, __ATOMIC_ACQUIRE);
    const uint32_t max_gain = ov02c10_user_gain_max_for(dev);
    if (gain > max_gain) gain = max_gain;
    if (gain_out) *gain_out = gain;
    return true;
}

esp_err_t ov02c10_user_gain_set(bool manual, uint32_t gain_index)
{
    if (gain_index > OV02C10_R17_720_GAIN_MAX_INDEX ||
        gain_index >= ARRAY_SIZE(ov02c10_gain_map)) {
        return ESP_ERR_INVALID_ARG;
    }

    __atomic_store_n(&s_ae_diag.user_gain_value, gain_index, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ae_diag.user_gain_enabled, manual ? 1U : 0U, __ATOMIC_RELEASE);

    esp_cam_sensor_device_t *dev = s_ae_diag_dev;
    if (!dev || !dev->priv) {
        return ESP_OK;
    }

    const uint32_t max_gain = ov02c10_user_gain_max_for(dev);
    if (manual) {
        const uint32_t apply_gain = gain_index > max_gain ? max_gain : gain_index;
        esp_err_t ret = ov02c10_ae_diag_set_gain_val(dev, apply_gain);
        if (ret == ESP_OK) {
            struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
            __atomic_store_n(&s_ae_diag.last_applied_gain,
                             cam->ov02c10_para.gain_index, __ATOMIC_RELEASE);
        }
        ESP_LOGI(TAG, "[CAM-USER-GAIN-R23] mode=MANUAL gain=%"PRIu32" max=%"PRIu32" ret=%s",
                 apply_gain, max_gain, esp_err_to_name(ret));
        return ret;
    }

    /* [CAM-AE-GOVERNOR-R24-20260923]
     * Legacy R23 API is no longer the production owner (web_smoke now talks
     * to esp_video_isp_pipeline_user_gain_set()). Keep this symbol safe for
     * any stale/internal caller: AUTO must never replay last_req_gain. */
    ESP_LOGI(TAG, "[CAM-USER-GAIN-R24] legacy mode=AUTO -> no stale gain write");
    return ESP_OK;
}

void ov02c10_user_gain_get(bool *manual, uint32_t *gain_index)
{
    if (manual) {
        *manual = __atomic_load_n(&s_ae_diag.user_gain_enabled,
                                  __ATOMIC_ACQUIRE) != 0U;
    }
    if (gain_index) {
        *gain_index = __atomic_load_n(&s_ae_diag.user_gain_value,
                                      __ATOMIC_ACQUIRE);
    }
}

static esp_err_t ov02c10_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = s_ov02c10_exp_min;
        qdesc->number.maximum = dev->cur_format->isp_info->isp_v1_info.vts - OV02C10_EXP_MAX_OFFSET; // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = 1;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.exp_def;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = EXPOSURE_OV02C10_TO_V4L2(s_ov02c10_exp_min, dev->cur_format);
        qdesc->number.maximum = EXPOSURE_OV02C10_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.vts - OV02C10_EXP_MAX_OFFSET), dev->cur_format); // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = MAX(EXPOSURE_OV02C10_TO_V4L2(0x01, dev->cur_format), 1);
        qdesc->default_value = EXPOSURE_OV02C10_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.exp_def), dev->cur_format);
        break;
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
#if CONFIG_CAMERA_OV02C10_ANA_GAIN_PRIORITY
        qdesc->enumeration.count = ov02c10_ae_diag_is_720p60(dev)
            ? MIN((size_t)OV02C10_R17_720_GAIN_COUNT, ARRAY_SIZE(ov02c10_total_gain_val_map))
            : MIN((size_t)OV02C10_R17_NORMAL_GAIN_COUNT, ARRAY_SIZE(ov02c10_total_gain_val_map));
#else
        qdesc->enumeration.count = s_limited_gain_index;
#endif
        qdesc->enumeration.elements = ov02c10_total_gain_val_map;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.gain_def; // gain index
        break;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
        break;
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
    default: {
        ESP_LOGD(TAG, "id=%"PRIx32" is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }
    return ret;
}

static esp_err_t ov02c10_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    s_ae_diag_dev = dev;
    esp_err_t ret = ESP_OK;
    struct ov02c10_cam *cam_ov02c10 = (struct ov02c10_cam *)dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        *(uint32_t *)arg = cam_ov02c10->ov02c10_para.exposure_val;
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        const uint32_t gain_index = cam_ov02c10->ov02c10_para.gain_index;
        *(uint32_t *)arg = gain_index;

        /*
         * [OV02C10-GAIN-REG-DIAG-AB1 2026-09-09]
         * Read-only physical gain-register snapshot.
         *
         * The existing ISP quality convergence diagnostic reads V4L2_CID_GAIN
         * at 0/1/3/5 s. That call lands here, so this gives a physical SCCB
         * readback at the same sampling points without changing AE/IPA/ISP.
         *
         * Current driver write mapping:
         *   0x3508 = analog coarse gain
         *   0x3509 = digital coarse gain
         *   0x350B = digital fine gain (high)
         * 0x350A/0x350C are also read as neighboring context only.
         */
        uint8_t reg_3508 = 0;
        uint8_t reg_3509 = 0;
        uint8_t reg_350a = 0;
        uint8_t reg_350b = 0;
        uint8_t reg_350c = 0;

        esp_err_t r3508 = ov02c10_read(dev->sccb_handle, 0x3508, &reg_3508);
        esp_err_t r3509 = ov02c10_read(dev->sccb_handle, 0x3509, &reg_3509);
        esp_err_t r350a = ov02c10_read(dev->sccb_handle, 0x350a, &reg_350a);
        esp_err_t r350b = ov02c10_read(dev->sccb_handle, 0x350b, &reg_350b);
        esp_err_t r350c = ov02c10_read(dev->sccb_handle, 0x350c, &reg_350c);

        bool map_ok = gain_index < ARRAY_SIZE(ov02c10_gain_map);
        uint8_t map_3508 = map_ok ? ov02c10_gain_map[gain_index].analog_gain : 0;
        uint8_t map_3509 = map_ok ? ov02c10_gain_map[gain_index].dgain_coarse : 0;
        uint8_t map_350b = map_ok ? ov02c10_gain_map[gain_index].dgain_fine : 0;

        if (HA_VERBOSE_DIAG_LOGS) ESP_LOGW(TAG,
                 "[OV02C10-GAIN-REG-DIAG] fmt=%s %ux%u idx=%"PRIu32
                 " read_ok=%u%u%u%u%u "
                 "rb{3508=%02x 3509=%02x 350A=%02x 350B=%02x 350C=%02x} "
                 "map{ok=%u 3508=%02x 3509=%02x 350B=%02x} match=%u",
                 (dev->cur_format && dev->cur_format->name) ? dev->cur_format->name : "?",
                 dev->cur_format ? (unsigned)dev->cur_format->width : 0U,
                 dev->cur_format ? (unsigned)dev->cur_format->height : 0U,
                 gain_index,
                 r3508 == ESP_OK ? 1U : 0U,
                 r3509 == ESP_OK ? 1U : 0U,
                 r350a == ESP_OK ? 1U : 0U,
                 r350b == ESP_OK ? 1U : 0U,
                 r350c == ESP_OK ? 1U : 0U,
                 reg_3508, reg_3509, reg_350a, reg_350b, reg_350c,
                 map_ok ? 1U : 0U, map_3508, map_3509, map_350b,
                 (map_ok &&
                  r3508 == ESP_OK && r3509 == ESP_OK && r350b == ESP_OK &&
                  reg_3508 == map_3508 &&
                  reg_3509 == map_3509 &&
                  reg_350b == map_350b) ? 1U : 0U);
        break;
    }
    default: {
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    }
    return ret;
}
 
 
//  static esp_err_t ov02c10_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
//  {
//      esp_err_t ret = ESP_OK;
//      switch (qdesc->id) {
//      case ESP_CAM_SENSOR_VFLIP:
//      case ESP_CAM_SENSOR_HMIRROR:
//          qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
//          qdesc->number.minimum = 0;
//          qdesc->number.maximum = 1;
//          qdesc->number.step = 1;
//          qdesc->default_value = 0;
//          break;
//      case ESP_CAM_SENSOR_EXPOSURE_VAL:
//          qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
//          qdesc->number.minimum = 2;
//          qdesc->number.maximum = 235;
//          qdesc->number.step = 1;
//          qdesc->default_value = 0;
//          break;
//      default: {
//          ESP_LOGI(TAG, "id=%"PRIx32" is not supported", qdesc->id);
//          ret = ESP_ERR_INVALID_ARG;
//          break;
//      }
//      }
//      return ret;
//  }
 
//  static esp_err_t ov02c10_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
//  {
//      return ESP_ERR_NOT_SUPPORTED;
//  }
 
//  static esp_err_t ov02c10_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
//  {
//      esp_err_t ret = ESP_OK;
 
//      switch (id) {
//      case ESP_CAM_SENSOR_VFLIP: {
//          int *value = (int *)arg;
 
//          ret = ov02c10_set_vflip(dev, *value);
//          break;
//      }
//      case ESP_CAM_SENSOR_HMIRROR: {
//          int *value = (int *)arg;
 
//          ret = ov02c10_set_mirror(dev, *value);
//          break;
//      }
//      case ESP_CAM_SENSOR_EXPOSURE_VAL: {
//          int *value = (int *)arg;
 
//          ret = ov02c10_set_AE_target(dev, *value);
//          break;
//      }
//      case ESP_CAM_SENSOR_GAIN: {
//         uint32_t u32_val = *(uint32_t *)arg;
//         ret = ov02c10_set_total_gain_val(dev, u32_val);
//         break;
//     }
//      default: {
//          ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
//          ret = ESP_ERR_INVALID_ARG;
//          break;
//      }
//      }
 
//      return ret;
//  }


static esp_err_t ov02c10_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    s_ae_diag_dev = dev;
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        const uint32_t req_exp = *(uint32_t *)arg;
        struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
        const uint32_t now_ms = ov02c10_ae_diag_now_ms();
        __atomic_store_n(&s_ae_diag.last_req_exp, req_exp, __ATOMIC_RELEASE);
        uint32_t hold_until = 0U;
        const bool hold = ov02c10_ae_diag_prepare_hold(cam, now_ms, &hold_until);
        const uint32_t apply_exp = hold ? __atomic_load_n(&s_ae_diag.hold_exp, __ATOMIC_ACQUIRE) : req_exp;
        if (hold && apply_exp != req_exp) __atomic_add_fetch(&s_ae_diag.suppressed_count, 1U, __ATOMIC_RELAXED);
        ret = ov02c10_set_exp_val(dev, apply_exp);
        if (ret == ESP_OK) __atomic_store_n(&s_ae_diag.last_applied_exp, cam->ov02c10_para.exposure_val, __ATOMIC_RELEASE);
        ov02c10_ae_diag_record(OV02C10_AE_PATH_EXP_VAL, now_ms, hold_until, hold);
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        const uint32_t exposure_us = *(uint32_t *)arg;
        const uint32_t req_exp = EXPOSURE_V4L2_TO_OV02C10(exposure_us, dev->cur_format);
        struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
        const uint32_t now_ms = ov02c10_ae_diag_now_ms();
        __atomic_store_n(&s_ae_diag.last_req_exp, req_exp, __ATOMIC_RELEASE);
        uint32_t hold_until = 0U;
        const bool hold = ov02c10_ae_diag_prepare_hold(cam, now_ms, &hold_until);
        const uint32_t apply_exp = hold ? __atomic_load_n(&s_ae_diag.hold_exp, __ATOMIC_ACQUIRE) : req_exp;
        if (hold && apply_exp != req_exp) __atomic_add_fetch(&s_ae_diag.suppressed_count, 1U, __ATOMIC_RELAXED);
        ret = ov02c10_set_exp_val(dev, apply_exp);
        if (ret == ESP_OK) __atomic_store_n(&s_ae_diag.last_applied_exp, cam->ov02c10_para.exposure_val, __ATOMIC_RELEASE);
        ov02c10_ae_diag_record(OV02C10_AE_PATH_EXP_US, now_ms, hold_until, hold);
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        const uint32_t req_gain = *(uint32_t *)arg;
        struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
        const uint32_t now_ms = ov02c10_ae_diag_now_ms();
        __atomic_store_n(&s_ae_diag.last_req_gain, req_gain, __ATOMIC_RELEASE);
        uint32_t hold_until = 0U;
        const bool hold = ov02c10_ae_diag_prepare_hold(cam, now_ms, &hold_until);
        uint32_t user_gain = req_gain;
        const bool user_forced = !hold &&
                                 ov02c10_user_gain_active_for(dev, &user_gain);
        uint32_t forced_gain = req_gain;
        const bool gain_forced = !hold && !user_forced &&
                                 ov02c10_ae_diag_gain_force_active_for(dev, now_ms, &forced_gain);
        const uint32_t apply_gain = hold ?
            __atomic_load_n(&s_ae_diag.hold_gain, __ATOMIC_ACQUIRE) :
            (user_forced ? user_gain : (gain_forced ? forced_gain : req_gain));
        if (hold && apply_gain != req_gain) {
            __atomic_add_fetch(&s_ae_diag.suppressed_count, 1U, __ATOMIC_RELAXED);
        } else if (gain_forced && apply_gain != req_gain) {
            __atomic_add_fetch(&s_ae_diag.gain_force_suppressed, 1U, __ATOMIC_RELAXED);
        }
        ret = ov02c10_ae_diag_set_gain_val(dev, apply_gain);
        if (ret == ESP_OK) __atomic_store_n(&s_ae_diag.last_applied_gain, cam->ov02c10_para.gain_index, __ATOMIC_RELEASE);
        ov02c10_ae_diag_record(OV02C10_AE_PATH_GAIN, now_ms, hold_until, hold);
        break;
    }
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        esp_cam_sensor_gh_exp_gain_t *value = (esp_cam_sensor_gh_exp_gain_t *)arg;
        struct ov02c10_cam *cam = (struct ov02c10_cam *)dev->priv;
        uint32_t req_exp = value->exposure_val;
        if (req_exp == 0U && value->exposure_us != 0U) req_exp = EXPOSURE_V4L2_TO_OV02C10(value->exposure_us, dev->cur_format);
        const uint32_t req_gain = value->gain_index;
        const uint32_t now_ms = ov02c10_ae_diag_now_ms();
        __atomic_store_n(&s_ae_diag.last_req_exp, req_exp, __ATOMIC_RELEASE);
        __atomic_store_n(&s_ae_diag.last_req_gain, req_gain, __ATOMIC_RELEASE);
        uint32_t hold_until = 0U;
        const bool hold = ov02c10_ae_diag_prepare_hold(cam, now_ms, &hold_until);
        const uint32_t apply_exp = hold ? __atomic_load_n(&s_ae_diag.hold_exp, __ATOMIC_ACQUIRE) : req_exp;
        uint32_t user_gain = req_gain;
        const bool user_forced = !hold &&
                                 ov02c10_user_gain_active_for(dev, &user_gain);
        uint32_t forced_gain = req_gain;
        const bool gain_forced = !hold && !user_forced &&
                                 ov02c10_ae_diag_gain_force_active_for(dev, now_ms, &forced_gain);
        const uint32_t apply_gain = hold ?
            __atomic_load_n(&s_ae_diag.hold_gain, __ATOMIC_ACQUIRE) :
            (user_forced ? user_gain : (gain_forced ? forced_gain : req_gain));
        if (hold && (apply_exp != req_exp || apply_gain != req_gain)) {
            __atomic_add_fetch(&s_ae_diag.suppressed_count, 1U, __ATOMIC_RELAXED);
        } else if (gain_forced && apply_gain != req_gain) {
            __atomic_add_fetch(&s_ae_diag.gain_force_suppressed, 1U, __ATOMIC_RELAXED);
        }
        ret = ov02c10_write(dev->sccb_handle, OV02C10_REG_GROUP_HOLD, OV02C10_GROUP_HOLD_START);
        ret |= ov02c10_set_exp_val(dev, apply_exp);
        ret |= ov02c10_ae_diag_set_gain_val(dev, apply_gain);
        ret |= ov02c10_write(dev->sccb_handle, OV02C10_REG_GROUP_HOLD_DELAY, OV02C10_GROUP_HOLD_DELAY_FRAMES);
        ret |= ov02c10_write(dev->sccb_handle, OV02C10_REG_GROUP_HOLD, OV02C10_GROUP_HOLD_END);
        if (ret == ESP_OK) {
            __atomic_store_n(&s_ae_diag.last_applied_exp, cam->ov02c10_para.exposure_val, __ATOMIC_RELEASE);
            __atomic_store_n(&s_ae_diag.last_applied_gain, cam->ov02c10_para.gain_index, __ATOMIC_RELEASE);
        }
        ov02c10_ae_diag_record(OV02C10_AE_PATH_GROUP, now_ms, hold_until, hold);
        break;
    }
    case ESP_CAM_SENSOR_VFLIP: {
        int *value = (int *)arg;
        ret = ov02c10_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        int *value = (int *)arg;
        ret = ov02c10_set_mirror(dev, *value);
        break;
    }
    default: {
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }

    return ret;
}
 
 static esp_err_t ov02c10_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
 {
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
 
     formats->count = ARRAY_SIZE(ov02c10_format_info);
     formats->format_array = &ov02c10_format_info[0];
     return ESP_OK;
 }
 
 static esp_err_t ov02c10_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
 {
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);
 
     sensor_cap->fmt_raw = 1;
     return ESP_OK;
 }
 
 static int ov02c10_get_sysclk(esp_cam_sensor_device_t *dev)
 {
     /* calculate sysclk */
     int xvclk = dev->cur_format->xclk / 10000;
     int sysclk = 0;
     uint8_t temp1, temp2;
     int pre_div02x, div_cnt7b, sdiv0, pll_rdiv, bit_div2x, sclk_div, VCO;
     const int pre_div02x_map[] = {2, 2, 4, 6, 8, 3, 12, 5, 16, 2, 2, 2, 2, 2, 2, 2};
     const int sdiv0_map[] = {16, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
     const int pll_rdiv_map[] = {1, 2};
     const int bit_div2x_map[] = {2, 2, 2, 2, 2, 2, 2, 2, 4, 2, 5, 2, 2, 2, 2, 2};
     const int sclk_div_map[] = {1, 2, 4, 1};
 
     ov02c10_read(dev->sccb_handle, 0x3037, &temp1);
     temp2 = temp1 & 0x0f;
     pre_div02x = pre_div02x_map[temp2];
     temp2 = (temp1 >> 4) & 0x01;
     pll_rdiv = pll_rdiv_map[temp2];
     ov02c10_read(dev->sccb_handle, 0x3036, &temp1);
 
     div_cnt7b = temp1;
 
     VCO = xvclk * 2 / pre_div02x * div_cnt7b;
     ov02c10_read(dev->sccb_handle, 0x3035, &temp1);
     temp2 = temp1 >> 4;
     sdiv0 = sdiv0_map[temp2];
     ov02c10_read(dev->sccb_handle, 0x3034, &temp1);
     temp2 = temp1 & 0x0f;
     bit_div2x = bit_div2x_map[temp2];
     ov02c10_read(dev->sccb_handle, 0x3106, &temp1);
     temp2 = (temp1 >> 2) & 0x03;
     sclk_div = sclk_div_map[temp2];
     sysclk = VCO * 2 / sdiv0 / pll_rdiv / bit_div2x / sclk_div;
     return sysclk;
 }
 
 static int ov02c10_get_hts(esp_cam_sensor_device_t *dev)
 {
     /* read HTS from register settings */
     int hts = 0;
     uint8_t temp1, temp2;
 
     ov02c10_read(dev->sccb_handle, 0x380c, &temp1);
     ov02c10_read(dev->sccb_handle, 0x380d, &temp2);
     hts = (temp1 << 8) + temp2;
     ESP_LOGI(TAG,"hts = 0x%x",hts);
     return hts;
 }
 
 static int ov02c10_get_vts(esp_cam_sensor_device_t *dev)
 {
     /* read VTS from register settings */
     int vts = 0;
     uint8_t temp1, temp2;
 
     /* total vertical size[15:8] high byte */
     ov02c10_read(dev->sccb_handle, 0x380e, &temp1);
     ov02c10_read(dev->sccb_handle, 0x380f, &temp2);
 
     vts = (temp1 << 8) + temp2;
     ESP_LOGI(TAG,"vts = 0x%x",vts);
     return vts;
 }
 
//  static int ov02c10_get_light_freq(esp_cam_sensor_device_t *dev)
//  {
//      /* get banding filter value */
//      uint8_t temp, temp1;
//      int light_freq = 0;
 
//      ov02c10_read(dev->sccb_handle, 0x3c01, &temp);
 
//      if (temp & 0x80) {
//          /* manual */
//          ov02c10_read(dev->sccb_handle, 0x3c00, &temp1);
//          if (temp1 & 0x04) {
//              /* 50Hz */
//              light_freq = 50;
//          } else {
//              /* 60Hz */
//              light_freq = 60;
//          }
//      } else {
//          /* auto */
//          ov02c10_read(dev->sccb_handle, 0x3c0c, &temp1);
//          if (temp1 & 0x01) {
//              /* 50Hz */
//              light_freq = 50;
//          } else {
//              light_freq = 60;
//          }
//      }
//      return light_freq;
//  }
 
//  static esp_err_t ov02c10_set_bandingfilter(esp_cam_sensor_device_t *dev)
//  {
//      esp_err_t ret = ESP_OK;
//      int prev_sysclk, prev_VTS, prev_HTS;
//      int band_step60, max_band60, band_step50, max_band50;
 
//      /* read preview PCLK */
//      prev_sysclk = ov02c10_get_sysclk(dev);
//      /* read preview HTS */
//      prev_HTS = ov02c10_get_hts(dev);
 
//      /* read preview VTS */
//      prev_VTS = ov02c10_get_vts(dev);
 
//      /* calculate banding filter */
//      /* 60Hz */
//      band_step60 = prev_sysclk * 100 / prev_HTS * 100 / 120;
//      ret = ov02c10_write(dev->sccb_handle, 0x3a0a, (uint8_t)(band_step60 >> 8));
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a0b, (uint8_t)(band_step60 & 0xff));
 
//      max_band60 = (int)((prev_VTS - 4) / band_step60);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a0d, (uint8_t)max_band60);
 
//      /* 50Hz */
//      band_step50 = prev_sysclk * 100 / prev_HTS;
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a08, (uint8_t)(band_step50 >> 8));
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a09, (uint8_t)(band_step50 & 0xff));
 
//      max_band50 = (int)((prev_VTS - 4) / band_step50);
//      ret |= ov02c10_write(dev->sccb_handle, 0x3a0e, (uint8_t)max_band50);
//      return ret;
//  }
 
 static esp_err_t ov02c10_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
 {
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    struct ov02c10_cam *cam_ov02c10 = (struct ov02c10_cam *)dev->priv;
    esp_err_t ret = ESP_OK;
    /* Depending on the interface type, an available configuration is automatically loaded.
    You can set the output format of the sensor without using query_format().*/
    if (format == NULL) {
        uint32_t idx = s_ov02c10_webrtc_format_index;
        if (idx >= ARRAY_SIZE(ov02c10_format_info)) idx = OV02C10_WEBRTC_FORMAT_1080P_INDEX;
        format = &ov02c10_format_info[idx];
        ESP_LOGI(TAG, "[WEBRTC-PROFILE] default format[%u]: %s", (unsigned)idx, format->name);
    }

    ret = ov02c10_write_array(dev->sccb_handle, (ov02c10_reginfo_t *)format->regs);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;
    // init para
    cam_ov02c10->ov02c10_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_ov02c10->ov02c10_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    cam_ov02c10->ov02c10_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - OV02C10_EXP_MAX_OFFSET;

    /* [CAM-USER-GAIN-R23-20260923]
     * S_FMT rewrites the sensor register table, including default gain.
     * Re-apply RAM manual gain immediately after the new format is active. */
    uint32_t user_gain = 0U;
    if (ov02c10_user_gain_active_for(dev, &user_gain)) {
        esp_err_t gr = ov02c10_ae_diag_set_gain_val(dev, user_gain);
        if (gr != ESP_OK) {
            ESP_LOGE(TAG, "[CAM-USER-GAIN-R23] set_format reapply gain=%"PRIu32" failed: %s",
                     user_gain, esp_err_to_name(gr));
            return gr;
        }
        __atomic_store_n(&s_ae_diag.last_applied_gain,
                         cam_ov02c10->ov02c10_para.gain_index, __ATOMIC_RELEASE);
        ESP_LOGI(TAG, "[CAM-USER-GAIN-R23] set_format reapply gain=%"PRIu32,
                 user_gain);
    }

    return ret;
 }
 
 static esp_err_t ov02c10_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
 {
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
 
     esp_err_t ret = ESP_FAIL;
 
     if (dev->cur_format != NULL) {
         memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
         ret = ESP_OK;
     }
     return ret;
 }
 
 static esp_err_t ov02c10_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
 {
     ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
 
     esp_err_t ret = ESP_FAIL;
     uint8_t regval;
     esp_cam_sensor_reg_val_t *sensor_reg;
     OV02C10_IO_MUX_LOCK(mux);
     switch (cmd) {
     case ESP_CAM_SENSOR_IOC_HW_RESET:
         ret = ov02c10_hw_reset(dev);
         break;
     case ESP_CAM_SENSOR_IOC_SW_RESET:
         ret = ov02c10_soft_reset(dev);
         break;
     case ESP_CAM_SENSOR_IOC_S_REG:
         sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
         ret = ov02c10_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
         break;
     case ESP_CAM_SENSOR_IOC_S_STREAM:
         // ret = ov02c10_set_test_pattern(dev, *(int *)arg);
         ret = ov02c10_set_stream(dev, *(int *)arg);
         
         break;
     case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
         ret = ov02c10_set_test_pattern(dev, *(int *)arg);
         break;
     case ESP_CAM_SENSOR_IOC_G_REG:
         sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
         ret = ov02c10_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
         if (ret == ESP_OK) {
             sensor_reg->value = regval;
         }
         break;
     case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
         ret = ov02c10_get_sensor_id(dev, arg);
         break;
     default:
         ret = ESP_ERR_INVALID_ARG;
         break;
     }
     OV02C10_IO_MUX_UNLOCK(mux);
     return ret;
 }
 
 static esp_err_t ov02c10_power_on(esp_cam_sensor_device_t *dev)
 {
     esp_err_t ret = ESP_OK;
 
     if (dev->xclk_pin >= 0) {
         OV02C10_ENABLE_OUT_CLOCK(dev->xclk_pin, dev->xclk_freq_hz);
     }
 
     if (dev->pwdn_pin >= 0) {
         gpio_config_t conf = { 0 };
         conf.pin_bit_mask = 1LL << dev->pwdn_pin;
         conf.mode = GPIO_MODE_OUTPUT;
         ret = gpio_config(&conf);
         ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "pwdn pin config failed");
 
         // carefully, logic is inverted compared to reset pin
         gpio_set_level(dev->pwdn_pin, 1);
         delay_ms(10);
         gpio_set_level(dev->pwdn_pin, 0);
         delay_ms(10);
     }
 
     if (dev->reset_pin >= 0) {
         gpio_config_t conf = { 0 };
         conf.pin_bit_mask = 1LL << dev->reset_pin;
         conf.mode = GPIO_MODE_OUTPUT;
         ret = gpio_config(&conf);
         ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "reset pin config failed");
 
         gpio_set_level(dev->reset_pin, 0);
         delay_ms(10);
         gpio_set_level(dev->reset_pin, 1);
         delay_ms(10);
     }
 
     return ret;
 }
 
 static esp_err_t ov02c10_power_off(esp_cam_sensor_device_t *dev)
 {
     esp_err_t ret = ESP_OK;
 
     if (dev->xclk_pin >= 0) {
         OV02C10_DISABLE_OUT_CLOCK(dev->xclk_pin);
     }
 
     if (dev->pwdn_pin >= 0) {
         gpio_set_level(dev->pwdn_pin, 0);
         delay_ms(10);
         gpio_set_level(dev->pwdn_pin, 1);
         delay_ms(10);
     }
 
     if (dev->reset_pin >= 0) {
         gpio_set_level(dev->reset_pin, 1);
         delay_ms(10);
         gpio_set_level(dev->reset_pin, 0);
         delay_ms(10);
     }
 
     return ret;
 }
 
 static esp_err_t ov02c10_delete(esp_cam_sensor_device_t *dev)
 {
     ESP_LOGD(TAG, "del ov02c10 (%p)", dev);
     if (s_ae_diag_dev == dev) s_ae_diag_dev = NULL;
     if (dev) {
         free(dev);
         dev = NULL;
     }
 
     return ESP_OK;
 }
 
 static const esp_cam_sensor_ops_t ov02c10_ops = {
     .query_para_desc = ov02c10_query_para_desc,
     .get_para_value = ov02c10_get_para_value,
     .set_para_value = ov02c10_set_para_value,
     .query_support_formats = ov02c10_query_support_formats,
     .query_support_capability = ov02c10_query_support_capability,
     .set_format = ov02c10_set_format,
     .get_format = ov02c10_get_format,
     .priv_ioctl = ov02c10_priv_ioctl,
     .del = ov02c10_delete
 };
 
 // We need manage these devices, and maybe need to add it into the private member of esp_device
 esp_cam_sensor_device_t *ov02c10_detect(esp_cam_sensor_config_t *config)
 {
    esp_cam_sensor_device_t *dev = NULL;
    struct ov02c10_cam *cam_ov02c10;
    s_limited_gain_index = ARRAY_SIZE(ov02c10_total_gain_val_map);
    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    cam_ov02c10 = heap_caps_calloc(1, sizeof(struct ov02c10_cam), MALLOC_CAP_DEFAULT);
    if (!cam_ov02c10) {
        ESP_LOGE(TAG, "failed to calloc cam");
        free(dev);
        return NULL;
    }

    dev->name = (char *)OV02C10_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &ov02c10_ops;
    dev->priv = cam_ov02c10;
    for (size_t i = 0; i < ARRAY_SIZE(ov02c10_total_gain_val_map); i++) {
        if (ov02c10_total_gain_val_map[i] > s_limited_gain) {
            s_limited_gain_index = i - 1;
            break;
        }
    }
    if (config->sensor_port != ESP_CAM_SENSOR_DVP) {
        uint32_t idx = s_ov02c10_webrtc_format_index;
        if (idx >= ARRAY_SIZE(ov02c10_format_info)) idx = OV02C10_WEBRTC_FORMAT_1080P_INDEX;
        dev->cur_format = &ov02c10_format_info[idx];
        ESP_LOGI(TAG, "[WEBRTC-PROFILE] forcing OV02C10 mode[%u]: %s %ux%u %ufps lanes=%u",
                 (unsigned)idx, dev->cur_format->name,
                 (unsigned)dev->cur_format->width, (unsigned)dev->cur_format->height,
                 (unsigned)dev->cur_format->fps,
                 (unsigned)dev->cur_format->mipi_info.lane_num);
    } 

    // Configure sensor power, clock, and SCCB port
    if (ov02c10_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    esp_err_t id_ret = ov02c10_get_sensor_id(dev, &dev->id);
    if (id_ret != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    }

    /*
     * [OV02C10-PID0-RECOVERY 2026-09-01]
     *
     * Observed failure signature:
     *   SCCB ID reads return without an API error, but PID remains 0x0000.
     *
     * Recovery is deliberately limited to PID==0 only:
     *   - Do NOT reset an unknown non-zero device at the same SCCB address.
     *   - Do NOT retry esp_video_init().
     *   - Do NOT deinit/reinit CSI or ISP.
     *
     * OV02C10 mode tables already use register 0x0103 = 0x01 as a sensor
     * software reset.  Here we issue that SAME direct write only when the
     * initial ID result is exactly zero, wait 10 ms, then re-read the ID once.
     */
        if (dev->id.pid == 0) {
        ESP_LOGW(TAG,
                 "[OV02C10-PID0-RECOVERY] initial PID=0x0000; "
                 "write SW reset 0x0103=0x01 and retry ID once");

        esp_err_t reset_ret = ov02c10_write(dev->sccb_handle, 0x0103, 0x01);
        if (reset_ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "[OV02C10-PID0-RECOVERY] SW reset write failed: %s",
                     esp_err_to_name(reset_ret));
            goto err_free_handler;
        }

        delay_ms(10);

        /* calloc() initially zeroed the ID; clear it explicitly before retry. */
        dev->id.pid = 0;
        id_ret = ov02c10_get_sensor_id(dev, &dev->id);

        if (id_ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "[OV02C10-PID0-RECOVERY] ID re-read failed after SW reset: %s",
                     esp_err_to_name(id_ret));
            goto err_free_handler;
        }

        if (dev->id.pid == OV02C10_PID) {
            ESP_LOGW(TAG,
                     "[OV02C10-PID0-RECOVERY] recovered PID=0x%04x after SW reset",
                     dev->id.pid);
        } else {
            ESP_LOGE(TAG,
                     "[OV02C10-PID0-RECOVERY] still invalid after SW reset, PID=0x%x",
                     dev->id.pid);
            goto err_free_handler;
        }
    } else if (dev->id.pid != OV02C10_PID) {
        ESP_LOGE(TAG, "Camera sensor is not OV02C10, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }

    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    return dev;

err_free_handler:
    ov02c10_power_off(dev);
    free(dev->priv);
    free(dev);

    return NULL;
 }
 
 #if CONFIG_CAMERA_OV02C10_AUTO_DETECT_MIPI_INTERFACE_SENSOR
 ESP_CAM_SENSOR_DETECT_FN(ov02c10_detect, ESP_CAM_SENSOR_MIPI_CSI, OV02C10_SCCB_ADDR)
 {
     ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
     return ov02c10_detect(config);
 }
 #endif
 
