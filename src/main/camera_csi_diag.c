#include "camera_csi_diag.h"

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "bsp/esp32_p4_function_ev_board.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#if CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#endif
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_async_memcpy.h"
#include "esp_cache.h"
#include "driver/ppa.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_cam_sensor.h"
#include "ov02c10_webrtc_profile.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_hw.h"
#include "linux/videodev2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "soc/hp_sys_clkrst_reg.h"
#include "hal/axi_icm_ll.h"
#include "lvgl.h"

#include "lcd_ui.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif

#if !CONFIG_CAMERA_OV02C10
#error "camera_csi_diag requires CONFIG_CAMERA_OV02C10=y"
#endif

#if !CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
#error "camera_csi_diag requires CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE=y"
#endif

static const char *TAG = "CAM_CSI";

/* [ISP-LIFECYCLE-C12A 2026-08-27]
 * Profile-split A/B.  1080p keeps the C10A EARLY full-controller restart that
 * is stable on hardware.  Native 720p60 deliberately skips that restart and
 * instead uses the previously-working post-S_FMT four-field software resync.
 * Keep these A/B-only hooks private to this diagnostic module. */
extern esp_err_t esp_video_isp_pipeline_full_restart_for_ab(void);
extern esp_err_t esp_video_isp_pipeline_720_resync_for_c12a(void);
extern esp_err_t esp_video_isp_pipeline_fresh_ipa_for_c12c_1080(void);


/* [COLOR-DIAG-20260823]
 * Read-only image-pipeline diagnostics for OV02C10 RAW10 -> ESP32-P4 ISP.
 * This intentionally does NOT modify exposure, gain, AWB, Bayer order, or
 * frame pixels, so the verified 1080p GOP1 ~28 fps path stays untouched.
 */
static const char *camera_color_diag_bayer_name(int bayer)
{
    switch (bayer) {
    case ESP_CAM_SENSOR_BAYER_RGGB: return "RGGB";
    case ESP_CAM_SENSOR_BAYER_GRBG: return "GRBG";
    case ESP_CAM_SENSOR_BAYER_GBRG: return "GBRG";
    case ESP_CAM_SENSOR_BAYER_BGGR: return "BGGR";
    default: return "UNKNOWN";
    }
}

static void camera_color_diag_log_build_config(void)
{
#if CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER
    const int pipeline_controller = 1;
#else
    const int pipeline_controller = 0;
#endif
#if CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE
    const int isp_video_device = 1;
#else
    const int isp_video_device = 0;
#endif
    ESP_LOGW(TAG,
             "[COLOR-DIAG] build ISP_PIPELINE_CONTROLLER=%d ISP_VIDEO_DEVICE=%d "
             "(controller=0 means no automatic IPA AE/AWB/CCM updates)",
             pipeline_controller, isp_video_device);
}

static void camera_color_diag_log_sensor_isp_info(const esp_cam_sensor_format_t *fmt)
{
    if (!fmt || !fmt->isp_info) {
        ESP_LOGW(TAG, "[COLOR-DIAG] sensor isp_info=NULL; Bayer/ISP metadata unavailable");
        return;
    }
    const esp_cam_sensor_isp_info_t *isp = fmt->isp_info;
    ESP_LOGW(TAG,
             "[COLOR-DIAG] sensor ISP meta bayer=%s(%d) pclk=%u vts=%u hts=%u "
             "tline_ns=%u gain_def=%u exp_def=%u",
             camera_color_diag_bayer_name((int)isp->isp_v1_info.bayer_type),
             (int)isp->isp_v1_info.bayer_type,
             (unsigned)isp->isp_v1_info.pclk,
             (unsigned)isp->isp_v1_info.vts,
             (unsigned)isp->isp_v1_info.hts,
             (unsigned)isp->isp_v1_info.tline_ns,
             (unsigned)isp->isp_v1_info.gain_def,
             (unsigned)isp->isp_v1_info.exp_def);
}


/* [720-COLOR-MATCH-OUYY-EVYY-DIAG-R1-20260910]
 * READ-ONLY post-ISP frame diagnostic using the ACTUAL ESP32-P4 camera layout.
 *
 * IMPORTANT CORRECTION:
 * V4L2_PIX_FMT_YUV420 on this ESP32-P4 path is not planar I420 in memory.
 * The project already documents/uses ESP_H264_RAW_FMT_O_UYY_E_VYY:
 *   - each row is width * 3 / 2 bytes
 *   - every 2 horizontal pixels are {chroma, Y, Y}
 *   - odd/even rows alternate U and V (project convention: odd-U/even-V)
 *
 * The previous YU12 diagnostic incorrectly interpreted the buffer as
 * planar Y + U + V, so its Y/U/V values must NOT be used for calibration.
 *
 * For 1080p this diagnostic samples the centered 1288x728 ROI.
 * For 720p60 it samples the whole 1288x728 frame.
 * It samples row pairs so both chroma phases are captured together.
 *
 * It never modifies frame pixels or any ISP/AE/AWB/WB/CCM/NVS/H264/WebRTC
 * setting. No dynamic allocation is used.
 */
typedef struct {
    uint32_t y_count;
    uint32_t u_count;
    uint32_t v_count;
    uint64_t y_sum;
    uint64_t u_sum;
    uint64_t v_sum;
    uint64_t u_abs_sum;
    uint64_t v_abs_sum;
    uint32_t y_hist[256];
} camera_ouyy_diag_stats_t;

/* [CAM-AUTO-COLOR-CAL-R1-20260910]
 * The browser-side one-click calibrator asks for a fresh measurement through
 * the existing /api/smoke/status URI. The next frame of the requested profile
 * fulfills exactly one request. This adds no continuous per-frame work. */
static portMUX_TYPE s_color_measure_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_color_measure_seq;
static uint32_t s_color_measure_pending;
static camera_h264_profile_t s_color_measure_pending_profile = CAMERA_H264_PROFILE_1080P;
static camera_color_measurement_t s_color_measure_latest;

uint32_t camera_color_measure_request(camera_h264_profile_t profile)
{
    if (profile != CAMERA_H264_PROFILE_1080P &&
        profile != CAMERA_H264_PROFILE_720P_NATIVE) {
        return 0;
    }

    taskENTER_CRITICAL(&s_color_measure_mux);
    s_color_measure_seq++;
    if (s_color_measure_seq == 0) s_color_measure_seq = 1;
    s_color_measure_pending = s_color_measure_seq;
    s_color_measure_pending_profile = profile;
    const uint32_t id = s_color_measure_pending;
    taskEXIT_CRITICAL(&s_color_measure_mux);
    return id;
}

void camera_color_measure_get(camera_color_measurement_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_color_measure_mux);
    *out = s_color_measure_latest;
    taskEXIT_CRITICAL(&s_color_measure_mux);
}

static uint32_t camera_color_measure_pending_for(camera_h264_profile_t profile)
{
    uint32_t id = 0;
    taskENTER_CRITICAL(&s_color_measure_mux);
    if (s_color_measure_pending != 0 &&
        s_color_measure_pending_profile == profile) {
        id = s_color_measure_pending;
    }
    taskEXIT_CRITICAL(&s_color_measure_mux);
    return id;
}

static void camera_color_measure_publish(uint32_t request_id,
                                         const camera_color_measurement_t *m)
{
    if (!request_id || !m || !m->valid) return;

    bool published = false;
    taskENTER_CRITICAL(&s_color_measure_mux);
    if (s_color_measure_pending == request_id &&
        s_color_measure_pending_profile == m->profile) {
        s_color_measure_latest = *m;
        s_color_measure_latest.request_id = request_id;
        s_color_measure_pending = 0;
        published = true;
    }
    taskEXIT_CRITICAL(&s_color_measure_mux);

    if (published) {
        if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,
                 "[CAM-AUTO-CAL-MEASURE] req=%u profile=%s "
                 "Y=%.3f p10=%u p50=%u p90=%u U=%.3f V=%.3f Uabs=%.3f Vabs=%.3f",
                 (unsigned)request_id,
                 camera_h264_stream_profile_name(m->profile),
                 (double)m->y_mean,
                 (unsigned)m->y_p10, (unsigned)m->y_p50, (unsigned)m->y_p90,
                 (double)m->u_mean, (double)m->v_mean,
                 (double)m->u_abs, (double)m->v_abs);
    }
}

static uint8_t camera_ouyy_diag_percentile(const uint32_t hist[256],
                                           uint32_t count,
                                           uint32_t pct)
{
    if (!hist || count == 0) return 0;
    uint64_t target = ((uint64_t)count * pct + 99U) / 100U;
    if (target == 0) target = 1;

    uint64_t acc = 0;
    for (uint32_t i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc >= target) return (uint8_t)i;
    }
    return 255;
}

static inline void camera_ouyy_diag_add_y(camera_ouyy_diag_stats_t *st,
                                          uint8_t yy)
{
    st->y_sum += yy;
    st->y_hist[yy]++;
    st->y_count++;
}

static bool camera_ouyy_diag_measure(const uint8_t *frame,
                                     size_t frame_bytes,
                                     uint32_t width,
                                     uint32_t height,
                                     camera_h264_profile_t profile_id,
                                     uint32_t elapsed_ms,
                                     camera_color_measurement_t *out,
                                     const char **roi_name_out,
                                     uint32_t *x0_out,
                                     uint32_t *y0_out,
                                     uint32_t *rw_out,
                                     uint32_t *rh_out)
{
    if (!frame || !out || width < 2 || height < 2 ||
        (width & 1U) || (height & 1U)) {
        return false;
    }

    const size_t row_bytes = ((size_t)width * 3U) / 2U;
    const size_t need = row_bytes * (size_t)height;
    if (frame_bytes < need) return false;

    uint32_t x0 = 0, y0 = 0, rw = width, rh = height;
    const char *roi_name = "full";

    if (width == 1920 && height == 1080) {
        x0 = 316;
        y0 = 176;
        rw = 1288;
        rh = 728;
        roi_name = "center1288x728";
    } else if (width == 1288 && height == 728) {
        roi_name = "full1288x728";
    }

    x0 &= ~1U;
    y0 &= ~1U;
    rw &= ~1U;
    rh &= ~1U;

    if (x0 + rw > width || y0 + rh > height || rw < 2 || rh < 2) {
        return false;
    }

    camera_ouyy_diag_stats_t st;
    memset(&st, 0, sizeof(st));

    const uint32_t x_step = 16U;
    const uint32_t y_pair_step = 16U;

    for (uint32_t y = y0; y + 1U < y0 + rh; y += y_pair_step) {
        const uint8_t *row_even = frame + (size_t)y * row_bytes;
        const uint8_t *row_odd  = frame + (size_t)(y + 1U) * row_bytes;

        for (uint32_t x = x0; x + 1U < x0 + rw; x += x_step) {
            const size_t off = ((size_t)x / 2U) * 3U;

            const uint8_t v = row_even[off + 0U];
            const uint8_t y_e0 = row_even[off + 1U];
            const uint8_t y_e1 = row_even[off + 2U];

            const uint8_t u = row_odd[off + 0U];
            const uint8_t y_o0 = row_odd[off + 1U];
            const uint8_t y_o1 = row_odd[off + 2U];

            st.v_sum += v;
            st.v_count++;
            {
                int d = (int)v - 128;
                st.v_abs_sum += (uint64_t)(d < 0 ? -d : d);
            }

            st.u_sum += u;
            st.u_count++;
            {
                int d = (int)u - 128;
                st.u_abs_sum += (uint64_t)(d < 0 ? -d : d);
            }

            camera_ouyy_diag_add_y(&st, y_e0);
            camera_ouyy_diag_add_y(&st, y_e1);
            camera_ouyy_diag_add_y(&st, y_o0);
            camera_ouyy_diag_add_y(&st, y_o1);
        }
    }

    if (!st.y_count || !st.u_count || !st.v_count) return false;

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->profile = profile_id;
    out->elapsed_ms = elapsed_ms;
    out->y_mean = (float)((double)st.y_sum / (double)st.y_count);
    out->u_mean = (float)((double)st.u_sum / (double)st.u_count);
    out->v_mean = (float)((double)st.v_sum / (double)st.v_count);
    out->u_abs = (float)((double)st.u_abs_sum / (double)st.u_count);
    out->v_abs = (float)((double)st.v_abs_sum / (double)st.v_count);
    out->y_p10 = camera_ouyy_diag_percentile(st.y_hist, st.y_count, 10);
    out->y_p50 = camera_ouyy_diag_percentile(st.y_hist, st.y_count, 50);
    out->y_p90 = camera_ouyy_diag_percentile(st.y_hist, st.y_count, 90);

    if (roi_name_out) *roi_name_out = roi_name;
    if (x0_out) *x0_out = x0;
    if (y0_out) *y0_out = y0;
    if (rw_out) *rw_out = rw;
    if (rh_out) *rh_out = rh;
    return true;
}

static void camera_yu12_diag_emit(const uint8_t *frame,
                                  size_t frame_bytes,
                                  uint32_t width,
                                  uint32_t height,
                                  const char *profile_name,
                                  camera_h264_profile_t profile_id,
                                  uint32_t elapsed_ms,
                                  uint8_t sample_index)
{
    camera_color_measurement_t m = {0};
    const char *roi_name = "?";
    uint32_t x0 = 0, y0 = 0, rw = 0, rh = 0;

    if (!camera_ouyy_diag_measure(frame, frame_bytes, width, height,
                                  profile_id, elapsed_ms, &m,
                                  &roi_name, &x0, &y0, &rw, &rh)) {
        const size_t row_bytes = ((size_t)width * 3U) / 2U;
        const size_t need = row_bytes * (size_t)height;
        ESP_LOGW(TAG,
                 "[OUYY-COLOR-MATCH] sample=%u profile=%s skipped: bytes=%u need=%u",
                 (unsigned)sample_index,
                 profile_name ? profile_name : "?",
                 (unsigned)frame_bytes,
                 (unsigned)need);
        return;
    }

    if (HA_VERBOSE_DIAG_LOGS) ESP_LOGW(TAG,
             "[OUYY-COLOR-MATCH] sample=%u elapsed=%ums profile=%s fmt=%ux%u "
             "layout=O_UYY_E_VYY roi=%s{x=%u y=%u w=%u h=%u} "
             "Y{mean=%.3f p10=%u p50=%u p90=%u n=%u} "
             "UV{Uodd=%.3f Veven=%.3f dU=%+.3f dV=%+.3f "
             "Uabs=%.3f Vabs=%.3f nU=%u nV=%u}",
             (unsigned)sample_index,
             (unsigned)elapsed_ms,
             profile_name ? profile_name : "?",
             (unsigned)width, (unsigned)height,
             roi_name,
             (unsigned)x0, (unsigned)y0, (unsigned)rw, (unsigned)rh,
             (double)m.y_mean,
             (unsigned)m.y_p10, (unsigned)m.y_p50, (unsigned)m.y_p90,
             14904U,
             (double)m.u_mean, (double)m.v_mean,
             (double)m.u_mean - 128.0, (double)m.v_mean - 128.0,
             (double)m.u_abs, (double)m.v_abs,
             3726U, 3726U);
}

/* [H264-FPS-CONFIG-AB-20260822]
 * Adds fpsab=1: same-boot cfg.fps 30/60/30 reopen A/B with camera STREAMOFF
 * and DSI hardware pattern held throughout. PTS step intentionally remains
 * 33 ms so cfg.fps is the only encoder-side variable.
 */

/* [H264-LCDPATTERN-AB-20260822]
 * Safe same-boot LCD framebuffer-traffic isolation for this live application.
 *
 * Do NOT deinitialize LVGL/BSP here. Background application tasks continue to
 * call bsp_display_lock(), and lvgl_port_deinit() destroys the LVGL mutex.
 * Instead keep the LVGL context alive, pause only its timer, and switch the
 * MIPI-DSI DPI panel to the ESP32-P4 hardware pattern generator.
 *
 * In ESP-IDF v5.5.x, esp_lcd_dpi_panel_set_pattern(non-NONE) disables the DPI
 * bridge framebuffer path before enabling the DSI host pattern generator. So
 * DSI PHY/video timing remains active while framebuffer/GDMA scanout reads are
 * removed. Pattern NONE restores normal framebuffer output afterwards.
 */
#include "esp_lvgl_port.h"
#include "esp_lcd_mipi_dsi.h"

/* esp_lvgl_port's LVGL9 display driver-data context is private. Its prefix is:
 * disp_type, io_handle, panel_handle, control_handle. We only read the public
 * esp_lcd panel handle from that prefix for this diagnostic. */
typedef struct {
    uint32_t disp_type;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_panel_handle_t control_handle;
} camera_diag_lvgl_display_ctx_prefix_t;

static esp_lcd_panel_handle_t camera_diag_lvgl_get_panel(lv_display_t *display)
{
    if (!display) return NULL;
    camera_diag_lvgl_display_ctx_prefix_t *ctx =
        (camera_diag_lvgl_display_ctx_prefix_t *)lv_display_get_driver_data(display);
    return ctx ? ctx->panel_handle : NULL;
}

static esp_err_t camera_diag_lcd_pattern_enter(lv_display_t *display,
                                                esp_lcd_panel_handle_t *panel_out,
                                                bool *lvgl_timer_paused_out)
{
    if (panel_out) *panel_out = NULL;
    if (lvgl_timer_paused_out) *lvgl_timer_paused_out = false;
    if (!display) return ESP_ERR_INVALID_ARG;

    esp_lcd_panel_handle_t panel = camera_diag_lvgl_get_panel(display);
    if (!panel) {
        ESP_LOGE(TAG, "[LCDPAT] LVGL display has no panel_handle");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lvgl_port_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCDPAT] lvgl_port_stop failed: %s", esp_err_to_name(err));
        return err;
    }
    if (lvgl_timer_paused_out) *lvgl_timer_paused_out = true;

    /* Wait until no LVGL handler/UI writer is inside the shared display
     * critical section before switching the DPI source. */
    if (!bsp_display_lock(1000)) {
        ESP_LOGE(TAG, "[LCDPAT] display lock timeout before pattern switch");
        (void)lvgl_port_resume();
        if (lvgl_timer_paused_out) *lvgl_timer_paused_out = false;
        return ESP_ERR_TIMEOUT;
    }
    err = esp_lcd_dpi_panel_set_pattern(panel, MIPI_DSI_PATTERN_BAR_VERTICAL);
    bsp_display_unlock();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[LCDPAT] set BAR_VERTICAL failed: %s", esp_err_to_name(err));
        (void)lvgl_port_resume();
        if (lvgl_timer_paused_out) *lvgl_timer_paused_out = false;
        return err;
    }

    if (panel_out) *panel_out = panel;
    ESP_LOGW(TAG,
             "[LCDPAT] DSI BAR_VERTICAL active: framebuffer scanout bypassed; LVGL mutex remains valid");
    return ESP_OK;
}

static esp_err_t camera_diag_lcd_pattern_leave(esp_lcd_panel_handle_t panel,
                                                bool *lvgl_timer_paused_io)
{
    esp_err_t first_err = ESP_OK;
    if (panel) {
        esp_err_t err = ESP_ERR_TIMEOUT;
        if (bsp_display_lock(0)) {
            err = esp_lcd_dpi_panel_set_pattern(panel, MIPI_DSI_PATTERN_NONE);
            bsp_display_unlock();
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[LCDPAT] restore pattern NONE failed: %s", esp_err_to_name(err));
            first_err = err;
        } else {
            ESP_LOGW(TAG, "[LCDPAT] framebuffer DPI path restored");
        }
    }
    if (lvgl_timer_paused_io && *lvgl_timer_paused_io) {
        esp_err_t err = lvgl_port_resume();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "[LCDPAT] lvgl_port_resume failed: %s", esp_err_to_name(err));
            if (first_err == ESP_OK) first_err = err;
        }
        *lvgl_timer_paused_io = false;
    }
    return first_err;
}

#define CAM_CSI_DEVICE              "/dev/video0"
#define CAM_CSI_WIDTH               1288U
#define CAM_CSI_HEIGHT              728U
#define CAM_CSI_BUFFER_COUNT        3U
#define CAM_CSI_FRAME_COUNT         12U
#define CAM_RAW10_FRAME_COUNT       30U
#define CAM_CAPTURE_BENCH_FRAME_COUNT 30U
#define CAM_CSI_DQ_TIMEOUT_SEC      3
#define CAM_CSI_MIN_DMA_LARGEST     (32U * 1024U)

/* ESP32-P4 rev1.x has no ISP image-crop hardware.  Keep the OV02C10/ISP
 * capture at native 1920x1080 YU12 (ESP_H264_RAW_FMT_O_UYY_E_VYY layout)
 * and crop a centered 1280x720 rectangle in memory for the H.264 input.
 * O_UYY_E_VYY stores 2 pixels in 3 bytes per row; even X/Y offsets preserve
 * both the byte grouping and odd-U/even-V line phase. */
#define H264_MEMCROP_W              1280U
#define H264_MEMCROP_H              720U
#define H264_MEMCROP_LEFT           320U
#define H264_MEMCROP_TOP            180U
#define H264_MEMCROP_BYTES          ((H264_MEMCROP_W * H264_MEMCROP_H * 3U) / 2U)
#define H264_DMA_CROP_BACKLOG       16U
#define H264_DMA_CROP_BATCH_ROWS    16U
#define H264_DMA_CROP_ALIGN         32U
#define H264_CROP_AB_PHASE_FRAMES   120U
#define H264_OVERLAP_FRAMES         120U
#define H264_OVERLAP_CAMAB_PHASE_FRAMES 120U
#define H264_OVERLAP_CAMAB_MIN_PUMP_FRAMES 100U
#define H264_OVERLAP_TASK_STACK     6144U
#define H264_OVERLAP_TASK_CORE      0

/* Continuous WebRTC product path. Keep all application work on Core0; Core1
 * remains reserved for the stepper pulse engine. 1080p H.264 HW setup needed
 * roughly a 140KB internal reference-frame allocation in the successful lab
 * run, so fail gracefully if the largest DMA/internal block is clearly below
 * that requirement. */
#define CAM_LIVE_TASK_CORE            0
#define CAM_LIVE_TASK_PRIO            4
#define CAM_LIVE_TASK_STACK           10240
#define CAM_LIVE_GOP_APPLY_TIMEOUT_MS   1200U
/* Repeated camera/ISP starts can leave the DMA heap fragmented even after
 * all camera buffers are released.  144KB was a useful preferred value for
 * a cold start, but it wrongly blocked a clean second start when the largest
 * block can settle around 90-104KB after a 720p60 Stop due to heap fragmentation.
 * Keep an 88KB hard floor so 720p60 -> 1080p can restart without reboot, while
 * still retaining a large safety margin above the historical low-DMA failure zone.
 * the old preferred threshold. */
#define CAM_LIVE_DMA_PREFERRED_LARGEST (144U * 1024U)
#define CAM_LIVE_MIN_DMA_LARGEST       (88U * 1024U)
#define CAM_LIVE_LOG_INTERVAL_US      (2LL * 1000000LL)
/* [CAM-FPS-2S-WINDOW-AB 2026-08-29]
 * The browser CAP/ENC values must describe current throughput, not the
 * lifetime average since PLAY.  Otherwise a temporary RTA slowdown remains
 * visible long after the camera has recovered.  Keep the old lifetime values
 * locally for the serial diagnostic log, while published stats use the most
 * recently completed two-second window. */
#define CAM_LIVE_FPS_WINDOW_US         (2LL * 1000000LL)
/* [CAM-720-QP39-GOP8-PROD-FINAL-R1-20260913]
 * 720p production result from the two-stage + final-matrix hardware sweep:
 * fixed QP39 + GOP8.  Keep 1288x728, nominal 60 fps, sensor/ISP/color and
 * all WebRTC/DMA/ref-pool behavior unchanged. */

/* [WEBRTC-DUAL-PROFILE-20260824]
 * Keep both previously verified sensor-native paths selectable.
 * 1080p historical fixed-QP30 reference: 1920x1080@30, 2-lane, GOP1 -> ~27-28 fps live.
 * 720p historical fixed-QP30 reference: native 1288x728@60, 2-lane, GOP1 -> prior direct-H264 wall ~55 fps.
 * The discarded 1080->720 CPU crop path is never used.
 */
typedef struct {
    camera_h264_profile_t id;
    const char *name;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t lanes;
    uint32_t gop;
    uint32_t bitrate;
    uint32_t qp;
    const char *stage;
} camera_live_profile_cfg_t;

static const camera_live_profile_cfg_t s_cam_live_profiles[] = {
    { CAMERA_H264_PROFILE_1080P,      "1080p",      1920U, 1080U, 30U, 2U, 5U, 4000000U, 35U, "streaming_1080p_gop5_qp35_prod" },
    { CAMERA_H264_PROFILE_720P_NATIVE,"720p60-native",1288U,  728U, 60U, 2U, 8U, 4000000U, 39U, "streaming_720p60_native_gop8_qp39_prod" },
};

static camera_h264_profile_t s_live_profile_selected = CAMERA_H264_PROFILE_1080P;

/* [CAM-1080-ADAPT-GOP-R2-20260912] Conservative start: 1080 begins at GOP5 and may probe GOP1 only
 * when the ESP-authoritative network controller proves enough headroom.
 * [CAM-720-QP39-GOP8-PROD-FINAL-R1-20260913] 720 stays fixed at production GOP8. */
static uint32_t s_live_1080_gop_selected = 5U;

static const camera_live_profile_cfg_t *camera_live_profile_cfg(camera_h264_profile_t id)
{
    for (size_t i = 0; i < sizeof(s_cam_live_profiles) / sizeof(s_cam_live_profiles[0]); ++i) {
        if (s_cam_live_profiles[i].id == id) return &s_cam_live_profiles[i];
    }
    return &s_cam_live_profiles[0];
}

const char *camera_h264_stream_profile_name(camera_h264_profile_t profile)
{
    return camera_live_profile_cfg(profile)->name;
}

#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif

#ifndef V4L2_PIX_FMT_SBGGR10
#define V4L2_PIX_FMT_SBGGR10 v4l2_fourcc('B', 'G', '1', '0')
#endif

typedef struct {
    void *start;
    size_t length;
} cam_map_buf_t;

typedef struct {
    bool consumed;
    bool running;
    bool video_inited;
} cam_csi_state_t;

static cam_csi_state_t s_state;

static size_t dma_free(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

static size_t dma_largest(void)
{
    return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
}

static size_t psram_free(void)
{
    return heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

typedef struct {
    uint32_t peri26;
    uint32_t peri27;
    uint32_t root0;
    uint32_t root1;
    uint32_t root2;
    uint32_t root3;
    uint32_t soc0;
    uint32_t soc1;
    uint32_t soc2;
    uint32_t soc3;
    uint32_t cpu_status0;
    uint32_t cpu_src_freq0;
    int32_t h264_src_sel;
} h264_clock_diag_t;

static h264_clock_diag_t h264_clock_diag_read(void)
{
    /* Read-only diagnostic. Do not modify RCC/H264 registers here. */
    h264_clock_diag_t d = {
        .peri26 = REG_READ(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG),
        .peri27 = REG_READ(HP_SYS_CLKRST_PERI_CLK_CTRL27_REG),
        .root0 = REG_READ(HP_SYS_CLKRST_ROOT_CLK_CTRL0_REG),
        .root1 = REG_READ(HP_SYS_CLKRST_ROOT_CLK_CTRL1_REG),
        .root2 = REG_READ(HP_SYS_CLKRST_ROOT_CLK_CTRL2_REG),
        .root3 = REG_READ(HP_SYS_CLKRST_ROOT_CLK_CTRL3_REG),
        .soc0 = REG_READ(HP_SYS_CLKRST_SOC_CLK_CTRL0_REG),
        .soc1 = REG_READ(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG),
        .soc2 = REG_READ(HP_SYS_CLKRST_SOC_CLK_CTRL2_REG),
        .soc3 = REG_READ(HP_SYS_CLKRST_SOC_CLK_CTRL3_REG),
        .cpu_status0 = REG_READ(HP_SYS_CLKRST_CPU_CLK_STATUS0_REG),
        .cpu_src_freq0 = REG_READ(HP_SYS_CLKRST_CPU_SRC_FREQ0_REG),
        .h264_src_sel = -1,
    };

    /* IDF generated register headers normally expose this field macro. If the
     * exact v5.5.4 header does not, keep -1 and use the raw PERI26 value plus
     * dump_h264_clock_defs.ps1 for decoding; the build remains safe. */
#ifdef HP_SYS_CLKRST_REG_H264_CLK_SRC_SEL
    d.h264_src_sel = (int32_t)REG_GET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG,
                                            HP_SYS_CLKRST_REG_H264_CLK_SRC_SEL);
#elif defined(HP_SYS_CLKRST_H264_CLK_SRC_SEL)
    d.h264_src_sel = (int32_t)REG_GET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG,
                                            HP_SYS_CLKRST_H264_CLK_SRC_SEL);
#endif
    return d;
}

static void h264_clock_diag_log(const char *phase, const h264_clock_diag_t *d)
{
    if (!d) return;
    ESP_LOGW(TAG,
             "[H264-CLK] %s READ-ONLY peri26=0x%08" PRIx32 " peri27=0x%08" PRIx32
             " src_sel=%ld root={%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32 "}"
             " soc={%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32 ",%08" PRIx32 "}"
             " cpu_status0=%08" PRIx32 " cpu_src_freq0=%08" PRIx32,
             phase ? phase : "?", d->peri26, d->peri27, (long)d->h264_src_sel,
             d->root0, d->root1, d->root2, d->root3,
             d->soc0, d->soc1, d->soc2, d->soc3,
             d->cpu_status0, d->cpu_src_freq0);
}

/* [AXI-QOS-READONLY-DIAG-20260822]
 * Read-only snapshot of the ESP32-P4 AXI interconnect QoS fields that directly
 * identify the DW-GDMA camera master ports and the H264 DMA master ports.
 *
 * IMPORTANT: this diagnostic does not write any QoS/priority/register value.
 * It exists only to prove the actual runtime reset/boot values before any A/B.
 */
typedef struct {
    uint32_t ar_raw;
    uint32_t aw_raw;

    uint8_t cpu_r;
    uint8_t cpu_w;
    uint8_t cache_r;
    uint8_t cache_w;
    uint8_t dma2d_r;
    uint8_t dma2d_w;

    uint8_t gdma_m1_r;
    uint8_t gdma_m1_w;
    uint8_t gdma_m2_r;
    uint8_t gdma_m2_w;

    uint8_t h264_m1_r;
    uint8_t h264_m1_w;
    uint8_t h264_m2_r;
    uint8_t h264_m2_w;

    uint8_t pdma_r;
    uint8_t pdma_w;
} axi_qos_diag_t;

static axi_qos_diag_t axi_qos_diag_read(void)
{
    axi_qos_diag_t d = {
        .ar_raw = AXI_ICM.mst_arqos_reg0.val,
        .aw_raw = AXI_ICM.mst_awqos_reg0.val,

        .cpu_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_cpu_arqos,
        .cpu_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_cpu_awqos,
        .cache_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_cache_arqos,
        .cache_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_cache_awqos,
        .dma2d_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_dma2d_arqos,
        .dma2d_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_dma2d_awqos,

        .gdma_m1_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_gdma_mst1_arqos,
        .gdma_m1_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_gdma_mst1_awqos,
        .gdma_m2_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_gdma_mst2_arqos,
        .gdma_m2_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_gdma_mst2_awqos,

        .h264_m1_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_h264_dma2d_m1_arqos,
        .h264_m1_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_h264_dma2d_m1_awqos,
        .h264_m2_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_h264_dma2d_m2_arqos,
        .h264_m2_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_h264_dma2d_m2_awqos,

        .pdma_r = (uint8_t)AXI_ICM.mst_arqos_reg0.reg_axi_pdma_int_arqos,
        .pdma_w = (uint8_t)AXI_ICM.mst_awqos_reg0.reg_pdma_int_awqos,
    };
    return d;
}

static void axi_qos_diag_log(const char *phase, const axi_qos_diag_t *d)
{
    if (!d) return;
    ESP_LOGW(TAG,
             "[AXI-QOS] %s READ-ONLY ar=0x%08" PRIx32 " aw=0x%08" PRIx32
             " cpu{r=%u,w=%u} cache{r=%u,w=%u} dma2d{r=%u,w=%u}"
             " gdma_m1{r=%u,w=%u} gdma_m2{r=%u,w=%u}"
             " h264_m1{r=%u,w=%u} h264_m2{r=%u,w=%u} pdma{r=%u,w=%u}",
             phase ? phase : "?", d->ar_raw, d->aw_raw,
             (unsigned)d->cpu_r, (unsigned)d->cpu_w,
             (unsigned)d->cache_r, (unsigned)d->cache_w,
             (unsigned)d->dma2d_r, (unsigned)d->dma2d_w,
             (unsigned)d->gdma_m1_r, (unsigned)d->gdma_m1_w,
             (unsigned)d->gdma_m2_r, (unsigned)d->gdma_m2_w,
             (unsigned)d->h264_m1_r, (unsigned)d->h264_m1_w,
             (unsigned)d->h264_m2_r, (unsigned)d->h264_m2_w,
             (unsigned)d->pdma_r, (unsigned)d->pdma_w);
}

static void fourcc_to_str(uint32_t f, char out[5])
{
    out[0] = (char)(f & 0xff);
    out[1] = (char)((f >> 8) & 0xff);
    out[2] = (char)((f >> 16) & 0xff);
    out[3] = (char)((f >> 24) & 0xff);
    out[4] = '\0';
}

static uint32_t sparse_fnv1a(const uint8_t *data, size_t len)
{
    uint32_t h = 2166136261u;
    if (data == NULL || len == 0) {
        return h;
    }

    size_t step = len / 2048U;
    if (step == 0) {
        step = 1;
    }
    for (size_t i = 0; i < len; i += step) {
        h ^= data[i];
        h *= 16777619u;
    }
    h ^= (uint32_t)len;
    h *= 16777619u;
    return h;
}

static esp_err_t h264_memcrop_ouyy_evyy(const uint8_t *src, size_t src_len,
                                        uint32_t src_w, uint32_t src_h,
                                        uint8_t *dst, size_t dst_len)
{
    if (!src || !dst ||
        src_w < (H264_MEMCROP_LEFT + H264_MEMCROP_W) ||
        src_h < (H264_MEMCROP_TOP + H264_MEMCROP_H) ||
        (src_w & 1U) || (src_h & 1U) ||
        (H264_MEMCROP_LEFT & 1U) || (H264_MEMCROP_TOP & 1U) ||
        dst_len < H264_MEMCROP_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t src_stride = ((size_t)src_w * 3U) / 2U;
    const size_t dst_stride = ((size_t)H264_MEMCROP_W * 3U) / 2U;
    const size_t x_off = ((size_t)H264_MEMCROP_LEFT / 2U) * 3U;
    const size_t last_row_end =
        ((size_t)(H264_MEMCROP_TOP + H264_MEMCROP_H - 1U) * src_stride) +
        x_off + dst_stride;
    if (src_len < last_row_end) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *src_row = src + (size_t)H264_MEMCROP_TOP * src_stride + x_off;
    for (uint32_t y = 0; y < H264_MEMCROP_H; ++y) {
        memcpy(dst + (size_t)y * dst_stride,
               src_row + (size_t)y * src_stride,
               dst_stride);
    }
    return ESP_OK;
}


/* [LIVE720-OVERLAP-20260823]
 * Producer task for the double-buffer experiment.  It is deliberately pinned
 * to Core0 so Core1 remains reserved for the stepper pulse engine.  The H.264
 * HW encoder blocks on its FRAME_DONE synchronization object while hardware is
 * running, so this Core0 task can execute DQBUF + CPU row-crop + QBUF during
 * that wait without touching the H.264 input buffer currently being encoded.
 */
typedef enum {
    H264_OVERLAP_JOB_OK = 0,
    H264_OVERLAP_JOB_DQBUF,
    H264_OVERLAP_JOB_BAD_BUFFER,
    H264_OVERLAP_JOB_CROP,
    H264_OVERLAP_JOB_QBUF,
} h264_overlap_job_status_t;

typedef struct {
    int cam_fd;
    cam_map_buf_t *maps;
    uint32_t mapped_count;
    uint32_t src_w;
    uint32_t src_h;

    SemaphoreHandle_t kick_sem;
    SemaphoreHandle_t done_sem;
    SemaphoreHandle_t exit_sem;
    TaskHandle_t task;
    volatile bool stop;

    /* Written by owner before kick_sem, consumed by worker. */
    uint8_t *dst;
    size_t dst_len;

    /* The controlled camera-DMA A/B reuses one dequeued raw frame in both
     * phases so image content and source address are identical. */
    bool fixed_src_mode;
    const uint8_t *fixed_src;
    size_t fixed_src_len;

    /* Written by worker before done_sem. */
    h264_overlap_job_status_t status;
    esp_err_t crop_ret;
    int io_errno;
    uint32_t dq_us;
    uint32_t crop_us;
    uint32_t raw_bytes;
    uint32_t raw_index;
} h264_overlap_crop_ctx_t;

static void h264_overlap_crop_task(void *arg)
{
    h264_overlap_crop_ctx_t *ctx = (h264_overlap_crop_ctx_t *)arg;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (xSemaphoreTake(ctx->kick_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (ctx->stop) {
            break;
        }

        ctx->status = H264_OVERLAP_JOB_OK;
        ctx->crop_ret = ESP_OK;
        ctx->io_errno = 0;
        ctx->dq_us = 0;
        ctx->crop_us = 0;
        ctx->raw_bytes = 0;
        ctx->raw_index = UINT32_MAX;

        if (ctx->fixed_src_mode) {
            if (!ctx->fixed_src || ctx->fixed_src_len == 0U) {
                ctx->io_errno = EFAULT;
                ctx->status = H264_OVERLAP_JOB_BAD_BUFFER;
            } else {
                ctx->raw_bytes = (uint32_t)ctx->fixed_src_len;
                int64_t c0 = esp_timer_get_time();
                ctx->crop_ret = h264_memcrop_ouyy_evyy(
                    ctx->fixed_src, ctx->fixed_src_len,
                    ctx->src_w, ctx->src_h, ctx->dst, ctx->dst_len);
                int64_t c1 = esp_timer_get_time();
                ctx->crop_us = (c1 > c0) ? (uint32_t)(c1 - c0) : 0U;
                if (ctx->crop_ret != ESP_OK) {
                    ctx->io_errno = EIO;
                    ctx->status = H264_OVERLAP_JOB_CROP;
                }
            }
        } else {
            struct v4l2_buffer b = {
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
            };

            int64_t dq0 = esp_timer_get_time();
            if (ioctl(ctx->cam_fd, VIDIOC_DQBUF, &b) != 0) {
                int64_t dq1 = esp_timer_get_time();
                ctx->dq_us = (dq1 > dq0) ? (uint32_t)(dq1 - dq0) : 0U;
                ctx->io_errno = errno;
                ctx->status = H264_OVERLAP_JOB_DQBUF;
                xSemaphoreGive(ctx->done_sem);
                continue;
            }
            int64_t dq1 = esp_timer_get_time();
            ctx->dq_us = (dq1 > dq0) ? (uint32_t)(dq1 - dq0) : 0U;
            ctx->raw_index = b.index;
            ctx->raw_bytes = b.bytesused;

            if (b.index >= ctx->mapped_count || !ctx->maps[b.index].start || b.bytesused == 0) {
                ctx->io_errno = EFAULT;
                ctx->status = H264_OVERLAP_JOB_BAD_BUFFER;
            } else {
                int64_t c0 = esp_timer_get_time();
                ctx->crop_ret = h264_memcrop_ouyy_evyy(
                    (const uint8_t *)ctx->maps[b.index].start, b.bytesused,
                    ctx->src_w, ctx->src_h, ctx->dst, ctx->dst_len);
                int64_t c1 = esp_timer_get_time();
                ctx->crop_us = (c1 > c0) ? (uint32_t)(c1 - c0) : 0U;
                if (ctx->crop_ret != ESP_OK) {
                    ctx->io_errno = EIO;
                    ctx->status = H264_OVERLAP_JOB_CROP;
                }
            }

            if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &b) != 0) {
                if (ctx->status == H264_OVERLAP_JOB_OK) {
                    ctx->io_errno = errno;
                    ctx->status = H264_OVERLAP_JOB_QBUF;
                }
            }
        }

        xSemaphoreGive(ctx->done_sem);
    }

    if (ctx->exit_sem) {
        xSemaphoreGive(ctx->exit_sem);
    }
    vTaskDelete(NULL);
}

/* [LIVE720-OVERLAP-CAMAB-PUMP-20260823]
 * Keep camera DMA active without changing the fixed raw frame used by H.264.
 * One source buffer remains held dequeued; this task cycles the other buffers.
 * STREAMOFF at the A/B boundary wakes a blocked DQBUF. */
typedef struct {
    int cam_fd;
    SemaphoreHandle_t exit_sem;
    TaskHandle_t task;
    volatile bool stop;
    volatile bool failed;
    int io_errno;
    uint32_t frames;
    uint64_t dq_us_total;
    uint32_t dq_us_max;
} h264_overlap_cam_pump_ctx_t;

static void h264_overlap_cam_pump_task(void *arg)
{
    h264_overlap_cam_pump_ctx_t *ctx = (h264_overlap_cam_pump_ctx_t *)arg;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }
    while (!ctx->stop) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        int64_t d0 = esp_timer_get_time();
        if (ioctl(ctx->cam_fd, VIDIOC_DQBUF, &b) != 0) {
            int e = errno;
            if (ctx->stop) break;
            /* CAMAB uses a short DQBUF timeout so the pump can terminate
             * promptly after STREAMOFF. A timeout while CAM_ON is not a
             * camera failure; just retry. */
            if (e == ETIMEDOUT || e == EAGAIN) {
                continue;
            }
            ctx->io_errno = e;
            ctx->failed = true;
            break;
        }
        int64_t d1 = esp_timer_get_time();
        uint32_t dus = (d1 > d0) ? (uint32_t)(d1 - d0) : 0U;
        ctx->dq_us_total += dus;
        if (dus > ctx->dq_us_max) ctx->dq_us_max = dus;
        if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &b) != 0) {
            int e = errno;
            if (ctx->stop) break;
            ctx->io_errno = e;
            ctx->failed = true;
            break;
        }
        ctx->frames++;

        /* A continuously-ready high-priority pump can consume every small
         * idle gap left by crop+H264 on Core0. Yield one tick occasionally;
         * at 30 fps this is infrequent enough not to disturb camera cadence
         * but prevents IDLE0 starvation/task-WDT. */
        if ((ctx->frames & 0x0FU) == 0U) {
            vTaskDelay(1);
        }
    }
    if (ctx->exit_sem) xSemaphoreGive(ctx->exit_sem);
    vTaskDelete(NULL);
}


/* PPA SRM single-transaction crop for ESP32-P4 rev1.x.
 *
 * WHY NOT PPA_SRM_COLOR_MODE_YUV420:
 * The H.264 input here is ESP_H264_RAW_FMT_O_UYY_E_VYY, a line-interleaved
 * layout where every pixel row occupies exactly width*3/2 bytes and each
 * horizontal pair of pixels is stored as a 3-byte {chroma,Y,Y} group.  The CPU
 * reference crop treats the frame as a pure 2D byte rectangle and copies it
 * verbatim, so the H.264 encoder receives untouched sensor bytes.
 *
 * If PPA is asked to process this data as PPA_SRM_COLOR_MODE_YUV420, the SRM
 * engine (a) runs a YUV<->RGB colour-space round trip (BT.601/709 matrix on the
 * Rx side and again on the Tx side) and (b) always passes the picture through
 * its bilinear scaler.  Even at scale 1.0 those two stages perturb the bytes,
 * so the output can never be byte-identical to the O_UYY_E_VYY reference and the
 * validation below would abort every time.  PPA_SRM_COLOR_MODE_YUV420 also
 * assumes a standard YUV420 memory layout, not this line-interleaved one.
 *
 * WHAT WE DO INSTEAD (byte-mover trick):
 * Re-interpret the source as an RGB888 picture.  Because O_UYY_E_VYY packs
 * exactly 3 bytes per 2 pixels, one O_UYY_E_VYY row of width W maps 1:1 onto an
 * RGB888 row of W/2 "pixels" (W*3/2 bytes).  RGB888->RGB888 at scale 1.0 does no
 * colour conversion, so the SRM engine moves the 3-byte groups atomically and
 * preserves the O_UYY_E_VYY structure.  The centred crop becomes:
 *     in  : pic_w = src_w/2, offset_x = LEFT/2, block_w = 1280/2  (RGB888 px)
 *     out : pic_w = 1280/2, block_w = 1280/2                      (RGB888 px)
 * Row geometry (offset_y/block_h/pic_h) is unchanged because rows are 1:1.
 * LEFT and W are even, so LEFT/2 and W/2 stay on exact 3-byte group boundaries.
 *
 * IMPORTANT: this remains a diagnostic.  It still validates the PPA output
 * byte-for-byte against the known-good CPU O_UYY_E_VYY row crop on one fresh
 * camera frame before any PPA performance measurement is allowed to run.  If the
 * SRM bilinear stage is not a true identity at scale 1.0 on this silicon, the
 * mismatch is caught here and the benchmark aborts rather than feeding corrupt
 * data to H.264.
 */
static esp_err_t h264_memcrop_ouyy_evyy_ppa(ppa_client_handle_t ppa_client,
                                            const uint8_t *src, size_t src_len,
                                            uint32_t src_w, uint32_t src_h,
                                            uint8_t *dst, size_t dst_len)
{
    if (!ppa_client || !src || !dst ||
        src_w < (H264_MEMCROP_LEFT + H264_MEMCROP_W) ||
        src_h < (H264_MEMCROP_TOP + H264_MEMCROP_H) ||
        (src_w & 1U) || (src_h & 1U) ||
        (H264_MEMCROP_LEFT & 1U) || (H264_MEMCROP_TOP & 1U) ||
        dst_len < H264_MEMCROP_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t native_need = ((size_t)src_w * (size_t)src_h * 3U) / 2U;
    if (src_len < native_need) {
        return ESP_ERR_INVALID_ARG;
    }

    /* RGB888 re-interpretation: 3 bytes per 2 source pixels -> W/2 RGB888 px.
     * All divisions are exact because src_w, H264_MEMCROP_W and H264_MEMCROP_LEFT
     * are guaranteed even by the checks above. Row geometry stays 1:1. */
    const uint32_t in_pic_w_rgb   = src_w / 2U;               /* 1920 -> 960  */
    const uint32_t in_off_x_rgb   = H264_MEMCROP_LEFT / 2U;   /* 320  -> 160  */
    const uint32_t block_w_rgb    = H264_MEMCROP_W / 2U;      /* 1280 -> 640  */

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = src,
            .pic_w = in_pic_w_rgb,
            .pic_h = src_h,
            .block_w = block_w_rgb,
            .block_h = H264_MEMCROP_H,
            .block_offset_x = in_off_x_rgb,
            .block_offset_y = H264_MEMCROP_TOP,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB888,
        },
        .out = {
            .buffer = dst,
            .buffer_size = H264_MEMCROP_BYTES,
            .pic_w = block_w_rgb,
            .pic_h = H264_MEMCROP_H,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB888,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1.0f,
        .scale_y = 1.0f,
        .mirror_x = false,
        .mirror_y = false,
        .rgb_swap = false,
        .byte_swap = false,
        .alpha_update_mode = PPA_ALPHA_NO_CHANGE,
        .mode = PPA_TRANS_MODE_BLOCKING,
        .user_data = NULL,
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_client, &op);
    if (ret != ESP_OK) {
        return ret;
    }

    /* PPA writes the PSRAM destination behind the CPU cache.  The H264 driver
     * later performs input cache writeback, so invalidate first to prevent any
     * stale cache line from overwriting the freshly generated PPA result. */
    return esp_cache_msync(dst, H264_MEMCROP_BYTES,
                           ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                           ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
}


static bool h264_dma_crop_done_cb(async_memcpy_handle_t mcp_hdl,
                                  async_memcpy_event_t *event,
                                  void *cb_args)
{
    (void)mcp_hdl;
    (void)event;
    SemaphoreHandle_t done = (SemaphoreHandle_t)cb_args;
    BaseType_t high_task_wakeup = pdFALSE;
    if (done) {
        xSemaphoreGiveFromISR(done, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

/* AXI-GDMA row-copy version of the same centered O_UYY_E_VYY crop.
 * The packed layout uses a 1.5-byte/pixel logical row: native 1920 -> 2880 B,
 * 720p 1280 -> 1920 B.  left=320 maps to x_off=480 B.  All source/dest
 * addresses and lengths are 32-byte aligned, so psram_trans_align=32 is used.
 * Rows are queued in 16-row batches.  The callback on the last row of each
 * batch is sufficient because a single async-memcpy DMA queue completes
 * transactions in submission order. */
static esp_err_t h264_memcrop_ouyy_evyy_dma(async_memcpy_handle_t dma_hdl,
                                            SemaphoreHandle_t done_sem,
                                            const uint8_t *src, size_t src_len,
                                            uint32_t src_w, uint32_t src_h,
                                            uint8_t *dst, size_t dst_len)
{
    if (!dma_hdl || !done_sem || !src || !dst ||
        src_w < (H264_MEMCROP_LEFT + H264_MEMCROP_W) ||
        src_h < (H264_MEMCROP_TOP + H264_MEMCROP_H) ||
        (src_w & 1U) || (src_h & 1U) ||
        (H264_MEMCROP_LEFT & 1U) || (H264_MEMCROP_TOP & 1U) ||
        dst_len < H264_MEMCROP_BYTES) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t src_stride = ((size_t)src_w * 3U) / 2U;
    const size_t dst_stride = ((size_t)H264_MEMCROP_W * 3U) / 2U;
    const size_t x_off = ((size_t)H264_MEMCROP_LEFT / 2U) * 3U;
    const size_t last_row_end =
        ((size_t)(H264_MEMCROP_TOP + H264_MEMCROP_H - 1U) * src_stride) +
        x_off + dst_stride;
    if (src_len < last_row_end ||
        (((uintptr_t)dst) & (H264_DMA_CROP_ALIGN - 1U)) != 0U ||
        (dst_stride & (H264_DMA_CROP_ALIGN - 1U)) != 0U ||
        (src_stride & (H264_DMA_CROP_ALIGN - 1U)) != 0U ||
        ((x_off) & (H264_DMA_CROP_ALIGN - 1U)) != 0U) {
        ESP_LOGE(TAG, "CROPAB DMA geometry/alignment invalid dst=%p src_stride=%u dst_stride=%u x_off=%u src_len=%u need=%u",
                 (void *)dst, (unsigned)src_stride, (unsigned)dst_stride, (unsigned)x_off,
                 (unsigned)src_len, (unsigned)last_row_end);
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *src_row0 = src + (size_t)H264_MEMCROP_TOP * src_stride + x_off;
    if ((((uintptr_t)src_row0) & (H264_DMA_CROP_ALIGN - 1U)) != 0U) {
        ESP_LOGE(TAG, "CROPAB DMA source alignment invalid src=%p src_row0=%p align=%u",
                 (const void *)src, (const void *)src_row0, (unsigned)H264_DMA_CROP_ALIGN);
        return ESP_ERR_INVALID_ARG;
    }

    /* Drain a stale completion token if a previous failed transaction left one. */
    while (xSemaphoreTake(done_sem, 0) == pdTRUE) {
    }

    for (uint32_t y0 = 0; y0 < H264_MEMCROP_H; y0 += H264_DMA_CROP_BATCH_ROWS) {
        uint32_t rows = H264_MEMCROP_H - y0;
        if (rows > H264_DMA_CROP_BATCH_ROWS) rows = H264_DMA_CROP_BATCH_ROWS;

        for (uint32_t r = 0; r < rows; ++r) {
            uint32_t y = y0 + r;
            bool batch_last = (r + 1U) == rows;
            esp_err_t ret = esp_async_memcpy(
                dma_hdl,
                dst + (size_t)y * dst_stride,
                (void *)(src_row0 + (size_t)y * src_stride),
                dst_stride,
                batch_last ? h264_dma_crop_done_cb : NULL,
                batch_last ? (void *)done_sem : NULL);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "CROPAB DMA submit failed y=%u ret=%s(0x%x)",
                         (unsigned)y, esp_err_to_name(ret), (unsigned)ret);
                return ret;
            }
        }

        if (xSemaphoreTake(done_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(TAG, "CROPAB DMA timeout batch_start=%u rows=%u",
                     (unsigned)y0, (unsigned)rows);
            return ESP_ERR_TIMEOUT;
        }
    }

    /* DMA has updated PSRAM behind the CPU cache.  Invalidate before the H.264
     * driver performs its input-cache writeback, otherwise stale cached lines
     * could overwrite the freshly DMA-copied frame.  Buffer and length are
     * cache-line aligned, so no UNALIGNED flag is needed. */
    esp_err_t sync_ret = esp_cache_msync(dst, H264_MEMCROP_BYTES,
                                         ESP_CACHE_MSYNC_FLAG_DIR_M2C |
                                         ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
    if (sync_ret != ESP_OK) {
        ESP_LOGE(TAG, "CROPAB DMA cache M2C sync failed ret=%s(0x%x)",
                 esp_err_to_name(sync_ret), (unsigned)sync_ret);
    }
    return sync_ret;
}

static esp_err_t send_json_error(httpd_req_t *req, const char *status,
                                 const char *stage, int err_no, esp_err_t esp_err)
{
    char body[420];
    snprintf(body, sizeof(body),
             "{\"ok\":false,\"stage\":\"%s\",\"errno\":%d,\"esp_err\":%d,"
             "\"esp_err_name\":\"%s\",\"reboot_before_retest\":true}",
             stage ? stage : "unknown", err_no, (int)esp_err,
             esp_err_to_name(esp_err));
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t init_video_once(void)
{
    if (s_state.video_inited) {
        return ESP_OK;
    }

    esp_err_t ret = bsp_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bsp_i2c_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();
    if (i2c == NULL) {
        ESP_LOGE(TAG, "BSP I2C handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c,
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        /* LCD DSI already owns/powers the shared MIPI LDO on this board. */
        .dont_init_ldo = true,
    };
    const esp_video_init_config_t cfg = {
        .csi = &csi,
    };

    ESP_LOGI(TAG,
             "esp_video_init begin sensor=OV02C10 (active mode queried after init), reuse BSP I2C, dont_init_ldo=1");
    ret = esp_video_init(&cfg);
    if (ret != ESP_OK) {
        s_state.video_inited = false;
        ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * [CAM-INIT-VERIFY-SAFE 2026-09-01]
     *
     * esp_video_init() may return ESP_OK even when OV02C10 detection failed
     * and /dev/video0 was not created.  Do not promote the persistent
     * video_inited state from the return value alone.
     *
     * IMPORTANT:
     * - No automatic esp_video_init() retry here.
     * - No esp_video_deinit() here.
     * - No ISP/controller restart here.
     *
     * Re-init/deinit safety after a partial sensor-detect failure must be
     * verified in esp_video internals before automatic recovery is added.
     */
    int verify_fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (verify_fd < 0) {
        const int verify_errno = errno;

        s_state.video_inited = false;

        ESP_LOGE(TAG,
                 "[CAM-INIT-VERIFY-SAFE] esp_video_init returned OK but %s "
                 "is missing errno=%d",
                 CAM_CSI_DEVICE, verify_errno);

        return ESP_ERR_NOT_FOUND;
    }

    close(verify_fd);

    s_state.video_inited = true;

    ESP_LOGI(TAG,
             "[CAM-INIT-VERIFY-SAFE] esp_video_init OK + %s present",
             CAM_CSI_DEVICE);

    return ESP_OK;
}

static esp_err_t csi_test_handler(httpd_req_t *req)
{
    if (s_state.running || camera_h264_stream_is_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera CSI diagnostic already running\"}");
    }
    if (s_state.consumed) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"one-shot diagnostic already consumed\","
            "\"reboot_before_retest\":true}");
    }

    s_state.running = true;
    s_state.consumed = true;

    const size_t dma_free_before = dma_free();
    const size_t dma_largest_before = dma_largest();
    const size_t psram_before = psram_free();

    ESP_LOGI(TAG,
             "one-shot begin dma_free=%u dma_largest=%u psram_free=%u",
             (unsigned)dma_free_before, (unsigned)dma_largest_before, (unsigned)psram_before);

    if (dma_largest_before < CAM_CSI_MIN_DMA_LARGEST) {
        ESP_LOGE(TAG, "DMA largest guard: %u < %u",
                 (unsigned)dma_largest_before, (unsigned)CAM_CSI_MIN_DMA_LARGEST);
        s_state.running = false;
        return send_json_error(req, "503 Service Unavailable", "dma_guard", 0, ESP_ERR_NO_MEM);
    }

    esp_err_t ret = init_video_once();
    if (ret != ESP_OK) {
        s_state.running = false;
        return send_json_error(req, "500 Internal Server Error", "esp_video_init", 0, ret);
    }

    int fd = -1;
    bool streaming = false;
    cam_map_buf_t maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t mapped_count = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;

    fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (fd < 0) {
        saved_errno = errno;
        fail_stage = "open_video0";
        goto fail;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
        saved_errno = errno;
        fail_stage = "VIDIOC_QUERYCAP";
        goto fail;
    }
    ESP_LOGI(TAG, "device driver=%s card=%s bus=%s caps=0x%08" PRIx32,
             cap.driver, cap.card, cap.bus_info, cap.capabilities);

    /*
     * Read the sensor's real active mode first. esp_video's initial capture
     * v4l2_format may still contain generic/default dimensions and sizeimage=0
     * before an application selects a capture format.  The OV02C10 sensor
     * format is authoritative for the capture width/height.
     */
    esp_cam_sensor_format_t sensor_fmt = {0};
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "VIDIOC_G_SENSOR_FMT";
        goto fail;
    }
    ESP_LOGI(TAG,
             "sensor active name=%s %ux%u fps=%u format=%d lanes=%u mipi_clk=%u",
             sensor_fmt.name ? sensor_fmt.name : "(null)",
             (unsigned)sensor_fmt.width, (unsigned)sensor_fmt.height,
             (unsigned)sensor_fmt.fps, (int)sensor_fmt.format,
             (unsigned)sensor_fmt.mipi_info.lane_num,
             (unsigned)sensor_fmt.mipi_info.mipi_clk);

    if (sensor_fmt.width == 0 || sensor_fmt.height == 0) {
        saved_errno = EINVAL;
        fail_stage = "G_SENSOR_FMT_invalid";
        goto fail;
    }

    for (uint32_t idx = 0; idx < 16; ++idx) {
        struct v4l2_fmtdesc d = {
            .index = idx,
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        };
        if (ioctl(fd, VIDIOC_ENUM_FMT, &d) != 0) {
            break;
        }
        char fcc[5];
        fourcc_to_str(d.pixelformat, fcc);
        ESP_LOGI(TAG, "fmt[%u]=%s desc=%s", (unsigned)idx, fcc, d.description);
    }

    /* Log the pre-S_FMT value only for diagnostics. sizeimage=0 here is not
     * treated as an error: esp_video fills the concrete buffer geometry when
     * an application selects a valid format. */
    struct v4l2_format fmt_initial = {0};
    fmt_initial.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt_initial) == 0) {
        char initial_fourcc[5];
        fourcc_to_str(fmt_initial.fmt.pix.pixelformat, initial_fourcc);
        ESP_LOGI(TAG,
                 "pre-S_FMT=%ux%u fourcc=%s sizeimage=%u bytesperline=%u",
                 (unsigned)fmt_initial.fmt.pix.width,
                 (unsigned)fmt_initial.fmt.pix.height,
                 initial_fourcc,
                 (unsigned)fmt_initial.fmt.pix.sizeimage,
                 (unsigned)fmt_initial.fmt.pix.bytesperline);
    } else {
        ESP_LOGW(TAG, "initial VIDIOC_G_FMT failed errno=%d; continue to explicit S_FMT", errno);
    }

    /* H.264 /dev/video11 accepts YUV420 as its input, so make YU12 the first
     * capture target. RGB565 and direct RAW10 are diagnostic fallbacks only. */
    static const uint32_t capture_candidates[] = {
        V4L2_PIX_FMT_YUV420,
        V4L2_PIX_FMT_RGB565,
        V4L2_PIX_FMT_SBGGR10,
    };

    struct v4l2_format fmt = {0};
    bool fmt_set = false;
    int last_s_fmt_errno = EINVAL;
    for (size_t ci = 0; ci < sizeof(capture_candidates) / sizeof(capture_candidates[0]); ++ci) {
        struct v4l2_format try_fmt = {0};
        try_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        try_fmt.fmt.pix.width = sensor_fmt.width;
        try_fmt.fmt.pix.height = sensor_fmt.height;
        try_fmt.fmt.pix.pixelformat = capture_candidates[ci];
        try_fmt.fmt.pix.field = V4L2_FIELD_NONE;

        char want_fourcc[5];
        fourcc_to_str(capture_candidates[ci], want_fourcc);
        ESP_LOGI(TAG, "S_FMT try %ux%u %s",
                 (unsigned)try_fmt.fmt.pix.width,
                 (unsigned)try_fmt.fmt.pix.height, want_fourcc);

        if (ioctl(fd, VIDIOC_S_FMT, &try_fmt) == 0) {
            fmt = try_fmt;
            fmt_set = true;
            char got_fourcc[5];
            fourcc_to_str(fmt.fmt.pix.pixelformat, got_fourcc);
            ESP_LOGI(TAG,
                     "S_FMT accepted request=%s -> %ux%u %s sizeimage=%u bytesperline=%u",
                     want_fourcc,
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     got_fourcc, (unsigned)fmt.fmt.pix.sizeimage,
                     (unsigned)fmt.fmt.pix.bytesperline);
            break;
        }

        last_s_fmt_errno = errno;
        ESP_LOGW(TAG, "S_FMT rejected %s errno=%d (%s)",
                 want_fourcc, last_s_fmt_errno, strerror(last_s_fmt_errno));
    }

    if (!fmt_set) {
        saved_errno = last_s_fmt_errno;
        fail_stage = "VIDIOC_S_FMT_all_candidates";
        goto fail;
    }

    /* Read back the active capture format. Do not require sizeimage != 0;
     * QUERYBUF is the authoritative allocation length for MMAP buffers. */
    struct v4l2_format fmt_readback = {0};
    fmt_readback.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt_readback) == 0) {
        fmt = fmt_readback;
    } else {
        ESP_LOGW(TAG, "post-S_FMT VIDIOC_G_FMT failed errno=%d; use S_FMT returned format", errno);
    }

    char fourcc[5];
    fourcc_to_str(fmt.fmt.pix.pixelformat, fourcc);
    ESP_LOGI(TAG,
             "capture active=%ux%u fourcc=%s sizeimage=%u bytesperline=%u field=%u",
             (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height, fourcc,
             (unsigned)fmt.fmt.pix.sizeimage, (unsigned)fmt.fmt.pix.bytesperline,
             (unsigned)fmt.fmt.pix.field);

    if (fmt.fmt.pix.width == 0 || fmt.fmt.pix.height == 0 ||
        fmt.fmt.pix.pixelformat == 0) {
        saved_errno = EINVAL;
        fail_stage = "S_FMT_invalid_readback";
        goto fail;
    }

    struct timeval timeout = {
        .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC,
        .tv_usec = 0,
    };
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_DQBUF_TIMEOUT failed errno=%d; continuing", errno);
    }

    struct v4l2_requestbuffers reqbuf = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &reqbuf) != 0) {
        saved_errno = errno;
        fail_stage = "VIDIOC_REQBUFS";
        goto fail;
    }
    if (reqbuf.count < 2) {
        saved_errno = ENOMEM;
        fail_stage = "REQBUFS_count_lt_2";
        goto fail;
    }
    if (reqbuf.count > CAM_CSI_BUFFER_COUNT) {
        reqbuf.count = CAM_CSI_BUFFER_COUNT;
    }

    for (uint32_t i = 0; i < reqbuf.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "VIDIOC_QUERYBUF";
            goto fail;
        }
        maps[i].length = b.length;
        maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd, b.m.offset);
        if (maps[i].start == MAP_FAILED || maps[i].start == NULL) {
            maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "mmap";
            goto fail;
        }
        mapped_count++;
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "VIDIOC_QBUF_initial";
            goto fail;
        }
        ESP_LOGI(TAG, "buffer[%u] len=%u mapped", (unsigned)i, (unsigned)b.length);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        saved_errno = errno;
        fail_stage = "VIDIOC_STREAMON";
        goto fail;
    }
    streaming = true;
    ESP_LOGI(TAG, "STREAMON OK; capture %u frames", (unsigned)CAM_CSI_FRAME_COUNT);

    uint32_t frames = 0;
    uint32_t bytes_min = UINT32_MAX;
    uint32_t bytes_max = 0;
    uint32_t first_hash = 0;
    uint32_t last_hash = 0;
    int64_t t_first = 0;
    int64_t t_last = 0;

    for (uint32_t n = 0; n < CAM_CSI_FRAME_COUNT; ++n) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "VIDIOC_DQBUF";
            goto fail;
        }
        if (b.index >= mapped_count || maps[b.index].start == NULL) {
            saved_errno = EFAULT;
            fail_stage = "DQBUF_bad_index";
            goto fail;
        }

        int64_t now = esp_timer_get_time();
        if (frames == 0) {
            t_first = now;
        }
        t_last = now;
        uint32_t used = b.bytesused;
        if (used < bytes_min) bytes_min = used;
        if (used > bytes_max) bytes_max = used;
        uint32_t hash = sparse_fnv1a((const uint8_t *)maps[b.index].start,
                                     used <= maps[b.index].length ? used : maps[b.index].length);
        if (frames == 0) first_hash = hash;
        last_hash = hash;
        frames++;

        ESP_LOGI(TAG, "frame=%u idx=%u seq=%u bytes=%u hash=%08" PRIx32,
                 (unsigned)frames, (unsigned)b.index, (unsigned)b.sequence,
                 (unsigned)used, hash);

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "VIDIOC_QBUF_requeue";
            goto fail;
        }
    }

    if (streaming) {
        int off_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd, VIDIOC_STREAMOFF, &off_type) != 0) {
            ESP_LOGW(TAG, "STREAMOFF failed errno=%d", errno);
        } else {
            ESP_LOGI(TAG, "STREAMOFF OK");
        }
        streaming = false;
    }

    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start != NULL) {
            munmap(maps[i].start, maps[i].length);
            maps[i].start = NULL;
        }
    }
    close(fd);
    fd = -1;

    const int64_t elapsed_us = (frames > 1 && t_last > t_first) ? (t_last - t_first) : 0;
    uint32_t fps_x100 = 0;
    if (elapsed_us > 0 && frames > 1) {
        fps_x100 = (uint32_t)(((uint64_t)(frames - 1) * 100000000ULL) / (uint64_t)elapsed_us);
    }

    const size_t dma_free_after = dma_free();
    const size_t dma_largest_after = dma_largest();
    const size_t psram_after = psram_free();

    ESP_LOGI(TAG,
             "PASS frames=%u fps=%u.%02u hash_first=%08" PRIx32 " hash_last=%08" PRIx32
             " dma_largest=%u->%u",
             (unsigned)frames, (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
             first_hash, last_hash,
             (unsigned)dma_largest_before, (unsigned)dma_largest_after);
    ESP_LOGW(TAG, "one-shot diagnostic consumed; reboot before another CSI stream test");

    char sensor_desc[160];
    snprintf(sensor_desc, sizeof(sensor_desc), "%ux%u RAW10 %u-lane %ufps",
             (unsigned)sensor_fmt.width, (unsigned)sensor_fmt.height,
             (unsigned)sensor_fmt.mipi_info.lane_num, (unsigned)sensor_fmt.fps);

    char body[900];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"sensor\":\"OV02C10\",\"chip_id\":\"0x5602\","
             "\"device\":\"%s\",\"sensor_input\":\"%s\","
             "\"capture_policy\":\"sensor_fmt_then_yuv420_rgb565_raw10_fallback\","
             "\"actual\":{\"width\":%u,\"height\":%u,\"fourcc\":\"%s\",\"sizeimage\":%u},"
             "\"frames\":%u,\"fps\":\"%u.%02u\",\"bytes_min\":%u,\"bytes_max\":%u,"
             "\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\","
             "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
             "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
             "\"reboot_before_retest\":true}",
             CAM_CSI_DEVICE, sensor_desc,
             (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height, fourcc,
             (unsigned)fmt.fmt.pix.sizeimage,
             (unsigned)frames, (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
             (unsigned)bytes_min, (unsigned)bytes_max, first_hash, last_hash,
             (unsigned)dma_free_before, (unsigned)dma_largest_before,
             (unsigned)dma_free_after, (unsigned)dma_largest_after,
             (unsigned)psram_before, (unsigned)psram_after);

    s_state.running = false;
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);

fail:
    ESP_LOGE(TAG, "FAIL stage=%s errno=%d (%s)",
             fail_stage ? fail_stage : "unknown", saved_errno, strerror(saved_errno));
    if (streaming && fd >= 0) {
        int off_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd, VIDIOC_STREAMOFF, &off_type);
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start != NULL) {
            munmap(maps[i].start, maps[i].length);
        }
    }
    if (fd >= 0) {
        close(fd);
    }
    s_state.running = false;
    ESP_LOGW(TAG, "diagnostic failed after hardware touch; reboot before retest");
    return send_json_error(req, "500 Internal Server Error",
                           fail_stage ? fail_stage : "unknown", saved_errno, ESP_FAIL);
}


typedef struct {
    uint32_t sps;
    uint32_t pps;
    uint32_t idr;
    uint32_t non_idr;
} h264_nal_stats_t;

static void h264_scan_annexb(const uint8_t *data, size_t len, h264_nal_stats_t *stats)
{
    if (!data || !stats || len < 4) {
        return;
    }

    size_t i = 0;
    while (i + 4 <= len) {
        size_t sc = 0;
        if (i + 3 < len && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            sc = 3;
        } else if (i + 4 < len && data[i] == 0 && data[i + 1] == 0 &&
                   data[i + 2] == 0 && data[i + 3] == 1) {
            sc = 4;
        }

        if (sc == 0) {
            ++i;
            continue;
        }

        if (i + sc < len) {
            uint8_t nal_type = data[i + sc] & 0x1f;
            if (nal_type == 7) stats->sps++;
            else if (nal_type == 8) stats->pps++;
            else if (nal_type == 5) stats->idr++;
            else if (nal_type == 1) stats->non_idr++;
        }
        i += sc + 1;
    }
}

static bool h264_get_sps_profile_level(const uint8_t *data, size_t len,
                                       uint8_t *profile_idc,
                                       uint8_t *constraint_flags,
                                       uint8_t *level_idc)
{
    if (!data || len < 8 || !profile_idc || !constraint_flags || !level_idc) {
        return false;
    }

    for (size_t i = 0; i + 7 < len; ++i) {
        size_t sc = 0;
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            sc = 3;
        } else if (i + 4 < len && data[i] == 0 && data[i + 1] == 0 &&
                   data[i + 2] == 0 && data[i + 3] == 1) {
            sc = 4;
        }
        if (!sc || i + sc + 3 >= len) {
            continue;
        }
        const uint8_t *nal = &data[i + sc];
        if ((nal[0] & 0x1f) == 7) {
            /* SPS RBSP starts after the one-byte NAL header:
             * profile_idc, constraint_set flags/reserved, level_idc. */
            *profile_idc = nal[1];
            *constraint_flags = nal[2];
            *level_idc = nal[3];
            return true;
        }
    }
    return false;
}

static void h264_set_ctrl_best_effort(int fd, uint32_t id, int32_t value, const char *name)
{
    struct v4l2_ext_control ctrl = {
        .id = id,
        .value = value,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CID_CODEC_CLASS,
        .count = 1,
        .controls = &ctrl,
    };

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGW(TAG, "H264 ctrl %s=%ld rejected errno=%d (%s); use driver default",
                 name, (long)value, errno, strerror(errno));
    } else {
        ESP_LOGI(TAG, "H264 ctrl %s=%ld", name, (long)value);
    }
}

static esp_err_t h264_set_ctrl_required(int fd, uint32_t id, int32_t value, const char *name)
{
    struct v4l2_ext_control ctrl = {
        .id = id,
        .value = value,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CID_CODEC_CLASS,
        .count = 1,
        .controls = &ctrl,
    };
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGE(TAG, "LIVE required H264 ctrl %s=%ld rejected errno=%d (%s)",
                 name, (long)value, errno, strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LIVE H264 ctrl %s=%ld", name, (long)value);
    return ESP_OK;
}

static void h264_streamparm_probe_best_effort(int fd, uint32_t fps)
{
    /* /dev/video11 is an H.264 M2M codec device.  esp_video does not require
     * VIDIOC_S_PARM on this node; frame cadence is supplied by the 30 fps
     * camera input.  Keep this only as a diagnostic probe and never abort the
     * live stream if the codec node rejects it. */
    struct v4l2_streamparm p = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    p.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    p.parm.capture.timeperframe.numerator = 1;
    p.parm.capture.timeperframe.denominator = fps;
    if (ioctl(fd, VIDIOC_S_PARM, &p) != 0) {
        ESP_LOGW(TAG,
                 "LIVE encoder S_PARM %ufps unsupported errno=%d (%s); continue, cadence follows camera",
                 (unsigned)fps, errno, strerror(errno));
        return;
    }
    struct v4l2_streamparm g = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    if (ioctl(fd, VIDIOC_G_PARM, &g) == 0) {
        const struct v4l2_fract *t = &g.parm.capture.timeperframe;
        uint32_t fps_x100 = (t->numerator && t->denominator)
            ? (uint32_t)(((uint64_t)t->denominator * 100ULL) / t->numerator) : 0U;
        ESP_LOGI(TAG, "LIVE encoder G_PARM=%u.%02u fps",
                 (unsigned)(fps_x100 / 100U), (unsigned)(fps_x100 % 100U));
    }
}

static esp_err_t h264_test_handler(httpd_req_t *req)
{
    if (s_state.running || camera_h264_stream_is_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera diagnostic already running\"}");
    }
    if (s_state.consumed) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"one-shot camera diagnostic already consumed\"," 
            "\"reboot_before_retest\":true}");
    }

    s_state.running = true;
    s_state.consumed = true;

    const size_t dma_free_before = dma_free();
    const size_t dma_largest_before = dma_largest();
    const size_t psram_before = psram_free();

    ESP_LOGI(TAG, "H264 one-shot begin dma_free=%u dma_largest=%u psram_free=%u",
             (unsigned)dma_free_before, (unsigned)dma_largest_before, (unsigned)psram_before);

    if (dma_largest_before < CAM_CSI_MIN_DMA_LARGEST) {
        s_state.running = false;
        return send_json_error(req, "503 Service Unavailable", "h264_dma_guard", 0, ESP_ERR_NO_MEM);
    }

    esp_err_t ret = init_video_once();
    if (ret != ESP_OK) {
        s_state.running = false;
        return send_json_error(req, "500 Internal Server Error", "h264_esp_video_init", 0, ret);
    }

    int cam_fd = -1;
    int enc_fd = -1;
    bool cam_streaming = false;
    bool enc_out_streaming = false;
    bool enc_cap_streaming = false;
    cam_map_buf_t cam_maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t cam_mapped_count = 0;
    void *enc_map = NULL;
    size_t enc_map_len = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t camera_frame_bytes = 0;

    cam_fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (cam_fd < 0) {
        saved_errno = errno;
        fail_stage = "h264_open_video0";
        goto h264_fail;
    }

    struct v4l2_capability cam_cap = {0};
    if (ioctl(cam_fd, VIDIOC_QUERYCAP, &cam_cap) != 0) {
        saved_errno = errno;
        fail_stage = "h264_camera_QUERYCAP";
        goto h264_fail;
    }

    esp_cam_sensor_format_t sensor_fmt = {0};
    if (ioctl(cam_fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_G_SENSOR_FMT";
        goto h264_fail;
    }
    width = sensor_fmt.width;
    height = sensor_fmt.height;
    if (width == 0 || height == 0) {
        saved_errno = EINVAL;
        fail_stage = "h264_sensor_fmt_invalid";
        goto h264_fail;
    }

    ESP_LOGI(TAG,
             "H264 sensor active name=%s %ux%u fps=%u lanes=%u mipi_clk=%u",
             sensor_fmt.name ? sensor_fmt.name : "(null)",
             (unsigned)width, (unsigned)height, (unsigned)sensor_fmt.fps,
             (unsigned)sensor_fmt.mipi_info.lane_num,
             (unsigned)sensor_fmt.mipi_info.mipi_clk);

    struct v4l2_format cam_fmt = {0};
    cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cam_fmt.fmt.pix.width = width;
    cam_fmt.fmt.pix.height = height;
    cam_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    cam_fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(cam_fd, VIDIOC_S_FMT, &cam_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_camera_S_FMT_YU12";
        goto h264_fail;
    }
    width = cam_fmt.fmt.pix.width;
    height = cam_fmt.fmt.pix.height;
    char cam_fourcc[5];
    fourcc_to_str(cam_fmt.fmt.pix.pixelformat, cam_fourcc);
    ESP_LOGI(TAG, "H264 camera format=%ux%u %s sizeimage=%u",
             (unsigned)width, (unsigned)height, cam_fourcc,
             (unsigned)cam_fmt.fmt.pix.sizeimage);
    if (cam_fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        saved_errno = EINVAL;
        fail_stage = "h264_camera_not_YU12";
        goto h264_fail;
    }

    struct timeval timeout = {
        .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC,
        .tv_usec = 0,
    };
    if (ioctl(cam_fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "camera DQ timeout ioctl rejected errno=%d; continue", errno);
    }

    struct v4l2_requestbuffers cam_req = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &cam_req) != 0 || cam_req.count < 2) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_camera_REQBUFS";
        goto h264_fail;
    }
    if (cam_req.count > CAM_CSI_BUFFER_COUNT) cam_req.count = CAM_CSI_BUFFER_COUNT;

    for (uint32_t i = 0; i < cam_req.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_QUERYBUF";
            goto h264_fail;
        }
        cam_maps[i].length = b.length;
        cam_maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, cam_fd, b.m.offset);
        if (cam_maps[i].start == MAP_FAILED || cam_maps[i].start == NULL) {
            cam_maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "h264_camera_mmap";
            goto h264_fail;
        }
        cam_mapped_count++;
        if (ioctl(cam_fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_QBUF_initial";
            goto h264_fail;
        }
        ESP_LOGI(TAG, "H264 camera buffer[%u] len=%u", (unsigned)i, (unsigned)b.length);
    }

    enc_fd = open("/dev/video11", O_RDWR);
    if (enc_fd < 0) {
        saved_errno = errno;
        fail_stage = "h264_open_video11";
        goto h264_fail;
    }

    struct v4l2_capability enc_capability = {0};
    if (ioctl(enc_fd, VIDIOC_QUERYCAP, &enc_capability) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_QUERYCAP";
        goto h264_fail;
    }
    ESP_LOGI(TAG, "H264 encoder driver=%s card=%s bus=%s caps=0x%08" PRIx32,
             enc_capability.driver, enc_capability.card, enc_capability.bus_info,
             enc_capability.capabilities);

    if (ioctl(enc_fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "encoder DQ timeout ioctl rejected errno=%d; continue", errno);
    }

    /* WebRTC-friendly starting point; if a control is unsupported, the driver default is retained. */
    h264_set_ctrl_best_effort(enc_fd, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 30, "i_period");
    h264_set_ctrl_best_effort(enc_fd, V4L2_CID_MPEG_VIDEO_BITRATE, 4000000, "bitrate");
    h264_set_ctrl_best_effort(enc_fd, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 20, "min_qp");
    h264_set_ctrl_best_effort(enc_fd, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 40, "max_qp");

    struct v4l2_format enc_in_fmt = {0};
    enc_in_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    enc_in_fmt.fmt.pix.width = width;
    enc_in_fmt.fmt.pix.height = height;
    enc_in_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(enc_fd, VIDIOC_S_FMT, &enc_in_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_S_FMT_OUTPUT_YU12";
        goto h264_fail;
    }
    char enc_in_fourcc[5];
    fourcc_to_str(enc_in_fmt.fmt.pix.pixelformat, enc_in_fourcc);
    ESP_LOGI(TAG, "H264 encoder input=%ux%u %s sizeimage=%u",
             (unsigned)enc_in_fmt.fmt.pix.width, (unsigned)enc_in_fmt.fmt.pix.height,
             enc_in_fourcc, (unsigned)enc_in_fmt.fmt.pix.sizeimage);

    struct v4l2_requestbuffers enc_out_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
    };
    if (ioctl(enc_fd, VIDIOC_REQBUFS, &enc_out_req) != 0 || enc_out_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_encoder_REQBUFS_OUTPUT";
        goto h264_fail;
    }
    struct v4l2_buffer enc_out_query = {
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
        .index = 0,
    };
    if (ioctl(enc_fd, VIDIOC_QUERYBUF, &enc_out_query) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_QUERYBUF_OUTPUT";
        goto h264_fail;
    }
    ESP_LOGI(TAG, "H264 encoder USERPTR slot len=%u", (unsigned)enc_out_query.length);

    struct v4l2_format enc_cap_fmt = {0};
    enc_cap_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    enc_cap_fmt.fmt.pix.width = width;
    enc_cap_fmt.fmt.pix.height = height;
    enc_cap_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    if (ioctl(enc_fd, VIDIOC_S_FMT, &enc_cap_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_S_FMT_CAPTURE_H264";
        goto h264_fail;
    }
    char enc_out_fourcc[5];
    fourcc_to_str(enc_cap_fmt.fmt.pix.pixelformat, enc_out_fourcc);
    ESP_LOGI(TAG, "H264 encoder output=%ux%u %s sizeimage=%u",
             (unsigned)enc_cap_fmt.fmt.pix.width, (unsigned)enc_cap_fmt.fmt.pix.height,
             enc_out_fourcc, (unsigned)enc_cap_fmt.fmt.pix.sizeimage);

    struct v4l2_requestbuffers enc_cap_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(enc_fd, VIDIOC_REQBUFS, &enc_cap_req) != 0 || enc_cap_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_encoder_REQBUFS_CAPTURE";
        goto h264_fail;
    }

    struct v4l2_buffer enc_cap_buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
        .index = 0,
    };
    if (ioctl(enc_fd, VIDIOC_QUERYBUF, &enc_cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_QUERYBUF_CAPTURE";
        goto h264_fail;
    }
    enc_map_len = enc_cap_buf.length;
    enc_map = mmap(NULL, enc_map_len, PROT_READ | PROT_WRITE,
                   MAP_SHARED, enc_fd, enc_cap_buf.m.offset);
    if (enc_map == MAP_FAILED || enc_map == NULL) {
        enc_map = NULL;
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_encoder_mmap_CAPTURE";
        goto h264_fail;
    }
    if (ioctl(enc_fd, VIDIOC_QBUF, &enc_cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_QBUF_CAPTURE_initial";
        goto h264_fail;
    }
    ESP_LOGI(TAG, "H264 bitstream buffer len=%u mapped", (unsigned)enc_map_len);

    int enc_cap_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(enc_fd, VIDIOC_STREAMON, &enc_cap_type) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_STREAMON_CAPTURE";
        goto h264_fail;
    }
    enc_cap_streaming = true;

    int enc_out_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(enc_fd, VIDIOC_STREAMON, &enc_out_type) != 0) {
        saved_errno = errno;
        fail_stage = "h264_encoder_STREAMON_OUTPUT";
        goto h264_fail;
    }
    enc_out_streaming = true;

    /* [C12H-AB1-REMOVE-WB-ONLY 2026-08-27]
     * C12B pre-STREAMON WB replay intentionally removed.
     * C12G S_SELECTION, 720 geometry/four-field/fresh-IPA, and
     * 1080 C12C fresh-IPA remain unchanged for this single-variable A/B. */
    int cam_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &cam_type) != 0) {
        saved_errno = errno;
        fail_stage = "h264_camera_STREAMON";
        goto h264_fail;
    }
    cam_streaming = true;

    ESP_LOGI(TAG, "H264 pipeline STREAMON OK; warmup=2 target=30 encoded frames");

    for (int w = 0; w < 2; ++w) {
        struct v4l2_buffer warm = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &warm) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_DQBUF_warmup";
            goto h264_fail;
        }
        if (ioctl(cam_fd, VIDIOC_QBUF, &warm) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_QBUF_warmup";
            goto h264_fail;
        }
    }

    const uint32_t target_frames = 30;
    const uint32_t max_attempts = 40;
    uint32_t camera_frames = 0;
    uint32_t encoded_frames = 0;
    uint32_t skipped_frames = 0;
    uint64_t encoded_total = 0;
    uint32_t encoded_min = UINT32_MAX;
    uint32_t encoded_max = 0;
    uint32_t hash_first = 0;
    uint32_t hash_last = 0;
    uint32_t keyflag_count = 0;
    h264_nal_stats_t nal = {0};
    int64_t t_first = 0;
    int64_t t_last = 0;
    uint64_t enc_us_total = 0;
    uint32_t enc_us_min = UINT32_MAX;
    uint32_t enc_us_max = 0;

    for (uint32_t attempt = 0; attempt < max_attempts && encoded_frames < target_frames; ++attempt) {
        struct v4l2_buffer cb = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &cb) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_DQBUF";
            goto h264_fail;
        }
        if (cb.index >= cam_mapped_count || cam_maps[cb.index].start == NULL) {
            saved_errno = EFAULT;
            fail_stage = "h264_camera_bad_index";
            goto h264_fail;
        }
        camera_frames++;
        camera_frame_bytes = cb.bytesused;

        struct v4l2_buffer enc_in = {
            .index = 0,
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_USERPTR,
            .m.userptr = (unsigned long)cam_maps[cb.index].start,
            .length = cb.bytesused,
        };
        int64_t enc_start = esp_timer_get_time();
        if (ioctl(enc_fd, VIDIOC_QBUF, &enc_in) != 0) {
            saved_errno = errno;
            fail_stage = "h264_encoder_QBUF_OUTPUT";
            goto h264_fail;
        }

        struct v4l2_buffer encoded = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(enc_fd, VIDIOC_DQBUF, &encoded) != 0) {
            saved_errno = errno;
            fail_stage = "h264_encoder_DQBUF_CAPTURE";
            goto h264_fail;
        }

        struct v4l2_buffer enc_done = {
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_USERPTR,
        };
        if (ioctl(enc_fd, VIDIOC_DQBUF, &enc_done) != 0) {
            saved_errno = errno;
            fail_stage = "h264_encoder_DQBUF_OUTPUT";
            goto h264_fail;
        }
        int64_t enc_end = esp_timer_get_time();
        uint32_t enc_us = (enc_end > enc_start) ? (uint32_t)(enc_end - enc_start) : 0;

        bool valid = (encoded.flags & V4L2_BUF_FLAG_DONE) && encoded.bytesused > 0 &&
                     encoded.index == 0 && encoded.bytesused <= enc_map_len;
        if (valid) {
            uint32_t used = encoded.bytesused;
            uint32_t hash = sparse_fnv1a((const uint8_t *)enc_map, used);
            if (encoded_frames == 0) {
                hash_first = hash;
                t_first = enc_end;
            }
            hash_last = hash;
            t_last = enc_end;
            encoded_frames++;
            encoded_total += used;
            if (used < encoded_min) encoded_min = used;
            if (used > encoded_max) encoded_max = used;
            enc_us_total += enc_us;
            if (enc_us < enc_us_min) enc_us_min = enc_us;
            if (enc_us > enc_us_max) enc_us_max = enc_us;
            if (encoded.flags & V4L2_BUF_FLAG_KEYFRAME) keyflag_count++;
            h264_scan_annexb((const uint8_t *)enc_map, used, &nal);

            ESP_LOGI(TAG,
                     "H264 frame=%u cam_idx=%u raw=%u h264=%u enc_us=%u flags=0x%08x hash=%08" PRIx32,
                     (unsigned)encoded_frames, (unsigned)cb.index, (unsigned)cb.bytesused,
                     (unsigned)used, (unsigned)enc_us, (unsigned)encoded.flags, hash);
        } else {
            skipped_frames++;
            ESP_LOGW(TAG, "H264 encoded frame skipped flags=0x%08x bytes=%u idx=%u",
                     (unsigned)encoded.flags, (unsigned)encoded.bytesused, (unsigned)encoded.index);
        }

        encoded.index = 0;
        encoded.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        encoded.memory = V4L2_MEMORY_MMAP;
        if (ioctl(enc_fd, VIDIOC_QBUF, &encoded) != 0) {
            saved_errno = errno;
            fail_stage = "h264_encoder_QBUF_CAPTURE_requeue";
            goto h264_fail;
        }
        if (ioctl(cam_fd, VIDIOC_QBUF, &cb) != 0) {
            saved_errno = errno;
            fail_stage = "h264_camera_QBUF_requeue";
            goto h264_fail;
        }
    }

    if (encoded_frames == 0) {
        saved_errno = EIO;
        fail_stage = "h264_no_encoded_frames";
        goto h264_fail;
    }

    if (cam_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
        cam_streaming = false;
    }
    if (enc_out_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
        enc_out_streaming = false;
    }
    if (enc_cap_streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
        enc_cap_streaming = false;
    }

    if (enc_map) {
        munmap(enc_map, enc_map_len);
        enc_map = NULL;
    }
    for (uint32_t i = 0; i < cam_mapped_count; ++i) {
        if (cam_maps[i].start) {
            munmap(cam_maps[i].start, cam_maps[i].length);
            cam_maps[i].start = NULL;
        }
    }
    close(enc_fd);
    enc_fd = -1;
    close(cam_fd);
    cam_fd = -1;

    uint32_t fps_x100 = 0;
    if (encoded_frames > 1 && t_last > t_first) {
        fps_x100 = (uint32_t)(((uint64_t)(encoded_frames - 1) * 100000000ULL) /
                              (uint64_t)(t_last - t_first));
    }
    uint32_t enc_avg = encoded_frames ? (uint32_t)(enc_us_total / encoded_frames) : 0;
    uint32_t bytes_avg = encoded_frames ? (uint32_t)(encoded_total / encoded_frames) : 0;

    const size_t dma_free_after = dma_free();
    const size_t dma_largest_after = dma_largest();
    const size_t psram_after = psram_free();

    ESP_LOGI(TAG,
             "H264 PASS frames=%u fps=%u.%02u bytes avg/min/max=%u/%u/%u enc_us avg/min/max=%u/%u/%u "
             "NAL{sps=%u pps=%u idr=%u p=%u} keyflags=%u dma_largest=%u->%u",
             (unsigned)encoded_frames, (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
             (unsigned)bytes_avg, (unsigned)encoded_min, (unsigned)encoded_max,
             (unsigned)enc_avg, (unsigned)enc_us_min, (unsigned)enc_us_max,
             (unsigned)nal.sps, (unsigned)nal.pps, (unsigned)nal.idr, (unsigned)nal.non_idr,
             (unsigned)keyflag_count, (unsigned)dma_largest_before, (unsigned)dma_largest_after);
    ESP_LOGW(TAG, "H264 one-shot consumed; reboot before any other camera stream test");

    char sensor_name[160];
    snprintf(sensor_name, sizeof(sensor_name), "%ux%u RAW10 %u-lane %ufps",
             (unsigned)sensor_fmt.width, (unsigned)sensor_fmt.height,
             (unsigned)sensor_fmt.mipi_info.lane_num, (unsigned)sensor_fmt.fps);

    char body[1500];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"sensor\":\"OV02C10\",\"chip_id\":\"0x5602\"," 
             "\"camera\":\"%s\",\"encoder\":\"/dev/video11\"," 
             "\"sensor_input\":\"%s\","
             "\"pipeline\":\"MIPI-CSI/ISP YU12 -> HW H264\"," 
             "\"actual\":{\"width\":%u,\"height\":%u,\"input_fourcc\":\"YU12\",\"output_fourcc\":\"H264\"},"
             "\"camera_frames\":%u,\"camera_frame_bytes\":%u,\"encoded_frames\":%u,\"skipped\":%u,"
             "\"fps\":\"%u.%02u\",\"h264_bytes\":{\"avg\":%u,\"min\":%u,\"max\":%u},"
             "\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},"
             "\"nal\":{\"sps\":%u,\"pps\":%u,\"idr\":%u,\"non_idr\":%u,\"keyflag\":%u},"
             "\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\","
             "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
             "\"psram\":{\"free_before\":%u,\"free_after\":%u},\"reboot_before_retest\":true}",
             CAM_CSI_DEVICE, sensor_name,
             (unsigned)width, (unsigned)height,
             (unsigned)camera_frames, (unsigned)camera_frame_bytes,
             (unsigned)encoded_frames, (unsigned)skipped_frames,
             (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
             (unsigned)bytes_avg, (unsigned)encoded_min, (unsigned)encoded_max,
             (unsigned)enc_avg, (unsigned)enc_us_min, (unsigned)enc_us_max,
             (unsigned)nal.sps, (unsigned)nal.pps, (unsigned)nal.idr, (unsigned)nal.non_idr,
             (unsigned)keyflag_count, hash_first, hash_last,
             (unsigned)dma_free_before, (unsigned)dma_largest_before,
             (unsigned)dma_free_after, (unsigned)dma_largest_after,
             (unsigned)psram_before, (unsigned)psram_after);

    s_state.running = false;
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);

h264_fail:
    ESP_LOGE(TAG, "H264 FAIL stage=%s errno=%d (%s)",
             fail_stage ? fail_stage : "unknown", saved_errno, strerror(saved_errno));
    if (cam_streaming && cam_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc_out_streaming && enc_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc_cap_streaming && enc_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc_map) {
        munmap(enc_map, enc_map_len);
    }
    for (uint32_t i = 0; i < cam_mapped_count; ++i) {
        if (cam_maps[i].start) {
            munmap(cam_maps[i].start, cam_maps[i].length);
        }
    }
    if (enc_fd >= 0) close(enc_fd);
    if (cam_fd >= 0) close(cam_fd);
    s_state.running = false;
    ESP_LOGW(TAG, "H264 diagnostic failed after hardware touch; reboot before retest");
    return send_json_error(req, "500 Internal Server Error",
                           fail_stage ? fail_stage : "h264_unknown", saved_errno, ESP_FAIL);
}


/* ========================================================================== */
/* Continuous camera -> HW H.264 live stream                                  */
/* ========================================================================== */

typedef struct {
    volatile bool              running;
    volatile bool              stop_requested;
    TaskHandle_t               task;
    SemaphoreHandle_t          done;
    camera_h264_frame_cb_t     cb;
    void                      *cb_ctx;
    /* [CAM-1080-LIVE-GOP-R1-20260913]
     * Cross-task request/ack only; the actual H264 parameter write is executed
     * by the camera worker between frames so encoder/DMA ownership never moves. */
    uint32_t                   gop_request;
    uint32_t                   gop_request_seq;
    uint32_t                   gop_applied_seq;
    esp_err_t                  gop_apply_result;
    camera_h264_stream_stats_t stats;
} camera_live_state_t;

static camera_live_state_t s_live;
static portMUX_TYPE s_live_mux = portMUX_INITIALIZER_UNLOCKED;

/* [CAM-DMA-STAGED-PREWARM V3 2026-08-24]
 * V2.1 proved two things on hardware:
 *   1) the DMA anchor itself works (freeing it restored an exact contiguous
 *      block), and
 *   2) holding a 144 KiB anchor while starting the full 1288x728 H.264 path
 *      leaves only ~94 KiB largest and the encoder cannot allocate its
 *      reference/parameter working set.
 *
 * V3 therefore keeps the anchor but removes the large 720p H.264 working set
 * from boot prewarm.  While the anchor is protected we first exercise the
 * exact /dev/video11 V4L2 H.264 CAPTURE-STREAMON path at a tiny 320x240
 * resolution (enough to create the H.264 HW lazy interrupt/lock objects), then
 * exercise only the native-720p camera/CSI/ISP capture path without H.264
 * (enough to create CSI/ISP controller mutexes + ISR descriptors).  All large
 * transient objects are then released, esp_video is deinitialized, and only
 * after that is the anchor freed.
 *
 * The prewarm reservation is deliberately independent from the runtime 1080p
 * guard.  V3 uses 156 KiB (159744 B): cold 1080p success was observed around
 * 152-156 KiB, while staged prewarm is small enough to run in the remainder.
 */
#define CAM_DMA_PREWARM_ANCHOR_BYTES   (156U * 1024U)
#define CAM_DMA_PREWARM_H264_WIDTH     320U
#define CAM_DMA_PREWARM_H264_HEIGHT    240U
#define CAM_DMA_PREWARM_CAMERA_FRAMES  3U

/* [CAM-DMA-LATE-ISR-STABILIZE V4 2026-08-24]
 * Boot staged-prewarm V3 successfully preserved a 156 KiB contiguous block,
 * but the first real 720p session (after Wi-Fi/SDIO/MQTT were up) still created
 * two new 16-byte intr_alloc vector descriptors: H.264 enc_open + ISP ISR.
 * Those two allocations are enough to split the protected region and collapse
 * the next 1080p largest block to ~100 KiB.  The reason is timing: boot prewarm
 * runs before the final interrupt-vector landscape exists.
 *
 * V4 therefore performs one tiny staged ISR warm-up again at the beginning of
 * the FIRST REAL camera task, i.e. after ICE/DTLS/network are already live but
 * immediately before the requested full-resolution pipeline.  A 152..156 KiB
 * anchor protects the 1080p-sized hole while 320x240 H.264 + camera/ISP briefly
 * acquire/release their final vector assignments.  The anchor is then released
 * and the requested 720p/1080p stream starts normally.  If this late warm-up
 * cannot run, it is diagnostic-only/fail-open: the real stream is still tried.
 */
#define CAM_DMA_LATE_ISR_ANCHOR_MAX_BYTES   (156U * 1024U)
#define CAM_DMA_LATE_ISR_ANCHOR_MIN_BYTES   (152U * 1024U)
#define CAM_DMA_LATE_ISR_MIN_REMAINDER      (48U * 1024U)
static bool s_dma_prewarm_attempted;
static bool s_dma_prewarm_active;
static bool s_dma_late_isr_attempted;
static esp_err_t camera_dma_late_isr_stabilize_once(void);

/* [REF-POOL 2026-08-24]
 * 720p->1080p no longer uses esp_restart(). The H.264 reference-frame
 * allocation is supplied by a persistent boot-time pool in esp_h264. */

esp_err_t camera_h264_stream_set_profile(camera_h264_profile_t profile)
{
    if (profile != CAMERA_H264_PROFILE_1080P &&
        profile != CAMERA_H264_PROFILE_720P_NATIVE) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_live_mux);
    if (s_live.running) {
        taskEXIT_CRITICAL(&s_live_mux);
        return ESP_ERR_INVALID_STATE;
    }
    s_live_profile_selected = profile;
    /* [CAM-1080-LIVE-GOP-R1-20260913] Every fresh 1080 session starts from
     * the safe GOP5 baseline. Adaptive GOP changes are applied in-place to the
     * already-open encoder and therefore never call set_profile(). */
    if (profile == CAMERA_H264_PROFILE_1080P) s_live_1080_gop_selected = 5U;
    taskEXIT_CRITICAL(&s_live_mux);

    ESP_LOGI(TAG, "LIVE next profile=%s", camera_h264_stream_profile_name(profile));
    return ESP_OK;
}

camera_h264_profile_t camera_h264_stream_get_profile(void)
{
    camera_h264_profile_t profile;
    taskENTER_CRITICAL(&s_live_mux);
    profile = s_live_profile_selected;
    taskEXIT_CRITICAL(&s_live_mux);
    return profile;
}

/* [CAM-1080-LIVE-GOP-R1-20260913]
 * 1080 GOP changes are now in-place. While stopped this only selects the GOP
 * for the next open. While running it posts a request to the camera worker;
 * that worker updates the already-open HW encoder between frames and ACKs it.
 * No camera task, V4L2 queue, H264 object, WEB20 pool, or DMA block is recreated. */
esp_err_t camera_h264_stream_set_gop(uint32_t gop)
{
    if (gop != 1U && gop != 5U) return ESP_ERR_INVALID_ARG;

    uint32_t request_seq = 0;
    taskENTER_CRITICAL(&s_live_mux);
    if (s_live_profile_selected != CAMERA_H264_PROFILE_1080P) {
        taskEXIT_CRITICAL(&s_live_mux);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_live.running) {
        s_live_1080_gop_selected = gop;
        taskEXIT_CRITICAL(&s_live_mux);
        ESP_LOGI(TAG, "[CAM-1080-LIVE-GOP-R1-20260913] next 1080 GOP=%u (stopped)",
                 (unsigned)gop);
        return ESP_OK;
    }

    if (s_live_1080_gop_selected == gop &&
        s_live.gop_request_seq == s_live.gop_applied_seq) {
        taskEXIT_CRITICAL(&s_live_mux);
        return ESP_OK;
    }

    request_seq = s_live.gop_request_seq + 1U;
    if (request_seq == 0U) request_seq = 1U;
    s_live.gop_request = gop;
    s_live.gop_request_seq = request_seq;
    s_live.gop_apply_result = ESP_ERR_INVALID_STATE;
    taskEXIT_CRITICAL(&s_live_mux);

    const int64_t deadline_us = esp_timer_get_time() +
                                (int64_t)CAM_LIVE_GOP_APPLY_TIMEOUT_MS * 1000LL;
    TickType_t poll_ticks = pdMS_TO_TICKS(5U);
    if (poll_ticks == 0) poll_ticks = 1;

    while (esp_timer_get_time() < deadline_us) {
        uint32_t applied_seq;
        esp_err_t result;
        bool running;
        taskENTER_CRITICAL(&s_live_mux);
        applied_seq = s_live.gop_applied_seq;
        result = s_live.gop_apply_result;
        running = s_live.running;
        taskEXIT_CRITICAL(&s_live_mux);

        if (applied_seq == request_seq) {
            return result;
        }
        if (!running) {
            return ESP_ERR_INVALID_STATE;
        }
        vTaskDelay(poll_ticks);
    }

    ESP_LOGE(TAG,
             "[CAM-1080-LIVE-GOP-R1-20260913] apply timeout target=%u seq=%u",
             (unsigned)gop, (unsigned)request_seq);
    return ESP_ERR_TIMEOUT;
}

uint32_t camera_h264_stream_get_gop(void)
{
    uint32_t gop;
    taskENTER_CRITICAL(&s_live_mux);
    gop = (s_live_profile_selected == CAMERA_H264_PROFILE_1080P)
        ? s_live_1080_gop_selected : 5U;
    taskEXIT_CRITICAL(&s_live_mux);
    return gop;
}

bool camera_h264_stream_is_active(void)
{
    bool running;
    taskENTER_CRITICAL(&s_live_mux);
    running = s_live.running;
    taskEXIT_CRITICAL(&s_live_mux);
    return running;
}

void camera_h264_stream_get_stats(camera_h264_stream_stats_t *out)
{
    if (!out) return;
    taskENTER_CRITICAL(&s_live_mux);
    *out = s_live.stats;
    taskEXIT_CRITICAL(&s_live_mux);
}

void camera_h264_stream_request_stop(void)
{
    taskENTER_CRITICAL(&s_live_mux);
    s_live.stop_requested = true;
    taskEXIT_CRITICAL(&s_live_mux);
}

static bool live_stop_requested(void)
{
    bool stop;
    taskENTER_CRITICAL(&s_live_mux);
    stop = s_live.stop_requested;
    taskEXIT_CRITICAL(&s_live_mux);
    return stop;
}

/* Run only on camera_h264_live_task, between encoded frames.
 * This is the key zero-fragmentation path: only one byte-sized GOP parameter
 * changes. The existing encoder, task stack, V4L2 queues, persistent 138KB ref,
 * and WEB20 20KB transient pool all stay exactly where they are. */
static void live_apply_pending_gop_request(void)
{
    uint32_t request = 0;
    uint32_t request_seq = 0;
    uint32_t applied_seq = 0;
    uint32_t old_gop = 0;

    taskENTER_CRITICAL(&s_live_mux);
    request_seq = s_live.gop_request_seq;
    applied_seq = s_live.gop_applied_seq;
    request = s_live.gop_request;
    old_gop = s_live_1080_gop_selected;
    taskEXIT_CRITICAL(&s_live_mux);

    if (request_seq == 0U || request_seq == applied_seq) return;

    const size_t dma_before_free = dma_free();
    const size_t dma_before_largest = dma_largest();
    esp_h264_err_t hr = esp_h264_enc_hw_active_set_gop((uint8_t)request);
    const esp_err_t result = (hr == ESP_H264_ERR_OK) ? ESP_OK : ESP_FAIL;
    const size_t dma_after_free = dma_free();
    const size_t dma_after_largest = dma_largest();

    taskENTER_CRITICAL(&s_live_mux);
    if (result == ESP_OK) {
        s_live_1080_gop_selected = request;
        s_live.stats.h264_gop = request;
    }
    s_live.gop_apply_result = result;
    s_live.gop_applied_seq = request_seq;
    taskEXIT_CRITICAL(&s_live_mux);

    ESP_LOGW(TAG,
             "[CAM-1080-LIVE-GOP-R1-20260913] worker apply GOP %u -> %u result=%s "
             "dma=%u/%u -> %u/%u (no restart/no alloc)",
             (unsigned)old_gop, (unsigned)request, esp_err_to_name(result),
             (unsigned)dma_before_free, (unsigned)dma_before_largest,
             (unsigned)dma_after_free, (unsigned)dma_after_largest);
}

static void camera_streamparm_30fps_best_effort(int fd, uint32_t sensor_fps)
{
    struct v4l2_streamparm before = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    if (ioctl(fd, VIDIOC_G_PARM, &before) == 0) {
        const struct v4l2_fract *t = &before.parm.capture.timeperframe;
        uint32_t fps_x100 = (t->numerator && t->denominator)
            ? (uint32_t)(((uint64_t)t->denominator * 100ULL) / t->numerator) : 0;
        ESP_LOGI(TAG, "LIVE camera G_PARM before=%u.%02u fps cap=0x%08x",
                 (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
                 (unsigned)before.parm.capture.capability);
    } else {
        ESP_LOGW(TAG, "LIVE camera G_PARM before rejected errno=%d (%s)", errno, strerror(errno));
    }

    uint32_t req_fps = sensor_fps ? sensor_fps : 30U;
    struct v4l2_streamparm setp = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    setp.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    setp.parm.capture.timeperframe.numerator = 1;
    setp.parm.capture.timeperframe.denominator = req_fps;
    if (ioctl(fd, VIDIOC_S_PARM, &setp) != 0) {
        ESP_LOGW(TAG, "LIVE camera S_PARM %ufps rejected errno=%d (%s); keep driver timing",
                 (unsigned)req_fps, errno, strerror(errno));
    } else {
        ESP_LOGI(TAG, "LIVE camera S_PARM requested=%ufps accepted", (unsigned)req_fps);
    }

    struct v4l2_streamparm after = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    if (ioctl(fd, VIDIOC_G_PARM, &after) == 0) {
        const struct v4l2_fract *t = &after.parm.capture.timeperframe;
        uint32_t fps_x100 = (t->numerator && t->denominator)
            ? (uint32_t)(((uint64_t)t->denominator * 100ULL) / t->numerator) : 0;
        ESP_LOGI(TAG, "LIVE camera G_PARM after=%u.%02u fps",
                 (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100));
    }
}

/* [WEBRTC-REQBUFS-RELEASE 2026-08-24]
 * esp_video keeps the video subsystem initialized across profile switches, but
 * each V4L2 queue must still return its per-stream buffers explicitly.  Merely
 * STREAMOFF + close can leave the internal DMA heap badly fragmented after the
 * high-rate 720p60 path.  1080p H.264 then fails in esp_h264_enc_hw_new() with
 * "No memory for reference frame" even though total DMA free is high.
 *
 * VIDIOC_REQBUFS(count=0) is the V4L2 operation for releasing a queue's
 * allocated buffers.  Do it after STREAMOFF and after userspace munmap, while
 * the fd is still open, so the allocator gets the best chance to coalesce the
 * returned internal blocks before the next profile starts.
 */
static void live_reqbufs_release_best_effort(int fd, int type, int memory, const char *name)
{
    if (fd < 0) return;
    struct v4l2_requestbuffers rb = {
        .count = 0,
        .type = type,
        .memory = memory,
    };
    errno = 0;
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) != 0) {
        ESP_LOGW(TAG, "[REQBUFS-RELEASE] %s count=0 failed errno=%d (%s)",
                 name ? name : "?", errno, strerror(errno));
    } else {
        ESP_LOGI(TAG, "[REQBUFS-RELEASE] %s count=0 OK", name ? name : "?");
    }
}


/* [C12M-PRODUCTION-LOG-QUIET 2026-08-27]
 * The old DMA heap trace was a one-shot diagnostic for profile-switch heap
 * investigation. Keep its implementation available, but disable it in normal
 * production so it neither traces allocations nor emits transition MARK logs. */
#define CAM_DMA_HEAP_TRACE_DIAG_ENABLE 0

/* [DMA-HEAP-TRACE-DIAG 2026-08-24]
 * One-shot diagnostic for the 720p60 -> 1080p contiguous INTERNAL heap loss.
 *
 * Trace starts immediately before the first 720p60 esp_video/camera setup,
 * after WebRTC/DTLS has already allocated its session objects.  On Stop we
 * pause new allocation recording but keep tracking frees, dump the INTERNAL
 * allocations that survived the complete V4L2 cleanup, and keep the trace
 * paused.  If the next 1080p start invokes esp_video_deinit(), we then stop
 * and dump the SAME trace again.  The difference between the two dumps tells
 * us exactly which allocations esp_video_deinit freed and which allocations
 * from the camera lifetime are still alive.
 *
 * The trace record buffer is deliberately allocated from PSRAM so this
 * diagnostic does not itself consume/fragment the scarce INTERNAL DMA heap.
 * IDF 5.5 allows an external-RAM standalone trace buffer; allocations made
 * from ISR context are not recorded in that case, which is acceptable here
 * because the suspected esp_video/ISP/H264 lifetime allocations occur in task
 * context.
 */
#if CONFIG_HEAP_TRACING_STANDALONE && CAM_DMA_HEAP_TRACE_DIAG_ENABLE
#define CAM_DMA_TRACE_RECORDS 1024U
static heap_trace_record_t *s_cam_dma_trace_records;
static bool s_cam_dma_trace_initialized;
static bool s_cam_dma_trace_active;
static bool s_cam_dma_trace_paused;
static bool s_cam_dma_trace_stop_dump_done;
static bool s_cam_dma_trace_final_dump_done;

static void cam_dma_trace_mark(const char *stage)
{
    /* Boot prewarm deliberately does not start heap tracing, but we still need
     * stage-by-stage contiguous-DMA telemetry while the anchor is held. */
    if (s_dma_prewarm_active) {
        ESP_LOGW(TAG,
                 "[CAM-DMA-ANCHOR] PREWARM MARK %-24s dma=%u/%u",
                 stage ? stage : "?",
                 (unsigned)dma_free(), (unsigned)dma_largest());
        return;
    }
    if (!s_cam_dma_trace_active && !s_cam_dma_trace_paused) return;
    ESP_LOGW(TAG,
             "[DMA-HEAP-TRACE] MARK %-28s ccount=0x%08" PRIx32 " records=%u dma=%u/%u",
             stage ? stage : "?", esp_cpu_get_cycle_count(),
             (unsigned)heap_trace_get_count(),
             (unsigned)dma_free(), (unsigned)dma_largest());
}

static void cam_dma_trace_start_720_once(camera_h264_profile_t profile_id)
{
    if (s_dma_prewarm_active) return;
    if (profile_id != CAMERA_H264_PROFILE_720P_NATIVE ||
        s_cam_dma_trace_active || s_cam_dma_trace_paused ||
        s_cam_dma_trace_final_dump_done) {
        return;
    }

    if (!s_cam_dma_trace_records) {
        const size_t bytes = sizeof(heap_trace_record_t) * CAM_DMA_TRACE_RECORDS;
        s_cam_dma_trace_records = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_cam_dma_trace_records) {
            ESP_LOGE(TAG, "[DMA-HEAP-TRACE] PSRAM record buffer alloc failed bytes=%u",
                     (unsigned)bytes);
            return;
        }
        ESP_LOGW(TAG, "[DMA-HEAP-TRACE] record buffer=%p records=%u bytes=%u PSRAM",
                 s_cam_dma_trace_records, (unsigned)CAM_DMA_TRACE_RECORDS, (unsigned)bytes);
    }

    if (!s_cam_dma_trace_initialized) {
        esp_err_t r = heap_trace_init_standalone(s_cam_dma_trace_records, CAM_DMA_TRACE_RECORDS);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "[DMA-HEAP-TRACE] heap_trace_init_standalone failed: %s",
                     esp_err_to_name(r));
            return;
        }
        s_cam_dma_trace_initialized = true;
    }

    esp_err_t r = heap_trace_start(HEAP_TRACE_LEAKS);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "[DMA-HEAP-TRACE] heap_trace_start failed: %s", esp_err_to_name(r));
        return;
    }
    s_cam_dma_trace_active = true;
    s_cam_dma_trace_paused = false;
    s_cam_dma_trace_stop_dump_done = false;
    ESP_LOGW(TAG,
             "[DMA-HEAP-TRACE] START before first 720p60 esp_video/camera setup dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());
    cam_dma_trace_mark("trace_start");
}

static void cam_dma_trace_pause_for_720_cleanup(camera_h264_profile_t profile_id)
{
    if (profile_id != CAMERA_H264_PROFILE_720P_NATIVE || !s_cam_dma_trace_active) {
        return;
    }
    esp_err_t r = heap_trace_alloc_pause();
    if (r == ESP_OK) {
        s_cam_dma_trace_active = false;
        s_cam_dma_trace_paused = true;
        ESP_LOGW(TAG,
                 "[DMA-HEAP-TRACE] PAUSE-ALLOC before 720p cleanup records=%u dma=%u/%u; frees remain tracked",
                 (unsigned)heap_trace_get_count(), (unsigned)dma_free(), (unsigned)dma_largest());
        cam_dma_trace_mark("pause_before_cleanup");
    } else {
        ESP_LOGE(TAG, "[DMA-HEAP-TRACE] heap_trace_alloc_pause failed: %s", esp_err_to_name(r));
    }
}

static void cam_dma_trace_dump_after_720_cleanup(void)
{
    if (!s_cam_dma_trace_paused || s_cam_dma_trace_stop_dump_done) return;

    /* WDT-safe diagnostic: do NOT print the full post-720 dump here.
     * heap_trace_dump_caps() can synchronously emit hundreds of records over
     * the 115200-baud console and was observed to trip the CPU0 interrupt WDT.
     * Keep the trace paused instead: allocations are frozen, while later frees
     * (esp_peer close + esp_video_deinit) continue removing records.  The much
     * smaller DUMP-B after esp_video_deinit is the one we actually need. */
    ESP_LOGW(TAG,
             "[DMA-HEAP-TRACE] SKIP DUMP-A (WDT-safe): records=%u dma=%u/%u; trace remains paused and frees stay tracked",
             (unsigned)heap_trace_get_count(), (unsigned)dma_free(), (unsigned)dma_largest());
    s_cam_dma_trace_stop_dump_done = true;
}

static void cam_dma_trace_finish_after_video_deinit(const char *reason)
{
    if (!s_cam_dma_trace_paused || s_cam_dma_trace_final_dump_done) return;
    esp_err_t r = heap_trace_stop();
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "[DMA-HEAP-TRACE] heap_trace_stop failed: %s", esp_err_to_name(r));
        return;
    }
    s_cam_dma_trace_paused = false;
    s_cam_dma_trace_final_dump_done = true;
    const size_t final_records = heap_trace_get_count();
    ESP_LOGW(TAG,
             "[DMA-HEAP-TRACE] ===== DUMP-B ONLY: surviving INTERNAL allocations after %s ===== records=%u dma=%u/%u",
             reason ? reason : "esp_video_deinit", (unsigned)final_records,
             (unsigned)dma_free(), (unsigned)dma_largest());
    heap_trace_dump_caps(MALLOC_CAP_INTERNAL);
    ESP_LOGW(TAG, "[DMA-HEAP-TRACE] ===== END DUMP-B =====");
}
#else
static inline void cam_dma_trace_start_720_once(camera_h264_profile_t profile_id)
{
    (void)profile_id;
}
static inline void cam_dma_trace_mark(const char *stage) { (void)stage; }
static inline void cam_dma_trace_pause_for_720_cleanup(camera_h264_profile_t profile_id) { (void)profile_id; }
static inline void cam_dma_trace_dump_after_720_cleanup(void) { }
static inline void cam_dma_trace_finish_after_video_deinit(const char *reason) { (void)reason; }
#endif

static void camera_h264_live_task(void *arg)
{
    (void)arg;
    int cam_fd = -1;
    int enc_fd = -1;
    bool cam_streaming = false;
    bool enc_out_streaming = false;
    bool enc_cap_streaming = false;
    bool lcd_stream_flag = false;
    bool owner_ref_held = false;   /* [OWNER-REF-GUARD 2026-09-11] */
    cam_map_buf_t cam_maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t cam_mapped_count = 0;
    void *enc_map = NULL;
    size_t enc_map_len = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;
    uint32_t width = 0, height = 0, sensor_fps = 0;
    int64_t start_us = 0;
    int64_t first_camera_us = 0;
    int64_t last_camera_us = 0;
    int64_t first_encoded_us = 0;
    int64_t last_encoded_us = 0;
    int64_t last_log_us = 0;
    int64_t fps_window_start_us = 0;
    uint32_t camera_frames = 0;
    uint32_t encoded_frames = 0;
    uint32_t fps_window_camera_base = 0;
    uint32_t fps_window_encoded_base = 0;
    uint32_t camera_fps_recent_x100 = 0;
    uint32_t encode_fps_recent_x100 = 0;
    uint32_t camera_fps_total_x100 = 0;
    uint32_t encode_fps_total_x100 = 0;
    uint32_t delivered_frames = 0;
    uint32_t consumer_drops = 0;
    uint32_t encoder_skips = 0;
    uint32_t enc_us_max = 0;
    uint32_t camera_dq_us_max = 0;
    uint64_t camera_dq_us_sum = 0;
    uint32_t consumer_us_max = 0;
    uint32_t loop_us_max = 0;
    uint32_t h264_bytes_max = 0;
    h264_nal_stats_t first_nal = {0};

    /* [720-COLOR-MATCH-YU12-DIAG-A1] 1s / 3s / 5s post-ISP samples. */
    uint8_t yu12_diag_sample_index = 0;
    static const uint32_t yu12_diag_targets_ms[] = {1000U, 3000U, 5000U};

    camera_h264_profile_t profile_id = camera_h264_stream_get_profile();

    /* [REF-POOL] No late ISR/anchor warm-up and no software reboot.
     * The encoder reuses the persistent boot-time H.264 reference pool. */

    /* [CAM-1080-ADAPT-GOP-R2-20260912] Per-generation copy: only 1080 GOP is runtime-selectable.
     * QP/resolution/ISP/color/fps policy remain exactly as before. */
    const camera_live_profile_cfg_t *profile_base = camera_live_profile_cfg(profile_id);
    camera_live_profile_cfg_t profile_runtime = *profile_base;
    if (profile_id == CAMERA_H264_PROFILE_1080P) {
        profile_runtime.gop = camera_h264_stream_get_gop();
    }
    const camera_live_profile_cfg_t *profile = &profile_runtime;
    /* Production fixed-QP policy:
     *   1080p = QP35 (existing validated production value)
     *   720p  = QP39 (2026-09-13 final matrix result)
     * min_qp == max_qp intentionally keeps the H.264 RC object disabled. */
    const uint32_t live_qp_min = profile->qp;
    const uint32_t live_qp_max = profile->qp;
    const char *live_qp_mode = "fixed";

    const size_t dma0 = dma_free();
    size_t largest0 = dma_largest();  /* mutable: may be re-sampled after esp_video_deinit defrag */
    const size_t ps0 = psram_free();
    ESP_LOGW(TAG, "[DMA-DIAG D3 camera_task_entry] free=%u largest=%u",
             (unsigned)dma0, (unsigned)largest0);
    ESP_LOGI(TAG, "LIVE start profile=%s target=%ux%u@%u lanes=%u GOP=%u QP=%u..%u %s dma_free=%u largest=%u psram=%u core=%d",
             profile->name, (unsigned)profile->width, (unsigned)profile->height,
             (unsigned)profile->fps, (unsigned)profile->lanes, (unsigned)profile->gop,
             (unsigned)live_qp_min, (unsigned)live_qp_max, live_qp_mode,
             (unsigned)dma0, (unsigned)largest0,
             (unsigned)ps0, xPortGetCoreID());

    taskENTER_CRITICAL(&s_live_mux);
    memset(&s_live.stats, 0, sizeof(s_live.stats));
    s_live.stats.running = true;
    s_live.stats.dma_free = (uint32_t)dma0;
    s_live.stats.dma_largest = (uint32_t)largest0;
    s_live.stats.psram_free = (uint32_t)ps0;
    snprintf(s_live.stats.last_stage, sizeof(s_live.stats.last_stage), "starting");
    taskEXIT_CRITICAL(&s_live_mux);

    /* [CAM-WEBRTC-SHIFT-GUARD 2026-08-22]
     * Block LCD random screen shifts for the entire WebRTC camera lifetime,
     * including camera/encoder initialization before STREAMON. A successful
     * full-screen shift can stall H.264 for tens of milliseconds. Cleanup
     * always releases this guard, including all startup failure paths. */
    lcd_ui_set_camera_stream_active(true);
    lcd_stream_flag = true;

    /* [WEBRTC-SWITCH-DMA-GUARD-V2 2026-08-24]
     * The large DMA guard is only meaningful before the first esp_video_init().
     * Once esp_video is initialized, its persistent CSI/ISP allocations remain
     * resident by design, so the largest free block naturally settles lower
     * (observed ~92 KB after a clean 720p60 Stop).  Re-applying the cold-start
     * threshold then falsely blocks a valid 720p60 -> 1080p restart.
     *
     * Keep the 88 KB floor for the first initialization, but on later profile
     * switches only require the generic 32 KB CSI safety floor and let the real
     * V4L2/H.264 allocation calls decide whether more memory is actually needed.
     */
    /* 1080p HW H.264 has a real internal-SRAM contiguous requirement.  The
     * prior 32/88 KB restart guard only hid the problem until STREAMON, where
     * esp_h264 reported "No memory for reference frame" at largest=94 KB.
     * Keep 144 KB as the 1080p requirement.  720p60 uses a smaller HW working
     * set and may legitimately start from the lower restart floor. */
    const size_t live_dma_floor = (profile->width >= 1920U)
                                ? CAM_LIVE_DMA_PREFERRED_LARGEST
                                : (s_state.video_inited
                                   ? CAM_CSI_MIN_DMA_LARGEST
                                   : CAM_LIVE_MIN_DMA_LARGEST);

    /* [REF-POOL] If the persistent reference pool is ready, 1080p no longer
     * needs a new ~138KB param->ref allocation from the residual heap.
     * Therefore bypass the old 147456-byte H.264 contiguous guard and retain
     * only the generic CSI safety floor. If reserve failed, keep the original
     * conservative guard for the stock malloc path. */
    const bool ref_pool_ready = esp_h264_enc_hw_ref_pool_ready();
    const bool ref_pool_external = true; /* [REF-PSRAM-GUARD AB2-LAZY] */
    size_t effective_floor = live_dma_floor;
    if (ref_pool_ready || ref_pool_external) {
        effective_floor = (24U * 1024U);
        ESP_LOGW(TAG,
                 "[SWITCH-DMA-GUARD-REFPOOL] pool ready; %s guard %u -> %u",
                 profile->name, (unsigned)live_dma_floor, (unsigned)effective_floor);
    }

    const size_t dma_d4_free = dma_free();
    const size_t dma_d4_largest = dma_largest();
    ESP_LOGW(TAG,
             "[DMA-DIAG D4 before_guard] free=%u largest=%u guard_snapshot=%u required=%u",
             (unsigned)dma_d4_free, (unsigned)dma_d4_largest,
             (unsigned)largest0, (unsigned)effective_floor);
    ESP_LOGW(TAG,
             "[SWITCH-DMA-GUARD-V3] video_inited=%d largest=%u required=%u base=%u pool=%d profile=%s",
             s_state.video_inited ? 1 : 0,
             (unsigned)largest0, (unsigned)effective_floor,
             (unsigned)live_dma_floor, ref_pool_ready ? 1 : 0, profile->name);

    if (largest0 < effective_floor) {
        /* Fallback only. With a healthy ref pool this path should normally not
         * run for 720p->1080p because the effective floor is only the generic
         * CSI safety floor. If residual DMA is even lower, try esp_video_deinit
         * once and re-sample before failing safely. */
        if (0 && s_state.video_inited) { /* [ISP-LIFECYCLE-A/B-NO-DEINIT] keep esp_video resident */
            ESP_LOGW(TAG,
                     "[SWITCH-DMA-DEFRAG] largest=%u < required=%u for %s; fallback esp_video_deinit",
                     (unsigned)largest0, (unsigned)effective_floor, profile->name);
            const size_t deinit_dma_before = dma_free();
            const size_t deinit_largest_before = dma_largest();
            cam_dma_trace_mark("before_esp_video_deinit");
            esp_err_t deinit_ret = esp_video_deinit();
            if (deinit_ret != ESP_OK) {
                ESP_LOGE(TAG, "[SWITCH-DMA-DEFRAG] esp_video_deinit failed: %s; keep video_inited=1",
                         esp_err_to_name(deinit_ret));
                saved_errno = EIO;
                fail_stage = "live_esp_video_deinit";
                goto live_fail;
            }
            s_state.video_inited = false;
            vTaskDelay(pdMS_TO_TICKS(30));
            const size_t deinit_dma_after = dma_free();
            largest0 = dma_largest();
            ESP_LOGW(TAG,
                     "[SWITCH-DMA-DEFRAG] after esp_video_deinit dma=%u->%u largest=%u->%u",
                     (unsigned)deinit_dma_before, (unsigned)deinit_dma_after,
                     (unsigned)deinit_largest_before, (unsigned)largest0);
            cam_dma_trace_mark("after_esp_video_deinit");
            cam_dma_trace_finish_after_video_deinit("esp_video_deinit");
        }
        if (largest0 < effective_floor) {
            saved_errno = ENOMEM;
            fail_stage = (profile->width >= 1920U)
                       ? "live_1080p_dma_contiguous_guard"
                       : (s_state.video_inited ? "live_restart_dma_guard" : "live_dma_guard");
            ESP_LOGE(TAG,
                     "LIVE %s needs contiguous DMA >=%u but largest=%u; fallback defrag did not recover enough",
                     profile->name, (unsigned)effective_floor, (unsigned)largest0);
            goto live_fail;
        }
    }

    /* Diagnostic trace begins after WebRTC setup but before any 720p esp_video/camera allocation. */
    cam_dma_trace_start_720_once(profile_id);

    if (ov02c10_webrtc_select_profile((uint32_t)profile_id) != ESP_OK) {
        saved_errno = EINVAL;
        fail_stage = "live_select_sensor_profile";
        goto live_fail;
    }

    /* [ISP-LIFECYCLE-C12A 2026-08-27]
     * PROFILE SPLIT, based on the hardware A/B results:
     *
     *   1080p: keep the C10A EARLY full-controller restart.  It is known to
     *          restore/retain correct 1080p colour and is done before the app
     *          opens /dev/video0, so it stays on the safe side of C9A.
     *
     *   720p60: DO NOT restart the controller here.  Earlier C2F-T hardware
     *           runs produced normal 720p while the long-lived controller was
     *           preserved and its four software feedback/dedup fields were
     *           rebased AFTER S_FMT, immediately before STREAMON.
     *
     * This is intentionally asymmetric.  C10A repeatedly reproduced the 720p
     * purple/~4.2KB-IDR state, so applying the same lifecycle to both profiles
     * is no longer a useful experiment. */
    const bool c12a_controller_already_existed = s_state.video_inited;
    const bool c12a_is_1080 = (profile->width >= 1920U);
    if (init_video_once() != ESP_OK) {
        saved_errno = EIO;
        fail_stage = "live_esp_video_init";
        goto live_fail;
    }
    cam_dma_trace_mark("after_esp_video_init");

    if (c12a_is_1080) {
        /* [CAM-FIRST-START-FULL-INIT-AB1-20260907]
         * Make the first 1080p Start use the same safe EARLY full-controller
         * restart that later 1080p Starts already use.  720p policy is unchanged. */
        ESP_LOGW(TAG,
                 "[CAM-FIRST-START-FULL-INIT-AB1] profile=%s controller_preexisted=%d -> full restart",
                 profile->name, c12a_controller_already_existed ? 1 : 0);
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12A] profile=%s path=1080: EARLY full controller restart before app camera open",
                 profile->name);
        esp_err_t c10a_ret = esp_video_isp_pipeline_full_restart_for_ab();
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12A] 1080 EARLY full controller restart ret=%s (0x%x)",
                 esp_err_to_name(c10a_ret), (unsigned)c10a_ret);
        if (c10a_ret != ESP_OK) {
            saved_errno = EIO;
            fail_stage = "live_isp_c12a_1080_full_restart";
            goto live_fail;
        }
    } else {
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12A] profile=%s path=720: KEEP controller; defer four-field resync until post-S_FMT",
                 profile->name);
    }

    ESP_LOGW(TAG, "[DMA-DIAG D10 after_isp_lifecycle_stage] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    cam_fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (cam_fd < 0) {
        saved_errno = errno;
        fail_stage = "live_open_video0";
        goto live_fail;
    }

    /* [OWNER-REF-GUARD 2026-09-11] esp_video keeps ONE reference-counted device
     * object for /dev/video0, shared by this cam_fd and the ISP pipeline's own
     * fd. The ISP's init_cam_dev() does open()+VIDIOC_SET_OWNER(0) and later
     * close() in deinit -> a net -1 on the shared reference every ISP restart
     * cycle. That stray -1 can drive esp_video reference to 0 while we are still
     * streaming, so esp_video_close() deinits and frees our live capture buffer
     * (-> next DQBUF fails EPERM "Not owner"); the unlocked REQBUFS(0) release in
     * cleanup then destroys the same, already-freed buffer -> tlsf double-free.
     * Hold one extra reference for the whole session so the ISP's -1 can never
     * reach 0 under us. Released by a matching SET_OWNER(0) before close below. */
    {
        int own = 1;
        if (ioctl(cam_fd, VIDIOC_SET_OWNER, &own) == 0) {
            owner_ref_held = true;
        } else {
            ESP_LOGW(TAG, "[OWNER-REF-GUARD] SET_OWNER(+1) errno=%d (%s); continuing",
                     errno, strerror(errno));
        }
    }

    /* esp_video_init() is intentionally retained across clean stream stops in
     * this project. Re-apply the selected sensor format explicitly before any
     * buffer allocation/STREAMON so 1080p <-> native720p60 switching works without
     * rebooting or reinitializing unrelated subsystems. */
    const esp_cam_sensor_format_t *requested_sensor_fmt =
        ov02c10_webrtc_get_profile_format((uint32_t)profile_id);
    if (!requested_sensor_fmt) {
        saved_errno = EINVAL;
        fail_stage = "live_get_sensor_profile";
        goto live_fail;
    }
    if (ioctl(cam_fd, VIDIOC_S_SENSOR_FMT, requested_sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "live_S_SENSOR_FMT_profile";
        ESP_LOGE(TAG, "LIVE set sensor profile=%s failed errno=%d (%s)",
                 profile->name, errno, strerror(errno));
        goto live_fail;
    }
    ESP_LOGI(TAG, "LIVE sensor profile applied=%s requested=%s",
             profile->name, requested_sensor_fmt->name ? requested_sensor_fmt->name : "(null)");

    esp_cam_sensor_format_t sensor_fmt = {0};
    if (ioctl(cam_fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "live_G_SENSOR_FMT";
        goto live_fail;
    }
    width = sensor_fmt.width;
    height = sensor_fmt.height;
    sensor_fps = sensor_fmt.fps;
    if (!width || !height) {
        saved_errno = EINVAL;
        fail_stage = "live_sensor_fmt_invalid";
        goto live_fail;
    }
    ESP_LOGI(TAG, "LIVE sensor=%s %ux%u RAW format=%d %u-lane nominal=%ufps",
             sensor_fmt.name ? sensor_fmt.name : "(null)",
             (unsigned)width, (unsigned)height, (int)sensor_fmt.format,
             (unsigned)sensor_fmt.mipi_info.lane_num, (unsigned)sensor_fps);

    /* [C12L-PRODUCTION-DIAG-CLEANUP 2026-08-27]
     * Keep the cheap one-shot build/sensor metadata logs, but do not open a
     * second /dev/video20 fd or issue read-only G_CTRL inventory/runtime probes.
     * Those probes were diagnostic-only and are not part of the ISP fix. */
    camera_color_diag_log_build_config();
    camera_color_diag_log_sensor_isp_info(&sensor_fmt);

    if (width != profile->width || height != profile->height ||
        sensor_fps != profile->fps ||
        sensor_fmt.mipi_info.lane_num != profile->lanes) {
        saved_errno = EINVAL;
        fail_stage = "live_sensor_profile_mismatch";
        ESP_LOGE(TAG,
                 "LIVE profile=%s requires %ux%u@%u %u-lane; active sensor is %ux%u@%u %u-lane",
                 profile->name, (unsigned)profile->width, (unsigned)profile->height,
                 (unsigned)profile->fps, (unsigned)profile->lanes,
                 (unsigned)width, (unsigned)height, (unsigned)sensor_fps,
                 (unsigned)sensor_fmt.mipi_info.lane_num);
        goto live_fail;
    }

    struct v4l2_format cam_fmt = {0};
    cam_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cam_fmt.fmt.pix.width = width;
    cam_fmt.fmt.pix.height = height;
    cam_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    cam_fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(cam_fd, VIDIOC_S_FMT, &cam_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "live_camera_S_FMT_YU12";
        goto live_fail;
    }
    width = cam_fmt.fmt.pix.width;
    height = cam_fmt.fmt.pix.height;
    if (cam_fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        saved_errno = EINVAL;
        fail_stage = "live_camera_not_YU12";
        goto live_fail;
    }
    if (width != profile->width || height != profile->height) {
        saved_errno = EINVAL;
        fail_stage = "live_camera_profile_size_mismatch";
        ESP_LOGE(TAG, "LIVE camera S_FMT returned %ux%u, expected %ux%u for %s",
                 (unsigned)width, (unsigned)height,
                 (unsigned)profile->width, (unsigned)profile->height, profile->name);
        goto live_fail;
    }
    ESP_LOGI(TAG, "LIVE camera=%ux%u YU12 profile=%s",
             (unsigned)width, (unsigned)height, profile->name);
    camera_streamparm_30fps_best_effort(cam_fd, profile->fps);

    struct timeval timeout = { .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC, .tv_usec = 0 };
    if (ioctl(cam_fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "LIVE camera DQ timeout ioctl rejected errno=%d", errno);
    }

    struct v4l2_requestbuffers cam_req = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &cam_req) != 0 || cam_req.count < 2) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "live_camera_REQBUFS";
        goto live_fail;
    }
    if (cam_req.count > CAM_CSI_BUFFER_COUNT) cam_req.count = CAM_CSI_BUFFER_COUNT;
    ESP_LOGI(TAG, "LIVE camera buffers requested=%u actual=%u",
             (unsigned)CAM_CSI_BUFFER_COUNT, (unsigned)cam_req.count);
    for (uint32_t i = 0; i < cam_req.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "live_camera_QUERYBUF";
            goto live_fail;
        }
        cam_maps[i].length = b.length;
        cam_maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, cam_fd, b.m.offset);
        if (!cam_maps[i].start || cam_maps[i].start == MAP_FAILED) {
            cam_maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "live_camera_mmap";
            goto live_fail;
        }
        cam_mapped_count++;
        if (ioctl(cam_fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "live_camera_QBUF_initial";
            goto live_fail;
        }
    }
    cam_dma_trace_mark("after_camera_buffers");
    ESP_LOGW(TAG, "[DMA-DIAG D11 after_camera_buffers_ready] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    enc_fd = open("/dev/video11", O_RDWR);
    if (enc_fd < 0) {
        saved_errno = errno;
        fail_stage = "live_open_video11";
        goto live_fail;
    }
    struct v4l2_capability enc_capability = {0};
    if (ioctl(enc_fd, VIDIOC_QUERYCAP, &enc_capability) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_QUERYCAP";
        goto live_fail;
    }
    ESP_LOGI(TAG, "LIVE encoder=%s bus=%s", enc_capability.driver, enc_capability.bus_info);
    cam_dma_trace_mark("after_encoder_open");
    if (ioctl(enc_fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "LIVE encoder DQ timeout ioctl rejected errno=%d", errno);
    }

    /* The esp_video H.264 codec object is created when the encoder CAPTURE
     * queue is STREAMON, so the production I_PERIOD/QP controls must be set
     * before that point.  1080p starts at GOP5; 720p production starts at GOP8.
     * 1080 live-GOP changes after open use the dedicated no-restart HW path. */
    if (h264_set_ctrl_required(enc_fd, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
                               profile->gop, "i_period_precreate") != ESP_OK ||
        h264_set_ctrl_required(enc_fd, V4L2_CID_MPEG_VIDEO_BITRATE,
                               profile->bitrate, "bitrate") != ESP_OK ||
        h264_set_ctrl_required(enc_fd, V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
                               live_qp_min, "min_qp") != ESP_OK ||
        h264_set_ctrl_required(enc_fd, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
                               live_qp_max, "max_qp") != ESP_OK) {
        saved_errno = errno ? errno : EINVAL;
        fail_stage = "live_encoder_required_ctrl";
        goto live_fail;
    }
    h264_streamparm_probe_best_effort(enc_fd, profile->fps);

    struct v4l2_format enc_in_fmt = {0};
    enc_in_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    enc_in_fmt.fmt.pix.width = width;
    enc_in_fmt.fmt.pix.height = height;
    enc_in_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    if (ioctl(enc_fd, VIDIOC_S_FMT, &enc_in_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_S_FMT_OUTPUT";
        goto live_fail;
    }
    struct v4l2_requestbuffers enc_out_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
    };
    if (ioctl(enc_fd, VIDIOC_REQBUFS, &enc_out_req) != 0 || enc_out_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "live_encoder_REQBUFS_OUTPUT";
        goto live_fail;
    }
    struct v4l2_buffer enc_out_query = {
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
        .index = 0,
    };
    if (ioctl(enc_fd, VIDIOC_QUERYBUF, &enc_out_query) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_QUERYBUF_OUTPUT";
        goto live_fail;
    }
    const uint32_t enc_input_required = enc_out_query.length;
    ESP_LOGI(TAG,
             "LIVE encoder input buffer need=%u bytes (S_FMT sizeimage=%u) for %ux%u YU12",
             (unsigned)enc_input_required, (unsigned)enc_in_fmt.fmt.pix.sizeimage,
             (unsigned)width, (unsigned)height);

    struct v4l2_format enc_cap_fmt = {0};
    enc_cap_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    enc_cap_fmt.fmt.pix.width = width;
    enc_cap_fmt.fmt.pix.height = height;
    enc_cap_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    if (ioctl(enc_fd, VIDIOC_S_FMT, &enc_cap_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_S_FMT_CAPTURE";
        goto live_fail;
    }
    struct v4l2_requestbuffers enc_cap_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(enc_fd, VIDIOC_REQBUFS, &enc_cap_req) != 0 || enc_cap_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "live_encoder_REQBUFS_CAPTURE";
        goto live_fail;
    }
    struct v4l2_buffer enc_cap_buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
        .index = 0,
    };
    if (ioctl(enc_fd, VIDIOC_QUERYBUF, &enc_cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_QUERYBUF_CAPTURE";
        goto live_fail;
    }
    enc_map_len = enc_cap_buf.length;
    enc_map = mmap(NULL, enc_map_len, PROT_READ | PROT_WRITE,
                   MAP_SHARED, enc_fd, enc_cap_buf.m.offset);
    if (!enc_map || enc_map == MAP_FAILED) {
        enc_map = NULL;
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "live_encoder_mmap_CAPTURE";
        goto live_fail;
    }
    if (ioctl(enc_fd, VIDIOC_QBUF, &enc_cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_QBUF_CAPTURE_initial";
        goto live_fail;
    }
    cam_dma_trace_mark("before_encoder_cap_streamon");
    ESP_LOGW(TAG, "[DMA-DIAG D12 before_encoder_cap_streamon] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    int enc_cap_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(enc_fd, VIDIOC_STREAMON, &enc_cap_type) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_STREAMON_CAPTURE";
        goto live_fail;
    }
    enc_cap_streaming = true;
    cam_dma_trace_mark("after_encoder_cap_streamon");
    ESP_LOGW(TAG, "[DMA-DIAG D13 after_encoder_cap_streamon_refpool] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    /* Do NOT rewrite I_PERIOD here; the codec was just created with the
     * pre-STREAMON production GOP above. */
    ESP_LOGI(TAG,
             "LIVE encoder profile=%s: %ux%u@%u GOP=%u QP=%u..%u %s bitrate=%u",
             profile->name, (unsigned)profile->width, (unsigned)profile->height,
             (unsigned)profile->fps, (unsigned)profile->gop,
             (unsigned)live_qp_min, (unsigned)live_qp_max, live_qp_mode,
             (unsigned)profile->bitrate);

    int enc_out_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    if (ioctl(enc_fd, VIDIOC_STREAMON, &enc_out_type) != 0) {
        saved_errno = errno;
        fail_stage = "live_encoder_STREAMON_OUTPUT";
        goto live_fail;
    }
    enc_out_streaming = true;
    cam_dma_trace_mark("after_encoder_out_streamon");

    /* [ISP-LIFECYCLE-C12E-720 2026-08-27]
     * C12D proved that after the application's 720 S_FMT the controller's
     * cached geometry is still 1920x1080 while a fresh G_FMT reports
     * 1288x728.  For 720 only, correct that cached width/height, keep the
     * existing four-field no-write rebase, then recreate/init IPA and apply
     * its initial metadata before camera STREAMON.
     *
     * No sensor register write, controller restart, metadata-fd reopen, or
     * camera-fd lifecycle is added.  1080 remains the proven C12C path. */
    if (!c12a_is_1080) {
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12E-720] 720 post-S_FMT/pre-STREAMON "
                 "geometry-refresh + four-field + fresh-IPA begin");
        esp_err_t c12e_720_ret = esp_video_isp_pipeline_720_resync_for_c12a();
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12E-720] 720 geometry+fresh-IPA ret=%s (0x%x)",
                 esp_err_to_name(c12e_720_ret), (unsigned)c12e_720_ret);
        if (c12e_720_ret != ESP_OK) {
            saved_errno = EIO;
            fail_stage = "live_isp_c12e_720_geometry_fresh_ipa";
            goto live_fail;
        }
    } else {
        /* [ISP-LIFECYCLE-C12C-1080 2026-08-27]
         * Incremental A/B over C12B: for 1080 only, recreate/reinitialize the
         * IPA at the proven post-S_FMT/pre-STREAMON point and apply its initial
         * metadata.  Do NOT add geometry refresh, sensor writes, metadata-fd
         * restart/reopen, or another controller lifecycle. */
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12C-1080] 1080 post-S_FMT/pre-STREAMON fresh-IPA-only begin");
        esp_err_t c12c_1080_ret = esp_video_isp_pipeline_fresh_ipa_for_c12c_1080();
        ESP_LOGW(TAG,
                 "[ISP-LIFECYCLE-C12C-1080] 1080 fresh-IPA-only ret=%s (0x%x)",
                 esp_err_to_name(c12c_1080_ret), (unsigned)c12c_1080_ret);
        if (c12c_1080_ret != ESP_OK) {
            saved_errno = EIO;
            fail_stage = "live_isp_c12c_1080_fresh_ipa";
            goto live_fail;
        }
    }

    ESP_LOGW(TAG, "[DMA-DIAG D14 after_fresh_ipa] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    int cam_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &cam_type) != 0) {
        saved_errno = errno;
        fail_stage = "live_camera_STREAMON";
        goto live_fail;
    }
    cam_streaming = true;
    cam_dma_trace_mark("after_camera_streamon");
    ESP_LOGW(TAG, "[DMA-DIAG D15 after_camera_streamon] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    ESP_LOGI(TAG, "LIVE pipeline STREAMON OK %ux%u YU12->H264; warmup=2",
             (unsigned)width, (unsigned)height);

    for (int w = 0; w < 2 && !live_stop_requested(); ++w) {
        struct v4l2_buffer warm = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &warm) != 0) {
            saved_errno = errno;
            fail_stage = "live_camera_DQBUF_warmup";
            goto live_fail;
        }
        if (ioctl(cam_fd, VIDIOC_QBUF, &warm) != 0) {
            saved_errno = errno;
            fail_stage = "live_camera_QBUF_warmup";
            goto live_fail;
        }
    }

    start_us = esp_timer_get_time();
    last_log_us = start_us;
    fps_window_start_us = start_us;
    taskENTER_CRITICAL(&s_live_mux);
    s_live.stats.width = width;
    s_live.stats.height = height;
    s_live.stats.sensor_fps = sensor_fps;
    s_live.stats.h264_gop = profile->gop;
    s_live.stats.h264_qp_min = live_qp_min;
    s_live.stats.h264_qp_max = live_qp_max;
    s_live.stats.h264_bitrate = profile->bitrate;
    snprintf(s_live.stats.last_stage, sizeof(s_live.stats.last_stage), "%s", profile->stage);
    taskEXIT_CRITICAL(&s_live_mux);

    while (!live_stop_requested()) {
        live_apply_pending_gop_request();
        int64_t loop_start_us = esp_timer_get_time();
        int64_t dq_start_us = loop_start_us;
        struct v4l2_buffer cb = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &cb) != 0) {
            if (live_stop_requested()) break;
            saved_errno = errno;
            fail_stage = "live_camera_DQBUF";
            goto live_fail;
        }
        if (cb.index >= cam_mapped_count || !cam_maps[cb.index].start) {
            saved_errno = EFAULT;
            fail_stage = "live_camera_bad_index";
            goto live_fail;
        }
        int64_t dq_done_us = esp_timer_get_time();
        uint32_t camera_dq_us = dq_done_us > dq_start_us ?
            (uint32_t)(dq_done_us - dq_start_us) : 0;
        camera_dq_us_sum += camera_dq_us;
        if (camera_dq_us > camera_dq_us_max) camera_dq_us_max = camera_dq_us;
        camera_frames++;
        if (!first_camera_us) first_camera_us = dq_done_us;
        last_camera_us = dq_done_us;

        /* USERPTR length is the buffer capacity required by the M2M encoder,
         * not necessarily the camera driver's bytesused value.  For 1920x1080
         * both happened to be identical.  Native 1288x728 may carry alignment
         * padding, so bytesused can be smaller than the encoder QUERYBUF
         * requirement and QBUF then returns EINVAL.  The camera MMAP capacity
         * is authoritative for whether zero-copy is safe. */
        uint32_t enc_user_len = enc_input_required ? enc_input_required : cb.bytesused;
        if (enc_user_len == 0 || cam_maps[cb.index].length < enc_user_len) {
            ESP_LOGE(TAG,
                     "LIVE zero-copy input too small: profile=%s cb{bytesused=%u index=%u} cam_map=%u enc_need=%u",
                     profile->name, (unsigned)cb.bytesused, (unsigned)cb.index,
                     (unsigned)cam_maps[cb.index].length, (unsigned)enc_user_len);
            saved_errno = EMSGSIZE;
            fail_stage = "live_encoder_input_capacity";
            goto live_fail;
        }
        if (camera_frames <= 3) {
            ESP_LOGI(TAG,
                     "LIVE zero-copy input frame=%u cb_bytesused=%u cam_map=%u enc_user_len=%u",
                     (unsigned)camera_frames, (unsigned)cb.bytesused,
                     (unsigned)cam_maps[cb.index].length, (unsigned)enc_user_len);
        }

        /* [CAM-AUTO-COLOR-CAL-R1-20260910]
         * No continuous overhead: only compute a fresh sparse O_UYY measurement
         * when the UI has explicitly requested one. */
        {
            const uint32_t color_req = camera_color_measure_pending_for(profile_id);
            if (color_req != 0) {
                const uint32_t elapsed_ms =
                    (dq_done_us > start_us) ? (uint32_t)((dq_done_us - start_us) / 1000) : 0U;
                camera_color_measurement_t cm = {0};
                if (camera_ouyy_diag_measure(
                        (const uint8_t *)cam_maps[cb.index].start,
                        cb.bytesused, width, height, profile_id, elapsed_ms,
                        &cm, NULL, NULL, NULL, NULL, NULL)) {
                    camera_color_measure_publish(color_req, &cm);
                }
            }
        }

        if (yu12_diag_sample_index <
                (sizeof(yu12_diag_targets_ms) / sizeof(yu12_diag_targets_ms[0]))) {
            const uint32_t elapsed_ms =
                (dq_done_us > start_us) ? (uint32_t)((dq_done_us - start_us) / 1000) : 0U;
            if (elapsed_ms >= yu12_diag_targets_ms[yu12_diag_sample_index]) {
                camera_yu12_diag_emit(
                    (const uint8_t *)cam_maps[cb.index].start,
                    cb.bytesused,
                    width,
                    height,
                    profile->name,
                    profile_id,
                    elapsed_ms,
                    yu12_diag_sample_index);
                yu12_diag_sample_index++;
            }
        }

        struct v4l2_buffer enc_in = {
            .index = 0,
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_USERPTR,
            .m.userptr = (unsigned long)cam_maps[cb.index].start,
            .length = enc_user_len,
        };
        int64_t enc_start = esp_timer_get_time();
        if (ioctl(enc_fd, VIDIOC_QBUF, &enc_in) != 0) {
            saved_errno = errno;
            ESP_LOGE(TAG,
                     "LIVE encoder QBUF failed profile=%s errno=%d cb_bytesused=%u cam_map=%u enc_len=%u enc_need=%u",
                     profile->name, errno, (unsigned)cb.bytesused,
                     (unsigned)cam_maps[cb.index].length, (unsigned)enc_in.length,
                     (unsigned)enc_input_required);
            fail_stage = "live_encoder_QBUF_OUTPUT";
            goto live_fail;
        }

        struct v4l2_buffer encoded = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(enc_fd, VIDIOC_DQBUF, &encoded) != 0) {
            saved_errno = errno;
            fail_stage = "live_encoder_DQBUF_CAPTURE";
            goto live_fail;
        }
        struct v4l2_buffer enc_done = {
            .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
            .memory = V4L2_MEMORY_USERPTR,
        };
        if (ioctl(enc_fd, VIDIOC_DQBUF, &enc_done) != 0) {
            saved_errno = errno;
            fail_stage = "live_encoder_DQBUF_OUTPUT";
            goto live_fail;
        }

        /* The encoder has returned its USERPTR OUTPUT buffer, so it no longer
         * owns the camera frame. Requeue the CSI buffer immediately, before
         * parsing/sending the encoded bitstream, so the camera/ISP can capture
         * the next frame in parallel with WebRTC/SRTP work. */
        if (ioctl(cam_fd, VIDIOC_QBUF, &cb) != 0) {
            saved_errno = errno;
            fail_stage = "live_camera_QBUF_early_requeue";
            goto live_fail;
        }

        int64_t now_us = esp_timer_get_time();
        uint32_t enc_us = now_us > enc_start ? (uint32_t)(now_us - enc_start) : 0;

        bool valid = (encoded.flags & V4L2_BUF_FLAG_DONE) && encoded.bytesused > 0 &&
                     encoded.index == 0 && encoded.bytesused <= enc_map_len;
        if (valid) {
            uint32_t used = encoded.bytesused;
            encoded_frames++;
            if (!first_encoded_us) first_encoded_us = now_us;
            last_encoded_us = now_us;
            if (used > h264_bytes_max) h264_bytes_max = used;
            if (enc_us > enc_us_max) enc_us_max = enc_us;

            if (profile->gop == 1U && encoded_frames <= 4) {
                h264_nal_stats_t verify_nal = {0};
                h264_scan_annexb((const uint8_t *)enc_map, used, &verify_nal);
                if (!verify_nal.idr) {
                    saved_errno = EPROTO;
                    fail_stage = "live_gop1_not_applied";
                    ESP_LOGE(TAG,
                             "LIVE GOP1 verification failed frame=%u NAL{id=%u p=%u}; "
                             "encoder was not created as all-IDR",
                             (unsigned)encoded_frames,
                             (unsigned)verify_nal.idr,
                             (unsigned)verify_nal.non_idr);
                    goto live_fail;
                }
                if (encoded_frames == 4) {
                    ESP_LOGI(TAG, "LIVE GOP1 verification PASS: first 4/4 access units are IDR");
                }
            }

            if (encoded_frames == 1) {
                const uint8_t *first_bs = (const uint8_t *)enc_map;
                h264_scan_annexb(first_bs, used, &first_nal);
                uint8_t profile_idc = 0, constraint_flags = 0, level_idc = 0;
                bool have_sps_profile = h264_get_sps_profile_level(first_bs, used,
                                                                   &profile_idc,
                                                                   &constraint_flags,
                                                                   &level_idc);
                ESP_LOGI(TAG, "LIVE first H264 bytes=%u NAL{sps=%u pps=%u idr=%u p=%u}",
                         (unsigned)used, (unsigned)first_nal.sps, (unsigned)first_nal.pps,
                         (unsigned)first_nal.idr, (unsigned)first_nal.non_idr);
                ESP_LOGW(TAG, "[DMA-DIAG D16 after_first_h264_frame] free=%u largest=%u",
                         (unsigned)dma_free(), (unsigned)dma_largest());
                if (have_sps_profile) {
                    ESP_LOGI(TAG,
                             "LIVE SPS profile-level-id=%02x%02x%02x "
                             "(profile_idc=%u constraints=0x%02x level_idc=%u)",
                             profile_idc, constraint_flags, level_idc,
                             (unsigned)profile_idc, (unsigned)constraint_flags,
                             (unsigned)level_idc);
                } else {
                    ESP_LOGW(TAG, "LIVE first IDR did not expose a parseable SPS profile/level");
                }
                if (!first_nal.idr) {
                    saved_errno = EPROTO;
                    fail_stage = "live_first_frame_not_idr";
                    ESP_LOGE(TAG, "LIVE expected GOP1/all-IDR but first access unit has no IDR NAL");
                    goto live_fail;
                }
            }

            uint32_t pts_ms = start_us && now_us > start_us
                ? (uint32_t)((now_us - start_us) / 1000LL) : 0;
            int cr = -1;
            camera_h264_frame_cb_t cb_fn = NULL;
            void *cb_ctx = NULL;
            taskENTER_CRITICAL(&s_live_mux);
            cb_fn = s_live.cb;
            cb_ctx = s_live.cb_ctx;
            taskEXIT_CRITICAL(&s_live_mux);
            int64_t consumer_start_us = esp_timer_get_time();
            if (cb_fn) cr = cb_fn((const uint8_t *)enc_map, used, pts_ms, cb_ctx);
            int64_t consumer_done_us = esp_timer_get_time();
            uint32_t consumer_us = consumer_done_us > consumer_start_us ?
                (uint32_t)(consumer_done_us - consumer_start_us) : 0;
            if (consumer_us > consumer_us_max) consumer_us_max = consumer_us;
            if (cr == 0) delivered_frames++;
            else consumer_drops++;

            camera_fps_total_x100 = 0;
            if (camera_frames > 1 && last_camera_us > first_camera_us) {
                camera_fps_total_x100 = (uint32_t)(((uint64_t)(camera_frames - 1) * 100000000ULL) /
                                                   (uint64_t)(last_camera_us - first_camera_us));
            }
            encode_fps_total_x100 = 0;
            if (encoded_frames > 1 && last_encoded_us > first_encoded_us) {
                encode_fps_total_x100 = (uint32_t)(((uint64_t)(encoded_frames - 1) * 100000000ULL) /
                                                   (uint64_t)(last_encoded_us - first_encoded_us));
            }

            /* Publish one stable, completed two-second window.  Until the
             * first window completes, retain the lifetime estimate so the UI
             * does not show 0 fps during startup.  No allocation or extra
             * task/timer is introduced. */
            int64_t fps_window_elapsed_us = now_us - fps_window_start_us;
            if (fps_window_elapsed_us >= CAM_LIVE_FPS_WINDOW_US) {
                camera_fps_recent_x100 = (uint32_t)(
                    ((uint64_t)(camera_frames - fps_window_camera_base) * 100000000ULL) /
                    (uint64_t)fps_window_elapsed_us);
                encode_fps_recent_x100 = (uint32_t)(
                    ((uint64_t)(encoded_frames - fps_window_encoded_base) * 100000000ULL) /
                    (uint64_t)fps_window_elapsed_us);
                fps_window_camera_base = camera_frames;
                fps_window_encoded_base = encoded_frames;
                fps_window_start_us = now_us;
            }
            const uint32_t camera_fps_publish_x100 = camera_fps_recent_x100
                ? camera_fps_recent_x100 : camera_fps_total_x100;
            const uint32_t encode_fps_publish_x100 = encode_fps_recent_x100
                ? encode_fps_recent_x100 : encode_fps_total_x100;
            taskENTER_CRITICAL(&s_live_mux);
            s_live.stats.camera_frames = camera_frames;
            s_live.stats.encoded_frames = encoded_frames;
            s_live.stats.delivered_frames = delivered_frames;
            s_live.stats.consumer_drops = consumer_drops;
            s_live.stats.encoder_skips = encoder_skips;
            s_live.stats.camera_fps_x100 = camera_fps_publish_x100;
            s_live.stats.encode_fps_x100 = encode_fps_publish_x100;
            s_live.stats.h264_bytes_last = used;
            s_live.stats.h264_bytes_max = h264_bytes_max;
            s_live.stats.encode_us_last = enc_us;
            s_live.stats.encode_us_max = enc_us_max;
            s_live.stats.camera_dq_us_last = camera_dq_us;
            s_live.stats.camera_dq_us_max = camera_dq_us_max;
            s_live.stats.camera_dq_us_avg = camera_frames ?
                (uint32_t)(camera_dq_us_sum / camera_frames) : 0;
            s_live.stats.consumer_us_last = consumer_us;
            s_live.stats.consumer_us_max = consumer_us_max;
            taskEXIT_CRITICAL(&s_live_mux);
        } else {
            encoder_skips++;
        }

        encoded.index = 0;
        encoded.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        encoded.memory = V4L2_MEMORY_MMAP;
        if (ioctl(enc_fd, VIDIOC_QBUF, &encoded) != 0) {
            saved_errno = errno;
            fail_stage = "live_encoder_QBUF_CAPTURE_requeue";
            goto live_fail;
        }
        int64_t loop_done_us = esp_timer_get_time();
        uint32_t loop_us = loop_done_us > loop_start_us ?
            (uint32_t)(loop_done_us - loop_start_us) : 0;
        if (loop_us > loop_us_max) loop_us_max = loop_us;
        taskENTER_CRITICAL(&s_live_mux);
        s_live.stats.loop_us_last = loop_us;
        s_live.stats.loop_us_max = loop_us_max;
        taskEXIT_CRITICAL(&s_live_mux);

        if (loop_done_us - last_log_us >= CAM_LIVE_LOG_INTERVAL_US) {
            size_t df = dma_free(), dl = dma_largest(), pf = psram_free();
            taskENTER_CRITICAL(&s_live_mux);
            s_live.stats.dma_free = (uint32_t)df;
            s_live.stats.dma_largest = (uint32_t)dl;
            s_live.stats.psram_free = (uint32_t)pf;
            taskEXIT_CRITICAL(&s_live_mux);
            if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,
                     "LIVE timing cam=%u enc=%u sent=%u drop=%u cap_fps=%u.%02u enc_fps=%u.%02u "
                     "total_fps=%u.%02u/%u.%02u "
                     "dq_us=%u avg=%u max=%u enc_us=%u max=%u consumer_us=%u max=%u "
                     "loop_us=%u max=%u dma=%u/%u psram=%u",
                     (unsigned)camera_frames, (unsigned)encoded_frames,
                     (unsigned)delivered_frames, (unsigned)consumer_drops,
                     (unsigned)(s_live.stats.camera_fps_x100 / 100),
                     (unsigned)(s_live.stats.camera_fps_x100 % 100),
                     (unsigned)(s_live.stats.encode_fps_x100 / 100),
                     (unsigned)(s_live.stats.encode_fps_x100 % 100),
                     (unsigned)(camera_fps_total_x100 / 100),
                     (unsigned)(camera_fps_total_x100 % 100),
                     (unsigned)(encode_fps_total_x100 / 100),
                     (unsigned)(encode_fps_total_x100 % 100),
                     (unsigned)s_live.stats.camera_dq_us_last,
                     (unsigned)s_live.stats.camera_dq_us_avg,
                     (unsigned)s_live.stats.camera_dq_us_max,
                     (unsigned)s_live.stats.encode_us_last, (unsigned)enc_us_max,
                     (unsigned)s_live.stats.consumer_us_last, (unsigned)consumer_us_max,
                     (unsigned)s_live.stats.loop_us_last, (unsigned)loop_us_max,
                     (unsigned)df, (unsigned)dl, (unsigned)pf);
            last_log_us = loop_done_us;
        }
    }

    fail_stage = "stopped";
    saved_errno = 0;
    goto live_cleanup;

live_fail:
    ESP_LOGE(TAG, "LIVE FAIL stage=%s errno=%d (%s)",
             fail_stage ? fail_stage : "unknown", saved_errno,
             saved_errno ? strerror(saved_errno) : "n/a");

live_cleanup:
    ESP_LOGW(TAG, "[DMA-DIAG D5 camera_cleanup_entry] free=%u largest=%u",
             (unsigned)dma_free(), (unsigned)dma_largest());
    /* Freeze new trace entries, but keep tracking frees performed by cleanup. */
    cam_dma_trace_pause_for_720_cleanup(profile_id);
    if (lcd_stream_flag) {
        lcd_ui_set_camera_stream_active(false);
        lcd_stream_flag = false;
    }
    if (cam_streaming && cam_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc_out_streaming && enc_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
    }
    if (enc_cap_streaming && enc_fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(enc_fd, VIDIOC_STREAMOFF, &type);
    }
    /* Unmap first, then explicitly release each V4L2 queue while its fd is
     * still open.  This is important for repeated 720p60 <-> 1080p switching. */
    if (enc_map) {
        munmap(enc_map, enc_map_len);
        enc_map = NULL;
    }
    for (uint32_t i = 0; i < cam_mapped_count; ++i) {
        if (cam_maps[i].start) {
            munmap(cam_maps[i].start, cam_maps[i].length);
            cam_maps[i].start = NULL;
        }
    }

    size_t rel_df0 = dma_free(), rel_dl0 = dma_largest(), rel_pf0 = psram_free();
    ESP_LOGI(TAG, "[REQBUFS-RELEASE] before dma=%u/%u psram=%u",
             (unsigned)rel_df0, (unsigned)rel_dl0, (unsigned)rel_pf0);

    live_reqbufs_release_best_effort(enc_fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                     V4L2_MEMORY_USERPTR, "h264-output");
    live_reqbufs_release_best_effort(enc_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                     V4L2_MEMORY_MMAP, "h264-capture");
    live_reqbufs_release_best_effort(cam_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                     V4L2_MEMORY_MMAP, "camera-capture");

    /* Allow any driver-side deferred queue cleanup to complete before the fd
     * close and before we sample the heap for the next profile. */
    vTaskDelay(pdMS_TO_TICKS(20));

    size_t rel_df1 = dma_free(), rel_dl1 = dma_largest(), rel_pf1 = psram_free();
    ESP_LOGI(TAG, "[REQBUFS-RELEASE] after  dma=%u/%u psram=%u delta_largest=%d",
             (unsigned)rel_df1, (unsigned)rel_dl1, (unsigned)rel_pf1,
             (int)rel_dl1 - (int)rel_dl0);
    ESP_LOGW(TAG, "[DMA-DIAG D6 after_reqbufs_release] free=%u largest=%u",
             (unsigned)rel_df1, (unsigned)rel_dl1);

    if (enc_fd >= 0) close(enc_fd);
    if (cam_fd >= 0) {
        /* [OWNER-REF-GUARD 2026-09-11] release the extra reference taken at open
         * (only if we actually took it) so the device still deinits when idle. */
        if (owner_ref_held) {
            int own = 0;
            ioctl(cam_fd, VIDIOC_SET_OWNER, &own);
            owner_ref_held = false;
        }
        close(cam_fd);
    }

    /* One scheduler turn after close makes the post-cleanup measurement more
     * representative for a subsequent Start request. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* [ISP-LIFECYCLE-C2F-T-20260826]
     * Timing-only A/B: the B+C gain-state resync was MOVED from here (Stop)
     * to the Start path (after camera VIDIOC_S_FMT, before camera VIDIOC_STREAMON).
     * Helper body unchanged. */
    cam_dma_trace_dump_after_720_cleanup();

    size_t df = dma_free(), dl = dma_largest(), pf = psram_free();
    ESP_LOGW(TAG, "[DMA-DIAG D7 after_camera_cleanup] free=%u largest=%u",
             (unsigned)df, (unsigned)dl);
    ESP_LOGI(TAG, "LIVE cleanup stage=%s frames=%u sent=%u drop=%u dma=%u/%u psram=%u",
             fail_stage ? fail_stage : "unknown", (unsigned)encoded_frames,
             (unsigned)delivered_frames, (unsigned)consumer_drops,
             (unsigned)df, (unsigned)dl, (unsigned)pf);

    taskENTER_CRITICAL(&s_live_mux);
    s_live.stats.running = false;
    s_live.stats.dma_free = (uint32_t)df;
    s_live.stats.dma_largest = (uint32_t)dl;
    s_live.stats.psram_free = (uint32_t)pf;
    s_live.stats.last_errno = saved_errno;
    snprintf(s_live.stats.last_stage, sizeof(s_live.stats.last_stage), "%s",
             fail_stage ? fail_stage : "unknown");
    s_live.running = false;
    s_live.task = NULL;
    taskEXIT_CRITICAL(&s_live_mux);

    if (s_live.done) xSemaphoreGive(s_live.done);
    vTaskDelete(NULL);
}

static esp_err_t camera_dma_prewarm_h264_tiny(int *out_errno,
                                               char *out_stage,
                                               size_t out_stage_len)
{
    int fd = -1;
    bool cap_streaming = false;
    void *cap_map = NULL;
    size_t cap_map_len = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] H264-TINY begin %ux%u dma=%u/%u",
             (unsigned)CAM_DMA_PREWARM_H264_WIDTH,
             (unsigned)CAM_DMA_PREWARM_H264_HEIGHT,
             (unsigned)dma_free(), (unsigned)dma_largest());

    fd = open("/dev/video11", O_RDWR);
    if (fd < 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_open_video11";
        goto fail;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_QUERYCAP";
        goto fail;
    }

    /* Use the production GOP1/all-IDR QP30..40 auto-RC control ordering: I_PERIOD must be set
     * before CAPTURE STREAMON because that is where the codec object is made. */
    if (h264_set_ctrl_required(fd, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
                               1, "prewarm_i_period") != ESP_OK ||
        h264_set_ctrl_required(fd, V4L2_CID_MPEG_VIDEO_BITRATE,
                               1000000, "prewarm_bitrate") != ESP_OK ||
        h264_set_ctrl_required(fd, V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
                               30, "prewarm_min_qp") != ESP_OK ||
        h264_set_ctrl_required(fd, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
                               30, "prewarm_max_qp") != ESP_OK) {
        saved_errno = errno ? errno : EINVAL;
        fail_stage = "h264_tiny_required_ctrl";
        goto fail;
    }

    struct v4l2_format in_fmt = {0};
    in_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    in_fmt.fmt.pix.width = CAM_DMA_PREWARM_H264_WIDTH;
    in_fmt.fmt.pix.height = CAM_DMA_PREWARM_H264_HEIGHT;
    in_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    in_fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &in_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_S_FMT_OUTPUT";
        goto fail;
    }
    if (in_fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        saved_errno = EINVAL;
        fail_stage = "h264_tiny_OUTPUT_not_YU12";
        goto fail;
    }

    struct v4l2_requestbuffers out_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &out_req) != 0 || out_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_tiny_REQBUFS_OUTPUT";
        goto fail;
    }
    struct v4l2_buffer out_query = {
        .type = V4L2_BUF_TYPE_VIDEO_OUTPUT,
        .memory = V4L2_MEMORY_USERPTR,
        .index = 0,
    };
    if (ioctl(fd, VIDIOC_QUERYBUF, &out_query) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_QUERYBUF_OUTPUT";
        goto fail;
    }

    struct v4l2_format cap_fmt = {0};
    cap_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_fmt.fmt.pix.width = CAM_DMA_PREWARM_H264_WIDTH;
    cap_fmt.fmt.pix.height = CAM_DMA_PREWARM_H264_HEIGHT;
    cap_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    cap_fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &cap_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_S_FMT_CAPTURE";
        goto fail;
    }

    struct v4l2_requestbuffers cap_req = {
        .count = 1,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &cap_req) != 0 || cap_req.count < 1) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_tiny_REQBUFS_CAPTURE";
        goto fail;
    }
    struct v4l2_buffer cap_buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
        .index = 0,
    };
    if (ioctl(fd, VIDIOC_QUERYBUF, &cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_QUERYBUF_CAPTURE";
        goto fail;
    }
    cap_map_len = cap_buf.length;
    cap_map = mmap(NULL, cap_map_len, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, cap_buf.m.offset);
    if (!cap_map || cap_map == MAP_FAILED) {
        cap_map = NULL;
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264_tiny_mmap_CAPTURE";
        goto fail;
    }
    if (ioctl(fd, VIDIOC_QBUF, &cap_buf) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_QBUF_CAPTURE";
        goto fail;
    }

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] H264-TINY before CAPTURE STREAMON input_need=%u dma=%u/%u",
             (unsigned)out_query.length,
             (unsigned)dma_free(), (unsigned)dma_largest());

    int cap_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &cap_type) != 0) {
        saved_errno = errno;
        fail_stage = "h264_tiny_STREAMON_CAPTURE";
        goto fail;
    }
    cap_streaming = true;

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] H264-TINY PASS dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    if (cap_streaming) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
        cap_streaming = false;
    }
    if (cap_map) {
        munmap(cap_map, cap_map_len);
        cap_map = NULL;
    }
    live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                     V4L2_MEMORY_MMAP, "prewarm-h264-capture");
    live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                     V4L2_MEMORY_USERPTR, "prewarm-h264-output");
    close(fd);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (out_errno) *out_errno = 0;
    if (out_stage && out_stage_len) snprintf(out_stage, out_stage_len, "ok");
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] H264-TINY cleanup dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());
    return ESP_OK;

fail:
    if (out_errno) *out_errno = saved_errno;
    if (out_stage && out_stage_len) {
        snprintf(out_stage, out_stage_len, "%s", fail_stage ? fail_stage : "h264_tiny_unknown");
    }
    ESP_LOGE(TAG,
             "[CAM-DMA-STAGED] H264-TINY FAIL stage=%s errno=%d (%s) dma=%u/%u",
             fail_stage ? fail_stage : "unknown", saved_errno,
             strerror(saved_errno ? saved_errno : EIO),
             (unsigned)dma_free(), (unsigned)dma_largest());
    if (cap_streaming && fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
    }
    if (cap_map) munmap(cap_map, cap_map_len);
    if (fd >= 0) {
        live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                         V4L2_MEMORY_MMAP, "prewarm-h264-capture");
        live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT,
                                         V4L2_MEMORY_USERPTR, "prewarm-h264-output");
        close(fd);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return saved_errno == ENOMEM ? ESP_ERR_NO_MEM : ESP_FAIL;
}

static esp_err_t camera_dma_prewarm_camera_isp_720(int *out_errno,
                                                   char *out_stage,
                                                   size_t out_stage_len)
{
    int fd = -1;
    bool streaming = false;
    cam_map_buf_t maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t mapped_count = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP begin dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (fd < 0) {
        saved_errno = errno;
        fail_stage = "camera_isp_open_video0";
        goto fail;
    }

    const esp_cam_sensor_format_t *requested_sensor_fmt =
        ov02c10_webrtc_get_profile_format((uint32_t)CAMERA_H264_PROFILE_720P_NATIVE);
    if (!requested_sensor_fmt) {
        saved_errno = EINVAL;
        fail_stage = "camera_isp_get_sensor_profile";
        goto fail;
    }
    if (ioctl(fd, VIDIOC_S_SENSOR_FMT, requested_sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "camera_isp_S_SENSOR_FMT";
        goto fail;
    }

    esp_cam_sensor_format_t sensor_fmt = {0};
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno;
        fail_stage = "camera_isp_G_SENSOR_FMT";
        goto fail;
    }
    if (sensor_fmt.width != 1288U || sensor_fmt.height != 728U ||
        sensor_fmt.mipi_info.lane_num != 2U) {
        saved_errno = EINVAL;
        fail_stage = "camera_isp_sensor_profile_mismatch";
        goto fail;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = sensor_fmt.width;
    fmt.fmt.pix.height = sensor_fmt.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        saved_errno = errno;
        fail_stage = "camera_isp_S_FMT_YU12";
        goto fail;
    }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        saved_errno = EINVAL;
        fail_stage = "camera_isp_not_YU12";
        goto fail;
    }
    camera_streamparm_30fps_best_effort(fd, sensor_fmt.fps ? sensor_fmt.fps : 60U);

    struct timeval timeout = { .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC, .tv_usec = 0 };
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGW(TAG, "[CAM-DMA-STAGED] CAMERA-ISP DQ timeout ioctl rejected errno=%d", errno);
    }

    struct v4l2_requestbuffers rb = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) != 0 || rb.count < 2) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "camera_isp_REQBUFS";
        goto fail;
    }
    if (rb.count > CAM_CSI_BUFFER_COUNT) rb.count = CAM_CSI_BUFFER_COUNT;

    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "camera_isp_QUERYBUF";
            goto fail;
        }
        maps[i].length = b.length;
        maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, fd, b.m.offset);
        if (!maps[i].start || maps[i].start == MAP_FAILED) {
            maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "camera_isp_mmap";
            goto fail;
        }
        mapped_count++;
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "camera_isp_QBUF_initial";
            goto fail;
        }
    }

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP before STREAMON dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        saved_errno = errno;
        fail_stage = "camera_isp_STREAMON";
        goto fail;
    }
    streaming = true;
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP STREAMON dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    for (uint32_t n = 0; n < CAM_DMA_PREWARM_CAMERA_FRAMES; ++n) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "camera_isp_DQBUF";
            goto fail;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno;
            fail_stage = "camera_isp_QBUF";
            goto fail;
        }
    }

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP PASS frames=%u dma=%u/%u",
             (unsigned)CAM_DMA_PREWARM_CAMERA_FRAMES,
             (unsigned)dma_free(), (unsigned)dma_largest());

    if (streaming) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
        streaming = false;
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                     V4L2_MEMORY_MMAP, "prewarm-camera-capture");
    close(fd);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (out_errno) *out_errno = 0;
    if (out_stage && out_stage_len) snprintf(out_stage, out_stage_len, "ok");
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP cleanup dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());
    return ESP_OK;

fail:
    if (out_errno) *out_errno = saved_errno;
    if (out_stage && out_stage_len) {
        snprintf(out_stage, out_stage_len, "%s", fail_stage ? fail_stage : "camera_isp_unknown");
    }
    ESP_LOGE(TAG,
             "[CAM-DMA-STAGED] CAMERA-ISP FAIL stage=%s errno=%d (%s) dma=%u/%u",
             fail_stage ? fail_stage : "unknown", saved_errno,
             strerror(saved_errno ? saved_errno : EIO),
             (unsigned)dma_free(), (unsigned)dma_largest());
    if (streaming && fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    if (fd >= 0) {
        live_reqbufs_release_best_effort(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                         V4L2_MEMORY_MMAP, "prewarm-camera-capture");
        close(fd);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return saved_errno == ENOMEM ? ESP_ERR_NO_MEM : ESP_FAIL;
}

esp_err_t camera_h264_dma_prewarm(void)
{
    if (s_dma_prewarm_attempted) {
        return ESP_OK;
    }
    s_dma_prewarm_attempted = true;

    if (s_state.running || s_state.consumed || camera_h264_stream_is_active()) {
        ESP_LOGW(TAG, "[CAM-DMA-STAGED] skipped: camera already busy/diagnostic consumed");
        return ESP_ERR_INVALID_STATE;
    }

    const camera_h264_profile_t saved_profile = camera_h264_stream_get_profile();
    const size_t dma_before = dma_free();
    const size_t largest_before = dma_largest();
    const size_t psram_before = psram_free();
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] begin anchor=%u tiny_h264=%ux%u camera=1288x728 dma=%u/%u psram=%u",
             (unsigned)CAM_DMA_PREWARM_ANCHOR_BYTES,
             (unsigned)CAM_DMA_PREWARM_H264_WIDTH,
             (unsigned)CAM_DMA_PREWARM_H264_HEIGHT,
             (unsigned)dma_before, (unsigned)largest_before, (unsigned)psram_before);

    void *dma_anchor = heap_caps_malloc(CAM_DMA_PREWARM_ANCHOR_BYTES,
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!dma_anchor) {
        ESP_LOGE(TAG,
                 "[CAM-DMA-STAGED] anchor reserve failed bytes=%u dma=%u/%u",
                 (unsigned)CAM_DMA_PREWARM_ANCHOR_BYTES,
                 (unsigned)dma_free(), (unsigned)dma_largest());
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] anchor reserved ptr=%p bytes=%u dma_now=%u/%u",
             dma_anchor, (unsigned)CAM_DMA_PREWARM_ANCHOR_BYTES,
             (unsigned)dma_free(), (unsigned)dma_largest());

    esp_err_t ret = camera_h264_stream_set_profile(CAMERA_H264_PROFILE_720P_NATIVE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[CAM-DMA-STAGED] set 720p profile failed: %s", esp_err_to_name(ret));
        heap_caps_free(dma_anchor);
        return ret;
    }

    int stage_errno = 0;
    char stage[64] = {0};
    bool h264_ok = false;
    bool camera_ok = false;
    esp_err_t h264_ret = ESP_FAIL;
    esp_err_t camera_ret = ESP_FAIL;
    esp_err_t deinit_ret = ESP_OK;

    s_dma_prewarm_active = true;

    if (ov02c10_webrtc_select_profile((uint32_t)CAMERA_H264_PROFILE_720P_NATIVE) != ESP_OK) {
        stage_errno = EINVAL;
        snprintf(stage, sizeof(stage), "select_720p_profile");
        ret = ESP_ERR_INVALID_ARG;
        goto staged_cleanup;
    }

    ret = init_video_once();
    if (ret != ESP_OK) {
        stage_errno = EIO;
        snprintf(stage, sizeof(stage), "esp_video_init");
        goto staged_cleanup;
    }
    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] after esp_video_init dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    h264_ret = camera_dma_prewarm_h264_tiny(&stage_errno, stage, sizeof(stage));
    if (h264_ret != ESP_OK) {
        ret = h264_ret;
        goto staged_cleanup;
    }
    h264_ok = true;

    camera_ret = camera_dma_prewarm_camera_isp_720(&stage_errno, stage, sizeof(stage));
    if (camera_ret != ESP_OK) {
        ret = camera_ret;
        goto staged_cleanup;
    }
    camera_ok = true;
    snprintf(stage, sizeof(stage), "ok");
    stage_errno = 0;
    ret = ESP_OK;

staged_cleanup:
    s_dma_prewarm_active = false;

    /* esp_video was intentionally kept alive across the two staged phases so
     * all esp_video/CSI/ISP lazy objects were created while the anchor was
     * protected.  Tear it down before releasing the anchor. */
    if (s_state.video_inited) {
        const size_t before_deinit = dma_largest();
        deinit_ret = esp_video_deinit();
        if (deinit_ret == ESP_OK) {
            s_state.video_inited = false;
            vTaskDelay(pdMS_TO_TICKS(40));
            ESP_LOGW(TAG,
                     "[CAM-DMA-STAGED] esp_video_deinit OK anchor-held largest=%u->%u",
                     (unsigned)before_deinit, (unsigned)dma_largest());
        } else {
            ESP_LOGE(TAG,
                     "[CAM-DMA-STAGED] esp_video_deinit failed: %s; keep video_inited=1",
                     esp_err_to_name(deinit_ret));
        }
    }

    const size_t held_free = dma_free();
    const size_t held_largest = dma_largest();
    heap_caps_free(dma_anchor);
    dma_anchor = NULL;
    vTaskDelay(pdMS_TO_TICKS(20));
    const size_t release_free = dma_free();
    const size_t release_largest = dma_largest();

    const bool staged_complete = h264_ok && camera_ok && deinit_ret == ESP_OK;
    const bool ready1080 = staged_complete &&
                           release_largest >= CAM_LIVE_DMA_PREFERRED_LARGEST;

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] RELEASE held=%u/%u -> free=%u/%u recovered_largest=%d h264_ok=%d camera_ok=%d ready1080=%d",
             (unsigned)held_free, (unsigned)held_largest,
             (unsigned)release_free, (unsigned)release_largest,
             (int)release_largest - (int)held_largest,
             h264_ok ? 1 : 0, camera_ok ? 1 : 0, ready1080 ? 1 : 0);

    (void)camera_h264_stream_set_profile(saved_profile);

    ESP_LOGW(TAG,
             "[CAM-DMA-STAGED] end stage=%s errno=%d ret=%s dma=%u/%u -> %u/%u psram=%u->%u",
             stage[0] ? stage : "unknown", stage_errno,
             esp_err_to_name(ret),
             (unsigned)dma_before, (unsigned)largest_before,
             (unsigned)release_free, (unsigned)release_largest,
             (unsigned)psram_before, (unsigned)psram_free());

    if (!h264_ok || !camera_ok) {
        return ret == ESP_OK ? ESP_FAIL : ret;
    }
    if (deinit_ret != ESP_OK) {
        return deinit_ret;
    }
    if (!ready1080) {
        ESP_LOGW(TAG,
                 "[CAM-DMA-STAGED] insufficient post-prewarm contiguous DMA: largest=%u < guard=%u",
                 (unsigned)release_largest,
                 (unsigned)CAM_LIVE_DMA_PREFERRED_LARGEST);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t camera_dma_late_isr_stabilize_once(void)
{
    if (s_dma_late_isr_attempted) {
        return ESP_OK;
    }
    s_dma_late_isr_attempted = true;

    if (s_state.video_inited) {
        ESP_LOGW(TAG,
                 "[CAM-DMA-LATE-ISR] skip: esp_video already resident dma=%u/%u",
                 (unsigned)dma_free(), (unsigned)dma_largest());
        return ESP_ERR_INVALID_STATE;
    }

    const size_t before_free = dma_free();
    const size_t before_largest = dma_largest();
    const size_t ps_before = psram_free();
    ESP_LOGW(TAG,
             "[CAM-DMA-LATE-ISR] begin after-network dma=%u/%u psram=%u",
             (unsigned)before_free, (unsigned)before_largest, (unsigned)ps_before);

    void *anchor = NULL;
    size_t anchor_bytes = 0;
    const size_t candidates[] = {
        CAM_DMA_LATE_ISR_ANCHOR_MAX_BYTES,
        CAM_DMA_LATE_ISR_ANCHOR_MIN_BYTES,
    };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        const size_t want = candidates[i];
        if (before_largest < want) continue;
        anchor = heap_caps_malloc(want, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!anchor) continue;
        const size_t rem_largest = dma_largest();
        if (rem_largest >= CAM_DMA_LATE_ISR_MIN_REMAINDER) {
            anchor_bytes = want;
            ESP_LOGW(TAG,
                     "[CAM-DMA-LATE-ISR] anchor reserved ptr=%p bytes=%u remainder=%u/%u",
                     anchor, (unsigned)want,
                     (unsigned)dma_free(), (unsigned)rem_largest);
            break;
        }
        ESP_LOGW(TAG,
                 "[CAM-DMA-LATE-ISR] anchor=%u leaves largest=%u < remainder_floor=%u; retry smaller",
                 (unsigned)want, (unsigned)rem_largest,
                 (unsigned)CAM_DMA_LATE_ISR_MIN_REMAINDER);
        heap_caps_free(anchor);
        anchor = NULL;
    }

    if (!anchor) {
        ESP_LOGW(TAG,
                 "[CAM-DMA-LATE-ISR] no safe anchor candidate; continue real stream dma=%u/%u",
                 (unsigned)dma_free(), (unsigned)dma_largest());
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_FAIL;
    esp_err_t h264_ret = ESP_FAIL;
    esp_err_t camera_ret = ESP_FAIL;
    esp_err_t deinit_ret = ESP_OK;
    int stage_errno = 0;
    char stage[64] = {0};
    bool h264_ok = false;
    bool camera_ok = false;

    s_dma_prewarm_active = true;

    if (ov02c10_webrtc_select_profile((uint32_t)CAMERA_H264_PROFILE_720P_NATIVE) != ESP_OK) {
        stage_errno = EINVAL;
        snprintf(stage, sizeof(stage), "late_select_720p_profile");
        ret = ESP_ERR_INVALID_ARG;
        goto late_cleanup;
    }

    ret = init_video_once();
    if (ret != ESP_OK) {
        stage_errno = EIO;
        snprintf(stage, sizeof(stage), "late_esp_video_init");
        goto late_cleanup;
    }
    ESP_LOGW(TAG,
             "[CAM-DMA-LATE-ISR] after esp_video_init dma=%u/%u",
             (unsigned)dma_free(), (unsigned)dma_largest());

    h264_ret = camera_dma_prewarm_h264_tiny(&stage_errno, stage, sizeof(stage));
    if (h264_ret != ESP_OK) {
        ret = h264_ret;
        goto late_cleanup;
    }
    h264_ok = true;

    camera_ret = camera_dma_prewarm_camera_isp_720(&stage_errno, stage, sizeof(stage));
    if (camera_ret != ESP_OK) {
        ret = camera_ret;
        goto late_cleanup;
    }
    camera_ok = true;
    ret = ESP_OK;

late_cleanup:
    if (s_state.video_inited) {
        const size_t l0 = dma_largest();
        deinit_ret = esp_video_deinit();
        if (deinit_ret == ESP_OK) {
            s_state.video_inited = false;
            ESP_LOGW(TAG,
                     "[CAM-DMA-LATE-ISR] esp_video_deinit OK anchor-held largest=%u->%u",
                     (unsigned)l0, (unsigned)dma_largest());
        } else {
            ESP_LOGE(TAG,
                     "[CAM-DMA-LATE-ISR] esp_video_deinit failed: %s",
                     esp_err_to_name(deinit_ret));
        }
    }

    s_dma_prewarm_active = false;

    const size_t held_free = dma_free();
    const size_t held_largest = dma_largest();
    heap_caps_free(anchor);
    anchor = NULL;
    const size_t release_free = dma_free();
    const size_t release_largest = dma_largest();
    const bool ready = h264_ok && camera_ok && deinit_ret == ESP_OK &&
                       release_largest >= anchor_bytes;

    ESP_LOGW(TAG,
             "[CAM-DMA-LATE-ISR] RELEASE anchor=%u held=%u/%u -> free=%u/%u h264_ok=%d camera_ok=%d ready=%d",
             (unsigned)anchor_bytes,
             (unsigned)held_free, (unsigned)held_largest,
             (unsigned)release_free, (unsigned)release_largest,
             h264_ok ? 1 : 0, camera_ok ? 1 : 0, ready ? 1 : 0);
    ESP_LOGW(TAG,
             "[CAM-DMA-LATE-ISR] end stage=%s errno=%d ret=%s dma=%u/%u -> %u/%u psram=%u->%u",
             stage[0] ? stage : "ok", stage_errno, esp_err_to_name(ret),
             (unsigned)before_free, (unsigned)before_largest,
             (unsigned)release_free, (unsigned)release_largest,
             (unsigned)ps_before, (unsigned)psram_free());

    if (!h264_ok || !camera_ok) return ret == ESP_OK ? ESP_FAIL : ret;
    if (deinit_ret != ESP_OK) return deinit_ret;
    if (!ready) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

esp_err_t camera_h264_stream_start(camera_h264_frame_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (s_state.running || s_state.consumed) {
        ESP_LOGW(TAG, "LIVE start rejected: one-shot diagnostic running/consumed; reboot first");
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(&s_live_mux);
    bool already = s_live.running;
    taskEXIT_CRITICAL(&s_live_mux);
    if (already) return ESP_ERR_INVALID_STATE;

    if (!s_live.done) {
        s_live.done = xSemaphoreCreateBinary();
        if (!s_live.done) return ESP_ERR_NO_MEM;
    }
    (void)xSemaphoreTake(s_live.done, 0); /* drain completion from previous clean stop */

    taskENTER_CRITICAL(&s_live_mux);
    s_live.stop_requested = false;
    s_live.cb = cb;
    s_live.cb_ctx = ctx;
    s_live.gop_request = s_live_1080_gop_selected;
    s_live.gop_request_seq = 0U;
    s_live.gop_applied_seq = 0U;
    s_live.gop_apply_result = ESP_OK;
    s_live.running = true;
    memset(&s_live.stats, 0, sizeof(s_live.stats));
    s_live.stats.running = true;
    snprintf(s_live.stats.last_stage, sizeof(s_live.stats.last_stage), "task_create");
    taskEXIT_CRITICAL(&s_live_mux);

    TaskHandle_t th = NULL;
    if (xTaskCreatePinnedToCore(camera_h264_live_task, "cam_h264_live",
                                CAM_LIVE_TASK_STACK, NULL, CAM_LIVE_TASK_PRIO,
                                &th, CAM_LIVE_TASK_CORE) != pdPASS) {
        taskENTER_CRITICAL(&s_live_mux);
        s_live.running = false;
        s_live.stats.running = false;
        s_live.stats.last_errno = ENOMEM;
        snprintf(s_live.stats.last_stage, sizeof(s_live.stats.last_stage), "task_create_failed");
        taskEXIT_CRITICAL(&s_live_mux);
        return ESP_ERR_NO_MEM;
    }
    taskENTER_CRITICAL(&s_live_mux);
    s_live.task = th;
    taskEXIT_CRITICAL(&s_live_mux);
    return ESP_OK;
}

esp_err_t camera_h264_stream_stop(uint32_t timeout_ms)
{
    camera_h264_stream_request_stop();
    if (!camera_h264_stream_is_active()) return ESP_OK;
    if (!s_live.done) return ESP_ERR_INVALID_STATE;
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms ? timeout_ms : 1U);
    if (xSemaphoreTake(s_live.done, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "LIVE stop timeout after %ums", (unsigned)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t camera_csi_capture_bench_http_impl(httpd_req_t *req,
                                                   uint32_t pixfmt,
                                                   const char *test_name)
{
    if (!req || !test_name) return ESP_ERR_INVALID_ARG;
    if (s_state.running || camera_h264_stream_is_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"camera busy\"}");
    }
    if (s_state.consumed) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera diagnostic already consumed\","
            "\"reboot_before_retest\":true}");
    }

    s_state.running = true;
    s_state.consumed = true;

    int fd = -1;
    bool streaming = false;
    cam_map_buf_t maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t mapped_count = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;
    esp_cam_sensor_format_t sensor_fmt = {0};
    uint32_t frames = 0;
    uint32_t dq_us_last = 0, dq_us_max = 0;
    uint64_t dq_us_sum = 0;
    uint32_t bytes_min = UINT32_MAX, bytes_max = 0;
    int64_t t_first = 0, t_last = 0;
    char req_fcc[5] = {0}, actual_fcc[5] = {0};
    fourcc_to_str(pixfmt, req_fcc);

    const size_t dma0 = dma_free(), dl0 = dma_largest(), ps0 = psram_free();
    ESP_LOGI(TAG, "CAPBENCH %s begin fmt=%s buffers=%u dma=%u/%u psram=%u",
             test_name, req_fcc, (unsigned)CAM_CSI_BUFFER_COUNT,
             (unsigned)dma0, (unsigned)dl0, (unsigned)ps0);

    esp_err_t ret = init_video_once();
    if (ret != ESP_OK) {
        s_state.running = false;
        return send_json_error(req, "500 Internal Server Error", "capbench_esp_video_init", 0, ret);
    }

    fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (fd < 0) { saved_errno = errno; fail_stage = "capbench_open_video0"; goto bench_fail; }
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno; fail_stage = "capbench_G_SENSOR_FMT"; goto bench_fail;
    }
    if (!sensor_fmt.width || !sensor_fmt.height) {
        saved_errno = EINVAL; fail_stage = "capbench_sensor_fmt_invalid"; goto bench_fail;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = sensor_fmt.width;
    fmt.fmt.pix.height = sensor_fmt.height;
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        saved_errno = errno; fail_stage = "capbench_S_FMT"; goto bench_fail;
    }
    fourcc_to_str(fmt.fmt.pix.pixelformat, actual_fcc);
    ESP_LOGI(TAG, "CAPBENCH %s active=%ux%u request=%s actual=%s sizeimage=%u",
             test_name, (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
             req_fcc, actual_fcc, (unsigned)fmt.fmt.pix.sizeimage);
    if (fmt.fmt.pix.pixelformat != pixfmt) {
        saved_errno = EINVAL; fail_stage = "capbench_driver_changed_format"; goto bench_fail;
    }

    camera_streamparm_30fps_best_effort(fd, sensor_fmt.fps);
    struct timeval timeout = { .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC, .tv_usec = 0 };
    (void)ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout);

    struct v4l2_requestbuffers rb = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) != 0 || rb.count < 2) {
        saved_errno = errno ? errno : ENOMEM; fail_stage = "capbench_REQBUFS"; goto bench_fail;
    }
    if (rb.count > CAM_CSI_BUFFER_COUNT) rb.count = CAM_CSI_BUFFER_COUNT;
    ESP_LOGI(TAG, "CAPBENCH %s buffers requested=%u actual=%u",
             test_name, (unsigned)CAM_CSI_BUFFER_COUNT, (unsigned)rb.count);

    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_QUERYBUF"; goto bench_fail;
        }
        maps[i].length = b.length;
        maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (!maps[i].start || maps[i].start == MAP_FAILED) {
            maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "capbench_mmap";
            goto bench_fail;
        }
        mapped_count++;
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_QBUF_initial"; goto bench_fail;
        }
        ESP_LOGI(TAG, "CAPBENCH %s buffer[%u] len=%u", test_name,
                 (unsigned)i, (unsigned)b.length);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        saved_errno = errno; fail_stage = "capbench_STREAMON"; goto bench_fail;
    }
    streaming = true;

    for (int w = 0; w < 3; ++w) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_DQBUF_warmup"; goto bench_fail;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_QBUF_warmup"; goto bench_fail;
        }
    }

    for (uint32_t n = 0; n < CAM_CAPTURE_BENCH_FRAME_COUNT; ++n) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        int64_t dq0 = esp_timer_get_time();
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_DQBUF"; goto bench_fail;
        }
        int64_t now = esp_timer_get_time();
        dq_us_last = now > dq0 ? (uint32_t)(now - dq0) : 0;
        dq_us_sum += dq_us_last;
        if (dq_us_last > dq_us_max) dq_us_max = dq_us_last;
        if (b.index >= mapped_count || !maps[b.index].start) {
            saved_errno = EFAULT; fail_stage = "capbench_bad_index"; goto bench_fail;
        }
        if (frames == 0) t_first = now;
        t_last = now;
        uint32_t used = b.bytesused;
        if (used < bytes_min) bytes_min = used;
        if (used > bytes_max) bytes_max = used;
        frames++;

        /* Return the frame immediately: this benchmark intentionally has no
         * H.264, hashing, memcpy, WebRTC or UI work in the capture loop. */
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "capbench_QBUF_requeue"; goto bench_fail;
        }
        if (frames <= 3 || (frames % 10U) == 0U) {
            ESP_LOGI(TAG, "CAPBENCH %s frame=%u seq=%u bytes=%u dq_us=%u",
                     test_name, (unsigned)frames, (unsigned)b.sequence,
                     (unsigned)used, (unsigned)dq_us_last);
        }
    }

    if (streaming) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
        streaming = false;
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    close(fd);
    fd = -1;

    {
        int64_t elapsed = (frames > 1 && t_last > t_first) ? (t_last - t_first) : 0;
        uint32_t fps_x100 = elapsed ?
            (uint32_t)(((uint64_t)(frames - 1) * 100000000ULL) / (uint64_t)elapsed) : 0;
        uint32_t dq_avg = frames ? (uint32_t)(dq_us_sum / frames) : 0;
        size_t dfa = dma_free(), dla = dma_largest(), psa = psram_free();
        ESP_LOGI(TAG,
                 "CAPBENCH %s PASS frames=%u fps=%u.%02u dq_us avg/max=%u/%u buffers=%u dma=%u/%u",
                 test_name, (unsigned)frames,
                 (unsigned)(fps_x100 / 100), (unsigned)(fps_x100 % 100),
                 (unsigned)dq_avg, (unsigned)dq_us_max, (unsigned)mapped_count,
                 (unsigned)dfa, (unsigned)dla);

        char body[900];
        snprintf(body, sizeof(body),
                 "{\"ok\":true,\"test\":\"%s\",\"sensor\":\"OV02C10\","
                 "\"width\":%u,\"height\":%u,\"fourcc\":\"%s\","
                 "\"nominal_fps\":%u,\"buffers\":%u,\"frames\":%u,\"fps_x100\":%u,"
                 "\"dq_us\":{\"last\":%u,\"avg\":%u,\"max\":%u},"
                 "\"bytes_min\":%u,\"bytes_max\":%u,"
                 "\"dma\":{\"free_before\":%u,\"largest_before\":%u,"
                 "\"free_after\":%u,\"largest_after\":%u},"
                 "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                 "\"reboot_before_retest\":true}",
                 test_name, (unsigned)sensor_fmt.width, (unsigned)sensor_fmt.height,
                 actual_fcc, (unsigned)sensor_fmt.fps, (unsigned)mapped_count,
                 (unsigned)frames, (unsigned)fps_x100,
                 (unsigned)dq_us_last, (unsigned)dq_avg, (unsigned)dq_us_max,
                 (unsigned)(bytes_min == UINT32_MAX ? 0 : bytes_min), (unsigned)bytes_max,
                 (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                 (unsigned)ps0, (unsigned)psa);
        s_state.running = false;
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_sendstr(req, body);
    }

bench_fail:
    if (streaming && fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(fd, VIDIOC_STREAMOFF, &t);
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    if (fd >= 0) close(fd);
    s_state.running = false;
    ESP_LOGE(TAG, "CAPBENCH %s FAIL stage=%s errno=%d (%s)", test_name,
             fail_stage ? fail_stage : "unknown", saved_errno,
             saved_errno ? strerror(saved_errno) : "n/a");
    return send_json_error(req, "500 Internal Server Error",
                           fail_stage ? fail_stage : "capbench_unknown",
                           saved_errno, ESP_FAIL);
}

esp_err_t camera_csi_capture_bench_http(httpd_req_t *req, uint32_t pixfmt,
                                        const char *test_name)
{
    return camera_csi_capture_bench_http_impl(req, pixfmt, test_name);
}


esp_err_t camera_h264_direct_bench_http(httpd_req_t *req)
{
    if (!req) return ESP_ERR_INVALID_ARG;
    if (s_state.running || camera_h264_stream_is_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"camera busy\"}");
    }
    if (s_state.consumed) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera diagnostic already consumed\"," 
            "\"reboot_before_retest\":true}");
    }

    /* Optional direct-bench controls.  With no parameters this preserves the
     * previous 20..40 RC / GOP30 baseline.  qp=N makes qp_min==qp_max and
     * therefore disables the esp_h264 RC path.  outkb=N deliberately changes
     * only the encoded-output buffer capacity.  gop=N changes only the GOP so
     * GOP=1 can isolate the all-IDR hardware path without touching HW regs. */
    int fixed_qp = -1;
    uint32_t requested_out_kb = 0;
    uint32_t requested_gop = 30;
    bool requested_camoff = false;
    bool requested_camhold = false;
    bool requested_camseq = false; /* same-boot CAMHOLD->CAMOFF using one held raw frame */
    bool requested_lcdpat = false; /* same-boot LCDON->LCDPATTERN, camera STREAMOFF throughout */
    bool requested_priab = false;  /* same-boot NORMAL/HIGH task-priority A/B, camera OFF + DSI pattern throughout */
    bool requested_fpsab = false;  /* same-boot encoder cfg.fps 30/60 A/B, camera OFF + DSI pattern throughout */
    bool requested_livegop = false; /* final 1080p live-camera GOP30 test: fresh DQBUF frame every encode, 120 frames */
    bool requested_live720 = false; /* 1280x720 live-camera GOP30 via centered memory crop from native 1920x1080 */
    bool requested_cropab = false; /* live720 same-boot CPU memcpy 120 -> AXI-GDMA 120 crop A/B */
    bool requested_ppaab = false;  /* live720: validate CPU vs PPA bytes, then CPU120 -> PPA120 */
    bool requested_overlap = false; /* live720: Core0 CPU crop of next frame overlaps H264 HW encode via ping-pong buffers */
    bool requested_overlapcamab = false; /* same held frame; camera DMA ON->OFF A/B while crop+H264 overlap */
    int requested_h264_qos = 0; /* benchmark-only AXI QoS A/B: 0=baseline, 1=H264 M1/M2 AR/AW QoS 1 */
    char query[224] = {0};
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0 && query_len < sizeof(query) &&
        httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(query, "qp", val, sizeof(val)) == ESP_OK) {
            char *end = NULL;
            long q = strtol(val, &end, 10);
            if (!end || *end != '\0' || q < 0 || q > 51) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"qp must be 0..51\"}");
            }
            fixed_qp = (int)q;
        }
        if (httpd_query_key_value(query, "outkb", val, sizeof(val)) == ESP_OK) {
            char *end = NULL;
            unsigned long kb = strtoul(val, &end, 10);
            if (!end || *end != '\0' || kb < 64 || kb > 3072) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"outkb must be 64..3072\"}");
            }
            requested_out_kb = (uint32_t)kb;
        }
        if (httpd_query_key_value(query, "gop", val, sizeof(val)) == ESP_OK) {
            char *end = NULL;
            unsigned long g = strtoul(val, &end, 10);
            if (!end || *end != '\0' || g < 1 || g > 255) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"gop must be 1..255\"}");
            }
            requested_gop = (uint32_t)g;
        }
        if (httpd_query_key_value(query, "camoff", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_camoff = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"camoff must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "camhold", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_camhold = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"camhold must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "camseq", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_camseq = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"camseq must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "lcdpat", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_lcdpat = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"lcdpat must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "priab", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_priab = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"priab must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "fpsab", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_fpsab = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"fpsab must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "livegop", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_livegop = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"livegop must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "live720", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_live720 = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"live720 must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "cropab", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_cropab = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"cropab must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "ppaab", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_ppaab = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"ppaab must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "overlap", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_overlap = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"overlap must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "overlapcamab", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_overlapcamab = true;
            } else if (!(strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0)) {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"overlapcamab must be 0/1 or false/true\"}");
            }
        }
        if (httpd_query_key_value(query, "h264qos", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "1") == 0 || strcasecmp(val, "true") == 0) {
                requested_h264_qos = 1;
            } else if (strcmp(val, "0") == 0 || strcasecmp(val, "false") == 0) {
                requested_h264_qos = 0;
            } else {
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json; charset=utf-8");
                return httpd_resp_sendstr(req,
                    "{\"ok\":false,\"error\":\"h264qos must be 0/1 or false/true\"}");
            }
        }
        unsigned exclusive_modes = (requested_camoff ? 1U : 0U) +
                                   (requested_camhold ? 1U : 0U) +
                                   (requested_camseq ? 1U : 0U) +
                                   (requested_lcdpat ? 1U : 0U) +
                                   (requested_priab ? 1U : 0U) +
                                   (requested_fpsab ? 1U : 0U) +
                                   (requested_livegop ? 1U : 0U) +
                                   (requested_live720 ? 1U : 0U);
        if (exclusive_modes > 1U) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"camoff, camhold, camseq, lcdpat, priab, fpsab, livegop and live720 are mutually exclusive\"}");
        }
        if ((requested_camseq || requested_lcdpat || requested_priab || requested_fpsab || requested_livegop || requested_live720) && requested_h264_qos != 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"camseq/lcdpat/priab/fpsab/livegop/live720 requires h264qos=0 for controlled benchmark\"}");
        }
        if (requested_cropab && !requested_live720) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"cropab=1 requires live720=1\"}");
        }
        if (requested_ppaab && !requested_live720) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"ppaab=1 requires live720=1\"}");
        }
        if (requested_cropab && requested_ppaab) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"cropab and ppaab cannot both be enabled\"}");
        }
        if (requested_overlap && !requested_live720) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"overlap=1 requires live720=1\"}");
        }
        if (requested_overlap && (requested_cropab || requested_ppaab)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"overlap cannot be combined with cropab or ppaab\"}");
        }
        if (requested_overlapcamab && !(requested_live720 && requested_overlap)) {
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            return httpd_resp_sendstr(req,
                "{\"ok\":false,\"error\":\"overlapcamab=1 requires live720=1&overlap=1\"}");
        }
    }

    ESP_LOGI(TAG,
             "H264-DIRECT NATIVE60-BUILD-20260823-V1 query_len=%u query=\"%s\" parsed{live720=%d overlap=%d overlapcamab=%d cropab=%d ppaab=%d}",
             (unsigned)query_len, query, requested_live720 ? 1 : 0, requested_overlap ? 1 : 0,
             requested_overlapcamab ? 1 : 0, requested_cropab ? 1 : 0, requested_ppaab ? 1 : 0);

    if (requested_livegop || requested_live720) {
        /* Dedicated live-camera GOP30 tests: do not let a stale gop= query accidentally change the workload. */
        requested_gop = 30U;
    }

    s_state.running = true;
    s_state.consumed = true;

    const size_t dma0 = dma_free();
    const size_t dl0 = dma_largest();
    const size_t ps0 = psram_free();
    ESP_LOGI(TAG, "H264-DIRECT begin dma=%u/%u psram=%u",
             (unsigned)dma0, (unsigned)dl0, (unsigned)ps0);

    if (dl0 < CAM_LIVE_MIN_DMA_LARGEST) {
        s_state.running = false;
        return send_json_error(req, "503 Service Unavailable",
                               "h264direct_dma_guard", ENOMEM, ESP_ERR_NO_MEM);
    }

    esp_err_t init_ret = init_video_once();
    if (init_ret != ESP_OK) {
        s_state.running = false;
        return send_json_error(req, "500 Internal Server Error",
                               "h264direct_esp_video_init", 0, init_ret);
    }

    int cam_fd = -1;
    bool cam_streaming = false;
    cam_map_buf_t maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t mapped_count = 0;
    uint8_t *out_buf = NULL;
    uint8_t *memcrop_buf = NULL;
    uint8_t *memcrop_buf2 = NULL;
    h264_overlap_crop_ctx_t overlap_ctx = {0};
    bool overlap_worker_started = false;
    h264_overlap_cam_pump_ctx_t overlap_pump_ctx = {0};
    bool overlap_pump_started = false;
    uint32_t overlap_prime_dq_us = 0;
    uint32_t overlap_prime_crop_us = 0;
    uint64_t overlap_wait_total = 0;
    uint32_t overlap_wait_max = 0;
    uint32_t overlap_wait_count = 0;
    uint32_t overlap_camab_frames[2] = {0, 0};
    uint64_t overlap_camab_crop_total[2] = {0, 0};
    uint32_t overlap_camab_crop_min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t overlap_camab_crop_max[2] = {0, 0};
    uint64_t overlap_camab_enc_total[2] = {0, 0};
    uint32_t overlap_camab_enc_min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t overlap_camab_enc_max[2] = {0, 0};
    uint64_t overlap_camab_wait_total[2] = {0, 0};
    uint32_t overlap_camab_wait_max[2] = {0, 0};
    uint32_t overlap_camab_wait_count[2] = {0, 0};
    uint32_t overlap_camab_idr[2] = {0, 0};
    uint32_t overlap_camab_i[2] = {0, 0};
    uint32_t overlap_camab_p[2] = {0, 0};
    uint32_t overlap_camab_hash_first[2] = {0, 0};
    uint32_t overlap_camab_hash_last[2] = {0, 0};
    int64_t overlap_camab_phase_start[2] = {0, 0};
    int64_t overlap_camab_phase_end[2] = {0, 0};
    uint32_t overlap_camab_held_hash_initial = 0;
    uint32_t overlap_camab_held_hash_after_on = 0;
    uint32_t overlap_camab_held_hash_after_off = 0;
    uint32_t overlap_camab_pump_frames = 0;
    uint64_t overlap_camab_pump_dq_total = 0;
    uint32_t overlap_camab_pump_dq_max = 0;
    async_memcpy_handle_t crop_dma_hdl = NULL;
    SemaphoreHandle_t crop_dma_done = NULL;
    bool crop_dma_installed = false;
    ppa_client_handle_t crop_ppa_client = NULL;
    uint8_t *ppa_verify_buf = NULL;
    bool ppa_validation_done = false;
    uint32_t ppa_validation_mismatches = 0;
    uint32_t ppa_validation_first_mismatch = UINT32_MAX;
    uint32_t ppa_validation_cpu_hash = 0;
    uint32_t ppa_validation_ppa_hash = 0;
    uint32_t ppa_validation_cpu_us = 0;
    uint32_t ppa_validation_ppa_us = 0;
    size_t out_capacity = 0;
    esp_h264_enc_handle_t enc = NULL;
    bool enc_opened = false;
    int saved_errno = 0;
    esp_h264_err_t saved_h264_err = ESP_H264_ERR_OK;
    const char *fail_stage = NULL;
    esp_cam_sensor_format_t sensor_fmt = {0};
    struct v4l2_buffer held_b = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    bool held_valid = false;
    uint32_t held_raw_hash = 0;
    uint32_t held_raw_hash_after_hold = 0;
    uint32_t held_raw_hash_after_off = 0;
    h264_clock_diag_t h264_clk_open = {0};
    h264_clock_diag_t h264_clk_end = {0};
    axi_qos_diag_t axi_qos_open = {0};
    axi_qos_diag_t axi_qos_end = {0};
    axi_qos_diag_t axi_qos_active_end = {0};
    bool h264_qos_applied = false;
    uint8_t h264_qos_saved_m1_r = 0, h264_qos_saved_m1_w = 0;
    uint8_t h264_qos_saved_m2_r = 0, h264_qos_saved_m2_w = 0;

    uint32_t frames = 0;
    uint32_t bytes_min = UINT32_MAX, bytes_max = 0;
    uint64_t bytes_total = 0;
    uint32_t enc_us_min = UINT32_MAX, enc_us_max = 0, enc_us_last = 0;
    uint64_t enc_us_total = 0;
    uint32_t dq_us_max = 0, dq_us_last = 0;
    uint64_t dq_us_total = 0;
    uint32_t crop_us_min = UINT32_MAX, crop_us_max = 0, crop_us_last = 0;
    uint64_t crop_us_total = 0;
    /* cropab phase 0=CPU memcpy, 1=AXI-GDMA; each phase is 120 fresh frames. */
    uint32_t cropab_frames[2] = {0, 0};
    uint64_t cropab_crop_total[2] = {0, 0};
    uint32_t cropab_crop_min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t cropab_crop_max[2] = {0, 0};
    uint64_t cropab_enc_total[2] = {0, 0};
    uint32_t cropab_enc_min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t cropab_enc_max[2] = {0, 0};
    uint64_t cropab_dq_total[2] = {0, 0};
    uint32_t cropab_dq_max[2] = {0, 0};
    uint32_t cropab_idr[2] = {0, 0};
    uint32_t cropab_i[2] = {0, 0};
    uint32_t cropab_p[2] = {0, 0};
    int64_t cropab_t_first[2] = {0, 0};
    int64_t cropab_t_last[2] = {0, 0};
    uint32_t idr_frames = 0, p_frames = 0, i_frames = 0;
    uint64_t idr_us_total = 0, p_us_total = 0, i_us_total = 0;
    uint32_t idr_us_min = UINT32_MAX, idr_us_max = 0;
    uint32_t p_us_min = UINT32_MAX, p_us_max = 0;
    uint32_t i_us_min = UINT32_MAX, i_us_max = 0;
    uint64_t idr_bytes_total = 0, p_bytes_total = 0, i_bytes_total = 0;
    uint32_t idr_bytes_min = UINT32_MAX, idr_bytes_max = 0;
    uint32_t p_bytes_min = UINT32_MAX, p_bytes_max = 0;
    uint32_t i_bytes_min = UINT32_MAX, i_bytes_max = 0;
    h264_nal_stats_t nal = {0};
    uint32_t hash_first = 0, hash_last = 0;
    int64_t t_first = 0, t_last = 0;

    /* [H264-SAMEBOOT-CAM-AB-20260822]
     * seq index 0 = CAMHOLD (camera DMA active), 1 = CAMOFF (STREAMOFF).
     * Both phases encode the exact same held V4L2 raw buffer in one request. */
    uint32_t seq_frames[2] = {0, 0};
    uint64_t seq_enc_total[2] = {0, 0};
    uint32_t seq_enc_min[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t seq_enc_max[2] = {0, 0};
    uint64_t seq_dq_total[2] = {0, 0};
    uint32_t seq_dq_max[2] = {0, 0};
    uint32_t seq_hash_first[2] = {0, 0};
    uint32_t seq_hash_last[2] = {0, 0};
    int64_t seq_t_first[2] = {0, 0};
    int64_t seq_t_last[2] = {0, 0};
    /* LCD framebuffer-scanout A/B diagnostics.  LVGL/BSP stays initialized;
     * phase2 uses the DSI hardware pattern generator and is restored before
     * the HTTP reply. */
    lv_display_t *lcdpat_display = NULL;
    esp_lcd_panel_handle_t lcdpat_panel = NULL;
    bool lcdpat_active = false;
    bool lcdpat_lvgl_timer_paused = false;
    bool lcdpat_restored = false;
    TaskHandle_t priab_task = NULL;
    UBaseType_t priab_prio_normal = 0;
    UBaseType_t priab_prio_high = 0;
    bool priab_prio_raised = false;
    uint32_t fpsab_reopen_count = 0;
    uint32_t diag_capture_actual_width = 0, diag_capture_actual_height = 0;
    int32_t diag_crop_left = 0, diag_crop_top = 0;
    uint32_t diag_crop_width = 0, diag_crop_height = 0;
    size_t lcdpat_dma_free_before_pattern = 0, lcdpat_dma_largest_before_pattern = 0;
    size_t lcdpat_psram_free_before_pattern = 0;
    size_t lcdpat_dma_free_after_pattern = 0, lcdpat_dma_largest_after_pattern = 0;
    size_t lcdpat_psram_free_after_pattern = 0;

    cam_fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (cam_fd < 0) {
        saved_errno = errno; fail_stage = "h264direct_open_video0"; goto direct_fail;
    }

    /* [LIVE720-MEMCROP-REV13-20260823]
     * ESP32-P4 ISP image crop exists only on revision 3.0+.  This board is
     * revision 1.3, so live720 deliberately leaves CSI/ISP capture at native
     * 1920x1080 YU12 and performs a centered row-copy crop after DQBUF. */
    if (ioctl(cam_fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno; fail_stage = "h264direct_G_SENSOR_FMT"; goto direct_fail;
    }
    if (!sensor_fmt.width || !sensor_fmt.height) {
        saved_errno = EINVAL; fail_stage = "h264direct_sensor_fmt_invalid"; goto direct_fail;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = sensor_fmt.width;
    fmt.fmt.pix.height = sensor_fmt.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) != 0) {
        saved_errno = errno; fail_stage = "h264direct_camera_S_FMT_YU12"; goto direct_fail;
    }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
        saved_errno = EINVAL; fail_stage = "h264direct_camera_not_YU12"; goto direct_fail;
    }

    diag_capture_actual_width = fmt.fmt.pix.width;
    diag_capture_actual_height = fmt.fmt.pix.height;
    if (requested_live720) {
        if (fmt.fmt.pix.width < (H264_MEMCROP_LEFT + H264_MEMCROP_W) ||
            fmt.fmt.pix.height < (H264_MEMCROP_TOP + H264_MEMCROP_H)) {
            saved_errno = EINVAL; fail_stage = "h264direct_live720_native_too_small"; goto direct_fail;
        }
        diag_crop_left = H264_MEMCROP_LEFT;
        diag_crop_top = H264_MEMCROP_TOP;
        diag_crop_width = H264_MEMCROP_W;
        diag_crop_height = H264_MEMCROP_H;
        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE720-MEMCROP native capture=%ux%u YU12 -> memory crop left=%ld top=%ld size=%ux%u bytes=%u",
                 (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                 (long)diag_crop_left, (long)diag_crop_top,
                 (unsigned)diag_crop_width, (unsigned)diag_crop_height,
                 (unsigned)H264_MEMCROP_BYTES);
    } else {
        ESP_LOGI(TAG, "H264-DIRECT camera S_FMT actual=%ux%u fourcc=0x%08x sizeimage=%u",
                 (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                 (unsigned)fmt.fmt.pix.pixelformat, (unsigned)fmt.fmt.pix.sizeimage);
    }

    camera_streamparm_30fps_best_effort(cam_fd, sensor_fmt.fps);
    struct timeval timeout = {.tv_sec = CAM_CSI_DQ_TIMEOUT_SEC, .tv_usec = 0};
    (void)ioctl(cam_fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout);

    struct v4l2_requestbuffers rb = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &rb) != 0 || rb.count < 2) {
        saved_errno = errno ? errno : ENOMEM;
        fail_stage = "h264direct_camera_REQBUFS";
        goto direct_fail;
    }
    if (rb.count > CAM_CSI_BUFFER_COUNT) rb.count = CAM_CSI_BUFFER_COUNT;

    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_camera_QUERYBUF"; goto direct_fail;
        }
        maps[i].length = b.length;
        maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, cam_fd, b.m.offset);
        if (!maps[i].start || maps[i].start == MAP_FAILED) {
            maps[i].start = NULL;
            saved_errno = errno ? errno : ENOMEM;
            fail_stage = "h264direct_camera_mmap";
            goto direct_fail;
        }
        mapped_count++;
        if (ioctl(cam_fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_camera_QBUF_initial"; goto direct_fail;
        }
    }

    /* Match the /dev/video11 settings used by the live path. esp_video maps
     * V4L2_PIX_FMT_YUV420 to ESP_H264_RAW_FMT_O_UYY_E_VYY internally, so use
     * the same hardware input format here for an apples-to-apples comparison. */
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
        .gop = requested_gop,
        .fps = (uint8_t)(sensor_fmt.fps ? sensor_fmt.fps : 30U),
        .res = {
            .width = (uint16_t)(requested_live720 ? H264_MEMCROP_W : fmt.fmt.pix.width),
            .height = (uint16_t)(requested_live720 ? H264_MEMCROP_H : fmt.fmt.pix.height),
        },
        .rc = {
            .bitrate = 4000000,
            .qp_min = (uint8_t)((fixed_qp >= 0) ? fixed_qp : 20),
            .qp_max = (uint8_t)((fixed_qp >= 0) ? fixed_qp : 40),
        },
    };

    saved_h264_err = esp_h264_enc_hw_new(&cfg, &enc);
    if (saved_h264_err != ESP_H264_ERR_OK || !enc) {
        fail_stage = "h264direct_enc_hw_new"; goto direct_fail;
    }
    saved_h264_err = esp_h264_enc_open(enc);
    if (saved_h264_err != ESP_H264_ERR_OK) {
        fail_stage = "h264direct_enc_open"; goto direct_fail;
    }
    enc_opened = true;

    h264_clk_open = h264_clock_diag_read();
    h264_clock_diag_log("after-enc-open", &h264_clk_open);
    axi_qos_open = axi_qos_diag_read();
    axi_qos_diag_log("after-enc-open", &axi_qos_open);

    /* [H264-AXI-QOS-AB-20260822]
     * Benchmark-only single-variable A/B.
     *
     * h264qos=0: baseline, no register writes.
     * h264qos=1: raise ONLY the H264 DMA master1/master2 AXI ARQOS/AWQOS
     *             fields from their saved runtime values to 1.
     *
     * DW-GDMA/CSI, DMA2D, CPU, cache and PDMA QoS are untouched.
     * Original H264 QoS values are restored on BOTH success and failure paths.
     * No clock, H264 core register, burst size, task/core, or allocation changes.
     */
    if (requested_h264_qos == 1) {
        h264_qos_saved_m1_r = axi_qos_open.h264_m1_r;
        h264_qos_saved_m1_w = axi_qos_open.h264_m1_w;
        h264_qos_saved_m2_r = axi_qos_open.h264_m2_r;
        h264_qos_saved_m2_w = axi_qos_open.h264_m2_w;

        axi_icm_ll_set_h264_dma_qos_arbiter_prio(0, 1, 1);
        axi_icm_ll_set_h264_dma_qos_arbiter_prio(1, 1, 1);
        h264_qos_applied = true;

        axi_qos_diag_t q = axi_qos_diag_read();
        axi_qos_diag_log("h264-qos=1-applied", &q);
        ESP_LOGW(TAG,
                 "[AXI-QOS-AB] applied H264-only M1/M2 AR/AW=1; saved m1{r=%u,w=%u} m2{r=%u,w=%u}",
                 (unsigned)h264_qos_saved_m1_r, (unsigned)h264_qos_saved_m1_w,
                 (unsigned)h264_qos_saved_m2_r, (unsigned)h264_qos_saved_m2_w);
    }

    /* The V4L2 wrapper allocates its bitstream buffer in PSRAM when SPIRAM is
     * enabled. Use the same memory class and a raw-frame-sized capacity to
     * avoid output overflow influencing the timing result. */
    out_capacity = ((size_t)cfg.res.width * (size_t)cfg.res.height * 3U) / 2U;
    if (requested_out_kb > 0) {
        out_capacity = (size_t)requested_out_kb * 1024U;
    }
    ESP_LOGI(TAG,
             "H264-DIRECT config gop=%u qp=%s%d..%d out_cap=%u bytes (%uKB) camoff=%d camhold=%d camseq=%d lcdpat=%d priab=%d fpsab=%d livegop=%d live720=%d overlap=%d overlapcamab=%d h264qos=%d",
             (unsigned)cfg.gop,
             (fixed_qp >= 0) ? "fixed:" : "rc:",
             (int)cfg.rc.qp_min, (int)cfg.rc.qp_max,
             (unsigned)out_capacity, (unsigned)(out_capacity / 1024U),
             requested_camoff ? 1 : 0,
             requested_camhold ? 1 : 0,
             requested_camseq ? 1 : 0,
             requested_lcdpat ? 1 : 0,
             requested_priab ? 1 : 0,
             requested_fpsab ? 1 : 0,
             requested_livegop ? 1 : 0,
             requested_live720 ? 1 : 0,
             requested_overlap ? 1 : 0,
             requested_overlapcamab ? 1 : 0,
             requested_h264_qos);
    out_buf = (uint8_t *)heap_caps_calloc(
        1, out_capacity,
        MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
    if (!out_buf) {
        saved_errno = ENOMEM; fail_stage = "h264direct_bitstream_alloc"; goto direct_fail;
    }
    if (requested_live720) {
        memcrop_buf = (uint8_t *)heap_caps_malloc(
            H264_MEMCROP_BYTES,
            MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
        if (!memcrop_buf) {
            saved_errno = ENOMEM; fail_stage = "h264direct_live720_memcrop_alloc"; goto direct_fail;
        }
        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE720-MEMCROP buffer allocated=%u bytes in PSRAM",
                 (unsigned)H264_MEMCROP_BYTES);

        if (requested_overlap) {
            memcrop_buf2 = (uint8_t *)heap_caps_malloc(
                H264_MEMCROP_BYTES,
                MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
            if (!memcrop_buf2) {
                saved_errno = ENOMEM; fail_stage = "h264direct_overlap_second_crop_alloc"; goto direct_fail;
            }
            ESP_LOGI(TAG,
                     "H264-DIRECT OVERLAP ping-pong crop buffers ready: 2 x %u bytes PSRAM",
                     (unsigned)H264_MEMCROP_BYTES);
        }

        if (requested_cropab) {
            crop_dma_done = xSemaphoreCreateBinary();
            if (!crop_dma_done) {
                saved_errno = ENOMEM; fail_stage = "h264direct_cropab_semaphore_alloc"; goto direct_fail;
            }
            async_memcpy_config_t dma_cfg = ASYNC_MEMCPY_DEFAULT_CONFIG();
            dma_cfg.backlog = H264_DMA_CROP_BACKLOG;
            dma_cfg.psram_trans_align = H264_DMA_CROP_ALIGN;
            dma_cfg.dma_burst_size = H264_DMA_CROP_ALIGN;
            esp_err_t dma_ret = esp_async_memcpy_install_gdma_axi(&dma_cfg, &crop_dma_hdl);
            if (dma_ret != ESP_OK || !crop_dma_hdl) {
                saved_errno = ENODEV; fail_stage = "h264direct_cropab_axi_gdma_install"; goto direct_fail;
            }
            crop_dma_installed = true;
            ESP_LOGI(TAG,
                     "H264-DIRECT CROPAB AXI-GDMA installed backlog=%u align=%u burst=%u batch_rows=%u",
                     (unsigned)H264_DMA_CROP_BACKLOG, (unsigned)H264_DMA_CROP_ALIGN,
                     (unsigned)H264_DMA_CROP_ALIGN, (unsigned)H264_DMA_CROP_BATCH_ROWS);
        }
        if (requested_ppaab) {
            ppa_verify_buf = (uint8_t *)heap_caps_malloc(
                H264_MEMCROP_BYTES,
                MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
            if (!ppa_verify_buf) {
                saved_errno = ENOMEM; fail_stage = "h264direct_ppaab_verify_alloc"; goto direct_fail;
            }
            ppa_client_config_t ppa_cfg = {
                .oper_type = PPA_OPERATION_SRM,
                .max_pending_trans_num = 1,
                .data_burst_length = PPA_DATA_BURST_LENGTH_128,
            };
            esp_err_t ppa_ret = ppa_register_client(&ppa_cfg, &crop_ppa_client);
            if (ppa_ret != ESP_OK || !crop_ppa_client) {
                saved_errno = ENODEV; fail_stage = "h264direct_ppaab_register_client"; goto direct_fail;
            }
            ESP_LOGI(TAG,
                     "H264-DIRECT PPAAB SRM client ready color=RGB888-reinterpret(O_UYY_E_VYY byte-mover) burst=128; same-frame byte validation will run before benchmark");
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &type) != 0) {
        saved_errno = errno; fail_stage = "h264direct_camera_STREAMON"; goto direct_fail;
    }
    cam_streaming = true;

    /* Camera-only warmup. Do not warm up the encoder because first-IDR cost is
     * part of the same workload that /dev/video11 sees. */
    for (int w = 0; w < 3; ++w) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_DQBUF_warmup"; goto direct_fail;
        }
        if (ioctl(cam_fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_QBUF_warmup"; goto direct_fail;
        }
    }

    if (requested_ppaab) {
        struct v4l2_buffer vb = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(cam_fd, VIDIOC_DQBUF, &vb) != 0) {
            saved_errno = errno; fail_stage = "h264direct_ppaab_validate_DQBUF"; goto direct_fail;
        }
        if (vb.index >= mapped_count || !maps[vb.index].start || vb.bytesused == 0) {
            saved_errno = EFAULT; fail_stage = "h264direct_ppaab_validate_bad_buffer"; goto direct_fail;
        }

        int64_t vc0 = esp_timer_get_time();
        esp_err_t cpu_vret = h264_memcrop_ouyy_evyy(
            (const uint8_t *)maps[vb.index].start, vb.bytesused,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            memcrop_buf, H264_MEMCROP_BYTES);
        int64_t vc1 = esp_timer_get_time();
        ppa_validation_cpu_us = (vc1 > vc0) ? (uint32_t)(vc1 - vc0) : 0U;
        if (cpu_vret != ESP_OK) {
            saved_errno = EIO; fail_stage = "h264direct_ppaab_validate_cpu_crop"; goto direct_fail;
        }

        int64_t vp0 = esp_timer_get_time();
        esp_err_t ppa_vret = h264_memcrop_ouyy_evyy_ppa(
            crop_ppa_client,
            (const uint8_t *)maps[vb.index].start, vb.bytesused,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            ppa_verify_buf, H264_MEMCROP_BYTES);
        int64_t vp1 = esp_timer_get_time();
        ppa_validation_ppa_us = (vp1 > vp0) ? (uint32_t)(vp1 - vp0) : 0U;
        if (ppa_vret != ESP_OK) {
            saved_errno = EIO; fail_stage = "h264direct_ppaab_validate_ppa_crop"; goto direct_fail;
        }

        ppa_validation_cpu_hash = sparse_fnv1a(memcrop_buf, H264_MEMCROP_BYTES);
        ppa_validation_ppa_hash = sparse_fnv1a(ppa_verify_buf, H264_MEMCROP_BYTES);
        ppa_validation_mismatches = 0U;
        ppa_validation_first_mismatch = UINT32_MAX;
        for (uint32_t i = 0; i < H264_MEMCROP_BYTES; ++i) {
            if (memcrop_buf[i] != ppa_verify_buf[i]) {
                if (ppa_validation_first_mismatch == UINT32_MAX) {
                    ppa_validation_first_mismatch = i;
                }
                ppa_validation_mismatches++;
            }
        }
        ppa_validation_done = true;

        ESP_LOGI(TAG,
                 "H264-DIRECT PPAAB VALIDATE same-frame cpu_us=%u ppa_us=%u cpu_hash=%08" PRIx32 " ppa_hash=%08" PRIx32 " mismatches=%u first=%u",
                 (unsigned)ppa_validation_cpu_us, (unsigned)ppa_validation_ppa_us,
                 ppa_validation_cpu_hash, ppa_validation_ppa_hash,
                 (unsigned)ppa_validation_mismatches,
                 (unsigned)(ppa_validation_first_mismatch == UINT32_MAX ? 0U : ppa_validation_first_mismatch));

        if (ioctl(cam_fd, VIDIOC_QBUF, &vb) != 0) {
            saved_errno = errno; fail_stage = "h264direct_ppaab_validate_QBUF"; goto direct_fail;
        }

        if (ppa_validation_mismatches != 0U) {
            saved_errno = EILSEQ;
            fail_stage = "h264direct_ppaab_validate_bytes_mismatch";
            goto direct_fail;
        }
        ESP_LOGI(TAG,
                 "H264-DIRECT PPAAB VALIDATE PASS byte-identical=%u bytes; starting controlled CPU120->PPA120 benchmark",
                 (unsigned)H264_MEMCROP_BYTES);
    }

    if (requested_overlap && requested_overlapcamab) {
        /* Controlled same-boot camera-memory-traffic A/B using the exact same
         * held 1920x1080 raw source in both phases. */
        overlap_ctx.cam_fd = cam_fd;
        overlap_ctx.maps = maps;
        overlap_ctx.mapped_count = mapped_count;
        overlap_ctx.src_w = fmt.fmt.pix.width;
        overlap_ctx.src_h = fmt.fmt.pix.height;
        overlap_ctx.fixed_src_mode = true;
        overlap_ctx.kick_sem = xSemaphoreCreateBinary();
        overlap_ctx.done_sem = xSemaphoreCreateBinary();
        overlap_ctx.exit_sem = xSemaphoreCreateBinary();
        if (!overlap_ctx.kick_sem || !overlap_ctx.done_sem || !overlap_ctx.exit_sem) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlapcamab_semaphore_alloc"; goto direct_fail;
        }
        UBaseType_t owner_prio = uxTaskPriorityGet(NULL);
        UBaseType_t overlap_prio = (owner_prio > 1U) ? (owner_prio - 1U) : 1U;
        BaseType_t task_ok = xTaskCreatePinnedToCore(
            h264_overlap_crop_task, "h264_crop_ab", H264_OVERLAP_TASK_STACK,
            &overlap_ctx, overlap_prio, &overlap_ctx.task, H264_OVERLAP_TASK_CORE);
        if (task_ok != pdPASS || !overlap_ctx.task) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlapcamab_crop_task_create"; goto direct_fail;
        }
        overlap_worker_started = true;

        struct v4l2_buffer ab_held = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        int64_t hd0 = esp_timer_get_time();
        if (ioctl(cam_fd, VIDIOC_DQBUF, &ab_held) != 0) {
            saved_errno = errno; fail_stage = "h264direct_overlapcamab_hold_DQBUF"; goto direct_fail;
        }
        int64_t hd1 = esp_timer_get_time();
        overlap_prime_dq_us = (hd1 > hd0) ? (uint32_t)(hd1 - hd0) : 0U;
        if (ab_held.index >= mapped_count || !maps[ab_held.index].start || ab_held.bytesused == 0) {
            saved_errno = EFAULT; fail_stage = "h264direct_overlapcamab_hold_bad_buffer"; goto direct_fail;
        }
        held_b = ab_held;
        held_valid = true;
        overlap_ctx.fixed_src = (const uint8_t *)maps[ab_held.index].start;
        overlap_ctx.fixed_src_len = ab_held.bytesused;
        overlap_camab_held_hash_initial = sparse_fnv1a(overlap_ctx.fixed_src, overlap_ctx.fixed_src_len);
        held_raw_hash = overlap_camab_held_hash_initial;

        int64_t pc0 = esp_timer_get_time();
        esp_err_t prime_crop_ret = h264_memcrop_ouyy_evyy(
            overlap_ctx.fixed_src, overlap_ctx.fixed_src_len,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            memcrop_buf, H264_MEMCROP_BYTES);
        int64_t pc1 = esp_timer_get_time();
        overlap_prime_crop_us = (pc1 > pc0) ? (uint32_t)(pc1 - pc0) : 0U;
        if (prime_crop_ret != ESP_OK) {
            saved_errno = EIO; fail_stage = "h264direct_overlapcamab_prime_crop"; goto direct_fail;
        }

        overlap_pump_ctx.cam_fd = cam_fd;
        overlap_pump_ctx.exit_sem = xSemaphoreCreateBinary();
        if (!overlap_pump_ctx.exit_sem) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlapcamab_pump_semaphore"; goto direct_fail;
        }
        /* V5 proved that owner+1 (prio 6 here) is too aggressive on Core0:
         * it can remove the remaining IDLE0 slots and trigger task-WDT.
         * Use the SAME priority as the HTTP owner. DQBUF normally blocks until
         * the next ~30 fps camera frame, so equal-priority time slicing is
         * sufficient to recycle completed buffers without starving Core0.
         * Core1 remains untouched. */
        UBaseType_t pump_prio = owner_prio;

        /* The normal diagnostic DQBUF timeout is 3 s. STREAMOFF does not
         * immediately wake esp_video_recv_element() on this IDF/esp_video path,
         * so V5's 1 s join timed out while the pump was still inside DQBUF.
         * Shorten only this CAMAB pump phase to 250 ms; normal timeout expiry is
         * treated as a retry by the pump task. */
        struct timeval pump_timeout = {
            .tv_sec = 0,
            .tv_usec = 250000,
        };
        if (ioctl(cam_fd, VIDIOC_S_DQBUF_TIMEOUT, &pump_timeout) != 0) {
            ESP_LOGW(TAG, "H264-DIRECT CAMAB pump DQBUF timeout=250ms set failed errno=%d", errno);
        }

        BaseType_t pump_ok = xTaskCreatePinnedToCore(
            h264_overlap_cam_pump_task, "h264_cam_pump", 4096U,
            &overlap_pump_ctx, pump_prio, &overlap_pump_ctx.task, H264_OVERLAP_TASK_CORE);
        if (pump_ok != pdPASS || !overlap_pump_ctx.task) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlapcamab_pump_task_create"; goto direct_fail;
        }
        overlap_pump_started = true;

        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE720-OVERLAP-CAMAB start held_idx=%u bytes=%u raw_hash=%08" PRIx32
                 " CAM_ON=%u CAM_OFF=%u core=%u prio=%u/%u/%u prime{dq=%u,crop=%u}",
                 (unsigned)ab_held.index, (unsigned)ab_held.bytesused, overlap_camab_held_hash_initial,
                 (unsigned)H264_OVERLAP_CAMAB_PHASE_FRAMES, (unsigned)H264_OVERLAP_CAMAB_PHASE_FRAMES,
                 (unsigned)H264_OVERLAP_TASK_CORE, (unsigned)owner_prio, (unsigned)overlap_prio,
                 (unsigned)pump_prio, (unsigned)overlap_prime_dq_us, (unsigned)overlap_prime_crop_us);

        for (uint32_t phase = 0; phase < 2U; ++phase) {
            uint8_t *cur_crop = memcrop_buf;
            uint8_t *next_crop = memcrop_buf2;
            overlap_camab_phase_start[phase] = esp_timer_get_time();
            for (uint32_t n = 0; n < H264_OVERLAP_CAMAB_PHASE_FRAMES; ++n) {
                bool launch_next = (n + 1U) < H264_OVERLAP_CAMAB_PHASE_FRAMES;
                if (launch_next) {
                    overlap_ctx.dst = next_crop;
                    overlap_ctx.dst_len = H264_MEMCROP_BYTES;
                    overlap_ctx.status = H264_OVERLAP_JOB_OK;
                    overlap_ctx.crop_ret = ESP_OK;
                    overlap_ctx.io_errno = 0;
                    if (xSemaphoreGive(overlap_ctx.kick_sem) != pdTRUE) {
                        saved_errno = EBUSY; fail_stage = "h264direct_overlapcamab_kick"; goto direct_fail;
                    }
                }

                uint32_t global_n = phase * H264_OVERLAP_CAMAB_PHASE_FRAMES + n;
                esp_h264_enc_in_frame_t in_frame = {
                    .raw_data = { .buffer = cur_crop, .len = H264_MEMCROP_BYTES },
                    .pts = global_n * 33U,
                };
                esp_h264_enc_out_frame_t out_frame = {
                    .raw_data = { .buffer = out_buf, .len = (uint32_t)out_capacity },
                };
                int64_t e0 = esp_timer_get_time();
                saved_h264_err = esp_h264_enc_process(enc, &in_frame, &out_frame);
                int64_t e1 = esp_timer_get_time();
                uint32_t eus = (e1 > e0) ? (uint32_t)(e1 - e0) : 0U;
                if (saved_h264_err != ESP_H264_ERR_OK) {
                    fail_stage = "h264direct_overlapcamab_enc_process"; goto direct_fail;
                }
                if (out_frame.length == 0 || out_frame.length > out_capacity) {
                    saved_errno = EIO; fail_stage = "h264direct_overlapcamab_empty_output"; goto direct_fail;
                }

                uint32_t cus = 0U, wus = 0U;
                if (launch_next) {
                    int64_t w0 = esp_timer_get_time();
                    if (xSemaphoreTake(overlap_ctx.done_sem, pdMS_TO_TICKS(4000)) != pdTRUE) {
                        saved_errno = ETIMEDOUT; fail_stage = "h264direct_overlapcamab_worker_timeout"; goto direct_fail;
                    }
                    int64_t w1 = esp_timer_get_time();
                    wus = (w1 > w0) ? (uint32_t)(w1 - w0) : 0U;
                    if (overlap_ctx.status != H264_OVERLAP_JOB_OK) {
                        saved_errno = overlap_ctx.io_errno ? overlap_ctx.io_errno : EIO;
                        fail_stage = "h264direct_overlapcamab_worker_crop"; goto direct_fail;
                    }
                    cus = overlap_ctx.crop_us;
                    overlap_camab_crop_total[phase] += cus;
                    if (cus < overlap_camab_crop_min[phase]) overlap_camab_crop_min[phase] = cus;
                    if (cus > overlap_camab_crop_max[phase]) overlap_camab_crop_max[phase] = cus;
                    overlap_camab_wait_total[phase] += wus;
                    overlap_camab_wait_count[phase]++;
                    if (wus > overlap_camab_wait_max[phase]) overlap_camab_wait_max[phase] = wus;
                }

                overlap_camab_enc_total[phase] += eus;
                if (eus < overlap_camab_enc_min[phase]) overlap_camab_enc_min[phase] = eus;
                if (eus > overlap_camab_enc_max[phase]) overlap_camab_enc_max[phase] = eus;
                enc_us_total += eus;
                enc_us_last = eus;
                if (eus < enc_us_min) enc_us_min = eus;
                if (eus > enc_us_max) enc_us_max = eus;

                uint32_t used = out_frame.length;
                uint32_t hash = sparse_fnv1a(out_buf, used);
                if (n == 0U) overlap_camab_hash_first[phase] = hash;
                overlap_camab_hash_last[phase] = hash;
                if (frames == 0U) { t_first = e1; hash_first = hash; }
                t_last = e1; hash_last = hash; frames++; overlap_camab_frames[phase]++;
                bytes_total += used;
                if (used < bytes_min) bytes_min = used;
                if (used > bytes_max) bytes_max = used;
                if (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR) { overlap_camab_idr[phase]++; idr_frames++; }
                else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_I) { overlap_camab_i[phase]++; i_frames++; }
                else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_P) { overlap_camab_p[phase]++; p_frames++; }
                h264_scan_annexb(out_buf, used, &nal);

                if (n < 3U || ((n + 1U) % 20U) == 0U) {
                    ESP_LOGI(TAG,
                             "H264-DIRECT CAMAB phase=%s frame=%u enc_us=%u crop_us=%u wait_us=%u h264=%u type=%d hash=%08" PRIx32,
                             phase == 0U ? "CAM_ON" : "CAM_OFF", (unsigned)(n + 1U),
                             (unsigned)eus, (unsigned)cus, (unsigned)wus,
                             (unsigned)used, (int)out_frame.frame_type, hash);
                }
                if (launch_next) { uint8_t *tmp = cur_crop; cur_crop = next_crop; next_crop = tmp; }
            }
            overlap_camab_phase_end[phase] = esp_timer_get_time();

            if (phase == 0U) {
                overlap_camab_held_hash_after_on = sparse_fnv1a(overlap_ctx.fixed_src, overlap_ctx.fixed_src_len);
                overlap_pump_ctx.stop = true;
                int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (ioctl(cam_fd, VIDIOC_STREAMOFF, &t) != 0) {
                    saved_errno = errno; fail_stage = "h264direct_overlapcamab_STREAMOFF"; goto direct_fail;
                }
                cam_streaming = false;
                if (xSemaphoreTake(overlap_pump_ctx.exit_sem, pdMS_TO_TICKS(1500)) != pdTRUE) {
                    saved_errno = ETIMEDOUT; fail_stage = "h264direct_overlapcamab_pump_exit"; goto direct_fail;
                }
                overlap_pump_started = false;
                overlap_pump_ctx.task = NULL;
                if (overlap_pump_ctx.failed) {
                    saved_errno = overlap_pump_ctx.io_errno ? overlap_pump_ctx.io_errno : EIO;
                    fail_stage = "h264direct_overlapcamab_pump_failed"; goto direct_fail;
                }
                overlap_camab_pump_frames = overlap_pump_ctx.frames;
                overlap_camab_pump_dq_total = overlap_pump_ctx.dq_us_total;
                overlap_camab_pump_dq_max = overlap_pump_ctx.dq_us_max;
                uint32_t pump_fps_x100_now = 0U;
                int64_t phase_on_us = overlap_camab_phase_end[0] - overlap_camab_phase_start[0];
                if (phase_on_us > 0) {
                    pump_fps_x100_now = (uint32_t)(((uint64_t)overlap_camab_pump_frames * 100000000ULL) /
                                                   (uint64_t)phase_on_us);
                }
                ESP_LOGI(TAG,
                         "H264-DIRECT CAMAB phaseA done -> STREAMOFF pump_frames=%u pump_fps=%u.%02u sustained=%d dq_avg/max=%u/%u held_hash=%08" PRIx32,
                         (unsigned)overlap_camab_pump_frames,
                         (unsigned)(pump_fps_x100_now / 100U), (unsigned)(pump_fps_x100_now % 100U),
                         overlap_camab_pump_frames >= H264_OVERLAP_CAMAB_MIN_PUMP_FRAMES ? 1 : 0,
                         (unsigned)(overlap_camab_pump_frames ? overlap_camab_pump_dq_total / overlap_camab_pump_frames : 0U),
                         (unsigned)overlap_camab_pump_dq_max, overlap_camab_held_hash_after_on);
            }
        }

        overlap_camab_held_hash_after_off = sparse_fnv1a(overlap_ctx.fixed_src, overlap_ctx.fixed_src_len);
        overlap_ctx.stop = true;
        (void)xSemaphoreGive(overlap_ctx.kick_sem);
        if (xSemaphoreTake(overlap_ctx.exit_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            saved_errno = ETIMEDOUT; fail_stage = "h264direct_overlapcamab_crop_worker_exit"; goto direct_fail;
        }
        overlap_worker_started = false;
        overlap_ctx.task = NULL;
        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE720-OVERLAP-CAMAB worker stopped raw_hash=%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32,
                 overlap_camab_held_hash_initial, overlap_camab_held_hash_after_on, overlap_camab_held_hash_after_off);
    } else if (requested_overlap) {
        /* Prime frame 0 synchronously.  Thereafter the worker obtains frame
         * N+1 and crops it into the alternate PSRAM buffer while H.264 HW is
         * encoding frame N.  No worker ever writes the buffer currently owned
         * by the H.264 DMA engine. */
        overlap_ctx.cam_fd = cam_fd;
        overlap_ctx.maps = maps;
        overlap_ctx.mapped_count = mapped_count;
        overlap_ctx.src_w = fmt.fmt.pix.width;
        overlap_ctx.src_h = fmt.fmt.pix.height;
        overlap_ctx.kick_sem = xSemaphoreCreateBinary();
        overlap_ctx.done_sem = xSemaphoreCreateBinary();
        overlap_ctx.exit_sem = xSemaphoreCreateBinary();
        if (!overlap_ctx.kick_sem || !overlap_ctx.done_sem || !overlap_ctx.exit_sem) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlap_semaphore_alloc"; goto direct_fail;
        }

        UBaseType_t owner_prio = uxTaskPriorityGet(NULL);
        UBaseType_t overlap_prio = (owner_prio > 1U) ? (owner_prio - 1U) : 1U;
        BaseType_t task_ok = xTaskCreatePinnedToCore(
            h264_overlap_crop_task, "h264_crop_overlap",
            H264_OVERLAP_TASK_STACK, &overlap_ctx, overlap_prio,
            &overlap_ctx.task, H264_OVERLAP_TASK_CORE);
        if (task_ok != pdPASS || !overlap_ctx.task) {
            saved_errno = ENOMEM; fail_stage = "h264direct_overlap_task_create"; goto direct_fail;
        }
        overlap_worker_started = true;

        struct v4l2_buffer prime_b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        int64_t pdq0 = esp_timer_get_time();
        if (ioctl(cam_fd, VIDIOC_DQBUF, &prime_b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_overlap_prime_DQBUF"; goto direct_fail;
        }
        int64_t pdq1 = esp_timer_get_time();
        overlap_prime_dq_us = (pdq1 > pdq0) ? (uint32_t)(pdq1 - pdq0) : 0U;
        if (prime_b.index >= mapped_count || !maps[prime_b.index].start || prime_b.bytesused == 0) {
            saved_errno = EFAULT; fail_stage = "h264direct_overlap_prime_bad_buffer"; goto direct_fail;
        }
        int64_t pc0 = esp_timer_get_time();
        esp_err_t prime_crop_ret = h264_memcrop_ouyy_evyy(
            (const uint8_t *)maps[prime_b.index].start, prime_b.bytesused,
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            memcrop_buf, H264_MEMCROP_BYTES);
        int64_t pc1 = esp_timer_get_time();
        overlap_prime_crop_us = (pc1 > pc0) ? (uint32_t)(pc1 - pc0) : 0U;
        if (prime_crop_ret != ESP_OK) {
            (void)ioctl(cam_fd, VIDIOC_QBUF, &prime_b);
            saved_errno = EIO; fail_stage = "h264direct_overlap_prime_crop"; goto direct_fail;
        }
        if (ioctl(cam_fd, VIDIOC_QBUF, &prime_b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_overlap_prime_QBUF"; goto direct_fail;
        }

        dq_us_total += overlap_prime_dq_us;
        dq_us_max = overlap_prime_dq_us;
        crop_us_total += overlap_prime_crop_us;
        crop_us_min = overlap_prime_crop_us;
        crop_us_max = overlap_prime_crop_us;

        uint8_t *cur_crop = memcrop_buf;
        uint8_t *next_crop = memcrop_buf2;
        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE720-OVERLAP start frames=%u worker_core=%u owner_prio=%u worker_prio=%u prime{dq=%u,crop=%u} crop+encode ping-pong; Core1 untouched",
                 (unsigned)H264_OVERLAP_FRAMES, (unsigned)H264_OVERLAP_TASK_CORE,
                 (unsigned)owner_prio, (unsigned)overlap_prio, (unsigned)overlap_prime_dq_us,
                 (unsigned)overlap_prime_crop_us);

        for (uint32_t n = 0; n < H264_OVERLAP_FRAMES; ++n) {
            bool launch_next = (n + 1U) < H264_OVERLAP_FRAMES;
            if (launch_next) {
                overlap_ctx.dst = next_crop;
                overlap_ctx.dst_len = H264_MEMCROP_BYTES;
                overlap_ctx.status = H264_OVERLAP_JOB_OK;
                overlap_ctx.crop_ret = ESP_OK;
                overlap_ctx.io_errno = 0;
                if (xSemaphoreGive(overlap_ctx.kick_sem) != pdTRUE) {
                    saved_errno = EBUSY; fail_stage = "h264direct_overlap_kick"; goto direct_fail;
                }
            }

            esp_h264_enc_in_frame_t in_frame = {
                .raw_data = {
                    .buffer = cur_crop,
                    .len = H264_MEMCROP_BYTES,
                },
                .pts = n * 33U,
            };
            esp_h264_enc_out_frame_t out_frame = {
                .raw_data = {
                    .buffer = out_buf,
                    .len = (uint32_t)out_capacity,
                },
            };

            int64_t e0 = esp_timer_get_time();
            saved_h264_err = esp_h264_enc_process(enc, &in_frame, &out_frame);
            int64_t e1 = esp_timer_get_time();
            enc_us_last = (e1 > e0) ? (uint32_t)(e1 - e0) : 0U;
            if (saved_h264_err != ESP_H264_ERR_OK) {
                fail_stage = "h264direct_overlap_enc_process";
                goto direct_fail;
            }
            if (out_frame.length == 0 || out_frame.length > out_capacity) {
                saved_errno = EIO; fail_stage = "h264direct_overlap_empty_output"; goto direct_fail;
            }

            uint32_t next_dq_us = 0U;
            uint32_t next_crop_us = 0U;
            uint32_t wait_us = 0U;
            if (launch_next) {
                int64_t w0 = esp_timer_get_time();
                if (xSemaphoreTake(overlap_ctx.done_sem, pdMS_TO_TICKS(4000)) != pdTRUE) {
                    saved_errno = ETIMEDOUT; fail_stage = "h264direct_overlap_worker_timeout"; goto direct_fail;
                }
                int64_t w1 = esp_timer_get_time();
                wait_us = (w1 > w0) ? (uint32_t)(w1 - w0) : 0U;
                if (overlap_ctx.status != H264_OVERLAP_JOB_OK) {
                    saved_errno = overlap_ctx.io_errno ? overlap_ctx.io_errno : EIO;
                    if (overlap_ctx.status == H264_OVERLAP_JOB_DQBUF) fail_stage = "h264direct_overlap_worker_DQBUF";
                    else if (overlap_ctx.status == H264_OVERLAP_JOB_BAD_BUFFER) fail_stage = "h264direct_overlap_worker_bad_buffer";
                    else if (overlap_ctx.status == H264_OVERLAP_JOB_CROP) fail_stage = "h264direct_overlap_worker_crop";
                    else fail_stage = "h264direct_overlap_worker_QBUF";
                    goto direct_fail;
                }
                next_dq_us = overlap_ctx.dq_us;
                next_crop_us = overlap_ctx.crop_us;
                dq_us_total += next_dq_us;
                if (next_dq_us > dq_us_max) dq_us_max = next_dq_us;
                crop_us_total += next_crop_us;
                if (next_crop_us < crop_us_min) crop_us_min = next_crop_us;
                if (next_crop_us > crop_us_max) crop_us_max = next_crop_us;
                overlap_wait_total += wait_us;
                overlap_wait_count++;
                if (wait_us > overlap_wait_max) overlap_wait_max = wait_us;
            }

            uint32_t used = out_frame.length;
            uint32_t hash = sparse_fnv1a(out_buf, used);
            if (frames == 0U) {
                t_first = e1;
                hash_first = hash;
            }
            t_last = e1;
            hash_last = hash;
            frames++;
            enc_us_total += enc_us_last;
            if (enc_us_last < enc_us_min) enc_us_min = enc_us_last;
            if (enc_us_last > enc_us_max) enc_us_max = enc_us_last;
            bytes_total += used;
            if (used < bytes_min) bytes_min = used;
            if (used > bytes_max) bytes_max = used;

            if (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR) {
                idr_frames++;
                idr_us_total += enc_us_last;
                idr_bytes_total += used;
                if (enc_us_last < idr_us_min) idr_us_min = enc_us_last;
                if (enc_us_last > idr_us_max) idr_us_max = enc_us_last;
                if (used < idr_bytes_min) idr_bytes_min = used;
                if (used > idr_bytes_max) idr_bytes_max = used;
            } else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_I) {
                i_frames++;
                i_us_total += enc_us_last;
                i_bytes_total += used;
                if (enc_us_last < i_us_min) i_us_min = enc_us_last;
                if (enc_us_last > i_us_max) i_us_max = enc_us_last;
                if (used < i_bytes_min) i_bytes_min = used;
                if (used > i_bytes_max) i_bytes_max = used;
            } else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_P) {
                p_frames++;
                p_us_total += enc_us_last;
                p_bytes_total += used;
                if (enc_us_last < p_us_min) p_us_min = enc_us_last;
                if (enc_us_last > p_us_max) p_us_max = enc_us_last;
                if (used < p_bytes_min) p_bytes_min = used;
                if (used > p_bytes_max) p_bytes_max = used;
            }
            h264_scan_annexb(out_buf, used, &nal);

            if (frames <= 3U || (frames % 10U) == 0U) {
                ESP_LOGI(TAG,
                         "H264-DIRECT frame=%u phase=CPU_H264_OVERLAP h264=%u enc_us=%u next_dq_us=%u next_crop_us=%u post_encode_wait_us=%u type=%d hash=%08" PRIx32,
                         (unsigned)frames, (unsigned)used, (unsigned)enc_us_last,
                         (unsigned)next_dq_us, (unsigned)next_crop_us,
                         (unsigned)wait_us, (int)out_frame.frame_type, hash);
            }

            if (launch_next) {
                uint8_t *tmp = cur_crop;
                cur_crop = next_crop;
                next_crop = tmp;
            }
        }

        overlap_ctx.stop = true;
        (void)xSemaphoreGive(overlap_ctx.kick_sem);
        if (xSemaphoreTake(overlap_ctx.exit_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            saved_errno = ETIMEDOUT; fail_stage = "h264direct_overlap_worker_exit"; goto direct_fail;
        }
        overlap_worker_started = false;
        overlap_ctx.task = NULL;
        ESP_LOGI(TAG, "H264-DIRECT LIVE720-OVERLAP worker stopped cleanly");
    }

    /* [H264-CAMERA-DMA-AB-20260822]
     * camoff=1: hold one real frame and STREAMOFF camera.
     * camhold=1: hold that exact same real frame but KEEP camera streaming.
     *              The remaining queued camera buffers are pumped once per
     *              encode iteration so CSI/ISP/camera DMA keeps running.
     *
     * camhold therefore controls image-content effects: H264 repeatedly sees
     * one identical YU12 frame in both camoff and camhold modes, while only
     * concurrent camera DMA activity changes.
     *
     * lcdpat=1: hold one real frame, STREAMOFF camera before either phase,
     * encode 60 frames with the normal MIPI-DSI/LVGL display running, then
     * switch framebuffer scanout to the DSI hardware pattern generator and encode the SAME raw frame for 60 more.
     * This isolates total LCD/MIPI-DSI scanout contention. No 3.1 MB copy is allocated. */
    if (requested_camoff || requested_camhold || requested_camseq || requested_lcdpat || requested_priab || requested_fpsab) {
        int64_t dq0 = esp_timer_get_time();
        if (ioctl(cam_fd, VIDIOC_DQBUF, &held_b) != 0) {
            saved_errno = errno; fail_stage = "h264direct_hold_DQBUF"; goto direct_fail;
        }
        int64_t dq1 = esp_timer_get_time();
        dq_us_last = (dq1 > dq0) ? (uint32_t)(dq1 - dq0) : 0;
        if (held_b.index >= mapped_count || !maps[held_b.index].start || held_b.bytesused == 0) {
            saved_errno = EFAULT; fail_stage = "h264direct_hold_bad_buffer"; goto direct_fail;
        }
        held_valid = true;
        held_raw_hash = sparse_fnv1a((const uint8_t *)maps[held_b.index].start, held_b.bytesused);
        ESP_LOGI(TAG, "H264-DIRECT held raw hash=%08" PRIx32 " idx=%u bytes=%u",
                 held_raw_hash, (unsigned)held_b.index, (unsigned)held_b.bytesused);

        if (requested_camoff || requested_lcdpat || requested_priab || requested_fpsab) {
            int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(cam_fd, VIDIOC_STREAMOFF, &t) != 0) {
                saved_errno = errno;
                fail_stage = requested_lcdpat ? "h264direct_lcdpat_camera_STREAMOFF" :
                             (requested_priab ? "h264direct_priab_camera_STREAMOFF" :
                              (requested_fpsab ? "h264direct_fpsab_camera_STREAMOFF" : "h264direct_camoff_STREAMOFF"));
                goto direct_fail;
            }
            cam_streaming = false;
            if (requested_lcdpat || requested_priab || requested_fpsab) {
                lcdpat_display = lv_display_get_default();
                if (!lcdpat_display) {
                    saved_errno = ENODEV;
                    fail_stage = requested_priab ? "h264direct_priab_no_default_display" :
                                 (requested_fpsab ? "h264direct_fpsab_no_default_display" : "h264direct_lcdpat_no_default_display");
                    goto direct_fail;
                }
                /* Reuse the existing LCD-camera guard so the screen-shift task
                 * will not intentionally redraw during the diagnostic. */
                lcd_ui_set_camera_stream_active(true);
                if (requested_priab) {
                    priab_task = xTaskGetCurrentTaskHandle();
                    priab_prio_normal = uxTaskPriorityGet(priab_task);
                    priab_prio_high = priab_prio_normal + 3U;
                    if (priab_prio_high >= (UBaseType_t)(configMAX_PRIORITIES - 1)) {
                        priab_prio_high = (UBaseType_t)(configMAX_PRIORITIES - 2);
                    }
                    if (priab_prio_high <= priab_prio_normal) {
                        saved_errno = ERANGE; fail_stage = "h264direct_priab_no_priority_headroom"; goto direct_fail;
                    }
                    esp_err_t lcd_pattern_err = camera_diag_lcd_pattern_enter(
                        lcdpat_display, &lcdpat_panel, &lcdpat_lvgl_timer_paused);
                    if (lcd_pattern_err != ESP_OK) {
                        saved_errno = EIO; fail_stage = "h264direct_priab_pattern_enter"; goto direct_fail;
                    }
                    lcdpat_active = true;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ESP_LOGI(TAG,
                             "H264-DIRECT PRIAB: camera STREAMOFF + DSI pattern active for BOTH priority conditions; "
                             "order=NORMAL30-HIGH60-NORMAL30 task=%p prio_normal=%u prio_high=%u raw_hash=%08" PRIx32,
                             (void *)priab_task, (unsigned)priab_prio_normal, (unsigned)priab_prio_high, held_raw_hash);
                } else if (requested_fpsab) {
                    esp_err_t lcd_pattern_err = camera_diag_lcd_pattern_enter(
                        lcdpat_display, &lcdpat_panel, &lcdpat_lvgl_timer_paused);
                    if (lcd_pattern_err != ESP_OK) {
                        saved_errno = EIO; fail_stage = "h264direct_fpsab_pattern_enter"; goto direct_fail;
                    }
                    lcdpat_active = true;
                    vTaskDelay(pdMS_TO_TICKS(100));
                    ESP_LOGI(TAG,
                             "H264-DIRECT FPSAB: camera STREAMOFF + DSI pattern active for ALL phases; "
                             "order=FPS30x30-FPS60x60-FPS30x30 cfg.fps-only A/B pts_step_ms=33 raw_hash=%08" PRIx32,
                             held_raw_hash);
                } else {
                    ESP_LOGI(TAG,
                             "H264-DIRECT LCDPAT: camera STREAMOFF before both phases; phase1 LCDON starts; display=%p raw_hash=%08" PRIx32,
                             (void *)lcdpat_display, held_raw_hash);
                }
            } else {
                ESP_LOGI(TAG,
                         "H264-DIRECT CAMOFF A/B: camera STREAMOFF; hold idx=%u bytes=%u dq_us=%u",
                         (unsigned)held_b.index, (unsigned)held_b.bytesused, (unsigned)dq_us_last);
            }
        } else if (requested_camseq) {
            ESP_LOGI(TAG,
                     "H264-DIRECT SAMEBOOT A/B: phase1 CAMHOLD starts; same held input idx=%u bytes=%u raw_hash=%08" PRIx32,
                     (unsigned)held_b.index, (unsigned)held_b.bytesused, held_raw_hash);
        } else {
            ESP_LOGI(TAG,
                     "H264-DIRECT CAMHOLD A/B: camera remains STREAMON; same held input idx=%u bytes=%u, other buffers pumped",
                     (unsigned)held_b.index, (unsigned)held_b.bytesused);
        }
    }

    const uint32_t target_frames = requested_overlap ? H264_OVERLAP_FRAMES :
        ((requested_cropab || requested_ppaab) ? (H264_CROP_AB_PHASE_FRAMES * 2U) :
        ((requested_livegop || requested_live720) ? 120U :
         ((requested_camseq || requested_lcdpat || requested_priab || requested_fpsab) ? 120U : 60U)));
    if (requested_livegop) {
        ESP_LOGI(TAG,
                 "H264-DIRECT LIVE-GOP30 FINAL: 1920x1080 fresh camera DQBUF every frame, STREAMON throughout, GOP=30 QP=%d, frames=%u",
                 (int)cfg.rc.qp_min, (unsigned)target_frames);
    } else if (requested_live720) {
        if (requested_overlap) {
            ESP_LOGI(TAG,
                     "H264-DIRECT LIVE720-OVERLAP benchmark already executed: CPU crop of frame N+1 on Core0 while H264 HW encodes frame N; frames=%u",
                     (unsigned)target_frames);
        } else if (requested_ppaab) {
            ESP_LOGI(TAG,
                     "H264-DIRECT LIVE720-PPAAB: capture=%ux%u crop=%ld,%ld %ux%u encoder=%ux%u validation=byte-identical same-boot order=CPU120->PPA_SRM120 fresh DQBUF, STREAMON throughout, GOP=30 QP=%d, frames=%u",
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     (long)diag_crop_left, (long)diag_crop_top,
                     (unsigned)diag_crop_width, (unsigned)diag_crop_height,
                     (unsigned)cfg.res.width, (unsigned)cfg.res.height,
                     (int)cfg.rc.qp_min, (unsigned)target_frames);
        } else if (requested_cropab) {
            ESP_LOGI(TAG,
                     "H264-DIRECT LIVE720-CROPAB: capture=%ux%u crop=%ld,%ld %ux%u encoder=%ux%u same-boot order=CPU120->AXI_GDMA120 fresh DQBUF, STREAMON throughout, GOP=30 QP=%d, frames=%u",
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     (long)diag_crop_left, (long)diag_crop_top,
                     (unsigned)diag_crop_width, (unsigned)diag_crop_height,
                     (unsigned)cfg.res.width, (unsigned)cfg.res.height,
                     (int)cfg.rc.qp_min, (unsigned)target_frames);
        } else {
            ESP_LOGI(TAG,
                     "H264-DIRECT LIVE720-MEMCROP-GOP30: capture=%ux%u crop=%ld,%ld %ux%u encoder=%ux%u fresh DQBUF + row-copy each frame, STREAMON throughout, GOP=30 QP=%d, frames=%u",
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     (long)diag_crop_left, (long)diag_crop_top,
                     (unsigned)diag_crop_width, (unsigned)diag_crop_height,
                     (unsigned)cfg.res.width, (unsigned)cfg.res.height,
                     (int)cfg.rc.qp_min, (unsigned)target_frames);
        }
    }
    if (!requested_overlap) {
    for (uint32_t n = 0; n < target_frames; ++n) {
        uint32_t cropab_phase = ((requested_cropab || requested_ppaab) && n >= H264_CROP_AB_PHASE_FRAMES) ? 1U : 0U;
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (requested_camoff || requested_camhold || requested_camseq || requested_lcdpat || requested_priab || requested_fpsab) {
            if (!held_valid) {
                saved_errno = EFAULT; fail_stage = "h264direct_hold_not_held"; goto direct_fail;
            }
            b = held_b;
            dq_us_last = 0;
        } else {
            int64_t dq0 = esp_timer_get_time();
            if (ioctl(cam_fd, VIDIOC_DQBUF, &b) != 0) {
                saved_errno = errno; fail_stage = "h264direct_camera_DQBUF"; goto direct_fail;
            }
            int64_t dq1 = esp_timer_get_time();
            dq_us_last = (dq1 > dq0) ? (uint32_t)(dq1 - dq0) : 0;
            dq_us_total += dq_us_last;
            if (dq_us_last > dq_us_max) dq_us_max = dq_us_last;
        }

        if (b.index >= mapped_count || !maps[b.index].start || b.bytesused == 0) {
            saved_errno = EFAULT; fail_stage = "h264direct_camera_bad_buffer"; goto direct_fail;
        }

        uint8_t *h264_input = (uint8_t *)maps[b.index].start;
        uint32_t h264_input_len = b.bytesused;
        crop_us_last = 0;
        if (requested_live720) {
            int64_t c0 = esp_timer_get_time();
            esp_err_t crop_ret;
            if (requested_ppaab && cropab_phase == 1U) {
                crop_ret = h264_memcrop_ouyy_evyy_ppa(
                    crop_ppa_client,
                    (const uint8_t *)maps[b.index].start, b.bytesused,
                    fmt.fmt.pix.width, fmt.fmt.pix.height,
                    memcrop_buf, H264_MEMCROP_BYTES);
            } else if (requested_cropab && cropab_phase == 1U) {
                crop_ret = h264_memcrop_ouyy_evyy_dma(
                    crop_dma_hdl, crop_dma_done,
                    (const uint8_t *)maps[b.index].start, b.bytesused,
                    fmt.fmt.pix.width, fmt.fmt.pix.height,
                    memcrop_buf, H264_MEMCROP_BYTES);
            } else {
                crop_ret = h264_memcrop_ouyy_evyy(
                    (const uint8_t *)maps[b.index].start, b.bytesused,
                    fmt.fmt.pix.width, fmt.fmt.pix.height,
                    memcrop_buf, H264_MEMCROP_BYTES);
            }
            int64_t c1 = esp_timer_get_time();
            crop_us_last = (c1 > c0) ? (uint32_t)(c1 - c0) : 0U;
            if (crop_ret != ESP_OK) {
                saved_errno = (crop_ret == ESP_ERR_TIMEOUT) ? ETIMEDOUT : EIO;
                fail_stage = (requested_ppaab && cropab_phase == 1U) ?
                    "h264direct_live720_ppa_crop" :
                    ((requested_cropab && cropab_phase == 1U) ?
                     "h264direct_live720_dma_crop_copy" : "h264direct_live720_memcrop_copy");
                goto direct_fail;
            }
            crop_us_total += crop_us_last;
            if (crop_us_last < crop_us_min) crop_us_min = crop_us_last;
            if (crop_us_last > crop_us_max) crop_us_max = crop_us_last;
            if (requested_cropab || requested_ppaab) {
                cropab_crop_total[cropab_phase] += crop_us_last;
                if (crop_us_last < cropab_crop_min[cropab_phase]) cropab_crop_min[cropab_phase] = crop_us_last;
                if (crop_us_last > cropab_crop_max[cropab_phase]) cropab_crop_max[cropab_phase] = crop_us_last;
            }
            h264_input = memcrop_buf;
            h264_input_len = H264_MEMCROP_BYTES;
        }

        esp_h264_enc_in_frame_t in_frame = {
            .raw_data = {
                .buffer = h264_input,
                .len = h264_input_len,
            },
            .pts = n * 33U,
        };
        esp_h264_enc_out_frame_t out_frame = {
            .raw_data = {
                .buffer = out_buf,
                .len = (uint32_t)out_capacity,
            },
        };

        int64_t e0 = esp_timer_get_time();
        saved_h264_err = esp_h264_enc_process(enc, &in_frame, &out_frame);
        int64_t e1 = esp_timer_get_time();
        enc_us_last = (e1 > e0) ? (uint32_t)(e1 - e0) : 0;
        if (saved_h264_err != ESP_H264_ERR_OK) {
            fail_stage = "h264direct_enc_process";
            goto direct_fail;
        }
        if (out_frame.length == 0 || out_frame.length > out_capacity) {
            saved_errno = EIO; fail_stage = "h264direct_empty_output"; goto direct_fail;
        }

        if (!requested_camoff && !requested_camhold && !requested_camseq && !requested_lcdpat && !requested_priab && !requested_fpsab) {
            if (ioctl(cam_fd, VIDIOC_QBUF, &b) != 0) {
                saved_errno = errno; fail_stage = "h264direct_camera_QBUF_requeue"; goto direct_fail;
            }
        } else if (requested_camhold || (requested_camseq && n < 60U)) {
            /* Keep camera DMA active without changing the H264 input frame.
             * One non-held completed buffer should be available at ~30 fps
             * after each ~34 ms encode. Requeue it immediately for the next
             * CSI/ISP DMA cycle. This occurs after enc_process timing. */
            struct v4l2_buffer pump = {
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
            };
            int64_t pdq0 = esp_timer_get_time();
            if (ioctl(cam_fd, VIDIOC_DQBUF, &pump) != 0) {
                saved_errno = errno; fail_stage = "h264direct_camhold_DQBUF_pump"; goto direct_fail;
            }
            int64_t pdq1 = esp_timer_get_time();
            uint32_t pump_us = (pdq1 > pdq0) ? (uint32_t)(pdq1 - pdq0) : 0;
            if (pump.index >= mapped_count || pump.index == held_b.index) {
                saved_errno = EFAULT; fail_stage = "h264direct_camhold_bad_pump"; goto direct_fail;
            }
            if (ioctl(cam_fd, VIDIOC_QBUF, &pump) != 0) {
                saved_errno = errno; fail_stage = "h264direct_camhold_QBUF_pump"; goto direct_fail;
            }
            dq_us_last = pump_us;
            dq_us_total += pump_us;
            if (pump_us > dq_us_max) dq_us_max = pump_us;
        }

        uint32_t used = out_frame.length;
        uint32_t hash = sparse_fnv1a(out_buf, used);

        if (requested_camseq || requested_lcdpat || requested_priab || requested_fpsab) {
            uint32_t sp = (requested_priab || requested_fpsab) ? ((n >= 30U && n < 90U) ? 1U : 0U)
                                                               : ((n < 60U) ? 0U : 1U);
            if (seq_frames[sp] == 0U) {
                seq_t_first[sp] = e1;
                seq_hash_first[sp] = hash;
            }
            seq_t_last[sp] = e1;
            seq_hash_last[sp] = hash;
            seq_frames[sp]++;
            seq_enc_total[sp] += enc_us_last;
            if (enc_us_last < seq_enc_min[sp]) seq_enc_min[sp] = enc_us_last;
            if (enc_us_last > seq_enc_max[sp]) seq_enc_max[sp] = enc_us_last;
            if (requested_camseq && sp == 0U) {
                seq_dq_total[sp] += dq_us_last;
                if (dq_us_last > seq_dq_max[sp]) seq_dq_max[sp] = dq_us_last;
            }
        }

        if (requested_cropab || requested_ppaab) {
            if (cropab_frames[cropab_phase] == 0U) {
                cropab_t_first[cropab_phase] = e1;
            }
            cropab_t_last[cropab_phase] = e1;
            cropab_frames[cropab_phase]++;
            cropab_enc_total[cropab_phase] += enc_us_last;
            if (enc_us_last < cropab_enc_min[cropab_phase]) cropab_enc_min[cropab_phase] = enc_us_last;
            if (enc_us_last > cropab_enc_max[cropab_phase]) cropab_enc_max[cropab_phase] = enc_us_last;
            cropab_dq_total[cropab_phase] += dq_us_last;
            if (dq_us_last > cropab_dq_max[cropab_phase]) cropab_dq_max[cropab_phase] = dq_us_last;
            if (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR) cropab_idr[cropab_phase]++;
            else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_I) cropab_i[cropab_phase]++;
            else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_P) cropab_p[cropab_phase]++;
        }

        if (frames == 0) {
            t_first = e1;
            hash_first = hash;
        }
        t_last = e1;
        hash_last = hash;
        frames++;
        enc_us_total += enc_us_last;
        if (enc_us_last < enc_us_min) enc_us_min = enc_us_last;
        if (enc_us_last > enc_us_max) enc_us_max = enc_us_last;
        bytes_total += used;
        if (used < bytes_min) bytes_min = used;
        if (used > bytes_max) bytes_max = used;
        if (out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR) {
            idr_frames++;
            idr_us_total += enc_us_last;
            idr_bytes_total += used;
            if (enc_us_last < idr_us_min) idr_us_min = enc_us_last;
            if (enc_us_last > idr_us_max) idr_us_max = enc_us_last;
            if (used < idr_bytes_min) idr_bytes_min = used;
            if (used > idr_bytes_max) idr_bytes_max = used;
        } else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_I) {
            i_frames++;
            i_us_total += enc_us_last;
            i_bytes_total += used;
            if (enc_us_last < i_us_min) i_us_min = enc_us_last;
            if (enc_us_last > i_us_max) i_us_max = enc_us_last;
            if (used < i_bytes_min) i_bytes_min = used;
            if (used > i_bytes_max) i_bytes_max = used;
        } else if (out_frame.frame_type == ESP_H264_FRAME_TYPE_P) {
            p_frames++;
            p_us_total += enc_us_last;
            p_bytes_total += used;
            if (enc_us_last < p_us_min) p_us_min = enc_us_last;
            if (enc_us_last > p_us_max) p_us_max = enc_us_last;
            if (used < p_bytes_min) p_bytes_min = used;
            if (used > p_bytes_max) p_bytes_max = used;
        }
        h264_scan_annexb(out_buf, used, &nal);

        if (frames <= 3 || (frames % 10U) == 0U ||
            (requested_livegop && out_frame.frame_type == ESP_H264_FRAME_TYPE_IDR) ||
            ((requested_camseq || requested_lcdpat) && (frames == 61U || frames == 62U || frames == 63U)) ||
            ((requested_priab || requested_fpsab) && (frames == 31U || frames == 32U || frames == 33U || frames == 91U || frames == 92U || frames == 93U))) {
            const char *phase_name = "single";
            if (requested_camseq) phase_name = (n < 60U) ? "CAMHOLD" : "CAMOFF";
            else if (requested_lcdpat) phase_name = (n < 60U) ? "LCDON" : "LCDPATTERN";
            else if (requested_priab) phase_name = (n >= 30U && n < 90U) ? "HIGH" : "NORMAL";
            else if (requested_fpsab) phase_name = (n >= 30U && n < 90U) ? "FPS60" : "FPS30";
            else if (requested_ppaab) phase_name = (cropab_phase == 0U) ? "CPU_MEMCPY" : "PPA_SRM";
            else if (requested_cropab) phase_name = (cropab_phase == 0U) ? "CPU_MEMCPY" : "AXI_GDMA";
            ESP_LOGI(TAG,
                     "H264-DIRECT frame=%u phase=%s raw=%u h264_in=%u h264=%u dq_us=%u crop_us=%u enc_us=%u type=%d hash=%08" PRIx32,
                     (unsigned)frames, phase_name, (unsigned)b.bytesused, (unsigned)h264_input_len, (unsigned)used,
                     (unsigned)dq_us_last, (unsigned)crop_us_last, (unsigned)enc_us_last,
                     (int)out_frame.frame_type, hash);
        }

        if ((requested_cropab || requested_ppaab) && n == (H264_CROP_AB_PHASE_FRAMES - 1U)) {
            uint32_t cpu_crop_avg_now = cropab_frames[0] ? (uint32_t)(cropab_crop_total[0] / cropab_frames[0]) : 0U;
            uint32_t cpu_enc_avg_now = cropab_frames[0] ? (uint32_t)(cropab_enc_total[0] / cropab_frames[0]) : 0U;
            uint32_t cpu_pipe_now = cpu_crop_avg_now + cpu_enc_avg_now;
            uint32_t cpu_pipe_fps_now = cpu_pipe_now ? (uint32_t)(100000000ULL / cpu_pipe_now) : 0U;
            ESP_LOGI(TAG,
                     "H264-DIRECT %s phase1 CPU done frames=%u crop_avg=%u enc_avg=%u pipeline=%u (%u.%02u fps); phase2 %s starts next frame",
                     requested_ppaab ? "PPAAB" : "CROPAB",
                     (unsigned)cropab_frames[0], (unsigned)cpu_crop_avg_now,
                     (unsigned)cpu_enc_avg_now, (unsigned)cpu_pipe_now,
                     (unsigned)(cpu_pipe_fps_now / 100U), (unsigned)(cpu_pipe_fps_now % 100U),
                     requested_ppaab ? "PPA-SRM" : "AXI-GDMA");
        }

        if (requested_camseq && n == 59U) {
            uint32_t hold_avg_now = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            held_raw_hash_after_hold = sparse_fnv1a((const uint8_t *)maps[held_b.index].start, held_b.bytesused);
            ESP_LOGI(TAG,
                     "H264-DIRECT SAMEBOOT phase1 CAMHOLD done frames=%u enc_avg=%u min=%u max=%u dq_avg=%u raw_hash_now=%08" PRIx32 "; STREAMOFF now",
                     (unsigned)seq_frames[0], (unsigned)hold_avg_now,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]),
                     (unsigned)seq_enc_max[0],
                     (unsigned)(seq_frames[0] ? (seq_dq_total[0] / seq_frames[0]) : 0U),
                     held_raw_hash_after_hold);
            int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(cam_fd, VIDIOC_STREAMOFF, &t) != 0) {
                saved_errno = errno; fail_stage = "h264direct_camseq_STREAMOFF"; goto direct_fail;
            }
            cam_streaming = false;
            dq_us_last = 0;
            ESP_LOGI(TAG,
                     "H264-DIRECT SAMEBOOT phase2 CAMOFF starts; SAME held raw hash=%08" PRIx32 " idx=%u bytes=%u",
                     held_raw_hash, (unsigned)held_b.index, (unsigned)held_b.bytesused);
        } else if (requested_lcdpat && n == 59U) {
            uint32_t lcdon_avg_now = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            held_raw_hash_after_hold = sparse_fnv1a((const uint8_t *)maps[held_b.index].start, held_b.bytesused);
            lcdpat_dma_free_before_pattern = dma_free();
            lcdpat_dma_largest_before_pattern = dma_largest();
            lcdpat_psram_free_before_pattern = psram_free();
            ESP_LOGI(TAG,
                     "H264-DIRECT LCDPAT phase1 LCDON done frames=%u enc_avg=%u min=%u max=%u raw_hash_now=%08" PRIx32
                     "; switching framebuffer scanout -> DSI HW pattern now dma=%u/%u psram=%u",
                     (unsigned)seq_frames[0], (unsigned)lcdon_avg_now,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]),
                     (unsigned)seq_enc_max[0], held_raw_hash_after_hold,
                     (unsigned)lcdpat_dma_free_before_pattern, (unsigned)lcdpat_dma_largest_before_pattern,
                     (unsigned)lcdpat_psram_free_before_pattern);

            esp_err_t lcd_pattern_err = camera_diag_lcd_pattern_enter(
                lcdpat_display, &lcdpat_panel, &lcdpat_lvgl_timer_paused);
            if (lcd_pattern_err != ESP_OK) {
                saved_errno = EIO;
                fail_stage = "h264direct_lcdpat_pattern_enter";
                ESP_LOGE(TAG, "H264-DIRECT LCDPAT pattern enter failed: %s", esp_err_to_name(lcd_pattern_err));
                goto direct_fail;
            }
            lcdpat_active = true;
            vTaskDelay(pdMS_TO_TICKS(100));

            lcdpat_dma_free_after_pattern = dma_free();
            lcdpat_dma_largest_after_pattern = dma_largest();
            lcdpat_psram_free_after_pattern = psram_free();
            dq_us_last = 0;
            ESP_LOGI(TAG,
                     "H264-DIRECT LCDPAT phase2 LCDPATTERN starts; camera already STREAMOFF; SAME raw hash=%08" PRIx32
                     " idx=%u bytes=%u panel=%p dma=%u/%u psram=%u",
                     held_raw_hash, (unsigned)held_b.index, (unsigned)held_b.bytesused,
                     (void *)lcdpat_panel,
                     (unsigned)lcdpat_dma_free_after_pattern, (unsigned)lcdpat_dma_largest_after_pattern,
                     (unsigned)lcdpat_psram_free_after_pattern);
        } else if (requested_priab && n == 29U) {
            uint32_t normal30_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            vTaskPrioritySet(priab_task, priab_prio_high);
            priab_prio_raised = true;
            ESP_LOGW(TAG,
                     "H264-DIRECT PRIAB transition NORMAL->HIGH after first 30 NORMAL frames avg=%u; priority %u -> %u",
                     (unsigned)normal30_avg, (unsigned)priab_prio_normal, (unsigned)priab_prio_high);
        } else if (requested_priab && n == 89U) {
            uint32_t high60_avg = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            vTaskPrioritySet(priab_task, priab_prio_normal);
            priab_prio_raised = false;
            ESP_LOGW(TAG,
                     "H264-DIRECT PRIAB transition HIGH->NORMAL after 60 HIGH frames avg=%u; priority restored %u",
                     (unsigned)high60_avg, (unsigned)priab_prio_normal);
        }
        else if (requested_fpsab && n == 29U) {
            uint32_t fps30_first_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            if (enc_opened && enc) {
                (void)esp_h264_enc_close(enc);
                enc_opened = false;
            }
            if (enc) {
                (void)esp_h264_enc_del(enc);
                enc = NULL;
            }
            cfg.fps = 60;
            saved_h264_err = esp_h264_enc_hw_new(&cfg, &enc);
            if (saved_h264_err != ESP_H264_ERR_OK || !enc) {
                fail_stage = "h264direct_fpsab_enc60_new"; goto direct_fail;
            }
            saved_h264_err = esp_h264_enc_open(enc);
            if (saved_h264_err != ESP_H264_ERR_OK) {
                fail_stage = "h264direct_fpsab_enc60_open"; goto direct_fail;
            }
            enc_opened = true;
            fpsab_reopen_count++;
            ESP_LOGW(TAG,
                     "H264-DIRECT FPSAB transition FPS30->FPS60 after first 30 FPS30 frames avg=%u; encoder reopened cfg.fps=%u reopen_count=%u",
                     (unsigned)fps30_first_avg, (unsigned)cfg.fps, (unsigned)fpsab_reopen_count);
        } else if (requested_fpsab && n == 89U) {
            uint32_t fps60_avg_now = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            if (enc_opened && enc) {
                (void)esp_h264_enc_close(enc);
                enc_opened = false;
            }
            if (enc) {
                (void)esp_h264_enc_del(enc);
                enc = NULL;
            }
            cfg.fps = 30;
            saved_h264_err = esp_h264_enc_hw_new(&cfg, &enc);
            if (saved_h264_err != ESP_H264_ERR_OK || !enc) {
                fail_stage = "h264direct_fpsab_enc30_reopen_new"; goto direct_fail;
            }
            saved_h264_err = esp_h264_enc_open(enc);
            if (saved_h264_err != ESP_H264_ERR_OK) {
                fail_stage = "h264direct_fpsab_enc30_reopen_open"; goto direct_fail;
            }
            enc_opened = true;
            fpsab_reopen_count++;
            ESP_LOGW(TAG,
                     "H264-DIRECT FPSAB transition FPS60->FPS30 after 60 FPS60 frames avg=%u; encoder reopened cfg.fps=%u reopen_count=%u",
                     (unsigned)fps60_avg_now, (unsigned)cfg.fps, (unsigned)fpsab_reopen_count);
        }
    }

    } /* !requested_overlap generic frame loop */

    if ((requested_camseq || requested_lcdpat || requested_priab || requested_fpsab) && held_valid) {
        held_raw_hash_after_off = sparse_fnv1a((const uint8_t *)maps[held_b.index].start, held_b.bytesused);
        if (requested_lcdpat) {
            ESP_LOGI(TAG,
                     "H264-DIRECT LCDPAT raw hash final initial=%08" PRIx32 " after_lcdon=%08" PRIx32 " after_lcdpattern=%08" PRIx32 " unchanged=%d",
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_hold && held_raw_hash == held_raw_hash_after_off) ? 1 : 0);
        } else if (requested_priab) {
            ESP_LOGI(TAG,
                     "H264-DIRECT PRIAB raw hash final initial=%08" PRIx32 " final=%08" PRIx32 " unchanged=%d",
                     held_raw_hash, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_off) ? 1 : 0);
        } else if (requested_fpsab) {
            ESP_LOGI(TAG,
                     "H264-DIRECT FPSAB raw hash final initial=%08" PRIx32 " final=%08" PRIx32 " unchanged=%d",
                     held_raw_hash, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_off) ? 1 : 0);
        } else {
            ESP_LOGI(TAG,
                     "H264-DIRECT SAMEBOOT raw hash final initial=%08" PRIx32 " after_hold=%08" PRIx32 " after_off=%08" PRIx32 " unchanged=%d",
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_hold && held_raw_hash == held_raw_hash_after_off) ? 1 : 0);
        }
    }

    if (requested_priab) {
        if (priab_prio_raised && priab_task) {
            vTaskPrioritySet(priab_task, priab_prio_normal);
            priab_prio_raised = false;
            ESP_LOGW(TAG, "H264-DIRECT PRIAB safety restore task priority=%u", (unsigned)priab_prio_normal);
        }
        /* This driver revision may report successful pattern-NONE restore yet
         * leave the panel visually black. The app remains alive; reboot after
         * reading the result is the defined display recovery. */
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            (void)camera_diag_lcd_pattern_leave(lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
        }
        lcd_ui_set_camera_stream_active(false);
        ESP_LOGW(TAG, "H264-DIRECT PRIAB complete; reboot after reading JSON to guarantee LCD recovery");
    }

    if (requested_fpsab) {
        /* Pattern mode may not visibly recover on this BSP revision. Restore
         * the software state best-effort, but define reboot as display recovery. */
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            (void)camera_diag_lcd_pattern_leave(lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
        }
        lcd_ui_set_camera_stream_active(false);
        ESP_LOGW(TAG, "H264-DIRECT FPSAB complete; reboot after reading JSON to guarantee LCD recovery");
    }

    if (requested_lcdpat) {
        /* Restore normal framebuffer output before the HTTP reply. LVGL was
         * never deinitialized, so clock/weather/UI tasks keep a valid mutex. */
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            esp_err_t restore_err = camera_diag_lcd_pattern_leave(
                lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
            lcdpat_restored = (restore_err == ESP_OK);
            if (restore_err != ESP_OK) {
                ESP_LOGE(TAG, "H264-DIRECT LCDPAT restore failed: %s", esp_err_to_name(restore_err));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        lcd_ui_set_camera_stream_active(false);
        ESP_LOGI(TAG, "H264-DIRECT LCDPAT normal framebuffer/LVGL restored=%d", lcdpat_restored ? 1 : 0);
    }

    if (h264_qos_applied) {
        axi_qos_active_end = axi_qos_diag_read();
        axi_qos_diag_log("after-60-frames-qos-active", &axi_qos_active_end);

        axi_icm_ll_set_h264_dma_qos_arbiter_prio(
            0, h264_qos_saved_m1_w, h264_qos_saved_m1_r);
        axi_icm_ll_set_h264_dma_qos_arbiter_prio(
            1, h264_qos_saved_m2_w, h264_qos_saved_m2_r);
        h264_qos_applied = false;

        axi_qos_diag_t q = axi_qos_diag_read();
        axi_qos_diag_log("after-h264-qos-restore", &q);
        ESP_LOGW(TAG, "[AXI-QOS-AB] restored original H264 M1/M2 AR/AW QoS");
    }

    if (requested_overlapcamab && overlap_pump_ctx.exit_sem) {
        if (overlap_pump_started) {
            overlap_pump_ctx.stop = true;
            if (cam_streaming) {
                int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                (void)ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
                cam_streaming = false;
            }
            (void)xSemaphoreTake(overlap_pump_ctx.exit_sem, pdMS_TO_TICKS(1000));
            overlap_pump_started = false;
            overlap_pump_ctx.task = NULL;
        }
        vSemaphoreDelete(overlap_pump_ctx.exit_sem);
        overlap_pump_ctx.exit_sem = NULL;
    }

    if (requested_overlap) {
        if (overlap_worker_started) {
            overlap_ctx.stop = true;
            (void)xSemaphoreGive(overlap_ctx.kick_sem);
            (void)xSemaphoreTake(overlap_ctx.exit_sem, pdMS_TO_TICKS(1000));
            overlap_worker_started = false;
            overlap_ctx.task = NULL;
        }
        if (overlap_ctx.kick_sem) { vSemaphoreDelete(overlap_ctx.kick_sem); overlap_ctx.kick_sem = NULL; }
        if (overlap_ctx.done_sem) { vSemaphoreDelete(overlap_ctx.done_sem); overlap_ctx.done_sem = NULL; }
        if (overlap_ctx.exit_sem) { vSemaphoreDelete(overlap_ctx.exit_sem); overlap_ctx.exit_sem = NULL; }
    }

    if (cam_streaming) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
        cam_streaming = false;
    }
    if (enc_opened) {
        (void)esp_h264_enc_close(enc);
        enc_opened = false;
    }
    if (enc) {
        (void)esp_h264_enc_del(enc);
        enc = NULL;
    }
    if (crop_dma_installed && crop_dma_hdl) {
        (void)esp_async_memcpy_uninstall(crop_dma_hdl);
        crop_dma_hdl = NULL;
        crop_dma_installed = false;
    }
    if (crop_dma_done) {
        vSemaphoreDelete(crop_dma_done);
        crop_dma_done = NULL;
    }
    if (crop_ppa_client) {
        (void)ppa_unregister_client(crop_ppa_client);
        crop_ppa_client = NULL;
    }
    if (ppa_verify_buf) {
        heap_caps_free(ppa_verify_buf);
        ppa_verify_buf = NULL;
    }
    if (out_buf) {
        heap_caps_free(out_buf);
        out_buf = NULL;
    }
    if (memcrop_buf) {
        heap_caps_free(memcrop_buf);
        memcrop_buf = NULL;
    }
    if (memcrop_buf2) {
        heap_caps_free(memcrop_buf2);
        memcrop_buf2 = NULL;
    }
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    if (cam_fd >= 0) {
        close(cam_fd);
        cam_fd = -1;
    }

    {
        uint32_t enc_avg = frames ? (uint32_t)(enc_us_total / frames) : 0;
        uint32_t dq_avg = frames ? (uint32_t)(dq_us_total / frames) : 0;
        uint32_t crop_avg = (requested_live720 && frames) ? (uint32_t)(crop_us_total / frames) : 0U;
        uint32_t pipeline_avg = enc_avg + crop_avg;
        uint32_t pipeline_fps_x100 = pipeline_avg ?
            (uint32_t)(100000000ULL / (uint64_t)pipeline_avg) : 0U;
        uint32_t process_fps_x100 = enc_avg ?
            (uint32_t)(100000000ULL / (uint64_t)enc_avg) : 0;
        uint32_t wall_fps_x100 = (frames > 1 && t_last > t_first) ?
            (uint32_t)(((uint64_t)(frames - 1) * 100000000ULL) /
                       (uint64_t)(t_last - t_first)) : 0;
        uint32_t bytes_avg = frames ? (uint32_t)(bytes_total / frames) : 0;
        uint32_t idr_us_avg = idr_frames ? (uint32_t)(idr_us_total / idr_frames) : 0;
        uint32_t p_us_avg = p_frames ? (uint32_t)(p_us_total / p_frames) : 0;
        uint32_t i_us_avg = i_frames ? (uint32_t)(i_us_total / i_frames) : 0;
        uint32_t idr_bytes_avg = idr_frames ? (uint32_t)(idr_bytes_total / idr_frames) : 0;
        uint32_t p_bytes_avg = p_frames ? (uint32_t)(p_bytes_total / p_frames) : 0;
        uint32_t i_bytes_avg = i_frames ? (uint32_t)(i_bytes_total / i_frames) : 0;
        size_t dfa = dma_free(), dla = dma_largest(), psa = psram_free();

        h264_clk_end = h264_clock_diag_read();
        const char *post_frames_label = (requested_cropab || requested_ppaab || requested_overlapcamab) ? "after-240-frames" :
            ((requested_camseq || requested_lcdpat || requested_priab || requested_fpsab || requested_livegop || requested_live720) ? "after-120-frames" : "after-60-frames");
        h264_clock_diag_log(post_frames_label, &h264_clk_end);
        axi_qos_end = axi_qos_diag_read();
        axi_qos_diag_log(post_frames_label, &axi_qos_end);

        if (requested_fpsab) {
            uint32_t fps30_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            uint32_t fps60_avg = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            uint32_t fps30_proc = fps30_avg ? (uint32_t)(100000000ULL / fps30_avg) : 0U;
            uint32_t fps60_proc = fps60_avg ? (uint32_t)(100000000ULL / fps60_avg) : 0U;
            int32_t delta_us = (int32_t)fps30_avg - (int32_t)fps60_avg;
            int32_t delta_pct_x100 = fps60_avg ?
                (int32_t)(((int64_t)delta_us * 10000LL) / (int64_t)fps60_avg) : 0;

            ESP_LOGI(TAG,
                     "H264-DIRECT FPSAB PASS raw=%08" PRIx32 "/%08" PRIx32
                     " FPS30 avg/min/max=%u/%u/%u process=%u.%02u"
                     " | FPS60 avg/min/max=%u/%u/%u process=%u.%02u"
                     " | fps30-minus-fps60=%ld us (%ld.%02ld%%) reopen_count=%u hashes fps30=%08" PRIx32 "/%08" PRIx32
                     " fps60=%08" PRIx32 "/%08" PRIx32,
                     held_raw_hash, held_raw_hash_after_off,
                     (unsigned)fps30_avg, (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)(fps30_proc / 100U), (unsigned)(fps30_proc % 100U),
                     (unsigned)fps60_avg, (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)(fps60_proc / 100U), (unsigned)(fps60_proc % 100U),
                     (long)delta_us, (long)(delta_pct_x100 / 100), (long)labs(delta_pct_x100 % 100),
                     (unsigned)fpsab_reopen_count,
                     seq_hash_first[0], seq_hash_last[0], seq_hash_first[1], seq_hash_last[1]);

            char fps_body[2400];
            snprintf(fps_body, sizeof(fps_body),
                     "{\"ok\":true,\"test\":\"esp_h264_sameboot_fps_config_ab\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1920x1080 YU12/O_UYY_E_VYY\","
                     "\"encoder_api\":\"esp_h264_enc_process\","
                     "\"config\":{\"fpsab\":true,\"fps_a\":30,\"fps_b\":60,\"gop\":%u,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"camoff\":true,\"lcd_pattern\":true,\"h264qos\":0},"
                     "\"same_boot\":true,\"same_held_frame\":true,\"camera_streamoff_entire_measurement\":true,\"dsi_pattern_entire_measurement\":true,"
                     "\"order\":\"fps30x30_fps60x60_fps30x30\",\"encoder_reopened_on_each_transition\":true,\"reopen_count\":%u,\"pts_step_ms_constant\":33,"
                     "\"held_raw_hash\":{\"initial\":\"%08" PRIx32 "\",\"final\":\"%08" PRIx32 "\",\"unchanged\":%s},"
                     "\"fps30\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"fps60\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"delta\":{\"fps30_minus_fps60_us\":%ld,\"fps30_minus_fps60_percent_x100\":%ld},"
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"Only esp_h264 encoder cfg.fps changes. Raw input, camera STREAMOFF, DSI hardware pattern, fixed QP, GOP, output buffer and PTS step remain constant. Output hash may differ between fps configs because codec headers can encode timing information.\","
                     "\"reboot_required_for_lcd_recovery\":true,\"reboot_before_retest\":true}",
                     (unsigned)requested_gop, (unsigned)((fixed_qp >= 0) ? fixed_qp : 20), (unsigned)((fixed_qp >= 0) ? fixed_qp : 40),
                     (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                     (unsigned)fpsab_reopen_count, held_raw_hash, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_off) ? "true" : "false",
                     (unsigned)seq_frames[0], (unsigned)fps30_avg,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)fps30_proc, seq_hash_first[0], seq_hash_last[0],
                     (unsigned)seq_frames[1], (unsigned)fps60_avg,
                     (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)fps60_proc, seq_hash_first[1], seq_hash_last[1],
                     (long)delta_us, (long)delta_pct_x100,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, fps_body);
        }

        if (requested_priab) {
            uint32_t normal_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            uint32_t high_avg = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            uint32_t normal_proc = normal_avg ? (uint32_t)(100000000ULL / normal_avg) : 0U;
            uint32_t high_proc = high_avg ? (uint32_t)(100000000ULL / high_avg) : 0U;
            int32_t delta_us = (int32_t)normal_avg - (int32_t)high_avg;
            int32_t delta_pct_x100 = high_avg ?
                (int32_t)(((int64_t)delta_us * 10000LL) / (int64_t)high_avg) : 0;

            ESP_LOGI(TAG,
                     "H264-DIRECT PRIAB PASS raw=%08" PRIx32 "/%08" PRIx32
                     " NORMAL avg/min/max=%u/%u/%u process=%u.%02u"
                     " | HIGH avg/min/max=%u/%u/%u process=%u.%02u"
                     " | normal-minus-high=%ld us (%ld.%02ld%%) prio=%u->%u hashes normal=%08" PRIx32 "/%08" PRIx32
                     " high=%08" PRIx32 "/%08" PRIx32,
                     held_raw_hash, held_raw_hash_after_off,
                     (unsigned)normal_avg, (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)(normal_proc / 100U), (unsigned)(normal_proc % 100U),
                     (unsigned)high_avg, (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)(high_proc / 100U), (unsigned)(high_proc % 100U),
                     (long)delta_us, (long)(delta_pct_x100 / 100), (long)labs(delta_pct_x100 % 100),
                     (unsigned)priab_prio_normal, (unsigned)priab_prio_high,
                     seq_hash_first[0], seq_hash_last[0], seq_hash_first[1], seq_hash_last[1]);

            char pri_body[2200];
            snprintf(pri_body, sizeof(pri_body),
                     "{\"ok\":true,\"test\":\"esp_h264_sameboot_priority_ab\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1920x1080 YU12/O_UYY_E_VYY\","
                     "\"encoder_api\":\"esp_h264_enc_process\","
                     "\"config\":{\"fps\":30,\"gop\":%u,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"priab\":true,\"camoff\":true,\"lcd_pattern\":true,\"h264qos\":0},"
                     "\"same_boot\":true,\"same_held_frame\":true,\"camera_streamoff_entire_measurement\":true,\"dsi_pattern_entire_measurement\":true,"
                     "\"order\":\"normal30_high60_normal30\",\"task_priority\":{\"normal\":%u,\"high\":%u,\"restored\":true},"
                     "\"held_raw_hash\":{\"initial\":\"%08" PRIx32 "\",\"final\":\"%08" PRIx32 "\",\"unchanged\":%s},"
                     "\"normal\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"high\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"delta\":{\"normal_minus_high_us\":%ld,\"normal_minus_high_percent_x100\":%ld},"
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"Positive delta means raising only the current H264 HTTP task priority reduced esp_h264_enc_process latency. Camera is STREAMOFF and DSI uses hardware pattern for all 120 measured frames.\","
                     "\"reboot_required_for_lcd_recovery\":true,\"reboot_before_retest\":true}",
                     (unsigned)cfg.gop, (unsigned)cfg.rc.qp_min, (unsigned)cfg.rc.qp_max,
                     (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                     (unsigned)priab_prio_normal, (unsigned)priab_prio_high,
                     held_raw_hash, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_off) ? "true" : "false",
                     (unsigned)seq_frames[0], (unsigned)normal_avg,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)normal_proc, seq_hash_first[0], seq_hash_last[0],
                     (unsigned)seq_frames[1], (unsigned)high_avg,
                     (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)high_proc, seq_hash_first[1], seq_hash_last[1],
                     (long)delta_us, (long)delta_pct_x100,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, pri_body);
        }

        if (requested_lcdpat) {
            uint32_t lcdon_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            uint32_t lcdpattern_avg = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            uint32_t lcdon_proc = lcdon_avg ? (uint32_t)(100000000ULL / lcdon_avg) : 0U;
            uint32_t lcdpattern_proc = lcdpattern_avg ? (uint32_t)(100000000ULL / lcdpattern_avg) : 0U;
            uint32_t lcdon_wall = (seq_frames[0] > 1U && seq_t_last[0] > seq_t_first[0]) ?
                (uint32_t)(((uint64_t)(seq_frames[0] - 1U) * 100000000ULL) /
                           (uint64_t)(seq_t_last[0] - seq_t_first[0])) : 0U;
            uint32_t lcdpattern_wall = (seq_frames[1] > 1U && seq_t_last[1] > seq_t_first[1]) ?
                (uint32_t)(((uint64_t)(seq_frames[1] - 1U) * 100000000ULL) /
                           (uint64_t)(seq_t_last[1] - seq_t_first[1])) : 0U;
            int32_t delta_us = (int32_t)lcdon_avg - (int32_t)lcdpattern_avg;
            int32_t delta_pct_x100 = lcdpattern_avg ?
                (int32_t)(((int64_t)delta_us * 10000LL) / (int64_t)lcdpattern_avg) : 0;

            ESP_LOGI(TAG,
                     "H264-DIRECT LCDPAT PASS raw=%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32
                     " LCDON avg/min/max=%u/%u/%u process=%u.%02u wall=%u.%02u"
                     " | LCDPATTERN avg/min/max=%u/%u/%u process=%u.%02u wall=%u.%02u"
                     " | lcdon-minus-lcdpattern=%ld us (%ld.%02ld%%) hashes on=%08" PRIx32 "/%08" PRIx32
                     " pattern=%08" PRIx32 "/%08" PRIx32 " restored=%d",
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (unsigned)lcdon_avg, (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)(lcdon_proc / 100U), (unsigned)(lcdon_proc % 100U),
                     (unsigned)(lcdon_wall / 100U), (unsigned)(lcdon_wall % 100U),
                     (unsigned)lcdpattern_avg, (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)(lcdpattern_proc / 100U), (unsigned)(lcdpattern_proc % 100U),
                     (unsigned)(lcdpattern_wall / 100U), (unsigned)(lcdpattern_wall % 100U),
                     (long)delta_us, (long)(delta_pct_x100 / 100), (long)labs(delta_pct_x100 % 100),
                     seq_hash_first[0], seq_hash_last[0], seq_hash_first[1], seq_hash_last[1],
                     lcdpat_restored ? 1 : 0);

            char lcd_body[2800];
            snprintf(lcd_body, sizeof(lcd_body),
                     "{\"ok\":true,\"test\":\"esp_h264_sameboot_lcdon_lcdpattern\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1920x1080 YU12/O_UYY_E_VYY\","
                     "\"encoder_api\":\"esp_h264_enc_process\","
                     "\"config\":{\"fps\":30,\"gop\":%u,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"lcdpat\":true,\"camoff\":true,\"h264qos\":0},"
                     "\"same_boot\":true,\"same_held_frame\":true,\"camera_streamoff_entire_measurement\":true,\"order\":\"lcdon_then_lcdpattern\","
                     "\"held_raw_hash\":{\"initial\":\"%08" PRIx32 "\",\"after_lcdon\":\"%08" PRIx32 "\",\"after_lcdpattern\":\"%08" PRIx32 "\",\"unchanged\":%s},\"held_index\":%u,\"held_bytes\":%u,"
                     "\"lcdon\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"wall_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"lcdpattern\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"wall_fps_x100\":%u,\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"delta\":{\"lcdon_minus_lcdpattern_us\":%ld,\"lcdon_minus_lcdpattern_percent_x100\":%ld},"
                     "\"lcd_pattern\":{\"mode\":\"MIPI_DSI_PATTERN_BAR_VERTICAL\",\"lvgl_timer_paused_during_phase2\":true,\"restored\":%s,\"settle_ms\":100,"
                     "\"dma_before\":{\"free\":%u,\"largest\":%u},\"dma_after\":{\"free\":%u,\"largest\":%u},"
                     "\"psram_before\":%u,\"psram_after\":%u},"
                     "\"dma\":{\"free_before_test\":%u,\"largest_before_test\":%u,\"free_after_test\":%u,\"largest_after_test\":%u},"
                     "\"psram\":{\"free_before_test\":%u,\"free_after_test\":%u},"
                     "\"note\":\"Positive delta means LCD framebuffer/GDMA scanout makes H264 slower. DSI timing stays active via hardware pattern; LVGL is never deinitialized and normal framebuffer mode is restored before reply.\","
                     "\"reboot_required_for_lcd_recovery\":%s,\"reboot_before_retest\":true}",
                     (unsigned)cfg.gop, (unsigned)cfg.rc.qp_min, (unsigned)cfg.rc.qp_max,
                     (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_hold && held_raw_hash == held_raw_hash_after_off) ? "true" : "false",
                     (unsigned)held_b.index, (unsigned)held_b.bytesused,
                     (unsigned)seq_frames[0], (unsigned)lcdon_avg,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)lcdon_proc, (unsigned)lcdon_wall, seq_hash_first[0], seq_hash_last[0],
                     (unsigned)seq_frames[1], (unsigned)lcdpattern_avg,
                     (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)lcdpattern_proc, (unsigned)lcdpattern_wall, seq_hash_first[1], seq_hash_last[1],
                     (long)delta_us, (long)delta_pct_x100,
                     lcdpat_restored ? "true" : "false",
                     (unsigned)lcdpat_dma_free_before_pattern, (unsigned)lcdpat_dma_largest_before_pattern,
                     (unsigned)lcdpat_dma_free_after_pattern, (unsigned)lcdpat_dma_largest_after_pattern,
                     (unsigned)lcdpat_psram_free_before_pattern, (unsigned)lcdpat_psram_free_after_pattern,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa,
                     lcdpat_restored ? "false" : "true");

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, lcd_body);
        }

        if (requested_camseq) {
            uint32_t hold_avg = seq_frames[0] ? (uint32_t)(seq_enc_total[0] / seq_frames[0]) : 0U;
            uint32_t off_avg = seq_frames[1] ? (uint32_t)(seq_enc_total[1] / seq_frames[1]) : 0U;
            uint32_t hold_proc = hold_avg ? (uint32_t)(100000000ULL / hold_avg) : 0U;
            uint32_t off_proc = off_avg ? (uint32_t)(100000000ULL / off_avg) : 0U;
            uint32_t hold_wall = (seq_frames[0] > 1U && seq_t_last[0] > seq_t_first[0]) ?
                (uint32_t)(((uint64_t)(seq_frames[0] - 1U) * 100000000ULL) /
                           (uint64_t)(seq_t_last[0] - seq_t_first[0])) : 0U;
            uint32_t off_wall = (seq_frames[1] > 1U && seq_t_last[1] > seq_t_first[1]) ?
                (uint32_t)(((uint64_t)(seq_frames[1] - 1U) * 100000000ULL) /
                           (uint64_t)(seq_t_last[1] - seq_t_first[1])) : 0U;
            uint32_t hold_dq_avg = seq_frames[0] ? (uint32_t)(seq_dq_total[0] / seq_frames[0]) : 0U;
            int32_t delta_us = (int32_t)hold_avg - (int32_t)off_avg;
            int32_t delta_pct_x100 = off_avg ?
                (int32_t)(((int64_t)delta_us * 10000LL) / (int64_t)off_avg) : 0;

            ESP_LOGI(TAG,
                     "H264-DIRECT SAMEBOOT PASS raw=%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32
                     " CAMHOLD avg/min/max=%u/%u/%u process=%u.%02u wall=%u.%02u dq_avg=%u"
                     " | CAMOFF avg/min/max=%u/%u/%u process=%u.%02u wall=%u.%02u"
                     " | hold-minus-off=%ld us (%ld.%02ld%%) hashes hold=%08" PRIx32 "/%08" PRIx32 " off=%08" PRIx32 "/%08" PRIx32,
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (unsigned)hold_avg, (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)(hold_proc / 100U), (unsigned)(hold_proc % 100U),
                     (unsigned)(hold_wall / 100U), (unsigned)(hold_wall % 100U), (unsigned)hold_dq_avg,
                     (unsigned)off_avg, (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)(off_proc / 100U), (unsigned)(off_proc % 100U),
                     (unsigned)(off_wall / 100U), (unsigned)(off_wall % 100U),
                     (long)delta_us, (long)(delta_pct_x100 / 100), (long)labs(delta_pct_x100 % 100),
                     seq_hash_first[0], seq_hash_last[0], seq_hash_first[1], seq_hash_last[1]);

            char seq_body[2300];
            snprintf(seq_body, sizeof(seq_body),
                     "{\"ok\":true,\"test\":\"esp_h264_sameboot_camhold_camoff\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1920x1080 YU12/O_UYY_E_VYY\","
                     "\"encoder_api\":\"esp_h264_enc_process\","
                     "\"config\":{\"fps\":30,\"gop\":%u,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"camseq\":true,\"h264qos\":0},"
                     "\"same_boot\":true,\"same_held_frame\":true,\"order\":\"camhold_then_camoff\","
                     "\"held_raw_hash\":{\"initial\":\"%08" PRIx32 "\",\"after_camhold\":\"%08" PRIx32 "\",\"after_camoff\":\"%08" PRIx32 "\",\"unchanged\":%s},\"held_index\":%u,\"held_bytes\":%u,"
                     "\"camhold\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"wall_fps_x100\":%u,\"camera_dq_us\":{\"avg\":%u,\"max\":%u},\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"camoff\":{\"frames\":%u,\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"process_fps_x100\":%u,\"wall_fps_x100\":%u,\"camera_dq_us\":{\"avg\":0,\"max\":0},\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"delta\":{\"camhold_minus_camoff_us\":%ld,\"camhold_minus_camoff_percent_x100\":%ld},"
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"Positive delta means camera CSI/ISP/DMA makes H264 slower; both phases use the identical held raw buffer in one boot\","
                     "\"reboot_before_retest\":true}",
                     (unsigned)cfg.gop, (unsigned)cfg.rc.qp_min, (unsigned)cfg.rc.qp_max,
                     (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                     held_raw_hash, held_raw_hash_after_hold, held_raw_hash_after_off,
                     (held_raw_hash == held_raw_hash_after_hold && held_raw_hash == held_raw_hash_after_off) ? "true" : "false",
                     (unsigned)held_b.index, (unsigned)held_b.bytesused,
                     (unsigned)seq_frames[0], (unsigned)hold_avg,
                     (unsigned)(seq_enc_min[0] == UINT32_MAX ? 0U : seq_enc_min[0]), (unsigned)seq_enc_max[0],
                     (unsigned)hold_proc, (unsigned)hold_wall, (unsigned)hold_dq_avg, (unsigned)seq_dq_max[0],
                     seq_hash_first[0], seq_hash_last[0],
                     (unsigned)seq_frames[1], (unsigned)off_avg,
                     (unsigned)(seq_enc_min[1] == UINT32_MAX ? 0U : seq_enc_min[1]), (unsigned)seq_enc_max[1],
                     (unsigned)off_proc, (unsigned)off_wall, seq_hash_first[1], seq_hash_last[1],
                     (long)delta_us, (long)delta_pct_x100,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, seq_body);
        }

        if (requested_overlapcamab) {
            uint32_t on_frames = overlap_camab_frames[0], off_frames = overlap_camab_frames[1];
            uint32_t on_jobs = overlap_camab_wait_count[0], off_jobs = overlap_camab_wait_count[1];
            uint32_t on_crop_avg = on_jobs ? (uint32_t)(overlap_camab_crop_total[0] / on_jobs) : 0U;
            uint32_t off_crop_avg = off_jobs ? (uint32_t)(overlap_camab_crop_total[1] / off_jobs) : 0U;
            uint32_t on_enc_avg = on_frames ? (uint32_t)(overlap_camab_enc_total[0] / on_frames) : 0U;
            uint32_t off_enc_avg = off_frames ? (uint32_t)(overlap_camab_enc_total[1] / off_frames) : 0U;
            uint32_t on_wait_avg = on_jobs ? (uint32_t)(overlap_camab_wait_total[0] / on_jobs) : 0U;
            uint32_t off_wait_avg = off_jobs ? (uint32_t)(overlap_camab_wait_total[1] / off_jobs) : 0U;
            uint32_t on_wall = (overlap_camab_phase_end[0] > overlap_camab_phase_start[0]) ?
                (uint32_t)(((uint64_t)on_frames * 100000000ULL) / (uint64_t)(overlap_camab_phase_end[0] - overlap_camab_phase_start[0])) : 0U;
            uint32_t off_wall = (overlap_camab_phase_end[1] > overlap_camab_phase_start[1]) ?
                (uint32_t)(((uint64_t)off_frames * 100000000ULL) / (uint64_t)(overlap_camab_phase_end[1] - overlap_camab_phase_start[1])) : 0U;
            uint32_t pump_dq_avg = overlap_camab_pump_frames ? (uint32_t)(overlap_camab_pump_dq_total / overlap_camab_pump_frames) : 0U;
            uint32_t pump_fps_x100 = (overlap_camab_phase_end[0] > overlap_camab_phase_start[0]) ?
                (uint32_t)(((uint64_t)overlap_camab_pump_frames * 100000000ULL) /
                           (uint64_t)(overlap_camab_phase_end[0] - overlap_camab_phase_start[0])) : 0U;
            bool pump_sustained = overlap_camab_pump_frames >= H264_OVERLAP_CAMAB_MIN_PUMP_FRAMES;
            int32_t crop_delta = (int32_t)on_crop_avg - (int32_t)off_crop_avg;
            int32_t enc_delta = (int32_t)on_enc_avg - (int32_t)off_enc_avg;
            int32_t wait_delta = (int32_t)on_wait_avg - (int32_t)off_wait_avg;
            int32_t wall_gain = (int32_t)off_wall - (int32_t)on_wall;
            int32_t crop_pct_x100 = off_crop_avg ? (int32_t)(((int64_t)crop_delta * 10000LL) / off_crop_avg) : 0;
            int32_t enc_pct_x100 = off_enc_avg ? (int32_t)(((int64_t)enc_delta * 10000LL) / off_enc_avg) : 0;
            bool raw_unchanged = overlap_camab_held_hash_initial == overlap_camab_held_hash_after_on &&
                                 overlap_camab_held_hash_initial == overlap_camab_held_hash_after_off;

            ESP_LOGI(TAG,
                     "H264-DIRECT OVERLAP-CAMAB PASS raw_same=%d pump=%u(%u.%02ufps sustained=%d) CAM_ON crop=%u enc=%u wait=%u wall=%u.%02u | CAM_OFF crop=%u enc=%u wait=%u wall=%u.%02u | delta crop=%ld enc=%ld wait=%ld wall_gain=%ld",
                     raw_unchanged ? 1 : 0, (unsigned)overlap_camab_pump_frames,
                     (unsigned)(pump_fps_x100/100U), (unsigned)(pump_fps_x100%100U), pump_sustained ? 1 : 0,
                     (unsigned)on_crop_avg, (unsigned)on_enc_avg, (unsigned)on_wait_avg,
                     (unsigned)(on_wall/100U), (unsigned)(on_wall%100U),
                     (unsigned)off_crop_avg, (unsigned)off_enc_avg, (unsigned)off_wait_avg,
                     (unsigned)(off_wall/100U), (unsigned)(off_wall%100U),
                     (long)crop_delta, (long)enc_delta, (long)wait_delta, (long)wall_gain);

            /* [OVERLAP-CAMAB-V5-PUMPFIX-20260823]
             * V5 keeps the V4 stack fix and raises the camera-pump task above the
             * owner so CAM_ON really sustains camera buffer recycling.
             * Do NOT place the JSON response buffer on the httpd task stack.
             * The httpd task has a ~9KB stack in this application; V3 pushed the
             * camera_h264_direct_bench_http() frame across its lower stack guard
             * before esp_video_init() could return. Keep the response in PSRAM. */
            const size_t overlapcamab_body_cap = 4300U;
            char *overlapcamab_body = (char *)heap_caps_malloc(
                overlapcamab_body_cap, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
            if (!overlapcamab_body) {
                s_state.running = false;
                return send_json_error(req, "503 Service Unavailable",
                                       "overlapcamab_json_alloc", ENOMEM, ESP_ERR_NO_MEM);
            }
            snprintf(overlapcamab_body, overlapcamab_body_cap,
                     "{\"ok\":true,\"test\":\"esp_h264_live_720p_overlap_camera_dma_sameboot_ab\","
                     "\"sensor\":\"OV02C10\",\"input\":\"same held 1920x1080 YU12/O_UYY_E_VYY -> centered CPU crop 1280x720\","
                     "\"encoder_api\":\"esp_h264_enc_process\",\"same_boot\":true,\"same_held_raw_frame\":true,"
                     "\"architecture\":{\"buffers\":2,\"crop_worker_core\":0,\"camera_pump_core\":0,\"camera_pump_priority_policy\":\"owner_equal_short_DQBUF_timeout_periodic_yield\",\"order\":\"CAM_ON_120_then_STREAMOFF_then_CAM_OFF_120\"},"
                     "\"encoder_resolution\":{\"width\":1280,\"height\":720},"
                     "\"held_raw\":{\"bytes\":%u,\"initial_hash\":\"%08" PRIx32 "\",\"after_cam_on_hash\":\"%08" PRIx32 "\",\"after_cam_off_hash\":\"%08" PRIx32 "\",\"unchanged\":%s},"
                     "\"cam_on\":{\"frames\":%u,\"camera_streaming\":true,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u,\"samples\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"post_encode_wait_us\":{\"avg\":%u,\"max\":%u,\"samples\":%u},\"camera_pump\":{\"frames\":%u,\"fps_x100\":%u,\"sustained\":%s,\"min_frames_required\":%u,\"dq_us_avg\":%u,\"dq_us_max\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u},\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"cam_off\":{\"frames\":%u,\"camera_streaming\":false,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u,\"samples\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"post_encode_wait_us\":{\"avg\":%u,\"max\":%u,\"samples\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u},\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\"},"
                     "\"delta\":{\"cam_on_minus_off_crop_us\":%ld,\"cam_on_minus_off_crop_percent_x100\":%ld,\"cam_on_minus_off_encode_us\":%ld,\"cam_on_minus_off_encode_percent_x100\":%ld,\"cam_on_minus_off_wait_us\":%ld,\"cam_off_wall_fps_gain_x100\":%ld},"
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"Controlled A/B V5: identical held raw source and identical 720p crop/H264 overlap. CAM_ON pump runs above owner priority but blocks in DQBUF, so it immediately recycles completed camera buffers without consuming Core1. Check camera_pump.sustained and fps_x100 before interpreting the A/B delta. Phase B begins after VIDIOC_STREAMOFF.\",\"reboot_before_retest\":true}",
                     (unsigned)overlap_ctx.fixed_src_len, overlap_camab_held_hash_initial, overlap_camab_held_hash_after_on,
                     overlap_camab_held_hash_after_off, raw_unchanged ? "true" : "false",
                     (unsigned)on_frames, (unsigned)on_wall, (unsigned)on_crop_avg,
                     (unsigned)(overlap_camab_crop_min[0] == UINT32_MAX ? 0U : overlap_camab_crop_min[0]), (unsigned)overlap_camab_crop_max[0], (unsigned)on_jobs,
                     (unsigned)on_enc_avg, (unsigned)(overlap_camab_enc_min[0] == UINT32_MAX ? 0U : overlap_camab_enc_min[0]), (unsigned)overlap_camab_enc_max[0],
                     (unsigned)on_wait_avg, (unsigned)overlap_camab_wait_max[0], (unsigned)on_jobs,
                     (unsigned)overlap_camab_pump_frames, (unsigned)pump_fps_x100, pump_sustained ? "true" : "false",
                     (unsigned)H264_OVERLAP_CAMAB_MIN_PUMP_FRAMES, (unsigned)pump_dq_avg, (unsigned)overlap_camab_pump_dq_max,
                     (unsigned)overlap_camab_idr[0], (unsigned)overlap_camab_i[0], (unsigned)overlap_camab_p[0], overlap_camab_hash_first[0], overlap_camab_hash_last[0],
                     (unsigned)off_frames, (unsigned)off_wall, (unsigned)off_crop_avg,
                     (unsigned)(overlap_camab_crop_min[1] == UINT32_MAX ? 0U : overlap_camab_crop_min[1]), (unsigned)overlap_camab_crop_max[1], (unsigned)off_jobs,
                     (unsigned)off_enc_avg, (unsigned)(overlap_camab_enc_min[1] == UINT32_MAX ? 0U : overlap_camab_enc_min[1]), (unsigned)overlap_camab_enc_max[1],
                     (unsigned)off_wait_avg, (unsigned)overlap_camab_wait_max[1], (unsigned)off_jobs,
                     (unsigned)overlap_camab_idr[1], (unsigned)overlap_camab_i[1], (unsigned)overlap_camab_p[1], overlap_camab_hash_first[1], overlap_camab_hash_last[1],
                     (long)crop_delta, (long)crop_pct_x100, (long)enc_delta, (long)enc_pct_x100,
                     (long)wait_delta, (long)wall_gain,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla, (unsigned)ps0, (unsigned)psa);
            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            esp_err_t overlapcamab_send_ret = httpd_resp_sendstr(req, overlapcamab_body);
            heap_caps_free(overlapcamab_body);
            return overlapcamab_send_ret;
        }

        if (requested_overlap) {
            uint32_t overlap_enc_avg = frames ? (uint32_t)(enc_us_total / frames) : 0U;
            uint32_t overlap_crop_avg = frames ? (uint32_t)(crop_us_total / frames) : 0U;
            uint32_t overlap_dq_avg = frames ? (uint32_t)(dq_us_total / frames) : 0U;
            uint32_t overlap_wait_avg = overlap_wait_count ? (uint32_t)(overlap_wait_total / overlap_wait_count) : 0U;
            uint32_t overlap_wall = (frames > 1U && t_last > t_first) ?
                (uint32_t)(((uint64_t)(frames - 1U) * 100000000ULL) /
                           (uint64_t)(t_last - t_first)) : 0U;
            uint32_t serial_equiv_us = overlap_crop_avg + overlap_enc_avg;
            uint32_t serial_equiv_fps = serial_equiv_us ? (uint32_t)(100000000ULL / serial_equiv_us) : 0U;

            ESP_LOGI(TAG,
                     "H264-DIRECT OVERLAP PASS frames=%u crop_avg=%u enc_avg=%u serial=%u (%u.%02u fps) wall=%u.%02u fps post_encode_wait_avg/max=%u/%u dq_avg=%u",
                     (unsigned)frames, (unsigned)overlap_crop_avg, (unsigned)overlap_enc_avg,
                     (unsigned)serial_equiv_us, (unsigned)(serial_equiv_fps / 100U),
                     (unsigned)(serial_equiv_fps % 100U), (unsigned)(overlap_wall / 100U),
                     (unsigned)(overlap_wall % 100U), (unsigned)overlap_wait_avg,
                     (unsigned)overlap_wait_max, (unsigned)overlap_dq_avg);

            char overlap_body[2600];
            snprintf(overlap_body, sizeof(overlap_body),
                     "{\"ok\":true,\"test\":\"esp_h264_live_720p_cpu_crop_h264_overlap\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1280x720 O_UYY_E_VYY centered CPU crop from native 1920x1080\","
                     "\"encoder_api\":\"esp_h264_enc_process\",\"same_boot\":true,"
                     "\"architecture\":{\"buffers\":2,\"crop_worker_core\":0,\"h264_hw_wait_yields_cpu\":true,\"order\":\"prime_frame0_then_overlap_crop_Nplus1_with_encode_N\"},"
                     "\"fresh_camera_frame_each_encode\":true,\"camera_streamon_entire_measurement\":true,"
                     "\"encoder_resolution\":{\"width\":1280,\"height\":720},"
                     "\"frames\":%u,\"wall_fps_x100\":%u,"
                     "\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u,\"prime\":%u},"
                     "\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},"
                     "\"camera_dq_us\":{\"avg\":%u,\"max\":%u,\"prime\":%u},"
                     "\"post_encode_wait_us\":{\"avg\":%u,\"max\":%u,\"samples\":%u},"
                     "\"serial_equivalent\":{\"us\":%u,\"fps_x100\":%u},"
                     "\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u},"
                     "\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\","
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"CPU crop remains byte-exact. A Core0 worker DQBUFs/crops/QBUFs frame N+1 while esp_h264_enc_process waits for H264 FRAME_DONE on frame N. Core1 is not used by the crop worker.\","
                     "\"reboot_before_retest\":true}",
                     (unsigned)frames, (unsigned)overlap_wall,
                     (unsigned)overlap_crop_avg, (unsigned)(crop_us_min == UINT32_MAX ? 0U : crop_us_min),
                     (unsigned)crop_us_max, (unsigned)overlap_prime_crop_us,
                     (unsigned)overlap_enc_avg, (unsigned)(enc_us_min == UINT32_MAX ? 0U : enc_us_min),
                     (unsigned)enc_us_max, (unsigned)overlap_dq_avg, (unsigned)dq_us_max,
                     (unsigned)overlap_prime_dq_us, (unsigned)overlap_wait_avg,
                     (unsigned)overlap_wait_max, (unsigned)overlap_wait_count,
                     (unsigned)serial_equiv_us, (unsigned)serial_equiv_fps,
                     (unsigned)idr_frames, (unsigned)i_frames, (unsigned)p_frames,
                     hash_first, hash_last,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, overlap_body);
        }

        if (requested_ppaab) {
            uint32_t cpu_frames = cropab_frames[0];
            uint32_t ppa_frames = cropab_frames[1];
            uint32_t cpu_crop_avg = cpu_frames ? (uint32_t)(cropab_crop_total[0] / cpu_frames) : 0U;
            uint32_t ppa_crop_avg = ppa_frames ? (uint32_t)(cropab_crop_total[1] / ppa_frames) : 0U;
            uint32_t cpu_enc_avg = cpu_frames ? (uint32_t)(cropab_enc_total[0] / cpu_frames) : 0U;
            uint32_t ppa_enc_avg = ppa_frames ? (uint32_t)(cropab_enc_total[1] / ppa_frames) : 0U;
            uint32_t cpu_dq_avg = cpu_frames ? (uint32_t)(cropab_dq_total[0] / cpu_frames) : 0U;
            uint32_t ppa_dq_avg = ppa_frames ? (uint32_t)(cropab_dq_total[1] / ppa_frames) : 0U;
            uint32_t cpu_pipe = cpu_crop_avg + cpu_enc_avg;
            uint32_t ppa_pipe = ppa_crop_avg + ppa_enc_avg;
            uint32_t cpu_pipe_fps = cpu_pipe ? (uint32_t)(100000000ULL / cpu_pipe) : 0U;
            uint32_t ppa_pipe_fps = ppa_pipe ? (uint32_t)(100000000ULL / ppa_pipe) : 0U;
            uint32_t cpu_wall = (cpu_frames > 1U && cropab_t_last[0] > cropab_t_first[0]) ?
                (uint32_t)(((uint64_t)(cpu_frames - 1U) * 100000000ULL) /
                           (uint64_t)(cropab_t_last[0] - cropab_t_first[0])) : 0U;
            uint32_t ppa_wall = (ppa_frames > 1U && cropab_t_last[1] > cropab_t_first[1]) ?
                (uint32_t)(((uint64_t)(ppa_frames - 1U) * 100000000ULL) /
                           (uint64_t)(cropab_t_last[1] - cropab_t_first[1])) : 0U;
            uint32_t crop_speedup_x100 = ppa_crop_avg ?
                (uint32_t)(((uint64_t)cpu_crop_avg * 100ULL) / ppa_crop_avg) : 0U;
            int32_t crop_saved_us = (int32_t)cpu_crop_avg - (int32_t)ppa_crop_avg;
            int32_t pipeline_saved_us = (int32_t)cpu_pipe - (int32_t)ppa_pipe;

            ESP_LOGI(TAG,
                     "H264-DIRECT PPAAB PASS validate_mismatch=%u CPU crop=%u pipe=%u wall=%u.%02u | PPA crop=%u pipe=%u wall=%u.%02u | crop speedup=%u.%02ux saved=%ldus pipeline_saved=%ldus",
                     (unsigned)ppa_validation_mismatches,
                     (unsigned)cpu_crop_avg, (unsigned)cpu_pipe,
                     (unsigned)(cpu_wall / 100U), (unsigned)(cpu_wall % 100U),
                     (unsigned)ppa_crop_avg, (unsigned)ppa_pipe,
                     (unsigned)(ppa_wall / 100U), (unsigned)(ppa_wall % 100U),
                     (unsigned)(crop_speedup_x100 / 100U), (unsigned)(crop_speedup_x100 % 100U),
                     (long)crop_saved_us, (long)pipeline_saved_us);

            char ppaab_body[3200];
            snprintf(ppaab_body, sizeof(ppaab_body),
                     "{\"ok\":true,\"test\":\"esp_h264_live_720p_cpu_vs_ppa_srm_crop_ab\","
                     "\"sensor\":\"OV02C10\",\"input\":\"1280x720 YU12/O_UYY_E_VYY centered crop from native 1920x1080\","
                     "\"encoder_api\":\"esp_h264_enc_process\",\"same_boot\":true,\"order\":\"validate_same_frame_then_cpu_memcpy_120_then_ppa_srm_120\","
                     "\"fresh_camera_frame_each_encode\":true,\"camera_streamon_entire_measurement\":true,"
                     "\"encoder_resolution\":{\"width\":1280,\"height\":720},"
                     "\"ppa\":{\"engine\":\"SRM\",\"color_mode\":\"RGB888_reinterpret_O_UYY_E_VYY\",\"scale_x\":1.0,\"scale_y\":1.0,\"rotation\":0,\"burst_bytes\":128,\"blocking\":true},"
                     "\"validation\":{\"done\":%s,\"byte_identical\":%s,\"bytes\":%u,\"mismatches\":%u,\"first_mismatch\":%u,\"cpu_hash\":\"%08" PRIx32 "\",\"ppa_hash\":\"%08" PRIx32 "\",\"cpu_us\":%u,\"ppa_us\":%u},"
                     "\"cpu\":{\"frames\":%u,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"pipeline_us\":{\"avg\":%u,\"fps_x100\":%u},\"camera_dq_us\":{\"avg\":%u,\"max\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u}},"
                     "\"ppa_srm\":{\"frames\":%u,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"pipeline_us\":{\"avg\":%u,\"fps_x100\":%u},\"camera_dq_us\":{\"avg\":%u,\"max\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u}},"
                     "\"delta\":{\"crop_speedup_x100\":%u,\"crop_saved_us\":%ld,\"pipeline_saved_us\":%ld,\"wall_fps_gain_x100\":%ld},"
                     "\"dma_heap\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"PPA phase is allowed only after one fresh frame produces a byte-identical 1382400-byte result versus the known-good CPU O_UYY_E_VYY crop. Same H264 input buffer address is reused in both measured phases.\","
                     "\"reboot_before_retest\":true}",
                     ppa_validation_done ? "true" : "false",
                     (ppa_validation_done && ppa_validation_mismatches == 0U) ? "true" : "false",
                     (unsigned)H264_MEMCROP_BYTES,
                     (unsigned)ppa_validation_mismatches,
                     (unsigned)(ppa_validation_first_mismatch == UINT32_MAX ? 0U : ppa_validation_first_mismatch),
                     ppa_validation_cpu_hash, ppa_validation_ppa_hash,
                     (unsigned)ppa_validation_cpu_us, (unsigned)ppa_validation_ppa_us,
                     (unsigned)cpu_frames, (unsigned)cpu_wall,
                     (unsigned)cpu_crop_avg, (unsigned)(cropab_crop_min[0] == UINT32_MAX ? 0U : cropab_crop_min[0]), (unsigned)cropab_crop_max[0],
                     (unsigned)cpu_enc_avg, (unsigned)(cropab_enc_min[0] == UINT32_MAX ? 0U : cropab_enc_min[0]), (unsigned)cropab_enc_max[0],
                     (unsigned)cpu_pipe, (unsigned)cpu_pipe_fps,
                     (unsigned)cpu_dq_avg, (unsigned)cropab_dq_max[0],
                     (unsigned)cropab_idr[0], (unsigned)cropab_i[0], (unsigned)cropab_p[0],
                     (unsigned)ppa_frames, (unsigned)ppa_wall,
                     (unsigned)ppa_crop_avg, (unsigned)(cropab_crop_min[1] == UINT32_MAX ? 0U : cropab_crop_min[1]), (unsigned)cropab_crop_max[1],
                     (unsigned)ppa_enc_avg, (unsigned)(cropab_enc_min[1] == UINT32_MAX ? 0U : cropab_enc_min[1]), (unsigned)cropab_enc_max[1],
                     (unsigned)ppa_pipe, (unsigned)ppa_pipe_fps,
                     (unsigned)ppa_dq_avg, (unsigned)cropab_dq_max[1],
                     (unsigned)cropab_idr[1], (unsigned)cropab_i[1], (unsigned)cropab_p[1],
                     (unsigned)crop_speedup_x100, (long)crop_saved_us, (long)pipeline_saved_us,
                     (long)((int32_t)ppa_wall - (int32_t)cpu_wall),
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, ppaab_body);
        }

        if (requested_cropab) {
            uint32_t cpu_frames = cropab_frames[0];
            uint32_t dma_frames = cropab_frames[1];
            uint32_t cpu_crop_avg = cpu_frames ? (uint32_t)(cropab_crop_total[0] / cpu_frames) : 0U;
            uint32_t dma_crop_avg = dma_frames ? (uint32_t)(cropab_crop_total[1] / dma_frames) : 0U;
            uint32_t cpu_enc_avg = cpu_frames ? (uint32_t)(cropab_enc_total[0] / cpu_frames) : 0U;
            uint32_t dma_enc_avg = dma_frames ? (uint32_t)(cropab_enc_total[1] / dma_frames) : 0U;
            uint32_t cpu_dq_avg = cpu_frames ? (uint32_t)(cropab_dq_total[0] / cpu_frames) : 0U;
            uint32_t dma_dq_avg = dma_frames ? (uint32_t)(cropab_dq_total[1] / dma_frames) : 0U;
            uint32_t cpu_pipe = cpu_crop_avg + cpu_enc_avg;
            uint32_t dma_pipe = dma_crop_avg + dma_enc_avg;
            uint32_t cpu_pipe_fps = cpu_pipe ? (uint32_t)(100000000ULL / cpu_pipe) : 0U;
            uint32_t dma_pipe_fps = dma_pipe ? (uint32_t)(100000000ULL / dma_pipe) : 0U;
            uint32_t cpu_wall = (cpu_frames > 1U && cropab_t_last[0] > cropab_t_first[0]) ?
                (uint32_t)(((uint64_t)(cpu_frames - 1U) * 100000000ULL) /
                           (uint64_t)(cropab_t_last[0] - cropab_t_first[0])) : 0U;
            uint32_t dma_wall = (dma_frames > 1U && cropab_t_last[1] > cropab_t_first[1]) ?
                (uint32_t)(((uint64_t)(dma_frames - 1U) * 100000000ULL) /
                           (uint64_t)(cropab_t_last[1] - cropab_t_first[1])) : 0U;
            uint32_t crop_speedup_x100 = dma_crop_avg ?
                (uint32_t)(((uint64_t)cpu_crop_avg * 100ULL) / dma_crop_avg) : 0U;
            int32_t crop_saved_us = (int32_t)cpu_crop_avg - (int32_t)dma_crop_avg;
            int32_t pipeline_saved_us = (int32_t)cpu_pipe - (int32_t)dma_pipe;

            ESP_LOGI(TAG,
                     "H264-DIRECT CROPAB PASS CPU crop=%u pipe=%u wall=%u.%02u | AXI-GDMA crop=%u pipe=%u wall=%u.%02u | crop speedup=%u.%02ux saved=%ldus pipeline_saved=%ldus",
                     (unsigned)cpu_crop_avg, (unsigned)cpu_pipe,
                     (unsigned)(cpu_wall / 100U), (unsigned)(cpu_wall % 100U),
                     (unsigned)dma_crop_avg, (unsigned)dma_pipe,
                     (unsigned)(dma_wall / 100U), (unsigned)(dma_wall % 100U),
                     (unsigned)(crop_speedup_x100 / 100U), (unsigned)(crop_speedup_x100 % 100U),
                     (long)crop_saved_us, (long)pipeline_saved_us);

            char cropab_body[2800];
            snprintf(cropab_body, sizeof(cropab_body),
                     "{\"ok\":true,\"test\":\"esp_h264_live_720p_cpu_vs_axi_gdma_crop_ab\"," 
                     "\"sensor\":\"OV02C10\",\"input\":\"1280x720 YU12/O_UYY_E_VYY centered memory crop from native 1920x1080\"," 
                     "\"encoder_api\":\"esp_h264_enc_process\",\"same_boot\":true,\"order\":\"cpu_memcpy_120_then_axi_gdma_120\"," 
                     "\"fresh_camera_frame_each_encode\":true,\"camera_streamon_entire_measurement\":true,"
                     "\"encoder_resolution\":{\"width\":1280,\"height\":720},"
                     "\"dma_crop_config\":{\"backend\":\"AXI_GDMA\",\"backlog\":%u,\"psram_align\":%u,\"burst\":%u,\"batch_rows\":%u},"
                     "\"cpu\":{\"frames\":%u,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"pipeline_us\":{\"avg\":%u,\"fps_x100\":%u},\"camera_dq_us\":{\"avg\":%u,\"max\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u}},"
                     "\"dma\":{\"frames\":%u,\"wall_fps_x100\":%u,\"crop_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"encode_us\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"pipeline_us\":{\"avg\":%u,\"fps_x100\":%u},\"camera_dq_us\":{\"avg\":%u,\"max\":%u},\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u}},"
                     "\"delta\":{\"crop_speedup_x100\":%u,\"crop_saved_us\":%ld,\"pipeline_saved_us\":%ld,\"wall_fps_gain_x100\":%ld},"
                     "\"dma_heap\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"Same-boot A/B on ESP32-P4 rev1.3: CPU row memcpy versus AXI-GDMA 32-byte-aligned row-copy. H264, crop geometry, camera stream, LCD and fresh-frame policy are unchanged.\","
                     "\"reboot_before_retest\":true}",
                     (unsigned)H264_DMA_CROP_BACKLOG, (unsigned)H264_DMA_CROP_ALIGN,
                     (unsigned)H264_DMA_CROP_ALIGN, (unsigned)H264_DMA_CROP_BATCH_ROWS,
                     (unsigned)cpu_frames, (unsigned)cpu_wall,
                     (unsigned)cpu_crop_avg, (unsigned)(cropab_crop_min[0] == UINT32_MAX ? 0U : cropab_crop_min[0]), (unsigned)cropab_crop_max[0],
                     (unsigned)cpu_enc_avg, (unsigned)(cropab_enc_min[0] == UINT32_MAX ? 0U : cropab_enc_min[0]), (unsigned)cropab_enc_max[0],
                     (unsigned)cpu_pipe, (unsigned)cpu_pipe_fps,
                     (unsigned)cpu_dq_avg, (unsigned)cropab_dq_max[0],
                     (unsigned)cropab_idr[0], (unsigned)cropab_i[0], (unsigned)cropab_p[0],
                     (unsigned)dma_frames, (unsigned)dma_wall,
                     (unsigned)dma_crop_avg, (unsigned)(cropab_crop_min[1] == UINT32_MAX ? 0U : cropab_crop_min[1]), (unsigned)cropab_crop_max[1],
                     (unsigned)dma_enc_avg, (unsigned)(cropab_enc_min[1] == UINT32_MAX ? 0U : cropab_enc_min[1]), (unsigned)cropab_enc_max[1],
                     (unsigned)dma_pipe, (unsigned)dma_pipe_fps,
                     (unsigned)dma_dq_avg, (unsigned)cropab_dq_max[1],
                     (unsigned)cropab_idr[1], (unsigned)cropab_i[1], (unsigned)cropab_p[1],
                     (unsigned)crop_speedup_x100, (long)crop_saved_us, (long)pipeline_saved_us,
                     (long)((int32_t)dma_wall - (int32_t)cpu_wall),
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, cropab_body);
        }

        if (requested_livegop || requested_live720) {
            const char *live_pass_label = requested_live720 ?
                "H264-DIRECT LIVE720-MEMCROP-GOP30 PASS" : "H264-DIRECT LIVE-GOP30 FINAL PASS";
            const char *live_test_name = requested_live720 ?
                "esp_h264_live_720p_memcrop_gop30" : "esp_h264_live_1080p_gop30_final";
            const char *live_input_name = requested_live720 ?
                "1280x720 YU12/O_UYY_E_VYY centered memory crop" : "1920x1080 YU12/O_UYY_E_VYY";
            const char *live_note = requested_live720 ?
                "1280x720 live-camera GOP30 using centered memory row-copy crop from native 1920x1080 YU12 because ESP32-P4 rev1.x has no ISP crop hardware; every encode uses a fresh camera frame while CSI/ISP/DMA and normal LCD remain active." :
                "Final 1080p live-camera GOP30 characterization: every encode uses a fresh V4L2 DQBUF frame while camera CSI/ISP/DMA and normal LCD remain active.";

            ESP_LOGI(TAG,
                     "%s frames=%u types{IDR=%u,I=%u,P=%u} "
                     "enc_us{IDR=%u/%u/%u P=%u/%u/%u} bytes{IDR=%u/%u/%u P=%u/%u/%u} "
                     "wall=%u.%02u encode_only=%u.%02u pipeline=%u.%02u dq_avg=%u crop_avg=%u capture=%ux%u encoder=%ux%u",
                     live_pass_label,
                     (unsigned)frames, (unsigned)idr_frames, (unsigned)i_frames, (unsigned)p_frames,
                     (unsigned)idr_us_avg, (unsigned)(idr_us_min == UINT32_MAX ? 0U : idr_us_min), (unsigned)idr_us_max,
                     (unsigned)p_us_avg, (unsigned)(p_us_min == UINT32_MAX ? 0U : p_us_min), (unsigned)p_us_max,
                     (unsigned)idr_bytes_avg, (unsigned)(idr_bytes_min == UINT32_MAX ? 0U : idr_bytes_min), (unsigned)idr_bytes_max,
                     (unsigned)p_bytes_avg, (unsigned)(p_bytes_min == UINT32_MAX ? 0U : p_bytes_min), (unsigned)p_bytes_max,
                     (unsigned)(wall_fps_x100 / 100U), (unsigned)(wall_fps_x100 % 100U),
                     (unsigned)(process_fps_x100 / 100U), (unsigned)(process_fps_x100 % 100U),
                     (unsigned)(pipeline_fps_x100 / 100U), (unsigned)(pipeline_fps_x100 % 100U),
                     (unsigned)dq_avg, (unsigned)crop_avg,
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     (unsigned)cfg.res.width, (unsigned)cfg.res.height);

            char live_body[3000];
            snprintf(live_body, sizeof(live_body),
                     "{\"ok\":true,\"test\":\"%s\","
                     "\"sensor\":\"OV02C10\",\"input\":\"%s\","
                     "\"encoder_api\":\"esp_h264_enc_process\","
                     "\"capture_requested\":{\"width\":%u,\"height\":%u},"
                     "\"capture_actual\":{\"width\":%u,\"height\":%u,\"fourcc\":\"YU12\"},"
                     "\"encoder_resolution\":{\"width\":%u,\"height\":%u},"
                     "\"config\":{\"fps\":30,\"gop\":30,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"camoff\":false,\"camhold\":false,\"livegop\":%s,\"live720\":%s,\"h264qos\":0},"
                     "\"frames\":%u,\"fresh_camera_frame_each_encode\":true,\"camera_streamon_entire_measurement\":true,"
                     "\"wall_fps_x100\":%u,\"process_fps_x100\":%u,\"pipeline_fps_x100\":%u,"
                     "\"crop_us\":{\"last\":%u,\"avg\":%u,\"min\":%u,\"max\":%u},"
                     "\"pipeline_us\":{\"avg\":%u},"
                     "\"encode_us\":{\"last\":%u,\"avg\":%u,\"min\":%u,\"max\":%u},"
                     "\"camera_dq_us\":{\"last\":%u,\"avg\":%u,\"max\":%u},"
                     "\"h264_bytes\":{\"avg\":%u,\"min\":%u,\"max\":%u},"
                     "\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u},"
                     "\"encode_by_type_us\":{\"idr\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"i\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"p\":{\"avg\":%u,\"min\":%u,\"max\":%u}},"
                     "\"bytes_by_type\":{\"idr\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"i\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"p\":{\"avg\":%u,\"min\":%u,\"max\":%u}},"
                     "\"nal\":{\"sps\":%u,\"pps\":%u,\"idr\":%u,\"non_idr\":%u},"
                     "\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\","
                     "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                     "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                     "\"note\":\"%s\","
                     "\"reboot_before_retest\":true}",
                     live_test_name, live_input_name,
                     requested_live720 ? 1280U : (unsigned)sensor_fmt.width,
                     requested_live720 ? 720U : (unsigned)sensor_fmt.height,
                     (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
                     (unsigned)cfg.res.width, (unsigned)cfg.res.height,
                     (unsigned)cfg.rc.qp_min, (unsigned)cfg.rc.qp_max,
                     (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                     requested_livegop ? "true" : "false", requested_live720 ? "true" : "false",
                     (unsigned)frames, (unsigned)wall_fps_x100, (unsigned)process_fps_x100, (unsigned)pipeline_fps_x100,
                     (unsigned)crop_us_last, (unsigned)crop_avg,
                     (unsigned)(crop_us_min == UINT32_MAX ? 0U : crop_us_min), (unsigned)crop_us_max,
                     (unsigned)pipeline_avg,
                     (unsigned)enc_us_last, (unsigned)enc_avg,
                     (unsigned)(enc_us_min == UINT32_MAX ? 0U : enc_us_min), (unsigned)enc_us_max,
                     (unsigned)dq_us_last, (unsigned)dq_avg, (unsigned)dq_us_max,
                     (unsigned)bytes_avg, (unsigned)(bytes_min == UINT32_MAX ? 0U : bytes_min), (unsigned)bytes_max,
                     (unsigned)idr_frames, (unsigned)i_frames, (unsigned)p_frames,
                     (unsigned)idr_us_avg, (unsigned)(idr_us_min == UINT32_MAX ? 0U : idr_us_min), (unsigned)idr_us_max,
                     (unsigned)i_us_avg, (unsigned)(i_us_min == UINT32_MAX ? 0U : i_us_min), (unsigned)i_us_max,
                     (unsigned)p_us_avg, (unsigned)(p_us_min == UINT32_MAX ? 0U : p_us_min), (unsigned)p_us_max,
                     (unsigned)idr_bytes_avg, (unsigned)(idr_bytes_min == UINT32_MAX ? 0U : idr_bytes_min), (unsigned)idr_bytes_max,
                     (unsigned)i_bytes_avg, (unsigned)(i_bytes_min == UINT32_MAX ? 0U : i_bytes_min), (unsigned)i_bytes_max,
                     (unsigned)p_bytes_avg, (unsigned)(p_bytes_min == UINT32_MAX ? 0U : p_bytes_min), (unsigned)p_bytes_max,
                     (unsigned)nal.sps, (unsigned)nal.pps, (unsigned)nal.idr, (unsigned)nal.non_idr,
                     hash_first, hash_last,
                     (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                     (unsigned)ps0, (unsigned)psa,
                     live_note);

            s_state.running = false;
            httpd_resp_set_type(req, "application/json; charset=utf-8");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_sendstr(req, live_body);
        }

        ESP_LOGI(TAG,
                 "H264-DIRECT PASS gop=%u qp=%d..%d outKB=%u frames=%u wall=%u.%02u process=%u.%02u enc_us avg/min/max=%u/%u/%u IDRavg=%u Pavg=%u dq_avg=%u NAL{%u,%u,%u,%u}",
                 (unsigned)cfg.gop,
                 (int)cfg.rc.qp_min, (int)cfg.rc.qp_max, (unsigned)(out_capacity / 1024U),
                 (unsigned)frames,
                 (unsigned)(wall_fps_x100 / 100), (unsigned)(wall_fps_x100 % 100),
                 (unsigned)(process_fps_x100 / 100), (unsigned)(process_fps_x100 % 100),
                 (unsigned)enc_avg, (unsigned)enc_us_min, (unsigned)enc_us_max,
                 (unsigned)idr_us_avg, (unsigned)p_us_avg, (unsigned)dq_avg,
                 (unsigned)nal.sps, (unsigned)nal.pps,
                 (unsigned)nal.idr, (unsigned)nal.non_idr);

        char body[2100];
        snprintf(body, sizeof(body),
                 "{\"ok\":true,\"test\":\"esp_h264_direct_hw\"," 
                 "\"sensor\":\"OV02C10\",\"input\":\"1920x1080 YU12/O_UYY_E_VYY\"," 
                 "\"encoder_api\":\"esp_h264_enc_process\"," 
                 "\"config\":{\"fps\":30,\"gop\":%u,\"bitrate\":4000000,\"qp_min\":%u,\"qp_max\":%u,\"fixed_qp\":%s,\"out_capacity\":%u,\"camoff\":%s,\"camhold\":%s,\"camseq\":false,\"h264qos\":%d},"
                 "\"frames\":%u,\"wall_fps_x100\":%u,\"process_fps_x100\":%u,"
                 "\"encode_us\":{\"last\":%u,\"avg\":%u,\"min\":%u,\"max\":%u},"
                 "\"camera_dq_us\":{\"last\":%u,\"avg\":%u,\"max\":%u},"
                 "\"h264_bytes\":{\"avg\":%u,\"min\":%u,\"max\":%u},"
                 "\"frame_types\":{\"idr\":%u,\"i\":%u,\"p\":%u},"
                 "\"encode_by_type_us\":{\"idr\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"i\":{\"avg\":%u,\"min\":%u,\"max\":%u},\"p\":{\"avg\":%u,\"min\":%u,\"max\":%u}},"
                 "\"nal\":{\"sps\":%u,\"pps\":%u,\"idr\":%u,\"non_idr\":%u},"
                 "\"hash_first\":\"%08" PRIx32 "\",\"hash_last\":\"%08" PRIx32 "\",\"held_raw_hash\":\"%08" PRIx32 "\","
                 "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                 "\"psram\":{\"free_before\":%u,\"free_after\":%u},"
                 "\"h264_clock_raw\":{\"peri26_open\":\"0x%08" PRIx32 "\",\"peri27_open\":\"0x%08" PRIx32 "\",\"src_sel\":%ld,\"peri26_end\":\"0x%08" PRIx32 "\"},"
                 "\"compare_to\":\"GOP1 1080p same-frame A/B; camhold keeps CSI/ISP camera DMA active\","
                 "\"reboot_before_retest\":true}",
                 (unsigned)cfg.gop,
                 (unsigned)cfg.rc.qp_min, (unsigned)cfg.rc.qp_max,
                 (fixed_qp >= 0) ? "true" : "false", (unsigned)out_capacity,
                 requested_camoff ? "true" : "false",
                 requested_camhold ? "true" : "false",
                 requested_h264_qos,
                 (unsigned)frames, (unsigned)wall_fps_x100, (unsigned)process_fps_x100,
                 (unsigned)enc_us_last, (unsigned)enc_avg,
                 (unsigned)(enc_us_min == UINT32_MAX ? 0 : enc_us_min), (unsigned)enc_us_max,
                 (unsigned)dq_us_last, (unsigned)dq_avg, (unsigned)dq_us_max,
                 (unsigned)bytes_avg, (unsigned)(bytes_min == UINT32_MAX ? 0 : bytes_min), (unsigned)bytes_max,
                 (unsigned)idr_frames, (unsigned)i_frames, (unsigned)p_frames,
                 (unsigned)idr_us_avg, (unsigned)(idr_us_min == UINT32_MAX ? 0 : idr_us_min), (unsigned)idr_us_max,
                 (unsigned)i_us_avg, (unsigned)(i_us_min == UINT32_MAX ? 0 : i_us_min), (unsigned)i_us_max,
                 (unsigned)p_us_avg, (unsigned)(p_us_min == UINT32_MAX ? 0 : p_us_min), (unsigned)p_us_max,
                 (unsigned)nal.sps, (unsigned)nal.pps, (unsigned)nal.idr, (unsigned)nal.non_idr,
                 hash_first, hash_last, held_raw_hash,
                 (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                 (unsigned)ps0, (unsigned)psa,
                 h264_clk_open.peri26, h264_clk_open.peri27,
                 (long)h264_clk_open.h264_src_sel, h264_clk_end.peri26);

        s_state.running = false;
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_sendstr(req, body);
    }

direct_fail:
    if (requested_fpsab) {
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            (void)camera_diag_lcd_pattern_leave(lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
        }
        lcd_ui_set_camera_stream_active(false);
    }
    if (requested_priab) {
        if (priab_prio_raised && priab_task) {
            vTaskPrioritySet(priab_task, priab_prio_normal);
            priab_prio_raised = false;
        }
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            (void)camera_diag_lcd_pattern_leave(lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
        }
        lcd_ui_set_camera_stream_active(false);
    }
    if (requested_lcdpat) {
        /* Failure after phase2 must also be non-destructive: restore the
         * framebuffer path and LVGL timer, then release the UI guard. */
        if (lcdpat_active || lcdpat_lvgl_timer_paused) {
            (void)camera_diag_lcd_pattern_leave(lcdpat_panel, &lcdpat_lvgl_timer_paused);
            lcdpat_active = false;
        }
        lcd_ui_set_camera_stream_active(false);
    }
    if (h264_qos_applied) {
        axi_icm_ll_set_h264_dma_qos_arbiter_prio(
            0, h264_qos_saved_m1_w, h264_qos_saved_m1_r);
        axi_icm_ll_set_h264_dma_qos_arbiter_prio(
            1, h264_qos_saved_m2_w, h264_qos_saved_m2_r);
        h264_qos_applied = false;
        axi_qos_diag_t q = axi_qos_diag_read();
        axi_qos_diag_log("fail-path-h264-qos-restored", &q);
        ESP_LOGW(TAG, "[AXI-QOS-AB] failure path restored original H264 M1/M2 AR/AW QoS");
    }
    ESP_LOGE(TAG, "H264-DIRECT FAIL stage=%s errno=%d h264_err=%d",
             fail_stage ? fail_stage : "unknown", saved_errno, (int)saved_h264_err);
    if (requested_overlapcamab && overlap_pump_started) {
        overlap_pump_ctx.stop = true;
        if (cam_streaming && cam_fd >= 0) {
            int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            (void)ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
            cam_streaming = false;
        }
        if (overlap_pump_ctx.exit_sem) (void)xSemaphoreTake(overlap_pump_ctx.exit_sem, pdMS_TO_TICKS(1000));
        overlap_pump_started = false;
        overlap_pump_ctx.task = NULL;
    }
    if (requested_overlap) {
        /* If the worker is blocked in DQBUF, STREAMOFF wakes it before join. */
        if (overlap_worker_started && cam_streaming && cam_fd >= 0) {
            int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            (void)ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
            cam_streaming = false;
        }
        if (overlap_worker_started) {
            overlap_ctx.stop = true;
            (void)xSemaphoreGive(overlap_ctx.kick_sem);
            (void)xSemaphoreTake(overlap_ctx.exit_sem, pdMS_TO_TICKS(4000));
            overlap_worker_started = false;
            overlap_ctx.task = NULL;
        }
        if (overlap_ctx.kick_sem) { vSemaphoreDelete(overlap_ctx.kick_sem); overlap_ctx.kick_sem = NULL; }
        if (overlap_ctx.done_sem) { vSemaphoreDelete(overlap_ctx.done_sem); overlap_ctx.done_sem = NULL; }
        if (overlap_ctx.exit_sem) { vSemaphoreDelete(overlap_ctx.exit_sem); overlap_ctx.exit_sem = NULL; }
    }
    if (cam_streaming && cam_fd >= 0) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
    }
    if (enc_opened && enc) (void)esp_h264_enc_close(enc);
    if (enc) (void)esp_h264_enc_del(enc);
    if (crop_dma_installed && crop_dma_hdl) {
        (void)esp_async_memcpy_uninstall(crop_dma_hdl);
        crop_dma_hdl = NULL;
        crop_dma_installed = false;
    }
    if (crop_dma_done) {
        vSemaphoreDelete(crop_dma_done);
        crop_dma_done = NULL;
    }
    if (crop_ppa_client) {
        (void)ppa_unregister_client(crop_ppa_client);
        crop_ppa_client = NULL;
    }
    if (ppa_verify_buf) {
        heap_caps_free(ppa_verify_buf);
        ppa_verify_buf = NULL;
    }
    if (overlap_pump_ctx.exit_sem) {
        vSemaphoreDelete(overlap_pump_ctx.exit_sem);
        overlap_pump_ctx.exit_sem = NULL;
    }
    if (out_buf) heap_caps_free(out_buf);
    if (memcrop_buf) heap_caps_free(memcrop_buf);
    if (memcrop_buf2) heap_caps_free(memcrop_buf2);
    for (uint32_t i = 0; i < mapped_count; ++i) {
        if (maps[i].start) munmap(maps[i].start, maps[i].length);
    }
    if (cam_fd >= 0) close(cam_fd);
    s_state.running = false;

    char body[1100];
    snprintf(body, sizeof(body),
             "{\"ok\":false,\"test\":\"%s\",\"stage\":\"%s\","
             "\"errno\":%d,\"h264_err\":%d,\"requested_capture\":\"%s\",\"actual_capture\":{\"width\":%u,\"height\":%u},"
             "\"ppa_validation\":{\"done\":%s,\"bytes\":%u,\"mismatches\":%u,\"first_mismatch\":%u,\"cpu_hash\":\"%08" PRIx32 "\",\"ppa_hash\":\"%08" PRIx32 "\",\"cpu_us\":%u,\"ppa_us\":%u},"
             "\"reboot_before_retest\":true}",
             requested_overlapcamab ? "esp_h264_live_720p_overlap_camera_dma_sameboot_ab" :
             (requested_overlap ? "esp_h264_live_720p_cpu_crop_h264_overlap" :
             (requested_ppaab ? "esp_h264_live_720p_cpu_vs_ppa_srm_crop_ab" :
              (requested_cropab ? "esp_h264_live_720p_cpu_vs_axi_gdma_crop_ab" :
               (requested_live720 ? "esp_h264_live_720p_memcrop_gop30" : "esp_h264_direct_hw")))),
             fail_stage ? fail_stage : "unknown", saved_errno, (int)saved_h264_err,
             requested_live720 ? "1280x720 YU12 via centered memory crop from native capture" : "sensor-native YU12",
             (unsigned)diag_capture_actual_width, (unsigned)diag_capture_actual_height,
             ppa_validation_done ? "true" : "false",
             (unsigned)H264_MEMCROP_BYTES, (unsigned)ppa_validation_mismatches,
             (unsigned)(ppa_validation_first_mismatch == UINT32_MAX ? 0U : ppa_validation_first_mismatch),
             ppa_validation_cpu_hash, ppa_validation_ppa_hash,
             (unsigned)ppa_validation_cpu_us, (unsigned)ppa_validation_ppa_us);
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, body);
}

esp_err_t camera_csi_raw10_diag_http(httpd_req_t *req)
{
    if (!req) return ESP_ERR_INVALID_ARG;
    if (s_state.running || camera_h264_stream_is_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera busy\"}");
    }
    if (s_state.consumed) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        return httpd_resp_sendstr(req,
            "{\"ok\":false,\"error\":\"camera diagnostic already consumed\","
            "\"reboot_before_retest\":true}");
    }

    s_state.running = true;
    s_state.consumed = true;
    int fd = -1;
    bool streaming = false;
    cam_map_buf_t maps[CAM_CSI_BUFFER_COUNT] = {0};
    uint32_t mapped_count = 0;
    int saved_errno = 0;
    const char *fail_stage = NULL;
    esp_cam_sensor_format_t sensor_fmt = {0};
    uint32_t frames = 0;
    uint32_t dq_us_last = 0, dq_us_max = 0;
    uint64_t dq_us_sum = 0;
    uint32_t bytes_min = UINT32_MAX, bytes_max = 0;
    int64_t t_first = 0, t_last = 0;

    const size_t dma0 = dma_free(), dl0 = dma_largest(), ps0 = psram_free();
    ESP_LOGI(TAG, "RAW10 one-shot begin dma=%u/%u psram=%u",
             (unsigned)dma0, (unsigned)dl0, (unsigned)ps0);

    esp_err_t ret = init_video_once();
    if (ret != ESP_OK) {
        s_state.running = false;
        return send_json_error(req, "500 Internal Server Error", "raw10_esp_video_init", 0, ret);
    }

    fd = open(CAM_CSI_DEVICE, O_RDWR);
    if (fd < 0) { saved_errno = errno; fail_stage = "raw10_open_video0"; goto raw_fail; }
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &sensor_fmt) != 0) {
        saved_errno = errno; fail_stage = "raw10_G_SENSOR_FMT"; goto raw_fail;
    }
    if (!sensor_fmt.width || !sensor_fmt.height) {
        saved_errno = EINVAL; fail_stage = "raw10_sensor_fmt_invalid"; goto raw_fail;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = sensor_fmt.width;
    fmt.fmt.pix.height = sensor_fmt.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        saved_errno = errno; fail_stage = "raw10_S_FMT_BG10"; goto raw_fail;
    }
    char fcc[5]; fourcc_to_str(fmt.fmt.pix.pixelformat, fcc);
    ESP_LOGI(TAG, "RAW10 active=%ux%u fourcc=%s sizeimage=%u",
             (unsigned)fmt.fmt.pix.width, (unsigned)fmt.fmt.pix.height,
             fcc, (unsigned)fmt.fmt.pix.sizeimage);
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_SBGGR10) {
        saved_errno = EINVAL; fail_stage = "raw10_driver_changed_format"; goto raw_fail;
    }

    camera_streamparm_30fps_best_effort(fd, sensor_fmt.fps);
    struct timeval timeout = { .tv_sec = CAM_CSI_DQ_TIMEOUT_SEC, .tv_usec = 0 };
    (void)ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout);

    struct v4l2_requestbuffers rb = {
        .count = CAM_CSI_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &rb) != 0 || rb.count < 2) {
        saved_errno = errno ? errno : ENOMEM; fail_stage = "raw10_REQBUFS"; goto raw_fail;
    }
    if (rb.count > CAM_CSI_BUFFER_COUNT) rb.count = CAM_CSI_BUFFER_COUNT;
    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_QUERYBUF"; goto raw_fail;
        }
        maps[i].length = b.length;
        maps[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (!maps[i].start || maps[i].start == MAP_FAILED) {
            maps[i].start = NULL; saved_errno = errno ? errno : ENOMEM;
            fail_stage = "raw10_mmap"; goto raw_fail;
        }
        mapped_count++;
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_QBUF_initial"; goto raw_fail;
        }
        ESP_LOGI(TAG, "RAW10 buffer[%u] len=%u", (unsigned)i, (unsigned)b.length);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        saved_errno = errno; fail_stage = "raw10_STREAMON"; goto raw_fail;
    }
    streaming = true;

    /* Two warm-up frames are excluded so the timing represents steady state. */
    for (int w = 0; w < 2; ++w) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_DQBUF_warmup"; goto raw_fail;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_QBUF_warmup"; goto raw_fail;
        }
    }

    for (uint32_t n = 0; n < CAM_RAW10_FRAME_COUNT; ++n) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        int64_t dq0 = esp_timer_get_time();
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_DQBUF"; goto raw_fail;
        }
        int64_t now = esp_timer_get_time();
        dq_us_last = now > dq0 ? (uint32_t)(now - dq0) : 0;
        dq_us_sum += dq_us_last;
        if (dq_us_last > dq_us_max) dq_us_max = dq_us_last;
        if (b.index >= mapped_count || !maps[b.index].start) {
            saved_errno = EFAULT; fail_stage = "raw10_bad_index"; goto raw_fail;
        }
        if (frames == 0) t_first = now;
        t_last = now;
        uint32_t used = b.bytesused;
        if (used < bytes_min) bytes_min = used;
        if (used > bytes_max) bytes_max = used;
        frames++;
        if ((frames <= 3) || ((frames % 10U) == 0U)) {
            ESP_LOGI(TAG, "RAW10 frame=%u seq=%u bytes=%u dq_us=%u",
                     (unsigned)frames, (unsigned)b.sequence, (unsigned)used,
                     (unsigned)dq_us_last);
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            saved_errno = errno; fail_stage = "raw10_QBUF_requeue"; goto raw_fail;
        }
    }

    if (streaming) { int t = V4L2_BUF_TYPE_VIDEO_CAPTURE; (void)ioctl(fd, VIDIOC_STREAMOFF, &t); streaming = false; }
    for (uint32_t i = 0; i < mapped_count; ++i) if (maps[i].start) munmap(maps[i].start, maps[i].length);
    close(fd); fd = -1;

    {
        int64_t elapsed = (frames > 1 && t_last > t_first) ? (t_last - t_first) : 0;
        uint32_t fps_x100 = elapsed ? (uint32_t)(((uint64_t)(frames - 1) * 100000000ULL) / (uint64_t)elapsed) : 0;
        uint32_t dq_avg = frames ? (uint32_t)(dq_us_sum / frames) : 0;
        size_t dfa = dma_free(), dla = dma_largest(), psa = psram_free();
        ESP_LOGI(TAG, "RAW10 PASS frames=%u fps=%u.%02u dq_us avg/max=%u/%u dma=%u/%u",
                 (unsigned)frames, (unsigned)(fps_x100/100), (unsigned)(fps_x100%100),
                 (unsigned)dq_avg, (unsigned)dq_us_max, (unsigned)dfa, (unsigned)dla);
        char body[760];
        snprintf(body, sizeof(body),
                 "{\"ok\":true,\"test\":\"RAW10_direct\",\"sensor\":\"OV02C10\","
                 "\"width\":%u,\"height\":%u,\"nominal_fps\":%u,\"frames\":%u,"
                 "\"fps_x100\":%u,\"dq_us\":{\"last\":%u,\"avg\":%u,\"max\":%u},"
                 "\"bytes_min\":%u,\"bytes_max\":%u,"
                 "\"dma\":{\"free_before\":%u,\"largest_before\":%u,\"free_after\":%u,\"largest_after\":%u},"
                 "\"psram\":{\"free_before\":%u,\"free_after\":%u},\"reboot_before_retest\":true}",
                 (unsigned)sensor_fmt.width, (unsigned)sensor_fmt.height, (unsigned)sensor_fmt.fps,
                 (unsigned)frames, (unsigned)fps_x100,
                 (unsigned)dq_us_last, (unsigned)dq_avg, (unsigned)dq_us_max,
                 (unsigned)(bytes_min == UINT32_MAX ? 0 : bytes_min), (unsigned)bytes_max,
                 (unsigned)dma0, (unsigned)dl0, (unsigned)dfa, (unsigned)dla,
                 (unsigned)ps0, (unsigned)psa);
        s_state.running = false;
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_sendstr(req, body);
    }

raw_fail:
    if (streaming && fd >= 0) { int t = V4L2_BUF_TYPE_VIDEO_CAPTURE; (void)ioctl(fd, VIDIOC_STREAMOFF, &t); }
    for (uint32_t i = 0; i < mapped_count; ++i) if (maps[i].start) munmap(maps[i].start, maps[i].length);
    if (fd >= 0) close(fd);
    s_state.running = false;
    ESP_LOGE(TAG, "RAW10 FAIL stage=%s errno=%d (%s)", fail_stage ? fail_stage : "unknown",
             saved_errno, saved_errno ? strerror(saved_errno) : "n/a");
    return send_json_error(req, "500 Internal Server Error",
                           fail_stage ? fail_stage : "raw10_unknown", saved_errno, ESP_FAIL);
}

esp_err_t camera_csi_diag_register(httpd_handle_t server)
{
    if (server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    static const httpd_uri_t csi_uri = {
        .uri = "/api/cam/csi-test",
        .method = HTTP_GET,
        .handler = csi_test_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t h264_uri = {
        .uri = "/api/cam/h264-test",
        .method = HTTP_GET,
        .handler = h264_test_handler,
        .user_ctx = NULL,
    };

    esp_err_t ret = httpd_register_uri_handler(server, &csi_uri);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = httpd_register_uri_handler(server, &h264_uri);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "registered GET /api/cam/csi-test + /api/cam/h264-test (mutual ONE-SHOT; reboot before retest)");
    return ESP_OK;
}
