#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register mutually-exclusive one-shot camera diagnostics:
 *   GET /api/cam/csi-test  : MIPI-CSI/ISP YUV420 capture
 *   GET /api/cam/h264-test : MIPI-CSI/ISP YUV420 -> /dev/video11 HW H.264
 * These diagnostics remain one-shot. The live WebRTC path below has its own
 * clean start/stop lifecycle and is the preferred product path.
 */
esp_err_t camera_csi_diag_register(httpd_handle_t server);

/* Live H.264 delivery callback.
 * data points to the encoder MMAP bitstream buffer and is valid only for the
 * duration of the callback. The consumer must send/copy it synchronously.
 * Return 0 on success; non-zero means that frame was dropped by the consumer.
 * pts_ms is a monotonic millisecond timestamp from live-stream start.
 */
typedef int (*camera_h264_frame_cb_t)(const uint8_t *data, size_t len,
                                      uint32_t pts_ms, void *ctx);

typedef enum {
    CAMERA_H264_PROFILE_1080P = 0,
    CAMERA_H264_PROFILE_720P_NATIVE = 1,
} camera_h264_profile_t;

/* Select the next live-stream profile. Selection is allowed only while the
 * camera stream is stopped. 720p-native means the successfully tested
 * 1288x728@60 mode (2 CSI lanes), not the discarded CPU crop path. */
esp_err_t camera_h264_stream_set_profile(camera_h264_profile_t profile);
camera_h264_profile_t camera_h264_stream_get_profile(void);

/* [CAM-1080-LIVE-GOP-R1-20260913]
 * 1080-only GOP selector used by the network adaptive controller.
 * While stopped it selects the GOP for the next encoder open. While running it
 * updates the already-open HW encoder in place and waits briefly for the camera
 * worker to accept the request. 720 remains fixed GOP5. */
esp_err_t camera_h264_stream_set_gop(uint32_t gop);
uint32_t  camera_h264_stream_get_gop(void);
const char *camera_h264_stream_profile_name(camera_h264_profile_t profile);

/* [CAM-AUTO-COLOR-CAL-R1-20260910]
 * On-demand, read-only post-ISP color measurement used by the production
 * one-click 720p color calibrator. A request is fulfilled by the next camera
 * frame of the requested profile. Values describe the same O_UYY_E_VYY sparse
 * ROI used by the verified diagnostic: centered 1288x728 for 1080p, full frame
 * for native 720p60. No frame pixels or ISP settings are modified here. */
typedef struct {
    bool                  valid;
    uint32_t              request_id;
    camera_h264_profile_t profile;
    uint32_t              elapsed_ms;
    float                 y_mean;
    uint8_t               y_p10;
    uint8_t               y_p50;
    uint8_t               y_p90;
    float                 u_mean;
    float                 v_mean;
    float                 u_abs;
    float                 v_abs;
} camera_color_measurement_t;

uint32_t camera_color_measure_request(camera_h264_profile_t profile);
void camera_color_measure_get(camera_color_measurement_t *out);

typedef struct {
    bool     running;
    uint32_t width;
    uint32_t height;
    uint32_t sensor_fps;
    uint32_t h264_gop;
    uint32_t h264_qp_min;
    uint32_t h264_qp_max;
    uint32_t h264_bitrate;
    uint32_t camera_fps_x100;
    uint32_t encode_fps_x100;
    uint32_t camera_frames;
    uint32_t encoded_frames;
    uint32_t delivered_frames;
    uint32_t consumer_drops;
    uint32_t encoder_skips;
    uint32_t h264_bytes_last;
    uint32_t h264_bytes_max;
    uint32_t encode_us_last;
    uint32_t encode_us_max;
    uint32_t camera_dq_us_last;
    uint32_t camera_dq_us_max;
    uint32_t camera_dq_us_avg;
    uint32_t consumer_us_last;
    uint32_t consumer_us_max;
    uint32_t loop_us_last;
    uint32_t loop_us_max;
    uint32_t dma_free;
    uint32_t dma_largest;
    uint32_t psram_free;
    int      last_errno;
    char     last_stage[48];
} camera_h264_stream_stats_t;

/* Start/stop the continuous OV02C10 -> CSI/ISP YU12 -> HW H.264 pipeline.
 * The worker is pinned to Core0; Core1 remains untouched for stepper timing.
 */
esp_err_t camera_h264_stream_start(camera_h264_frame_cb_t cb, void *ctx);
void      camera_h264_stream_request_stop(void);
esp_err_t camera_h264_stream_stop(uint32_t timeout_ms);
bool      camera_h264_stream_is_active(void);
void      camera_h264_stream_get_stats(camera_h264_stream_stats_t *out);

/* One-time boot DMA-anchor prewarm. Call after LCD/BSP I2C init and before
 * network services. It reserves a 144 KB INTERNAL|DMA anchor, exercises native
 * 720p60 CSI/ISP + H.264 so permanent lazy locks/interrupt descriptors are
 * forced outside the protected span, cleans up + esp_video_deinit(), then frees
 * the anchor to restore a contiguous block for later 1080p H.264. */
esp_err_t camera_h264_dma_prewarm(void);

/* Capture-only benchmark used to isolate the /dev/video0 CSI/ISP stage.
 * The caller selects a V4L2 pixel format (e.g. YUV420 or RGB565).
 * It uses the same one-shot/reboot policy as the RAW10 diagnostic.
 */
esp_err_t camera_csi_capture_bench_http(httpd_req_t *req, uint32_t pixfmt,
                                        const char *test_name);

/* One-shot direct ESP_H264 HW API benchmark. It captures YU12 from /dev/video0
 * and calls esp_h264_enc_process() directly, bypassing /dev/video11. Reboot
 * before/after because it shares the same destructive camera diagnostic state. */
esp_err_t camera_h264_direct_bench_http(httpd_req_t *req);

/* One-shot RAW10 capture timing test, exposed through the existing
 * /api/smoke/status handler as ?raw10=1 so no extra httpd URI slot is used.
 * Do not call while live WebRTC is active. Reboot before starting another
 * camera stream after this diagnostic.
 */
esp_err_t camera_csi_raw10_diag_http(httpd_req_t *req);

#ifdef __cplusplus
}
#endif
