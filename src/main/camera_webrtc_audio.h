#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_WEBRTC_AUDIO_SAMPLE_RATE 24000U
#define CAM_WEBRTC_AUDIO_CHANNELS    1U
#define CAM_WEBRTC_AUDIO_BITS        16U
#define CAM_WEBRTC_AUDIO_FRAME_MS    20U
#define CAM_WEBRTC_AUDIO_BITRATE     48000U

typedef int (*camera_webrtc_audio_frame_cb_t)(const uint8_t *data,
                                               size_t len,
                                               uint32_t pts_ms,
                                               void *ctx);

esp_err_t camera_webrtc_audio_init(void);
esp_err_t camera_webrtc_audio_start(camera_webrtc_audio_frame_cb_t cb, void *ctx);
void camera_webrtc_audio_request_stop(void);
esp_err_t camera_webrtc_audio_stop(uint32_t timeout_ms);
bool camera_webrtc_audio_is_active(void);

/* [TALKBACK-MIC-MUTE-PHASE2-R1-20260917]
 * Atomic/non-blocking half-duplex gate. Capture/encoder stay alive; when true,
 * outgoing panel-mic PCM is zeroed before Opus encode to prevent acoustic echo. */
void camera_webrtc_audio_set_talkback_mute(bool enable);
bool camera_webrtc_audio_get_talkback_mute(void);

/* [CAM-AUDIO-REMOVE-PCM-WAV-DIAG-V1-20260902]
 * PCM WAV diagnostic API removed after root-cause isolation. */
#ifdef __cplusplus
}
#endif