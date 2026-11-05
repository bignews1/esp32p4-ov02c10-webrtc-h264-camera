/*
 * camera_webrtc_audio.c
 * [CAM-AUDIO-OPUS24K-V4-20260902]
 *
 * ESP32-P4-Function-EV-Board on-board ES8311 microphone:
 *   24 kHz / 16-bit / mono -> Opus 20 ms / 48 kbps -> WebRTC fanout.
 *
 * The board has one duplex audio clock. Existing synthesized UI sounds are
 * therefore moved from 22050 Hz to 24000 Hz too. Their durations/frequencies
 * remain defined in ms/Hz and are rebuilt at boot.
 *
 * Core1 stays stepper-only. Audio capture/encode runs on Core0.
 */
#include "camera_webrtc_audio.h"

#include <stdint.h>
#include <math.h> /* [CAM-AUDIO-BOARD-NOISE-CLEAN-V2-20260902] biquad cleanup */
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "bsp_board_extra.h"
#include "esp_audio_enc.h"
#include "esp_audio_enc_reg.h"
#include "esp_audio_enc_default.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif

static const char *TAG = "CAM_AUDIO";

#define CAM_AUDIO_TASK_CORE       0
#define CAM_AUDIO_TASK_PRIO       5
#define CAM_AUDIO_TASK_STACK      (24 * 1024) /* PSRAM stack; protect internal DMA SRAM */
#define CAM_AUDIO_INPUT_CHANNELS  1U /* [CAM-AUDIO-MONO-CLEAN-V1-20260902] ES8311 ADC is mono */
#define CAM_AUDIO_LOG_MS          2000U

/* [CAM-AUDIO-REMOVE-PCM-WAV-DIAG-V1-20260902]
 * 240KB diagnostic PSRAM buffer removed; production audio path retained. */
static SemaphoreHandle_t s_ctl_lock;
static SemaphoreHandle_t s_done;
static TaskHandle_t s_task;
static volatile bool s_stop_requested;
static volatile bool s_active;
/* [TALKBACK-MIC-MUTE-PHASE2-R1-20260917] Atomic via __atomic builtins. */
static bool s_talkback_mute;
static camera_webrtc_audio_frame_cb_t s_cb;
static void *s_cb_ctx;

esp_err_t camera_webrtc_audio_init(void)
{
    if (s_ctl_lock) return ESP_OK;
    s_ctl_lock = xSemaphoreCreateMutex();
    return s_ctl_lock ? ESP_OK : ESP_ERR_NO_MEM;
}

/* [CAM-AUDIO-REMOVE-PCM-WAV-DIAG-V1-20260902]
 * Diagnostic allocator/API removed. */
static void mono_clean_copy(const int16_t *raw, int16_t *mono,
                            size_t samples,
                            uint32_t *raw_peak_out,
                            uint32_t *clean_peak_out,
                            uint32_t *mean_abs_out,
                            uint32_t *impulse_fix_out)
{
    uint64_t sum = 0;
    uint32_t raw_peak = 0;
    uint32_t clean_peak = 0;
    uint32_t fixes = 0;

    for (size_t i = 0; i < samples; ++i) {
        int32_t x = raw[i];
        int32_t ax = x < 0 ? -x : x;
        if ((uint32_t)ax > raw_peak) raw_peak = (uint32_t)ax;

        if (i > 0 && (i + 1U) < samples) {
            int32_t p = raw[i - 1U];
            int32_t q = raw[i + 1U];
            int32_t ap = p < 0 ? -p : p;
            int32_t aq = q < 0 ? -q : q;
            int32_t jump_p = x - p; if (jump_p < 0) jump_p = -jump_p;
            int32_t jump_q = x - q; if (jump_q < 0) jump_q = -jump_q;
            int32_t neigh = p - q;  if (neigh < 0) neigh = -neigh;

            /* Require an extreme isolated impulse; do not act as a limiter. */
            if (ax >= 26000 && ap < 12000 && aq < 12000 &&
                jump_p > 18000 && jump_q > 18000 && neigh < 8000) {
                x = (p + q) / 2;
                fixes++;
            }
        }

        mono[i] = (int16_t)x;
        int32_t ac = x < 0 ? -x : x;
        if ((uint32_t)ac > clean_peak) clean_peak = (uint32_t)ac;
        sum += (uint32_t)ac;
    }

    if (raw_peak_out) *raw_peak_out = raw_peak;
    if (clean_peak_out) *clean_peak_out = clean_peak;
    if (mean_abs_out) *mean_abs_out = samples ? (uint32_t)(sum / samples) : 0U;
    if (impulse_fix_out) *impulse_fix_out = fixes;
}
/* [CAM-AUDIO-WHITE-NOISE-EXPANDER-V4-20260903]
 * Production cleanup after RAW-WAV diagnosis.
 *
 * Keep:
 *   - 70 Hz Butterworth HPF
 *   - proven LCD backlight PWM = 48 kHz
 *
 * Remove:
 *   - fixed 285.5 / 476 / 876.1 Hz diagnostic notches
 *
 * Add:
 *   - gentle downward expander for quiet gaps
 *   - no hard mute; V6 max attenuation remains 18 dB
 *   - detector uses max RMS of the two 10 ms halves in each 20 ms frame
 *   - V6 hysteresis: OPEN -40 dBFS / CLOSE -54 dBFS
 *   - V6 hold 300 ms
 *   - speech opening ~5 ms, quiet attenuation ~800 ms
 *
 * mono_clean_copy() is intentionally untouched in V4.
 */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} cam_audio_biquad_t;

static cam_audio_biquad_t s_clean_hpf;
static bool s_clean_ready;

static bool s_exp_open;
static uint32_t s_exp_hold_frames;
static float s_exp_gain;

#define CAM_EXP_OPEN_DBFS          (-40.0f)
#define CAM_EXP_CLOSE_DBFS         (-54.0f) /* [CAM-AUDIO-WHITE-NOISE-EXPANDER-V6-TAIL-SMOOTH-20260903] preserve weak speech tails */
#define CAM_EXP_MAX_ATTEN_DB       (18.0f)  /* keep V5 hiss suppression */
#define CAM_EXP_HOLD_FRAMES        (15U)    /* 15 x 20 ms = 300 ms */
#define CAM_EXP_OPEN_MS            (5.0f)
#define CAM_EXP_CLOSE_MS           (800.0f) /* slow fade into quiet attenuation */
/* [CAM-AUDIO-V6-MEDIA-RECOVERY-V3-EXACT-20260903] exact-current-source combined patch */

static void cleanup_biquad_reset(cam_audio_biquad_t *q)
{
    q->z1 = 0.0f;
    q->z2 = 0.0f;
}

static void cleanup_biquad_hpf(cam_audio_biquad_t *q, float hz)
{
    const float fs = (float)CAM_WEBRTC_AUDIO_SAMPLE_RATE;
    const float Q = 0.7071067811865475f;
    const float w0 = 6.2831853071795864769f * hz / fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw / (2.0f * Q);
    const float a0 = 1.0f + alpha;

    q->b0 = ((1.0f + cw) * 0.5f) / a0;
    q->b1 = (-(1.0f + cw)) / a0;
    q->b2 = ((1.0f + cw) * 0.5f) / a0;
    q->a1 = (-2.0f * cw) / a0;
    q->a2 = (1.0f - alpha) / a0;
    cleanup_biquad_reset(q);
}

static inline float cleanup_biquad_run(cam_audio_biquad_t *q, float x)
{
    const float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

static inline float cam_audio_dbfs_from_rms(float rms)
{
    if (rms < 0.0001f) return -120.0f;
    return 20.0f * log10f(rms / 32768.0f);
}

static void camera_audio_cleanup_init(void)
{
    cleanup_biquad_hpf(&s_clean_hpf, 70.0f);

    /* Start OPEN so stream startup cannot eat the first word. */
    s_exp_open = true;
    s_exp_hold_frames = CAM_EXP_HOLD_FRAMES;
    s_exp_gain = 1.0f;
    s_clean_ready = true;

    ESP_LOGW(TAG,
             "[WHITE-NOISE-EXPANDER-V6] ON HPF=70Hz open=-40dBFS close=-54dBFS hold=300ms max=-18dB open=5ms close=800ms LCD_PWM=48kHz");
}

static void camera_audio_cleanup_process(int16_t *pcm, size_t samples)
{
    if (!s_clean_ready) camera_audio_cleanup_init();
    if (!pcm || samples == 0U) return;

    /* HPF + two-half RMS detector. */
    const size_t half = samples / 2U;
    double ss0 = 0.0;
    double ss1 = 0.0;

    for (size_t n = 0; n < samples; ++n) {
        float x = cleanup_biquad_run(&s_clean_hpf, (float)pcm[n]);

        if (x > 32767.0f) x = 32767.0f;
        else if (x < -32768.0f) x = -32768.0f;

        pcm[n] = (int16_t)lrintf(x);

        const double xx = (double)x * (double)x;
        if (n < half) ss0 += xx;
        else ss1 += xx;
    }

    const size_t n0 = half ? half : samples;
    const size_t n1 = (samples > half) ? (samples - half) : n0;
    const float rms0 = sqrtf((float)(ss0 / (double)n0));
    const float rms1 = sqrtf((float)(ss1 / (double)n1));
    const float rms = (rms0 > rms1) ? rms0 : rms1;
    const float level_db = cam_audio_dbfs_from_rms(rms);

    /* Hysteresis + hold. */
    if (level_db >= CAM_EXP_OPEN_DBFS) {
        s_exp_open = true;
        s_exp_hold_frames = CAM_EXP_HOLD_FRAMES;
    } else if (s_exp_open) {
        if (level_db > CAM_EXP_CLOSE_DBFS) {
            s_exp_hold_frames = CAM_EXP_HOLD_FRAMES;
        } else if (s_exp_hold_frames > 0U) {
            s_exp_hold_frames--;
        } else {
            s_exp_open = false;
        }
    }

    /* About 2:1 downward expansion below -40 dBFS, capped at -18 dB. */
    float target_gain = 1.0f;
    if (!s_exp_open) {
        float atten_db = CAM_EXP_OPEN_DBFS - level_db;
        if (atten_db < 0.0f) atten_db = 0.0f;
        if (atten_db > CAM_EXP_MAX_ATTEN_DB) atten_db = CAM_EXP_MAX_ATTEN_DB;
        target_gain = powf(10.0f, -atten_db / 20.0f);
    }

    const float fs = (float)CAM_WEBRTC_AUDIO_SAMPLE_RATE;
    const float open_tau = CAM_EXP_OPEN_MS / 1000.0f;
    const float close_tau = CAM_EXP_CLOSE_MS / 1000.0f;
    const float open_k = 1.0f - expf(-1.0f / (fs * open_tau));
    const float close_k = 1.0f - expf(-1.0f / (fs * close_tau));

    for (size_t n = 0; n < samples; ++n) {
        const float k = (target_gain > s_exp_gain) ? open_k : close_k;
        s_exp_gain += (target_gain - s_exp_gain) * k;

        float y = (float)pcm[n] * s_exp_gain;
        if (y > 32767.0f) y = 32767.0f;
        else if (y < -32768.0f) y = -32768.0f;
        pcm[n] = (int16_t)lrintf(y);
    }
}
static void camera_audio_task(void *arg)
{
    (void)arg;

    s_clean_ready = false; /* [CAM-AUDIO-BOARD-NOISE-CLEAN-V2-20260902] reset per stream */
    camera_audio_cleanup_init();

    esp_audio_enc_handle_t enc = NULL;
    uint8_t *raw_stereo = NULL;
    uint8_t *pcm_mono = NULL;
    uint8_t *opus = NULL;
    int pcm_bytes = 0;
    int opus_capacity = 0;

    uint32_t frames = 0;
    uint32_t read_err = 0;
    uint32_t enc_err = 0;
    uint32_t fanout_drop = 0;
    uint64_t opus_bytes = 0;
    bool last_talkback_mute = false;
    uint32_t last_raw_peak = 0;
    uint32_t last_peak = 0;
    uint32_t last_mean_abs = 0;
    uint32_t last_clip = 0;
    uint64_t total_clip = 0;

    int64_t started_ms = esp_timer_get_time() / 1000;
    int64_t last_log_ms = started_ms;

    /* esp_audio_codec 2.5.x documented common-registration path. */
    esp_audio_enc_register_default();
    esp_audio_err_t ar = ESP_AUDIO_ERR_OK;

    esp_opus_enc_config_t opus_cfg = ESP_OPUS_ENC_CONFIG_DEFAULT();
    opus_cfg.sample_rate = CAM_WEBRTC_AUDIO_SAMPLE_RATE;
    opus_cfg.bits_per_sample = CAM_WEBRTC_AUDIO_BITS;
    opus_cfg.channel = CAM_WEBRTC_AUDIO_CHANNELS;
    opus_cfg.bitrate = CAM_WEBRTC_AUDIO_BITRATE;
    opus_cfg.frame_duration = ESP_OPUS_ENC_FRAME_DURATION_20_MS;
    opus_cfg.application_mode = ESP_OPUS_ENC_APPLICATION_AUDIO;

    esp_audio_enc_config_t enc_cfg = {
        .type = ESP_AUDIO_TYPE_OPUS,
        .cfg = &opus_cfg,
        .cfg_sz = sizeof(opus_cfg),
    };

    ar = esp_audio_enc_open(&enc_cfg, &enc);
    if (ar != ESP_AUDIO_ERR_OK || !enc) {
        ESP_LOGE(TAG, "[OPUS24K] open failed ret=%d", (int)ar);
        goto done;
    }

    ar = esp_audio_enc_get_frame_size(enc, &pcm_bytes, &opus_capacity);
    if (ar != ESP_AUDIO_ERR_OK || pcm_bytes <= 0 || opus_capacity <= 0 ||
        (pcm_bytes % (int)sizeof(int16_t)) != 0) {
        ESP_LOGE(TAG, "[OPUS24K] bad frame sizes ret=%d pcm=%d out=%d",
                 (int)ar, pcm_bytes, opus_capacity);
        goto done;
    }

    const size_t mono_samples = (size_t)pcm_bytes / sizeof(int16_t);
    const size_t raw_bytes = mono_samples * CAM_AUDIO_INPUT_CHANNELS * sizeof(int16_t);

    raw_stereo = (uint8_t *)heap_caps_malloc(raw_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    pcm_mono   = (uint8_t *)heap_caps_malloc((size_t)pcm_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    opus       = (uint8_t *)heap_caps_malloc((size_t)opus_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!raw_stereo || !pcm_mono || !opus) {
        ESP_LOGE(TAG, "[OPUS24K] buffer OOM raw=%u pcm=%d out=%d psram=%u",
                 (unsigned)raw_bytes, pcm_bytes, opus_capacity,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        goto done;
    }

    s_active = true;
    ESP_LOGI(TAG,
             "[OPUS24K] start mic=%uHz raw_ch=%u -> mono frame=%ums bitrate=%u pcm=%d out_cap=%d core=%d",
             (unsigned)CAM_WEBRTC_AUDIO_SAMPLE_RATE,
             (unsigned)CAM_AUDIO_INPUT_CHANNELS,
             (unsigned)CAM_WEBRTC_AUDIO_FRAME_MS,
             (unsigned)CAM_WEBRTC_AUDIO_BITRATE,
             pcm_bytes, opus_capacity, xPortGetCoreID());

    while (!s_stop_requested) {
        size_t bytes_read = 0;
        esp_err_t rd = bsp_extra_i2s_read(raw_stereo, raw_bytes, &bytes_read, 100);
        if (rd != ESP_OK || bytes_read != raw_bytes) {
            read_err++;
            if ((read_err & 0x1fU) == 1U) {
                ESP_LOGW(TAG, "[OPUS24K] mic read ret=%s bytes=%u/%u count=%u",
                         esp_err_to_name(rd), (unsigned)bytes_read,
                         (unsigned)raw_bytes, (unsigned)read_err);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* [CAM-AUDIO-REMOVE-PCM-WAV-DIAG-V1-20260902]
         * Production path: codec RX -> mono cleanup -> board-noise DSP -> Opus. */
        mono_clean_copy((const int16_t *)raw_stereo,
                        (int16_t *)pcm_mono,
                        mono_samples,
                        &last_raw_peak,
                        &last_peak,
                        &last_mean_abs,
                        &last_clip);
        total_clip += last_clip; /* now cumulative isolated-impulse fixes */

        /* [CAM-AUDIO-BOARD-NOISE-CLEAN-V2-20260902]
         * RAW WAV diagnostic remains unfiltered; only outgoing WebRTC audio
         * gets the speech-safe board-noise cleanup. */
        camera_audio_cleanup_process((int16_t *)pcm_mono, mono_samples);

        /* [TALKBACK-MIC-MUTE-PHASE2-R1-20260917]
         * Keep ADC/I2S RX + Opus encoder resident during PTT. Only zero the
         * outgoing panel-mic PCM while remote talkback owns the speaker. */
        const bool talkback_mute = __atomic_load_n(&s_talkback_mute, __ATOMIC_ACQUIRE);
        if (talkback_mute) memset(pcm_mono, 0, (size_t)pcm_bytes);
        if (talkback_mute != last_talkback_mute) {
            ESP_LOGI(TAG, "[TALKBACK-MIC-MUTE-PHASE2-R1-20260917] %s; capture+encoder kept",
                     talkback_mute ? "ON" : "OFF");
            last_talkback_mute = talkback_mute;
        }

        esp_audio_enc_in_frame_t in_frame = {
            .buffer = pcm_mono,
            .len = pcm_bytes,
        };
        esp_audio_enc_out_frame_t out_frame = {
            .buffer = opus,
            .len = opus_capacity,
        };

        ar = esp_audio_enc_process(enc, &in_frame, &out_frame);
        if (ar != ESP_AUDIO_ERR_OK || out_frame.encoded_bytes <= 0) {
            enc_err++;
            if ((enc_err & 0x1fU) == 1U) {
                ESP_LOGW(TAG, "[OPUS24K] encode ret=%d encoded=%d count=%u",
                         (int)ar, out_frame.encoded_bytes, (unsigned)enc_err);
            }
            continue;
        }

        uint32_t pts_ms = frames * CAM_WEBRTC_AUDIO_FRAME_MS; /* [CAM-AUDIO-MONO-CLEAN-V1-20260902] exact media clock */
        camera_webrtc_audio_frame_cb_t cb = s_cb;
        int cb_ret = cb ? cb(opus, (size_t)out_frame.encoded_bytes, pts_ms, s_cb_ctx) : -1;
        if (cb_ret != 0) fanout_drop++;

        frames++;
        opus_bytes += (uint32_t)out_frame.encoded_bytes;

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_log_ms >= CAM_AUDIO_LOG_MS) {
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            uint32_t avg = frames ? (uint32_t)(opus_bytes / frames) : 0U;
            if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,
                     "[OPUS24K] frames=%u avg=%uB raw_peak=%u clean_peak=%u meanabs=%u impulse_fix=%u/%llu pts=%ums read_err=%u enc_err=%u fanout_drop=%u stack_hwm=%u",
                     (unsigned)frames, (unsigned)avg,
                     (unsigned)last_raw_peak, (unsigned)last_peak,
                     (unsigned)last_mean_abs, (unsigned)last_clip,
                     (unsigned long long)total_clip,
                     (unsigned)(frames * CAM_WEBRTC_AUDIO_FRAME_MS),
                     (unsigned)read_err, (unsigned)enc_err,
                     (unsigned)fanout_drop, (unsigned)hwm);
            last_log_ms = now_ms;
        }
    }

done:
    s_active = false;
    if (enc) esp_audio_enc_close(enc);
    if (raw_stereo) heap_caps_free(raw_stereo);
    if (pcm_mono) heap_caps_free(pcm_mono);
    if (opus) heap_caps_free(opus);

    ESP_LOGI(TAG, "[OPUS24K] stopped frames=%u read_err=%u enc_err=%u fanout_drop=%u",
             (unsigned)frames, (unsigned)read_err,
             (unsigned)enc_err, (unsigned)fanout_drop);

    if (s_ctl_lock && xSemaphoreTake(s_ctl_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_task = NULL;
        xSemaphoreGive(s_ctl_lock);
    } else {
        s_task = NULL;
    }
    if (s_done) xSemaphoreGive(s_done);
    vTaskDeleteWithCaps(NULL);
}

esp_err_t camera_webrtc_audio_start(camera_webrtc_audio_frame_cb_t cb, void *ctx)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    if (!s_ctl_lock) {
        esp_err_t e = camera_webrtc_audio_init();
        if (e != ESP_OK) return e;
    }

    if (xSemaphoreTake(s_ctl_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_task) {
        s_cb = cb;
        s_cb_ctx = ctx;
        xSemaphoreGive(s_ctl_lock);
        return ESP_OK;
    }

    if (s_done) {
        vSemaphoreDelete(s_done);
        s_done = NULL;
    }
    s_done = xSemaphoreCreateBinary();
    if (!s_done) {
        xSemaphoreGive(s_ctl_lock);
        return ESP_ERR_NO_MEM;
    }

    s_cb = cb;
    s_cb_ctx = ctx;
    s_stop_requested = false;
    s_active = false;

    /* [CAM-AUDIO-MONO-CLEAN-V1-20260902]
     * Espressif's ESP32-P4 Function-EV-Board microphone example opens the
     * ES8311 record path as channel=1. Do that here without changing speaker TX. */
    esp_err_t mono_ret = bsp_extra_codec_set_fs_record(
        CAM_WEBRTC_AUDIO_SAMPLE_RATE, CAM_WEBRTC_AUDIO_BITS, 1U);
    if (mono_ret != ESP_OK) {
        ESP_LOGE(TAG, "[MIC-MONO] record-path setup failed: %s",
                 esp_err_to_name(mono_ret));
        xSemaphoreGive(s_ctl_lock);
        return mono_ret;
    }

    /* [CAM-AUDIO-FORCE-GAIN-DSP18-V1-20260902]
     * Force the selected hardware gain after the mono RX reopen. */
    float hw_gain_db = bsp_extra_codec_input_gain_get();
    esp_err_t gain_ret = bsp_extra_codec_input_gain_set(hw_gain_db);
    if (gain_ret == ESP_OK) {
        ESP_LOGI(TAG, "[MIC-GAIN-FORCE] applied=%.0f dB, RX=mono DSP=0dB(x1)",
                 (double)hw_gain_db);
    } else {
        ESP_LOGW(TAG, "[MIC-GAIN-FORCE] apply %.0f dB failed: %s; DSP=0dB(x1)",
                 (double)hw_gain_db, esp_err_to_name(gain_ret));
    }

    if (xTaskCreatePinnedToCoreWithCaps(camera_audio_task, "cam_audio",
                                        CAM_AUDIO_TASK_STACK, NULL,
                                        CAM_AUDIO_TASK_PRIO, &s_task,
                                        CAM_AUDIO_TASK_CORE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        vSemaphoreDelete(s_done);
        s_done = NULL;
        s_task = NULL;
        xSemaphoreGive(s_ctl_lock);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_ctl_lock);
    return ESP_OK;
}

void camera_webrtc_audio_request_stop(void)
{
    s_stop_requested = true;
}

esp_err_t camera_webrtc_audio_stop(uint32_t timeout_ms)
{
    if (!s_ctl_lock) return ESP_OK;
    s_stop_requested = true;

    if (xSemaphoreTake(s_ctl_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    TaskHandle_t task = s_task;
    SemaphoreHandle_t done = s_done;
    xSemaphoreGive(s_ctl_lock);

    if (!task) return ESP_OK;
    if (!done || xSemaphoreTake(done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool camera_webrtc_audio_is_active(void)
{
    return s_active;
}

void camera_webrtc_audio_set_talkback_mute(bool enable)
{
    __atomic_store_n(&s_talkback_mute, enable, __ATOMIC_RELEASE);
}

bool camera_webrtc_audio_get_talkback_mute(void)
{
    return __atomic_load_n(&s_talkback_mute, __ATOMIC_ACQUIRE);
}
