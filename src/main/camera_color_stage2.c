/*
 * [CAM-COLOR2-PER-PROFILE-20260910]
 *
 * Hue + WB are stored independently for 1080p and 720p60.
 *
 * Design:
 *  - one POST URI: /api/cam/color2
 *  - request carries "profile"
 *  - preview is RAM-only
 *  - save=true persists ONLY the requested profile
 *  - reset resets ONLY the requested profile
 *  - backend stream start explicitly applies the selected profile
 *  - no camera lifecycle / CSI / H264 / WebRTC / TURN changes
 *
 * Migration for this exact project state:
 *  - 720p60 keeps the existing legacy keys hue/wbm/wbt
 *    because the latest runtime log shows they were saved while 720 was selected.
 *  - 1080p uses new keys h1080/m1080/t1080.
 *  - if 1080 keys do not exist yet, 1080 starts at hue=0 / WB=AUTO / 6500K.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "cJSON.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "nvs.h"

#include "camera_color_stage2.h"

/* Production log quiet switch. Set to 1 temporarily when deep diagnostics are needed. */
#ifndef HA_VERBOSE_DIAG_LOGS
#define HA_VERBOSE_DIAG_LOGS 0
#endif

#define COLOR2_URI              "/api/cam/color2"
#define COLOR2_NVS_NS           "cam_color2"

#define COLOR2_720_HUE          "hue"
#define COLOR2_720_WB_MANUAL    "wbm"
#define COLOR2_720_WB_TEMP      "wbt"
#define COLOR2_720_R_TRIM       "rtrim"
#define COLOR2_720_B_TRIM       "btrim"

#define COLOR2_1080_HUE         "h1080"
#define COLOR2_1080_WB_MANUAL   "m1080"
#define COLOR2_1080_WB_TEMP     "t1080"
#define COLOR2_1080_R_TRIM      "r1080"
#define COLOR2_1080_B_TRIM      "b1080"

#define COLOR2_HUE_MIN          (-128)
#define COLOR2_HUE_MAX          127
#define COLOR2_TEMP_MIN_K       3000
#define COLOR2_TEMP_MAX_K       9000
#define COLOR2_TEMP_NEUTRAL_K   6500
#define COLOR2_TRIM_MIN_PCT     70
#define COLOR2_TRIM_MAX_PCT     130
#define COLOR2_TRIM_DEFAULT_PCT 100
#define COLOR2_BODY_MAX         512

typedef enum {
    COLOR2_PROFILE_1080 = 0,
    COLOR2_PROFILE_720  = 1,
    COLOR2_PROFILE_COUNT
} color2_profile_t;

typedef struct {
    int hue;
    bool wb_manual;
    int wb_temp_k;
    int red_trim_pct;
    int blue_trim_pct;
    bool dirty;
} color2_state_t;

static const char *TAG = "CAM_COLOR2";
static SemaphoreHandle_t s_lock;
static bool s_loaded;
static color2_profile_t s_active_profile = COLOR2_PROFILE_1080;

static color2_state_t s_state[COLOR2_PROFILE_COUNT] = {
    [COLOR2_PROFILE_1080] = {
        .hue = 0, .wb_manual = false, .wb_temp_k = COLOR2_TEMP_NEUTRAL_K,
        .red_trim_pct = COLOR2_TRIM_DEFAULT_PCT, .blue_trim_pct = COLOR2_TRIM_DEFAULT_PCT,
        .dirty = false
    },
    [COLOR2_PROFILE_720] = {
        .hue = 0, .wb_manual = false, .wb_temp_k = COLOR2_TEMP_NEUTRAL_K,
        .red_trim_pct = COLOR2_TRIM_DEFAULT_PCT, .blue_trim_pct = COLOR2_TRIM_DEFAULT_PCT,
        .dirty = false
    },
};

extern esp_err_t esp_video_isp_pipeline_color2_set(bool enabled,
                                                    int hue,
                                                    bool wb_manual,
                                                    int wb_temp_k,
                                                    int red_trim_pct,
                                                    int blue_trim_pct);

static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static const char *profile_name(color2_profile_t p)
{
    return p == COLOR2_PROFILE_720 ? "720p60" : "1080p";
}

static color2_profile_t profile_from_text(const char *txt)
{
    if (txt &&
        (!strcasecmp(txt, "720") ||
         !strcasecmp(txt, "720p") ||
         !strcasecmp(txt, "720p-native") ||
         !strcasecmp(txt, "720p60-native") ||
         !strcasecmp(txt, "720p-highspeed") ||
         !strcasecmp(txt, "1288x728"))) {
        return COLOR2_PROFILE_720;
    }
    return COLOR2_PROFILE_1080;
}

static color2_profile_t profile_from_json(const cJSON *root)
{
    const cJSON *jp = cJSON_GetObjectItemCaseSensitive(root, "profile");
    return profile_from_text(cJSON_IsString(jp) ? jp->valuestring : NULL);
}

static bool enabled_for(const color2_state_t *st)
{
    return st->hue != 0 || st->wb_manual;
}

static void defaults_for(color2_state_t *st)
{
    st->hue = 0;
    st->wb_manual = false;
    st->wb_temp_k = COLOR2_TEMP_NEUTRAL_K;
    st->red_trim_pct = COLOR2_TRIM_DEFAULT_PCT;
    st->blue_trim_pct = COLOR2_TRIM_DEFAULT_PCT;
    st->dirty = false;
}

static void keys_for(color2_profile_t p,
                     const char **hue_key,
                     const char **manual_key,
                     const char **temp_key,
                     const char **red_trim_key,
                     const char **blue_trim_key)
{
    if (p == COLOR2_PROFILE_720) {
        *hue_key = COLOR2_720_HUE;
        *manual_key = COLOR2_720_WB_MANUAL;
        *temp_key = COLOR2_720_WB_TEMP;
        *red_trim_key = COLOR2_720_R_TRIM;
        *blue_trim_key = COLOR2_720_B_TRIM;
    } else {
        *hue_key = COLOR2_1080_HUE;
        *manual_key = COLOR2_1080_WB_MANUAL;
        *temp_key = COLOR2_1080_WB_TEMP;
        *red_trim_key = COLOR2_1080_R_TRIM;
        *blue_trim_key = COLOR2_1080_B_TRIM;
    }
}

static esp_err_t apply_profile_locked(color2_profile_t p)
{
    color2_state_t *st = &s_state[p];
    s_active_profile = p;

    esp_err_t err = esp_video_isp_pipeline_color2_set(enabled_for(st),
                                                       st->hue,
                                                       st->wb_manual,
                                                       st->wb_temp_k,
                                                       st->red_trim_pct,
                                                       st->blue_trim_pct);
    if (err == ESP_OK) {
        if (HA_VERBOSE_DIAG_LOGS) ESP_LOGI(TAG,
                 "[CAM-RB-TRIM-R1] apply profile=%s hue=%d wb=%s temp=%dK R=%d%% G=100%% B=%d%% active=%u",
                 profile_name(p), st->hue,
                 st->wb_manual ? "MANUAL" : "AUTO",
                 st->wb_temp_k, st->red_trim_pct, st->blue_trim_pct,
                 enabled_for(st) ? 1U : 0U);
    }
    return err;
}

static esp_err_t save_profile_locked(color2_profile_t p)
{
    const char *kh, *km, *kt, *kr, *kb;
    keys_for(p, &kh, &km, &kt, &kr, &kb);

    color2_state_t *st = &s_state[p];

    nvs_handle_t h;
    esp_err_t err = nvs_open(COLOR2_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_i16(h, kh, (int16_t)st->hue);
    if (err == ESP_OK) err = nvs_set_u8(h, km, st->wb_manual ? 1U : 0U);
    if (err == ESP_OK) err = nvs_set_i16(h, kt, (int16_t)st->wb_temp_k);
    if (err == ESP_OK) err = nvs_set_i16(h, kr, (int16_t)st->red_trim_pct);
    if (err == ESP_OK) err = nvs_set_i16(h, kb, (int16_t)st->blue_trim_pct);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        st->dirty = false;
        ESP_LOGI(TAG,
                 "[CAM-RB-TRIM-R1] saved profile=%s hue=%d wb=%s temp=%dK R=%d%% G=100%% B=%d%%",
                 profile_name(p), st->hue,
                 st->wb_manual ? "MANUAL" : "AUTO",
                 st->wb_temp_k, st->red_trim_pct, st->blue_trim_pct);
    }
    return err;
}

static void load_profile_from_handle(nvs_handle_t h, color2_profile_t p)
{
    const char *kh, *km, *kt, *kr, *kb;
    keys_for(p, &kh, &km, &kt, &kr, &kb);

    color2_state_t *st = &s_state[p];
    defaults_for(st);

    int16_t i16v;
    uint8_t u8v;

    if (nvs_get_i16(h, kh, &i16v) == ESP_OK) {
        st->hue = clamp_i((int)i16v, COLOR2_HUE_MIN, COLOR2_HUE_MAX);
    }
    if (nvs_get_u8(h, km, &u8v) == ESP_OK) {
        st->wb_manual = (u8v != 0);
    }
    if (nvs_get_i16(h, kt, &i16v) == ESP_OK) {
        st->wb_temp_k = clamp_i((int)i16v, COLOR2_TEMP_MIN_K, COLOR2_TEMP_MAX_K);
    }
    if (nvs_get_i16(h, kr, &i16v) == ESP_OK) {
        st->red_trim_pct = clamp_i((int)i16v, COLOR2_TRIM_MIN_PCT, COLOR2_TRIM_MAX_PCT);
    }
    if (nvs_get_i16(h, kb, &i16v) == ESP_OK) {
        st->blue_trim_pct = clamp_i((int)i16v, COLOR2_TRIM_MIN_PCT, COLOR2_TRIM_MAX_PCT);
    }
    st->dirty = false;
}

static esp_err_t load_all_locked(void)
{
    if (s_loaded) return ESP_OK;

    defaults_for(&s_state[COLOR2_PROFILE_1080]);
    defaults_for(&s_state[COLOR2_PROFILE_720]);

    nvs_handle_t h;
    esp_err_t err = nvs_open(COLOR2_NVS_NS, NVS_READONLY, &h);
    if (err == ESP_OK) {
        load_profile_from_handle(h, COLOR2_PROFILE_1080);
        load_profile_from_handle(h, COLOR2_PROFILE_720);
        nvs_close(h);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS load skipped: %s", esp_err_to_name(err));
    }

    s_loaded = true;

    ESP_LOGI(TAG,
             "[CAM-COLOR2-NVS] loaded 1080 hue=%d wb=%s temp=%dK | 720 hue=%d wb=%s temp=%dK",
             s_state[COLOR2_PROFILE_1080].hue,
             s_state[COLOR2_PROFILE_1080].wb_manual ? "MANUAL" : "AUTO",
             s_state[COLOR2_PROFILE_1080].wb_temp_k,
             s_state[COLOR2_PROFILE_720].hue,
             s_state[COLOR2_PROFILE_720].wb_manual ? "MANUAL" : "AUTO",
             s_state[COLOR2_PROFILE_720].wb_temp_k);

    return ESP_OK;
}

static esp_err_t send_state_locked(httpd_req_t *req,
                                   color2_profile_t p,
                                   const char *status)
{
    const color2_state_t *st = &s_state[p];

    cJSON *root = cJSON_CreateObject();
    cJSON *obj = cJSON_CreateObject();
    if (!root || !obj) {
        cJSON_Delete(root);
        cJSON_Delete(obj);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no_mem");
    }

    cJSON_AddBoolToObject(root, "ok", true);
    if (status) cJSON_AddStringToObject(root, "status", status);
    cJSON_AddStringToObject(root, "profile", profile_name(p));
    cJSON_AddItemToObject(root, "color2", obj);
    cJSON_AddBoolToObject(obj, "enabled", enabled_for(st));
    cJSON_AddNumberToObject(obj, "hue", st->hue);
    cJSON_AddBoolToObject(obj, "wb_manual", st->wb_manual);
    cJSON_AddNumberToObject(obj, "wb_temp_k", st->wb_temp_k);
    cJSON_AddNumberToObject(obj, "red_trim_pct", st->red_trim_pct);
    cJSON_AddNumberToObject(obj, "green_ref_pct", 100);
    cJSON_AddNumberToObject(obj, "blue_trim_pct", st->blue_trim_pct);
    cJSON_AddBoolToObject(obj, "dirty", st->dirty);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no_mem");
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_sendstr(req, json);
    free(json);
    return err;
}

static cJSON *read_json(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= COLOR2_BODY_MAX) return NULL;

    char body[COLOR2_BODY_MAX];
    int want = req->content_len;
    int got = 0;

    while (got < want) {
        int r = httpd_req_recv(req, body + got, want - got);
        if (r <= 0) return NULL;
        got += r;
    }
    body[got] = '\0';
    return cJSON_Parse(body);
}

static esp_err_t color2_post(httpd_req_t *req)
{
    if (!s_lock) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "not_ready");
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "busy");
    }

    esp_err_t err = load_all_locked();
    if (err != ESP_OK) {
        xSemaphoreGive(s_lock);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "isp_state");
    }

    cJSON *in = read_json(req);
    if (!in) {
        xSemaphoreGive(s_lock);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad_json");
    }

    color2_profile_t p = profile_from_json(in);

    const cJSON *jop = cJSON_GetObjectItemCaseSensitive(in, "op");
    const char *op = cJSON_IsString(jop) ? jop->valuestring : "get";
    const char *status = "loaded";

    if (strcmp(op, "get") == 0) {
        /* State only. Stream-start backend selects what is applied. */
    } else if (strcmp(op, "reset") == 0) {
        defaults_for(&s_state[p]);
        err = apply_profile_locked(p);
        if (err == ESP_OK) err = save_profile_locked(p);
        status = "reset_saved";
    } else if (strcmp(op, "set") == 0) {
        color2_state_t *st = &s_state[p];

        const cJSON *jh = cJSON_GetObjectItemCaseSensitive(in, "hue");
        const cJSON *jm = cJSON_GetObjectItemCaseSensitive(in, "wb_manual");
        const cJSON *jt = cJSON_GetObjectItemCaseSensitive(in, "wb_temp_k");
        const cJSON *jr = cJSON_GetObjectItemCaseSensitive(in, "red_trim_pct");
        const cJSON *jb = cJSON_GetObjectItemCaseSensitive(in, "blue_trim_pct");
        const cJSON *js = cJSON_GetObjectItemCaseSensitive(in, "save");

        if (cJSON_IsNumber(jh)) {
            st->hue = clamp_i(jh->valueint, COLOR2_HUE_MIN, COLOR2_HUE_MAX);
        }
        if (cJSON_IsBool(jm)) {
            st->wb_manual = cJSON_IsTrue(jm);
        }
        if (cJSON_IsNumber(jt)) {
            st->wb_temp_k = clamp_i(jt->valueint, COLOR2_TEMP_MIN_K, COLOR2_TEMP_MAX_K);
        }
        if (cJSON_IsNumber(jr)) {
            st->red_trim_pct = clamp_i(jr->valueint, COLOR2_TRIM_MIN_PCT, COLOR2_TRIM_MAX_PCT);
        }
        if (cJSON_IsNumber(jb)) {
            st->blue_trim_pct = clamp_i(jb->valueint, COLOR2_TRIM_MIN_PCT, COLOR2_TRIM_MAX_PCT);
        }

        st->dirty = true;
        err = apply_profile_locked(p);
        if (err == ESP_OK && cJSON_IsTrue(js)) {
            err = save_profile_locked(p);
            status = "saved";
        } else {
            status = "preview";
        }
    } else {
        cJSON_Delete(in);
        xSemaphoreGive(s_lock);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad_op");
    }

    cJSON_Delete(in);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "operation %s profile=%s failed: %s",
                 op, profile_name(p), esp_err_to_name(err));
        xSemaphoreGive(s_lock);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "apply_failed");
    }

    err = send_state_locked(req, p, status);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t camera_color_stage2_apply_profile(bool is_720)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = load_all_locked();
    if (err == ESP_OK) {
        err = apply_profile_locked(is_720 ? COLOR2_PROFILE_720 : COLOR2_PROFILE_1080);
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t camera_color_stage2_register(httpd_handle_t server)
{
    if (!server) return ESP_ERR_INVALID_ARG;

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = load_all_locked();
    if (err == ESP_OK) {
        /* Preserve startup behavior. Actual selected profile is re-applied at stream start. */
        err = apply_profile_locked(COLOR2_PROFILE_1080);
    }

    xSemaphoreGive(s_lock);
    if (err != ESP_OK) return err;

    static const httpd_uri_t uri = {
        .uri = COLOR2_URI,
        .method = HTTP_POST,
        .handler = color2_post,
        .user_ctx = NULL,
    };

    err = httpd_register_uri_handler(server, &uri);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "registered POST %s (per-profile op=get/set/reset)", COLOR2_URI);
    } else {
        ESP_LOGE(TAG, "failed to register %s: %s", COLOR2_URI, esp_err_to_name(err));
    }
    return err;
}
