/*
 * Camera color Stage2 — Hue + WB AUTO/MANUAL + visual color-temperature trim.
 * 2026-09-01
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t camera_color_stage2_register(httpd_handle_t server);

/* Apply saved Hue/WB for the selected camera profile.
 * is_720=false: 1080p, true: 720p60. */
esp_err_t camera_color_stage2_apply_profile(bool is_720);

#ifdef __cplusplus
}
#endif