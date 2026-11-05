#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_cam_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* profile: 0 = 1920x1080@30 2-lane,
 *          1 = native high-speed 1288x728@60 2-lane.
 * The 720-class profile reuses the exact register table from the successful
 * NATIVE60 hardware test performed on 2026-08-23. */
esp_err_t ov02c10_webrtc_select_profile(uint32_t profile);
const esp_cam_sensor_format_t *ov02c10_webrtc_get_profile_format(uint32_t profile);

#ifdef __cplusplus
}
#endif
