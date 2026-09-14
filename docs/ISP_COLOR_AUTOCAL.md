# ISP and Color Auto-Calibration

The camera UI includes manual image tuning and automatic color calibration for
both 1080p and 720p60 profiles.

## What Is Saved

The UI stores profile-specific tuning values in NVS:

- saturation
- contrast
- gamma
- sharpness
- hue
- white-balance mode
- white-balance temperature

1080p and 720p60 use separate saved values so tuning one profile does not
silently overwrite the other.

## Profile Targets

| Profile | Expected Frame Size | Purpose |
|---|---:|---|
| 1080p | 1920x1080 | Main high-quality camera view |
| 720p60 | 1288x728 | Higher FPS/lower bandwidth view |

The auto-calibration code checks that the active camera profile matches the
requested calibration target. If 720p60 calibration is requested, it expects the
720 frame size; if 1080p calibration is requested, it expects the 1080 frame
size.

## Why GOP Does Not Matter for Calibration

GOP and QP affect H.264 compression after the camera image is prepared. Color
auto-calibration should be based on the camera/ISP image before H.264 packet
structure is relevant.

So a live status such as:

```text
1080p - GOP15
```

does not mean the calibration is using a GOP15-encoded image as the color
reference. GOP15 is a streaming/network choice; the color calibration target is
the profile's raw/ISP image path.

## Practical Notes

- Use 1080p auto-calibration for the 1080p profile.
- Use 720p color auto-calibration for the 720p60 profile.
- Keep the profile-specific NVS split; it prevents 720 tuning from degrading
  1080 quality and vice versa.
