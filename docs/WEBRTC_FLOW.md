# WebRTC Flow

The browser creates the offer. The ESP32-P4 answers and streams H.264 video plus
Opus audio.

## Browser Side

- Uses STUN for direct path discovery.
- Uses TURN/TLS on TCP 443 as fallback.
- Displays browser decoded FPS using `framesDecoded` deltas.
- Displays RX bitrate using inbound RTP `bytesReceived` deltas.
- Keeps the LIVE line short and places QP/GOP/RX/CAP/ENC/DROP/DMA/ICE/AUD/VIEW
  in a fixed two-row status grid.
- Provides per-profile camera tuning controls and auto-calibration buttons.

## ESP Side

- Keeps up to two viewer sessions.
- Uses per-page random client tokens to reclaim orphaned sessions safely.
- Supports controlled profile switching between 1080p and 720p60.
- Keeps ICE/TURN behavior separate from camera QP/GOP decisions.
- Keeps ISP/color settings separate per profile and persists them in NVS.

## Network Profiles

1080 LTE mode favors stability with `QP32/GOP15`. 1080 Wi-Fi mode favors image
quality with `QP30/GOP5`. The fixed-QP delta sync fix is required for clean
QP30 operation without encoder reopen.

## Auto-Calibration Relationship

The live status may show `GOP5`, `GOP8`, or `GOP15`, but color auto-calibration
is based on the camera/raw/ISP sampling path before H.264 compression. The GOP
only controls encoded stream structure; it should not be used as the color
calibration reference.
