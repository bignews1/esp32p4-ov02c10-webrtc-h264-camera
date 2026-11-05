# Contributing

Start with `examples/ov02c10_webrtc`. Keep its basic camera/WebRTC baseline
small; advanced experiments can stay in the reference sources under `src/`.

When reporting a problem or opening a pull request, include:

- ESP-IDF and component versions, board name and ESP32-P4 chip revision.
- Camera model, format, C6 hosted firmware and whether you use LAN or TURN.
- Reproduction steps and a short log with credentials, personal hostnames,
  IP addresses and face data removed.
- What you compiled and what you actually tested on hardware. Do not describe
  compile-only results as working video, recognition accuracy or performance.

Use LF for source files. Do not commit `.work/`, `sdkconfig`, Wi-Fi/TURN
credentials, private keys, face photographs, enrolled embeddings, NVS dumps,
firmware archives, font files or development backups. Keep upstream license
and copyright notices. New deployment options should use local configuration
rather than hardcoded private values.

Small, focused changes to build reproducibility, sensor bring-up, bounded
buffer handling, reconnect behavior and clear integration documentation are
especially useful. Provide measurements when claiming an FPS/latency gain.
