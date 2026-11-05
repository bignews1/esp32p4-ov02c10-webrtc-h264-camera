# Provenance

The preparation recipe uses the public Espressif repository:
https://github.com/espressif/esp-webrtc-solution

Pinned commit: `dc3ab928137916d7b214a9a8ffe52e0e81794471`.
Application: `solutions/doorbell_local` plus its upstream components.
The fetched files retain their existing license notices. They are not copied
from private home-panel history.

The external OV02C10 driver in `components/ov02c10` originates from Espressif's
camera driver work (esp-video-components PR 46), with subsequent project
modifications to timing, format profiles, gain/AE diagnostics and registration.
It retains the original Espressif copyright and Apache-2.0 identifiers.
Register tables and ISP tuning are generic sensor settings, not personal data.
The recipe/build helpers and this documentation are original repository material
covered by the root MIT license. Apache-2.0 text is included at
`../../third_party_licenses/Apache-2.0.txt`.
