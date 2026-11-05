# OV02C10 WebRTC example

A separate ESP-IDF application recipe for the ESP32-P4 Function EV Board with
an OV02C10 MIPI camera. It prepares Espressif's local WebRTC application at a
pinned commit and adds this repository's OV02C10 driver. It does not depend on
the private home-panel project or its configuration.

The initial baseline sends 1920x1080 at a requested 30 fps over H.264 WebRTC,
with the upstream board audio path and browser UI. Actual frame rate depends
on hardware/network conditions. It does **not** import the large experimental
OWNER UI, custom production QP/GOP handling, or all DMA tuning from `src/`.
Those remain integration references after the basic stream works.

## Requirements

- ESP32-P4 Function EV Board, OV02C10 camera, working ESP32-C6 hosted firmware
  for the board's SDIO Wi-Fi interface, microphone/speaker as needed.
- ESP-IDF **5.5.4** terminal, Git, Python, CMake and Ninja.
- Internet for the pinned upstream checkout and component downloads.
- Python board helper: `python -m pip install esp-bmgr-assist==0.8.4`.
  Use the Python in the active ESP-IDF terminal.

## Prepare and build

Use a separate clone of this public repository. These commands work in an
ESP-IDF PowerShell terminal or a shell with ESP-IDF exported:

```text
git clone https://github.com/bignews1/esp32p4-ov02c10-webrtc-h264-camera.git p4-camera-demo
cd p4-camera-demo/examples/ov02c10_webrtc
python -m pip install esp-bmgr-assist==0.8.4
python prepare.py
python build.py configure
python build.py menuconfig
```

In **Public OV02C10 example**, enter your own Wi-Fi SSID and password. They are
stored in the generated, ignored application's `sdkconfig`, not in published
source files. Save, then:

```text
python build.py build
python build.py verify
```

The upstream application is created at:

```text
.work/esp-webrtc-solution/solutions/doorbell_local
```

The upstream revision is pinned in `prepare.py`; another revision is rejected.
Preparation is idempotent and preserves existing local settings and keys.
Builds are incremental. No `clean`, `fullclean` or flashing runs automatically.

Capture and audio versions are pinned for P4 1.x compatibility. GMF core and
image effects are also constrained to compatible API generations. See
`VALIDATION.md` and `TESTED_COMPONENT_VERSIONS.json` for the successful build.

## Flash and connect

Enter the generated application directory, then flash your selected test board.
For example on Windows, replace `COM5` with its actual port:

```text
cd .work/esp-webrtc-solution/solutions/doorbell_local
idf.py -p COM5 flash monitor
```

The serial log prints an address such as
`https://<board-ip>/webrtc/test`. Open that exact address on the same LAN.
The generated HTTPS certificate is self-signed: the browser will require a
local certificate exception and may need its microphone permission enabled.
Press the upstream page's connect/start control; confirm video first, then audio.
This baseline uses local signaling. Remote TURN deployment is outside this
example and no private relay credentials are included.

## Board and camera details

- Selects the two-lane OV02C10 1920x1080/30 format; disables the default SC2336
  and optional USB camera and pedestrian model.
- Targets ESP32-P4 revision 1.x with compatible capture/audio releases pinned
  in the preparation recipe. Revision 3.x requires a separate profile and
  matching codec binaries; changing the minimum chip revision alone is not a
  sufficient port. ESP-IDF treats these as incompatible hardware variants.
- Uses board manager's ESP32-P4 Function EV Board wiring, including shared I2C
  and MIPI LDO ownership. Board-specific defaults are generated first and the
  OV02C10 configuration overlay is applied last.
- SDIO wiring comes from that board definition (CLK18, CMD19, D0–D3 14–17,
  C6 reset54). A mismatched C6 firmware/protocol still needs board-side setup.
- Uses the external driver registration and local ISP JSON from `components/`.
- No display, relay, stepper or home automation application code is imported.
  Upstream board definitions can list other peripherals; this does not import
  the private panel UI.

## Privacy and licensing

Generated code/dependencies, Wi-Fi settings, binaries, certificates and private
keys remain under ignored `.work/`. HTTPS keys are freshly generated locally;
the upstream example's shared sample key is replaced before build. Do not
commit generated application folders or firmware containing credentials.

The Espressif application retains its upstream public-domain/CC0 example
notices. The OV02C10 files retain Apache-2.0 notices and local modifications;
see `NOTICE.md` and `../../third_party_licenses/Apache-2.0.txt`.

See `VALIDATION.md` for exactly what has been checked. A successful compilation
does not establish physical camera operation or measured streaming performance.
