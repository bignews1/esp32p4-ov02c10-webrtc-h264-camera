# Validation — 2026-11-05

## Passed

- Prepared a new checkout from the pinned public Espressif commit.
- Ran board generation, CMake configuration and a full firmware build in a
  fresh directory using ESP-IDF **5.5.4**, Linux/Python 3.12 and the
  `riscv32-esp-elf` GCC 14.2.0 (`esp-14.2.0_20260121`) toolchain.
- Produced `doorbell_demo.bin`: **3,675,072 bytes**; the IDF partition-size
  check passed. No firmware or private HTTPS key is published here.
- `python build.py verify` passed with compiled minimum revision **100**,
  matching the P4 1.x profile, and the expected OV02C10 configuration.
- Confirmed the linked ELF retains `ov02c10_detect`, so sensor auto-detection
  is not discarded by the linker.
- Matched the published helpers and driver files against the fresh build's
  inputs. Driver hashes are in `DRIVER_SHA256.json`.
- Repeated preparation without changing generated settings or HTTPS keys.
  Refused an unrelated existing upstream checkout. Verified fresh local keys
  replace the upstream shared sample key.
- Verified LF source output and exclusion of generated private work files
  from Git. Tested component versions are recorded separately in
  `TESTED_COMPONENT_VERSIONS.json` (an audit record, not a portable lock file).

The hosted build environment needed a local fallback to `os.getppid()` in
Component Manager's process-ID discovery because process enumeration was
unavailable. This was a host-tool adjustment, not a firmware/source patch;
it is not included in the example and is normally unnecessary on a developer PC.
Upstream deprecated/unused Kconfig warnings remain; the required generated
camera and chip configuration was checked explicitly.

## Not performed

- Physical board flashing, camera capture, browser streaming or audio tests.
- Runtime DMA/memory, frame rate, image quality, reconnect or long-duration tests.
- Windows PowerShell 5.1 / Python 3.11 execution.
- Revision 3.x operation. This recipe pins compatible libraries for revision 1.x.
- OWNER recognition/face tracking validation; these features are not part of
  the small baseline example.

## Compatibility decisions

The current upstream audio codec 2.6 and effects 1.4 require P4 >=3.0.
This recipe preserves the hardware checks and selects codec **2.5.0**,
effects **1.3.0~1**, capture **1.0.3**, GMF core **1.0.0** and image effects
**1.1.x**. The GMF core pin also avoids mixing incompatible 1.0 and 1.1
audio-element lock layouts through broad transitive version constraints.

The earlier optimized snapshot under `src/` is preserved separately.
Compile success for this baseline does not validate that larger snapshot or
establish the performance numbers documented for earlier private field tests.
