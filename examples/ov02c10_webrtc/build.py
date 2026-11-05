#!/usr/bin/env python3
"""Configure/build the separate public example with the active ESP-IDF Python."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from prepare import prepare, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["configure", "menuconfig", "build", "verify"])
    args = parser.parse_args()
    app = prepare()
    idf_path = os.environ.get("IDF_PATH")
    if not idf_path:
        parser.error("Open an ESP-IDF 5.5.4 terminal (export.sh / export.ps1) first.")
    idf = Path(idf_path) / "tools/idf.py"
    if not idf.is_file():
        parser.error("IDF_PATH does not contain tools/idf.py")
    if args.action == "verify":
        config = app / "sdkconfig"
        if not config.exists():
            parser.error("Run configure/build first")
        text = config.read_text()
        for item in ("CONFIG_CAMERA_OV02C10=y", "CONFIG_CAMERA_OV02C10_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y",
                     "CONFIG_CAMERA_OV02C10_MIPI_RAW10_1920x1080_2LAN_30FPS=y"):
            if item not in text.splitlines():
                raise RuntimeError("Missing required camera setting: " + item)
        if "CONFIG_CAMERA_SC2336=y" in text.splitlines():
            raise RuntimeError("SC2336 is still enabled; camera defaults were overridden")
        binary = app / "build/doorbell_demo.bin"
        if not binary.exists():
            raise RuntimeError("Firmware not found; run build")
        if binary.stat().st_mtime_ns < config.stat().st_mtime_ns:
            raise RuntimeError("Firmware is older than sdkconfig; rebuild before verifying")
        compiled = json.loads((app / "build/config/sdkconfig.json").read_text())
        revision = next((line.split("=", 1)[1] for line in text.splitlines()
                         if line.startswith("CONFIG_ESP32P4_REV_MIN_FULL=")), None)
        if revision is None or str(compiled.get("ESP32P4_REV_MIN_FULL")) != revision:
            raise RuntimeError("Compiled chip revision does not match sdkconfig")
        if revision != "100" or not compiled.get("ESP32P4_SELECTS_REV_LESS_V3"):
            raise RuntimeError("This recipe targets revision 1.x with matching legacy codec binaries")
        print(f"PASS: OV02C10 configuration, fresh firmware and compiled revision {revision}")
        return
    env = os.environ.copy()
    env["IDF_TARGET"] = "esp32p4"
    defaults = "sdkconfig.defaults;sdkconfig.defaults.esp32p4;sdkconfig.defaults.ov02c10"
    def idf_run(*commands):
        subprocess.run([sys.executable, str(idf), "-D", "SDKCONFIG_DEFAULTS=" + defaults,
                        *commands], cwd=app, env=env, check=True)
    if args.action == "configure":
        # Generates board glue; an isolated application, never the user's panel.
        idf_run("gen-bmgr-config", "-b", "esp32_p4_function_ev_board")
        board_defaults = app / "components/gen_bmgr_codes/board_manager.defaults"
        if board_defaults.exists():
            lines = board_defaults.read_text().splitlines()
            board_defaults.write_text("\n".join(
                line for line in lines if "CONFIG_CAMERA_SC2336" not in line) + "\n",
                encoding="utf-8", newline="\n")
        idf_run("reconfigure")
    else:
        idf_run(args.action)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        sys.exit(str(error))
