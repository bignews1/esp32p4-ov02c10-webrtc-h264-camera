#!/usr/bin/env python3
"""Prepare an isolated, pinned Espressif WebRTC example; never flash a board."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
UPSTREAM = "https://github.com/espressif/esp-webrtc-solution.git"
REVISION = "dc3ab928137916d7b214a9a8ffe52e0e81794471"


def run(*args, cwd=None):
    subprocess.run(args, cwd=cwd, check=True)


def replace_once(text, pattern, replacement):
    text, count = re.subn(pattern, lambda _: replacement, text)
    if count != 1:
        raise RuntimeError(f"Upstream layout changed: {pattern!r}: {count} matches")
    return text


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def certificate(app):
    # ESP-IDF already depends on cryptography. Keys stay in ignored .work/.
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID
    cert_dir = app / "main/certs"
    marker = cert_dir / ".locally-generated"
    if marker.exists():
        return
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "esp32p4-camera.test")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name)
            .public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=365))
            .add_extension(x509.SubjectAlternativeName([
                x509.DNSName("esp32p4-camera.test")]), critical=False)
            .sign(key, hashes.SHA256()))
    cert_dir.mkdir(parents=True, exist_ok=True)
    (cert_dir / "prvtkey.pem").write_bytes(key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
        serialization.NoEncryption()))
    (cert_dir / "servercert.pem").write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    write(marker, "Generated locally; never publish this private key.\n")


def prepare(upstream_source=None):
    checkout = ROOT / ".work/esp-webrtc-solution"
    if not checkout.exists():
        checkout.parent.mkdir(parents=True, exist_ok=True)
        if upstream_source:
            run("git", "clone", "--no-hardlinks", str(upstream_source), str(checkout))
        else:
            run("git", "clone", "--no-checkout", UPSTREAM, str(checkout))
        run("git", "checkout", "--detach", REVISION, cwd=checkout)
    actual = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=checkout, text=True).strip()
    if actual != REVISION:
        raise RuntimeError("Existing upstream checkout is at another revision; use a fresh example clone.")
    app = checkout / "solutions/doorbell_local"
    marker = app / ".public-example-prepared"
    if marker.exists():
        if marker.read_text().strip() != REVISION:
            raise RuntimeError("Preparation version mismatch; use a fresh example clone.")
        print(f"Already prepared: {app}")
        return app
    if (app / "sdkconfig").exists():
        raise RuntimeError("Refusing to modify an already configured upstream application.")
    # Newer audio binaries require P4 revision >=3.0. Pin compatible releases
    # rather than removing the upstream hardware compatibility checks.
    render_manifest = checkout / "components/av_render/idf_component.yml"
    text = render_manifest.read_text()
    text = replace_once(text, r'espressif/esp_audio_codec: "~2.6"',
                        'espressif/esp_audio_codec: "==2.5.0"')
    text = replace_once(text, r'espressif/esp_audio_effects: "~1.4"',
                        'espressif/esp_audio_effects: "==1.3.0~1"')
    text = replace_once(text, r'espressif/esp_image_effects: "~1.2"',
                        'espressif/esp_image_effects: "~1.1.0"')
    write(render_manifest, text)
    webrtc_manifest = checkout / "components/esp_webrtc/idf_component.yml"
    text = replace_once(webrtc_manifest.read_text(), r'espressif/esp_capture: "~1.1"',
                        'espressif/esp_capture: "==1.0.3"')
    write(webrtc_manifest, text)
    # gmf_audio 1.0 uses the core 1.0 audio-element layout. A broad ^1.0
    # dependency otherwise selects core 1.1 with a different lock layout.
    main_manifest = app / "main/idf_component.yml"
    text = replace_once(main_manifest.read_text(), r"dependencies:\n",
                        'dependencies:\n  espressif/gmf_core: "==1.0.0"\n')
    write(main_manifest, text)
    settings = app / "main/settings.h"
    text = settings.read_text()
    text = replace_once(text, r"#define VIDEO_FPS\s+25", "#define VIDEO_FPS    30")
    text = replace_once(text, r'#define WIFI_SSID\s+"XXXX"', "#define WIFI_SSID CONFIG_EXAMPLE_WIFI_SSID")
    text = replace_once(text, r'#define WIFI_PASSWORD\s+"XXXX"', "#define WIFI_PASSWORD CONFIG_EXAMPLE_WIFI_PASSWORD")
    write(settings, text)
    write(app / "main/Kconfig.projbuild", '''menu "Public OV02C10 example"
config EXAMPLE_WIFI_SSID
    string "Wi-Fi SSID (local sdkconfig only)"
    default ""
config EXAMPLE_WIFI_PASSWORD
    string "Wi-Fi password (local sdkconfig only)"
    default ""
endmenu
''')
    defaults = app / "sdkconfig.defaults.esp32p4"
    text = defaults.read_text()
    # Board manager defaults may enable SC2336. The build's final defaults
    # overlay, passed explicitly in build.py, selects OV02C10 after board setup.
    text = "\n".join(line for line in text.splitlines() if not any(
        key in line for key in ("CONFIG_CAMERA_SC2336", "CONFIG_ESP_VIDEO_ENABLE_USB_UVC_VIDEO_DEVICE",
                                "CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE", "CONFIG_ESP_CAPTURE_ENABLE_VIDEO_DECODER")))
    write(defaults, text + "\n")
    write(app / "sdkconfig.defaults.ov02c10", (ROOT / "sdkconfig.defaults.ov02c10").read_text())
    shutil.copytree(ROOT / "components/ov02c10", app / "components/ov02c10", dirs_exist_ok=True)
    certificate(app)
    write(marker, REVISION + "\n")
    source_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in sorted((ROOT / "components/ov02c10").rglob("*")) if p.is_file()}
    write(ROOT / ".work/preparation.json", json.dumps({
        "upstream": UPSTREAM, "revision": REVISION,
        "driver_sha256": source_hashes,
    }, indent=2) + "\n")
    print(f"Prepared: {app}")
    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream-source", type=Path, help="Optional local clone at the pinned revision")
    args = parser.parse_args()
    try:
        prepare(args.upstream_source)
    except (OSError, RuntimeError, subprocess.CalledProcessError, ImportError) as error:
        sys.exit(f"Preparation failed: {error}")
