"""
PlatformIO extra_script: reads the ESP32's factory base MAC over serial
via esptool and converts it to match the string produced on-device by:
    
    char esp_chipid[18] = {};
    uint64_t chipid_num = ESP.getEfuseMac();
    snprintf(esp_chipid, sizeof(esp_chipid), "%llX", chipid_num);

! Why this is needed: 
ESP.getEfuseMac() does not compute an ID, it reads
the chip's burned-in base MAC from eFuse. `esptool.py read_mac` reports
the same bytes in normal MAC order (XX:XX:XX:XX:XX:XX), but the Arduino
core packs those bytes in REVERSE order into the uint64_t it returns.
So a naive hex-join of esptool's output will NOT match esp_chipid.

Usage in platformio.ini:

    [env:esp32dev]
    platform = espressif32
    board = esp32dev
    framework = arduino
    upload_port = /dev/ttyUSB0      ; must be known at build time
    extra_scripts = pre:scripts/extract_chipid.py

Result:
    - Adds -D DEVICE_CHIP_ID=\"<hex>\" to CPPDEFINES, usable in code as:
          const char* chip_id = DEVICE_CHIP_ID;  #? TBD whether to use this in the codebase
    - Writes the same value to chip_id.txt in the project root for reference or debugging.
"""

Import("env")

import os
import re
import subprocess
import sys


def find_esptool_script():
    """
    PlatformIO ships esptool as the 'tool-esptoolpy' package (a standalone
    script), NOT as a pip-installable module in the penv. So `python -m
    esptool` fails even though the tool is present. Resolve its real path
    through PlatformIO's own package manager instead of guessing.
    """
    try:
        platform = env.PioPlatform()
        pkg_dir = platform.get_package_dir("tool-esptoolpy")
    except Exception as e:
        print(f"[extract_chipid] Could not resolve tool-esptoolpy package dir: {e}")
        return None

    if not pkg_dir:
        print("[extract_chipid] tool-esptoolpy package not installed.")
        return None

    script_path = os.path.join(pkg_dir, "esptool.py")
    if not os.path.isfile(script_path):
        # Newer esptool packages sometimes ship as a package dir, not a flat script
        script_path = os.path.join(pkg_dir, "esptool", "__init__.py")

    if not os.path.isfile(script_path):
        print(f"[extract_chipid] esptool.py not found under {pkg_dir}")
        return None

    return script_path


def read_mac_via_esptool(port):
    """Call esptool as a subprocess (via its resolved script path) and parse the reported MAC."""
    esptool_path = find_esptool_script()
    if not esptool_path:
        return None

    try:
        result = subprocess.run(
            [sys.executable, esptool_path, "--port", port, "read_mac"],
            capture_output=True,
            text=True,
            timeout=15,
        )
    except Exception as e:
        print(f"[extract_chipid] Failed to invoke esptool: {e}")
        return None

    if result.returncode != 0:
        print(f"[extract_chipid] esptool exited with error:\n{result.stderr}")
        return None

    match = re.search(r"MAC:\s*([0-9A-Fa-f:]+)", result.stdout)
    if not match:
        print("[extract_chipid] Could not find MAC in esptool output")
        return None

    return match.group(1)


def mac_to_efuse_mac_hex(mac_str):
    """
    Convert 'AA:BB:CC:DD:EE:FF' (esptool order) into the hex string
    that ESP.getEfuseMac() + %llX would print on-device (reversed byte
    order, no leading zeros — matching C's default %llX formatting).
    """
    mac_bytes = [int(b, 16) for b in mac_str.split(":")]
    reversed_bytes = mac_bytes[::-1]
    hex_str = "".join(f"{b:02X}" for b in reversed_bytes)
    return hex_str.lstrip("0") or "0"


def main():
    port = env.get("UPLOAD_PORT") or env.subst("$UPLOAD_PORT")
    if not port or port.startswith("$"):
        print(
            "[extract_chipid] No UPLOAD_PORT resolved at pre-build time. "
            "Set upload_port explicitly in platformio.ini for this to work "
            "(PlatformIO's auto-detect happens too late for extra_scripts)."
        )
        return
    
    mac_str = read_mac_via_esptool(port)
    if not mac_str:
        print("[extract_chipid] Skipping chip id injection (read failed).")
        return

    chip_id_hex = mac_to_efuse_mac_hex(mac_str)
    print(f"[extract_chipid] esptool MAC: {mac_str} -> chip id: {chip_id_hex}")

    env.Append(CPPDEFINES=[("DEVICE_CHIP_ID", f'\\"{chip_id_hex}\\"')])

    out_path = env.subst("$PROJECT_DIR") + "/chip_id.txt"
    with open(out_path, "w") as f:
        f.write(chip_id_hex)


main()