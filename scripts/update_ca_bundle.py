from pathlib import Path
import urllib.request

URL = "https://curl.se/ca/cacert.pem"
OUT = Path("src/utils/mozilla_ca_bundle.h")

VAR_NAME = "MOZILLA_CA_BUNDLE"
FILENAME_VAR = "MOZILLA_PEM_FILENAME"
PEM_FILENAME = "mozillaca.pem"
TEMPLATE = Path("ca_bundle_template.h")


def write_template_fallback(reason: Exception):
    print(f"CA bundle download failed: {reason}")
    print(f"Using fallback template: {TEMPLATE}")

    if not TEMPLATE.exists():
        raise FileNotFoundError(f"Fallback template not found: {TEMPLATE}")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(TEMPLATE.read_text(encoding="utf-8"), encoding="utf-8")


def cpp_raw_string(text: str) -> str:
    # Use a custom delimiter so normal PEM text cannot close the raw string.
    return f'R"CA_BUNDLE({text})CA_BUNDLE"'


def update_ca_bundle():
    try:
        print(f"Downloading CA bundle from {URL}")

        with urllib.request.urlopen(URL, timeout=30) as response:
            pem = response.read().decode("utf-8")

        OUT.parent.mkdir(parents=True, exist_ok=True)

        header = f"""#pragma once

    #include <Arduino.h>

    const char {FILENAME_VAR}[] PROGMEM = "{PEM_FILENAME}";

    const char {VAR_NAME}[] PROGMEM = {cpp_raw_string(pem)};

    const unsigned int MOZILLA_CA_BUNDLE_LEN = sizeof({VAR_NAME}) - 1;
    """

        OUT.write_text(header, encoding="utf-8")
        print(f"Wrote {OUT}")

    except Exception as exc:
        write_template_fallback(exc)


update_ca_bundle()
