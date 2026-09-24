# Quectel modem services

This library targets the project's Quectel EC200U-CN. GNSS and UFS commands follow
Quectel's EC200U application notes; other vendors and Quectel families have not
been validated. GNSS is optional on EC200U variants and requires suitable
hardware, firmware, and an antenna.

## Responsibilities and ownership

| Class | Responsibility |
| --- | --- |
| `AtClient` | Synchronous AT transport, response framing, explicit byte transfers |
| `QuectelModem` | Board startup, SIM/network control, existing HTTP/TLS operations; owns GNSS and file services |
| `QuectelGnss` | GNSS power commands, engine state, validated location fixes |
| `QuectelFileSystem` | Modem UFS metadata, binary transfers, handle cleanup |
| `GsmMqttClient` | Existing MQTT protocol operations |
| `src/main.cpp` | Owns the UART and service instances; coordinates startup, telemetry and configuration subscriptions |

Services borrow a single `AtClient`; it borrows an Arduino `Stream`. Dependencies
must outlive their borrowers. Use one owner/task for the entire modem channel:
these APIs are blocking and are **not thread-safe**, and unsolicited messages are
not queued. Do not interleave file transactions with MQTT/HTTP or direct UART
commands. GNSS polling can consume unrelated URCs while awaiting its response;
applications needing reliable asynchronous delivery need a URC dispatcher first.

`AtClient::serial()` returns `Stream&`. Board UART setup is explicit through
`QuectelModem::beginSerial(HardwareSerial&)`, using the same UART passed to the AT
client. `main.cpp` constructs `modemSerial`, `modemAt`, `modem`, and `gsmMqtt` in
dependency order, using board settings from `global_configs.h`. It reads modem
state and errors from the objects and caches display metadata in `gsm::RuntimeInfo`.
There is no default-modem singleton or handler include in the active application.

The previous application is preserved exactly in
[`src/main_legacy_reference.cpp.bak`](../../main_legacy_reference.cpp.bak), together
with the unchanged [`src/utils/GSM_handler.h`](../GSM_handler.h). The `.bak` suffix
keeps the old `setup()`/`loop()` out of PlatformIO's source discovery. See the
[migration notes](../../../docs/gsm-main-migration.md) for the call mapping and
backup restoration instructions.

## GNSS

Within `main.cpp`, after the normal modem startup, the owned modem exposes GNSS:

```cpp
auto &gnss = modem.gnss();
if (gnss.supported() && gnss.start()) {
    // Start the application acquisition deadline. A fix is not immediate.
}

// Poll later from the same task that owns all modem operations.
gsm::GnssFix fix;
switch (gnss.readFix(fix)) {
case gsm::GnssResult::Fix:
    // Use fix.latitude / fix.longitude (signed decimal degrees).
    // UTC, date, altitude, speed, course, HDOP and satellite count are also available.
    break;
case gsm::GnssResult::NoFix:
    // Retry on a later application cycle until the acquisition deadline.
    break;
default:
    // Handle disabled GNSS, unsupported firmware, invalid data or command failure.
    break;
}

// At the end of acquisition, including its failure/timeout path:
if (!gnss.stop()) {
    // Confirm/recover engine state before assuming the low-power budget is met.
}
```

`readFix` makes one bounded query and clears its output on failure. It does not
wait for satellite acquisition or change network/time configuration. `enabled`
queries the modem rather than trusting a cached power state. Numeric CME errors
516, 505 and 502 map to `NoFix`, `NotRunning` and `Unsupported`; other errors and
timeouts return `Error`. Use numeric modem error reporting (`AT+CMEE=1`) for these
distinctions; verbose errors may fall back to `Error`. `supported()` returning
false can also indicate a transport failure, so it is not permanent capability
detection. Starting an already running engine may return a modem error.

GNSS is never started by `initialize()`. `sleep()` does not stop it automatically;
the application controls the acquisition and shutdown policy.

## Modem file system

This is the modem's UFS, separate from ESP32 LittleFS and SD storage. The initial
API supports **root files** with filenames up to 63 printable ASCII bytes. Both
`settings.bin` and `UFS:settings.bin` are accepted. Directories, other storage
volumes, wildcard deletion and AT command delimiters are rejected.

```cpp
auto &files = modem.fileSystem();

gsm::ModemStorageInfo storage;
if (files.space(storage)) {
    // storage.freeBytes and storage.totalBytes
}

const uint8_t content[] = {0x00, 0x0D, 0x0A, 0xFF};
bool saved = files.write("sample.bin", content, sizeof(content));

uint8_t bytes[32];
size_t bytesRead = 0;
bool read = files.read("sample.bin", bytes, sizeof(bytes), bytesRead);
// No trailing NUL is added. A short successful read indicates EOF.
// For larger files, repeat read with an explicit byte offset.

gsm::ModemFileInfo entries[8];
size_t count = 0;
bool complete = files.list(entries, 8, count);
// false means command/parse failure or insufficient capacity, not a complete list.

gsm::ModemFileInfo entry;
bool found = files.info("sample.bin", entry);
bool removed = files.remove("sample.bin");
```

`writeText` is the text convenience method. `main.cpp` uses the file service
directly for certificate storage. `QuectelModem::fileExists` and
`QuectelModem::writeFile` also delegate to it. The old `GsmFileCheck` and
`GsmWriteFile` functions remain only in the reference handler.
Writes replace/truncate a file, including an empty replacement. They
are not atomic: errors can leave a partial file. Each 512-byte block requires the
modem's byte-count acknowledgement, and success also requires a confirmed close.

Reads use the exact `CONNECT <length>` framing, so embedded NULs, CR/LF, and `OK`
are preserved. `bytesRead` may be nonzero on failure; use the boolean result
before treating the data as a successful read. Text replies are capped at 4096
bytes; exceeding this limit returns failure rather than silently truncating.
Listings can therefore fail even when the caller supplies enough entry slots.

Handles are closed after completed transactions, including explicit modem errors.
An incomplete binary transfer, unknown handle, or failed close invalidates the AT
session. Further `sendCommand` calls fail without sending bytes. Reset the modem
to reclaim handles and restore framing; `hardwareReset` resets the transport
session after its warmup delay. With no reset pin, use an external reset/power
cycle, then call `AtClient::resetSession()` after command mode is known to be
restored. Do not clear the flag merely to retry: the modem could still be in data
mode. The raw `serial()` accessor bypasses these guards and is for controlled
legacy integration only.

## Validation and references

Run `bash scripts/test_gsm.sh` for scripted UART regression tests with the host
C++ compiler and UndefinedBehaviorSanitizer. Optional AddressSanitizer:
`GSM_SANITIZERS=address,undefined bash scripts/test_gsm.sh` on a host where that
runtime is supported. Tests do not require PlatformIO downloads or a modem.
The host runner requires `tests/gsm/test_gsm.cpp` and `tests/gsm/support/Arduino.h`;
these files are absent from the current checkout, so host tests must be restored
before using the runner.
Run `pio run -e esp32_s3_quectel_v4` for the firmware build.

Hardware validation remains necessary for GNSS acquisition/shutdown, antenna
performance, firmware-specific error responses, and large UFS transfers. No
device has been flashed as part of this change.

- [Quectel EC200U GNSS application note](https://quectel.com/content/uploads/2024/04/Quectel_EC200U_SeriesEG912U-GL_GNSS_Application_Note_V1.2.pdf)
- [Quectel EC200U FILE application note](https://quectel.com/content/uploads/2024/02/Quectel_EC200UEG91xU_Series_FILE_Application_Note_V1.4.pdf)
- [Refactor review and remaining work](../../../docs/gsm-refactor-review.md)
