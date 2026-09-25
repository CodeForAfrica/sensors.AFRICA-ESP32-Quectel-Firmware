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

## Startup and reset

Call `beginSerial(modemSerial)` before `initialize()`. The former opens the UART
and probes AT. If the modem does not respond, it runs the board-tested power
sequence: PWRKEY GPIO LOW, 1-second settle, LOW for 500 ms, then HIGH for
2500 ms and held HIGH, followed by `serialWarmupMs` (4 seconds by default). A failed initial probe does not prevent calling `initialize()`
for recovery, provided the UART supplied to `beginSerial` matches `AtClient`.

`initialize()` attempts `AT+CFUN=1,1` first, allowing its documented 15-second
response time. After acknowledgement it waits `resetWarmupMs` (30 seconds by
default) and polls AT for up to 60 seconds. Failure triggers one hardware reset
and another readiness check. An invalid binary session skips CFUN and proceeds
directly to hardware recovery: AT text could otherwise become file contents.
A timed-out CFUN command gets the reboot warmup interval before hardware
recovery because its acknowledgement may have been lost during reboot.

Both `softReset()` and `hardwareReset()` return `bool`; success requires AT
communication after reboot. Reset clears cached SIM, GPRS and MQTT connection
state while preserving configured SIM PIN and accumulated failure counts.
`initialize()` reapplies command settings and validates the SIM after reboot;
SIM/configuration errors return failure without another hardware reset.
No reset pin means failed software recovery returns an error for the caller.

The board's tested MCU levels take precedence over the active-low signals at the
bare modem pins. GPIO 16 (PWRKEY) uses normal `OUTPUT` mode and the held-HIGH
startup sequence above. GPIO 42 (reset control) uses normal `OUTPUT` mode and
`LowHighLow`: LOW idle, HIGH for `resetPulseMs`, then LOW again. A responsive
modem skips the PWRKEY sequence entirely. The earlier direct-pin/open-drain
assumption was incorrect for this board and has been removed. The supplied tested
firmware matches the inverting NPN drivers in the local
`Resources/Quectel_EC200U_Series_Hardware_Design_V1.0.pdf`, Figures 10 and 14
(printed pages 37 and 40; PDF pages 38 and 41). MCU HIGH turns the transistor on
and pulls the modem pin LOW; MCU LOW turns it off and releases the modem pin.
That document is Quectel's reference circuit, rather than a PCB-specific schematic.

The legacy final PWRKEY GPIO HIGH keeps the modem's PWRKEY LOW through that driver.
This preserves the tested always-on behavior, rather than a released power-on
pulse. The manual notes that held-low PWRKEY prevents AT-command power-off.
An orderly shutdown feature would need to release the control first; do not assume
the present held-HIGH startup sequence supports `AT+QPOWD` shutdown unchanged.

`resetPulseMs` defaults to 120 ms and is clamped to a minimum of 100 ms.
`resetWarmupMs` defaults to 30000 ms and starts after reset is released. Both are
32-bit millisecond settings. The reset API deliberately takes only the sequence:

```cpp
modem.hardwareReset(gsm::ResetSequence::LowHighLow);
```

Do not pass `resetWarmupMs` as a second argument. The former second argument was
an 8-bit pulse duration, so 30000 converted to 48 ms (then clamped to 100 ms).
Change `SerialConfig::resetPulseMs` explicitly when a different pulse is needed.

CFUN is a firmware-requested reboot with full functionality restored. PWRKEY
requests power-on or orderly shutdown, depending on state and pulse length.
RESET_N forces a baseband reset; none of these commands directly switches the
external VBAT_RF or VBAT_BB supply. Internal rail sequencing is modem-controlled.
Quectel recommends RESET_N only when orderly shutdown through AT+QPOWD or PWRKEY
is unavailable. This implementation uses RESET_N after failed software recovery;
it does not implement a verified PWRKEY shutdown/power-on cycle or monitor STATUS.
A responsive modem does not inherently need a reboot at every host startup;
the explicit reboot here follows this application's requested startup policy.

- [EC200U hardware design, sections 3.6–3.7](https://www.quectel.com/content/uploads/2024/02/Quectel_EC200U_Series_Hardware_Design_V1.2.pdf)
- [EC200U AT commands, AT+CFUN](https://quectel.com/content/uploads/2024/02/Quectel_EC200UEG91xUEG915G_Series-AT-Commands-Manual_V1.1.pdf)

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
session after its warmup delay and checks AT readiness. With no reset pin, use an external reset/power
cycle, then call `AtClient::resetSession()` after command mode is known to be
restored. Do not clear the flag merely to retry: the modem could still be in data
mode. The raw `serial()` accessor bypasses these guards and is for controlled
legacy integration only.

## Validation and references

Run `bash scripts/test_gsm.sh` for scripted UART regression tests with the host
C++ compiler and UndefinedBehaviorSanitizer. Optional AddressSanitizer:
`GSM_SANITIZERS=address,undefined bash scripts/test_gsm.sh` on a host where that
runtime is supported. Tests do not require PlatformIO downloads or a modem.
The current host suite covers 14 startup/reset scenarios using scripted UART
responses and a virtual clock/GPIO recorder. It does not cover the older GNSS
and file-transfer regression scenarios that were absent from this checkout.
Run `pio run -e esp32_s3_quectel_v4` for the firmware build.

Hardware validation remains necessary for GNSS acquisition/shutdown, antenna
performance, firmware-specific error responses, and large UFS transfers. No
device has been flashed as part of this change.

- [Quectel EC200U GNSS application note](https://quectel.com/content/uploads/2024/04/Quectel_EC200U_SeriesEG912U-GL_GNSS_Application_Note_V1.2.pdf)
- [Quectel EC200U FILE application note](https://quectel.com/content/uploads/2024/02/Quectel_EC200UEG91xU_Series_FILE_Application_Note_V1.4.pdf)
- [Refactor review and remaining work](../../../docs/gsm-refactor-review.md)
