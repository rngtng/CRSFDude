# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

```bash
pio run -e esp32c3 -t upload    # build and flash
pio device monitor              # serial monitor (115200 baud)
```

No test harness — this is embedded firmware. Verify by flashing and checking serial output for `[rx:N tx:N /s]` counters.

## Architecture

ESP32-C3 firmware acting as a CRSF external module in a JR bay. Single-wire half-duplex on GPIO 20.

**`src/main.cpp`** — Application loop: polls for RC data, sends telemetry every 5th packet, prints channel values.

**`lib/CRSFDude/`** — Reusable library handling all protocol internals:
- Frame parsing with CRC8 validation (polynomial 0xD5)
- 16-channel decoding (packed 11-bit)
- Half-duplex TX/RX switching via ESP32-C3 GPIO matrix
- EdgeTX Device Ping/Info handshake (auto-response)
- Telemetry frame building (flight mode, device info, raw frames)

## Critical Constraints

**Signal is inverted.** JR bay S.PORT has a hardware inverter. Inversion handled via GPIO matrix (`gpio_matrix_in/out` with `true` flag), NOT `uart_set_line_inverse` (causes conflicts on ESP32-C3).

**Half-duplex pin release.** After TX, must call `gpio_reset_pin()` then re-enable RX. Without this, ESP32-C3 keeps driving the TX output and corrupts subsequent RX. This is C3-specific — original ESP32 releases on `gpio_set_direction(INPUT)`.

**EdgeTX handshake is mandatory.** When the radio receives any valid telemetry, it sends Device Ping (0x28). Must respond with Device Info (0x29) or the radio permanently freezes (requires power cycle). The library handles this automatically in `processFrame()`.

**Link Stats enables telemetry streaming.** EdgeTX silently drops ALL sensor data (except flight mode) unless Link Statistics (0x14) with non-zero RxQuality has been received. Must be sent regularly.

**One frame per response window.** Sending multiple telemetry frames back-to-back causes collisions with the radio's next TX cycle. Rotate sensor types across response windows.

**Baud rate is 420000.** Non-standard, required by CRSF protocol. Must match the radio's configured external module baud rate exactly.

## CRSF Frame Format

```
[SYNC 0xC8] [LENGTH] [TYPE] [PAYLOAD...] [CRC8]
```

Sync bytes: `0xC8` (broadcast) or `0xEE` (module address). Both accepted on RX.
