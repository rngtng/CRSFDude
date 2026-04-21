# TODO

## Library improvements

- [ ] Make UART number configurable (hardcoded to UART1 — fine for ESP32-C3 but limits portability)
- [ ] Add `onChannelsReceived` callback as alternative to polling `update()`
- [ ] Support ESP32 original (GPIO matrix behaves differently — no `gpio_reset_pin()` needed)
- [ ] Publish to PlatformIO registry
- [ ] Add `sendLinkStats()` with sensible defaults (e.g. `sendLinkStatsDefault()`)

## Telemetry

- [ ] Add remaining CRSF sensor types: RPM (0x0C), Temperature (0x0D), Cell Voltages (0x0E), Airspeed (0x0A)
- [ ] Add Link RX (0x1C) and Link TX (0x1D) stats
- [ ] Support RADIO_ID timing correction frame (0x3A) for proper mixer sync

## Protocol

- [ ] Respond to REQUEST_SETTINGS (0x2A) for LUA script compatibility
- [ ] Handle COMMAND frames (0x32)
- [ ] Support Model ID frame from radio
- [ ] Allow configurable device name for Device Info response

## Testing

- [x] Unit tests for CRC8, frame building, channel decoding (`pio test -e native`)
- [x] CI workflow for native tests (GitHub Actions)
- [x] Document expected EdgeTX sensor names for each telemetry type

## Ideas

- [ ] Auto-detect baud rate on startup (scan common rates, validate CRC)
- [ ] Batch multiple small telemetry frames in one TX window (needs timing analysis)
- [ ] WiFi/BLE bridge — expose RC channels and telemetry over network
- [ ] OLED display showing channel values and link stats
- [ ] Contribute half-duplex + handshake support back to CRSFforArduino
