# CRSFDude

ESP32-C3 firmware that acts as a CRSF external module in a JR bay. Reads RC channels from the radio over half-duplex inverted UART and sends telemetry back.

## Features

- Single-wire half-duplex CRSF on GPIO 20 (inverted signal, 420kbaud)
- Decodes all 16 RC channels (packed 11-bit)
- Sends telemetry back to EdgeTX (flight mode, battery, etc.)
- Handles EdgeTX Device Ping/Info handshake automatically
- Includes reusable `CRSFProtocol` library under `lib/`

## Hardware

- **Board:** ESP32-C3 DevKit
- **CRSF Pin:** GPIO 20 — connects to JR bay S.PORT (signal pin)
- **Signal:** Inverted UART, idle LOW, 420000 baud, 8N1

## Build & Flash

```bash
pio run -e esp32c3 -t upload
```

## Monitor

```bash
pio device monitor
```

Output:
```
CRSFDude starting...
CH1:  992  CH2: 1024  CH3:  998  CH4:  992  [rx:150 tx:30 /s]
```

## CRSFProtocol Library

Reusable library in `lib/CRSFProtocol/`. Handles all protocol internals:

```cpp
#include "CRSFProtocol.h"

CRSFProtocol crsf;

void setup() {
    crsf.begin(20, 420000);  // pin, baud
}

void loop() {
    if (crsf.update()) {
        uint16_t ch1 = crsf.getChannel(0);  // 0-2047
        crsf.sendFlightMode("ACRO");
    }
}
```

### API

| Method | Description |
|--------|-------------|
| `begin(pin, baud)` | Initialize half-duplex UART |
| `update()` | Parse incoming frames, returns `true` on new RC data |
| `getChannel(ch)` | Read channel value (0-15, 11-bit) |
| `sendFlightMode(str)` | Send flight mode telemetry |
| `sendDeviceInfo(name)` | Send device info (auto-called on Device Ping) |
| `sendFrame(buf, len)` | Send raw CRSF frame |

## Key Learnings

1. **Signal is inverted** on JR bay S.PORT — EdgeTX radios have a hardware inverter IC
2. **GPIO matrix** handles inversion and half-duplex pin switching on ESP32-C3 (no `uart_set_line_inverse`)
3. **`gpio_reset_pin()` required** on ESP32-C3 to fully release the TX output after sending
4. **EdgeTX handshake** is mandatory — radio sends Device Ping (0x28) after detecting telemetry; must respond with Device Info (0x29) or radio freezes permanently
