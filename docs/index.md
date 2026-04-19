---
layout: default
title: CRSFDude
---

# CRSFDude

Custom CRSF external module for ESP32-C3 — reads RC channels and sends telemetry over half-duplex inverted UART in a JR bay.

- [Blog Post](BLOGPOST) — the full story of building CRSFDude, from inverted signals to frozen radios
- [Retro](RETRO) — what went well, what didn't, lessons learned
- [Source Code on GitHub](https://github.com/rngtng/CRSFDude)

## Quick Start

```cpp
#include "CRSFProtocol.h"

CRSFProtocol crsf;

void setup() {
    crsf.begin(20, 420000);
}

void loop() {
    if (crsf.update()) {
        uint16_t throttle = crsf.getChannel(2);
        crsf.sendFlightMode("ACRO");
    }
}
```

## Key Learnings

1. **JR bay CRSF is inverted** — hardware inverter on the radio, wire idles LOW
2. **ESP32-C3 half-duplex = GPIO matrix switching** — `gpio_reset_pin()` required to release TX
3. **EdgeTX Device Ping handshake is mandatory** — respond to 0x28 with 0x29 or the radio freezes
