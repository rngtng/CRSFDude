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
        // Link stats must be sent to enable sensor discovery
        crsf.sendLinkStats(90, 90, 100, 10, 0, 4, 3, 80, 100, 8);
        crsf.sendBattery(111, 15, 1200, 75);  // 11.1V, 1.5A, 1200mAh, 75%
    }
}
```

## Telemetry Types

> **Important:** `sendLinkStats()` with non-zero RxQuality must be sent regularly — EdgeTX silently drops all other sensors without it.

| Method | EdgeTX Sensor |
|--------|---------------|
| `sendLinkStats(...)` | RQly, RSSI, SNR, RFMD, TPWR (enables streaming) |
| `sendFlightMode("ACRO")` | FM (top bar) |
| `sendBattery(V, A, mAh, %)` | VBAT, Curr, Capa, Bat% |
| `sendGPS(lat, lon, spd, hdg, alt, sats)` | GPS, GSpd, Hdg, GAlt, Sats |
| `sendAttitude(pitch, roll, yaw)` | Ptch, Roll, Yaw |
| `sendBaroAltitude(cm)` | Alt |
| `sendVario(cm/s)` | VSpd |
