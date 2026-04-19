# CRSFDude

CRSF packet counter and test firmware for ESP32-C3.

## Prerequisites

- [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation.html)

## Build & Flash

### CRSF Reader

Reads CRSF serial on GPIO 20 (RX) / 21 (TX) at 420kbaud and reports packets/sec on USB serial.

```bash
pio run -e esp32c3 -t upload
```

### Hello World (Serial Test)

Prints "Hello World" every second on USB serial. Useful to verify serial connection works.

```bash
pio run -e test_serial -t upload
```

## Monitor Serial Output

```bash
pio device monitor
```
