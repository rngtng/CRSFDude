# Building a Custom CRSF Module with ESP32-C3: A Deep Dive into Half-Duplex Hell

I wanted to build a custom CRSF external module that sits in the JR bay of my EdgeTX radio. Read RC channels in, send telemetry back. Simple, right? It took two days and a lot of debugging to get a single telemetry value to show up on my radio. Here's what I learned.

## The Goal

Plug an ESP32-C3 into the JR module bay and have it:
- Read CRSF RC channel data from the radio
- Send telemetry (like flight mode) back to EdgeTX
- All on a single wire — the JR bay only has one signal pin

## Step 1: Reading CRSF Data

CRSF is a serial protocol at 420000 baud, 8N1. Frames start with a sync byte (0xC8 or 0xEE), followed by length, type, payload, and a CRC8 checksum. Simple enough.

I wired GPIO 20 to the JR bay signal pin, fired up `HardwareSerial`, and... garbage. No valid frames.

**Problem:** The signal is **inverted**. EdgeTX radios have a hardware inverter on the S.PORT line. The CRSF spec even says: *"Half duplex connections are typically INVERTED polarity (idle low)."*

**Fix:** On ESP32-C3, `uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV)` inverts the RX signal in hardware. Zero CPU cost. With that, frames started flowing — 150 packets/second of clean RC channel data.

## Step 2: The Half-Duplex Challenge

Now for TX. Same pin, same wire. The radio sends RC data, then the module has a brief window to respond with telemetry. On STM32 (what most radios use), the USART has native half-duplex mode. ESP32-C3 does not.

I tried everything:
- `uart_set_line_inverse` with both RX and TX inversion — killed RX permanently
- Open-drain GPIO mode — inverted idle state drives the line LOW, blocking the radio
- `CrsfSerial.end()` + `begin()` to reinit after TX — data lost during reinit

**What finally worked:** GPIO matrix switching, modeled after the [mLRS project](https://github.com/olliw42/mLRS). The ESP32's GPIO matrix lets you dynamically route peripheral signals to pins:

```cpp
// Switch to TX mode
gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, true);  // disconnect RX
gpio_set_direction(pin, GPIO_MODE_OUTPUT);
gpio_matrix_out(pin, U1TXD_OUT_IDX, true, false);            // connect TX (inverted)

// Switch back to RX
gpio_reset_pin(pin);  // fully release pin on ESP32-C3
gpio_set_direction(pin, GPIO_MODE_INPUT);
gpio_matrix_in(pin, U1RXD_IN_IDX, true);                     // reconnect RX (inverted)
```

Critical detail: on ESP32-C3, `gpio_set_direction(INPUT)` alone doesn't release the TX output. You need `gpio_reset_pin()` to fully detach the peripheral. This is different from the original ESP32.

The TX-to-RX switch runs in a FreeRTOS task that waits for `uart_wait_tx_done()` before switching — this keeps the main parsing loop non-blocking.

## Step 3: The Radio Froze

I could see my flight mode telemetry value "ON" appear on the radio. Success! Then the radio stopped sending RC data. Permanently. Required a full power cycle.

This was the most frustrating bug. The half-duplex was working. The frame format was valid. CRC checked out. But the radio froze after receiving just one telemetry frame.

**Root cause:** EdgeTX's CRSF handshake protocol. When the radio receives *any* valid telemetry frame from the external module bay, it enters a module initialization sequence:

1. Radio marks the module as "alive"
2. Radio sends a **Model ID** frame
3. Radio sends a **Device Ping** (0x28)
4. Radio waits for **Device Info** (0x29) response
5. Without that response, the radio halts its CRSF output indefinitely

I found this by reading the [EdgeTX source code](https://github.com/EdgeTX/edgetx) — specifically `crossfire.cpp` where `processCrossfireTelemetryFrame()` triggers `CRSF_FRAME_MODELID` and the subsequent ping/query flow.

**Fix:** Listen for Device Ping frames and respond with Device Info:

```cpp
if (frameType == CRSF_FRAMETYPE_DEVICE_PING) {
    sendDeviceInfo("CRSFDude");
}
```

After adding this, the radio stayed happy. RC channels kept flowing, telemetry showed up, no freezes.

## What I Ended Up With

A clean `CRSFProtocol` library that handles:
- CRC8 calculation
- Frame parsing with sync byte alignment
- All 16 RC channels (packed 11-bit decoding)
- Half-duplex GPIO matrix switching with inverted signal
- Telemetry TX (flight mode, device info, raw frames)
- EdgeTX handshake (auto-responds to Device Ping)

The main application code is ~40 lines:

```cpp
CRSFProtocol crsf;

void setup() {
    crsf.begin(20, 420000);
}

void loop() {
    if (crsf.update()) {
        uint16_t ch1 = crsf.getChannel(0);
        crsf.sendFlightMode("ACRO");
    }
}
```

## Key Takeaways

1. **JR bay CRSF is inverted** — the radio has a hardware inverter. The wire idles LOW, not HIGH.

2. **ESP32-C3 half-duplex needs GPIO matrix switching** — no native half-duplex UART support. Route TX/RX signals to the pin dynamically. Use `gpio_reset_pin()` to fully release the pin after TX.

3. **Don't block the main loop during TX** — use a FreeRTOS task to wait for `uart_wait_tx_done()` and restore RX mode.

4. **EdgeTX requires a Device Info handshake** — if you send any telemetry without responding to Device Ping, the radio freezes permanently. This is undocumented outside the source code.

5. **Auto-detect baud rate** by scanning common rates and checking for valid CRC. During development, this saved hours when I wasn't sure if the transmitter was at 115200 or 420000.

## Resources

- [CRSF Protocol Wiki — Physical Layer](https://github.com/crsf-wg/crsf/wiki/Physical-Layer)
- [mLRS — JR Pin5 ESP32 Implementation](https://github.com/olliw42/mLRS) (the GPIO matrix pattern)
- [EdgeTX Source — CRSF Telemetry](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.cpp) (handshake logic)
- [ExpressLRS — CRSFHandset.cpp](https://github.com/ExpressLRS/ExpressLRS) (half-duplex reference)
- [CRSFDude on GitHub](https://github.com/TODO) (this project)
