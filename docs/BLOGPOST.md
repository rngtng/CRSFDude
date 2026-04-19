---
layout: default
title: "CRSFDude: Custom CRSF Module with ESP32-C3 aka Half-Duplex Hell"
---

# CRSFDude: Custom CRSF Module with ESP32-C3 aka Half-Duplex Hell

So I had this idea — plug an ESP32-C3 into the JR bay of my EdgeTX radio, read RC channels, send telemetry back. One signal wire, how hard can it be? Spoiler: two days of debugging to get a single telemetry value on screen. Here's the ride.

## What I wanted

- Read CRSF RC channel data from the radio
- Send telemetry (flight mode, battery, you name it) back to EdgeTX
- All on a single wire — that's all the JR bay gives you

## Reading CRSF — the inverted surprise

CRSF is a serial protocol. 420000 baud, 8N1. Frames start with a sync byte (0xC8), followed by length, type, payload, CRC8. Straightforward stuff.

Wired GPIO 20 to the JR bay signal pin, `HardwareSerial.begin()`, and... garbage. Built an auto-detect that scans baud rates and validates CRC — nothing at any rate. Hmm.

Turns out the signal is **inverted**! EdgeTX radios have a hardware inverter IC on the S.PORT line. The [CRSF spec](https://github.com/crsf-wg/crsf/wiki/Physical-Layer) even says it: *"Half duplex connections are typically INVERTED polarity (idle low)."* Would've been nice to read that first ;)

The fix is a one-liner. ESP32-C3 can invert the RX signal in hardware:

```cpp
uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
```

Zero CPU cost. Boom — 150 packets/second of clean RC channel data.

## Half-Duplex — me wantz TX too!

Same pin, same wire. Radio sends RC data, then the module gets a brief window to respond with telemetry. STM32 (what radios use) has native half-duplex UART. ESP32-C3? Nope.

Oh boy, I tried everything:
- `uart_set_line_inverse` with both RX+TX inversion — killed RX permanently
- Open-drain GPIO mode — inverted idle drives the line LOW, blocks the radio
- `Serial.end()` + `begin()` to reinit after TX — data lost during reinit
- Various `gpio_matrix_out` + `SIG_GPIO_OUT_IDX` detach combos — pin stays driving

**What finally worked:** GPIO matrix switching, modeled after the [mLRS project](https://github.com/olliw42/mLRS). ESP32's GPIO matrix lets you dynamically route peripheral signals to pins:

```cpp
// TX mode: disconnect RX, connect TX (inverted)
gpio_matrix_in(MATRIX_DETACH_IN_LOW, U1RXD_IN_IDX, true);
gpio_set_direction(pin, GPIO_MODE_OUTPUT);
gpio_matrix_out(pin, U1TXD_OUT_IDX, true, false);

// Back to RX mode
gpio_reset_pin(pin);    // <-- crucial on ESP32-C3!
gpio_set_direction(pin, GPIO_MODE_INPUT);
gpio_matrix_in(pin, U1RXD_IN_IDX, true);
```

Heads up: on ESP32-C3, `gpio_set_direction(INPUT)` alone does NOT release the TX output. You need `gpio_reset_pin()` to fully detach. Wasted hours on this. The original ESP32 behaves differently here.

The TX-to-RX switch runs in a FreeRTOS task that waits for `uart_wait_tx_done()` before switching back — keeps the main parsing loop non-blocking.

## The Radio Froze. Like, permanently.

I could see my flight mode value "ON" show up on the radio. YES! Then the radio stopped sending RC data. Dead. Required a full power cycle to recover.

This was *the* most frustrating part. Half-duplex was working. Frame format valid. CRC correct. But the radio froze after receiving just one telemetry frame. Every. Single. Time.

**Root cause:** Buried in the [EdgeTX source](https://github.com/EdgeTX/edgetx) I found the CRSF handshake protocol. When the radio receives *any* valid telemetry from the external module bay, it enters a module initialization sequence:

1. Radio marks module as "alive"
2. Radio sends a **Model ID** frame
3. Radio sends a **Device Ping** (0x28)
4. Radio waits for **Device Info** (0x29) response
5. Without that response → radio halts CRSF output. Forever.

This is completely undocumented outside the source code. The fix is almost comically simple:

```cpp
if (frameType == CRSF_FRAMETYPE_DEVICE_PING) {
    sendDeviceInfo("CRSFDude");
}
```

Voila! Radio stays happy. RC channels keep flowing, telemetry shows up, no freezes.

## What I ended up with

A reusable [`CRSFProtocol` library](https://github.com/rngtng/CRSFDude) for ESP32-C3 that handles all the gnarly bits:

- CRC8 calculation + frame parsing
- All 16 RC channels (packed 11-bit decoding)
- Half-duplex GPIO matrix switching with inverted signal
- Telemetry TX (flight mode, device info, raw frames)
- EdgeTX handshake (auto-responds to Device Ping)

The actual application? ~40 lines:

```cpp
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

## TL;DR — what I learned

1. **JR bay CRSF is inverted** — hardware inverter on the radio. Wire idles LOW.
2. **ESP32-C3 half-duplex = GPIO matrix juggling** — no native support. Route TX/RX dynamically. `gpio_reset_pin()` is your friend on C3.
3. **Don't block during TX** — FreeRTOS task for `uart_wait_tx_done()` + RX restore.
4. **EdgeTX Device Ping handshake is mandatory** — respond to 0x28 with 0x29 or the radio freezes. Not documented anywhere except the source.
5. **Auto-detect baud rate** early — scan rates, check CRC. Saved me when I wasn't sure if the radio was at 115200 or 420000.

## Resources & shout-outs

- [CRSF Protocol Wiki — Physical Layer](https://github.com/crsf-wg/crsf/wiki/Physical-Layer)
- [mLRS](https://github.com/olliw42/mLRS) — the GPIO matrix pattern for JR Pin5 on ESP32
- [EdgeTX Source](https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.cpp) — where I found the handshake logic
- [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) — half-duplex reference implementation
- [CRSFDude on GitHub](https://github.com/rngtng/CRSFDude)
