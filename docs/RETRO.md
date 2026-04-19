# CRSFDude — Session Retro

## What slowed us down

### Trial-and-error on half-duplex pin switching
~15 iterations of GPIO matrix / uart_set_line_inverse / open-drain combos. Each required flash + test + radio restart. Should have searched the internet earlier for existing implementations (mLRS, ESP32 forums). Had to be nudged twice to do web searches.

**Lesson:** Search for working reference implementations before trying to figure out low-level hardware behavior from scratch.

### Misdiagnosing the radio freeze
Many iterations assuming it was a pin release problem. It was actually a protocol problem (missing Device Ping handshake). Only found it after cloning EdgeTX and reading the source.

**Lesson:** When behavior is "works once then stops forever", think protocol state machine, not just hardware.

### Not reading the CRSF spec first
The inverted signal is documented. The half-duplex timing is documented. Could have skipped the auto-detect detour by reading the spec up front.

## How to go faster next time

- **Point at reference code early.** Best unblocking moves were cloning EdgeTX and linking the mLRS wiki. Reading existing code beats guessing hardware behavior.
- **Full hardware setup up front.** "Single wire, half-duplex, JR bay, ESP32-C3" from the start would have skipped several dead ends (two-pin attempts, wrong TX pin assumptions).
- **Push for web searches sooner.** Both times it was suggested, it unblocked us (mLRS implementation, ESP32 forum).
- **Establish test protocol early.** "Restart radio between tests" wasn't obvious at first. A few times we thought code was broken but the radio was just stuck from a previous attempt.

## What went well

- Auto-detect baud scanner was great for initial signal discovery
- Incremental approach (RX first → TX → telemetry) kept momentum
- Live testing + pasting serial output made diagnosis fast and clear
- Reading the EdgeTX source was the breakthrough — wouldn't have found the handshake any other way
- End result is genuinely useful: standalone ESP32-C3 CRSF module library with half-duplex TX and EdgeTX handshake support
