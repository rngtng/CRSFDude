#pragma once
#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_ll.h"
#include "crsf_protocol.h"

class CRSFDude {
public:
    // Initialize UART and half-duplex on given pin
    void begin(uint8_t pin, uint32_t baudRate = 420000);

    // Call from loop() — parses incoming frames, returns true when new RC data available
    bool update();

    // Read decoded RC channel (0-based index, 11-bit value 0-2047)
    uint16_t getChannel(uint8_t ch) const;

    // Send a raw CRSF frame (sync, length, type, payload, CRC all included)
    void sendFrame(const uint8_t *frame, uint8_t length);

    // --- Telemetry helpers ---

    // Flight mode string (shows in EdgeTX top bar)
    void sendFlightMode(const char *mode);

    // Battery: voltage in 0.1V, current in 0.1A, capacity in mAh, remaining in %
    void sendBattery(uint16_t voltage, uint16_t current, uint32_t capacity, uint8_t remaining);

    // GPS: lat/lon in degrees*1e7, groundspeed in cm/s, heading in degrees*100, altitude in m, satellites
    void sendGPS(int32_t latitude, int32_t longitude, uint16_t groundspeed,
                 uint16_t heading, uint16_t altitude, uint8_t satellites);

    // Attitude: pitch/roll/yaw in radians*10000 (e.g. 1.5rad = 15000)
    void sendAttitude(int16_t pitch, int16_t roll, int16_t yaw);

    // Barometric altitude in cm (e.g. 1234 = 12.34m)
    void sendBaroAltitude(int32_t altitudeCm);

    // Variometer: vertical speed in cm/s
    void sendVario(int16_t verticalSpeed);

    // Link statistics — MUST be sent to enable telemetry streaming in EdgeTX
    // Without this, all sensors except flight mode are silently dropped
    void sendLinkStats(uint8_t rxRssi1, uint8_t rxRssi2, uint8_t rxQuality, int8_t rxSnr,
                       uint8_t antenna, uint8_t rfMode, uint8_t txPower,
                       uint8_t txRssi, uint8_t txQuality, int8_t txSnr);

    // Device info (required for EdgeTX handshake)
    void sendDeviceInfo(const char *deviceName);

    // Packet counters
    uint32_t rxPacketCount = 0;
    uint32_t txPacketCount = 0;

    // Active model ID (0-63, set by radio via Model Select command)
    uint8_t modelId = 0;

    // Callbacks
    void (*onDevicePing)() = nullptr;          // Device Ping received (default: auto-responds)
    void (*onModelIdChanged)(uint8_t id) = nullptr; // Model ID changed by radio

    // CRC8 utility (public for custom frame building)
    static uint8_t crc8(const uint8_t *data, uint16_t length);

private:
    // Half-duplex GPIO matrix switching
    uint8_t _pin = 0;
    static void halfDuplexEnableRX(uint8_t pin);
    static void halfDuplexEnableTX(uint8_t pin);

    // Build a simple telemetry frame: writes sync, length, type, then returns pointer after type
    // Call crc8 and append CRC after filling payload
    uint8_t buildFrameHeader(uint8_t *frame, uint8_t type, uint8_t payloadLength);

    // Parser state
    uint8_t _parseBuffer[CRSF_MAX_PACKET_LEN];
    uint8_t _parseBufferLen = 0;
    uint16_t _channels[16] = {};

    void alignBufferToSync();
    void processFrame(uint8_t frameType, uint8_t totalLength);

    static inline bool isSyncByte(uint8_t b) {
        return crsfIsSyncByte(b);
    }

    HardwareSerial _serial{1};
};
