#pragma once
#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_ll.h"

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_SYNC_BYTE_MODULE   0xEE
#define CRSF_MAX_PACKET_LEN     64
#define CRSF_MIN_PACKET_LEN     4
#define CRSF_CRC_POLY           0xD5

#define CRSF_FRAMETYPE_RC_CHANNELS  0x16
#define CRSF_FRAMETYPE_DEVICE_PING  0x28
#define CRSF_FRAMETYPE_DEVICE_INFO  0x29
#define CRSF_FRAMETYPE_FLIGHT_MODE  0x21

#define CRSF_ADDRESS_RADIO          0xEA
#define CRSF_ADDRESS_MODULE         0xEE

class CRSFProtocol {
public:
    // Initialize UART and half-duplex on given pin
    void begin(uint8_t pin, uint32_t baudRate = 420000);

    // Call from loop() — parses incoming frames, returns true when new RC data available
    bool update();

    // Read decoded RC channel (0-based index, 11-bit value 0-2047)
    uint16_t getChannel(uint8_t ch) const;

    // Send a telemetry frame (raw CRSF frame including sync, length, type, payload, CRC)
    void sendFrame(const uint8_t *frame, uint8_t length);

    // Build and send a flight mode string
    void sendFlightMode(const char *mode);

    // Build and send device info (required for EdgeTX handshake)
    void sendDeviceInfo(const char *deviceName);

    // Packet counters
    uint32_t rxPacketCount = 0;
    uint32_t txPacketCount = 0;

    // Callback: called when Device Ping received (default: auto-responds with device info)
    void (*onDevicePing)() = nullptr;

    // CRC utility
    static uint8_t crc8(const uint8_t *data, uint16_t length);

private:
    static void crc8_init();
    static uint8_t crc8_table[256];
    static bool crc8_initialized;

    // Half-duplex GPIO matrix switching
    uint8_t _pin = 0;
    static void IRAM_ATTR halfDuplexEnableRX(uint8_t pin);
    static void IRAM_ATTR halfDuplexEnableTX(uint8_t pin);

    // TX done task
    static TaskHandle_t _txDoneTaskHandle;
    static uint8_t _txDonePin;
    static void txDoneTask(void *param);

    // Parser state
    uint8_t _parseBuffer[CRSF_MAX_PACKET_LEN];
    uint8_t _parseBufferLen = 0;
    uint16_t _channels[16] = {};
    const char *_deviceName = "CRSFDevice";

    void alignBufferToSync();
    void processFrame(uint8_t frameType, uint8_t totalLength);

    static inline bool isSyncByte(uint8_t b) {
        return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_MODULE;
    }

    HardwareSerial _serial{1};
};
