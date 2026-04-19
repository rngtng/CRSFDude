#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_ll.h"

// CRSF Protocol
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_SYNC_BYTE_TX     0xEE
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_MIN_PACKET_LEN   4
#define CRSF_CRC_POLY         0xD5
#define CRSF_BAUD             420000
#define CRSF_RC_CHANNELS      0x16

#define CRSF_PIN 20
#define MATRIX_DETACH_IN_LOW 0x30

// CRC
static uint8_t crc8tab[256];

static void crc8_init()
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        crc8tab[i] = crc;
    }
}

static uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
        crc = crc8tab[crc ^ *data++];
    return crc;
}

// UART
HardwareSerial CrsfSerial(1);

static inline bool isSyncByte(uint8_t b)
{
    return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_TX;
}

// Half-duplex GPIO matrix switching — exact mLRS pattern
static void IRAM_ATTR pin5_rx_enable()
{
    gpio_set_pull_mode((gpio_num_t)CRSF_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_direction((gpio_num_t)CRSF_PIN, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)CRSF_PIN, U1RXD_IN_IDX, true);
}

static void IRAM_ATTR pin5_tx_enable()
{
    // Remap: TX to pin 20, RX disconnected
    uart_set_pin(UART_NUM_1, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Re-apply TX inversion at GPIO matrix
    gpio_matrix_out((gpio_num_t)CRSF_PIN, U1TXD_OUT_IDX, true, false);
}

// TX done task on Core 0 — mLRS pattern: wait for TX complete, then switch to RX
static TaskHandle_t txDoneTaskHandle = nullptr;

static void txDoneTask(void *param)
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20));

        // Fully reset pin 20: detach from all peripherals, return to GPIO
        gpio_reset_pin((gpio_num_t)CRSF_PIN);
        // Now configure as RX input
        gpio_set_direction((gpio_num_t)CRSF_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)CRSF_PIN, GPIO_PULLDOWN_ONLY);
        gpio_matrix_in((gpio_num_t)CRSF_PIN, U1RXD_IN_IDX, true);
        // Reset RX FIFO
        uart_ll_rxfifo_rst(UART_LL_GET_HW(1));
    }
}

// RC Channels (packed 11-bit)
static uint16_t channels[4];

static void decodeChannels(const uint8_t *p)
{
    channels[0] = ((uint16_t)p[0]      | (uint16_t)p[1] << 8) & 0x07FF;
    channels[1] = ((uint16_t)p[1] >> 3 | (uint16_t)p[2] << 5) & 0x07FF;
    channels[2] = ((uint16_t)p[2] >> 6 | (uint16_t)p[3] << 2 | (uint16_t)p[4] << 10) & 0x07FF;
    channels[3] = ((uint16_t)p[4] >> 1 | (uint16_t)p[5] << 7) & 0x07FF;
}

// Parser state
static uint8_t inBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t bufferPtr = 0;
static uint32_t pktCount = 0;
static uint32_t txCount = 0;
static uint32_t lastReportTime = 0;

static void crsfSend(const uint8_t *buf, uint8_t len)
{
    pin5_tx_enable();
    uart_write_bytes(UART_NUM_1, (const char *)buf, len);
    // Notify the TX done task (runs on Core 0) to handle completion + RX restore
    xTaskNotifyGive(txDoneTaskHandle);
    bufferPtr = 0;
    txCount++;
}

// Respond to Device Ping with Device Info — required for CRSF handshake
static void sendDeviceInfo()
{
    // Device Info frame (0x29) — extended header format
    const char deviceName[] = "CRSFDude";
    uint8_t nameLen = sizeof(deviceName); // includes null terminator
    uint8_t buf[48];
    uint8_t i = 0;
    buf[i++] = 0xC8;                    // sync
    // length filled later
    uint8_t lenIdx = i++;
    buf[i++] = 0x29;                    // type: Device Info
    buf[i++] = 0xEA;                    // dest: radio
    buf[i++] = 0xEE;                    // origin: external module
    memcpy(&buf[i], deviceName, nameLen);
    i += nameLen;
    // Serial number (4 bytes)
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0; buf[i++] = 1;
    // Hardware ID (4 bytes)
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0; buf[i++] = 1;
    // Firmware ID (4 bytes)
    buf[i++] = 0; buf[i++] = 0; buf[i++] = 0; buf[i++] = 1;
    // Parameter count + protocol version
    buf[i++] = 0;   // parameter count
    buf[i++] = 0;   // protocol version
    buf[lenIdx] = i - 1; // length: everything after sync+len, including CRC
    buf[i] = crc8_calc(&buf[2], i - 2);
    i++;
    crsfSend(buf, i);
}

static void sendFlightMode()
{
    static uint32_t lastToggleTime = 0;
    static bool isSignalOn = false;
    uint32_t now = millis();
    if (now - lastToggleTime >= 2000) {
        isSignalOn = !isSignalOn;
        lastToggleTime = now;
    }
    const char* mode = isSignalOn ? "ON" : "OFF";
    uint8_t strLen = strlen(mode) + 1;
    uint8_t frameLen = 1 + strLen + 1;
    uint8_t buffer[32];
    buffer[0] = 0xC8;
    buffer[1] = frameLen;
    buffer[2] = 0x21;
    memcpy(&buffer[3], mode, strLen);
    buffer[3 + strLen] = crc8_calc(&buffer[2], strLen + 1);
    crsfSend(buffer, 3 + strLen + 1);
}

static void alignBufferToSync()
{
    uint8_t i = 1;
    while (i < bufferPtr && !isSyncByte(inBuffer[i]))
        i++;
    if (i < bufferPtr)
    {
        bufferPtr -= i;
        memmove(inBuffer, inBuffer + i, bufferPtr);
    }
    else
    {
        bufferPtr = 0;
    }
}

static void handleInput()
{
    int available = CrsfSerial.available();
    if (available <= 0) return;

    int toRead = min(available, (int)(CRSF_MAX_PACKET_LEN - bufferPtr));
    if (toRead <= 0) { bufferPtr = 0; return; }

    bufferPtr += CrsfSerial.readBytes(&inBuffer[bufferPtr], toRead);

    while (bufferPtr >= CRSF_MIN_PACKET_LEN)
    {
        if (!isSyncByte(inBuffer[0])) { alignBufferToSync(); continue; }
        if (bufferPtr < 2) break;

        uint8_t totalLen = inBuffer[1] + 2;
        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN)
        {
            alignBufferToSync();
            continue;
        }
        if (bufferPtr < totalLen) break;

        if (crc8_calc(&inBuffer[2], totalLen - 3) == inBuffer[totalLen - 1])
        {
            uint8_t frameType = inBuffer[2];

            if (frameType == CRSF_RC_CHANNELS && totalLen >= 26)
            {
                pktCount++;
                decodeChannels(&inBuffer[3]);
                if (pktCount % 5 == 0)
                {
                    sendFlightMode();
                    return;
                }
            }
            else if (frameType == 0x28) // Device Ping
            {
                sendDeviceInfo();
                return;
            }
        }

        uint8_t remaining = bufferPtr - totalLen;
        if (remaining > 0)
            memmove(inBuffer, inBuffer + totalLen, remaining);
        bufferPtr = remaining;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(1000);
    Serial.println("CRSFDude starting...");

    crc8_init();
    CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_PIN, CRSF_PIN);
    CrsfSerial.setTimeout(0);

    // Start TX done task on Core 0 (main loop runs on Core 1)
    xTaskCreatePinnedToCore(txDoneTask, "TxDone", 2048, NULL, 5,
                            &txDoneTaskHandle, 0);

    pin5_rx_enable();
    lastReportTime = millis();
}

void loop()
{
    handleInput();

    uint32_t now = millis();
    if (now - lastReportTime >= 1000)
    {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [rx:%u tx:%u /s]\n",
                      channels[0], channels[1], channels[2], channels[3], pktCount, txCount);
        pktCount = txCount = 0;
        lastReportTime = now;
    }
}
