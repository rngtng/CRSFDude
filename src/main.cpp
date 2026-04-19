#include <Arduino.h>
#include "driver/uart.h"

// CRSF Protocol
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_SYNC_BYTE_TX     0xEE
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_MIN_PACKET_LEN   4
#define CRSF_CRC_POLY         0xD5
#define CRSF_BAUD             420000
#define CRSF_RC_CHANNELS      0x16

// Pin Config
#define CRSF_RX_PIN 20
#define CRSF_TX_PIN 21

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

// RC Channels (packed 11-bit)
static uint16_t channels[4];

static void decodeChannels(const uint8_t *p)
{
    channels[0] = ((uint16_t)p[0]      | (uint16_t)p[1] << 8) & 0x07FF;
    channels[1] = ((uint16_t)p[1] >> 3 | (uint16_t)p[2] << 5) & 0x07FF;
    channels[2] = ((uint16_t)p[2] >> 6 | (uint16_t)p[3] << 2 | (uint16_t)p[4] << 10) & 0x07FF;
    channels[3] = ((uint16_t)p[4] >> 1 | (uint16_t)p[5] << 7) & 0x07FF;
}

// Parser
static uint8_t inBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t bufferPtr = 0;
static uint32_t pktCount = 0;
static uint32_t lastReportTime = 0;

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
            if (inBuffer[2] == CRSF_RC_CHANNELS && totalLen >= 26)
            {
                pktCount++;
                decodeChannels(&inBuffer[3]);
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
    CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    CrsfSerial.setTimeout(0);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);

    lastReportTime = millis();
}

void loop()
{
    handleInput();

    uint32_t now = millis();
    if (now - lastReportTime >= 1000)
    {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [%u pkt/s]\n",
                      channels[0], channels[1], channels[2], channels[3], pktCount);
        pktCount = 0;
        lastReportTime = now;
    }
}
