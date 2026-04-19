#include <Arduino.h>
#include "driver/uart.h"

// --- CRSF Protocol ---
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_SYNC_BYTE_TX     0xEE
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_MIN_PACKET_LEN   4
#define CRSF_CRC_POLY         0xD5

// CRSF frame types
#define CRSF_FRAMETYPE_GPS                0x02
#define CRSF_FRAMETYPE_VARIO              0x07
#define CRSF_FRAMETYPE_BATTERY            0x08
#define CRSF_FRAMETYPE_BARO_ALT           0x09
#define CRSF_FRAMETYPE_HEARTBEAT          0x0B
#define CRSF_FRAMETYPE_LINK_STATISTICS    0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE           0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE        0x21
#define CRSF_FRAMETYPE_DEVICE_PING        0x28
#define CRSF_FRAMETYPE_DEVICE_INFO        0x29
#define CRSF_FRAMETYPE_PARAMETER_READ     0x2C
#define CRSF_FRAMETYPE_PARAMETER_WRITE    0x2D
#define CRSF_FRAMETYPE_COMMAND            0x32

// --- Pin Config ---
#define CRSF_RX_PIN 20
#define CRSF_TX_PIN 20

// --- CRC ---
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

// --- UART ---
static const uint32_t baudRates[] = { 115200, 420000, 400000, 921600 };
static const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

HardwareSerial CrsfSerial(1);

static inline bool isSyncByte(uint8_t b)
{
    return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_TX;
}

static void setRxInversion(bool inverted)
{
    if (inverted)
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
    else
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);
}

static int countValidFrames(const uint8_t *buf, int len)
{
    int count = 0, pos = 0;
    while (pos < len - 3)
    {
        if (!isSyncByte(buf[pos])) { pos++; continue; }
        uint8_t totalLen = buf[pos + 1] + 2;
        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN) { pos++; continue; }
        if (pos + totalLen > len) break;
        if (crc8_calc(&buf[pos + 2], totalLen - 3) == buf[pos + totalLen - 1])
            count++;
        pos += totalLen;
    }
    return count;
}

static bool detectedInverted = false;

static uint32_t detectBaudRate()
{
    Serial.println("Auto-detecting CRSF...");

    for (int inv = 0; inv <= 1; inv++)
    {
        bool inverted = (inv == 1);
        for (int i = 0; i < numBaudRates; i++)
        {
            CrsfSerial.end();
            delay(50);
            CrsfSerial.begin(baudRates[i], SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
            CrsfSerial.setTimeout(0);
            setRxInversion(inverted);
            delay(100);
            while (CrsfSerial.available()) CrsfSerial.read();
            delay(50);

            uint8_t buf[128];
            int got = 0;
            uint32_t t0 = millis();
            while (got < 128 && millis() - t0 < 500)
            {
                if (CrsfSerial.available())
                    buf[got++] = CrsfSerial.read();
            }

            int valid = countValidFrames(buf, got);
            if (valid >= 3)
            {
                Serial.printf("Detected: %u baud, %s, %d frames\n",
                              baudRates[i], inverted ? "inverted" : "normal", valid);
                detectedInverted = inverted;
                return baudRates[i];
            }
        }
    }

    Serial.println("No CRSF detected, defaulting to 115200 inverted");
    detectedInverted = true;
    return 115200;
}

// --- RC Channel decoding (packed 11-bit channels) ---
static uint16_t channels[16];

static void decodeChannels(const uint8_t *payload)
{
    channels[0]  = ((uint16_t)payload[0]       | (uint16_t)payload[1]  << 8) & 0x07FF;
    channels[1]  = ((uint16_t)payload[1]  >> 3 | (uint16_t)payload[2]  << 5) & 0x07FF;
    channels[2]  = ((uint16_t)payload[2]  >> 6 | (uint16_t)payload[3]  << 2 | (uint16_t)payload[4]  << 10) & 0x07FF;
    channels[3]  = ((uint16_t)payload[4]  >> 1 | (uint16_t)payload[5]  << 7) & 0x07FF;
    channels[4]  = ((uint16_t)payload[5]  >> 4 | (uint16_t)payload[6]  << 4) & 0x07FF;
    channels[5]  = ((uint16_t)payload[6]  >> 7 | (uint16_t)payload[7]  << 1 | (uint16_t)payload[8]  << 9) & 0x07FF;
    channels[6]  = ((uint16_t)payload[8]  >> 2 | (uint16_t)payload[9]  << 6) & 0x07FF;
    channels[7]  = ((uint16_t)payload[9]  >> 5 | (uint16_t)payload[10] << 3) & 0x07FF;
    channels[8]  = ((uint16_t)payload[11]      | (uint16_t)payload[12] << 8) & 0x07FF;
    channels[9]  = ((uint16_t)payload[12] >> 3 | (uint16_t)payload[13] << 5) & 0x07FF;
    channels[10] = ((uint16_t)payload[13] >> 6 | (uint16_t)payload[14] << 2 | (uint16_t)payload[15] << 10) & 0x07FF;
    channels[11] = ((uint16_t)payload[15] >> 1 | (uint16_t)payload[16] << 7) & 0x07FF;
    channels[12] = ((uint16_t)payload[16] >> 4 | (uint16_t)payload[17] << 4) & 0x07FF;
    channels[13] = ((uint16_t)payload[17] >> 7 | (uint16_t)payload[18] << 1 | (uint16_t)payload[19] << 9) & 0x07FF;
    channels[14] = ((uint16_t)payload[19] >> 2 | (uint16_t)payload[20] << 6) & 0x07FF;
    channels[15] = ((uint16_t)payload[20] >> 5 | (uint16_t)payload[21] << 3) & 0x07FF;
}

// --- Frame type name ---
static const char* frameTypeName(uint8_t type)
{
    switch (type)
    {
        case CRSF_FRAMETYPE_GPS:                return "GPS";
        case CRSF_FRAMETYPE_VARIO:              return "VARIO";
        case CRSF_FRAMETYPE_BATTERY:            return "BATTERY";
        case CRSF_FRAMETYPE_BARO_ALT:           return "BARO";
        case CRSF_FRAMETYPE_HEARTBEAT:          return "HEARTBEAT";
        case CRSF_FRAMETYPE_LINK_STATISTICS:     return "LINK_STATS";
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:  return "RC_CHANNELS";
        case CRSF_FRAMETYPE_ATTITUDE:           return "ATTITUDE";
        case CRSF_FRAMETYPE_FLIGHT_MODE:        return "FLIGHT_MODE";
        case CRSF_FRAMETYPE_DEVICE_PING:        return "DEV_PING";
        case CRSF_FRAMETYPE_DEVICE_INFO:        return "DEV_INFO";
        case CRSF_FRAMETYPE_PARAMETER_READ:     return "PARAM_READ";
        case CRSF_FRAMETYPE_PARAMETER_WRITE:    return "PARAM_WRITE";
        case CRSF_FRAMETYPE_COMMAND:            return "COMMAND";
        default:                                return "UNKNOWN";
    }
}

// --- Parser state ---
static uint8_t inBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t bufferPtr = 0;
static uint32_t goodPktCount = 0;
static uint32_t badPktCount = 0;
static uint32_t badSyncCount = 0;
static uint32_t badLenCount = 0;
static uint32_t badCrcCount = 0;
static uint32_t lastReportTime = 0;
static uint8_t lastBadFrame[CRSF_MAX_PACKET_LEN];
static uint8_t lastBadLen = 0;
static bool badFrameCaptured = false;
static uint32_t frameTypeCounts[256] = {};

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

static void processFrame(const uint8_t *frame, uint8_t totalLen)
{
    uint8_t type = frame[2];
    frameTypeCounts[type]++;

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && totalLen >= 26)
        decodeChannels(&frame[3]);
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
        if (!isSyncByte(inBuffer[0]))
        {
            badPktCount++;
            badSyncCount++;
            alignBufferToSync();
            continue;
        }

        if (bufferPtr < 2) break;

        uint8_t totalLen = inBuffer[1] + 2;

        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN)
        {
            badPktCount++;
            badLenCount++;
            if (!badFrameCaptured)
            {
                memcpy(lastBadFrame, inBuffer, min((int)bufferPtr, (int)CRSF_MAX_PACKET_LEN));
                lastBadLen = min(bufferPtr, (uint8_t)16);
                badFrameCaptured = true;
            }
            alignBufferToSync();
            continue;
        }

        if (bufferPtr < totalLen) break;

        if (crc8_calc(&inBuffer[2], totalLen - 3) == inBuffer[totalLen - 1])
        {
            goodPktCount++;
            processFrame(inBuffer, totalLen);
        }
        else
        {
            badPktCount++;
            badCrcCount++;
            if (!badFrameCaptured)
            {
                memcpy(lastBadFrame, inBuffer, totalLen);
                lastBadLen = totalLen;
                badFrameCaptured = true;
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

    uint32_t baud = detectBaudRate();
    CrsfSerial.end();
    delay(50);
    CrsfSerial.begin(baud, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    CrsfSerial.setTimeout(0);
    setRxInversion(detectedInverted);

    lastReportTime = millis();
}

void loop()
{
    handleInput();

    uint32_t now = millis();
    if (now - lastReportTime >= 1000)
    {
        uint32_t good = goodPktCount;
        uint32_t bad = badPktCount;
        uint32_t bSync = badSyncCount;
        uint32_t bLen = badLenCount;
        uint32_t bCrc = badCrcCount;
        goodPktCount = badPktCount = badSyncCount = badLenCount = badCrcCount = 0;

        // Summary line
        Serial.printf("\n--- %u ok, %u bad (sync:%u len:%u crc:%u) /sec ---\n",
                      good, bad, bSync, bLen, bCrc);

        // Show one captured bad frame
        if (badFrameCaptured)
        {
            Serial.print("  bad frame: ");
            for (int j = 0; j < lastBadLen; j++)
                Serial.printf("%02X ", lastBadFrame[j]);
            Serial.println();
            badFrameCaptured = false;
        }

        // Frame type breakdown
        for (int t = 0; t < 256; t++)
        {
            if (frameTypeCounts[t] > 0)
            {
                Serial.printf("  0x%02X %-14s %u/sec\n", t, frameTypeName(t), frameTypeCounts[t]);
                frameTypeCounts[t] = 0;
            }
        }

        // RC channels (if received)
        if (channels[0] != 0 || channels[1] != 0)
        {
            Serial.printf("  CH1-4: %4u %4u %4u %4u  CH5-8: %4u %4u %4u %4u\n",
                          channels[0], channels[1], channels[2], channels[3],
                          channels[4], channels[5], channels[6], channels[7]);
        }

        lastReportTime = now;
    }
}
