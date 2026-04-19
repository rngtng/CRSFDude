#include <Arduino.h>
#include "driver/uart.h"

// --- CRSF Protocol Constants ---
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_SYNC_BYTE_TX     0xEE
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_MIN_PACKET_LEN   4
#define CRSF_CRC_POLY         0xD5

// --- CRSF Pin Config ---
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

// --- Baud rate scanning ---
static const uint32_t baudRates[] = { 115200, 420000, 400000, 921600, 1870000, 3750000, 5250000 };
static const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

HardwareSerial CrsfSerial(1);

static inline bool isSyncByte(uint8_t b)
{
    return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_TX;
}

// Try to find valid CRSF frames in raw buffer
static int countValidFrames(const uint8_t *buf, int len)
{
    int count = 0;
    int pos = 0;
    while (pos < len - 3)
    {
        if (!isSyncByte(buf[pos])) { pos++; continue; }
        uint8_t frameLen = buf[pos + 1];
        uint8_t totalLen = frameLen + 2;
        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN) { pos++; continue; }
        if (pos + totalLen > len) break;
        uint8_t crc = crc8_calc(&buf[pos + 2], totalLen - 3);
        if (crc == buf[pos + totalLen - 1])
            count++;
        pos += totalLen;
    }
    return count;
}

static bool detectedInverted = false;

static void setRxInversion(bool inverted)
{
    if (inverted)
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
    else
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);
}

static int probeUart(uint32_t baud, bool inverted, uint8_t *buf, int bufSize)
{
    CrsfSerial.end();
    delay(50);
    CrsfSerial.begin(baud, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    CrsfSerial.setTimeout(0);
    setRxInversion(inverted);
    delay(100);

    while (CrsfSerial.available()) CrsfSerial.read();
    delay(50);

    int got = 0;
    uint32_t t0 = millis();
    while (got < bufSize && millis() - t0 < 500)
    {
        if (CrsfSerial.available())
            buf[got++] = CrsfSerial.read();
    }
    return got;
}

static uint32_t detectBaudRate()
{
    Serial.println("Auto-detecting baud rate (normal + inverted)...");

    for (int inv = 0; inv <= 1; inv++)
    {
        bool inverted = (inv == 1);
        Serial.printf("--- %s signal ---\n", inverted ? "INVERTED" : "NORMAL");

        for (int i = 0; i < numBaudRates; i++)
        {
            uint8_t buf[128];
            int got = probeUart(baudRates[i], inverted, buf, 128);
            int valid = countValidFrames(buf, got);

            Serial.printf("  %u baud: %d bytes, %d valid [", baudRates[i], got, valid);
            for (int j = 0; j < min(got, 32); j++)
                Serial.printf("%02X ", buf[j]);
            Serial.println("]");

            if (valid >= 3)
            {
                Serial.printf(">>> Detected CRSF at %u baud %s (%d valid frames)\n",
                              baudRates[i], inverted ? "(inverted)" : "(normal)", valid);
                detectedInverted = inverted;
                return baudRates[i];
            }
        }
    }

    Serial.println("No valid CRSF detected. Defaulting to 420000 normal.");
    detectedInverted = false;
    return 420000;
}

// --- Parser state ---
static uint8_t inBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t bufferPtr = 0;
static uint32_t goodPktCount = 0;
static uint32_t badCrcCount = 0;
static uint32_t badLenCount = 0;
static uint32_t syncLostCount = 0;
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
        if (!isSyncByte(inBuffer[0]))
        {
            syncLostCount++;
            alignBufferToSync();
            continue;
        }

        if (bufferPtr < 2) break;

        uint8_t frameLen = inBuffer[1];
        uint8_t totalLen = frameLen + 2;

        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN)
        {
            badLenCount++;
            alignBufferToSync();
            continue;
        }

        if (bufferPtr < totalLen) break;

        uint8_t crc = crc8_calc(&inBuffer[2], totalLen - 3);
        if (crc == inBuffer[totalLen - 1])
            goodPktCount++;
        else
            badCrcCount++;

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
        uint32_t badCrc = badCrcCount;
        uint32_t badLen = badLenCount;
        uint32_t syncLost = syncLostCount;
        goodPktCount = badCrcCount = badLenCount = syncLostCount = 0;

        uint32_t total = good + badCrc + badLen;
        if (total > 0 || syncLost > 0)
        {
            Serial.printf("CRSF: %u ok, %u bad_crc, %u bad_len, %u sync_lost /sec\n",
                          good, badCrc, badLen, syncLost);
        }
        else
        {
            Serial.println("CRSF: no signal");
        }

        lastReportTime = now;
    }
}
