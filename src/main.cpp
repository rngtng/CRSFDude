#include <Arduino.h>

// --- CRSF Protocol Constants (from ExpressLRS crsf_protocol.h) ---
#define CRSF_SYNC_BYTE        0xC8
#define CRSF_MAX_PACKET_LEN   64
#define CRSF_MIN_PACKET_LEN   4

// CRC8 with polynomial 0xD5 (standard CRSF)
#define CRSF_CRC_POLY 0xD5

// --- CRSF Pin Config ---
#define CRSF_RX_PIN 20
#define CRSF_TX_PIN 21
//#define CRSF_BAUD   420000  // Common CRSF baud for receivers/handsets
#define CRSF_BAUD   115200  // Common CRSF baud for receivers/handsets

// --- CRC Lookup Table ---
static uint8_t crc8tab[256];

static void crc8_init()
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        }
        crc8tab[i] = crc;
    }
}

static uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc8tab[crc ^ *data++];
    }
    return crc;
}

// --- CRSF Parser State ---
static uint8_t inBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t bufferPtr = 0;

// --- Packet Counter ---
static volatile uint32_t goodPktCount = 0;
static uint32_t lastReportTime = 0;

// UART for CRSF input (UART1 on ESP32)
HardwareSerial CrsfSerial(1);

void setup()
{
    // USB serial for debug output
    Serial.begin(115200);
    Serial.println("CRSF Reader starting...");

    // CRSF serial input — mirrors CRSFHandset::Begin()
    CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    CrsfSerial.setTimeout(0);

    crc8_init();

    lastReportTime = millis();
}

// Align buffer to next sync byte (like CRSFHandset::alignBufferToSync)
static void alignBufferToSync()
{
    uint8_t i = 1;
    while (i < bufferPtr && inBuffer[i] != CRSF_SYNC_BYTE)
    {
        i++;
    }
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

// Read and parse CRSF frames (modeled after CRSFHandset::handleInput)
static void handleInput()
{
    int available = CrsfSerial.available();
    if (available <= 0) return;

            goodPktCount++;
    int toRead = min(available, (int)(CRSF_MAX_PACKET_LEN - bufferPtr));
    if (toRead <= 0)
    {
        bufferPtr = 0;
        return;
    }

    bufferPtr += CrsfSerial.readBytes(&inBuffer[bufferPtr], toRead);

    // Process all complete frames in buffer
    while (bufferPtr >= CRSF_MIN_PACKET_LEN)
    {
        // First byte must be sync
        if (inBuffer[0] != CRSF_SYNC_BYTE)
        {
            alignBufferToSync();
            continue;
        }

        // Need at least sync + len
        if (bufferPtr < 2) break;

        uint8_t frameLen = inBuffer[1]; // type + payload + crc
        uint8_t totalLen = frameLen + 2; // + sync + len bytes

        // Validate length
        if (totalLen < CRSF_MIN_PACKET_LEN || totalLen > CRSF_MAX_PACKET_LEN)
        {
            // Bad length — skip sync byte, realign
            alignBufferToSync();
            continue;
        }

        // Wait for full frame
        if (bufferPtr < totalLen) break;

        // CRC check: calc over type + payload (excludes sync, len, and crc byte)
        uint8_t crc = crc8_calc(&inBuffer[2], totalLen - 3);
        if (crc == inBuffer[totalLen - 1])
        {
            goodPktCount++;
        }

        // Consume frame from buffer
        uint8_t remaining = bufferPtr - totalLen;
        if (remaining > 0)
        {
            memmove(inBuffer, inBuffer + totalLen, remaining);
        }
        bufferPtr = remaining;
    }
}

void loop()
{
    handleInput();
    // Report every second
    uint32_t now = millis();
    if (now - lastReportTime >= 1000)
    {
        uint32_t count = goodPktCount;
        goodPktCount = 0;

        if (count > 0)
        {
            Serial.printf("CRSF: %u packets/sec\n", count);
        }
        else
        {
            Serial.println("CRSF: no signal");
        }

        lastReportTime = now;
    }
}
