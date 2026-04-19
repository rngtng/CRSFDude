#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_ll.h"

// CRSF protocol
#define CRSF_SYNC        0xC8
#define CRSF_MAX_LEN     64
#define CRSF_MIN_LEN     4
#define CRSF_CRC_POLY    0xD5
#define CRSF_BAUD        420000
#define CRSF_PIN         20
#define CRSF_TYPE_RC     0x16
#define CRSF_TYPE_PING   0x28
#define CRSF_TYPE_INFO   0x29
#define CRSF_TYPE_FM     0x21

// CRC8 lookup table
static uint8_t crc8tab[256];

static void crc8_init()
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        crc8tab[i] = crc;
    }
}

static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
        crc = crc8tab[crc ^ *data++];
    return crc;
}

// Half-duplex GPIO matrix switching (inverted signal, mLRS pattern)
static void IRAM_ATTR rxEnable()
{
    gpio_set_pull_mode((gpio_num_t)CRSF_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_direction((gpio_num_t)CRSF_PIN, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)CRSF_PIN, U1RXD_IN_IDX, true);
}

static void IRAM_ATTR txEnable()
{
    uart_set_pin(UART_NUM_1, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_matrix_out((gpio_num_t)CRSF_PIN, U1TXD_OUT_IDX, true, false);
}

// TX done task: wait for transmit complete, then restore RX
static TaskHandle_t txTaskHandle = nullptr;

static void txDoneTask(void *param)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20));
        gpio_reset_pin((gpio_num_t)CRSF_PIN);
        rxEnable();
        uart_ll_rxfifo_rst(UART_LL_GET_HW(1));
    }
}

// UART + parser state
HardwareSerial CrsfSerial(1);
static uint8_t inBuf[CRSF_MAX_LEN];
static uint8_t bufPtr = 0;
static uint16_t channels[4];
static uint32_t rxCount = 0;
static uint32_t txCount = 0;
static uint32_t lastReport = 0;

static inline bool isSync(uint8_t b) { return b == CRSF_SYNC || b == 0xEE; }

static void crsfSend(const uint8_t *buf, uint8_t len)
{
    txEnable();
    uart_write_bytes(UART_NUM_1, (const char *)buf, len);
    xTaskNotifyGive(txTaskHandle);
    bufPtr = 0;
    txCount++;
}

// EdgeTX handshake: respond to Device Ping (0x28) with Device Info (0x29)
static void sendDeviceInfo()
{
    const char name[] = "CRSFDude";
    uint8_t nlen = sizeof(name);
    uint8_t buf[40], i = 0;
    buf[i++] = CRSF_SYNC;
    uint8_t li = i++;          // length placeholder
    buf[i++] = CRSF_TYPE_INFO; // type
    buf[i++] = 0xEA;          // dest: radio
    buf[i++] = 0xEE;          // origin: module
    memcpy(&buf[i], name, nlen); i += nlen;
    for (int k = 0; k < 12; k++) buf[i++] = 0; // serial + hw + fw (4+4+4)
    buf[i++] = 0; buf[i++] = 0; // params + proto version
    buf[li] = i - 1;
    buf[i] = crc8(&buf[2], i - 2); i++;
    crsfSend(buf, i);
}

static void sendFlightMode()
{
    static uint32_t lastToggle = 0;
    static bool on = false;
    if (millis() - lastToggle >= 2000) { on = !on; lastToggle = millis(); }

    const char *mode = on ? "ON" : "OFF";
    uint8_t slen = strlen(mode) + 1;
    uint8_t buf[16], i = 0;
    buf[i++] = CRSF_SYNC;
    buf[i++] = 1 + slen + 1;  // type + string + crc
    buf[i++] = CRSF_TYPE_FM;
    memcpy(&buf[i], mode, slen); i += slen;
    buf[i] = crc8(&buf[2], i - 2); i++;
    crsfSend(buf, i);
}

static void decodeChannels(const uint8_t *p)
{
    channels[0] = ((uint16_t)p[0]      | (uint16_t)p[1] << 8) & 0x07FF;
    channels[1] = ((uint16_t)p[1] >> 3 | (uint16_t)p[2] << 5) & 0x07FF;
    channels[2] = ((uint16_t)p[2] >> 6 | (uint16_t)p[3] << 2 | (uint16_t)p[4] << 10) & 0x07FF;
    channels[3] = ((uint16_t)p[4] >> 1 | (uint16_t)p[5] << 7) & 0x07FF;
}

static void alignToSync()
{
    uint8_t i = 1;
    while (i < bufPtr && !isSync(inBuf[i])) i++;
    if (i < bufPtr) { bufPtr -= i; memmove(inBuf, inBuf + i, bufPtr); }
    else bufPtr = 0;
}

static void handleInput()
{
    int avail = CrsfSerial.available();
    if (avail <= 0) return;

    int toRead = min(avail, (int)(CRSF_MAX_LEN - bufPtr));
    if (toRead <= 0) { bufPtr = 0; return; }
    bufPtr += CrsfSerial.readBytes(&inBuf[bufPtr], toRead);

    while (bufPtr >= CRSF_MIN_LEN)
    {
        if (!isSync(inBuf[0])) { alignToSync(); continue; }
        if (bufPtr < 2) break;

        uint8_t totalLen = inBuf[1] + 2;
        if (totalLen < CRSF_MIN_LEN || totalLen > CRSF_MAX_LEN) { alignToSync(); continue; }
        if (bufPtr < totalLen) break;

        if (crc8(&inBuf[2], totalLen - 3) == inBuf[totalLen - 1])
        {
            uint8_t type = inBuf[2];
            if (type == CRSF_TYPE_RC && totalLen >= 26) {
                rxCount++;
                decodeChannels(&inBuf[3]);
                if (rxCount % 5 == 0) { sendFlightMode(); return; }
            } else if (type == CRSF_TYPE_PING) {
                sendDeviceInfo();
                return;
            }
        }

        uint8_t rem = bufPtr - totalLen;
        if (rem > 0) memmove(inBuf, inBuf + totalLen, rem);
        bufPtr = rem;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(1000);
    Serial.println("CRSFDude starting...");

    crc8_init();
    CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_PIN, CRSF_PIN);
    CrsfSerial.setTimeout(0);
    xTaskCreatePinnedToCore(txDoneTask, "TxDone", 2048, NULL, 5, &txTaskHandle, 0);
    rxEnable();

    lastReport = millis();
}

void loop()
{
    handleInput();

    uint32_t now = millis();
    if (now - lastReport >= 1000) {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [rx:%u tx:%u /s]\n",
                      channels[0], channels[1], channels[2], channels[3], rxCount, txCount);
        rxCount = txCount = 0;
        lastReport = now;
    }
}
