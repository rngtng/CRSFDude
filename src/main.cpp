#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_ll.h"

// CRSF Protocol
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_SYNC_BYTE_MODULE   0xEE
#define CRSF_MAX_PACKET_LEN     64
#define CRSF_MIN_PACKET_LEN     4
#define CRSF_CRC_POLY           0xD5
#define CRSF_BAUD               420000
#define CRSF_PIN                20

#define CRSF_FRAMETYPE_RC_CHANNELS  0x16
#define CRSF_FRAMETYPE_DEVICE_PING  0x28
#define CRSF_FRAMETYPE_DEVICE_INFO  0x29
#define CRSF_FRAMETYPE_FLIGHT_MODE  0x21

#define CRSF_ADDRESS_RADIO          0xEA
#define CRSF_ADDRESS_MODULE         0xEE

// CRC8 (polynomial 0xD5)
static uint8_t crc8_table[256];

static void crc8_init()
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        crc8_table[i] = crc;
    }
}

static uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
        crc = crc8_table[crc ^ *data++];
    return crc;
}

// Half-duplex: inverted single-wire via GPIO matrix (mLRS pattern)
static void IRAM_ATTR halfDuplexEnableRX()
{
    gpio_set_pull_mode((gpio_num_t)CRSF_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_direction((gpio_num_t)CRSF_PIN, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)CRSF_PIN, U1RXD_IN_IDX, true);
}

static void IRAM_ATTR halfDuplexEnableTX()
{
    uart_set_pin(UART_NUM_1, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_matrix_out((gpio_num_t)CRSF_PIN, U1TXD_OUT_IDX, true, false);
}

// FreeRTOS task: wait for TX complete, then restore RX mode
static TaskHandle_t txDoneTaskHandle = nullptr;

static void txDoneTask(void *param)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20));
        gpio_reset_pin((gpio_num_t)CRSF_PIN);
        halfDuplexEnableRX();
        uart_ll_rxfifo_rst(UART_LL_GET_HW(1));
    }
}

// UART + parser state
HardwareSerial CrsfSerial(1);
static uint8_t parseBuffer[CRSF_MAX_PACKET_LEN];
static uint8_t parseBufferLen = 0;
static uint16_t channels[4];
static uint32_t rxPacketCount = 0;
static uint32_t txPacketCount = 0;
static uint32_t lastReportTime = 0;

static inline bool isSyncByte(uint8_t b)
{
    return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_MODULE;
}

static void crsfSend(const uint8_t *buffer, uint8_t length)
{
    halfDuplexEnableTX();
    uart_write_bytes(UART_NUM_1, (const char *)buffer, length);
    xTaskNotifyGive(txDoneTaskHandle);
    parseBufferLen = 0;
    txPacketCount++;
}

// EdgeTX handshake: respond to Device Ping with Device Info
static void sendDeviceInfo()
{
    const char deviceName[] = "CRSFDude";
    uint8_t nameLength = sizeof(deviceName);
    uint8_t frame[40];
    uint8_t pos = 0;

    frame[pos++] = CRSF_SYNC_BYTE;
    uint8_t lengthIndex = pos++;
    frame[pos++] = CRSF_FRAMETYPE_DEVICE_INFO;
    frame[pos++] = CRSF_ADDRESS_RADIO;
    frame[pos++] = CRSF_ADDRESS_MODULE;
    memcpy(&frame[pos], deviceName, nameLength); pos += nameLength;
    memset(&frame[pos], 0, 12); pos += 12;  // serial + hardware + firmware IDs
    frame[pos++] = 0;  // parameter count
    frame[pos++] = 0;  // protocol version

    frame[lengthIndex] = pos - 1;
    frame[pos] = crc8_calc(&frame[2], pos - 2); pos++;
    crsfSend(frame, pos);
}

static void sendFlightMode()
{
    static uint32_t lastToggleTime = 0;
    static bool signalOn = false;
    if (millis() - lastToggleTime >= 2000) { signalOn = !signalOn; lastToggleTime = millis(); }

    const char *mode = signalOn ? "ON" : "OFF";
    uint8_t stringLength = strlen(mode) + 1;
    uint8_t frame[16];
    uint8_t pos = 0;

    frame[pos++] = CRSF_SYNC_BYTE;
    frame[pos++] = 1 + stringLength + 1;  // type + string + crc
    frame[pos++] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&frame[pos], mode, stringLength); pos += stringLength;
    frame[pos] = crc8_calc(&frame[2], pos - 2); pos++;
    crsfSend(frame, pos);
}

static void decodeChannels(const uint8_t *payload)
{
    channels[0] = ((uint16_t)payload[0]      | (uint16_t)payload[1] << 8) & 0x07FF;
    channels[1] = ((uint16_t)payload[1] >> 3 | (uint16_t)payload[2] << 5) & 0x07FF;
    channels[2] = ((uint16_t)payload[2] >> 6 | (uint16_t)payload[3] << 2 | (uint16_t)payload[4] << 10) & 0x07FF;
    channels[3] = ((uint16_t)payload[4] >> 1 | (uint16_t)payload[5] << 7) & 0x07FF;
}

static void alignBufferToSync()
{
    uint8_t i = 1;
    while (i < parseBufferLen && !isSyncByte(parseBuffer[i])) i++;
    if (i < parseBufferLen) { parseBufferLen -= i; memmove(parseBuffer, parseBuffer + i, parseBufferLen); }
    else parseBufferLen = 0;
}

static void handleInput()
{
    int available = CrsfSerial.available();
    if (available <= 0) return;

    int toRead = min(available, (int)(CRSF_MAX_PACKET_LEN - parseBufferLen));
    if (toRead <= 0) { parseBufferLen = 0; return; }
    parseBufferLen += CrsfSerial.readBytes(&parseBuffer[parseBufferLen], toRead);

    while (parseBufferLen >= CRSF_MIN_PACKET_LEN)
    {
        if (!isSyncByte(parseBuffer[0])) { alignBufferToSync(); continue; }
        if (parseBufferLen < 2) break;

        uint8_t totalLength = parseBuffer[1] + 2;
        if (totalLength < CRSF_MIN_PACKET_LEN || totalLength > CRSF_MAX_PACKET_LEN) { alignBufferToSync(); continue; }
        if (parseBufferLen < totalLength) break;

        if (crc8_calc(&parseBuffer[2], totalLength - 3) == parseBuffer[totalLength - 1])
        {
            uint8_t frameType = parseBuffer[2];

            if (frameType == CRSF_FRAMETYPE_RC_CHANNELS && totalLength >= 26) {
                rxPacketCount++;
                decodeChannels(&parseBuffer[3]);
                if (rxPacketCount % 5 == 0) { sendFlightMode(); return; }
            } else if (frameType == CRSF_FRAMETYPE_DEVICE_PING) {
                sendDeviceInfo();
                return;
            }
        }

        uint8_t remaining = parseBufferLen - totalLength;
        if (remaining > 0) memmove(parseBuffer, parseBuffer + totalLength, remaining);
        parseBufferLen = remaining;
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
    xTaskCreatePinnedToCore(txDoneTask, "TxDone", 2048, NULL, 5, &txDoneTaskHandle, 0);
    halfDuplexEnableRX();

    lastReportTime = millis();
}

void loop()
{
    handleInput();

    uint32_t now = millis();
    if (now - lastReportTime >= 1000) {
        Serial.printf("CH1: %4u  CH2: %4u  CH3: %4u  CH4: %4u  [rx:%u tx:%u /s]\n",
                      channels[0], channels[1], channels[2], channels[3],
                      rxPacketCount, txPacketCount);
        rxPacketCount = txPacketCount = 0;
        lastReportTime = now;
    }
}
