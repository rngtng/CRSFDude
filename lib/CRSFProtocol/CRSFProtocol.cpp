#include "CRSFProtocol.h"
#include <string.h>

// Static members
uint8_t CRSFProtocol::crc8_table[256];
bool CRSFProtocol::crc8_initialized = false;
TaskHandle_t CRSFProtocol::_txDoneTaskHandle = nullptr;
uint8_t CRSFProtocol::_txDonePin = 0;

// CRC8 with polynomial 0xD5
void CRSFProtocol::crc8_init()
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        crc8_table[i] = crc;
    }
    crc8_initialized = true;
}

uint8_t CRSFProtocol::crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0;
    while (length--)
        crc = crc8_table[crc ^ *data++];
    return crc;
}

// Half-duplex: inverted single-wire via GPIO matrix (mLRS pattern)
void IRAM_ATTR CRSFProtocol::halfDuplexEnableRX(uint8_t pin)
{
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    gpio_matrix_in((gpio_num_t)pin, U1RXD_IN_IDX, true);
}

void IRAM_ATTR CRSFProtocol::halfDuplexEnableTX(uint8_t pin)
{
    uart_set_pin(UART_NUM_1, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_matrix_out((gpio_num_t)pin, U1TXD_OUT_IDX, true, false);
}

// FreeRTOS task: wait for TX complete, then restore RX
void CRSFProtocol::txDoneTask(void *param)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20));
        gpio_reset_pin((gpio_num_t)_txDonePin);
        halfDuplexEnableRX(_txDonePin);
        uart_ll_rxfifo_rst(UART_LL_GET_HW(1));
    }
}

void CRSFProtocol::begin(uint8_t pin, uint32_t baudRate)
{
    _pin = pin;
    _txDonePin = pin;

    if (!crc8_initialized) crc8_init();

    _serial.begin(baudRate, SERIAL_8N1, pin, pin);
    _serial.setTimeout(0);

    xTaskCreatePinnedToCore(txDoneTask, "CRSFTxDone", 2048, NULL, 5, &_txDoneTaskHandle, 0);
    halfDuplexEnableRX(pin);
}

bool CRSFProtocol::update()
{
    int available = _serial.available();
    if (available <= 0) return false;

    int toRead = min(available, (int)(CRSF_MAX_PACKET_LEN - _parseBufferLen));
    if (toRead <= 0) { _parseBufferLen = 0; return false; }
    _parseBufferLen += _serial.readBytes(&_parseBuffer[_parseBufferLen], toRead);

    bool gotChannels = false;

    while (_parseBufferLen >= CRSF_MIN_PACKET_LEN)
    {
        if (!isSyncByte(_parseBuffer[0])) { alignBufferToSync(); continue; }
        if (_parseBufferLen < 2) break;

        uint8_t totalLength = _parseBuffer[1] + 2;
        if (totalLength < CRSF_MIN_PACKET_LEN || totalLength > CRSF_MAX_PACKET_LEN) {
            alignBufferToSync();
            continue;
        }
        if (_parseBufferLen < totalLength) break;

        if (crc8(&_parseBuffer[2], totalLength - 3) == _parseBuffer[totalLength - 1]) {
            uint8_t frameType = _parseBuffer[2];
            processFrame(frameType, totalLength);
            if (frameType == CRSF_FRAMETYPE_RC_CHANNELS) gotChannels = true;
        }

        // sendFrame() inside processFrame() zeroes _parseBufferLen, so
        // use totalLength against the pre-call length to advance safely.
        if (_parseBufferLen >= totalLength) {
            uint8_t remaining = _parseBufferLen - totalLength;
            if (remaining > 0) memmove(_parseBuffer, _parseBuffer + totalLength, remaining);
            _parseBufferLen = remaining;
        } else {
            _parseBufferLen = 0;
        }
    }

    return gotChannels;
}

void CRSFProtocol::processFrame(uint8_t frameType, uint8_t totalLength)
{
    if (frameType == CRSF_FRAMETYPE_RC_CHANNELS && totalLength >= 26) {
        rxPacketCount++;
        const uint8_t *p = &_parseBuffer[3];
        _channels[0]  = ((uint16_t)p[0]      | (uint16_t)p[1]  << 8) & 0x07FF;
        _channels[1]  = ((uint16_t)p[1] >> 3  | (uint16_t)p[2]  << 5) & 0x07FF;
        _channels[2]  = ((uint16_t)p[2] >> 6  | (uint16_t)p[3]  << 2 | (uint16_t)p[4] << 10) & 0x07FF;
        _channels[3]  = ((uint16_t)p[4] >> 1  | (uint16_t)p[5]  << 7) & 0x07FF;
        _channels[4]  = ((uint16_t)p[5] >> 4  | (uint16_t)p[6]  << 4) & 0x07FF;
        _channels[5]  = ((uint16_t)p[6] >> 7  | (uint16_t)p[7]  << 1 | (uint16_t)p[8] << 9) & 0x07FF;
        _channels[6]  = ((uint16_t)p[8] >> 2  | (uint16_t)p[9]  << 6) & 0x07FF;
        _channels[7]  = ((uint16_t)p[9] >> 5  | (uint16_t)p[10] << 3) & 0x07FF;
        _channels[8]  = ((uint16_t)p[11]      | (uint16_t)p[12] << 8) & 0x07FF;
        _channels[9]  = ((uint16_t)p[12] >> 3 | (uint16_t)p[13] << 5) & 0x07FF;
        _channels[10] = ((uint16_t)p[13] >> 6 | (uint16_t)p[14] << 2 | (uint16_t)p[15] << 10) & 0x07FF;
        _channels[11] = ((uint16_t)p[15] >> 1 | (uint16_t)p[16] << 7) & 0x07FF;
        _channels[12] = ((uint16_t)p[16] >> 4 | (uint16_t)p[17] << 4) & 0x07FF;
        _channels[13] = ((uint16_t)p[17] >> 7 | (uint16_t)p[18] << 1 | (uint16_t)p[19] << 9) & 0x07FF;
        _channels[14] = ((uint16_t)p[19] >> 2 | (uint16_t)p[20] << 6) & 0x07FF;
        _channels[15] = ((uint16_t)p[20] >> 5 | (uint16_t)p[21] << 3) & 0x07FF;
    }
    else if (frameType == CRSF_FRAMETYPE_DEVICE_PING) {
        if (onDevicePing) {
            onDevicePing();
        } else {
            sendDeviceInfo(_deviceName);
        }
    }
}

uint16_t CRSFProtocol::getChannel(uint8_t ch) const
{
    return (ch < 16) ? _channels[ch] : 0;
}

void CRSFProtocol::sendFrame(const uint8_t *frame, uint8_t length)
{
    halfDuplexEnableTX(_pin);
    uart_write_bytes(UART_NUM_1, (const char *)frame, length);
    xTaskNotifyGive(_txDoneTaskHandle);
    _parseBufferLen = 0;
    txPacketCount++;
}

void CRSFProtocol::sendDeviceInfo(const char *deviceName)
{
    _deviceName = deviceName;
    uint8_t nameLength = strlen(deviceName) + 1;
    uint8_t frame[48];
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
    frame[pos] = crc8(&frame[2], pos - 2); pos++;
    sendFrame(frame, pos);
}

void CRSFProtocol::sendFlightMode(const char *mode)
{
    uint8_t stringLength = strlen(mode) + 1;
    uint8_t frame[32];
    uint8_t pos = 0;

    frame[pos++] = CRSF_SYNC_BYTE;
    frame[pos++] = 1 + stringLength + 1;
    frame[pos++] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&frame[pos], mode, stringLength); pos += stringLength;
    frame[pos] = crc8(&frame[2], pos - 2); pos++;
    sendFrame(frame, pos);
}

void CRSFProtocol::alignBufferToSync()
{
    uint8_t i = 1;
    while (i < _parseBufferLen && !isSyncByte(_parseBuffer[i])) i++;
    if (i < _parseBufferLen) { _parseBufferLen -= i; memmove(_parseBuffer, _parseBuffer + i, _parseBufferLen); }
    else _parseBufferLen = 0;
}
