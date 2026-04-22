#pragma once
#include <stdint.h>
#include <string.h>

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_SYNC_BYTE_MODULE   0xEE
#define CRSF_MAX_PACKET_LEN     64
#define CRSF_MIN_PACKET_LEN     4
#define CRSF_CRC_POLY           0xD5

// Frame types
#define CRSF_FRAMETYPE_GPS          0x02
#define CRSF_FRAMETYPE_VARIO        0x07
#define CRSF_FRAMETYPE_BATTERY      0x08
#define CRSF_FRAMETYPE_BARO_ALT     0x09
#define CRSF_FRAMETYPE_LINK_STATS   0x14
#define CRSF_FRAMETYPE_RC_CHANNELS  0x16
#define CRSF_FRAMETYPE_ATTITUDE     0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE  0x21
#define CRSF_FRAMETYPE_DEVICE_PING  0x28
#define CRSF_FRAMETYPE_DEVICE_INFO  0x29
#define CRSF_FRAMETYPE_COMMAND      0x32

// Command sub-types
#define CRSF_SUBCOMMAND_CRSF           0x10
#define CRSF_COMMAND_MODEL_SELECT_ID   0x05

// Addresses
#define CRSF_ADDRESS_RADIO          0xEA
#define CRSF_ADDRESS_MODULE         0xEE

// CRC8 lookup table and functions (header-only, one definition per TU via static)
static uint8_t _crsf_crc8_table[256];

static inline void crsfCrc8Init()
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? CRSF_CRC_POLY : 0);
        _crsf_crc8_table[i] = crc;
    }
}

static inline uint8_t crsfCrc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0;
    while (length--)
        crc = _crsf_crc8_table[crc ^ *data++];
    return crc;
}

// Channel decode: 22 bytes packed 11-bit → 16 channels
static inline void crsfDecodeChannels(const uint8_t *p, uint16_t *ch)
{
    ch[0]  = ((uint16_t)p[0]       | (uint16_t)p[1]  << 8) & 0x07FF;
    ch[1]  = ((uint16_t)p[1] >> 3  | (uint16_t)p[2]  << 5) & 0x07FF;
    ch[2]  = ((uint16_t)p[2] >> 6  | (uint16_t)p[3]  << 2 | (uint16_t)p[4] << 10) & 0x07FF;
    ch[3]  = ((uint16_t)p[4] >> 1  | (uint16_t)p[5]  << 7) & 0x07FF;
    ch[4]  = ((uint16_t)p[5] >> 4  | (uint16_t)p[6]  << 4) & 0x07FF;
    ch[5]  = ((uint16_t)p[6] >> 7  | (uint16_t)p[7]  << 1 | (uint16_t)p[8] << 9) & 0x07FF;
    ch[6]  = ((uint16_t)p[8] >> 2  | (uint16_t)p[9]  << 6) & 0x07FF;
    ch[7]  = ((uint16_t)p[9] >> 5  | (uint16_t)p[10] << 3) & 0x07FF;
    ch[8]  = ((uint16_t)p[11]      | (uint16_t)p[12] << 8) & 0x07FF;
    ch[9]  = ((uint16_t)p[12] >> 3 | (uint16_t)p[13] << 5) & 0x07FF;
    ch[10] = ((uint16_t)p[13] >> 6 | (uint16_t)p[14] << 2 | (uint16_t)p[15] << 10) & 0x07FF;
    ch[11] = ((uint16_t)p[15] >> 1 | (uint16_t)p[16] << 7) & 0x07FF;
    ch[12] = ((uint16_t)p[16] >> 4 | (uint16_t)p[17] << 4) & 0x07FF;
    ch[13] = ((uint16_t)p[17] >> 7 | (uint16_t)p[18] << 1 | (uint16_t)p[19] << 9) & 0x07FF;
    ch[14] = ((uint16_t)p[19] >> 2 | (uint16_t)p[20] << 6) & 0x07FF;
    ch[15] = ((uint16_t)p[20] >> 5 | (uint16_t)p[21] << 3) & 0x07FF;
}

// Channel encode: 16 channels → 22 bytes packed 11-bit
static inline void crsfEncodeChannels(const uint16_t *ch, uint8_t *p)
{
    memset(p, 0, 22);
    p[0]  = ch[0] & 0xFF;
    p[1]  = (ch[0] >> 8) | ((ch[1] & 0x1F) << 3);
    p[2]  = (ch[1] >> 5) | ((ch[2] & 0x03) << 6);
    p[3]  = (ch[2] >> 2) & 0xFF;
    p[4]  = (ch[2] >> 10) | ((ch[3] & 0x7F) << 1);
    p[5]  = (ch[3] >> 7) | ((ch[4] & 0x0F) << 4);
    p[6]  = (ch[4] >> 4) | ((ch[5] & 0x01) << 7);
    p[7]  = (ch[5] >> 1) & 0xFF;
    p[8]  = (ch[5] >> 9) | ((ch[6] & 0x3F) << 2);
    p[9]  = (ch[6] >> 6) | ((ch[7] & 0x07) << 5);
    p[10] = (ch[7] >> 3);
    p[11] = ch[8] & 0xFF;
    p[12] = (ch[8] >> 8) | ((ch[9] & 0x1F) << 3);
    p[13] = (ch[9] >> 5) | ((ch[10] & 0x03) << 6);
    p[14] = (ch[10] >> 2) & 0xFF;
    p[15] = (ch[10] >> 10) | ((ch[11] & 0x7F) << 1);
    p[16] = (ch[11] >> 7) | ((ch[12] & 0x0F) << 4);
    p[17] = (ch[12] >> 4) | ((ch[13] & 0x01) << 7);
    p[18] = (ch[13] >> 1) & 0xFF;
    p[19] = (ch[13] >> 9) | ((ch[14] & 0x3F) << 2);
    p[20] = (ch[14] >> 6) | ((ch[15] & 0x07) << 5);
    p[21] = (ch[15] >> 3);
}

static inline bool crsfIsSyncByte(uint8_t b)
{
    return b == CRSF_SYNC_BYTE || b == CRSF_SYNC_BYTE_MODULE;
}
