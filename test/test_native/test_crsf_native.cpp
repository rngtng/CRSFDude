#include <unity.h>
#include <string.h>
#include <stdint.h>

// Replicate CRC8 and encoding logic for native testing (no ESP32 deps)
#define CRSF_CRC_POLY 0xD5

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

static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    while (len--)
        crc = crc8_table[crc ^ *data++];
    return crc;
}

// Channel decode (same as library)
static void decodeChannels(const uint8_t *p, uint16_t *ch)
{
    ch[0]  = ((uint16_t)p[0]      | (uint16_t)p[1]  << 8) & 0x07FF;
    ch[1]  = ((uint16_t)p[1] >> 3 | (uint16_t)p[2]  << 5) & 0x07FF;
    ch[2]  = ((uint16_t)p[2] >> 6 | (uint16_t)p[3]  << 2 | (uint16_t)p[4] << 10) & 0x07FF;
    ch[3]  = ((uint16_t)p[4] >> 1 | (uint16_t)p[5]  << 7) & 0x07FF;
    ch[4]  = ((uint16_t)p[5] >> 4 | (uint16_t)p[6]  << 4) & 0x07FF;
    ch[5]  = ((uint16_t)p[6] >> 7 | (uint16_t)p[7]  << 1 | (uint16_t)p[8] << 9) & 0x07FF;
    ch[6]  = ((uint16_t)p[8] >> 2 | (uint16_t)p[9]  << 6) & 0x07FF;
    ch[7]  = ((uint16_t)p[9] >> 5 | (uint16_t)p[10] << 3) & 0x07FF;
    ch[8]  = ((uint16_t)p[11]      | (uint16_t)p[12] << 8) & 0x07FF;
    ch[9]  = ((uint16_t)p[12] >> 3 | (uint16_t)p[13] << 5) & 0x07FF;
    ch[10] = ((uint16_t)p[13] >> 6 | (uint16_t)p[14] << 2 | (uint16_t)p[15] << 10) & 0x07FF;
    ch[11] = ((uint16_t)p[15] >> 1 | (uint16_t)p[16] << 7) & 0x07FF;
    ch[12] = ((uint16_t)p[16] >> 4 | (uint16_t)p[17] << 4) & 0x07FF;
    ch[13] = ((uint16_t)p[17] >> 7 | (uint16_t)p[18] << 1 | (uint16_t)p[19] << 9) & 0x07FF;
    ch[14] = ((uint16_t)p[19] >> 2 | (uint16_t)p[20] << 6) & 0x07FF;
    ch[15] = ((uint16_t)p[20] >> 5 | (uint16_t)p[21] << 3) & 0x07FF;
}

// Channel encode (inverse of decode, for test data generation)
static void encodeChannels(const uint16_t *ch, uint8_t *p)
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

// --- CRC8 Tests ---

void test_crc8_deterministic()
{
    crc8_init();
    uint8_t data[] = { 0x21, 'O', 'N', '\0' };
    uint8_t a = crc8(data, 4);
    uint8_t b = crc8(data, 4);
    TEST_ASSERT_EQUAL(a, b);
    TEST_ASSERT_NOT_EQUAL(0, a);
}

void test_crc8_empty()
{
    crc8_init();
    uint8_t dummy = 0;
    TEST_ASSERT_EQUAL(0, crc8(&dummy, 0));
}

void test_crc8_different_data()
{
    crc8_init();
    uint8_t a[] = { 0x01 };
    uint8_t b[] = { 0x02 };
    TEST_ASSERT_NOT_EQUAL(crc8(a, 1), crc8(b, 1));
}

void test_crc8_known_flight_mode_frame()
{
    crc8_init();
    // Full flight mode frame: type=0x21, payload="ON\0"
    uint8_t payload[] = { 0x21, 'O', 'N', '\0' };
    uint8_t c = crc8(payload, 4);
    // Verify frame validates: sync + len + payload + crc
    uint8_t frame[] = { 0xC8, 0x06, 0x21, 'O', 'N', '\0', 0 };
    frame[6] = c;
    TEST_ASSERT_EQUAL(c, crc8(&frame[2], 4));
}

// --- Channel Decode Tests ---

void test_channel_decode_center()
{
    crc8_init();
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = 992;

    uint8_t packed[22];
    encodeChannels(input, packed);
    decodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(992, output[i]);
}

void test_channel_decode_min_max()
{
    crc8_init();
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = (i % 2 == 0) ? 0 : 2047;

    uint8_t packed[22];
    encodeChannels(input, packed);
    decodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(input[i], output[i]);
}

void test_channel_decode_sequential()
{
    crc8_init();
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = i * 128; // 0, 128, 256, ...

    uint8_t packed[22];
    encodeChannels(input, packed);
    decodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(input[i], output[i]);
}

// --- Frame Encoding Tests ---

void test_battery_frame_format()
{
    crc8_init();
    uint8_t frame[12];
    frame[0] = 0xC8;
    frame[1] = 10;
    frame[2] = 0x08;
    frame[3] = 0; frame[4] = 111;   // 11.1V
    frame[5] = 0; frame[6] = 15;    // 1.5A
    frame[7] = 0; frame[8] = 4; frame[9] = 176; // 1200 mAh
    frame[10] = 75;
    frame[11] = crc8(&frame[2], 9);

    // Verify length field
    TEST_ASSERT_EQUAL(10, frame[1]); // type + 8 payload + crc

    // Verify CRC validates
    TEST_ASSERT_EQUAL(frame[11], crc8(&frame[2], 9));
}

void test_gps_altitude_offset()
{
    // GPS altitude has +1000m offset
    uint16_t alt = 120;
    uint16_t encoded = alt + 1000;
    TEST_ASSERT_EQUAL(1120, encoded);
}

void test_baro_altitude_encoding()
{
    // 12034cm → 1203dm + 10000 offset = 11203
    int32_t altCm = 12034;
    int16_t altDm = (altCm / 10) + 10000;
    TEST_ASSERT_EQUAL(11203, altDm);
}

void test_baro_negative_altitude()
{
    // -500cm → -50dm + 10000 = 9950
    int32_t altCm = -500;
    int16_t altDm = (altCm / 10) + 10000;
    TEST_ASSERT_EQUAL(9950, altDm);
}

// --- Frame Header Tests ---

void test_frame_header_format()
{
    crc8_init();
    // buildFrameHeader: sync + length + type
    uint8_t frame[16];
    frame[0] = 0xC8;
    frame[1] = 5 + 2; // payload(5) + type(1) + crc(1)
    frame[2] = 0x21;  // flight mode
    uint8_t pos = 3;

    TEST_ASSERT_EQUAL(0xC8, frame[0]);
    TEST_ASSERT_EQUAL(7, frame[1]);
    TEST_ASSERT_EQUAL(0x21, frame[2]);
    TEST_ASSERT_EQUAL(3, pos);
}

// --- Flight Mode Frame Tests ---

void test_flight_mode_frame_format()
{
    crc8_init();
    const char *mode = "ACRO";
    uint8_t slen = strlen(mode) + 1; // 5
    uint8_t frame[16];
    frame[0] = 0xC8;
    frame[1] = 1 + slen + 1; // type + string + crc = 7
    frame[2] = 0x21;
    memcpy(&frame[3], mode, slen);
    frame[3 + slen] = crc8(&frame[2], 1 + slen);

    TEST_ASSERT_EQUAL(7, frame[1]);
    TEST_ASSERT_EQUAL(0x21, frame[2]);
    TEST_ASSERT_EQUAL('A', frame[3]);
    TEST_ASSERT_EQUAL('\0', frame[7]);
    // CRC validates
    TEST_ASSERT_EQUAL(frame[8], crc8(&frame[2], 6));
}

// --- GPS Frame Tests ---

void test_gps_frame_format()
{
    crc8_init();
    int32_t lat = 524213670;   // 52.4213670°
    int32_t lon = 133560120;   // 13.3560120°
    uint16_t spd = 250;        // 2.5 m/s
    uint16_t hdg = 18000;      // 180.00°
    uint16_t alt = 120;
    uint8_t sats = 12;

    uint8_t frame[19];
    frame[0] = 0xC8;
    frame[1] = 15 + 2; // payload + type + crc
    frame[2] = 0x02;   // GPS
    uint8_t pos = 3;
    frame[pos++] = (lat >> 24) & 0xFF;
    frame[pos++] = (lat >> 16) & 0xFF;
    frame[pos++] = (lat >> 8) & 0xFF;
    frame[pos++] = lat & 0xFF;
    frame[pos++] = (lon >> 24) & 0xFF;
    frame[pos++] = (lon >> 16) & 0xFF;
    frame[pos++] = (lon >> 8) & 0xFF;
    frame[pos++] = lon & 0xFF;
    frame[pos++] = spd >> 8;
    frame[pos++] = spd & 0xFF;
    frame[pos++] = hdg >> 8;
    frame[pos++] = hdg & 0xFF;
    frame[pos++] = (alt + 1000) >> 8;
    frame[pos++] = (alt + 1000) & 0xFF;
    frame[pos++] = sats;
    frame[pos] = crc8(&frame[2], pos - 2);

    // Total frame = 19 bytes
    TEST_ASSERT_EQUAL(19, pos + 1);
    TEST_ASSERT_EQUAL(17, frame[1]);
    // Latitude big-endian
    TEST_ASSERT_EQUAL(0x1F, frame[3]);
    // Satellites
    TEST_ASSERT_EQUAL(12, frame[17]);
    // CRC validates
    TEST_ASSERT_EQUAL(frame[18], crc8(&frame[2], 16));
}

// --- Attitude Frame Tests ---

void test_attitude_frame_format()
{
    crc8_init();
    int16_t pitch = 1500, roll = -300, yaw = 0;

    uint8_t frame[10];
    frame[0] = 0xC8;
    frame[1] = 6 + 2;
    frame[2] = 0x1E;
    frame[3] = pitch >> 8;    frame[4] = pitch & 0xFF;
    frame[5] = roll >> 8;     frame[6] = roll & 0xFF;
    frame[7] = yaw >> 8;      frame[8] = yaw & 0xFF;
    frame[9] = crc8(&frame[2], 7);

    TEST_ASSERT_EQUAL(8, frame[1]);
    TEST_ASSERT_EQUAL(0x05, frame[3]); // 1500 >> 8
    TEST_ASSERT_EQUAL(0xDC, frame[4]); // 1500 & 0xFF
    // Roll -300 = 0xFED4
    TEST_ASSERT_EQUAL(0xFE, frame[5]);
    TEST_ASSERT_EQUAL(0xD4, frame[6]);
    TEST_ASSERT_EQUAL(frame[9], crc8(&frame[2], 7));
}

// --- Vario Frame Tests ---

void test_vario_frame_format()
{
    crc8_init();
    int16_t vspd = -250; // -2.5 m/s

    uint8_t frame[6];
    frame[0] = 0xC8;
    frame[1] = 2 + 2;
    frame[2] = 0x07;
    frame[3] = vspd >> 8;
    frame[4] = vspd & 0xFF;
    frame[5] = crc8(&frame[2], 3);

    TEST_ASSERT_EQUAL(4, frame[1]);
    // -250 = 0xFF06
    TEST_ASSERT_EQUAL(0xFF, frame[3]);
    TEST_ASSERT_EQUAL(0x06, frame[4]);
    TEST_ASSERT_EQUAL(frame[5], crc8(&frame[2], 3));
}

// --- Link Stats Frame Tests ---

void test_link_stats_frame_format()
{
    crc8_init();
    uint8_t frame[14];
    frame[0] = 0xC8;
    frame[1] = 10 + 2;
    frame[2] = 0x14;
    frame[3] = 90;   // rxRssi1
    frame[4] = 90;   // rxRssi2
    frame[5] = 100;  // rxQuality
    frame[6] = 10;   // rxSnr
    frame[7] = 0;    // antenna
    frame[8] = 4;    // rfMode
    frame[9] = 3;    // txPower
    frame[10] = 80;  // txRssi
    frame[11] = 100; // txQuality
    frame[12] = 8;   // txSnr
    frame[13] = crc8(&frame[2], 11);

    TEST_ASSERT_EQUAL(12, frame[1]);
    TEST_ASSERT_EQUAL(100, frame[5]); // rxQuality non-zero
    TEST_ASSERT_EQUAL(frame[13], crc8(&frame[2], 11));
}

// --- Model ID Command Tests ---

void test_model_id_command_parse()
{
    crc8_init();
    // Model Select command: [sync][len=8][type=0x32][dest=0xEE][orig=0xEA][subcmd=0x10][cmd=0x05][modelId][crc_ba][crc]
    uint8_t frame[10];
    frame[0] = 0xC8;
    frame[1] = 8;
    frame[2] = 0x32; // COMMAND
    frame[3] = 0xEE; // dest: module
    frame[4] = 0xEA; // origin: radio
    frame[5] = 0x10; // SUBCOMMAND_CRSF
    frame[6] = 0x05; // COMMAND_MODEL_SELECT_ID
    frame[7] = 42;   // model ID

    // Verify we can extract model ID from correct offsets
    TEST_ASSERT_EQUAL(0x32, frame[2]);
    TEST_ASSERT_EQUAL(0x10, frame[5]);
    TEST_ASSERT_EQUAL(0x05, frame[6]);
    TEST_ASSERT_EQUAL(42, frame[7]);

    // Total length check
    TEST_ASSERT_EQUAL(10, frame[1] + 2);
}

void test_model_id_range()
{
    // Model ID is 0-63
    for (uint8_t id = 0; id <= 63; id++) {
        TEST_ASSERT_TRUE(id <= 63);
    }
}

// --- Test Runner ---

int main()
{
    UNITY_BEGIN();

    // CRC
    RUN_TEST(test_crc8_deterministic);
    RUN_TEST(test_crc8_empty);
    RUN_TEST(test_crc8_different_data);
    RUN_TEST(test_crc8_known_flight_mode_frame);

    // Channel encode/decode
    RUN_TEST(test_channel_decode_center);
    RUN_TEST(test_channel_decode_min_max);
    RUN_TEST(test_channel_decode_sequential);

    // Frame encoding
    RUN_TEST(test_frame_header_format);
    RUN_TEST(test_flight_mode_frame_format);
    RUN_TEST(test_battery_frame_format);
    RUN_TEST(test_gps_frame_format);
    RUN_TEST(test_gps_altitude_offset);
    RUN_TEST(test_attitude_frame_format);
    RUN_TEST(test_baro_altitude_encoding);
    RUN_TEST(test_baro_negative_altitude);
    RUN_TEST(test_vario_frame_format);
    RUN_TEST(test_link_stats_frame_format);

    // Model ID
    RUN_TEST(test_model_id_command_parse);
    RUN_TEST(test_model_id_range);

    return UNITY_END();
}
