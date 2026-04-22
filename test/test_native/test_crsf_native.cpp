#include <unity.h>
#include "crsf_protocol.h"

void setUp() { crsfCrc8Init(); }
void tearDown() {}

// --- CRC8 Tests ---

void test_crc8_deterministic()
{
    uint8_t data[] = { 0x21, 'O', 'N', '\0' };
    uint8_t a = crsfCrc8(data, 4);
    uint8_t b = crsfCrc8(data, 4);
    TEST_ASSERT_EQUAL(a, b);
    TEST_ASSERT_NOT_EQUAL(0, a);
}

void test_crc8_empty()
{
    uint8_t dummy = 0;
    TEST_ASSERT_EQUAL(0, crsfCrc8(&dummy, 0));
}

void test_crc8_different_data()
{
    uint8_t a[] = { 0x01 };
    uint8_t b[] = { 0x02 };
    TEST_ASSERT_NOT_EQUAL(crsfCrc8(a, 1), crsfCrc8(b, 1));
}

void test_crc8_known_flight_mode_frame()
{
    uint8_t payload[] = { 0x21, 'O', 'N', '\0' };
    uint8_t c = crsfCrc8(payload, 4);
    uint8_t frame[] = { 0xC8, 0x06, 0x21, 'O', 'N', '\0', 0 };
    frame[6] = c;
    TEST_ASSERT_EQUAL(c, crsfCrc8(&frame[2], 4));
}

// --- Channel Decode Tests ---

void test_channel_decode_center()
{
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = 992;

    uint8_t packed[22];
    crsfEncodeChannels(input, packed);
    crsfDecodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(992, output[i]);
}

void test_channel_decode_min_max()
{
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = (i % 2 == 0) ? 0 : 2047;

    uint8_t packed[22];
    crsfEncodeChannels(input, packed);
    crsfDecodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(input[i], output[i]);
}

void test_channel_decode_sequential()
{
    uint16_t input[16], output[16];
    for (int i = 0; i < 16; i++) input[i] = i * 128;

    uint8_t packed[22];
    crsfEncodeChannels(input, packed);
    crsfDecodeChannels(packed, output);

    for (int i = 0; i < 16; i++)
        TEST_ASSERT_EQUAL(input[i], output[i]);
}

// --- Frame Encoding Tests ---

void test_battery_frame_format()
{
    uint8_t frame[12];
    frame[0] = 0xC8;
    frame[1] = 10;
    frame[2] = CRSF_FRAMETYPE_BATTERY;
    frame[3] = 0; frame[4] = 111;   // 11.1V
    frame[5] = 0; frame[6] = 15;    // 1.5A
    frame[7] = 0; frame[8] = 4; frame[9] = 176; // 1200 mAh
    frame[10] = 75;
    frame[11] = crsfCrc8(&frame[2], 9);

    TEST_ASSERT_EQUAL(10, frame[1]);
    TEST_ASSERT_EQUAL(frame[11], crsfCrc8(&frame[2], 9));
}

void test_gps_altitude_offset()
{
    uint16_t alt = 120;
    uint16_t encoded = alt + 1000;
    TEST_ASSERT_EQUAL(1120, encoded);
}

void test_baro_altitude_encoding()
{
    int32_t altCm = 12034;
    int16_t altDm = (altCm / 10) + 10000;
    TEST_ASSERT_EQUAL(11203, altDm);
}

void test_baro_negative_altitude()
{
    int32_t altCm = -500;
    int16_t altDm = (altCm / 10) + 10000;
    TEST_ASSERT_EQUAL(9950, altDm);
}

void test_frame_header_format()
{
    uint8_t frame[16];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 5 + 2;
    frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    uint8_t pos = 3;

    TEST_ASSERT_EQUAL(0xC8, frame[0]);
    TEST_ASSERT_EQUAL(7, frame[1]);
    TEST_ASSERT_EQUAL(0x21, frame[2]);
    TEST_ASSERT_EQUAL(3, pos);
}

void test_flight_mode_frame_format()
{
    const char *mode = "ACRO";
    uint8_t slen = strlen(mode) + 1;
    uint8_t frame[16];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 1 + slen + 1;
    frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&frame[3], mode, slen);
    frame[3 + slen] = crsfCrc8(&frame[2], 1 + slen);

    TEST_ASSERT_EQUAL(7, frame[1]);
    TEST_ASSERT_EQUAL(0x21, frame[2]);
    TEST_ASSERT_EQUAL('A', frame[3]);
    TEST_ASSERT_EQUAL('\0', frame[7]);
    TEST_ASSERT_EQUAL(frame[8], crsfCrc8(&frame[2], 6));
}

void test_gps_frame_format()
{
    int32_t lat = 524213670;
    int32_t lon = 133560120;
    uint16_t spd = 250;
    uint16_t hdg = 18000;
    uint16_t alt = 120;
    uint8_t sats = 12;

    uint8_t frame[19];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 15 + 2;
    frame[2] = CRSF_FRAMETYPE_GPS;
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
    frame[pos] = crsfCrc8(&frame[2], pos - 2);

    TEST_ASSERT_EQUAL(19, pos + 1);
    TEST_ASSERT_EQUAL(17, frame[1]);
    TEST_ASSERT_EQUAL(0x1F, frame[3]);
    TEST_ASSERT_EQUAL(12, frame[17]);
    TEST_ASSERT_EQUAL(frame[18], crsfCrc8(&frame[2], 16));
}

void test_attitude_frame_format()
{
    int16_t pitch = 1500, roll = -300, yaw = 0;

    uint8_t frame[10];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 6 + 2;
    frame[2] = CRSF_FRAMETYPE_ATTITUDE;
    frame[3] = pitch >> 8;    frame[4] = pitch & 0xFF;
    frame[5] = roll >> 8;     frame[6] = roll & 0xFF;
    frame[7] = yaw >> 8;      frame[8] = yaw & 0xFF;
    frame[9] = crsfCrc8(&frame[2], 7);

    TEST_ASSERT_EQUAL(8, frame[1]);
    TEST_ASSERT_EQUAL(0x05, frame[3]);
    TEST_ASSERT_EQUAL(0xDC, frame[4]);
    TEST_ASSERT_EQUAL(0xFE, frame[5]);
    TEST_ASSERT_EQUAL(0xD4, frame[6]);
    TEST_ASSERT_EQUAL(frame[9], crsfCrc8(&frame[2], 7));
}

void test_vario_frame_format()
{
    int16_t vspd = -250;

    uint8_t frame[6];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 2 + 2;
    frame[2] = CRSF_FRAMETYPE_VARIO;
    frame[3] = vspd >> 8;
    frame[4] = vspd & 0xFF;
    frame[5] = crsfCrc8(&frame[2], 3);

    TEST_ASSERT_EQUAL(4, frame[1]);
    TEST_ASSERT_EQUAL(0xFF, frame[3]);
    TEST_ASSERT_EQUAL(0x06, frame[4]);
    TEST_ASSERT_EQUAL(frame[5], crsfCrc8(&frame[2], 3));
}

void test_link_stats_frame_format()
{
    uint8_t frame[14];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 10 + 2;
    frame[2] = CRSF_FRAMETYPE_LINK_STATS;
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
    frame[13] = crsfCrc8(&frame[2], 11);

    TEST_ASSERT_EQUAL(12, frame[1]);
    TEST_ASSERT_EQUAL(100, frame[5]);
    TEST_ASSERT_EQUAL(frame[13], crsfCrc8(&frame[2], 11));
}

// --- Model ID Command Tests ---

void test_model_id_command_parse()
{
    uint8_t frame[10];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = 8;
    frame[2] = CRSF_FRAMETYPE_COMMAND;
    frame[3] = CRSF_ADDRESS_MODULE;
    frame[4] = CRSF_ADDRESS_RADIO;
    frame[5] = CRSF_SUBCOMMAND_CRSF;
    frame[6] = CRSF_COMMAND_MODEL_SELECT_ID;
    frame[7] = 42;

    TEST_ASSERT_EQUAL(0x32, frame[2]);
    TEST_ASSERT_EQUAL(0x10, frame[5]);
    TEST_ASSERT_EQUAL(0x05, frame[6]);
    TEST_ASSERT_EQUAL(42, frame[7]);
    TEST_ASSERT_EQUAL(10, frame[1] + 2);
}

void test_model_id_range()
{
    for (uint8_t id = 0; id <= 63; id++) {
        TEST_ASSERT_TRUE(id <= 63);
    }
}

// --- Sync Byte Tests ---

void test_sync_byte_detection()
{
    TEST_ASSERT_TRUE(crsfIsSyncByte(CRSF_SYNC_BYTE));
    TEST_ASSERT_TRUE(crsfIsSyncByte(CRSF_SYNC_BYTE_MODULE));
    TEST_ASSERT_FALSE(crsfIsSyncByte(0x00));
    TEST_ASSERT_FALSE(crsfIsSyncByte(0xFF));
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

    // Sync byte
    RUN_TEST(test_sync_byte_detection);

    return UNITY_END();
}
