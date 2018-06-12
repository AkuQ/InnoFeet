
#include "gmock/gmock.h"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedMacroInspection"

extern "C" {
    #include "byte_conversion.h"
}

using namespace testing;


#define print_bin(NUM) ({ \
    uint64_t num = (uint64_t)(NUM); \
    int w = sizeof((NUM)) * 8; \
    char b[w + 1]; \
    for(int i = w - 1; i >= 0; i--) { \
        (num & 1u) && (b[i] = '1') || (b[i] = '0'); \
        num >>= 1u; \
    } \
    b[w] = '\0'; \
    printf(b); printf("\n"); \
})


TEST(basic_check, test_int_to_bytes) {
    unsigned int ui = 0b0
            | 0b00000000000000000000000010000011
            | 0b00000000000000001000010100000000
            | 0b00000000100001110000000000000000
            | 0b10001001000000000000000000000000;
    int i = 0b0
            | 0b00000000000000000000000010000011
            | 0b00000000000000001000010100000000
            | 0b00000000100001110000000000000000
            | 0b01001001000000000000000000000000;


    byte actual[5];

    // correct positive value
    // unsigned
    ASSERT_EQ(uint_to_bytes(ui, actual, 4), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 4),
            ElementsAreArray(std::vector<byte>({0b10001001, 0b10000111, 0b010000101, 0b10000011}))
    );
    // signed
    ASSERT_EQ(int_to_bytes(i, actual, 4), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 4),
            ElementsAreArray(std::vector<byte>({0b01001001, 0b10000111, 0b010000101, 0b10000011}))
    );
    //signed with extra byte
    ASSERT_EQ(int_to_bytes(i, actual, 5), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 5),
            ElementsAreArray(std::vector<byte>({0b00000000, 0b01001001, 0b10000111, 0b010000101, 0b10000011}))
    );

    // correct negative value
    //i's two's complement = 10110110 01111000 01111010 01111101:
    ASSERT_EQ(0b10110110011110000111101001111101, -i);
    ASSERT_EQ(int_to_bytes(-i, actual, 4), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 4),
            ElementsAreArray(std::vector<byte>({0b10110110, 0b01111000, 0b01111010, 0b01111101}))
    );
    //with extra byte
    ASSERT_EQ(int_to_bytes(-i, actual, 5), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 5),
            ElementsAreArray(std::vector<byte>({0b11111111, 0b10110110, 0b01111000, 0b01111010, 0b01111101}))
    );

    // Edge case values pass
    ASSERT_EQ(uint_to_bytes(0xFFFFFF, actual, 3), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 3),
            ElementsAreArray(std::vector<byte>({0xFF, 0xFF, 0xFF}))
    );
    ASSERT_EQ(int_to_bytes(0x7FFFFF, actual, 3), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 3),
            ElementsAreArray(std::vector<byte>({0x7F, 0xFF, 0xFF}))
    );
    ASSERT_EQ(int_to_bytes(-1, actual, 3), SUCCESS);
    ASSERT_THAT(
            std::vector<byte>(actual, actual + 3),
            ElementsAreArray(std::vector<byte>({0xFF, 0xFF, 0xFF}))
    );

    // byte array too short
    ASSERT_EQ(uint_to_bytes(ui, actual, 3), ERROR);
    ASSERT_EQ(int_to_bytes(i, actual, 3), ERROR);

    // last byte reserves complement bit
    ASSERT_EQ(int_to_bytes(0b1000111100001111 , actual, 2), ERROR);
}


TEST(basic_check, test_bytes_to_values) {

    int i = 0;
    i += 0b001u << 0u;
    i += 0b010u << 8u;
    i += 0b011u << 16u;
    i += 0b100u << 24u;

    // unsigned int1
    byte input1[] = {0b100, 0b011, 0b010, 0b001};
    ASSERT_EQ(bytes_to_uint(input1, 4), i);

    // signed int1
    byte complement1[] = {0b100u ^ 0xffu, 0b011u ^ 0xffu, 0b010u ^ 0xffu, 0b01u ^ 0xffu - 1};
    // Check that complement bytes written out correctly:
    ASSERT_EQ(-i, (complement1[0] << 24u) | (complement1[1] << 16u) | (complement1[2] << 8u) | complement1[3]);
    ASSERT_EQ(bytes_to_int(complement1, 4), -i);
//    byte complement2[] = {0xff, 0xff, 0xff, 0b100u ^ 0xffu, 0b011u ^ 0xffu, 0b010u ^ 0xffu, 0b01u ^ 0xffu - 1};
//    ASSERT_EQ(bytes_to_int(complement2, 7), -i); todo: return int64

    // unsigned float1
    int m = 0xffffff;
    float r = 3.14;
    float expected = (float)0x444444 / m * r;
    byte input2[] = {0x44, 0x44, 0x44};
    ASSERT_EQ(bytes_to_ufloat(input2, 3, r), expected);
    // min1
    byte input3[] = {0, 0, 0};
    ASSERT_EQ(bytes_to_ufloat(input3, 3, r), 0);
    // max1
    byte input4[] = {0xff, 0xff, 0xff};
    ASSERT_EQ(bytes_to_ufloat(input4, 3, r), 3.14f);

    // signed float1
    m = 0xffffff;
    m += 0b01111111u << 8u * 3;
    r = 3.14;
    expected = float(0x04444444) / float(m) * r;
    byte input5[] = {0x04, 0x44, 0x44, 0x44};
    ASSERT_EQ(bytes_to_float(input5, 4, r), expected);
    // min1
    byte input6[] = {0b10000000, 0, 0, 0};
    ASSERT_EQ(bytes_to_float(input6, 4, r), -3.14f);
    // max1;
    byte input7[] = {0b01111111, 0xff, 0xff, 0xff};
    ASSERT_EQ(bytes_to_float(input7, 4, r), 3.14f);
}

TEST(basic_check, test_float_to_bytes) {
    byte buffer[10];

    // correct unsigned1
    float f = 0.5;
    float r = 2.0;
    int expected = (int)(roundf(0xffff * (f / r)));

    ASSERT_EQ(ufloat_to_bytes(f, r, buffer, 2), SUCCESS);
    ASSERT_EQ(bytes_to_int(buffer, 2), expected);
    ASSERT_EQ(ufloat_to_bytes((float)f, (float)r, buffer, 2), SUCCESS);
    ASSERT_EQ(bytes_to_int(buffer, 2), expected);

    //Min-max unsigned values pass:
    ASSERT_EQ(ufloat_to_bytes(0.0f, 3.33f, buffer, 8), SUCCESS);
    ASSERT_EQ(ufloat_to_bytes(3.33f, 3.33f, buffer, 8), SUCCESS);

    // over/underflow unsigned
    ASSERT_EQ(ufloat_to_bytes(3.3300001f, 3.33f, buffer, 8), ERROR);
    ASSERT_EQ(ufloat_to_bytes(3.33000001f, 3.33f, buffer, 8), SUCCESS); //non-meaningful decimal places
    ASSERT_EQ(ufloat_to_bytes(-0.000000000000000001f, 3.33f, buffer, 8), ERROR);

    //Todo: calculate max precision for byte rounding?

    // correct signed1
    f = 1.0f / 3.0f;
    r = 1.0f;
    expected = int(roundf(0x7fffff * (f / r)));

    //positive value:
    ASSERT_EQ(float_to_bytes(f, r, buffer, 3), SUCCESS);
    ASSERT_EQ(bytes_to_int(buffer, 3), expected);

    //negative value:
    f = -f;
    ASSERT_EQ(float_to_bytes(f, r, buffer, 3), SUCCESS);
    ASSERT_EQ(bytes_to_int(buffer, 3), 1-expected);

    //Min-max signed values pass:
    ASSERT_EQ(float_to_bytes(-3.33f, 3.33f, buffer, 8), SUCCESS);
    ASSERT_EQ(float_to_bytes(3.33f, 3.33f, buffer, 8), SUCCESS);

    // over/underflow signed
    ASSERT_EQ(float_to_bytes(3.3300001f, 3.33f, buffer, 8), ERROR);
    ASSERT_EQ(float_to_bytes(3.33000001f, 3.33f, buffer, 8), SUCCESS); //non-meaningful decimal places
    ASSERT_EQ(float_to_bytes(-3.3300001f, 3.33f, buffer, 8), ERROR);
    ASSERT_EQ(float_to_bytes(-3.33000001f, 3.33f, buffer, 8), SUCCESS); //non-meaningful decimal places

    // Too many bytes:
    ASSERT_EQ(ufloat_to_bytes(0.0, 3.33, buffer, sizeof(u_int64_t) + 1), ERROR);
    ASSERT_EQ(float_to_bytes(0.0, 3.33, buffer,  sizeof(u_int64_t) + 1), ERROR);
}

TEST(basic_check, test_values_to_bytes_and_back) {
    byte buffer[4];

    int expected = 77777;
    uint_to_bytes((unsigned)expected, buffer, 4);
    ASSERT_EQ(bytes_to_uint(buffer, 4), expected);

    expected = -77777;
    int_to_bytes(expected, buffer, 4);
    ASSERT_EQ(bytes_to_int(buffer, 4), expected);

    auto expectedf = (float)M_PI;
    ufloat_to_bytes(expectedf, 10.0, buffer, 4);
    ASSERT_EQ(bytes_to_ufloat(buffer, 4, 10.0), expectedf);

    float_to_bytes(-expectedf, 10.0, buffer, 4);
    ASSERT_EQ(bytes_to_float(buffer, 4, 10.0), -expectedf);
}

#pragma clang diagnostic pop