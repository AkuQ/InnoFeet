
#include "gmock/gmock.h"
extern "C" {
    #include "i2c.h"
}

using namespace testing;

TEST(basic_check, test_initialization){
    int dev_address = 0x68;
    auto dummy_read_bytes = [](int dev_addr, int mem_addr, byte* const bytes, int n){return 0;};
    auto dummy_write_bytes = [](int dev_addr, int mem_addr, const byte* bytes, int n){return 0;};

    I2C_Interface* i2c = i2c_init(dev_address, dummy_read_bytes, dummy_write_bytes);

    ASSERT_EQ(i2c_get_device_address(i2c), dev_address);
}


TEST(basic_check, test_reading_bytes){
    int dev_addess = 0x68;
    auto dummy_read_bytes = [](int dev_addr, int mem_addr, byte* const  bytes, int n){
        char* mem = (char *)("0123456789 HELLO_WORLD!");
        for(int i = 0; i < n; i++) {
            (bytes)[i] = (byte)mem[mem_addr + i];
        }
        return 0;
    };
    auto dummy_write_bytes = [](int dev_addr, int mem_addr, const byte* bytes, int n){return 0;};

    I2C_Interface* i2c = i2c_init(dev_addess, dummy_read_bytes, dummy_write_bytes);

    byte buffer;
    ASSERT_EQ(i2c_read_byte(i2c, 5, &buffer), SUCCESS);
    ASSERT_EQ(buffer, '5');
    ASSERT_EQ(i2c_read_byte(i2c, 16, &buffer), SUCCESS);
    ASSERT_EQ(buffer, '_');

    byte buffer_array[6];
    ASSERT_EQ(i2c_read_bytes(i2c, 3, buffer_array, 6), SUCCESS);
    ASSERT_TRUE(strcmp("3456789", (char*)buffer_array));
    ASSERT_EQ(i2c_read_bytes(i2c, 10, buffer_array, 6), SUCCESS);
    ASSERT_TRUE(strcmp("HELLO", (char*)buffer_array));
}

TEST(basic_check, test_writing_bytes){
    int dev_address = 0x68;
    static char test_mem[16];
    memset(test_mem,'.',16);

    auto dummy_read_bytes = [](int dev_addr, int mem_addr, byte* const bytes, int n){return 0;};
    auto write_test_mem = [](int dev_addr, int mem_addr, const byte* bytes, int n){
        for(int i = 0; i < n; i++) {
            test_mem[mem_addr + i] = bytes[i];
        }
        return SUCCESS;
    };

    I2C_Interface* i2c = i2c_init(dev_address, dummy_read_bytes, write_test_mem);

    ASSERT_EQ(i2c_write_bytes(i2c, 8, (byte*)"HOLA!", 5), SUCCESS);
    ASSERT_STREQ("........HOLA!...", test_mem);
    ASSERT_EQ(i2c_write_bytes(i2c, 2,  (byte*)"12345", 2), SUCCESS);
    ASSERT_STREQ("..12....HOLA!...", test_mem);

    ASSERT_EQ(i2c_write_byte(i2c, 13, '?'), SUCCESS);
    ASSERT_STREQ("..12....HOLA!?..", test_mem);
    ASSERT_EQ(i2c_write_byte(i2c, 2, '3'), SUCCESS);
    ASSERT_STREQ("..32....HOLA!?..", test_mem);
}

TEST(basic_check, test_writing_bits){
    int dev_address = 0x68;
    static char test_mem[16];
    memset(test_mem,'?',16); // b01111111

    auto read_test_mem = [](int dev_addr, int mem_addr, byte* const bytes, int n){
        for(int i = 0; i < n; i++) {
            (bytes)[i] = (byte)test_mem[mem_addr + i];
        }
        return 0;
    };
    auto write_test_mem = [](int dev_addr, int mem_addr, const byte* bytes, int n){
        for(int i = 0; i < n; i++) {
            test_mem[mem_addr + i] = bytes[i];
        }
        return SUCCESS;
    };

    I2C_Interface* i2c = i2c_init(dev_address, read_test_mem, write_test_mem);
    ASSERT_EQ(i2c_write_bits(i2c, 3, 0, 0b1), SUCCESS); //Returns success
    ASSERT_STREQ("???>????????????", test_mem); //Writes select byte

    i2c_write_bits(i2c, 3, 0b11111111); //Reset to 1
    ASSERT_EQ(test_mem[3], (char)0b11111111);
    i2c_write_bits(i2c, 3, 0, 0b1111 << 4u); //Reset 4 left bytes to 0
    ASSERT_EQ(test_mem[3], (char)0b00001111);
    i2c_write_bits(i2c, 3, 0b10101010, 0b00111100); // Write 4 middle bits
    ASSERT_EQ(test_mem[3], (char)0b00101011);
}
