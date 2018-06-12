

#include <sensor_const.h>

#include "gmock/gmock.h"

extern "C" {
    #include "sensor.h"
    #include "i2c.h"
    #include <byte_conversion.h>
}

using namespace testing;


static byte test_mem[256];
static int read_test_memory(int dev_address, int mem_address, byte* const buffer, int n){
    for(int i = 0; i < n; i++)
        buffer[i] = test_mem[i + mem_address];
    return SUCCESS;
}
static int write_test_memory(int dev_address, int mem_address, const byte* buffer, int n){
    for(int i = 0; i < n; i++)
        test_mem[i + mem_address] = buffer[i];
    return SUCCESS;
}


class SensorTest : public ::Test{
protected:
    I2C_Interface* i2c;

    virtual void SetUp(){
        memset(test_mem, 0, 256);
        i2c = i2c_init(0, read_test_memory, write_test_memory);
    }
    virtual void TearDown(){
        free(i2c);
    }
};


TEST_F(SensorTest, test_initialization) {
    Sensor* sensor = sensor_init(i2c);

    //todo test assertions

}

TEST_F(SensorTest, test_properties) {
    int val[64];

    //test value initalization:
    test_mem[GYRO_CONFIG.REG] = GyroRange::DPS_500;
    test_mem[ACCEL_CONFIG.REG] = AcclRange::G_16;

    Sensor* sensor = sensor_init(i2c);


    ASSERT_EQ(sensor_get_gyro_range(sensor), 500);
    ASSERT_EQ(sensor_get_accl_range(sensor), 16);

    // set and get:
    ASSERT_EQ(sensor_set_accl_range(sensor, AcclRange::G_8), SUCCESS);
    ASSERT_EQ(sensor_set_gyro_range(sensor, GyroRange::DPS_2000), SUCCESS);
    //todo: set magn range
    ASSERT_EQ(sensor_set_wake_on_motion_threshold(sensor, -1), ERROR); //value too small
    ASSERT_EQ(sensor_set_wake_on_motion_threshold(sensor, 1021), ERROR); //value too small
    ASSERT_EQ(sensor_set_wake_on_motion_threshold(sensor, 555), SUCCESS);

    ASSERT_EQ(sensor_get_accl_range(sensor), 8);
    ASSERT_EQ(sensor_get_gyro_range(sensor), 2000);
//    ASSERT_EQ(sensor_get_magn_range(sensor, val+2), SUCCESS); //todo: set magn range
    ASSERT_NEAR(sensor_get_wake_on_motion_threshold(sensor), 555, 1020.0 / pow(2, 1 * 8)); // max storage value (256) smaller than encoded range (0~1020)
}

TEST_F(SensorTest, test_measure_all) {
    Sensor* sensor = sensor_init(i2c);

    float buffer[11];
    buffer[10] = -1;
    byte raw[23] = {
        /*A:*/ 0x0F, 0xFF, 0x00, 0xFF, 0x00, 0x0F,
        /*T:*/ 0x0E, 0xEE,
        /*G:*/ 0x0C, 0xCC, 0x00, 0xCC, 0x00, 0x0C,
        /*i:*/ 0xB1, 0xB2, 0xB3,
        /*M:*/ 0x0A, 0xAA, 0x00, 0xAA, 0x00, 0x0A,
    };
    memcpy(test_mem+ACCEL_OUT, raw, 23);

    sensor_measure_all(sensor, buffer);

    float expected[9];
    int range;
    range = sensor_get_accl_range(sensor);
    expected[0] = bytes_to_float(raw+0, 2, range);
    expected[1] = bytes_to_float(raw+2, 2, range);
    expected[2] = bytes_to_float(raw+4, 2, range);
    range = sensor_get_gyro_range(sensor);
    expected[3] = bytes_to_float(raw+8, 2, range);
    expected[4] = bytes_to_float(raw+10, 2, range);
    expected[5] = bytes_to_float(raw+12, 2, range);
    range = sensor_get_magn_range(sensor);
    expected[6] = bytes_to_float(raw+17, 2, range);
    expected[7] = bytes_to_float(raw+19, 2, range);
    expected[8] = bytes_to_float(raw+21, 2, range);

    //todo: assert temperature:
    // bytes_to_float(raw+6, 2, expected+9);

    ASSERT_THAT(
        std::vector<float>(buffer, buffer + 9),
        ElementsAreArray(expected)
    );
    ASSERT_EQ(buffer[10], -1);
 }

TEST_F(SensorTest, test_measure_all_as_bytes) {
    Sensor* sensor = sensor_init(i2c);

    byte buffer[21];
    buffer[20] = 255u;
    byte raw[23] = {
            /*A:*/ 0x0F, 0xFF, 0x00, 0xFF, 0x00, 0x0F,
            /*T:*/ 0x0E, 0xEE,
            /*G:*/ 0x0C, 0xCC, 0x00, 0xCC, 0x00, 0x0C,
            /*i:*/ 0xB1, 0xB2, 0xB3,
            /*M:*/ 0x0A, 0xAA, 0x00, 0xAA, 0x00, 0x0A,
    };
    memcpy(test_mem+ACCEL_OUT, raw, 23);

    sensor_measure_all_bytes(sensor, buffer);

    byte expected[] = {
            0x0Fu, 0xFFu, 0x00u, 0xFFu, 0x00u, 0x0Fu,
            0x0Cu, 0xCCu, 0x00u, 0xCCu, 0x00u, 0x0Cu,
            0x0Au, 0xAAu, 0x00u, 0xAAu, 0x00u, 0x0Au,
            0x0Eu, 0xEEu,
    };

    ASSERT_THAT(
            std::vector<float>(buffer, buffer + 20),
            ElementsAreArray(expected)
    );
    ASSERT_EQ(buffer[20], 255u);
}

TEST_F(SensorTest, test_configure_interrupts) {
    Sensor* sensor = sensor_init(i2c);

    //check that initial values are set to 0
    ASSERT_EQ(test_mem[INT_PIN_CFG.REG], 0);
    ASSERT_EQ(test_mem[INT_ENABLE.REG], 0);

    //Set all possible bits to 1
    sensor_interrupts_set_pin(sensor, true, true, ClearInterruptOn::READ_ANY);
    ASSERT_EQ(
            test_mem[INT_PIN_CFG.REG],
            INT_PIN_CFG.ACTL | INT_PIN_CFG.OPEN | INT_PIN_CFG.LATCH_EN | INT_PIN_CFG.ANYRD_2CLEAR
    );

    sensor_interrupts_set_source(sensor, true, true, true);
    ASSERT_EQ(test_mem[INT_ENABLE.REG], INT_ENABLE.RAW_RDY | INT_ENABLE.WOM | INT_ENABLE.FIFO_OVERFLOW);

    //Test rest of ClearInterruptOn flags:
    sensor_interrupts_set_pin(sensor, false, false, ClearInterruptOn::PULSE);
    ASSERT_EQ(test_mem[INT_PIN_CFG.REG], 0);
    sensor_interrupts_set_pin(sensor, false, false, ClearInterruptOn::READ_ANY);
    ASSERT_EQ(test_mem[INT_PIN_CFG.REG], INT_PIN_CFG.LATCH_EN | INT_PIN_CFG.ANYRD_2CLEAR);
    sensor_interrupts_set_pin(sensor, false, false, ClearInterruptOn::READ_INT_STATUS);
    ASSERT_EQ(test_mem[INT_PIN_CFG.REG], INT_PIN_CFG.LATCH_EN );
}

