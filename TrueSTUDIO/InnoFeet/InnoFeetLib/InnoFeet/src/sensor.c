
#include <stdlib.h>
#include <time.h>
#include <memory.h>

#include "byte_conversion.h"
#include "sensor.h"

typedef struct _Sensor {
    I2C_Interface * i2c;

    int gyro_range;
    int accl_range;
    int magn_range;
    int wake_on_motion_threshold;

    struct {
        int WIA, INFO, ST1, ST2;
        int HXH, HYH, HZH;
        int CNTL1, CNTL2;
    } magn_reg;
} _Sensor;


int _init_magnetometer(Sensor*);
int _init_properties(Sensor*);

Sensor* sensor_init(I2C_Interface *i2c) {
    Sensor* sensor = malloc(sizeof(*sensor));
    sensor->i2c = i2c;
    sensor_reset(sensor); //todo: handle error
    return sensor;
};

int sensor_reset(Sensor* self) {
    TRY(i2c_write_bits(self->i2c, PWR_MGMNT_1.REG, PWR_MGMNT_1.H_RESET));
    TRY(_init_magnetometer(self));
    TRY(_init_properties(self));

    return SUCCESS;
}

int _init_magnetometer(Sensor *self){
    //Enable master:
    i2c_write_bits(self->i2c, INT_PIN_CFG.REG, 0, INT_PIN_CFG.BYPASS_EN); // Disable bypass
    i2c_write_byte(self->i2c, I2C_MST_CTRL.REG, I2C_MST_CTRL.WAIT_FOR_ES | I2C_MST_CTRL.P_NSR | I2C_MST_CTRL.CLK.khz400);
        // Don't send ready-interrupt until slave read, stop (instead of restart) between slave reads:
    i2c_write_byte(self->i2c, USER_CTRL.REG, USER_CTRL.I2C_MST_EN);

    //Slave 0 for reading sensors
    i2c_write_byte(self->i2c, I2C_SLV0_CTRL.REG, 0x00); // Disable for config
    i2c_write_byte(self->i2c, I2C_SLV0_ADDR.REG, MAGN_DEV_ADDR | I2C_SLV0_ADDR.RNW);
    i2c_write_byte(self->i2c, I2C_SLV0_REG, 3u);  // Start reading from WIA (Device ID)
    byte read_n_bytes = 10u;
    byte slv0_flags =  I2C_SLV0_CTRL.EN | I2C_SLV0_CTRL.BYTE_SW | I2C_SLV0_CTRL.GRP;
    i2c_write_byte(self->i2c, I2C_SLV0_CTRL.REG, slv0_flags | read_n_bytes); //Read 10 bytes

    self->magn_reg.WIA = EXT_SENS_DATA_00 + 0;
    self->magn_reg.INFO = EXT_SENS_DATA_00 + 2;
    self->magn_reg.ST1 = EXT_SENS_DATA_00 + 1;
//    self->magn_reg.HXL = EXT_SENS_DATA_00 + 4;
    self->magn_reg.HXH = EXT_SENS_DATA_00 + 3;
//    self->magn_reg.HYL = EXT_SENS_DATA_00 + 6;
    self->magn_reg.HYH = EXT_SENS_DATA_00 + 5;
//    self->magn_reg.HZL = EXT_SENS_DATA_00 + 8;
    self->magn_reg.HZH = EXT_SENS_DATA_00 + 7;
    self->magn_reg.ST2 = EXT_SENS_DATA_00 + 9;

    //Slave 1 for config
    read_n_bytes = 2u;
    i2c_write_byte(self->i2c, I2C_SLV0_CTRL.REG + 3, 0x00); // Disable for config
    i2c_write_byte(self->i2c, I2C_SLV0_CTRL.REG + 3, MAGN_DEV_ADDR);
    i2c_write_byte(self->i2c, I2C_SLV0_REG + 3, 0x00);  // Start reading from CNTL1
    i2c_write_byte(self->i2c, I2C_SLV0_CTRL.REG + 3, I2C_SLV0_CTRL.EN | read_n_bytes); //Read 2 bytes

    self->magn_reg.CNTL1 = EXT_SENS_DATA_00 + 10;
    self->magn_reg.CNTL2 = EXT_SENS_DATA_00 + 11;

    //
    i2c_write_bits(self->i2c, self->magn_reg.CNTL1, MAGN_BIT);
    return SUCCESS; //todo: Handle errors
}

int _init_properties(Sensor* self) {
    byte temp;

    TRY(i2c_read_byte(self->i2c, GYRO_CONFIG.REG, &temp));
    self->gyro_range = GyroRangeAsInt((enum GyroRange) (temp & GYRO_CONFIG.FS_SEL));

    TRY(i2c_read_byte(self->i2c, ACCEL_CONFIG.REG, &temp));
    self->accl_range = AcclRangeAsInt((enum AcclRange) (temp & ACCEL_CONFIG.FS_SEL));

    self->magn_range = 4912; //todo: how is magn scale set?

    TRY(i2c_read_byte(self->i2c, WOM_THR, &temp));
    self->wake_on_motion_threshold = (int)bytes_to_ufloat(&temp, 1, 1020);
}

int sensor_interrupts_set_pin(Sensor* self, bool active_low, bool open_drain, enum ClearInterruptOn clear_on) {
    byte config_bits = 0;

    active_low && (config_bits |= INT_PIN_CFG.ACTL);
    open_drain && (config_bits |= INT_PIN_CFG.OPEN);
    clear_on != PULSE && (config_bits |= INT_PIN_CFG.LATCH_EN);
    clear_on == READ_ANY && (config_bits |= INT_PIN_CFG.ANYRD_2CLEAR);
    //INT_PIN_CFG.BYPASS_EN used when external device connected to EDA/ECL pins - ignored, no ext sensors
    //INT_PIN_CFG.FSYNC_MODE_EN enables fsync as an INPUT(?) that causes an interrupt - ignored, no ext interrupts
    //INT_PIN_CFG.FSYNC_ACTL - ignored also

    TRY(i2c_write_byte(self->i2c, INT_PIN_CFG.REG, config_bits));
    return SUCCESS;
}

int sensor_interrupts_set_source(Sensor* self, bool on_data_ready, bool on_motion, bool on_fifo_overflow) {
    byte config_bits = 0;
    on_data_ready && (config_bits |= INT_ENABLE.RAW_RDY);
    on_motion && (config_bits |= INT_ENABLE.WOM);
    on_fifo_overflow && (config_bits |= INT_ENABLE.FIFO_OVERFLOW);

    TRY(i2c_write_byte(self->i2c, INT_ENABLE.REG, config_bits));
    if(on_motion)
        TRY(i2c_write_byte(self->i2c, ACCEL_INTEL_CTRL.REG, ACCEL_INTEL_CTRL.EN | ACCEL_INTEL_CTRL.MODE));
    return SUCCESS;
}


//PROPERTIES:

int sensor_get_gyro_range(Sensor* self){ return self->gyro_range; }
int sensor_set_gyro_range(Sensor* self, enum GyroRange r){
    TRY(i2c_write_bits(self->i2c, GYRO_CONFIG.REG, r, GYRO_CONFIG.FS_SEL));
    self->gyro_range = GyroRangeAsInt(r);
    return SUCCESS;
}
int sensor_get_accl_range(Sensor* self){ return self->accl_range; }
int sensor_set_accl_range(Sensor* self, enum AcclRange r){
    TRY(i2c_write_bits(self->i2c, ACCEL_CONFIG.REG, r, ACCEL_CONFIG.FS_SEL));
    self->accl_range = AcclRangeAsInt(r);
    return SUCCESS;
}
int sensor_get_magn_range(Sensor* self){ return self->magn_range; }

int sensor_get_wake_on_motion_threshold(Sensor *self) { return self->wake_on_motion_threshold; }
int sensor_set_wake_on_motion_threshold(Sensor *self, int threshold) {
    byte th_byte;
    TRY(ufloat_to_bytes((float)threshold, 1020, &th_byte, 1));
    TRY(i2c_write_byte(self->i2c, WOM_THR, th_byte));
    self->wake_on_motion_threshold = (int)bytes_to_ufloat(&th_byte, 1, 1020); //Range 0~1020 won't fit 1 byte precisely
    return SUCCESS;
}


//METHODS:

int sensor_measure_temp(Sensor* self, int buffer[1]){
    byte data[6];
    TRY(i2c_read_bytes(self->i2c, TEMP_OUT, data, 2));
    buffer[0] = bytes_to_int(data+0, 2);
    return SUCCESS;
}
int sensor_measure_gyro(Sensor* self, int buffer[3]){
    byte data[6];
    TRY(i2c_read_bytes(self->i2c, GYRO_OUT, data, 6));
    buffer[0] = bytes_to_int(data+0, 2);
    buffer[0] = bytes_to_int(data+2, 2);
    buffer[0] = bytes_to_int(data+4, 2);
    return SUCCESS;
}
int sensor_measure_accl(Sensor* self, int buffer[3]){
    byte data[6];
    TRY(i2c_read_bytes(self->i2c, ACCEL_OUT, data, 6));
    buffer[0] = bytes_to_int(data+0, 2);
    buffer[0] = bytes_to_int(data+2, 2);
    buffer[0] = bytes_to_int(data+4, 2);
    return SUCCESS;
}
int sensor_measure_magn(Sensor* self, int buffer[3]){
    byte data[6];
    TRY(i2c_read_bytes(self->i2c, self->magn_reg.HXH, data, 6));
    buffer[0] = bytes_to_int(data+0, 2);
    buffer[0] = bytes_to_int(data+2, 2);
    buffer[0] = bytes_to_int(data+4, 2);
    return SUCCESS;
}

int sensor_measure_all(Sensor* self, float buffer[10]){
    byte data[23]; // gyro 6, temp 2, accl 6, ignored 3, magn 6
    TRY(i2c_read_bytes(self->i2c, ACCEL_OUT, data, 23));

    buffer[0] = bytes_to_float(data+0, 2, self->accl_range); //Accl 0-5
    buffer[1] = bytes_to_float(data+2, 2, self->accl_range);
    buffer[2] = bytes_to_float(data+4, 2, self->accl_range);
    // Skip Temp 6-7 for now, cpy to end of buffer instead
    buffer[3] = bytes_to_float(data+8, 2, self->gyro_range); //Gyro 8-13
    buffer[4] = bytes_to_float(data+10, 2, self->gyro_range);
    buffer[5] = bytes_to_float(data+12, 2, self->gyro_range);
    // 14, 15, 16 skipped (Magn device ID, info, status 1)
    buffer[6] = bytes_to_float(data+17, 2, self->magn_range); // Magn 17-22
    buffer[7] = bytes_to_float(data+19, 2, self->magn_range);
    buffer[8] = bytes_to_float(data+21, 2, self->magn_range);

    buffer[9] = (float)bytes_to_int(data+6, 2); //Temp 6-7 //todo: how does temperature scale?
    return SUCCESS;
}

int sensor_measure_all_bytes(Sensor* self, byte buffer[20]){
    byte data[23]; // gyro 6, temp 2, accl 6, ignored 3, magn 6
    TRY(i2c_read_bytes(self->i2c, ACCEL_OUT, data, 23));

    memcpy(buffer+0, data+0, 6); //Accl 0-5
    // Skip Temp 6-7 for now, cpy to end of buffer instead
    memcpy(buffer+6, data+8, 6); //Gyro 8-13
    // 14, 15, 16 skipped (Magn device ID, info, status 1)
    memcpy(buffer+12, data+17, 6); // Magn 17-22
    memcpy(buffer+18, data+6, 2); //Temp 6-7
    return SUCCESS;
}

int _sensor_interrupts_clear1(Sensor* self) {
    byte temp;
    return _sensor_interrupts_clear2(self, &temp);
}
int _sensor_interrupts_clear2(Sensor* self, byte cause[1]){
    TRY(i2c_read_byte(self->i2c, INT_STATUS.REG, cause));
    return SUCCESS;
}

