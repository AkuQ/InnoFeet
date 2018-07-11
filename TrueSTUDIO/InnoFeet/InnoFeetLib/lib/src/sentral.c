

#include <stdint.h>

#include "sentral.h"
#include "byte_conversion.h"

typedef struct _SENtral {
    I2C_Interface * i2c;

    int gyro_range;
    int accl_range;
    int magn_range;
   
} _SENtral;

#define REG SENtralRegister.
#define FLAG SENtralBitFlags.
#define MASK SENtralBitFlags.

//PRIVATE:

int _load_param(SENtral* self, enum SENtralParamSetter param, byte value[const 4]) {
    byte ack_param;

    TRY(i2c_write_bytes(self->i2c, REG LoadParamBytes,      value, 4)); //todo: can attach below ParamRequest to this multi-byte write ()
    TRY(i2c_write_byte(self->i2c,  REG ParamRequest,        MASK Parameter & param));
    TRY(i2c_write_bits(self->i2c,  REG AlgorithmControl,    FLAG ParamTransfer);
    TRY(i2c_read_byte(self->i2c,   REG ParamAcknowledge,    &ack_param));
    TRY(i2c_write_byte(self->i2c,  REG ParamRequest,        FLAG ClearParam));
    TRY(i2c_write_bits(self->i2c,  REG AlgorithmControl, 0, FLAG ParamTransfer));
    if(ack_param != param)
        return ERROR; //todo: more descriptive error
    return SUCCESS;
}

int _load_params(SENtral* self, enum SENtralParamSetter param, int n, byte * buffer) {
    byte ack_param;
    TRY(i2c_write_bytes(self->i2c, REG LoadParamBytes,      buffer, 4)); //todo: can attach below ParamRequest to this multi-byte write ()
    TRY(i2c_write_byte(self->i2c,  REG ParamRequest,        MASK Parameter & param));
    TRY(i2c_write_bits(self->i2c,  REG AlgorithmControl,    FLAG ParamTransfer);
    TRY(i2c_read_byte(self->i2c,   REG ParamAcknowledge,    &ack_param));
    TRY(i2c_write_byte(self->i2c,  REG ParamRequest,        FLAG ClearParam));
    TRY(i2c_write_bits(self->i2c,  REG AlgorithmControl, 0, FLAG ParamTransfer));
    if(ack_param != param)
        return ERROR; //todo: more descriptive error
    return SUCCESS;
}

int _retrieve_param(SENtral* self, enum SENtralParamGetter param, byte value[4]) {
    byte ack_param;

    TRY(i2c_write_byte(self->i2c,   REG ParamRequest,        MASK Parameter & param));
    TRY(i2c_write_bits(self->i2c,   REG AlgorithmControl,    FLAG ParamTransfer);
    do TRY(i2c_read_byte(self->i2c, REG ParamAcknowledge,    &ack_param)) while(ack_param != param); //todo: endless loop handling
    TRY(i2c_read_bytes(self->i2c,   REG RetrieveParamBytes,  value, 4));
    TRY(i2c_write_byte(self->i2c,   REG ParamRequest,        FLAG ClearParam));
    TRY(i2c_write_bits(self->i2c,   REG AlgorithmControl, 0, FLAG ParamTransfer));
    return SUCCESS;
}

int _retrieve_params(SENtral* self, enum SENtralParamGetter * param, int n, byte* buffer) {
    byte ack_param;

    for (int i = 0; i < n; i++) {
        TRY(i2c_write_byte(self->i2c,   REG ParamRequest,       MASK Parameter & param[i]));
        if(i == 0) TRY(i2c_write_bits(self->i2c, REG AlgorithmControl, FLAG ParamTransfer);
        do TRY(i2c_read_byte(self->i2c, REG ParamAcknowledge,   &ack_param)) while(ack_param != param[i]); //todo: endless loop handling
        TRY(i2c_read_bytes(self->i2c,   REG RetrieveParamBytes, buffer+(i*4), 4));
    }
    TRY(i2c_write_byte(self->i2c, REG ParamRequest,     FLAG ClearParam));
    TRY(i2c_write_bits(self->i2c, REG AlgorithmControl, 0, FLAG ParamTransfer));
    return SUCCESS;
}


int _set_gyro_range(SENtral* self, int value) {
    byte load[4] = {0,0,0,0};
    TRY(int_to_bytes((int64_t)value, load, 4));

    return SUCCESS;
}

int _init_properties(SENtral* self) {
    byte temp[8];
    TRY(_retrieve_param(self, GetMagnAccelRange, temp+0));
    TRY(_retrieve_param(self, GetGyroDriveID, temp+4));
    self->magn_range = bytes_to_uint(temp+0, 2);
    self->accl_range = bytes_to_uint(temp+2, 2);
    self->gyro_range = bytes_to_uint(temp+4, 2);
    return SUCCESS;
}



//INIT:
SENtral* sensor_init(I2C_Interface* i2c){}

//PROPERTIES:
/// \return  Maximum gyroscope measurement ± in degrees per second
int sensor_get_gyro_range(SENtral* self){}
int sensor_set_gyro_range(SENtral* self, enum GyroRange r){}
/// \return  Maximum accelerometer measurement ± in Gs
int sensor_get_accl_range(SENtral* self){}
int sensor_set_accl_range(SENtral* self, enum AcclRange r){}
/// \return  Maximum magnetometer measurement ± in μT
int sensor_get_magn_range(SENtral* self){}
int sensor_set_accl_range(SENtral* self, enum MagnRange r){}

//METHODS:
int sensor_interrupts_set_source(SENtral* self, bool cpu_rest, bool error, bool qtern, bool magn, bool accl, bool gyro){}
int sensor_set_data_rates(SENtral* self, byte magn_Hz, byte accl_dHz, byte gyro_dHz, byte qtern_div){}
int sensor_set_orientation_mode(SENtral* self, bool hpr){}

int sensor_start(SENtral* self){}
int sensor_standby(SENtral* self){}
int sensor_reset(SENtral* self){}

int sensor_measure_qtern(SENtral* self, int buffer[5]){}
int sensor_measure_gyro(SENtral* self, int buffer[4]){}
int sensor_measure_accl(SENtral* self, int buffer[4]){}
int sensor_measure_magn(SENtral* self, int buffer[4]){}
int sensor_measure_all(SENtral* self, float buffer[10]){}
int sensor_measure_all_bytes(SENtral* self, byte buffer[20]){}
