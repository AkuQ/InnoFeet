

#include <stdlib.h>
#include <math.h>

#include "sentral.h"
#include "byte_conversion.h"


const SENtralInit SENtralInitDefaults = {
        {1, 1, 1, 1, 1, 1},
        {100, 10, 10, 1},
        0, 0
};

typedef struct _SENtral {
    I2C_Interface * i2c;
    SENtralInit init;
    unsigned short gyro_range;
    unsigned short accl_range;
    unsigned short magn_range;
} _SENtral;

//State:
#define UNKNOWN         0
#define INITIALIZED     1
#define NORMAL          2
#define STANDBY         3
#define PASS_THROUGH    4

#define MAGN_FS SENtral_MAGN_FS
#define ACCL_FS SENtral_ACCL_FS
#define GYRO_FS SENtral_GYRO_FS

#define REG  SENtralRegister.
#define FLAG SENtralBitFlags.
#define MASK SENtralBitFlags.

#define WRITE(a, b)         TRY(i2c_write_byte(self->i2c, a, b))
#define WRITE_N(a, b, c)    TRY(i2c_write_bytes(self->i2c, a, b, c))
#define READ(a, b)          TRY(i2c_read_byte(self->i2c, a, b))
#define READ_N(a, b, c)     TRY(i2c_read_bytes(self->i2c, a, b, c))
#define WRITE_B(a, b)       TRY(i2c_write_bits(self->i2c, a, b))
#define WRITE_M(a, b, c)    TRY(i2c_write_bits(self->i2c, a, b, c))
#define UNSET_B(a, b)       TRY(i2c_write_bits(self->i2c, a, 0, b))

//PRIVATE:

int _load_param(SENtral* self, enum SENtralParamSetter param, byte value[4]) {
    byte ack_param;

    WRITE_N(   REG LoadParamBytes,      value, 4); //todo optimization: can attach below ParamRequest to this multi-byte write
    WRITE(     REG ParamRequest,        FLAG LoadParam | MASK Parameter & param);
    WRITE_B(   REG AlgorithmControl,    FLAG ParamTransfer);
    do READ(   REG ParamAcknowledge,    &ack_param) while (ack_param != param);
    WRITE(     REG ParamRequest,        FLAG ClearParam);
    UNSET_B(   REG AlgorithmControl,    FLAG ParamTransfer);
    return SUCCESS;
}

int _retrieve_param(SENtral* self, enum SENtralParamGetter param, byte buffer[4]) {
    byte ack_param;

    WRITE(   REG ParamRequest,        MASK Parameter & param);
    WRITE_B( REG AlgorithmControl,    FLAG ParamTransfer);
    do READ( REG ParamAcknowledge,    &ack_param) while(ack_param != param); //todo: endless loop handling
    READ_N(  REG RetrieveParamBytes,  buffer, 4);
    WRITE(   REG ParamRequest,        FLAG ClearParam);
    UNSET_B( REG AlgorithmControl,    FLAG ParamTransfer);
    return SUCCESS;
}

int _parse_sensor_bytes(SENtral* self, byte bytes[8], float buffer[3], int scale){
    buffer[0] = bytes_to_float(bytes+0, 2, scale); //X
    buffer[1] = bytes_to_float(bytes+2, 2, scale); //Y
    buffer[2] = bytes_to_float(bytes+4, 2, scale); //Z
    return bytes_to_uint(bytes+6, 2); //Time
}

int _parse_qtern_bytes(SENtral* self, byte bytes[18], float buffer[4]){
    if(self->init.euler_mode) {
        buffer[0] = bytes_to_float(bytes+0, 4, (float)M_PI  ); //QX
        buffer[1] = bytes_to_float(bytes+4, 4, (float)M_PI_2); //QY
        buffer[2] = bytes_to_float(bytes+8, 4, (float)M_PI  ); //QZ //todo: bytes to double
        buffer[3] = 0.0; //QW
    }
    else {
        buffer[0] = bytes_to_ufloat(bytes+0,  4, 1.0); //QX
        buffer[1] = bytes_to_ufloat(bytes+4,  4, 1.0); //QY
        buffer[2] = bytes_to_ufloat(bytes+8,  4, 1.0); //QZ
        buffer[3] = bytes_to_ufloat(bytes+12, 4, 1.0); //QW
    }
    return bytes_to_uint(bytes+16, 2); //QTime
}

int _reset(SENtral* self) {
    byte status;

    READ( REG SentralStatus, &status);
    TRY(status & FLAG EEPROM ? SUCCESS : ERROR);
    while(!(status & FLAG EEUploadDone)) //todo: handle eternal loop
        READ( REG SentralStatus, &status);
    TRY(status & FLAG EEUploadError ? ERROR : SUCCESS);
}

int _sentral_interrupts_set_source(SENtral *self, bool cpu_rest, bool error, bool magn, bool accl, bool gyro, bool qtern){
    byte flags = 0;
    cpu_rest && (flags |= FLAG CPURest);
    error    && (flags |= FLAG Error);
    qtern    && (flags |= FLAG QuaternionResult);
    magn     && (flags |= FLAG MagResult);
    accl     && (flags |= FLAG AccelResult);
    gyro     && (flags |= FLAG GyroResult);

    WRITE( REG EnableEvents, flags);
    return SUCCESS;
}

int _sentral_set_data_rates(SENtral *self, byte magn_Hz, byte accl_dHz, byte gyro_dHz, byte qtern_div){
    byte rates[] = {
            magn_Hz,
            accl_dHz,
            gyro_dHz
    };
    WRITE_N( REG MagRate, rates, 3); //Todo: check what data rate was actually used, uses closest matching
    WRITE( REG QRateDivisor, qtern_div);
    return SUCCESS;
}

int _sentral_set_algorithm_mode(SENtral *self, bool euler, bool raw_data){
	byte mask = FLAG HPRoutput & FLAG RawDataEnable;
	byte flags = 0;
	euler 		&& (flags |= FLAG HPRoutput);
	raw_data 	&& (flags |= FLAG RawDataEnable);

	WRITE_M( REG AlgorithmControl, flags, mask );
    return SUCCESS;
}

int _sentral_init_properties(SENtral* self) {
    byte temp[8];
    TRY(_retrieve_param(self, GetMagnAccelRange, temp+0));
    TRY(_retrieve_param(self, GetGyroRange, temp+4));
    self->magn_range = (unsigned short)bytes_to_uint(temp+0, 2);
    self->accl_range = (unsigned short)bytes_to_uint(temp+2, 2);
    self->gyro_range = (unsigned short)bytes_to_uint(temp+4, 2);

    return SUCCESS;
}

int _sentral_start(SENtral *self){
    WRITE_B( REG HostControl, FLAG RunEnable);
    TRY(_sentral_init_properties(self));
    return SUCCESS;
}

//INIT:
SENtral* sentral_init(I2C_Interface* i2c, SENtralInit init){
    SENtral* self = malloc(sizeof(*self));
    self->i2c = i2c;
    self->init = init;

    _reset(self);

    _sentral_set_data_rates(self,
            init.data_rates.magn_Hz,
            init.data_rates.accl_dHz,
            init.data_rates.gyro_dHz,
            init.data_rates.qtern_div);
    _sentral_interrupts_set_source(self,
            init.interrupts.cpu_reset,
            init.interrupts.error,
            init.interrupts.magn,
            init.interrupts.accl,
            init.interrupts.gyro,
            init.interrupts.qtern);
    _sentral_set_algorithm_mode(self,
            init.euler_mode,
			init.raw_data);

    _sentral_start(self);

    return self;
}

//PROPERTIES:

unsigned short sentral_get_gyro_range(SENtral* self) { return self->gyro_range; }
int sentral_set_gyro_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(r, value, 2);
    TRY(_load_param(self, SetGyroRange, value));
    TRY(_retrieve_param(self, GetGyroRange, value));
    self->gyro_range = (unsigned short)bytes_to_uint(value, 2);
    return SUCCESS;
}

unsigned short sentral_get_accl_range(SENtral* self){ return self->accl_range; }
int sentral_set_accl_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(self->magn_range, value+0, 2);
    uint_to_bytes(r,                value+2, 2);
    TRY(_load_param(self, SetMagnAccelRange, value));
    TRY(_retrieve_param(self, GetMagnAccelRange, value));
    self->accl_range = (unsigned short)bytes_to_uint(value+2, 2);
    return SUCCESS;
}

unsigned short sentral_get_magn_range(SENtral* self){ return self->magn_range; }
int sentral_set_magn_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(r,                value+0, 2);
    uint_to_bytes(self->accl_range, value+2, 2);
    TRY(_load_param(self, SetMagnAccelRange, value));
    TRY(_retrieve_param(self, GetMagnAccelRange, value));
    self->magn_range = (unsigned short)bytes_to_uint(value+0, 2);
    return SUCCESS;
}

//METHODS:

int sentral_standby(SENtral* self){
    UNSET_B( REG HostControl, FLAG RunEnable);
    return SUCCESS;
}

int sentral_reset(SENtral* self){ //todo
    return SUCCESS;
}

int sentral_interrupts_clear(SENtral* self, byte cause[1]){
    READ( REG EventStatus, cause);
    return SUCCESS;
}


int sentral_measure_magn(SENtral* self, float buffer[3], int timestamp[1]){
    byte data[8];
    TRY(sentral_measure_magn_bytes(self, data));
    timestamp[0] = _parse_sensor_bytes(self, data, buffer, MAGN_FS);
    return SUCCESS;
}

int sentral_measure_accl(SENtral* self, float buffer[3], int timestamp[1]){
    byte data[8];
    TRY(sentral_measure_accl_bytes(self, data));
    timestamp[0] = _parse_sensor_bytes(self, data, buffer, ACCL_FS);
    return SUCCESS;
}

int sentral_measure_gyro(SENtral* self, float buffer[3], int timestamp[1]){
    byte data[8];
    TRY(sentral_measure_gyro_bytes(self, data))
    timestamp[0] = _parse_sensor_bytes(self, data, buffer, GYRO_FS);
    return SUCCESS;
}

int sentral_measure_qtern(SENtral* self, float buffer[4], int timestamp[1]) {
    byte data[18];
    TRY(sentral_measure_qtern_bytes(self, data));
    timestamp[0] = _parse_qtern_bytes(self, data, buffer);
    return SUCCESS;
}

int sentral_measure_all(SENtral* self, float results[13], int timestamps[4]){
    byte data[42];
    TRY(sentral_measure_all_bytes(self, data));

    timestamps[0] = _parse_qtern_bytes(self,  data+0,  results+0);
    timestamps[1] = _parse_sensor_bytes(self, data+18, results+4,  MAGN_FS);
    timestamps[2] = _parse_sensor_bytes(self, data+26, results+7,  ACCL_FS);
    timestamps[3] = _parse_sensor_bytes(self, data+34, results+10, GYRO_FS);
    return SUCCESS;
}

int sentral_measure_magn_bytes(SENtral* self, byte buffer[8]){
    READ_N( REG MX, buffer, 8);
    return SUCCESS;
}

int sentral_measure_accl_bytes(SENtral* self, byte buffer[8]){
    READ_N( REG AX, buffer, 8);
    return SUCCESS;
}

int sentral_measure_gyro_bytes(SENtral* self, byte buffer[8]){
    READ_N( REG GX, buffer, 8);
    return SUCCESS;
}

int sentral_measure_qtern_bytes(SENtral* self, byte buffer[18]){
    READ_N( REG QX, buffer, 18);
    return SUCCESS;
}

int sentral_measure_all_bytes(SENtral* self, byte buffer[42]){
    READ_N( REG Results, buffer, 42);
    return SUCCESS;
}
