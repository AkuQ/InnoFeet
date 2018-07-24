

#include <stdlib.h>
#include <math.h>

#include "sentral.h"
#include "byte_conversion.h"


typedef struct _SENtral {
    I2C_Interface * i2c;
    int state;
    bool euler;
    unsigned short gyro_range;
    unsigned short accl_range;
    unsigned short magn_range;
   
} _SENtral;

//State:
#define UNKNOWN 0
#define INITIALIZED 1
#define NORMAL 2
#define STANDBY 3
#define PASS_THROUGH 4

#define MAGN_FS SENtral_MAGN_FS
#define ACCL_FS SENtral_ACCL_FS
#define GYRO_FS SENtral_GYRO_FS

#define REG SENtralRegister.
#define FLAG SENtralBitFlags.
#define MASK SENtralBitFlags.

#define WRITE(a, b) TRY(i2c_write_byte(self->i2c, a, b))
#define WRITE_N(a, b, c) TRY(i2c_write_bytes(self->i2c, a, b, c))
#define READ(a, b) TRY(i2c_read_byte(self->i2c, a, b))
#define READ_N(a, b, c) TRY(i2c_read_bytes(self->i2c, a, b, c))
#define WRITE_B(a, b) TRY(i2c_write_bits(self->i2c, a, b))
#define UNSET_B(a, b) TRY(i2c_write_bits(self->i2c, a, 0, b))

//PRIVATE:

int _load_param(SENtral* self, enum SENtralParamSetter param, byte value[4]) {
    byte ack_param;

    WRITE_N(   REG LoadParamBytes,      value, 4); //todo: can attach below ParamRequest to this multi-byte write ()
    WRITE(     REG ParamRequest,        FLAG LoadParam | MASK Parameter & param);
    WRITE_B(   REG AlgorithmControl,    FLAG ParamTransfer);
    READ(      REG ParamAcknowledge,    &ack_param);
    WRITE(     REG ParamRequest,        FLAG ClearParam);
    UNSET_B(   REG AlgorithmControl,    FLAG ParamTransfer);
    if(ack_param != param)
        return ERROR; //todo: more descriptive error
    return SUCCESS;
}

int _retrieve_param(SENtral* self, enum SENtralParamGetter param, byte buffer[4]) {
    byte ack_param;

    WRITE(   REG ParamRequest,        MASK Parameter & param);
    WRITE_B( REG AlgorithmControl,    FLAG ParamTransfer);
    do READ( REG ParamAcknowledge,    &ack_param) while(ack_param != param); //todo: endless loop handling
    READ_N(  REG RetrieveParamBytes,  buffer, 4);
    WRITE(   REG ParamRequest,        FLAG ClearParam);
    UNSET_B( REG AlgorithmControl,  FLAG ParamTransfer);
    return SUCCESS;
}

int _retrieve_params(SENtral* self, enum SENtralParamGetter * param, int n, byte* buffer) {
    byte ack_param;

    for (int i = 0; i < n; i++) {
        WRITE(   REG ParamRequest,       MASK Parameter & param[i]);
        if(i == 0) WRITE_B( REG AlgorithmControl, FLAG ParamTransfer);
        do READ( REG ParamAcknowledge,   &ack_param) while(ack_param != param[i]); //todo: endless loop handling
        READ_N(  REG RetrieveParamBytes, buffer+(i*4), 4);
    }
    WRITE(   REG ParamRequest,     FLAG ClearParam);
    UNSET_B( REG AlgorithmControl, FLAG ParamTransfer);
    return SUCCESS;
}

int _init_properties(SENtral* self) {
    byte temp[8];
    TRY(_retrieve_param(self, GetMagnAccelRange, temp+0));
    TRY(_retrieve_param(self, GetGyrolRange, temp+4));
    self->magn_range = (unsigned short)bytes_to_uint(temp+0, 2);
    self->accl_range = (unsigned short)bytes_to_uint(temp+2, 2);
    self->gyro_range = (unsigned short)bytes_to_uint(temp+4, 2);

    READ( REG AlgorithmControl, temp);
    self->euler = (bool)(temp[0] & FLAG HPRoutput);

    return SUCCESS;
}

int _reset(SENtral* self) {
    byte status;

    READ( REG SentralStatus, &status);
    TRY((status & FLAG EEPROM) != 0);
    while(status & FLAG EEUploadDone)
        READ( REG SentralStatus, &status);
    TRY((status & FLAG EEUploadError) == 0);
    self->state = INITIALIZED;
    _init_properties(self);
}


//INIT:
SENtral* sentral_init(I2C_Interface* i2c){
    SENtral* sentral = malloc(sizeof(*sentral));
    sentral->i2c = i2c;
    sentral->state = UNKNOWN;
    _reset(sentral);
    return sentral;
}

//PROPERTIES:
/// \return  Maximum gyroscope measurement ± in degrees per second
unsigned short sentral_get_gyro_range(SENtral* self) { return self->gyro_range; }
int sentral_set_gyro_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(r, value, 2);
    TRY(_load_param(self, SetGyroRange, value));
    self->gyro_range = r;
    return SUCCESS;
}
/// \return  Maximum accelerometer measurement ± in Gs
unsigned short sentral_get_accl_range(SENtral* self){ return self->accl_range; }
int sentral_set_accl_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(self->magn_range, value+0, 2);
    uint_to_bytes(r, value+2, 2);
    TRY(_load_param(self, SetMagnAccelRange, value));
    self->accl_range = r;
    return SUCCESS;
}
/// \return  Maximum magnetometer measurement ± in μT
unsigned short sentral_get_magn_range(SENtral* self){ return self->magn_range; }
int sentral_set_mang_range(SENtral* self, unsigned short r){
    byte value[4] = {0, 0, 0, 0};
    uint_to_bytes(r, value+0, 2);
    uint_to_bytes(self->accl_range, value+2, 2);
    TRY(_load_param(self, SetMagnAccelRange, value));
    self->magn_range = r;
    return SUCCESS;
}

//METHODS:
int sentral_interrupts_set_source(SENtral* self, bool cpu_rest, bool error, bool qtern, bool magn, bool accl, bool gyro){
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

int sentral_set_data_rates(SENtral* self, byte magn_Hz, byte accl_dHz, byte gyro_dHz, byte qtern_div){
    byte rates[] = {
            magn_Hz,
            accl_dHz,
            gyro_dHz
    };
    WRITE_N( REG MagRate, rates, 3); //Todo: check what data rate was actually used, uses closest matching
    WRITE( REG QRateDivisor, qtern_div);
    return SUCCESS;
}
int sentral_set_orientation_mode(SENtral* self, bool euler){
    byte flag = 0;
    euler && (flag |= FLAG HPRoutput);
    WRITE_B( REG AlgorithmControl, flag);
    self->euler = euler;
    return SUCCESS;
}
bool sentral_euler_mode(SENtral* self) { return self->euler; }



int sentral_start(SENtral* self){
    WRITE_B( REG HostControl, FLAG RunEnable);
    return SUCCESS;
}
int sentral_standby(SENtral* self){
    UNSET_B( REG HostControl, FLAG RunEnable);
    return SUCCESS;
}
int sentral_reset(SENtral* self){
    self->state = UNKNOWN;

}

int _sentral_interrupts_clear1(SENtral* self) {
    byte temp;
    return _sentral_interrupts_clear2(self, &temp);
}
int _sentral_interrupts_clear2(SENtral* self, byte cause[1]){
    READ( REG EventStatus, cause);
    return SUCCESS;
}

int sentral_measure_qtern(SENtral* self, float buffer[4], int timestamp[1]){}
int sentral_measure_gyro(SENtral* self, float buffer[3], int timestamp[1]){}
int sentral_measure_accl(SENtral* self, float buffer[3], int timestamp[1]){}
int sentral_measure_magn(SENtral* self, float buffer[3], int timestamp[1]){}
int sentral_measure_all(SENtral* self, float results[13], int timestamps[4]){
    byte data[42];
    TRY(sentral_measure_all_bytes(self, data));

    if(self->euler) {
        results[0] = bytes_to_float(data+0, 4, (float)M_PI  ); //QX
        results[1] = bytes_to_float(data+4, 4, (float)M_PI_2); //QY
        results[2] = bytes_to_float(data+8, 4, (float)M_PI  ); //QZ //todo: bytes to double
        results[3] = 0.0; //QW
    }
    else {
        results[0] = bytes_to_ufloat(data+0,  4, 1.0); //QX
        results[1] = bytes_to_ufloat(data+4,  4, 1.0); //QY
        results[2] = bytes_to_ufloat(data+8,  4, 1.0); //QZ
        results[3] = bytes_to_ufloat(data+12, 4, 1.0); //QW
    }
    timestamps[0] = bytes_to_uint(data+16, 2); //QTime

    results[4] = bytes_to_float(data+18, 2, MAGN_FS); //MX
    results[5] = bytes_to_float(data+20, 2, MAGN_FS); //MY
    results[6] = bytes_to_float(data+22, 2, MAGN_FS); //MZ
    timestamps[1] = bytes_to_uint(data+24, 2); //MTime

    results[7] = bytes_to_float(data+26, 2, ACCL_FS); //AX
    results[8] = bytes_to_float(data+28, 2, ACCL_FS); //AY
    results[9] = bytes_to_float(data+30, 2, ACCL_FS); //AZ
    timestamps[2] = bytes_to_uint(data+32, 2); //ATime

    results[10] = bytes_to_float(data+34, 2, GYRO_FS); //GX
    results[11] = bytes_to_float(data+36, 2, GYRO_FS); //GY
    results[12] = bytes_to_float(data+38, 2, GYRO_FS); //GZ
    timestamps[3] = bytes_to_uint(data+40, 2); //GTime

    return SUCCESS;
}
int sentral_measure_all_bytes(SENtral* self, byte buffer[42]){
    READ_N( REG Results, buffer, 30);
    return SUCCESS;
}
