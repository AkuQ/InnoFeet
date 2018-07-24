

#ifndef PROJECT_SENTRAL_H
#define PROJECT_SENTRAL_H

#include "globals.h"
#include "sentral_const.h"
#include "i2c.h"

//TYPE:
typedef struct _SENtral SENtral;

//INIT:
SENtral* sentral_init(I2C_Interface* i2c);

//PROPERTIES:
/// \return  Maximum gyroscope measurement ± in degrees per second
unsigned short sentral_get_gyro_range(SENtral*);
int sentral_set_gyro_range(SENtral*, unsigned short r);
/// \return  Maximum accelerometer measurement ± in Gs
unsigned short sentral_get_accl_range(SENtral*);
int sentral_set_accl_range(SENtral*, unsigned short r);
/// \return  Maximum magnetometer measurement ± in μT
unsigned short sentral_get_magn_range(SENtral*);
int sentral_set_magn_range(SENtral*, unsigned short r);

//METHODS:
int sentral_interrupts_set_source(SENtral*, bool cpu_rest, bool error, bool qtern, bool magn, bool accl, bool gyro);
int sentral_set_data_rates(SENtral*, byte magn_Hz, byte accl_dHz, byte gyro_dHz, byte qtern_div);
int sentral_set_orientation_mode(SENtral*, bool hpr);
bool sentral_euler_mode(SENtral*);

int sentral_start(SENtral*);
int sentral_standby(SENtral*);
int sentral_reset(SENtral*);

#define sentral_interrupts_clear(...) OVERLOAD(_sentral_interrupts_clear, __VA_ARGS__)
int _sentral_interrupts_clear1(SENtral*);
int _sentral_interrupts_clear2(SENtral*, byte cause[1]);

int sentral_measure_qtern(SENtral*, float buffer[4], int timestamp[1]);
int sentral_measure_gyro(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_accl(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_magn(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_all(SENtral*, float results[13], int timestamps[4]);
int sentral_measure_all_bytes(SENtral*, byte buffer[42]);

#endif //PROJECT_SENTRAL_H
