

#ifndef PROJECT_SENTRAL_H
#define PROJECT_SENTRAL_H

#include "globals.h"
#include "sentral_const.h"
#include "i2c.h"

//TYPE:
typedef struct _SENtral SENtral;

//INIT:
SENtral* sensor_init(I2C_Interface* i2c);

//PROPERTIES:
/// \return  Maximum gyroscope measurement ± in degrees per second
int sensor_get_gyro_range(SENtral*);
int sensor_set_gyro_range(SENtral*, enum GyroRange r);
/// \return  Maximum accelerometer measurement ± in Gs
int sensor_get_accl_range(SENtral*);
int sensor_set_accl_range(SENtral*, enum AcclRange r);
/// \return  Maximum magnetometer measurement ± in μT
int sensor_get_magn_range(SENtral*);
int sensor_set_accl_range(SENtral*, enum MagnRange r);

//METHODS:
int sensor_interrupts_set_source(SENtral*, bool cpu_rest, bool error, bool qtern, bool magn, bool accl, bool gyro);
int sensor_set_data_rates(SENtral*, byte magn_Hz, byte accl_dHz, byte gyro_dHz, byte qtern_div);
int sensor_set_orientation_mode(SENtral*, bool hpr);

int sensor_start(SENtral*);
int sensor_standby(SENtral*);
int sensor_reset(SENtral*);

int sensor_measure_qtern(SENtral*, int buffer[4]);
int sensor_measure_gyro(SENtral*, int buffer[3]);
int sensor_measure_accl(SENtral*, int buffer[3]);
int sensor_measure_magn(SENtral*, int buffer[3]);
int sensor_measure_all(SENtral*, float buffer[10]);
int sensor_measure_all_bytes(SENtral*, byte buffer[20]);

#endif //PROJECT_SENTRAL_H
