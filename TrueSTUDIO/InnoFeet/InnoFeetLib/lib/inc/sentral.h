

#ifndef PROJECT_SENTRAL_H
#define PROJECT_SENTRAL_H

#include "globals.h"
#include "sentral_const.h"
#include "i2c.h"

//TYPE:
struct SENtralInit {
    struct {bool cpu_reset, error, magn, accl, gyro, qtern;} interrupts;
    struct {byte magn_Hz, accl_dHz, gyro_dHz, qtern_div;} data_rates;
    bool euler_mode;
};
typedef struct _SENtral SENtral;


//INIT:
SENtral* sentral_init(I2C_Interface* i2c, struct SENtralInit init);

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

int sentral_standby(SENtral*);
int sentral_reset(SENtral*); //todo

int sentral_interrupts_clear(SENtral*, byte buffer[1]);

int sentral_measure_magn(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_accl(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_gyro(SENtral*, float buffer[3], int timestamp[1]);
int sentral_measure_qtern(SENtral*, float buffer[4], int timestamp[1]);
int sentral_measure_all(SENtral*, float results[13], int timestamps[4]);

int sentral_measure_magn_bytes(SENtral*, byte buffer[8]);
int sentral_measure_accl_bytes(SENtral*, byte buffer[8]);
int sentral_measure_gyro_bytes(SENtral*, byte buffer[8]);
int sentral_measure_qtern_bytes(SENtral*, byte buffer[18]);
int sentral_measure_all_bytes(SENtral*, byte buffer[42]);

#endif //PROJECT_SENTRAL_H
