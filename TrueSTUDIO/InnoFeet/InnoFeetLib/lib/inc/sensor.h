
#ifndef INNOFEET_SENSOR_H
#define INNOFEET_SENSOR_H

#include "i2c.h"
#include "sensor_const.h"


//TYPE:
typedef struct _Sensor Sensor;

//INIT:
Sensor* sensor_init(I2C_Interface* i2c);


//PROPERTIES:
/// \return  Maximum gyroscope measurement ± in degrees per second
int sensor_get_gyro_range(Sensor*);
int sensor_set_gyro_range(Sensor*, enum GyroRange r);
/// \return  Maximum accelerometer measurement ± in Gs
int sensor_get_accl_range(Sensor*);
int sensor_set_accl_range(Sensor*, enum AcclRange r);
/// \return  Maximum magnetometer measurement ± in μT
int sensor_get_magn_range(Sensor*);

/// \return Wake-on motion threshold in mG
int sensor_get_wake_on_motion_threshold(Sensor *);
/// \arg threshold 0 ~ 1020 mG
int sensor_set_wake_on_motion_threshold(Sensor *, int threshold);
//todo update docs ^


//METHODS:

int sensor_reset(Sensor*);

int sensor_interrupts_set_pin(Sensor*, bool active_low, bool open_drain, enum ClearInterruptOn clear_on);
int sensor_interrupts_set_source(Sensor*, bool on_data_ready, bool on_motion, bool on_fifo_overflow);

/** Clears sensor's interrupt if interrupt status has been set to clear on reading interrupt status,
 * See function sensor_interrupts_set_pin().
 * \param self Sensor instance
 * \param cause (optional) Return buffer for getting the cause of interrupt, compare bitwise against enum InterruptCauses.
 * \return Success/Error
 */
#define sensor_interrupts_clear(...) OVERLOAD(_sensor_interrupts_clear, __VA_ARGS__)
int _sensor_interrupts_clear1(Sensor*);
int _sensor_interrupts_clear2(Sensor*, byte cause[1]);

int sensor_measure_temp(Sensor*, int buffer[1]);
int sensor_measure_gyro(Sensor*, int buffer[3]);
int sensor_measure_accl(Sensor*, int buffer[3]);
int sensor_measure_magn(Sensor*, int buffer[3]);
int sensor_measure_all(Sensor*, float buffer[10]);
int sensor_measure_all_bytes(Sensor*, byte buffer[20]);



#endif //INNOFEET_SENSOR_H
