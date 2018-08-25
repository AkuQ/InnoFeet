

#ifndef PROJECT_SENTRAL_H
#define PROJECT_SENTRAL_H

#include "globals.h"
#include "sentral_const.h"
#include "i2c.h"

//TYPE:

/// \brief Initialization parameters for sentral_init().
struct SENtralInit {
    /// \brief Enables interrupt sources.
    struct {bool cpu_reset, error, magn, accl, gyro, qtern;} interrupts;
    /// \brief Sets sensor data rates.
    /// \attention Note that magnetometer rate is given in hertz but accelerometer and gyroscope are in decahertz.
    /// Quaternion rate is given as divisor of gyroscope rate (e.g. if gyroscope rate is 10 dHz and quaternion should
    /// be updated at 50Hz, set qtern_div to 2). Quaternion divisor 0 will be interpreted as 1.
    /// \warning Sensors might not not support all data rates and actual values should be checked after initialization.
    /// \note Setting data rate to 0 will disable that sensor, but this will also disable SENtral's calibration algorithms.
    struct {byte magn_Hz, accl_dHz, gyro_dHz, qtern_div;} data_rates;
    bool euler_mode; /**< When set to TRUE sentral returns orientation in heading-pitch-roll. Set to FALSE for quaternion.*/
} extern const SENtralInitDefaults;
typedef struct _SENtral SENtral;


//INIT:

SENtral* sentral_init(I2C_Interface* i2c, struct SENtralInit init);


//PROPERTIES:

/// \return Maximum magnetometer measurement ± in μT.
unsigned short sentral_get_magn_range(SENtral*);

/// \return Maximum accelerometer measurement ± in Gs.
unsigned short sentral_get_accl_range(SENtral*);

/// \return  Maximum gyroscope measurement ± in degrees per second.
unsigned short sentral_get_gyro_range(SENtral*);

/// \warning Actual value should be checked with sentral_get_magn_range() afterwards, sensor might not support desired value.
/// \param r Maximum magnetometer measurement ± in μT.
int sentral_set_magn_range(SENtral*, unsigned short r);

/// \warning Actual value should be checked with sentral_get_accl_range() afterwards, sensor might not support desired value.
/// \param r Maximum accelerometer measurement ± in Gs.
int sentral_set_accl_range(SENtral*, unsigned short r);

/// \warning Actual value should be checked with sentral_get_gyro_range() afterwards, sensor might not support desired value.
/// \param r Maximum gyroscope measurement ± in degrees per second.
int sentral_set_gyro_range(SENtral*, unsigned short r);


//METHODS:

int sentral_standby(SENtral*);
int sentral_reset(SENtral*); //todo

int sentral_interrupts_clear(SENtral*, byte buffer[1]);

/// \param buffer Outputs X, Y, Z, W.<br>(W always 0 if sensor in euler mode.)
/// \param timestamp Outputs timestamp (0 – 2048 msec).
int sentral_measure_qtern(SENtral*, float buffer[4], int timestamp[1]);

/// \param buffer Outputs X, Y, Z.
/// \param timestamp Outputs timestamp (0 – 2048 msec).
int sentral_measure_magn(SENtral*, float buffer[3], int timestamp[1]);

/// \param buffer Outputs X, Y, Z.
/// \param timestamp Outputs timestamp (0 – 2048 msec).
int sentral_measure_accl(SENtral*, float buffer[3], int timestamp[1]);

/// \param buffer Outputs X, Y, Z.
/// \param timestamp Outputs timestamp (0 – 2048 msec).
int sentral_measure_gyro(SENtral*, float buffer[3], int timestamp[1]);

/// \param buffer Outputs:<ol start="0"><li>Qtern X</li><li>Qtern Y</li><li>Qtern Z</li>
/// <li>Qtern W (always 0 if sensor in euler mode)</li><li>Magn X</li><li>Magn Y</li><li>Magn Z</li>
/// <li>Accl X</li><li>Accl Y</li><li>Accl Z</li><li>Gyro X</li><li>Gyro Y</li><li>Gyro Z</li></ol>
/// \param timestamp Outputs timestamps for Qtern, Magn, Accl, Gyro.<br>().
int sentral_measure_all(SENtral*, float results[13], int timestamps[4]);


/// Scale X, Y, Z, W to range 0.0 – 1.0 if in quaternion mode.
/// In euler mode X, Y, Z are scaled against π, π/2, π respectively (W is ignored). \note Little-endian byte order.
/// \param buffer Outputs: <ul style="list-style-type:none"><li>[0:3] – X</li><li>[4:7] – Y</li><li> [8:11] – Z</li>
/// <li>[12:15] – W</li><li>[16:17] – Timestamp</li></ul>
int sentral_measure_qtern_bytes(SENtral*, byte buffer[18]);
/// Scale X, Y, Z against SENtral_MAGN_FS. \note Little-endian byte order.
/// \param buffer Outputs: <ul style="list-style-type:none"><li>[0:1] – X</li><li>[2:3] – Y</li><li>[4:5] – Z</li>
/// <li>[6:7] – Timestamp</li></ul>
int sentral_measure_magn_bytes(SENtral*, byte buffer[8]);
/// Scale X, Y, Z against SENtral_ACCL_FS.\note Little-endian byte order.
/// \param buffer Outputs: <ul style="list-style-type:none"><li>[0:1] – X</li><li>[2:3] – Y</li><li>[4:5] – Z</li>
/// <li>[6:7] – Timestamp</li></ul>
int sentral_measure_accl_bytes(SENtral*, byte buffer[8]);
/// Scale X, Y, Z against SENtral_ACCL_FS. \note Little-endian byte order.
/// \param buffer Outputs: <ul style="list-style-type:none"><li>[0:1] – X</li><li>[2:3] – Y</li><li>[4:5] – Z</li><li>[6:7] – Timestamp</li></ul>
int sentral_measure_gyro_bytes(SENtral*, byte buffer[8]);
/// Scale quaternion X, Y, Z, W to range 0.0 – 1.0 if in quaternion mode.In euler mode X, Y, Z are scaled against
/// π, π/2, π respectively (W is ignored).<br> Magnetometer, accelerometer and gyroscope X, Y, Z are scaled against
/// SENtral_MAGN_FS, SENtral_ACCL_FS, SENtral_GYRO_FS. \note Little-endian byte order.
/// \param buffer Outputs: <ul style="list-style-type:none"><li>[0:3] – QX</li><li>[4:7] – QY</li><li>[8:11] – QZ</li>
/// <li>[12:15] – QW</li><li>[16:17] – QTime</li>
/// <li>[18:19] – MX</li><li>[20:21] – MY</li><li>[22:23] – MZ</li><li>[24:25] – MTime</li>
/// <li>[26:27] – AX</li><li>[28:29] – AY</li><li>[30:31] – AZ</li><li>[32:33] – ATime</li>
/// <li>[34:35] – GX</li><li>[36:37] – GY</li><li>[38:39] – GZ</li><li>[40:41] – GTime</li></ul>
int sentral_measure_all_bytes(SENtral*, byte buffer[42]);

#endif //PROJECT_SENTRAL_H
