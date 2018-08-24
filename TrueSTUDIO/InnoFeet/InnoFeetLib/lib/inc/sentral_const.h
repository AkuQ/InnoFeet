

#ifndef PROJECT_SENTRAL_CONST_H
#define PROJECT_SENTRAL_CONST_H

#include "globals.h"

struct _SENtralRegister {
// Results:
    byte Results; /**< 0x00<br>All results in orderd Quaternion X/Y/Z/W/T, Mag/Accl/Gyro (X,Y,Z,T).
            <br>Quaternion results are 4 bytes, other results and timestamps 2;; 30;; */
    byte QX; /**< 0x00<br>Normalized Quaternion – X, or Heading; 	float32; 4; 0.0-1.0 or ±PI; */
    byte QY; /**< 0x04<br>Normalized Quaternion – Y, or Pitch; 		float32; 4; 0.0-1.0 or ±PI/2; */
    byte QZ; /**< 0x08<br>Normalized Quaternion – Z, or Roll; 		float32; 4; 0.0-1.0 or ±PI; */
    byte QW; /**< 0x0C<br>Normalized Quaternion – W, or 0.0; 		float32; 4; 0.0-1.0; */
    byte QTime; /**< 0x10<br>Quaternion Data Timestamp;				Uint16; 2; 0 – 2048 msec; */
    byte MX; /**< 0x12<br>Magnetic Field – X Axis, or Raw Mag Data; int16; 2; ±1000 µT when scaled; */
    byte MY; /**< 0x14<br>Magnetic Field – Y Axis, or Raw Mag Data; int16; 2; ±1000 µT when scaled; */
    byte MZ; /**< 0x16<br>Magnetic Field – Z Axis, or Raw Mag Data; int16; 2; ±1000 µT when scaled; */
    byte MTime; /**< 0x18<br>Magnetometer Interrupt Timestamp;			Uint16; 2; 0 – 2048 msec; */
    byte AX; /**< 0x1A<br>Linear Acceleration – X Axis, or Raw Accel Data; int16; 2; ±16 g when scaled; */
    byte AY; /**< 0x1C<br>Linear Acceleration – Y Axis, or Raw Accel Data; int16; 2; ±16 g when scaled; */
    byte AZ; /**< 0x1E<br>Linear Acceleration – Z Axis, or Raw Accel Data; int16; 2; ±16 g when scaled; */
    byte ATime; /**< 0x20<br>Accelerometer Interrupt Timestamp;		Uint16; 2; 0 – 2048 msec; */
    byte GX; /**< 0x22<br>Rotational Velocity – X Axis, or Raw Gyro Data; Int16; 2; ±5000°/s when scaled; */
    byte GY; /**< 0x24<br>Rotational Velocity – Y Axis, or Raw Gyro Data; Int16; 2; ±5000°/s when scaled; */
    byte GZ; /**< 0x26<br>Rotational Velocity – Z Axis, or Raw Gyro Data; Int16; 2; ±5000°/s when scaled; */
    byte GTime; /**< 0x28<br>Gyroscope Interrupt Timestamp;			Uint16; 2; 0.0 – 2048 msec; */

// Status:
    byte SentralStatus; /**< 0x37<br>; [7:0]; 1;
			<br>[0] EEPROM. 1 = EEPROM detected
			<br>[1] EEUploadDone. 1 = EEPROM upload completed
			<br>[2] EEUploadError. 1 = Calculated CRC of EEPROM is incorrect. Only valid when EEUploadDone = 1.
			<br>[3] Idle. 1 = Device in Unprogrammed or Initialized state.
			<br>[4] NoEEPROM. 1 = No EEPROM detected.
    ;*/
    byte AlgorithmStatus; /**< 0x38<br>Is SENtral in Standby State;		; 1; Bit[0] 1 or 0; */
    byte PassThroughStatus; /**< 0x9E<br>Is SENtral in Pass-Through State;	; 1; Bit[0] 1 or 0; */
    byte EventStatus; /**< 0x35<br>1 indicates a new event has been generated.; [7:0]; 1;
			<br>[0] CPURest. SENtral Configuration File needs uploading. See Section 4.1.
			<br>[1] Error.
			<br>[2] QuaternionResult.
			<br>[3] MagResult.
			<br>[4] AccelResult.
			<br>[5] GyroResult.
	;*/
    byte RAMVersion; /**< 0x72<br>Unexpected Configuration File revision level; Uint16; 2;; The revision number. See enum SENtralConfigVersion. */

// Errors:
    byte SensorStatus; /**< 0x36<br>; [7:0]; 1;
			<br>[0] MagNACK. 1 = NACK from magnetometer.
			<br>[1] AccelNACK. 1 = NACK from accelerometer.
			<br>[2] GyroNACK. 1 = NACK from gyroscope.
			<br>[4] MagDeviceIDErr. 1 = Unexpected DeviceID from magnetometer.
			<br>[5] AccelDeviceIDErr. 1 = Unexpected DeviceID from accelerometer.
			<br>[6] GyroDeviceIDErr. 1 = Unexpected DeviceID from gyroscope.
	;*/
    byte ErrorRegister; /**< 0x50<br>Non-zero value indicated an error. See enum SENtralErrors; [7:0]; 1;; */


// Control:
    byte HostControl; /**< 0x34<br>; [7:0]; 1;
			[0] 1 = RunEnable. 0 = Enable Initialized State (Standby State generally is preferred since enabling Initialized State resets the SENtral algorithm, including calibration data.)
	;*/
    byte PassThroughControl; /**< 0xA0<br>Set SENtral Pass-Through State;	; 1; Bit[0] 1 or 0; */
    byte ResetReq; /**< 0x9B<br>; [7:0]; [0] ResetRequest. 1 = Emulate a hard power down/power up; */
// (Initialization):
    byte AlgorithmControl; /**< 0x54<br>; 1;
			<br>[[0] StandbyEnable. 1 = Enable Standby state
			<br>[[1] RawDataEnable. 1 = Raw data provided in MX, MY, MZ, AX, AY, AZ, GX, GY, & GZ. 0 = Scaled sensor data.
			<br>[[2] HPRoutput. 1 = Heading, pitch, and roll output in QX, QY, & QZ. QW = 0.0. 0 = Quaternion outputs.
			<br>[[7] ParamTransfer. 1 = Enable Parameter Transfer.
	;*/
    byte EnableEvents; /**< 0x33<br>1 indicates an interrupt to the host will be generated for the event; [7:0]; 1;
			<br>[0] CPUReset. Non-maskable.
			<br>[1] Error.
			<br>[2] QuaternionResult.
			<br>[3] MagResult.
			<br>[4] AccelResult.
			<br>[5] GyroResult.
	;*/
    byte MagRate; /**< 0x55<br>Requested magnetometer output data rate. Set to 0 to disable magnetometer.;; 1; Hz; */
    byte AccelRate; /**< 0x56<br>Requested accelerometer output data rate divided by 10. Set to 0 to disable accelerometer.;; 1; Hz/10; */
    byte GyroRate; /**< 0x57<br>Requested gyroscope output data rate divided by 10. Set to 0 to disable gyroscope.;; 1; Hz/10; */
    byte QRateDivisor; /**< 0x32<br>Along with GyroRate, establishes output data rate for quaternion data. Default 0, interpreted as 1 (use GyroRate);; 1; GyroRate / QRateDivisor; */

// Parameter Transfer Registers: AlgorithmControl[7] bit set to 1.
    byte ParamRequest; /**< 0x64<br>; [7:0]; //todo: reference AlgorithmControl setup
			[0:6] Parameter number to be uploaded or retrieved.
			[7] Load/Save bit. 1 = Load, 0 = Save.
	;*/
    byte ParamAcknowledge; /**< 0x3A<br>; [7:0]; //todo: reference AlgorithmControl setup
			[0:6] provide the parameter number that was uploaded or retrieved.
			[7] Load/Retrieve bit. 1 = Load, 0 = Retrieve.
	;*/
    byte LoadParamBytes; /**< 0x60<br>Parameter value to be loaded; float32; 4;; //todo: reference AlgorithmControl setup */
    byte LoadParamByte0; /**< 0x60<br>Parameter value to be loaded – LSB; float8; 1;; */
    byte LoadParamByte1; /**< 0x61<br>Parameter value to be loaded – LSB + 1; float8; 1;; */
    byte LoadParamByte2; /**< 0x62<br>Parameter value to be loaded – MSB – 1; float8; 1;; */
    byte LoadParamByte3; /**< 0x63<br>Parameter value to be loaded – MSB; float8; 1;; */
    byte RetrieveParamBytes; /**< 0x3B<br>Parameter value read from Sentral; float32; 4;; //todo: reference AlgorithmControl setup */
    byte RetrieveParamByte0; /**< 0x3B<br>Parameter value read from Sentral – LSB; float8; 1;; */
    byte RetrieveParamByte1; /**< 0x3C<br>Parameter value read from Sentral – LSB + 1; float8; 1;; */
    byte RetrieveParamByte2; /**< 0x3D<br>Parameter value read from Sentral – MSB – 1; float8; 1;; */
    byte RetrieveParamByte3; /**< 0x3E<br>Parameter value read from Sentral – MSB; float8; 1;; */
} extern const SENtralRegister;


struct _SENtralBitFlags {
    //SentralStatus:
    byte EEPROM; /**< SentralStatus[0]<br>EEPROM detected */
    byte EEUploadDone; /**< SentralStatus[1]<br>EEPROM upload completed */
    byte EEUploadError; /**< SentralStatus[2]<br>Calculated CRC of EEPROM is incorrect. Only valid when EEUploadDone = 1. */
    byte Idle; /**< SentralStatus[3]<br>Device in Unprogrammed or Initialized state. */
    byte NoEEPROM; /**< SentralStatus[4]<br>No EEPROM detected. */
    //EventStatus:
    byte CPURest; /**< EventStatus[0]<br>EnableEvents[0]<br>SENtral Configuration File needs uploading. See Section 4.1. //todo: reference Section */
    byte Error; /**< EventStatus[1]<br>EnableEvents[1] */
    byte QuaternionResult; /**< EventStatus[2]<br>EnableEvents[2] */
    byte MagResult; /**< EventStatus[3]<br>EnableEvents[3] */
    byte AccelResult; /**< EventStatus[4]<br>EnableEvents[4] */
    byte GyroResult; /**< EventStatus[5]<br>EnableEvents[5] */
    //HostControl:
    byte RunEnable; /**< HostControl[0]<br>Enable Initialized State (Standby State generally is preferred since
            enabling Initialized State resets the SENtral algorithm, including calibration data.) */
    //AlgorithmControl:
    byte StandbyEnable; /**< AlgorithmControl[0]<br>Enable Standby state */
    byte RawDataEnable; /**< AlgorithmControl[1]<br>1 = Raw data provided in MX, MY, MZ, AX, AY, AZ, GX, GY, & GZ.<br>0 = Scaled sensor data. */
    byte HPRoutput; /**< AlgorithmControl[2]<br>1 = Heading, pitch, and roll output in QX, QY, & QZ. QW = 0.0.<br>0 = Quaternion outputs. */
    byte ParamTransfer; /**< AlgorithmControl[7]<br>Enable Parameter Transfer. */
    //ParamRequest & ParamAcknowledge:
    byte Parameter; /**< ParamRequest[0:6]<br>ParamAcknowledge[0:6]<br>Mask for parameter number to be uploaded or retrieved. */
    byte LoadParam; /**< ParamRequest[7]<br> ParamAcknowledge[7]<br>Load or Save bit.<br>1 = Load<br>0 = Save. */
    byte ClearParam; /**< ParamRequest[0:7]<br> ParamAcknowledge[0:7]<br>Clear transferred parameter by setting all bits to 0.*/
} extern const SENtralBitFlags;


enum SENtralErrors {
    NoError 			= (byte)0x00, /**< No error. */
    InvalidSampleRate	= (byte)0x80, /**< Invalid sample rate selected. Check sensor rate settings. */
    MathError			= (byte)0x30, /**< Mathematical Error. Check for software updates. */
    MagnInitFailed		= (byte)0x21, /**< Magnetometer initialization failed. This error can be caused by a wrong driver, physically bad sensor connection, or incorrect I2C device address in the driver. */
    AcclInitFailed		= (byte)0x22, /**< Accelerometer initialization failed. This error can be caused by a wrong driver, physically bad sensor connection, or incorrect I2C device address in the driver. */
    GyroInitFailed		= (byte)0x24, /**< Gyroscope initialization failed. This error can be caused by a wrong driver, physically bad sensor connection, or incorrect I2C device address in the driver. */
    MagnRateFailure 	= (byte)0x11, /**< Magnetometer rate failure This error indicates the given sensor is unreliable and has stopped producing data. */
    AcclRateFailure 	= (byte)0x12, /**< Accelerometer rate failure This error indicates the given sensor is unreliable and has stopped producing data. */
    GyroRateFailure 	= (byte)0x14 /**< Gyroscope rate failure This error indicates the given sensor is unreliable and has stopped producing data. */
};

enum SENtralConfigVersion {
    ConfigVer_1_0 = 0x0C04,
    ConfigVer_1_1 = 0x0CD5,
    ConfigVer_1_2 = 0x0E02
};

enum SENtralParamGetter {
    GetMagnAccelRange 	= (byte)0x4A, /**< RetrieveParamBytes:
			<br>[0:1] Magnetometer range (µT)
			<br>[2:3] Accelerometer range (g)
	*/
    GetGyroRange 		= (byte)0x4B, /**< RetrieveParamBytes:<br>[0:1] Gyroscope range (dps) */
    GetMagnAcclDriverID = (byte)0x4D, /**< RetrieveParamBytes:
			<br>[0] Magnetometer driver revision
			<br>[1] Magnetometer driver ID
			<br>[2] Accelerometer driver revision
			<br>[3] Accelerometer driver ID
	*/
    GetGyroDriveID 		= (byte)0x4E, /**< RetrieveParamBytes:
			<br>[0] Gyroscope driver revision
			<br>[1] Gyroscope driver ID
	*/
    GetAlgorithmID 		= (byte)0x50 /**< RetrieveParamBytes:
			<br>[0] Algorithm minor revision
			<br>[1] Algorithm major revision
	*/
};

enum SENtralParamSetter {
    SetMagnAccelRange 	= (byte)0xCA, /**< LoadParamBytes:
			<br>[[0:1] Magnetometer range (µT)
			<br>[[2:3] Accelerometer range (g)
	*/
    SetGyroRange 		= (byte)0xCB, /**< LoadParamBytes<br>[0:1] Gyroscope range (dps) */
};

// Other constants:
#define SENtral_TIMESTAMP_RESOLUTION_Hz 32000
#define SENtral_DEVICE_ADDRESS 0x28
#define SENtral_MAGN_FS 1000
#define SENtral_ACCL_FS 16
#define SENtral_GYRO_FS 5000

#endif //PROJECT_SENTRAL_CONST_H
