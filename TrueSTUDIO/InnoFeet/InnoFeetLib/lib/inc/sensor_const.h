
#ifndef INNOFEET_SENSOR_REG_H
#define INNOFEET_SENSOR_REG_H

#include "globals.h"


// Enumerations:
enum GyroRange {DPS_250 = (byte)0b00000, DPS_500 = (byte)0b01000, DPS_1000 = (byte)0b10000, DPS_2000 = (byte)0b11000, DPS_MAX = DPS_2000};
enum AcclRange {G_2 = (byte)0b00000, G_4 = (byte)0b01000, G_8 = (byte)0b10000, G_16 = (byte)0b11000, G_MAX = G_16};
enum ClearInterruptOn{READ_ANY = (byte)0b01, READ_INT_STATUS = (byte)0b00, PULSE = (byte)0b11}; //Note: Do not chance bit-masks, READ_ANY & PULSE must evaluate true, etc.

static int GyroRangeAsInt(enum GyroRange en) {
    switch (en) {
        case DPS_250: return 250; case DPS_500: return 500; case DPS_1000: return 1000; case DPS_2000: return 2000;
        default: return -1;
    }
}
static int AcclRangeAsInt(enum AcclRange en){
    switch (en) {
        case G_2: return 2; case G_4: return 4; case G_8: return 8; case G_16: return 16;
        default: return -1;
    }
}

//Registers and bit-flags:

#define SENSOR_DEV_ADDR 0x68

#define SMPDIV 0x19u
//define CONFIG 0x1Au
struct CONFIG {
    byte REG;
    byte FIFO_MODE;
    struct {
        byte DISABLE, TEMP_OUT_L, GYRO_XOUT_L, GYRO_YOUT_L, GYRO_ZOUT_L, ACCEL_XOUT_L, ACCEL_YOUT_L, ACCEL_ZOUT_L;
    } EXT_SYNC_SET; /**<
            * Enables the FSYNC pin data to be sampled.<br><br>Fsync will be latched to capture short strobes. This will
            * be done such that if Fsync toggles, the latched value toggles, but won’t toggle again until the new latched
            * value is captured by the sample rate strobe. This is a requirement for working with some 3 party devices
            * that have fsync strobes shorter than our sample rate.*/
    struct {
        byte Hz_250gyro_4000temp, Hz_184gyro_188, Hz_92gyro_98temp, Hz_41gyro_42temp, Hz_20both, Hz_10both, Hz_5both,
        Hz_3600gyro_4000temp;
    } DLPF_CFG /**<
            * For the DLPF to be used, fchoice[1:0] must be set to 2’b11, fchoice_b[1:0] is 2’b00.<br><br>The DLPF is
            * configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b’00. The gyroscope and temperature sensor are filtered
            * according to the value of DLPF_CFG and FCHOICE_B. Note that FCHOICE is the inverted value of FCHOICE_B
            * (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).*/;
};
extern const struct CONFIG CONFIG;


struct GYRO_CONFIG {
    byte REG;
    byte X_TEST; /**<X Gyro self-test*/
    byte Y_TEST; /**<Y Gyro self-test*/
    byte Z_TEST; /**<Z Gyro self-test*/
    byte FS_SEL; /**<Gyro Full Scale Select:<br>00 = +250dps<br>01 = +500 dps<br>10 = +1000 dps<br>11 = +2000 dps*/
    byte FCHOICE; /**<
            Used to bypass DLPF as shown in enum DLPF. NOTE: Register is Fchoice_b (inverted version of Fchoice),
            table 1 uses Fchoice (which is the inverted version of this register).*/ //todo DLPF enumeration
};
extern const struct GYRO_CONFIG GYRO_CONFIG;

struct ACCEL_CONFIG {
    byte REG;
    byte X_TEST; /**<X Accel self-test*/
    byte Y_TEST; /**<Y Accel self-test*/
    byte Z_TEST; /**<Z Accel self-test*/
    byte FS_SEL; /**<Accel Full Scale Select:<br>±2g (00), ±4g (01), ±8g (10), ±16g (11)*/
};
extern const struct ACCEL_CONFIG ACCEL_CONFIG;
#define ACCEL_CONFIG2 0x1Du
#define LP_ACCEL_ODR 0x1Eu
#define WOM_THR 0x1Fu

/** REG: 35 (0x23)<br><br>
 * 1 – Write the specified registers to the FIFO at the sample rate; If enabled, buffering of data occurs
 * even if data path is in standby. <br>0 – function is disabled
 */
struct FIFO_ENABLE {
    byte REG;
    byte TEMP_OUT;
    byte GYRO, GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT;
    byte ACCEL;
    byte SLV_2, SLV_1, SLV_0;
};
extern const struct FIFO_ENABLE FIFO_ENABLE;


struct I2C_MST_CTRL {
    byte REG;
    byte MULT_MST_EN; /**<
            Enables multi-master capability. When disabled, clocking to the I2C_MST_IF can be disabled when not in use
            and the logic to detect lost arbitration is disabled.*/
    byte WAIT_FOR_ES; /**<
            Delays the data ready interrupt until external sensor data is loaded. If I2C_MST_IF is disabled, the
            interrupt will still occur.*/
    byte SLV_3_FIFO_EN; /**<
            1 – write EXT_SENS_DATA registers associated to SLV_3 (as determined by I2C_SLV0_CTRL and I2C_SLV1_CTRL and
            I2C_SLV2_CTRL) to the FIFO at the sample rate.<br>0 – function is disabled*/
    byte P_NSR;/**<
            This bit controls the I2C Master’s transition from one slave read to the next slave read. If 0, there is a
            restart between reads. If 1, there is a stop between reads.*/
    struct {
        byte khz348, khz333, khz320, khz308, khz296, khz286, khz276, khz267, khz258, khz500, khz471, khz444, khz421,
                khz400, khz381, khz364;
    } CLK; /**<I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU-9255 internal 8MHz clock.*/
};
extern const struct I2C_MST_CTRL I2C_MST_CTRL;

struct I2C_SLV0_ADDR {
    byte REG;
    byte RNW; /**<1 – Transfer is a read<br>0 – Transfer is a write*/
};
extern const struct I2C_SLV0_ADDR I2C_SLV0_ADDR;

#define I2C_SLV0_REG 0x26u

struct I2C_SLV0_CTRL {
    byte REG;
    byte EN; /**<
            1 – Enable reading data from this slave at the sample rate and storing data at the first available
            EXT_SENS_DATA register, which is always EXT_SENS_DATA_00 for I2C slave 0.<br>
            0 – function is disabled for this slave*/
    byte BYTE_SW; /**<
            1 – Swap bytes when reading both the low and high byte of a word. Note there is nothing to swap after
            reading the first byte if I2C_SLV0_REG[0] = 1, or if the last byte read has a register address lsb = 0.<br><br>
            For example, if I2C_SLV0_REG = 0x1, and I2C_SLV0_LENG = 0x4:<br>
            1) The first byte read from address 0x1 will be stored at EXT_SENS_DATA_00,<br>
            2) the second and third bytes will be read and swapped, so the data read from address 0x2 will be stored at
            EXT_SENS_DATA_02, and the data read from address 0x3 will be stored at EXT_SENS_DATA_01,<br>
            3) The last byte read from address 0x4 will be stored at EXT_SENS_DATA_03 I2C_SLV0_BYTE_SW<br><br>
            0 – no swapping occurs, bytes are written in order read.*/
    byte REG_DIS; /**<When set, the transaction does not write a register value, it will only read data, or write data*/
    byte GRP; /**<
            External sensor data typically comes in as groups of two bytes. This bit is used to determine if the groups
            are from the slave’s register address 0 and 1, 2 and 3, etc.., or if the groups are address 1 and 2, 3 and 4,
            etc..<br><br>
            0 indicates slave register addresses 0 and 1 are grouped together (odd numbered register ends the group). 1
            indicates slave register addresses 1 and 2 are grouped together (even numbered register ends the group).
            This allows byte swapping of registers that are grouped starting at any address.*/
};
extern const struct I2C_SLV0_CTRL I2C_SLV0_CTRL;


#define I2C_MST_ST 0x036u;

struct INT_PIN_CFG {
    byte REG;
    byte ACTL; /**<1 – The logic level for INT pin is active low.<br>0 – The logic level for INT pin is active high.*/
    byte OPEN; /**<1 – INT pin is configured as open drain.<br>0 – INT pin is configured as push-pull.*/
    byte LATCH_EN; /**<
            1 – INT pin level held until interrupt status is cleared.<br>
            0 – INT pin indicates interrupt pulse’s is width 50us. */
    byte ANYRD_2CLEAR; /**<
            1 – Interrupt status is cleared if any read operation is performed.<br>
            0 – Interrupt status is cleared only by reading INT_STATUS register*/
    byte ACTL_FSYNC; /**<
            1 – The logic level for the FSYNC pin as an interrupt is active low.<br>
            0 – The logic level for the FSYNC pin as an interrupt is active high.*/
    byte FSYNC_MODE_EN; /**<
            1 – This enables the FSYNC pin to be used as an interrupt. A transition to the active level described by the
            ACTL_FSYNC bit will cause an interrupt. The status of the interrupt is read in the I2C Master Status
            register PASS_THROUGH bit.<br>
            0 – This disables the FSYNC pin from causing an interrupt.*/
    byte BYPASS_EN; /**<
            When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into ‘bypass mode’ when the i2c
            master interface is disabled. The pins will float high due to the internal pull-up if not enabled and the
            i2c master interface is disabled.*/
};
extern const struct INT_PIN_CFG INT_PIN_CFG;

struct INT_ENABLE {
    byte REG;
    byte WOM; /**<1 – Enable interrupt for wake on motion to propagate to interrupt pin.<br>0 – function is disabled.*/
    byte FIFO_OVERFLOW; /**<1 – Enable interrupt for fifo overflow to propagate to interrupt pin.<br>0 – function is disabled.*/
    byte FSYNC; /**<1 – Enable Fsync interrupt to propagate to interrupt pin.<br>0 – function is disabled.*/
    byte RAW_RDY; /**<
            1 – Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. The timing of the interrupt can
            vary depending on the setting in register I2C_MST_CTRL, bit [6] WAIT_FOR_ES.<br>
            0 – function is disabled.*/
};
extern const struct INT_ENABLE INT_ENABLE;

struct INT_STATUS {
    byte REG;
    byte WOM; /**<1 – Wake on motion interrupt occurred.*/
    byte FIFO_OVERFLOW; /**<1 – Fifo Overflow interrupt occurred. Note that the oldest data is has been dropped from the FIFO.*/
    byte FSYNC; /**<1 – Fsync interrupt occurred.*/
    byte RAW_RDY; /**<
           1 – Sensor Register Raw Data sensors are updated and Ready to be read. The timing of the interrupt can vary
           depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.*/
};
extern const struct INT_STATUS INT_STATUS;


#define ACCEL_OUT 0x3Bu
#define TEMP_OUT 0x41u
#define GYRO_OUT 0x43u
#define EXT_SENS_DATA_00 0x49u

#define I2C_SLV0_DO 0x63u

#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET 0x68

struct ACCEL_INTEL_CTRL {
    byte REG;
    byte EN; /**<This bit enables the Wake-on-Motion detection logic.*/
    byte MODE; /**< This bit defines<br>1 = Compare the current sample with the previous sample.<br>0 = Not used.*/
};
extern const struct ACCEL_INTEL_CTRL ACCEL_INTEL_CTRL;


struct USER_CTRL {
    byte REG;
    byte FIFO_EN; /**<
            1 – Enable FIFO operation mode.<br>0 – Disable FIFO access from serial interface. To disable FIFO writes by
            dma, use FIFO_EN register. To disable possible FIFO writes from DMP, disable the DMP.*/
    byte I2C_MST_EN; /**<
            1 – Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.<br>
            0 – Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically driven by pins SDA/SDI and SCL/ SCLK.
            <br><br>NOTE: DMP will run when enabled, even if all internal sensors are disabled, except when the sample
            rate is set to 8Khz.*/
    byte I2C_IF_DIS; /**<
            1 – Reset I2C Slave module and put the serial interface in SPI mode only. This bit auto clears after one clock cycle.*/
    byte FIFO_RST; /**<1 – Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.*/
    byte I2C_MST_RST; /**<
            1 – Reset I2C Master module. Reset is asynchronous. This bit auto clears after one clock cycle.<br><br>
            NOTE: This bit should only be set when the I2C master has hung. If this bit is set during an active I2C m
            aster transaction, the I2C slave will hang, which will require the host to reset the slave.*/
    byte SIG_COND_RST; /**<
            1 – Reset all gyro digital signal path, accel digital signal path, and temp digital signal path. This bit
            also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.*/
};
extern const struct USER_CTRL USER_CTRL;

struct PWR_MGMNT_1 {
    byte REG;
    byte H_RESET; /**<
            1 – Reset the internal registers and restores the default settings. Write a 1 to
            set the reset, the bit will auto clear.*/
    byte SLEEP; /**<When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)*/
    byte CYCLE; /**<
            When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample
            at a rate determined by LP_ACCEL_ODR register<br><br>NOTE: When all accelerometer axis are disabled via
            PWR_MGMT_2 register bits and cycle is enabled, the chip will wake up at the rate determined by the respective
            registers above, but will not take any samples.*/
    byte GYRO_STANDBY; /**<
            When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power
            mode that allows quick enabling of the gyros.*/
    byte PD_PTAT; /**<Power down internal PTAT voltage generator and PTAT ADC*/
    byte CLKSEL_MASK;
    struct {
        byte OSC_20MHZ; /**<Internal 20MHz oscillator*/
        byte AUTO; /**<Auto selects the best available clock source – PLL if ready, else use the Internal oscillator*/
        byte STOP;/**<Stops the clock and keeps timing generator in reset*/
    } CLKSEL;/**<(After OTP loads, the inverse of PU_SLEEP_MODE bit will be written to CLKSEL[0])*/
};
extern const struct PWR_MGMNT_1 PWR_MGMNT_1;

#define PWR_MGMNT_2 0x6Cu
#define FIFO_COUNT 0x72u
#define FIFO_RW 0x74u
#define WHO_AM_I 0x75u


//MAGNETOMETER
#define MAGN_DEV_ADDR (byte)0x0Cu
#define MAGN_BIT (byte)0b10000u


#endif //INNOFEET_SENSOR_REG_H
