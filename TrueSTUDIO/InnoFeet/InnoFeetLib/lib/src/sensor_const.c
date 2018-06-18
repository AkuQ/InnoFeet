
#include "sensor_const.h"

const struct GYRO_CONFIG GYRO_CONFIG = {
        0x1Bu,
        128u, 64u, 32u, 24u, 3u
};

const struct ACCEL_CONFIG ACCEL_CONFIG = {
        0x1Cu,
        128u, 64u, 32u, 24u
};

const struct I2C_MST_CTRL I2C_MST_CTRL = {
        0x24u,
        128u, 64u, 32u, 16u,
        {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u}
};

const struct I2C_SLV0_ADDR I2C_SLV0_ADDR = {
        0x25u,
        128u
};


const struct I2C_SLV0_CTRL I2C_SLV0_CTRL = {
        0x27u,
        128u, 64u, 32u, 16u
};


const struct INT_PIN_CFG INT_PIN_CFG = {
        0x37u,
        128u, 64u, 32u, 16u, 8u, 4u, 2u
};

const struct INT_ENABLE INT_ENABLE = {
        0x38u,
        64u, 16u, 8u, 1u
};

const struct INT_STATUS INT_STATUS = {
        0x3Au,
        64u, 16u, 8u, 1u
};

const struct ACCEL_INTEL_CTRL ACCEL_INTEL_CTRL = {
        0x69u,
        128u, 64u
};

const struct USER_CTRL USER_CTRL = {
        0x6Au,
        64u, 32u, 16u, 4u, 2u, 1u
};

const struct PWR_MGMNT_1 PWR_MGMNT_1 = {
        0x6Bu,
        128u, 64u, 32u, 16u, 8u, 7u,
        {0u, 1u, 7u}
};

