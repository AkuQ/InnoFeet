

#ifndef INNOFEET_I2C_H
#define INNOFEET_I2C_H

#include "globals.h"


typedef struct _I2C_Interface I2C_Interface;

// INIT:
I2C_Interface* i2c_init(
        int device_address,
        int (*read_bytes_cb)(int dev_address, int mem_address, byte* const buffer, int n),
        int (*write_bytes_cb)(int dev_address, int mem_address, const byte* bytes, int n)
);

//PROPERTIES:
int i2c_get_device_address(I2C_Interface *self);

// METHODS:
int i2c_read_byte(I2C_Interface*, int address, byte* buffer);
int i2c_read_bytes(I2C_Interface*, int address, byte* buffer, int n);
int i2c_write_byte(I2C_Interface*, int address, byte value);
int i2c_write_bytes(I2C_Interface*, int address, const byte* value, int n);

/** Write specific bits in a memory address.<br>(Will perform an I2C read before writing).
 * \param self I2C_Interface instance
 * \param address Memory address of write
 * \param bits Bits to write
 * \param mask (optional) Bits to write. If not set only bits that are set to 1 will be written.
 * \return Success/Error
 */
#define i2c_write_bits(...) OVERLOAD(_i2c_write_bits, __VA_ARGS__)
int _i2c_write_bits3(I2C_Interface*, int address, byte bits);
int _i2c_write_bits4(I2C_Interface*, int address, byte bits, byte mask);

int i2c_print(I2C_Interface*);


#endif //INNOFEET_I2C_H
