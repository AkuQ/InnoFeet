
#include <stdlib.h>
#include <stdio.h>
#include "i2c.h"
#include "globals.h"

typedef struct _I2C_Interface {
    //PUBLIC:
    int device_address;
    //PRIVATE:
    int (*read_bytes_cb)(int, int, byte* const, int);
    int (*write_bytes_cb)(int, int, const byte*, int);
} _I2C_Interface;

I2C_Interface* i2c_init(
        int device_address,
        int (*read_bytes_cb)(int, int, byte* const, int),
        int (*write_bytes_cb)(int, int, const byte*, int)
) {
    I2C_Interface* i2c = malloc(sizeof(*i2c));
    i2c->device_address = device_address;
    i2c->write_bytes_cb = write_bytes_cb;
    i2c->read_bytes_cb = read_bytes_cb;
    return i2c;
}

//PROPERTIES:

int i2c_get_device_address(I2C_Interface *self) {
    return self->device_address;
}

//METHODS:

int i2c_read_byte(I2C_Interface* self, int address, byte* const buffer) {
    return i2c_read_bytes(self, address, buffer, 1);
}

int i2c_read_bytes(I2C_Interface* self, int address, byte* const buffer, int n) {
    if(self->read_bytes_cb(self->device_address, address, buffer, n) == SUCCESS) {
        return SUCCESS;
    }
    else {
        return ERROR;
    };
}

int i2c_write_byte(I2C_Interface* self, int address, byte value) {
    return i2c_write_bytes(self, address, &value, 1);
}

int i2c_write_bytes(I2C_Interface* self, int address, const byte *value, int n) {
    if(self->write_bytes_cb(self->device_address, address, value, n) == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

int _i2c_write_bits3(I2C_Interface* self, int address, byte bits) {
    return _i2c_write_bits4(self, address, bits, bits);
}
int _i2c_write_bits4(I2C_Interface* self, int address, byte bits, byte mask){
    byte old_val;
    TRY(i2c_read_byte(self, address, &old_val));
    TRY(i2c_write_byte(self, address, (old_val & ~mask) | (bits & mask)));
    return SUCCESS;
}

int i2c_print(I2C_Interface* self) {
    printf("i2c_get_device_address %d\n", self->device_address);
    return 0;
}
