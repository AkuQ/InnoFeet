
#ifndef INNOFEET_BYTE_CONVERSION_H
#define INNOFEET_BYTE_CONVERSION_H

#include "globals.h"

int uint_to_bytes(u_int64_t i, byte *buffer, unsigned int n);
int int_to_bytes(int64_t i, byte *buffer, unsigned int n);
int ufloat_to_bytes(float f, float r, byte *buffer, unsigned int n);
int float_to_bytes(float f, float r, byte *buffer, unsigned int n);

unsigned int bytes_to_uint(const byte* b, unsigned int n);
int bytes_to_int(const byte* b, unsigned int n);
float bytes_to_ufloat(const byte* b, unsigned int n, float r);
float bytes_to_float(const byte* b, unsigned int n, float r);

#endif //INNOFEET_BYTE_CONVERSION_H
