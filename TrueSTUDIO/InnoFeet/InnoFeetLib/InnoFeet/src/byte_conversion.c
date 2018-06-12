
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "byte_conversion.h"


static inline u_int64_t nbyte_max(u_int64_t *value, unsigned int nbytes) {
    if(nbytes > sizeof(u_int64_t)) return ERROR; //todo: handle arbitrary len //todo: error msg/type
    *value = ((u_int64_t)1 << nbytes * 8 - 1) - 1; //add shift overflow-guard bit to the left
    *value = ((*value) << 1) | 1; // overflow-guard bit to 1
    return SUCCESS;
}


int uint_to_bytes(u_int64_t i, byte *buffer, unsigned int n) {
    if(i >= 1LU << 8 * n) return ERROR;
    //        raise ValueError("Value %d does not fit %d bytes as an unsigned int (max: %d)" % (orig_i, n, (1 << 8 * n) - 1))

    for(int j = n - 1; j >= 0; j--) {
        buffer[j] = (byte)(i & 0b11111111u);
        i >>= 8;
    }
    return SUCCESS;
}

int int_to_bytes(int64_t i, byte *buffer, unsigned int n) {
    u_int64_t mask = (1LU << 8u * n - 1) - 1; //Bit mask of n bytes wide minus 1 (complement bit)
    u_int64_t bits = *(u_int64_t*)&i;

    if((i < 0 && ~(bits | mask)) || (i > 0 && bits > mask)) return ERROR;
    //        printf("Value %lu does not fit %d bytes as a signed int (range: -%lu ~ %lu)\n", i, n, (u_int64_t)powl(2,n*8 -1), (u_int64_t)powl(2,n*8 -1));

    for(int j = n - 1; j >= 0; j--) {
        buffer[j] = (byte)(bits & 0b11111111u);
        bits >>= 8;
    }


    return SUCCESS;
}

int ufloat_to_bytes(float f, float r, byte *buffer, unsigned int n) {
    if(0 > f || f > r) return ERROR; //todo: error msg/type

    u_int64_t max;
    TRY(nbyte_max(&max, n));
    u_int64_t i = (u_int64_t)(roundf(f / r * max));
    return uint_to_bytes(i, buffer, n);
}

int float_to_bytes(float f, float r, byte *buffer, unsigned int n) {
    if(-r > f || f > r) return ERROR; //todo: error msg/type
    u_int64_t max;
    TRY(nbyte_max(&max, n));
    max >>= 1;
    int64_t i = (int64_t)(roundf(f / r * max));
    return int_to_bytes(i, buffer, n);
}

unsigned int bytes_to_uint(const byte* b, unsigned int n){
    unsigned int ret = 0;
    for(int i = 0; i < n; i++)
        ret = (ret << 8u) + b[i];
    return ret;
}

int bytes_to_int(const byte* b, unsigned int n) {
    int complement_bit = b[0] & 128u;
    int ret = bytes_to_uint(b, n);
    if(complement_bit)
        ret -= (1u << n * 8) - 1;
    return ret;
}


float bytes_to_ufloat(const byte* b, unsigned int n, float r) {
    float v = (float)bytes_to_uint(b, n);
    u_int64_t max;
    TRY(nbyte_max(&max, n)); //todo: handle error
    return v / max * r;
}


float bytes_to_float(const byte* b, unsigned int n, float r) {
    float v = (float)bytes_to_int(b, n);
    u_int64_t max;
    TRY(nbyte_max(&max, n)); //todo: handle error
    max >>= 1;
    return v / max * r;
}