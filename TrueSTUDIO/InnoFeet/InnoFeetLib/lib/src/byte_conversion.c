
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "byte_conversion.h"

//#define INNOFEET_LARGE_ENDIAN

static inline u_int64_t nbyte_max(u_int64_t *value, unsigned int nbytes) {
    if(nbytes > sizeof(u_int64_t)) return ERROR; //todo: handle arbitrary len //todo: error msg/type
    *value = ((u_int64_t)1 << nbytes * 8 - 1) - 1; //add shift overflow-guard bit to the left
    *value = ((*value) << 1) | 1; // overflow-guard bit to 1
    return SUCCESS;
}

static inline void populate_bytes(u_int64_t src, byte* bytes, int n) {
#ifdef INNOFEET_LARGE_ENDIAN
    for(int i = n - 1; i >= 0; i--) {
#else
    for(int i = 0; i < n; i++) {
#endif
        bytes[i] = (byte)(src & 0b11111111u);
        src >>= 8;
    }
}

static inline u_int64_t depopulate_bytes( const byte* bytes, int n) {
    u_int64_t ret = 0;
#ifdef INNOFEET_LARGE_ENDIAN
    for(int i = 0; i < n; i++)
#else
    for(int i = n - 1; i >= 0; i--)
#endif
        ret = (ret << 8u) + bytes[i];
    return ret;
}


int uint_to_bytes(u_int64_t i, byte *buffer, unsigned int n) {
    if(i >= 1LU << 8 * n) return ERROR;
    //        raise ValueError("Value %d does not fit %d bytes as an unsigned int (max: %d)" % (orig_i, n, (1 << 8 * n) - 1))
    populate_bytes(i, buffer, n);
    return SUCCESS;
}

int int_to_bytes(int64_t i, byte *buffer, unsigned int n) {
    u_int64_t mask = (1LU << 8u * n - 1) - 1; //Bit mask of n bytes wide minus 1 (complement bit)
    u_int64_t bits = *(u_int64_t*)&i;

    if((i < 0 && ~(bits | mask)) || (i > 0 && bits > mask)) return ERROR;
    //        printf("Value %lu does not fit %d bytes as a signed int (range: -%lu ~ %lu)\n", i, n, (u_int64_t)powl(2,n*8 -1), (u_int64_t)powl(2,n*8 -1));

    populate_bytes(i, buffer, n);

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
    return (int)depopulate_bytes(b, n);
}

int bytes_to_int(const byte* b, unsigned int n) {
#ifdef INNOFEET_LARGE_ENDIAN
    int complement_bit = b[0] & 128u;
#else
    int complement_bit = b[n-1] & 128u;
#endif
    int ret = bytes_to_uint(b, n);
    if(complement_bit)
        ret -= (1u << n * 8) - 1;
    return ret;
}


float bytes_to_ufloat(const byte* b, unsigned int n, float r) {
	u_int64_t max;
	TRY(nbyte_max(&max, n)); //todo: handle error

    float v = (float)bytes_to_uint(b, n);
    return v / max * r;
}


float bytes_to_float(const byte* b, unsigned int n, float r) {
	u_int64_t max;
	TRY(nbyte_max(&max, n)); //todo: handle error
	max >>= 1;

    float v = (float)bytes_to_int(b, n);
    return v / max * r;
}
