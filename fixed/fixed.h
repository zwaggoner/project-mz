#ifndef FIXED_H
#define FIXED_H

#ifdef FIXED_EXAMPLE
#include <stdint.h>
#else
#include <linux/types.h>
typedef u64 uint64_t;
typedef s64 int64_t;
typedef u32 uint32_t;
typedef s32 int32_t;
typedef u16 uint16_t;
typedef s16 int16_t;
typedef u8 uint8_t;
typedef s8 int8_t;
#endif

#define POINT 16
#define FIXEDPOINT_ERR_STR(err) ((err) ? ((err == OVERFLOW || err == UNDERFLOW) ? ((err == OVERFLOW) ? "Overflow" : "Underflow") : "Unknown") : "None")

typedef enum
{
	NONE = 0,
	OVERFLOW = -1,
	UNDERFLOW = -2,	
} FixedPointError;

typedef union
{
	float f_float;
	uint32_t f_bits;
} FloatToBits;

typedef int32_t fixed32_t;

fixed32_t float_to_fixed(float f, FixedPointError* err);
float fixed_to_float(fixed32_t fixed);
fixed32_t fixed_multiply(fixed32_t op1, fixed32_t op2, FixedPointError* err);
fixed32_t fixed_divide(fixed32_t op1, fixed32_t op2, FixedPointError* err);

#endif
