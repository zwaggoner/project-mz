/**
 * \file fixed.h
 * \brief Header for fixed point library
 */

#ifndef FIXED_H
#define FIXED_H

#ifdef FIXED_EXAMPLE
#include <stdint.h>
#else
#include <linux/types.h>
#include <asm/div64.h>
typedef u64 uint64_t;
typedef s64 int64_t;
typedef u32 uint32_t;
typedef s32 int32_t;
typedef u16 uint16_t;
typedef s16 int16_t;
typedef u8 uint8_t;
typedef s8 int8_t;
#endif

/**
 * \def POINT
 * Defines the point of the fixed point library
 */
#define POINT 16
/**
 * Returns the error string describing the Fixed Point Error that occured
 */
#define FIXEDPOINT_ERR_STR(err) ((err) ? ((err == OVERFLOW || err == UNDERFLOW) ? ((err == OVERFLOW) ? "Overflow" : "Underflow") : "Unknown") : "None")

/**
 * Defining the different fixed point library errors
 */
typedef enum
{
	NONE = 0, /**< No Error occured */
	OVERFLOW = -1, /**< Overflow error occured */
	UNDERFLOW = -2,	/**< Underflow error occured */
} FixedPointError;

/**
 * Defines the fixed point type
 */
typedef int32_t fixed32_t;

/**
 * \brief Converts 32 bit floating point to 32 bit fixed point
 * \param f The floating point number
 * \param err The fixed point error that occured
 * \return The converted fixed point number
 */
fixed32_t float_to_fixed(float f, FixedPointError* err);

/**
 * \brief Converts 32 bit fixed point number to a 32 bit floating point number
 * \param fixed The fixed point number
 * \return The floating point number
 */
float fixed_to_float(fixed32_t fixed);

/**
 * \brief Implements the multiplication of two 32 bit fixed point numbers op1*op2 WARNING: Do not use for multiplication by integer
 * \param op1 The first operand in the multiplication
 * \param op2 The second operand in the multiplication
 * \param err The error of the multiplication operation 
 * \return The result of the multiplication 
 */
fixed32_t fixed_multiply(fixed32_t op1, fixed32_t op2, FixedPointError* err);

/**
 * \brief Implements the division of two fixed point nubmers op1/op2 WARNING: Do not use for division by integer
 * \param op1 First operand in division 
 * \param op2 Second operand in division
 * \param err Error in the division operation
 * \return The result of the division  
 */
fixed32_t fixed_divide(fixed32_t op1, fixed32_t op2, FixedPointError* err);

#endif
