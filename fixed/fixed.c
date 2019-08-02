#include "fixed.h"

#define SIGN_MASK 0x80000000
#define EXPONENT_MASK 0x7F800000
#define FRACTION_MASK 0x007FFFFF
#define NORMALIZED_MASK 0x800000
#define EXPONENT_OFFSET 127

#define SIGN_BIT_NUM 31
#define EXPONENT_START_BIT_NUM 23

typedef union
{
	float f_float;
	uint32_t f_bits;
} FloatToBits;

fixed32_t float_to_fixed(float f, FixedPointError* err)
{
	fixed32_t ret = 0;
	int32_t sign = 0;
	int8_t exponent = 0;
	uint32_t fraction = 0;
	uint8_t i = 0;

	FloatToBits f_conv;

        // Extract the sign exponent and fraction from the floating point number
	f_conv.f_float = f;
	sign = (f_conv.f_bits & SIGN_MASK) >> SIGN_BIT_NUM;
	exponent = (f_conv.f_bits & EXPONENT_MASK) >> EXPONENT_START_BIT_NUM;
	fraction = (f_conv.f_bits & FRACTION_MASK) | ((exponent) ? NORMALIZED_MASK : 0);

	// Convert the exponent into the amount needed to shift the floating point fraction for fixed point
	exponent = exponent - EXPONENT_OFFSET - (EXPONENT_START_BIT_NUM - POINT);
	// Convert the sign into something that can be multiplied
	sign = (sign) ? -1 : 1;
	

	if(exponent > 0)
	{
		// Positive shift, left shift number into correct place 
		ret = (fraction << exponent);			
	
		// Check location of leading 1 for overflow	
		for(i = 0; i < 31; i++)
		{
			if(fraction & (0x80000000 >> i))
			{
				break;
			}
		}

		if(exponent >= i && ((sign * ret) != ret))
		{
			*err = OVERFLOW;
		}
		else
		{
			*err = NONE;
		}
	}
	else
	{
		// Negative shift, right shift number into correct place
		ret = (fraction >> (-1*exponent));

		// Check location of leading 1 for underflow
		for(i = 0; i < 31; i++)
		{
			if(fraction & (0x80000000 >> i))
			{
				break;
			}
		}

		if(-1*exponent + i > SIGN_BIT_NUM)
		{
			*err = UNDERFLOW;
		}
		else
		{
			*err = NONE;
		}
	}

	// Make return have the correct sign
	ret = sign * ret;

	return ret;	
}

float fixed_to_float(fixed32_t fixed)
{
	float ret = 0;
	int32_t shift;
	uint8_t i = 0;

	FloatToBits f_conv;
	f_conv.f_bits = 0;

	// Determine sign and set sign bit
	f_conv.f_bits |= (fixed & SIGN_MASK) ? 0x80000000 : 0x00000000;

	if(fixed != 0)
	{
		// Take absolute value of the fixed point representation
		fixed = fixed * ((f_conv.f_bits) ? -1 : 1);

		// Find location of the leading one to determine shift amount
		for(i = 0; i < 31; i++)
		{
			if(fixed & (0x80000000 >> i))
			{
				break;
			}
		}

		// Calculate shift amount of leading 1 location
		shift = EXPONENT_START_BIT_NUM - (SIGN_BIT_NUM - i);

		// Shift in the correct direction and set fractional part
		if(shift < 0)
		{
			f_conv.f_bits |= ((fixed >> (-1*shift)) & FRACTION_MASK);
		}
		else
		{
			f_conv.f_bits |= ((fixed << shift) & FRACTION_MASK);
		}

		// Set use leading 1 location to set the exponent
		f_conv.f_bits |= (((SIGN_BIT_NUM - POINT) - i) + EXPONENT_OFFSET) << EXPONENT_START_BIT_NUM;

		ret = f_conv.f_float;  		
	}

	return ret;
}

fixed32_t fixed_multiply(fixed32_t op1, fixed32_t op2, FixedPointError* err)
{
	int64_t op1_64 = 0;
	int64_t op2_64 = 0;
	int64_t res64 = 0;
	fixed32_t ret;

	op1_64 = op1;
	op2_64 = op2;

	res64 = (op1_64 * op2_64) >> POINT;
	ret = res64;

	if(((res64 & ((uint64_t)SIGN_MASK << 32)) >> 63) != (((uint32_t)ret & SIGN_MASK) >> 31))
	{
		*err = OVERFLOW;
	}
	else if(ret == 0 && (op1 != 0) && (op2 != 0))
	{
		*err = UNDERFLOW;
	}
	else
	{
		*err = NONE;
	}

	return ret;
}

fixed32_t fixed_divide(fixed32_t op1, fixed32_t op2, FixedPointError* err)
{
	int64_t op1_64 = op1;
	uint64_t res64 = 0;
	fixed32_t ret;

#ifdef FIXED_EXAMPLE
	res64 = (op1_64 << POINT) / op2;
#else
  res64 = op1_64 << POINT;
  do_div(res64, op2);
#endif

  ret = res64;

	if(((res64 & ((uint64_t)SIGN_MASK << 32)) >> 63) != (((uint32_t)ret & SIGN_MASK) >> 31))
	{
		*err = OVERFLOW;
	}
	else if(res64 == 0 && (op1 != 0))
	{
		*err = UNDERFLOW;
	}
	else
	{
		*err = NONE;
	}

	return ret;
}
