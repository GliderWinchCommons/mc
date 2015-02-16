/******************************************************************************
* File Name          : pay_flt_cnv.c
* Date First Issued  : 01/15/015
* Board              : little endian
* Description        : Conversion to/from float to half-float IEEE-754
*******************************************************************************/
/*
These routines are used to translate a float, 3/4 float, and half-float to/from
the CAN msg payload.  

The access to the payload is via a pointer and the there is no restriction on the
memory align boundary.

Endianness is not inlcuded in these routines.  They are based on little endian and
for big endian code will have to be added to deal with the reversed byte order.

Half-float is the IEEE 754 ARM "alternate" format which does not deal with NAN or 
subnormal values, but allows the value ranges to be +/- 131071.

3 qtr floats is a here-defined format that simply drops off the lower 8 bits of a
single precision float and does not make any adjustments to the exponent or other 
bits.

float is the standard IEEE 754 float.  Since the CAN payload start byte for the 
float may not be on a natural memory boundary, the packing/unpacking is included
in these routines.
*/
#include <stdint.h>
//#include "common.h"
#include <stdio.h>

union UIF
{
	float f;
	uint32_t ui;
	uint8_t uc[4];
};	

/******************************************************************************
 * float payhalffptofloat(uint8_t *p);
 * @brief 	: Convert CAN payload bytes holding half-float to float
 * @param	: p = pointer to payload start byte 
 * @return	: float
*******************************************************************************/
float payhalffptofloat(uint8_t *p)
{
	uint32_t 	uiman;	// Mantissa
	uint32_t 	uisign;	// Sign
	int32_t 	uiexp;	// Exponent
	union UIF flt;
/*	
	uiman = *p | (*(p+1) << 8);	//	does char get promoted to uint32_t before shift?
	uisign = (*(p+1) & 0x80) << 24;	// Save sign bit  //	does char get promoted to uint32_t before shift?
	uiman &= 0x03ff;		// Lower 10 bits is half-float mantissa
	uiman = uiman << 13;		// Position for full-float
	uiexp = ((*(p+1) >> 2) & 0x1f );
	uiexp += (127-15);
	uiexp = (uiexp << 23) & 0x7f800000;	//	why is this needed?
	flt.ui = uisign | uiexp | uiman; // Float is the composite
*/
	uiman = ((uint32_t)(*p)) | (((uint32_t) (*(p+1))) << 8);
	uiman &= 0x03ff;		// lower 10 bits is half-float mantissa	
	uiman = uiman << 13;	// position for full-float
	uisign = ((uint32_t)(*(p+1) & 0x80)) << 24;	// save sign bit 
	uiexp = ((*(p+1) >> 2) & 0x1f );
	uiexp += (127-15);
	uiexp = uiexp << 23;	// position for full-float
	flt.ui = uisign | uiexp | uiman; // Float is the composite
	
	return flt.f;
}
/******************************************************************************
 * void floattopayhalffp(uint8_t *p, float f);
 * @brief 	: Convert float to CAN payload bytes holding half-float representation
 * @param	: p = pointer to payload start byte 
 * @param	: f = single precision float to be converted to payload
*******************************************************************************/
void floattopayhalffp(uint8_t *p, float f)
{	
	union UIF flt;
	

	/*
	if (f > 1.31071E+5)
	{
		f = 1.31071E+5;
	}
	if (f < 1.31071E-4)
	{
		f = 1.31071E-4;
	}
	flt.f = f;	 
	*(p+0)  = (flt.ui >> 13);	// Get lo-ord part of mantissa
	*(p+1)  = ((flt.ui >> 21) & 0x03); // Get highest 2 bits of mantissa
	*(p+1) |= ((flt.ui >> 21) & 0x3c);	// Get full-float exponent
	*(p+1) |= ((flt.ui & 0xc0000000) >> 24); // Lastly the sign bit
	*/


	float absf = f;
	//	does not deal with subnormals and rounding yet
	if (f < (float) 0.0)
	 {
	 	f = -f;
	 }		 
	if (absf == (float) 0.0)
		{
			*p = *(p+1) = 0;
			return;
		}
	if (absf > 2047 * 64)	//	131008
	{
		absf = 2047 * 64;
	}
	if (absf < (float) 1.0 / (1 << 14))	//	2^-14 = 6.103515625e-5
	{
		absf = (float) 1.0 / (1 << 14);
	}
	flt.f = absf;	 
	*p = (flt.ui >> 13);					// Get low-ord part of mantissa
	*(p+1)  = (flt.ui >> 21) & 0x03; 		// Get highest 2 bits of mantissa
	*(p+1) |= (flt.ui >> 21) & 0x3c;		// Get 4 lsbs of exponent
	*(p+1) |= (flt.ui & 0xc0000000) >> 24;	// Lastly the sign bit and exponent msb
	return;
}
/******************************************************************************
 * void floattopay3qtrfp(uint8_t *p, float f);
 * @brief 	: Convert float to CAN payload bytes holding 3/4 fp representation
 * @param	: p = pointer to payload start byte 
 * @param	: f = single precision float to be converted to payload
*******************************************************************************/
void floattopay3qtrfp(uint8_t *p, float f)
{
	union UIF flt;
	flt.f = f;
	*(p+0) = flt.uc[1];
	*(p+1) = flt.uc[2];
	*(p+2) = flt.uc[3];
	return;
}
/******************************************************************************
 * float pay3qtrfptofloat(uint8_t *p);
 * @brief 	: Convert CAN payload bytes holding 3/4 fp representation to float
 * @param	: p = pointer to payload start byte 
 * @return	: float
*******************************************************************************/
float pay3qtrfptofloat(uint8_t *p)
{
	union UIF flt;
	flt.uc[0] = 0;
	flt.uc[1] = *(p+0);
	flt.uc[2] = *(p+1);
	flt.uc[3] = *(p+2);
	return flt.f;
}	
/******************************************************************************
 * void floattopaysinglefp(uint8_t *p, float f);
 * @brief 	: Convert float to CAN payload bytes holding single precision representation
 * @param	: p = pointer to payload start byte 
 * @param	: f = single precision float to be converted to payload
*******************************************************************************/
void floattopaysinglefp(uint8_t *p, float f)
{
	union UIF flt;
	flt.f = f;
	*(p+0) = flt.uc[0];
	*(p+1) = flt.uc[1];
	*(p+2) = flt.uc[2];
	*(p+3) = flt.uc[3];

	return;
}
/******************************************************************************
 * float paysinglefptofloat(uint8_t *p);
 * @brief 	: Convert float to CAN payload bytes holding single precision representation
 * @param	: p = pointer to payload start byte 
 * @return	: float
*******************************************************************************/
float paysinglefptofloat(uint8_t *p)
{
	union UIF flt;
	flt.uc[0] = *(p+0);
	flt.uc[1] = *(p+1);
	flt.uc[2] = *(p+2);
	flt.uc[3] = *(p+3);
	return flt.f;
}	
