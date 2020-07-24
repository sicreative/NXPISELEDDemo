/****************************************************************************//*!
*
* @file     filter.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Filter implementation
*
*******************************************************************************/
#ifndef __FILTER_H
#define __FILTER_H

/*******************************************************************************
* Type defines
*******************************************************************************/
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef volatile signed char vint8_t;
typedef volatile unsigned char vuint8_t;

typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef volatile signed short vint16_t;
typedef volatile unsigned short vuint16_t;

typedef signed long int32_t;
typedef unsigned long uint32_t;
typedef volatile signed long vint32_t;
typedef volatile unsigned long vuint32_t;

typedef int16_t   tFrac16;        // 16-bit signed fractional Q1.15 type
typedef int32_t   tFrac32;        // 32-bit signed fractional Q1.31 type
typedef float     tFloat;         // single precision float type
typedef int32_t   tS32;
typedef uint32_t   tU32;
typedef uint16_t   tU16;
typedef signed long   tS64;       // signed 64-bit integer type

/*******************************************************************************
* Project Related Defines
*******************************************************************************/
// Maximal positive value of a signed 32-bit fixed point fractional number.
#define SFRACT_MAX           (0.9999999995343387126922607421875)

// Maximal negative value of a signed 16-bit fixed point fractional number.
#define SFRACT_MIN          (-1.0)

// Maximal positive value of a signed 16-bit fixed point integer number.
#ifndef INT16_MAX
#define INT16_MAX           ((tFrac16) 0x7fff)
#endif

// Maximal negative value of a signed 16-bit fixed point integer number.
#ifndef INT16_MIN
#define INT16_MIN           ((tFrac16) 0x8000)
#endif

// Maximal positive value of a signed 32-bit fixed point integer number.
#ifndef INT32_MAX
#define INT32_MAX           ((tS32) 0x7fffffff)
#endif

// Maximal positive value of a signed 32-bit fixed point integer number.
#ifndef INT32_MIN
#define INT32_MIN           ((tS32) 0x80000000U)
#endif

// Convert a signed fractional [-1,1) number into a 16-bit fixed point number in format Q1.15.
#define FRAC16(x)           ((tFrac16) (((tFloat) x) < (SFRACT_MAX) ? (((tFloat) x) >= SFRACT_MIN ? ((tFloat) x)*((tFloat) 0x8000U) : INT16_MIN) : INT16_MAX))

// Macro converting a signed fractional [-1,1) number into a 32-bit fixed point number in format Q1.31.
#define FRAC32(x)           ((tFrac32) (((x) < SFRACT_MAX) ? (((x) >= SFRACT_MIN) ? ((x)*2147483648.0) : INT32_MIN) : INT32_MAX))

/*******************************************************************************
* Function prototypes
******************************************************************************/
void FilterIIR1Init(void);
void FilterIIR1BufferInit(uint8_t elec, tFrac32 valueBufferX, tFrac32 valueBufferY, uint8_t frequencyID);
tFrac32 FilterIIR1(uint8_t elec, tFrac32 x_k, uint8_t frequencyID);


#endif /* __FILTER_H */
