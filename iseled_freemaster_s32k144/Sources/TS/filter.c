/****************************************************************************//*!
*
* @file     filter.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Filter implementation
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include <TS/ets.h>
#include <TS/filter.h>
#include <TS/main.h>
#include "S32K144.h"

/*******************************************************************************
* Variables definition
*******************************************************************************/
// IIR1 filter coefficients
tFrac32 FilterIIR1CoeffB0[NUMBER_OF_FILTERS_USED], FilterIIR1CoeffB1[NUMBER_OF_FILTERS_USED], FilterIIR1CoeffA1[NUMBER_OF_FILTERS_USED];
#if SLIDER_ENABLE
// IIR1 filter buffer x(k-1), y(k-1)
tFrac32 FilterIIR1BufferX[NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES], FilterIIR1BufferY[NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES];
// IIR1 Filter type
extern uint8_t   LPFilterType[NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES];
#else
// IIR1 filter buffer x(k-1), y(k-1)
tFrac32 FilterIIR1BufferX[NUMBER_OF_ELECTRODES][NUMBER_OF_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES], FilterIIR1BufferY[NUMBER_OF_ELECTRODES][NUMBER_OF_FILTERS_USED][NUMBER_OF_HOPPING_FREQUENCIES];
// IIR1 Filter type
extern uint8_t   LPFilterType[NUMBER_OF_ELECTRODES];
#endif
/****************************************************************************
* Math functions
****************************************************************************/
static inline tFrac32 MLIB_ShR_F32(register tFrac32 f32In1,register tU16 u16In2)
{
  return((tFrac32)(f32In1 >> u16In2));
}

static inline tFrac32 Mul_F32_C(register tFrac32 f32In1, register tFrac32 f32In2)
{
  register tS32 s32UpperIn1, s32UpperIn2;
  register tU32 u32LowerIn1, u32LowerIn2, u32ResultLower, u32ResultUpper;
  register tU32 u32Result0, u32Result1, u32Result2;

  s32UpperIn1 = MLIB_ShR_F32(f32In1,(tU16)16U);
  u32LowerIn1 = (tU32)(f32In1 & (tFrac32)0x0000FFFFU);
  s32UpperIn2 = MLIB_ShR_F32(f32In2,(tU16)16U);
  u32LowerIn2 = (tU32)(f32In2 & (tFrac32)0x0000FFFFU);
  u32ResultUpper = u32LowerIn1 * u32LowerIn2;
  u32ResultLower = u32ResultUpper >> (tU16)16U;

  u32ResultUpper = ((tU32)s32UpperIn1 * u32LowerIn2) + u32ResultLower;
  u32Result2 = u32ResultUpper & (tU32)0x0000FFFFU;
  u32Result1 = (tU32)((tS32)u32ResultUpper >> (tU16)16U);
  u32ResultUpper = (u32LowerIn1 * (tU32)s32UpperIn2) + u32Result2;

  u32ResultLower = (tU32)((tS32)u32ResultUpper >> (tU16)16U);
  u32Result0 = (((tU32)s32UpperIn1 * (tU32)s32UpperIn2) + u32Result1) + u32ResultLower;
  u32Result0 = (u32Result0 << (tU16)1U) | ((u32ResultUpper << (tU16)16U) >> (tU16)31U);

  return((tFrac32)u32Result0);
}

/*****************************************************************************
*
* Function: void FilterIIR1Init(void)
*
* Description: Init IIR1 low pass filter
*
*****************************************************************************/
void FilterIIR1Init(void)
{
#ifdef FILTER_1
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_1] = FRAC32(FILTER_1_COEF_B0);
	FilterIIR1CoeffB1[FILTER_1] = FRAC32(FILTER_1_COEF_B1);
	FilterIIR1CoeffA1[FILTER_1] = FRAC32(FILTER_1_COEF_A0);
#endif

#ifdef FILTER_2
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_2] = FRAC32(FILTER_2_COEF_B0);
	FilterIIR1CoeffB1[FILTER_2] = FRAC32(FILTER_2_COEF_B1);
	FilterIIR1CoeffA1[FILTER_2] = FRAC32(FILTER_2_COEF_A0);
#endif

#ifdef FILTER_3
	// Load coefficients
	FilterIIR1CoeffB0[FILTER_3] = FRAC32(FILTER_3_COEF_B0);
	FilterIIR1CoeffB1[FILTER_3] = FRAC32(FILTER_3_COEF_B1);
	FilterIIR1CoeffA1[FILTER_3] = FRAC32(FILTER_3_COEF_A0);
#endif

}

/*****************************************************************************
*
* void FilterIIR1BufferInit(uint8_t elec, tFrac32 valueBufferX, tFrac32 valueBufferY, uint8_t frequencyID)
*
* Description: Init IIR1 low pass filter buffer
*
*****************************************************************************/
void FilterIIR1BufferInit(uint8_t elec, tFrac32 valueBufferX, tFrac32 valueBufferY, uint8_t frequencyID)
{
	FilterIIR1BufferX[elec][LPFilterType[elec]][frequencyID] = valueBufferX << IIR_FILTER_VALUE_SHIFT;
	FilterIIR1BufferY[elec][LPFilterType[elec]][frequencyID] = valueBufferY << IIR_FILTER_VALUE_SHIFT;
}

/*****************************************************************************
*
* Function: tFrac16 FilterIIR1(uint8_t elec, tFrac32 x_k, uint8_t frequencyID)
*
* Description: IIR1 filter implementation
*              y(k) = b0*x(k) + b1*x(k-1) - a1*y(k-1)
*
*****************************************************************************/
tFrac32 FilterIIR1(uint8_t elec, tFrac32 x_k, uint8_t frequencyID)
{
	register tFrac32 M1;
	register tFrac32 M2;
	register tFrac32 M3;
	register tFrac32 Acc;
	register tFrac32 y_k;

	// Shift left input value to achieve calculation the highest resolution
	x_k = x_k << IIR_FILTER_VALUE_SHIFT;

    // M1 = b0 * x(k)
	M1 = Mul_F32_C(FilterIIR1CoeffB0[LPFilterType[elec]], x_k);

    // M2 = b1 * x(k-1)
    M2 = Mul_F32_C(FilterIIR1CoeffB1[LPFilterType[elec]], FilterIIR1BufferX[elec][LPFilterType[elec]][frequencyID]);

    // M3 = a1 * y(k-1)
    M3 = Mul_F32_C(FilterIIR1CoeffA1[LPFilterType[elec]], FilterIIR1BufferY[elec][LPFilterType[elec]][frequencyID]);

    // Acc = M2 - M3
    Acc = (tFrac32)(M2 - M3);

    // Acc = Acc + M1
    Acc = (tFrac32)(Acc + M1);

    // Load output
    y_k = Acc;

	// IIR1 filter buffer x(k-1), y(k-1)
	// x(k-1)
	FilterIIR1BufferX[elec][LPFilterType[elec]][frequencyID] = x_k;
	// y(k-1)
	FilterIIR1BufferY[elec][LPFilterType[elec]][frequencyID] = y_k;

	// Shift right the result to compensate value increase due to desired highest resolution
	y_k = y_k >> IIR_FILTER_VALUE_SHIFT;

    // Return value
    return(y_k);
}
