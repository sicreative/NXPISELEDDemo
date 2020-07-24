/****************************************************************************//*!
*
* @file     adc.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    ADC inline function 1 header file
*
*******************************************************************************/
#ifndef __ADC_INLINE_FCN1_H
#define __ADC_INLINE_FCN1_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/ets.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: static inline int16_t EquivalentVoltageDigitalization(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Output: Result of the equivalent voltage conversion
*
* Description: Equivalent voltage conversion
*
*****************************************************************************/
static inline int16_t EquivalentVoltageDigitalization(tElecStruct *pElectrodeStruct)
{
	// Wait for conversion complete flag
	while(pElectrodeStruct->adcBasePtr->SC1[0] < 0x80)
	{}
	// Store result, clear COCO flag
	return pElectrodeStruct->adcBasePtr->R[0];
}

/*****************************************************************************
*
* Function: static inline void SimultaneousEquivalentVoltageDigitalization(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
*
* Input: Addresses of a pair of slider electrodes structures in electrodes structure array
*
* Description: Simultaneous Equivalent voltage conversion
*
*****************************************************************************/
static inline void SimultaneousEquivalentVoltageDigitalization(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
{
	// Configure both slider electrodes pins as analog input
	pElectrode0Struct->portBasePtr->PCR[pElectrode0Struct->pinNumberCext]= PCR_ANA;
	pElectrode1Struct->portBasePtr->PCR[pElectrode1Struct->pinNumberCext] = PCR_ANA;

	// Clear the SIM_SW_trigger flag to TRGMUX
	SIM->MISCTRL1 = 0;

	// Wait for conversion complete flag for both ADCs
	while(pElectrode0Struct->adcBasePtr->SC1[0] < 0x80 && pElectrode1Struct->adcBasePtr->SC1[0] < 0x80)
	{}
}

#endif /* __ADC_INLINE_FCN1_H */
