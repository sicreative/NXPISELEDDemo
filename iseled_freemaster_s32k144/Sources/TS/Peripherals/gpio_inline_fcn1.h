/****************************************************************************//*!
*
* @file     gpio_inline_fcn1.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    GPIO inline functions 1 header file
*
*******************************************************************************/
#ifndef __GPIO_INLINE_FCN1_H
#define __GPIO_INLINE_FCN1_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/ets.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: static inline void ChargeDistribution(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Description: Charge distribution
*
*****************************************************************************/
static inline void ChargeDistribution(tElecStruct *pElectrodeStruct)
{
   // Drive electrode GPIO low
	pElectrodeStruct->gpioBasePtr->PCOR = 1 << pElectrodeStruct->pinNumberElec;
	//REG_RMW32(&(pElectrodeStruct->gpioBasePtr->PCOR), 1 << pElectrodeStruct->pinNumberElec, 1 << pElectrodeStruct->pinNumberElec);
    // Configure electrode pin as GPIO to precharge electrode
    pElectrodeStruct->portBasePtr->PCR[pElectrodeStruct->pinNumberElec] = PCR_GPIO;

    // Drive Cext GPIO high
    pElectrodeStruct->gpioBasePtr->PSOR = 1 << pElectrodeStruct->pinNumberCext;
    //REG_RMW32(&(pElectrodeStruct->gpioBasePtr->PSOR), 1 << pElectrodeStruct->pinNumberCext, 1 << pElectrodeStruct->pinNumberCext);
    // Configure Cext pin as GPIO to precharge Cext
    pElectrodeStruct->portBasePtr->PCR[pElectrodeStruct->pinNumberCext] = PCR_GPIO;

    // Configure Electrode and Cext pins as outputs at the same time
    //pElectrodeStruct->gpioBasePtr->PDDR |= pElectrodeStruct->portMask;
    REG_WRITE32(&(pElectrodeStruct->gpioBasePtr->PDDR), ((REG_READ32(&(pElectrodeStruct->gpioBasePtr->PDDR))) | (pElectrodeStruct->portMask)));

}

/*****************************************************************************
 *
 * Function: static inline void SimultaneousChargeDistribution(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
 *
 * Input: Addresses of a pair of slider electrodes structures in electrodes structure array
 *
 * Description: Simultaneous Charge distribution for two slider electrodes at once
 *
 *****************************************************************************/
static inline void SimultaneousChargeDistribution(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
{
	// Drive both electrode pins GPIO low
	pElectrode0Struct->gpioBasePtr->PCOR = (1 << pElectrode0Struct->pinNumberElec) | (1 << pElectrode1Struct->pinNumberElec);

	// Configure electrode pins as GPIO to precharge electrodes
	pElectrode0Struct->portBasePtr->PCR[pElectrode0Struct->pinNumberElec] = PCR_GPIO;
	pElectrode1Struct->portBasePtr->PCR[pElectrode1Struct->pinNumberElec] = PCR_GPIO;

	// Drive both Cext pins GPIO high
	pElectrode0Struct->gpioBasePtr->PSOR = (1 << pElectrode0Struct->pinNumberCext) | (1 << pElectrode1Struct->pinNumberCext);

	// Configure Cexts pins as GPIO to precharge Cexts
	pElectrode0Struct->portBasePtr->PCR[pElectrode0Struct->pinNumberCext] = PCR_GPIO;
	pElectrode1Struct->portBasePtr->PCR[pElectrode1Struct->pinNumberCext] = PCR_GPIO;

	// Configure both Electrodes and Cext pins as outputs at the same time
	pElectrode0Struct->gpioBasePtr->PDDR = (pElectrode0Struct->portMask + pElectrode1Struct->portMask);

}

/*****************************************************************************
*
* Function: static inline void ChargeRedistribution(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Description: Charge redistribution
*
*****************************************************************************/
static inline void ChargeRedistribution(tElecStruct *pElectrodeStruct)
{
	// Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(pElectrodeStruct->gpioBasePtr->PDDR), ((REG_READ32(&(pElectrodeStruct->gpioBasePtr->PDDR))) & (~(pElectrodeStruct->portMask))));
}

/*****************************************************************************
 *
 * Function: static inline void SimultaneousChargeRedistribution(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
 *
 * Input: Addresses of a pair of slider electrodes structures in electrodes structure array
 *
 * Description: Simultaneous Charge distribution for two slider electrodes at once
 *
 *****************************************************************************/
static inline void SimultaneousChargeRedistribution(tElecStruct *pElectrode0Struct, tElecStruct *pElectrode1Struct)
{
	// Configure Electrode and Cext pins as inputs at the same time
	pElectrode0Struct->gpioBasePtr->PDDR &= ~(pElectrode0Struct->portMask + pElectrode1Struct->portMask);
}

/*****************************************************************************
*
* Function: static inline void ElectrodeGnd(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Description: Drive electrode pins low level
*
*****************************************************************************/
static inline void ElectrodeGnd(tElecStruct *pElectrodeStruct)
{
	// Drive electrode GPIO low
	REG_RMW32(&(pElectrodeStruct->gpioBasePtr->PCOR),1 << pElectrodeStruct->pinNumberElec,1 << pElectrodeStruct->pinNumberElec);
    // Configure electrode pin as GPIO
	pElectrodeStruct->portBasePtr->PCR[pElectrodeStruct->pinNumberElec] = PCR_GPIO;

    // Drive Cext GPIO low
	REG_RMW32(&(pElectrodeStruct->gpioBasePtr->PCOR),1 << pElectrodeStruct->pinNumberCext, 1 << pElectrodeStruct->pinNumberCext);
    // Configure Cext pin as GPIO
	pElectrodeStruct->portBasePtr->PCR[pElectrodeStruct->pinNumberCext] = PCR_GPIO;

    // Configure Electrode and Cext pins as outputs at the same time
	REG_WRITE32(&(pElectrodeStruct->gpioBasePtr->PDDR), ((REG_READ32(&(pElectrodeStruct->gpioBasePtr->PDDR))) | (pElectrodeStruct->portMask)));
}

/*****************************************************************************
*
* Function: static inline void ElectrodeFloat(tElecStruct *pElectrodeStruct)
*
* Input: Address of a single electrode structure in electrodes structure array
*
* Description: Configure electrode pins as an inputs (high impedance)
*
*****************************************************************************/
static inline void ElectrodeFloat(tElecStruct *pElectrodeStruct)
{
    // Configure Electrode and Cext pins as inputs at the same time
	REG_WRITE32(&(pElectrodeStruct->gpioBasePtr->PDDR), ((REG_READ32(&(pElectrodeStruct->gpioBasePtr->PDDR))) & (~(pElectrodeStruct->portMask))));
}

#endif /* __GPIO_INLINE_FCN1_H */
