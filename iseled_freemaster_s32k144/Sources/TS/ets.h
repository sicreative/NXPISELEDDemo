/****************************************************************************//*!
*
* @file     ets.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Electrode touch sense routines for S32K144
*
*******************************************************************************/
#ifndef __ETS_H
#define __ETS_H

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include "S32K144.h"

/*******************************************************************************
* Calibration after power-up and reset
* Define number of cycles to calculate DC tracker power-up value
* Electrodes sensing period 1ms period
******************************************************************************/
#define NUMBER_OF_CYCLES_DCTRACKER_PWRUP  100

/*******************************************************************************
* Touch sense application power-up init done
******************************************************************************/
#define TOUCH_SENSE_APP_PWRUP_INIT_DONE  0x0FFF

/*******************************************************************************
* Return to the wake-up function period (1s)
******************************************************************************/
#if DECIMATION_FILTER
	#define ELEC_WAKEUP_ACTIVATE_COUNTER    (1000 / ELECTRODES_SENSE_PERIOD_DF)
#else
	#define ELEC_WAKEUP_ACTIVATE_COUNTER    (1000 / ELECTRODES_SENSE_PERIOD)
#endif
/*******************************************************************************
* PCR defines
******************************************************************************/
// Configure pin as GPIO, clear ISF, ISF disabled, ALT1=GPIO, high drive strength, disable pulls, fast slew rate
#define PCR_GPIO	0x01000140
// Configure pin as analog input
#define PCR_ANA		0x00000000

/*******************************************************************************
* Type defines
******************************************************************************/
typedef union
{
	uint8_t byte;
    struct
    {
    	uint8_t selfTrimDone:1;                // LSB
    	uint8_t na:7;                          // MSB
    }bit;
}tElecStatus;

typedef struct
{
	// Hardware
	ADC_Type   *adcBasePtr;
	uint32_t   adcChNum:32;
	PORT_Type  *portBasePtr;
	GPIO_Type  *gpioBasePtr;
	uint32_t   pinNumberElec:32;
	uint32_t   pinNumberCext:32;
	uint32_t   portMask:32;
	uint32_t   ELEC_DMAMUX;
	uint32_t   ELEC_TRGMUX;
}
tElecStruct;

// guard pin hardware config
typedef struct
{
	// Hardware
	PORT_Type  *portBasePtr;
	GPIO_Type  *gpioBasePtr;
	uint32_t   pinNumberGuard:32;
	uint32_t   portMask;

} tGuardStruct;

/*******************************************************************************
* Function prototypes
******************************************************************************/
void ElectrodeStructureInit(void);
void ElectrodeTouchSenseInit(void);
void ElectodeBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr);
void ElectodeCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr);
int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift);
void DCTrackerShiftDecrease(void);
void DCTrackerShiftIncrease(void);

void ElectrodeSelfTrimSense(void);
void ElectrodeWakeElecSense(void);
void ElectrodeWakeAndTouchElecSense(void);
void ElectrodeTouchElecSense(void);
void ElectrodeADCchannelOffset(void);

void ElectrodeSensingCyclesChange(void);
void ElectrodeSensingCyclesChangeEGS(void);
void ElectrodeTouchDetect(uint32_t electrodeNum);

void FrequencyHop(void);

void DecimationFilter(uint32_t electrodeNum);

void ElecOversamplingActivation(void);
void ElecOversamplingDeactivation(void);

void ElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum);

void GuardStructureInit(void);

void ElectrodeCapToVoltConvELCH_Proximity(uint32_t electrodeNum);
void ElectrodeCapToVoltConvELCH_DMA_Proximity(uint32_t electrodeNum);
#endif /* __ETS_H */

