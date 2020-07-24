/****************************************************************************//*!
*
* @file     flextimer.c
*
* @version  1.0.0.0
*
* @date     July-2017
*
* @brief    FlexTimer routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include <TS/ets.h>
//#include <TS/FreeMASTER/S32xx/freemaster.h>
#include <TS/main.h>
#include <TS/Peripherals/flextimer.h>
#include "S32K144.h"

/*******************************************************************************
* Variables
******************************************************************************/
// Backlight PWM duty cycle from 0 to 100
uint16_t  backlightPWMDutyCycle;

/*****************************************************************************
*
* Function: FTM2_Init(void)
*
* Description: Init FTM2 module:
*              - PWM period 200Hz
*              - Heater PWM duty cycle from 0 to 100
*
*****************************************************************************/
void FTM2_Init(void)
{
	// Set Modulo (200Hz PWM frequency @ 6MHz FIRCDIV1 clock)
	FTM2->MOD = FTM_MOD_MOD(1875-1);
	// Counter init value
	FTM2->CNTIN = FTM_CNTIN_INIT(0);
	// Enable PWM high-true pulses (clear output on Match)
	FTM2->CONTROLS[0].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[4].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	FTM2->CONTROLS[7].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	// Match value reset
	FTM2->CONTROLS[0].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[1].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[2].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[3].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[4].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[5].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[6].CnV = FTM_CnV_VAL(0);
	FTM2->CONTROLS[7].CnV = FTM_CnV_VAL(0);
	// Reset counter
	FTM2->CNT = 0;
	// Divide clock by 128,FTM system clock, enable PWM channel 0 - 7
	FTM2->SC = FTM_SC_PS(7) | FTM_SC_CLKS(1) | 0x00FF0000;
}

/*****************************************************************************
*
* Function: void LoadBacklightPWMDutyCycle(uint16_t PWMDutyCycle)
*
* Description: Keyboard backlight PWM duty cycle in range from 0 to 100, step 1
*
*****************************************************************************/
void LoadBacklightPWMDutyCycle(uint16_t PWMDutyCycle)
{
	uint16_t   PWMDutyCycleRecalculated;


	// Load PWM duty cycle
	if (PWMDutyCycle == 100)
	{
		// LEDs ON
		PWMDutyCycleRecalculated = 1875;
	}
	else
	{
		// Recalculate PWM duty match value to resolution of FlexTimer PWM
		PWMDutyCycleRecalculated = PWMDutyCycle * 18;
	}

	// Match value reset
	FTM2->CONTROLS[0].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[1].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[2].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[3].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[4].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[5].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[6].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
	FTM2->CONTROLS[7].CnV = FTM_CnV_VAL(PWMDutyCycleRecalculated);
}
