/****************************************************************************//*!
*
* @file     power_mode.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Power modes for S32K144
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/main.h>
#include <TS/power_mode.h>
#include "S32K144.h"

/*******************************************************************************
* Variables
******************************************************************************/
uint8_t  lowPowerModeCtrl, lowPowerModeEnable;

/*****************************************************************************
*
* Function: void Run_to_VLPS(void)
*
* Description: Enter VLPS. All main clocks are gated off.
*
*****************************************************************************/
void Run_to_VLPS(void)
{
	// Disable system oscillator clock monitor
	SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCCM_MASK;
	// Disable PLL clock monitor
	SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLCM_MASK;
	 // Enable very low power modes
	SMC->PMPROT |= SMC_PMPROT_AVLP_MASK;
	// Enable Stop Modes in the PMC
	SMC->PMCTRL |= SMC_PMCTRL_STOPM(0b10);
	//Set bias enable bit in PMC
	PMC->REGSC |= PMC_REGSC_BIASEN_MASK;
	// Enable Stop Modes in the Core
	S32_SCB ->SCR |= S32_SCB_SCR_SLEEPDEEP_MASK;
    // Transition from RUN to VLPS
	if(SMC->PMSTAT == RUN)
	{
		// Go to VLPS mode
		asm("WFI");
	}
	else
	{
		// Error trap
		while(1);
	}
}

/*****************************************************************************
*
* Function: void Run_to_VLPR(void)
*
* Description: Transition from RUN to VLPR
*
*****************************************************************************/
void Run_to_VLPR(void)
{
	if(SMC->PMSTAT==RUN)
	{
		SMC->PMCTRL=SMC_PMCTRL_RUNM(0b10);

		// Wait for transition
		while(SMC->PMSTAT!=VLPR)
		{};
	}
	else
	{
		// Error trap
		while(1);
	}
}

/*****************************************************************************
*
* Function: void Run_to_VLPR(void)
*
* Description:  Transition from VLPR to VLPW
*
*****************************************************************************/
void VLPR_to_VLPS(void)
{
	// Enable Stop Modes in the PMC
	SMC->PMCTRL|=SMC_PMCTRL_STOPM(0b10);

	// Enable Stop Modes in the core
	S32_SCB ->SCR|=S32_SCB_SCR_SLEEPDEEP_MASK;

	if(SMC->PMSTAT==VLPR)
	{
		// Move to to VLPS
		asm("WFI");
	}
	else
	{
		// Error trap
		while(1);
	}
}
