/****************************************************************************//*!
*
* @file     timer.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    PCC routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/ets.h>
#include <TS/main.h>
#include <TS/power_mode.h>
#include <TS/Peripherals/pcc.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: void PCC_Init(uint8_t clkMode)
*
* Description: Init PCC
*
*****************************************************************************/
void PCC_Init(uint8_t clkMode)
{
    // PORTA 
    PCC->PCCn[PCC_PORTA_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTB 
    PCC->PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTC 
    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTD 
    PCC->PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK;
    // PORTE 
    PCC->PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK;

    // Configure PCC based on selected clock mode
    switch (clkMode)
    {
    	case (RUN_FIRC):
		{
    	    // LPTMR0
    		if(PCC->PCCn[PCC_LPTMR0_INDEX] & 0x40000000){PCC->PCCn[PCC_LPTMR0_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPTMR0_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division

    	    // ADC0
    	    if(PCC->PCCn[PCC_ADC0_INDEX] & 0x40000000){PCC->PCCn[PCC_ADC0_INDEX]  = 0x80000000;}
    	    PCC->PCCn[PCC_ADC0_INDEX] = 0xC3000000;			// enable clock, use FIRCDIV2 clock, no clock division

    	    // ADC1
    	    if(PCC->PCCn[PCC_ADC1_INDEX] & 0x40000000){PCC->PCCn[PCC_ADC1_INDEX]  = 0x80000000;}
    	    PCC->PCCn[PCC_ADC1_INDEX] = 0xC3000000;			// enable clock, use FIRCDIV2 clock, no clock division

    	    // FTM2
    	    if(PCC->PCCn[PCC_FTM2_INDEX] & 0x40000000){PCC->PCCn[PCC_FTM2_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_FTM2_INDEX] = 0xC0000000;         // Enable clock, no clock division

#if FREQUENCY_HOPPING
    		//LPIT
    		if(PCC->PCCn[PCC_LPIT_INDEX] & 0x40000000){PCC->PCCn[PCC_LPIT_INDEX]  = 0x80000000;}
    		PCC->PCCn[PCC_LPIT_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division
#endif

#if(LOW_POWER_MODE == LPM_DISABLE)
    	    // LPUART1
    	    PCC->PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK;
    	    PCC->PCCn[PCC_LPUART1_INDEX] = 0xC3000000;		// enable clock, use FIRCDIV2 clock, no clock division
#endif

    	    // DMAMUX
    	    PCC->PCCn[PCC_DMAMUX_INDEX] |= PCC_PCCn_CGC_MASK; 		// clock enable

    		break;
		}
    }
}
