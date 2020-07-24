/****************************************************************************//*!
*
* @file     scg.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    SCG routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/main.h>
#include <TS/power_mode.h>
#include <TS/Peripherals/scg.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: void SCG_Init(uint8_t clkMode)
*
* Description: Init SCG
*
*****************************************************************************/
void SCG_Init(uint8_t clkMode)
{
	// Configure SCG based on selected clock mode
	switch(clkMode)
	{
		case(RUN_FIRC):
		{
	    	/* FIRC mode after reset */
	    	/* FIRC 48MHz, core clock 48MHz, system clock 48MHz, bus clock 48MHz, Flash clock 24MHz */
			/* FIRC Configuration 48MHz */
			SCG->FIRCDIV = SCG_FIRCDIV_FIRCDIV1(0b01)	   /*FIRC DIV1=1*/
						  |SCG_FIRCDIV_FIRCDIV2(0b01);	   /*FIRC DIV2=1*/

			SCG->FIRCCFG =SCG_FIRCCFG_RANGE(0b00);	/* Fast IRC trimmed 48MHz*/

			while(SCG->FIRCCSR & SCG_FIRCCSR_LK_MASK); /*Is PLL control and status register locked?*/

			SCG->FIRCCSR = SCG_FIRCCSR_FIRCEN_MASK;  /*Enable FIRC*/

			while(!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK)); /*Check that FIRC clock is valid*/

			/* RUN Clock Configuration */
			SCG->RCCR=SCG_RCCR_SCS(0b0011) /* FIRC as clock source*/
					  |SCG_RCCR_DIVCORE(0b00) /* DIVCORE=1, Core clock = 48 MHz*/
					  |SCG_RCCR_DIVBUS(0b00)  /* DIVBUS=1, Bus clock = 48 MHz*/
					  |SCG_RCCR_DIVSLOW(0b01);/* DIVSLOW=2, Flash clock= 24 MHz*/


			break;
		}
	}

	// Enable very low power modes
	SMC->PMPROT = SMC_PMPROT_AVLP_MASK;
}
