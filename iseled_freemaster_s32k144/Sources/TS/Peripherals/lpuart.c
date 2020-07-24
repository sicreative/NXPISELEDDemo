/****************************************************************************//*!
*
* @file     lpuart.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    LPUART1 routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/main.h>
#include <TS/power_mode.h>
#include <TS/Peripherals/lpuart.h>
#include <TS/Peripherals/scg.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: void LPUART1_Init(uint8_t clkMode)
*
* Description: Init LPUART1
*
*****************************************************************************/
void LPUART1_Init(uint8_t clkMode)
{
	// Configure LPUART1 based on selected clock mode
	switch (clkMode)
    {
    	case (RUN_FIRC):
		{
    		// UART1 configured to run from FIRCDIV2
    	    LPUART1->BAUD = LPUART_BAUD_OSR(15) | LPUART_BAUD_SBR((48000000) / (16 * UART_BAUD_RATE));
    	    LPUART1->CTRL = LPUART_CTRL_RE_MASK | LPUART_CTRL_TE_MASK;

    		break;
		}
    }
}
