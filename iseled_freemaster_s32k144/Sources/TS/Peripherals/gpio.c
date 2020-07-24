/****************************************************************************//*!
*
* @file     gpio.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    GPIO routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Peripherals/gpio.h>
#include "S32K144.h"

/*****************************************************************************
*
* Function: void RGB_LED_Init(void)
*
* Description: Init RGB LED pins
*
*****************************************************************************/
void RGB_LED_Init(void)
{
    // PTD0, RGB LED BLUE
    // PTD0 GPIO pin
    PORTD->PCR[0] = PORT_PCR_MUX(1);
    // Turn OFF LED
    //REG_RMW32(&(PTD->PSOR), 1 << 0, 1 << 0);
    PTD->PSOR = 1 << 0;
    // PTD0 configured as output
    //REG_RMW32(&(PTD->PDDR), 1 << 0, 1 << 0);
    PTD->PDDR |= 1 << 0;

	// PTD15, RGB LED RED
    // PTD15 GPIO pin
    PORTD->PCR[15] = PORT_PCR_MUX(1);
    // Turn OFF LED
    //REG_RMW32(&(PTD->PSOR), 1 << 15, 1 << 15);
    PTD->PSOR = 1 << 15;
    // PTD15 configured as output
    //REG_RMW32(&(PTD->PDDR), 1 << 15, 1 << 15);
    PTD->PDDR |= 1 << 15;

    // PTD16, RGB LED GREEN
    // PTD16 GPIO pin
    PORTD->PCR[16] = PORT_PCR_MUX(1);
    // Turn OFF LED
    //REG_RMW32(&(PTD->PSOR), 1<<16, 1<<16);
    PTD->PSOR = 1 << 16;
    // PTD16 configured as output
    //REG_RMW32(&(PTD->PDDR), 1<<16, 1<<16);
    PTD->PDDR |= 1 << 16;
}

/*****************************************************************************
*
* Function: void Backlight_LED_Init(void)
*
* Description: Init backlight LED pins
*
*****************************************************************************/
void Backlight_LED_Init(void)
{
    // LED0
    // PTD12 GPIO pin
    PORTD->PCR[12] = PORT_PCR_MUX(2);

    // LED1
    // PTC12 GPIO pin
    PORTC->PCR[12] = PORT_PCR_MUX(3);

    // LED2
    // PTD5 GPIO pin
    PORTD->PCR[5] = PORT_PCR_MUX(2);

    // LED3
    // PTD10 GPIO pin
    PORTD->PCR[10] = PORT_PCR_MUX(2);

    // LED4
    // PTD13 GPIO pin
    PORTD->PCR[13] = PORT_PCR_MUX(2);

    // LED5
    // PTD11 GPIO pin
    PORTD->PCR[11] = PORT_PCR_MUX(2);

    // LED6
    // PTD14 GPIO pin
    PORTD->PCR[14] = PORT_PCR_MUX(2);
}
