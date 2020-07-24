/****************************************************************************//*!
*
* @file     adc.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    ADC routines
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/ets.h>
#include <TS/main.h>
#include <TS/power_mode.h>
#include <TS/Peripherals/adc.h>
#include "S32K144.h"


/*******************************************************************************
* Variables
*******************************************************************************/
extern int16_t calibrationGainADC0;
extern int16_t calibrationGainADC1;

/*****************************************************************************
*
* Function: void ADC0_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
*
* Description: Init ADC0
*
* Note: sampleTime value in range from 2 to 255
*
*****************************************************************************/
void ADC0_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
{
	// Configure ADC based on selected clock mode
	switch (clkMode)
    {
    	case (RUN_FIRC):
		{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
    		// set module connections between PDB and ADCs
    		SIM->ADCOPT |= SIM_ADCOPT_ADC0PRETRGSEL(0b00)			// ADC0 pretrigger source select: 0b00: PDB , 0b01= TRGMUX , 0b10=SW
    						| SIM_ADCOPT_ADC0SWPRETRG(0b000)		// ADC0 SW pretrigger source select: 0b000: SW pretrig disabled , 0b100= SW pretrig 0 , 0b101= SW pretrig 1 , 0b110= SW pretrig 2, 0b111= SW pretrig 3
							| SIM_ADCOPT_ADC0TRGSEL(0b0);

    	    // ADC input clock selected in PCC, FIRC 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    // software trigger, no compare, DMA on, default VREFH/L
    	    ADC0->SC2 = 0x00000004;
    	    // Conv. Complete Interrupt enable: 0=disabled , 1=enabled
    	    ADC0->SC1[0] |= ADC_SC1_AIEN(0b0);

    		break;
#else
    	    // ADC input clock selected in PCC, FIRC 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC0->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC0->CFG2 = sampleTime;
    	    // software trigger, no compare, no DMA, default VREFH/L
    	    ADC0->SC2 = 0x00000000;

    		break;
#endif
		}
    }
}

/*****************************************************************************
*
* Function: void ADC1_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
*
* Description: Init ADC1
*
* Note: sampleTime value in range from 2 to 255
*
*****************************************************************************/
void ADC1_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode)
{
	// Configure ADC based on selected clock mode
	switch (clkMode)
    {
    	case (RUN_FIRC):
		{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
    		// set module connections between PDB and ADCs
    		SIM->ADCOPT |= SIM_ADCOPT_ADC1PRETRGSEL(0b00)			// ADC0 pretrigger source select: 0b00: PDB , 0b01= TRGMUX , 0b10=SW
    						| SIM_ADCOPT_ADC1SWPRETRG(0b000)		// ADC0 SW pretrigger source select: 0b000: SW pretrig disabled , 0b100= SW pretrig 0 , 0b101= SW pretrig 1 , 0b110= SW pretrig 2, 0b111= SW pretrig 3
							| SIM_ADCOPT_ADC1TRGSEL(0b0);

    	    // ADC input clock selected in PCC, FIRC 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC1->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC1->CFG2 = sampleTime;
    	    // software trigger, no compare, DMA on, default VREFH/L
    	    ADC1->SC2 = 0x00000004;
    	    // Conv. Complete Interrupt enable: 0=disabled , 1=enabled
    	    ADC1->SC1[0] |= ADC_SC1_AIEN(0b0);

    		break;
#else
    	    // ADC input clock selected in PCC, FIRC 48MHz
    	    // Should be max 64MHz, divide by 1
    	    // ADC input clock = 48/1 = 48MHz, tclk = 21ns
    	    // 12bit resolution, ADCK division by 1, no internal alternate clocks but only selectable via PCC
    	    ADC1->CFG1 = 0x00000004;
    	    // set sample time, 11 - 1 = 10, 10 x 18ns = 0.18us
    	    ADC1->CFG2 = sampleTime;
    	    // software trigger, no compare, no DMA, default VREFH/L
    	    ADC1->SC2 = 0x00000000;

    		break;
#endif
		}
    }
}

#if (NUMBER_OF_USED_ADC_MODULES == 2)
/*****************************************************************************
 *
 * Function: void ADCs_simultaneous_HW_trigger_preset(void)
 *
 * Description: Preset configuration for both ADC0 and ADC1 for voltage digitalization
 * 				at the same time
 *
 *****************************************************************************/
void ADCs_SimultaneousHWtrigger(void)
{
	// Voltage digitalization preparations before generating SIM_sw_trigger to initiate HW trigger of the ADC

	// Hardware trigger select
	ADC1->SC2 |= ADC_SC2_ADTRG(1);
	ADC0->SC2 |= ADC_SC2_ADTRG(1);

	// Select sim_sw_trigger as source for TRGMUX
	// SIM_SW_TRIG -> TRGMUX_ADC0(out12) -> ADC0_ADWHT
	TRGMUX->TRGMUXn[TRGMUX_ADC0_INDEX]=TRGMUX_TRGMUXn_SEL0(63);
	// SIM_SW_TRIG -> TRGMUX_ADC1(out16) -> ADC1_ADWHT
	TRGMUX->TRGMUXn[TRGMUX_ADC1_INDEX]=TRGMUX_TRGMUXn_SEL0(63);

	// ADC0 & ADC1: software pretrigger, software pretrigger 0,TRGMUX trigger source
	SIM->ADCOPT = SIM_ADCOPT_ADC1PRETRGSEL(2) | SIM_ADCOPT_ADC1SWPRETRG(4) | \
			SIM_ADCOPT_ADC1TRGSEL(1) | SIM_ADCOPT_ADC0PRETRGSEL(2) | \
			SIM_ADCOPT_ADC0SWPRETRG(4) | SIM_ADCOPT_ADC0TRGSEL(1);

	// Reset the sample time (different conversion trigger moment in time)
	ADC0->CFG2 = ADC_HWTRIGGER_SLIDER_SAMPLE_TIME;
	ADC1->CFG2 = ADC_HWTRIGGER_SLIDER_SAMPLE_TIME;
}
#endif
/*****************************************************************************
 *
 * Function: void ADCs_SetBackToSWtrigger(void)
 *
 * Description: Preset configuration for both ADC0 and ADC1 for
 * 				voltage digitalization one by one
 *
 *****************************************************************************/
void ADCs_SetBackToSWtrigger(void)
{
	// ADC0 & ADC1: software pretrigger, software pretrigger 0, PDB trigger source
	SIM->ADCOPT = SIM_ADCOPT_ADC1PRETRGSEL(0) | SIM_ADCOPT_ADC1SWPRETRG(0) | \
			SIM_ADCOPT_ADC1TRGSEL(0) | SIM_ADCOPT_ADC0PRETRGSEL(0) | \
			SIM_ADCOPT_ADC0SWPRETRG(0) | SIM_ADCOPT_ADC0TRGSEL(0);

	// Software trigger select
	ADC1->SC2 &= ~ADC_SC2_ADTRG(1);
	ADC0->SC2 &= ~ADC_SC2_ADTRG(1);

	// Software trigger, no compare, no DMA, default VREFH/L
	//ADC1->SC2 = 0x00000000;
	//ADC0->SC2 = 0x00000000;
	//dma on
/*	ADC1->SC2 = 0x00000004;
	ADC0->SC2 = 0x00000004;*/

	// Reset the sample time (different conversion trigger moment in time)
	ADC0->CFG2 = ADC_SWTRIGGER_SAMPLE_TIME;
	ADC1->CFG2 = ADC_SWTRIGGER_SAMPLE_TIME;
}

/*****************************************************************************
*
* Function: int16_t ADC0_Calibration(void)
*
* Description: Init ADC0 calibration
*
*****************************************************************************/
int16_t ADC0_Calibration(void){

	//ADC clock divided by 2 (48/2=24MHz)
	ADC0->CFG1=0x00000024;

	// Clear registers
	ADC0->CLPS=0x00000000;
	ADC0->CLP3=0x00000000;
	ADC0->CLP2=0x00000000;
	ADC0->CLP1=0x00000000;
	ADC0->CLP0=0x00000000;
	ADC0->CLPX=0x00000000;
	ADC0->CLP9=0x00000000;

	/*// Clear registers offset
	ADC0->CLP0_OFS=0x00000000;
	ADC0->CLP1_OFS=0x00000000;
	ADC0->CLP2_OFS=0x00000000;
	ADC0->CLP3_OFS=0x00000000;
	ADC0->CLPX_OFS=0x00000000;
	ADC0->CLP9_OFS=0x00000000;
	ADC0->CLPS_OFS=0x00000000;*/

	// Hardware averaging enable, 32 samples, start calibration
	ADC0->SC3 = ADC_SC3_AVGE(1) | ADC_SC3_AVGS(3) | ADC_SC3_CAL(1);

	// Wait for conversion complete flag
	while(ADC0->SC1[0] < 0x80)
	{}

	// Hardware averaging off
	ADC0->SC3 = ADC_SC3_AVGE(0) | ADC_SC3_AVGS(0);

	// Return the gain register value
	return ADC0->G;
}

/*****************************************************************************
*
* Function: int16_t ADC1_Calibration(void)
*
* Description: Init ADC1 calibration
*
*****************************************************************************/
int16_t ADC1_Calibration(void){

	//ADC clock divided by 2 (48/2=24MHz)
	ADC1->CFG1=0x00000024;

	// Clear registers
	ADC1->CLPS=0x00000000;
	ADC1->CLP3=0x00000000;
	ADC1->CLP2=0x00000000;
	ADC1->CLP1=0x00000000;
	ADC1->CLP0=0x00000000;
	ADC1->CLPX=0x00000000;
	ADC1->CLP9=0x00000000;

	/*// Clear registers offset
	ADC1->CLP0_OFS=0x00000000;
	ADC1->CLP1_OFS=0x00000000;
	ADC1->CLP2_OFS=0x00000000;
	ADC1->CLP3_OFS=0x00000000;
	ADC1->CLPX_OFS=0x00000000;
	ADC1->CLP9_OFS=0x00000000;
	ADC1->CLPS_OFS=0x00000000;*/

	// Hardware averaging enable, 32 samples, start calibration
	ADC1->SC3 = ADC_SC3_AVGE(1) | ADC_SC3_AVGS(3) | ADC_SC3_CAL(1);

	// Wait for conversion complete flag
	while(ADC1->SC1[0] < 0x80)
	{}

	// Hardware averaging off
	ADC1->SC3 = ADC_SC3_AVGE(0) | ADC_SC3_AVGS(0);

	// Return the gain register value
	return ADC1->G;
}

/*****************************************************************************
*
* Function: void ClearADCsGain(void)
*
* Description: Clear ADC0 and ADC1 gain registers
*
*****************************************************************************/
void ClearADCsGain(void){

	ADC0->G = 0x00000000;

	ADC1->G = 0x00000000;
}

/*****************************************************************************
*
* Function: void SetADCsGain(void)
*
* Description: Set ADC0 and ADC1 gain registers on calibrated value
*
*****************************************************************************/
void SetADCsGain(void){

	ADC0->G = calibrationGainADC0;

	ADC1->G = calibrationGainADC1;
}
