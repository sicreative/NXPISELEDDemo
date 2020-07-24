/****************************************************************************//*!
*
* @file     adc.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    ADC routines header file
*
*******************************************************************************/
#ifndef __ADC_H
#define __ADC_H

/*******************************************************************************
* Function prototypes
******************************************************************************/
void ADC0_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode);
void ADC1_Init(uint32_t sampleTime, uint32_t avgSel, uint8_t clkMode);
void ADCs_SimultaneousHWtrigger(void);
void ADCs_SetBackToSWtrigger(void);
int16_t ADC0_Calibration(void);
int16_t ADC1_Calibration(void);
void ClearADCsGain(void);
void SetADCsGain(void);

#endif /* __ADC_H */
