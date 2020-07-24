/****************************************************************************//*!
*
* @file  	slider.h
*
* @version  1.0.0.0
*
* @date     December-2017
*
* @brief    Slider electrode touch sense routines for S32K144
*
*******************************************************************************/

#ifndef SLIDER_H_
#define SLIDER_H_

/******************************************************************************
* Typedefs and structures       (scope: module-local)
******************************************************************************/
typedef unsigned char       tBool;          /*!< basic boolean type */

#ifndef FALSE
#define FALSE ((tBool)0)                    /*!< Boolean type FALSE constant */
#endif

#ifndef TRUE
#define TRUE ((tBool)1)                     /*!< Boolean type TRUE constant */
#endif

/*******************************************************************************
* Function prototypes
******************************************************************************/
void SliderElectrodeStructureInit(void);
void SliderElectrodeADCchannelOffset(void);
void SliderElectrodeTouchSenseInit(void);
void SliderDCTrackerShiftDecrease(void);
void SliderDCTrackerShiftIncrease(void);
void SliderElectrodeSelfTrimSense(void);
void SliderElectrodeSelfTrim(void);
void SliderDifferenceDataHelperThresholds(void);

void SliderElectrodeCapToVoltConvELCH(uint32_t sliderElectrodeNum);
void SliderSimultaneousElectrodeCapToVoltConvELCH(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num);
void SliderElectrodeCapToVoltConvELCHGuard(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num);
void SliderElectrodesTouchElecSense(void);
void SliderDataCalculation(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num);
void SliderDifferenceDataElectrodeTouchQualify (uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num);

void SliderelectrodeWakeElecSense(void);
void SliderelectrodeWakeAndTouchElecSense(void);

void FindMinimumAndDeleteIt(int16_t sliderInputSamplesArray[]);
void FindMaximumAndDeleteIt(int16_t sliderInputSamplesArray[]);
void SliderSensingCyclesChange(void);
void SliderSensingCyclesChangeEGS(void);

void SliderElectrodeWakeElecFilterLoad(void);
void SliderNoiseCanceling(void);

void SliderElecOversamplingActivation(void);
void SliderElecOversamplingDeactivation(void);

void SliderElectrodeTouchDetect(uint32_t electrodeNum);

void SliderDecimationFilter(uint32_t electrodeNum);

void SliderElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum);
void SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num);

#endif /* SLIDER_H_ */
