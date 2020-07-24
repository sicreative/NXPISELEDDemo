/****************************************************************************//*!
*
* @file  	ets.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Electrode touch sense routines for S32K144
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include <TS/ets.h>
#include <TS/filter.h>
#include <TS/main.h>
#include <TS/Peripherals/adc.h>
#include <TS/Peripherals/adc_inline_fcn1.h>
#include <TS/Peripherals/gpio.h>
#include <TS/Peripherals/gpio_inline_fcn1.h>
#include <TS/Peripherals/tcd.h>
#include <TS/Peripherals/timer.h>
#include <TS/slider.h>
#include "S32K144.h"
#include "tcd.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
// Electrodes status
tElecStatus   electrodesStatus;

// Electrode structure
tElecStruct  elecStruct[NUMBER_OF_ELECTRODES], *pElecStruct;
// Guard pin structure
tGuardStruct guardStruct[1];

#if SLIDER_ENABLE
extern tElecStruct  sliderElecStruct[NUMBER_OF_SLIDER_ELECTRODES];
#endif

// Electrode capacitance to equivalent voltage conversion
int16_t   numberOfElectrodeSensingCyclesPerSample;
int32_t   adcDataElectrodeDischargeRaw[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
int16_t   adcDataElectrodeDischargeRawSample[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE];
uint32_t   adcDataElectrodeDischargeRawCalc;
uint8_t   elecNum, elecNumAct;
uint16_t sampleNum;
uint16_t  chargeDistributionPeriod, chargeDistributionPeriodTmp;

// Proximity EGS
#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
int16_t   adcDataElectrodeDischargeRawSampleProximity[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY];
#endif

// Wake up
uint16_t   electrodeWakeUpActivateCounter;
uint16_t   electrodesVirtualEGSActivateCounter;
extern uint8_t sliderElecNumAct;

// Electrode DC tracker self-trim after power-up or reset
int32_t   adcDataElectrodeDischargeBuffer[NUMBER_OF_ELECTRODES];
uint16_t  adcDataElectrodeDischargeBufferCounter[NUMBER_OF_ELECTRODES];

// DC Tracker
int32_t   DCTrackerDataBufferRaw[NUMBER_OF_ELECTRODES];
int32_t   DCTrackerDataBuffer[NUMBER_OF_ELECTRODES];
uint8_t   DCTrackerDataShift[NUMBER_OF_ELECTRODES];

// LP Filter
tFrac32   LPFilterData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
#if SLIDER_ENABLE
uint8_t   LPFilterType[NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES];
#else
uint8_t   LPFilterType[NUMBER_OF_ELECTRODES];
#endif
// Detector
int32_t   detectorThresholdTouch[NUMBER_OF_ELECTRODES], detectorThresholdRelease[NUMBER_OF_ELECTRODES];
int16_t   detectorThresholdTouchDelta[NUMBER_OF_ELECTRODES], detectorThresholdReleaseDelta[NUMBER_OF_ELECTRODES];
uint8_t   electrodeTouch[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
uint8_t   electrodesVirtualEGSTouch;

// Touch qualification
uint8_t   electrodeTouchQualified[NUMBER_OF_ELECTRODES];
uint8_t   electrodeTouchQualifiedReport;
uint8_t   electrodeTouchNumberPlusOne;

// Low power mode
extern uint8_t  lowPowerModeCtrl;

// FrequencyHopping
uint8_t   frequencyID,frequencyIDsave;
uint8_t   frequencyHoppingActivation;

// Decimation filter
int32_t   adcDataElectrodeDischargeRawDF[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];

// Oversampling
uint8_t electrodesOversamplingActivationReport;

// DMA control flags
extern volatile uint32_t ready_dma_flag, start_dma_flag;
/*****************************************************************************
 *
 * Function: void ElectrodeStructureInit(void)
 *
 * Description: Electrodes structure init
 *
 *****************************************************************************/
void ElectrodeStructureInit(void)
{
#ifdef ELEC0
	// Load electrode 0 hardware data
	elecStruct[0].adcBasePtr = ELEC0_ADC;
	elecStruct[0].adcChNum = ELEC0_ADC_CHANNEL;
	elecStruct[0].portBasePtr = ELEC0_PORT;
	elecStruct[0].gpioBasePtr = ELEC0_GPIO;
	elecStruct[0].pinNumberElec = ELEC0_ELEC_GPIO_PIN;
	elecStruct[0].pinNumberCext = ELEC0_CEXT_GPIO_PIN;
	elecStruct[0].portMask = ELEC0_PORT_MASK;
	elecStruct[0].ELEC_DMAMUX = ELEC0_DMAMUX;
	elecStruct[0].ELEC_TRGMUX = ELEC0_TRGMUX;
	// Load electrode 0 application data
	DCTrackerDataShift[0] = ELEC0_DCTRACKER_FILTER_FACTOR;
	LPFilterType[0] = ELEC0_LPFILTER_TYPE;
	detectorThresholdTouchDelta[0] = ELEC0_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[0] = ELEC0_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC1
	// Load electrode 1 hardware data
	elecStruct[1].adcBasePtr = ELEC1_ADC;
	elecStruct[1].adcChNum = ELEC1_ADC_CHANNEL;
	elecStruct[1].portBasePtr = ELEC1_PORT;
	elecStruct[1].gpioBasePtr = ELEC1_GPIO;
	elecStruct[1].pinNumberElec = ELEC1_ELEC_GPIO_PIN;
	elecStruct[1].pinNumberCext = ELEC1_CEXT_GPIO_PIN;
	elecStruct[1].portMask = ELEC1_PORT_MASK;
	elecStruct[1].ELEC_DMAMUX = ELEC1_DMAMUX;
	elecStruct[1].ELEC_TRGMUX = ELEC1_TRGMUX;
	// Load electrode 1 application data
	DCTrackerDataShift[1] = ELEC1_DCTRACKER_FILTER_FACTOR;
	LPFilterType[1] = ELEC1_LPFILTER_TYPE;
	detectorThresholdTouchDelta[1] = ELEC1_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[1] = ELEC1_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC2
	// Load electrode 2 hardware data
	elecStruct[2].adcBasePtr = ELEC2_ADC;
	elecStruct[2].adcChNum = ELEC2_ADC_CHANNEL;
	elecStruct[2].portBasePtr = ELEC2_PORT;
	elecStruct[2].gpioBasePtr = ELEC2_GPIO;
	elecStruct[2].pinNumberElec = ELEC2_ELEC_GPIO_PIN;
	elecStruct[2].pinNumberCext = ELEC2_CEXT_GPIO_PIN;
	elecStruct[2].portMask = ELEC2_PORT_MASK;
	elecStruct[2].ELEC_DMAMUX = ELEC2_DMAMUX;
	elecStruct[2].ELEC_TRGMUX = ELEC2_TRGMUX;
	// Load electrode 2 application data
	DCTrackerDataShift[2] = ELEC2_DCTRACKER_FILTER_FACTOR;
	LPFilterType[2] = ELEC2_LPFILTER_TYPE;
	detectorThresholdTouchDelta[2] = ELEC2_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[2] = ELEC2_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC3
	// Load electrode 3 hardware data
	elecStruct[3].adcBasePtr = ELEC3_ADC;
	elecStruct[3].adcChNum = ELEC3_ADC_CHANNEL;
	elecStruct[3].portBasePtr = ELEC3_PORT;
	elecStruct[3].gpioBasePtr = ELEC3_GPIO;
	elecStruct[3].pinNumberElec = ELEC3_ELEC_GPIO_PIN;
	elecStruct[3].pinNumberCext = ELEC3_CEXT_GPIO_PIN;
	elecStruct[3].portMask = ELEC3_PORT_MASK;
	elecStruct[3].ELEC_DMAMUX = ELEC3_DMAMUX;
	elecStruct[3].ELEC_TRGMUX = ELEC3_TRGMUX;
	// Load electrode 3 application data
	DCTrackerDataShift[3] = ELEC3_DCTRACKER_FILTER_FACTOR;
	LPFilterType[3] = ELEC3_LPFILTER_TYPE;
	detectorThresholdTouchDelta[3] = ELEC3_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[3] = ELEC3_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC4
	// Load electrode 4 hardware data
	elecStruct[4].adcBasePtr = ELEC4_ADC;
	elecStruct[4].adcChNum = ELEC4_ADC_CHANNEL;
	elecStruct[4].portBasePtr = ELEC4_PORT;
	elecStruct[4].gpioBasePtr = ELEC4_GPIO;
	elecStruct[4].pinNumberElec = ELEC4_ELEC_GPIO_PIN;
	elecStruct[4].pinNumberCext = ELEC4_CEXT_GPIO_PIN;
	elecStruct[4].portMask = ELEC4_PORT_MASK;
	elecStruct[4].ELEC_DMAMUX = ELEC4_DMAMUX;
	elecStruct[4].ELEC_TRGMUX = ELEC4_TRGMUX;
	// Load electrode 4 application data
	DCTrackerDataShift[4] = ELEC4_DCTRACKER_FILTER_FACTOR;
	LPFilterType[4] = ELEC4_LPFILTER_TYPE;
	detectorThresholdTouchDelta[4] = ELEC4_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[4] = ELEC4_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC5
	// Load electrode 5 hardware data
	elecStruct[5].adcBasePtr = ELEC5_ADC;
	elecStruct[5].adcChNum = ELEC5_ADC_CHANNEL;
	elecStruct[5].portBasePtr = ELEC5_PORT;
	elecStruct[5].gpioBasePtr = ELEC5_GPIO;
	elecStruct[5].pinNumberElec = ELEC5_ELEC_GPIO_PIN;
	elecStruct[5].pinNumberCext = ELEC5_CEXT_GPIO_PIN;
	elecStruct[5].portMask = ELEC5_PORT_MASK;
	elecStruct[5].ELEC_DMAMUX = ELEC5_DMAMUX;
	elecStruct[5].ELEC_TRGMUX = ELEC5_TRGMUX;
	// Load electrode 5 application data
	DCTrackerDataShift[5] = ELEC5_DCTRACKER_FILTER_FACTOR;
	LPFilterType[5] = ELEC5_LPFILTER_TYPE;
	detectorThresholdTouchDelta[5] = ELEC5_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[5] = ELEC5_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC6
	// Load electrode 6 hardware data
	elecStruct[6].adcBasePtr = ELEC6_ADC;
	elecStruct[6].adcChNum = ELEC6_ADC_CHANNEL;
	elecStruct[6].portBasePtr = ELEC6_PORT;
	elecStruct[6].gpioBasePtr = ELEC6_GPIO;
	elecStruct[6].pinNumberElec = ELEC6_ELEC_GPIO_PIN;
	elecStruct[6].pinNumberCext = ELEC6_CEXT_GPIO_PIN;
	elecStruct[6].portMask = ELEC6_PORT_MASK;
	elecStruct[6].ELEC_DMAMUX = ELEC6_DMAMUX;
	elecStruct[6].ELEC_TRGMUX = ELEC6_TRGMUX;
	// Load electrode 6 application data
	DCTrackerDataShift[6] = ELEC6_DCTRACKER_FILTER_FACTOR;
	LPFilterType[6] = ELEC6_LPFILTER_TYPE;
	detectorThresholdTouchDelta[6] = ELEC6_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[6] = ELEC6_RELEASE_THRESHOLD_DELTA;
#endif

#ifdef ELEC7
	// Load electrode 7 hardware data
	elecStruct[7].adcBasePtr = ELEC7_ADC;
	elecStruct[7].adcChNum = ELEC7_ADC_CHANNEL;
	elecStruct[7].portBasePtr = ELEC7_PORT;
	elecStruct[7].gpioBasePtr = ELEC7_GPIO;
	elecStruct[7].pinNumberElec = ELEC7_ELEC_GPIO_PIN;
	elecStruct[7].pinNumberCext = ELEC7_CEXT_GPIO_PIN;
	elecStruct[7].portMask = ELEC7_PORT_MASK;
	elecStruct[7].ELEC_DMAMUX = ELEC7_DMAMUX;
	elecStruct[7].ELEC_TRGMUX = ELEC7_TRGMUX;
	// Load electrode 7 application data
	DCTrackerDataShift[7] = ELEC7_DCTRACKER_FILTER_FACTOR;
	LPFilterType[7] = ELEC7_LPFILTER_TYPE;
	detectorThresholdTouchDelta[7] = ELEC7_TOUCH_THRESHOLD_DELTA;
	detectorThresholdReleaseDelta[7] = ELEC7_RELEASE_THRESHOLD_DELTA;
#endif

	// If EGS is OFF and TS method is oversampling
#if !defined(WAKE_UP_ELECTRODE) && TS_RAW_DATA_CALCULATION == OVERSAMPLING
	// All touch button electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
	}
#endif

#ifdef WAKE_UP_ELECTRODE
#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING) || (DECIMATION_FILTER) || (FREQUENCY_HOPPING)
	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
	}
#endif
#endif
}

#if (GUARD)
/*****************************************************************************
*
* Function: void GuardStructureInit(void)
*
* Description: Guard structure init
*
*****************************************************************************/
void GuardStructureInit(void)
{
	// Load guard hardware data
	guardStruct[0].portBasePtr = GUARD_PORT;
	guardStruct[0].gpioBasePtr = GUARD_GPIO;
	guardStruct[0].pinNumberGuard = GUARD_GPIO_PIN;
	guardStruct[0].portMask = GUARD_PORT_MASK;

	// Guard pin as GPIO
    guardStruct[0].portBasePtr->PCR[guardStruct[0].pinNumberGuard] = PORT_PCR_MUX(0b01);
    // Guard pin output
    guardStruct[0].gpioBasePtr->PDDR |= 1 << guardStruct[0].pinNumberGuard;
}
#endif

/*****************************************************************************
 *
 * Function: void ElectrodeADCchannelOffset(void)
 *
 * Description: Fixes ADC channel number in case it is higher than 15
 *
 *****************************************************************************/
void ElectrodeADCchannelOffset(void)
{
	// All electrodes
	for(elecNum = 0; elecNum<NUMBER_OF_ELECTRODES; elecNum++)
	{
		// ADC channel number higher than 15?
		if(elecStruct[elecNum].adcChNum > 15)
		{
			// Add offset
			elecStruct[elecNum].adcChNum += ELECTRODE_ADC_CHANNEL_OFFSET;
		}
	}
}

/*****************************************************************************
 *
 * Function: void ElectrodeTouchSenseInit(void)
 *
 * Description: Init touch sense variables
 *
 *****************************************************************************/
void ElectrodeTouchSenseInit(void)
{
	// Electrodes structure init
	ElectrodeStructureInit();

#if (GUARD)
	// Guard pin structure init
	GuardStructureInit();
#endif

	// Cext pin ADC channel number correction (in case higher ADC channel number used)
	ElectrodeADCchannelOffset();

	// Electrode and Cext charge distribution period
	chargeDistributionPeriod = 0;

	// Reset electrodes status
	electrodesStatus.byte = 0;

	// Init LP IIR Filter
	FilterIIR1Init();

	// Reset
	electrodeTouchNumberPlusOne = 0;

	// Reset - report that all touch button electrodes proximity released
	electrodesVirtualEGSTouch = 0;

	// Reset the number of sensing cycles per sample to idle (default)
	numberOfElectrodeSensingCyclesPerSample = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE;

	// Reset frequencyID
	frequencyID = 0;
}

/*****************************************************************************
 *
 * Function: void ElectrodeCapToVoltConvELCH(uint32_t electrodeNum)
 *
 * Description: Convert electrode capacitance to equivalent voltage by CPU
 *
 *****************************************************************************/
void ElectrodeCapToVoltConvELCH(uint32_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Electrode capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample); sampleNum++)
	{

		// Distribute Electrode and Cext charge
		ChargeDistribution(&elecStruct[electrodeNum]);
		// Delay to distribute charge
		chargeDistributionPeriodTmp = chargeDistributionPeriod;
		while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

		#if (GUARD)
	    //Guard pin = 0
	    guardStruct[0].gpioBasePtr->PCOR = 1 << guardStruct[0].pinNumberGuard;
		#endif

		// If compiler optimization is not -O3 (is set to none -> -O0), then set TS_ASM_OPTIMIZE macro to 1 in ts_cfg.h
#if(TS_ASM_OPTIMIZE == 1)
		asm volatile (
				/***** ASSEMBLY TEMPLATE ****************************************/
				// Start Cext voltage ADC conversion
				"str %1, [%0, #0]\n\t"    // store %1 to address in %0 with 0 byte offset
				// Redistribute Electrode and Cext charge
				"str %3, [%2, #0]"        // store %3 to address in %2 with 0 byte offset
#if (GUARD)
			// Guard pin = 0
			"\n\tstr %5, [%4, #0]"        // store %5 to address in %4 with 0 byte offset
#endif
				/***** LIST OF OUTPUT OPERANDS **********************************/
				:
				/***** LIST OF INPUT OPERANDS ***********************************/
				:  "r" (&elecStruct[electrodeNum].adcBasePtr->SC1[0]),  // operand %0
				   "r" (elecStruct[electrodeNum].adcChNum),             // operand %1
				   "r" (&elecStruct[electrodeNum].gpioBasePtr->PDDR),   // operand %2
				   "r" ((elecStruct[electrodeNum].gpioBasePtr->PDDR) & ~(elecStruct[electrodeNum].portMask))    // operand %3
#if (GUARD)
			   ,"r" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
			   "r" (guardStruct[0].portMask)  // operand %5
#endif
		);
#else
		// Start Cext voltage ADC conversion
		elecStruct[electrodeNum].adcBasePtr->SC1[0] = elecStruct[electrodeNum].adcChNum;

		// Redistribute Electrode and Cext charge
		ChargeRedistribution(&elecStruct[electrodeNum]);

#if (GUARD)
		//Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif
#endif



		// Equivalent voltage digitalization
		adcDataElectrodeDischargeRawSample[sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);

#if JITTERING
#if (JITTERING_OPTION == 2)
		// Jitter sample rate
		Jitter(adcDataElectrodeDischargeRaw[electrodeNum][frequencyID]);
#endif
#endif
	}

#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Calculate samples sum
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSample[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE)
	{
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (adcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE)
	{
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (adcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_ACTIVE);
	}

#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Calculate samples value average
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSample[sampleNum];
	}
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (int32_t)(adcDataElectrodeDischargeRawCalc / numberOfElectrodeSensingCyclesPerSample);
#else
#error Please select valid TS method in ts_cfg_general.h
#endif

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	R_RMW32(&(DES_GPIO->PSOR), DES_PIN, 1);
#endif

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

/*****************************************************************************
 *
 * Function: void ElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum)
 *
 * Description: Convert electrode capacitance to equivalent voltage by DMA
 *
 *****************************************************************************/
void ElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Start DMA for scanning touch button electrodes
	start_TS_DMA(electrodeNum,(NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample));												// start DMA for electrode
	while (!ready_dma_flag);												// wait for result
	ready_dma_flag = 0;														// reset flag

#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Calculate samples sum
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSample[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE)
	{
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (adcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE)
	{
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (adcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_ACTIVE);
	}

#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Calculate samples value average
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSample); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSample[sampleNum];
	}
	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (int32_t)(adcDataElectrodeDischargeRawCalc / numberOfElectrodeSensingCyclesPerSample);

	#else
#error Please select valid TS method in ts_cfg_general.h
#endif

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	R_RMW32(&(DES_GPIO->PSOR), DES_PIN, 1);
#endif

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
/*****************************************************************************
 *
 * Function: void ElectrodeCapToVoltConvELCH_Proximity(uint32_t electrodeNum)
 *
 * Description: Convert proximity electrode capacitance to equivalent voltage by CPU
 *
 *****************************************************************************/
void ElectrodeCapToVoltConvELCH_Proximity(uint32_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Electrode capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY); sampleNum++)
	{

		// Distribute Electrode and Cext charge
		ChargeDistribution(&elecStruct[electrodeNum]);
		// Delay to distribute charge
		chargeDistributionPeriodTmp = chargeDistributionPeriod;
		while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

		#if (GUARD)
	    //Guard pin = 0
	    guardStruct[0].gpioBasePtr->PCOR = 1 << guardStruct[0].pinNumberGuard;
		#endif

		// If compiler optimization is not -O3 (is set to none -> -O0), then set TS_ASM_OPTIMIZE macro to 1 in ts_cfg.h
#if(TS_ASM_OPTIMIZE == 1)
		asm volatile (
				/***** ASSEMBLY TEMPLATE ****************************************/
				// Start Cext voltage ADC conversion
				"str %1, [%0, #0]\n\t"    // store %1 to address in %0 with 0 byte offset
				// Redistribute Electrode and Cext charge
				"str %3, [%2, #0]"        // store %3 to address in %2 with 0 byte offset
#if (GUARD)
			// Guard pin = 0
			"\n\tstr %5, [%4, #0]"        // store %5 to address in %4 with 0 byte offset
#endif
				/***** LIST OF OUTPUT OPERANDS **********************************/
				:
				/***** LIST OF INPUT OPERANDS ***********************************/
				:  "r" (&elecStruct[electrodeNum].adcBasePtr->SC1[0]),  // operand %0
				   "r" (elecStruct[electrodeNum].adcChNum),             // operand %1
				   "r" (&elecStruct[electrodeNum].gpioBasePtr->PDDR),   // operand %2
				   "r" ((elecStruct[electrodeNum].gpioBasePtr->PDDR) & ~(elecStruct[electrodeNum].portMask))    // operand %3
#if (GUARD)
			   ,"r" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
			   "r" (guardStruct[0].portMask)  // operand %5
#endif
		);
#else
		// Start Cext voltage ADC conversion
		elecStruct[electrodeNum].adcBasePtr->SC1[0] = elecStruct[electrodeNum].adcChNum;

		// Redistribute Electrode and Cext charge
		ChargeRedistribution(&elecStruct[electrodeNum]);
#if (GUARD)
		//Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif

		// Equivalent voltage digitalization
		adcDataElectrodeDischargeRawSampleProximity[sampleNum] = EquivalentVoltageDigitalization(&elecStruct[electrodeNum]);

#if JITTERING
#if (JITTERING_OPTION == 2)
		// Jitter sample rate
		Jitter(adcDataElectrodeDischargeRaw[electrodeNum][frequencyID]);
#endif
#endif
	}

	// Calculate samples sum
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSampleProximity[sampleNum];
	}

	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = adcDataElectrodeDischargeRawCalc;

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	R_RMW32(&(DES_GPIO->PSOR), DES_PIN, 1);
#endif

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}
/*****************************************************************************
 *
 * Function: void ElectrodeCapToVoltConvELCH_DMA_Proximity(uint32_t electrodeNum)
 *
 * Description: Convert proximity electrode capacitance to equivalent voltage by DMA
 *
 *****************************************************************************/
void ElectrodeCapToVoltConvELCH_DMA_Proximity(uint32_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Start DMA for scanning EGS in proximity functionality
	start_TS_DMA(electrodeNum,(NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY));												// start DMA for electrode
	while (!ready_dma_flag);												// wait for result
	ready_dma_flag = 0;														// reset flag

	// Calculate samples sum
	adcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY); sampleNum++)
	{
		adcDataElectrodeDischargeRawCalc = adcDataElectrodeDischargeRawCalc + adcDataElectrodeDischargeRawSampleProximity[sampleNum];
	}

	adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = adcDataElectrodeDischargeRawCalc;

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	R_RMW32(&(DES_GPIO->PSOR), DES_PIN, 1);
#endif

	// Set ADCs gain back to calibrated value
	SetADCsGain();
}
#endif
/*****************************************************************************
 *
 * Function: void ElectodeBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
 *
 * Description: Sum data
 *
 *****************************************************************************/
void ElectodeBufferInitVal(int32_t inputSignal, int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
{
	int32_t  outputSignal;
	uint16_t sumCounter;


	outputSignal = *outputSignalPtr;
	sumCounter = *sumCounterPtr;

	sumCounter++;
	outputSignal = outputSignal + inputSignal;

	*outputSignalPtr = outputSignal;
	*sumCounterPtr = sumCounter;
}

/*****************************************************************************
 *
 * Function: void ElectodeCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
 *
 * Description: Calculate data average value
 *
 *****************************************************************************/
void ElectodeCalInitVal(int32_t *outputSignalPtr, uint16_t *sumCounterPtr)
{
	int32_t  outputSignal;
	uint16_t sumCounter;


	outputSignal = *outputSignalPtr;
	sumCounter = *sumCounterPtr;

	outputSignal = outputSignal / sumCounter;

	*outputSignalPtr = outputSignal;
	*sumCounterPtr = TOUCH_SENSE_APP_PWRUP_INIT_DONE;
}

/*****************************************************************************
 *
 * Function: int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift)
 *
 * Description: DC tracker (baseline) filter
 *
 *****************************************************************************/
int32_t DCTracker(int32_t inputSignal, int32_t *outputSignalRawPtr, uint8_t shift)
{	
	int32_t  outputSignalRaw;


	outputSignalRaw = *outputSignalRawPtr;

	if(inputSignal > (outputSignalRaw >> shift))
	{
		outputSignalRaw = outputSignalRaw + 1;
	}
	else if (inputSignal < (outputSignalRaw >> shift))
	{
		outputSignalRaw = outputSignalRaw - 1;
	}

	*outputSignalRawPtr = outputSignalRaw;

	return  (outputSignalRaw >> shift);
}
/*****************************************************************************
 *
 * Function: void DCTrackerShiftIncrease(void)
 *
 * Description: DCtrackershift increase
 *
 *****************************************************************************/
void DCTrackerShiftIncrease(void)
{
	// All touch button electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Load new shift to update the DC tracker
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE;
	}

	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Load DC tracker buffer
		DCTrackerDataBufferRaw[elecNum] = (DCTrackerDataBuffer[elecNum]) << ((DCTrackerDataShift[elecNum]));
	}
}

/*****************************************************************************
 *
 * Function: void DCTrackerShiftDecrease(void)
 *
 * Description: DCtrackershift decrease
 *
 *****************************************************************************/
void DCTrackerShiftDecrease(void)
{
	// All touch button electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Load new shift to update the DC tracker
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
	}

	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Load DC tracker buffer
		DCTrackerDataBufferRaw[elecNum] = (DCTrackerDataBuffer[elecNum]) << ((DCTrackerDataShift[elecNum]));
	}
}
/*****************************************************************************
 *
 * Function: void ElectrodeSelfTrim(void)
 *
 * Description: Run electrode self-trim algorithm
 *
 *****************************************************************************/
void ElectrodeSelfTrim(void)
{
	// Device after power-up / reset ?
	if (adcDataElectrodeDischargeBufferCounter[0] < NUMBER_OF_CYCLES_DCTRACKER_PWRUP)
	{
		// All touch button (and EGS) electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// Store touch electrode init value
			ElectodeBufferInitVal(adcDataElectrodeDischargeRaw[elecNum][frequencyID], &(adcDataElectrodeDischargeBuffer[elecNum]), &(adcDataElectrodeDischargeBufferCounter[elecNum]));
		}
	}
	// Calculate and load init value
	else
	{
		// All touch button (and EGS) electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// Calculate touch electrode init value
			ElectodeCalInitVal(&(adcDataElectrodeDischargeBuffer[elecNum]), &(adcDataElectrodeDischargeBufferCounter[elecNum]));
		}

		// Set electrode status "self-trim done" flag
		electrodesStatus.bit.selfTrimDone = YES;
	}
}

#if MORE_THAN_ONE_SENSOR_TOUCHED
/*****************************************************************************
 *
 * Function: void electrodeTouchQualify(void)
 *
 * Description: Qualify touch button electrode touch
 *
 *****************************************************************************/
void electrodeTouchQualify(void)
{
  electrodeTouchQualifiedReport = 0;
	// For all touch button electrodes check touch event
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if FREQUENCY_HOPPING
		// Electrode has been touched on both scanning frequencies?
		if ((electrodeTouch[elecNum][0] == 1) && (electrodeTouch[elecNum][1] == 1)  && (electrodeTouchNumberPlusOne == 0) &&
				numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE && frequencyID == 1)
#else
			// Electrode has been touched?
			if ((electrodeTouch[elecNum][frequencyID] == 1) &&
					numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE)
#endif
			{
				// Report electrode touch
				electrodeTouchQualifiedReport = 1;

				// Report electrode touch
				electrodeTouchQualified[elecNum] = 1;
			}
			else
			{
			  electrodeTouchQualified[elecNum] = 0;
			}

	}
}
#else

/*****************************************************************************
 *
 * Function: void electrodeTouchQualify(void)
 *
 * Description: Qualify touch button electrode touch
 *
 *****************************************************************************/
void electrodeTouchQualify(void)
{
  // For all touch button electrodes check touch event
  for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
  {
#if FREQUENCY_HOPPING
    // Electrode has been touched on both scanning frequencies?
    if ((electrodeTouch[elecNum][0] == 1) && (electrodeTouch[elecNum][1] == 1)  && (electrodeTouchNumberPlusOne == 0) &&
        numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE && frequencyID == 1)
#else
      // Electrode has been touched?
      if ((electrodeTouch[elecNum][frequencyID] == 1) && (electrodeTouchNumberPlusOne == 0) &&
          numberOfElectrodeSensingCyclesPerSample == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE)
#endif
      {
        // Report electrode touch
        electrodeTouchQualifiedReport = 1;

        // Report electrode touch
        electrodeTouchQualified[elecNum] = 1;

        // Load electrode number + 1
        electrodeTouchNumberPlusOne = elecNum + 1;
      }

    // Any electrode touched?
    if (electrodeTouchNumberPlusOne != 0)
    {
#if FREQUENCY_HOPPING
      // Electrode, that was touched, has been released on first or second scanning frequency?
      if((electrodeTouch[(electrodeTouchNumberPlusOne - 1)][0] == 0) || (electrodeTouch[(electrodeTouchNumberPlusOne - 1)][1] == 0))
#else
        // Electrode, that was touched, has been released?
        if(electrodeTouch[(electrodeTouchNumberPlusOne - 1)][frequencyID] == 0)
#endif
        {
          // Report electrode release
          electrodeTouchQualifiedReport = 0;
          // Report electrode release
          electrodeTouchQualified[(electrodeTouchNumberPlusOne - 1)] = 0;
          // Reset
          electrodeTouchNumberPlusOne = 0;
        }
    }
  }
}

#endif
/*****************************************************************************
 *
 * Function: void ElectrodeTouchDetect(uint32_t electrodeNum)
 *
 * Description: Detect touch button electrode touch
 *
 *****************************************************************************/
void ElectrodeTouchDetect(uint32_t electrodeNum)
{
	// Touch button Electrode touched ?
	if (LPFilterData[electrodeNum][frequencyID] < detectorThresholdTouch[electrodeNum])
	{
		// Touched
		electrodeTouch[electrodeNum][frequencyID] = 1;

#ifdef WAKE_UP_ELECTRODE
		// Load counter to do not return to the wake-up function
		// Do not return to the wake-up function, when electrode released and electrode signal value is above touch and bellow release threshold
		// Expected SR <= 1s
		electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;
#else
		// Load counter to prevent switching of number of samples per cycle to idle
		electrodesVirtualEGSActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;
#endif
	}
	// Electrode released?
	else if ((LPFilterData[electrodeNum][frequencyID] > detectorThresholdRelease[electrodeNum]) && (electrodeTouch[electrodeNum][frequencyID] == 1))
	{
		// Released
		electrodeTouch[electrodeNum][frequencyID] = 0;
	}
}

/*****************************************************************************
 *
 * Function: void ElectrodeSelfTrimSense(void)
 *
 * Description: Electrodes self-trim after power-up or reset
 *
 *****************************************************************************/
void ElectrodeSelfTrimSense(void)
{
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeFloat(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}
#endif

	// All touch button (and EGS) electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
		// Convert electrode capacitance to equivalent voltage by DMA
		ElectrodeCapToVoltConvELCH_DMA(elecNum);
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
		if(elecNum == WAKE_UP_ELECTRODE)
		{
		ElectrodeCapToVoltConvELCH_DMA_Proximity(elecNum);
		}
		#endif
#else
		// Convert electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH(elecNum);
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
		if(elecNum == WAKE_UP_ELECTRODE)
		{
		ElectrodeCapToVoltConvELCH_Proximity(elecNum);
		}
		#endif
#endif
	}

	// Drive all touch button (and EGS) electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Drive all slider electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}
#endif

	// All touch button (and EGS) electrodes self trim
	ElectrodeSelfTrim();

	// Electrodes self-trim done?
	if(electrodesStatus.bit.selfTrimDone == YES)
	{
		// All touch button (and EGS) electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
		{
			// DC tracker data
			DCTrackerDataBuffer[elecNum] = adcDataElectrodeDischargeBuffer[elecNum];
			// Load DC tracker buffer
			DCTrackerDataBufferRaw[elecNum] = (DCTrackerDataBuffer[elecNum]) << ((DCTrackerDataShift[elecNum]));
			// Electrode touch and release thresholds
			detectorThresholdTouch[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdTouchDelta[elecNum];
			detectorThresholdRelease[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdReleaseDelta[elecNum];
		}

		// All (both) used scanning frequencies (freemaster init)
		for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
		{
			// All touch button (and EGS) electrodes
			for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
			{
				// Touch electrodes LP IIR filter buffer init (FM only)
				LPFilterData[elecNum][frequencyID] = DCTrackerDataBuffer[elecNum];
				// Raw data preset as baseline
				adcDataElectrodeDischargeRaw[elecNum][frequencyID] = DCTrackerDataBuffer[elecNum];

#if DECIMATION_FILTER
				// Pre-load DF array
				adcDataElectrodeDischargeRawDF[elecNum][frequencyID] = DCTrackerDataBuffer[elecNum];
#endif
			}
		}
		// Reset frequencyID
		frequencyID = 0;

		// All (both) used scanning frequencies
		for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
		{
			// All touch button (and EGS) electrodes
			for (elecNum = 0; elecNum < (NUMBER_OF_ELECTRODES); elecNum++)
			{
				// Init IIR filter
				FilterIIR1BufferInit(elecNum, ((tFrac32)(DCTrackerDataBuffer[elecNum])), ((tFrac32)(DCTrackerDataBuffer[elecNum])), frequencyID);
			}
		}
		// Reset frequencyID
		frequencyID = 0;

#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
		// Enable LPTMR
		LPTMR0_Init(LPTMR_ELEC_SENSE_PROXIMITY);
#else
	#if (DECIMATION_FILTER && OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_NO)
			// Enable LPTMR
			LPTMR0_Init(LPTMR_ELEC_SENSE_DF);
	#else
			// Enable LPTMR
			LPTMR0_Init(LPTMR_ELEC_SENSE);
	#endif
#endif

		// Enable low power mode
		lowPowerModeCtrl = ON;
	}

#ifdef DEBUG_ALGORITHM
	// Pin set
	R_RMW32(&(DA_GPIO->PSOR), DA_PIN, 1);
#endif
}
#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
/*****************************************************************************
 *
 * Function: void ElecOversamplingActivation(void)
 *
 * Description: Activate touch button electrodes DC tracker adjustments for oversampling - to active mode
 *
 *****************************************************************************/
void ElecOversamplingActivation(void)
{
	// All touch button electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE;

		// Set DCTracker value to correspond to number of samples taken in Active state
		DCTrackerDataBuffer[elecNum] = DCTrackerDataBuffer[elecNum] << (OVERSAMPLING_VALUE_SHIFT_ACTIVE - OVERSAMPLING_VALUE_SHIFT_IDLE);

		// Load DC tracker buffer
		DCTrackerDataBufferRaw[elecNum] = (DCTrackerDataBuffer[elecNum]) << ((DCTrackerDataShift[elecNum]));

		// Touch electrodes touch and release thresholds recalculation
		detectorThresholdTouch[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdTouchDelta[elecNum];
		detectorThresholdRelease[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdReleaseDelta[elecNum];
	}

#if DECIMATION_FILTER
	// Save frequencyID
	frequencyIDsave = frequencyID;

	// All (both) used scanning frequencies
	for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
	{
		// All touch button electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
		{
			// Load DF array with DC tracker value - used when oversampling with EGS
			adcDataElectrodeDischargeRawDF[elecNum][frequencyID] = DCTrackerDataBuffer[elecNum];
		}
	}
	// Set back original frequencyID
	frequencyID = frequencyIDsave;
#endif
}

/*****************************************************************************
 *
 * Function: void ElecOversamplingDeactivation(void)
 *
 * Description: Deactivate touch button electrodes DC tracker adjustments for oversampling - back to idle mode
 *
 *****************************************************************************/
void ElecOversamplingDeactivation(void)
{
	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		DCTrackerDataShift[elecNum] = ELEC_DCTRACKER_FILTER_FACTOR_IDLE;

		// Set DCTracker value to correspond to number of samples taken in Idle state
		DCTrackerDataBuffer[elecNum] = DCTrackerDataBuffer[elecNum] >> (OVERSAMPLING_VALUE_SHIFT_ACTIVE - OVERSAMPLING_VALUE_SHIFT_IDLE);

		// Load DC tracker buffer
		DCTrackerDataBufferRaw[elecNum] = (DCTrackerDataBuffer[elecNum]) << ((DCTrackerDataShift[elecNum]));

		// Touch electrodes touch and release thresholds recalculation
		detectorThresholdTouch[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdTouchDelta[elecNum];
		detectorThresholdRelease[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdReleaseDelta[elecNum];
	}
}
#endif
// Wake up electrode define
#ifdef WAKE_UP_ELECTRODE
/*****************************************************************************
 *
 * Function: void ElectrodeWakeElecSense(void)
 *
 * Description: Sense wake-up (EGS) electrode to detect possible touch event
 *
 *****************************************************************************/
void ElectrodeWakeElecSense(void)
{
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeFloat(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}
#endif

#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
		// Convert wake-up (EGS) electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH_DMA_Proximity(WAKE_UP_ELECTRODE);
		#else
		// Convert wake-up (EGS) electrode capacitance to equivalent voltage by DMA
		ElectrodeCapToVoltConvELCH_DMA(WAKE_UP_ELECTRODE);
		#endif
#else
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
		// Convert wake-up (EGS) electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH_Proximity(WAKE_UP_ELECTRODE);
		#else
		// Convert wake-up (EGS) electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH(WAKE_UP_ELECTRODE);
		#endif
#endif

	// Update Wake-up DC Tracker
	DCTrackerDataBuffer[WAKE_UP_ELECTRODE] = DCTracker(adcDataElectrodeDischargeRaw[WAKE_UP_ELECTRODE][frequencyID], &(DCTrackerDataBufferRaw[WAKE_UP_ELECTRODE]), DCTrackerDataShift[WAKE_UP_ELECTRODE]);
#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
	// Fill the proximity electrode raw data into filtered data
	LPFilterData[WAKE_UP_ELECTRODE][frequencyID] = adcDataElectrodeDischargeRaw[WAKE_UP_ELECTRODE][frequencyID];
#else
	// Filter Wake-up electrode signal using IIR LP filter
	LPFilterData[WAKE_UP_ELECTRODE][frequencyID] = FilterIIR1(WAKE_UP_ELECTRODE, (tFrac32)(adcDataElectrodeDischargeRaw[WAKE_UP_ELECTRODE][frequencyID]), frequencyID);
#endif
	// Wake-up electrode touch & release threshold
	detectorThresholdTouch[WAKE_UP_ELECTRODE] = DCTrackerDataBuffer[WAKE_UP_ELECTRODE] - detectorThresholdTouchDelta[WAKE_UP_ELECTRODE];
	detectorThresholdRelease[WAKE_UP_ELECTRODE] = DCTrackerDataBuffer[WAKE_UP_ELECTRODE] - detectorThresholdReleaseDelta[WAKE_UP_ELECTRODE];

	/*************************************************************************************************
	 Touch button electrodes & slider electrodes slow baseline update routine when wake-up EGS used
	 **************************************************************************************************/
#if SLIDER_ENABLE
	// Update one touch button electrode baseline per wake up period
	if (elecNumAct < (NUMBER_OF_ELECTRODES - 1))
	{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
		// Convert electrode capacitance to equivalent voltage by DMA
		ElectrodeCapToVoltConvELCH_DMA(elecNumAct);
#else
		// Convert electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH(elecNumAct);

#endif
		// DC tracker calculation
		DCTrackerDataBuffer[elecNumAct] = DCTracker(adcDataElectrodeDischargeRaw[elecNumAct][frequencyID], &(DCTrackerDataBufferRaw[elecNumAct]), DCTrackerDataShift[elecNumAct]);
		// Increment to next touch button electrode, that will be updated in next wake up cycle (30ms period)
		elecNumAct++;
	}

	// Update slider electrodes baselines if all touch buttons are updated (elecNumact == WakeUpElectrode)
	else if(elecNumAct == (NUMBER_OF_ELECTRODES - 1) && sliderElecNumAct < (NUMBER_OF_SLIDER_ELECTRODES))
	{
#if (NUMBER_OF_USED_ADC_MODULES == 2)
		// Preset configuration for both ADC0 and ADC1 for voltage digitalization at the same time
		ADCs_SimultaneousHWtrigger();
		// Update both slider electrodes baselines at once
		SliderelectrodeWakeElecSense();
		// Reset configuration for both ADC0 and ADC1 for voltage digitalization one by one
		ADCs_SetBackToSWtrigger();

		// Increment by two slider electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			sliderElecNumAct++;
		}

#else
		// Update both slider electrodes baselines
		SliderelectrodeWakeElecSense();

		// Increment by two slider electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			sliderElecNumAct++;
		}
#endif
	}

	else
	{
		// Start from begin
		elecNumAct = 0;
		sliderElecNumAct = 0;
	}

#else

	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	// Convert electrode capacitance to equivalent voltage by DMA
	ElectrodeCapToVoltConvELCH_DMA(elecNumAct);
	#else
	// Convert electrode capacitance to equivalent voltage by CPU
	ElectrodeCapToVoltConvELCH(elecNumAct);
	#endif

	// DC tracker calculation
	DCTrackerDataBuffer[elecNumAct] = DCTracker(adcDataElectrodeDischargeRaw[elecNumAct][frequencyID], &(DCTrackerDataBufferRaw[elecNumAct]), DCTrackerDataShift[elecNumAct]);

	// Next touch electrode
	if (elecNumAct < (NUMBER_OF_ELECTRODES - 2))
	{
		elecNumAct++;
	}
	else
	{
		// Start from begin
		elecNumAct = 0;
	}
#endif
	/****************************************************************************************************
	END OF Touch button electrodes & slider electrodes slow baseline update routine when wake-up EGS used
	 ****************************************************************************************************/

	// Drive all touch button (and EGS) electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Drive all slider electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}
#endif

	// Proximity - wake up EGS electrode touched? (frequencyID at this stage of code should always be 0)
	if(LPFilterData[WAKE_UP_ELECTRODE][frequencyID] < detectorThresholdTouch[WAKE_UP_ELECTRODE])
	{
		// Touched
		electrodeTouch[WAKE_UP_ELECTRODE][frequencyID] = 1;

		// Change the number of ADC sensing cycles per sample for touch buttons
		ElectrodeSensingCyclesChangeEGS();

#if SLIDER_ENABLE
		// Change the number of ADC sensing cycles per sample for slider
		SliderSensingCyclesChangeEGS();
#endif

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
		// Activate touch button electrodes DC tracker adjustments for oversampling - to active mode
		ElecOversamplingActivation();

#if SLIDER_ENABLE
		// Activate slider electrodes DC tracker adjustments for oversampling - to active mode
		SliderElecOversamplingActivation();
#endif
#else
#if DECIMATION_FILTER || FREQUENCY_HOPPING
		// Increase DC tracker Shift to slow down the DC tracker filter
		DCTrackerShiftIncrease();
#if SLIDER_ENABLE
		// Increase DC tracker Shift to slow down the DC tracker filter
		SliderDCTrackerShiftIncrease();
#endif
#endif
#endif

		/* IIR LP Filter Buffer initiation after EGS touch*/

		// Save frequencyID
		frequencyIDsave = frequencyID;

		// All (both) used scanning frequencies
		for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
		{
			// All touch button electrodes
			for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
			{
				// Set IIR filter initial value to (DCTracker - threshold / 2)
				FilterIIR1BufferInit(elecNum, ((tFrac32)(DCTrackerDataBuffer[elecNum] - (detectorThresholdTouchDelta[elecNum] / 2))), \
						((tFrac32)(DCTrackerDataBuffer[elecNum] - (detectorThresholdTouchDelta[elecNum] / 2))), frequencyID);
			}
		}

#if SLIDER_ENABLE
		// All (both) used scanning frequencies
		for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
		{
			// Set IIR filter initial value to (DCTracker - threshold / 2) for slider electrodes
			SliderElectrodeWakeElecFilterLoad();
		}
#endif
		// Set back original frequencyID
		frequencyID = frequencyIDsave;

		// Load counter to do not return to the wake-up function
		// When wake-up happens, 1 second period given to detect touch event
		electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;

		// Sense all touch button and slider electrodes
		ElectrodeWakeAndTouchElecSense();
	}
#ifdef DEBUG_ALGORITHM
	// Pin set
	R_RMW32(&(DA_GPIO->PSOR), DA_PIN, 1);
#endif
}

/*****************************************************************************
 *
 * Function: void ElectrodeWakeAndTouchElecSense(void)
 *
 * Description: Sense all keypad electrodes (all touch button and slider electrodes)
 *
 *****************************************************************************/
void ElectrodeWakeAndTouchElecSense(void)
{
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeFloat(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}
#endif

	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
		// Convert electrode capacitance to equivalent voltage by DMA
		ElectrodeCapToVoltConvELCH_DMA(elecNum);
#else
		// Convert electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH(elecNum);
#endif

		// Update DC Tracker, if electrode not touched
		if (electrodeTouch[elecNum][frequencyID] == 0)
		{
			// Update DC tracker
			DCTrackerDataBuffer[elecNum] = DCTracker(adcDataElectrodeDischargeRaw[elecNum][frequencyID], &(DCTrackerDataBufferRaw[elecNum]), DCTrackerDataShift[elecNum]);
		}
#if DECIMATION_FILTER
		// Decimation filtering
		DecimationFilter(elecNum);
#endif

#if DECIMATION_FILTER
		// IIR LP Filter fed by DF data
		LPFilterData[elecNum][frequencyID] = FilterIIR1(elecNum, (tFrac32)(adcDataElectrodeDischargeRawDF[elecNum][frequencyID]), frequencyID);
#else
		// IIR LP filter fed by raw data
		LPFilterData[elecNum][frequencyID] = FilterIIR1(elecNum, (tFrac32)(adcDataElectrodeDischargeRaw[elecNum][frequencyID]), frequencyID);
#endif

		// Electrode touch and release thresholds
		detectorThresholdTouch[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdTouchDelta[elecNum];
		detectorThresholdRelease[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdReleaseDelta[elecNum];

		// Electrode touched ?
		ElectrodeTouchDetect(elecNum);
	}

	// Drive all touch button (and EGS) electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Drive all slider electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}
#endif

	// Qualify the touch event
	// More the one electrode reports touch?
	electrodeTouchQualify();

#if SLIDER_ENABLE
#if (NUMBER_OF_USED_ADC_MODULES == 2)
	// Preset configuration for both ADC0 and ADC1 for voltage digitalization at the same time
	ADCs_SimultaneousHWtrigger();
	// Sense electrode touch event at slider electrodes
	SliderElectrodesTouchElecSense();
	// Reset configuration for both ADC0 and ADC1 for voltage digitalization one by one
	ADCs_SetBackToSWtrigger();
#else
	// Sense electrode touch event at slider electrodes
	SliderElectrodesTouchElecSense();
#endif
#endif

	// Release to wake-up electrode control
	if (electrodeWakeUpActivateCounter == 0)
	{
		// Released
		electrodeTouch[WAKE_UP_ELECTRODE][0] = 0;

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
		// Deactivate touch button electrodes DC tracker adjustments for oversampling - back to idle mode
		ElecOversamplingDeactivation();

#if SLIDER_ENABLE
		// Deactivate slider electrodes DC tracker adjustments for oversampling - back to idle mode
		SliderElecOversamplingDeactivation();
#endif
#else
#if DECIMATION_FILTER || FREQUENCY_HOPPING
		// Decrease DC tracker Shift to speed up the DC tracker filter
		DCTrackerShiftDecrease();
#if SLIDER_ENABLE
		// Decrease DC tracker Shift to speed up the DC tracker filter
		SliderDCTrackerShiftDecrease();
#endif
#endif
#endif

		// Change the number of ADC sensing cycles per sample (for touch buttons)
		ElectrodeSensingCyclesChangeEGS();

#if SLIDER_ENABLE
		// Change the number of ADC sensing cycles per sample (for slider)
		SliderSensingCyclesChangeEGS();
#endif

		/* IIR LP Filter Buffer of EGS reset after EGS release*/

		// Save frequencyID
		frequencyIDsave = frequencyID;

		// All (both) used scanning frequencies
		for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
		{
			// Load wake-up electrode IIR1 LP Filter buffer (X, Y) with DC tracker value
			FilterIIR1BufferInit(WAKE_UP_ELECTRODE, ((tFrac32)(DCTrackerDataBuffer[WAKE_UP_ELECTRODE])), (((tFrac32)(DCTrackerDataBuffer[WAKE_UP_ELECTRODE]))),frequencyID);
		}
		// Set back original frequencyID
		frequencyID = frequencyIDsave;
	}
	else
	{
#if FREQUENCY_HOPPING
		// Decrement wake up counter only during core period/frequency (30 ms)
		if(frequencyID == 0)
#endif
		{
			// Decrement counter
			electrodeWakeUpActivateCounter--;
		}
	}

#ifdef DEBUG_ALGORITHM
	// Pin set
	R_RMW32(&(DA_GPIO->PSOR), DA_PIN, 1);
#endif
}
/*****************************************************************************
 *
 * Function: void ElectrodeSensingCyclesChangeEGS(void)
 *
 * Description: Changes the number of sensing cycles per sample based on whether the proximity EGS was touched
 *
 *****************************************************************************/
void ElectrodeSensingCyclesChangeEGS(void)
{
	if(electrodeTouch[WAKE_UP_ELECTRODE][0] == 1)
	{
		// Change the number of touch buttons sensing cycles per sample to active
		numberOfElectrodeSensingCyclesPerSample = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE;
	}
	else
	{
		// Change the number of touch buttons sensing cycles per sample to idle
		numberOfElectrodeSensingCyclesPerSample = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE;
	}
}

// Wake up electrode define
#else

/*****************************************************************************
 *
 * Function: void ElectrodeTouchElecSense(void)
 *
 * Description: Sense all touch button electrodes
 *
 *****************************************************************************/
void ElectrodeTouchElecSense(void)
{
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeFloat(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}
#endif

	// All touch button electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
	{
#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
		// Convert electrode capacitance to equivalent voltage by DMA
		ElectrodeCapToVoltConvELCH_DMA(elecNum);
#else
		// Convert electrode capacitance to equivalent voltage by CPU
		ElectrodeCapToVoltConvELCH(elecNum);
#endif
		// Update DC Tracker, if electrode not touched
		if (electrodeTouch[elecNum][frequencyID] == 0)
		{
			// Update DC Tracker
			DCTrackerDataBuffer[elecNum] = DCTracker(adcDataElectrodeDischargeRaw[elecNum][frequencyID], &(DCTrackerDataBufferRaw[elecNum]), DCTrackerDataShift[elecNum]);
		}

#if DECIMATION_FILTER
		// Decimation filtering
		DecimationFilter(elecNum);
#endif

#if DECIMATION_FILTER
		// IIR LP Filter fed by DF data
		LPFilterData[elecNum][frequencyID] = FilterIIR1(elecNum, (tFrac32)(adcDataElectrodeDischargeRawDF[elecNum][frequencyID]), frequencyID);
#else
		// IIR LP filter fed by raw data
		LPFilterData[elecNum][frequencyID] = FilterIIR1(elecNum, (tFrac32)(adcDataElectrodeDischargeRaw[elecNum][frequencyID]), frequencyID);
#endif

		// Calculate electrode touch and release thresholds
		detectorThresholdTouch[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdTouchDelta[elecNum];
		detectorThresholdRelease[elecNum] = DCTrackerDataBuffer[elecNum] - detectorThresholdReleaseDelta[elecNum];

		// Electrode touched ?
		ElectrodeTouchDetect(elecNum);

	}

	// Drive all touch button (and EGS) electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&elecStruct[elecNum]);
	}

#if SLIDER_ENABLE
	// Drive all slider electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}
#endif

	// Qualify the touch event
	// More the one electrode reports touch?
	electrodeTouchQualify();

#ifdef DEBUG_ALGORITHM
	// Pin set
	R_RMW32(&(DA_GPIO->PSOR), DA_PIN, 1);
#endif

}

/*****************************************************************************
 *
 * Function: void ElectrodeSensingCyclesChange(void)
 *
 * Description: Changes the number of sensing cycles per sample for touch buttons based on detected proximity on touch buttons
 *
 *****************************************************************************/
void ElectrodeSensingCyclesChange(void)
{
	uint8_t electrodeNum;

	// Detect if any of the button electrodes was reported as proximately touched
	for(electrodeNum = 0; electrodeNum < NUMBER_OF_TOUCH_ELECTRODES; electrodeNum++)
	{
#if FREQUENCY_HOPPING
		// If on both frequencies proximity detected
		if((LPFilterData[electrodeNum][0] < (DCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA)) &&  \
				(LPFilterData[electrodeNum][1] < (DCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA)))
#else
			// If proximity detected
			if(LPFilterData[electrodeNum][frequencyID] < (DCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA))
#endif
			{
				// Report wake up
				electrodesVirtualEGSActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;

				// If this is first touch
				if (electrodesVirtualEGSTouch != 1)
				{
					// Report that some of the touch button electrodes proximately touched
					electrodesVirtualEGSTouch = 1;

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
					// Activate the oversampling just once after first confirmed proximity
					if(electrodesOversamplingActivationReport != 1)
					{
						// Activate touch button electrodes DC tracker adjustments for oversampling - to active mode
						ElecOversamplingActivation();

						// Report electrodes Oversampling activation done
						electrodesOversamplingActivationReport = 1;

						// Save frequencyID
						frequencyIDsave = frequencyID;

						// All (both) used scanning frequencies
						for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
						{

							// All touch button electrodes
							for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
							{
								// Load IIR1 LP Filter buffer (X, Y) exactly with corresponding value
								FilterIIR1BufferInit(elecNum, ((tFrac32)(DCTrackerDataBuffer[elecNum] - (VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA << (OVERSAMPLING_VALUE_SHIFT_ACTIVE - OVERSAMPLING_VALUE_SHIFT_IDLE)))), \
										((tFrac32)(DCTrackerDataBuffer[elecNum] - (VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA << (OVERSAMPLING_VALUE_SHIFT_ACTIVE - OVERSAMPLING_VALUE_SHIFT_IDLE)))),frequencyID);

							}
						}
						// Set back original frequencyID
						frequencyID = frequencyIDsave;

#if DECIMATION_FILTER
						// Save frequencyID
						frequencyIDsave = frequencyID;

						// All (both) used scanning frequencies
						for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
						{

							// All touch button electrodes
							for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
							{
								// Load DF array exactly with corresponding value
								adcDataElectrodeDischargeRawDF[elecNum][frequencyID]= DCTrackerDataBuffer[elecNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA;
							}
						}
						// Set back original frequencyID
						frequencyID = frequencyIDsave;
#endif
					}
#endif
					// Change the number of touch buttons sensing cycles per sample to active
					numberOfElectrodeSensingCyclesPerSample = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE;

				}
				// Exit the for loop
				break;
			}

			else
			{
				// Came to the last touch button electrode (did not manage to escape the for loop so far)
				if(electrodeNum == (NUMBER_OF_TOUCH_ELECTRODES - 1))
				{
					// Wake up counter finished counting?
					if(electrodesVirtualEGSActivateCounter == 0)
					{
						// Report that all touch button electrodes proximity released
						electrodesVirtualEGSTouch = 0;

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
						// De-activate the oversampling just once after first confirmed proximity release
						if(electrodeNum == (NUMBER_OF_TOUCH_ELECTRODES-1) && electrodesOversamplingActivationReport == 1)
						{
							// Deactivate touch button electrodes DC tracker adjustments for oversampling - back to idle mode
							ElecOversamplingDeactivation();

							// Save frequencyID
							frequencyIDsave = frequencyID;

							// All (both) used scanning frequencies
							for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
							{

								// All touch button electrodes
								for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
								{
									// Load IIR1 LP Filter buffer (X, Y) with DC tracker value
									FilterIIR1BufferInit(elecNum, ((tFrac32)(DCTrackerDataBuffer[elecNum])), (((tFrac32)(DCTrackerDataBuffer[elecNum]))),frequencyID);
								}
							}
							// Set back original frequencyID
							frequencyID = frequencyIDsave;

							// Report electrodes Oversampling De-activation done
							electrodesOversamplingActivationReport = 0;

#if DECIMATION_FILTER
							// Save frequencyID
							frequencyIDsave = frequencyID;

							// All (both) used scanning frequencies
							for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
							{

								// All touch button electrodes
								for (elecNum = 0; elecNum < NUMBER_OF_TOUCH_ELECTRODES; elecNum++)
								{
									// Load DF array with new DC tracker value
									adcDataElectrodeDischargeRawDF[elecNum][frequencyID]= DCTrackerDataBuffer[elecNum];
								}
							}
							// Set back original frequencyID
							frequencyID = frequencyIDsave;
#endif
						}
#endif

						// Change the number of touch buttons sensing cycles per sample to idle
						numberOfElectrodeSensingCyclesPerSample = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE;
					}
					else
					{
						// Decrement wake up activate counter
						if(electrodesVirtualEGSActivateCounter != 0)
						{
							electrodesVirtualEGSActivateCounter--;
						}
					}
				}
			}
	}
}

// Wake up electrode define
#endif

#if FREQUENCY_HOPPING
/*****************************************************************************
 *
 * Function: void FrequencyHop(void)
 *
 * Description: Changes scanning frequency/period of electrodes samples
 *
 *****************************************************************************/
void FrequencyHop(void)
{
#ifdef WAKE_UP_ELECTRODE
	// If EGS touched
	if(electrodeTouch[WAKE_UP_ELECTRODE][0] == 1)
#else
		// Frequency hopping all the time
		if(1)
#endif
		{
			// Report FH activation for touch qualification
			frequencyHoppingActivation = 1;

			// Period/Frequency switching
			switch (frequencyID)
			{
			case 0:
			{
				// Report switch of scanning frequency/period
				frequencyID = 1;
				break;
			}

			case 1:
			{
				// Report switch of scanning frequency/period back to normal(core period 30 ms)
				frequencyID = 0;
				break;
			}
			}

		}
		else
		{
			// Report FH DEactivation
			frequencyHoppingActivation = 0;

			// Report switch of scanning frequency/period back to normal(core period 30 ms)
			frequencyID = 0;
		}
}

#endif

#if DECIMATION_FILTER
/*****************************************************************************
 *
 * Function: void DecimationFilter(uint32_t electrodeNum)
 *
 * Description: Increment or decrement decimation filter value
 *
 *****************************************************************************/
void DecimationFilter(uint32_t electrodeNum)
{

	// Is the electrode raw data value still smaller than electrode decimation filter value?
	if(adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] < adcDataElectrodeDischargeRawDF[electrodeNum][frequencyID])
	{
		// Decrement DF value - follow the raw data down
		adcDataElectrodeDischargeRawDF[electrodeNum][frequencyID] -= DECIMATION_STEP ;
	}
	// Is the electrode raw data value same as electrode decimation filter value?
	else if ((adcDataElectrodeDischargeRaw[electrodeNum][frequencyID] == adcDataElectrodeDischargeRawDF[electrodeNum][frequencyID]))
	{
		// Do nothing
	}

	else
	{
		// Increment DF value - follow  the raw data back up to the baseline
		adcDataElectrodeDischargeRawDF[electrodeNum][frequencyID] += DECIMATION_STEP;
	}

}

#endif
