/****************************************************************************//*!
*
* @file  	slider.c
*
* @version  1.0.0.0
*
* @date     December-2017
*
* @brief    Slider electrode touch sense routines for S32K144
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
#if SLIDER_ENABLE
/*******************************************************************************
 * Variables
 ******************************************************************************/
// Slider Electrodes status
tElecStatus   sliderElectrodesStatus;
// Slider Electrode structure
tElecStruct  sliderElecStruct[NUMBER_OF_SLIDER_ELECTRODES];
extern tElecStruct  elecStruct[NUMBER_OF_ELECTRODES];
// Guard pin structure
extern tGuardStruct guardStruct[1];

// Slider electrode capacitance to equivalent voltage conversion
int16_t   numberOfElectrodeSensingCyclesPerSampleSlider, sliderDroppedSamples;
int32_t   sliderAdcDataElectrodeDischargeRaw[NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
int16_t   sliderAdcDataElectrodeDischargeRawSample[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
int16_t   sliderAdcDataElectrodeDischargeRawSampleElec0[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
int16_t   sliderAdcDataElectrodeDischargeRawSampleElec1[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
uint32_t   sliderAdcDataElectrodeDischargeRawCalc;

// Extern variables for slider electrode capacitance to equivalent voltage conversion
extern uint8_t elecNum;
extern uint16_t sampleNum;
extern uint16_t  chargeDistributionPeriod, chargeDistributionPeriodTmp;

// Wake up
extern uint16_t  electrodeWakeUpActivateCounter;
uint16_t  sliderVirtualEGSActivateCounter;
uint8_t   sliderElecNumAct;

// Slider Electrode DC tracker self-trim after power-up or reset
int32_t   sliderAdcDataElectrodeDischargeBuffer[NUMBER_OF_SLIDER_ELECTRODES];
uint16_t  sliderAdcDataElectrodeDischargeBufferCounter[NUMBER_OF_SLIDER_ELECTRODES];

// DC Tracker
int32_t   sliderDCTrackerDataBufferRaw[NUMBER_OF_SLIDER_ELECTRODES];
int32_t   sliderDCTrackerDataBuffer[NUMBER_OF_SLIDER_ELECTRODES];
uint8_t   sliderDCTrackerDataShift[NUMBER_OF_SLIDER_ELECTRODES];

// LP Filter
tFrac32   sliderLPFilterData[NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern uint8_t   LPFilterType[NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES];
extern tFrac32   LPFilterData[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];// For sampling change EGS

// Detector
int32_t   sliderDetectorThresholdTouch[NUMBER_OF_SLIDER_ELECTRODES];
int16_t   sliderDetectorThresholdTouchDelta[NUMBER_OF_SLIDER_ELECTRODES];
uint8_t   sliderElectrodeTouch[NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern 	uint8_t   electrodeTouch[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern int32_t   detectorThresholdTouch[NUMBER_OF_ELECTRODES];// For sampling change EGS
uint8_t   sliderVirtualEGSTouch;

// Slider Touch qualification
uint8_t   sliderElectrodeTouchQualified;
uint8_t	  sliderElectrodeTouchQualifiedReport;
tBool hitHysteresis;
tBool positionDeciced;
tBool firstSliderTouch;
int8_t  positionCounter;
uint8_t thresholdNum;

// Slider data
int16_t sliderDifferenceData, sliderDifferenceDataFiltered;
int16_t sliderAdditionData;
int16_t sliderAdditionDataThreshold;
int16_t sliderAbsoluteRawData, sliderAbsoluteRawDataFiltered;

// Slider helper differential data thresholds for Freemaster
int16_t sliderDiffThreshold[NUMBER_OF_THRESHOLDS_FREEMASTER + NUMBER_OF_SLIDER_THRESHOLDS] = {0};// PreInit to zero because of Freemaster
int16_t *sliderLastDiffThreshold;

// Slider defines variables
int8_t numberOfHysteresisBlindPoints;

// Low power mode
extern uint8_t  lowPowerModeCtrl;

// FrequencyHopping
extern uint8_t   frequencyID,frequencyIDsave;
extern uint8_t   frequencyHoppingActivation;

// Decimation filter
int32_t   sliderAdcDataElectrodeDischargeRawDF[NUMBER_OF_SLIDER_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];

// Oversampling
uint8_t sliderOversamplingActivationReport;

// DMA control flags
extern volatile uint32_t ready_dma_flag, start_dma_flag;

/*****************************************************************************
 *
 * Function: void SliderElectrodeStructureInit(void)
 *
 * Description: Slider electrodes structure init
 *
 *****************************************************************************/
void SliderElectrodeStructureInit(void)
{
#ifdef SLIDER_ELEC0
	// Load slider electrode 0 hardware data
	sliderElecStruct[0].adcBasePtr = SLIDER_ELEC0_ADC;
	sliderElecStruct[0].adcChNum = SLIDER_ELEC0_ADC_CHANNEL;
	sliderElecStruct[0].portBasePtr = SLIDER_ELEC0_PORT;
	sliderElecStruct[0].gpioBasePtr = SLIDER_ELEC0_GPIO;
	sliderElecStruct[0].pinNumberElec = SLIDER_ELEC0_ELEC_GPIO_PIN;
	sliderElecStruct[0].pinNumberCext = SLIDER_ELEC0_CEXT_GPIO_PIN;
	sliderElecStruct[0].portMask = SLIDER_ELEC0_PORT_MASK;
	sliderElecStruct[0].ELEC_DMAMUX = SLIDER_ELEC0_DMAMUX;
	sliderElecStruct[0].ELEC_TRGMUX = SLIDER_ELEC0_TRGMUX;
	// Load electrode 0 application data
	sliderDCTrackerDataShift[0] = SLIDER_ELEC0_DCTRACKER_FILTER_FACTOR;
	LPFilterType[NUMBER_OF_ELECTRODES + 0] = SLIDER_ELEC0_LPFILTER_TYPE;
	sliderDetectorThresholdTouchDelta[0] = SLIDER_ELEC0_TOUCH_THRESHOLD_DELTA;
#endif

#ifdef SLIDER_ELEC1
	// Load slider electrode 1 hardware data
	sliderElecStruct[1].adcBasePtr = SLIDER_ELEC1_ADC;
	sliderElecStruct[1].adcChNum = SLIDER_ELEC1_ADC_CHANNEL;
	sliderElecStruct[1].portBasePtr = SLIDER_ELEC1_PORT;
	sliderElecStruct[1].gpioBasePtr = SLIDER_ELEC1_GPIO;
	sliderElecStruct[1].pinNumberElec = SLIDER_ELEC1_ELEC_GPIO_PIN;
	sliderElecStruct[1].pinNumberCext = SLIDER_ELEC1_CEXT_GPIO_PIN;
	sliderElecStruct[1].portMask = SLIDER_ELEC1_PORT_MASK;
	sliderElecStruct[1].ELEC_DMAMUX = SLIDER_ELEC1_DMAMUX;
	sliderElecStruct[1].ELEC_TRGMUX = SLIDER_ELEC1_TRGMUX;
	// Load slider electrode 1 application data
	sliderDCTrackerDataShift[1] = SLIDER_ELEC1_DCTRACKER_FILTER_FACTOR;
	LPFilterType[NUMBER_OF_ELECTRODES + 1] = SLIDER_ELEC1_LPFILTER_TYPE;
	sliderDetectorThresholdTouchDelta[1] = SLIDER_ELEC1_TOUCH_THRESHOLD_DELTA;
#endif

	// If EGS is OFF and TS method is oversampling
	#if !defined(WAKE_UP_ELECTRODE) && TS_RAW_DATA_CALCULATION == OVERSAMPLING
	// All slider electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
	}
	#endif

	#ifdef WAKE_UP_ELECTRODE
	#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING) || (DECIMATION_FILTER) || (FREQUENCY_HOPPING)
	// All slider electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
	}
	#endif
	#endif
}

/*****************************************************************************
*
* Function: void SliderElectrodeADCchannelOffset(void)
*
* Description: Fixes ADC channel number in case it is higher than 15
*
*****************************************************************************/
void SliderElectrodeADCchannelOffset(void)
{
	// All slider electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// ADC channel number higher than 15?
		if(sliderElecStruct[elecNum].adcChNum > 15)
		{
			// Add offset
			sliderElecStruct[elecNum].adcChNum += ELECTRODE_ADC_CHANNEL_OFFSET;
		}
	}

}

/*****************************************************************************
 *
 * Function: void SliderElectrodeTouchSenseInit(void)
 *
 * Description: Init touch sense variables for slider electrodes
 *
 *****************************************************************************/
void SliderElectrodeTouchSenseInit(void)
{

	// Slider electrodes structure init
	SliderElectrodeStructureInit();

	// Cext pin ADC channel number correction (in case higher ADC channel number used)
	SliderElectrodeADCchannelOffset();

	// Reset slider electrodes status
	sliderElectrodesStatus.byte = 0;

	// Reset slider First touch indicator
	firstSliderTouch = TRUE;

	// Slider Default Variables
	// Get helper thresholds to see the thresholds in Freemaster
	SliderDifferenceDataHelperThresholds();

	// Get default number of hysteresis blind points
	numberOfHysteresisBlindPoints = NUMBER_OF_HYSTERESIS_BLIND_POINTS;

	// Set the default idle number of sensing cycles for slider electrodes
	numberOfElectrodeSensingCyclesPerSampleSlider = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE;

	// Reset the number of dropped samples
	sliderDroppedSamples = 0;

	// Reset - report that all slider electrodes proximity released
	sliderVirtualEGSTouch = 0;

	// Set the default slider addition data threshold of y-direction false touch canceling
	sliderAdditionDataThreshold = SLIDER_ADDITION_DATA_THRESHOLD_DEFAULT;

	// Set slider difference and addition data to theirs baseline
	sliderDifferenceData = SLIDER_DIFFADD_DATA_OFFSET;
	sliderAdditionData = SLIDER_DIFFADD_DATA_OFFSET;

}
/*****************************************************************************
 *
 * Function: void SliderDifferenceDataHelperThresholds (void)
 *
 * Description: Slider helper differential data thresholds for Freemaster
 *
 *****************************************************************************/
void SliderDifferenceDataHelperThresholds (void)
{
	// Automatic difference thresholds generation/loading, theoretically unlimited number of thresholds
	#if  AUTO_DIFF_THRESHOLDS_GEN

	uint8_t segmentNum;

		for(segmentNum = 0; segmentNum < NUMBER_OF_SLIDER_THRESHOLDS; segmentNum++)
		{
			sliderDiffThreshold[segmentNum] = (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP*(segmentNum + 1)));
		}

	#else // Manual difference thresholds loading, maximum 7 thresholds -> 8 segments
	#ifdef SLIDER_DIFF_THRESHOLD0
		sliderDiffThreshold[0] = SLIDER_DIFF_THRESHOLD0;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD1
		sliderDiffThreshold[1] = SLIDER_DIFF_THRESHOLD1;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD2
		sliderDiffThreshold[2] = SLIDER_DIFF_THRESHOLD2;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD3
		sliderDiffThreshold[3] = SLIDER_DIFF_THRESHOLD3;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD4
		sliderDiffThreshold[4] = SLIDER_DIFF_THRESHOLD4;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD5
		sliderDiffThreshold[5] = SLIDER_DIFF_THRESHOLD5;
	#endif

	#ifdef SLIDER_DIFF_THRESHOLD6
		sliderDiffThreshold[6] = SLIDER_DIFF_THRESHOLD6;
	#endif

	#endif
		// Varying last difference threshold pointer assignment
		sliderLastDiffThreshold = &sliderDiffThreshold[NUMBER_OF_SLIDER_THRESHOLDS-1];
}

/*****************************************************************************
 *
 * Function: void electrodeSelfTrim(void)
 *
 * Description: Run slider electrodes self-trim algorithm
 *
 *****************************************************************************/
void SliderElectrodeSelfTrim(void)
{
	// Device after power-up / reset ?
	if (sliderAdcDataElectrodeDischargeBufferCounter[0] < NUMBER_OF_CYCLES_DCTRACKER_PWRUP)
	{
		// All Slider electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
		{
			// Store Slider electrode init value
			ElectodeBufferInitVal(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderAdcDataElectrodeDischargeBuffer[elecNum]), &(sliderAdcDataElectrodeDischargeBufferCounter[elecNum]));
		}
	}
	// Calculate and load init value
	else
	{
		// All Slider electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
		{
			// Calculate Slider electrode init value
			ElectodeCalInitVal(&(sliderAdcDataElectrodeDischargeBuffer[elecNum]), &(sliderAdcDataElectrodeDischargeBufferCounter[elecNum]));
		}

		// Set slider electrode status "self-trim done" flag
		sliderElectrodesStatus.bit.selfTrimDone = YES;
	}
}
/*****************************************************************************
 *
 * Function: void SliderElectrodeSelfTrimSense(void)
 *
 * Description: Slider electrodes self-trim after power-up or reset
 *
 *****************************************************************************/
void SliderElectrodeSelfTrimSense(void)
{
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif

	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
	    ElectrodeFloat(&elecStruct[elecNum]);
	}

#if (NUMBER_OF_USED_ADC_MODULES == 2)
	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)

		// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once by DMA
		SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(SLIDER_ELEC0, SLIDER_ELEC1);

	#else
		// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once BY CPU
		SliderSimultaneousElectrodeCapToVoltConvELCH(SLIDER_ELEC0, SLIDER_ELEC1);
	#endif

#else
	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		// Convert Slider electrode capacitance to equivalent voltage one by one by DMA (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCH_DMA(elecNum);
	}
	#else
		// Convert Slider electrode capacitance to equivalent voltage one by one by CPU (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCHGuard(SLIDER_ELEC0, SLIDER_ELEC1);
	#endif

#endif
	// Drive all slider electrodes to GND
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}

	// Drive all touch button (and EGS) electrodes to GND
 	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
 	{
 		ElectrodeGnd(&elecStruct[elecNum]);
 	}

	// Slider electrodes self trim
	SliderElectrodeSelfTrim();

	// Slider Electrodes self-trim done?
	if(sliderElectrodesStatus.bit.selfTrimDone == YES)
	{
		// All slider electrodes
		for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
		{
			// DC tracker data
			sliderDCTrackerDataBuffer[elecNum] = sliderAdcDataElectrodeDischargeBuffer[elecNum];
			// Load DC tracker buffer
			sliderDCTrackerDataBufferRaw[elecNum] = (sliderDCTrackerDataBuffer[elecNum]) << ((sliderDCTrackerDataShift[elecNum]));
			// Slider Electrode touch and release thresholds
			sliderDetectorThresholdTouch[elecNum] = sliderDCTrackerDataBuffer[elecNum] - sliderDetectorThresholdTouchDelta[elecNum];
		}

		// All (both) used scanning frequencies
        for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
        {
        	// All slider electrodes
        	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
        	{
    			// Slider electrodes LP IIR filter buffer init (FM only)
    			sliderLPFilterData[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum];
        		// Slider Raw data preset as baseline
        		sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum];

			#if DECIMATION_FILTER
				// Pre-load DF array
				sliderAdcDataElectrodeDischargeRawDF[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum];
			#endif
        	}
        }
        // Reset frequencyID
        frequencyID = 0;

        // All (both) used scanning frequencies
        for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
        {
    		// All Slider electrodes
    		for (elecNum = NUMBER_OF_ELECTRODES; elecNum < (NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
    		{
    			// Init IIR filter
    			FilterIIR1BufferInit(elecNum, ((tFrac32)(sliderDCTrackerDataBuffer[(elecNum) - NUMBER_OF_ELECTRODES])), ((tFrac32)(sliderDCTrackerDataBuffer[(elecNum) - NUMBER_OF_ELECTRODES])), frequencyID);
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

/*****************************************************************************
 *
 * Function: void SliderDCTrackerShiftIncrease(void)
 *
 * Description: DCtrackershift increase
 *
 *****************************************************************************/
void SliderDCTrackerShiftIncrease(void)
{
		// All slider electrodes
		for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			// Load new shift to update the DC tracker
			sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE;
		}

		// All slider electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			// Load DC tracker buffer
			sliderDCTrackerDataBufferRaw[elecNum] = (sliderDCTrackerDataBuffer[elecNum]) << ((sliderDCTrackerDataShift[elecNum]));
		}
}

/*****************************************************************************
 *
 * Function: void SliderDCTrackerShiftDecrease(void)
 *
 * Description: DCtrackershift decrease
 *
 *****************************************************************************/
void SliderDCTrackerShiftDecrease(void)
{
		// All slider electrodes
		for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			// Load new shift to update the DC tracker
			sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE;
		}

		// All slider electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			// Load DC tracker buffer
			sliderDCTrackerDataBufferRaw[elecNum] = (sliderDCTrackerDataBuffer[elecNum]) << ((sliderDCTrackerDataShift[elecNum]));
		}
}
/*****************************************************************************
 *
 * Function: void SliderElectrodeCapToVoltConvELCH(uint32_t sliderElectrodeNum)
 *
 * Input: Number of the slider electrode
 *
 * Description: Convert slider electrode capacitance to equivalent voltage one by one by CPU
 *
 * Note: This function is not used as the second slider electrode is not acting as guard when scanning the first one
 *
 *****************************************************************************/
void SliderElectrodeCapToVoltConvELCH(uint32_t sliderElectrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif


	// Slider electrode capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider; sampleNum++)
	{
		// Distribute Electrode and Cext charge
		ChargeDistribution(&sliderElecStruct[sliderElectrodeNum]);
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
            :  "r" (&sliderElecStruct[sliderElectrodeNum].adcBasePtr->SC1[0]),  // operand %0
               "r" (sliderElecStruct[sliderElectrodeNum].adcChNum),             // operand %1
               "r" (&sliderElecStruct[sliderElectrodeNum].gpioBasePtr->PDDR),   // operand %2
               "r" ((sliderElecStruct[sliderElectrodeNum].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrodeNum].portMask))    // operand %3
#if (GUARD)
			   ,"r" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
			   "r" (guardStruct[0].portMask)  // operand %5
#endif
			   );
#else
		// Start Cext voltage ADC conversion
		sliderElecStruct[sliderElectrodeNum].adcBasePtr->SC1[0] = sliderElecStruct[sliderElectrodeNum].adcChNum;

		// Redistribute Electrode and Cext charge
		ChargeRedistribution(&sliderElecStruct[sliderElectrodeNum]);

#if (GUARD)
		//Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif

		// Equivalent voltage digitalization
		sliderAdcDataElectrodeDischargeRawSample[sampleNum] = EquivalentVoltageDigitalization(&sliderElecStruct[sliderElectrodeNum]);

		#if JITTERING
			#if (JITTERING_OPTION == 2)
				// Jitter sample rate
				Jitter(sliderAdcDataElectrodeDischargeRaw[sliderElectrodeNum][frequencyID]);
			#endif
		#endif
	}

#if SLIDER_NOISE_CANCELING
	SliderNoiseCanceling();
#endif

	#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Calculate samples sum
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrodeNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrodeNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
	}

	#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Calculate samples value average
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
	}
	sliderAdcDataElectrodeDischargeRaw[sliderElectrodeNum][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));
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
 * Function: void SliderSimultaneousElectrodeCapToVoltConvELCH(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
 *
 * Description: Drive&Convert both slider electrodes capacitance to equivalent voltage simultaneously - with 2 ADC modules at one time (HW trigger) by CPU
 *
 * Note I: Both slider electrode are scanned at once and guard pin can be used as a guard
 *
 * Note II: Both (All) slider electrodes pins must be on the same port, each slider electrode Cext must be on different ADC module
 *
 *****************************************************************************/
void SliderSimultaneousElectrodeCapToVoltConvELCH(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Simultaneous slider electrodes capacitance to voltage conversion
	for (sampleNum = 0; sampleNum < NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider; sampleNum++)
	{
		// Distribute Electrode and Cext charge
		SimultaneousChargeDistribution(&sliderElecStruct[sliderElectrode0Num],&sliderElecStruct[sliderElectrode1Num]);
		// Delay to distribute charge
		chargeDistributionPeriodTmp = chargeDistributionPeriod;
		while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

		#if (GUARD > 0)
		// Guard pin = 0
		guardStruct[0].gpioBasePtr->PCOR = 1 << guardStruct[0].pinNumberGuard;
		#endif

		// Cext voltage conversion - channel PRE-assignment for both slider electrodes
		sliderElecStruct[sliderElectrode0Num].adcBasePtr->SC1[0] = sliderElecStruct[sliderElectrode0Num].adcChNum;
		sliderElecStruct[sliderElectrode1Num].adcBasePtr->SC1[0] = sliderElecStruct[sliderElectrode1Num].adcChNum;

		// If compiler optimization is not -O3 (is set to none -> -O0), then set TS_ASM_OPTIMIZE macro to 1 in ts_cfg.h
#if(TS_ASM_OPTIMIZE == 1)
        asm volatile (
            /***** ASSEMBLY TEMPLATE ****************************************/
            // Start Cext voltage ADC conversion
            "str %1, [%0, #0]\n\t"    // store %1 to address in %0 with 0 byte offset
            // Redistribute Electrode and Cext charge
            "str %3, [%2, #0]"        // store %3 to address in %2 with 0 byte offset
#if (GUARD) // Guard pin = 1
			// Redistribute EGS Electrode and Cext charge
			"\n\tstr %5, [%4, #0]"        // store %5 to address in %4 with 0 byte offset
#endif
            /***** LIST OF OUTPUT OPERANDS **********************************/
            :
            /***** LIST OF INPUT OPERANDS ***********************************/
            :  "r" (&SIM->MISCTRL1),  // operand %0
               "r" (SIM_MISCTRL1_SW_TRG_MASK),             // operand %1
               "r" (&sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR),   // operand %2
               "r" ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask))    // operand %3
#if (GUARD)
			   ,"r" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
			   "r" (guardStruct[0].portMask)  // operand %5
#endif
        );
#else
		// Generate SW trigger to TRGMUX -> and thus initiate the Cext voltage conversion in both ADC0 and ADC1 at the same time
		// Optimization of code must be set the highest (-O3)
		SIM->MISCTRL1 = SIM_MISCTRL1_SW_TRG_MASK;

		// Redistribute Electrode and Cext charge
		SimultaneousChargeRedistribution(&sliderElecStruct[sliderElectrode0Num],&sliderElecStruct[sliderElectrode1Num]);

#if (GUARD)
		//Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif
			// Equivalent voltage digitalization for both electrodes of slider at once
			SimultaneousEquivalentVoltageDigitalization(&sliderElecStruct[sliderElectrode0Num],&sliderElecStruct[sliderElectrode1Num]);

			// Store result of the first and second electrode, clear COCO flag
			sliderAdcDataElectrodeDischargeRawSampleElec0[sampleNum] = sliderElecStruct[sliderElectrode0Num].adcBasePtr->R[0];
			sliderAdcDataElectrodeDischargeRawSampleElec1[sampleNum] = sliderElecStruct[sliderElectrode1Num].adcBasePtr->R[0];

		#if JITTERING
			#if (JITTERING_OPTION == 2)
				// Jitter sample rate
				Jitter(sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID]);
			#endif
		#endif
	}

#if SLIDER_NOISE_CANCELING
	SliderNoiseCanceling();
#endif

	#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Calculate samples sum
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec0[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
	}

	// Calculate samples sum
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec1[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
	}

	#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Calculate samples value average
	// First electrode
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec0[sampleNum];
	}
	sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));


	// Calculate samples value average
	// Second electrode
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec1[sampleNum];
	}
	sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));
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
 * Function: void SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum)
 *
 * Description: Drive&Convert both slider electrodes capacitance to equivalent voltage simultaneously - with 2 ADC modules at one time (HW trigger) by DMA
 *
 * Note I: Both slider electrode are scanned at once and guard pin can be used as a guard
 *
 * Note II: Both (All) slider electrodes pins must be on the same port, each slider electrode Cext must be on different ADC module
 *
 *****************************************************************************/
void SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
{

	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Start DMA for scanning both slider electrodes simultaneously
	start_TS_SLIDER_Simultaneous_DMA(sliderElectrode0Num, sliderElectrode1Num, (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider));																		// start DMA for slider
	while (!ready_dma_flag);																				// wait for result
	ready_dma_flag = 0;																						// reset flag

#if SLIDER_NOISE_CANCELING
	SliderNoiseCanceling();
#endif

#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Calculate samples sum
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec0[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
	}

	// Calculate samples sum
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec1[sampleNum];
	}

	if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
	}
	else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
	{
		sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
	}

#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Calculate samples value average
	// First electrode
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec0[sampleNum];
	}
	sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));


	// Calculate samples value average
	// Second electrode
	sliderAdcDataElectrodeDischargeRawCalc = 0;
	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSampleElec1[sampleNum];
	}
	sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));

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
 * Function: void SliderElectrodeCapToVoltConvELCHGuard(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
 *
 * Description: Drive both slider electrodes simultaneously, but convert slider electrodes one by one - with one ADC module at a time (SW trigger) by CPU
 *
 * Note I: One slider electrode is scanned and the other (together with guard pin, if configured) serves as guard
 *
 * Note II: Both (All) slider electrodes pins must be on the same port, each slider electrode Cext can be on the same or different ADC module
 *
 *****************************************************************************/
void SliderElectrodeCapToVoltConvELCHGuard(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Convert Slider electrode capacitance to equivalent voltage one by one
	for(elecNum = 0 ; elecNum < NUMBER_OF_SLIDER_ELECTRODES ; elecNum++)
	{
		// Slider electrodes capacitance to voltage conversion
		for (sampleNum = 0; sampleNum < NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider; sampleNum++)
		{
			// Distribute Electrode and Cext charge
			SimultaneousChargeDistribution(&sliderElecStruct[sliderElectrode0Num],&sliderElecStruct[sliderElectrode1Num]);
			// Delay to distribute charge
			chargeDistributionPeriodTmp = chargeDistributionPeriod;
			while (chargeDistributionPeriodTmp) {chargeDistributionPeriodTmp--;}

			#if (GUARD)
			// Guard pin = 0
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
            :  "r" (&sliderElecStruct[elecNum].adcBasePtr->SC1[0]),  // operand %0
               "r" (sliderElecStruct[elecNum].adcChNum),             // operand %1
               "r" (&sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR),   // operand %2
               "r" ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask))    // operand %3
#if (GUARD)
			   ,"r" (&guardStruct[0].gpioBasePtr->PSOR), // operand %4
			   "r" (guardStruct[0].portMask)  // operand %5
#endif
        );
#else
        // Start Cext voltage ADC conversion
		sliderElecStruct[elecNum].adcBasePtr->SC1[0] = sliderElecStruct[elecNum].adcChNum;

		// Redistribute Electrode and Cext charge
		SimultaneousChargeRedistribution(&sliderElecStruct[sliderElectrode0Num],&sliderElecStruct[sliderElectrode1Num]);

#if (GUARD)
		// Guard pin = 1
		guardStruct[0].gpioBasePtr->PSOR = 1 << guardStruct[0].pinNumberGuard;
#endif

#endif
			// Equivalent voltage digitalization
			sliderAdcDataElectrodeDischargeRawSample[sampleNum] = EquivalentVoltageDigitalization(&sliderElecStruct[elecNum]);

			#if JITTERING
				#if (JITTERING_OPTION == 2)
					Jitter(sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID]);
				#endif
			#endif
		}

#if SLIDER_NOISE_CANCELING
		SliderNoiseCanceling();
#endif

		#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
		// Calculate samples sum
		sliderAdcDataElectrodeDischargeRawCalc = 0;
		for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
		{
			sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
		}

		if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
		{
			sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
		}
		else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
		{
			sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
		}

		#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
		// Calculate samples value average
		sliderAdcDataElectrodeDischargeRawCalc = 0;
		for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
		{
			sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
		}
		sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));
		#else
		#error Please select valid TS method in ts_cfg_general.h
		#endif

	}
	#ifdef DEBUG_ELECTRODE_SENSE
	// Pin set
	R_RMW32(&(DES_GPIO->PSOR), DES_PIN, 1);
	#endif
	// Set ADCs gain back to calibrated value
	SetADCsGain();
}

/*****************************************************************************
 *
 * Function: void SliderElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum)
 *
 * Description: Drive both slider electrodes simultaneously, but convert slider electrodes one by one - with one ADC module at a time (SW trigger) by DMA
 *
 * Note I: One slider electrode is scanned and the other (together with guard pin, if configured) serves as guard (configured in tcd)
 *
 * Note II: Both (All) slider electrodes pins must be on the same port, each slider electrode Cext can be on the same or different ADC module
 *
 *****************************************************************************/
void SliderElectrodeCapToVoltConvELCH_DMA(uint32_t electrodeNum)
{
	// Clear ADCs gain (lower noise)
	ClearADCsGain();

#ifdef DEBUG_ELECTRODE_SENSE
	// Pin clear
	R_RMW32(&(DES_GPIO->PCOR), DES_PIN, 1);
#endif

	// Start DMA for scanning slider electrodes
	start_TS_SLIDER_DMA(electrodeNum, (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider));																		// start DMA for slider
	while (!ready_dma_flag);																				// wait for result
	ready_dma_flag = 0;																						// reset flag

#if SLIDER_NOISE_CANCELING
		SliderNoiseCanceling();
#endif

		#if(TS_RAW_DATA_CALCULATION == OVERSAMPLING)
		// Calculate samples sum
		sliderAdcDataElectrodeDischargeRawCalc = 0;
		for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
		{
			sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
		}

		if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE)
		{
			sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);
		}
		else if (numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
		{
			sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (sliderAdcDataElectrodeDischargeRawCalc >> OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE);
		}

		#elif(TS_RAW_DATA_CALCULATION == AVERAGING)
		// Calculate samples value average
		sliderAdcDataElectrodeDischargeRawCalc = 0;
		for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
		{
			sliderAdcDataElectrodeDischargeRawCalc = sliderAdcDataElectrodeDischargeRawCalc + sliderAdcDataElectrodeDischargeRawSample[sampleNum];
		}
		sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] = (int32_t)(sliderAdcDataElectrodeDischargeRawCalc / (numberOfElectrodeSensingCyclesPerSampleSlider-sliderDroppedSamples));
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
* Function: void SliderElectrodeTouchDetect(uint32_t electrodeNum)
*
* Description: Detect electrode touch
*
*****************************************************************************/
void SliderElectrodeTouchDetect(uint32_t electrodeNum)
{
	// Slider electrode touched ? z-axis touch detection
	if (sliderLPFilterData[electrodeNum][frequencyID] < sliderDetectorThresholdTouch[electrodeNum])

	{
		// y-axis touch confirmation
		if (sliderAdditionData < sliderAdditionDataThreshold)
		{
			// Report touch detected and confirmed
			sliderElectrodeTouch[electrodeNum][frequencyID] = 2;

			#ifdef WAKE_UP_ELECTRODE
			// Load counter to do not return to the wake-up function
			// Do not return to the wake-up function, when electrode released and electrode signal value is above touch and bellow release threshold
			// Expected SR <= 1s
			electrodeWakeUpActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;
			#else
			// Load counter to prevent switching of number of samples per cycle to idle
			sliderVirtualEGSActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;
			#endif

		}
		else
		{
			// Report touch detected, but not confirmed
			sliderElectrodeTouch[electrodeNum][frequencyID] = 1;
		}
	}
	// Electrode released?
	else
	{
		// Report released
		sliderElectrodeTouch[electrodeNum][frequencyID] = 0;
	}

	// Fast release detection
	// Previously touched slider Electrode FAST RELEASED? z-axis touch detection
	if (sliderElectrodeTouch[electrodeNum][frequencyID] > 1)
	{
		// Slider electrode raw data risen above touch threshold?
		if (sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] > sliderDetectorThresholdTouch[electrodeNum])
		{
			// Report released
			sliderElectrodeTouch[electrodeNum][frequencyID] = 0;
		}
	}
}
/*****************************************************************************
 *
 * Function: void SliderElectrodeTouchElecSense(void)
 *
 * Description: Sense slider electrodes
 *
 *****************************************************************************/
void SliderElectrodesTouchElecSense(void)
{
	// Disable interrupts
	//DisableInterrupts;

#ifndef WAKE_UP_ELECTRODE
#ifdef DEBUG_ALGORITHM
	// Pin clear
	R_RMW32(&(DA_GPIO->PCOR), DA_PIN, 1);
#endif
#endif

	// Configure all slider electrodes floating
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		ElectrodeFloat(&sliderElecStruct[elecNum]);
	}

	// Configure all touch button (and EGS) electrodes floating
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
	    ElectrodeFloat(&elecStruct[elecNum]);
	}

#if (NUMBER_OF_USED_ADC_MODULES == 2)
	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)

		// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once	by DMA
		SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(SLIDER_ELEC0,SLIDER_ELEC1);

	#else
		// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once by CPU
		SliderSimultaneousElectrodeCapToVoltConvELCH(SLIDER_ELEC0, SLIDER_ELEC1);
	#endif
#else

	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	for (elecNum = 0; elecNum < (NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		// Convert Slider electrode capacitance to equivalent voltage one by one using DMA (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCH_DMA(elecNum);
	}
	#else
		// Convert Slider electrode capacitance to equivalent voltage one by one using CPU (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCHGuard(SLIDER_ELEC0, SLIDER_ELEC1);
	#endif
#endif

	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC Tracker, if both slider electrodes not touched
		if (sliderElectrodeTouch[SLIDER_ELEC0][frequencyID] < 1 && sliderElectrodeTouch[SLIDER_ELEC1][frequencyID] < 1)
		{
			// Update DC Tracker
			sliderDCTrackerDataBuffer[elecNum] = DCTracker(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderDCTrackerDataBufferRaw[elecNum]), sliderDCTrackerDataShift[elecNum]);
		}

#if DECIMATION_FILTER
		// Decimation filtering
		SliderDecimationFilter(elecNum);
#endif
		// Calculate the relative touch threshold of the slider electrode
		sliderDetectorThresholdTouch[elecNum] = sliderDCTrackerDataBuffer[elecNum] - sliderDetectorThresholdTouchDelta[elecNum];
	}

	// All slider electrodes
	for(elecNum = NUMBER_OF_ELECTRODES; elecNum < (NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
#if DECIMATION_FILTER
		// Filter Slider electrode DF signal using IIR LP filter
		sliderLPFilterData[(elecNum) - NUMBER_OF_ELECTRODES][frequencyID] = FilterIIR1(elecNum, (tFrac32)(sliderAdcDataElectrodeDischargeRawDF[(elecNum) - NUMBER_OF_ELECTRODES][frequencyID]), frequencyID);
#else
		// Filter Slider electrode raw signal using IIR LP filter
		sliderLPFilterData[(elecNum) - NUMBER_OF_ELECTRODES][frequencyID] = FilterIIR1(elecNum, (tFrac32)(sliderAdcDataElectrodeDischargeRaw[(elecNum) - NUMBER_OF_ELECTRODES][frequencyID]), frequencyID);
#endif
	}

	// Calculate slider addition and difference data
	SliderDataCalculation(SLIDER_ELEC0, SLIDER_ELEC1);

	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Slider electrode touched?
		SliderElectrodeTouchDetect(elecNum);
	}

	// Qualify the slider touch event - x-axis touch qualification
	SliderDifferenceDataElectrodeTouchQualify(SLIDER_ELEC0, SLIDER_ELEC1);

	// Drive all Slider electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&sliderElecStruct[elecNum]);
	}

	// Drive all touch button (and EGS) electrodes to GND
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		ElectrodeGnd(&elecStruct[elecNum]);
	}
#ifndef WAKE_UP_ELECTRODE
#ifdef DEBUG_ALGORITHM
	// Pin set
	R_RMW32(&(DA_GPIO->PSOR), DA_PIN, 1);
#endif
#endif

	// Enable interrupts
	//EnableInterrupts;
}

/*****************************************************************************
 *
 * Function: void SliderDataCalculation(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
 *
 * Input: Numbers of the slider electrodes pair (e.g. 0, 1)
 *
 * Description: Calculate slider differential and addition data (el0-el1 etc.)
 *
 *****************************************************************************/
void SliderDataCalculation(uint32_t sliderElectrode0Num, uint32_t sliderElectrode1Num)
{
	// Difference data
	// Subtract the raw data from each other, normalize them with respect to their different baselines, and add SLIDER_DIFFADD_DATA_OFFSET (new baseline)
	// el0-el1
	sliderDifferenceData = (sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] - sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID]) - (sliderDCTrackerDataBuffer[sliderElectrode0Num] - sliderDCTrackerDataBuffer[sliderElectrode1Num]);
	sliderDifferenceData += SLIDER_DIFFADD_DATA_OFFSET;

	// Difference data filtered
	// Subtract the filtered data from each other, normalize them with respect to their different baselines, and add SLIDER_DIFFADD_DATA_OFFSET (new baseline)
	// el0-el1
	sliderDifferenceDataFiltered = (sliderLPFilterData[sliderElectrode0Num][frequencyID] - sliderLPFilterData[sliderElectrode1Num][frequencyID]) - (sliderDCTrackerDataBuffer[sliderElectrode0Num] - sliderDCTrackerDataBuffer[sliderElectrode1Num]);
	sliderDifferenceDataFiltered += SLIDER_DIFFADD_DATA_OFFSET;

	// Addition data
	// Add the raw data of both electrodes, normalize them with respect to their different baselines, and add SLIDER_DIFFADD_DATA_OFFSET (new baseline)
	// el0+el1
	sliderAdditionData = (sliderAdcDataElectrodeDischargeRaw[sliderElectrode0Num][frequencyID] + sliderAdcDataElectrodeDischargeRaw[sliderElectrode1Num][frequencyID]) - (sliderDCTrackerDataBuffer[sliderElectrode0Num] + sliderDCTrackerDataBuffer[sliderElectrode1Num]);
	sliderAdditionData += SLIDER_DIFFADD_DATA_OFFSET;

	// Calculate slider absolute raw and filtered data value, slider resolution: approx. range 0-49 (50 values)
	sliderAbsoluteRawData = (sliderDifferenceData - NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE);
	sliderAbsoluteRawDataFiltered = (sliderDifferenceDataFiltered - NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE);

}

/*****************************************************************************
 *
 * Function: void SliderDifferenceDataElectrodeTouchQualify (uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num)
 *
 * Input: Numbers of the slider electrodes pair (e.g. 0, 1)
 *
 * Description: Qualification of the slider touch event when using the differential data = ((el0-el1)-(el1-el0))
 *
 *****************************************************************************/
void SliderDifferenceDataElectrodeTouchQualify (uint32_t sliderElectrode0Num, uint32_t sliderElectrode1Num)
{

#if FREQUENCY_HOPPING
	// At least one of the slider electrodes reported as touched AND the number of samples per cycle has been risen (from previous mcu duty cycle)
	if (((sliderElectrodeTouch[sliderElectrode0Num][0] > 1 && sliderElectrodeTouch[sliderElectrode0Num][1] > 1) || (sliderElectrodeTouch[sliderElectrode1Num][0] > 1 && sliderElectrodeTouch[sliderElectrode1Num][1] > 1)) &&
			numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE && frequencyID == 1)
#else
		// At least one of the slider electrodes reported as touched AND the number of samples per cycle has been risen (from previous mcu duty cycle)
	if ((sliderElectrodeTouch[sliderElectrode0Num][frequencyID] > 1 || sliderElectrodeTouch[sliderElectrode1Num][frequencyID] > 1) &&
			numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
#endif
	{

		// Report slider electrode touch for flextimer for background lightning, prevent entry to VLPS
		sliderElectrodeTouchQualifiedReport = 1;
		// Reset  positionCounter of the slider RGB lights
		positionCounter = 1;
		// Set hysteresis detection to false
		hitHysteresis = FALSE;
		// Set decided position to false
		positionDeciced = FALSE;

		// Decide the position of finger based on thresholds in cfg

		// At least one threshold defined in cfg?
#if (NUMBER_OF_SLIDER_THRESHOLDS > 0)

		// Are the data below the first threshold - threshold[0]?
		if (sliderDifferenceData < sliderDiffThreshold[0])
		{
			// Check if the data had hit the hysteresis interval below the first threshold
			if (sliderDifferenceData > (sliderDiffThreshold[0] - numberOfHysteresisBlindPoints))
			{
				// Is this a first touch of the slider? -> qualify the position anyway
				if(firstSliderTouch == TRUE)
				{
					sliderElectrodeTouchQualified = positionCounter;
					positionDeciced = TRUE;
					firstSliderTouch = FALSE;
				}

				else
					// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
					hitHysteresis = TRUE;
			}

			// Else qualify the position on slider
			else
			{
				sliderElectrodeTouchQualified = positionCounter;
				positionDeciced = TRUE;
				firstSliderTouch = FALSE;
			}
		}

#if (NUMBER_OF_SLIDER_THRESHOLDS > 1)// More than one threshold defined in cfg?

		// Are the data somewhere between the threshold[0] and the last threshold then?
		else if((sliderDifferenceData >= sliderDiffThreshold[0]) &&
				(sliderDifferenceData < *sliderLastDiffThreshold))
		{

			// For cycle to determine the position across the thresholds - between the threshold[0] and the last threshold
			for(thresholdNum = 0; thresholdNum<NUMBER_OF_SLIDER_THRESHOLDS; thresholdNum++)
			{
				// Increase the  positionCounter of the RGB lights by one
				positionCounter++;

				if (sliderDifferenceData >= sliderDiffThreshold[thresholdNum] &&
						sliderDifferenceData < sliderDiffThreshold[thresholdNum+1])
				{
					// Now check if the data had hit the top or bottom hysteresis interval within the e.g first and e.g second threshold (under first and above second threshold)
					if (sliderDifferenceData > (sliderDiffThreshold[thresholdNum] + numberOfHysteresisBlindPoints) &&
							sliderDifferenceData < (sliderDiffThreshold[thresholdNum+1] - numberOfHysteresisBlindPoints))
					{
						// The data are in the active area = between the hysteresis intervals - thus qualify the position on slider
						sliderElectrodeTouchQualified = positionCounter;
						positionDeciced = TRUE;
						firstSliderTouch = FALSE;
					}

					else
					{
						// Is this a first touch of the slider? -> qualify the position anyway
						if(firstSliderTouch == TRUE)
						{
							sliderElectrodeTouchQualified = positionCounter;
							positionDeciced = TRUE;
							firstSliderTouch = FALSE;
						}

						else
							// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
							hitHysteresis = TRUE;
					}

					// Exit for cycle
					break;
				}
			}
		}

#endif
		// The data did not fit between any thresholds - are the data above the last threshold?
		else
		{
			// Check if the data were previously marked as within hysteresis interval
			if(hitHysteresis == TRUE || positionDeciced == TRUE)
			{
				// Do not change the sliderElectrodeTouchQualified
			}

			// Check for hysteresis above the last threshold (similar principle as checking for hysteresis below the first threshold)
			else
			{
				if(sliderDifferenceData >= *sliderLastDiffThreshold &&
						sliderDifferenceData < (*sliderLastDiffThreshold + numberOfHysteresisBlindPoints))
				{
					// Is this a first touch of the slider? -> qualify the position anyway as the highest
					if(firstSliderTouch == TRUE)
					{
						sliderElectrodeTouchQualified = NUMBER_OF_SLIDER_SEGMENTS;
						positionDeciced = TRUE;
						firstSliderTouch = FALSE;
					}

					else
						// It hit the blind hysteresis interval and the first touch has already been done - set hitHysteresis True
						hitHysteresis = TRUE;

				}

				// The data were not within the hysteresis interval
				else
				{
					// Qualify the position on slider as the highest
					sliderElectrodeTouchQualified = NUMBER_OF_SLIDER_SEGMENTS;
					positionDeciced = TRUE;
					firstSliderTouch = FALSE;
				}
			}
		}
#endif

	}

	// Else no touch?
	else
	{
#if FREQUENCY_HOPPING
		if (frequencyID == 1)
#endif
		{
			// Report slider electrode release for flextimer for background lightning and for jumping to VLPS
			sliderElectrodeTouchQualifiedReport = 0;

			// Report no slider qualified touch
			sliderElectrodeTouchQualified = 0;

			// Reset the firstSliderTouch variable
			firstSliderTouch = TRUE;
		}
	}
}

/*****************************************************************************
 *
 * Function: void SliderelectrodeWakeElecSense(void)
 *
 * Description: Sense slider electrodes to keep track of the electrodes baselines while scanning the wake up electrode
 *
 *****************************************************************************/

void SliderelectrodeWakeElecSense(void)
{
#if (NUMBER_OF_USED_ADC_MODULES == 2)
	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once by DMA
	SliderSimultaneousElectrodeCapToVoltConvELCH_DMA(SLIDER_ELEC0,SLIDER_ELEC1);
	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC Tracker of both slider electrodes
		sliderDCTrackerDataBuffer[elecNum] = DCTracker(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderDCTrackerDataBufferRaw[elecNum]), sliderDCTrackerDataShift[elecNum]);
	}
	#else
		// Convert slider electrodes capacitance to equivalent voltage simultaneously - at once by CPU
		SliderSimultaneousElectrodeCapToVoltConvELCH(SLIDER_ELEC0, SLIDER_ELEC1);
		// All slider electrodes
		for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
		{
			// Update DC Tracker of both slider electrodes
			sliderDCTrackerDataBuffer[elecNum] = DCTracker(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderDCTrackerDataBufferRaw[elecNum]), sliderDCTrackerDataShift[elecNum]);
		}
	#endif
#else
	#if (TS_ELECTRODE_DATA_ACQUISITION == DMA_ELECTRODE_DATA_ACQUISITION)
	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Convert Slider electrode capacitance to equivalent voltage one by one by DMA (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCH_DMA(elecNum);
		// Update DC Tracker of both slider electrodes
		sliderDCTrackerDataBuffer[elecNum] = DCTracker(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderDCTrackerDataBufferRaw[elecNum]), sliderDCTrackerDataShift[elecNum]);
	}
	#else
		// Convert Slider electrode capacitance to equivalent voltage one by one by CPU (the other slider electrode serves as a guard)
		SliderElectrodeCapToVoltConvELCHGuard(SLIDER_ELEC0, SLIDER_ELEC1);
	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC Tracker of both slider electrodes
		sliderDCTrackerDataBuffer[elecNum] = DCTracker(sliderAdcDataElectrodeDischargeRaw[elecNum][frequencyID], &(sliderDCTrackerDataBufferRaw[elecNum]), sliderDCTrackerDataShift[elecNum]);
	}
	#endif
#endif

}

/*****************************************************************************
 *
 * Function: void SliderElectrodeWakeElecFilterLoad(void)
 *
 * Description: Update slider electrodes filter in case wake-up (EGS) electrode was touched
 *
 *****************************************************************************/
void SliderElectrodeWakeElecFilterLoad(void)
{
	// All slider electrodes
	for (elecNum = NUMBER_OF_ELECTRODES; elecNum < (NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
	{
		// Touch not detected nor confirmed on both slider electrodes?
		if (sliderElectrodeTouch[SLIDER_ELEC0][frequencyID] < 1 && sliderElectrodeTouch[SLIDER_ELEC1][frequencyID] < 1)
		{
		// Load IIR1 LP Filter buffer (X, Y) with (sliderDCTracker - threshold / 2)
		FilterIIR1BufferInit(elecNum, ((tFrac32)(sliderDCTrackerDataBuffer[(elecNum) - NUMBER_OF_ELECTRODES] - (sliderDetectorThresholdTouchDelta[(elecNum) - NUMBER_OF_ELECTRODES] / 2))), \
				((tFrac32)((sliderDCTrackerDataBuffer[(elecNum) - NUMBER_OF_ELECTRODES]- (sliderDetectorThresholdTouchDelta[(elecNum) - NUMBER_OF_ELECTRODES] / 2)))), frequencyID);

		}
	}

}
#if SLIDER_NOISE_CANCELING
/*****************************************************************************
 *
 * Function: void SliderNoiseCanceling(void)
 *
 * Description: Slider noise canceling function
 *
 *****************************************************************************/
void SliderNoiseCanceling(void)
{
	switch(NUMBER_OF_USED_ADC_MODULES)
	{
	case 2:
	{
		// Find minimal sample values for each electrode and cancel it
		if(numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
		{

			for(sampleNum=0; sampleNum < 1; sampleNum++)
			{
				FindMinimumAndDeleteIt(sliderAdcDataElectrodeDischargeRawSampleElec0);
				FindMinimumAndDeleteIt(sliderAdcDataElectrodeDischargeRawSampleElec1);
			}
			sliderDroppedSamples = 1;
		}
		else

			sliderDroppedSamples = 0;
	}
	break;

	case 1:
	{
		// Find minimal sample values for each electrode and cancel it
		if(numberOfElectrodeSensingCyclesPerSampleSlider == NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE)
		{

			for(sampleNum=0; sampleNum<1; sampleNum++)
			{
				FindMinimumAndDeleteIt(sliderAdcDataElectrodeDischargeRawSample);
			}
			sliderDroppedSamples = 1;
		}
		else

			sliderDroppedSamples = 0;

	}
	break;
	}

}
/*****************************************************************************
 *
 * Function: void FindMinimumAndDeleteIt(int16_t sliderInputSamplesArray[])
 *
 * Input: Array of ADC cycles (scans) for one averaged sample
 *
 * Description: Finds minimum element in an array and deletes it, can be used multiple times
 *
 *****************************************************************************/
void FindMinimumAndDeleteIt(int16_t sliderInputSamplesArray[])
{

	int16_t min, index, i, sampleNum;
	i = 0;

	do
	{
		min = sliderInputSamplesArray[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE+i];//set the minimum on [1] place in the array
		index = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE+i;
		i++;
	}while(min == 0);// While the minimum is zero, try  next element

	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < (NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider); sampleNum++)
	{
		//if the array element is smaller than the set minimum and is not 0
		if (sliderInputSamplesArray[sampleNum] < min && sliderInputSamplesArray[sampleNum] > 0)
		{
			index = sampleNum;
			min = sliderInputSamplesArray[sampleNum];
		}
	}

	sliderInputSamplesArray[index] = 0;// Delete that minimum element
}

/*****************************************************************************
 *
 * Function: void FindMaximumAndDeleteIt(int16_t sliderInputSamplesArray[])
 *
 * Input: Array of ADC cycles (scans) for one averaged sample
 *
 * Description: Finds maximum element in an array and deletes it, can be used multiple times
 *
 *****************************************************************************/
void FindMaximumAndDeleteIt(int16_t sliderInputSamplesArray[])
{

	int16_t max, index, sampleNum;

	max = sliderInputSamplesArray[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE];
	index = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE;

	for (sampleNum = NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE; sampleNum < NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + numberOfElectrodeSensingCyclesPerSampleSlider; sampleNum++)
	{
		if (sliderInputSamplesArray[sampleNum] > max)
		{
			index = sampleNum;
			max = sliderInputSamplesArray[sampleNum];
		}
	}

	sliderInputSamplesArray[index] = 0;
}
#endif

#ifdef WAKE_UP_ELECTRODE
/*****************************************************************************
 *
 * Function: void SliderSensingCyclesChangeEGS(void)
 *
 * Description: Changes the number of sensing cycles for slider electrodes per sample based on whether the proximity EGS was touched
 *
 *****************************************************************************/
void SliderSensingCyclesChangeEGS(void)
{
	if(electrodeTouch[WAKE_UP_ELECTRODE][0] == 1)
	{
		// Change the number of slider sensing cycles per sample to active
		numberOfElectrodeSensingCyclesPerSampleSlider = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE;
	}
	else
	{
		// Change the number of slider sensing cycles per sample to idle
		numberOfElectrodeSensingCyclesPerSampleSlider = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE;
	}
}
#else
/*****************************************************************************
 *
 * Function: void SliderSensingCyclesChange(void)
 *
 * Description: Changes the number of sensing cycles per sample for slider electrodes
 *
 *****************************************************************************/
void SliderSensingCyclesChange(void)
{
	uint8_t electrodeNum;

	// Detect if any of the slider electrodes was reported as proximately touched
	for(electrodeNum = 0; electrodeNum < NUMBER_OF_SLIDER_ELECTRODES; electrodeNum++)
	{
#if FREQUENCY_HOPPING
		// If on both frequencies proximity detected
		if((sliderLPFilterData[electrodeNum][0] < (sliderDCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA)) &&  \
				(sliderLPFilterData[electrodeNum][1] < (sliderDCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA)))
#else
			// If proximity detected
			if(sliderLPFilterData[electrodeNum][frequencyID] < (sliderDCTrackerDataBuffer[electrodeNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA))
#endif
			{
				// Report wake up
				sliderVirtualEGSActivateCounter = ELEC_WAKEUP_ACTIVATE_COUNTER;

				// If this is first touch
				if (sliderVirtualEGSTouch != 1)
				{
					// Report that slider electrodes proximately touched
					sliderVirtualEGSTouch = 1;

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
					// Activate the oversampling just once after first confirmed proximity
					if (sliderOversamplingActivationReport != 1)
					{
						// Activate slider electrodes DC tracker adjustments for oversampling - to active mode
						SliderElecOversamplingActivation();

						// Report slider Oversampling activation done
						sliderOversamplingActivationReport = 1;

						// Save frequencyID
						frequencyIDsave = frequencyID;

						// All (both) used scanning frequencies
						for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
						{
							// All slider electrodes
							for(elecNum = NUMBER_OF_ELECTRODES; elecNum < (NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
							{
								// Load IIR1 LP Filter buffer (X, Y) exactly with corresponding value
								FilterIIR1BufferInit(elecNum, ((tFrac32)(sliderDCTrackerDataBuffer[elecNum - NUMBER_OF_ELECTRODES] - (VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA << (OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE - OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE)))), \
										((tFrac32)(sliderDCTrackerDataBuffer[elecNum - NUMBER_OF_ELECTRODES] - (VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA << (OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE - OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE)))), frequencyID);
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
							// All slider electrodes
							for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
							{
								// Load DF array exactly with corresponding value
								sliderAdcDataElectrodeDischargeRawDF[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum] - VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA;
							}
						}
						// Set back original frequencyID
						frequencyID = frequencyIDsave;
#endif

					}
#endif
					// Change the number of slider sensing cycles per sample to active
					numberOfElectrodeSensingCyclesPerSampleSlider = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE;

				}
				// Exit the for loop
				break;
			}

			else
			{
				// Came to the last slider electrode (did not manage to escape the loop so far)
				if(electrodeNum == (NUMBER_OF_SLIDER_ELECTRODES - 1))
				{
					// Wake up counter
					if(sliderVirtualEGSActivateCounter == 0)
					{
						// Report that slider electrodes proximity released
						sliderVirtualEGSTouch = 0;

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
						if (sliderOversamplingActivationReport == 1)
						{
							// Deactivate slider electrodes DC tracker adjustments for oversampling - back to idle mode
							SliderElecOversamplingDeactivation();

							// Save frequencyID
							frequencyIDsave = frequencyID;

							// Load DF array with new DC tracker value
							for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
							{
								// All slider electrodes
								for(elecNum = NUMBER_OF_ELECTRODES; elecNum < (NUMBER_OF_ELECTRODES + NUMBER_OF_SLIDER_ELECTRODES); elecNum++)
								{
									// Load IIR1 LP Filter buffer (X, Y) with DC tracker value
									FilterIIR1BufferInit(elecNum, ((tFrac32)(sliderDCTrackerDataBuffer[elecNum - NUMBER_OF_ELECTRODES])), \
											((tFrac32)(sliderDCTrackerDataBuffer[elecNum - NUMBER_OF_ELECTRODES])), frequencyID);
								}
							}
							// Set back original frequencyID
							frequencyID = frequencyIDsave;

							// Report electrodes Oversampling de-activation done
							sliderOversamplingActivationReport = 0;

#if DECIMATION_FILTER
							// Save frequencyID
							frequencyIDsave = frequencyID;

							// All (both) used scanning frequencies
							for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
							{
								// All slider electrodes
								for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
								{
									// Load DF array with new DC tracker value
									sliderAdcDataElectrodeDischargeRawDF[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum];
								}
							}
							// Set back original frequencyID
							frequencyID = frequencyIDsave;
#endif
						}
#endif
						// Change the number of slider sensing cycles per sample to idle
						numberOfElectrodeSensingCyclesPerSampleSlider = NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE;

					}
					else
					{
						// Decrement wake up activate counter
						if(sliderVirtualEGSActivateCounter != 0)
						{
							sliderVirtualEGSActivateCounter--;
						}
					}
				}
			}
	}
}
#endif

#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
/*****************************************************************************
 *
 * Function: void SliderElecOversamplingActivation(void)
 *
 * Description: Activate slider electrodes DC tracker adjustments for oversampling - to active mode
 *
 *****************************************************************************/
void SliderElecOversamplingActivation(void)
{
	// All slider electrodes
	for(elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Load new shift to update the DC tracker
		sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE;

		// Set DCTracker value to correspond to number of samples taken in Active state
		sliderDCTrackerDataBuffer[elecNum] = sliderDCTrackerDataBuffer[elecNum] << (OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE - OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);

		// Load DC tracker buffer
		sliderDCTrackerDataBufferRaw[elecNum] = (sliderDCTrackerDataBuffer[elecNum]) << ((sliderDCTrackerDataShift[elecNum]));

		// Slider electrode threshold recalculation
		sliderDetectorThresholdTouch[elecNum] = sliderDCTrackerDataBuffer[elecNum] - sliderDetectorThresholdTouchDelta[elecNum];
	}

		#if DECIMATION_FILTER
		   // Save frequencyID
		   frequencyIDsave = frequencyID;

		   // All (both) used scanning frequencies
			for (frequencyID = 0; frequencyID < NUMBER_OF_HOPPING_FREQUENCIES; frequencyID++)
			{
				// All slider electrodes
				for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
				{
					// Load DF array with DC tracker value  - used when oversampling with EGS
					sliderAdcDataElectrodeDischargeRawDF[elecNum][frequencyID] = sliderDCTrackerDataBuffer[elecNum];
				}
			}
			// Set back original frequencyID
			frequencyID = frequencyIDsave;
		#endif
}

/*****************************************************************************
 *
 * Function: void SliderElecOversamplingDeactivation(void)
 *
 * Description: Deactivate slider electrodes DC tracker adjustments for oversampling - back to idle mode
 *
 *****************************************************************************/
void SliderElecOversamplingDeactivation(void)
{
	// All slider electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		// Update DC tracker shift
		sliderDCTrackerDataShift[elecNum] = SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE;

		// Set DCTracker value to correspond to number of samples taken in Active state
		sliderDCTrackerDataBuffer[elecNum] = sliderDCTrackerDataBuffer[elecNum] >> (OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE - OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE);

		// Load DC tracker buffer
		sliderDCTrackerDataBufferRaw[elecNum] = (sliderDCTrackerDataBuffer[elecNum]) << ((sliderDCTrackerDataShift[elecNum]));

		// Slider electrode threshold recalculation
		sliderDetectorThresholdTouch[elecNum] = sliderDCTrackerDataBuffer[elecNum] - sliderDetectorThresholdTouchDelta[elecNum];
	}

}
#endif

#if DECIMATION_FILTER
/*****************************************************************************
 *
 * Function: void SliderDecimationFilter(uint32_t electrodeNum)
 *
 * Description: Incerement or decrement decimation filter value
 *
 *****************************************************************************/
void SliderDecimationFilter(uint32_t electrodeNum)
{
		// Is the electrode raw data value still smaller than electrode decimation filter value?
		if(sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] < sliderAdcDataElectrodeDischargeRawDF[electrodeNum][frequencyID])
		{
			// Decrement DF value - follow the raw data down
			sliderAdcDataElectrodeDischargeRawDF[electrodeNum][frequencyID] -= DECIMATION_STEP ;
		}
		// Is the electrode raw data value same as electrode decimation filter value?
		else if ((sliderAdcDataElectrodeDischargeRaw[electrodeNum][frequencyID] == sliderAdcDataElectrodeDischargeRawDF[electrodeNum][frequencyID]))
		{
			// Do nothing
		}

		else
		{
			// Increment DF value - follow  the raw data back up to the baseline
			sliderAdcDataElectrodeDischargeRawDF[electrodeNum][frequencyID] += DECIMATION_STEP;
		}
}

#endif

// Slider enable
#endif
