/*
 * @file     2pad_app.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Application configuration file for 2pad EVB demo
*
 */

#ifndef CFG_2PAD_EVB_2PAD_APP_H_
#define CFG_2PAD_EVB_2PAD_APP_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/2pad_EVB/2pad_hw.h>
#include <TS/Cfg/ts_cfg_general.h>

#if (REFERENCE_DESIGN_BOARD == S32K144_2PAD_EVB)
/*******************************************************************************
* Modify: Number of following electrode pre-cycles per single sample
******************************************************************************/
// Value from 1 to 2
#define NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE    1

/*******************************************************************************
* Modify: Number of following electrode cycles per single sample
******************************************************************************/
#if TS_RAW_DATA_CALCULATION == AVERAGING
	// IDLE MODE, when EGS/touch buttons are not touched
	// Value from 1 to 4  to achieve MCU 70uA max average current consumption
	#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE   4
	// ACTIVE MODE, when EGS/touch buttons are touched
	// Number of samples, when module in active mode, max value 128
	#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE  16

#elif TS_RAW_DATA_CALCULATION == OVERSAMPLING

	/*******************************************************************************
	* Modify: Define oversampling resolution, when using 12bit ADC:
				14bit resolution: 14
				15bit resolution: 15
				16bit resolution: 16
	******************************************************************************/
	// Define resolution
	#define OVERSAMPLING_RESULT_RESOLUTION_BITS   14

	// IDLE MODE, when EGS/touch buttons are not touched
	// Value from 1 to 4  to achieve MCU 70uA max average current consumption
	// In idle mode the sampling is basically 13 bits Oversampling
	#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE   4
	#define OVERSAMPLING_VALUE_SHIFT_IDLE  1

	// ACTIVE MODE, when EGS/touch buttons are touched
	// Number of samples, when module in active mode, max value 256

	// 14bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS == 14)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE   16
		#define OVERSAMPLING_VALUE_SHIFT_ACTIVE  2
	#endif

	// 15bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS == 15)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE   64
		#define OVERSAMPLING_VALUE_SHIFT_ACTIVE  3
	#endif

	// 16bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS == 16)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE   256
		#define OVERSAMPLING_VALUE_SHIFT_ACTIVE  4
	#endif

#endif
/*******************************************************************************
* Do not modify!
******************************************************************************/
// Required for IIR filter correct operation.
// SHIFT_NUMBER = (2^31 / (2^12 * NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_IDLE))
// Go to: https://www.wolframalpha.com/
// Calculate: IIR_FILTER_VALUE_SHIFT = round down [log2(SHIFT_NUMBER)]
// Calculated for max 128 samples
#define IIR_FILTER_VALUE_SHIFT  12

#if (NUMBER_OF_ELECTRODES > 0)
	/*******************************************************************************
	* Modify: Jittering ON/OFF and jittering bits
	* Note: Jittering option 2 only works when CPU is selected for electrode data acquisition
	******************************************************************************/
	// YES (JITTERING_ON) or NO (JITTERING_OFF) optional jittering sample rate control
	#define JITTERING JITTERING_OFF

		#if JITTERING
			// Modify: Define jittering option: 1 - jitter at the beginning of IRQ handler, 2 - jitter in between each following ADC sample scan
			#define JITTERING_OPTION 2

			// Modify: Value of 2 to 8
			#define NUMBER_OF_JITTERING_BITS 3

			// Do not modify!: the jittering_mask_macro
			#define JITTERING_MASK ((1 << NUMBER_OF_JITTERING_BITS)-1)
		#endif

	/*******************************************************************************
	* Modify: Frequency hopping ON/OFF
	* WARNING: if TS_RAW_DATA_CALCULATION == OVERSAMPLING then OVERSAMPLING_RESULT_RESOLUTION_BITS and
	*  must be chosen appropriately - so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
	******************************************************************************/
	// YES (FREQUENCY_HOPPING_ON) or NO (FREQUENCY_HOPPING_OFF) optional frequency hopping
	#define FREQUENCY_HOPPING FREQUENCY_HOPPING_OFF

		#if FREQUENCY_HOPPING
			// Do not modify!: 1 core scanning period/frequency (30ms) + 1 extra scanning period/frequency
			// SW written for maximum of 2 scanning periods/frequencies only
			#define NUMBER_OF_HOPPING_FREQUENCIES 2

			// Modify: Extra scanning period/frequency 330us after the core period
			// (LPIT fed by 48 MHz clock => Value 48 000 equals delay 1 ms)
			#define ELECTRODES_SENSE_PERIOD_FH   15840

			// Modify: Frequency hopping filter factor addition for DCTracker
			#define FH_DCTRACKER_FILTER_FACTOR 1
		#else
			// Do not modify!: 1 core scanning period/frequency (30ms)
			#define NUMBER_OF_HOPPING_FREQUENCIES 1

			// Do not Modify!: Frequency hopping disabled - no filter factor addition for DCTracker
			#define FH_DCTRACKER_FILTER_FACTOR 0
		#endif

	/*******************************************************************************
	* Modify: Decimation filter ON/OFF
	* WARNING: if TS_RAW_DATA_CALCULATION == OVERSAMPLING then OVERSAMPLING_RESULT_RESOLUTION_BITS and
	*  must be chosen appropriately - so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
	******************************************************************************/
	// YES (DECIMATION_FILTER_ON) or NO (DECIMATION_FILTER_OFF) optional decimation filter
	#define DECIMATION_FILTER DECIMATION_FILTER_OFF

		#if DECIMATION_FILTER
			// Modify: LPTMR new wake-up period - 3 ms
			// Value between 3 and 30
			#define ELECTRODES_SENSE_PERIOD_DF 3

			// Modify: Decimation filter filter factor addition for DCTracker
			#define DF_DCTRACKER_FILTER_FACTOR (3 - FH_DCTRACKER_FILTER_FACTOR)

			#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
				// Modify: Decimation filter step (minimal step 1)
				#define DECIMATION_STEP 10
			#elif (TS_RAW_DATA_CALCULATION == AVERAGING)
				// Modify: Decimation filter step (minimal step 1)
				#define DECIMATION_STEP 1
			#else
				#error Please select valid demo hardware in ts_cfg_general.h
			#endif
		#else
			// Do not Modify!: Decimation filter disabled - no filter factor addition for DCTracker
			#define DF_DCTRACKER_FILTER_FACTOR 0
		#endif
#else
	// Do not Modify!
	#define FH_DCTRACKER_FILTER_FACTOR 0
	#define DF_DCTRACKER_FILTER_FACTOR 0
	#define NUMBER_OF_HOPPING_FREQUENCIES 1

#endif

/*******************************************************************************
* Modify: Define application mode, select only one option designed for 7 electrode keyboard:
*         MODE_REACTION_TIME_90MS and MODE_CUSTOM
*
* Note: Response time stands for period of electrodes sensing (scan period)
*       Reaction time stands for period between electrode touch and application report "electrode touched"
******************************************************************************/
// If "#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_YES": MCU response time 30ms, MCU reaction time 90ms, MCU average current consumption 70uA
// If "#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_NO": MCU response time 30ms, MCU reaction time 90ms, MCU average current consumption 130uA
#define MODE_REACTION_TIME_90MS

// Custom settings:
// Preset to same operation as in case of MODE_REACTION_TIME_90MS
// If "#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_YES": MCU response time 30ms, MCU reaction time 90ms, MCU average current consumption 70uA
// If "#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_NO": MCU response time 30ms, MCU reaction time 90ms, MCU average current consumption 130uA
//#define MODE_CUSTOM

/*******************************************************************************
* Modify: Define electrodes sensing (scanning) cycle in milliseconds [ms].
*         Value from 1 to 65535.
******************************************************************************/
#ifdef MODE_REACTION_TIME_90MS
	#define ELECTRODES_SENSE_PERIOD   30
#endif
#ifdef MODE_CUSTOM
	#define ELECTRODES_SENSE_PERIOD   30
#endif

/*******************************************************************************
* Modify: Define filter parameters.
*         Coefficients B0, B1 and A0 are dependent on filter cutoff frequency [Hz]
*         and electrode sensing period, see ELECTRODES_SENSE_PERIOD above.
*         Coefficient for filter are calculated with:
*         http://engineerjs.com/?sidebar=docs/iir.html
*
******************************************************************************/
#ifdef MODE_REACTION_TIME_90MS
	#if DECIMATION_FILTER
		// Define electrodes IIR filter parameters
		// ELECTRODES_SENSE_PERIOD_DF   3
		#define FILTER_1                   0
		// Cutoff frequency in Hertz [Hz]
		#define FILTER_1_CUTOFF_FREQ_HZ    1
		// Coefficient B0
		#define FILTER_1_COEF_B0          0.009337054753656
		// Coefficient B1
		#define FILTER_1_COEF_B1          0.009337054753656
		// Coefficient A0
		#define FILTER_1_COEF_A0         -0.981325890492688
	#else
		// Define electrodes IIR filter parameters
		// ELECTRODES_SENSE_PERIOD   30
		#define FILTER_1                   0
		// Cutoff frequency in Hertz [Hz]
		#define FILTER_1_CUTOFF_FREQ_HZ    1
		// Coefficient B0
		#define FILTER_1_COEF_B0           0.086364027013762
		// Coefficient B1
		#define FILTER_1_COEF_B1           0.086364027013762
		// Coefficient A0
		#define FILTER_1_COEF_A0           -0.827271945972476
	#endif
#endif
#ifdef MODE_CUSTOM
		// Define electrodes IIR filter parameters
		// ELECTRODES_SENSE_PERIOD   30
		#define FILTER_1                   0
		// Cutoff frequency in Hertz [Hz]
		#define FILTER_1_CUTOFF_FREQ_HZ    1
		// Coefficient B0
		#define FILTER_1_COEF_B0           0.086364027013762
		// Coefficient B1
		#define FILTER_1_COEF_B1           0.086364027013762
		// Coefficient A0
		#define FILTER_1_COEF_A0           -0.827271945972476
#endif

// Number of used filters
#define NUMBER_OF_FILTERS_USED     1


/*******************************************************************************
* Modify: Electrode 0 to 2 common defines
******************************************************************************/
#if(TS_RAW_DATA_CALCULATION == AVERAGING)
	// DC tracker response, when in Idle mode
	// Value 5 equals 1s at 30ms sampling period
	#define ELEC_DCTRACKER_FILTER_FACTOR_IDLE     5
	// DC tracker response, when in Active mode
	// Value 5 equals 1s at 30ms sampling period
	#define ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	// Electrode touch threshold relative to DC tracker value
	#define ELEC_TOUCH_THRESHOLD_DELTA       22
	// Electrode release threshold relative to DC tracker value
	#define ELEC_RELEASE_THRESHOLD_DELTA     20
#elif (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// DC tracker response, when in Idle mode
	// Value 5 equals 1s at 30ms sampling period
	#define ELEC_DCTRACKER_FILTER_FACTOR_IDLE     5
	// DC tracker response, when in Active mode
	// Value 5 equals 1s at 30ms sampling period
	#define ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	// Electrode touch threshold relative to DC tracker value
	#define ELEC_TOUCH_THRESHOLD_DELTA            50
	// Electrode release threshold relative to DC tracker value
	#define ELEC_RELEASE_THRESHOLD_DELTA          40
#else
	#error Please select valid TS method in ts_cfg_general.h
#endif
/*******************************************************************************
* Modify: Electrode 0 defines
******************************************************************************/
#ifdef ELEC0
	// DC tracker response, the highest number, the slower response.
	// Select values from 1 to 8
	// Value 5 equals 1s at 30ms sampling period
	#define ELEC0_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	// IIR1 LP filter selection
	#define ELEC0_LPFILTER_TYPE               FILTER_1
	// Electrode touch threshold relative to DC tracker value
	#define ELEC0_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	// Electrode release threshold relative to DC tracker value
	#define ELEC0_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Electrode 1 defines
******************************************************************************/
#ifdef ELEC1
	#define ELEC1_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define ELEC1_LPFILTER_TYPE               FILTER_1
	#define ELEC1_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	#define ELEC1_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Virtual EGS Electrode touch threshold (for samples switching)
******************************************************************************/
#if(TS_RAW_DATA_CALCULATION == AVERAGING)
	// Virtual EGS touch threshold relative to DC tracker value
	#define VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA 3
#elif (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Virtual EGS touch threshold relative to DC tracker value
	#define VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA 6
#else
	#error Please select valid TS method in ts_cfg_general.h
#endif

/*******************************************************************************
* Modify: Keypad backlight PWM dutycycle from 0 to 100
******************************************************************************/
#define BACKLIGHT_PWM_DUTYCYCLE    30

/*******************************************************************************
* Modify: Keypad backlight ON period in seconds
******************************************************************************/
#define BACKLIGHT_ON_PERIOD    3

#endif
#endif /* CFG_2PAD_EVB_2PAD_APP_H_ */
