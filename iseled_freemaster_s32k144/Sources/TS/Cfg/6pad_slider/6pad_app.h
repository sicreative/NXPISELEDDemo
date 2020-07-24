/*
 * @file     6pad_app.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Application configuration file for 6pad keypad with slider reference design
*
 */

#ifndef CFG_6PAD_SLIDER_6PAD_APP_H_
#define CFG_6PAD_SLIDER_6PAD_APP_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/6pad_slider/6pad_hw.h>
#include <TS/Cfg/ts_cfg_general.h>

#if (REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER)
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
* Modify: Additional wake-up electrode 5-10 cm proximity functionality
******************************************************************************/
#if (OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_YES)
// YES (WAKE_UP_ELECTRODE_PROXIMITY_YES) or NO (WAKE_UP_ELECTRODE_PROXIMITY_NO) optional wake-up electrode proximity
// If YES, the EGS electrode will provide 5-10 cm proximity
// If NO, the EGS electrode will just provide wake up when electrodes touched
#define WAKE_UP_ELECTRODE_PROXIMITY   WAKE_UP_ELECTRODE_PROXIMITY_NO
#endif
/*******************************************************************************
* Modify: Additional wake-up proximity electrode number of  electrode cycles per single sample
******************************************************************************/
#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
	// Value from 32 to 256
	#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY  64
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

#if (NUMBER_OF_ELECTRODES > 0 || NUMBER_OF_SLIDER_ELECTRODES > 0)
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
	* OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER must be chosen appropriately -
	* so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
	* WARNING: if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES) then NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY
	* and ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH must be chosen appropriately -
	* so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
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
	* OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER must be chosen appropriately -
	* so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
	* WARNING: if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES) then NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY
	* and ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH must be chosen appropriately -
	* so that all the sampling fits in ELECTRODES_SENSE_PERIOD/ELECTRODES_SENSE_PERIOD_FH
	******************************************************************************/
	// YES (DECIMATION_FILTER_ON) or NO (DECIMATION_FILTER_OFF) optional decimation filter
	#define DECIMATION_FILTER DECIMATION_FILTER_OFF

		#if DECIMATION_FILTER
			// Modify: LPTMR new wake-up period - 4 ms
			// Value between 3 and 30
			#define ELECTRODES_SENSE_PERIOD_DF 4

			// Modify: Decimation filter filter factor addition for DCTracker
			#define DF_DCTRACKER_FILTER_FACTOR (3 - FH_DCTRACKER_FILTER_FACTOR)

			#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
				// Modify: Decimation filter step (minimal step 1)
				#define DECIMATION_STEP 10
			#elif (TS_RAW_DATA_CALCULATION == AVERAGING)
				// Modify: Decimation filter step (minimal step 1)
				#define DECIMATION_STEP 2
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
#if (OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_YES)
	#ifdef MODE_REACTION_TIME_90MS
		#define ELECTRODES_SENSE_PERIOD   30
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
			#define ELECTRODES_SENSE_PERIOD_PROXIMITY   100
		#endif
	#endif
	#ifdef MODE_CUSTOM
		#define ELECTRODES_SENSE_PERIOD   30
		#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
			#define ELECTRODES_SENSE_PERIOD_PROXIMITY   100
		#endif
	#endif
#else
	#ifdef MODE_REACTION_TIME_90MS
		#define ELECTRODES_SENSE_PERIOD   30
	#endif
	#ifdef MODE_CUSTOM
		#define ELECTRODES_SENSE_PERIOD   30
	#endif
#endif

/*******************************************************************************
* Modify: Define filter parameters.
*         Coefficients B0, B1 and A0 are dependent on filter cutoff frequency [Hz]
*         and electrode sensing period, see ELECTRODES_SENSE_PERIOD above.
*         Coefficient for filter are calculated with:
*         http://engineerjs.com/?sidebar=docs/iir.html
*
******************************************************************************/
#if (OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_YES)
	#ifdef MODE_REACTION_TIME_90MS
		#if DECIMATION_FILTER
			// Define electrodes IIR filter parameters
			// ELECTRODES_SENSE_PERIOD_DF  4
			#define FILTER_1                   0
			// Cutoff frequency in Hertz [Hz]
			#define FILTER_1_CUTOFF_FREQ_HZ    1
			// Coefficient B0
			#define FILTER_1_COEF_B0          0.012411061909675
			// Coefficient B1
			#define FILTER_1_COEF_B1          0.012411061909675
			// Coefficient A0
			#define FILTER_1_COEF_A0         -0.975177876180649

			// Define EGS IIR filter parameters
			#define FILTER_2                   1
			#define FILTER_2_CUTOFF_FREQ_HZ    1
			#define FILTER_2_COEF_B0           0.086364027013762
			#define FILTER_2_COEF_B1           0.086364027013762
			#define FILTER_2_COEF_A0           -0.827271945972476

			#if SLIDER_ENABLE
				// Slider LPF 1 Hz
				#define SLIDER_FILTER_1            2
				#define FILTER_3  				   SLIDER_FILTER_1
				#define FILTER_3_CUTOFF_FREQ_HZ    1
				#define FILTER_3_COEF_B0            0.012411061909675
				#define FILTER_3_COEF_B1            0.012411061909675
				#define FILTER_3_COEF_A0           -0.975177876180649

			#endif
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

			// Define EGS IIR filter parameters
			#define FILTER_2                   1
			#define FILTER_2_CUTOFF_FREQ_HZ    1
			#define FILTER_2_COEF_B0           0.086364027013762
			#define FILTER_2_COEF_B1           0.086364027013762
			#define FILTER_2_COEF_A0           -0.827271945972476

			#if SLIDER_ENABLE
				// Slider LPF 1 Hz
				#define SLIDER_FILTER_1            2
				#define FILTER_3  				   SLIDER_FILTER_1
				#define FILTER_3_CUTOFF_FREQ_HZ    1
				#define FILTER_3_COEF_B0           0.086364027013762
				#define FILTER_3_COEF_B1           0.086364027013762
				#define FILTER_3_COEF_A0           -0.827271945972476
			#endif

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

			// Define EGS IIR filter parameters
			#define FILTER_2                   1
			#define FILTER_2_CUTOFF_FREQ_HZ    1
			#define FILTER_2_COEF_B0           0.086364027013762
			#define FILTER_2_COEF_B1           0.086364027013762
			#define FILTER_2_COEF_A0           -0.827271945972476

			#if SLIDER_ENABLE
				// Slider LPF 1 Hz
				#define SLIDER_FILTER_1            2
				#define FILTER_3  				   SLIDER_FILTER_1
				#define FILTER_3_CUTOFF_FREQ_HZ    1
				#define FILTER_3_COEF_B0           0.086364027013762
				#define FILTER_3_COEF_B1           0.086364027013762
				#define FILTER_3_COEF_A0           -0.827271945972476
			#endif
		#endif

		// Number of used filters
		#define NUMBER_OF_FILTERS_USED     3

#else
		#ifdef MODE_REACTION_TIME_90MS
			#if DECIMATION_FILTER
				// Define electrodes IIR filter parameters
				// ELECTRODES_SENSE_PERIOD_DF   4
				#define FILTER_1                   0
				// Cutoff frequency in Hertz [Hz]
				#define FILTER_1_CUTOFF_FREQ_HZ    1
				// Coefficient B0
				#define FILTER_1_COEF_B0          0.012411061909675
				// Coefficient B1
				#define FILTER_1_COEF_B1          0.012411061909675
				// Coefficient A0
				#define FILTER_1_COEF_A0         -0.975177876180649

				#if SLIDER_ENABLE
					// Slider LPF 1 Hz
					#define SLIDER_FILTER_1            1
					#define FILTER_3  				   SLIDER_FILTER_1
					#define FILTER_3_CUTOFF_FREQ_HZ    1
					#define FILTER_3_COEF_B0            0.012411061909675
					#define FILTER_3_COEF_B1            0.012411061909675
					#define FILTER_3_COEF_A0           -0.975177876180649

				#endif
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

				#if SLIDER_ENABLE
					// Slider LPF 1 Hz
					#define SLIDER_FILTER_1            1
					#define FILTER_3  				   SLIDER_FILTER_1
					#define FILTER_3_CUTOFF_FREQ_HZ    1
					#define FILTER_3_COEF_B0           0.086364027013762
					#define FILTER_3_COEF_B1           0.086364027013762
					#define FILTER_3_COEF_A0           -0.827271945972476
				#endif
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

			#if SLIDER_ENABLE
				// Slider LPF 1 Hz
				#define SLIDER_FILTER_1            1
				#define FILTER_3  				   SLIDER_FILTER_1
				#define FILTER_3_CUTOFF_FREQ_HZ    1
				#define FILTER_3_COEF_B0           0.086364027013762
				#define FILTER_3_COEF_B1           0.086364027013762
				#define FILTER_3_COEF_A0           -0.827271945972476
			#endif
		#endif
		// Number of used filters
		#define NUMBER_OF_FILTERS_USED     2
#endif

/*******************************************************************************
* Modify: Electrode 0 to 6 common defines
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
	#ifdef WAKE_UP_ELECTRODE
		// DC tracker response, when in Idle mode
		// Value 5 equals 1s at 30ms sampling period
		#define ELEC_DCTRACKER_FILTER_FACTOR_IDLE     1
		// DC tracker response, when in Active mode
		// Value 5 equals 1s at 30ms sampling period
		#define ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (3 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	#else
		// DC tracker response, when in Idle mode
		// Value 5 equals 1s at 30ms sampling period
		#define ELEC_DCTRACKER_FILTER_FACTOR_IDLE     5
		// DC tracker response, when in Active mode
		// Value 5 equals 1s at 30ms sampling period
		#define ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	#endif
	// Electrode touch threshold relative to DC tracker value
	#define ELEC_TOUCH_THRESHOLD_DELTA            50
	// Electrode release threshold relative to DC tracker value
	#define ELEC_RELEASE_THRESHOLD_DELTA          40
#else
	#error Please select valid raw data calculation method in ts_cfg_general.h
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
* Modify: Electrode 2 defines
******************************************************************************/
#ifdef ELEC2
	#define ELEC2_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define ELEC2_LPFILTER_TYPE               FILTER_1
	#define ELEC2_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	#define ELEC2_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Electrode 3 defines
******************************************************************************/
#ifdef ELEC3
	#define ELEC3_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define ELEC3_LPFILTER_TYPE               FILTER_1
	#define ELEC3_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	#define ELEC3_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Electrode 4 defines
******************************************************************************/
#ifdef ELEC4
	#define ELEC4_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define ELEC4_LPFILTER_TYPE               FILTER_1
	#define ELEC4_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	#define ELEC4_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Electrode 5 defines
******************************************************************************/
#ifdef ELEC5
	#define ELEC5_DCTRACKER_FILTER_FACTOR     ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define ELEC5_LPFILTER_TYPE               FILTER_1
	#define ELEC5_TOUCH_THRESHOLD_DELTA       ELEC_TOUCH_THRESHOLD_DELTA
	#define ELEC5_RELEASE_THRESHOLD_DELTA     ELEC_RELEASE_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: Electrode 6 defines (EGS)
******************************************************************************/
#ifdef ELEC6
	#define ELEC6_DCTRACKER_FILTER_FACTOR     5
	#define ELEC6_LPFILTER_TYPE               FILTER_2

#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
	// Electrode touch threshold relative to DC tracker value
		#define ELEC6_TOUCH_THRESHOLD_DELTA       100
		// Electrode release threshold relative to DC tracker value
		#define ELEC6_RELEASE_THRESHOLD_DELTA     90
#else
	#if(TS_RAW_DATA_CALCULATION == AVERAGING)
			// Electrode touch threshold relative to DC tracker value
			#define ELEC6_TOUCH_THRESHOLD_DELTA       2
			// Electrode release threshold relative to DC tracker value
			#define ELEC6_RELEASE_THRESHOLD_DELTA     1
		#elif (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
			// Electrode touch threshold relative to DC tracker value
			#define ELEC6_TOUCH_THRESHOLD_DELTA       6
			// Electrode release threshold relative to DC tracker value
			#define ELEC6_RELEASE_THRESHOLD_DELTA     4
		#else
			#error Please select valid TS method in ts_cfg_general.h
		#endif
	#endif
#endif

/*******************************************************************************
* Modify: Virtual EGS Electrode touch threshold (for samples switching)
******************************************************************************/
#ifndef WAKE_UP_ELECTRODE
	#if(TS_RAW_DATA_CALCULATION == AVERAGING)
		// Virtual EGS touch threshold relative to DC tracker value
		#define VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA 3
	#elif (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
		// Virtual EGS touch threshold relative to DC tracker value
		#define VIRTUAL_EGS_TOUCH_THRESHOLD_DELTA 6
	#else
		#error Please select valid raw data calculation method in ts_cfg_general.h
	#endif
#endif

/*******************************************************************************
* Modify: Keypad backlight PWM dutycycle from 0 to 100
******************************************************************************/
#define BACKLIGHT_PWM_DUTYCYCLE    30

/*******************************************************************************
* Modify: Keypad backlight ON period in seconds
******************************************************************************/
#define BACKLIGHT_ON_PERIOD    3

/*******************************************************************************
* SLIDER DEFINES
******************************************************************************/
#if SLIDER_ENABLE

/*******************************************************************************
* Modify: Number of  slider segments
******************************************************************************/
// Number from 2 to 8 (5 is optimal)
#define NUMBER_OF_SLIDER_SEGMENTS 5

/*******************************************************************************
* Modify: Number of slider electrode cycles per single sample, slider segments
******************************************************************************/
#if TS_RAW_DATA_CALCULATION == AVERAGING
// IDLE MODE, when EGS/touch buttons/slider are not touched
// Value from 1 to 6 (when slider idle)
#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE 	4

// ACTIVE MODE, when EGS/touch buttons/slider are touched
// Number of samples, when module in active mode, max value 128
#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE	32

#elif TS_RAW_DATA_CALCULATION == OVERSAMPLING

/*******************************************************************************
* Modify: Define oversampling resolution, when using 12bit ADC:
			14bit resolution: 14
			15bit resolution: 15
			16bit resolution: 16
******************************************************************************/
	// Define resolution
	#define OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER   14

/*******************************************************************************
* Do not Modify: Predefined oversampling sample numbers
******************************************************************************/
	// IDLE MODE, when EGS/touch buttons/slider are not touched
	// Value from 1 to 4  to achieve MCU 70uA max average current consumption
	// In idle mode the sampling is basically 13 bits Oversampling
	#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_IDLE   4
	#define OVERSAMPLING_VALUE_SHIFT_SLIDER_IDLE  1

	// ACTIVE MODE, when EGS/touch buttons/slider are touched
	// Number of samples, when module in active mode, max value 256

	// 14bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER == 14)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE   16
		#define OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE  2
	#endif

	// 15bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER == 15)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE   64
		#define OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE  3
	#endif

	// 16bit resolution
	#if (OVERSAMPLING_RESULT_RESOLUTION_BITS_SLIDER == 16)
		#define NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE   256
		#define OVERSAMPLING_VALUE_SHIFT_SLIDER_ACTIVE  4
	#endif
#endif

/*******************************************************************************
* Modify: Slider Electrode 0 to 1 common defines
******************************************************************************/
#if(TS_RAW_DATA_CALCULATION == AVERAGING)
	// DC tracker response, when in Idle mode
	// Value 5 equals 0.5s at 30ms sampling period
	#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE   5
	// DC tracker response, when in Active mode
	// Value 5 equals 0.5s at 30ms sampling period
	#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	// Electrode touch threshold relative to DC tracker value
	#define SLIDER_ELEC_TOUCH_THRESHOLD_DELTA       17
#elif (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	#ifdef WAKE_UP_ELECTRODE
		// DC tracker response, when in Idle mode
		// Value 5 equals 1s at 30ms sampling period
		#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE     1
		// DC tracker response, when in Active mode
		// Value 5 equals 1s at 30ms sampling period
		#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (3 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	#else
		// DC tracker response, when in Idle mode
		// Value 5 equals 1s at 30ms sampling period
		#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_IDLE     5
		// DC tracker response, when in Active mode
		// Value 5 equals 1s at 30ms sampling period
		#define SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE   (5 + DF_DCTRACKER_FILTER_FACTOR + FH_DCTRACKER_FILTER_FACTOR)
	#endif
	// Electrode touch threshold relative to DC tracker value
	#define SLIDER_ELEC_TOUCH_THRESHOLD_DELTA            50
#else
	#error Please select valid raw data calculation method in ts_cfg_general.h
#endif

/*******************************************************************************
* Modify: APP Slider electrode 0 defines
******************************************************************************/
#ifdef SLIDER_ELEC0
	// DC tracker response, the highest number, the slower response.
	// Select values from 1 to 8
	#define SLIDER_ELEC0_DCTRACKER_FILTER_FACTOR     SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	// IIR1 LP filter selection
	#define SLIDER_ELEC0_LPFILTER_TYPE               SLIDER_FILTER_1
	// Slider electrode touch threshold relative to DC tracker value
	#define SLIDER_ELEC0_TOUCH_THRESHOLD_DELTA       SLIDER_ELEC_TOUCH_THRESHOLD_DELTA
#endif

/*******************************************************************************
* Modify: APP Slider electrode 1 defines
******************************************************************************/
#ifdef  SLIDER_ELEC1
	#define SLIDER_ELEC1_DCTRACKER_FILTER_FACTOR     SLIDER_ELEC_DCTRACKER_FILTER_FACTOR_ACTIVE
	#define SLIDER_ELEC1_LPFILTER_TYPE               SLIDER_FILTER_1
	#define SLIDER_ELEC1_TOUCH_THRESHOLD_DELTA       SLIDER_ELEC_TOUCH_THRESHOLD_DELTA
#endif

/*******************************************************************************
 * Modify ONLY after tuning in Freemaster: DEFAULT Slider defines after power-up/reset
 ******************************************************************************/
#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Number of points above and below each threshold, where the slider is insensitive to change
	// Number from 1 to 30
	#define NUMBER_OF_HYSTERESIS_BLIND_POINTS 6

	// Difference data
	// Maximal amplitude of the difference data observed in freemaster
	#define NUMBER_OF_MAXIMAL_OBSERVED_DIFFERENCE 3110
	// Minimal amplitude of the difference data observed in freemaster
	#define NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE 2920

	// Addition data
	// Threshold of addition data
	#define SLIDER_ADDITION_DATA_THRESHOLD_DEFAULT 2870
#elif (TS_RAW_DATA_CALCULATION == AVERAGING)
	// Number of points above and below each threshold, where the slider is insensitive to change
	// Number from 1 to 6
	#define NUMBER_OF_HYSTERESIS_BLIND_POINTS 2

	// Difference data
	// Maximal amplitude of the difference data observed in freemaster
	#define NUMBER_OF_MAXIMAL_OBSERVED_DIFFERENCE 1030
	// Minimal amplitude of the difference data observed in freemaster
	#define NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE 980

	// Addition data
	// Threshold of addition data
	#define SLIDER_ADDITION_DATA_THRESHOLD_DEFAULT 966
#else
	#error Please select valid raw data calculation method in ts_cfg_general.h
#endif

/*******************************************************************************
* Do not modify! Default Slider SW defines after power-up/reset
******************************************************************************/
#if (TS_RAW_DATA_CALCULATION == OVERSAMPLING)
	// Default offset for difference and addition data
	#define SLIDER_DIFFADD_DATA_OFFSET 3000
#elif (TS_RAW_DATA_CALCULATION == AVERAGING)
	// Default offset for difference and addition data
	#define SLIDER_DIFFADD_DATA_OFFSET 1000
#else
	#error Please select valid raw data calculation method in ts_cfg_general.h
#endif

// Slider difference data range calculation
#define SLIDER_RANGE (NUMBER_OF_MAXIMAL_OBSERVED_DIFFERENCE - NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE)

#if (NUMBER_OF_SLIDER_SEGMENTS > 0 && NUMBER_OF_SLIDER_SEGMENTS < 9 )
	// Slider difference data step between thresholds
	#define SLIDER_STEP (SLIDER_RANGE / NUMBER_OF_SLIDER_SEGMENTS)
	// Number of slider difference data thresholds
	#define NUMBER_OF_SLIDER_THRESHOLDS (NUMBER_OF_SLIDER_SEGMENTS - 1)

	// Define to alloc certain memory for sliderDiffThreshold array - Freemaster
	#define NUMBER_OF_THRESHOLDS_FREEMASTER 7
#endif

/*******************************************************************************
* Modify ONLY after tuning in Freemaster: Slider difference data thresholds defines
******************************************************************************/

// 1 to automatically generate difference thresholds, 0 to manually set difference thresholds
#define AUTO_DIFF_THRESHOLDS_GEN 1

#if  (AUTO_DIFF_THRESHOLDS_GEN == 0)

	// Manual setting of difference thresholds - maximum 7 thresholds -> 8 segments
	// Modify ONLY the line with SLIDER_DIFF_THRESHOLD defines
	// for example: #define SLIDER_DIFF_THRESHOLD1 1010

	#if (NUMBER_OF_SLIDER_SEGMENTS > 1)
	#define SLIDER_DIFF_THRESHOLD0 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + SLIDER_STEP)
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 2)
	#define SLIDER_DIFF_THRESHOLD1 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 2))
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 3)
	#define SLIDER_DIFF_THRESHOLD2 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 3))
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 4)
	#define SLIDER_DIFF_THRESHOLD3 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 4))
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 5)
	#define SLIDER_DIFF_THRESHOLD4 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 5))
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 6)
	#define SLIDER_DIFF_THRESHOLD5 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 6))
	#endif

	#if (NUMBER_OF_SLIDER_SEGMENTS > 7)
	#define SLIDER_DIFF_THRESHOLD6 (NUMBER_OF_MINIMAL_OBSERVED_DIFFERENCE + (SLIDER_STEP * 7))
	#endif

#endif

#endif
#endif

#endif /* CFG_6PAD_SLIDER_6PAD_APP_H_ */
