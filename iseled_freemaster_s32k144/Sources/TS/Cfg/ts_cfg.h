/*
 * @file     ts_cfg.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Touch sense configuration file which gathers and includes all other cfg files into TS application source codes
*
 */

#ifndef CFG_TS_CFG_H_
#define CFG_TS_CFG_H_
/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg_general.h>
#include "S32K144.h"

#if (REFERENCE_DESIGN_BOARD == S32K144_2PAD_EVB)
#include <TS/Cfg/2pad_EVB/2pad_hw.h>
#include <TS/Cfg/2pad_EVB/2pad_app.h>
#elif (REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER)
#include "6pad_hw.h"
#include "6pad_app.h"
#elif (REFERENCE_DESIGN_BOARD == S32K144_7PAD_KEYPAD)
#include "7pad_hw.h"
#include "7pad_app.h"
#else
#error Please select valid reference design board in ts_cfg_general.h
#endif

/*******************************************************************************
* Modify: Low power mode enable
*         If low power mode is enabled (LPM_ENABLE), the application debug and
*         FreeMASTER data visualization are disabled.
*         If low power mode is disabled (LPM_DISABLE), the application debug and
*         FreeMASTER data visualization are enabled.
******************************************************************************/
#define LOW_POWER_MODE   LPM_DISABLE

/*******************************************************************************
* Modify: Assembly optimization for CPU to avoid dependency on (-O3) compilation optimization (1-ON, 0-OFF)
* 		  Modify if lower than -O3 compiler optimization is used (for example -O0, -O1, -O2)
* Note:	  This assembly optimization for CPU ensures (determines) a constant minimal number of instructions between
* 		  the start of ADC conversion command and switching the pins to input state command (charge redistribution).
* 		  At all levels of compiler optimization (-O0, -O1, -O2. -O3) this assembly optimization for CPU prevents
* 		  the compiler to put any unnecessary instructions in this crucial period between the two commands
* 		  and thus shift the ADC conversion moment before/long time after the equivalent voltage is set.
* WARNING:The -O3 compiler optimization itself has almost the same efficiency as this assembly optimization,
* 		  however sometimes even on -O3 there might be a delay between the ADC conversion command and switching the pins to input state command
* 		  Therefore, the best solution is to keep -O3 compiler optimization and put the assembly optimization ON (1)
******************************************************************************/
#define TS_ASM_OPTIMIZE     0

/*******************************************************************************
* Modify: Define ADC software trigger sample time for all electrodes
******************************************************************************/
#if TS_ELECTRODE_DATA_ACQUISITION == CPU_ELECTRODE_DATA_ACQUISITION
	// CPU used for electrodes data gathering
	// Value from 14 to 255
	#define ADC_SWTRIGGER_SAMPLE_TIME 14
#else
	// DMA used for electrodes data gathering
	// Value from 30 to 255
	#define ADC_SWTRIGGER_SAMPLE_TIME 35
#endif

/*******************************************************************************
* Do Not Modify!: Define ADC hardware trigger sample time for slider electrodes
* when scanning slider electrodes simultaneously
******************************************************************************/
#if SLIDER_ENABLE
	#if (NUMBER_OF_USED_ADC_MODULES == 2)
		// ADC sample time adjustment for hardware trigger of the slider electrodes ADC conversion
		#if TS_ELECTRODE_DATA_ACQUISITION == CPU_ELECTRODE_DATA_ACQUISITION
			// CPU used for electrodes data gathering
			// Value from 5 to 255
			#define ADC_HWTRIGGER_SLIDER_SAMPLE_TIME 8
		#else
			// DMA used for electrodes data gathering
			// Value from 25 to 255
			#define ADC_HWTRIGGER_SLIDER_SAMPLE_TIME 28
		#endif
	#endif
#endif
/*******************************************************************************
* Modify: If needed, configure debug pins. Uncomment required option.
******************************************************************************/
// Debug electrode sensing cycle
//#define DEBUG_ELECTRODE_SENSE
// Debug application algorithm
//#define DEBUG_ALGORITHM

// Defined?
#ifdef DEBUG_ELECTRODE_SENSE
	// Assign PORT
	#define DES_PORT  PORTE
	// Assign GPIO
	#define DES_GPIO  PTE
	// Assign pin
	#define DES_PIN   10

#endif

// Defined?
#ifdef DEBUG_ALGORITHM
	// Assign PORT
	#define DA_PORT  PORTE
	// Assign GPIO
	#define DA_GPIO  PTE
	// Assign pin
	#define DA_PIN   11
#endif

#define MORE_THAN_ONE_SENSOR_TOUCHED 1
#endif /* CFG_TS_CFG_H_ */
