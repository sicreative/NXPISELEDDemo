/*
 * @file     6pad_hw.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Hardware configuration file for 6pad keypad with slider reference design
*
 */

#ifndef CFG_6PAD_SLIDER_6PAD_HW_H_
#define CFG_6PAD_SLIDER_6PAD_HW_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg_general.h>
#include "S32K144.h"

#if (REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER)
/*******************************************************************************
* Modify: Define number of electrodes
******************************************************************************/
// Number of touch button electrodes from 1 to 6
#define NUMBER_OF_TOUCH_ELECTRODES   6
// YES (WAKE_ELEC_YES) or NO (WAKE_ELEC_NO) optional wake-up electrode
// If YES, an additional electrode defined, assigned the highest number
#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_YES

/*******************************************************************************
* Do not modify !
******************************************************************************/
#if (OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_YES)
	// Add an extra electrode for wake-up function
	#define NUMBER_OF_ELECTRODES   (NUMBER_OF_TOUCH_ELECTRODES + 1)
	// Wake-up electrode number assignment
	#define WAKE_UP_ELECTRODE      (NUMBER_OF_ELECTRODES - 1)
#endif

#if (OPTIONAL_WAKE_UP_ELECTRODE == WAKE_ELEC_NO)
	// Application uses touch electrodes only
	#define NUMBER_OF_ELECTRODES   NUMBER_OF_TOUCH_ELECTRODES
#endif

/*******************************************************************************
* Modify: Electrode 0 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 0)
	#define ELEC0
	#define ELEC0_ADC             ADC1      // Modify: ADC module
	#define ELEC0_ADC_CHANNEL     1         // Modify: Cext ADC channel
	#define ELEC0_PORT            PORTA     // Modify: Electrode and Cext PORT
	#define ELEC0_GPIO            PTA       // Modify: Electrode and Cext GPIO
	#define ELEC0_ELEC_GPIO_PIN   2        	// Modify: Electrode GPIO pin
	#define ELEC0_CEXT_GPIO_PIN   3		   	// Modify: Cext GPIO pin
	#define ELEC0_PORT_MASK       (1 << ELEC0_ELEC_GPIO_PIN) | (1 << ELEC0_CEXT_GPIO_PIN)
	#define ELEC0_TRGMUX		  32		// Modify: 30 for ADC0, 32 for ADC1
	#define ELEC0_DMAMUX		  43		// Modify: 42 for ADC0, 43 for ADC1
#endif

/*******************************************************************************
* Modify: Electrode 1 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 1)
	#define ELEC1
	#define ELEC1_ADC             ADC0
	#define ELEC1_ADC_CHANNEL     5
	#define ELEC1_PORT            PORTB
	#define ELEC1_GPIO            PTB
	#define ELEC1_ELEC_GPIO_PIN   0
	#define ELEC1_CEXT_GPIO_PIN   1
	#define ELEC1_PORT_MASK       (1 << ELEC1_ELEC_GPIO_PIN) | (1 << ELEC1_CEXT_GPIO_PIN)
	#define ELEC1_TRGMUX		  30
	#define ELEC1_DMAMUX		  42

#endif

/*******************************************************************************
* Modify: Electrode 2 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 2)
	#define ELEC2
	#define ELEC2_ADC             ADC0
	#define ELEC2_ADC_CHANNEL     3
	#define ELEC2_PORT            PORTA
	#define ELEC2_GPIO            PTA
	#define ELEC2_ELEC_GPIO_PIN   6
	#define ELEC2_CEXT_GPIO_PIN   7
	#define ELEC2_PORT_MASK       (1 << ELEC2_ELEC_GPIO_PIN) | (1 << ELEC2_CEXT_GPIO_PIN)
	#define ELEC2_TRGMUX		  30
	#define ELEC2_DMAMUX		  42

#endif

/*******************************************************************************
* Modify: Electrode 3 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 3)
	#define ELEC3
	#define ELEC3_ADC             ADC0
	#define ELEC3_ADC_CHANNEL     7
	#define ELEC3_PORT            PORTB
	#define ELEC3_GPIO            PTB
	#define ELEC3_ELEC_GPIO_PIN   5
	#define ELEC3_CEXT_GPIO_PIN   3
	#define ELEC3_PORT_MASK       (1 << ELEC3_ELEC_GPIO_PIN) | (1 << ELEC3_CEXT_GPIO_PIN)
	#define ELEC3_TRGMUX		  30
	#define ELEC3_DMAMUX		  42

#endif

/*******************************************************************************
* Modify: Electrode 4 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 4)
	#define ELEC4
	#define ELEC4_ADC             ADC0
	#define ELEC4_ADC_CHANNEL     15
	#define ELEC4_PORT            PORTC
	#define ELEC4_GPIO            PTC
	#define ELEC4_ELEC_GPIO_PIN   16
	#define ELEC4_CEXT_GPIO_PIN   17
	#define ELEC4_PORT_MASK       (1 << ELEC4_ELEC_GPIO_PIN) | (1 << ELEC4_CEXT_GPIO_PIN)
	#define ELEC4_TRGMUX		  30
	#define ELEC4_DMAMUX		  42
#endif

/*******************************************************************************
* Modify: Electrode 5 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 5)
	#define ELEC5
	#define ELEC5_ADC             ADC1
	#define ELEC5_ADC_CHANNEL     13
	#define ELEC5_PORT            PORTA
	#define ELEC5_GPIO            PTA
	#define ELEC5_ELEC_GPIO_PIN   15
	#define ELEC5_CEXT_GPIO_PIN   16
	#define ELEC5_PORT_MASK       (1 << ELEC5_ELEC_GPIO_PIN) | (1 << ELEC5_CEXT_GPIO_PIN)
	#define ELEC5_TRGMUX		  32
	#define ELEC5_DMAMUX		  43
#endif

/*******************************************************************************
* Modify: Electrode 6 defines - wake up EGS
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 6)
	#define ELEC6
	#define ELEC6_ADC             ADC0
	#define ELEC6_ADC_CHANNEL     1
	#define ELEC6_PORT            PORTA
	#define ELEC6_GPIO            PTA
	#define ELEC6_ELEC_GPIO_PIN   0
	#define ELEC6_CEXT_GPIO_PIN   1
	#define ELEC6_PORT_MASK       (1 << ELEC6_ELEC_GPIO_PIN) | (1 << ELEC6_CEXT_GPIO_PIN)
	#define ELEC6_TRGMUX		  30
	#define ELEC6_DMAMUX		  42
#endif


/*******************************************************************************
* Modify: Guarding option
*  (GUARD_OFF) no guarding
*  (GUARD_ON) guarding driven by user configurable pin
******************************************************************************/
#define GUARD GUARD_OFF

/*******************************************************************************
* Modify: Guard 0 defines
******************************************************************************/
#if (GUARD)
	#define GUARD_PORT           PORTE     // Modify: Guard PORT
	#define GUARD_GPIO           PTE       // Modify: Guard GPIO
	#define GUARD_GPIO_PIN       4         // Modify: Guard GPIO pin
	#define GUARD_PORT_MASK      (1 << GUARD_GPIO_PIN) // Modify: Guard PORT mask
#endif

/*******************************************************************************
* SLIDER HW DEFINES
******************************************************************************/
/*******************************************************************************
* Modify: Enable or disable slider electrodes
******************************************************************************/
// YES (SLIDER_YES) or NO (SLIDER_NO) optional Slider electrodes
#define SLIDER_ENABLE SLIDER_YES

/*******************************************************************************
* Do not Modify! Default settings of: Number of ADC modules, number of slider electrodes
******************************************************************************/
#if SLIDER_ENABLE
	// Number of ADC modules used for conversion at one time (also depends on number of ADC modules available on the MCU)
	// 1 (ADC0 OR ADC1 module separately) or 2 (both ADC0 AND ADC1 simultaneously)
	#define NUMBER_OF_USED_ADC_MODULES 1

	#if (NUMBER_OF_USED_ADC_MODULES == 2 || NUMBER_OF_USED_ADC_MODULES == 1)
		// YES (SLIDER_NOISE_CANCELING_ON) or NO (SLIDER_NOISE_CANCELING_OFF)
		// Works only with averaging, does not work with oversampling
		#define SLIDER_NOISE_CANCELING SLIDER_NOISE_CANCELING_OFF
	#endif

	// Default number of slider electrodes
	// SW written for one slider only, consisting of 2 wedge electrodes
	#define NUMBER_OF_SLIDER_ELECTRODES  2
	#define SLIDER_ELEC0				 0
	#define SLIDER_ELEC1				 1
#else
	// Slider disabled - zero number of slider electrodes
	#define NUMBER_OF_SLIDER_ELECTRODES  0
#endif

#if SLIDER_ENABLE
/*******************************************************************************
* Modify: HW Slider Electrode 0 defines
******************************************************************************/
	#define SLIDER_ELEC0_ADC             ADC0		// Modify: ADC module
	#define SLIDER_ELEC0_ADC_CHANNEL     6			// Modify: Cext ADC channel
	#define SLIDER_ELEC0_PORT            PORTB		// Modify: Electrode and Cext PORT
	#define SLIDER_ELEC0_GPIO            PTB		// Modify: Electrode and Cext GPIO
	#define SLIDER_ELEC0_ELEC_GPIO_PIN   4			// Modify: Electrode GPIO pin
	#define SLIDER_ELEC0_CEXT_GPIO_PIN   2			// Modify: Cext GPIO pin
	#define SLIDER_ELEC0_PORT_MASK       (1 << SLIDER_ELEC0_ELEC_GPIO_PIN) | (1 << SLIDER_ELEC0_CEXT_GPIO_PIN)
	#define SLIDER_ELEC0_TRGMUX		  	 30
	#define SLIDER_ELEC0_DMAMUX		  	 42

/*******************************************************************************
* Modify: HW Slider Electrode 1 defines
******************************************************************************/
	#define SLIDER_ELEC1_ADC             ADC1
	#define SLIDER_ELEC1_ADC_CHANNEL     8
	#define SLIDER_ELEC1_PORT            PORTB
	#define SLIDER_ELEC1_GPIO            PTB
	#define SLIDER_ELEC1_ELEC_GPIO_PIN   12
	#define SLIDER_ELEC1_CEXT_GPIO_PIN   13
	#define SLIDER_ELEC1_PORT_MASK       (1 << SLIDER_ELEC1_ELEC_GPIO_PIN) | (1 << SLIDER_ELEC1_CEXT_GPIO_PIN)
	#define SLIDER_ELEC1_TRGMUX		  	 32
	#define SLIDER_ELEC1_DMAMUX		  	 43
#endif

#endif

#endif /* CFG_6PAD_SLIDER_6PAD_HW_H_ */
