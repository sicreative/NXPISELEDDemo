/*
 * @file     2pad_hw.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Hardware configuration file for 2pad EVB demo
*
 */

#ifndef CFG_2PAD_EVB_2PAD_HW_H_
#define CFG_2PAD_EVB_2PAD_HW_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg_general.h>
#include "S32K144.h"

#if (REFERENCE_DESIGN_BOARD == S32K144_2PAD_EVB)
/*******************************************************************************
* Do not modify!: Define number of electrodes
******************************************************************************/
// Number of touch button electrodes from 1 to 2
#define NUMBER_OF_TOUCH_ELECTRODES   2
// YES (WAKE_ELEC_YES) or NO (WAKE_ELEC_NO) optional wake-up electrode
// There is no EGS available on EVB
#define OPTIONAL_WAKE_UP_ELECTRODE   WAKE_ELEC_NO
#define WAKE_UP_ELECTRODE_PROXIMITY   WAKE_UP_ELECTRODE_PROXIMITY_NO

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
	#define ELEC0_ADC             ADC0      // Modify: ADC module
	#define ELEC0_ADC_CHANNEL     8         // Modify: Cext ADC channel
	#define ELEC0_PORT            PORTC     // Modify: Electrode and Cext PORT
	#define ELEC0_GPIO            PTC       // Modify: Electrode and Cext GPIO
	#define ELEC0_ELEC_GPIO_PIN   2         // Modify: Electrode GPIO pin
	#define ELEC0_CEXT_GPIO_PIN   0         // Modify: Cext GPIO pin
	#define ELEC0_PORT_MASK       (1 << ELEC0_ELEC_GPIO_PIN) | (1 << ELEC0_CEXT_GPIO_PIN)
	#define ELEC0_TRGMUX		  30		// Modify: 30 for ADC0, 32 for ADC1
	#define ELEC0_DMAMUX		  42		// Modify: 42 for ADC0, 43 for ADC1
#endif

/*******************************************************************************
* Modify: Electrode 1 defines
******************************************************************************/
#if (NUMBER_OF_ELECTRODES > 1)
	#define ELEC1
	#define ELEC1_ADC             ADC0
	#define ELEC1_ADC_CHANNEL     11
	#define ELEC1_PORT            PORTC
	#define ELEC1_GPIO            PTC
	#define ELEC1_ELEC_GPIO_PIN   1
	#define ELEC1_CEXT_GPIO_PIN   3
	#define ELEC1_PORT_MASK       (1 << ELEC1_ELEC_GPIO_PIN) | (1 << ELEC1_CEXT_GPIO_PIN)
	#define ELEC1_TRGMUX		  30
	#define ELEC1_DMAMUX		  42
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

#endif

#endif /* CFG_2PAD_EVB_2PAD_HW_H_ */
