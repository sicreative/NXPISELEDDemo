/*
 * @file     ts_cfg_general.h
*
* @version  1.0.0.0
*
* @date     January-2019
*
* @brief    Touch sense main configuration file
*
 */

#ifndef CFG_TS_CFG_GENERAL_H_
#define CFG_TS_CFG_GENERAL_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg_do_not_modify.h>

/*******************************************************************************
* Modify: Reference design Hardware Selection
* 	S32K144_2PAD_EVB 			- Two pads demo on S32K144 EVB
* 	S32K144_7PAD_KEYPAD 		- Reference design board - 7 pad keypad
* 	S32K144_6PAD_KEYPAD_SLIDER 	- Reference design board - 6 pad keypad with slider
******************************************************************************/
// Select the available type of hardware for TS application (S32K144_2PAD_EVB, S32K144_7PAD_KEYPAD or S32K144_6PAD_KEYPAD_SLIDER)
#define REFERENCE_DESIGN_BOARD 	S32K144_2PAD_EVB

/*******************************************************************************
* Modify: Define the main formula of electrode raw data calculation
* 	AVERAGING		- Electrode sample is calculated as an average of multiple ADC scans (electrode sensing cycles)
* 	OVERSAMPLING	- Electrode sample is calculated as a sum of multiple ADC scans (electrode sensing cycles)
******************************************************************************/
// Define the main raw data calculation formula (AVERAGING OR OVERSAMPLING)
#define TS_RAW_DATA_CALCULATION  AVERAGING

/*******************************************************************************
* Modify: Electrode data acquisition method
*  If CPU_ELECTRODE_DATA_ACQUISITION chosen, the CPU drives the electrodes and gathers data of all electrodes
*  If DMA_ELECTRODE_DATA_ACQUISITION chosen, the DMA drives the electrodes and gathers data of all electrodes
******************************************************************************/
#define TS_ELECTRODE_DATA_ACQUISITION     CPU_ELECTRODE_DATA_ACQUISITION


#endif /* CFG_TS_CFG_GENERAL_H_ */
