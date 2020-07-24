/*
 * @file     ts_cfg_do_not_modify.h
*
* @version  1.0.0.0
*
* @date     February-2019
*
* @brief    Do not modify this cfg file
*
 */

#ifndef CFG_TS_CFG_DO_NOT_MODIFY_H_
#define CFG_TS_CFG_DO_NOT_MODIFY_H_

/*******************************************************************************
* Do not modify ! Demo hardware list
******************************************************************************/
#define S32K144_2PAD_EVB 			1
#define S32K144_7PAD_KEYPAD 		2
#define S32K144_6PAD_KEYPAD_SLIDER 	3

/*******************************************************************************
* Do not modify ! TS method list
******************************************************************************/
#define AVERAGING 			1
#define OVERSAMPLING 		2

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define LPM_ENABLE       1
#define LPM_DISABLE      0
#define ELECTRODE_ADC_CHANNEL_OFFSET 16

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define WAKE_ELEC_YES    1
#define WAKE_ELEC_NO     0

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define SLIDER_YES    	 1
#define SLIDER_NO        0
#define SLIDER_NOISE_CANCELING_ON    	1
#define SLIDER_NOISE_CANCELING_OFF      0

/*******************************************************************************
* Do not modify! Jittering defines
******************************************************************************/
#define JITTERING_ON   1
#define JITTERING_OFF   0

/*******************************************************************************
* Do not Modify! Frequency hopping defines
******************************************************************************/
#define FREQUENCY_HOPPING_ON   1
#define FREQUENCY_HOPPING_OFF   0

/*******************************************************************************
* Do not modify! Decimation filter defines
******************************************************************************/
#define DECIMATION_FILTER_ON   1
#define DECIMATION_FILTER_OFF   0

/*******************************************************************************
* Do not modify! Electrode data acquisition method
******************************************************************************/
#define CPU_ELECTRODE_DATA_ACQUISITION   0
#define DMA_ELECTRODE_DATA_ACQUISITION   1

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define GUARD_OFF     0
#define GUARD_ON     1

/*******************************************************************************
* Do not modify !
******************************************************************************/
#define WAKE_UP_ELECTRODE_PROXIMITY_NO 0
#define WAKE_UP_ELECTRODE_PROXIMITY_YES 1

#endif /* CFG_TS_CFG_DO_NOT_MODIFY_H_ */
