/****************************************************************************//*!
*
* @file     flextimer.h
*
* @version  1.0.0.0
*
* @date     July-2017
*
* @brief    FlexTimer routines header file
*
*******************************************************************************/

/*******************************************************************************
* Includes
*******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include <TS/ets.h>

/*******************************************************************************
* Defines
******************************************************************************/
#if (DECIMATION_FILTER && WAKE_UP_ELECTRODE_PROXIMITY)
#define KEYPAD_BACKLIGHT_ON_PERIOD    ((((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD_DF)*1/3) + (((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD_PROXIMITY)*2/3))
#elif DECIMATION_FILTER
#define KEYPAD_BACKLIGHT_ON_PERIOD    ((((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD_DF)*1/3) + (((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD)*2/3))
#elif WAKE_UP_ELECTRODE_PROXIMITY
#define KEYPAD_BACKLIGHT_ON_PERIOD    ((((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD)*1/3) + (((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD_PROXIMITY)*2/3))
#else
#define KEYPAD_BACKLIGHT_ON_PERIOD    ((BACKLIGHT_ON_PERIOD * 1000) / ELECTRODES_SENSE_PERIOD)
#endif
/*******************************************************************************
* Function prototypes
******************************************************************************/
void FTM2_Init(void);
void LoadBacklightPWMDutyCycle(uint16_t PWMDutyCycle);
