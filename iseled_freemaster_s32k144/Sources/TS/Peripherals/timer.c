/****************************************************************************//*!
*
* @file     timer.c
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    Timer routines
*
*******************************************************************************/

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include <TS/ets.h>
//#include <TS/FreeMASTER/S32xx/freemaster.h>
#include <TS/main.h>
#include <TS/Peripherals/adc.h>
#include <TS/Peripherals/flextimer.h>
#include <TS/Peripherals/gpio_inline_fcn1.h>
#include <TS/Peripherals/tcd.h>
#include <TS/Peripherals/timer.h>
#include <TS/slider.h>
#include "S32K144.h"
#include "tcd.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
// All electrodes status
extern tElecStatus  electrodesStatus, sliderElectrodesStatus;
extern tElecStruct elecStruct[NUMBER_OF_ELECTRODES];

#if SLIDER_ENABLE
extern int32_t sliderAdcDataElectrodeDischargeRaw[NUMBER_OF_SLIDER_ELECTRODES];
extern tElecStruct sliderElecStruct[NUMBER_OF_SLIDER_ELECTRODES];
#endif
#ifdef WAKE_UP_ELECTRODE
// Electrode touch
extern uint8_t  electrodeTouch[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
extern uint8_t  virtualEGSelectrodeTouch;
#endif

// Electrode qualified reports any electrode touch
extern uint8_t electrodeTouchQualifiedReport;
#if SLIDER_ENABLE
extern uint8_t sliderElectrodeTouchQualifiedReport;
#endif

// Low power mode
extern uint8_t  lowPowerModeCtrl;

// Backlight PWM duty cycle from 0 to 100
extern uint16_t  backlightPWMDutyCycle;


extern bool adc_busy;

// Backlight ON counter
uint32_t backlightCounter;

// Jittering
extern int32_t adcDataElectrodeDischargeRaw[NUMBER_OF_ELECTRODES][NUMBER_OF_HOPPING_FREQUENCIES];
#if SLIDER_ENABLE
extern int32_t sliderAdcDataElectrodeDischargeRaw[NUMBER_OF_SLIDER_ELECTRODES];
#endif
int16_t jitterRead;

// Frequency hopping
extern uint8_t frequencyHoppingActivation, frequencyID;

// DMA control flags
extern volatile uint32_t ready_dma_flag, start_dma_flag;
/*****************************************************************************
 *
 * Function: void LPTMR0_Init(uint32_t timeout)
 *
 * Description: Init LPTMR
 *
 *****************************************************************************/
void LPTMR0_Init(uint32_t timeout)
{
	// Disable LPTMR
	LPTMR0->CSR = 0x00000000;
	// Bypass prescaler
	LPTMR0->PSR = 0x00000005;
	// Set timeout period
	LPTMR0->CMR = timeout;
	// Clear TCF, enable interrupt, count rising edges, reset when TCF is set, counter mode, enable timer
	LPTMR0->CSR = 0x000000C1;
}

/*****************************************************************************
 *
 * Function: void LPTMR0_CMRupdate(uint32_t timeout)
 *
 * Description: Sets new timeout period of LPTMR
 *
 *****************************************************************************/
void LPTMR0_CMRupdate(uint32_t timeout)
{
	// Set new LPTMR timeout period
	LPTMR0->CMR = timeout;
}

/*****************************************************************************
 *
 * Function: void LPTMR0_IRQHandler(void)
 *
 * Description: LPTMR interrupt
 *
 *****************************************************************************/
void LPTMR0_IRQHandler(void)
{
#if(DECIMATION_FILTER != 1)
	// Clear TCF LPTMR
	//REG_RMW32(&LPTMR0->CSR, 1 << 7, 1 << 7);
	LPTMR0->CSR |= 1 << 7;
#endif

	// Electrodes self-trim done?
#if SLIDER_ENABLE
	if(electrodesStatus.bit.selfTrimDone == YES || sliderElectrodesStatus.bit.selfTrimDone == YES)
#else
		if(electrodesStatus.bit.selfTrimDone == YES)
#endif
		{

#ifdef WAKE_UP_ELECTRODE
			// Wake-up electrode touched?
			if (electrodeTouch[WAKE_UP_ELECTRODE][frequencyID] == YES)
			{
#if JITTERING
#if (JITTERING_OPTION == 1)
			#if ((REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER) && (NUMBER_OF_ELECTRODES < 1))
			// Jitter sample rate
			Jitter(sliderAdcDataElectrodeDischargeRaw[0]);
			#else
			// Jitter sample rate
			Jitter(adcDataElectrodeDischargeRaw[0][frequencyID]);
			#endif
#endif
#endif
				// Sense all electrodes
				ElectrodeWakeAndTouchElecSense();
			}
			else
			{
#if JITTERING
#if (JITTERING_OPTION == 1)
			#if ((REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER) && (NUMBER_OF_ELECTRODES < 1))
			Jitter(sliderAdcDataElectrodeDischargeRaw[0]);
			#else
			Jitter(adcDataElectrodeDischargeRaw[WAKE_UP_ELECTRODE][frequencyID]);
			#endif
#endif
#endif
				// Sense electrode wake up touch event
				ElectrodeWakeElecSense();
			}

#if FREQUENCY_HOPPING
			// Change the frequency ID if EGS touched
			FrequencyHop();

			if(frequencyHoppingActivation == 1)
			{
				//If LPIT disabled (after power-up/wakeup), enable LPIT
				if((LPIT0->TMR[0].TCTRL & LPIT_TMR_TCTRL_T_EN_MASK) == 0)
				{
					//Enable LPIT
					LPIT_Enable();
				}

				// Wait for LPIT to timeout
				while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {}

				// Clear LPIT0 timer flag 0
				LPIT0->MSR = LPIT_MSR_TIF0_MASK;

				// Disable LPIT
				LPIT_Disable();

				// Sense all electrodes again on second period/frequency
				ElectrodeWakeAndTouchElecSense();

				// Change the frequency ID
				FrequencyHop();
			}

#endif
			// Any electrode touched?
#if SLIDER_ENABLE
			if (electrodeTouchQualifiedReport == 1 || sliderElectrodeTouchQualifiedReport == 1)
#else
				if (electrodeTouchQualifiedReport == 1)
#endif
				{
					// MCU in RUN mode only
					lowPowerModeCtrl = OFF;

					// Turn ON backlight?
					if(backlightCounter == 0)
					{
						// Reset FTM 2 counter
						FTM2->CNT = 0;
						// Load PWM duty cycle
						LoadBacklightPWMDutyCycle(backlightPWMDutyCycle);
						// Load counter period to keep backlight ON
						backlightCounter = KEYPAD_BACKLIGHT_ON_PERIOD;
					}

					// Load counter to keep backlight ON?
					if(backlightCounter != 100)
					{
						// Load counter period to keep backlight ON
						backlightCounter = KEYPAD_BACKLIGHT_ON_PERIOD;
					}
				}
				else
				{
					// Act upon backlightCounter value
					switch(backlightCounter)
					{
					case 0:
					{	// Default: no action
						break;
					}
					case 1:
					{
						// MCU enters the low power mode with periodical wake-up by LPTMR
						lowPowerModeCtrl = ON;
						// Reset counter
						backlightCounter = 0;

						break;
					}
					case 2:
					{
						// Turn OFF backlight
						LoadBacklightPWMDutyCycle(0);
						// Load counter value
						backlightCounter = 1;

						break;
					}
					default:
					{
						// Keyboard backlight ON?
						if (backlightCounter > 2)
						{
							// Decrement counter
							backlightCounter--;
						}

						break;
					}
					}
				}

#else // EGS not used
#if JITTERING
#if (JITTERING_OPTION == 1)
			#if ((REFERENCE_DESIGN_BOARD == S32K144_6PAD_KEYPAD_SLIDER) && (NUMBER_OF_ELECTRODES < 1))
			// Jitter sample rate
			Jitter(sliderAdcDataElectrodeDischargeRaw[0]);
			#else
			// Jitter sample rate
			Jitter(adcDataElectrodeDischargeRaw[0][frequencyID]);
			#endif
#endif
#endif
			// Wait ADC free
			while (adc_busy);

			// Sense touch buttons electrodes touch event
			adc_busy = true;
			ElectrodeTouchElecSense();
			adc_busy = false;


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

#if FREQUENCY_HOPPING
			// Change the frequency ID
			FrequencyHop();

			if(frequencyHoppingActivation == 1)
			{
				//If LPIT disabled (after power-up/wakeup), enable LPIT
				if((LPIT0->TMR[0].TCTRL & LPIT_TMR_TCTRL_T_EN_MASK) == 0)
				{
					//Enable LPIT
					LPIT_Enable();
				}

				// Wait for LPIT to timeout
				while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {}

				// Clear LPIT0 timer flag 0
				LPIT0->MSR = LPIT_MSR_TIF0_MASK;

				// Disable LPIT
				LPIT_Disable();

				// Sense touch buttons electrodes again on second period/frequency
				ElectrodeTouchElecSense();

#if SLIDER_ENABLE
#if (NUMBER_OF_USED_ADC_MODULES == 2)
			// Preset configuration for both ADC0 and ADC1 for voltage digitalization at the same time
			ADCs_SimultaneousHWtrigger();
			// Sense electrode touch event at slider electrodes on second period/frequency
			SliderElectrodesTouchElecSense();
			// Reset configuration for both ADC0 and ADC1 for voltage digitalization one by one
			ADCs_SetBackToSWtrigger();
#else
			// Sense electrode touch event at slider electrodes on second period/frequency
			SliderElectrodesTouchElecSense();
#endif
#endif
			// If any touch button touched - change the number of sensing cycles per sample
			ElectrodeSensingCyclesChange();

#if SLIDER_ENABLE
			// Change the number of sensing cycles per sample for slider
			SliderSensingCyclesChange();
#endif
			// Change the frequency ID
			FrequencyHop();
			}
#else
			// If any touch button touched - change the number of sensing cycles per sample and sense them again
			ElectrodeSensingCyclesChange();

#if SLIDER_ENABLE
			// Change the number of sensing cycles per sample for slider
			SliderSensingCyclesChange();
#endif

#endif

			// Any electrode touched?
#if SLIDER_ENABLE
			if (electrodeTouchQualifiedReport == 1 || sliderElectrodeTouchQualifiedReport == 1)
#else
				if (electrodeTouchQualifiedReport == 1)
#endif
				{
					// MCU in RUN mode only
					lowPowerModeCtrl = OFF;

					// Turn ON backlight?
					if(backlightCounter == 0)
					{
						// Reset FTM 2 counter
						FTM2->CNT = 0;
						// Load PWM duty cycle
						LoadBacklightPWMDutyCycle(backlightPWMDutyCycle);
						// Load counter period to keep backlight ON
						backlightCounter = KEYPAD_BACKLIGHT_ON_PERIOD;
					}

					// Load counter to keep backlight ON?
					if(backlightCounter != 100)
					{
						// Load counter period to keep backlight ON
						backlightCounter = KEYPAD_BACKLIGHT_ON_PERIOD;
					}
				}
				else
				{
					// Act upon backlightCounter value
					switch(backlightCounter)
					{
					case 0:
					{	// Default: no action
						break;
					}
					case 1:
					{
						// MCU enters the low power mode with periodical wake-up by LPTMR
						lowPowerModeCtrl = ON;
						// Reset counter
						backlightCounter = 0;

						break;
					}
					case 2:
					{
						// Turn OFF backlight
						LoadBacklightPWMDutyCycle(0);
						// Load counter value
						backlightCounter = 1;

						break;
					}
					default:
					{
						// Keyboard backlight ON?
						if (backlightCounter > 2)
						{
							// Decrement counter
							backlightCounter--;
						}

						break;
					}
					}
				}

#endif

#if(LOW_POWER_MODE == LPM_DISABLE)
			// Recorder time base defined by ELECTRODES_SENSE_PERIOD
			//FMSTR_Recorder();
#endif

// Set new LPTMR timeout period (change of timeout period in case of wake-up electrode in proximity mode OR application uses decimation filter)
			// When not using EGS but using DECIMATION FILTER the LPTMR CMR is set on the end of trimming function forever
#ifdef WAKE_UP_ELECTRODE // If wake up electrode is defined
	#if DECIMATION_FILTER || (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES) // If decimation filter is On or proximity by wake up electrode is used
				if(electrodeTouch[WAKE_UP_ELECTRODE][0] == 1) // EGS touched / proximity detected
				{
	#if DECIMATION_FILTER
					// Set LPTMR timeout period of decimation filter (3ms)
					LPTMR0_CMRupdate(LPTMR_ELEC_SENSE_DF);
	#else
					// Set LPTMR 30 ms timeout period
					LPTMR0_CMRupdate(LPTMR_ELEC_SENSE);
	#endif
				}
				else // EGS released / proximity not detected
				{
	#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
					// Set LPTMR proximity timeout period
					LPTMR0_CMRupdate(LPTMR_ELEC_SENSE_PROXIMITY);
	#else
					// Set LPTMR 30 ms timeout period
					LPTMR0_CMRupdate(LPTMR_ELEC_SENSE);
	#endif
				}
	#endif
#endif


		}
		else
		{
			// Self-trim touch button electrodes after power-up / reset
			ElectrodeSelfTrimSense();

#if SLIDER_ENABLE
	#if (NUMBER_OF_USED_ADC_MODULES == 2)
				// Preset configuration for both ADC0 and ADC1 for voltage digitalization at the same time
				ADCs_SimultaneousHWtrigger();
				// Self-trim slider electrodes after power-up / reset
				SliderElectrodeSelfTrimSense();
				// Reset configuration for both ADC0 and ADC1 for voltage digitalization one by one
				ADCs_SetBackToSWtrigger();
	#else
				// Self-trim slider electrodes after power-up / reset
				SliderElectrodeSelfTrimSense();
	#endif
#endif
		}

#if DECIMATION_FILTER
	// Clear TCF LPTMR
	//REG_RMW32(&LPTMR0->CSR, 1 << 7, 1 << 7);
	LPTMR0->CSR |= 1 << 7;
#endif
}

/*****************************************************************************
 *
 * Function: void LPIT_Init(uint32_t timeout)
 *
 * Description: Init LPIT
 *
 *****************************************************************************/
void LPIT_Init(uint32_t timeout)
{
	LPIT0->MCR = 0x00000001; /* DBG_EN-0: Timer chans stop in Debug mode */
	/* DOZE_EN=0: Timer chans are stopped in DOZE mode */
	/* SW_RST=0: SW reset does not reset timer chans, regs */
	/* M_CEN=1: enable module clk (allows writing other LPIT0 regs)*/

	LPIT0->TMR[0].TVAL = timeout; /* Chan 0 Timeout period: 0.33 ms */
}

/*****************************************************************************
 *
 * Function: void LPIT_Enable(void)
 *
 * Description: Enables LPIT
 *
 *****************************************************************************/
void LPIT_Enable(void)
{
	// LPIT ch0 Enable
	LPIT0->TMR[0].TCTRL = 0x00000001; 	/* T_EN=1: Timer channel is enabled */
	/* CHAIN=0: channel chaining is disabled */
	/* MODE=0: 32 periodic counter mode */
	/* TSOT=0: Timer decrements immediately based on restart */
	/* TSOI=0: Timer does not stop after timeout */
	/* TROT=0 Timer will not reload on trigger */
	/* TRG_SRC=0: External trigger source */
	/* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}

/*****************************************************************************
 *
 * Function: void LPIT_Disable(void)
 *
 * Description: Disables LPIT
 *
 *****************************************************************************/
void LPIT_Disable(void)
{
	// LPIT ch0 Disable
	LPIT0->TMR[0].TCTRL = 0x00000000;
}

#if JITTERING
#pragma GCC push_options
#pragma GCC optimize ("O1")
/*****************************************************************************
 *
 * Function: void Jitter(void)
 *
 * Description: Creates random short delay in order to jitter the sample rate
 *
 * Input: ADC raw data of either EGS electrode or touch button electrode or slider electrode
 *
 *****************************************************************************/
void Jitter(int32_t jitter)
{
	// Mask the raw data
	jitter = jitter & JITTERING_MASK;
	// Store jitter value for Freemaster
	jitterRead = jitter;
	// Delay
	while(jitter--);

}
#pragma GCC pop_options
#endif

/*****************************************************************************
 *
 * Function: void DMA1_IRQHandler(void)
 *
 * Description: DMA Channel 1 Interrupt Routine. Measurement ready flag sets
 *
 *****************************************************************************/
/*
void DMA1_IRQHandler (void)
{
	// ISR is triggered after all samples are stored in the array

	DMA->CINT = DMA_CINT_CINT(1);  											// Clear CH2 Interrupt Request flag
	DMA->TCD[0].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  				// Clear Channel 0 Done flag
	DMA->TCD[1].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;					// Clear Channel 1 Done flag

	// measurement ready flag
	ready_dma_flag = 1;
}*/

/*****************************************************************************
 *
 * Function: void DMA2_IRQHandler(void)
 *
 * Description: DMA Channel 2 Interrupt Routine. Measurement ready flag sets
 *
 *****************************************************************************/
/*
void DMA2_IRQHandler (void)
{
	// ISR is triggered after all samples are stored in the array

	DMA->CINT = DMA_CINT_CINT(2);  											// Clear CH2 Interrupt Request flag
	DMA->TCD[0].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  				// Clear Channel 0 Done flag
	DMA->TCD[1].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;					// Clear Channel 1 Done flag
	DMA->TCD[2].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;					// Clear Channel 1 Done flag

	// measurement ready flag
	ready_dma_flag = 1;
}*/
