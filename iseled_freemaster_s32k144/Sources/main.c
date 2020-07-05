/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "pin_mux.h"
#include "pwrMan1.h"
#include "clockMan1.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "iseled_driver.h"
#include "flexio.h"
#include "adConv1.h"
#include "lpuart1.h"


/*#define CPU_IN_HSRUN_MODE	0*/

#define NR_OF_LEDS      17U

#define LED0            15U
#define LED1            16U
#define LED_GPIO        PTD


#define BUTTON0			12U
#define BUTTON1			13U
#define BUTTON_GPIO		PTC

#define ADC_CONT_CH		0
#define ADC_THRESHOLD   20
#define ADC_MAX 		(1<<12)
#define ADC_MIN			0;



/* Timing PAL channel used for 50 Hz LED refresh rate */
#define REFRESH_TIMING_CHANNEL	    1UL

/* Period in nanosecond unit */
#define PERIOD_BY_NS	20000000UL /* The period is 20 miliseconds */

typedef struct
{
	uint8_t TestNr;
	uint16_t Address;
} Test_Values;

typedef struct
{
	uint16_t Param;
	uint16_t Address;
} Set_Config_Values;

typedef struct
{
	uint8_t Red;
	uint8_t Green;
	uint8_t Blue;
	uint16_t Address;
} Set_RGB_Values;

typedef struct
{
	uint8_t Mode;
	uint16_t Address;
} Adc_Cal_Values;

typedef enum
{
	BUS_FREE,
	OPERATION_ONGOING
} APP_STATES;



volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */

uint32_t digLEDResultBufferStrip1[64];
digLED_ReadDataResultType digLEDResultStrip1 = {.chainLength = 1, .retData = &digLEDResultBufferStrip1[0]};

uint32_t ServiceNumber = 0;

Set_RGB_Values Set_RGB_Params = {0};
Set_RGB_Values Set_DIM_Params = {0};
Test_Values Test_Params = {0};
Set_Config_Values Set_Config_Params = {0};
Set_Config_Values Red_PWM  = {0};
Set_Config_Values Green_PWM  = {0};
Set_Config_Values Blue_PWM  = {0};
Set_Config_Values Green_Cur  = {0};
Set_Config_Values Blue_Cur  = {0};
Set_Config_Values BIAS = {0};
Set_Config_Values MCAST = {0};
uint8_t paramNumber = 1;
uint32_t taskFlag = 0;
Adc_Cal_Values Adc_Cal_Param;
Set_Config_Values Adc_Dac_Param;

digLED_InitType testInitType;
uint8_t strip = 0;
uint8_t initFlag = 0;
uint8_t repeatFlag = 1;
uint64_t counterResolution;
volatile uint8_t stateFlag = 0;
volatile uint8_t stripCallback;
volatile bool digLED_FPSFlag = false;
volatile APP_STATES appState = BUS_FREE;


volatile uint16_t adc_reading = 0;

/* This should be declared in the application - used by FlexIO for ISELED */
flexio_device_state_t flexIODeviceState;

uint32_t nrOfLEDs = NR_OF_LEDS;

/* Input buffer used for the digLED_Send_Cmd_Block function. This buffer needs to be updated
 * before each function call as it is also used by the driver to encode the commands.
 * The COLOR parameter in each member of the array represents the 24bit hexadecimal value
 * of the color to be sent*/
digLED_SendCmdBlockType cmdBlock[NR_OF_LEDS] = {
		   /*RED |GREEN |BLUE |LED ADDRESS|COMMAND */
			{0xA1, 0xBB, 0xCC, 1, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 2, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 3, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 4, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 5, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 6, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 7, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 8, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 9, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 10, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 11, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 12, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 13, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 14, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 15, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 16, SET_RGB_BLOCK, {0}},
			{0xA1, 0xBB, 0xCC, 17, SET_RGB_BLOCK, {0}},
	};

void FreemasterStateMachine(void);

/* Delay function - do nothing for a number of cycles */
void delay(volatile int cycles)
{
    while(cycles--);
}

/*Give each LED a unique random color one after another*/
void scanColor(void)
{
	static uint8_t ledNr = 1;
	uint8_t r, g, b;
	if(digLED_FPSFlag == true)
	{

		digLED_FPSFlag = false;
		r = rand() % 256;
		g = rand() % 256;
		b = rand() % 256;
		ledNr++;
		if(ledNr > nrOfLEDs)
		{
			ledNr = 1U;
		}
		digLED_Set_RGB(r, g, b, ledNr, strip);
	}
}

/*Populate the input buffer with random colors and transmit them to all LEDs using a single
 * DMA or Interrupts driven transfer.*/
void dmaBlock(digLED_SendCmdBlockType* data)
{
	uint32_t i;
	/*poll the timer flag to get a burst of commands every 20 ms*/
	if(digLED_FPSFlag == true)
	{
		digLED_FPSFlag = false;
		/*set LEDs with random colors*/
		for(i=0; i<nrOfLEDs; i++)
		{
			data[i].addr = (i+1);
			data[i].red = rand() % 256;
			data[i].green = rand() % 256;
			data[i].blue = rand() % 256;
			data[i].cmd = SET_RGB_BLOCK;
		}
		digLED_Send_Cmd_Block(data, nrOfLEDs, strip);
	}
}

/* The ISELED callback that notifies the application about the transmission status*/
void digLED_Callback(digLED_CommEventType state, uint8_t stripNr)
{
	stateFlag = state;
	stripCallback = stripNr;
	appState = BUS_FREE;
}

/* Timing pal callback provides 20 ms intervals (50 fps)*/
void fpsUpdateCallback(void *userData)
{
	(void)userData;
    digLED_FPSFlag = true;
}

bool read_GPIO_Pin(pins_channel_type_t pin,const GPIO_Type * const gpio){

	return (PINS_DRV_ReadPins(gpio) >> pin) & 1;


}

void updateDim(){

	float dim = adc_reading / (float) ADC_MAX;




	digLED_Set_RGB((uint8_t)Set_RGB_Params.Red*dim,(uint8_t) Set_RGB_Params.Green*dim,(uint8_t) Set_RGB_Params.Blue*dim, Set_RGB_Params.Address, strip);

}

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
  /* Write your local variable definition here */
	testInitType.crcEnable = 1;
	testInitType.firstLedAdr = 1;
	testInitType.tempCmpEnable = 0;
	testInitType.voltSwing = 0;
	testInitType.phaseShift = 0;



	CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
	CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

	PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

	digLED_Init_Interface(NUMBER_OF_INTERFACES, iseled1_InitConfig);
	
	/*Initialize timing pal instance used by the ISELED Driver for the timeout mechanism*/
	TIMING_Init(&timing_pal1_instance, &timing_pal1_InitConfig);

	/* Initialize strip */
	digLEDResultStrip1.chainLength = nrOfLEDs;

	digLED_Init_Strip(&testInitType, &digLEDResultStrip1, strip);



	Set_RGB_Params.Red = 255;
	Set_RGB_Params.Green = 255;
	Set_RGB_Params.Blue = 0;

    /* Setup GPIO pins
     *  -   LED0 and LED1 as output
     */
    PINS_DRV_SetPinsDirection(LED_GPIO, (1 << LED0)|(1 << LED1));

    /* Turn off both LEDs on EVB */
    PINS_DRV_SetPins(LED_GPIO, (1 << LED0)|(1 << LED1));

    POWER_SYS_Init(&powerConfigsArr, 2U, &powerStaticCallbacksConfigsArr, 0U);

    /* Initialize ADC and auto calibration for ADC */
    ADC_DRV_ConfigConverter(INST_ADCONV1,&adConv1_ConvConfig0);

    ADC_DRV_AutoCalibration(INST_ADCONV1);


    /* Initialize UART console */
 //   LPUART_DRV_Init(INST_LPUART1,&lpuart1_State,&lpuart1_InitConfig0);





#ifdef CPU_IN_HSRUN_MODE
    {
    	POWER_SYS_SetMode(1U, POWER_MANAGER_POLICY_AGREEMENT);
    }
#else
    {
    	POWER_SYS_SetMode(0U, POWER_MANAGER_POLICY_AGREEMENT);
    }
#endif

    delay(100000);
    /* Get tick resolution in microseconds unit for TIMING over LPIT */
	TIMING_GetResolution(&timing_pal1_instance, TIMER_RESOLUTION_TYPE_NANOSECOND, &counterResolution);
    /*Start the timer necessary for the 50fps required by the updating functions (scanColor and dmaBlock)*/
    TIMING_StartChannel(&timing_pal1_instance, REFRESH_TIMING_CHANNEL, PERIOD_BY_NS/counterResolution);
	while(1)
	{
		switch(appState)
		{
			case BUS_FREE:
				FreemasterStateMachine();
				break;
			case OPERATION_ONGOING:
				/*Transmission in progress, application dedicated time frame*/
				break;
		}


		/* config channel of Potentiometer and start measure */
	    ADC_DRV_ConfigChan(INST_ADCONV1,ADC_CONT_CH,&adConv1_ChnConfig0);

	    /* wait reading finish */
	    ADC_DRV_WaitConvDone(INST_ADCONV1);

	    uint16_t adc_result;
	    /* get result */
	    ADC_DRV_GetChanResult(INST_ADCONV1,ADC_CONT_CH,&adc_result);

	    uint16_t delta = adc_result>adc_reading ? adc_result-adc_reading:adc_reading-adc_result;

	    if (delta>ADC_THRESHOLD ){







	    	printf("ADC: %d \r\n",adc_result);

	    	adc_reading = adc_result;

	    	updateDim();


	    }

	   if (read_GPIO_Pin(BUTTON0,BUTTON_GPIO)) {
		   if (Set_RGB_Params.Red==255)
			   Set_RGB_Params.Red = 0;
		   else
			   Set_RGB_Params.Red = 255;

		  updateDim();
	   }

	   if (read_GPIO_Pin(BUTTON1,BUTTON_GPIO)) {
		   if (Set_RGB_Params.Green==255)
			   Set_RGB_Params.Green = 0;
		   else
			   Set_RGB_Params.Green = 255;

		   updateDim();
	   }



	}

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */

void FreemasterStateMachine(void)
{
	/* Statemachine for FreeMaster
	 * each command is called once after setting the corresponding ServiceNumber
	 * the parameters have to be set in advance */
	switch(ServiceNumber)
	{
		case 0:				/* Do nothing */
        break;
	  	case 1:				/* execute digLED_Init */
			appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Init_Strip(&testInitType, &digLEDResultStrip1, strip);
			ServiceNumber = 0;
	  	break;
	  	case 2:				/* execute digLED_Set_RGB */
	  		appState = OPERATION_ONGOING;
			digLED_Set_RGB(Set_RGB_Params.Red, Set_RGB_Params.Green, Set_RGB_Params.Blue, Set_RGB_Params.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 3:
	  		appState = OPERATION_ONGOING;
			digLED_Test(Test_Params.TestNr, Test_Params.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 4:
	  		appState = OPERATION_ONGOING;
			digLED_Reset(strip);
			ServiceNumber = 0;
	  	break;
	  	case 5:
	  		appState = OPERATION_ONGOING;
			digLED_Set_Config(Set_Config_Params.Param, Set_Config_Params.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 6:
	  		appState = OPERATION_ONGOING;
			digLED_Set_PWM_Red(Red_PWM.Param, Red_PWM.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 7:
	  		appState = OPERATION_ONGOING;
			digLED_Set_PWM_Green(Green_PWM.Param, Green_PWM.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 8:
	  		appState = OPERATION_ONGOING;
			digLED_Set_PWM_Blue(Blue_PWM.Param, Blue_PWM.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 9:
	  		appState = OPERATION_ONGOING;
			digLED_Set_Cur_Green(Green_Cur.Param, Green_Cur.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 10:
	  		appState = OPERATION_ONGOING;
			digLED_Set_Cur_Blue(Blue_Cur.Param, Blue_Cur.Address, strip);
			ServiceNumber = 0;
	  	break;
	  	case 11:
	  		appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_Param(paramNumber, &digLEDResultStrip1, strip);
			ServiceNumber = 0;
	  	break;
	  	case 12:
	  		appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_Temp(&digLEDResultStrip1, strip);
			ServiceNumber = 0;
	  	break;
	  	case 13:
	  		appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_Status(&digLEDResultStrip1, strip);
			ServiceNumber = 0;
	  	break;
	  	case 14:
	  		appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_Diagnostic(&digLEDResultStrip1, strip);
			ServiceNumber = 0;
	  	break;
	  	case 17:
	  		appState = OPERATION_ONGOING;
			digLED_Set_Bias(BIAS.Param,BIAS.Address, strip);
			ServiceNumber = 0;
	  	break;
        case 23:
        	appState = OPERATION_ONGOING;
			digLED_Set_Trg_Adc_Cal(Adc_Cal_Param.Mode, Adc_Cal_Param.Address, strip);
			ServiceNumber = 0;
        break;
        case 24:
        	appState = OPERATION_ONGOING;
			digLED_Set_Adc_Dac(Adc_Dac_Param.Param, Adc_Dac_Param.Address, strip);
			ServiceNumber = 0;
        break;
        case 25:
        	appState = OPERATION_ONGOING;
			digLED_Set_Dim(Set_DIM_Params.Red, Set_DIM_Params.Green, Set_DIM_Params.Blue, Set_DIM_Params.Address, strip);
			ServiceNumber = 0;
		break;
		case 26:
			appState = OPERATION_ONGOING;
			digLED_Define_Mcast(MCAST.Param, MCAST.Address, strip);
			ServiceNumber = 0;
		break;
		case 27:
			appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_PWM_Red (&digLEDResultStrip1, strip);
			ServiceNumber = 0;
		break;
		case 28:
			appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_PWM_Green (&digLEDResultStrip1, strip);
			ServiceNumber = 0;
		break;
		case 29:
			appState = OPERATION_ONGOING;
			digLEDResultStrip1.chainLength = nrOfLEDs;
			digLED_Read_PWM_Blue (&digLEDResultStrip1, strip);
			ServiceNumber = 0;
		break;
		case 30:
			appState = OPERATION_ONGOING;
			digLED_Ping(&digLEDResultStrip1, strip);
			ServiceNumber = 0;
		break;
        case 33:
        	scanColor();
        	if(repeatFlag == 0)
        	{
        		ServiceNumber = 0;
        	}
        	break;
        case 34:
        	dmaBlock(cmdBlock);
        	if(repeatFlag == 0)
			{
        		/*Effect stopped, stop the timer*/
				ServiceNumber = 0;
			}
        	break;
	  	default:
	  	break;
	}
}


//
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
