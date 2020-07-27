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
#include <math.h>
#include "iseled_driver.h"
#include "flexio.h"
#include "adConv1.h"
#include "lpuart1.h"


#include "main.h"
#include "scg.h"
#include "timer.h"
#include "adc.h"
#include "pcc.h"
#include "lpuart.h"
#include "ets.h"
#include "power_mode.h"
#include "ts_cfg.h"
#include "gpio.h"
#include "gpio_inline_fcn2.h"
#include "flextimer.h"
#include "slider.h"

#include "ets.h"
#include "tcd.h"


/*#define CPU_IN_HSRUN_MODE	0*/

#define NR_OF_MODES		4U

#define NORMAL_MODE		0U
#define AUTO_MODE		1U
#define POINT_MOVE_MODE	2U
#define SKY_MODE		3U

#define NR_OF_LEDS      17U
#define NR_OF_LEDCOLOR	15U


#define NR_OF_TEST		6U
#define NR_OF_PARM		12U

#define LED0            15U
#define LED1            16U
#define LED_GPIO        PTD


#define BUTTON0			12U
#define BUTTON1			13U
#define BUTTON_GPIO		PTC
#define BUTTON_IRQ_TYPE PORTC_IRQn
#define BUTTON_REPEAT_DELAY_COUNT 10U /* count for button delay enable for prevent repeated trigger, time = count*PERIOD_BY_NS */

#define ADC_CONT_CH		0
#define ADC_THRESHOLD   20
#define ADC_MAX 		(1<<12)
#define ADC_MIN			0







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

uint32_t digLEDResultBufferStrip1[34];
digLED_ReadDataResultType digLEDResultStrip1 = {.chainLength = 1, .retData = digLEDResultBufferStrip1};

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
 uint8_t button_delay_count = 0;

/* for triggering led update */
volatile bool updateLED = true;

/* for potentiometer */
volatile uint16_t adc_reading = 0;
volatile uint16_t adc_prev_reading = 0;

/*for prevent sometime of use potentiometer and touchpad for ADC meassurement*/
bool adc_busy = false;

/*for display led information, stop update LED while enquire led data*/
bool showled_busy = false;
bool showled_trigger = false;



//TS Area
int16_t calibrationGainADC0;
int16_t calibrationGainADC1;
uint8_t  clockMode;
// Low power mode
extern uint8_t  lowPowerModeCtrl, lowPowerModeEnable;
extern tElecStatus  electrodesStatus, sliderElectrodesStatus;
uint8_t  electrodeTouchQualifiedDisplay;
uint8_t  sliderElectrodeTouchQualifiedDisplay;
uint8_t   elecNum, elecNumAct;
// Touch qualification
uint8_t   electrodeTouchQualified[NUMBER_OF_ELECTRODES];
uint8_t   electrodeTouchQualifiedReport;
uint8_t   electrodeTouchNumberPlusOne;

/* This should be declared in the application - used by FlexIO for ISELED */
flexio_device_state_t flexIODeviceState;

uint32_t nrOfLEDs = NR_OF_LEDS;

uint8_t numOfLEDOn =	17U;


/*storage current and prev color for color change, index of led_color_array */
uint8_t ledColor = 0;
uint8_t ledPrevColor = 0;

/*storage led running mode*/
uint8_t ledMode = 0;

/*duration of a loop of animation  */
uint16_t animColorTransDuration = 1*50;
/*counter for animation*/
uint16_t animCounter = 0;

typedef struct
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;

}led_color;


/*for mode 2 moving drop */
int drop_center_pos = 5;
float drop_decay_factor = 0.2;
float drop_center_dim = 0.02;


/* for overall RGB value */
   led_color rgb;

 /* for overall gradually pos  and dim value*/
 float pos,d;

/* array for individual led */
led_color ledcolor[NR_OF_LEDS];
/* array for individual dim*/
float dim[NR_OF_LEDS];


/*Mode 3 Sky Night Effect*/

/*cycle*/
#define SKY_CYCLE (60*50)

uint32_t skyCount = 0;
uint32_t skyCycle = SKY_CYCLE;
uint32_t skySunrise = SKY_CYCLE/4;
uint32_t skySunset = SKY_CYCLE/4*3;
uint32_t skyTransferDur = SKY_CYCLE/8;
/*for random deviation */
float skyDelta[NR_OF_LEDS];
/*for cloud effect*/
uint8_t skyCloudPattern[NR_OF_LEDS];
/*Movement of Cloud 0:no move, 1:left, 2:right*/
uint8_t skyCloudDir = 0;

/*color pattern for cloud 0:clear, 1:from clear to cloud, 2: from cloud to clear, 3:cloud,
 * will added deviation for more natural feel */
led_color skyBlue[4] = {{20,60,255},{40,120,255},{40,120,255},{80,150,255}};



/*array for led change while push button or change auto */
const led_color led_color_array[NR_OF_LEDCOLOR] = {
		{0xFF, 0xFF, 0xFF}, //WHITE
		{0xFF, 0x00, 0x00},	//RED
		{0x00, 0xFF, 0x00}, //GREEN
		{0x00, 0x00, 0xFF}, //BLUE
		{0xFF, 0xFF, 0x00}, //YELLOW
		{0x00, 0xFF, 0xFF}, //CYAN
		{0xFF, 0x00, 0xFF}, //MAGENTA
		{0xF0, 0xF6, 0x8C}, //KHAKI
		{0xFF, 0xD7, 0x00}, //GOLD
		{0xFF, 0xA5, 0x00}, //ORANGE
		{0x3F, 0xEC, 0x3F}, //SEA GREEN
		{0x72, 0xBB, 0xFF}, //SKY BLUE
		{0x30, 0x00, 0x82}, //INDIGO
		{0xF5, 0xF5, 0x80}, //BEIGE
		{0xC7, 0xC7, 0xFF}  //LAVENDER


};

/* for show LED diagnosis */
const char* led_test_string[NR_OF_TEST] = {
		"Ref Voltage1 5v",
		"Ref Voltage2 1.5v",
		"Vf red",
		"Vf green",
		"Vf blue",
		"bias voltage"


};





/* for show LED parm */
const char* led_parm_string[NR_OF_PARM] = {
		"configuration",
		"max PWM value for red LED",
		"max PWM value for green LED",
		"max PWM value for blue LED",
		"peak current for green LED",
		"peak current for blue LED",
		"temperature sensor offset value",
		"ADC offset and Vref cal. values",
		"bias reference",
		"temp. comp. LUT init. base value",
		"temp. comp. LUT init. offset value",
		"last fuse"


};


/* Input buffer used for the digLED_Send_Cmd_Block function. This buffer needs to be updated
 * before each function call as it is also used by the driver to encode the commands.
 * The COLOR parameter in each member of the array represents the 24bit hexadecimal value
 * of the color to be sent
 *
 * We update both dim and color, so require double size of total LEDs.
 *
 * */
digLED_SendCmdBlockType cmdBlock[NR_OF_LEDS*2] = {
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
			{0xA1, 0xBB, 0xCC, 17, SET_RGB_BLOCK, {0}}

	};


void updateColorTick(void);
void shiftColor(bool);

/* Delay function - do nothing for a number of cycles */
void delay(volatile int cycles)
{
    while(cycles--);
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

    /* run ADC measurement  */






    if (!adc_busy){
    	adc_busy = true;
    	ADC_DRV_ConfigChan(INST_ADCONV1,ADC_CONT_CH,&adConv1_ChnConfig0);
    }
    /* button delay counter for hold on detect */
    if (button_delay_count)
    	--button_delay_count;


}

bool read_GPIO_Pin(pins_channel_type_t pin,const GPIO_Type * const gpio){

	return (PINS_DRV_ReadPins(gpio) >> pin) & 1;


}




/*
 * Kelvin to RGB ref based on
 * https://tannerhelland.com/2012/09/18/convert-temperature-rgb-algorithm-code.html */

led_color KtoRGB(uint16_t k){
	led_color result;


	float temp = k/100;
	float c;

	if (temp <= 66)
		result.red = 255;
	else
	{
		c = temp-60;
		c = 329.698727446 * pow(c,-0.1332047592);
		if (c<0)
			result.red = 0;
		else if (c>255)
			result.red = 255;
		else
			result.red = c;
	}

	if (temp <=66){
		c = temp;
		c = 99.4708025861 * log(c) - 161.1195681661;
	}else{
		c = temp - 60;
		c = 288.1221695283 * pow (c,-0.0755148492);
	}

	if (c<0)
		result.green = 0;
	else if (c>255)
		result.green = 255;
	else
		result.green = c;

	if (temp>=66){
		result.blue = 255;
	}else{
		if (temp<19)
			result.blue = 0;
		else{
			c = temp-10;
			c = 138.5177312231*log(c) - 305.0447927307;
			if (c<0)
				result.blue = 0;
			else if (c>255)
				result.blue = 255;
			else
				result.blue = c;
			}


		}


		return result;



	}





/* Main animation update function */


void updateColorTick(void){


	if (!animCounter || showled_busy)
		return;






/* normalized pos of animation cycle  */
	pos = (float)animCounter/animColorTransDuration;

/* overall dim by potentiometer  */
	d = (adc_prev_reading-adc_reading>0?
			(adc_prev_reading-adc_reading)*pos+adc_reading:
			adc_reading-(adc_reading-adc_prev_reading)*pos) / (float) ADC_MAX;
















	if (ledMode==NORMAL_MODE || ledMode==AUTO_MODE || ledMode==POINT_MOVE_MODE){

		 /*gradually color change effect*/

			rgb.red = led_color_array[ledPrevColor].red-led_color_array[ledColor].red>0?
					(led_color_array[ledPrevColor].red-led_color_array[ledColor].red)*pos+led_color_array[ledColor].red:
					led_color_array[ledColor].red-(led_color_array[ledColor].red-led_color_array[ledPrevColor].red)*pos;
			rgb.blue = led_color_array[ledPrevColor].blue-led_color_array[ledColor].blue>0?
					(led_color_array[ledPrevColor].blue-led_color_array[ledColor].blue)*pos+led_color_array[ledColor].blue:
					led_color_array[ledColor].blue-(led_color_array[ledColor].blue-led_color_array[ledPrevColor].blue)*pos;
			rgb.green = led_color_array[ledPrevColor].green-led_color_array[ledColor].green>0?
					(led_color_array[ledPrevColor].green-led_color_array[ledColor].green)*pos+led_color_array[ledColor].green:
					led_color_array[ledColor].green-(led_color_array[ledColor].green-led_color_array[ledPrevColor].green)*pos;

		   /*control for individual led */
		   for(int i=0;i<nrOfLEDs;i++){

				ledcolor[i].red = rgb.red;

				ledcolor[i].blue = rgb.blue;

				ledcolor[i].green = rgb.green;

				dim[i] = d;

				/* moving effect for ledMode 2 */
				if (ledMode==POINT_MOVE_MODE){


					float dist = drop_center_pos>=i?drop_center_pos-i+(1-pos):i-drop_center_pos-1+pos;

					float factor = dist*drop_decay_factor;

					factor+=drop_center_dim;

					if (factor>=1)
						continue;


					factor*=factor;


					if (factor>1)
						factor=1;

					dim[i]*=factor;


				}

		   }

	}else if (ledMode==SKY_MODE){

		/*sky mode to emulate night and day*/


		float colortemp = 10000;
		bool night = false;

		if (skyCount>skySunrise-skyTransferDur && skyCount<skySunset+skyTransferDur ){

			/* sunrising color temp from 3000 to 10000 */

			if (skyCount<skySunrise+skyTransferDur){
					/*from night to day */
					d *= 1/(float)(skyTransferDur*2)*(skyCount-(skySunrise-skyTransferDur));
					colortemp = 7000/(float)(skyTransferDur*2);
					colortemp *= skyCount-(skySunrise-skyTransferDur);
					colortemp +=3000;
					colortemp -=rand()%500;
					rgb = KtoRGB(colortemp);

			}else if (skyCount>skySunset-skyTransferDur){
				/*from day to night */

				d *= 1/(float)(skyTransferDur*2)*((skySunset+skyTransferDur)-skyCount);
							colortemp = 7000/(float)(skyTransferDur*2);
								colortemp *=(skySunset+skyTransferDur)-skyCount;
								colortemp +=3000;
								colortemp -=rand()%500;

							rgb =  KtoRGB(colortemp);

			}else{
				/* Day time */
					colortemp -=rand()%200;
					rgb =  KtoRGB(colortemp);

			}

			/* update cloud effect */
			if (!(skyCount%20)){


				/* chance to change direction or stop */
				if (!(rand()%5) ){
					if (skyCloudDir==0)
						skyCloudDir=rand()%2+1;
					else
						skyCloudDir=0;

				}

				// forward
				if (skyCloudDir==1){


					for(int i=NR_OF_LEDS-1;i>0;i--)
						skyCloudPattern[i]=skyCloudPattern[i-1];

					if (skyCloudPattern[1]==2)
						skyCloudPattern[0]=0;
					else if (skyCloudPattern[1]==1)
						skyCloudPattern[0]=3;
					else if (skyCloudPattern[1]==3)
						skyCloudPattern[0]=rand()%4?3:2;
					else
						skyCloudPattern[0]=rand()%4?0:1;
				}//backward
				else if (skyCloudDir==2){

					for(int i=0;i<NR_OF_LEDS;i++)
						skyCloudPattern[i]=skyCloudPattern[i+1];

					if (skyCloudPattern[NR_OF_LEDS-2]==2)
						skyCloudPattern[NR_OF_LEDS-1]=0;
					else if (skyCloudPattern[NR_OF_LEDS-2]==1)
						skyCloudPattern[NR_OF_LEDS-1]=3;
					else if (skyCloudPattern[NR_OF_LEDS-2]==3)
						skyCloudPattern[NR_OF_LEDS-1]=rand()%4?3:2;
					else
						skyCloudPattern[NR_OF_LEDS-1]=rand()%4?0:1;



				}
			}



		}else if (skyCount==skySunrise-skyTransferDur){

			/*INIT sunrise,reset cloud pattern */
			for(int i=0;i<NR_OF_LEDS;i++){

							if (i==0)
								skyCloudPattern[i]=rand()%4;
							else if (skyCloudPattern[i-1]==1)
								skyCloudPattern[i]=3;
							else if (skyCloudPattern[i-1]==2)
								skyCloudPattern[i]=0;
							else if (skyCloudPattern[i-1]==3)
								skyCloudPattern[i]=rand()%4?3:2;
							else
								skyCloudPattern[i]=rand()%4?0:1;

						}

		}
		else if (skyCount==skySunset+skyTransferDur){


			/*INIT sunset, reset all led to full off*/

			rgb.red = 0;
			rgb.green = 0;
			rgb.blue = 0;

			/*reset pattern*/

			for(int i=0;i<NR_OF_LEDS;i++){


				ledcolor[i].red = 0;
				ledcolor[i].green = 0;
				ledcolor[i].blue = 0;
				dim[i] = 0;







			}

			night=true;


		}else if (skyCount<skySunrise || skyCount>skySunset){
			/* Night time */


			night = true;

		}



		for(int i=0;i<nrOfLEDs;i++){






			if (night){
			/* random make star */
				if (ledcolor[i].red!=0){
					if (!(rand()%50)){
						ledcolor[i].red=0;
						ledcolor[i].green=0;
						ledcolor[i].blue=0;
					}

				}else{
					if (!(rand()%500)){
						/* random select color temp from 3000 to 10000 for star */
						led_color star = KtoRGB(rand()%7000+3000);

						ledcolor[i].red=star.red;
						ledcolor[i].green=star.green;
						ledcolor[i].blue=star.blue;

						dim[i]=d-(float)rand()/(float)(RAND_MAX/0.4);

					}
				}

			}else{


				/*mixing of color temperature and cloud/clear color  */
				ledcolor[i].red = (rgb.red/255.0) * (skyBlue[skyCloudPattern[i]].red/255.0) * 255;
				ledcolor[i].green= (rgb.green/255.0)  * (skyBlue[skyCloudPattern[i]].green/255.0) * 255;
				ledcolor[i].blue= (rgb.blue/255.0)  * (skyBlue[skyCloudPattern[i]].blue/255.0) * 255;;

				dim[i]= d;

				/*add some random deviation */
				uint8_t rem = skyCount%500;
				if (!rem){
					if (rand()%2)
						skyDelta[i]=(float)rand()/(float)(RAND_MAX/(d*0.02));
					else
						skyDelta[i]=0;
				}else if (skyDelta[i]){


					if (rem<150)
						skyDelta[i]-=(float)rand()/(float)(RAND_MAX/(d*0.02));
					else if (rem>350)
						skyDelta[i]+=(float)rand()/(float)(RAND_MAX/(d*0.02));
				}

				dim[i] -= skyDelta[i];


				if (dim[i]<0.0)
					dim[i]=0.0;
				else if (dim[i]>1.0)
					dim[i]=1.0;


			}





		}

		++skyCount;
		if (skyCount>=skyCycle)
			skyCount=0;


	}



	/*prepare cmdblock for batch sending though DMA */


	for(int i=0;i<nrOfLEDs;i++){


		   /*calculate dim value */
			uint8_t red_dim = 0;
			float value = ledcolor[i].red*dim[i];
			while (value<128 && red_dim<3){
				++red_dim;
				value*=2;
			}

			uint8_t redcolor = value;

			uint8_t green_dim = 0;
			value = ledcolor[i].green*dim[i];

				while (value<128 && green_dim<3){
					++green_dim;
					value*=2;
				}

			uint8_t greencolor = value;

			uint8_t blue_dim = 0;
			value = ledcolor[i].blue*dim[i];

				while (value<128 && blue_dim<3){
					++blue_dim;
					value*=2;
				}

			uint8_t bluecolor = value;



			cmdBlock[i].addr = i+1;
			cmdBlock[i].red = red_dim;
			cmdBlock[i].green = green_dim;
			cmdBlock[i].blue = blue_dim;
			cmdBlock[i].cmd = SET_DIM_BLOCK;




			cmdBlock[i+nrOfLEDs].addr = i+1;
			cmdBlock[i+nrOfLEDs].red = numOfLEDOn>i?redcolor:0;
			cmdBlock[i+nrOfLEDs].green = numOfLEDOn>i?greencolor:0;
			cmdBlock[i+nrOfLEDs].blue = numOfLEDOn>i?bluecolor:0;
			cmdBlock[i+nrOfLEDs].cmd = SET_RGB_BLOCK;



		}

	/*send batch block */

	digLED_ReturnType result;
	result=digLED_Send_Cmd_Block(cmdBlock, nrOfLEDs*2, strip);

		if (result==DIGLED_EVALUATION_OVER){
			/*evaluation over, sample reset the led driver*/
			digLED_Init_Interface(NUMBER_OF_INTERFACES, iseled1_InitConfig);

			printf("Reset digLED driver while evaluation over\r\n");

				/* Initialize strip */
				digLEDResultStrip1.chainLength = nrOfLEDs;

				digLED_Init_Strip(&testInitType, &digLEDResultStrip1, strip);

				return;


		}


/* unexpected error, stop loop */
	if (result!=DIGLED_OK){
		animCounter = 0;
		return;
	}

	if (--animCounter)
		return;

	/*update prev value*/
	adc_prev_reading = adc_reading;
	ledPrevColor=ledColor;

	/*post-process of anim */

	if (ledMode==NORMAL_MODE){



		showled_trigger = true;
	}else if (ledMode==AUTO_MODE){



		updateLED = true;



	}else if (ledMode==POINT_MOVE_MODE){


		int limit = 1;

		if (++drop_center_pos>(int)(limit+nrOfLEDs))
			drop_center_pos=-limit;


		updateLED = true;


	}else if (ledMode==SKY_MODE){
		updateLED = true;
	}



}



/*function for start animation  */

void updateColor(){

	DEV_ASSERT(numOfLEDOn <= nrOfLEDs );



	if (ledMode==AUTO_MODE){
		shiftColor(true);
	}else if (ledMode==SKY_MODE){


	}

	animCounter = animColorTransDuration;


}

/* IRQ callback for reading potentiometer */
void adc_irq_callback(void){

	 	 /* get result */
	    uint16_t adc_result;
	    ADC_DRV_GetChanResult(INST_ADCONV1,ADC_CONT_CH,&adc_result);

	    uint16_t delta = adc_reading<adc_result?adc_result-adc_reading:adc_reading-adc_result;

	    if (delta>ADC_THRESHOLD && !updateLED){


	    	adc_prev_reading = adc_reading;
	    	// alignment on edge
	    	if (adc_result>ADC_MAX-ADC_THRESHOLD)
	    		adc_reading = ADC_MAX;
	    	else if (adc_result<ADC_THRESHOLD)
	    		adc_reading = ADC_MIN;
	    	else
	    		adc_reading = adc_result;

	    	updateLED = true;

	    }

	    adc_busy = false;

}


/*function for change color as increment or decrement index of led_color_array */
void shiftColor(bool inc){
	if (!animCounter)
		ledPrevColor = ledColor;
	if (inc){
		 ++ledColor;
		if (ledColor>=NR_OF_LEDCOLOR)
			ledColor=0;
	}else{
		 --ledColor;

		 /* unsigned int 0-1 is the max unsigned value */
		if (ledColor>NR_OF_LEDCOLOR)
			ledColor=NR_OF_LEDCOLOR-1;


	}

}

/*Handling Button IRQ when pushed*/

void button_irq_callback(void){

	/* clear port interupt flag */
			PINS_DRV_ClearPortIntFlagCmd(PORTC);

	/* button suspend count for hold on detection */
	if (button_delay_count){
		button_delay_count = BUTTON_REPEAT_DELAY_COUNT;
		return;
	}

	button_delay_count = BUTTON_REPEAT_DELAY_COUNT;



	  if (read_GPIO_Pin(BUTTON0,BUTTON_GPIO)) {

		  shiftColor(true);
		  updateLED = true;



	  }else if (read_GPIO_Pin(BUTTON1,BUTTON_GPIO)) {

		  shiftColor(false);
		  updateLED = true;

	}






}

/* show LED info to UART console */
void showLedInfo(void){

	if (showled_busy)
		return;
	showled_busy = true;

	digLED_ReturnType ret;



	digLEDResultStrip1.chainLength = nrOfLEDs;


	for(int i=0;i<NR_OF_TEST;i++){

		do{
			stateFlag = 0;
			digLED_Test(i+1,0,strip);
			while (!stateFlag );

			stateFlag = 0;
			ret = digLED_Read_Diagnostic (&digLEDResultStrip1, strip);

			while (!stateFlag );

		}while(ret!=DIGLED_OK);


		printf("%s: ",led_test_string[i]);

		for(uint8_t j=0;j<digLEDResultStrip1.chainLength;j++){

			float value = digLEDResultStrip1.retData[j]&0xFFF;


			/* Ref value INLC10AQ Datasheet of Voltage detection sensitivities */



			if (i==5)
				value -=0x12d;
			else if (i==1)
				value -=0x17c;
			else
				value /=93;


			if (j==0)
				printf("%.2f",value);
			else
				printf(",%.2f",value);
		}


		printf("\r\n");
	}




	for(int i=0;i<NR_OF_PARM;i++){

		do{

			stateFlag = 0;
			ret = digLED_Read_Param (i==NR_OF_PARM-1?15:i+1,&digLEDResultStrip1, strip);

			while (!stateFlag );

		}while(ret!=DIGLED_OK);


		printf("%s: ",led_parm_string[i]);

		for(uint8_t j=0;j<digLEDResultStrip1.chainLength;j++){

			uint16_t value = digLEDResultStrip1.retData[j]&0xFFF;

			if (j==0)
				printf("0x%03X",value);
			else
				printf(",0x%03X",value);
		}


		printf("\r\n");
	}


	uint32_t tempoffset[34];
	digLED_ReadDataResultType tempoffsetStrip1 = {.chainLength = NR_OF_LEDS, .retData = tempoffset};

	do{

			stateFlag = 0;
			ret = digLED_Read_Param (7,&tempoffsetStrip1, strip);

			while (!stateFlag );

		}while(ret!=DIGLED_OK);


	do{

			stateFlag = 0;
			ret = digLED_Read_Temp (&digLEDResultStrip1, strip);

			while (!stateFlag );

		}while(ret!=DIGLED_OK);




	printf("temp: ");

			for(uint8_t j=0;j<digLEDResultStrip1.chainLength;j++){

				float value = tempoffsetStrip1.retData[j]&0xFFF;

				value += 94.57-(digLEDResultStrip1.retData[j]&0xFFF);

				value /= 0.87;

				if (j==0)
					printf("%.3f",value);
				else
					printf(",%.3f",value);
			}


		printf("\r\n");







		for (int i=0;i<3;i++){

			char* color;
			digLED_ReturnType (*fun)(digLED_ReadDataResultType*, uint8_t );

			switch (i){
				case 0:
					color = "Red";
					fun = &digLED_Read_PWM_Red;
					break;
				case 1:
					color = "Green";
					fun = &digLED_Read_PWM_Green;
					break;
				case 2:
					color = "Blue";
					fun = &digLED_Read_PWM_Blue;
					break;

			}

			do{

					stateFlag = 0;
					ret = fun(&digLEDResultStrip1, strip);

					while (!stateFlag );

				}while(ret!=DIGLED_OK);


			printf("%s PWM: ",color);

				for(uint8_t j=0;j<digLEDResultStrip1.chainLength;j++){

					uint32_t value = digLEDResultStrip1.retData[j]&0xFFF;

					if (j==0)
						printf("%lu",value);
					else
						printf(",%lu",value);
				}


				printf("\r\n");



		}




		showled_busy=false;

}


/*Handling touchpad event*/

void touchPadControl(void)
{

  static uint8_t previousElectrodeValue[NUMBER_OF_ELECTRODES] = {0xFF, 0xFF};

  if ((previousElectrodeValue[0] != electrodeTouchQualified[0]) || (previousElectrodeValue[1] != electrodeTouchQualified[1]))
  {

	  if (!previousElectrodeValue[0] && electrodeTouchQualified[0]){

		  if (ledMode>0)
			  --ledMode;
		  else if (numOfLEDOn>1)
			  --numOfLEDOn;



		  printf("touch change #0\r\n");


	  }else if (!previousElectrodeValue[1] && electrodeTouchQualified[1]){

		  if (numOfLEDOn<NR_OF_LEDS)
			  ++numOfLEDOn;
		  else if (ledMode<NR_OF_MODES)
			  ++ledMode;

		  printf("touch change #1\r\n");

	  }

	  if (ledMode==SKY_MODE){

			 skyCount = 0;
		 for(int i=0;i<nrOfLEDs;i++){
			 ledcolor[i].red=0;
			 ledcolor[i].blue=0;
			 ledcolor[i].green=0;

		 }

	  }



	  updateLED = true;




  }

  previousElectrodeValue[0] = electrodeTouchQualified[0];
  previousElectrodeValue[1] = electrodeTouchQualified[1];

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
	testInitType.tempCmpEnable = 1;
	testInitType.voltSwing = 0;
	testInitType.phaseShift = 0;



	clockMode = RUN_FIRC;


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

    /* Setup GPIO pins
     *  -   LED0 and LED1 as output
     */
    PINS_DRV_SetPinsDirection(LED_GPIO, (1 << LED0)|(1 << LED1));

    /* Turn off both LEDs on EVB */
    PINS_DRV_SetPins(LED_GPIO, (1 << LED0)|(1 << LED1));


    POWER_SYS_Init(&powerConfigsArr, 2U, &powerStaticCallbacksConfigsArr, 0U);

    /* Initialize ADC and auto calibration for ADC */
    ADC_DRV_ConfigConverter(INST_ADCONV1,&adConv1_ConvConfig0);

    /* Run ADC Auto Calibration */
    ADC_DRV_AutoCalibration(INST_ADCONV1);

    /* Config Handler of ADC */
    INT_SYS_InstallHandler(ADC_DRV_GetInterruptNumber(INST_ADCONV1),adc_irq_callback,(isr_t*)0);
    INT_SYS_EnableIRQ(ADC_DRV_GetInterruptNumber(INST_ADCONV1));


    /* config channel of Potentiometer and start first measure */
    ADC_DRV_ConfigChan(INST_ADCONV1,ADC_CONT_CH,&adConv1_ChnConfig0);


    /* Config Handler of Button */
    INT_SYS_InstallHandler(BUTTON_IRQ_TYPE,button_irq_callback,(isr_t*) 0);
    INT_SYS_EnableIRQ(BUTTON_IRQ_TYPE);

    INT_SYS_DisableIRQGlobal();


    // Flash and enable I/D cache and write buffer
    LMEM->PCCCR = LMEM_PCCCR_GO_MASK | LMEM_PCCCR_INVW1_MASK | \
    LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_ENCACHE_MASK;

  // Wake-up timer init
    LPTMR0_Init(LPTMR_ELEC_CAL);

    // ADC0 calibration init
    //  calibrationGainADC0 = ADC0_Calibration();

    calibrationGainADC0 = ADC0->G;

  // FTM2 init
    FTM2_Init();

  // Init electrode touch sense
    ElectrodeTouchSenseInit();


    NVIC_SET_IRQ_PRIORITY_LEVEL(DMA1_IRQn,1);// set higher priority for dma
    NVIC_SET_IRQ_PRIORITY_LEVEL(LPTMR0_IRQn,2);// set lower priority for lptmr

    NVIC_IRQ_ENABLE(DMA1_IRQn); // Enable DMA ch1 Interrupts
    NVIC_IRQ_ENABLE(LPTMR0_IRQn); // Enable LPTMR0 interrupts in NVIC


    INT_SYS_EnableIRQGlobal();



#ifdef CPU_IN_HSRUN_MODE
    {
    	POWER_SYS_SetMode(1U, POWER_MANAGER_POLICY_AGREEMENT);
    }
#else
    {
    	POWER_SYS_SetMode(0U, POWER_MANAGER_POLICY_AGREEMENT);
    }
#endif

    delay(150000);
    /* Get tick resolution in microseconds unit for TIMING over LPIT */
	TIMING_GetResolution(&timing_pal1_instance, TIMER_RESOLUTION_TYPE_NANOSECOND, &counterResolution);
    /*Start the timer necessary for the 50fps required by the updating functions (scanColor and dmaBlock)*/
    TIMING_StartChannel(&timing_pal1_instance, REFRESH_TIMING_CHANNEL, PERIOD_BY_NS/counterResolution);



	while(1)
	{


		if (digLED_FPSFlag){


			if (updateLED){

			  updateColor();

			  updateLED = false;
			}

			if (showled_trigger){
				showLedInfo();
				showled_trigger = false;
			}


			updateColorTick();


			digLED_FPSFlag = false;

		}


		touchPadControl();

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
