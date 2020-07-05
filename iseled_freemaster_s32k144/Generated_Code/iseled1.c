/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : iseled1.c
**     Project     : iseled_freemaster_s32k144
**     Processor   : S32K144_100
**     Component   : iseled
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2020-06-25, 00:57, # CodeGen: 0
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file iseled1.c
** @version 01.00
*/         
/*!
**  @addtogroup iseled1_module iseled1 module documentation
**  @{
*/         

/* MODULE iseled1.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files in application code.
 */

#include "iseled1.h"


/*! Global configuration of iseled1 */
digLED_PinConfigType pinConfigFlexIO[NUMBER_OF_STRIPS_FLEXIO] =
{    
	   {
		       .dataPin = ISELED_PIN_9,
		       .clockPin = ISELED_PIN_7,
	   },
};

digLED_TimeoutType timeoutChannelsFlexIO[NUMBER_OF_STRIPS_FLEXIO] = 
{
	   {
		       &timing_pal1_instance,
		       3000U,
		       0U,
	   },
};


digLED_ConfigType iseled1_InitConfig[NUMBER_OF_INTERFACES] =
{
	   {
		       .nrOfStrips = NUMBER_OF_STRIPS_FLEXIO,
		       digLED_Callback,
		       DIGLED_INTERFACE_FLEXIO,
		       pinConfigFlexIO,
		       timeoutChannelsFlexIO,
		       ISELED_USING_DMA,
		       1U,
		       0U,
	   },	
};



/* END iseled1. */
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

