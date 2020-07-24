/*
 * tcd.c
 *
 *  Created on: 10.12.2018
 *      Author: nxf50214
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include <TS/Cfg/ts_cfg.h>
#include <TS/ets.h>
#include <TS/Peripherals/tcd.h>

/*******************************************************************************
 * Variables
 *******************************************************************************/
extern uint8_t   elecNum;
extern int16_t   adcDataElectrodeDischargeRawSample[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_ACTIVE];

#if SLIDER_ENABLE
extern int16_t   sliderAdcDataElectrodeDischargeRawSample[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
extern int16_t   sliderAdcDataElectrodeDischargeRawSampleElec0[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
extern int16_t   sliderAdcDataElectrodeDischargeRawSampleElec1[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_SLIDER_ACTIVE];
#endif

#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
extern int16_t   adcDataElectrodeDischargeRawSampleProximity[NUMBER_OF_ELECTRODE_SENSING_PRECYCLES_PER_SAMPLE + NUMBER_OF_ELECTRODE_SENSING_CYCLES_PER_SAMPLE_PROXIMITY];
#endif

// Guard pin structure
extern tGuardStruct guardStruct[1];

// Scatter - Gather TCDs set for electrodes
edma_software_tcd_t stcd_ch0[NUMBER_OF_ELECTRODES][8]  __attribute__((aligned(32)));

// Address set for electrodes
uint32_t SADDR_CONFIG[NUMBER_OF_ELECTRODES][8];
uint32_t* DADDR_CONFIG[NUMBER_OF_ELECTRODES][8];

// Hardware config of electrodes
extern tElecStruct elecStruct[NUMBER_OF_ELECTRODES];

#if SLIDER_ENABLE
// Scatter - Gather TCDs set for slider
edma_software_tcd_t slider_stcd_ch0[NUMBER_OF_SLIDER_ELECTRODES][13]  __attribute__((aligned(32)));

// Addresses set for slider
uint32_t SADDR_CONFIG_SLIDER[NUMBER_OF_SLIDER_ELECTRODES][13];
uint32_t* DADDR_CONFIG_SLIDER[NUMBER_OF_SLIDER_ELECTRODES][13];

// Hardware config of slider
extern tElecStruct elecStruct[NUMBER_OF_ELECTRODES], sliderElecStruct[NUMBER_OF_SLIDER_ELECTRODES];
#endif

// TCD instructions index
uint8_t tcdInstructionNum;

// DMA control flags
volatile uint32_t ready_dma_flag, start_dma_flag;

/*****************************************************************************
 *
 * Function: void start_TS_DMA(uint32_t electrodeNum)
 *
 * Description: Starts DMA touch sense algorithm for an electrode
 *
 *****************************************************************************/
void start_TS_DMA(uint32_t electrodeNum, int16_t sensingCycles)
{
	// Reconfigure DMA touch sense engine for the next electrode

	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																							// disable DMAMUX
	TRGMUX->TRGMUXn[TRGMUX_DMAMUX0_INDEX] |= TRGMUX_TRGMUXn_SEL1(elecStruct[electrodeNum].ELEC_TRGMUX);								// adcX_COCO[0] flag triggers DMA_Ch1
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(elecStruct[electrodeNum].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);		// enable DMAMUX

	DMA->CDNE = DMA_CDNE_CADN_MASK;																										// Clear DONE status bit
	DMA->CERQ = DMA_CERQ_CERQ(1);																										// Enable request for CH1
	DMA->CERR = DMA_CERR_CAEI_MASK;																										// Clear errors

	// DMA channel 0 config
	DMA->TCD[0].SADDR = (uint32_t) stcd_ch0[electrodeNum][0].SADDR;  																	// The 1st TCD Source Address
	DMA->TCD[0].DADDR = (uint32_t) stcd_ch0[electrodeNum][0].DADDR;     																// Destination Address of the buffer
	DMA->TCD[0].DLASTSGA = (uint32_t)(&stcd_ch0[electrodeNum][1]);    																// Scatter-Gather link to the next TCD
	DMA->TCD[0].CSR = DMA_TCD_CSR_ESG(1);																								// The current channel is in scatter-gather mode

	// DMA channel 1 config
	DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(sensingCycles) |					// Current iteration counter is set to required samples quantity
							   DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|																	// The minor channel-to-channel linking is enabled
							   DMA_TCD_CITER_ELINKYES_LINKCH(0);																		// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 						// Starting major iteration count is set to required samples quantity
							   DMA_TCD_BITER_ELINKNO_ELINK(0) ;  																		// The major channel-to-channel linking is disabled

	DMA->TCD[1].SADDR = (uint32_t) &(elecStruct[electrodeNum].adcBasePtr->R[0]);  													// Address of the ADC result
	#if (WAKE_UP_ELECTRODE_PROXIMITY == WAKE_UP_ELECTRODE_PROXIMITY_YES)
	if (electrodeNum == WAKE_UP_ELECTRODE)
	{
		DMA->TCD[1].DADDR = (uint32_t) &adcDataElectrodeDischargeRawSampleProximity[0];     											// Destination Address of the storage array
	}
	else
	{
		DMA->TCD[1].DADDR = (uint32_t) &adcDataElectrodeDischargeRawSample[0];     														// Destination Address of the storage array
	}
	#else
	DMA->TCD[1].DADDR = (uint32_t) &adcDataElectrodeDischargeRawSample[0];     														// Destination Address of the storage array
	#endif
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    						// Destination last address adjustment -2 bytes x required samples quantity

#if (SLIDER_ENABLE && NUMBER_OF_USED_ADC_MODULES == 2)
	DMA->TCD[1].CSR = DMA_TCD_CSR_BWC(0)         |         																	// BWC=0: No eDMA engine stalls - full bandwidth
		                    DMA_TCD_CSR_MAJORELINK(0)  |        																// The channel-to-channel linking is enabled
		                    DMA_TCD_CSR_MAJORLINKCH(0) |       																	// channel 0 will be called from ch1
		                    DMA_TCD_CSR_ESG(0)         |         																// The current channel TCD mode is normal
		                    DMA_TCD_CSR_DREQ(1)        |         																// The channel's ERQ bit will be cleared to prevent triggering from ADC side using.
		                    DMA_TCD_CSR_INTHALF(0)     |         																// The half-point interrupt is disabled
		                    DMA_TCD_CSR_INTMAJOR(1)    |         																// The end-of-major loop interrupt is enabled
		                    DMA_TCD_CSR_START(0);                																// The channel is not explicitly started
#endif
	// CH1 enable request
	DMA->SERQ = DMA_SERQ_SERQ(1);

	// Start DMA
	DMA->TCD[0].CSR |= DMA_TCD_CSR_START(1);
}

/*****************************************************************************
 *
 * Function: void DMA_init(void)
 *
 * Description: DMA init
 *
 *****************************************************************************/
void DMA_init(int16_t sensingCycles)
{
	DMA->CR |= DMA_CR_CLM(0) |								// cont.link mode
			   DMA_CR_CX(0) |								// cancel transf
			   DMA_CR_ECX(0) |								// error cancel transfer
			   DMA_CR_EDBG(0) |								// enable debug
			   DMA_CR_EMLM(0) |								// enable minor loop maping
			   DMA_CR_ERCA(0) |								// enable round robin channel arbitration
			   DMA_CR_HALT(0) |								// halt DMA operations
			   DMA_CR_HOE(0);								// halt on error

	// DMA ch0 TCD config
	DMA->TCD[0].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  					// Clear Channel Done flag
	DMA->TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(1) | 				// Starting major iteration count is 1
							    DMA_TCD_BITER_ELINKNO_ELINK(0);  				// The minor channel-to-channel linking is disabled
	DMA->TCD[0].SOFF = DMA_TCD_SOFF_SOFF(0);                					// Source Offset
	DMA->TCD[0].ATTR = DMA_TCD_ATTR_SMOD(0)  |              					// Source address modulo feature is disabled
					   DMA_TCD_ATTR_SSIZE(2) |              					// Source data transfer size: 1: 16-bit, 2=32-bit
					   DMA_TCD_ATTR_DMOD(0)  |              					// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
					   DMA_TCD_ATTR_DSIZE(2);               					// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[0].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(4);					// Minor Byte Transfer Count is 4-bytes
	DMA->TCD[0].SLAST = DMA_TCD_SLAST_SLAST(0);             					// Last Source Address Adjustment is 0
	DMA->TCD[0].DOFF = DMA_TCD_DOFF_DOFF(0);                					// Destination Address Signed Offset is 0
	DMA->TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(1) | 				// Current Major Iteration Count is 1
							    DMA_TCD_CITER_ELINKNO_ELINK(0);  				// The channel-to-channel linking is disabled
	DMA->TCD[0].CSR = DMA_TCD_CSR_BWC(0)         |         						// BWC=0: No eDMA engine stalls - full bandwidth
					  DMA_TCD_CSR_MAJORELINK(0)  |        						// The channel-to-channel linking is enabled
					  DMA_TCD_CSR_MAJORLINKCH(0) |       						// no channel linking
					  DMA_TCD_CSR_ESG(1)         |         						// The current channel's TCD is in scatter/gather mode
					  DMA_TCD_CSR_DREQ(0)        |         						// The channel's ERQ bit is not affected
					  DMA_TCD_CSR_INTHALF(0)     |         						// The half-point interrupt is disabled
					  DMA_TCD_CSR_INTMAJOR(0)    |         						// The end-of-major loop interrupt is enabled
					  DMA_TCD_CSR_START(0);                						// The channel is not explicitly started

	// DMA channel 1 config
	DMA->TCD[1].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  																// Clear Channel Done flag
	DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(sensingCycles) |		// Current iteration count is 4^n (oversampling to 12+n bits ADC resolution)
			  	  	  	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|													// The minor channel-to-channel linking is enabled
								   DMA_TCD_CITER_ELINKYES_LINKCH(0);														// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 			// Starting major iteration count is 4^n (oversampling to 12+n bits ADC resolution)
	                               DMA_TCD_BITER_ELINKNO_ELINK(0) ;  														// The major channel-to-channel linking is disabled
	DMA->TCD[1].SOFF = DMA_TCD_SOFF_SOFF(0);                																// Source Offset
	DMA->TCD[1].ATTR = DMA_TCD_ATTR_SMOD(0)  |              																// Source address modulo feature is disabled
	                    DMA_TCD_ATTR_SSIZE(1) |              																// Source data transfer size: 1: 16-bit, 2=32-bit
	                    DMA_TCD_ATTR_DMOD(0)  |              																// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
	                    DMA_TCD_ATTR_DSIZE(1);               																// Destination data transfer size: 1: 16-bit, 2=32-bit
	DMA->TCD[1].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(2);
	DMA->TCD[1].SLAST = DMA_TCD_SLAST_SLAST(0);             																// Last Source Address Adjustment is 0
	DMA->TCD[1].DOFF = DMA_TCD_DOFF_DOFF(2);                																// Destination Address Signed Offset is 2
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    			// Destination last address adjustment 2 bytes x 256
	DMA->TCD[1].CSR = DMA_TCD_CSR_BWC(0)         |         																	// BWC=0: No eDMA engine stalls - full bandwidth
	                    DMA_TCD_CSR_MAJORELINK(0)  |        																// The channel-to-channel linking is enabled
	                    DMA_TCD_CSR_MAJORLINKCH(0) |       																	// channel 0 will be called from ch1
	                    DMA_TCD_CSR_ESG(0)         |         																// The current channel TCD mode is normal
	                    DMA_TCD_CSR_DREQ(1)        |         																// The channel's ERQ bit will be cleared to prevent triggering from ADC side using.
	                    DMA_TCD_CSR_INTHALF(0)     |         																// The half-point interrupt is disabled
	                    DMA_TCD_CSR_INTMAJOR(1)    |         																// The end-of-major loop interrupt is enabled
	                    DMA_TCD_CSR_START(0);                																// The channel is not explicitly started

#if (SLIDER_ENABLE && NUMBER_OF_USED_ADC_MODULES == 2)

	// DMA channel 2 config
		DMA->TCD[2].CSR &= 0xFFFFFFFF ^ DMA_TCD_CSR_DONE_MASK;  																// Clear Channel Done flag
		DMA->TCD[2].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(sensingCycles) |		// Current iteration count is 4^n (oversampling to 12+n bits ADC resolution)
				  	  	  	  	  	   DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|													// The minor channel-to-channel linking is enabled
									   DMA_TCD_CITER_ELINKYES_LINKCH(0);														// The minor channel-to-channel link is to Ch 0
		DMA->TCD[2].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 			// Starting major iteration count is 4^n (oversampling to 12+n bits ADC resolution)
		                               DMA_TCD_BITER_ELINKNO_ELINK(0) ;  														// The major channel-to-channel linking is disabled
		DMA->TCD[2].SOFF = DMA_TCD_SOFF_SOFF(0);                																// Source Offset
		DMA->TCD[2].ATTR = DMA_TCD_ATTR_SMOD(0)  |              																// Source address modulo feature is disabled
		                    DMA_TCD_ATTR_SSIZE(1) |              																// Source data transfer size: 1: 16-bit, 2=32-bit
		                    DMA_TCD_ATTR_DMOD(0)  |              																// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
		                    DMA_TCD_ATTR_DSIZE(1);               																// Destination data transfer size: 1: 16-bit, 2=32-bit
		DMA->TCD[2].NBYTES.MLNO = DMA_TCD_NBYTES_MLNO_NBYTES(2);
		DMA->TCD[2].SLAST = DMA_TCD_SLAST_SLAST(0);             																// Last Source Address Adjustment is 0
		DMA->TCD[2].DOFF = DMA_TCD_DOFF_DOFF(2);                																// Destination Address Signed Offset is 2
		DMA->TCD[2].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    			// Destination last address adjustment 2 bytes x 256
		DMA->TCD[2].CSR = DMA_TCD_CSR_BWC(0)         |         																	// BWC=0: No eDMA engine stalls - full bandwidth
		                    DMA_TCD_CSR_MAJORELINK(0)  |        																// The channel-to-channel linking is enabled
		                    DMA_TCD_CSR_MAJORLINKCH(0) |       																	// channel 0 will be called from ch1
		                    DMA_TCD_CSR_ESG(0)         |         																// The current channel TCD mode is normal
		                    DMA_TCD_CSR_DREQ(1)        |         																// The channel's ERQ bit will be cleared to prevent triggering from ADC side using.
		                    DMA_TCD_CSR_INTHALF(0)     |         																// The half-point interrupt is disabled
		                    DMA_TCD_CSR_INTMAJOR(1)    |         																// The end-of-major loop interrupt is enabled
		                    DMA_TCD_CSR_START(0);                																// The channel is not explicitly started
#endif
}

/*****************************************************************************
 *
 * Function: void TCD_Elec_Init(void)
 *
 * Description: TCD electrodes structures init
 *
 *****************************************************************************/
void TCD_Keypads_Init(void)
{
	// Scatter-Gather parameters configuration for electrodes
	for (elecNum = 0; elecNum < NUMBER_OF_ELECTRODES; elecNum++)
	{
		//SADDR_CONFIG - value to write to the register
		//DADDR_CONFIG - address of the register

#if (GUARD) // Guard pin as guard
		// Electrode Pin = 0;
		SADDR_CONFIG[elecNum][0] = (uint32_t) (1 << elecStruct[elecNum].pinNumberElec);
	    DADDR_CONFIG[elecNum][0] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PCOR;
	    // Electrode Pin as GPIO
		SADDR_CONFIG[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG[elecNum][1] = (uint32_t *) &elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberElec];
	    // Cext Pin = 1;
	    SADDR_CONFIG[elecNum][2] = (uint32_t) (1 << elecStruct[elecNum].pinNumberCext);
	    DADDR_CONFIG[elecNum][2] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PSOR;
	    // Cext Pin as GPIO
	    SADDR_CONFIG[elecNum][3] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG[elecNum][3] = (uint32_t *) &elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG[elecNum][4] = (uint32_t) ((elecStruct[elecNum].gpioBasePtr->PDDR) | (elecStruct[elecNum].portMask));
	    DADDR_CONFIG[elecNum][4] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;
	    // Guard pin = 0
	    SADDR_CONFIG[elecNum][5] = (uint32_t) (1 << guardStruct[0].pinNumberGuard);
	    DADDR_CONFIG[elecNum][5] = (uint32_t *) &guardStruct[0].gpioBasePtr->PCOR;
	    // ADC read Cext pin voltage
	    SADDR_CONFIG[elecNum][6] = (uint32_t) ADC_SC1_ADCH(elecStruct[elecNum].adcChNum);
	    DADDR_CONFIG[elecNum][6] = (uint32_t *) &elecStruct[elecNum].adcBasePtr->SC1[0];
		// Elec and Cext electrodes as input
	    SADDR_CONFIG[elecNum][7] = (uint32_t) (elecStruct[elecNum].gpioBasePtr->PDDR) & ~(elecStruct[elecNum].portMask);
	    DADDR_CONFIG[elecNum][7] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;

#else // No guard
		//SADDR_CONFIG - value to write to the register
		//DADDR_CONFIG - address of the register

		// Electrode Pin = 0;
		SADDR_CONFIG[elecNum][0] = (uint32_t) (1 << elecStruct[elecNum].pinNumberElec);
	    DADDR_CONFIG[elecNum][0] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PCOR;
	    // Electrode Pin as GPIO
		SADDR_CONFIG[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG[elecNum][1] = (uint32_t *) &elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberElec];
	    // Cext Pin = 1;
	    SADDR_CONFIG[elecNum][2] = (uint32_t) (1 << elecStruct[elecNum].pinNumberCext);
	    DADDR_CONFIG[elecNum][2] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PSOR;
	    // Cext Pin as GPIO
	    SADDR_CONFIG[elecNum][3] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG[elecNum][3] = (uint32_t *) &elecStruct[elecNum].portBasePtr->PCR[elecStruct[elecNum].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG[elecNum][4] = (uint32_t) ((elecStruct[elecNum].gpioBasePtr->PDDR) | (elecStruct[elecNum].portMask));
	    DADDR_CONFIG[elecNum][4] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;
	    // ADC read Cext pin voltage
	    SADDR_CONFIG[elecNum][5] = (uint32_t) ADC_SC1_ADCH(elecStruct[elecNum].adcChNum);
	    DADDR_CONFIG[elecNum][5] = (uint32_t *) &elecStruct[elecNum].adcBasePtr->SC1[0];
		// Elec and Cext electrodes as input
	    SADDR_CONFIG[elecNum][6] = (uint32_t) (elecStruct[elecNum].gpioBasePtr->PDDR) & ~(elecStruct[elecNum].portMask);
	    DADDR_CONFIG[elecNum][6] = (uint32_t *) &elecStruct[elecNum].gpioBasePtr->PDDR;

#endif
	    // Scatter / gather TCDs configuration for each electrode
		for (tcdInstructionNum = 0; tcdInstructionNum < 8; tcdInstructionNum++)
		{
			stcd_ch0[elecNum][tcdInstructionNum].CSR = DMA_TCD_CSR_BWC(0)         |         				// BWC=0: No eDMA engine stalls - full bandwidth
								 DMA_TCD_CSR_MAJORELINK(0)  |        				// The channel-to-channel linking is enabled
								 DMA_TCD_CSR_MAJORLINKCH(0) |       				// no channel linking
								 DMA_TCD_CSR_ESG(1)         |         				// The current channel's TCD is in scatter/gather mode
								 DMA_TCD_CSR_DREQ(0)        |         				// The channel's ERQ bit is not affected
								 DMA_TCD_CSR_INTHALF(0)     |         				// The half-point interrupt is disabled
								 DMA_TCD_CSR_INTMAJOR(0)    |         				// The end-of-major loop interrupt is enabled
								 DMA_TCD_CSR_START(1);                				// The channel is not explicitly started
			stcd_ch0[elecNum][tcdInstructionNum].BITER = DMA_TCD_BITER_ELINKNO_BITER(1) | 				// Starting major iteration count is 1
							       DMA_TCD_BITER_ELINKNO_ELINK(0);  				// The minor channel-to-channel linking is disabled
			stcd_ch0[elecNum][tcdInstructionNum].SADDR = (uint32_t) &SADDR_CONFIG[elecNum][tcdInstructionNum];					// Source address
			stcd_ch0[elecNum][tcdInstructionNum].SOFF = DMA_TCD_SOFF_SOFF(0);                				// Source Offset
			stcd_ch0[elecNum][tcdInstructionNum].ATTR = DMA_TCD_ATTR_SMOD(0)  |              				// Source address modulo feature is disabled
								  DMA_TCD_ATTR_SSIZE(2) |              				// Source data transfer size: 1: 16-bit, 2=32-bit
								  DMA_TCD_ATTR_DMOD(0)  |              				// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
								  DMA_TCD_ATTR_DSIZE(2);               				// Destination data transfer size: 1: 16-bit, 2=32-bit
			stcd_ch0[elecNum][tcdInstructionNum].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(4); 					// Minor Byte Transfer Count is 1-byte
			stcd_ch0[elecNum][tcdInstructionNum].SLAST = DMA_TCD_SLAST_SLAST(0);     						// Last Source Address Adjustment is 0
			stcd_ch0[elecNum][tcdInstructionNum].DADDR = (uint32_t) DADDR_CONFIG[elecNum][tcdInstructionNum];					// Destination address
			stcd_ch0[elecNum][tcdInstructionNum].DOFF = DMA_TCD_DOFF_DOFF(0);                				// Destination Address Signed Offset is 0
			stcd_ch0[elecNum][tcdInstructionNum].CITER = DMA_TCD_CITER_ELINKNO_CITER(1) | 				// Current Major Iteration Count is 1
							  	   DMA_TCD_CITER_ELINKNO_ELINK(0);  				// The channel-to-channel linking is disabled
			stcd_ch0[elecNum][tcdInstructionNum].DLAST_SGA = (uint32_t) &stcd_ch0[elecNum][tcdInstructionNum+1];					// Scatter gather link to the next TCD

		}
		stcd_ch0[elecNum][0].CSR &= ~DMA_TCD_CSR_START_MASK;								// DMA channel 0 autostart off
#if (GUARD)
		stcd_ch0[elecNum][7].DLASTSGA = (uint32_t)(&stcd_ch0[elecNum][0]);    					// Scatter Gather loop to the 1st TCD
		// Guard pin = 1,  delay 60ns
		stcd_ch0[elecNum][7].SOFF = DMA_TCD_SOFF_SOFF(((uint32_t) &guardStruct[0].portMask)-((uint32_t) &SADDR_CONFIG[elecNum][7])); // Source Offset to immediately jump on next source address - guard portmask source address
		stcd_ch0[elecNum][7].DOFF = DMA_TCD_DOFF_DOFF(((uint32_t) &guardStruct[0].gpioBasePtr->PSOR)-((uint32_t) DADDR_CONFIG[elecNum][7])); // Destination Address Offset to immediately jump to next destination address - guard pin PSOR
		stcd_ch0[elecNum][7].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(8); // Minor Byte Transfer Count is 2-byte
#else
		stcd_ch0[elecNum][6].DLAST_SGA = (uint32_t)(&stcd_ch0[elecNum][0]);    					// Scatter Gather loop to the 1st TCD
#endif
	}
}

#if SLIDER_ENABLE
/*****************************************************************************
*
* Function: void start_TS_SLIDER_DMA(uint32_t electrodeNum)
*
* Description: Starts DMA touch sense algorithm for Slider electrodeNum
*
*****************************************************************************/
void start_TS_SLIDER_DMA(uint32_t electrodeNum, int16_t sensingCycles)
{
	// Reconfigure DMA touch sense engine for the electrodeNum

	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																								// disable DMAMUX
	TRGMUX->TRGMUXn[TRGMUX_DMAMUX0_INDEX] |= TRGMUX_TRGMUXn_SEL1(sliderElecStruct[electrodeNum].ELEC_TRGMUX);							// adcX_COCO[0] triggers DMA_Ch1
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(sliderElecStruct[electrodeNum].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// enable DMAMUX

	DMA->CDNE = DMA_CDNE_CADN_MASK;																								// Clear DONE status bit
	DMA->CERQ = DMA_CERQ_CERQ(1);																								// Enable request for CH1
	DMA->CERR = DMA_CERR_CAEI_MASK;																								// Clear errors

	// DMA channel 0 config
	DMA->TCD[0].SADDR = (uint32_t) slider_stcd_ch0[electrodeNum][0].SADDR;  													// The 1st TCD Source Address
	DMA->TCD[0].DADDR = (uint32_t) slider_stcd_ch0[electrodeNum][0].DADDR;     												// Destination Address of the buffer
	DMA->TCD[0].DLASTSGA = (uint32_t)(&slider_stcd_ch0[electrodeNum][1]);    												// Scatter-Gather link to the next TCD
	DMA->TCD[0].CSR = DMA_TCD_CSR_ESG(1);																						// The current channel is in scatter-gather mode

	// DMA channel 1 config
	DMA->TCD[1].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(sensingCycles) |			// Current iteration counter is set to required samples quantity
							   DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|															// The minor channel-to-channel linking is enabled
							   DMA_TCD_CITER_ELINKYES_LINKCH(0);																// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 				// Starting major iteration count is set to required samples quantity
							   DMA_TCD_BITER_ELINKNO_ELINK(0) ;  																// The major channel-to-channel linking is disabled

	DMA->TCD[1].SADDR = (uint32_t) &(sliderElecStruct[electrodeNum].adcBasePtr->R[0]);  										// Address of the ADC result
	DMA->TCD[1].DADDR = (uint32_t) &sliderAdcDataElectrodeDischargeRawSample[0];     											// Destination Address of the storage array
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    				// Destination last address adjustment -2 bytes x required samples quantity

	// CH1 enable request
	DMA->SERQ = DMA_SERQ_SERQ(1);
	// Start DMA
	DMA->TCD[0].CSR |= DMA_TCD_CSR_START(1);
}

/*****************************************************************************
 *
 * Function: void start_TS_SLIDER_Simultaneous_DMA(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num, int16_t sensingCycles)
 *
 * Description: Starts DMA touch sense algorithm for both Slider electrodes simultaneously
 *
 *****************************************************************************/
void start_TS_SLIDER_Simultaneous_DMA(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num, int16_t sensingCycles)
{
	// Reconfigure DMA touch sense engine for the electrodeNum

	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_ENBL(0);																								// disable DMAMUX
	TRGMUX->TRGMUXn[TRGMUX_DMAMUX0_INDEX] |= TRGMUX_TRGMUXn_SEL1(sliderElecStruct[sliderElectrode0Num].ELEC_TRGMUX);							// adcX_COCO[0] triggers DMA_Ch1
	DMAMUX->CHCFG[1] = DMAMUX_CHCFG_SOURCE(sliderElecStruct[sliderElectrode0Num].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// enable DMAMUX

	// DMA triggering by electrode's ADC config
	DMAMUX->CHCFG[2] = DMAMUX_CHCFG_ENBL(0);																								// disable DMAMUX
	TRGMUX->TRGMUXn[TRGMUX_DMAMUX0_INDEX] |= TRGMUX_TRGMUXn_SEL2(sliderElecStruct[sliderElectrode1Num].ELEC_TRGMUX);							// adcX_COCO[0] triggers DMA_Ch1
	DMAMUX->CHCFG[2] = DMAMUX_CHCFG_SOURCE(sliderElecStruct[sliderElectrode1Num].ELEC_DMAMUX) | DMAMUX_CHCFG_TRIG(0) | DMAMUX_CHCFG_ENBL(1);	// enable DMAMUX

	DMA->CDNE = DMA_CDNE_CADN_MASK;																								// Clear DONE status bit
	DMA->CERQ = DMA_CERQ_CERQ(1);																								// Enable request for CH1
	DMA->CERR = DMA_CERR_CAEI_MASK;																								// Clear errors

	// DMA channel 0 config
	DMA->TCD[0].SADDR = (uint32_t) slider_stcd_ch0[sliderElectrode0Num][0].SADDR;  													// The 1st TCD Source Address
	DMA->TCD[0].DADDR = (uint32_t) slider_stcd_ch0[sliderElectrode0Num][0].DADDR;     												// Destination Address of the buffer
	DMA->TCD[0].DLASTSGA = (uint32_t)(&slider_stcd_ch0[sliderElectrode0Num][1]);    												// Scatter-Gather link to the next TCD
	DMA->TCD[0].CSR = DMA_TCD_CSR_ESG(1);																						// The current channel is in scatter-gather mode

	// DMA channel 1 config
	DMA->TCD[1].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO_CITER(sensingCycles) |			// Current iteration counter is set to required samples quantity
			DMA_TCD_CITER_ELINKNO_ELINK(0)	 	|															// The minor channel-to-channel linking is enabled
			DMA_TCD_CITER_ELINKYES_LINKCH(0);																// The minor channel-to-channel link is to Ch 0
	DMA->TCD[1].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 				// Starting major iteration count is set to required samples quantity
			DMA_TCD_BITER_ELINKNO_ELINK(0) ;  																// The major channel-to-channel linking is disabled

	DMA->TCD[1].SADDR = (uint32_t) &(sliderElecStruct[sliderElectrode0Num].adcBasePtr->R[0]);  										// Address of the ADC result
	DMA->TCD[1].DADDR = (uint32_t) &sliderAdcDataElectrodeDischargeRawSampleElec0[0];     											// Destination Address of the storage array
	DMA->TCD[1].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    				// Destination last address adjustment -2 bytes x required samples quantity

	DMA->TCD[1].CSR = DMA_TCD_CSR_BWC(0)         |         																	// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(0)  |        																// The channel-to-channel linking is enabled
			DMA_TCD_CSR_MAJORLINKCH(0) |       																	// channel 0 will be called from ch1
			DMA_TCD_CSR_ESG(0)         |         																// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(1)        |         																// The channel's ERQ bit will be cleared to prevent triggering from ADC side using.
			DMA_TCD_CSR_INTHALF(0)     |         																// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(0)    |         																// The end-of-major loop interrupt is enabled
			DMA_TCD_CSR_START(0);                																// The channel is not explicitly started

	// DMA channel 2 config
	DMA->TCD[2].CITER.ELINKYES = DMA_TCD_CITER_ELINKYES_CITER_LE(sensingCycles) |			// Current iteration counter is set to required samples quantity
			DMA_TCD_CITER_ELINKYES_ELINK(1)	 	|															// The minor channel-to-channel linking is enabled
			DMA_TCD_CITER_ELINKYES_LINKCH(0);																// The minor channel-to-channel link is to Ch 0
	DMA->TCD[2].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO_BITER(sensingCycles) | 				// Starting major iteration count is set to required samples quantity
			DMA_TCD_BITER_ELINKNO_ELINK(0);

	DMA->TCD[2].SADDR = (uint32_t) &(sliderElecStruct[sliderElectrode1Num].adcBasePtr->R[0]);  										// Address of the ADC result
	DMA->TCD[2].DADDR = (uint32_t) &sliderAdcDataElectrodeDischargeRawSampleElec1[0];     											// Destination Address of the storage array
	DMA->TCD[2].DLASTSGA = DMA_TCD_DLASTSGA_DLASTSGA(-2*sensingCycles);    				// Destination last address adjustment -2 bytes x required samples quantity

	DMA->TCD[2].CSR = DMA_TCD_CSR_BWC(0)         |         																	// BWC=0: No eDMA engine stalls - full bandwidth
			DMA_TCD_CSR_MAJORELINK(0)  |        																// The channel-to-channel linking is enabled
			DMA_TCD_CSR_MAJORLINKCH(0) |       																	// channel 0 will be called from ch1
			DMA_TCD_CSR_ESG(0)         |         																// The current channel TCD mode is normal
			DMA_TCD_CSR_DREQ(1)        |         																// The channel's ERQ bit will be cleared to prevent triggering from ADC side using.
			DMA_TCD_CSR_INTHALF(0)     |         																// The half-point interrupt is disabled
			DMA_TCD_CSR_INTMAJOR(1)    |         																// The end-of-major loop interrupt is enabled
			DMA_TCD_CSR_START(0);																				// The channel is not explicitly started

	// CH1 enable request
	DMA->SERQ = DMA_SERQ_SERQ(1);
	// CH2 enable request
	DMA->SERQ = DMA_SERQ_SERQ(2);

	// Start DMA
	DMA->TCD[0].CSR |= DMA_TCD_CSR_START(1);
}

/*****************************************************************************
 *
 * Function: void TCD_Slider_Init(void)
 *
 * Description: TCD Slider electrodes structures init
 *
 *****************************************************************************/
void TCD_Slider_Init(void)
{
	uint8_t sliderElectrode0Num = 0, sliderElectrode1Num = 1;
	// Scatter-Gather parameters configuration for Slider pads
	for (elecNum = 0; elecNum < NUMBER_OF_SLIDER_ELECTRODES; elecNum++)
	{
		//SADDR_CONFIG - value to write to the register
		//DADDR_CONFIG - address of the register

#if (NUMBER_OF_USED_ADC_MODULES == 1)
	#if (GUARD) // Guard pin as guard
		// Electrodes Pins = 0;
		SADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberElec | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberElec);
	    DADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PCOR;
	    // Electrode0 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberElec];
	    // Electrode1 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberElec];
	    // Cexts Pins = 1;
	    SADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberCext | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberCext);
	    DADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PSOR;
	    // Cext0 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberCext];
	    // Cext1 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t) ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) | (sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask));
	    DADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
	    // Guard pin = 0
	    SADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t) (1 << guardStruct[0].pinNumberGuard);
	    DADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t *) &guardStruct[0].gpioBasePtr->PCOR;
	    // ADC read Cext pin voltage
	    SADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[elecNum].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t *) &sliderElecStruct[elecNum].adcBasePtr->SC1[0];
	    // Elec and Cext electrodes as input
	    SADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t) (sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask);
	    DADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;

	#else // No guard
		// Electrodes Pins = 0;
		SADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberElec | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberElec);
	    DADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PCOR;
	    // Electrode0 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberElec];
	    // Electrode1 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberElec];
	    // Cexts Pins = 1;
	    SADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberCext | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberCext);
	    DADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PSOR;
	    // Cext0 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberCext];
	    // Cext1 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t) ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) | (sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask));
	    DADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
	    // ADC read Cext pin voltage
	    SADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[elecNum].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t *) &sliderElecStruct[elecNum].adcBasePtr->SC1[0];
	    // Elec and Cext electrodes as input
	    SADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t) (sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask);
	    DADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;

	#endif
#elif (NUMBER_OF_USED_ADC_MODULES == 2)

	  #if (GUARD) // Guard pin as guard
		// Electrodes Pins = 0;
		SADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberElec | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberElec);
	    DADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PCOR;
	    // Electrode0 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberElec];
	    // Electrode1 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberElec];
	    // Cexts Pins = 1;
	    SADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberCext | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberCext);
	    DADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PSOR;
	    // Cext0 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberCext];
	    // Cext1 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t) ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) | (sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask));
	    DADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
	    // Guard pin = 0
	    SADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t) (1 << guardStruct[0].pinNumberGuard);
	    DADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t *) &guardStruct[0].gpioBasePtr->PCOR;
	    // Cext voltage conversion - channel PRE-assignment for both slider electrodes
	    SADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[sliderElectrode0Num].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].adcBasePtr->SC1[0];
	    SADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[sliderElectrode1Num].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].adcBasePtr->SC1[0];
	    // Generate SW trigger to TRGMUX -> and thus initiate the Cext voltage conversion in both ADC0 and ADC1 at the same time
	    SADDR_CONFIG_SLIDER[elecNum][10] = (uint32_t) SIM_MISCTRL1_SW_TRG_MASK;
	    DADDR_CONFIG_SLIDER[elecNum][10] = (uint32_t *) &(SIM->MISCTRL1);
	    // Elec and Cext electrodes as input
	    SADDR_CONFIG_SLIDER[elecNum][11] = (uint32_t) (sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask);
	    DADDR_CONFIG_SLIDER[elecNum][11] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
		// Clear the SIM_SW_trigger flag to TRGMUX
	    SADDR_CONFIG_SLIDER[elecNum][12] = (uint32_t) SIM_MISCTRL1_SW_TRG(0);
	    DADDR_CONFIG_SLIDER[elecNum][12] = (uint32_t *) &(SIM->MISCTRL1);

	 #else // No guard
		// Electrodes Pins = 0;
		SADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberElec | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberElec);
	    DADDR_CONFIG_SLIDER[elecNum][0] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PCOR;
	    // Electrode0 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][1] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberElec];
	    // Electrode1 Pin as GPIO
		SADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][2] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberElec];
	    // Cexts Pins = 1;
	    SADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t) (1 << sliderElecStruct[sliderElectrode0Num].pinNumberCext | 1 << sliderElecStruct[sliderElectrode1Num].pinNumberCext);
	    DADDR_CONFIG_SLIDER[elecNum][3] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PSOR;
	    // Cext0 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][4] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode0Num].pinNumberCext];
	    // Cext1 Pin as GPIO
	    SADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t) PORT_PCR_MUX(0b001);
	    DADDR_CONFIG_SLIDER[elecNum][5] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].portBasePtr->PCR[sliderElecStruct[sliderElectrode1Num].pinNumberCext];
	    // Electrode and Cext pins = output
	    SADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t) ((sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) | (sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask));
	    DADDR_CONFIG_SLIDER[elecNum][6] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
	    // Cext voltage conversion - channel PRE-assignment for both slider electrodes
	    SADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[sliderElectrode0Num].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][7] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].adcBasePtr->SC1[0];
	    SADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t) ADC_SC1_ADCH(sliderElecStruct[sliderElectrode1Num].adcChNum);
	    DADDR_CONFIG_SLIDER[elecNum][8] = (uint32_t *) &sliderElecStruct[sliderElectrode1Num].adcBasePtr->SC1[0];
	    // Generate SW trigger to TRGMUX -> and thus initiate the Cext voltage conversion in both ADC0 and ADC1 at the same time
	    SADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t) SIM_MISCTRL1_SW_TRG_MASK;
	    DADDR_CONFIG_SLIDER[elecNum][9] = (uint32_t *) &(SIM->MISCTRL1);
	    // Elec and Cext electrodes as input
	    SADDR_CONFIG_SLIDER[elecNum][10] = (uint32_t) (sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR) & ~(sliderElecStruct[sliderElectrode0Num].portMask + sliderElecStruct[sliderElectrode1Num].portMask);
	    DADDR_CONFIG_SLIDER[elecNum][10] = (uint32_t *) &sliderElecStruct[sliderElectrode0Num].gpioBasePtr->PDDR;
		// Clear the SIM_SW_trigger flag to TRGMUX
	    SADDR_CONFIG_SLIDER[elecNum][11] = (uint32_t) SIM_MISCTRL1_SW_TRG(0);
	    DADDR_CONFIG_SLIDER[elecNum][11] = (uint32_t *) &(SIM->MISCTRL1);
	#endif
#endif
		for (tcdInstructionNum = 0; tcdInstructionNum < 13; tcdInstructionNum++)
		{
			slider_stcd_ch0[elecNum][tcdInstructionNum].CSR = DMA_TCD_CSR_BWC(0)         |         		// BWC=0: No eDMA engine stalls - full bandwidth
								 DMA_TCD_CSR_MAJORELINK(0)  |        				// The channel-to-channel linking is enabled
								 DMA_TCD_CSR_MAJORLINKCH(0) |       				// no channel linking
								 DMA_TCD_CSR_ESG(1)         |         				// The current channel's TCD is in scatter/gather mode
								 DMA_TCD_CSR_DREQ(0)        |         				// The channel's ERQ bit is not affected
								 DMA_TCD_CSR_INTHALF(0)     |         				// The half-point interrupt is disabled
								 DMA_TCD_CSR_INTMAJOR(0)    |         				// The end-of-major loop interrupt is enabled
								 DMA_TCD_CSR_START(1);                				// The channel is not explicitly started
			slider_stcd_ch0[elecNum][tcdInstructionNum].BITER = DMA_TCD_BITER_ELINKNO_BITER(1) | 			// Starting major iteration count is 1
							       DMA_TCD_BITER_ELINKNO_ELINK(0);  				// The minor channel-to-channel linking is disabled
			slider_stcd_ch0[elecNum][tcdInstructionNum].SADDR = (uint32_t) &SADDR_CONFIG_SLIDER[elecNum][tcdInstructionNum];	// Source address
			slider_stcd_ch0[elecNum][tcdInstructionNum].SOFF = DMA_TCD_SOFF_SOFF(0);                		// Source Offset
			slider_stcd_ch0[elecNum][tcdInstructionNum].ATTR = DMA_TCD_ATTR_SMOD(0)  |              		// Source address modulo feature is disabled
								  DMA_TCD_ATTR_SSIZE(2) |              				// Source data transfer size: 1: 16-bit, 2=32-bit
								  DMA_TCD_ATTR_DMOD(0)  |              				// Destination address modulo feature: 0=disabled, x= x power of 2 buffer[DMOD=4->buffer of 16bytes]
								  DMA_TCD_ATTR_DSIZE(2);               				// Destination data transfer size: 1: 16-bit, 2=32-bit
			slider_stcd_ch0[elecNum][tcdInstructionNum].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(4); 			// Minor Byte Transfer Count is 1-byte
			slider_stcd_ch0[elecNum][tcdInstructionNum].SLAST = DMA_TCD_SLAST_SLAST(0);     				// Last Source Address Adjustment is 0
			slider_stcd_ch0[elecNum][tcdInstructionNum].DADDR = (uint32_t) DADDR_CONFIG_SLIDER[elecNum][tcdInstructionNum];		// Destination address
			slider_stcd_ch0[elecNum][tcdInstructionNum].DOFF = DMA_TCD_DOFF_DOFF(0);                		// Destination Address Signed Offset is 0
			slider_stcd_ch0[elecNum][tcdInstructionNum].CITER = DMA_TCD_CITER_ELINKNO_CITER(1) | 			// Current Major Iteration Count is 1
							  	   DMA_TCD_CITER_ELINKNO_ELINK(0);  				// The channel-to-channel linking is disabled
			slider_stcd_ch0[elecNum][tcdInstructionNum].DLASTSGA = (uint32_t) &slider_stcd_ch0[elecNum][tcdInstructionNum+1];	// Scatter gather link to the next TCD
		}
		slider_stcd_ch0[elecNum][0].CSR &= ~DMA_TCD_CSR_START_MASK;						// DMA channel 0 autostart off
#if (NUMBER_OF_USED_ADC_MODULES == 1)
#if (GUARD)
		slider_stcd_ch0[elecNum][9].DLASTSGA = (uint32_t)(&slider_stcd_ch0[elecNum][0]);    	// SGA to the 1st STCD
		// Guard pin = 1,  delay 60ns
		slider_stcd_ch0[elecNum][9].SOFF = DMA_TCD_SOFF_SOFF(((uint32_t) &guardStruct[0].portMask)-((uint32_t) &SADDR_CONFIG_SLIDER[elecNum][9])); // Source Offset to immediately jump on next source address - guard portmask source address
		slider_stcd_ch0[elecNum][9].DOFF = DMA_TCD_DOFF_DOFF(((uint32_t) &guardStruct[0].gpioBasePtr->PSOR)-((uint32_t) DADDR_CONFIG_SLIDER[elecNum][9])); // Destination Address Offset to immediately jump to next destination address - guard pin PSOR
		slider_stcd_ch0[elecNum][9].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(8); // Minor Byte Transfer Count is 2-byte
	#else
		slider_stcd_ch0[elecNum][8].DLASTSGA = (uint32_t)(&slider_stcd_ch0[elecNum][0]);    	// SGA to the 1st STCD
	#endif
#elif (NUMBER_OF_USED_ADC_MODULES == 2)
	#if (GUARD)
		slider_stcd_ch0[elecNum][12].DLASTSGA = (uint32_t)(&slider_stcd_ch0[elecNum][0]);    	// SGA to the 1st STCD
		// Guard pin = 1,  delay 60ns
		slider_stcd_ch0[elecNum][11].SOFF = DMA_TCD_SOFF_SOFF(((uint32_t) &guardStruct[0].portMask)-((uint32_t) &SADDR_CONFIG_SLIDER[elecNum][11])); // Source Offset to immediately jump on next source address - guard portmask source address
		slider_stcd_ch0[elecNum][11].DOFF = DMA_TCD_DOFF_DOFF(((uint32_t) &guardStruct[0].gpioBasePtr->PSOR)-((uint32_t) DADDR_CONFIG_SLIDER[elecNum][11])); // Destination Address Offset to immediately jump to next destination address - guard pin PSOR
		slider_stcd_ch0[elecNum][11].NBYTES = DMA_TCD_NBYTES_MLNO_NBYTES(8); // Minor Byte Transfer Count is 2-byte
	#else
		slider_stcd_ch0[elecNum][11].DLASTSGA = (uint32_t)(&slider_stcd_ch0[elecNum][0]);    	// SGA to the 1st STCD
	#endif
#endif
	}

}
#endif
