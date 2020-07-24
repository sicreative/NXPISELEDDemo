/*
 * tcd.h
 *
 *  Created on: 10.12.2018
 *      Author: nxf50214
 */

#ifndef TCD_H_
#define TCD_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include "S32K144.h"
#include "edma_driver.h"

/*******************************************************************************
* Type defines
******************************************************************************/
//TCD for DMA Scatter - Gather implementation
//typedef struct
//{
//    uint32_t SADDR;
//    int16_t SOFF;
//    uint16_t ATTR;
//    uint32_t NBYTES;
//    int32_t SLAST;
//    uint32_t DADDR;
//    int16_t DOFF;
//    uint16_t CITER;
//    int32_t DLASTSGA;
//    uint16_t CSR;
//    uint16_t BITER;
//} edma_software_tcd_t;

/*******************************************************************************
* Function prototypes
******************************************************************************/
void DMA_init(int16_t sensingCycles);
void start_TS_DMA(uint32_t electrodeNum, int16_t sensingCycles);
void TCD_Keypads_Init(void);
void TCD_Slider_Init(void);
void start_TS_SLIDER_DMA(uint32_t electrodeNum,int16_t sensingCycles);
void start_TS_SLIDER_Simultaneous_DMA(uint32_t sliderElectrode0Num,uint32_t sliderElectrode1Num, int16_t sensingCycles);

#endif /* TCD_H_ */
