/****************************************************************************//*!
*
* @file     scg.h
*
* @version  1.0.0.0
*
* @date     September-2017
*
* @brief    SCG routines header file
*
*******************************************************************************/
#ifndef __SCG_H
#define __SCG_H

#include "S32K144.h"

/*******************************************************************************
* Defines
*******************************************************************************/
#define CORE_CLK 80000000

#if(CORE_CLK == 112000000)
/* SCGOUTCLK = 112MHz, 8MHz EXTAL */
#define PREDIV      0       /* EXTAL div by 1 (8 MHz) */
#define MUL         12      /* VCO div by 28 */
#define DIVCORE     0       /* Core clock / system clock = SPLLCLK / 1 (112 MHz) */
#define DIVBUS      1       /* Bus clock = SPLLCLK / 2 (56 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 5 (28 MHz) */
#define WAIT_STATES (DIVSLOW - DIVCORE)

#elif(CORE_CLK == 100000000)
/* SPLLCLK = 100MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         9       /* VCO div by 25 */
#define DIVCORE     0       /* Core clock / system clock = SPLLCLK / 1 (100 MHz) */
#define DIVBUS      1       /* Bus clock = SPLLCLK / 2 (50 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 5 (25 MHz) */
#define WAIT_STATES (DIVSLOW - DIVCORE)

#elif(CORE_CLK == 80000000)
/* SPLLCLK = 80MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         4       /* VCO div by 20 */
#define DIVCORE     0       /* Core clock / system clock = SPLLCLK / 1 (80 MHz) */
#define DIVBUS      1       /* Bus clock = SPLLCLK / 2 (40 MHz) */
#define DIVSLOW     2       /* Flash clock = SPLLCLK / 5 (26.67 MHz) */
#define WAIT_STATES (DIVSLOW - DIVCORE)

#elif(CORE_CLK == 60000000)
/* SPLLCLK = 120MHz, 8MHz EXTAL */
#define PREDIV      0       /* EXTAL div by 1 (8 MHz) */
#define MUL         14      /* VCO div by 30 */
#define DIVCORE     1       /* Core clock / system clock = SPLLCLK / 2 (60 MHz) */
#define DIVBUS      3       /* Bus clock = SPLLCLK / 4 (30 MHz) */
#define DIVSLOW     4       /* Flash clock = SPLLCLK / 5 (24 MHz) */
#define WAIT_STATES 2

#elif(CORE_CLK == 50000000)
/* SPLLCLK = 100MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         9       /* VCO div by 25 */
#define DIVCORE     1       /* Core clock / system clock = SPLLCLK / 2 (50 MHz) */
#define DIVBUS      3       /* Bus clock = SPLLCLK / 4 (25 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 4 (25 MHz) */
#define WAIT_STATES 1

#elif(CORE_CLK == 40000000)
/* SPLLCLK = 80MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         4       /* VCO div by 20 */
#define DIVCORE     1       /* Core clock / system clock = SPLLCLK / 2 (40 MHz) */
#define DIVBUS      3       /* Bus clock = SPLLCLK / 4 (20 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 4 (20 MHz) */
#define WAIT_STATES 1

#elif(CORE_CLK == 25000000)
/* SPLLCLK = 100MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         9       /* VCO div by 25 */
#define DIVCORE     3       /* Core clock / system clock = SPLLCLK / 4 (25 MHz) */
#define DIVBUS      3       /* Bus clock = SPLLCLK / 4 (25 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 4 (25 MHz) */
#define WAIT_STATES 0

#elif(CORE_CLK == 20000000)
/* SPLLCLK = 80MHz, 8MHz EXTAL */
#define PREDIV      0       /* XTAL div by 1 (8 MHz) */
#define MUL         4       /* VCO div by 20 */
#define DIVCORE     3       /* Core clock / system clock = SPLLCLK / 4 (20 MHz) */
#define DIVBUS      3       /* Bus clock = SPLLCLK / 4 (20 MHz) */
#define DIVSLOW     3       /* Flash clock = SPLLCLK / 4 (20 MHz) */
#define WAIT_STATES 0

#else
#error "Unsupported CORE_CLK value!"
#endif


/*******************************************************************************
* Function prototypes
******************************************************************************/
void SCG_Init(uint8_t clkMode);


#endif /* __SCG_H */
