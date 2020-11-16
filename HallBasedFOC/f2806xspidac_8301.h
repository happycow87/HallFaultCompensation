#ifndef __F280X_SPIDAC_H__
#define __F280X_SPIDAC_H__

#ifdef __cplusplus
extern "C"{
#endif

#include "f2806xbmsk.h"

/*----------------------------------------------------------------------------
Initialization constant for the F280X Time-Base Control Registers for PWM Generation. 
Sets up the timer to run free upon emulation suspend, count up-down mode
prescaler 1.
----------------------------------------------------------------------------*/
#define PWMDAC_INIT_STATE ( FREE_RUN_FLAG +         \
                            PRDLD_IMMEDIATE  +       \
                            TIMER_CNT_UPDN +         \
                            HSPCLKDIV_PRESCALE_X_1 + \
                            CLKDIV_PRESCALE_X_1  +   \
                            PHSDIR_CNT_UP    +       \
                            CNTLD_DISABLE )

/*----------------------------------------------------------------------------
Initialization constant for the F280X Compare Control Register. 
----------------------------------------------------------------------------*/
#define PWMDAC_CMPCTL_INIT_STATE ( LOADAMODE_ZRO + \
                                   LOADBMODE_ZRO + \
                                   SHDWAMODE_SHADOW + \
                                   SHDWBMODE_SHADOW )

/*----------------------------------------------------------------------------
Initialization constant for the F280X Action Qualifier Output A Register. 
----------------------------------------------------------------------------*/
#define PWMDAC_AQCTLA_INIT_STATE ( CAD_SET + CAU_CLEAR )
#define PWMDAC_AQCTLB_INIT_STATE ( CBD_SET + CBU_CLEAR )

/*----------------------------------------------------------------------------
Initialization constant for the F280X Dead-Band Generator registers for PWM Generation. 
Sets up the dead band for PWMDAC and sets up dead band values.
----------------------------------------------------------------------------*/
#define PWMDAC_DBCTL_INIT_STATE   BP_DISABLE 

/*----------------------------------------------------------------------------
Initialization constant for the F280X PWM Chopper Control register for PWM Generation. 
----------------------------------------------------------------------------*/
#define  PWMDAC_PCCTL_INIT_STATE  CHPEN_DISABLE

/*----------------------------------------------------------------------------
Initialization constant for the F280X Trip Zone Select Register 
----------------------------------------------------------------------------*/
#define  PWMDAC_TZSEL_INIT_STATE  DISABLE_TZSEL
              
/*----------------------------------------------------------------------------
Initialization constant for the F280X Trip Zone Control Register 
----------------------------------------------------------------------------*/
#define  PWMDAC_TZCTL_INIT_STATE ( TZA_HI_Z + TZB_HI_Z + \
                                   DCAEVT1_HI_Z + DCAEVT2_HI_Z + \
                                   DCBEVT1_HI_Z + DCBEVT2_HI_Z )

/*-----------------------------------------------------------------------------
Define the structure of the SPIDAC Driver Object 
-----------------------------------------------------------------------------*/
typedef struct{   
  	int16 *SpiDacInPointer0;   	// Input: Pointer to source data output on SPIDAC channel 0
	int16 *SpiDacInPointer1;    // Input: Pointer to source data output on SPIDAC channel 1
	int16 *SpiDacInPointer2;    // Input: Pointer to source data output on SPIDAC channel 2
	int16 *SpiDacInPointer3;    // Input: Pointer to source data output on SPIDAC channel 3
	Uint16 DataMax;     		// Parameter: SPIDAC resolution (Q0)
 	} SPIDAC;          

/*-----------------------------------------------------------------------------
Define a SPIDAC_handle
-----------------------------------------------------------------------------*/
typedef SPIDAC *SPIDAC_handle;

/*------------------------------------------------------------------------------
Default Initializers for the F280X PWMGEN Object 
------------------------------------------------------------------------------*/
#define F280X_SPIDAC_DEFAULTS {	(int16 *)0x300,	\
								(int16 *)0x300,	\
								(int16 *)0x300,	\
								(int16 *)0x300,	\
								512				\
								}

#define SPIDAC_DEFAULTS F280X_SPIDAC_DEFAULTS

/*------------------------------------------------------------------------------
	SPIDAC Init & SPIDAC Update Macro Definitions
------------------------------------------------------------------------------*/

#define SPIDAC_INIT_MACRO(v)										\
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;	/* SPI-A */				\
																	\
	/* Setup the SPI CCR register */								\
	SpiaRegs.SPICCR.bit.SPISWRESET = 0;								\
	SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;							\
	SpiaRegs.SPICCR.bit.SPILBK = 0;									\
	SpiaRegs.SPICCR.bit.SPICHAR = 0xF;								\
																	\
	/* Setup the SPI CTL register */								\
	SpiaRegs.SPICTL.bit.OVERRUNINTENA = 0;							\
	SpiaRegs.SPICTL.bit.CLK_PHASE = 1;								\
	SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;							\
	SpiaRegs.SPICTL.bit.TALK = 1;									\
	SpiaRegs.SPICTL.bit.SPIINTENA = 0;								\
																	\
	/* Initialize the SPI BRR (Baud Rate Register) to 1 Mb */		\
	SpiaRegs.SPIBRR = 3; /* reduce -> increase clock */				\
																	\
	/* Setup the SPI FIFO TX path */								\
	SpiaRegs.SPIFFTX.bit.SPIRST = 0;								\
	SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;								\
	SpiaRegs.SPIFFTX.bit.TXFIFO = 1;								\
	SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;								\
	SpiaRegs.SPIFFTX.bit.SPIRST = 1;								\
																	\
	/* Setup the SPI FIFO RX path */								\
	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;							\
	SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;								\
	SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;							\
																	\
	/* Setup the SPI FIFO CTL */									\
	SpiaRegs.SPIFFCT.bit.TXDLY = 1; /* reduce for more channel */	\
																	\
	/* Take the SPI port out of reset */							\
	SpiaRegs.SPICCR.bit.SPISWRESET = 1;								\
																	\
	/* Setup the SPI priority register */							\
	SpiaRegs.SPIPRI.bit.FREE = 1;									\
																	\
	/* Set the LDAC pin high */										\
	GpioDataRegs.GPASET.bit.GPIO19 = 1;

	int32 TmpData[4];
	int16 SpiBuf[4];
	unsigned int index;

#define SPIDAC_MACRO(v)																			    \
	TmpData[0] = (int32)v.DataMax * (int32)(*v.SpiDacInPointer0); /* Q15 = Q0*Q15*/				    \
	TmpData[1] = (int32)v.DataMax * (int32)(*v.SpiDacInPointer1); /* Q15 = Q0*Q15*/				    \
	TmpData[2] = (int32)v.DataMax * (int32)(*v.SpiDacInPointer2); /* Q15 = Q0*Q15*/				    \
	TmpData[3] = (int32)v.DataMax * (int32)(*v.SpiDacInPointer3); /* Q15 = Q0*Q15*/				    \
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 0;														    \
	for(index = 0 ; index < 4 ; index++){	            										    \
		while(SpiaRegs.SPIFFTX.bit.TXFFST); /* wait for the transmit FIFO to clear */			        \
		SpiaRegs.SPITXBUF = (int16)(TmpData[index]>>16)+(int16)(v.DataMax>>1)<<2 | 1<<12 | index<<14;	\
		while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG); /* wait for the transmit buffer to clear */	    \
	}																							    \
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 0;														    \
	GpioDataRegs.GPASET.bit.GPIO19 = 1;

#ifdef __cplusplus
}
#endif

#endif
