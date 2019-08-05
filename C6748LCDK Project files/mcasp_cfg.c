/*
 * mcasp_cfg.c
 *
 * This file contains the test / demo code to demonstrate the Audio component
 * driver functionality on SYS/BIOS 6.

*/

/** \file     mcasp_cfg.c
 *
 *  \brief    sample application for demostration of audio playing
 *
 * This file contains the McASP driver Implementation, You can set all the
 * registers related to McASP here. Please have a look at the McASP Data sheet
 * to understand the available registers configuration options. McASP Data sheet
 * is separately available on TI's website and it is different from the Device datasheet.
 *
 *
 *             (C) Copyright 2009, Texas Instruments, Inc
 */

/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */

#include <xdc/std.h>
#include <ti/sysbios/io/IOM.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <mcasp_drv.h>
#include <mcasp_cfg.h>
#include <mcasp_tune.h>
#include <ti/csl/csl_chip.h>
#include <ti/sdo/edma3/drv/edma3_drv.h>
#include <ti/sdo/edma3/rm/edma3_rm.h>
#include <ti/sdo/edma3/drv/sample/bios6_edma3_drv_sample.h>
#include "ICodec.h"
#include "stdio.h"
#include "string.h"

void GblErrXmt(Mcasp_errCbStatus errCbStat);
void GblErrRcv(Mcasp_errCbStatus errCbStat);

/* ========================================================================== */
/*                          IMPORTED VARIABLES                                */
/* ========================================================================== */


extern EDMA3_DRV_Handle hEdma;
extern HeapMem_Handle myHeap;

/* ========================================================================== */
/*                          McASP Init config                                 */
/* ========================================================================== */



Mcasp_HwSetupData mcaspRcvSetup = {
        /* .rmask    = */ 0xFFFFFFFF, /* All the data bits are to be used     */
#if defined (DSP_MODE)
        /* .rfmt     = */ 0x000080f0,
#else /* I2S MODE*/
    /* .rfmt     = */ 0x000180F0,
#endif								  /* 0/1 bit delay from framsync
                                       * MSB first
                                       * No extra bit padding
                                       * Padding bit (ignore)
                                       * slot Size is 32
                                       * Reads from DMA port
                                       * NO rotation
                                       */
#if defined (MCASP_MASTER)
	#if defined (DSP_MODE)
    /* .afsrctl  = */ 0x00000002,           /* burst mode,
                                             * Frame sync is one bit
                                             * internally generated frame sync
                                             * Rising edge is start of frame
                                             */

	#else /* I2S MODE*/
          /* .afsrctl  = */ 0x00000113,     /* I2S mode,
                                             * Frame sync is one word
                                             * Internally generated frame sync
                                             * Falling edge is start of frame
                                             */
	#endif
#else
	#if defined (DSP_MODE)
    /* .afsrctl  = */ 0x00000000,           /* burst mode,
                                             * Frame sync is one bit
                                             * Externally generated frame sync
                                             * Rising edge is start of frame
                                             */

	#else /* I2S MODE*/
          /* .afsrctl  = */ 0x00000111,     /* I2S mode,
                                             * Frame sync is one word
                                             * Externally generated frame sync
                                             * Falling edge is start of frame
                                             */
	#endif
#endif

#if defined (DSP_MODE)
/* .rtdm     = */ 0x00000001,           /* slot 1 is active (DSP)
                                         *              */

#else /* I2S MODE*/
      /* .rtdm     = */ 0x00000003,     /* 2 slots are active (I2S)
                                         *            */
#endif
        /* .rintctl  = */ 0x00000003, /* sync error and overrun error         */
        /* .rstat    = */ 0x000001FF, /* reset any existing status bits       */
        /* .revtctl  = */ 0x00000000, /* DMA request is enabled or disabled   */
        {
#if defined (MCASP_MASTER)
		#if defined (DSP_MODE)
			/* .aclkrctl  = */ 0x00000020,
			/* .ahclkrctl = */ 0x00008001,
		#else /* I2S MODE*/
			/* .aclkrctl  = */ 0x00000027,	/* Div (8), Internal Source, rising edge */
			/* .ahclkrctl = */ 0x00008013,	/* Div (20), Internal AUX_CLK Source */
		#endif
#else
		#if defined (DSP_MODE)
			/* .aclkrctl  = */ 0x00000000,
		#else /* I2S MODE*/
			/* .aclkrctl  = */ 0x00000080,	/* External Source, rising edge */
		#endif
			/* .ahclkrctl = */ 0x00000000,	/* Don't Care */
#endif
             /* .rclkchk   = */ 0x00000000
        }
} ;

Mcasp_HwSetupData mcaspXmtSetup = {
        /* .xmask    = */ 0xFFFFFFFF, /* All the data bits are to be used     */
		#if defined (DSP_MODE)
			/* .xfmt     = */ 0x000080F0,
		#else /* I2S MODE*/
			/* .xfmt     = */ 0x000180F0,
		#endif 						  /*
                                       * 0/1 bit delay from framsync
                                       * MSB first
                                       * No extra bit padding
                                       * Padding bit (ignore)
                                       * slot Size is 32
                                       * Reads from DMA port
                                       * 0-bit rotation
                                       */
#if defined (MCASP_MASTER)
		#if defined (DSP_MODE)
		/* .afsxctl  = */ 0x00000002,       /* burst mode,
											 * Frame sync is one bit
											 * Internally generated frame sync
											 * Rising edge is start of frame
											 */
		/* .xtdm     = */ 0x00000001,       /* slot 1 is active (DSP) */
		#else /*I2S MODE*/
			  /* .afsxctl  = */ 0x00000113, /* I2S mode,
											 * Frame sync is one word
											 * internally generated frame sync
											 * Falling edge is start of frame
											 */
			  /* .xtdm     = */ 0x00000003, /* 2 slots are active (I2S) */
		#endif
#else
		#if defined (DSP_MODE)
		/* .afsxctl  = */ 0x00000000,       /* burst mode,
											 * Frame sync is one bit
											 * Rising edge is start of frame
											 * externally generated frame sync
											 */
		/* .xtdm     = */ 0x00000001,       /* slot 1 is active (DSP) */
		#else /*I2S MODE*/
			  /* .afsxctl  = */ 0x00000111, /* I2S mode,
											 * Frame sync is one word
											 * Externally generated frame sync
											 * Falling edge is start of frame
											 */
			  /* .xtdm     = */ 0x00000003, /* 2 slots are active (I2S) */
		#endif
#endif
        /* .xintctl  = */ 0x00000003, /* sync error,overrun error,clK error   */
        /* .xstat    = */ 0x000001FF, /* reset any existing status bits       */
        /* .xevtctl  = */ 0x00000000, /* DMA request is enabled or disabled   */
        {
#if defined (MCASP_MASTER)
        #if defined (DSP_MODE)
			/* .aclkxctl  = */ 0x00000027,
			/* .ahclkxctl = */ 0x00008001,

		#else /* I2S MODE*/
			/* .aclkxctl  = */ 0x000000A7, /* Div (8), Internal Source, SYNC, Falling edge */
			/* .ahclkxctl = */ 0x00008000, /* Div (20), Internal AUX_CLK Source */
		#endif
#else
		#if defined (DSP_MODE)
			/* .aclkxctl  = */ 0x00000000,	/* External Source, SYNC */

		#else /* I2S MODE*/
			/* .aclkxctl  = */ 0x00000080, /* External Source, SYNC, Falling edge */
		#endif
#endif
             /* .xclkchk   = */ 0x00000000
        },

};

/* McAsp channel parameters                                  */
Mcasp_ChanParams  mcasp_chanparam[2]=
{
    {
        RX_NUM_SERIALIZER,                    /* number of serialisers      */
        {Mcasp_SerializerNum_14, }, /* serialiser index           */
        &mcaspRcvSetup,
        TRUE,
        Mcasp_OpMode_TDM,          /* Mode (TDM/DIT)             */
        Mcasp_WordLength_32,
        NULL,
        0,
        NULL,
		(Mcasp_GblCallback)&GblErrRcv,
#if defined (DSP_MODE)
        1,
		Mcasp_BufferFormat_1SER_1SLOT,
#else      /* I2S MODE*/
        2, /* number of TDM channels      */
		Mcasp_BufferFormat_1SER_MULTISLOT_INTERLEAVED,
#endif
        TRUE,
		RX_FIFO_EVENT_DMA_RATIO,
        TRUE,
        Mcasp_WordBitsSelect_LSB
    },
    {
        TX_NUM_SERIALIZER,                   /* number of serialisers       */
        {Mcasp_SerializerNum_13,},
        &mcaspXmtSetup,
        TRUE,
        Mcasp_OpMode_TDM,
        Mcasp_WordLength_32,      /* word width                  */
        NULL,
        0,
        NULL,
		(Mcasp_GblCallback)&GblErrXmt,
#if defined (DSP_MODE)
        1,
		Mcasp_BufferFormat_1SER_1SLOT,
#else      /* I2S MODE*/
        2, /* number of TDM channels      */
		Mcasp_BufferFormat_1SER_MULTISLOT_INTERLEAVED,
#endif
        TRUE,
		TX_FIFO_EVENT_DMA_RATIO,
        TRUE,
        Mcasp_WordBitsSelect_LSB
    }
};

/*
 * ======== createStreams ========
 */
/*  Here you can configure the sample rate of the CODEC*/
ICodec_ChannelConfig AIC31_config =
{
		48000,  /* sampling rate for codec */
		0,  /* gain (%) for codec      */
		0x00,
		0x00
};
