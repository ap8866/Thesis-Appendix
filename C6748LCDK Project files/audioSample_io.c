/*
 * audioSample_io.c
 *
 * This file contains  code to demonstrate the Audio component
 * driver functionality on SYS/BIOS 6.
 *
 *
 */


/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */
#include "Myheader.h"
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
#include <ti/csl/csl_chip.h>
#include <xdc/runtime/Log.h>

#define MCASP_EDMA_ENABLED
#include <ti/sdo/edma3/drv/edma3_drv.h>
#include <ti/sdo/edma3/rm/edma3_rm.h>
#include <ti/sdo/edma3/drv/sample/bios6_edma3_drv_sample.h>

#include "mcasp_osal.h"
#include <ti/drv/mcasp/soc/mcasp_soc.h>
#include "ICodec.h"
#include "mcasp_cfg.h"
#include "MCASP_log.h"
#include "stdio.h"
#include "string.h"
#include <ti/csl/cslr_mcasp.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Cache.h>

/* Number of bytes nedded to store all the samples of chan1 and chan2 data in integer type*/
uint16_t chan_frame_size = BUFLEN*2;

/* Number of bytes nedded to store all the samples of chan1 and chan2 data in float type*/
uint16_t  float_frame_size = BUFLEN*4;


/* ========================================================================== */
/*                          IMPORTED VARIABLES                                */
/* ========================================================================== */

/*Heap memory handle which stores all the pointers and arrays*/
extern HeapMem_Handle Sample_heap;
extern HeapMem_Handle myHeap;
#define max(x,y) ( ((x) > (y))?(x):(y) )
/* ========================================================================== */
/*                          MACRO DEFINITIONS                                 */
/* ========================================================================== */

/* This is the number of buffers used by the application to be issued and reclaimed 
   This number can be higher than 2 (Ping pong) also. The McASP driver puts them in 
   a queue internally and process them in order and give back to the application */
#define NUM_BUFS                (2) /* For ping pong */
int alg_fft =1;

#define PRIME_COUNT                (2)

#define TOTAL_PACKETS_TO_TRANSFER  (98+2) /* Including priming */
#define NUM_APP_BUFS_QUEUE_ENTRIES  max(PRIME_COUNT,NUM_BUFS)
#define MCASP_TEST_MAX_SERIALIZERS  (TX_NUM_SERIALIZER) /* defined in mcasp_cfg.h */
#define MCASPFRAMES_RX_SIZE        (BUFLEN*sizeof(uint32_t)*MCASP_TEST_MAX_SERIALIZERS)

/* Putting it here so it can be changed through emulator */
volatile uint32_t test_num_bufs = NUM_BUFS;
volatile uint32_t test_prime_count = PRIME_COUNT;

#if defined(AIC_CODEC)
#include <Aic31.h>
Ptr  hAicDev;
Ptr  hAicChannel;
#endif

/* Function prototype */
static Void createStreams();
static Void prime();

int32_t *rxbuf[NUM_BUFS];
int32_t *txbuf[NUM_BUFS];

/* McASP Device handles */
Ptr  hMcaspDev;

/* McASP Device parameters */
Mcasp_Params mcaspParams;


/* Channel Handles */
Ptr hMcaspTxChan;
Ptr hMcaspRxChan;

/*Flags used by the Rx and Tx channel*/
volatile int RxFlag=0,TxFlag=0;
Semaphore_Params params;
int mcaspApiTest = FALSE;

/*Queue for Mcasp buffer*/
typedef struct McASP_App_Buffer_s {
    Osal_Queue_Elem  link;
    uint8_t *buf;
    uint32_t index;
}McASP_App_BufferInfo_t;

/* For TX and RX */
typedef enum {
    APP_BUFFER_RX=0,
    APP_BUFFER_TX=1
}McASP_App_Buffer_Direction_e;

Osal_Queue_Elem McASP_App_Buffer_FreeList[2];
Osal_Queue_Elem McASP_App_Buffer_TransitList[2];

McASP_App_BufferInfo_t McASP_App_bufs[2][NUM_APP_BUFS_QUEUE_ENTRIES];


/* Initialize the queue of buffer info  and push them in to the free queues */
void McASP_App_BufferInfo_Init()
{
    uint32_t i,bufno;
    Osal_Queue_construct(&McASP_App_Buffer_FreeList[APP_BUFFER_RX], (void *)NULL);
    Osal_Queue_construct(&McASP_App_Buffer_TransitList[APP_BUFFER_RX], (void *)NULL);

    Osal_Queue_construct(&McASP_App_Buffer_FreeList[APP_BUFFER_TX], (void *)NULL);
    Osal_Queue_construct(&McASP_App_Buffer_TransitList[APP_BUFFER_TX], (void *)NULL);

    /* Put buffers in the queue */
    for(i=0;i<test_prime_count;i++) {
        /* For priming, we would need to put 'prime_count' number of buffers in to the Free List to begin with.
         * During priming process, these will be popped and submitted to the McASP */
        bufno= i%test_num_bufs;	/* There are only NUM_BUFS buffers allocated from memory. If more buffers are needed for priming, say 4, these
	                                       buffers tx/rxBufs[NUM_BUFS]are to be re-used. */
        McASP_App_bufs[APP_BUFFER_RX][i].buf = rxbuf[bufno];
        McASP_App_bufs[APP_BUFFER_RX][i].index = bufno;
        Osal_Queue_put((Osal_Queue_handle(&McASP_App_Buffer_FreeList[APP_BUFFER_RX])),(Osal_Queue_Elem *)&McASP_App_bufs[APP_BUFFER_RX][i]);

        McASP_App_bufs[APP_BUFFER_TX][i].buf = txbuf[bufno];
        McASP_App_bufs[APP_BUFFER_TX][i].index = bufno;
        Osal_Queue_put((Osal_Queue_handle(&McASP_App_Buffer_FreeList[APP_BUFFER_TX])),(Osal_Queue_Elem *)&McASP_App_bufs[APP_BUFFER_TX][i]);
    }
}
/* Pop a buffer not in use */
McASP_App_BufferInfo_t * McASP_App_Buffers_PopFree(McASP_App_Buffer_Direction_e dir)
{
    McASP_App_BufferInfo_t *ptr;
    ptr = Osal_Queue_get(&McASP_App_Buffer_FreeList[dir]);
    if(ptr!=NULL) {
        Osal_Queue_put((Osal_Queue_handle(&McASP_App_Buffer_TransitList[dir])),(Osal_Queue_Elem *)ptr);
    }
    return(ptr);
}

/* Get from the transit list */
McASP_App_BufferInfo_t * McASP_App_Buffers_PopTransitBuf(McASP_App_Buffer_Direction_e dir)
{
    McASP_App_BufferInfo_t *ptr;
    ptr = Osal_Queue_get(&McASP_App_Buffer_TransitList[dir]);
    return(ptr);
}
/* Release to free list */
void McASP_App_Buffers_Release(McASP_App_BufferInfo_t *bufQueueEntry,McASP_App_Buffer_Direction_e dir)
{
    Osal_Queue_put((Osal_Queue_handle(&McASP_App_Buffer_FreeList[dir])),(Osal_Queue_Elem *)bufQueueEntry);
}


Error_Block eb;

int mcaspControlChanTest(void * mcaspChan);

/**************************************************************************************/
/*   FUNCTION DESCRIPTION: This utility function converts local GEM L2 address in to global
    memory addresses used by the EDMA inside McASP
 */
/**************************************************************************************/
static uint32_t getGlobalAddr (uint32_t addr)
{
    if ((addr >= 0x800000) && (addr < 0x1000000))
    {
#ifdef _TMS320C6X
        uint32_t coreNum;

        /* Get the core number. */
        coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

#if defined(SOC_AM572x) || defined(SOC_AM571x)
        /* Compute the global address. */
        return ((1 << 30) | (coreNum << 24) | (addr & 0x00ffffff));

#else
        /* Compute the global address. */
        return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
#endif
#else
        return addr;
#endif
    }
    else
    {
        /* non-L2 address range */
        return addr;
    }
}
/*********************** APPLICATION DEFINED FUNCTIONS: Begin ****************************/
/* The below functions need to be defined by the application and are registered to the
   McASP driver during instantiation 
 */
/*
 * This call back function is provided to the McASP driver during mcaspCreateChan()
 * and is called at the end of a transaction. This example uses the same call back function
 * for both TX and RX transfers and the call back argument is not being used in this
 * application and hence we pass NULL during mcaspCreateChan() as the call back argument.
 * This need not be the case for other applications where they could use a seperate
 * call back function for TX and RX. Also they could provide a non-NULL argument as
 * the call back argument and use it in their implementation of the call back function(s).
 */
void mcaspAppCallback(void* arg, MCASP_Packet *ioBuf)
{

    McASP_App_BufferInfo_t *bufInfo_transit; /* The buffer in transit */


    if(ioBuf->cmd == MCASP_READ)
    {
        RxFlag++;
        /* Get the buffer which is supposed to be the next. It should be in the transit queue */
        bufInfo_transit = McASP_App_Buffers_PopTransitBuf(APP_BUFFER_RX);      
        if(ioBuf->addr != (void *)getGlobalAddr((uint32_t)bufInfo_transit->buf)) {
            MCASP_log("Rx Buf Address mismatch on packet [%d] Expected %p, got %p with size %d\n",RxFlag,(void *)getGlobalAddr((uint32_t)bufInfo_transit->buf),ioBuf->addr,ioBuf->size);

        }


        /* Release it back to the Free list */ 
        McASP_App_Buffers_Release(bufInfo_transit,APP_BUFFER_RX);

        /* post semaphore */
        Semaphore_post(semR);

    } else if(ioBuf->cmd == MCASP_WRITE)
    {
        TxFlag++;

        /* Get the buffer which is supposed to be in the top of the list */
        bufInfo_transit = McASP_App_Buffers_PopTransitBuf(APP_BUFFER_TX);
        if(ioBuf->addr != (void *)getGlobalAddr((uint32_t)bufInfo_transit->buf)) {
            MCASP_log("Tx Buf Address mismatch on packet[%d] Expected %p, got %p with size %d\n",TxFlag,(void *)getGlobalAddr((uint32_t)bufInfo_transit->buf),ioBuf->addr,ioBuf->size);

        }

        /* Release it back to the Free list */
        McASP_App_Buffers_Release(bufInfo_transit,APP_BUFFER_TX);

        /* post semaphore */
        Semaphore_post(semT);
    } else {
        MCASP_log("Callback: Spurious packet ! Buff addr = %p with size %d",ioBuf->addr,ioBuf->size);

    }


}
/*
 * This call back is used during interrupt processing and is defined by the
 * application for error handling. These functions are called back from within the
 * mcasp driver when an error interrupt happens and macspIsr() is being called.
 * The sample error handling functions just records these errors which
 * are later used for analyzing the errors seen.
 */
/* The below variables are used to quit the frame processing loop if an error occurs */
int gblErrFlagXmt=0;
int gblErrFlagRcv=0;
/* The below variables are used to analyze the errors if an error interrupt happens */
Mcasp_errCbStatus errCbStatusXmt;
Mcasp_errCbStatus errCbStatusRcv;

/* Error handler for Transmit side */
void GblErrXmt(Mcasp_errCbStatus errCbStat)
{
    gblErrFlagXmt=1;
    errCbStatusXmt=errCbStat;
}
/* Error handler for Rcv side */
void GblErrRcv(Mcasp_errCbStatus errCbStat)
{
    gblErrFlagRcv=1;
    errCbStatusRcv=errCbStat;
}
/*********************** APPLICATION DEFINED FUNCTIONS: End ****************************/




/**************************************************************************************/
/* FUNCTION DESCRIPTION: This function analyzes the result of error interrupts, if it
 * happened
 */
/**************************************************************************************/	 
void mcaspAnalyzeErrors(Mcasp_errCbStatus *errCbStat)
{
    MCASP_log("\n------------ Error stats --------------\n");
    MCASP_log("*****  isClkFailErr : %d\n",errCbStat->isClkFailErr);
    MCASP_log("*****  isDMAErr    : %d\n",errCbStat->isDMAErr);
    MCASP_log("*****  isSyncErr   : %d\n",errCbStat->isSyncErr);
    MCASP_log("*****  retVal      : %d \n",errCbStat->retVal);
    MCASP_log("*****  isRcvOvrRunOrTxUndRunErr : %d \n",errCbStat->isRcvOvrRunOrTxUndRunErr);
}

/**************************************************************************************/
/* Sample Watchdog routine for parsing errors */
/**************************************************************************************/
void ErrorWatchDogRoutine()
{
    Mcasp_errCbStatus errCbStat_Xmt;
    Mcasp_errCbStatus errCbStat_Rcv;

    /* Query the transmit channel's error stat */
    mcaspControlChan(hMcaspTxChan,Mcasp_IOCTL_CHAN_QUERY_ERROR_STATS,&errCbStat_Xmt);
    mcaspControlChan(hMcaspTxChan,Mcasp_IOCTL_CHAN_QUERY_ERROR_STATS,&errCbStat_Rcv);

    /* For testing purpose, just print the contents. Application may choose to run this in
     * a background task and take action. In this example we are just printing the error */
    MCASP_log("\n\n******************* Transmit Watch dog stats ****************");
    mcaspAnalyzeErrors(&errCbStat_Xmt);

    MCASP_log("\n\n******************* Receive Watch dog stats ****************");
    mcaspAnalyzeErrors(&errCbStat_Rcv);
}

/**************************************************************************************/
/*   FUNCTION DESCRIPTION: This function creates the McASP channels for Tx and Rx 
     This function also creates the codec channels (if any)
 */
/**************************************************************************************/	 
static Void createStreams()
{
    int status;

#if defined(AIC_CODEC)    
    char remName[10]="aic";
    int mode = IOM_INPUT;
#endif	
#if !defined(MCASP_MASTER)
    /* Configure the external clock: In Slave mode, McASP is not the master, start initializing the external clock provider (AIC codec below),
   before configuring McASP clocks (in mcaspCreateChan() below) 
     */
#if defined(AIC_CODEC)
    /* In this case AIC provides the frame clocks, hence we need to start it first */
    status = aic31MdCreateChan(
            &hAicChannel,
            hAicDev,
            remName,
            mode,
            (Ptr)(&AIC31_config),
            mcaspAppCallback,
            NULL);

    if ((NULL == hAicChannel) &&
            (IOM_COMPLETED != status))
    {
        MCASP_log("AIC Create Channel Failed\n");
        BIOS_exit(0);
    }
#endif

#endif

    /* Create Mcasp channel for Tx */
    status = mcaspCreateChan(&hMcaspTxChan, hMcaspDev,
                             MCASP_OUTPUT,
                             &mcasp_chanparam[1],
                             mcaspAppCallback, NULL);

    if((status != MCASP_COMPLETED) || (hMcaspTxChan == NULL))
    {
        MCASP_log("mcaspCreateChan for McASP2 Tx Failed\n");
        BIOS_exit(0);
    }


    /* Create Mcasp channel for Rx */
    status = mcaspCreateChan(&hMcaspRxChan, hMcaspDev,
                             MCASP_INPUT,
                             &mcasp_chanparam[0],
                             mcaspAppCallback, NULL);
    if((status != MCASP_COMPLETED) || (hMcaspRxChan == NULL))
    {
        MCASP_log("mcaspCreateChan for McASP2 Rx Failed\n");
        BIOS_exit(0);
    }



#if defined(MCASP_MASTER) 
    /* If MCASP master, configure the clock of the slave device attached to McASP now.
    In the below case, it is the AIC codec */

#if defined(AIC_CODEC)
    status = aic31MdCreateChan(
            &hAicChannel,
            hAicDev,
            remName,
            mode,
            (Ptr)(&AIC31_config),
            (IOM_TiomCallback)&mcaspAppCallback,
            NULL);

    if ((NULL == hAicChannel) &&
            (IOM_COMPLETED != status))
    {
        MCASP_log("AIC Create Channel Failed\n");
    }
    else
    {

    }
#endif

#endif

}

/*
 * ======== prime ========
 */
MCASP_Packet rxFrame[NUM_APP_BUFS_QUEUE_ENTRIES];
MCASP_Packet txFrame[NUM_APP_BUFS_QUEUE_ENTRIES];

#include <ti/sysbios/family/c64p/Hwi.h>

Hwi_Handle myHwi;

static Void prime()
{
    Error_Block  eb;
    int32_t        count = 0, status;
    IHeap_Handle iheap;
    IHeap_Handle sam_heap;
    uint32_t tx_bytes_per_sample=(mcasp_chanparam[1].wordWidth/8);
    uint32_t rx_bytes_per_sample=(mcasp_chanparam[0].wordWidth/8);
    /* This represents the actual  number of bytes being transferred by the
     * DMA to/from the Host memory to the McASP peripheral. This include all serializers and timeslots.
     * BUFLEN contains the samples per serializer (inclusive of its timeslots) */
    //    uint32_t tx_frame_size = BUFLEN*mcasp_chanparam[1].noOfSerRequested*tx_bytes_per_sample;
    uint32_t tx_frame_size = 4108;  //1027 samples in each buffer
    uint32_t rx_frame_size = BUFLEN*mcasp_chanparam[0].noOfSerRequested*rx_bytes_per_sample;
    McASP_App_BufferInfo_t *appBuf_ptr;
    sam_heap = HeapMem_Handle_to_xdc_runtime_IHeap(Sample_heap);
    iheap = HeapMem_Handle_to_xdc_runtime_IHeap(myHeap);
    Error_init(&eb);
    chan1 = Memory_calloc(sam_heap, chan_frame_size, BUFALIGN,&eb);
    chan2 = Memory_calloc(sam_heap, chan_frame_size, BUFALIGN,&eb);
    chan1_i = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    chan1_q = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    chan2_i = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    chan2_q = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    cos_adc_dig = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    sin_adc_dig = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);

#if defined(ALG_FFT)
    ilp_1 = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    qlp_1 = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    ilp_2 = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    qlp_2 = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    Itx = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    Qtx = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    Qrx = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);
    Irx = Memory_calloc(sam_heap, 8192, BUFALIGN,&eb);

#else

    ilp_1 = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    qlp_1 = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    ilp_2 = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    qlp_2 = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
    Itx = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
    Qtx = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
    Qrx = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
    Irx = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);

#endif
cos_dac_buf[0] = Memory_calloc(sam_heap, tx_frame_size, BUFALIGN,&eb);
cos_dac_buf[1] = Memory_calloc(sam_heap, tx_frame_size, BUFALIGN,&eb);
chan1_V = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);
chan2_V = Memory_calloc(sam_heap, float_frame_size, BUFALIGN,&eb);


txratio= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
rxratio= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
phitx= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
phirx= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
Amp1_V= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
Amp2_V= Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
mypack= Memory_calloc(sam_heap, sizeof(mypack), BUFALIGN,&eb);
NULLING_OFFSET_I = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
NULLING_OFFSET_Q = Memory_calloc(sam_heap, 8, BUFALIGN,&eb);
laser_data = Memory_calloc(sam_heap, 15, BUFALIGN,&eb);
/* Allocate buffers for the SIO buffer exchanges                          */
for(count = 0; count < (NUM_BUFS ); count ++)
{
    rxbuf[count] = Memory_calloc(iheap, rx_frame_size,BUFALIGN, &eb);
    if(NULL == rxbuf[count])
    {
        MCASP_log("\r\nMEM_calloc failed.\n");
    }
}

/* Allocate buffers for the SIO buffer exchanges                          */
for(count = 0; count < test_num_bufs; count ++)
{
    txbuf[count] = Memory_calloc(iheap, tx_frame_size,BUFALIGN, &eb);
    if(NULL == txbuf[count])
    {
        MCASP_log("\r\nMEM_calloc failed.\n");
    }
}


McASP_App_BufferInfo_Init();


for(count = 0; count < test_prime_count; count ++)
{
    /* Pop a free application buffer */
    appBuf_ptr=McASP_App_Buffers_PopFree(APP_BUFFER_RX);
    /* Issue the first & second empty buffers to the input stream         */
    memset((uint8_t *)appBuf_ptr->buf, (0xB0+count), rx_frame_size);

    /* RX frame processing */
    rxFrame[count].cmd = MCASP_READ;
    rxFrame[count].addr = (void*)(getGlobalAddr((uint32_t)appBuf_ptr->buf));
    rxFrame[count].size = rx_frame_size;
    rxFrame[count].arg = (uint32_t) hMcaspRxChan;
    rxFrame[count].status = 0;
    rxFrame[count].misc = 1;   /* reserved - used in callback to indicate asynch packet */

    status = mcaspSubmitChan(hMcaspRxChan, &(rxFrame[count]));



    if((status != MCASP_PENDING))
        MCASP_log ("Debug: Error McASP2 RX : Prime  buffer  #%d submission FAILED\n", count);


}
for(count = 0; count < (test_prime_count); count ++)
{
    /* Pop a free application buffer */
    appBuf_ptr=McASP_App_Buffers_PopFree(APP_BUFFER_TX);
    memset((uint8_t *)appBuf_ptr->buf, (0xA0+count), tx_frame_size);
    /* TX frame processing */
    txFrame[count].cmd = MCASP_WRITE;
    txFrame[count].addr = (void*)(getGlobalAddr((uint32_t)appBuf_ptr->buf));
    txFrame[count].size = tx_frame_size;

    txFrame[count].arg = (uint32_t) hMcaspTxChan;
    txFrame[count].status = 0;
    txFrame[count].misc = 1;   /* reserved - used in callback to indicate asynch packet */



    status = mcaspSubmitChan(hMcaspTxChan, &(txFrame[count]));
    if((status != MCASP_PENDING))
        MCASP_log ("Debug: Error McASP2 TX : Prime  buffer  #%d submission FAILED\n", count);

}

}


extern EDMA3_DRV_GblConfigParams sampleEdma3GblCfgParams[];
/* EnableEDMA event in the SampleCfg*/
static void enableEDMAHwEvent(uint32_t edmaNum, uint32_t eventNo) {
    sampleEdma3GblCfgParams[edmaNum].dmaChannelHwEvtMap[eventNo/32] |= (1 << (eventNo%32));
}

EDMA3_DRV_Handle McaspApp_edmaInit(Mcasp_HwInfo *cfg)
{
    EDMA3_DRV_Handle hEdma;
    EDMA3_DRV_Result edmaResult = 0;

    enableEDMAHwEvent(EDMACC_NUM,MCASP_RX_DMA_CH);
    enableEDMAHwEvent(EDMACC_NUM,MCASP_TX_DMA_CH);

    hEdma = edma3init(EDMACC_NUM, &edmaResult);

    if (edmaResult != EDMA3_DRV_SOK)
    {
        /* Report EDMA Error
         */
        MCASP_log("\nEDMA driver initialization unsuccessful\n");
    }
    else
    {
        MCASP_log("\nEDMA driver initialization successful.\n");
    }

    return hEdma;
}

/*
 * ======== echo ========
 * This function copies from the input SIO to the output SIO. You could
 * easily replace the copy function with a signal processing algorithm.
 */
extern Int aic31MdBindDev(Ptr *, Int, Ptr);

int gtxFrameIndexCount=0;
int grxFrameIndexCount=0;
int itemp;
int result, pwr_status, fs_status, bck_status;
int total_frames_sent=0;








Void Audio_echo_Task()
{
    Mcasp_HwInfo hwInfo;
    int j, uartread, uartwrite, k, idx;
    char ldata[12] = {0};
    char trig[1] = {24};
    volatile int32_t i32Count, status = 0;
    uint16_t chan_index= 0;
    uint16_t cnt = 0;
    int32_t chan1avg=0, chan2avg=0;
    hMcaspDev  = NULL;
    size_t readsize_l = 12;
    size_t writesize_l = 1;
    uint32_t semTimeout;
    int32_t txSemStatus, rxSemStatus;
    uint32_t rx_frames = 0, tx_frames = 0;

    uint32_t tx_bytes_per_sample=(mcasp_chanparam[1].wordWidth/8);
    uint32_t rx_bytes_per_sample=(mcasp_chanparam[0].wordWidth/8);
    /* This represents the actual  number of bytes being transferred by the
     * DMA to/from the Host memory to the McASP peripheral. This include all serializers and timeslots.
     * BUFLEN contains the samples per serializer (inclusive of its timeslots) */
    //    uint32_t tx_frame_size = BUFLEN*TX_NUM_SERIALIZER*tx_bytes_per_sample;
    uint32_t tx_frame_size = 4108;
    uint32_t rx_frame_size = BUFLEN*RX_NUM_SERIALIZER*rx_bytes_per_sample;

    McASP_App_BufferInfo_t *appBuf_ptr_tx = NULL;
    McASP_App_BufferInfo_t *appBuf_ptr_rx = NULL;


    Mcasp_socGetInitCfg(MCASP_NUM, &hwInfo);

    hwInfo.dmaHandle = McaspApp_edmaInit(&hwInfo);


    Mcasp_socSetInitCfg(MCASP_NUM, &hwInfo);

    /* 2. SEM Initializations */
    Semaphore_Params_init(&params);

    /* Create semaphores to wait for buffer reclaiming */
    semR = Semaphore_create(0, &params, &eb);
    semT = Semaphore_create(0, &params, &eb);

    /* 3. McASP Initializations */
    /* Initialize McASP Tx and Rx parameters */

    mcaspParams = Mcasp_PARAMS;


    status = mcaspBindDev(&hMcaspDev, MCASP_NUM, &mcaspParams);
    if((status != MCASP_COMPLETED) || (hMcaspDev == NULL))
    {
        MCASP_log("mcaspBindDev for McASP Failed\n");
        abort();
    }


    /* Bind AIC Codec */
    aic31MdBindDev(&hAicDev, 0, (Ptr)&Aic31_PARAMS);


    /* Call createStream function to create I/O streams                       */
    createStreams();
    /* Call prime function to do priming                                      */
    prime();
    /* Cosine and Sinwave generation inside the DSP*/
    double M_PIE = 3.14159265358979323846;
    double phi;
    uint16_t freq = REF_FREQ;
    double sample_time = 0.0;
    double sample_rate = 48000;
    double sinavg = 0, cosavg = 0;
    double time = 1/sample_rate;

    for ( cnt = 0 ; cnt < BUFLEN-1; cnt++ )
    {
        phi = 2 * M_PIE * freq * sample_time;
        *(cos_adc_dig + cnt) = (float)(cos(phi));
        cosavg += (double)*(cos_adc_dig + cnt);
        *(sin_adc_dig + cnt) = (float)(sin(phi));
        sample_time = sample_time+ time;
        sinavg +=(double) *(sin_adc_dig + cnt);

    }
    sinavg /= BUFLEN;
    cosavg /= BUFLEN;
    for ( cnt = 0 ; cnt < BUFLEN-1; cnt++ )
    {
        *(cos_adc_dig + cnt) = *(cos_adc_dig + cnt) - (float)cosavg;
        *(sin_adc_dig + cnt) = *(sin_adc_dig + cnt) - (float)sinavg;
    }
    sample_time = 0.0;

    for ( cnt = 0 ; cnt < 1027; cnt++ )
    {
        phi = 2 * M_PIE * 5585 * sample_time;


        *(cos_dac_buf[0] + cnt) = (int32_t)(32767 *(cos(phi)));;
        sample_time =sample_time + time;

    }

    for ( cnt = 0 ; cnt < 1027; cnt++ )
    {
        phi = 2 * M_PIE * 5585 * sample_time;
        *(cos_dac_buf[1] + cnt) = (int32_t)(32767 *(cos(phi)));
        sample_time = sample_time + time;

    }
    //*(cos_dac_buf[1] + 1023) = (int32_t)(32567);
    memcpy((void *)((uint8_t *)txbuf[0]),(void *)((uint8_t *)cos_dac_buf[0]),tx_frame_size);
    memcpy((void *)((uint8_t *)txbuf[1]),(void *)((uint8_t *)cos_dac_buf[1]),tx_frame_size);


    MCASP_log("Initialization complete. priming about to begin \n");




    /* Since Rx data arrival due to Tx buffering in non device-loopback cases,
     * and because we need data from Rx before submitting to the Tx stream, we
     * wait indefinitely for both to complete.
     */
    semTimeout = BIOS_WAIT_FOREVER;
    *NULLING_OFFSET_I = 0;
    *NULLING_OFFSET_Q = 0;

    /* Forever loop to continuously receive and transmit audio data           */
    for (i32Count = 0; i32Count >= 0; i32Count++)
    {

        if(gblErrFlagXmt || gblErrFlagRcv)
            break;


        Log_info0("\r\n loop started\n");
        rxSemStatus = Semaphore_pend(semR, semTimeout);
        if(i32Count<0)
        {
            txSemStatus = Semaphore_pend(semT, semTimeout);
        }
        if(rxSemStatus == TRUE)
        {
            /* Reclaim full buffer from the input stream if ready */
            appBuf_ptr_rx = McASP_App_Buffers_PopFree(APP_BUFFER_RX);
            grxFrameIndexCount = appBuf_ptr_rx->index;

            Cache_inv((void *)((uint8_t *)appBuf_ptr_rx->buf),rx_frame_size,Cache_Type_ALL, TRUE);
        }

        if(txSemStatus == TRUE & i32Count<0)
        {
            /* Reclaim full buffer from the output stream if ready */
            appBuf_ptr_tx = McASP_App_Buffers_PopFree(APP_BUFFER_TX);
            gtxFrameIndexCount = appBuf_ptr_tx->index;
        }



        /******************************* Sample Processing Begins ***************************/
        /* (BUFLEN* RX_NUM_SERIALIZER) 32-bit samples samples have been accumulated in rxbuf[grxFrameIndexCount] now.
	       Application specific processing on these samples, before sending it back to McASP via 
	       txbuf[grxFrameIndexCount].
		   APPLICATION SPECIFIC PROCESSING could be done here. Below are the few audio demos and their
		   application specific processing shown below.
         */

        /* DEFAULT CASE: Copy the frame received and send it back to Tx buffer.
		   This way the audio received by McASP from the remote device, is loopbacked and sent back
		   to the device here.
         */
        //	memcpy((void *)((uint8_t *)appBuf_ptr_tx->buf),(void *)((uint8_t *)appBuf_ptr_rx->buf),rx_frame_size);
        for(chan_index = 0;chan_index < BUFLEN-1;chan_index++)
        {

            //   		    *(chan1+ chan_index) = ((((appBuf_ptr_rx->buf[ appBuf_ptr_rx->index] + chan_index))  & 0xFFFF0000) >> 16);
            //   		    *(chan2+ chan_index) =(((appBuf_ptr_rx->buf[ appBuf_ptr_rx->index]+ chan_index))  & 0x0000FFFF);
            *(chan1+ chan_index) = (((*(rxbuf[gtxFrameIndexCount] + chan_index))  & 0xFFFF0000) >> 16);
            chan1avg += (int32_t)*(chan1+ chan_index);
            *(chan2+ chan_index) =((*(rxbuf[gtxFrameIndexCount] + chan_index))  & 0x0000FFFF);
            chan2avg += (int32_t)*(chan2+ chan_index);
        }
        chan1avg /= BUFLEN;
        chan2avg /=BUFLEN;
        for(chan_index = 0;chan_index < BUFLEN-1;chan_index++)
        {
            *(chan1+ chan_index) =*(chan1+ chan_index) -(int16_t)chan1avg;
            *(chan2+ chan_index) =*(chan2+ chan_index) -(int16_t)chan2avg;
        }

        Log_info0("\r\n sample process start\n");
//        UART_write(myuart1, trig, readsize_l);
        calculate_IQ(chan1, chan2);

        chan1avg = 0;
        chan2avg = 0;
        //   (*callback)(myuart1, laser_data, readsize_l);
        //   uartwrite = UART_write(myuart1, "24", 2);

        //To enable laser
//        uartread = UART_read(myuart1, ldata, readsize_l);
//
//        if(uartread == 0)
//        {
//            mypack->laser[0] = 'N';
//            mypack->laser[1] = 'a';
//            mypack->laser[2] = 'N';
//            mypack->laser[3] = 0;
//            mypack->laser[4] = 0;
//            mypack->laser[5] = 0;
//            mypack->laser[6] = 0;
//            mypack->laser[7] = 0;
//            mypack->laser[8] = 0;
//
//        }
//        else
//        {
//            for(idx = 0; idx<13; idx++)
//            {
//                mypack->laser[idx] = ldata[idx];
//            }
//        }


        Log_info0("\r\n Samples Processed\n");


        /******************************* Sample Processing End ***************************/
        if(rxSemStatus == TRUE)
        {

            /* Issue an empty buffer to the input stream                          */
            rxFrame[grxFrameIndexCount].cmd = MCASP_READ;
            rxFrame[grxFrameIndexCount].addr = (void*)getGlobalAddr((uint32_t)rxbuf[grxFrameIndexCount]);
            rxFrame[grxFrameIndexCount].size = rx_frame_size;
            rxFrame[grxFrameIndexCount].arg = (uint32_t) hMcaspRxChan;
            rxFrame[grxFrameIndexCount].status = 0;
            rxFrame[grxFrameIndexCount].misc = 1;   /* reserved - used in callback to indicate asynch packet */

            status = mcaspSubmitChan(hMcaspRxChan, &(rxFrame[grxFrameIndexCount]));
            if((status != MCASP_PENDING))
                Log_info0 ("Debug: Error McASP RX :  buffer  #%d submission FAILED\n");

        }
        if(txSemStatus == TRUE & i32Count<0)
        {

            Cache_wbInv((void *)((uint8_t *)appBuf_ptr_tx->buf),tx_frame_size,Cache_Type_ALL, TRUE);

            /* Issue full buffer to the output stream                             */
            /* TX frame processing */
            txFrame[gtxFrameIndexCount].cmd = MCASP_WRITE;
            txFrame[gtxFrameIndexCount].addr = (void*)getGlobalAddr((uint32_t)appBuf_ptr_tx->buf);
            txFrame[gtxFrameIndexCount].size = tx_frame_size;
            txFrame[gtxFrameIndexCount].arg = (uint32_t) hMcaspTxChan;
            txFrame[gtxFrameIndexCount].status = 0;
            txFrame[gtxFrameIndexCount].misc = 1;   /* reserved - used in callback to indicate asynch packet */



            status = mcaspSubmitChan(hMcaspTxChan, &(txFrame[gtxFrameIndexCount]));
            if((status != MCASP_PENDING))
                Log_info0 ("Debug: Error McASP TX : Prime  buffer  #%d submission FAILED\n");


        }




    } /* end of for (i32Count = 0; i32Count >= 0; i32Count++) */

    MCASP_log("\nTotal frames sent:     %d", tx_frames);
    MCASP_log("\nTotal frames received: %d", rx_frames);
    ErrorWatchDogRoutine();
    if(gblErrFlagXmt) {
        MCASP_log("\n Transmit ERROR occured\n");
        mcaspAnalyzeErrors(&errCbStatusXmt);
    }

    if(gblErrFlagRcv) {
        MCASP_log("\n Receive ERROR occured\n");
        mcaspAnalyzeErrors(&errCbStatusRcv);
    }


    MCASP_log("\nDeleting Rx channel");
    status = mcaspDeleteChan(hMcaspRxChan);
    MCASP_log("\nDeleting Tx channel");
    status = mcaspDeleteChan(hMcaspTxChan);
    MCASP_log("\nUnBinding Mcasp");
    status = mcaspUnBindDev(hMcaspDev);

    {
        IHeap_Handle iheap;

        iheap = HeapMem_Handle_to_xdc_runtime_IHeap(myHeap);
        Error_init(&eb);
        for(i32Count = 0; i32Count < (test_num_bufs); i32Count ++)
        {
            Memory_free(iheap,rxbuf[i32Count],rx_frame_size);
            Memory_free(iheap,txbuf[i32Count],tx_frame_size);
        }
    }
    /* Display profiling results */


    /* Application specific report(if any) */
    BIOS_exit(0);
}

/*
 * This function tests the mcaspControlChan API.
 */
int mcaspControlChanTest(void * mcaspChan)
{
    int status;

    /* Testing the reconfiguration of wordWidth: first change it to 16. */
    mcasp_chanparam[0].wordWidth = Mcasp_WordLength_16;
    status = mcaspControlChan(mcaspChan, Mcasp_IOCTL_CHAN_PARAMS_WORD_WIDTH, &mcasp_chanparam[0]);
    if(status != MCASP_COMPLETED) {
        return status;
    }

    /* Then change it back to 32. This shouldn't impact audio playing out. */
    mcasp_chanparam[0].wordWidth = Mcasp_WordLength_32;
    status = mcaspControlChan(mcaspChan, Mcasp_IOCTL_CHAN_PARAMS_WORD_WIDTH, &mcasp_chanparam[0]);

    return status;
}

/* ========================================================================== */
/*                             END OF FILE                                    */
/* ========================================================================== */
