/*
 * Myheader.h
 *
 *  Created on: Mar 15, 2019
 *      Author: Prashant Agrawal
 *      This header file contains all the global variables used by this project.
 *
 */

#ifndef MYHEADER_H_
#define MYHEADER_H_


/* Neccasary header files*/
#include <ti/sysbios/knl/Semaphore.h>
#include <mcasp_drv.h>
#include <xdc/std.h>
#include <stdlib.h>
#include "stdio.h"
#include "string.h"
#include <math.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/uart/test/src/UART_board.h>
#include <ti/drv/uart/src/v0/UART_v0.h>
#include <ti/drv/uart/soc/UART_soc.h>


/*
 * Buffers placed in external memory are aligned on a 128 bytes boundary.
 * In addition, the buffer should be of a size multiple of 128 bytes for
 * the cache work optimally on the C6x.
 */
/* Number of samples to be processed in a frame  */
/* This should always be power of 2 or 4 so the library functions can process them*/
#define BUFLEN                  (1024)


/* This macro defines the frequency of reference signal used to calculate In-phase and Quadrature values*/
#define REF_FREQ                (5585)

/* alignment of buffer for use of L2 cache(Please do not change this) */
#define BUFALIGN                128

/*
 * The ADC has two channels and the data arrive in a 32-bit word format.Therefore to process the data, we have
 * to seperate the 32-bit word into two 16-bit word, the first 16-bit word is channel 1 data and the last 16-bit
 * are channel2 data. They are in fixed point 'int16' format.
 */

/* Pointer to channel 1 data array*/
int16_t *chan1;

/* Pointer to channel 2 data array*/
int16_t *chan2;

/*Pointer to an array containing voltage converted values of channel 1 and channel 2 data */
float *chan1_V, *chan2_V;

/* Pointers to array containing non filtered I and Q values of the of respected channel */
float *chan1_i;
float *chan1_q;
float *chan2_i;
float *chan2_q;

/* Pointer to Value containing the value of I and Q after filtering/Mean */
float *Itx, *Qtx, *Irx, *Qrx;

/* Pointer to value containing the ratio of Q/I of respected channels*/
float *txratio , *rxratio;

/* Pointer to float values containing the Phase of channel 1 and cahnnel2 */
float *phirx, *phitx;

/* Variable to store the value of phase difference of channel 1 and 2*/
float phasediff;

/* Handle for Rx and Tx Semaphore used in Audio echo task()*/
Semaphore_Handle semR,semT;
Semaphore_Params params;

/*Pointer to array containing the internally generated cosine and sine signal */
float *cos_adc_dig;
float *sin_adc_dig;

/*
 * Pointers to the array containing the dot product of channel data with the internally generated
 * reference signal. ilp_1 stands for I-Lowpass-Channel 1.(This is non-filtered data)
 * */
float *ilp_1,*qlp_1;
float *ilp_2, *qlp_2;

/* Pointer to an array containing the signal which needs to be sent to TX channel*/
int32_t *cos_dac_buf[2];

/*Pointer to values containing the peak Amplitude of channel 1 and channel 2*/
float *Amp1_V, *Amp2_V;

typedef  struct  {
    int16_t chan1_data[128];
    int16_t chan2_data[128];
//    float   itx;
//    float   qtx;
    float   irx;
    float   qrx;
    float   phase_diff;
    float   chan1_amp;
    float   chan2_amp;
    char    laser[10];

}UDP_pack;

UDP_pack* mypack;

float *NULLING_OFFSET_I;
float *NULLING_OFFSET_Q;

/*UART related variables*/
UART_HwAttrs uart_cfg;
UART_Params params_uart;
UART_Handle myuart1;
UART_Callback callback;
char* laser_data;



/* Function Prototype */
void calculate_IQ(int16_t *channel1, int16_t *channel2);





#endif /* MYHEADER_H_ */
