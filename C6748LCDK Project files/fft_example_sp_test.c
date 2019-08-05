/* ======================================================================= */
/* fft_sp_example.c -- FFT Example File                                    */
/*                                                                         */
/*        This example demonstrates the usage of the various FFT kernels   */
/*        provided with the C6x DSPLIB. The example shows:                 */
/*        - Twiddle factor generation for the various kernels              */
/*        - Scaling that needs to be applied to get similar output         */
/*          when using different kernels                                   */
/*        - Demonstrates the usage of FFT APIs                             */
/*                                                                         */
/* Rev 0.0.1                                                               */
/*                                                                         */
/* Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/  */ 
/*                                                                         */
/*                                                                         */
/*  Redistribution and use in source and binary forms, with or without     */
/*  modification, are permitted provided that the following conditions     */
/*  are met:                                                               */
/*                                                                         */
/*    Redistributions of source code must retain the above copyright       */
/*    notice, this list of conditions and the following disclaimer.        */
/*                                                                         */
/*    Redistributions in binary form must reproduce the above copyright    */
/*    notice, this list of conditions and the following disclaimer in the  */
/*    documentation and/or other materials provided with the               */
/*    distribution.                                                        */
/*                                                                         */
/*    Neither the name of Texas Instruments Incorporated nor the names of  */
/*    its contributors may be used to endorse or promote products derived  */
/*    from this software without specific prior written permission.        */
/*                                                                         */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   */
/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   */
/*                                                                         */
/* ----------------------------------------------------------------------- */
/*            Copyright (c) 2011 Texas Instruments, Incorporated.          */
/*                           All Rights Reserved.                          */
/* ======================================================================= */

#include <stdint.h>
#include <math.h>
#include <ti/dsplib/dsplib.h>
#include <Myheader.h>
#include <ti\dsplib\src\DSPF_sp_fftSPxSP\c674\DSPF_sp_fftSPxSP_cn.h>
#include <atansp.h>
#include "DSPF_sp_maxval.h"
//#include "DSPF_sp_fftSPxSP.h"
void calculate_IQ(int16_t *channel1, int16_t *channel2);

/* Global definitions */
/* Number of samples for which FFT needs to be calculated */
#define N 1024
/* Number of unique sine waves in input data */
#define NUM_SIN_WAVES 4

/* Align the tables that we have to use */
#pragma DATA_ALIGN(x_ref, 8);
float   x_ref [2*N];

#pragma DATA_ALIGN(x_sp, 8);
float   x_sp [2*N];
#pragma DATA_ALIGN(y_sp, 8);
float   y_sp [2*N];
#pragma DATA_ALIGN(w_sp, 8);
float   w_sp [2*N];

unsigned char brev[64] = {
                          0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
                          0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
                          0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
                          0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
                          0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
                          0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
                          0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
                          0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};


/* 
    The seperateRealImg function seperates the real and imaginary data
    of the FFT output. This is needed so that the data can be plotted
    using the CCS graph feature
 */
uint16_t comcnt = 0;
float y_real_sp [N];
float y_imag_sp [N];

seperateRealImg () {
    int i, j;

    for (i = 0, j = 0; j < N; i+=2, j++) {
        y_real_sp[j] = y_sp[i];
        y_imag_sp[j] = y_sp[i + 1];
    }
}

/*
    The main function that implements the example functionality.
    This example demonstrates the usage of the various FFT kernels provided 
    with the C6x DSPLIB. The example shows:
        - Twiddle factor generation for the various kernels
        - Needed scaling to get correct output
        - Demonstrates usage of FFT APIs
 */

/* Function for generating Specialized sequence of twiddle factors */
void gen_twiddle_fft_sp (float *w, int n)
{
    int i, j, k;
    double x_t, y_t, theta1, theta2, theta3;
    const double PI = 3.141592654;

    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
            theta1 = 2 * PI * i / n;
            x_t = cos (theta1);
            y_t = sin (theta1);
            w[k] = (float) x_t;
            w[k + 1] = (float) y_t;

            theta2 = 4 * PI * i / n;
            x_t = cos (theta2);
            y_t = sin (theta2);
            w[k + 2] = (float) x_t;
            w[k + 3] = (float) y_t;

            theta3 = 6 * PI * i / n;
            x_t = cos (theta3);
            y_t = sin (theta3);
            w[k + 4] = (float) x_t;
            w[k + 5] = (float) y_t;
            k += 6;
        }
    }
}

void calculate_IQ(int16_t *channel1, int16_t *channel2)
{
    uint16_t index=0;
    for(index =0 ; index <(N)-1; index++)
    {
        *(chan1_V+index)=(float)*(channel1+index);
        *(chan2_V+index)=(float)*(channel2+index);
        *(chan1_V+index)= (float)(*(chan1_V+index) / 32768.0);   //converting value into voltages
        *(chan2_V+index)= (float)(*(chan2_V+index) / 32768.0);

    }
    for(index =0 ; index <N-1; index++)
    {
        (*(chan1_i+ index))=   (float)((*(chan1_V + index))* (*(cos_adc_dig + index))) ;
        (*(chan1_q+ index))=   (float)((*(chan1_V + index))* (*(sin_adc_dig + index))) ;
        (*(chan2_i+ index))=   (float) ((*(chan2_V + index))* (*(cos_adc_dig + index))) ;
        (*(chan2_q+ index))=   (float) ((*(chan2_V + index))*(*(sin_adc_dig + index))) ;



    }
    index = 0;
    *Amp1_V = DSPF_sp_maxval(chan1_V, N);
    *Amp2_V = DSPF_sp_maxval(chan2_V, N);
    for(index =0 ; index <(2*N)-1; index+=2)
    {
        *(ilp_1+index)= (*(chan1_i+comcnt));
        *(qlp_1+index)= (*(chan1_q+comcnt));
        *(ilp_2+index)= (*(chan2_i+comcnt));
        *(qlp_2+index)= (*(chan2_q+comcnt));

        *(ilp_1+index+1)= 0.0;
        *(qlp_1+index+1)= 0.0;
        *(ilp_2+index+1)= 0.0;
        *(qlp_2+index+1)= 0.0;
        comcnt++;
    }
    /* Genarate twiddle factors */
    gen_twiddle_fft_sp(w_sp, N);

//    /* Call FFT routine */
    DSPF_sp_fftSPxSP_cn(N, ilp_1, w_sp, Itx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, qlp_1, w_sp, Qtx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, ilp_2, w_sp, Irx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, qlp_2, w_sp, Qrx, brev, 4, 0, N);

//    /* Call the test code to seperate the real and imaginary data */

    index=0;
    comcnt = 0;


    *txratio = (*Qtx)/(*Itx);
    *rxratio = (*Qrx)/(*Irx);

    //    atansp_v (txratio,phitx, 1);
    //    atansp_v (rxratio,phirx, 1);

    /*  Tx Phase: */

    if(*txratio >=0.0)
    {
        if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (txratio,phitx, 1);
        } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
            /* second quadrant */
            atansp_v (txratio,phitx, 1);
            *phitx += 1.570796326794897;
        } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
            /*  third qudrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  3.1415926535897931;
        } else {
            if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  4.712388980384690;
            }
        }
    }

    else {


        if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (txratio,phitx, 1);

        } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
            /* second quadrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  3.1415926535897931;
        } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
            /*  third qudrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  4.7123889803846901;
        } else {
            if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  6.2831853071795871;
            }
        }
    }




    /*  Rx Phase: */

    if(*rxratio >= 0.0)
    {

        if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (rxratio,phirx, 1);
        } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
            /* second quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += 1.570796326794897;
        } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
            /*  third qudrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += 3.1415926535897931;
        } else {
            if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx +=  4.712388980384690;
            }
        }
    }

    else
    {
        if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += 1.570796326794897;
        } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
            /* second quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx +=  3.1415926535897931;
        } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
            /*  third qudrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += 4.7123889803846901;
        } else {
            if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx +=  6.2831853071795871;
            }
        }

    }
    /*  converting rad in degree, Multiplying by 180/pi */
    (*phitx) *= 57.295779513082323;
    (*phirx) *= 57.295779513082323;


    if(*phitx> *phirx){
        phasediff = (*phitx)-(*phirx);
    }
    else {
        phasediff = 360-(*phirx)+(*phitx);
    }
    //    phasediff = phasediff + 0.0450;

}

/* ======================================================================== */
/*  End of file:  fft_example.c                                             */
/* ------------------------------------------------------------------------ */
/*            Copyright (c) 2011 Texas Instruments, Incorporated.           */
/*                           All Rights Reserved.                           */
/* ======================================================================== */

