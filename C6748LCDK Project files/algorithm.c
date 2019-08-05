/*
 * algorithm_FIR.c
 *
 *  Created on: Mar 15, 2019
 *      Author: pagrawal
 *      test comment changed to test the develop branch
 */

/* ========================================================================== */
#include <xdc/std.h>
#include <stdint.h>
#include <ti/dsplib/dsplib.h>
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "Myheader.h"
#include <DSPF_sp_fir_gen.h>
#include <ti\dsplib\src\DSPF_sp_fftSPxSP\c674\DSPF_sp_fftSPxSP_cn.h>
#include <atansp.h>
#include "DSPF_sp_maxval.h"
#include "DSPF_sp_vecmul.h"
#include <ti/ndk/inc/netmain.h>




//void calculate_IQ(int16_t *channel1, int16_t *channel2);



#if !defined(ALG_FFT)
void calculate_IQ(int16_t *channel1, int16_t *channel2)
{

/*
 * Below are defined constants used in algorithm*/
    /*constant for pie*/
    float F_PIE = 3.1415926535897931;
    /*constant for pie/2*/
    float F_PIE_2 = 1.570796326794897;
    /*constant for 3*pie/2 */
    float F_PIE_32 = 4.712388980384690;
    /*constant for 2*pie*/
    float F_2PIE = 6.2831853071795871;
    /*constant for coverting degree to radian*/
    float DEG_CONS = 57.295779513082323;
    /*constant for coverting digitral values to voltages*/
    float ADC_SCALE = 32768.0;

    /*Number of samples that need to be proccesed*/
    int16_t N_S = BUFLEN;

    /*The order of the filter to be used in the algorithm*/
    int Filt_ord = 24;

    /*Some counter variables for the for loops*/
    volatile uint16_t index= 0;
    volatile uint16_t k =0;

    /* Constant array containing the coefficients of the filters*/
    const float  Coeff[24]=   {0.00637986025278828, 0.00774508101796138, 0.0117344515426911,  0.0180560913812019,  0.0262438997406527,  0.0356921461941437,  0.0457005257353353,  0.0555263168651078,  0.0644397464034482, 0.0717784216439608,  0.0769967577874166,
                               0.0797067014352924,  0.0797067014352924,  0.0769967577874166,  0.0717784216439608,  0.0644397464034482,  0.0555263168651078,  0.0457005257353353,  0.0356921461941437,  0.0262438997406527,  0.0180560913812019,  0.0117344515426911,
                               0.00774508101796138, 0.00637986025278828};

    /*for loop that convert the number of samples with digital values into voltage values*/
    for(index =0 ; index <(N_S)-1; index++)
    {
        *(chan1_V+index)=(float)*(channel1+index);
        *(chan2_V+index)=(float)*(channel2+index);
        /*converting value into voltages*/
        *(chan1_V+index)= (float)(*(chan1_V+index) / ADC_SCALE);
        *(chan2_V+index)= (float)(*(chan2_V+index) / ADC_SCALE);
    }

    /*
     * Max value function that calculates the maximum value of an array
     *  and gives the peak amplitude if the signal*/
//    *Amp1_V = DSPF_sp_maxval(chan1_V, N_S);
//    *Amp2_V = DSPF_sp_maxval(chan2_V, N_S);
    /*Channel 1 is tx and Channel 2 is Rx*/

    /*
     * This routine calculates the dot product of the received signal(chan1_V and chan2_V)
     *  and the internal generated reference signal(cos_adc_dig and sin_adc_dig). The dot
     *  product results are stored in as unfiltered I and Q values in the respective arrays
     *  named as chan1_i, chan1_q, chan2_i, chan2_q. N_S is the number of samples that needs
     *  tom be multiplied*/
    DSPF_sp_vecmul(chan1_V, cos_adc_dig ,chan1_i, N_S);
    DSPF_sp_vecmul(chan1_V, sin_adc_dig ,chan1_q, N_S);
    DSPF_sp_vecmul(chan2_V, cos_adc_dig ,chan2_i, N_S);
    DSPF_sp_vecmul(chan2_V, sin_adc_dig ,chan2_q, N_S);

    /*
     * This routine calculates the FIR filter response of the data supplied. The first argument
     * contains the input array which is our unfiltered i and q arrays. The second argument is an
     * an array containing the coefficients of the filter(Coeff). The third argument is the array
     * which will store the results of the filter. The fourth argument is the order of the filter
     * in integer format. The fifth argument is the number of output samples you need as a result
     * */
    int16_t out_sample = N_S - Filt_ord;
    DSPF_sp_fir_gen_cn(chan1_i, Coeff, ilp_1, 24, out_sample);
    DSPF_sp_fir_gen_cn(chan1_q, Coeff, qlp_1, 24, out_sample );
    DSPF_sp_fir_gen_cn(chan2_i, Coeff, ilp_2, 24, out_sample );
    DSPF_sp_fir_gen_cn(chan2_q, Coeff, qlp_2, 24, out_sample );

    /*The for loop below adds all the output samples from the FIR filter results to calculate the mean
     * of the I and Q values. */
    for ( k = 0; k < out_sample; k++) {
        *Itx += *(ilp_1+k);
        *Qtx += *(qlp_1+k);
        *Irx += *(ilp_2+k);
        *Qrx += *(qlp_2+k);
    }
    /*Dividing the total to get rhe mean*/
    *Itx /= out_sample;
    *Qtx /= out_sample;
    *Irx /= out_sample;
    *Qrx /= out_sample;

    *Amp1_V = (float)sqrt( (((double)*Itx) * ((double)*Itx)) + (((double)*Qtx) * ((double)*Qtx)) );
    *Amp2_V = (float)sqrt( (((double)*Irx) * ((double)*Irx)) + (((double)*Qrx) * ((double)*Qrx)) );
    /*The ratio of Q/I is saved in the pointer txratio and rxratio*/
    *txratio = (*Qtx)/(*Itx);
    *rxratio = (*Qrx)/(*Irx);

    /*  Tx Phase: */
    /*if txphase is positive than finding the respective quadrant*/
    if(*txratio >=0.0)
    {
        if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (txratio,phitx, 1);
        } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
            /* second quadrant */
            atansp_v (txratio,phitx, 1);
            *phitx += F_PIE_2;
        } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
            /*  third qudrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  F_PIE;
        } else {
            if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  F_PIE_32;
            }
        }
    }

    /* If tx phase is negative than finding the respective quadrant*/
    else {


        if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (txratio,phitx, 1);

        } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
            /* second quadrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  F_PIE;
        } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
            /*  third qudrant */
            atansp_v (txratio,phitx, 1);
            *phitx +=  F_PIE_32;
        } else {
            if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  F_2PIE;
            }
        }
    }

    /*  Rx Phase: */
    /* If Rx phase is positive than finding the respective quadrant*/
    if(*rxratio >= 0.0)
    {

        if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (rxratio,phirx, 1);
        } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
            /* second quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += F_PIE_2;
        } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
            /*  third qudrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += F_PIE;
        } else {
            if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx +=  F_PIE_32;
            }
        }
    }

    /* If Rx phase is negative than finding the respective quadrant*/
    else
    {
        if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
            /*  first Quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += F_PIE_2;
        } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
            /* second quadrant */
            atansp_v (rxratio,phirx, 1);
            *phirx +=  F_PIE;
        } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
            /*  third qudrant */
            atansp_v (rxratio,phirx, 1);
            *phirx += F_PIE_32;
        } else {
            if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                /*  fourth quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx +=  F_2PIE;
            }
        }
    }

    /*calculating hte phase difference*/
    if(*phitx> *phirx){
        phasediff = (*phitx)-(*phirx);
    }
    else {
        phasediff = F_2PIE-(*phirx)+(*phitx);
    }


    *Itx = sqrt( ((*Itx) * (*Itx)) + ((*Qtx) * (*Qtx)) );
    *Qtx = 0;
    *Irx = sqrt( ((*Irx) * (*Irx)) + ((*Qrx) * (*Qrx)) );
    *Qrx = (*Irx) * cos((double) phasediff)/ (*Itx);
    *Irx = (*Irx) * sin( (double) phasediff)/ (*Itx);
    /* Phasedifference in degree*/
    phasediff *= DEG_CONS;

}

#else

/*
 * Below are defined constants used in algorithm*/
    /*constant for pie*/
    float F_PIE = 3.1415926535897931;
    /*constant for pie/2*/
    float F_PIE_2 = 1.570796326794897;
    /*constant for 3*pie/2 */
    float F_PIE_32 = 4.712388980384690;
    /*constant for 2*pie*/
    float F_2PIE = 6.2831853071795871;
    /*constant for converting degree to radian*/
    float DEG_CONS = 57.295779513082323;
    /*constant for converting digital values to voltages*/
    float ADC_SCALE = 32768.0;

/* Number of samples for which FFT needs to be calculated */
uint16_t N = BUFLEN;
/*counter to rearrange the array in real and imaginary numbers*/
uint16_t comcnt = 0;

/*assign the twiddle factor a part of memory which is 8 bytes aligned*/
#pragma DATA_ALIGN(w_sp, 8);
float   w_sp [2*BUFLEN];
int twi_cnt;

/*Bit reversal needed to calculate the FFT in the algorithm*/
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
    /*for loop that convert the number of samples with digital values into voltage values*/
    for(index =0 ; index <(N)-1; index++)
    {
        *(chan1_V+index)=(float)*(channel1+index);
        *(chan2_V+index)=(float)*(channel2+index);
        /*converting value into voltages*/
        *(chan1_V+index)= (float)(*(chan1_V+index) / ADC_SCALE);
        *(chan2_V+index)= (float)(*(chan2_V+index) / ADC_SCALE);

    }

     /*
     * This routine calculates the dot product of the received signal(chan1_V and chan2_V)
     *  and the internal generated reference signal(cos_adc_dig and sin_adc_dig). The dot
     *  product results are stored in as unfiltered I and Q values in the respective arrays
     *  named as chan1_i, chan1_q, chan2_i, chan2_q. N_S is the number of samples that needs
     *  tom be multiplied*/
    DSPF_sp_vecmul(chan1_V, cos_adc_dig ,chan1_i, N);
    DSPF_sp_vecmul(chan1_V, sin_adc_dig ,chan1_q, N);
    DSPF_sp_vecmul(chan2_V, cos_adc_dig ,chan2_i, N);
    DSPF_sp_vecmul(chan2_V, sin_adc_dig ,chan2_q, N);

    /*Max value function that calculates the maximum value of an array
     *  and gives the peak amplitude if the signal*/
//    *Amp1_V = DSPF_sp_maxval(chan1_V, N);
//    *Amp2_V = DSPF_sp_maxval(chan2_V, N);

    /* To calculate FFT we need to pass an array which contains the real and imaginary values
     * of the signal arranged alternatively. For example: [Re0, Im0, Re1, Im1, Re3, Im3....]*,
     * but our imaginary part of the signal is zero because the signal is real and therefore
     * we are creating an array that contains the data asmentioned above*/
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

    /*Resetting the above loop counter*/
    comcnt = 0;

if (twi_cnt <1)
{
    /* Genarate twiddle factors */
    gen_twiddle_fft_sp(w_sp, N);
    twi_cnt++;
}
    /* Call FFT routine */
    DSPF_sp_fftSPxSP_cn(N, ilp_1, w_sp, Itx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, qlp_1, w_sp, Qtx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, ilp_2, w_sp, Irx, brev, 4, 0, N);
    DSPF_sp_fftSPxSP_cn(N, qlp_2, w_sp, Qrx, brev, 4, 0, N);

    /*The ratio of Q/I is saved in the pointer txratio and rxratio*/
    *txratio = (*Qtx)/(*Itx);
    *rxratio = (*Qrx)/(*Irx);

    /*Scaling the FFT output values*/
    *Itx /= N;
    *Qtx /= N;
    *Irx /= N;
    *Qrx /= N;

    *Amp1_V = (float)sqrt( (((double)*Itx) * ((double)*Itx)) + (((double)*Qtx) * ((double)*Qtx)) );
    *Amp2_V = (float)sqrt( (((double)*Irx) * ((double)*Irx)) + (((double)*Qrx) * ((double)*Qrx)) );
    /*  Tx Phase: */
    /*if txphase is positive than finding the respective quadrant*/

    if(*txratio >=0.0)
        {
            if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
                /*  first Quadrant */
                atansp_v (txratio,phitx, 1);
            } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
                /* second quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx += F_PIE_2;
            } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
                /*  third qudrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  F_PIE;
            } else {
                if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                    /*  fourth quadrant */
                    atansp_v (txratio,phitx, 1);
                    *phitx +=  F_PIE_32;
                }
            }
        }

    /* If tx phase is negative than finding the respective quadrant*/
        else {

            if ((*Itx >= 0.0) && (*Qtx >= 0.0)) {
                /*  first Quadrant */
                atansp_v (txratio,phitx, 1);

            } else if ((*Itx < 0.0) && (*Qtx >= 0.0)) {
                /* second quadrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  F_PIE;
            } else if ((*Itx < 0.0) && (*Qtx < 0.0)) {
                /*  third qudrant */
                atansp_v (txratio,phitx, 1);
                *phitx +=  F_PIE_32;
            } else {
                if ((*Itx >= 0.0) && (*Qtx < 0.0)) {
                    /*  fourth quadrant */
                    atansp_v (txratio,phitx, 1);
                    *phitx +=  F_2PIE;
                }
            }
        }

        /*  Rx Phase: */
        /* If Rx phase is positive than finding the respective quadrant*/
        if(*rxratio >= 0.0)
        {

            if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
                /*  first Quadrant */
                atansp_v (rxratio,phirx, 1);
            } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
                /* second quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx += F_PIE_2;
            } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
                /*  third qudrant */
                atansp_v (rxratio,phirx, 1);
                *phirx += F_PIE;
            } else {
                if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                    /*  fourth quadrant */
                    atansp_v (rxratio,phirx, 1);
                    *phirx +=  F_PIE_32;
                }
            }
        }

        /* If Rx phase is negative than finding the respective quadrant*/
        else
        {
            if ((*Irx >= 0.0) && (*Qrx >= 0.0)) {
                /*  first Quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx += F_PIE_2;
            } else if ((*Irx < 0.0) && (*Qrx >= 0.0)) {
                /* second quadrant */
                atansp_v (rxratio,phirx, 1);
                *phirx +=  F_PIE;
            } else if ((*Irx < 0.0) && (*Qrx < 0.0)) {
                /*  third qudrant */
                atansp_v (rxratio,phirx, 1);
                *phirx += F_PIE_32;
            } else {
                if ((*Irx >= 0.0) && (*Qrx < 0.0)) {
                    /*  fourth quadrant */
                    atansp_v (rxratio,phirx, 1);
                    *phirx +=  F_2PIE;
                }
            }
        }

        if(*phitx> *phirx){
            phasediff = (*phitx)-(*phirx);
        }
        else {
            phasediff = F_2PIE-(*phirx)+(*phitx);
        }


        *Itx = (float)sqrt( (((double)*Itx) * ((double)*Itx)) + (((double)*Qtx) * ((double)*Qtx)) );
        *Qtx = 0;

        *Irx = (float)sqrt( (((double)*Irx) * ((double)*Irx)) + (((double)*Qrx) * ((double)*Qrx)) );
        *Qrx = (*Irx) *((float) cos((double) phasediff))/ (*Itx);
        *Irx = (*Irx) *((float) sin( (double) phasediff))/ (*Itx);

        *Qrx = (*Qrx);
        *Irx = (*Irx);

        /* Phase difference in degree */
        phasediff *= DEG_CONS;

}
#endif
