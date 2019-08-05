/*
 * udpHello.c
 *
 * This program implements a UDP echo server, which get triggered when
 * data is sent to the DSP. The data can contain anything to trigger
 * the for loop inside this task. Once the data is received, it can be
 * processed inside this loop and action can be taken depending upon the
 * data(string) received.
 *
 */


#include <ti/ndk/inc/netmain.h>
#include"Myheader.h"
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>


int dtask_udp_hello( SOCKET s, uint32_t unused )
{
    /* Some fo the variables and struct of RTOS used to set the socket options*/
    struct sockaddr_in sin1;
    struct timeval     to;
    int                i, tmp, err, cnt;
    char               *pBuf;
    void*             hBuffer;
    float               ioffset,qoffset;
    int                 one, ten, hun, ton;
    int                 acs_offset = 48;
    uint16_t            new_freq = 0;

    double M_PIE = 3.14159265358979323846;
    double phi;
    double sample_time = 0.0;
    double sample_rate = 48000;
    double sinavg = 0, cosavg = 0;
    double time = 1/sample_rate;

    //    char laserdata[15] = {0};

    (void)unused;

    /*IP configurstion in 32bit decimal. This is something to be used when you have
     * to send the data to multiple IP's*/
    sin1.sin_family =AF_INET;
    /*Port = 7*/
    /*To cahnge the port you have to change it in helloWorld_omapl13x.c*/
    sin1.sin_port =7;
    /* IP = 169.254.43.115*/
    sin1.sin_addr.s_addr = 2852016755;
    /* Configure our socket timeout to be 3 seconds*/
    to.tv_sec  = 3;
    to.tv_usec = 0;
    setsockopt( s, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof( to ) );
    setsockopt( s, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof( to ) );
    /*Allocating memory to the string received*/
    pBuf = (char*)calloc(10, sizeof(char));

    /* The for-ever loop that is triggered when data is received*/
    for(;;)
    {
        /* The size of the address, it is used as an argument*/
        tmp = sizeof( sin1 );
        //  (*callback)(myuart1, laser_data, readsize_l);
        /*This functions receives the data from the ethernet port and saves the
         * data in the second argument. it returns the number of bytes received
         * from the UDP server*/
        i = (int)recvncfrom( s, (void **)&pBuf, 0, &sin1, &tmp, &hBuffer );

        /* Action that needs to be taken when any data is received*/
        if( i >= 0 )
        {
            /* Wrapping the data in the UDP packet to send it collectively*/
            for(i =0; i<128 ;i++)
            {
                mypack ->chan1_data[i] = *(chan1+i);
                mypack ->chan2_data[i] = *(chan2+i);
            }
            //            mypack ->itx = *Itx;
            //            mypack ->qtx = *Qtx;
            mypack ->irx = *Irx - ioffset;
            mypack ->qrx = *Qrx - qoffset;
            mypack ->chan1_amp = *Amp1_V;
            mypack ->chan2_amp = *Amp2_V;
            mypack ->phase_diff = phasediff;


            /*Wrapping of the data ends here*/
            /*This function sends the data starting from the address of the second argument
             * and the size of the data is specified in the third argument*/
            sendto( s,mypack,541, 0, &sin1, sizeof(sin1) ); //532 without the laser data

            if(pBuf[0] == 'N'&& pBuf[1] == 'U' && pBuf[2] == 'L' && pBuf[3] == 'L')
            {
                ioffset = *Irx;
                qoffset = *Qrx;
            }

            if(pBuf[0] == 'F')
            {

                one = (int) pBuf[4] - acs_offset;
                ten = (int) pBuf[3] - acs_offset;
                hun = (int) pBuf[2] - acs_offset;
                ton = (int) pBuf[1] - acs_offset;

                new_freq = (ton*1000) + (hun*100) + (ten*10) + one;

                for ( cnt = 0 ; cnt < BUFLEN-1; cnt++ )
                {
                    phi = 2 * M_PIE * new_freq * sample_time;
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
            }



            /*Checking for error if the send has failed*/
            err = fdError();

            /*Clearing the received data buffer*/
            recvncfree( hBuffer );

            /*Logging the task*/
            Log_info0("\r\nData sent via UDP\n");


        }
        else
            break;
    }

    /* Since the socket is still open, return "1"
     * (we need to leave UDP sockets open) */
    return(1);
}


