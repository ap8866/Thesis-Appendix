/*
 * audioSample_main.c
 *


/* file   audioSample_main.c*/


/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */

#include <xdc/std.h>
#include <string.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/io/GIO.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <mcasp_drv.h>
#include <ti/sysbios/io/IOM.h>
#ifdef AIC_CODEC
#include <Aic31.h>
#endif
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#if !defined(SOC_AM65XX)
#include <ti/sdo/edma3/drv/edma3_drv.h>
#include <ti/csl/csl_edma3.h>
#endif

#include <ti/csl/csl_bootcfg.h>
#include <ti/drv/mcasp/soc/mcasp_soc.h>
#include"Myheader.h"



/* Handle to the EDMA driver instance                                         */
EDMA3_DRV_Handle hEdma;

uint32_t baud = 38400;
extern void configureAudio(void);
extern void configMcASP_SocHwInfo(void);
void Board_initUART(void);
/* ========================================================================== */
/*                           FUNCTION DEFINITIONS                             */
/* ========================================================================== */


/**
 *  \brief  Void main(Void)
 *
 *   Main function of the sample application. This function enables
 *   the mcasp instance in the power sleep controller and also
 *   enables the pinmux for the mcasp instance. This also powers up
 *   any codecs if attached to McASP like the AIC codec, before switching to 
 *   the task to Audio_echo_task().
 *
 *  \param  None
 *  \return None
 */
volatile int emuwait=0;
int main()
{
    Board_initUART();
    UART_socGetInitCfg(1U, &uart_cfg);
    UART_socSetInitCfg(1U, &uart_cfg);
    //while(emuwait);
    /* enable the pinmux & PSC-enable for the mcasp device    */
    configureAudio();

    /* Initializing McASP HwInfo parameters */
    McaspDevice_init();

    /* Perform SOC specific McASP HwInfo Configuration for non-default parameters
     * using the socGetConfig() and socSetConfig(). Please note that
      this is being called AFTER McaspDevice_init() which initializes with the
      default parameters */
    configMcASP_SocHwInfo();
#if defined(AIC_CODEC)
    Aic31_init();
#endif

    UART_Params_init(&params_uart);
    params_uart.baudRate = baud;
    params_uart.readCallback = callback;
    params_uart.readReturnMode = UART_RETURN_FULL;
    params_uart.readTimeout = 1000;
    myuart1 = UART_open(1U , &params_uart);
    Log_info0("\r\nAudio Sample Main\n");

    BIOS_start();

    return 0;
}
/*
 * Mcasp init function called when creating the driver.
 */

void Board_initUART(void)
{
    Board_initCfg boardCfg;


    boardCfg = BOARD_INIT_PINMUX_CONFIG |
            BOARD_INIT_MODULE_CLOCK  |
            BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);
}


/* ========================================================================== */
/*                                END OF FILE                                 */
/* ========================================================================== */
