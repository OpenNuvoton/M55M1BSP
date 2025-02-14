/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to implement NEC IR protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NEC_IR_driver.h"

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal LIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO0_MODULE);

    /* Select PSIO module clock source as LIRC and PSIO module clock divider as 2 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_LIRC, CLK_PSIODIV_PSIO0DIV(2));

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOE_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PSIO multi-function pin CH0(PE.14) */
    SET_PSIO0_CH0_PE14();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_PSIO_NEC_CFG sConfig;
    volatile int32_t i32TimeOutCnt = SystemCoreClock;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|               NEC IR sample code                     | \n");
    printf("|      Please check waveform on PSIO_CH0(PE.14)        | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0 and pin 0 */
    sConfig.u8SlotCtrl   = PSIO_SC0;
    sConfig.u8TxPin      = PSIO_PIN0;

    /* Initialize PSIO setting for NEC IR protocol */
    PSIO_NEC_Open(&sConfig);

    /* Send  0x1(Address), ~0x1(/Address), 0x2(Command), ~0x2(/Command) */
    PSIO_NEC_Send(&sConfig, 0x1, ~0x1, 0x2, ~0x2);

    /* Wait transfer done */
    i32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (PSIO_NEC_TransferDone(&sConfig))
    {
        if (--i32TimeOutCnt <= 0)
        {
            printf("Wait for PSIO transfer done time-out!\n");
            goto lexit;
        }
    }

    /* Send  Repeat signal */
    PSIO_NEC_Repeat(&sConfig);

    /* Wait transfer done */
    i32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (PSIO_NEC_TransferDone(&sConfig))
    {
        if (--i32TimeOutCnt <= 0)
        {
            printf("Wait for PSIO transfer done time-out!\n");
            goto lexit;
        }
    }

    /* Release PSIO setting */
    PSIO_NEC_Close(&sConfig);

    printf("Complete!\n");

lexit:

    while (1);
}
