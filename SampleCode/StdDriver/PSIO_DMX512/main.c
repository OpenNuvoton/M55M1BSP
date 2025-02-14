/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to implement DMX512 protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "DMX512_driver.h"

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 8 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(8));

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PSIO multi-function pin CH0(PE.14) and CH1(PE.15) */
    SET_PSIO0_CH0_PE14();
    SET_PSIO0_CH1_PE15();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    S_PSIO_DMX512_CFG sConfig;
    uint16_t au16RxBuf[5];
    uint32_t i, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+ \n");
    printf("|   DMX512 Protocol Test Code                            | \n");
    printf("|   Please connected PSIO_CH0(PE.14) to PSIO_CH1(PE.15)   | \n");
    printf("+--------------------------------------------------------+ \n");

    /* Reset PSIO */
    SYS_ResetModule(SYS_PSIO0RST);

    /* Lock protected registers */
    SYS_LockReg();

    /* Use slot controller 0 and pin 0  for TX */
    /* Use slot controller 1 and pin 1  for RX */
    sConfig.u8TxSlotCounter      = PSIO_SC0;
    sConfig.u8RxSlotCounter      = PSIO_SC1;
    sConfig.u8TxPin              = PSIO_PIN0;
    sConfig.u8RxPin              = PSIO_PIN1;

    /* Initialize PSIO setting for DMX512 */
    PSIO_DMX512_Open(&sConfig);
    NVIC_EnableIRQ(PSIO_IRQn);

    while (1)
    {

        for (i = 1 ; i < 6 ; i++)
        {
            PSIO_DMX512_getChannelData(&sConfig, i, &au16RxBuf[0]);
            printf("press any key to continue\n");
            getchar();

            PSIO_DMX512_Tx(&sConfig, 0, eDMX512_BREAK_START);
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x59, eDMX512_DATA);     /* Channel 1 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x67, eDMX512_DATA);     /* Channel 2 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x79, eDMX512_DATA);     /* Channel 3 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x88, eDMX512_DATA);     /* Channel 4 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x55, eDMX512_DATA);     /* Channel 5 data */
            CLK_SysTickDelay(50);

            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (!(*sConfig.pu8RcvDone))
            {
                if (--u32TimeOutCnt == 0)
                {
                    printf("Wait for data time-out!\n");
                    goto lexit;
                }
            }

            printf("%u: 0x%02X\n", i, (uint8_t)DMX512_GET_DATA(au16RxBuf[0]));
        }
    }

lexit:

    while (1);
}
