/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to implement PS/2 host protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "PS2_Host_driver.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
S_PSIO_PS2 g_sConfig;
volatile uint32_t g_u32TxData = 0, g_u32RxData = 0;
volatile uint32_t g_u32RxACK = 1;

NVT_ITCM void PSIO_IRQHandler(void)
{
    static uint8_t u8BitNumber = 0;
    uint8_t u8INT0Flag;
    uint32_t u32TimeOutCnt;

    /* Get INT0 interrupt flag */
    u8INT0Flag = PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);

    if (u8INT0Flag)
    {
        /* Clear INT0 interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);
    }
    else
    {
        printf("Unknown interrupt occur!!!\n");
    }

    if ((PSIO_PS2_GET_STATUS() == eHOST_READ) || (PSIO_PS2_GET_STATUS() == eHOST_READY_TO_READ))
    {
        static uint32_t u32RxBuffer = 0;

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

        /* Wait input buffer full */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk << (g_sConfig.u8DataPin * 4)))
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PSIO time-out!\n");
                return;
            }
        }

        /* Receive 11 bit */
        u32RxBuffer |= (PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin) << u8BitNumber);

        if (u8BitNumber == 10)
        {
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
            g_u32RxData = u32RxBuffer;
            u32RxBuffer = 0;
        }
        else
        {
            PSIO_PS2_SET_STATUS(eHOST_READ);
            u8BitNumber++;
        }
    }
    else if (PSIO_PS2_GET_STATUS() == eHOST_WRITE)
    {
        if (u8BitNumber == 10)
        {
            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC))
            {
                if (--u32TimeOutCnt == 0)
                {
                    printf("Wait for PSIO time-out!\n");
                    return;
                }
            }

            /* Read data from buffer */
            g_u32RxACK = PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin);

            /* Set pin interval status as high */
            PSIO->GNCT[g_sConfig.u8DataPin].GENCTL = (PSIO->GNCT[g_sConfig.u8DataPin].GENCTL & ~PSIO_GNCT_GENCTL_INTERVAL_Msk)
                                                     | (PSIO_HIGH_LEVEL << PSIO_GNCT_GENCTL_INTERVAL_Pos);
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
        }
        else if (u8BitNumber == 9)
        {

            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC))
            {
                if (--u32TimeOutCnt == 0)
                {
                    printf("Wait for PSIO time-out!\n");
                    return;
                }
            }

            /* Set check point action */
            /* For more efficient, accessing register directly */
            PSIO->GNCT[g_sConfig.u8DataPin].CPCTL1   = PSIO_IN_BUFFER;     //input buffer
            u8BitNumber++;
        }
        else
        {
            /* Send 9 bit */
            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            u8BitNumber++;
        }
    }
}

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

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 1 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(1));

    /* Enable UART module clock */
    SetDebugUartCLK();

    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

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
    uint32_t u32RxData = 0x0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("******************************************************\n");
    printf("|               PS/2 Host Sample Code                |\n");
    printf("|      Please connected PSIO_CH0(PE.14)(Clock)       |\n");
    printf("|      and PSIO_CH1(PE.15)(Data).                     |\n");
    printf("******************************************************\n");

    /* Use slot controller 0/1 and pin 0/1 */
    g_sConfig.u8ClockSC        = PSIO_SC0;
    g_sConfig.u8DataSC         = PSIO_SC1;
    g_sConfig.u8ClockPin       = PSIO_PIN0;
    g_sConfig.u8DataPin        = PSIO_PIN1;
    g_sConfig.p32ClockMFP      = &PE14;
    g_sConfig.p32DataMFP       = &PE15;

    /* Initialize PSIO setting for PS/2 host protocol */
    PSIO_PS2_Open(&g_sConfig);

    /* Lock protected registers */
    SYS_LockReg();

    while (1)
    {
        /* Set PSIO on read signal state */
        if (PSIO_PS2_GET_STATUS() != eHOST_READY_TO_READ)
        {
            PSIO_PS2_HostRead(&g_sConfig);
        }

        /* Receiving data */
        while (PSIO_PS2_GET_STATUS() == eHOST_READ);

        /* Data was received */
        if (PSIO_PS2_GET_STATUS() == eHOST_IDLE)
        {
            PSIO_PS2_SET_STATUS(eHOST_READY_TO_READ);
            u32RxData = g_u32RxData;
            printf("[Start]0x%x, [Data]0x%x, [Parity]0x%x, [Stop]0x%x\n",
                   u32RxData & 0x1, (u32RxData >> 1) & 0xFF, (u32RxData >> 9) & 0x1, (u32RxData >> 10) & 0x1);
        }

        /* Send data */
        if (!(DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            uint32_t u32Data;

            u32Data = DEBUG_PORT->DAT;
            g_u32TxData = PSIO_Encode_TxData(&u32Data);
            printf("[Host send to device]0x%x, 0x%x\n", u32Data, g_u32TxData);

            if (PSIO_PS2_HostSend(&g_sConfig) < 0)
                goto lexit;

            while (PSIO_PS2_GET_STATUS() == eHOST_WRITE);
        }
    }

lexit:

    while (1);

}
