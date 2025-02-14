/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use I3C0 Master to process In-Band Interrupt request from an I3C Slave.
 *          This sample code needs to work with I3C_Slave_IBI.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t   g_RxBuf[I3C_DEVICE_RX_BUF_CNT], g_TxBuf[I3C_DEVICE_RX_BUF_CNT];

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set SCL slew rate to GPIO_SLEWCTL_FAST0, SDA slew rate to GPIO_SLEWCTL_HIGH */
    GPIO_SetSlewCtl(PB, BIT1, GPIO_SLEWCTL_FAST0);
    GPIO_SetSlewCtl(PB, BIT0, GPIO_SLEWCTL_HIGH);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t    i;
    uint8_t     *pu8Data;
    int32_t     ret;
    uint32_t    response, cnt, data;
    char        chTrgIO;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    I3C0 Master Read/Write and IBI Sample Code  |\n");
    printf("+------------------------------------------------+\n\n");
    /* Initial I3C0 default settings */
    I3C_Open(I3C0, I3C_MASTER, 0, 0);
    /* Enable I3C0 controller */
    I3C_Enable(I3C0);
    /* Dynamic Address for Enter Dynamic Address Assignment (ENTDAA) */
    I3C_SetDeviceAddr(I3C0, 0, I3C_DEVTYPE_I3C, 0x18, 0x00);

    while (1)
    {
        printf("press any key to broadcast ENTDAA\n");
        getchar();

        if (1 == I3C_BroadcastENTDAA(I3C0, 1))
        {
            printf("I3C Device found:\n");
            printf(" - Provisional ID = 0x%08X%02X\n", I3C0->DEV1CH[0], I3C0->DEV1CH[1]);
            printf(" - BCR, DCR = 0x%08X\n", I3C0->DEV1CH[2]);
            printf(" - DADR = 0x%08X\n\n", I3C0->DEV1CH[3]);
            break;
        }
    }

    I3C0->DEV1ADR |= I3C_DEVADR_IBIWDAT_Msk;
    I3C0->QUETHCTL = ((I3C0->QUETHCTL & I3C_QUETHCTL_IBIDATTH_Msk)
                      | (1 << I3C_QUETHCTL_IBIDATTH_Pos));
    pu8Data = (uint8_t *)g_TxBuf;

    for (i = 0; i < 16; i++)
    {
        pu8Data[i] = i;
    }

    while (1)
    {
        if (I3C0->INTSTS & I3C_INTSTS_IBITH_Msk)
        {
            response = I3C0->IBISTS;
            printf("IBI Status = 0x%08X\n", response);
            cnt = (response & I3C_IBISTS_DATLEN_Msk);
            cnt = (cnt + 3) / 4;

            for (i = 0; i < cnt; i++)
            {
                data = I3C0->IBIQUE;
                printf("IBI Data[%d] = 0x%08X\n", i, data);
            }
        }

        /*
            Press any key to to write and read I3C Slave.
            The write data should be equal to the received data.
        */
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            chTrgIO = (char)DEBUG_PORT->DAT;
            NVT_UNUSED(chTrgIO);
            printf("press any key to Write I3C Target \n");
            getchar();
            ret = I3C_Write(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_TxBuf, 16);

            if (ret != I3C_STS_NO_ERR)
            {
                printf("I3C_Write Fail\n");

                while (1);
            }

            printf("press any key to Read I3C Target \n");
            getchar();
            ret = I3C_Read(I3C0, 0, I3C_DEVI3C_SPEED_SDR0, (uint32_t *)g_RxBuf, 16);

            if (ret != I3C_STS_NO_ERR)
            {
                printf("I3C_Read Fail\n");

                while (1);
            }

            for (i = 0; i < 4; i++)
            {
                if (g_RxBuf[i] != g_TxBuf[i])
                {
                    printf("Compare Data Fail\n");

                    while (1);
                }
            }

            printf("Compare Data Pass\n\n");
            pu8Data[0] += 1;
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
