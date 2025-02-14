/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Read/write EEPROM via LPI2C interface
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if (LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if (s_LPI2C0HandlerFn != NULL)
        {
            s_LPI2C0HandlerFn(u32Status);
        }
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = LPI2C_GET_STATUS(LPI2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Rx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen < 2)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA | LPI2C_CTL_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = LPI2C_GET_DATA(LPI2C0);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
        g_u8EndFlag = 1;
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Tx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        LPI2C_SET_DATA(LPI2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA | LPI2C_CTL_STO | LPI2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
            g_u8EndFlag = 1;
        }
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

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
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);
    /* Set multi-function pins for LPI2C0 SDA and SCL */
    SET_LPI2C0_SDA_PB4();
    SET_LPI2C0_SCL_PB5();
    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C0_Close(void)
{
    /* Disable LPI2C0 interrupt and clear corresponding NVIC bit */
    LPI2C_DisableInt(LPI2C0);
    NVIC_DisableIRQ(LPI2C0_IRQn);
    /* Disable LPI2C0 and close LPI2C0 clock */
    LPI2C_Close(LPI2C0);
    CLK_DisableModuleClock(LPI2C0_MODULE);
}

void LPI2C0_Init(void)
{
    /* Open LPI2C0 and set clock to 100k */
    LPI2C_Open(LPI2C0, 100000);
    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i, u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+-------------------------------------------------------+\n");
    printf("|        LPI2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");
    /* Init LPI2C0 to access EEPROM */
    LPI2C0_Init();
    g_u8DeviceAddr = 0x50;

    for (i = 0; i < 2; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);
        g_u8DataLen = 0;
        g_u8EndFlag = 0;
        /* LPI2C function to write data to slave */
        s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterTx;
        /* LPI2C as master sends START signal */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);
        /* Wait LPI2C Tx Finish */
        u32TimeOutCnt = LPI2C_TIMEOUT;

        while (g_u8EndFlag == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LPI2C Tx finish time-out!\n");

                while (1);
            }
        }

        g_u8EndFlag = 0;
        /* Make sure LPI2C0 STOP already */
        u32TimeOutCnt = LPI2C_TIMEOUT;

        while (LPI2C0->CTL0 & LPI2C_CTL0_STO_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LPI2C STOP time-out!\n");

                while (1);
            }
        }

        /* Wait write operation complete */
        CLK_SysTickDelay(10000);
        /* LPI2C function to read data from slave */
        s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterRx;
        g_u8DataLen = 0;
        g_u8DeviceAddr = 0x50;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);
        /* Wait LPI2C Rx Finish */
        u32TimeOutCnt = LPI2C_TIMEOUT;

        while (g_u8EndFlag == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LPI2C Rx finish time-out!\n");

                while (1);
            }
        }

        /* Make sure LPI2C0 STOP already */
        u32TimeOutCnt = LPI2C_TIMEOUT;

        while (LPI2C0->CTL0 & LPI2C_CTL0_STO_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LPI2C STOP time-out!\n");

                while (1);
            }
        }

        /* Compare data */
        if (g_u8RxData != g_au8TxData[2])
        {
            printf("LPI2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            goto lexit;
        }

        printf(".");
    }

    printf("LPI2C Access EEPROM Test OK\n");
lexit:
    /* LPI2C function to read data from slave */
    s_LPI2C0HandlerFn = NULL;
    LPI2C0_Close();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
