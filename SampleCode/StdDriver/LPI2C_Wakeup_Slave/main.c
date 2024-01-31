/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief
 *          Show how to wake up MCU from Power-down mode through LPI2C interface.
 *          This sample code needs to work with LPI2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_POWER_DOWN_MODE    PMC_NPD0

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t slave_buff_addr;
uint8_t g_au8SlvData[256];
uint8_t g_au8SlvRxData[3];
uint8_t g_u8SlvPWRDNWK, g_u8SlvLPI2CWK;
uint8_t g_u8DeviceAddr;
uint8_t g_u8SlvDataLen;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;


NVT_ITCM void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /* Check LPI2C Wake-up interrupt flag set or not */
    if (LPI2C_GET_WAKEUP_FLAG(LPI2C0))
    {
        g_u8SlvLPI2CWK = LPI2C0->WKSTS;

        /* Waiting for LPI2C response ACK finish */
        while (!(LPI2C0->WKSTS & LPI2C_WKSTS_WKAKDONE_Msk));

        /* Clear Wakeup done flag, LPI2C will release bus */
        LPI2C0->WKSTS = LPI2C_WKSTS_WKAKDONE_Msk;
        /* Clear LPI2C Wake-up interrupt flag */
        LPI2C_CLEAR_WAKEUP_FLAG(LPI2C0);
        // CPU read interrupt flag register to wait write(clear) instruction completement.
        LPI2C_GET_WAKEUP_FLAG(LPI2C0);
        return;
    }

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
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PMC_IRQHandler(void)
{
    uint32_t u32Status;

    /* check power down wakeup flag */
    if ((PMC->INTSTS & PMC_INTSTS_PDWKIF_Msk) == PMC_INTSTS_PDWKIF_Msk)
    {
        g_u8SlvPWRDNWK = PMC->INTSTS;
        PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;
        // CPU read interrupt flag register to wait write(clear) instruction completement.
        u32Status = PMC->INTSTS;
    }

    NVT_UNUSED(u32Status);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Set Power-down mode */
    PMC_SetPowerDownMode(TEST_POWER_DOWN_MODE, PMC_PLCTL_PLSEL_PL1);
    /* Enter to Power-down mode */
    PMC_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Slave Transmit/Receive Callback Function                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_SlaveTRx(uint32_t u32Status)
{
    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char)LPI2C_GET_DATA(LPI2C0);
        g_u8SlvDataLen++;

        if (g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }

        if (g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }

        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in LPI2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
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
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
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
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable LPI2C0 module clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);
    /* Set multi-function pins for LPI2C0 SDA and SCL */
    SET_LPI2C0_SDA_PB4();
    SET_LPI2C0_SCL_PB5();
    /* LPI2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOB_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PB, (BIT4 | BIT5));
    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C0_Init(void)
{
    /* Open LPI2C module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);
    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));
    /* Set LPI2C 4 Slave Addresses */
    LPI2C_SetSlaveAddr(LPI2C0, 0, 0x15, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    LPI2C_SetSlaveAddr(LPI2C0, 1, 0x35, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    LPI2C_SetSlaveAddr(LPI2C0, 2, 0x55, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    LPI2C_SetSlaveAddr(LPI2C0, 3, 0x75, LPI2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
    /* Set LPI2C 4 Slave Addresses Mask */
    LPI2C_SetSlaveAddrMask(LPI2C0, 0, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 1, 0x04);
    LPI2C_SetSlaveAddrMask(LPI2C0, 2, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 3, 0x04);
    /* Enable LPI2C interrupt */
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
        This sample code is LPI2C SLAVE mode and it simulates EEPROM function
    */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("| LPI2C Driver Sample Code (Slave) for wake-up & access Slave test     |\n");
    printf("|                                                                      |\n");
    printf("| LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)                      |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("Configure LPI2C0 as a slave.\n");
    printf("The I/O connection for LPI2C0:\n");
    /* Init LPI2C0 */
    LPI2C0_Init();

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* LPI2C function to Transmit/Receive data as slave */
    s_LPI2C0HandlerFn = LPI2C_SlaveTRx;
    /* Set LPI2C0 enter Not Address SLAVE mode */
    LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable power wake-up interrupt */
    PMC->INTEN |= PMC_INTEN_PDWKIEN_Msk;
    NVIC_EnableIRQ(PMC_IRQn);
    g_u8SlvPWRDNWK = 0;
    /* Enable LPI2C wake-up */
    LPI2C_EnableWakeup(LPI2C0);
    g_u8SlvLPI2CWK = 0;
    printf("\n");
    printf("\n");
    printf("Press any key to enter power down status.\n");
    getchar();

    if (((LPI2C0->CTL0)&LPI2C_CTL0_SI_Msk) != 0)
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }

    /* Enter to Power-down mode */
    PowerDownFunction();
    u32TimeOutCnt = LPI2C_TIMEOUT;

    /* Waiting for system wake-up and LPI2C wake-up finish */
    while ((g_u8SlvPWRDNWK == 0) && (g_u8SlvLPI2CWK == 0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait LPI2C wake-up finish time-out!\n");

            while (1);
        }
    }

    /* Wake-up Interrupt Message */
    printf("Power-down Wake-up INT 0x%x\n", g_u8SlvPWRDNWK);
    printf("LPI2C0 WAKE INT 0x%x\n", g_u8SlvLPI2CWK);
    /* Disable power wake-up interrupt */
    PMC->INTEN &= ~PMC_INTEN_PDWKIEN_Msk;
    NVIC_DisableIRQ(PMC_IRQn);
    /* Lock protected registers */
    SYS_LockReg();
    printf("\n");
    printf("Slave wake-up from power down status.\n");
    printf("\n");
    printf("Slave Waiting for receiving data.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
