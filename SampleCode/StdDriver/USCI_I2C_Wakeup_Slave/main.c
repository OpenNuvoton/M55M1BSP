/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to wake up USCI_I2C from Deep Sleep mode.
 *          This sample code needs to work with USCI_I2C_Master.
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
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8RxData[4];

volatile uint32_t slave_buff_addr;
volatile uint16_t g_u16RecvAddr;
volatile uint8_t g_u8DataLenS;
volatile uint8_t g_u8SlvPWRDNWK = 0, g_u8SlvI2CWK = 0;
volatile uint32_t g_u32WKfromAddr;

volatile enum UI2C_SLAVE_EVENT s_Event;

typedef void (*UI2C_FUNC)(uint32_t u32Status);

volatile static UI2C_FUNC s_UI2C0HandlerFn = NULL;
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

NVT_ITCM void USCI0_IRQHandler(void)
{
    uint32_t u32Status;
    CLK_EnableModuleClock(USCI0_MODULE);
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);

    if (s_UI2C0HandlerFn != NULL)
    {
        s_UI2C0HandlerFn(u32Status);
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = UI2C_GET_PROT_STATUS(UI2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C0 toggle wake-up                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SLV_Toggle_Wakeup(uint32_t u32Status)
{
    uint32_t temp;

    if ((UI2C0->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        g_u32WKfromAddr = 0;
        g_u8SlvI2CWK = 1;
        /* Clear WKF INT Flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C0);
        return;
    }

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);
        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C0, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }

            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            temp = UI2C_GET_DATA(UI2C0);
            g_au8RxData[g_u8DataLenS] = temp;
            g_u8DataLenS++;

            if (g_u8DataLenS == 2)
            {
                temp = (g_au8RxData[0] << 8);
                temp += g_au8RxData[1];
                slave_buff_addr = temp;
            }

            if (g_u8DataLenS == 3)
            {
                temp = g_au8RxData[2];
                g_au8SlvData[slave_buff_addr] = temp;
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);
        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UI2C0 address match wake-up                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UI2C_SLV_Address_Wakeup(uint32_t u32Status)
{
    uint32_t temp;

    if ((UI2C0->WKSTS & UI2C_WKSTS_WKF_Msk) == UI2C_WKSTS_WKF_Msk)
    {
        g_u32WKfromAddr = 1;
        g_u8SlvI2CWK = 1;

        /* */
        while ((UI2C0->PROTSTS & UI2C_PROTSTS_WKAKDONE_Msk) == 0) {};

        /* Clear WK flag */
        UI2C_CLR_WAKEUP_FLAG(UI2C0);

        UI2C0->PROTSTS = UI2C_PROTSTS_WKAKDONE_Msk;

        return;
    }

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);
        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (s_Event == SLAVE_ADDRESS_ACK)
        {
            g_u8DataLenS = 0;

            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                s_Event = SLAVE_SEND_DATA;
                UI2C_SET_DATA(UI2C0, g_au8SlvData[slave_buff_addr]);
                slave_buff_addr++;
            }
            else
            {
                s_Event = SLAVE_GET_DATA;
            }

            g_u16RecvAddr = (uint8_t)UI2C_GET_DATA(UI2C0);
        }
        else if (s_Event == SLAVE_GET_DATA)
        {
            temp = UI2C_GET_DATA(UI2C0);
            g_au8RxData[g_u8DataLenS] = temp;
            g_u8DataLenS++;

            if (g_u8DataLenS == 2)
            {
                temp = (g_au8RxData[0] << 8);
                temp += g_au8RxData[1];
                slave_buff_addr = temp;
            }

            if (g_u8DataLenS == 3)
            {
                temp = g_au8RxData[2];
                g_au8SlvData[slave_buff_addr] = temp;
                g_u8DataLenS = 0;
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);
        /* Event process */
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        g_u8DataLenS = 0;
        s_Event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
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
    /* Enable USCI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);
    /* Set multi-function pins for UI2C0 SDA and SCL */
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    /* USCI_I2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPIOA_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT10 | BIT11));
    /* Lock protected registers */
    SYS_LockReg();
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open UI2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);
    /* Get UI2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));
    /* Set UI2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    /* Set UI2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address : 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address : 0x4 */
    /* Enable UI2C0 protocol interrupt */
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk));
    NVIC_EnableIRQ(USCI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i, u32TimeOutCnt;
    uint8_t  ch;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("| USCI_I2C Driver Sample Code (Slave) for wake-up & access Slave test |\n");
    printf("| Needs to work with USCI_I2C_Master sample code.                     |\n");
    printf("|      UI2C Master (I2C0) <---> UI2C Slave (I2C0)                     |\n");
    printf("| !! This sample code requires two boards for testing !!              |\n");
    printf("+---------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure UI2C0 as a Slave\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA10), UI2C0_SCL(PA11)\n");
    /* Init UI2C0 100KHz */
    UI2C0_Init(100000);
    printf("[T] I/O Toggle Wake-up Mode\n");
    printf("[A] Address Match Wake-up Mode\n");
    printf("Select: ");
    ch =  getchar();

    if ((ch == 'T') || (ch == 't'))
    {
        printf("(T)oggle\n");
        /* Enable UI2C0 toggle mode wake-up */
        UI2C_EnableWakeup(UI2C0, UI2C_DATA_TOGGLE_WK);
        s_Event = SLAVE_ADDRESS_ACK;
        /* I2C function to Slave receive/transmit data */
        s_UI2C0HandlerFn = UI2C_SLV_Toggle_Wakeup;
    }
    else
    {
        /* Default Mode*/
        printf("(A)ddress math\n");
        /* Enable UI2C0 address match mode wake-up */
        UI2C_EnableWakeup(UI2C0, UI2C_ADDR_MATCH_WK);
        s_Event = SLAVE_GET_DATA;
        /* UI2C0 function to Slave receive/transmit data */
        s_UI2C0HandlerFn = UI2C_SLV_Address_Wakeup;
    }

    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable power wake-up interrupt */
    PMC->INTEN |= PMC_INTEN_PDWKIEN_Msk;
    NVIC_EnableIRQ(PMC_IRQn);
    g_u8SlvPWRDNWK = 0;
    /* System power down enable */
    printf("\nCHIP enter power down status.\n");
    /* Waiting for UART printf finish*/
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Clear flag before enter power-down mode */
    if (UI2C0->PROTSTS != 0)
    {
        UI2C0->PROTSTS = UI2C0->PROTSTS;
    }

    /* Set Power-down mode */
    PMC_SetPowerDownMode(TEST_POWER_DOWN_MODE, PMC_PLCTL_PLSEL_PL1);
    /* Enter to Power-down mode */
    PMC_PowerDown();
    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();
    u32TimeOutCnt = UI2C_TIMEOUT;

    /* Waiting for system wake-up and USCI_I2C wake-up finish */
    while ((g_u8SlvPWRDNWK == 0) && (g_u8SlvI2CWK == 0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait USCI_I2C wake-up finish time-out!\n");

            while (1);
        }
    }

    if (g_u32WKfromAddr)
    {
        printf("UI2C0 [A]ddress match Wake-up from Deep Sleep\n");
    }
    else
    {
        printf("UI2C0 [T]oggle Wake-up from Deep Sleep\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
