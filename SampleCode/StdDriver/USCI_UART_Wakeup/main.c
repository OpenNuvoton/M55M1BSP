/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system from Power-down mode by USCI interrupt in UART mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* UUART can support NPD0 ~ NDP1 power-down mode */
#define TEST_POWER_DOWN_MODE    PMC_NPD0

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void USCI_UART_DataWakeUp(void);
void USCI_UART_CTSWakeUp(void);
void USCI_UART_PowerDown_TestItem(void);
void USCI_UART_PowerDownWakeUpTest(void);
void PowerDownFunction(void);
void SYS_Init(void);
void USCI0_Init(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set Power-down mode */
    PMC_SetPowerDownMode(TEST_POWER_DOWN_MODE, PMC_PLCTL_PLSEL_PL0);

    /* Enter to Power-down mode */
    PMC_PowerDown();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable USCI0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9), USCI0_CTL0(PC.13) and USCI0_CTL1(PA.8) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
    SET_USCI0_CTL1_PA8();
    SET_USCI0_CTL0_PC13();

}


void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(SYS_USCI0RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Init USCI0 */
    USCI0_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("USCI UART Sample Program\n\n");

    /* USCI UART Power-down and Wake-up sample function */
    USCI_UART_PowerDownWakeUpTest();

    printf("\nUSCI UART Sample Program End\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI0 interrupt event                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void USCI0_IRQHandler(void)
{
    // TESTCHIP_ONLY
    CLK_WaitModuleClockReady(USCI0_MODULE);
    // TESTCHIP_ONLY
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);

    uint32_t u32IntSts = UUART_GET_PROT_STATUS(UUART0);
    uint32_t u32WkSts = UUART_GET_WAKEUP_FLAG(UUART0);

    if (u32WkSts & UUART_WKSTS_WKF_Msk) /* USCI UART wake-up flag */
    {
        UUART_CLR_WAKEUP_FLAG(UUART0);
        printf("USCI UART wake-up.\n");
    }
    else if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk) /* USCI UART receive end interrupt flag */
    {
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        while (UUART_GET_RX_EMPTY(UUART0) == 0)
        {
            printf("Data: 0x%X\n", UUART_READ(UUART0));
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_CTSWKEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Toggle USCI-UART0 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_DataWakeUp(void)
{
    uint16_t u16regCLKDIV, u16regPDSCNT, u16regDSCNT;
    uint32_t u32WakeupCount, u32PCLK;
    double fWakeupTime = 71;  /* unit: us */

    /* Get PCLK */
    u32PCLK = CLK_GetPCLK0Freq();

    /* Get Divider */
    u16regCLKDIV = (UUART0->BRGEN & UUART_BRGEN_CLKDIV_Msk) >> UUART_BRGEN_CLKDIV_Pos;
    u16regPDSCNT = (UUART0->BRGEN & UUART_BRGEN_PDSCNT_Msk) >> UUART_BRGEN_PDSCNT_Pos;
    u16regDSCNT = (UUART0->BRGEN & UUART_BRGEN_DSCNT_Msk) >> UUART_BRGEN_DSCNT_Pos;

    /* Calculate wake-up counter */
    u32WakeupCount = (uint32_t)(fWakeupTime * (u32PCLK / 1000000) / ((u16regCLKDIV + 1) * (u16regPDSCNT + 1) * (u16regDSCNT + 1)));

    if (u32WakeupCount > 15)
    {
        printf("Fail to calculate wake-up counter. USCI-UART would not get correct data after wake-up.\n");
    }

    /* Enable UART data wake-up function */
    UUART_EnableWakeup(UUART0, UUART_PROTCTL_DATWKEN_Msk);

    /* Set UART data wake-up counter.
       It indicates how many clock cycle does UART can get the first bit (start bit)
       when the device is wake-up from Power-down mode.
       Clock is (USCI clock source)/((CLKDIV+1)(PDSCNT+1)) */
    UUART0->PROTCTL = (UUART0->PROTCTL & (~UUART_PROTCTL_WAKECNT_Msk)) | (u32WakeupCount << UUART_PROTCTL_WAKECNT_Pos);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to USCI-UART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI-UART Power-down and wake-up test                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    printf("Due to PLL clock stable too slow.\n");
    printf("Before demo USCI UART wake-up, this demo code will switch HCLK from PLL to HIRC.\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nCPU @ %dHz\n", SystemCoreClock);

    /* Set UUART0 Line config */
    UUART_SetLine_Config(UUART0, 9600, UUART_WORD_LEN_8, UUART_PARITY_NONE, UUART_STOP_BIT_1);

    /* Enable UART receive end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    USCI_UART_PowerDown_TestItem();
    u32Item = (uint32_t)getchar();
    printf("%c\n\n", u32Item);

    switch (u32Item)
    {
        case '1':
            USCI_UART_CTSWakeUp();
            break;

        case '2':
            USCI_UART_DataWakeUp();
            break;

        default:
            return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Wait until USCI UART data transmission is finished */
    while (UUART0->PROTSTS & UUART_PROTSTS_RXBUSY_Msk);

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n");
    getchar();

    /* Disable UART wake-up function */
    UUART_DisableWakeup(UUART0);

    /* Disable UART receive end interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Wait the USCI0 peripheral clock  */
    CLK_WaitModuleClockReady(USCI0_MODULE);
    /* Wait the UART0 peripheral clock  */
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);
    /* Lock protected registers */
    SYS_LockReg();

    /* Set UUART0 Line config */
    UUART_SetLine_Config(UUART0, 9600, UUART_WORD_LEN_8, UUART_PARITY_NONE, UUART_STOP_BIT_1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
