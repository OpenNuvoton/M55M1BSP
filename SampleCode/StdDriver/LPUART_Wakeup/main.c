/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system form Power-down mode by LPUART interrupt.
 * @note
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RS485_ADDRESS 0xC0

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_DataWakeUp(void);
void LPUART_CTSWakeUp(void);
void LPUART_RxThresholdWakeUp(void);
void LPUART_RS485WakeUp(void);
void LPUART_PowerDown_TestItem(void);
void LPUART_PowerDownWakeUpTest(void);

/* LPUART can support NPD0,NDP1,NPD3 power-down mode */
#define TEST_POWER_DOWN_MODE    PMC_NPD0

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

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select LPUART0 clock source is HIRC and Low Power UART module clock divider as 1*/
    CLK_SetModuleClock(LPUART0_MODULE, CLK_LPUARTSEL_LPUART0SEL_HIRC, CLK_LPUARTDIV_LPUART0DIV(1));

    /* Enable LPUART0 peripheral clock */
    CLK_EnableModuleClock(LPUART0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set debug uart multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PA multi-function pins for LPUART0 TXD and RXD */
    SET_LPUART0_RXD_PA0();
    SET_LPUART0_TXD_PA1();
    SET_LPUART0_nRTS_PB10();
    SET_LPUART0_nCTS_PB11();

}

void LPUART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPUART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset LPUART0 */
    SYS_ResetModule(SYS_LPUART0RST);

    /* Configure LPUART0 and set LPUART0 Baudrate */
    LPUART_Open(LPUART0, 9600);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init LPUART0 */
    LPUART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* clear all wakeup flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nLPUART Sample Program\n");

    /* LPUART Power-down and Wake-up sample function */
    LPUART_PowerDownWakeUpTest();

    printf("LPUART Sample Program End.\n");

    while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle LPUART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPUART0_IRQHandler(void)
{
    uint32_t u32Data;
    // TESTCHIP_ONLY
    CLK_WaitModuleClockReady(LPUART0_MODULE);
    // TESTCHIP_ONLY
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);

    if (LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_WKINT_Msk))    /* LPUART wake-up interrupt flag */
    {
        LPUART_ClearIntFlag(LPUART0, LPUART_INTSTS_WKINT_Msk);
        printf("LPUART wake-up.\n");
        UART_WAIT_TX_EMPTY(DEBUG_PORT);
    }
    else if (LPUART_GET_INT_FLAG(LPUART0, LPUART_INTSTS_RDAINT_Msk | LPUART_INTSTS_RXTOINT_Msk))    /* LPUART receive data available flag */
    {
        while (LPUART_GET_RX_EMPTY(LPUART0) == 0)
        {
            u32Data = LPUART_READ(LPUART0);

            if (u32Data & LPUART_DAT_PARITY_Msk)
                printf("Address: 0x%X\n", (u32Data & 0xFF));
            else
                printf("Data: 0x%X\n", u32Data);
        }
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_CTSWakeUp(void)
{
    /* Enable LPUART nCTS wake-up frunction */
    LPUART0->WKCTL |= LPUART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode NPD%d.\n", (int)(TEST_POWER_DOWN_MODE));
    printf("Toggle LPUART0 nCTS to wake-up system.\n\n");

}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_DataWakeUp(void)
{
    /* Enable LPUART data wake-up frunction */
    LPUART0->WKCTL |= LPUART_WKCTL_WKDATEN_Msk;

    /* Set LPUART data wake-up start bit compensation value.
       It indicates how many clock cycle selected by LPUART_CLK does the LPUART controller can get the first bit (start bit)
       when the device is wake-up from power-down mode.
       If LPUART_CLK is selected as HIRC(12MHz) and the HIRC stable time is about 52.03us,
       the data wake-up start bit compensation value can be set as 0x270. */
    LPUART0->DWKCOMP = 0x270;

    printf("System enter to Power-down mode NPD%d.\n", (int)(TEST_POWER_DOWN_MODE));
    printf("Send data with baud rate 9600bps to LPUART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Rx threshold and time-out Function                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_RxThresholdWakeUp(void)
{
    /* Wait data transmission is finished and select LPUART clock source as LXT */
    while ((LPUART0->FIFOSTS & LPUART_FIFOSTS_TXEMPTYF_Msk) == 0);

    while ((LPUART0->FIFOSTS & LPUART_FIFOSTS_RXIDLE_Msk) == 0);

    /* Keep LPUART clock for HIRC work in power down mode */
    LPUART0->AUTOCTL |=  LPUART_AUTOCTL_AOEN_Msk | LPUART_AUTOCTL_CKAWOEN_Msk;

    /* Set LPUART baud rate and baud rate compensation */
    LPUART_Open(LPUART0, 9600);

    /* Enable LPUART Rx Threshold and Rx time-out wake-up frunction */
    LPUART0->WKCTL |= LPUART_WKCTL_WKRFRTEN_Msk | LPUART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    LPUART0->FIFO = (LPUART0->FIFO & (~LPUART_FIFO_RFITL_Msk)) | LPUART_FIFO_RFITL_4BYTES;

    /* Enable LPUART Rx time-out function */
    LPUART_SetTimeoutCnt(LPUART0, 40);

    printf("System enter to Power-down mode NPD%d.\n", (int)(TEST_POWER_DOWN_MODE));
    printf("Send data with baud rate 9600bps to LPUART0 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART RS485 address match (AAD mode) function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_RS485WakeUp(void)
{

    /* Keep LPUART clock for HIRC work in power down mode */
    LPUART0->AUTOCTL |=  LPUART_AUTOCTL_AOEN_Msk | LPUART_AUTOCTL_CKAWOEN_Msk;

    /* Set LPUART baud rate and baud rate compensation */
    LPUART_Open(LPUART0, 9600);

    /* RS485 address match (AAD mode) setting */
    LPUART_SelectRS485Mode(LPUART0, LPUART_ALTCTL_RS485AAD_Msk, RS485_ADDRESS);

    /* Enable parity source selection function */
    LPUART0->LINE |= (LPUART_LINE_PSS_Msk | LPUART_LINE_PBE_Msk);

    /* Enable LPUART RS485 address match, Rx Threshold and Rx time-out wake-up frunction */
    LPUART0->WKCTL |= LPUART_WKCTL_WKRFRTEN_Msk | LPUART_WKCTL_WKRS485EN_Msk | LPUART_WKCTL_WKTOUTEN_Msk;

    /* Enable LPUART Rx time-out function */
    LPUART_SetTimeoutCnt(LPUART0, 40);

    printf("System enter to Power-down mode NPD%d.\n", (int)(TEST_POWER_DOWN_MODE));
    printf("Send RS485 address byte 0x%X with baud rate 9600bps to LPUART0 to wake-up system.\n\n", RS485_ADDRESS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  LPUART Power-down and wake-up test                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("| [3] Rx threshold and time-out wake-up test                |\n");
    printf("| [4] RS485 wake-up test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~4): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPUART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void LPUART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Lock protected registers */
    SYS_LockReg();

    /* Enable LPUART wake-up and receive data available interrupt */
    NVIC_EnableIRQ(LPUART0_IRQn);
    LPUART_EnableInt(LPUART0, LPUART_INTEN_WKIEN_Msk | LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RXTOIEN_Msk);

    LPUART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);

    switch (u32Item)
    {
        case '1':
            LPUART_CTSWakeUp();
            break;

        case '2':
            LPUART_DataWakeUp();
            break;

        case '3':
            LPUART_RxThresholdWakeUp();
            break;

        case '4':
            LPUART_RS485WakeUp();
            break;

        default:
            return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n\n");
    getchar();

    /* Disable LPUART wake-up function */
    LPUART0->WKCTL = 0;

    /* Disable LPUART Interrupt */
    LPUART_DisableInt(LPUART0, LPUART_INTEN_WKIEN_Msk | LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RXTOIEN_Msk);

}
