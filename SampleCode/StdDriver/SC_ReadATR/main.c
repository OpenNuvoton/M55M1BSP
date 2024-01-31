/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Read the smartcard ATR from smartcard 0 interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF         0 // Smartcard interface 0

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
NVT_ITCM void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */

    // Card insert/remove event occurred, no need to check other event...
    if (SCLIB_CheckCDEvent(SC_INTF))
        return;

    // Check if there's any timeout event occurs. If so, it usually indicates an error
    SCLIB_CheckTimeOutEvent(SC_INTF);

    // Check transmit and receive interrupt, all data transmission take place in this function
    SCLIB_CheckTxRxEvent(SC_INTF);

    /*
        Check if there's any transmission error occurred (e.g. parity error, frame error...)
        These errors will induce SCLIB to deactivation smartcard eventually.
    */
    SCLIB_CheckErrorEvent(SC_INTF);

    return;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable SC0 module clock and clock source from HIRC divide 3, 4MHz*/
    CLK_SetModuleClock(SC0_MODULE, CLK_SCSEL_SC0SEL_HIRC, CLK_SCDIV_SC0DIV(3));

    /* Enable module clock */
    CLK_EnableModuleClock(SC0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set SC0 multi-function pin */
    SET_SC0_PWR_PB2();
    SET_SC0_RST_PB3();
    SET_SC0_DAT_PB4();
    SET_SC0_CLK_PB5();
    SET_SC0_nCD_PC12();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    SCLIB_CARD_INFO_T sCardInfo;
    int32_t i32Ret;
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %uHz\n", SystemCoreClock);
    printf("+--------------------------------------+\n");
    printf("|    Read Smartcard ATR Sample Code    |\n");
    printf("+--------------------------------------+\n\n");
    printf("# I/O configuration:\n");
    printf("    SC0PWR (PB.2)  <--> smart card slot power pin\n");
    printf("    SC0RST (PB.3)  <--> smart card slot reset pin\n");
    printf("    SC0CLK (PB.5)  <--> smart card slot clock pin\n");
    printf("    SC0DAT (PB.4)  <--> smart card slot data pin\n");
    printf("    SC0CD  (PC.12) <--> smart card slot card detect pin\n");
    printf("\nThis sample code reads ATR from smartcard...\n");

    /*
        Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
        The second and third parameter needs to be set according to the board design
    */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    // Wait 'til card insert
    while (SC_IsCardInserted(SC0) == FALSE);

    /*
        Activate slot 0, and disable EMV2000 check during card activation
        EMV is a technical standard for smart payment cards and for payment terminals and automated teller
        machines that can accept them. It has a stricter checking rule than ISO 7816-3. If the second
        parameter set as TRUE, SCLIB will report activation failure for cards comply with ISO 7816 but not EMV2000
    */
    i32Ret = SCLIB_Activate(SC_INTF, FALSE);

    if (i32Ret == SCLIB_SUCCESS)
    {
        /*
            Use SCLIB_GetCardInfo to get information about the card, which includes ATR.

            An Answer To Reset (ATR) is a message output by a contact Smart Card conforming to
            ISO/IEC 7816 standards, following electrical reset of the card's chip by a card reader.
            The ATR conveys information about the communication parameters proposed by the card,
            and the card's nature and state.                                --Wikipedia
        */
        SCLIB_GetCardInfo(SC_INTF, &sCardInfo);
        printf("\nATR: ");

        for (i = 0; i < sCardInfo.ATR_Len; i++)
            printf("%02x ", sCardInfo.ATR_Buf[i]);

        printf("\n");
    }
    else
        printf("\nSmartcard activate failed\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
