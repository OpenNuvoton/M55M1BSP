/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to implement Wiegand26 protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "HZ1050_driver_RFID.h"

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

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

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 2 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(2));

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
    S_PSIO_HZ1050 sConfig;
    uint32_t u32Data = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|          HZ1050 WIEGAND26 RFID Test Code             | \n");
    printf("| Please connected PSIO_CH0(PE.14) and PSIO_CH1(PE.15) | \n");
    printf("| to device.                                           | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0 and pin 0 */
    sConfig.u8SlotCtrl   = PSIO_SC0;
    sConfig.u8Data0Pin   = PSIO_PIN0;
    sConfig.u8Data1Pin   = PSIO_PIN1;

    /* Reset PSIO */
    SYS_ResetModule(SYS_PSIO0RST);

    /* Lock protected registers */
    SYS_LockReg();

    /* Initialize PSIO setting for HZ1050 */
    PSIO_HZ1050_Init(&sConfig);

    do
    {
        /* Read data from HZ1050 */
        PSIO_HZ1050_Read(&sConfig, &u32Data);

        printf("[Even Parity]:0x%x, [Odd Parity]:0x%x, [Facility Code]:0x%x, [Card Code]:0x%x \n"
               , (u32Data & EVEN_PARITY_MSK) >> EVEN_PARITY_POS, (u32Data & ODD_PARITY_MSK) >> ODD_PARITY_POS
               , (u32Data & FACILITY_CODE_MSK) >> FACILITY_CODE_POS, (u32Data & CARD_CODE_MSK) >> CARD_CODE_POS);
    } while (1);

}
