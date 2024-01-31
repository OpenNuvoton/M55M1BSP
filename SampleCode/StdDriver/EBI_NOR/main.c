/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Configure EBI interface to access NOR Flash connected on EBI interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

void Configure_EBI_16BIT_Pins(void);
void SYS_Init(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* AD0 ~ AD15*/
    SET_EBI_AD0_PC0();
    SET_EBI_AD1_PC1();
    SET_EBI_AD2_PC2();
    SET_EBI_AD3_PC3();
    SET_EBI_AD4_PC4();
    SET_EBI_AD5_PC5();
    SET_EBI_AD6_PD8();
    SET_EBI_AD7_PD9();
    SET_EBI_AD8_PE14();
    SET_EBI_AD9_PE15();
    SET_EBI_AD10_PE1();
    SET_EBI_AD11_PE0();
    SET_EBI_AD12_PH8();
    SET_EBI_AD13_PH9();
    SET_EBI_AD14_PH10();
    SET_EBI_AD15_PH11();

    /* ADDR16 ~ ADDR19; */
    SET_EBI_ADR16_PF9();
    SET_EBI_ADR17_PF8();
    SET_EBI_ADR18_PF7();
    SET_EBI_ADR19_PF6();

    /* EBI nWR and nRD pins on PA.10 and PA.11 */
    SET_EBI_nWR_PA10();
    SET_EBI_nRD_PA11();

    /* EBI nWRH and nWRL pins on PB.6 and PB.7 */
    SET_EBI_nWRH_PB6();
    SET_EBI_nWRL_PB7();

    /* EBI nCS1 pin on PD.11 */
    SET_EBI_nCS1_PD11();

    /* EBI ALE pin on PA.8 */
    SET_EBI_ALE_PA8();
}

void SYS_Init(void)
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

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable EBI clock */
    CLK_EnableModuleClock(EBI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Initialize MPU Region */
    InitPreDefMPURegion(NULL, 0);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*   - AD0 ~ AD5 on PC.0~5                                              *\n");
    printf("*   - AD6 ~ AD7 on PD.8 and PD.9                                       *\n");
    printf("*   - AD8 ~ AD9 on PE.14 and PE.15                                     *\n");
    printf("*   - AD10 ~ AD11 on PE.1 and PE.0                                     *\n");
    printf("*   - AD12 ~ AD15 on PH.8~11                                           *\n");
    printf("*   - ADR16 ~ ADR19 on PF.9~6                                          *\n");
    printf("*   - nWR on PA.10                                                     *\n");
    printf("*   - nRD on PA.11                                                     *\n");
    printf("*   - nWRL on PB.7                                                     *\n");
    printf("*   - nWRH on PB.6                                                     *\n");
    printf("*   - nCS1 on PD.11                                                    *\n");
    printf("*   - ALE on PA.8                                                      *\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOW, 0, EBI_CS_ACTIVE_LOW);

    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);

    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);

    if ((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        goto lexit;
    }
    else
    {
        printf("PASS !!!\n\n");
    }

    /* Step 2, erase chip */
    if (NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
    {
        printf("FAIL !!!\n\n");
        goto lexit;
    }

    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;

        if (NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData);
        }
    }

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + (u32Addr / 2)) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);

        if (u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1)) + u32Addr, u16WData, u16RData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16RData);
        }
    }

    printf(">> Program flash OK !!!                             \n\n");

lexit:

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI0_MODULE);

    while (1) {}
}
