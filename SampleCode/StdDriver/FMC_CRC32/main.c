/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use FMC CRC32 ISP command to calculate
 *          CRC32 checksum of APROM, LDROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t    u32Data, u32ChkSum;    /* Temporary data */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------+\n");
    printf("|    M55M1 FMC CRC32 Sample Code     |\n");
    printf("+------------------------------------+\n");

    SYS_UnlockReg();                /* Unlock protected registers */
    FMC_Open();                     /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();        /* Read company ID. Should be 0x530000DA. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    printf("\nLDROM (0xF100000 ~ 0xF102000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_LDROM_SIZE);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }

    printf("0x%x\n", u32ChkSum);       /* Print out LDROM CRC32 check sum value */


    printf("\nAPROM bank0 (0x%08X ~ 0x%08X) CRC32 checksum =>  ", (uint32_t)FMC_APROM_BASE, (uint32_t)(FMC_APROM_BASE + FMC_BANK_SIZE));

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM bank 0.
     *  Note that FMC CRC32 checksum calculation area must not cross bank boundary.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, FMC_BANK_SIZE);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }

    printf("0x%x\n", u32ChkSum);       /* Print out APROM CRC32 check sum value */

    /* Request FMC hardware to run CRC32 calculation on APROM bank 1. */
    printf("\nAPROM bank1 (0x%08X ~ 0x%08X) CRC32 checksum =>  ", (uint32_t)FMC_APROM_BANK1_BASE, (uint32_t)(FMC_APROM_BANK1_BASE + FMC_BANK_SIZE));
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BANK1_BASE, FMC_BANK_SIZE);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank1 CRC32 checksum!\n");
        goto lexit;
    }

    printf("0x%x\n", u32ChkSum);       /* Print out APROM CRC32 check sum value */

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    printf("\nEnd of FMC CRC32 Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
