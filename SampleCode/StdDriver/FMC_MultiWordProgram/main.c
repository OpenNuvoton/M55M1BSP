/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Implement FMC multi word program function executed in ITCM
 *          to program embedded APROM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             (FMC_APROM_BASE + FMC_FLASH_PAGE_SIZE * 2)
#define APROM_TEST_END              (APROM_TEST_BASE + (FMC_FLASH_PAGE_SIZE * 8))

uint32_t    g_au32PageBuf[FMC_FLASH_PAGE_SIZE / 4];

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
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
    uint32_t u32Addr;
    int32_t  i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("| M55M1 FMC Multi Word Program Sample Code |\n");
    printf("+------------------------------------------+\n");
    SYS_UnlockReg();            /* Unlock protected registers */
    FMC_Open();                 /* Enable FMC ISP function */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM erase/program */

    for (u32Addr = APROM_TEST_BASE; u32Addr < APROM_TEST_END; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", u32Addr);
        printf("Erase ... \n");

        if (FMC_Erase(u32Addr) < 0)
        {
            printf("    Failed !\n");
            goto ErrorExit;
        }

        printf("Program ... \n");

        /* Prepare test pattern */
        for (i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
            g_au32PageBuf[i / 4] = u32Addr + i;

        i = FMC_WriteMultiple(u32Addr, g_au32PageBuf, FMC_FLASH_PAGE_SIZE);

        if ((i <= 0) || i != (FMC_FLASH_PAGE_SIZE))
        {
            printf("    [Failed] Programmed length = %d\n", i);
            goto ErrorExit;
        }

        printf("    [OK] Programmed length = %d\n", i);
        printf("Verify ... \n");

        for (i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
        {
            if (FMC_Read(u32Addr + i) != g_au32PageBuf[i / 4])
            {
                printf("    [Failed] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x !\n", u32Addr + i, g_au32PageBuf[i / 4], FMC_Read(u32Addr + i));
                goto ErrorExit;
            }

            if (g_FMC_i32ErrCode != 0)
            {
                printf("    Read address 0x%x failed !\n", u32Addr + i);
                goto ErrorExit;
            }
        }

        printf("    [OK]\n");
    }

ErrorExit:
    printf("\n\nMulti-word program demo done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
