/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use FMC APROM Write Protect function
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

int main()
{
    uint32_t i, u32ISPReadData, u32CPUReadData, u32Addr, u32StartAddr, u32EndAddr, u32ISPFF;
    int32_t i32ISPSTS = 0;
    int i8GetCh;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------+\n");
    printf("|    M55M1 FMC APWPROT Sample Code   |\n");
    printf("+------------------------------------+\n");

    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    /* Enable FMC APROM update function */
    FMC_ENABLE_AP_UPDATE();

    /* Set APROM testing range */
    u32StartAddr = FMC_APROM_BASE + FMC_APWPROT_BLOCK_SIZE * 2;
    u32EndAddr   = FMC_APROM_END;

    /* Clear all APROM protect settings */
    for (i = 0; i < 64; i++)
    {
        FMC_DISABLE_APWPROT((i / 32), (1 << (i & 0x1F)));
    }

    /* Erase testing APROM range */
    printf("\nErase APROM ...\n");

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Erase page: 0x%08X \r", u32Addr);
        i32ISPSTS = FMC_Erase(u32Addr);

        if (i32ISPSTS)
        {
            printf("[Error] Erase failed at addr 0x%08X \n", u32Addr);
            goto lexit;
        }
    }

    /* Verify testing APROM range after erase done */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32ISPReadData = FMC_Read(u32Addr);
        u32ISPFF = FMC_GET_FAIL_FLAG();

        if (u32ISPFF)
        {
            printf("(%d) [Error] ISP fail flag: 0x%08X !\n",  __LINE__, u32ISPFF);
            goto lexit;
        }

        u32CPUReadData = M32(u32Addr);

        /* Verify testing APROM range, ISP read should be 0xFFFFFFFF */
        if (u32ISPReadData != 0xFFFFFFFF)
        {
            printf("(%d) [Error] ISP read 0x%08X is 0x%08X (should be 0xFFFFFFFF) !\n", __LINE__, u32Addr, u32ISPReadData);
            goto lexit;
        }

        /* Verify testing APROM range, CPU read should be 0xFFFFFFFF */
        if (u32CPUReadData != 0xFFFFFFFF)
        {
            printf("(%d) [Error] CPU read 0x%08X is 0x%08X (should be 0xFFFFFFFF) !\n",  __LINE__, u32Addr, u32CPUReadData);
            goto lexit;
        }
    }

    /* Sequential program on testing APROM block */
    printf("\nProgram APROM ...\n");

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (u32Addr % FMC_FLASH_PAGE_SIZE == 0)
            printf("Program page: 0x%08X \r", u32Addr);

        /* Program testing APROM range with its address */
        FMC_Write(u32Addr, u32Addr);
        u32ISPReadData = FMC_Read(u32Addr);

        u32ISPFF = FMC_GET_FAIL_FLAG();

        if (u32ISPFF)
        {
            printf("(%d) [Error] ISP fail flag: 0x%08X !\n",  __LINE__, u32ISPFF);
            goto lexit;
        }

        u32CPUReadData = M32(u32Addr);

        /* Verify testing APROM range, ISP read should be its address */
        if (u32ISPReadData != u32Addr)
        {
            printf("(%d) [Error] ISP read 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32ISPReadData, u32Addr);
            goto lexit;
        }

        /* Verify testing APROM range, CPU read should be its address */
        if (u32CPUReadData != u32Addr)
        {
            printf("(%d) [Error] CPU read 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32CPUReadData, u32Addr);
            goto lexit;
        }
    }

    printf("\nProgram done.\n");

    /* Set APROM Protect block */
    printf("\n\n");
    printf("+---------------------------+\n");
    printf("|  APROM Write Protect Test |\n");
    printf("+---------------------------+\n");

    printf("+---------------------------+\n");
    printf("|   Dump APWPROT Settings   |\n");
    printf("+---------------------------+\n");

    for (i = 0; i < 64; i++)
    {
        printf("APPROEN%02d [%s] address is 0x%08X ~ 0x%08X\n", i, FMC_IS_APWPROT((i / 32), 1 << (i & 0x1F)) ? "ENABLE " : "DISABLE",
               (uint32_t)(FMC_APROM_BASE + (i * FMC_APWPROT_BLOCK_SIZE)),
               (uint32_t)(FMC_APROM_BASE + ((i + 1)*FMC_APWPROT_BLOCK_SIZE - 1)));
    }

    printf("+------------------------------------------------------------------+\n");
    printf("| Select protected APROM Block                                     |\n");
    printf("+------------------------------------------------------------------+\n");

    while (1)
    {
        /* User select the protect range: block 0 ~ 63 */
        printf("\nPlease input number 0 ~ 63 and press enter:\n");
        scanf("%d", &i8GetCh);
        printf("\nSelect APPROEN%d\n\n", i8GetCh);

        /* Set the protect address by the selected block */
        u32StartAddr = FMC_APROM_BASE + (i8GetCh * FMC_APWPROT_BLOCK_SIZE);
        u32EndAddr   = FMC_APROM_BASE + ((i8GetCh + 1) * FMC_APWPROT_BLOCK_SIZE);

        if (i8GetCh < 2)
        {
            /* In this sample code, skip the block 0 and 1 testing because it's the code execution region */
            /* if user wants to test block 0 and 1, please let code execute in SRAM */
            printf("\tAPROM 0x%08X ~ 0x%08X is code execution region !\n\n", u32StartAddr, u32EndAddr);
        }
        else if (i8GetCh > 63)
        {
            /*Skip if the selected range exceeds APROM region*/
            printf("\tAddress 0x%08X ~ 0x%08X exceeds APROM range !\n\n", u32StartAddr, u32EndAddr);
        }
        else
        {
            /* Enable APROM protected function for selected region */
            printf("\tAPROM protect ENABLE from APROM 0x%08X ~ 0x%08X\n\n", u32StartAddr, u32EndAddr);
            FMC_ENABLE_APWPROT((i8GetCh / 32), (1 << (i8GetCh & 0x1F)));
        }

        /* User can select to continue to set other APROM protect region, or end the setting and start the protect function testing */
        printf("Press 'e' to exit APROM protect setting, others to continue setting other range.\n");

        if (getchar() == 'e')
            break;
    }

    printf("+---------------------------+\n");
    printf("|   Dump APWPROT Settings   |\n");
    printf("+---------------------------+\n");

    for (i = 0; i < 64; i++)
    {
        printf("APPROEN%02d [%s] address is 0x%08X ~ 0x%08X\n", i, FMC_IS_APWPROT((i / 32), 1 << (i & 0x1F)) ? " ENABLE" : "DISABLE",
               (uint32_t)(FMC_APROM_BASE + (i * FMC_APWPROT_BLOCK_SIZE)),
               (uint32_t)(FMC_APROM_BASE + ((i + 1) * FMC_APWPROT_BLOCK_SIZE - 1)));
    }

    for (i = 0; i < 64; i++)
    {
        if (FMC_IS_APWPROT((i / 32), 1 << (i & 0x1F)))
        {
            /* Set APROM protected address by selected region */
            u32StartAddr = FMC_APROM_BASE + (i * FMC_APWPROT_BLOCK_SIZE);
            u32EndAddr   = FMC_APROM_BASE + ((i + 1) * FMC_APWPROT_BLOCK_SIZE);

            printf("\n\nAPPROEN%02d Testing, address is 0x%08X ~ 0x%08X\n", i, u32StartAddr, u32EndAddr - 1);

            for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
            {
                u32ISPReadData = FMC_Read(u32Addr);
                u32ISPFF = FMC_GET_FAIL_FLAG();

                if (u32ISPFF)
                {
                    printf("(%d) [Error] ISP fail flag: 0x%08X !\n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(u32Addr);

                /* Verify protected APROM range, ISP read should be its address */
                if (u32ISPReadData != u32Addr)
                {
                    printf("(%d) [Error] ISP read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32ISPReadData, u32Addr);
                    goto lexit;
                }

                /* Verify protected APROM range, CPU read should be its address */
                if (u32CPUReadData != u32Addr)
                {
                    printf("(%d) [Error] CPU read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32CPUReadData, u32Addr);
                    goto lexit;
                }
            }

            /* After APROM protect enable, do erase testing */
            printf("\n\t   ENABLE APROM Protect, erase testing start (It should be erase FAIL.) ...\n");

            for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
            {
                i32ISPSTS = FMC_Erase(u32Addr);

                if (i32ISPSTS == 0)
                {
                    printf("(%d) [Error] ISP erase should be fail, i32ISPSTS: 0x%08X !\n", __LINE__, i32ISPSTS);
                    goto lexit;
                }
            }

            /* After APROM protect enable, verify APROM after erase testing */
            printf("\n\t   Verify after erase APROM when ENABLE APROM Protect (It should not be erased.) ...\n");

            for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
            {
                u32ISPReadData = FMC_Read(u32Addr);
                u32ISPFF = FMC_GET_FAIL_FLAG();

                if (u32ISPFF)
                {
                    printf("(%d) [Error] ISP fail flag: 0x%08X !\n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(u32Addr);

                /* Verify protected APROM range, ISP read should be its address */
                if (u32ISPReadData != u32Addr)
                {
                    printf("(%d) [Error] ISP read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32ISPReadData, u32Addr);
                    goto lexit;
                }

                /* Verify protected APROM range, CPU read should be its address */
                if (u32CPUReadData != u32Addr)
                {
                    printf("(%d) [Error] CPU read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32CPUReadData, u32Addr);
                    goto lexit;
                }
            }

            printf("\n\t=> Erase check OK. All data is not erased.\n");

            printf("\n\t   When ENABLE APROM Protect, program APROM test (It should not be programmed.) ...\n");

            for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
            {
                /* Program 0 to protected region */
                if (FMC_Write(u32Addr, 0) != 0 || FMC_GET_FAIL_FLAG())
                {
                    FMC_CLR_FAIL_FLAG();
                }
                else
                {
                    printf("(%d) [Error] ISP fail flag should be set !\n", __LINE__);
                    goto lexit;
                }

                u32ISPReadData = FMC_Read(u32Addr);
                u32ISPFF = FMC_GET_FAIL_FLAG();

                if (u32ISPFF)
                {
                    printf("(%d) [Error] ISP fail flag: 0x%08X !\n",  __LINE__, u32ISPFF);
                    goto lexit;
                }

                u32CPUReadData = M32(u32Addr);

                /* Verify protected APROM range, ISP read should be its address */
                if (u32ISPReadData != u32Addr)
                {
                    printf("(%d) [Error] ISP read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32ISPReadData, u32Addr);
                    goto lexit;
                }

                /* Verify protected APROM range, CPU read should be its address */
                if (u32CPUReadData != u32Addr)
                {
                    printf("(%d) [Error] CPU read protected 0x%08X is 0x%08X (should be 0x%08X) !\n", __LINE__, u32Addr, u32CPUReadData, u32Addr);
                    goto lexit;
                }

            }

            printf("\n\t=> Program check OK. All data are not programmed.\n");
        }
    }

lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nEnd of FMC APWPROT Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
