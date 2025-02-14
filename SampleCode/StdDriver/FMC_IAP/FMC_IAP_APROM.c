/******************************************************************************
* @file    main.c
* @version V1.00
* @brief   IAP boot from APROM sample code
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

typedef void (*PFN_FUNC_PTR)(void);
extern uint32_t  LDROM_IMAGE_BASE, LDROM_IMAGE_LIMIT;   /* Symbol of LDROM image start and end */

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable ISP module clock */
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   u32i, u32j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize;

    printf("Program image to flash address 0x%x ... ", u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;

    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + u32i);

        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            FMC_Write(u32FlashAddr + u32i + u32j, pu32Loader[(u32i + u32j) / 4]);
        }
    }

    printf("OK.\n");

    printf("Verify ... ");

    /* Verify loader */
    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            u32Data = FMC_Read(u32FlashAddr + u32i + u32j);

            if (u32Data != pu32Loader[(u32i + u32j) / 4])
            {
                printf("Data mismatch on 0x%x ! [0x%x] != [0x%x]\n", u32FlashAddr + u32i + u32j, u32Data, pu32Loader[(u32i + u32j) / 4]);
                return -1;
            }

            if (u32i + u32j >= u32ImageSize)
                break;
        }
    }

    printf("OK.\n");
    return 0;
}

int main(void)
{
    uint8_t     u8Item;
    uint32_t    u32Data;
    char *ai8BootMode[] = { "APROM", "LDROM" };

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\n");
    printf("+-------------------------------------+\n");
    printf("|     M55M1 FMC IAP Sample Code       |\n");
    printf("|          [APROM code]               |\n");
    printf("+-------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    /* Get boot mode */
    printf("  Boot Mode ............... [%s]\n", ai8BootMode[FMC_GetBootSource()]);

    u32Data = FMC_ReadCID();
    printf("  Company ID .............. [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID .............. [0x%08x]\n", u32Data);

    /* Read User Configuration */
    printf("  User Config 0 ........... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ........... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    do
    {
        printf("\n\n\n");
        printf("+-------------------------------------+\n");
        printf("| Sample Code Menu                    |\n");
        printf("+-------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM          |\n");
        printf("| [1] Run IAP code in LDROM           |\n");
        printf("+-------------------------------------+\n");
        u8Item = getchar();
        printf("Select [%c]\n", u8Item);

        switch (u8Item)
        {
            case '0':
                FMC_ENABLE_LD_UPDATE();

                if (LoadImage((uint32_t)&LDROM_IMAGE_BASE, (uint32_t)&LDROM_IMAGE_LIMIT,
                              FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
                {
                    printf("Load image to LDROM failed !\n");
                    goto lexit;
                }

                FMC_DISABLE_LD_UPDATE();
                break;

            case '1':
                printf("\n\nChange VECMAP and branch to LDROM ...\n");
                UART_WAIT_TX_EMPTY(DEBUG_PORT); /* To make sure all message has been print out */

                /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
                __set_PRIMASK(1);

                /* Set VECMAP to LDROM for booting from LDROM */
                FMC_SetVectorPageAddr(FMC_LDROM_BASE);

                /* Software reset to boot to LDROM */
                NVIC_SystemReset();

                break;

            default :
                break;
        }
    } while (1);


lexit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nEnd of FMC IAP Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
