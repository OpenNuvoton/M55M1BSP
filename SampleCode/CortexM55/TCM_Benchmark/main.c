/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Performance Comparison of CRC32 Implementation on I/D-TCM vs. Flash/SRAM
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define CRC32_BUF_SIZE   1024
#define CRC32_POLYNOMIAL 0xEDB88320U

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/

NVT_DTCM uint32_t g_au32DTCMBuf[CRC32_BUF_SIZE ] __ALIGNED(32);
uint32_t g_au32SRAMBuf[CRC32_BUF_SIZE] __ALIGNED(32);

__STATIC_INLINE void SysTickStart(void)
{
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

__STATIC_INLINE uint32_t GetSysTick(void)
{
    uint32_t ticks;
    ticks = 0xFFFFFF - SysTick->VAL;
    SysTick->CTRL = 0;
    return ticks;
}

__STATIC_INLINE uint32_t crc32(const char *s, size_t n)
{
    uint32_t crc = 0xFFFFFFFF;
    uint8_t *current = (unsigned char *) s;

    while (n--)
    {
        uint32_t j;
        crc ^= *current++;

        for (j = 0; j < 8; j++)
            if (crc & 1)
            {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            }
            else
            {
                crc =  crc >> 1;
            }
    }

    return ~crc;
}

uint32_t CRC32_OnFlash(const char *s, size_t n)
{
    return crc32(s, n);
}

NVT_ITCM uint32_t CRC32_OnITCM(const char *s, size_t n)
{
    return crc32(s, n);
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
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t    i;
    uint32_t    crc32_0, crc32_1;
    uint32_t    ticks_0, ticks_1;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+--------------------------------------+\n");
    printf("|    CRC32 on I/D-TCM vs Flash/SRAM    |\n");
    printf("+--------------------------------------+\n\n");

    for (i = 0; i < CRC32_BUF_SIZE; i++)
    {
        g_au32DTCMBuf[i] = i;
        g_au32SRAMBuf[i] = i;
    }

    /*
        CRC32_OnITCM is placed in ITCM region
        g_au32DTCMBuf is placed in DTCM region
    */
    SysTickStart();
    crc32_0 = CRC32_OnITCM((char *)g_au32DTCMBuf, CRC32_BUF_SIZE * 4);
    ticks_0 = GetSysTick();
    /*
        CRC32_OnFlash & g_au32SRAMBuf are place in default RO & RW region according to link script setting.
    */
    SysTickStart();
    crc32_1 = CRC32_OnFlash((char *)g_au32SRAMBuf, CRC32_BUF_SIZE * 4);
    ticks_1 = GetSysTick();
    printf("CRC32_OnITCM  @0x%08X , g_au32DTCMBuf @0x%08X , SysTicks = %d, CRC32 = 0x%08X\n"
           , (uint32_t)CRC32_OnITCM, (uint32_t)g_au32DTCMBuf
           , ticks_0, crc32_0
          );
    printf("CRC32_OnFlash @0x%08X , g_au32SRAMBuf @0x%08X , SysTicks = %d, CRC32 = 0x%08X\n"
           , (uint32_t)CRC32_OnFlash, (uint32_t)g_au32SRAMBuf
           , ticks_1, crc32_1
          );

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
