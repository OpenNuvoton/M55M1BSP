/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Change system clock to different PLL frequency and output system clock from CLKO pin.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SIGNATURE   0x125ab234
#define FLAG_ADDR   0x20001FFC

int32_t pi(void);
void Delay(uint32_t x);
void SYS_PLL_Test(void);
void SYS_Init(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t g_ai32f[PI_NUM + 1];
static uint32_t g_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t g_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for (; b - c;)
        g_ai32f[b++] = a / 5;

    i = 0;

    for (; (void)(d = 0), g = c * 2; c -= 14, g_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if (i == 19)
            break;

        for (b = c; (void)(d += g_ai32f[b] * a), (void)(g_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }

    i32Err = 0;

    for (i = 0; i < 19; i++)
    {
        if (g_au32piTbl[i] != (uint32_t)g_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void Delay(uint32_t x)
{
    uint32_t i;

    for (i = 0; i < x; i++)
    {
        __NOP();
        __NOP();
    }
}

static uint32_t g_au32PllSetting[] =
{
    FREQ_50MHZ,    /* PLL = 50MHz */
    FREQ_100MHZ,   /* PLL = 100MHz */
    FREQ_150MHZ,   /* PLL = 150MHz */
    FREQ_180MHZ,   /* PLL = 180MHz */
};

void SYS_PLL_Test(void)
{
    uint32_t  u32Idx;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                            */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for (u32Idx = 0; u32Idx < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; u32Idx++)
    {
        /* Select SCLK clock source from PLL */
        CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, g_au32PllSetting[u32Idx]);

        printf("Change system clock to %d Hz ...", SystemCoreClock);

        /* Output selected clock to CKO, CKO Clock = SCLK / 2^(2 + 1) */
        CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 3, CLK_CLKOCTL_DIV1EN_DIV_FREQSEL);

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x40000);

        if (pi())
        {
            printf("[FAIL]\n");
        }
        else
        {
            printf("[OK]\n");
        }

        printf("Press any key to next frequency\n");

        getchar();

        /* Disable CLKO clock */
        CLK_DisableCKO();
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Enable PLL0 180MHz clock and set all bus clock */
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

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32data, u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|     PLL Clock Output Sample Code      |\n");
    printf("+---------------------------------------+\n");

    if (M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n\n");
        getchar();
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS_ClearResetSrc(u32data);

    /* Unlock protected registers for SCLK clock source settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before HCLK clock source setting and CPU Reset */
    if (SYS_IsRegLocked() == 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Run PLL Test */
    SYS_PLL_Test();

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Wait for message send out */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Select SCLK clock source as HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLK_STATUS register */
    CLK_DisableAPLL(CLK_APLL0_SELECT);

    /* Reset CPU */
    SYS_ResetCPU();
}
