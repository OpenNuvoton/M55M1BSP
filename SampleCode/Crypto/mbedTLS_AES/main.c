/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/11/22 2:06p $
 * @brief    Show how mbedTLS AES function works.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "common.h"
#include "mbedtls/aes.h"


#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1
#define AES_BLOCK_SIZE         (16)

void SYS_Init(void);

volatile uint32_t g_u32Ticks = 0;


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


    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}


void SysTick_Handler()
{
    g_u32Ticks++;
}

#define TEST_AES
int32_t main(void)
{
    int  i32Ret = MBEDTLS_EXIT_SUCCESS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    SysTick_Config(SystemCoreClock / 1000);

    printf("MBEDTLS AES self test ...\n");
#if(NVT_DCACHE_ON==1)
    printf("  NVT_DCACHE_ON  \n");
#endif
#ifdef MBEDTLS_AES_ALT
    printf("Hardware Accelerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif


#ifdef TEST_AES
    g_u32Ticks = 0;
    i32Ret = mbedtls_aes_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_GCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_gcm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_CCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_ccm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

    if (i32Ret < 0)
    {
        printf("Test fail!\n");
    }

    printf("Test Done!\n");

    while (1);

}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/