/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/11/22 2:06p $
 * @brief    Show how mbedTLS RSA function works.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "mbedtls/rsa.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "NuMicro.h"

#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1

int mbedtls_rsa_self_test(int verbose);

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

    /* Debug debug uart clock setting*/
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

    printf("MBEDTLS RSA self test ...\n");

#ifdef MBEDTLS_RSA_ALT
    printf("Hardware Accelerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif

    g_u32Ticks = 0;
    i32Ret = mbedtls_rsa_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);

    if (i32Ret < 0)
    {
        printf("Test fail!\n");
    }

    printf("Test Done!\n");

    while (1);

}


void show(void)
{
    int i, j;
    int n = 128;
    uint8_t *pu8;

    printf("\n");

    for (i = 0; i < 3; i++)
    {
        printf("SADDR[%d]", i);
        pu8 = (uint8_t *)CRYPTO->RSA_SADDR[i];

        for (j = 0; j < n; j++)
        {
            if ((j & 0xf) == 0)
                printf("\n");

            printf("%02x ", pu8[j]);
        }

        printf("\n");
    }

    printf("DADDR");
    pu8 = (uint8_t *)CRYPTO->RSA_DADDR;

    for (j = 0; j < n; j++)
    {
        if ((j & 0xf) == 0)
            printf("\n");

        printf("%02x ", pu8[j]);
    }

    printf("\n");

}

void dump(uint8_t *p, uint32_t size)
{
    int i;

    for (i = 0; i < size; i++)
    {
        if ((i & 0xf) == 0)
            printf("\n");

        printf("%02x ", p[i]);
    }

    printf("\n");


}

