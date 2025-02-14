/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_RSA code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define RSA_BIT_LEN      2048
// Standard buffer alignment when DCache is disabled
__ALIGNED(4) static char    N[RSA_KBUF_HLEN] =
    "bad47a84c1782e4dbdd913f2a261fc8b65838412c6e45a2068ed6d7f16e9cdf4462b39119563cafb74b9cbf25cfd544bdae23bff0ebe7f6441042b7e109b9a8afaa056821ef8efaab219d21d6763484785622d918d395a2a31f2ece8385a8131e5ff143314a82e21afd713bae817cc0ee3514d4839007ccb55d68409c97a18ab62fa6f9f89b3f94a2777c47d6136775a56a9a0127f682470bef831fbec4bcd7b5095a7823fd70745d37d1bf72b63c4b1b4a3d0581e74bf9ade93cc46148617553931a79d92e9e488ef47223ee6f6c061884b13c9065b591139de13c1ea2927491ed00fb793cd68f463f5f64baa53916b46c818ab99706557a1c2d50d232577d1";
__ALIGNED(4) static char    E[RSA_KBUF_HLEN] =
    "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010001";
__ALIGNED(4) static char    d[RSA_KBUF_HLEN] =
    "40d60f24b61d76783d3bb1dc00b55f96a2a686f59b3750fdb15c40251c370c65cada222673811bc6b305ed7c90ffcb3abdddc8336612ff13b42a75cb7c88fb936291b523d80acce5a0842c724ed85a1393faf3d470bda8083fa84dc5f31499844f0c7c1e93fb1f734a5a29fb31a35c8a0822455f1c850a49e8629714ec6a2657efe75ec1ca6e62f9a3756c9b20b4855bdc9a3ab58c43d8af85b837a7fd15aa1149c119cfe960c05a9d4cea69c9fb6a897145674882bf57241d77c054dc4c94e8349d376296137eb421686159cb878d15d171eda8692834afc871988f203fc822c5dcee7f6c48df663ea3dc755e7dc06aebd41d05f1ca2891e2679783244d068f";
__ALIGNED(4) static char    Msg[RSA_KBUF_HLEN] =
    "70992c9d95a4908d2a94b3ab9fa1cd643f120e326f9d7808af50cac42c4b0b4eeb7f0d4df303a568fbfb82b0f58300d25357645721bb71861caf81b27a56082c80a146499fb4eab5bde4493f5d00f1a437bbc360dfcd8056fe6be10e608adb30b6c2f7652428b8d32d362945982a46585d2102ef7995a8ba6e8ad8fd16bd7ae8f53c3d7fcfba290b57ce7f8f09c828d6f2d3ce56f131bd9461e5667e5b73edac77f504dac4f202a9570eb4515b2bf516407db831518db8a2083ec701e8fd387c430bb1a72deca5b49d429cf9deb09cc4518dc5f57c089aa2d3420e567e732102c2c92b88a07c69d70917140ab3823c63f312d3f11fa87ba29da3c7224b4fb4bc";

#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for improved performance when DCache is enabled
    // Note sizeof(s_sRSABuf) is multiply of 128(also, multiply of DCACHE_LINE_SIZE(32))
    RSA_BUF_NORMAL_T s_sRSABuf __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Standard buffer alignment when DCache is disabled
    __ALIGNED(4) static RSA_BUF_NORMAL_T s_sRSABuf;
#endif

static volatile int g_RSA_done;
static volatile int g_RSA_error;

void CRYPTO_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRYPTO_IRQHandler(void)
{
    if (PRNG_GET_INT_FLAG(CRYPTO))
    {
        PRNG_CLR_INT_FLAG(CRYPTO);
    }

    if (SHA_GET_INT_FLAG(CRYPTO))
    {
        SHA_CLR_INT_FLAG(CRYPTO);
    }

    if (RSA_GET_INT_FLAG(CRYPTO))
    {
        g_RSA_done = 1;

        if (RSA_GET_INT_FLAG(CRYPTO)&CRYPTO_INTSTS_RSAEIF_Msk)
        {
            g_RSA_error = 1;
            printf("RSA error flag is set!!\n");
        }

        RSA_CLR_INT_FLAG(CRYPTO);
    }
}

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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    char    OutputResult[RSA_KBUF_HLEN];
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n+---------------------------------------------+\n");
    printf("|   Crypto RSA sample                         |\n");
    printf("+---------------------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);

    RSA_ENABLE_INT(CRYPTO);

    g_RSA_done = 0;
    g_RSA_error = 0;

    printf("Private key (N,d) -\n");
    printf("    N = %s\n", N);
    printf("    d = %s\n", d);
    printf("Public key (N,e) -\n");
    printf("    E = %s\n", E);

    /*---------------------------------------
     *  RSA sign
     *---------------------------------------*/
    /* Configure RSA operation mode and key length */
    if (RSA_Open(CRYPTO, RSA_MODE_NORMAL, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 0) != 0)
    {
        printf("\nRSA buffer size is incorrect!!\n");
        return -1;
    }

    /* Set RSA private key */
    //Update au32RsaM, au32RsaN
    RSA_SetKey(CRYPTO, d);
    RSA_SetDMATransfer(CRYPTO, Msg, N, 0, 0);

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(&s_sRSABuf, sizeof(s_sRSABuf));//For sRSABuf.au32RsaM, au32RsaN
#endif
    RSA_Start(CRYPTO);

    /* Waiting for RSA operation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_RSA_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for RSA operation done time-out!\n");
            return -1;
        }
    }

    /* Check error flag */
    if (g_RSA_error)
    {
        printf("\nRSA has error!!\n");
        return -1;
    }

#if (NVT_DCACHE_ON == 1)
    SCB_InvalidateDCache_by_Addr(&s_sRSABuf, sizeof(s_sRSABuf));//For sRSABuf.au32RsaOutput
#endif
    /* Get RSA output result*/
    //Read OutputResult(stack) from sRSABuf.au32RsaOutput
    RSA_Read(CRYPTO, OutputResult);

    printf("\nRSA sign: %s\n", OutputResult);

    /*---------------------------------------
     *  RSA verify
     *---------------------------------------*/
    g_RSA_done = 0;
    g_RSA_error = 0;

    /* Configure RSA operation mode and key length */
    if (RSA_Open(CRYPTO, RSA_MODE_NORMAL, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 0))
    {
        printf("\nRSA buffer size is incorrect!!\n");
        return -1;
    }

    /* Set RSA public key */
    RSA_SetKey(CRYPTO, E);
    RSA_SetDMATransfer(CRYPTO, OutputResult, N, 0, 0);
#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(&s_sRSABuf.au32RsaOutput, sizeof(s_sRSABuf));//For sRSABuf.au32RsaM, au32RsaN
#endif
    RSA_Start(CRYPTO);

    /* Waiting for RSA operation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_RSA_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for RSA operation done time-out!\n");
            return -1;
        }
    }

    /* Check error flag */
    if (g_RSA_error)
    {
        printf("\nRSA has error!!\n");
        return -1;
    }

    /* Get RSA output result */
#if (NVT_DCACHE_ON == 1)
    SCB_InvalidateDCache_by_Addr(&s_sRSABuf.au32RsaOutput, sizeof(s_sRSABuf.au32RsaOutput));//For sRSABuf.au32Output
    //Read OutputResult(stack) from sRSABuf.au32RsaOutput
#endif
    RSA_Read(CRYPTO, OutputResult);
    printf("\nRSA Output: %s\n", OutputResult);

    /* Verify the message */
    if (strcasecmp(OutputResult, Msg) == 0)
        printf("\nRSA signature verify OK.\n");
    else
    {
        printf("\nRSA signature verify failed!!\n");
        //return -1;
    }

    printf("\nDone.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
