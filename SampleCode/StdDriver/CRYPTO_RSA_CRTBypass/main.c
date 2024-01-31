/**************************************************************************//**
* @file    main.c
* @version V1.00
* @brief   CRYPTO_RSA_CRTBypass code for M55M1 series MCU
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define RSA_BIT_LEN      2048

__ALIGNED(4) static char    N[RSA_KBUF_HLEN] =
    "bad47a84c1782e4dbdd913f2a261fc8b65838412c6e45a2068ed6d7f16e9cdf4462b39119563cafb74b9cbf25cfd544bdae23bff0ebe7f6441042b7e109b9a8afaa056821ef8efaab219d21d6763484785622d918d395a2a31f2ece8385a8131e5ff143314a82e21afd713bae817cc0ee3514d4839007ccb55d68409c97a18ab62fa6f9f89b3f94a2777c47d6136775a56a9a0127f682470bef831fbec4bcd7b5095a7823fd70745d37d1bf72b63c4b1b4a3d0581e74bf9ade93cc46148617553931a79d92e9e488ef47223ee6f6c061884b13c9065b591139de13c1ea2927491ed00fb793cd68f463f5f64baa53916b46c818ab99706557a1c2d50d232577d1";
__ALIGNED(4) static char    d[RSA_KBUF_HLEN] =
    "40d60f24b61d76783d3bb1dc00b55f96a2a686f59b3750fdb15c40251c370c65cada222673811bc6b305ed7c90ffcb3abdddc8336612ff13b42a75cb7c88fb936291b523d80acce5a0842c724ed85a1393faf3d470bda8083fa84dc5f31499844f0c7c1e93fb1f734a5a29fb31a35c8a0822455f1c850a49e8629714ec6a2657efe75ec1ca6e62f9a3756c9b20b4855bdc9a3ab58c43d8af85b837a7fd15aa1149c119cfe960c05a9d4cea69c9fb6a897145674882bf57241d77c054dc4c94e8349d376296137eb421686159cb878d15d171eda8692834afc871988f203fc822c5dcee7f6c48df663ea3dc755e7dc06aebd41d05f1ca2891e2679783244d068f";
__ALIGNED(4) static char    P[RSA_KBUF_HLEN] =
    "e7c9e4b3e5d7ac9e83be08328105356dfeefe222f26c95378effd2150fadf7ba23f5b4705d82e4f1bc45057067c7def73e2100f756ee6d547965fa4f24b85d68867f03d7c886d1dbcca4c589745701b362a1f1417f471d8475b6b7a16a4c48ef1f556edc3f0ff6ba13d365d6e82751f207d91101c8eea1013ccdd9e1de4c387f";
__ALIGNED(4) static char    Q[RSA_KBUF_HLEN] =
    "ce58602e051f0f4647c4ec57f682e5737fc482a8a1ffac9043bba4fba3387d7dd2154507af1e28bd81b61fcdfe35f9734e0d9b53682ec785f1f6e6224f63d10bf78484b83a4254f333d0fb3f3e9e1834bede52e3078ac279a862fb90af266d7591c81f20b718d07d51bfc221b66a25403b4ac1a68d673fdd959b01ecf3d0a7af";
__ALIGNED(4) static char    Msg[RSA_KBUF_HLEN] =
    "70992c9d95a4908d2a94b3ab9fa1cd643f120e326f9d7808af50cac42c4b0b4eeb7f0d4df303a568fbfb82b0f58300d25357645721bb71861caf81b27a56082c80a146499fb4eab5bde4493f5d00f1a437bbc360dfcd8056fe6be10e608adb30b6c2f7652428b8d32d362945982a46585d2102ef7995a8ba6e8ad8fd16bd7ae8f53c3d7fcfba290b57ce7f8f09c828d6f2d3ce56f131bd9461e5667e5b73edac77f504dac4f202a9570eb4515b2bf516407db831518db8a2083ec701e8fd387c430bb1a72deca5b49d429cf9deb09cc4518dc5f57c089aa2d3420e567e732102c2c92b88a07c69d70917140ab3823c63f312d3f11fa87ba29da3c7224b4fb4bc";
__ALIGNED(4) static char    Sign[RSA_KBUF_HLEN] =
    "7e65b998a05f626b028c75dc3fbf98963dce66d0f4c3ae4237cff304d84d8836cb6bad9ac86f9d1b8a28dd70404788b869d2429f1ec0663e51b753f7451c6b4645d99126e457c1dac49551d86a8a974a3131e9b371d5c214cc9ff240c299bd0e62dbc7a9a2dad9fa5404adb00632d36332d5be6106e9e6ec81cac45cd339cc87abbe7f89430800e16e032a66210b25e926eda243d9f09955496ddbc77ef74f17fee41c4435e78b46965b713d72ce8a31af641538add387fedfd88bb22a42eb3bda40f72ecad941dbffdd47b3e77737da741553a45b630d070bcc5205804bf80ee2d51612875dbc4796960052f1687e0074007e6a33ab8b2085c033f9892b6f74";
__ALIGNED(4) static char    Msg2[RSA_KBUF_HLEN] =
    "2b7ce910993b660fccfb6bf22ec4e4251fa39470bea2e0f747209b7e844a84b81bf09f89917d2a73fcb4e9a5a429f5891234d3f1374acf20c584d44336a6dee61891d5ead50035d3df8df03f319d57a77eff90459def4dbdc7b04f2bb94996c0c1c87990d43df3fa6e7480a9588c863670936e0152cd2461bfc10f7fb203299bbd8d91ec79f3ad56304b89af70a784925e07bbf0ffe5629beb0438bc6ca65629f8d6c0b671ab4a0e9345e843322c591dddbbe8ee8f7d72a47fabe53d40286dcfaefc05d8954e43573f17421afb5c849e5fbaff312d64bb81794bc9fce3a809dcfef87696b7993d3c39a66d27f01c6cd32e043c7c2dd07487c0c4001b49bb2555";
__ALIGNED(4) static char    Sign2[RSA_KBUF_HLEN] =
    "11111111a05f626b028c75dc3fbf98963dce66d0f4c3ae4237cff304d84d8836cb6bad9ac86f9d1b8a28dd70404788b869d2429f1ec0663e51b753f7451c6b4645d99126e457c1dac49551d86a8a974a3131e9b371d5c214cc9ff240c299bd0e62dbc7a9a2dad9fa5404adb00632d36332d5be6106e9e6ec81cac45cd339cc87abbe7f89430800e16e032a66210b25e926eda243d9f09955496ddbc77ef74f17fee41c4435e78b46965b713d72ce8a31af641538add387fedfd88bb22a42eb3bda40f72ecad941dbffdd47b3e77737da741553a45b630d070bcc5205804bf80ee2d51612875dbc4796960052f1687e0074007e6a33ab8b2085c033f922222222";

static RSA_BUF_CRT_T s_sRSABuf;
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


    /* Enable PLL0 200MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

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
    printf("|    Crypto RSA CRT/CRT bypass mode sample    |\n");
    printf("+---------------------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);

    RSA_ENABLE_INT(CRYPTO);

    g_RSA_done = 0;
    g_RSA_error = 0;

    printf("Private key (N,d) -\n");
    printf("    N = %s\n", N);
    printf("    d = %s\n", d);

    /*---------------------------------------
     *  RSA first sign with CRT mode
     *---------------------------------------*/
    /* Configure RSA operation mode and key length */
    if (RSA_Open(CRYPTO, RSA_MODE_CRT, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 0) != 0)
    {
        printf("\nRSA buffer size is incorrect!!\n");
        return -1;
    }

    /* Set RSA private key */
    RSA_SetKey(CRYPTO, d);
    RSA_SetDMATransfer(CRYPTO, Msg, N, P, Q);
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
    RSA_Read(CRYPTO, OutputResult);
    printf("\nRSA sign 1: %s\n", OutputResult);

    /* Verify the signature */
    if (strcasecmp(OutputResult, Sign) == 0)
        printf("\nRSA signature 1 verify OK.\n\n");
    else
    {
        printf("\nRSA signature 1 verify failed!!\n\n");
        return -1;
    }

    /*--------------------------------------------------------------------------------
     *  RSA second sign with CRT Bypass mode to speed up performance(use the same key)
     *--------------------------------------------------------------------------------*/
    g_RSA_done = 0;
    g_RSA_error = 0;

    /* Configure RSA operation mode and key length */
    if (RSA_Open(CRYPTO, RSA_MODE_CRTBYPASS, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 0) != 0)
    {
        printf("\nRSA buffer size is incorrect!!\n");
        return -1;
    }

    /* Use the same key, and change Msg */
    RSA_SetDMATransfer(CRYPTO, Msg2, N, P, Q);
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
    RSA_Read(CRYPTO, OutputResult);
    printf("\nRSA sign 2: %s\n", OutputResult);

    /* Verify the signature */
    if (strcasecmp(OutputResult, Sign2) == 0)
        printf("\nRSA signature 2 verify OK.\n\n");
    else
    {
        printf("\nRSA signature 2 verify failed!!\n\n");
        return -1;
    }

    printf("\nDone.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
