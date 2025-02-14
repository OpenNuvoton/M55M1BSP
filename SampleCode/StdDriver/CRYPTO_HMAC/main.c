/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_HMAC code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


// DCache-line related//
#define HMAC_SRC_SIZE (1024)
//------------------------------------------------------------------------------
#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for improved performance when DCache is enabled
    uint8_t gau8HMACSrc[DCACHE_ALIGN_LINE_SIZE(HMAC_SRC_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Standard buffer alignment when DCache is disabled
    __ALIGNED(4) uint8_t gau8HMACSrc[HMAC_SRC_SIZE];
#endif
__ALIGNED(32) static uint8_t gau8HMAC[1024] = {0};       /* Buffer for HMAC golden pattern */

static volatile int g_HMAC_done;
void CRYPTO_IRQHandler(void);
int IsHexChar(char c);
uint8_t Char2Hex(uint8_t c);
uint32_t Str2Hex(char *str, uint8_t *hex, int swap);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

void CRYPTO_IRQHandler()
{
    if (CRYPTO->INTSTS & (CRYPTO_INTSTS_HMACIF_Msk | CRYPTO_INTSTS_HMACEIF_Msk))
    {
        g_HMAC_done = 1;
        CRYPTO->INTSTS = CRYPTO_INTSTS_HMACIF_Msk | CRYPTO_INTSTS_HMACEIF_Msk;
    }
}

int IsHexChar(char c)
{
    if ((c >= '0') && (c <= '9'))
        return 1;

    if ((c >= 'a') && (c <= 'f'))
        return 1;

    if ((c >= 'A') && (c <= 'F'))
        return 1;

    return 0;
}


uint8_t Char2Hex(uint8_t c)
{
    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    return 0;
}


uint32_t Str2Hex(char *str, uint8_t *hex, int swap)
{
    uint32_t i, count = 0, actual_len;
    uint8_t val8;

    while (*str)
    {
        if (!IsHexChar(*str))
        {
            return count;
        }

        val8 = Char2Hex(*str);
        str++;

        if (!IsHexChar(*str))
        {
            return count;
        }

        val8 = (uint8_t)(val8 << 4) | Char2Hex(*str);
        str++;

        hex[count] = val8;
        count++;
    }

    actual_len = count;

    for (; count % 4 ; count++)
        hex[count] = 0;

    if (!swap)
        return actual_len;

    // SWAP
    for (i = 0; i < count; i += 4)
    {
        val8 = hex[i];
        hex[i] = hex[i + 3];
        hex[i + 3] = val8;

        val8 = hex[i + 1];
        hex[i + 1] = hex[i + 2];
        hex[i + 2] = val8;
    }

    return actual_len;
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
int  main(void)
{
    int32_t i;
    uint8_t *pu8;
    uint32_t u32KeyLen, u32KeyLenAlign, u32MacLen, u32MsgLen;
#ifdef TEST_HMAC_SHA256
    char *keyStr = "6f35628d65813435534b5d67fbdb54cb33403d04e843103e6399f806cb5df95febbdd61236f33245";
    char *msg    =
        "752cff52e4b90768558e5369e75d97c69643509a5e5904e0a386cbe4d0970ef73f918f675945a9aefe26daea27587e8dc909dd56fd0468805f834039b345f855cfe19c44b55af241fff3ffcd8045cd5c288e6c4e284c3720570b58e4d47b8feeedc52fd1401f698a209fccfa3b4c0d9a797b046a2759f82a54c41ccd7b5f592b";
    char *hmac   = "05d1243e6465ed9620c9aec1c351a186"; // The golden pattern
    uint32_t u32OpMode = HMAC_MODE_SHA256;
#else
    char *keyStr =
        "726374c4b8df517510db9159b730f93431e0cd468d4f3821eab0edb93abd0fba46ab4f1ef35d54fec3d85fa89ef72ff3d35f22cf5ab69e205c10afcdf4aaf11338dbb12073474fddb556e60b8ee52f91163ba314303ee0c910e64e87fbf302214edbe3f2";
    char *msg    =
        "ac939659dc5f668c9969c0530422e3417a462c8b665e8db25a883a625f7aa59b89c5ad0ece5712ca17442d1798c6dea25d82c5db260cb59c75ae650be56569c1bd2d612cc57e71315917f116bbfa65a0aeb8af7840ee83d3e7101c52cf652d2773531b7a6bdd690b846a741816c860819270522a5b0cdfa1d736c501c583d916";
    char *hmac   = "bd3d2df6f9d284b421a43e5f9cb94bc4ff88a88243f1f0133bad0fb1791f6569"; // The golden pattern
    uint32_t u32OpMode = HMAC_MODE_SHA512;
#endif

    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+-----------------------------------------+\n");
    printf("|             Crypto HMAC Demo            |\n");
    printf("+-----------------------------------------+\n");

    /* Enable HMAC Interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);
    SHA_ENABLE_INT(CRYPTO);

    /* Init Feedback buffer content*/
    memset(gau8HMACSrc, 0, sizeof(gau8HMACSrc));

    /* Prepare the key and message for DMA. The format is key + msg */
    u32KeyLen = Str2Hex(keyStr, &gau8HMACSrc[0], 0);
    u32MsgLen = Str2Hex(msg,    &gau8HMACSrc[(u32KeyLen + 3) & 0xfffffffc], 0);

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(gau8HMACSrc, sizeof(gau8HMACSrc));
#endif
    u32MacLen = Str2Hex(hmac,   &gau8HMAC[0], 0);

    /* Key alignment for DMA */
    u32KeyLenAlign = ((u32KeyLen + 3) & 0xfffffffc);


    printf("Key (%3d bytes):\n%s\n", u32KeyLen, keyStr);
    printf("Msg (%3d bytes):\n%s\n", u32MsgLen, msg);


    SHA_Open(CRYPTO, u32OpMode, SHA_IN_OUT_SWAP, u32KeyLen);
    SHA_SetDMATransfer(CRYPTO, (uint32_t)&gau8HMACSrc[0], u32MsgLen + u32KeyLenAlign);

    g_HMAC_done = 0;
    SHA_Start(CRYPTO, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_HMAC_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for HMAC time-out!\n");
            return -1;
        }
    }

    printf("\nHMAC Output:\n");

    pu8 = (uint8_t *)&CRYPTO->HMAC_DGST[0];

    for (i = 0; i < 32; i++)
    {
        printf("%02x", pu8[i]);
    }

    printf("\n");


    if (memcmp((const void *)CRYPTO->HMAC_DGST, (const void *)gau8HMAC, u32MacLen) != 0)
    {
        printf("\n  !!Output is wrong!!\n");
    }

    printf("\nHMAC Demo Done\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
