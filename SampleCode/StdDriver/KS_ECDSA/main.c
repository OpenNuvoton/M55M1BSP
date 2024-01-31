/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo to use the ECC ECDSA with Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define USE_KS_KEY      1

static volatile int g_SHA_done = 0;

/* Alignment for DMA */
__ALIGNED(4) static const char sc_strMsg[] = "This is a message. It could be encypted.";

void Hex2Reg(char input[], uint32_t volatile reg[]);
void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);
void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize);
void SYS_Init(void);
void UART_Init(void);

NVT_ITCM void CRYPTO_IRQHandler(void)
{
    if (SHA_GET_INT_FLAG(CRYPTO))
    {
        g_SHA_done = 1;
        SHA_CLR_INT_FLAG(CRYPTO);
    }

    ECC_Complete(CRYPTO);
}

void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize)
{
    int nIdx, i, j;

    nIdx = 0;

    while (u32BufByteSize > 0)
    {
        j = u32BufByteSize;

        if (j > 16)
        {
            j = 16;
        }

        printf("0x%04X  ", nIdx);

        for (i = 0; i < j; i++)
            printf("%02x ", pu8Buf[nIdx + i]);

        for (; i < 16; i++)
            printf("   ");

        printf("  ");

        for (i = 0; i < j; i++)
        {
            if ((pu8Buf[nIdx + i] >= 0x20) && (pu8Buf[nIdx + i] < 127))
                printf("%c", pu8Buf[nIdx + i]);
            else
                printf(".");

            u32BufByteSize--;
        }


        nIdx += j;
        printf("\n");
    }

    printf("\n");
}

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

    /* Enable module clock */
    CLK_EnableModuleClock(KS0_MODULE);
    CLK_EnableModuleClock(CRYPTO0_MODULE);

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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i;
    uint8_t *pu8;
    int32_t err;
    int32_t i32KeyIdx_d, i32KeyIdx_k, i32KeyIdx_Qx, i32KeyIdx_Qy;
    uint32_t time;
    uint32_t au32ECC_N[18] = {0};
    char d[]  = "5D98653E0F30167D90C33F401C0ABCA9C37D481DB0E0FB362660CA2417DFA9D7";
    char Qx[] = "0000000000000000000000000000000000000000000000000000000000000000";
    char Qy[] = "0000000000000000000000000000000000000000000000000000000000000000";
    char R[]  = "0000000000000000000000000000000000000000000000000000000000000000";
    char S[]  = "0000000000000000000000000000000000000000000000000000000000000000";
    char hash[] = "0000000000000000000000000000000000000000000000000000000000000000";
    uint32_t au32Key[8] = {0};
    uint32_t u32TimeOutCnt;

#ifndef USE_KS_KEY
    uint32_t u32Seed;
    char k[128];
#endif

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("CPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|        M55M1 KS ECDSA Sample Code        |\n");
    printf("+------------------------------------------+\n");

    /* Init Key Store */
    KS_Open();

    /* Enable CRYPTO interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);
    ECC_ENABLE_INT(CRYPTO);
    SHA_ENABLE_INT(CRYPTO);

    /* Reset SysTick to measure time */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;


    /*------------------------------------------------------------------------------*/
    /* Write private key to Key Store */
    Hex2Reg(d, au32Key);

    printf("  d  = %s\n", d);

    i32KeyIdx_d = KS_Write(KS_SRAM, KS_META_ECC | KS_META_256, au32Key);

    if (i32KeyIdx_d < 0)
    {
        printf("Fail to write key to Key Store !\n");
        KS_EraseAll(KS_SRAM);
        goto lexit;
    }

    printf("i32KeyIdx_d = %d, remain size = %d\n", i32KeyIdx_d, KS_GetRemainSize(KS_SRAM));

    if (!ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_256, d))
    {
        /* Invalid key */
        printf("Current private key is not valid. You should set a new one.\n");
        goto lexit;
    }

    /*------------------------------------------------------------------------------*/
    /* Calculate Public Key */

    SysTick->VAL = 0;
#ifndef USE_KS_KEY

    if (ECC_GeneratePublicKey(CRYPTO, CURVE_P_256, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed !\n");
        goto lexit;
    }

#else

    /* Use the private key in Key Store to generate the publick key */
    if (ECC_GeneratePublicKey_KS(CRYPTO, CURVE_P_256, i32KeyIdx_d, FALSE, Qx, Qy) < 0)
    {
        printf("ECC key generation failed !\n");
        goto lexit;
    }

#endif
    time = 0xffffff - SysTick->VAL;

    printf("  Qx = %s\n", Qx);
    printf("  Qy = %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);


    /*------------------------------------------------------------------------------*/
    /* Write random k to Key Store */

#ifndef USE_KS_KEY
    /* Init TRNG */
    RNG_Open();
    /* Get PRNG Seed */
    RNG_Random(&u32Seed, 1);

    /* Set PRNG Seed */
    PRNG_Open(CRYPTO, PRNG_KEY_SIZE_256, PRNG_SEED_RELOAD, u32Seed);
    PRNG_Start(CRYPTO);
    PRNG_Open(CRYPTO, PRNG_KEY_SIZE_256, PRNG_SEED_CONT, u32Seed);

    /* Generate k by TRNG */
    PRNG_Start(CRYPTO);
    CRYPTO_Reg2Hex(64, (uint32_t *)CRYPTO->PRNG_KEY, k);

    printf("  k  = %s\n", k);

    if (!ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_256, k))
    {
        /* Invalid key */
        printf("Current private key is not valid. You should set a new one.\n");
        goto lexit;
    }

#else

    /* NOTE: k must be generated by RNG when using ECC_GenerateSignature_KS */
    /* N value of CURVE_P_256 */
    au32ECC_N[7] = 0xFFFFFFFFul;
    au32ECC_N[6] = 0x00000001ul;
    au32ECC_N[5] = 0x00000000ul;
    au32ECC_N[4] = 0x00000000ul;
    au32ECC_N[3] = 0x00000000ul;
    au32ECC_N[2] = 0xFFFFFFFFul;
    au32ECC_N[1] = 0xFFFFFFFFul;
    au32ECC_N[0] = 0xFFFFFFFFul;

    err = RNG_ECDSA_Init(PRNG_KEY_SIZE_256, au32ECC_N);

    if (err)
    {
        printf("PRNG ECDSA Inital failed !\n");
        goto lexit;
    }

    /* Generate a key to key store */
    i32KeyIdx_k = RNG_ECDSA(PRNG_KEY_SIZE_256);

    if (i32KeyIdx_k < 0)
    {
        printf("Fail to write k to KS SRAM !\n");
        goto lexit;
    }

    printf("i32KeyIdx_k = %d, remain size = %d\n", i32KeyIdx_k, KS_GetRemainSize(KS_SRAM));
#endif

    /*------------------------------------------------------------------------------*/
    /* Calculate the hash of the message */
    SHA_Open(CRYPTO, SHA_MODE_SHA256, SHA_IN_OUT_SWAP, 0);
    SHA_SetDMATransfer(CRYPTO, (uint32_t)&sc_strMsg[0], sizeof(sc_strMsg) - 1);
    g_SHA_done = 0;
    /* Start SHA calculation */
    SHA_Start(CRYPTO, CRYPTO_DMA_ONE_SHOT);

    /* Waiting for SHA calcuation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_SHA_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for SHA calcuation done time-out !\n");
            goto lexit;
        }
    }

    /* Read SHA calculation result */
    SHA_Read(CRYPTO, au32Key);
    Reg2Hex(64, au32Key, hash);
    printf("Message = %s\n", sc_strMsg);
    printf("Hash = ");
    pu8 = (uint8_t *)au32Key;

    for (i = 0; i < 32; i++)
        printf("%02x", pu8[i]);

    printf("\n");

    /*------------------------------------------------------------------------------*/
    /* Sign the message */

    SysTick->VAL = 0;
#ifndef USE_KS_KEY

    if (ECC_GenerateSignature(CRYPTO, CURVE_P_256, hash, d, k, R, S) < 0)
    {
        printf("ECC digital signature generation failed !\n");
        goto lexit;
    }

#else

    /* Use the private key in Key Store to sign the hash data of message */
    if (ECC_GenerateSignature_KS(CRYPTO, CURVE_P_256, hash, i32KeyIdx_d, i32KeyIdx_k, R, S) < 0)
    {
        printf("ECC digital signature generation failed !\n");
        goto lexit;
    }

    /* Delete the random k */
    KS_EraseKey(i32KeyIdx_k);

#endif
    time = 0xffffff - SysTick->VAL;

    printf("  R  = %s\n", R);
    printf("  S  = %s\n", S);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    /*------------------------------------------------------------------------------*/
    /* Verify the signature */

#ifndef USE_KS_KEY
    SysTick->VAL = 0;
    err = ECC_VerifySignature(CRYPTO, CURVE_P_256, hash, Qx, Qy, R, S);
#else

    Hex2Reg(Qx, au32Key);
    i32KeyIdx_Qx = KS_Write(KS_SRAM, KS_META_ECC | KS_META_256, au32Key);

    if (i32KeyIdx_Qx < 0)
    {
        printf("Fail to write key to Key Store !\n");
        goto lexit;
    }

    printf("i32KeyIdx_Qx = %d, remain size = %d\n", i32KeyIdx_Qx, KS_GetRemainSize(KS_SRAM));

    Hex2Reg(Qy, au32Key);
    i32KeyIdx_Qy = KS_Write(KS_SRAM, KS_META_ECC | KS_META_256, au32Key);

    if (i32KeyIdx_Qy < 0)
    {
        printf("Fail to write key to Key Store !\n");
        goto lexit;
    }

    printf("i32KeyIdx_Qy = %d, remain size = %d\n", i32KeyIdx_Qy, KS_GetRemainSize(KS_SRAM));

    SysTick->VAL = 0;
    err = ECC_VerifySignature_KS(CRYPTO, CURVE_P_256, hash, i32KeyIdx_Qx, i32KeyIdx_Qy, R, S);

#endif
    time = 0xffffff - SysTick->VAL;

    if (err < 0)
    {
        printf("ECC digital signature verification failed !\n");
        goto lexit;
    }
    else
    {
        printf("ECC digital signature verification OK.\n");
    }


    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    printf("Done\n");

lexit:

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
