/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demo to use ECC ECDH with Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize);
void SYS_Init(void);
void UART_Init(void);

NVT_ITCM void CRYPTO_IRQHandler(void)
{
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
    int32_t err;
    int32_t i32KeyIdx_d, i32ShareKeyIdx;
    uint32_t time;
    uint32_t au32ECC_N[18] = {0};

    /* Public Key of B sdie. Gen by private key = 74C57C8F23BE25F0EF591CEF81E89D1DE08CFDD7E3CADC8670A757D07A961DF0 */
    char Qx[] = "43A5FC3C3347670FADC59972E0DE55E3FBC9055DA48F49DBA1A29E3CC602A2F6";
    char Qy[] = "6232E772198811E1EB541640B872DC137ABF1D01FB8F3AE60F3432D26091772F";

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("CPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|         M55M1 KS ECDH Sample Code        |\n");
    printf("+------------------------------------------+\n");

    /* Init Key Store */
    KS_Open();

    /* Enable CRYPTO interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);
    ECC_ENABLE_INT(CRYPTO);
    SHA_ENABLE_INT(CRYPTO);

    for (i = 0; i < 18; i++)
        au32ECC_N[i] = 0;

    au32ECC_N[7] = 0xFFFFFFFFul;
    au32ECC_N[6] = 0x00000001ul;
    au32ECC_N[5] = 0x00000000ul;
    au32ECC_N[4] = 0x00000000ul;
    au32ECC_N[3] = 0x00000000ul;
    au32ECC_N[2] = 0xFFFFFFFFul;
    au32ECC_N[1] = 0xFFFFFFFFul;
    au32ECC_N[0] = 0xFFFFFFFFul;

    /* We need to prepare N first for RNG to generate a private key */
    err = RNG_ECDH_Init(PRNG_KEY_SIZE_256, au32ECC_N);

    if (err)
    {
        printf("PRNG ECDH Inital failed\n");
        goto lexit;
    }


    /* Reset SysTick to measure time */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /*------------------------------------------------------------------------------*/
    /* Generate private key to Key Store */
    /* NOTE: The private key must be generated form RNG hardware when using ECDH with key store */
    i32KeyIdx_d = RNG_ECDH(PRNG_KEY_SIZE_256);

    if (i32KeyIdx_d < 0)
    {
        printf("[FAILED]\n");
        printf("Fail to write k to KS SRAM\n");
        goto lexit;
    }

    /*-----------------------------------------------------------------------------------------------*/
    /* Calcualte Share Key by private key A and publick key B */
    SysTick->VAL = 0;

    if ((i32ShareKeyIdx = ECC_GenerateSecretZ_KS(CRYPTO, CURVE_P_256, i32KeyIdx_d, NULL, Qx, Qy, 2, 4, NULL)) < 0)
    {
        printf("ECC ECDH share key calculation fail\n");
        goto lexit;
    }

    printf("Share Key Idx for A Side = %d, remain size = %d\n", i32ShareKeyIdx, KS_GetRemainSize(KS_SRAM));

    time = 0xffffff - SysTick->VAL;

    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    printf("Done\n");

lexit:

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
