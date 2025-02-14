/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo to use the AES in Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define BUFF_SIZE   1024

static volatile int32_t s_i32AES_Done = FALSE, s_i32AES_Err = FALSE;

static uint8_t g_au8In[] __ALIGNED(4) =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};
static uint8_t g_au8Out[BUFF_SIZE] __ALIGNED(4);
static uint8_t g_au8Out2[BUFF_SIZE] __ALIGNED(4);

int AES_Test(CRYPTO_T *pCrypto, KS_MEM_Type mem, int32_t keyIdx);
void DumpBuf(uint8_t *pu8Buf, uint32_t u32BufByteSize);
void SYS_Init(void);
void UART_Init(void);

NVT_ITCM void CRYPTO_IRQHandler(void)
{
    if (AES_GET_INT_FLAG(CRYPTO) & CRYPTO_INTSTS_AESIF_Msk)
    {
        s_i32AES_Done = TRUE;
    }

    if (AES_GET_INT_FLAG(CRYPTO) & CRYPTO_INTSTS_AESEIF_Msk)
    {
        s_i32AES_Err = TRUE;
    }

    AES_CLR_INT_FLAG(CRYPTO);
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
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

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
    int32_t i32KeyIdx;

    //Key = 0x7A29E38E063FF08A2F7A7F2A93484D6F
    uint32_t au32Key[4] = { 0x93484D6F, 0x2F7A7F2A, 0x063FF08A, 0x7A29E38E };

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|         M55M1 KS AES Sample Code         |\n");
    printf("+------------------------------------------+\n");

    KS_Open();
    //KS_EraseAll(KS_SRAM);

    i32KeyIdx = KS_Write(KS_SRAM, KS_META_128 | KS_META_AES, au32Key);

    if (i32KeyIdx < 0)
    {
        printf("Fail to write key to KS SRAM !\n KS SRAM is full then KS_EraseAll(KS_SRAM) to erase.\n");
        printf("KS SRAM remain size = %d, remain key count: %d\n",
               KS_GetRemainSize(KS_SRAM), KS_GetRemainKeyCount(KS_SRAM));
        KS_EraseAll(KS_SRAM);
        goto lexit;
    }

    printf("KS SRAM remain size: %d\n", KS_GetRemainSize(KS_SRAM));


    printf("The key in key store:\n");
    DumpBuf((uint8_t *)&au32Key[0], 16);

    AES_Test(CRYPTO, KS_SRAM, i32KeyIdx);

    printf("Done\n");

lexit:

    while (1) {}
}


int AES_Test(CRYPTO_T *pCrypto, KS_MEM_Type mem, int32_t keyIdx)
{
    uint32_t u32DataByteSize;
    // IV = 0xF8C44B6FBDF96B835547FF45DE1FFC92
    uint32_t au32IV[4] = { 0xDE1FFC92, 0x5547FF45, 0xBDF96B83, 0xF8C44B6F };
    uint32_t u32TimeOutCnt;

    (void)mem;

    /* Enable Crypto interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);

    /* Original data */
    printf("The input data:\n");
    DumpBuf(g_au8In, sizeof(g_au8In));

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(pCrypto, 0, 1, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(pCrypto, KS_SRAM, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(pCrypto, 0, au32IV);

    /* Prepare the source data and output buffer */
    u32DataByteSize = sizeof(g_au8In);

    if (u32DataByteSize & 0xf)
    {
        printf("Alignment Error!\n");
        printf("For AES CFB mode, Input data must be 16 bytes (128 bits) alignment.\n");
        return -1;
    }

    AES_SetDMATransfer(pCrypto, 0, (uint32_t)g_au8In, (uint32_t)g_au8Out, (uint32_t)u32DataByteSize);

    AES_ENABLE_INT(pCrypto);
    s_i32AES_Done = FALSE;
    s_i32AES_Err = FALSE;

    /* Start AES encode */
    AES_Start(pCrypto, 0, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!s_i32AES_Done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES encode time-out!\n");
            return -1;
        }
    }

    if (s_i32AES_Err)
    {
        printf("AES Encode Fail!\n");
        return -1;
    }

    printf("AES encrypt done. The output data:\n");
    DumpBuf(g_au8Out, 16);

    /*---------------------------------------
     *  AES-128 ECB mode decrypt
     *---------------------------------------*/
    AES_Open(pCrypto, 0, 0, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(pCrypto, KS_SRAM, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(pCrypto, 0, au32IV);

    AES_SetDMATransfer(pCrypto, 0, (uint32_t)g_au8Out, (uint32_t)g_au8Out2, (uint32_t)u32DataByteSize);

    AES_ENABLE_INT(pCrypto);
    s_i32AES_Done = FALSE;
    s_i32AES_Err = FALSE;

    /* Start AES decode */
    AES_Start(pCrypto, 0, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!s_i32AES_Done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES decode time-out!\n");
            return -1;
        }
    }

    if (s_i32AES_Err)
    {
        printf("AES Decode Fail!\n");
        return -1;
    }

    printf("AES decrypt done. Decode Data:\n");
    DumpBuf(g_au8Out2, u32DataByteSize);

    if (memcmp((char *)g_au8In, (char *)g_au8Out2, (uint32_t)u32DataByteSize))
    {
        printf("[FAILED]\n");
        return -1;
    }


    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
