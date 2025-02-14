/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to use KDF to derive new key saved in KeyStore (KS)
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/* Maximum key length of KDF derived to KS is 571 bits = 571 / 32 = 17.8 words. */
uint32_t g_au32Keyout[18]  = { 0 };

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
    CLK_EnableModuleClock(KDF0_MODULE);
    CLK_EnableModuleClock(TRNG0_MODULE);
    CLK_EnableModuleClock(KS0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

void DumpKeyFromKS(KS_MEM_Type eMemType, int32_t i32KeyIdx, uint32_t u32WordCnt)
{
    int32_t i;

    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));

    if (KS_Read(eMemType, i32KeyIdx, g_au32Keyout, u32WordCnt) < 0)
    {
        printf("[Error] Failed to read key %d from KS !\n", i32KeyIdx);
        return ;
    }

    printf("[Key in %s] Derived key: ", (eMemType == KS_SRAM) ? "SRAM" : "Flash");

    for (i = 0; i < u32WordCnt; i++)
        printf("%08X", g_au32Keyout[i]);

    printf("\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32RetCode = 0;
    int32_t i32KeyIdx;
    uint8_t au8KeyIn[32]   = "user secret key input";
    uint8_t au8Salt[32]    = "user secret salt";
    uint8_t au8Label[12]   = "user label";
    uint8_t au8Context[16] = "user context";

    NVT_UNUSED(i32RetCode);

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|  M55M1 KDF Derive Key to KS Sample Code  |\n");
    printf("+------------------------------------------+\n");

    if ((i32RetCode = KS_Open()) != 0)
    {
        printf("KS_Open failed !\n");
        goto Error_Exit;
    }

    /* [HKDF] Derive key to KS with user specified Key and PRNG generated Salt/Label/Conext */
    printf("\n[HKDF] Derive key with PRNG generated Salt/Label/Conext\n");
    KDF_SetKeyInput(au8KeyIn, sizeof(au8KeyIn));        /* Set user key input  */

    if ((i32KeyIdx = KDF_DeriveKeyToKS(KS_SRAM, eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_RANDOM | KDF_LABEL_FROM_RANDOM | KDF_CONTEXT_FROM_RANDOM), KDF_KS_KEYSIZE_128, KDF_KS_OWNER_CPU)) < 0)
    {
        printf("Failed to derive key to KS !\n");
        goto Error_Exit;
    }

    printf("  Derived key index: %d\n", i32KeyIdx);
    DumpKeyFromKS(KS_SRAM, i32KeyIdx, 4);

    /* [HKDF] Derive key with user specified Key and TRNG generated Salt/Label/Conext */
    printf("\n[HKDF] Derive key with user specified Key and TRNG generated Salt/Label/Conext\n");

    if ((i32RetCode = TRNG_Open()) != 0)
    {
        printf("TRNG_Open failed !\n");
        goto Error_Exit;
    }

    KDF_SetKeyInput(au8KeyIn, sizeof(au8KeyIn));        /* Set user key input  */

    if ((i32KeyIdx = KDF_DeriveKeyToKS(KS_FLASH, eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_RANDOM | KDF_LABEL_FROM_RANDOM | KDF_CONTEXT_FROM_RANDOM), KDF_KS_KEYSIZE_256, KDF_KS_OWNER_CPU)) < 0)
    {
        printf("Failed to derive key to KS !\n");
        goto Error_Exit;
    }

    printf("  Derived key index: %d\n", i32KeyIdx);
    DumpKeyFromKS(KS_FLASH, i32KeyIdx, 8);

    /* [HKDF] Derive key with user specified Key/Salt/Label/Context in register */
    printf("\n[HKDF] Derive key with user specified Key/Salt/Label/Context in register\n");
    KDF_SetKeyInput(au8KeyIn,  sizeof(au8KeyIn));       /* Set user key input  */
    KDF_SetSalt(au8Salt,       sizeof(au8Salt));        /* Set user salt       */
    KDF_SetLabel(au8Label,     sizeof(au8Label));       /* Set user label      */
    KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */

    if ((i32KeyIdx = KDF_DeriveKeyToKS(KS_SRAM, eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), KDF_KS_KEYSIZE_256, KDF_KS_OWNER_CPU)) < 0)
    {
        printf("[Error] Failed to derive key to KS !\n");
        goto Error_Exit;
    }

    printf("  Derived key index: %d\n", i32KeyIdx);
    DumpKeyFromKS(KS_SRAM, i32KeyIdx, 8);

    /* [HKDF] Derive key with user defined KDF root key in KS OTP and Salt/Label/Context in register */
    printf("\n[HKDF] Derive key with KS_KDF_ROOT_OTPKEY and Salt/Label/Context in register\n");
    printf("Note: This test will write a key to KS_KDF_ROOT_OTPKEY if it is empty.\n");
    printf("      * Please be aware that KS_KDF_ROOT_OTPKEY can only be updated three times.\n");
    printf("Press 'y' to continue with the KS_KDF_ROOT_OTPKEY test, or any other key to skip.\n");

    if (getchar() == 'y')
    {
        /* Write KDF root OTP key to KS first */
        if (KS_GET_OTPKEY_STS(KS_KDF_ROOT_OTPKEY) == TRUE)
        {
            printf("  KDF root key is not empty - Skip write KDF root key.\n");
        }
        else if (KS_WriteOTP(KS_KDF_ROOT_OTPKEY, KS_META_256, (uint32_t *)au8KeyIn) != KS_KDF_ROOT_OTPKEY)
        {
            printf("[Error] Write KDF root key failed !\n");
            goto Error_Exit;
        }
        else
            printf("  Write KDF root key successfully.\n");

        KDF_SetSalt(au8Salt,       sizeof(au8Salt));        /* Set user salt       */
        KDF_SetLabel(au8Label,     sizeof(au8Label));       /* Set user label      */
        KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */

        if ((i32KeyIdx = KDF_DeriveKeyToKS(KS_SRAM, eKDF_MODE_HKDF, (KDF_KEYIN_FROM_NVM | KDF_SALT_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), KDF_KS_KEYSIZE_256, KDF_KS_OWNER_CPU)) < 0)
        {
            printf("[Error] Failed to derive key to KS !\n");
            goto Error_Exit;
        }

        printf("  Derived key index: %d\n", i32KeyIdx);
        DumpKeyFromKS(KS_SRAM, i32KeyIdx, 8);
    }

    /* [KBKDF] Derive key to KS with user specified Key/Label/Context in register */
    printf("\n[KBKDF] Derive key to KS with user specified Key/Label/Context in register\n");
    KDF_SetKeyInput(au8KeyIn,  sizeof(au8KeyIn));       /* Set user key input  */
    KDF_SetLabel(au8Label,     sizeof(au8Label));       /* Set user label      */
    KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */

    if ((i32KeyIdx = KDF_DeriveKeyToKS(KS_SRAM, eKDF_MODE_KBKDF, (KDF_KEYIN_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), KDF_KS_KEYSIZE_571, KDF_KS_OWNER_CPU)) < 0)
    {
        printf("[Error] Failed to derive key to KS !\n");
        goto Error_Exit;
    }

    printf("  Derived key index: %d\n", i32KeyIdx);
    DumpKeyFromKS(KS_SRAM, i32KeyIdx, 18);

Error_Exit:
    printf("\nDone\n");

    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
