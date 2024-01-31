/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to use KDF to derive new key
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/* KDF supports maximum key length is 65280 bits = 2040 words */
uint32_t g_au32Keyout[16]  = { 0 };

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

void DumpKey(uint32_t *pu32KeyBuf, uint32_t u32WordCnt)
{
    int32_t i;

    printf("Derived key: ");

    for (i = 0; i < u32WordCnt; i++)
        printf("%08X", pu32KeyBuf[i]);

    printf("\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t  i32RetCode = 0;
    uint8_t  au8KeyIn[32]   = "user secret key input",
                              au8Salt[32]    = "user secret salt",
                                               au8Label[12]   = "user label",
                                                                au8Context[16] = "user conext";

    NVT_UNUSED(i32RetCode);

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|     M55M1 KDF Derive Key Sample Code     |\n");
    printf("+------------------------------------------+\n");

    /* [HKDF] Derive key with user specified Key and PRNG generated Salt/Label/Conext */
    printf("\n[HKDF] Derive key with user key input and PRNG generated Salt/Label/Conext\n");
    KDF_SetKeyInput(au8KeyIn, sizeof(au8KeyIn));        /* Set user key input  */
    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));    /* Clear keyout buffer */

    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_RANDOM | KDF_LABEL_FROM_RANDOM | KDF_CONTEXT_FROM_RANDOM), 256, g_au32Keyout)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    DumpKey(g_au32Keyout, 256 / 32);

    /* [HKDF] Derive key with user specified Key and TRNG generated Salt/Label/Conext */
    printf("\n[HKDF] Derive key with user key input and TRNG generated Salt/Label/Conext\n");

    if ((i32RetCode = TRNG_Open()) != 0)
    {
        printf("TRNG_Open failed !\n");
        goto Error_Exit;
    }

    KDF_SetKeyInput(au8KeyIn, sizeof(au8KeyIn));        /* Set user key input  */
    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));    /* Clear keyout buffer */

    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_RANDOM | KDF_LABEL_FROM_RANDOM | KDF_CONTEXT_FROM_RANDOM), 512, g_au32Keyout)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    DumpKey(g_au32Keyout, 512 / 32);

    /* [HKDF] Derive key with user specified Key/Salt/Label/Context in register */
    printf("\n[HKDF] Derive key with user specified Key/Salt/Label/Context in register\n");
    KDF_SetKeyInput(au8KeyIn,   sizeof(au8KeyIn));      /* Set user key input  */
    KDF_SetSalt(au8Salt,    sizeof(au8Salt));           /* Set user salt       */
    KDF_SetLabel(au8Label,   sizeof(au8Label));         /* Set user label      */
    KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */
    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));    /* Clear keyout buffer */

    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), 256, g_au32Keyout)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    DumpKey(g_au32Keyout, 256 / 32);

    /* [HKDF] Derive key with user specified key in KS OTP and Salt/Label/Context in register */
    printf("\n[HKDF] Derive key with user specified key in KS OTP and Salt/Label/Context in register\n");

    if ((i32RetCode = KS_Open()) != 0)
    {
        printf("KS_Open failed !\n");
        goto Error_Exit;
    }

    /* Write KDF root OTP key to KS first */
    if (KS_GET_OTPKEY_STS(KS_KDF_ROOT_OTPKEY) == TRUE)
    {
        printf("KDF root key is not empty - Skip write KDF root key.\n");
    }
    else if (KS_WriteOTP(KS_KDF_ROOT_OTPKEY, KS_META_256, (uint32_t *)au8KeyIn) != KS_KDF_ROOT_OTPKEY)
    {
        printf("Write KDF root key failed !\n");
        goto Error_Exit;
    }

    KDF_SetSalt(au8Salt,    sizeof(au8Salt));           /* Set user salt       */
    KDF_SetLabel(au8Label,   sizeof(au8Label));         /* Set user label      */
    KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */
    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));    /* Clear keyout buffer */

    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_HKDF, (KDF_KEYIN_FROM_NVM | KDF_SALT_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), 512, g_au32Keyout)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    DumpKey(g_au32Keyout, 512 / 32);

    /* [KBKDF] Derive key with user specified Key/Label/Context in register */
    printf("\n[KBKDF] Derive key with user specified Key/Label/Context in register\n");
    KDF_SetKeyInput(au8KeyIn,   sizeof(au8KeyIn));      /* Set user key input  */
    KDF_SetLabel(au8Label,   sizeof(au8Label));         /* Set user label      */
    KDF_SetContext(au8Context, sizeof(au8Context));     /* Set user context    */
    memset(g_au32Keyout, 0x0, sizeof(g_au32Keyout));    /* Clear keyout buffer */

    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_KBKDF, (KDF_KEYIN_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), 256, g_au32Keyout)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    DumpKey(g_au32Keyout, 256 / 32);

    printf("\nDone\n");

Error_Exit:

    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
