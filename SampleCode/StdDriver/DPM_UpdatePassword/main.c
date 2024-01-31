/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to update and compare Secure DPM password.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au32Pwd_5[4] = { 0x55555555, 0x55555555, 0x55555555, 0x55555555 };
    uint32_t au32Pwd_A[4] = { 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA, 0xAAAAAAAA };
    int32_t i32RetVal;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|     M55M1 Secure DPM Update Password     |\n");
    printf("+------------------------------------------+\n");
    printf("\nCPU @ %d Hz\n", SystemCoreClock);

    /* Get DPM status */
    printf("Secure DPM manages Secure region debug authority.\n");
    printf("Get Secure DPM status:\n");
    printf("Secure region debug is %s.\n", DPM_GetDebugDisable(SECURE_DPM) ? "Disabled" : "Enabled");
    printf("Secure region debug is %s.\n\n", DPM_GetDebugLock(SECURE_DPM) ? "Locked" : "not Locked");

    /* Update password */
    printf("Update Secure DPM password:    ");

    i32RetVal = DPM_SetPasswordUpdate(SECURE_DPM, au32Pwd_5);

    if (i32RetVal == 0)
    {
        printf("\n  Password update has reached maximum time ! Please erase chip.\n");
        goto lexit;
    }
    else if (i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("OK.\n");
    }

    /* Compare password fail */
    printf("Compare with wrong password:   ");

    i32RetVal = DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_A);

    if (i32RetVal == 0)
    {
        if (DPM_GetPasswordErrorFlag(SECURE_DPM) == 1)
        {
            printf("Password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);
        }
        else
        {
            printf("OK.\n");
        }
    }
    else if (i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("\n  Password compare has reached maximum time ! Please erase chip.\n");
        goto lexit;
    }

    /* Compare password pass */
    printf("Compare with correct password: ");

    i32RetVal = DPM_SetPasswordCompare(SECURE_DPM, au32Pwd_5);

    if (i32RetVal == 0)
    {
        if (DPM_GetPasswordErrorFlag(SECURE_DPM) == 1)
        {
            printf("Password is wrong.\n");
            DPM_ClearPasswordErrorFlag(SECURE_DPM);
        }
        else if (i32RetVal == DPM_TIMEOUT_ERR)
        {
            printf("Wait for DPM busy flag cleared time-out!\n");
            goto lexit;
        }
        else
        {
            printf("OK.\n");
        }
    }
    else if (i32RetVal == DPM_TIMEOUT_ERR)
    {
        printf("Wait for DPM busy flag cleared time-out!\n");
        goto lexit;
    }
    else
    {
        printf("\n  Password compare has reached maximum time ! Please erase chip.");
        goto lexit;
    }

    printf("\nDone.\n");

lexit:

    while (1);
}



/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
