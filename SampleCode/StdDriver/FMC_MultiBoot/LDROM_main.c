/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   A multi-boot system to boot different applications from different address.
 *          This sample code implemented 1 LDROM code and 4 APROM code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
/* Empty function to reduce code size for GCC */
void ProcessHardFault(uint32_t *pu32StackFrame) {}
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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

int main(void)
{
    uint8_t u8Char;     /* variables */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0 ~ 0x3FF.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot_0x100000, RO=0x100000
            FMC_MultiBoot_0x160000, RO=0x160000
            FMC_MultiBoot_0x1C0000, RO=0x1C0000
            FMC_MultiBoot_0x220000, RO=0x220000
            FMC_MultiBoot_0x280000, RO=0x280000
            FMC_MultiBoot_LDROM,    RO=0xF100000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\nVECMAP: 0x%08X\n", FMC_GetVECMAP());
    printf("PC    : 0x%08X\n", __PC());
    printf("Press any key to Boot from 0x100000.\n");
    getchar();
    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */
    FMC_SetVectorPageAddr(0x100000);

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed !\n");
        goto lexit;
    }

    /* Use reset CPU only or reset system to reset to new vector page */
#if 0
    printf("SYS_ResetCPU\n");
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();
#else
    printf("NVIC_SystemReset\n");
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Reset System to reset to new vector page. */
    NVIC_SystemReset();
#endif

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
