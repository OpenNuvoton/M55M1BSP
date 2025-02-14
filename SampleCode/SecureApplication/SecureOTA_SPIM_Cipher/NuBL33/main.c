/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to generate the NuBL33 and can be authenticated by NuBL2.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL32/NuBL32.h"

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
extern uint32_t GetSystemCoreClock(void);   /* NuBL32 Non-secure callable function */

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    printf("\n\nCPU @ %d Hz\n", GetSystemCoreClock());
    printf("+-------------------------------------------------+\n");
#ifdef __NEW_VER__
    printf("|  SecureOTA_SPIM_Cipher - NuBL33_v2 Sample Code  |\n");
#else
    printf("|  SecureOTA_SPIM_Cipher - NuBL33_v1 Sample Code  |\n");
#endif
    printf("+-------------------------------------------------+\n\n");

    printf("System is executing in NuBL33.\n\n");

    while (1) {}
}
