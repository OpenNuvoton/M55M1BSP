/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Configure memory region as read-only region
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

/*
 * Reference: https://www.keil.com/pack/doc/CMSIS/Core/html/group__mpu8__functions.html
 */

/* Base address and size must be 32-byte aligned */
#define REGION_FLASH_RW_BASE_ADDR   (MPU_INIT_BASE(0))
#define REGION_FLASH_RW_SIZE        (MPU_INIT_SIZE(0))
#define REGION_FLASH_RW_END_ADDR    (MPU_INIT_LIMIT(0))

#define REGION_SRAM_RW_BASE_ADDR    (MPU_INIT_BASE(1))
#define REGION_SRAM_RW_SIZE         (MPU_INIT_SIZE(1))
#define REGION_SRAM_RW_END_ADDR     (MPU_INIT_LIMIT(1))

#define REGION_SRAM_RO_BASE_ADDR    (MPU_INIT_BASE(2))
#define REGION_SRAM_RO_SIZE         (MPU_INIT_SIZE(2))
#define REGION_SRAM_RO_END_ADDR     (MPU_INIT_LIMIT(2))

void MemManage_Handler(void)
{
    uint32_t u32LR = 0;
    uint32_t *pu32SP;

    __ASM volatile("mov %0, lr\n" : "=r"(u32LR));

    if (u32LR & BIT2)
        pu32SP = (uint32_t *)__get_PSP();
    else
        pu32SP = (uint32_t *)__get_MSP();

    /*
     * 1. Disable MPU to allow simple return from MemManage_Handler().
     *    MemManage fault typically indicates code failure, and would
     *    be resolved by reset or terminating faulty thread in OS.
     * 2: Call ARM_MPU_Disable() will allow the code touch
     *    illegal address to be executed after return from
     *    HardFault_Handler(). If this line is comment out, this code
     *    will keep enter MemManage_Handler().
     */
    ARM_MPU_Disable();
    printf("\n  Memory Fault !\n");

    if (SCB->CFSR & SCB_CFSR_IACCVIOL_Msk)
    {
        printf("  Instruction access violation flag is raised.\n");
        /* Check disassembly code of MemManage_Handler to get stack offset of return address */
        printf("  Fault address: 0x%08X\n", pu32SP[10]);
    }

    if (SCB->CFSR & SCB_CFSR_DACCVIOL_Msk)
    {
        printf("  Data access violation flag is raised.\n");

        if (SCB->CFSR & SCB_CFSR_MMARVALID_Msk)
            printf("  Fault address: 0x%08X\n", SCB->MMFAR);
    }

    /* Clear MemManage fault status register */
    SCB->CFSR = SCB_CFSR_MEMFAULTSR_Msk;
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

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

#define RO(set)     (set)   /* Read-Only: Set to 1 for a read-only memory region.                   */
#define NP(set)     (set)   /* Non-Privileged: Set to 1 for a non-privileged memory region.         */
#define XN(set)     (set)   /* eXecute Never: Set to 1 for a non-executable memory region.          */

void MPU_TestAccess(void)
{
    /* Disable I-Cache and D-Cache before config MPU */
    SCB_DisableICache();
    SCB_DisableDCache();

    /* Configure MPU memory region defined in mpu_config_M55M1.h */
    InitPreDefMPURegion(NULL, 0);

    /* Enable I-Cache and D-Cache */
    SCB_EnableDCache();
    SCB_EnableICache();

    printf("\n==============================================\n");
    printf("RW Memory Region (Flash Memory) configuration:\n");
    printf("==============================================\n");
    printf(" Start address: 0x%08X\n", (uint32_t)REGION_FLASH_RW_BASE_ADDR);
    printf(" End address  : 0x%08X\n", (uint32_t)REGION_FLASH_RW_END_ADDR);
    printf(" Size         : %d KB\n", (uint32_t)(REGION_FLASH_RW_SIZE / 1024));
    printf(" Memory Type  : Normal\n");
    printf(" Permission   : Full access\n");
    printf("----------------------------------------------\n");

    printf("Press any key to test read access from RW memory region (Flash Memory).\n");
    getchar();
    printf("\nRead value from 0x%08X is 0x%08X.\n", (uint32_t)REGION_FLASH_RW_BASE_ADDR, M32(REGION_FLASH_RW_BASE_ADDR));

    printf("\n==============================================\n");
    printf("RW Memory Region (SRAM Memory) configuration:\n");
    printf("==============================================\n");
    printf(" Start address: 0x%08X\n", (uint32_t)REGION_SRAM_RW_BASE_ADDR);
    printf(" End address  : 0x%08X\n", (uint32_t)REGION_SRAM_RW_END_ADDR);
    printf(" Size         : %d KB\n", (uint32_t)(REGION_SRAM_RW_SIZE / 1024));
    printf(" Memory Type  : Normal\n");
    printf(" Permission   : Full access\n");
    printf("----------------------------------------------\n");

    printf("Press any key to test read access from RW memory region (SRAM Memory).\n");
    getchar();
    printf("\nRead value from 0x%08X is 0x%08X.\n", REGION_SRAM_RW_BASE_ADDR, M32(REGION_SRAM_RW_BASE_ADDR));

    printf("\n==============================================\n");
    printf("RO Memory Region (SRAM Memory) configuration:\n");
    printf("==============================================\n");
    printf(" Start address: 0x%08X\n", (uint32_t)REGION_SRAM_RO_BASE_ADDR);
    printf(" End address  : 0x%08X\n", (uint32_t)REGION_SRAM_RO_END_ADDR);
    printf(" Size         : %d KB\n", (uint32_t)(REGION_SRAM_RO_SIZE / 1024));
    printf(" Memory Type  : Normal\n");
    printf(" Permission   : No write access\n");
    printf("----------------------------------------------\n");

    printf("Press any key to test read/write access from RO memory region (SRAM Memory).\n");
    printf("(It should trigger a memory fault exception.)\n");
    getchar();
    printf("\nRead value from 0x%08X is 0x%08X.\n", REGION_SRAM_RO_BASE_ADDR, M32(REGION_SRAM_RO_BASE_ADDR));
    printf("\nWrite 0 to 0x%08X.\n", REGION_SRAM_RO_BASE_ADDR);
    M32(REGION_SRAM_RO_BASE_ADDR) = 0;
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|  M55M1 MPU Access Permission Sample Code |\n");
    printf("+------------------------------------------+\n");
    printf("MPU region count: %d\n", (uint32_t)(MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos);
    MPU_TestAccess();

    printf("\nDone\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
