/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to select SRAM power mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


extern int IsDebugFifoEmpty(void);
static volatile uint8_t s_u8IsINTEvent;

void WDT0_IRQHandler(void);
void PowerDownFunction(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void WDT0_IRQHandler(void)
{
    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0))
    {
        printf("WDT time-out!\n");

        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }

    if (WDT_GET_TIMEOUT_WAKEUP_FLAG(WDT0))
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG(WDT0);
    }

    s_u8IsINTEvent = 1;

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(&WDT0->STATUS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);

    /* Enter to Power-down mode */
    PMC_PowerDown();
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable CRC0 module clock */
    CLK_EnableModuleClock(CRC0_MODULE);

    /* Select WDT0 clock source from LXT */
    CLK_SetModuleClock(WDT0_MODULE, CLK_WDTSEL_WDT0SEL_LXT, NA);

    /* Enable WDT0 module clock */
    CLK_EnableModuleClock(WDT0_MODULE);

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
    uint32_t au32SRAMCheckSum[13] = {0};
    uint32_t u32SRAMSize = 65536;
    uint32_t u32Idx, u32Addr, u32SRAMStartAddr = 0;
    uint32_t u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|       SRAM Power Mode Sample Code     |\n");
    printf("+---------------------------------------+\n\n");

    printf("Press any key to start test\n");

    getchar();

    /*
        SRAM power mode can select as normal mode, retention mode and power shut down mode.
        SRAM is able to access only in normal mode and SRAM clock have to be enabled.
        The unused SRAM can be set in power shut down mode, but SRAM data will not kept.
        This sample code will set SRAM bank1 (0x20180000 - 0x201FFFFF) in normal mode,
        and set SRAM bank2 (0x20200000 - 0x2024_FFFF) in power shut down mode.
        The SRAM bank2 checksum will be different after setting in power shut down mode.
    */

    /* Unlock protected registers before setting SRAM power mode */
    SYS_UnlockReg();

    /* Select SRAM power mode:
       SRAM bank1 is in normal mode.
       SRAM bank2 is in normal mode.
    */
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM0PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM1PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM2PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM3PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM4PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM5PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM6PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB1PC_SRAM7PMS_Msk, PMC_SYSRB1PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM0PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM1PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM2PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM3PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM4PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);

    /* Calculate SRAM checksum */
    printf("Calculate SRAM checksum before Power-down:\n\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20180000;

    /* Calculate SRAM checksum */
    for (u32Idx = 0; u32Idx < 13; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for (u32Addr = u32SRAMStartAddr; u32Addr < (u32SRAMStartAddr + u32SRAMSize); u32Addr += 4)
        {
            CRC_WRITE_DATA(inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[0]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[1]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[2]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 5 Checksum [0x%08X]\n",   au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 6 Checksum [0x%08X]\n",   au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 7 Checksum [0x%08X]\n\n", au32SRAMCheckSum[7]);
    printf("SRAM Bank2 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[8]);
    printf("SRAM Bank2 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[10]);
    printf("SRAM Bank2 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[11]);
    printf("SRAM Bank2 Region 4 Checksum [0x%08X]\n\n", au32SRAMCheckSum[12]);

    /* Select SRAM power mode:
       SRAM bank2 is in power shut down mode.
    */
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM0PMS_Msk, PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM1PMS_Msk, PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM2PMS_Msk, PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM3PMS_Msk, PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM4PMS_Msk, PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN);

    /* Enter to Power-down mode and wake-up by WDT interrupt */
    printf("Enter to Power-down mode ...\n");

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT0, WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT0);

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (s_u8IsINTEvent == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!");
            break;
        }
    }

    printf("wake-up!\n\n");

    /* Select SRAM power mode:
       SRAM bank2 is in normal mode.
    */
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM0PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM1PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM2PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM3PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);
    PMC_SetSRAMPowerMode(PMC_SYSRB2PC_SRAM4PMS_Msk, PMC_SYSRB2PC_SRAM_NORMAL);

    /* Calculate SRAM checksum */
    printf("Calculate SRAM CheckSum after wake-up:\n\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = 0x20180000;

    /* Calculate SRAM checksum */
    for (u32Idx = 0; u32Idx < 13; u32Idx++)
    {
        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for (u32Addr = u32SRAMStartAddr; u32Addr < u32SRAMStartAddr + u32SRAMSize; u32Addr += 4)
        {
            CRC_WRITE_DATA(inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Bank1 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[0]);
    printf("SRAM Bank1 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[1]);
    printf("SRAM Bank1 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[2]);
    printf("SRAM Bank1 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[3]);
    printf("SRAM Bank1 Region 4 Checksum [0x%08X]\n",   au32SRAMCheckSum[4]);
    printf("SRAM Bank1 Region 5 Checksum [0x%08X]\n",   au32SRAMCheckSum[5]);
    printf("SRAM Bank1 Region 6 Checksum [0x%08X]\n",   au32SRAMCheckSum[6]);
    printf("SRAM Bank1 Region 7 Checksum [0x%08X]\n\n", au32SRAMCheckSum[7]);
    printf("SRAM Bank2 Region 0 Checksum [0x%08X]\n",   au32SRAMCheckSum[8]);
    printf("SRAM Bank2 Region 1 Checksum [0x%08X]\n",   au32SRAMCheckSum[9]);
    printf("SRAM Bank2 Region 2 Checksum [0x%08X]\n",   au32SRAMCheckSum[10]);
    printf("SRAM Bank2 Region 3 Checksum [0x%08X]\n",   au32SRAMCheckSum[11]);
    printf("SRAM Bank2 Region 4 Checksum [0x%08X]\n\n", au32SRAMCheckSum[12]);

    /* Disable WDT NVIC */
    NVIC_DisableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Close(WDT0);

    /* Disable WDT interrupt function */
    WDT_DisableInt(WDT0);

    printf("Test Done.\n");

    while (1);
}
