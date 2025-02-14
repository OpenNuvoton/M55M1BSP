/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRC_CRC32_DMA code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

volatile uint32_t  g_u32IsTestOver = 0;
volatile uint8_t   g_u8CRCDoneFlag = 0;
/* 0:No INT, 1:CRC done, 2:CRC abort, 3:CRC unknow fail,
   4:CRC cinfig fails, 8: CRC DMA access fails.*/
void CRC_IRQHandler(void)
{
    volatile uint32_t reg;

    reg = CRC->DMASTS;

    if ((reg & CRC_DMASTS_ABORTED_Msk) == CRC_DMASTS_ABORTED_Msk)   /* target abort */
    {
        g_u8CRCDoneFlag = 0x2;
        printf("abort flag 0x%x\n", reg);
        CRC->DMASTS |= CRC_DMASTS_ABORTED_Msk;
    }

    if ((reg & CRC_DMASTS_CFGERR_Msk) == CRC_DMASTS_CFGERR_Msk) /* config error */
    {
        g_u8CRCDoneFlag = 0x4;
        printf("config error 0x%x\n", reg);
        CRC->DMASTS |= CRC_DMASTS_CFGERR_Msk;
    }

    if ((reg & CRC_DMASTS_ACCERR_Msk) == CRC_DMASTS_ACCERR_Msk) /* access error */
    {
        g_u8CRCDoneFlag = 0x8;
        printf("access error 0x%x\n", reg);
        CRC->DMASTS |= CRC_DMASTS_ACCERR_Msk;
    }

    if ((reg & CRC_DMASTS_FINISH_Msk) == CRC_DMASTS_FINISH_Msk) /* transfer done */
    {
        g_u8CRCDoneFlag = 0x1;
        printf("*transfer done 0x%x\n", reg);
        CRC->DMASTS |= CRC_DMASTS_FINISH_Msk;
    }
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRC0 module clock */
    CLK_EnableModuleClock(CRC0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}


uint32_t GetDMAMasterChecksum(uint32_t u32Address, uint32_t u32Size)
{
    /*Set input data address for CRC DMA Master*/
    CRC_SET_DMA_SADDR(CRC, (uint32_t)u32Address);

    CRC_SET_DMACNT_WORD(CRC, u32Size / 4);

    CRC_ENABLE_DMA_INT(CRC);

    CRC_DMA_START(CRC);

    while (CRC->DMACTL & CRC_DMACTL_START_Msk) {};

    CRC_IRQHandler();

    return CRC->CHECKSUM;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    uint32_t u32CRC32Checksum, u32DMAChecksum;
    uint32_t addr, size;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    size = 1024 * 2;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    CRC32 with DMA Sample Code                      |\n");
    printf("|       - Get APROM first %d bytes CRC result by    |\n", size);
    printf("|          a.) CPU write CRC data register directly   |\n");
    printf("|          b.) DMA Master write CRC data register           |\n");
    printf("+-----------------------------------------------------+\n\n");

    /*  Case a. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

    /* Start to execute CRC-CRC32 operation */
    for (addr = FMC_APROM_BASE; addr < (FMC_APROM_BASE + size); addr += 4)
    {
        CRC_WRITE_DATA(inpw(addr));
    }

    u32CRC32Checksum = CRC_GetChecksum();

    /*  Case b. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);
    u32DMAChecksum = GetDMAMasterChecksum(FMC_APROM_BASE, size);

    printf("APROM first %d bytes checksum:\n", size);
    printf("   - by CPU write:   0x%x\n", u32CRC32Checksum);
    printf("   - by DMA write:   0x%x\n", u32DMAChecksum);

    if ((u32CRC32Checksum) == (u32DMAChecksum))
    {
        if ((u32CRC32Checksum == 0) || (u32CRC32Checksum == 0xFFFFFFFF))
        {
            printf("\n[Get checksum ... WRONG]");
        }
        else
        {
            printf("\n[Compare checksum ... PASS]");
        }
    }
    else
    {
        printf("\n[Compare checksum ... WRONG]");
    }

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC0_MODULE);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
