/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Firmware update sample code (Bank0 Loader).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"

#define CREATE_BANK1_APP   1
#define PLL_CLOCK          FREQ_180MHZ

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32ExecBank;             /* CPU executing in which Bank              */
static volatile uint32_t  s_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  s_u32DbAddr;               /* dual bank program current flash address  */
static volatile uint32_t  s_u32TickCnt;              /* timer ticks - 100 ticks per second       */
/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void StartTimer0(void);
void SYS_Init(void);
void Download(void);

NVT_ITCM uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for (u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }

    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void SysTick_Handler(void)
{
    /* Increase timer tick */
    s_u32TickCnt++;

    /* Calculate CRC32 value, just to consume CPU time  */
    FuncCrc32(DTCM_BASE, 0x100);
}

NVT_ITCM void WDT0_IRQHandler(void)
{
    WDT_RESET_COUNTER(WDT0);

    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }
}

void StartTimer0(void)
{
    /* Set TIMER0 clock source  */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HXT, 0);
    /* Enable TIMER0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Disable timer */
    TIMER_Stop(TIMER0);
    /* Clear interrupt status */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* Clear timer counter */
    //TIMER0->CNT = 0;

    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 12);
    /* Maximum time  */
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFE);
    /* Start timer */
    TIMER_Start(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable external high speed crystal (HXT) */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for external high speed crystal ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable module clock */
    CLK_EnableModuleClock(CRC0_MODULE);
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

void Download(void)
{
    int32_t  i32Err;
    uint32_t u32ExecBank = s_u32ExecBank;

    printf("\n Bank%d processing, download data to Bank%d APP Base.\n\n\n", u32ExecBank, u32ExecBank);

    /* Dual bank background program address */
    s_u32DbAddr   = APP_BASE;
    /* Dual bank background program length */
    s_u32DbLength = APP_SIZE;

    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Setup SysTick Timer for 1 ms interrupts  */
        printf("[Error] SysTick_Config failed !\n");

        while (1);
    }

    StartTimer0();

    /* Use Xmodem to download firmware from PC*/
    i32Err = XmodemRecv(s_u32DbAddr);

    if (i32Err < 0)
    {
        printf("\n[Error] Xmodem transfer fail !\n");

        while (1);
    }
    else
    {
        printf("\nXomdem transfer done.\n");
        printf("Total trnasfer size is %d\n", i32Err);
    }

    printf("\n Firmware download completed.\n");
}

#ifdef TESTCHIP_ONLY
// [Begin] TESTCHIP_ONLY - This function should be removed in M55M1.
uint32_t FMC_CheckAllOne(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < (u32StartAddr + u32ByteSize); u32Addr += 4)
    {
        if (FMC_Read(u32Addr) != 0xFFFFFFFF)
        {
            return READ_ALLONE_NOT;
        }
    }

    return READ_ALLONE_YES;
}

uint32_t FMC_GetChkSum(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32CRC32Checksum = 0xFFFFFFFF;

    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFFul, CRC_CPU_WDATA_32);
    CRC_SET_DMA_SADDR(CRC, u32StartAddr);
    CRC_SET_DMACNT_WORD(CRC, u32ByteSize / 4);
    CRC_DMA_START(CRC);

    while ((CRC_GET_STATUS(CRC) & CRC_DMASTS_FINISH_Msk) == 0)
        ;

    CRC->DMASTS = CRC_DMASTS_FINISH_Msk;
    u32CRC32Checksum = CRC_GetChecksum();

    return u32CRC32Checksum;
}
// [End] TESTCHIP_ONLY - This function should be removed in real chip.
#endif

int main()
{
    uint8_t u8GetCh;
    uint32_t i, u32ExecBank;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        /* Check CPU run at Bank0 or Bank1*/
        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank%d Loader processing \n\n", s_u32ExecBank);

        /* Get loader CRC */
        u32Loader0ChkSum = FMC_GetChkSum(LOADER_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_APROM_BANK_SIZE + LOADER_BASE, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08X\n Loader1 checksum: 0x%08X\n", u32Loader0ChkSum, u32Loader1ChkSum);

        /* Get app CRC */
        u32App0ChkSum = FMC_GetChkSum(APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_APROM_BANK_SIZE + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08X \n App1 checksum: 0x%08X\n", u32App0ChkSum, u32App1ChkSum);

        /* Write firmware CRC in CRC base for following checking */
        printf("\n Firmware CRC in [0x%08X] is [0x%08X]\n", (uint32_t)FW_CRC_BASE, FMC_Read(FW_CRC_BASE));

        if (FMC_Read(FW_CRC_BASE) == 0xFFFFFFFF)
        {
            FMC_Write(FW_CRC_BASE, u32App0ChkSum);
            printf("\n Update Firmware CRC in [0x%08X] is [0x%08X]\n", (uint32_t)FW_CRC_BASE, FMC_Read(FW_CRC_BASE));
        }

        /* Write backup firmware CRC in backup CRC base for following checking */
        printf("\n Backup Firmware CRC in [0x%08X] is [0x%08X]\n", (uint32_t)BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));

        if (FMC_Read(BACKUP_FW_CRC_BASE) == 0xFFFFFFFF)
        {

            FMC_Write(BACKUP_FW_CRC_BASE, u32App1ChkSum);
            printf("\n Update Firmware CRC in [0x%08X] is [0x%08X]\n", (uint32_t)BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));
        }

        /* Create the other bank loader for executing bank remap */
        if ((s_u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create Bank%d Loader ... \n",  s_u32ExecBank ^ 1);

            /* Erase loader region */
            for (i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_APROM_BANK_SIZE * (s_u32ExecBank ^ 1) + i);
            }

            /* Create loader in the other bank */
            for (i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 4)
            {
                u32ExecBank = s_u32ExecBank;
                FMC_Write(FMC_APROM_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i));
            }

            printf(" Create Bank%d Loader completed. \n", (s_u32ExecBank ^ 1));
        }

#if CREATE_BANK1_APP

        if ((s_u32ExecBank == 0) && ((FMC_CheckAllOne((APP_BASE + FMC_APROM_BANK_SIZE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create Bank%d App ... \n", s_u32ExecBank ^ 1);

            /* Erase app region */
            for (i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (s_u32ExecBank ^ 1) + i);
            }

            /* Create app in the other bank (just for test) */
            for (i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 4)
            {
                u32ExecBank = s_u32ExecBank;
                FMC_Write(FMC_BANK_SIZE * (u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * u32ExecBank) + i));
            }

            printf(" Create Bank%d App completed! \n", (s_u32ExecBank ^ 1));
        }

#endif

        /* To check if system has been reset by WDT time-out reset or not */
        if (WDT_GET_RESET_FLAG(WDT0) == 1)
        {
            WDT_CLEAR_RESET_FLAG(WDT0);
            printf("\n === System reset by WDT time-out event === \n");
            printf(" Press any key to remap back to backup FW\n");
            getchar();

            /* Remap to Bank1 to execute backup firmware */
            FMC_RemapBank(1);
            s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
            printf("\n Bank%d Loader after remap  \n\n", s_u32ExecBank);

            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            SYS_ResetCPU();
        }

        printf("\n Execute Bank%d APP ? [y/n] \n", s_u32ExecBank);
        u8GetCh = (uint8_t)getchar();
        printf("\n User select [%c] \n", u8GetCh);

        if (u8GetCh == 'y')
        {
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            SYS_ResetCPU();
        }
        else
        {
            printf("\n Download new firmware ? [y/n] \n");
            u8GetCh = (uint8_t)getchar();
            printf("\n User select [%c] \n", u8GetCh);

            if (u8GetCh == 'y')
            {
                /* Download new firmware */
                Download();
                printf("\nPress any key to execute new firmware.\n");
                getchar();
                /* Remap to App */
                FMC_SetVectorPageAddr(APP_BASE);
                SYS_ResetCPU();
            }
            else
            {
                /* Remap to Loader */
                FMC_SetVectorPageAddr(LOADER_BASE);
                SYS_ResetCPU();
            }

        }

    } while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
