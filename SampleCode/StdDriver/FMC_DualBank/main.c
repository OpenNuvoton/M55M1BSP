/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show FMC dual bank capability.
 *          Program APROM bank1 while code is running on APROM bank0
 *          without being blocked.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define DB_PROG_LEN         (4 * FMC_FLASH_PAGE_SIZE)  /* Background program length  */
#define CRC32_LOOP_CNT      500                        /* Loop count                 */

/*
 *  Dual bank background program state
 */
enum
{
    DB_STATE_START,                              /* Start background dual bank program       */
    DB_STATE_ERASE,                              /* Executing ISP page erase                 */
    DB_STATE_PROGRAM,                            /* Executing ISP write                      */
    DB_STATE_DONE,                               /* All queued ISP operations finished. Idle */
    DB_STATE_FAIL                                /* ISP command failed or verify error       */
};

static volatile int32_t   db_state = DB_STATE_DONE; /* dual bank background program state       */
static volatile uint32_t  db_length;                /* dual bank program remaining length       */
static volatile uint32_t  db_addr;                  /* dual bank program current flash address  */
static volatile uint32_t  g_tick_cnt;               /* timer ticks - 100 ticks per second       */
static volatile uint32_t  s_u32CRC = 0;

NVT_ITCM void SysTick_Handler(void)
{
    g_tick_cnt++;                                /* increase timer tick                      */

    if (db_state == DB_STATE_DONE)               /* Background program is in idle state      */
    {
        return;
    }

    if (db_length == 0)                          /* Background program done?                 */
    {
        db_state = DB_STATE_DONE;                /* enter idle state                         */
        return;
    }

    if (FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
        return;                                  /* ISP is busy, postpone to next called     */

    /*
     *  Dual-bank background program...
     */
    switch (db_state)
    {
        case DB_STATE_START:
            if (db_addr & ~FMC_PAGE_ADDR_MASK)
            {
                printf("Warning - dual bank start address is not page aligned!\n");
                db_state = DB_STATE_FAIL;
                break;
            }

            if (db_length & ~FMC_PAGE_ADDR_MASK)
            {
                printf("Warning - dual bank length is not page aligned!\n");
                db_state = DB_STATE_FAIL;
                break;
            }

            db_state = DB_STATE_ERASE;              /* Next state is to erase flash            */
            break;

        case DB_STATE_ERASE:
            printf("Erase 0x%x [%d]\n", db_addr, g_tick_cnt);
            FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;   /* ISP page erase command                   */
            FMC->ISPADDR = db_addr;                 /* page address                             */
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;    /* trigger ISP page erase and no wait       */

            db_state = DB_STATE_PROGRAM;            /* Next state is to program flash           */
            break;

        case DB_STATE_PROGRAM:
            if ((db_addr & ~FMC_PAGE_ADDR_MASK) == 0)
                printf("Erase done [%d]\n", g_tick_cnt);

            FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;      /* ISP word program command                 */
            FMC->ISPADDR = db_addr;                 /* word program address                     */
            FMC->ISPDAT  = db_addr;                 /* 32-bits data to be programmed            */
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;    /* trigger ISP program and no wait          */

            db_addr += 4;                           /* advance to next word                     */
            db_length -= 4;

            if ((db_addr & ~FMC_PAGE_ADDR_MASK) == 0)
            {
                /* have reached start of next page          */
                db_state = DB_STATE_ERASE;          /* next state, erase page                   */
            }

            break;

        default:
            printf("Unknown db_state state!\n");
            break;
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 20MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

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

static const uint32_t crc32_tab[] =
{
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

#define CRC_SEED    0xFFFFFFFF

uint32_t  func_crc32(uint32_t start, uint32_t len)
{
    uint32_t  crc = CRC_SEED;
    uint32_t  idx, data32;
    uint8_t   data8;
    int       i;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for (idx = 0; idx < len; idx += 4)
    {
        data32 = *(uint32_t *)(start + idx);

        for (i = 0; i < 4; i++)
        {
            data8 = (data32 >> (i * 8)) & 0xff;
            crc = crc32_tab[(crc ^ data8) & 0xFF] ^ (crc >> 8);
        }
    }

    /* CHECKSUM_COM */
    return crc ^ ~0U;
}

void StartTimer0(void)
{
    volatile int32_t i32Timeout = 1000;

    CLK_EnableModuleClock(TMR0_MODULE);     /* Enable TIMER0 clock          */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, MODULE_NoMsk);

    TIMER_Stop(TIMER0);
    TIMER_ClearIntFlag(TIMER0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);
    TIMER_SET_PRESCALE_VALUE(TIMER0, ((__HIRC / 1000000) << TIMER_CTL_PSC_Pos));
    TIMER_ResetCounter(TIMER0);
    TIMER_Start(TIMER0);

    while (TIMER_IS_ACTIVE(TIMER0) == 0)
    {
        if (i32Timeout-- < 0)
        {
            printf("Timer0 active timeout !\n");
            break;
        }
    }
}

int main(void)
{
    uint32_t    u32LoopCnt;     /* Loop counter         */
    uint32_t    u32Addr;        /* Flash address        */
    uint32_t    u32TimerCnt;    /* TIMER0 counter value */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|    M55M1 FMC Dual Bank Sample Code       |\n");
    printf("+------------------------------------------+\n");

    SYS_UnlockReg();                /* Unlock protected registers       */
    FMC_Open();                     /* Enable FMC ISP function          */
    FMC_ENABLE_AP_UPDATE();         /* Enable FMC erase/program APROM   */

    db_state = DB_STATE_DONE;       /* Dual bank program state idle     */
    s_u32CRC = 0;

    SysTick_Config(1000);
    StartTimer0();

    for (u32LoopCnt = 0; u32LoopCnt < CRC32_LOOP_CNT; u32LoopCnt++)
    {
        s_u32CRC += func_crc32(FMC_APROM_BASE, 0x10000);      /* Calculate 64KB CRC32 value, just to consume CPU time  */
    }

    u32TimerCnt = TIMER_GetCounter(TIMER0);

    /* TIMER0->CNT is the elapsed us */
    printf("\nTime elapsed without program bank1: %d.%d seconds. Ticks: %d\n\n", u32TimerCnt / 1000000, u32TimerCnt / 1000, g_tick_cnt);

    db_addr   = FMC_APROM_BANK1_BASE;   /* Dual bank background program address */
    db_length = DB_PROG_LEN;            /* Dual bank background length          */
    db_state  = DB_STATE_START;         /* Start background dual bank program   */
    s_u32CRC  = 0;

    SysTick_Config(1000);
    StartTimer0();

    for (u32LoopCnt = 0; u32LoopCnt < CRC32_LOOP_CNT; u32LoopCnt++)
    {
        s_u32CRC += func_crc32(FMC_APROM_BASE, 0x10000);      /* Calculate 64KB CRC32 value, just to consume CPU time  */
    }

    u32TimerCnt = TIMER_GetCounter(TIMER0);

    /* TIMER0->CNT is the elapsed us */
    printf("\nTime elapsed with program bank1: %d.%d seconds. Ticks: %d\n\n", u32TimerCnt / 1000000, u32TimerCnt / 1000, g_tick_cnt);

    while (db_state != DB_STATE_DONE) ;  /* Wait until the dual bank program is done */

#if  (NVT_DCACHE_ON == 1)
    // Invalidate the D-Cache for the programmed region to ensure data consistency when D-Cache is enabled
    SCB_InvalidateDCache_by_Addr((void *)FMC_APROM_BANK1_BASE, DB_PROG_LEN);
#endif

    /*
     *  Verify ...
     */
    for (u32Addr = FMC_APROM_BANK1_BASE; u32Addr < FMC_APROM_BANK1_BASE + DB_PROG_LEN; u32Addr += 4)
    {
        if (inpw(u32Addr) != u32Addr)
        {
            printf("Flash address 0x%x verify failed ! expect: 0x%x, read: 0x%x.\n", u32Addr, u32Addr, inpw(u32Addr));
            goto lexit;
        }
    }

    printf("Verify OK.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function  */
    SYS_LockReg();                     /* Lock protected registers  */
    printf("\nEnd of FMC Dual Bank Sample Code\n");

    /* Got no where to go, just u32LoopCnt forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
