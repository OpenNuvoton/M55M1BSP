/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show SPIM DMA mode read/write octal flash function.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPIM_PORT                   SPIM0
#define SPIM_PORT_DIV               (1)
#define TRIM_PAT_SIZE               128

#define FLASH_BLOCK_SIZE            (8 * 1024)     /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000        /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048

#define OCFLH_SECTOR_SIZE           0x1000
#define OCFLH_TRIM_ADDR             (0x02000000 - OCFLH_SECTOR_SIZE) //(0x800000)

//------------------------------------------------------------------------------
__attribute__((aligned(64))) static uint8_t g_buff[BUFFER_SIZE] = {0};

//------------------------------------------------------------------------------
/* Program SDR Command Phase */
extern SPIM_PHASE_T gsMt02hWrCMD;
extern SPIM_PHASE_T gsMtC2hWrCMD;
extern SPIM_PHASE_T gsMt8EhWrCMD;

/* Octal SDR Read Command Phase */
extern SPIM_PHASE_T gsMt0BhRdCMD;
extern SPIM_PHASE_T gsMtCBhRdCMD;
extern SPIM_PHASE_T gsMtCChRdCMD;
extern SPIM_PHASE_T gsMt9DhRdCMD;
extern SPIM_PHASE_T gsMtFDhRdCMD;

/* Program DDR Command Phase */
extern SPIM_PHASE_T gsMt02hWrDDRCMD;
/* Octal DDR Read Command Phase */
extern SPIM_PHASE_T gsMt8BhRdDDRCMD;

//------------------------------------------------------------------------------
void SYS_Init(void)
{
    /*
         Set I/O slew rate to FAST1 (100 MHz).
         Use FAST1 if targeting 1.8V devices for better timing margin.
         Adjust if signal issues or EMI are observed.
     */
    uint32_t u32SlewRate = GPIO_SLEWCTL_FAST1;

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /*
      Enable PLL0 clock.
      200 MHz is recommended for stable Octal SPI Flash operation.
      Users may manually change to 220 MHz for higher performance,
      but stability is not guaranteed at that frequency.
    */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_200MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLKN_PH12();
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_D4_PH14();
    SET_SPIM0_D5_PH15();
    SET_SPIM0_D6_PG13();
    SET_SPIM0_D7_PG14();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_RESETN_PJ2();
    SET_SPIM0_RWDS_PG15();
    SET_SPIM0_SS_PJ7();

    PG->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk |
                  GPIO_SMTEN_SMTEN14_Msk |
                  GPIO_SMTEN_SMTEN15_Msk);
    PH->SMTEN |= (GPIO_SMTEN_SMTEN12_Msk |
                  GPIO_SMTEN_SMTEN13_Msk |
                  GPIO_SMTEN_SMTEN14_Msk |
                  GPIO_SMTEN_SMTEN15_Msk);
    PJ->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk |
                  GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PG, BIT13, u32SlewRate);
    GPIO_SetSlewCtl(PG, BIT14, u32SlewRate);
    GPIO_SetSlewCtl(PG, BIT15, u32SlewRate);

    GPIO_SetSlewCtl(PH, BIT12, u32SlewRate);
    GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);
    GPIO_SetSlewCtl(PH, BIT14, u32SlewRate);
    GPIO_SetSlewCtl(PH, BIT15, u32SlewRate);

    GPIO_SetSlewCtl(PJ, BIT2, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT3, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT4, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT5, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT6, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT7, u32SlewRate);
}

/**
 * @brief Selects the most stable delay value from a pass mask by finding the optimal position
 *        within the longest consecutive valid delay window.
 *
 * @param u32PassMask A 32-bit mask where each bit represents whether a delay value passed
 *                    timing validation. A '1' indicates a valid/passing delay, '0' indicates failure.
 *
 * @return The index of the selected stable delay value within the valid window.
 *         - Returns 0 if no valid delay window is found (all bits are 0).
 *         - For wide windows (size > 2): Returns the center position, offset inward by 1 step
 *           from each boundary to maintain safety margin.
 *         - For narrow windows (size <= 2): Returns a position biased toward the higher index
 *           to prioritize hold time stability.
 *
 * @note This function assumes SPIM_MAX_RX_DLY_NUM is defined and represents the maximum
 *       number of possible RX delay positions to scan.
 */
static uint8_t selectStableDelay(uint32_t u32PassMask)
{
    uint8_t u8MaxWindow = 0, u8CurrentWindow = 0, u8StartIdx = 0;

    /* Scan for the longest consecutive '1's sequence */
    for (uint8_t i = 0; i < SPIM_MAX_RX_DLY_NUM; i++)
    {
        if (u32PassMask & (1 << i))
        {
            if (++u8CurrentWindow > u8MaxWindow)
            {
                u8MaxWindow = u8CurrentWindow;
                u8StartIdx = i - u8CurrentWindow + 1;
            }
        }
        else
        {
            u8CurrentWindow = 0;
        }
    }

    if (u8MaxWindow == 0) return 0;

    /* If window is narrow (like your 0x18), the center is the only safe bet.
       If window is wide, we shrink boundaries by 1 step to stay safe. */
    if (u8MaxWindow > 2)
    {
        return (u8StartIdx + 1) + ((u8MaxWindow - 2) / 2);
    }

    /* For narrow windows (size 2), return the higher index to favor hold time */
    return u8StartIdx + (u8MaxWindow / 2);
}

/**
 * @brief Trim RX clock delay to find optimal sampling point for SPI flash reads
 *
 * @details This function calibrates the RX clock delay by:
 *          1. Using the last 4KB sector of flash to store a known trim pattern
 *          2. Testing all possible RX delay values in both DMA and DMM modes
 *          3. Selecting the most stable delay value that works in both modes
 *
 * @param[in] spim        Pointer to SPIM module
 * @param[in] psWbWrCMD   Write command phase configuration
 * @param[in] psWbRdCMD   Read command phase configuration
 *
 * @note The last 4KB sector (calculated by SPIM_GetLastSectorAddr) is used to avoid
 *       interfering with application code storage. This sector is safe to use for
 *       temporary trim pattern storage.
 */
void SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psMTWrCMD, SPIM_PHASE_T *psMTRdCMD)
{
    uint32_t u32SrcAddr = OCFLH_TRIM_ADDR;
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(spim);
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint32_t u32DMAMask = 0;
    uint32_t u32DMMMask = 0;
    uint32_t u32LoopAddr = 0;

    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[(TRIM_PAT_SIZE / 8)] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerifyBuf = (uint8_t *)au64VerifyBuf;
    uint8_t u8Pass = 0;
    uint8_t u8DlyNum = 0;

    /* 1. Generate Pattern & Ensure Flash Content (Safe Clock) */
    for (uint32_t k = 0; k < sizeof(au64TrimPattern); k++)
    {
        uint32_t val = (k & 0x0F) ^ (k >> 4) ^ (k >> 3);

        if (k & 0x01) val = ~val;

        pu8TrimPattern[k] = ~(uint8_t)(val ^ (k << 3) ^ (k >> 2));
    }

    SPIM_SET_CLOCK_DIVIDER(spim, 16);

    SPIM_DMADMM_InitPhase(spim, &gsMt0BhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMA_Read(spim, u32SrcAddr, (gsMt0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32),
                  sizeof(au64VerifyBuf), pu8VerifyBuf, gsMt0BhRdCMD.u32CMDCode, SPIM_OP_ENABLE);

    if (memcmp(pu8TrimPattern, pu8VerifyBuf, sizeof(au64VerifyBuf)) != 0)
    {
        SPIM_EraseBlock(spim, u32SrcAddr, (psMTWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                        OPCODE_SE_4K, SPIM_PhaseModeToNBit(psMTWrCMD->u32CMDPhase), SPIM_OP_ENABLE);
        SPIM_DMA_Write(spim, u32SrcAddr, (psMTWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                       sizeof(au64TrimPattern), pu8TrimPattern, psMTWrCMD->u32CMDCode);
    }

    /* Restore High-Speed Clock */
    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

    /* --- STAGE 1: EVALUATE DMA MODE --- */
    SPIM_DMADMM_InitPhase(spim, psMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    for (u8DlyNum = 0; u8DlyNum < SPIM_MAX_RX_DLY_NUM; u8DlyNum++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8DlyNum);
        u8Pass = 1;

        for (uint32_t loop = 0; loop < 4; loop++)   // Increase stress
        {
            memset(pu8VerifyBuf, 0, TRIM_PAT_SIZE);
#if (NVT_DCACHE_ON == 1)
            SCB_InvalidateDCache_by_Addr((uint32_t *)pu8VerifyBuf, TRIM_PAT_SIZE);
#endif
            SPIM_DMA_Read(spim, u32SrcAddr, (psMTRdCMD->u32AddrWidth == PHASE_WIDTH_32),
                          TRIM_PAT_SIZE, pu8VerifyBuf, psMTRdCMD->u32CMDCode, SPIM_OP_ENABLE);

            if (memcmp(pu8TrimPattern, pu8VerifyBuf, TRIM_PAT_SIZE) != 0)
            {
                u8Pass = 0;
                break;
            }
        }

        if (u8Pass) u32DMAMask |= (1 << u8DlyNum);
    }

    /* --- STAGE 2: EVALUATE DMM MODE --- */
    SPIM_DMADMM_InitPhase(spim, psMTRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim, (psMTRdCMD->u32AddrWidth == PHASE_WIDTH_32), psMTRdCMD->u32CMDCode, 1);

    for (u8DlyNum = 0; u8DlyNum < SPIM_MAX_RX_DLY_NUM; u8DlyNum++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8DlyNum);
        u8Pass = 1;

        for (uint32_t loop = 0; loop < 4; loop++)
        {
#if (NVT_DCACHE_ON == 1)
            SCB_InvalidateDCache_by_Addr((uint32_t *)(u32DMMAddr + u32SrcAddr), TRIM_PAT_SIZE);
#endif

            for (u32LoopAddr = 0; u32LoopAddr < TRIM_PAT_SIZE; u32LoopAddr += 8)
            {
                /* Read 8 bytes of data from the device */
                if (*(volatile uint64_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr) != *(uint64_t *)&pu8TrimPattern[u32LoopAddr])
                {
                    u8Pass = 0;
                    break;
                }
            }

            if (u8Pass == 0) break;
        }

        if (u8Pass) u32DMMMask |= (1 << u8DlyNum);
    }

    /* --- STAGE 3: FINAL SELECTION --- */
    uint32_t u32FinalMask = u32DMAMask & u32DMMMask;
    uint8_t u8FinalDelay = selectStableDelay(u32FinalMask);

    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8FinalDelay);

    //printf("\n[SPIM TRIM REPORT]\nDMA Pass Mask: 0x%08X\nDMM Pass Mask: 0x%08X\n", u32DMAMask, u32DMMMask);
    printf("Final Selected Optimal Delay: %d\n", u8FinalDelay);
}

/**
 * @brief Trim DLL (Delay Locked Loop) delay for Octal DDR mode flash reads
 *
 * @details This function calibrates the DLL delay for Octal DDR mode by:
 *          1. Entering OPI (Octal Peripheral Interface) mode
 *          2. Using the last 4KB sector to store a known trim pattern
 *          3. Testing all possible DLL delay values in both DMA and DMM modes
 *          4. Selecting the most stable delay value that works in both modes
 *          5. Exiting OPI mode after calibration
 *
 * @param[in] spim        Pointer to SPIM module
 * @param[in] psMTWrCMD   Octal mode write command phase configuration
 * @param[in] psMTRdCMD   Octal mode read command phase configuration
 *
 * @note This function is specifically for Octal DDR mode operation.
 *       It requires Octal mode command configurations (8-bit data phase).
 *       The function temporarily enters OPI mode, performs calibration,
 *       then exits OPI mode to return to normal operation.
 */
void SPIM_TrimDLLDelayNum(SPIM_T *spim, SPIM_PHASE_T *psMTWrCMD, SPIM_PHASE_T *psMTRdCMD)
{
    uint32_t u32SrcAddr = OCFLH_TRIM_ADDR;
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(spim);
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint32_t u32DMAMask = 0;              /* Bitmask for DMA mode passing delays */
    uint32_t u32DMMMask = 0;              /* Bitmask for DMM mode passing delays */
    uint32_t u32LoopAddr = 0;

    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[(TRIM_PAT_SIZE / 8)] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerifyBuf = (uint8_t *)au64VerifyBuf;
    uint8_t u8Pass = 0;
    uint8_t u8DlyNum = 0;

    /* 1. Generate trim pattern with alternating bit pattern for robustness */
    for (uint32_t k = 0; k < sizeof(au64TrimPattern); k++)
    {
        uint32_t val = (k & 0x0F) ^ (k >> 4) ^ (k >> 3);

        if (k & 0x01) val = ~val;

        pu8TrimPattern[k] = ~(uint8_t)(val ^ (k << 3) ^ (k >> 2));
    }

    /* 2. Use safe clock divider to ensure flash access stability during pattern programming */
    SPIM_SET_CLOCK_DIVIDER(spim, 16);

    /* 3. Enter Octal Peripheral Interface mode for DDR operation */
    SPIM_EnterOPIMode_MICRON(spim);

    /* 4. Read existing flash content to verify if trim pattern is already present */
    SPIM_DMADMM_InitPhase(spim, psMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMA_Read(spim, u32SrcAddr, (psMTRdCMD->u32AddrWidth == PHASE_WIDTH_32),
                  sizeof(au64VerifyBuf), pu8VerifyBuf, psMTRdCMD->u32CMDCode, SPIM_OP_ENABLE);

    /* 5. If flash content doesn't match trim pattern, erase and reprogram it */
    if (memcmp(pu8TrimPattern, pu8VerifyBuf, sizeof(au64VerifyBuf)) != 0)
    {
        SPIM_EraseBlock(spim, u32SrcAddr, (psMTWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                        OPCODE_SE_4K, SPIM_PhaseModeToNBit(psMTWrCMD->u32CMDPhase), SPIM_OP_ENABLE);
        SPIM_DMA_Write(spim, u32SrcAddr, (psMTWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                       sizeof(au64TrimPattern), pu8TrimPattern, psMTWrCMD->u32CMDCode);
    }

    /* 6. Restore original high-speed clock divider */
    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

    /* --- STAGE 1: EVALUATE DMA MODE --- */
    /* Test all DLL delay values in DMA page read mode */
    SPIM_DMADMM_InitPhase(spim, psMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    for (u8DlyNum = 0; u8DlyNum < SPIM_MAX_DLL_LATENCY; u8DlyNum++)
    {
        /* Set current DLL delay value */
        SPIM_SetDLLDelayNum(spim, u8DlyNum);
        u8Pass = 1;

        /* Perform multiple stress test iterations */
        for (uint32_t loop = 0; loop < 4; loop++)
        {
            memset(pu8VerifyBuf, 0, TRIM_PAT_SIZE);
#if (NVT_DCACHE_ON == 1)
            /* Invalidate cache to ensure fresh data from flash */
            SCB_InvalidateDCache_by_Addr((uint32_t *)pu8VerifyBuf, TRIM_PAT_SIZE);
#endif
            /* Read trim pattern from flash using DMA */
            SPIM_DMA_Read(spim, u32SrcAddr, (psMTRdCMD->u32AddrWidth == PHASE_WIDTH_32),
                          TRIM_PAT_SIZE, pu8VerifyBuf, psMTRdCMD->u32CMDCode, SPIM_OP_ENABLE);

            /* Verify read data matches expected pattern */
            if (memcmp(pu8TrimPattern, pu8VerifyBuf, TRIM_PAT_SIZE) != 0)
            {
                u8Pass = 0;
                break;
            }
        }

        /* Mark this delay value as passing in DMA mode */
        if (u8Pass) u32DMAMask |= (1 << u8DlyNum);
    }

    /* --- STAGE 2: EVALUATE DMM MODE --- */
    /* Test all DLL delay values in DMM (Direct Memory Mapped) mode */
    SPIM_DMADMM_InitPhase(spim, psMTRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim, (psMTRdCMD->u32AddrWidth == PHASE_WIDTH_32), psMTRdCMD->u32CMDCode, 1);

    for (u8DlyNum = 0; u8DlyNum < SPIM_MAX_DLL_LATENCY; u8DlyNum++)
    {
        /* Set current DLL delay value */
        SPIM_SetDLLDelayNum(spim, u8DlyNum);
        u8Pass = 1;

        /* Perform multiple stress test iterations */
        for (uint32_t loop = 0; loop < 4; loop++)
        {
#if (NVT_DCACHE_ON == 1)
            /* Invalidate cache for DMM address space */
            SCB_InvalidateDCache_by_Addr((uint32_t *)(u32DMMAddr + u32SrcAddr), TRIM_PAT_SIZE);
#endif

            /* Read and verify trim pattern via DMM address space (8 bytes at a time) */
            for (u32LoopAddr = 0; u32LoopAddr < TRIM_PAT_SIZE; u32LoopAddr += 8)
            {
                /* Read 8 bytes of data from the device via DMM */
                if (*(volatile uint64_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr) != *(uint64_t *)&pu8TrimPattern[u32LoopAddr])
                {
                    u8Pass = 0;
                    break;
                }
            }

            if (u8Pass == 0) break;
        }

        /* Mark this delay value as passing in DMM mode */
        if (u8Pass) u32DMMMask |= (1 << u8DlyNum);
    }

    /* 7. Exit OPI mode if Octal mode command is detected */
    if (psMTRdCMD->u32CMDPhase == PHASE_OCTAL_MODE)
    {
        SPIM_ExitOPIMode_MICRON(spim);
    }

    /* --- STAGE 3: FINAL SELECTION --- */
    /* Find delay values that pass both DMA and DMM tests */
    uint32_t u32FinalMask = u32DMAMask & u32DMMMask;

    /* Select the most stable delay from valid window */
    uint8_t u8FinalDelay = selectStableDelay(u32FinalMask);

    /* 8. Apply final selected DLL delay value */
    SPIM_SetDLLDelayNum(spim, u8FinalDelay);

    /* Output trimming results */
    //printf("\n[SPIM TRIM REPORT]\nDMA Pass Mask: 0x%08X\nDMM Pass Mask: 0x%08X\n", u32DMAMask, u32DMMMask);
    printf("Final Selected Optimal Delay: %d\n", u8FinalDelay);
}

int dma_read_write(uint32_t u32Is4ByteAddr, uint32_t u32RdCmd, uint32_t WrCmd, uint32_t u32DDREn)
{
    uint32_t i = 0, offset = 0;             /* variables */
    uint32_t *pData = NULL;
    uint32_t u32EraseNBit = 1;

    /* Octal Flash in Octal DDR mode, erase command uses 8 data lines */
    if (u32DDREn == SPIM_OP_ENABLE)
    {
        u32EraseNBit = 8;
    }

    /*
     *  Erase flash page
     */
    printf("Erase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, u32Is4ByteAddr, OPCODE_BE_64K, u32EraseNBit, SPIM_OP_ENABLE);
    printf("done.\n");

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);

        SPIM_IO_ReadByPhase(SPIM_PORT, &gsMt0BhRdCMD, TEST_BLOCK_ADDR + offset, g_buff, BUFFER_SIZE);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != 0xFFFFFFFF)
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x!\n", TEST_BLOCK_ADDR + i, *pData);
                return -1;
            }
        }
    }

    printf("done.\n");

    /*
     *  Program data to flash block
     */
    printf("Program sequential data to flash block 0x%x...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);
        }

#if (NVT_DCACHE_ON == 1)
        SCB_CleanDCache_by_Addr((volatile uint8_t *)&g_buff, (uint32_t)sizeof(g_buff));
#endif

        SPIM_DMA_Write(SPIM_PORT, TEST_BLOCK_ADDR + offset, u32Is4ByteAddr, BUFFER_SIZE, g_buff, WrCmd);
    }

    printf("done.\n");

    /*
     *  Verify flash block data
     */
    printf("Verify SPI flash block 0x%x data...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        SCB_InvalidateDCache_by_Addr((volatile uint8_t *)&g_buff, (uint32_t)sizeof(g_buff));
#endif

        SPIM_DMA_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, u32Is4ByteAddr, BUFFER_SIZE, g_buff, u32RdCmd, SPIM_OP_ENABLE);

        pData = (uint32_t *)&g_buff[0];

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if ((*pData) != (((i << 16) | (TEST_BLOCK_ADDR + offset + i)) & 0xFFFFFFFF))
            {
                uint32_t u32CmpData = ((i << 16) | (TEST_BLOCK_ADDR + offset + i));

                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i,
                       *pData,
                       u32CmpData);
                return -1;
            }
        }
    }

    printf("done.\n");

    return 0;
}

int main()
{
    uint8_t idBuf[3];
    uint32_t u32Is4ByteAddr = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+------------------------------------------------------------------------+\n");
    printf("|      SPIM DMA mode read/write octal flash(Micron MT35XU02G) sample     |\n");
    printf("+------------------------------------------------------------------------+\n");

    /**
     * @brief Set SPIM clock as HCLK divided by 1
     * @note  Octal flash in DDR mode only support 1 or 2.
     */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, SPIM_PORT_DIV);

    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != SPIM_OK)          /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           idBuf[0], idBuf[1], idBuf[2]);

    printf("\n[Fast IO Read] 3-bytes address mode, Fast Read Octal SDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtCBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtC2hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsMtC2hWrCMD, &gsMtCBhRdCMD);

    u32Is4ByteAddr = (gsMtCBhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsMtCBhRdCMD.u32CMDCode, gsMtC2hWrCMD.u32CMDCode, SPIM_OP_DISABLE) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast IO Read] 4-bytes address mode, Fast Read Octal SDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtCChRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8EhWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsMt8EhWrCMD, &gsMtCChRdCMD);

    u32Is4ByteAddr = (gsMtCChRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsMtCChRdCMD.u32CMDCode, gsMt8EhWrCMD.u32CMDCode, SPIM_OP_DISABLE) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Output DDR Mode] 4-bytes address mode, Fast Read Octal DDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8BhRdDDRCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt02hWrDDRCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimDLLDelayNum(SPIM_PORT, &gsMt02hWrDDRCMD, &gsMt8BhRdDDRCMD);

    u32Is4ByteAddr = (gsMt8BhRdDDRCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    SPIM_EnterOPIMode_MICRON(SPIM_PORT); /* Enable Octal Flash DDR Mode */

    if (dma_read_write(u32Is4ByteAddr, gsMt8BhRdDDRCMD.u32CMDCode, gsMt02hWrDDRCMD.u32CMDCode, SPIM_OP_ENABLE) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    SPIM_ExitOPIMode_MICRON(SPIM_PORT);  /* Disable Octal Flash DDR Mode */

    printf("[OK].\n");

    printf("\nOctal SPI Flash read/write demo done.\n");
lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
