/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show SPIM DMA mode read/write function.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPIM_PORT                   SPIM0
#define SPIM_PORT_DIV               1
#define TRIM_PAT_SIZE               128

//------------------------------------------------------------------------------
#define FLASH_BLOCK_SIZE            (8 * 1024)     /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             (0x10000)      /* Test block address on SPI flash. */
#define BUFFER_SIZE                 (2048)
#define VERIFY_PATTEN               (0xFFFFFFFF)

//------------------------------------------------------------------------------
#if (NVT_DCACHE_ON == 1)
// DCache-line aligned buffer for improved performance when DCache is enabled
uint8_t gau8buff[DCACHE_ALIGN_LINE_SIZE(BUFFER_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
#else
// Standard buffer alignment when DCache is disabled
uint8_t gau8buff[BUFFER_SIZE] __attribute__((aligned(32))) = {0};
#endif

//------------------------------------------------------------------------------
/* Program Command Phase */
extern SPIM_PHASE_T gsWb02hWrCMD;
extern SPIM_PHASE_T gsWb12hWrCMD;

/* Standard Read Command Phase */
extern SPIM_PHASE_T gsWb0BhRdCMD;

/* Dual Read Command Phase */
extern SPIM_PHASE_T gsWbBBhRdCMD;
extern SPIM_PHASE_T gsWbBChRdCMD;

/* Quad Read Command Phase */
extern SPIM_PHASE_T gsWbEBhRdCMD;
extern SPIM_PHASE_T gsWbEChRdCMD;

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

    /* Enable PLL0 clock */
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
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_SS_PJ7();

    PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

    PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);

    GPIO_SetSlewCtl(PJ, BIT3, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT4, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT5, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT6, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT7, u32SlewRate);
}

/**
 * @brief Get Flash Size and calculate the last address for Magic Number
 * @param[in] spim The pointer of the specified SPIM module
 * @param[out] u32FlashSize Total size of flash in bytes
 * @return uint32_t The start address of the last 4KB sector
 */
uint32_t SPIM_GetLastSectorAddr(SPIM_T *spim)
{
    uint8_t au8ID[3];
    uint32_t size = 0;

    /* 1. Read JEDEC ID (9Fh) */
    // Note: Assuming you have a function to read 3-byte ID
    SPIM_ReadJedecId(spim, au8ID, sizeof(au8ID), SPIM_BITMODE_1);

    /* 2. Decode Capacity ID (The 3rd byte) */
    /* Formula: 2 ^ ID_Value */
    if (au8ID[2] >= 0x10 && au8ID[2] <= 0x22) // Typical range for 64KB to 4GB
    {
        size = 1 << au8ID[2];
    }
    else
    {
        size = 0; // Unknown or error
        printf("Error: Unknown Flash Capacity ID 0x%X\n", au8ID[2]);
        return 0;
    }

    /* 3. Return the address of the last 4KB sector */
    /* We usually use the last sector to avoid interfering with code */
    return (size - 0x1000);
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
void SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbWrCMD, SPIM_PHASE_T *psWbRdCMD)
{
    uint32_t u32SrcAddr = SPIM_GetLastSectorAddr(spim);
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(spim);
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint32_t u32DMAMask = 0;
    uint32_t u32DMMMask = 0;
    uint32_t u32LoopAddr = 0;

    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] __attribute__((aligned(8))) = {0};
    uint64_t au64VerifyBuf[(TRIM_PAT_SIZE / 8)] __attribute__((aligned(8))) = {0};
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
    SPIM_DMADMM_InitPhase(spim, &gsWb0BhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMA_Read(spim, u32SrcAddr, (gsWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32),
                  sizeof(au64VerifyBuf), pu8VerifyBuf, gsWb0BhRdCMD.u32CMDCode, SPIM_OP_ENABLE);

    if (memcmp(pu8TrimPattern, pu8VerifyBuf, sizeof(au64VerifyBuf)) != 0)
    {
        SPIM_EraseBlock(spim, u32SrcAddr, (psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                        OPCODE_SE_4K, SPIM_PhaseModeToNBit(psWbWrCMD->u32CMDPhase), SPIM_OP_ENABLE);
        SPIM_DMA_Write(spim, u32SrcAddr, (psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32),
                       sizeof(au64TrimPattern), pu8TrimPattern, psWbWrCMD->u32CMDCode);
    }

    /* Restore High-Speed Clock */
    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

    /* --- STAGE 1: EVALUATE DMA MODE --- */
    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

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
            SPIM_DMA_Read(spim, u32SrcAddr, (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32),
                          TRIM_PAT_SIZE, pu8VerifyBuf, psWbRdCMD->u32CMDCode, SPIM_OP_ENABLE);

            if (memcmp(pu8TrimPattern, pu8VerifyBuf, TRIM_PAT_SIZE) != 0)
            {
                u8Pass = 0;
                break;
            }
        }

        if (u8Pass) u32DMAMask |= (1 << u8DlyNum);
    }

    /* --- STAGE 2: EVALUATE DMM MODE --- */
    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim, (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32), psWbRdCMD->u32CMDCode, 1);

    for (u8DlyNum = 0; u8DlyNum < SPIM_MAX_RX_DLY_NUM; u8DlyNum++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8DlyNum);
        u8Pass = 1;

        for (uint32_t loop = 0; loop < 4; loop++)
        {
#if (NVT_DCACHE_ON == 1)
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
        }

        if (u8Pass) u32DMMMask |= (1 << u8DlyNum);
    }

    /* --- STAGE 3: FINAL SELECTION --- */
    uint32_t u32FinalMask = u32DMAMask & u32DMMMask;
    uint8_t u8FinalDelay = selectStableDelay(u32FinalMask);

    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8FinalDelay);

    printf("Final Selected Optimal Delay: %d\n", u8FinalDelay);
}

/*
 *  Test DMA read/write SPI flash
 *
 *  @note    This test function will erase/write/read flash data from/to
 *           SPI flash block 0x10000, and then verify the data.
 */
int dma_read_write(int is4ByteAddr, uint32_t u32RdCmd, uint32_t WrCmd)
{
    uint32_t u32i;
    uint32_t u32Offset;
    uint32_t *pu32Data = (uint32_t *)gau8buff;

    /*
     * Test Description:
     *   Erase, read, write and verify a block of SPI flash.
     */
    printf("\n=== Test DMA read/write SPI flash ===\n");

    /* Erase flash block */
    printf("\tErase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, is4ByteAddr, OPCODE_BE_64K, SPIM_BITMODE_1, SPIM_OP_ENABLE);
    printf("done.\n");

    /* Verify flash block be erased */
    printf("\tVerify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (u32Offset = 0; u32Offset < FLASH_BLOCK_SIZE; u32Offset += BUFFER_SIZE)
    {
#if (NVT_DCACHE_ON == 1)
        // If the data cache is enabled, invalidate the cache for the DMA read buffer
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif
        SPIM_DMA_Read(SPIM_PORT, TEST_BLOCK_ADDR + u32Offset, is4ByteAddr, BUFFER_SIZE, gau8buff, u32RdCmd, SPIM_OP_ENABLE);

        pu32Data = (uint32_t *)gau8buff;

        for (u32i = 0; u32i < BUFFER_SIZE; u32i += 4, pu32Data++)
        {
            if (*pu32Data != VERIFY_PATTEN)
            {
                printf("FAILED!\n");
                return SPIM_ERR_FAIL;
            }
        }
    }

    printf("done.\n");

    /* Program data to flash block */
    printf("\tProgram sequential data to flash block 0x%x...", TEST_BLOCK_ADDR);

    for (u32Offset = 0; u32Offset < FLASH_BLOCK_SIZE; u32Offset += BUFFER_SIZE)
    {
        pu32Data = (uint32_t *)gau8buff;

        for (u32i = 0; u32i < BUFFER_SIZE; u32i += 4, pu32Data++)
            (*pu32Data) = (u32i << 16) | (TEST_BLOCK_ADDR + u32Offset + u32i);

#if (NVT_DCACHE_ON == 1)
        // Clean the data cache to ensure data consistency before writing to flash
        SCB_CleanDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        SPIM_DMA_Write(SPIM_PORT, TEST_BLOCK_ADDR + u32Offset, is4ByteAddr, BUFFER_SIZE, gau8buff, WrCmd);
    }

    printf("done.\n");

    /* Verify flash block data */
    printf("\tVerify SPI flash block 0x%x data...", TEST_BLOCK_ADDR);

    for (u32Offset = 0; u32Offset < FLASH_BLOCK_SIZE; u32Offset += BUFFER_SIZE)
    {
#if (NVT_DCACHE_ON == 1)
        // If the data cache is enabled, invalidate the cache for the DMA read buffer
        // to ensure the data read from flash is not from the cache
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        SPIM_DMA_Read(SPIM_PORT, TEST_BLOCK_ADDR + u32Offset, is4ByteAddr, BUFFER_SIZE, gau8buff, u32RdCmd, 1);

        pu32Data = (uint32_t *)gau8buff;

        for (u32i = 0; u32i < BUFFER_SIZE; u32i += 4, pu32Data++)
        {
            if (*pu32Data != ((u32i << 16) | (TEST_BLOCK_ADDR + u32Offset + u32i)))
            {
                printf("FAILED! %x\n", *pu32Data);
                return SPIM_ERR_FAIL;
            }
        }
    }

    printf("done.\n");

    return SPIM_OK;
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

    printf("+-------------------------------------------+\n");
    printf("|      SPIM DMA mode read/write sample      |\n");
    printf("+-------------------------------------------+\n");

    /* Set SPIM clock as HCLK divided by 1 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, SPIM_PORT_DIV);

    /* Initialized SPI flash */
    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != SPIM_OK)
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    /* Read JEDEC ID */
    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           idBuf[0], idBuf[1], idBuf[2]);

    printf("\n[Fast Read] 3-bytes address mode, Fast Read command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWb0BhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWb0BhRdCMD);

    u32Is4ByteAddr = (gsWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsWb0BhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Dual Output] 3-bytes address mode, Fast Read Dual command...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbBBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWbBBhRdCMD);

    u32Is4ByteAddr = (gsWbBBhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsWbBBhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Quad Output] 3-bytes address mode, Fast Read Quad command...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbEBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWbEBhRdCMD);

    u32Is4ByteAddr = (gsWbEBhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsWbEBhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWb12hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);
    printf("\n[Fast Read Dual I/O] 4-bytes address mode, dual read...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbBChRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb12hWrCMD, &gsWbBChRdCMD);

    u32Is4ByteAddr = (gsWbBChRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsWbBChRdCMD.u32CMDCode, gsWb12hWrCMD.u32CMDCode) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Quad I/O] 4-bytes address mode, quad read...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbEChRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb12hWrCMD, &gsWbEChRdCMD);

    u32Is4ByteAddr = (gsWbEChRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsWbEChRdCMD.u32CMDCode, gsWb12hWrCMD.u32CMDCode) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\nSPIM DMA read/write demo done.\n");

lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
