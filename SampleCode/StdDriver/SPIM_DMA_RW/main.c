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
#define TRIM_PAT_SIZE               32

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
    uint32_t u32SlewRate = GPIO_SLEWCTL_FAST0;

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
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
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
}

/**
 * @brief Check if the given array of values is consecutive.
 *
 * @param psDlyNumRange Pointer to the structure to store the range of consecutive values.
 * @param au8Src Array of values to be checked.
 * @param size Size of the array.
 */
static uint8_t isConsecutive(uint8_t au8Src[], uint32_t size)
{
    uint8_t u8Find = 0, u8StartIdx = 0, u8MaxRang = 0;
    uint32_t u32i = 0, u32j = 1;

    // Check if the sequence is increasing or decreasing
    bool increasing = au8Src[1] > au8Src[0];

    // Iterate over the array
    for (u32i = 1; u32i < size; ++u32i)
    {
        // Check if the current element is consecutive to the previous one
        if ((increasing && au8Src[u32i] != au8Src[u32i - 1] + 1) ||
                (!increasing && au8Src[u32i] != au8Src[u32i - 1] - 1))
        {
            // Update the start and end indices of the consecutive range
            u8Find = u32i;
            u32j = 0;
        }

        // Increment the number of consecutive elements
        u32j++;

        // Update the range if the current range is longer than the previous one
        if (u32j >= u8MaxRang)
        {
            u8StartIdx = u8Find;
            u8MaxRang = u32j;
        }
    }

    return (u8MaxRang > 2) ?
           au8Src[((u8StartIdx + u8MaxRang / 2) + (((u8MaxRang % 2) != 0) ? 1 : 0)) - 1] :
           au8Src[u8StartIdx];
}

/**
 * @brief Trim DLL component delay number
 *
 * @details This function is used to trim the delay number of DLL component,
 *          it can improve the SPIM clock performance.
 *
 * @param[in] spim The pointer of the specified SPIM module
 *
 */
void SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbWrCMD, SPIM_PHASE_T *psWbRdCMD)
{
    uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayRes[SPIM_MAX_DLL_LATENCY] = {0};
    uint32_t u32PatternSize = TRIM_PAT_SIZE;
    uint32_t u32ReTrimMaxCnt = 6;
    uint32_t u32LoopAddr = 0;
    uint32_t u32Val = 0;
    uint32_t u32i = 0;
    uint32_t u32j = 0;
    uint32_t u32k = 0;
    uint32_t u32ReTrimCnt = 0;
    uint32_t u32SrcAddr = 0;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim); // Divider value
    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[(TRIM_PAT_SIZE / 8)] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerifyBuf = (uint8_t *)au64VerifyBuf;
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(spim);

    /* Create Trim Pattern */
    for (u32k = 0; u32k < sizeof(au64TrimPattern); u32k++)
    {
        u32Val = (u32k & 0x0F) ^ (u32k >> 4) ^ (u32k >> 3);

        if (u32k & 0x01)
        {
            u32Val = ~u32Val;
        }

        pu8TrimPattern[u32k] = ~(uint8_t)(u32Val ^ (u32k << 3) ^ (u32k >> 2));
    }

    /* Set SPIM clock divider to 8 */
    SPIM_SET_CLOCK_DIVIDER(spim, 8);

    /* Erase 64KB block */
    SPIM_EraseBlock(spim,
                    u32SrcAddr,
                    psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_BE_64K,
                    SPIM_PhaseModeToNBit(psWbWrCMD->u32CMDPhase),
                    SPIM_OP_ENABLE);

    /* Write trim pattern */
    SPIM_DMA_Write(spim,
                   u32SrcAddr,
                   psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                   sizeof(au64TrimPattern),
                   pu8TrimPattern,
                   psWbWrCMD->u32CMDCode);

    /* Restore clock divider */
    SPIM_SET_CLOCK_DIVIDER(spim, u32Div); // Restore clock divider

    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    for (u32ReTrimCnt = 0; u32ReTrimCnt < u32ReTrimMaxCnt; u32ReTrimCnt++)
    {
        for (u8RdDelay = 0; u8RdDelay < SPIM_MAX_RX_DLY_NUM; u8RdDelay++)
        {
            /* Set DLL calibration to select the valid delay step number */
            SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);

            memset(pu8VerifyBuf, 0, sizeof(au64VerifyBuf));

            /* Calculate the pattern size based on the trim count */
            u32PatternSize =
                (((u32ReTrimCnt == 2) || (u32ReTrimCnt >= 3)) && (u8RdDelay == 0)) ?
                (TRIM_PAT_SIZE - 0x08) :
                TRIM_PAT_SIZE;

            /* Read data from the HyperRAM */
            u32LoopAddr = 0;

            for (u32k = 0; u32k < u32PatternSize; u32k += 0x08)
            {
#if (NVT_DCACHE_ON == 1)
                // Invalidate the data cache for the trimmed data
                // Address is determined based on the re-trim count
                SCB_InvalidateDCache_by_Addr(
                    (volatile uint32_t *)((u32ReTrimCnt == 1) ? u32SrcAddr : (u32DMMAddr + u32SrcAddr)),
                    (int32_t)TRIM_PAT_SIZE * 2); // Size of the trimmed data is 256 bytes
#endif

                if (u32ReTrimCnt == 1)
                {
                    SPIM_DMA_Read(SPIM_PORT,
                                  (u32SrcAddr + u32LoopAddr),
                                  (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                  0x08,
                                  &pu8VerifyBuf[u32k],
                                  psWbRdCMD->u32CMDCode,
                                  SPIM_OP_ENABLE);
                }
                else
                {
                    SPIM_EnterDirectMapMode(spim,
                                            (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                            psWbRdCMD->u32CMDCode,
                                            1);

                    /* Read 8 bytes of data from the HyperRAM */
                    *(volatile uint64_t *)&pu8VerifyBuf[u32k] = *(volatile uint64_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                }

                if ((u32i = memcmp(&pu8TrimPattern[u32LoopAddr], &pu8VerifyBuf[u32k], 0x08)) != 0)
                {
                    break;
                }

                u32LoopAddr += (u32ReTrimCnt >= 3) ? 0x10 : 0x08;
            }

            u8RdDelayRes[u8RdDelay] += ((u32i == 0) ? 1 : 0);
        }
    }

    u32j = 0;

    for (u32i = 0; u32i < SPIM_MAX_RX_DLY_NUM; u32i++)
    {
        if (u8RdDelayRes[u32i] == u32ReTrimMaxCnt)
        {
            u8RdDelayRes[u32j++] = u32i;
        }
    }

    u8RdDelay = (u32j < 2) ? u8RdDelayRes[0] : isConsecutive(u8RdDelayRes, u32j);

    printf("RX Delay Num : %d\r\n", u8RdDelay);
    /* Set the number of intermediate delay steps */
    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);
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

    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != SPIM_OK)          /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

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
