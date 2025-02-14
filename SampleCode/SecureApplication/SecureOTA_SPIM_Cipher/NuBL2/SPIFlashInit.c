#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "SPIFlashInit.h"

static SPIM_PHASE_T s_sWb0BhRdCMD =
{
    /* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
    CMD_DMA_FAST_READ,                                                          // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                        // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   // Data Phase
    8,                                                                          // Dummy Cycle Phase
};

static SPIM_PHASE_T s_sWb02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                                    // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  // Data Phase
    0,
};

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

    return (u8MaxRang >= 2) ? au8Src[((u8StartIdx + u8MaxRang / 2) + (((u8MaxRang % 2) != 0) ? 1 : 0))] : au8Src[u8StartIdx];
}

/**
 * @brief Trim DLL component delay number
 *
 * @details This function is used to trim the delay number of DLL component,
 *          it can improve the SPIM clock performance.
 *
 * @param[in] psSPIM The pointer of the specified SPIM module
 *
 */
void SPIM_TrimRxClkDlyNum(SPIM_T *psSPIM, SPIM_PHASE_T *psWbWrCMD, SPIM_PHASE_T *psWbRdCMD)
{
    if (psSPIM == NULL)
    {
        return;
    }

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
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(psSPIM); // Divider value
    uint8_t au8TrimPattern[TRIM_PAT_SIZE * 2] = {0};
    uint8_t au8VerifyBuf[TRIM_PAT_SIZE] = {0};
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(psSPIM);

    /* Create Trim Pattern */
    for (u32k = 0; u32k < sizeof(au8TrimPattern); u32k++)
    {
        u32Val = (u32k & 0x0F) ^ (u32k >> 4) ^ (u32k >> 3);

        if (u32k & 0x01)
        {
            u32Val = ~u32Val;
        }

        au8TrimPattern[u32k] = ~(uint8_t)(u32Val ^ (u32k << 3) ^ (u32k >> 2));
    }

    /* Set SPIM clock divider to 8 */
    SPIM_SET_CLOCK_DIVIDER(psSPIM, 8);

    /* Erase 64KB block */
    SPIM_EraseBlock(psSPIM,
                    u32SrcAddr,
                    psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_BE_64K,
                    SPIM_PhaseModeToNBit(psWbWrCMD->u32CMDPhase),
                    SPIM_OP_ENABLE);

    /* Write trim pattern */
    SPIM_DMA_Write(psSPIM,
                   u32SrcAddr,
                   psWbWrCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                   sizeof(au8TrimPattern),
                   au8TrimPattern,
                   psWbWrCMD->u32CMDCode);

    /* Restore clock divider */
    SPIM_SET_CLOCK_DIVIDER(psSPIM, u32Div); // Restore clock divider

    SPIM_DMADMM_InitPhase(psSPIM, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(psSPIM, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    for (u32ReTrimCnt = 0; u32ReTrimCnt < u32ReTrimMaxCnt; u32ReTrimCnt++)
    {
        for (u8RdDelay = 0; u8RdDelay < SPIM_MAX_RX_DLY_NUM; u8RdDelay++)
        {
            /* Set DLL calibration to select the valid delay step number */
            SPIM_SET_RXCLKDLY_RDDLYSEL(psSPIM, u8RdDelay);

            memset(au8VerifyBuf, 0, TRIM_PAT_SIZE);

#if (NVT_DCACHE_ON == 1)
            // Invalidate the data cache for the DMA read buffer
            SCB_InvalidateDCache_by_Addr(
                (volatile uint32_t *)((u32ReTrimCnt == 1) ? u32SrcAddr : (u32DMMAddr + u32SrcAddr)), // Determine address based on re-trim count
                (int32_t)TRIM_PAT_SIZE * 2); // Invalidate cache for the specified size
#endif

            /* Calculate the pattern size based on the trim count */
            u32PatternSize =
                (((u32ReTrimCnt == 2) || (u32ReTrimCnt >= 3)) && (u8RdDelay == 0)) ?
                (TRIM_PAT_SIZE - 0x08) :
                TRIM_PAT_SIZE;

            /* Read data from the HyperRAM */
            u32LoopAddr = 0;

            for (u32k = 0; u32k < u32PatternSize; u32k += 0x08)
            {
                if (u32ReTrimCnt == 1)
                {
                    SPIM_DMA_Read(SPIM_PORT,
                                  (u32SrcAddr + u32LoopAddr),
                                  (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                  0x08,
                                  &au8VerifyBuf[u32k],
                                  psWbRdCMD->u32CMDCode,
                                  SPIM_OP_ENABLE);
                }
                else
                {
                    SPIM_EnterDirectMapMode(psSPIM,
                                            (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                            psWbRdCMD->u32CMDCode,
                                            1);

                    /* Read 8 bytes of data from the HyperRAM */
                    *(volatile uint32_t *)&au8VerifyBuf[u32k] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                    *(volatile uint32_t *)&au8VerifyBuf[u32k + 4] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr + 4);
                }

                if ((u32i = memcmp(&au8TrimPattern[u32LoopAddr], &au8VerifyBuf[u32k], 0x08)) != 0)
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
    SPIM_SET_RXCLKDLY_RDDLYSEL(psSPIM, u8RdDelay);
}

int32_t SPIFlash_Init(SPIM_T *psSPIM)
{
    uint8_t idBuf[3] = {0};

    /* Set SPIM clock as HCLK divided by 1 */
    SPIM_SET_CLOCK_DIVIDER(psSPIM, SPIM_PORT_DIV);

    /* Disable SPIM Cipher */
    SPIM_DISABLE_CIPHER(psSPIM);

    /* Initialized SPI flash */
    if (SPIM_InitFlash(psSPIM, SPIM_OP_ENABLE) != SPIM_OK)
    {
        printf("SPIM flash initialize failed!\n");

        while (1) {}
    }

    SPIM_ReadJedecId(psSPIM, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM_DMADMM_InitPhase(psSPIM, &s_sWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);
    SPIM_DMADMM_InitPhase(psSPIM, &s_sWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

#ifdef SPI_FLASH_FIXED_RDDLY
    /* Set the number of intermediate delay steps */
    SPIM_SET_RXCLKDLY_RDDLYSEL(psSPIM, SPI_FLASH_FIXED_RDDLY);
#else
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &s_sWb02hWrCMD, &s_sWb0BhRdCMD);
#endif

    //SPIM_DMADMM_InitPhase(psSPIM, &s_sWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(psSPIM,
                            (s_sWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                            s_sWb0BhRdCMD.u32CMDCode,
                            1);
    return 0;
}

int32_t SPIFlash_SetReadMode(uint32_t u32ReadMode)
{
    SPIM_DMADMM_InitPhase(SPIM_PORT, &s_sWb0BhRdCMD, u32ReadMode);
    return 0;
}

int32_t SPIFlash_EraseBlock(SPIM_T *psSPIM, uint32_t u32Addr)
{
    SPIM_EraseBlock(psSPIM,
                    u32Addr,
                    s_sWb02hWrCMD.u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_BE_64K,
                    SPIM_PhaseModeToNBit(s_sWb02hWrCMD.u32CMDPhase),
                    SPIM_OP_ENABLE);

    return 0;
}

int32_t SPIFlash_ErasePage(SPIM_T *psSPIM, uint32_t u32Addr)
{
    SPIM_EraseBlock(psSPIM,
                    u32Addr,
                    s_sWb02hWrCMD.u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_SE_4K,
                    SPIM_PhaseModeToNBit(s_sWb02hWrCMD.u32CMDPhase),
                    SPIM_OP_ENABLE);
    return 0;
}

int32_t SPIFlash_WritePage(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize)
{
    uint32_t u32Is4ByteAddr = (s_sWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    uint32_t u32WrCmd = s_sWb02hWrCMD.u32CMDCode;

    SPIM_DMA_Write(SPIM_PORT, u32Offset, u32Is4ByteAddr, u32ByteSize, (uint8_t *)pu32Data, u32WrCmd);
    return 0;
}

int32_t SPIFlash_ReadPage(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize)
{
    uint32_t u32Is4ByteAddr = (s_sWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    uint32_t u32RdCmd = s_sWb0BhRdCMD.u32CMDCode;

    SPIM_DMA_Read(SPIM_PORT, u32Offset, u32Is4ByteAddr, u32ByteSize, (uint8_t *)pu32Data, u32RdCmd, SPIM_OP_ENABLE);
    return 0;
}

int32_t SPIFlash_Write(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize)
{
    SPIM_IO_WriteByPhase(SPIM_PORT, &s_sWb02hWrCMD, u32Offset, (uint8_t *)pu32Data, 4, SPIM_OP_ENABLE);
    return 0;
}

uint32_t SPIFlash_GetFlashSize(SPIM_T *psSPIM)
{
    return SPIM_DMM_SIZE;
}
