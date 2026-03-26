/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to make an application booting from APROM
 *          with a sub-routine resided on SPI Flash.
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
#define FLH_SECTOR_SIZE             0x1000
#define FLH_TRIM_ADDR               (0x400000 - FLH_SECTOR_SIZE)

//------------------------------------------------------------------------------
#define USE_4_BYTES_MODE            0   /* W25Q20 does not support 4-bytes address mode. */

static SPIM_PHASE_T sWb0BhRdCMD =
{
    /* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
    CMD_DMA_FAST_READ,                                                        // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                      // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                     // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE, // Data Phase
    8,                                                                        // Dummy Cycle Phase
    PHASE_DISABLE_CONT_READ, 0, 0, 0,
};

static SPIM_PHASE_T sWb02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                                    //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    0,
    PHASE_DISABLE_CONT_READ, 0, 0, 0,
};

//------------------------------------------------------------------------------
void spim_routine(void);
void SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbWrCMD, SPIM_PHASE_T *psWbRdCMD);

//------------------------------------------------------------------------------
void SPIFlash_Init(SPIM_T *spim)
{
    uint8_t idBuf[3] = {0};

    /* Set SPIM clock as HCLK divided by 1 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, SPIM_PORT_DIV);

    /* Initialized SPI flash */
    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != SPIM_OK)
    {
        printf("SPIM flash initialize failed!\n");

        while (1) {}
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM_DMADMM_InitPhase(SPIM_PORT, &sWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &sWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    SPIM_TrimRxClkDlyNum(SPIM_PORT, &sWb02hWrCMD, &sWb0BhRdCMD);

    SPIM_DMADMM_InitPhase(SPIM_PORT, &sWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim,
                            (sWb0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                            sWb0BhRdCMD.u32CMDCode,
                            1);
}

void SPIM_SetDMMAddrNonCacheable(void)
{
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(SPIM_PORT);

    /* Disable D-Cache */
    SCB_DisableDCache();

    /* Configure MPU memory attribute */
    /*
     * Attribute 0
     * Memory Type = Normal
     * Attribute   = Outer Non-cacheable, Inner Non-cacheable
     */
    ARM_MPU_SetMemAttr(0UL, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    /* Configure MPU memory regions */
    ARM_MPU_SetRegion(0UL,                                                          /* Region 0 */
                      ARM_MPU_RBAR((uint32_t)u32DMMAddr, ARM_MPU_SH_NON, 0, 0, 0),  /* Non-shareable, read/write, privileged, non-executable */
                      ARM_MPU_RLAR((uint32_t)u32DMMAddr + 0x10000, 0)               /* Use Attr 0 */
                     );
    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable D-Cache */
    SCB_EnableDCache();
}

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

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_RESETN_PJ2();
    SET_SPIM0_SS_PJ7();

    PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);
    PJ->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk |
                  GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);

    GPIO_SetSlewCtl(PJ, BIT2, u32SlewRate);
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
    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMA_Read(spim, u32SrcAddr, (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32),
                  sizeof(au64VerifyBuf), pu8VerifyBuf, psWbRdCMD->u32CMDCode, SPIM_OP_ENABLE);

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

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O    */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /**
     * GCC project users must use the ICP tool to burn binary to APROM and
     * SPI flash separately, and after entering the debugger, only APROM code can source debug.
    */
    printf("+--------------------------------------------------+\n");
    printf("|      SPIM DMM mode running program on flash      |\n");
    printf("+--------------------------------------------------+\n");

    SPIFlash_Init(SPIM_PORT);

    while (1)
    {
        printf("\n\nProgram is currently running on APROM flash.\n");
        printf("Press any key to branch to sub-routine on SPIM flash...\n");

        getchar();

        spim_routine();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
