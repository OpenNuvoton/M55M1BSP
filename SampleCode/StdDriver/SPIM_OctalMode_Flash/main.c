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
#define TRIM_PAT_SIZE               32

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

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);
    CLK_EnableModuleClock(OTFC0_MODULE);

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

void OctalFlash_SetConfigRegDiv(SPIM_T *spim, uint32_t u32Restore)
{
    static uint32_t u32Div;
    static uint32_t u32RxClkDly;

    if (u32Restore == SPIM_OP_DISABLE)
    {
        u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
        u32RxClkDly = SPIM_GET_RXCLKDLY_RDDLYSEL(spim);

        SPIM_SET_CLOCK_DIVIDER(spim, 16);
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, 0);
    }
    else
    {
        SPIM_SET_CLOCK_DIVIDER(spim, u32Div);
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u32RxClkDly);
    }
}

void OctalFlash_EnterDDRMode(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {0xE7};
    SPIM_PHASE_T sWrNVCRegCMD =
    {
        OPCODE_WR_VCONFIG,                                                          //Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                        //Command Phase
        PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
        0,
    };

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* Enable 4-byte address mode */
    SPIM_Enable_4Bytes_Mode(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);
    SPIM_SET_4BYTE_ADDR(spim, SPIM_OP_ENABLE);

    /* Set non-volatile register enter octal DDR mode */
    SPIM_IO_WriteByPhase(spim, &sWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf), SPIM_OP_DISABLE);

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_ENABLE);

    SPIM_SET_DTR_MODE(spim, SPIM_OP_ENABLE);
}

void OctalFlash_ExitDDRMode(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {0xFF};
    SPIM_PHASE_T sWrNVCRegCMD =
    {
        OPCODE_WR_VCONFIG,                                                      //Command Code
        PHASE_OCTAL_MODE, PHASE_WIDTH_8, PHASE_ENABLE_DTR,                      //Command Phase
        PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_ENABLE_DTR,                     //Address Phase
        PHASE_OCTAL_MODE, PHASE_ORDER_MODE0, PHASE_ENABLE_DTR, SPIM_OP_ENABLE,  //Data Phase
        0,
    };

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* Set non-volatile register exit octal DDR mode */
    SPIM_IO_WriteByPhase(spim, &sWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf), SPIM_OP_DISABLE);

    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);

    /* Disable 4-byte Address mode */
    SPIM_Enable_4Bytes_Mode(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_ENABLE);
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

void SPIM_TrimDLLDelayNum(SPIM_T *spim, SPIM_PHASE_T *pMTWrCMD, SPIM_PHASE_T *pMTRdCMD)
{
    if (spim == NULL)
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
    uint32_t u32SrcAddr = OCFLH_TRIM_ADDR;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim); // Divider value
    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[TRIM_PAT_SIZE / 8] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerifyBuf = (uint8_t *)au64VerifyBuf;
    uint32_t u32DMMAddr = SPIM_GET_DMMADDR(spim);
    SPIM_PHASE_T sMTRdCMDTmp = {0};

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

    OctalFlash_EnterDDRMode(spim);

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_DISABLE);

    SPIM_EraseBlock(spim,
                    u32SrcAddr,
                    (pMTWrCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_SE_4K,
                    SPIM_PhaseModeToNBit(pMTWrCMD->u32CMDPhase),
                    SPIM_OP_ENABLE);

    SPIM_DMA_Write(spim,
                   u32SrcAddr,
                   ((pMTWrCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE),
                   sizeof(au64TrimPattern),
                   pu8TrimPattern,
                   pMTWrCMD->u32CMDCode);

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_ENABLE);

    memcpy((uint8_t *)&sMTRdCMDTmp, (uint8_t *)pMTRdCMD, sizeof(SPIM_PHASE_T));

    if ((pMTRdCMD->u32RDQS == SPIM_OP_ENABLE) && (pMTRdCMD->u32CMDPhase != PHASE_OCTAL_MODE))
    {
        pMTRdCMD->u32CMDPhase = PHASE_OCTAL_MODE;
        pMTRdCMD->u32CMDWidth = PHASE_WIDTH_16;
        pMTRdCMD->u32CMDDTR = PHASE_ENABLE_DTR;

        pMTRdCMD->u32AddrPhase = PHASE_OCTAL_MODE;
        pMTRdCMD->u32AddrWidth = PHASE_WIDTH_32;
        pMTRdCMD->u32AddrDTR = PHASE_ENABLE_DTR;

        pMTRdCMD->u32DataPhase = PHASE_OCTAL_MODE;
        pMTRdCMD->u32DataDTR = PHASE_ENABLE_DTR;
        pMTRdCMD->u32RDQS = SPIM_OP_ENABLE;
        pMTRdCMD->u32DcNum = 16;
    }

    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    for (u32ReTrimCnt = 0; u32ReTrimCnt < u32ReTrimMaxCnt; u32ReTrimCnt++)
    {
        u32LoopAddr = 0;

        for (u8RdDelay = 0; u8RdDelay < SPIM_MAX_DLL_LATENCY; u8RdDelay++)
        {
            /* Set DLL calibration to select the valid delay step number */
            SPIM_SetDLLDelayNum(spim, u8RdDelay);

            memset(pu8VerifyBuf, 0, TRIM_PAT_SIZE);

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
                // Invalidate the data cache for the source or DMM address
                SCB_InvalidateDCache_by_Addr(
                    (volatile uint32_t *)((u32ReTrimCnt == 1) ? u32SrcAddr : (u32DMMAddr + u32SrcAddr)),
                    (int32_t)TRIM_PAT_SIZE * 2);
#endif

                if (u32ReTrimCnt == 1)
                {
                    SPIM_DMA_Read(SPIM_PORT,
                                  (u32SrcAddr + u32LoopAddr),
                                  (pMTRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                  0x08,
                                  &pu8VerifyBuf[u32k],
                                  pMTRdCMD->u32CMDCode,
                                  SPIM_OP_ENABLE);
                }
                else
                {
                    SPIM_EnterDirectMapMode(spim,
                                            (pMTRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                            pMTRdCMD->u32CMDCode,
                                            1);

                    /* Read 8 bytes of data from the HyperRAM */
                    *(volatile uint32_t *)&pu8VerifyBuf[u32k] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                    *(volatile uint32_t *)&pu8VerifyBuf[u32k + 4] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr + 4);
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

    /* Set Octal Flash Write Enable*/
    memcpy((uint8_t *)pMTRdCMD, (uint8_t *)&sMTRdCMDTmp, sizeof(SPIM_PHASE_T));

    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    OctalFlash_ExitDDRMode(spim);

    u32j = 0;

    for (u32i = 0; u32i < SPIM_MAX_RX_DLY_NUM; u32i++)
    {
        if (u8RdDelayRes[u32i] == u32ReTrimMaxCnt)
        {
            u8RdDelayRes[u32j++] = u32i;
        }
    }

    u8RdDelay = (u32j < 2) ? u8RdDelayRes[0] : isConsecutive(u8RdDelayRes, u32j);

    printf("Set DLL Num : %d\r\n", u8RdDelay);
    /* Set the number of intermediate delay steps */
    SPIM_SetDLLDelayNum(spim, u8RdDelay);
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
void SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *pMTWrCMD, SPIM_PHASE_T *pMTRdCMD)
{
    if (spim == NULL)
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
    uint32_t u32SrcAddr = OCFLH_TRIM_ADDR;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim); // Divider value
    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[TRIM_PAT_SIZE / 8] = {0};
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

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_DISABLE);

    SPIM_EraseBlock(spim,
                    u32SrcAddr,
                    (pMTWrCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_SE_4K,
                    SPIM_PhaseModeToNBit(pMTWrCMD->u32CMDPhase),
                    SPIM_OP_ENABLE);

    SPIM_DMA_Write(spim,
                   u32SrcAddr,
                   ((pMTWrCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE),
                   sizeof(au64TrimPattern),
                   pu8TrimPattern,
                   pMTWrCMD->u32CMDCode);

    OctalFlash_SetConfigRegDiv(spim, SPIM_OP_ENABLE);

    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(spim, pMTRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    for (u32ReTrimCnt = 0; u32ReTrimCnt < u32ReTrimMaxCnt; u32ReTrimCnt++)
    {
        u32LoopAddr = 0;

        for (u8RdDelay = 0; u8RdDelay < SPIM_MAX_RX_DLY_NUM; u8RdDelay++)
        {
            /* Set DLL calibration to select the valid delay step number */
            SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);

            memset(pu8VerifyBuf, 0, TRIM_PAT_SIZE);

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
                // Invalidate the data cache for the source or DMM address based on the retrim count
                SCB_InvalidateDCache_by_Addr(
                    (volatile uint32_t *)((u32ReTrimCnt == 1) ? u32SrcAddr : (u32DMMAddr + u32SrcAddr)),
                    (int32_t)TRIM_PAT_SIZE * 2);
#endif

                if (u32ReTrimCnt == 1)
                {
                    SPIM_DMA_Read(SPIM_PORT,
                                  (u32SrcAddr + u32LoopAddr),
                                  (pMTRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                  0x08,
                                  &pu8VerifyBuf[u32k],
                                  pMTRdCMD->u32CMDCode,
                                  SPIM_OP_ENABLE);
                }
                else
                {
                    SPIM_EnterDirectMapMode(spim,
                                            (pMTRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                                            pMTRdCMD->u32CMDCode,
                                            1);

                    /* Read 8 bytes of data from the HyperRAM */
                    //*(volatile uint64_t *)&pu8VerifyBuf[u32k] = *(volatile uint64_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                    *(volatile uint32_t *)&pu8VerifyBuf[u32k] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                    *(volatile uint32_t *)&pu8VerifyBuf[u32k + 4] = *(volatile uint32_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr + 4);
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

    OctalFlash_ExitDDRMode(SPIM_PORT);

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

    printf("\n[Fast Read Output] 3-bytes address mode, Fast Read Octal DDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt9DhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtC2hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimDLLDelayNum(SPIM_PORT, &gsMtC2hWrCMD, &gsMt9DhRdCMD);

    u32Is4ByteAddr = (gsMt9DhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsMt9DhRdCMD.u32CMDCode, gsMtC2hWrCMD.u32CMDCode, SPIM_OP_DISABLE) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast IO Read] 4-bytes address mode, Fast Read Octal DDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtFDhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8EhWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimDLLDelayNum(SPIM_PORT, &gsMt8EhWrCMD, &gsMtFDhRdCMD);

    u32Is4ByteAddr = (gsMtFDhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;

    if (dma_read_write(u32Is4ByteAddr, gsMtFDhRdCMD.u32CMDCode, gsMt8EhWrCMD.u32CMDCode, SPIM_OP_DISABLE) != SPIM_OK)
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

    OctalFlash_EnterDDRMode(SPIM_PORT); /* Enable Octal Flash DDR Mode */

    if (dma_read_write(u32Is4ByteAddr, gsMt8BhRdDDRCMD.u32CMDCode, gsMt02hWrDDRCMD.u32CMDCode, SPIM_OP_ENABLE) != SPIM_OK)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    OctalFlash_ExitDDRMode(SPIM_PORT);  /* Disable Octal Flash DDR Mode */

    printf("[OK].\n");

    printf("\nOctal SPI Flash read/write demo done.\n");
lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
