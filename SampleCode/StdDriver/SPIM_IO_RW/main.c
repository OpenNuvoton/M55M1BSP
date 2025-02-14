/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to issue SPI Flash erase, program, and read commands under SPIM I/O mode.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPIM_PORT                   SPIM0
#define SPIM_PORT_DIV               1
#define USE_4_BYTES_MODE            0     /* W25Q20 does not support 4-bytes address mode. */
#define MFP_SELECT                  0     /* Multi-function pin select                     */

#define FLASH_BLOCK_SIZE            (8 * 1024)  /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000     /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048
#define TRIM_PAT_SIZE               32

//------------------------------------------------------------------------------
#if (NVT_DCACHE_ON == 1)
/* DCache-line aligned buffer for improved performance when DCache is enabled */
uint8_t gau8buff[DCACHE_ALIGN_LINE_SIZE(BUFFER_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
#else
/* Standard buffer alignment when DCache is disabled */
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

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);
    CLK_EnableModuleClock(OTFC0_MODULE);

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

int main()
{
    uint32_t i = 0, offset = 0;
    uint32_t *pData = NULL;
    uint8_t idBuf[3] = {0};

    /* Unlock register lock protect */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+-------------------------------------------+\n");
    printf("|      SPIM I/O mode read/write sample      |\n");
    printf("+-------------------------------------------+\n");

    /* Set SPIM clock as HCLK divided by 1 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, SPIM_PORT_DIV);

    SPIM_DISABLE_CIPHER(SPIM_PORT);

    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != SPIM_OK)      /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWb0BhRdCMD);

    SPIM_WinbondUnlock(SPIM_PORT, SPIM_BITMODE_1);

    if (SPIM_Enable_4Bytes_Mode(SPIM_PORT, USE_4_BYTES_MODE, SPIM_BITMODE_1) != SPIM_OK)
    {
        printf("SPIM_Enable_4Bytes_Mode failed!\n");
        goto lexit;
    }

    /*
     *  Erase flash page
     */
    printf("\r\nErase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, USE_4_BYTES_MODE, OPCODE_BE_64K, SPIM_BITMODE_1, SPIM_OP_ENABLE);
    printf("done.\n");

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clear the buffer to 0 before reading from the SPI flash
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a read operation from the SPI flash
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, gau8buff, OPCODE_FAST_READ,
                     SPIM_BITMODE_1, SPIM_BITMODE_1, SPIM_BITMODE_1,
                     8);

        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value (0xFFFFFFFF)
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != 0xFFFFFFFF)
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x!\n", TEST_BLOCK_ADDR + i, *pData);
                goto lexit;
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
        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Populate the buffer with sequential data for programming
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);
        }

#if (NVT_DCACHE_ON == 1)
        // Clean the data cache to ensure the buffer data is written to memory
        SCB_CleanDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Write the prepared data to the SPI flash
        SPIM_IO_Write(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                      BUFFER_SIZE, gau8buff, OPCODE_PP,
                      SPIM_BITMODE_1, SPIM_BITMODE_1, SPIM_BITMODE_1);
    }

    printf("done.\n");

    /*
     *  Read and compare flash data
     */
    printf("Verify SPI flash block 0x%x data with Fast Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clean the buffer with zeros before reading the SPI flash data
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a read operation from the SPI flash
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, gau8buff, OPCODE_FAST_READ,
                     SPIM_BITMODE_1, SPIM_BITMODE_1, SPIM_BITMODE_1,
                     8);

        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");


    /*
     *  Read and compare flash data
     */
    printf("Verify SPI flash block 0x%x data with Fast Read Dual Output command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clear the buffer with zeros before reading from the SPI flash
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a Fast Dual Output read operation from the SPI flash
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, gau8buff, OPCODE_FAST_DUAL_READ,
                     SPIM_BITMODE_1, SPIM_BITMODE_1, SPIM_BITMODE_2,
                     8);

        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");

    /*
     *  Read and compare flash data
     */
    printf("Verify SPI flash block 0x%x data with Fast Quad Read Output command...", TEST_BLOCK_ADDR);

    SPIM_SetQuadEnable(SPIM_PORT, SPIM_OP_ENABLE, SPIM_BITMODE_1);    /* Enable SPI flash quad mode */

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clear the buffer with zeros before reading from the SPI flash
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a Fast Quad Output read operation from the SPI flash
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, gau8buff, CMD_DMA_FAST_READ_QUAD_OUTPUT,
                     SPIM_BITMODE_1, SPIM_BITMODE_1, SPIM_BITMODE_4,
                     8);

        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    SPIM_SetQuadEnable(SPIM_PORT, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    printf("done.\n");


    /*
     *  Erase flash page
     */
    printf("\r\nErase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, USE_4_BYTES_MODE, OPCODE_BE_64K, SPIM_BITMODE_1, SPIM_OP_ENABLE);
    printf("done.\n");

    /*
     *  Phase IO Program data to flash block and Phase IO Read and compare flash data
     */
    printf("Phase IO Program sequential data to flash block 0x%x...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        pData = (uint32_t *)gau8buff;

        // Fill the buffer with sequential data
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);
        }

#if (NVT_DCACHE_ON == 1)
        // Clean the data cache for the buffer before writing to the SPI flash
        SCB_CleanDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a Phase IO Write operation from the buffer to the SPI flash
        SPIM_IO_WriteByPhase(SPIM_PORT, &gsWb02hWrCMD, TEST_BLOCK_ADDR + offset,
                             gau8buff, BUFFER_SIZE, SPIM_OP_ENABLE);
    }

    printf("done.\n");

    printf("Verify SPI flash block 0x%x data with Phase IO Fast Dual Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clear the buffer with zeros before reading from the SPI flash
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a Phase IO Read operation from the SPI flash to the buffer
        SPIM_IO_ReadByPhase(SPIM_PORT, &gsWbBBhRdCMD, TEST_BLOCK_ADDR + offset, gau8buff, BUFFER_SIZE);

        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");

    printf("Verify SPI flash block 0x%x data with Phase IO Fast Quad Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        // Clear the buffer with zeros before reading from the SPI flash
        memset(gau8buff, 0, BUFFER_SIZE);

#if (NVT_DCACHE_ON == 1)
        // Invalidate the data cache for the buffer to ensure data consistency
        SCB_InvalidateDCache_by_Addr((uint32_t *)&gau8buff, sizeof(gau8buff));
#endif

        // Perform a Phase IO Read operation from the SPI flash to the buffer
        SPIM_IO_ReadByPhase(SPIM_PORT, &gsWbEBhRdCMD, TEST_BLOCK_ADDR + offset, gau8buff, BUFFER_SIZE);

        // Initialize the pointer to the buffer data
        pData = (uint32_t *)gau8buff;

        // Verify the data read from the SPI flash with the expected value
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");

    printf("\nSPIM I/O read/write demo done.\n");

lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
