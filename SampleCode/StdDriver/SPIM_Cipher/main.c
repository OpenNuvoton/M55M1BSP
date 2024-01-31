/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show SPIM DMA read/write with cipher enabled. This sample code
 *          also dumps SPI Flash content via I/O mode read to prove it is
 *          encrypted cipher context.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPIM_PORT                   SPIM0

//------------------------------------------------------------------------------
#define FLASH_BLOCK_SIZE            (8 * 1024)      /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000         /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048
//#define DMM_MODE_TRIM

//------------------------------------------------------------------------------
__attribute__((aligned(32))) uint8_t g_buff[BUFFER_SIZE] = {0};

/* SPIM cipher key User defined. */
uint32_t gau32AESKey[8] =
{
    0x93484D6F, //Key0
    0x2F7A7F2A, //Key1
    0x063FF08A, //Key2
    0x7A29E38E, //Key3
    0x7A29E38E, //Scramble
    0x063FF08A, //NONCE0
    0x2F7A7F2A, //NONCE1
    0x93484D6F, //NONCE2
};

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
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

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

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Enable TRNG module clock */
    CLK_EnableModuleClock(TRNG0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    if (SPIM_PORT == SPIM0)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);
        CLK_EnableModuleClock(OTFC0_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM0_CLK_PC4();
        SET_SPIM0_MISO_PG12();
        SET_SPIM0_MOSI_PG11();
        SET_SPIM0_D2_PC0();
        SET_SPIM0_D3_PG10();
        SET_SPIM0_SS_PC3();

        PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk);

        PG->SMTEN |= (GPIO_SMTEN_SMTEN10_Msk |
                      GPIO_SMTEN_SMTEN11_Msk |
                      GPIO_SMTEN_SMTEN12_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);
    }
    else if (SPIM_PORT == SPIM1)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);
        CLK_EnableModuleClock(OTFC1_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM1_CLK_PH13();
        SET_SPIM1_MISO_PJ5();
        SET_SPIM1_MOSI_PJ6();
        SET_SPIM1_D2_PJ4();
        SET_SPIM1_D3_PJ3();
        SET_SPIM1_SS_PJ7();

        PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

        PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PH, BIT13, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PJ, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT7, GPIO_SLEWCTL_HIGH);
    }
}

void DumpBufferHex(uint8_t *pucBuff, int nSize)
{
    int     nIdx, i;

    nIdx = 0;

    while (nSize > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);

        printf("  ");

        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");

            nSize--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

int SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbWrCMD, SPIM_PHASE_T *psWbRdCMD)
{
    volatile uint8_t u8RdDelay = 0;
    uint8_t u8TmpRdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[0x0F] = {0};
    uint32_t u32SAddr = 0x0;
    volatile uint32_t u32i = 0;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint8_t au8TrimPatten[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
    uint8_t au8CmpBuf[32] = {0};
#ifdef DMM_MODE_TRIM
    uint32_t u32RdDataCnt = 0;
    uint32_t u32DMMAddr = SPIM_GetDMMAddress(spim);
    uint32_t *pu32RdData = NULL;

    SPIM_DMADMM_InitPhase(SPIM_PORT, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
#endif //

    SPIM_SET_CLOCK_DIVIDER(spim, 8);

    SPIM_EraseBlock(spim,
                    u32SAddr,
                    psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                    OPCODE_BE_64K,
                    1,
                    SPIM_OP_ENABLE);

    SPIM_DMA_Write(spim,
                   u32SAddr,
                   psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                   sizeof(au8TrimPatten),
                   au8TrimPatten,
                   psWbWrCMD->u32CMDCode);

    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

#ifdef DMM_MODE_TRIM
    //SPIM_DMM_ReadPhase(pSPIMx, psWbRdCMD, 1);
    SPIM_EnterDirectMapMode(spim,
                            psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                            psWbRdCMD->u32CMDCode,
                            1);
#endif

    for (u8RdDelay = 0; u8RdDelay <= 0xF; u8RdDelay++)
    {
        u8TmpRdDelay = u8RdDelay;
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8TmpRdDelay);

        memset(au8CmpBuf, 0, sizeof(au8TrimPatten));

#ifndef DMM_MODE_TRIM
        SPIM_DMA_Read(spim,
                      u32SAddr,
                      psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32 ? SPIM_OP_ENABLE : SPIM_OP_DISABLE,
                      sizeof(au8TrimPatten),
                      au8CmpBuf,
                      psWbRdCMD->u32CMDCode,
                      SPIM_OP_ENABLE);
#else
        u32RdDataCnt = 0;
        pu32RdData = (uint32_t *)au8CmpBuf;

        //SPIM_IO_ReadPhase(pSPIMx, pMT0BhRdCMD, u32SAddr, tstbuf2, sizeof(au8TrimPatten));

        for (u32i = u32SAddr; u32i < (u32SAddr + sizeof(au8TrimPatten)); u32i += 4)
        {
            pu32RdData[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
        }

#endif

        // Compare.
        if (memcmp(au8TrimPatten, au8CmpBuf, sizeof(au8TrimPatten)) == 0)
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    if (u8RdDelayIdx >= 2)
    {
        u8RdDelayIdx = (u8RdDelayIdx / 2) /*- 1*/;
    }
    else
    {
        u8RdDelayIdx = 0;
    }

    printf("\r\nRX Delay = %d\r\n\r\n", u8RdDelayRes[u8RdDelayIdx]);
    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelayRes[u8RdDelayIdx]);

    return u8RdDelay;
}

int dma_read_write(int is4ByteAddr, uint32_t u32RdCmd, uint32_t WrCmd)
{
    uint32_t i = 0, offset = 0;             /* variables */
    uint32_t *pData = NULL;

    SPIM_ENABLE_CIPHER(SPIM_PORT);

    /*
     *  Erase flash page
     */
    printf("Erase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, is4ByteAddr, OPCODE_BE_64K, 1, 1);
    printf("done.\n");

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, is4ByteAddr, BUFFER_SIZE,
                     g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);

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
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);

        SPIM_DMA_Write(SPIM_PORT, TEST_BLOCK_ADDR + offset, is4ByteAddr, BUFFER_SIZE, g_buff, WrCmd);
    }

    printf("done.\n");

    /*
     *  Verify flash block data
     */
    printf("Verify SPI flash block 0x%x data...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_DMA_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, is4ByteAddr, BUFFER_SIZE, g_buff, u32RdCmd, 1);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if ((*pData) != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                return -1;
            }
        }
    }

    printf("done.\n");

    memset(g_buff, 0, 64);
    printf("\n\nDump SPI flash with I/O read. It's the SPIM cipher encrypted text.\n");
    SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR, is4ByteAddr, BUFFER_SIZE, g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);
    DumpBufferHex(g_buff, 64);

    memset(g_buff, 0, 64);
    printf("\n\nDump SPI flash with DMA read. It's the plain text. "
           "The cipher text was decrypted by SPIM while doing DMA read.\n");
    SPIM_DMA_Read(SPIM_PORT, TEST_BLOCK_ADDR, is4ByteAddr, BUFFER_SIZE, g_buff, u32RdCmd, 1);
    DumpBufferHex(g_buff, 64);

    SPIM_DISABLE_CIPHER(SPIM_PORT);

    return 0;
}

int main()
{
    uint8_t idBuf[3];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                            */
    /*------------------------------------------------------------------------*/
    printf("+-------------------------------------------+\n");
    printf("|        SPIM DMA Cipher mode sample        |\n");
    printf("+-------------------------------------------+\n");

    /* Set SPIM clock as HCLK divided by 1 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, 1);

    SPIM_DISABLE_CIPHER(SPIM_PORT);

    /* Set Cipher Key and protection region */
    if (SPIM_PORT == SPIM0)
    {
        OTFC_SetKeyFromKeyReg(OTFC0, gau32AESKey, OTFC_PR_0, TEST_BLOCK_ADDR, BUFFER_SIZE);
        OTFC_ENABLE_PR(OTFC0, OTFC_PR_0);
    }
    else
    {
        OTFC_SetKeyFromKeyReg(OTFC1, gau32AESKey, OTFC_PR_0, TEST_BLOCK_ADDR, BUFFER_SIZE);
        OTFC_ENABLE_PR(OTFC1, OTFC_PR_0);
    }

    if (SPIM_InitFlash(SPIM_PORT, SPIM_OP_ENABLE) != 0)          /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), 1, 0);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           idBuf[0], idBuf[1], idBuf[2]);

    printf("\n[Fast Read] 3-bytes address mode, Fast Read command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWb0BhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWb0BhRdCMD);

    if (dma_read_write(SPIM_OP_DISABLE, gsWb0BhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Dual Output] 3-bytes address mode, Fast Read Dual command...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbBBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWbBBhRdCMD);

    if (dma_read_write(SPIM_OP_DISABLE, gsWbBBhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Quad Output] 3-bytes address mode, Fast Read Quad command...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbEBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb02hWrCMD, &gsWbEBhRdCMD);

    if (dma_read_write(SPIM_OP_DISABLE, gsWbEBhRdCMD.u32CMDCode, gsWb02hWrCMD.u32CMDCode) < 0)
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

    if (dma_read_write(SPIM_OP_ENABLE, gsWbBChRdCMD.u32CMDCode, gsWb12hWrCMD.u32CMDCode) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Quad I/O] 4-bytes address mode, quad read...\r\n");
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsWbEChRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRxClkDlyNum(SPIM_PORT, &gsWb12hWrCMD, &gsWbEChRdCMD);

    if (dma_read_write(SPIM_OP_ENABLE, gsWbEChRdCMD.u32CMDCode, gsWb12hWrCMD.u32CMDCode) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\nSPIM DMA cipher mode read/write demo done.\n");

lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
