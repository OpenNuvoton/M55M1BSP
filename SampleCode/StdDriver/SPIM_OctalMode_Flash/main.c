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

#define FLASH_BLOCK_SIZE            (8 * 1024)     /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000        /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048

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

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
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

        /* Init SPIM multi-function pins */
        SET_SPIM0_CLKN_PC5();
        SET_SPIM0_CLK_PC4();
        SET_SPIM0_D2_PC0();
        SET_SPIM0_D3_PG10();
        SET_SPIM0_D4_PG9();
        SET_SPIM0_D5_PG13();
        SET_SPIM0_D6_PG14();
        SET_SPIM0_D7_PG15();
        SET_SPIM0_MISO_PG12();
        SET_SPIM0_MOSI_PG11();
        SET_SPIM0_RESETN_PC2();
        SET_SPIM0_RWDS_PC1();
        SET_SPIM0_SS_PC3();

        PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                      GPIO_SMTEN_SMTEN1_Msk |
                      GPIO_SMTEN_SMTEN2_Msk |
                      GPIO_SMTEN_SMTEN3_Msk |
                      GPIO_SMTEN_SMTEN4_Msk |
                      GPIO_SMTEN_SMTEN5_Msk);

        PG->SMTEN |= (GPIO_SMTEN_SMTEN9_Msk |
                      GPIO_SMTEN_SMTEN10_Msk |
                      GPIO_SMTEN_SMTEN11_Msk |
                      GPIO_SMTEN_SMTEN12_Msk |
                      GPIO_SMTEN_SMTEN13_Msk |
                      GPIO_SMTEN_SMTEN14_Msk |
                      GPIO_SMTEN_SMTEN15_Msk);

        /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
        GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT1, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT2, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PC, BIT5, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PG, BIT9, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT13, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT14, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PG, BIT15, GPIO_SLEWCTL_HIGH);
    }
    else if (SPIM_PORT == SPIM1)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);

        /* Init SPIM multi-function pins */
        SET_SPIM1_CLKN_PH12();
        SET_SPIM1_CLK_PH13();
        SET_SPIM1_D2_PJ4();
        SET_SPIM1_D3_PJ3();
        SET_SPIM1_D4_PH15();
        SET_SPIM1_D5_PD7();
        SET_SPIM1_D6_PD6();
        SET_SPIM1_D7_PD5();
        SET_SPIM1_MISO_PJ5();
        SET_SPIM1_MOSI_PJ6();
        SET_SPIM1_RESETN_PJ2();
        SET_SPIM1_RWDS_PH14();
        SET_SPIM1_SS_PJ7();

        PD->SMTEN |= (GPIO_SMTEN_SMTEN5_Msk |
                      GPIO_SMTEN_SMTEN6_Msk |
                      GPIO_SMTEN_SMTEN7_Msk);
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
        GPIO_SetSlewCtl(PD, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PD, BIT7, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PH, BIT12, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT13, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT14, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PH, BIT15, GPIO_SLEWCTL_HIGH);

        GPIO_SetSlewCtl(PJ, BIT2, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT3, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT4, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT5, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT6, GPIO_SLEWCTL_HIGH);
        GPIO_SetSlewCtl(PJ, BIT7, GPIO_SLEWCTL_HIGH);
    }
}

int SPIM_TrimRXCLKDLY(SPIM_T *spim)
{
    uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[0xF] = {0};
    uint32_t u32SAddr = 0x100;
    uint8_t au8SrcData[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
    uint8_t au8DestData[32] = {0};

    SPIM_EraseBlock(spim, u32SAddr, SPIM_OP_DISABLE, OPCODE_BE_64K, SPIM_OP_ENABLE, SPIM_OP_ENABLE);

    SPIM_DMADMM_InitPhase(spim, &gsMt02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    SPIM_DMA_Write(spim,
                   u32SAddr,
                   ((gsMt02hWrCMD.u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL),
                   sizeof(au8SrcData),
                   au8SrcData,
                   gsMt02hWrCMD.u32CMDCode);

    SPIM_DMADMM_InitPhase(spim, &gsMt0BhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);

    for (u8RdDelay = 0; u8RdDelay <= 0xF; u8RdDelay++)
    {
        memset(au8DestData, 0, sizeof(au8DestData));
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);

        SPIM_DMA_Read(spim,
                      u32SAddr,
                      ((gsMt0BhRdCMD.u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL),
                      sizeof(au8DestData),
                      au8DestData,
                      gsMt0BhRdCMD.u32CMDCode,
                      SPIM_OP_ENABLE);

        // Compare.
        if (memcmp(au8SrcData, au8DestData, sizeof(au8DestData)) == 0)
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    if (u8RdDelayIdx >= 2)
    {
        u8RdDelayIdx = ((u8RdDelayIdx / 2) - 1);
    }
    else
    {
        u8RdDelayIdx = 0;
    }

    printf("\r\nRX Delay = %d\r\n\r\n", u8RdDelayRes[u8RdDelayIdx]);
    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelayRes[u8RdDelayIdx]);

    return u8RdDelay;
}

void OctalFlash_EnterDDRMode(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {0xE7};
    SPIM_PHASE_T gWrNVCRegCMD =
    {
        OPCODE_WR_VCONFIG,                                          //Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,       //Command Phase
        0, 0, 0,                                                    //Register command not Address
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR,   //Data Phase
        0,
    };

    /* Disable 4-byte address mode */
    SPIM_Enable_4Bytes_Mode(spim, 0, 1);

    /* Set non-volatile register enter octal DDR mode */
    SPIM_IO_WritePhase(spim, &gWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf));

    /* Enable 4-byte address mode */
    SPIM_Enable_4Bytes_Mode(spim, 1, 8);
}

void OctalFlash_ExitDDRMode(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {0xFF};
    SPIM_PHASE_T gWrNVCRegCMD =
    {
        OPCODE_WR_VCONFIG,                                     //Command Code
        PHASE_OCTAL_MODE, PHASE_WIDTH_8, PHASE_ENABLE_DTR,     //Command Phase
        0, 0, 0,                                               //Register Command not Address
        PHASE_OCTAL_MODE, PHASE_ORDER_MODE0, PHASE_ENABLE_DTR, //Data Phase
        0,
    };

    /* Set non-volatile register exit octal DDR mode */
    SPIM_IO_WritePhase(spim, &gWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf));

    /* Disable 4-byte Address mode */
    SPIM_Enable_4Bytes_Mode(spim, 0, 1);
}

int dma_read_write(int is4ByteAddr, uint32_t u32RdCmd, uint32_t WrCmd, uint32_t u32DDREn)
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
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, is4ByteAddr, OPCODE_BE_64K, u32EraseNBit, SPIM_OP_ENABLE);
    printf("done.\n");

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);

        SPIM_IO_ReadPhase(SPIM_PORT, &gsMt0BhRdCMD, TEST_BLOCK_ADDR + offset, g_buff, BUFFER_SIZE);

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

        pData = (uint32_t *)&g_buff[0];

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if ((*pData) != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n",
                       TEST_BLOCK_ADDR + i,
                       *pData,
                       ((i << 16) | (TEST_BLOCK_ADDR + offset + i)));
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

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                            */
    /*------------------------------------------------------------------------*/
    printf("+------------------------------------------------------------------------+\n");
    printf("|      SPIM DMA mode read/write octal flash(Micron MT35XU02G) sample     |\n");
    printf("+------------------------------------------------------------------------+\n");

    /**
     * @brief Set SPIM clock as HCLK divided by 8
     * @note  Octal flash in DDR mode only support 1 or 2.
     */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, 8);

    SPIM_DISABLE_CIPHER(SPIM_PORT);

    SPIM_DISABLE_CACHE(SPIM_PORT);

    if (SPIM_InitFlash(SPIM_PORT, 1) != 0)          /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), 1, 0);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           idBuf[0], idBuf[1], idBuf[2]);

    /* Trim delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRXCLKDLY(SPIM_PORT);

    printf("\n[Fast IO Read] 3-bytes address mode, Fast Read Octal SDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtCBhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtC2hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    if (dma_read_write(0, gsMtCBhRdCMD.u32CMDCode, gsMtC2hWrCMD.u32CMDCode, SPIM_OP_DISABLE) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast IO Read] 4-bytes address mode, Fast Read Octal SDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtCChRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8EhWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    if (dma_read_write(1, gsMtCChRdCMD.u32CMDCode, gsMt8EhWrCMD.u32CMDCode, SPIM_OP_DISABLE) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Output] 3-bytes address mode, Fast Read Octal DDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt9DhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtC2hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    if (dma_read_write(0, gsMt9DhRdCMD.u32CMDCode, gsMtC2hWrCMD.u32CMDCode, SPIM_OP_DISABLE) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast IO Read] 4-bytes address mode, Fast Read Octal DDR command...\r\n");

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMtFDhRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8EhWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    if (dma_read_write(1, gsMtFDhRdCMD.u32CMDCode, gsMt8EhWrCMD.u32CMDCode, SPIM_OP_DISABLE) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    printf("[OK].\n");

    printf("\n[Fast Read Output DDR Mode] 4-bytes address mode, Fast Read Octal DDR command...\r\n");

    OctalFlash_EnterDDRMode(SPIM_PORT); /* Enable Octal Flash DDR Mode */

    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt8BhRdDDRCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM_PORT, &gsMt02hWrDDRCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    if (dma_read_write(1, gsMt8BhRdDDRCMD.u32CMDCode, gsMt02hWrDDRCMD.u32CMDCode, SPIM_OP_ENABLE) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }

    OctalFlash_ExitDDRMode(SPIM_PORT);  /* Disable Octal Flash DDR Mode */

    printf("[OK].\n");

lexit:

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
