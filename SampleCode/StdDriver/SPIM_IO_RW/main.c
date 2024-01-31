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

#define USE_4_BYTES_MODE            0     /* W25Q20 does not support 4-bytes address mode. */
#define MFP_SELECT                  0     /* Multi-function pin select                     */

#define FLASH_BLOCK_SIZE            (8 * 1024)  /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000     /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048

//------------------------------------------------------------------------------
__attribute__((aligned(32))) uint8_t g_buff[BUFFER_SIZE] = {0};

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

    if (SPIM_PORT == SPIM0)
    {
        /* Enable SPIM module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);

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

    /* Set SPIM clock as HCLK divided by 8 */
    SPIM_SET_CLOCK_DIVIDER(SPIM_PORT, 8);

    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDDLYSEL(SPIM_PORT, 0);

    if (SPIM_InitFlash(SPIM_PORT, 1) != 0)      /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_DISABLE_CIPHER(SPIM_PORT);

    SPIM_ReadJedecId(SPIM_PORT, idBuf, sizeof(idBuf), 1, 0);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM_WinbondUnlock(SPIM_PORT, 1);

    if (SPIM_Enable_4Bytes_Mode(SPIM_PORT, USE_4_BYTES_MODE, 1) != 0)
    {
        printf("SPIM_Enable_4Bytes_Mode failed!\n");
        goto lexit;
    }

    /*
     *  Erase flash page
     */
    printf("\r\nErase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, USE_4_BYTES_MODE, OPCODE_BE_64K, 1, 1);
    printf("done.\n");

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);

        pData = (uint32_t *)g_buff;

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
        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);
        }

        SPIM_IO_Write(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                      BUFFER_SIZE, g_buff, OPCODE_PP, 1, 1, 1);
    }

    printf("done.\n");

    /*
     *  Read and compare flash data
     */
    printf("Verify SPI flash block 0x%x data with Fast Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
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
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, g_buff, OPCODE_FAST_DUAL_READ, 1, 1, 2, 1);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");

    /*
     *  Read and compare flash data
     */
    printf("Verify SPI flash block 0x%x data with Fast Quad Read Output command...", TEST_BLOCK_ADDR);

    SPIM_SetQuadEnable(SPIM_PORT, 1, 1);    /* Enable SPI flash quad mode */

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(SPIM_PORT, TEST_BLOCK_ADDR + offset, USE_4_BYTES_MODE,
                     BUFFER_SIZE, g_buff, CMD_DMA_FAST_READ_QUAD_OUTPUT, 1, 1, 4, 1);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    SPIM_SetQuadEnable(SPIM_PORT, 0, 1);

    printf("done.\n");


    /*
     *  Erase flash page
     */
    printf("\r\nErase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(SPIM_PORT, TEST_BLOCK_ADDR, USE_4_BYTES_MODE, OPCODE_BE_64K, 1, 1);
    printf("done.\n");

    /*
     *  Phase IO Program data to flash block and Phase IO Read and compare flash data
     */
    printf("Phase IO Program sequential data to flash block 0x%x...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            (*pData) = (i << 16) | (TEST_BLOCK_ADDR + offset + i);
        }

        SPIM_IO_WritePhase(SPIM_PORT, &gsWb02hWrCMD, TEST_BLOCK_ADDR + offset,
                           g_buff, BUFFER_SIZE);
    }

    printf("done.\n");

    printf("Verify SPI flash block 0x%x data with Phase IO Fast Dual Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);

        SPIM_IO_ReadPhase(SPIM_PORT, &gsWbBBhRdCMD, TEST_BLOCK_ADDR + offset, g_buff, BUFFER_SIZE);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                goto lexit;
            }
        }
    }

    printf("done.\n");

    printf("Verify SPI flash block 0x%x data with Phase IO Fast Quad Read command...", TEST_BLOCK_ADDR);

    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);

        SPIM_IO_ReadPhase(SPIM_PORT, &gsWbEBhRdCMD, TEST_BLOCK_ADDR + offset, g_buff, BUFFER_SIZE);

        pData = (uint32_t *)g_buff;

        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR + i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
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
