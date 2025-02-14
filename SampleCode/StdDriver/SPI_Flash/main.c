/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Access SPI flash through SPI interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define TEST_NUMBER                 1         /* page numbers */
#define TEST_LENGTH                 256       /* length */

#define SPI_FLASH_PORT              SPI0

/* SPI Flash Operation Code */
#define OPCODE_DMY                  (0x00U)   /* Dummy data */
#define OPCODE_WREN                 (0x06U)   /* Write enable */
#define OPCODE_RDSR                 (0x05U)   /* Read status register #1*/
#define OPCODE_WRSR                 (0x01U)   /* Write status register #1 */
#define OPCODE_RDSR2                (0x35U)   /* Read status register #2*/
#define OPCODE_WRSR2                (0x31U)   /* Write status register #2 */
#define OPCODE_RDSR3                (0x15U)   /* Read status register #3*/
#define OPCODE_WRSR3                (0x11U)   /* Write status register #3 */
#define OPCODE_PP                   (0x02U)   /* Page program (up to 256 bytes) */
#define OPCODE_SE_4K                (0x20U)   /* Erase 4KB sector */
#define OPCODE_BE_32K               (0x52U)   /* Erase 32KB block */
#define OPCODE_CHIP_ERASE           (0xC7U)   /* Erase whole flash chip */
#define OPCODE_BE_64K               (0xD8U)   /* Erase 64KB block */
#define OPCODE_READ_ID              (0x90U)   /* Read ID */
#define OPCODE_RDID                 (0x9fU)   /* Read JEDEC ID */

#define OPCODE_NORM_READ            (0x03U)   /* Read data bytes */
#define OPCODE_FAST_READ            (0x0BU)   /* Read data bytes */
#define OPCODE_FAST_DUAL_READ       (0x3BU)   /* Read data bytes */
#define OPCODE_FAST_QUAD_READ       (0x6BU)   /* Read data bytes */
#define OPCODE_FAST_QUAD_IO_READ    (0xEBU)  /* Read data bytes */

#define FLH_IS_BUSY                 (0x01)

#define FLH_W25Q80                  (0xEF13)
#define FLH_W25Q16                  (0xEF14)
#define FLH_W25Q32                  (0xEF15)
#define FLH_W25Q64                  (0xEF16)
#define FLH_W25Q128                 (0xEF17)
#define FLH_W25Q256                 (0xEF18)

//------------------------------------------------------------------------------
static uint8_t s_au8SrcArray[TEST_LENGTH] = {0};
static uint8_t s_au8DestArray[TEST_LENGTH] = {0};

//------------------------------------------------------------------------------
uint16_t SpiFlash_ReadMidDid(void);
void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value);
int32_t SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SYS_Init(void);

//------------------------------------------------------------------------------
__STATIC_INLINE void wait_SPI_IS_BUSY(SPI_T *spi)
{
    volatile int32_t i32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (SPI_IS_BUSY(spi))
    {
        if (--i32TimeOutCnt <= 0)
        {
            printf("Wait for SPI time-out!\n");
            break;
        }
    }
}

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_READ_ID);

    // send 24-bit '0', dummy
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);

    // receive 16-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while (!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);

    return (uint16_t)((u8RxData[4] << 8) | u8RxData[5]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_WREN);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_CHIP_ERASE);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_RDSR);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_WREN);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_WRSR);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

int32_t SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;
    volatile int32_t i32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if (--i32TimeOutCnt <= 0)
        {
            printf("Wait for SPI time-out!\n");
            return SPI_ERR_TIMEOUT;
        }

        u8ReturnValue = SpiFlash_ReadStatusReg();
    } while ((u8ReturnValue & FLH_IS_BUSY) != 0); // check the BUSY bit

    return SPI_OK;
}

void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_WREN);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress & 0xFF);

    // write data
    while (1)
    {
        if (!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);

            if (u32Cnt > 255) break;
        }
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_NORM_READ);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress & 0xFF);

    wait_SPI_IS_BUSY(SPI_FLASH_PORT);
    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // read data
    for (u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, OPCODE_DMY);
        wait_SPI_IS_BUSY(SPI_FLASH_PORT);
        u8DataBuffer[u32Cnt] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

void SYS_Init(void)
{
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

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup SPI0 multi-function pins */
    SET_SPI0_SS_PA3();
    SET_SPI0_CLK_PA2();
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0x3F, GPIO_SLEWCTL_HIGH);
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint16_t u16ID;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);

    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                        SPI Sample with SPI Flash                        |\n");
    printf("+-------------------------------------------------------------------------+\n");

    u16ID = SpiFlash_ReadMidDid();

    if (u16ID == FLH_W25Q80)
        printf("Flash found: W25Q80 ...\n");
    else if (u16ID == FLH_W25Q16)
        printf("Flash found: W25Q16 ...\n");
    else if (u16ID == FLH_W25Q32)
        printf("Flash found: W25Q32 ...\n");
    else if (u16ID == FLH_W25Q64)
        printf("Flash found: W25Q64 ...\n");
    else if (u16ID == FLH_W25Q128)
        printf("Flash found: W25Q128 ...\n");
    else if (u16ID == FLH_W25Q256)
        printf("Flash found: W25Q256 ...\n");
    else
    {
        printf("Wrong ID, 0x%x\n", u16ID);

        while (1);
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    if (SpiFlash_WaitReady() < 0) return SPI_ERR_FAIL;

    printf("[OK]\n");

    /* init source data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);

        if (SpiFlash_WaitReady() < 0) return SPI_ERR_FAIL;

        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8DestArray[u32ByteCount] = 0;
    }

    printf("Normal Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;

    for (u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_NormalRead(u32FlashAddress, s_au8DestArray);
        u32FlashAddress += 0x100;

        for (u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if (s_au8DestArray[u32ByteCount] != s_au8SrcArray[u32ByteCount])
                u32Error ++;
        }
    }

    if (u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    while (1);
}
