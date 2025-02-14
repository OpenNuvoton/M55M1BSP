/**************************************************************************//**
 * @file        hyperflash_init.h
 * @version     V1.00
 * @brief       HyperFlash initial function
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __SPI_FLASH_INIT_H__
#define __SPI_FLASH_INIT_H__

#define SPIM_PORT                   SPIM0
#define SPIM_PORT_DIV               1
#define TRIM_PAT_SIZE               32
#define SPI_FLASH_PAGE_SIZE         0x1000
#define SPI_FLASH_FIXED_RDDLY       4

#ifdef __cplusplus
extern "C"
{
#endif

int32_t SPIFlash_Init(SPIM_T *psSPIM);
int32_t SPIFlash_SetReadMode(uint32_t u32ReadMode);
int32_t SPIFlash_EraseBlock(SPIM_T *psSPIM, uint32_t u32Addr);
int32_t SPIFlash_ErasePage(SPIM_T *psSPIM, uint32_t u32Addr);
int32_t SPIFlash_WritePage(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize);
int32_t SPIFlash_ReadPage(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize);
int32_t SPIFlash_Write(SPIM_T *psSPIM, uint32_t u32Offset, uint32_t *pu32Data, uint32_t u32ByteSize);
uint32_t SPIFlash_GetFlashSize(SPIM_T *psSPIM);

#ifdef __cplusplus
}
#endif

#endif  // __SPI_FLASH_INIT_H__

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
