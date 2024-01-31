/**************************************************************************//**
 * @file    hyperFlash_init.h
 * @version V1.00
 * @brief   HyerRAM driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __HYPERFLASH_INIT_H__
#define __HYPERFLASH_INIT_H__

#include <stdint.h>

/*----------------------------------------------------------------------------*/
/* Include related headers                                                    */
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/
//------------------------------------------------------------------------------
// HyerRAM Test Port
//------------------------------------------------------------------------------
#define DLL_MAX_DELAY_NUM                   (0x1F) //0x1F

#define TEST_BUFF_SIZE                      (0x200)

//------------------------------------------------------------------------------
// HyperFlash Operation Command
//------------------------------------------------------------------------------
//#define HFLH_START_ADDR                     (0x0)
#define HFLH_PAGE_SIZE                      (0x200)
#define HFLH_SECTOR_SIZE                      (0x40000)
/*
 * @Note: one sector = 256K
 * @Note: S26KL512S and S26KS512S Sector Count = 256
 * @Note: S26KL256S and S26KS256S Sector Count = 128
*/
#define HFLH_MAX_SECTOR                     (256)

//------------------------------------------------------------------------------
#define HF_CMD_NOOP_CODE                       (0x0)
#define HF_CMD_COMMON_555                      (0x555)
#define HF_CMD_COMMON_AA                       (0xAA)
#define HF_CMD_COMMON_2AA                      (0x2AA)
#define HF_CMD_COMMON_55                       (0x55)

#define HF_CMD_70                              (0x70) //Read Status Register
#define HF_CMD_71                              (0x71) //Clear Status Register
#define HF_CMD_B9                              (0xB9) //Enter Power Down
#define HF_CMD_34                              (0x34) //Set Power Down Reset Timer
#define HF_CMD_3C                              (0x3C) //Read Power Down Reset Timer
#define HF_CMD_36                              (0x36) //Load Interrupt Config Register
#define HF_CMD_C4                              (0xC4) //Read Interrupt Config Register
#define HF_CMD_37                              (0x37) //Load Interrupt Status Register
#define HF_CMD_C5                              (0xC5) //Read Interrupt Status Register
#define HF_CMD_38                              (0x38) //Load Volatile Config Register
#define HF_CMD_C7                              (0xC7) //Read Volatile Config Register
#define HF_CMD_39                              (0x39) //Program Non-Volatile Config Register
#define HF_CMD_C8                              (0xC8) //Erase Non-Volatile Config Register
#define HF_CMD_C6                              (0xC6) //Read Non-Volatile Config Register
#define HF_CMD_A0                              (0xA0) //Word Program
#define HF_CMD_25                              (0x25) //Write Buffer
#define HF_CMD_29                              (0x29) //Program Buffer to Flash (Confirm)
#define HF_CMD_F0                              (0xF0) //Write To Buffer Abort Reset
#define HF_CMD_98                              (0x98) //Read ID Command
#define HF_CMD_50                              (0x50) ////Clear ECC Error

#define HF_CMD_80                              (0x80) //Erase first commnad
#define HF_CMD_10                              (0x10) //Erase Chip Commnad
#define HF_CMD_30                              (0x30) //Erase Sector Commnad
#define HF_CMD_33                              (0x33) //Blank Check

#define ERASE_NON_VOL_CONF_REG              (0xC8)
#define LOAD_VCR_REG                        (0x38)
#define READ_VCR_REG                        (0xC7)
#define WRITE_NVCR_REG                      (0x39)
#define READ_NVCR_REG                       (0xC6)
#define CMD_WORD_PROGRAM                    (0xA0)

//------------------------------------------------------------------------------
// HyperFlash Common Macro
//------------------------------------------------------------------------------
#define HFLH_MAX_CS_LOW                     0xFFFF//(0xFFFF)//(800)
#define HFLH_WR_ACCTIME                     (1)
#define HFLH_RD_ACCTIME                     (9)

/** @addtogroup HyperFlash_Driver HyperFlash Driver
  @{
*/

/** @addtogroup HyerRAM_EXPORTED_CONSTANTS HyerRAM Exported Constants
  @{
*/

//extern uint8_t tstbuf[];
//extern uint8_t tstbuf2[];

/*----------------------------------------------------------------------------*/
/* Define Function Prototypes                                                 */
/*----------------------------------------------------------------------------*/
void DumpBufferHex(uint8_t *pucBuff, int nSize);
void popDat(uint8_t buf[], uint32_t bufSize);

void HyperFlash_PinConfig(SPIM_T *pSPIMx);

/* SPIM Init HyperBus Mode */
void HyperFlash_Init(SPIM_T *pSPIMx, uint32_t u32CacheOn);
void HyperFlash_EraseSector(SPIM_T *pSPIMx, uint32_t u32SectorCnt);
void HyperFlash_ChipErase(SPIM_T *pSPIMx);

/* HyperFlash Training DLL Delay Time Function */
int SPIM_HyperFlashDLLDelayTimeTraining(SPIM_T *pSPIMx);

/* HyperFlash CMD IO R/W API */
void HyperFlash_DMMWriteData(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvWrBuf, uint32_t u32NTx);
void HyperFlash_DMMReadData(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvRdBuf, uint32_t u32NRx);

void HyperFlash_DMARead(SPIM_T *spim, uint32_t u32SAddr, uint8_t *pu8RdBuf, uint32_t u32NRx);
void HyperFlash_DMAWrite(SPIM_T *spim, uint32_t u32SAddr, uint8_t *pu8WrBuf, uint32_t u32NTx);

void HyperFlash_IO_Read(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvRdBuf, uint32_t u32NRx);
void HyperFlash_IO_Write(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvWrBuf, uint32_t u32NTx);
void HyperFlash_DMMRead(SPIM_T *pSPIMx);
void HyperFlash_DMMWrite(SPIM_T *pSPIMx);

//------------------------------------------------------------------------------
// OTFC
//------------------------------------------------------------------------------
void *GetAESKey0(uint8_t u8Index);
void SetAESKey0(uint8_t u8Index, uint32_t u32Key);
void OTFC_CipherEnKeyFormReg(SPIM_T *pSPIMx,
                             uint32_t u32PRBlock,
                             uint32_t u32PRStartAddr,
                             uint32_t u32PRSize,
                             uint32_t *pu32AESKey,
                             uint32_t DFAEn);

/** @} end of group HyerRAM_EXPORTED_FUNCTIONS */
/** @} end of group HyerRAM_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __HYPERFLASH_INIT_H__ */
