/**************************************************************************//**
 * @file     fmc.c
 * @version  V1.00
 * @brief    FMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FMC_Driver FMC Driver
  @{
*/

int32_t g_FMC_i32ErrCode = FMC_OK; /*!< FMC global error code */

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
  @{
*/

/**
  * @brief Execute FMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *         Return 0xFFFFFFFF if read failed.
  * @note   Global error code g_FMC_i32ErrCode
  *         FMC_ERR_TIMEOUT  Read time-out
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD  = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return FMC_ERR_TIMEOUT;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        g_FMC_i32ErrCode = FMC_ERR_READ_FAILED;
        return (uint32_t)FMC_ERR_READ_FAILED;
    }

    return FMC->ISPDAT;
}


/**
  * @brief Execute FMC_ISPCMD_READ_64 command to read a double-word from flash.
  * @param[in]  u32Addr   Address of the flash location to be read.
  *             It must be a double-word aligned address.
  * @param[out] pu32Data0  Place holder of word 0 read from flash address u32Addr.
  * @param[out] pu32Data1  Place holder of word 0 read from flash address u32Addr+4.
  * @return   FMC_OK              Success
  * @return   FMC_ERR_TIMEOUT     Read time-out
  * @return   FMC_ERR_READ_FAILED Read Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT  Read time-out
  */
int32_t FMC_Read_64(uint32_t u32Addr, uint32_t *pu32Data0, uint32_t *pu32Data1)
{
    int32_t  i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD  = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT  = 0x0UL;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_READ_FAILED;
        i32RetCode = FMC_ERR_READ_FAILED;
    }
    else
    {
        *pu32Data0 = FMC->MPDAT0;
        *pu32Data1 = FMC->MPDAT1;
    }

    return i32RetCode;
}


/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   FMC_OK   Success
  * @return   FMC_ERR_PROG_FAILED  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_PROG_FAILED  Program failed
  *           FMC_ERR_TIMEOUT      Time-out
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT  = u32Data;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return FMC_ERR_TIMEOUT;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        return FMC_ERR_PROG_FAILED;
    }

    return FMC_OK;
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM_64 to program a double-word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a double-word aligned address.
  * @param[in]  u32Data0   The word data to be programmed to flash address u32Addr.
  * @param[in]  u32Data1   The word data to be programmed to flash address u32Addr+4.
  * @return   FMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write8Bytes(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0  = u32Data0;
    FMC->MPDAT1  = u32Data1;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    return i32RetCode;
}

/**
  * @brief   Program Multi-Word data into specified address of flash.
  *          This function must be executed in SRAM.
  * @param[in]  u32Addr    Start flash address in APROM where the data chunk to be programmed into.
  *                        This address must be 16-bytes aligned to flash address.
  * @param[in]  pu32Buf    Buffer that carry the data chunk.
  * @param[in]  u32ByteLen Length of the data chunk in bytes.
  * @retval   >=0  Number of data bytes were programmed.
  * @retval   FMC_ERR_INVALID_PARAM: Invalid address or data length.
  * @retval   FMC_ERR_TIMEOUT: Program time-out.
  * @retval   FMC_ERR_PROG_FAILED: Program failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_INVALID_PARAM: Invalid address or data length.
  *           FMC_ERR_TIMEOUT: Program time-out
  *           FMC_ERR_PROG_FAILED: Program failed
  *
  * @details  If multi-word programming is interrupted during state transition,
  *           this function restores the write pointer from MPADDR and retries from
  *           the interrupted position until the chunk is completed or an error occurs.
  */
int32_t FMC_WriteMultiple(uint32_t u32Addr, const uint32_t pu32Buf[], uint32_t u32ByteLen)
{
    uint32_t i;
    uint32_t u32ByteOffset;
    uint32_t u32OnProg;
    int32_t i32RetVal = 0;
    int32_t i32TimeOutCnt;
    uint32_t u32MPStatus = 0;

    g_FMC_i32ErrCode = FMC_OK;

    /* pu32Buf must be valid, u32Addr and u32ByteLen must be multiple of 16. */
    if ((pu32Buf == (const uint32_t *)NULL) || ((u32Addr % 16U) != 0U) || ((u32ByteLen % 16U) != 0U))
    {
        g_FMC_i32ErrCode = FMC_ERR_INVALID_PARAM;
        return FMC_ERR_INVALID_PARAM;
    }

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM_MUL;

    for (u32ByteOffset = 0; u32ByteOffset < u32ByteLen; u32ByteOffset += FMC_MULTI_WORD_PROG_LEN)
    {
        uint32_t u32ChunkEnd;
        uint32_t u32BufIdx;

        u32BufIdx = u32ByteOffset / 4U;
        u32ChunkEnd = ((u32ByteOffset + FMC_MULTI_WORD_PROG_LEN) > u32ByteLen) ? u32ByteLen : (u32ByteOffset + FMC_MULTI_WORD_PROG_LEN);
        FMC->ISPADDR = u32Addr + u32ByteOffset;

        do
        {
            int32_t  i32ErrStatus;

            i32ErrStatus  = FMC_OK;

            if (u32BufIdx < (u32ChunkEnd / 4U))
            {
                u32OnProg = TRUE;
            }
            else
            {
                u32OnProg = FALSE;
                break;
            }

            FMC->MPDAT0 = pu32Buf[u32BufIdx];
            FMC->MPDAT1 = pu32Buf[u32BufIdx + 1U];
            FMC->MPDAT2 = pu32Buf[u32BufIdx + 2U];
            FMC->MPDAT3 = pu32Buf[u32BufIdx + 3U];
            i32RetVal += 16;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            u32BufIdx += 4U;

            for (i = u32BufIdx; i < (u32ChunkEnd / 4U); i += 4U)   /* Max data length is FMC_MULTI_WORD_PROG_LEN bytes (FMC_MULTI_WORD_PROG_LEN/4 words) */
            {
                i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                do
                {
                    if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0U)
                    {
                        /* ISP multi-word programming is interrupted.
                         * Resume address is derived from MPADDR and retry starts from this position.
                         */
                        FMC->ISPADDR = (FMC->MPADDR + 8U) & (~0xFU);
                        u32BufIdx = (FMC->ISPADDR - u32Addr) / 4U;
                        i32RetVal = FMC->ISPADDR - u32Addr;
                        i32ErrStatus = FMC_ERR_PROG_INTERRUPTED;
                    }

                    if (i32TimeOutCnt-- <= 0)
                    {
                        g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                        return FMC_ERR_TIMEOUT;
                    }
                } while ((FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk)) && (i32ErrStatus == FMC_OK));

                if (i32ErrStatus == FMC_OK)
                {
                    i32RetVal += 8;

                    /* Update new data for D0 and D1 */
                    FMC->MPDAT0 = pu32Buf[i];
                    FMC->MPDAT1 = pu32Buf[i + 1U];
                    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                    do
                    {
                        if ((FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk) == 0U)
                        {
                            /* ISP multi-word programming is interrupted.
                             * Resume address is derived from MPADDR and retry starts from this position.
                             */
                            FMC->ISPADDR = (FMC->MPADDR + 8U) & (~0xFU);
                            u32BufIdx = (FMC->ISPADDR - u32Addr) / 4U;
                            i32RetVal = FMC->ISPADDR - u32Addr;
                            i32ErrStatus = FMC_ERR_PROG_INTERRUPTED;
                        }

                        if (i32TimeOutCnt-- <= 0)
                        {
                            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                            return FMC_ERR_TIMEOUT;
                        }
                    } while ((FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk)) && (i32ErrStatus == FMC_OK));

                    if (i32ErrStatus == FMC_OK)
                    {
                        i32RetVal += 8;

                        /* Update new data for D2 and D3 */
                        FMC->MPDAT2 = pu32Buf[i + 2U];
                        FMC->MPDAT3 = pu32Buf[i + 3U];
                    }

                    if ((i + 4U) >= (u32ChunkEnd / 4U))
                    {
                        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                        do
                        {
                            u32MPStatus = FMC->MPSTS;

                            if (((u32MPStatus & FMC_MPSTS_MPBUSY_Msk) == 0U) && (u32MPStatus & (0xFU << FMC_MPSTS_D0_Pos)))
                            {
                                /* Final in-flight data exists but MP engine is interrupted.
                                 * Recover address and continue in the next do-while iteration.
                                 */
                                FMC->ISPADDR = (FMC->MPADDR + 8U) & (~0xFU);
                                u32BufIdx = (FMC->ISPADDR - u32Addr) / 4U;
                                i32RetVal = FMC->ISPADDR - u32Addr;
                                i32ErrStatus = FMC_ERR_PROG_INTERRUPTED;
                            }

                            if (i32TimeOutCnt-- <= 0)
                            {
                                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                                return FMC_ERR_TIMEOUT;
                            }
                        } while ((u32MPStatus & (0xFU << FMC_MPSTS_D0_Pos)) && (i32ErrStatus == FMC_OK));
                    }
                }

                if (i32ErrStatus != FMC_OK)
                {
                    if (FMC_GET_FAIL_FLAG())
                    {
                        FMC_CLR_FAIL_FLAG();
                        i32ErrStatus = FMC_ERR_PROG_FAILED;
                        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
                    }

                    break;
                }
            }

            /* Only stop retry loop when this chunk is not in interrupted-retry state. */
            if (i32ErrStatus != (int32_t)FMC_ERR_PROG_INTERRUPTED)
            {
                u32OnProg = FALSE;
                i32TimeOutCnt = FMC_TIMEOUT_WRITE;

                while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
                {
                    if (i32TimeOutCnt-- <= 0)
                    {
                        g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                        return FMC_ERR_TIMEOUT;
                    }
                }
            }
        } while (u32OnProg);
    }

    if (FMC_GET_FAIL_FLAG())
    {
        FMC_CLR_FAIL_FLAG();
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        return FMC_ERR_PROG_FAILED;
    }

    return i32RetVal;
}

/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 8192 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a page aligned address.
  * @return ISP page erase success or not.
  * @retval   FMC_OK                Success
  * @retval   FMC_ERR_ERASE_FAILED  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT       Erase time-out
  *           FMC_ERR_ERASE_FAILED  Erase failed
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_ERASE_FAILED;
        i32RetCode = FMC_ERR_ERASE_FAILED;
    }

    return i32RetCode;
}

/**
  * @brief       Read the User Configuration words.
  *
  * @param[out]  u32Config  The word buffer to store the User Configuration data.
  * @param[in]   u32Count   The word count to be read.
  *
  * @retval       0 Success
  * @retval      -1 Failed
  *
  * @details     This function is used to read the settings of user configuration.
  *              if u32Count = 1, Only CONFIG0 will be returned to the buffer specified by u32Config.
  *              if u32Count = 2, Both CONFIG0 and CONFIG1 will be returned.
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    uint32_t i;

    g_FMC_i32ErrCode = FMC_OK;

    for (i = 0u; i < u32Count; i++)
    {
        u32Config[i] = FMC_Read(FMC_CONFIG_BASE + (i * 4U));
    }

    return g_FMC_i32ErrCode;
}

/**
  * @brief Execute ISP commands to erase then write User Configuration.
  * @param[in] u32ConfigAddr   Address of config word
  * @param[in] u32ConfigVal    New config value to be written.
  * @return Success or not.
  * @retval   FMC_OK   Success
  * @retval   -1  Erase/program/read/verify failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           < 0  Errors caused by erase/program/read failed or time-out
  */
int32_t FMC_WriteConfig(uint32_t u32ConfigAddr, uint32_t u32ConfigVal)
{
    FMC_ENABLE_CFG_UPDATE();

    if (FMC_EraseConfig(u32ConfigAddr) != 0)
    {
        return -1;
    }

    if (FMC_Read(u32ConfigAddr) != 0xFFFFFFFFU)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (g_FMC_i32ErrCode != FMC_OK)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (FMC_Write(u32ConfigAddr, u32ConfigVal) != FMC_OK)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (FMC_Read(u32ConfigAddr) != u32ConfigVal)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (g_FMC_i32ErrCode != FMC_OK)
    {
        FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    FMC_DISABLE_CFG_UPDATE();
    return FMC_OK;
}

/**
  * @brief      Execute FMC_ISPCMD_CFG_ERASE command to erase a config word.
  * @param[in]  u32ConfigAddr Address of config word to be erased.
  * @return     ISP config erase success or not.
  * @retval     FMC_OK                Success
  * @retval     FMC_ERR_ERASE_FAILED  Erase failed
  *
  * @note       Global error code g_FMC_i32ErrCode
  *             FMC_ERR_TIMEOUT       Erase time-out
  *             FMC_ERR_ERASE_FAILED  Erase failed
  */
int32_t FMC_EraseConfig(uint32_t u32ConfigAddr)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD  = FMC_ISPCMD_CFG_ERASE;
    FMC->ISPADDR = u32ConfigAddr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_ERASE_FAILED;
        i32RetCode = FMC_ERR_ERASE_FAILED;
    }

    return i32RetCode;
}

/**
  * @brief     Remap Bank0/Bank1
  *
  * @param[in] u32Bank    Bank Num which will remap to.
  *
  * @retval    FMC_OK               Success
  * @retval    FMC_ERR_TIMEOUT      Program timeout
  * @retval    FMC_ERR_PROG_FAILED  Program failed
  *
  * @details  Remap Bank0/Bank1
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_RemapBank(uint32_t u32Bank)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD  = FMC_ISPCMD_BANK_REMAP;
    FMC->ISPADDR = u32Bank;
    FMC->ISPDAT  = 0x5AA55AA5UL;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    if (i32RetCode == FMC_OK)
    {
#if (NVT_DCACHE_ON == 1)
        uint32_t u32APROM_Size = FMC_APROM_SIZE;
#endif

        /* Because bank remap takes effect immediately after FMC_ISPCMD_BANK_REMAP command is done,
         * invalidate I-Cache after bank remap to ensure instruction consistency.
         */
        SCB_InvalidateICache();
#if (NVT_DCACHE_ON == 1)
        // Invalidate D-Cache after bank remap to ensure data consistency when D-Cache is enabled.
        SCB_InvalidateDCache_by_Addr((void *)FMC_APROM_BASE, (int32_t)u32APROM_Size);
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
        SCB_InvalidateDCache_by_Addr((void *)(FMC_APROM_BASE + NS_OFFSET), (int32_t)u32APROM_Size);
#endif
#endif
    }

    return i32RetCode;
}

/**
  * @brief Execute FMC_ISPCMD_BANK_ERASE command to erase a flash block.
  * @param[in]  u32BankAddr Base address of the flash bank to be erased.
  * @return ISP page erase success or not.
  * @retval   FMC_OK                Success
  * @retval   FMC_ERR_TIMEOUT       Erase time-out
  * @retval   FMC_ERR_ERASE_FAILED  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT       Erase time-out
  *           FMC_ERR_ERASE_FAILED  Erase failed
  */
int32_t FMC_Erase_Bank(uint32_t u32BankAddr)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD  = FMC_ISPCMD_BANK_ERASE;
    FMC->ISPADDR = u32BankAddr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_ERASE_FAILED;
        i32RetCode = FMC_ERR_ERASE_FAILED;
    }

    return i32RetCode;
}

/**
  * @brief     Config NPU XOM Region and only XOMR3 supports NPU XOM
  * @param[in] u32XomBase   The XOM region base address.
  * @param[in] u8XomPageCnt The XOM page count of region size.
  * @param[in] u32DRBound   The data region boundary in NPU XOM region
  *
  * @retval   FMC_OK   Success
  * @retval   1        XOM is has already actived.
  * @retval   FMC_ERR_TIMEOUT       Program timeout
  * @retval   FMC_ERR_PROG_FAILED   Program failed.
  *
  * @details  Program NPU XOM base address, XOM size (page count) and data region boundary
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT       Program timeout
  *           FMC_ERR_PROG_FAILED   Program failed
  */
int32_t FMC_ConfigNPUXOM(uint32_t u32XomBase, uint8_t u8XomPageCnt, uint32_t u32DRBound)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    i32RetCode = FMC_GetXOMState(XOMR3);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    // Set XOMR3BASE
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_XOM_BASE + (XOMR3 * 0x10U);
    FMC->ISPDAT  = u32XomBase;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    // Set XOMR3SIZE
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_XOM_BASE + ((XOMR3 * 0x10U) + 0x04U);
    FMC->ISPDAT  = u8XomPageCnt;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    // Set XOMR3DRBOUNDARY
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_XOM_BASE + ((XOMR3 * 0x10U) + 0x0CU);
    FMC->ISPDAT  = u32DRBound;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    // Write 0xA to XOMR3CTRL to active (Need chip reset to active)
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_XOM_BASE + ((XOMR3 * 0x10U) + 0x08U);
    FMC->ISPDAT  = 0xA;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
        i32RetCode = FMC_ERR_PROG_FAILED;
    }

    return i32RetCode;
}

/**
  * @brief     Config XOM Region
  * @param[in] u32XomNum    The XOM number(0~3)
  * @param[in] u32XomBase   The XOM region base address.
  * @param[in] u8XomPageCnt The XOM page count of region size.
  *
  * @retval   FMC_OK   Success
  * @retval   1   XOM is has already actived.
  * @retval   FMC_ERR_TIMEOUT       Program timeout
  * @retval   FMC_ERR_PROG_FAILED   Program failed.
  * @retval   FMC_ERR_INVALID_PARAM Invalid XOM number.
  *
  * @details  Program XOM base address and XOM size(page)
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT       Program timeout
  *           FMC_ERR_PROG_FAILED   Program failed
  *           FMC_ERR_INVALID_PARAM Invalid XOM number.
  */
int32_t FMC_ConfigXOM(uint32_t u32XomNum, uint32_t u32XomBase, uint8_t u8XomPageCnt)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32XomNum >= XOMR_CNT)
    {
        g_FMC_i32ErrCode = FMC_ERR_INVALID_PARAM;
        i32RetCode = FMC_ERR_INVALID_PARAM;
    }

    if (i32RetCode == FMC_OK)
    {
        i32RetCode = FMC_GetXOMState(u32XomNum);
    }

    if (i32RetCode == FMC_OK)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10U);
        FMC->ISPDAT  = u32XomBase;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
            i32RetCode = FMC_ERR_PROG_FAILED;
        }
    }

    if (i32RetCode == FMC_OK)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + ((u32XomNum * 0x10U) + 0x04U);
        FMC->ISPDAT  = u8XomPageCnt;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
            i32RetCode = FMC_ERR_PROG_FAILED;
        }
    }

    if (i32RetCode == FMC_OK)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + ((u32XomNum * 0x10U) + 0x08U);
        FMC->ISPDAT  = 0u;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = FMC_ERR_PROG_FAILED;
            i32RetCode = FMC_ERR_PROG_FAILED;
        }
    }

    return i32RetCode;
}

/**
  * @brief  Check the XOM is actived or not.
  *
  * @param[in] u32XomNum    The xom number(0~3).
  *
  * @retval   1   XOM is actived.
  * @retval   0   XOM is not actived.
  * @retval   FMC_ERR_INVALID_PARAM  Invalid XOM number.
  *
  * @details To get specify XOMRn(n=0~3) active status
  */
int32_t FMC_GetXOMState(uint32_t u32XomNum)
{
    if (u32XomNum >= XOMR_CNT)
    {
        return FMC_ERR_INVALID_PARAM;
    }

    return ((((FMC->XOMSTS) & 0xfUL) & (1UL << u32XomNum)) >> u32XomNum);
}

/**
  * @brief  Execute Erase XOM Region
  *
  * @param[in]  u32XomNum  The XOMRn(n=0~3)
  *
  * @return   XOM erase success or not.
  * @retval   FMC_OK                Success
  * @retval   FMC_ERR_TIMEOUT       Erase timeout
  * @retval   FMC_ERR_ERASE_FAILED  Erase failed
  * @retval   FMC_ERR_INVALID_PARAM Invalid XOM number
  *
  * @details  Execute FMC_ISPCMD_PAGE_ERASE command to erase XOM.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT       Program failed or program time-out
  *           FMC_ERR_INVALID_PARAM Invalid XOM number.
  */
int32_t FMC_EraseXOM(uint32_t u32XomNum)
{
    uint32_t u32Addr;
    int32_t  i32Active;
    int32_t  i32RetCode = FMC_OK;
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32XomNum >= XOMR_CNT)
    {
        i32RetCode = FMC_ERR_INVALID_PARAM;
    }

    if (i32RetCode == FMC_OK)
    {
        i32Active = FMC_GetXOMState(u32XomNum);

        if (i32Active)
        {
            switch (u32XomNum)
            {
                case 0U:
                    u32Addr = (FMC->XOMR0STS & 0xFFFFFF00U) >> 8U;
                    break;

                case 1U:
                    u32Addr = (FMC->XOMR1STS & 0xFFFFFF00U) >> 8U;
                    break;

                case 2U:
                    u32Addr = (FMC->XOMR2STS & 0xFFFFFF00U) >> 8U;
                    break;

                case 3U:
                    u32Addr = (FMC->XOMR3STS & 0xFFFFFF00U) >> 8U;
                    break;

                default:
                    /* Should not be here */
                    i32RetCode = FMC_ERR_INVALID_PARAM;
                    g_FMC_i32ErrCode = i32RetCode;
                    return i32RetCode;
            }

            FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPDAT  = 0x55AA03U;
            FMC->ISPTRG  = 0x1U;
#if defined (ISBEN) && (ISBEN == 1)
            __ISB();
#endif
            i32TimeOutCnt = FMC_TIMEOUT_ERASE;

            while (FMC->ISPTRG)
            {
                if (i32TimeOutCnt-- <= 0)
                {
                    i32RetCode = FMC_ERR_TIMEOUT;
                    break;
                }
            }

            /* Check ISPFF flag to know whether erase OK or fail. */
            if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
            {
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
                i32RetCode = FMC_ERR_ERASE_FAILED;
            }
        }
        else
        {
            i32RetCode = FMC_ERR_INVALID_PARAM;
        }
    }

    g_FMC_i32ErrCode = i32RetCode;
    return i32RetCode;
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t FMC_GetBootSource(void)
{
    uint32_t u32Config0;

    u32Config0 = FMC_Read(FMC_USER_CONFIG_0);

    if ((g_FMC_i32ErrCode != FMC_OK) || FMC_GET_FAIL_FLAG())
    {
        FMC_CLR_FAIL_FLAG();
        return FMC_ERR_READ_FAILED;
    }

    return (((u32Config0 & BIT7) == 0U) ? 1U : 0U);
}

/**
  * @brief  Read the 64-bits data from the specified OTP.
  * @param[in] u32OtpNum    The OTP number.
  * @param[in] pu32LowWord   Low word of the 64-bits data.
  * @param[in] pu32HighWord  High word of the 64-bits data.
  * @retval   FMC_OK   Success
  * @retval   -1  Read failed.
  * @retval   -2  Invalid OTP number.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read failed or time-out
  *           -2  Invalid OTP number
  */
int32_t FMC_ReadOTP(uint32_t u32OtpNum, uint32_t *pu32LowWord, uint32_t *pu32HighWord)
{
    int32_t i32RetCode = FMC_OK;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32OtpNum >= FMC_OTP_ENTRY_CNT)
    {
        g_FMC_i32ErrCode = -2;
        i32RetCode = -2;
    }

    if (i32RetCode == FMC_OK)
    {
        int32_t i32TimeOutCnt;

        FMC->ISPCMD  = FMC_ISPCMD_READ_64;
        FMC->ISPADDR = FMC_OTP_BASE + (u32OtpNum * 8UL);
        FMC->ISPDAT  = 0x0UL;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_READ;

        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32RetCode = -1;
        }
        else
        {
            *pu32LowWord = FMC->MPDAT0;
            *pu32HighWord = FMC->MPDAT1;
        }
    }

    return i32RetCode;
}

/**
  * @brief Program a 64-bits data to the specified OTP.
  * @param[in] u32OtpNum    The OTP number.
  * @param[in] u32LowWord   Low word of the 64-bits data.
  * @param[in] u32HighWord  High word of the 64-bits data.
  * @retval   FMC_OK   Success
  * @retval   -1  Program failed.
  * @retval   -2  Invalid OTP number.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  *           -2  Invalid OTP number
  */
int32_t FMC_WriteOTP(uint32_t u32OtpNum, uint32_t u32LowWord, uint32_t u32HighWord)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32OtpNum >= FMC_OTP_ENTRY_CNT)
    {
        g_FMC_i32ErrCode = -2;
        i32RetCode = -2;
    }

    if (i32RetCode == FMC_OK)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_OTP_BASE + (u32OtpNum * 8UL);
        FMC->ISPDAT  = u32LowWord;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32RetCode = -1;
        }
    }

    if (i32RetCode == FMC_OK)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_OTP_BASE + (u32OtpNum * 8UL) + 4UL;
        FMC->ISPDAT  = u32HighWord;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32RetCode = -1;
        }
    }

    return i32RetCode;
}

/**
  * @brief  Lock the specified OTP.
  * @param[in] u32OtpNum    The OTP number.
  * @retval    FMC_OK  Success
  * @retval   -1  Failed to write OTP lock bits.
  * @retval   -2  Invalid OTP number.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Failed to write OTP lock bits or write time-out
  *           -2  Invalid OTP number
  */
int32_t FMC_LockOTP(uint32_t u32OtpNum)
{
    int32_t i32RetCode = FMC_OK;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32OtpNum >= FMC_OTP_ENTRY_CNT)
    {
        g_FMC_i32ErrCode = -2;
        i32RetCode = -2;
    }

    if (i32RetCode == FMC_OK)
    {
        int32_t i32TimeOutCnt;

        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_OTP_BASE + 0x800UL + (u32OtpNum * 4UL);
        FMC->ISPDAT  = 0UL;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32RetCode = -1;
        }
    }

    return i32RetCode;
}

/**
  * @brief  Check the OTP is locked or not.
  * @param[in] u32OtpNum    The OTP number.
  * @retval   1   OTP is locked.
  * @retval   0   OTP is not locked.
  * @retval   -1  Failed to read OTP lock bits.
  * @retval   -2  Invalid OTP number.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Failed to read OTP lock bits or read time-out
  *           -2  Invalid OTP number
  */
int32_t FMC_IsOTPLocked(uint32_t u32OtpNum)
{
    int32_t i32RetCode = FMC_OK;

    g_FMC_i32ErrCode = FMC_OK;

    if (u32OtpNum >= FMC_OTP_ENTRY_CNT)
    {
        g_FMC_i32ErrCode = -2;
        i32RetCode = -2;
    }

    if (i32RetCode == FMC_OK)
    {
        int32_t i32TimeOutCnt;

        FMC->ISPCMD  = FMC_ISPCMD_READ;
        FMC->ISPADDR = FMC_OTP_BASE + 0x800UL + (u32OtpNum * 4UL);
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_READ;

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                i32RetCode = FMC_ERR_TIMEOUT;
                break;
            }
        }

        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32RetCode = -1;
        }
        else
        {
            if (FMC->ISPDAT != 0xFFFFFFFFUL)
            {
                g_FMC_i32ErrCode = -1;
                i32RetCode = 1;   /* Lock work was progrmmed. OTP was locked. */
            }
        }
    }

    return i32RetCode;
}

/**
  * @brief     Config Secure Conceal Region
  * @param[in] u32Base          Secure conceal base address. It must be page alignment and cannot be first page of APROM
  * @param[in] u32PageCnt       Secure conceal page count.
  * @param[in] bActiveEnable    TRUE:  Allowed to active secure conceal region
  *                             FALSE: Secure conceal region cannot be activated.
  *
  * @retval   FMC_OK                     Success
  * @retval   FMC_ERR_TIMEOUT            Operation timeout
  * @retval   FMC_ERR_SC_INVALID_BASE    Invalid secure concel base address.
  * @retval   FMC_ERR_SC_INVALID_PAGECNT Invalid secure concel page count.
  * @retval   FMC_ERR_SC_ENABLED         Secure concel is enabled and cannot be modified.
  *
  * @details  Program secure conceal base address, page count and active enable.
  */
int32_t  FMC_ConfigSecureConceal(uint32_t u32Base, uint32_t u32PageCnt, uint32_t bActiveEnable)
{
    int32_t  i32RetCode = FMC_OK;
    uint32_t u32Config6;

    if ((u32Base == FMC_APROM_BASE) || (u32Base % FMC_FLASH_PAGE_SIZE) != 0U)
    {
        return FMC_ERR_SC_INVALID_BASE;
    }

    if (u32PageCnt == 0U)
    {
        return FMC_ERR_SC_INVALID_PAGECNT;
    }

    u32Config6 = FMC_Read(FMC_USER_CONFIG_6);

    if (u32Config6 != 0xFFFFFFFFU)
    {
        return FMC_ERR_SC_ENABLED;
    }

    if ((FMC_Read(FMC_USER_CONFIG_4) == u32Base) && (FMC_Read(FMC_USER_CONFIG_5) == u32PageCnt))
    {
        return FMC_OK;
    }

    i32RetCode = FMC_EraseConfig(FMC_USER_CONFIG_4);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    i32RetCode = FMC_EraseConfig(FMC_USER_CONFIG_5);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    i32RetCode = FMC_EraseConfig(FMC_USER_CONFIG_6);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    i32RetCode = FMC_Write(FMC_USER_CONFIG_4, u32Base);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    i32RetCode = FMC_Write(FMC_USER_CONFIG_5, u32PageCnt);

    if (i32RetCode != FMC_OK)
    {
        return i32RetCode;
    }

    if (bActiveEnable)
    {
        i32RetCode = FMC_Write(FMC_USER_CONFIG_6, bActiveEnable);

        if (i32RetCode != FMC_OK)
        {
            return i32RetCode;
        }
    }

    return i32RetCode;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @return Success or not.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  * @retval   Others      CRC32 value
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT        Run/Read check sum time-out failed
  *           FMC_ERR_INVALID_PARAM  u32Addr or u32count must be aligned with 8 KB page alignment
  */
uint32_t  FMC_GetChkSum(uint32_t u32Addr, uint32_t u32count)
{
    int32_t i32RetCode;

    g_FMC_i32ErrCode = FMC_OK;

    if ((u32Addr % FMC_FLASH_PAGE_SIZE) || (u32count % FMC_FLASH_PAGE_SIZE))
    {
        g_FMC_i32ErrCode = FMC_ERR_INVALID_PARAM;
        i32RetCode = FMC_ERR_INVALID_PARAM;
    }
    else
    {
        int32_t i32TimeOutCnt;

        FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT  = u32count;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_CHKSUM;

        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                return FMC_ERR_TIMEOUT;
            }
        }

        FMC->ISPCMD  = FMC_ISPCMD_READ_CKS;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_CHKSUM;

        while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                return FMC_ERR_TIMEOUT;
            }
        }

        return FMC->ISPDAT;
    }

    return i32RetCode;
}

/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @retval   READ_ALLONE_YES      The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT  Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
  */

uint32_t  FMC_CheckAllOne(uint32_t u32Addr, uint32_t u32count)
{
    int32_t i32RetCode = READ_ALLONE_CMD_FAIL;
    int32_t i32TimeOutCnt0;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPSTS = FMC_ISPSTS_ALLONE_Msk;   /* Clear check all one bit */

    FMC->ISPCMD   = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR  = u32Addr;
    FMC->ISPDAT   = u32count;
    FMC->ISPTRG   = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt0 = FMC_TIMEOUT_CHKALLONE;

    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if (i32TimeOutCnt0-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if (g_FMC_i32ErrCode == FMC_OK)
    {
        int32_t i32TimeOutCnt1 = FMC_TIMEOUT_CHKALLONE;

        do
        {
            FMC->ISPCMD  = FMC_ISPCMD_READ_ALL1;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

            i32TimeOutCnt0 = FMC_TIMEOUT_CHKALLONE;

            while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
            {
                if (i32TimeOutCnt0-- <= 0)
                {
                    g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                    break;
                }
            }

            if (i32TimeOutCnt1-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            }
        } while ((FMC->ISPDAT == 0UL) && (g_FMC_i32ErrCode == FMC_OK));
    }

    if (g_FMC_i32ErrCode == FMC_OK)
    {
        if (FMC->ISPDAT == READ_ALLONE_YES)
        {
            i32RetCode = READ_ALLONE_YES;
        }
        else if (FMC->ISPDAT == READ_ALLONE_NOT)
        {
            i32RetCode = READ_ALLONE_NOT;
        }
        else
        {
            g_FMC_i32ErrCode = FMC_ERR_READ_FAILED;
        }
    }

    return i32RetCode;
}

/** @} end of group FMC_EXPORTED_FUNCTIONS */
/** @} end of group FMC_Driver */
/** @} end of group Standard_Driver */
