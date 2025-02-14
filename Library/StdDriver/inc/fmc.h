/**************************************************************************//**
 * @file     fmc.h
 * @version  V1.00
 * @brief    FMC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __FMC_H__
#define __FMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup FMC_Driver FMC Driver
    @{
*/

/** @addtogroup FMC_EXPORTED_CONSTANTS FMC Exported Constants
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_APROM_BASE          0x00100000UL                            /*!< APROM base address                 \hideinitializer */
#define FMC_APROM_BANK_SIZE     0x00100000UL                            /*!< APROM Bank Size                    \hideinitializer */
#define FMC_APROM_BANK0_END     (FMC_APROM_BASE + FMC_APROM_BANK_SIZE)  /*!< APROM bank0 end address            \hideinitializer */
#define FMC_APROM_BANK0_BASE    FMC_APROM_BASE                          /*!< APROM bank0 base address           \hideinitializer */
#define FMC_APROM_BANK1_BASE    FMC_APROM_BANK0_END                     /*!< APROM bank1 base address           \hideinitializer */
#define FMC_APROM_SIZE          (FMC_APROM_BANK_SIZE * 2)               /*!< APROM Size                         \hideinitializer */
#define FMC_APROM_END           (FMC_APROM_BASE + FMC_APROM_SIZE)       /*!< APROM end address                  \hideinitializer */

#define FMC_LDROM_BASE          0x0F100000UL                            /*!< LDROM base address                 \hideinitializer */
#define FMC_LDROM_SIZE          0x2000UL                                /*!< LDROM Size (8 Kbytes)              \hideinitializer */
#define FMC_LDROM_END           (FMC_LDROM_BASE + FMC_LDROM_SIZE)       /*!< LDROM end address                  \hideinitializer */

#define FMC_XOM_BASE            0x0F300040UL                            /*!< XOM  Base Address                  \hideinitializer */
#define FMC_XOM_SIZE            0x0F300044UL                            /*!< XOM  Size                          \hideinitializer */
#define FMC_XOM_CTRL            0x0F300048UL                            /*!< XOM  Control                       \hideinitializer */
#define FMC_XOMR0_BASE          FMC_XOM_BASE                            /*!< XOMR 0 Base Address                \hideinitializer */
#define FMC_XOMR1_BASE          (FMC_XOM_BASE + 0x10UL)                 /*!< XOMR 1 Base Address                \hideinitializer */
#define FMC_XOMR2_BASE          (FMC_XOM_BASE + 0x20UL)                 /*!< XOMR 2 Base Address                \hideinitializer */
#define FMC_XOMR3_BASE          (FMC_XOM_BASE + 0x30UL)                 /*!< XOMR 3 Base Address                \hideinitializer */

#define FMC_CONFIG_BASE         0x0F300000UL                            /*!< User Configuration address         \hideinitializer */
#define FMC_USER_CONFIG_0       FMC_CONFIG_BASE                         /*!< User Config 0 address              \hideinitializer */
#define FMC_USER_CONFIG_1       (FMC_CONFIG_BASE + 0x4UL)               /*!< User Config 1 address              \hideinitializer */
#define FMC_USER_CONFIG_2       (FMC_CONFIG_BASE + 0x8UL)               /*!< User Config 2 address              \hideinitializer */
#define FMC_USER_CONFIG_3       (FMC_CONFIG_BASE + 0xCUL)               /*!< User Config 3 address              \hideinitializer */
#define FMC_USER_CONFIG_4       (FMC_CONFIG_BASE + 0x10UL)              /*!< User Config 4 address              \hideinitializer */
#define FMC_USER_CONFIG_5       (FMC_CONFIG_BASE + 0x14UL)              /*!< User Config 5 address              \hideinitializer */
#define FMC_USER_CONFIG_6       (FMC_CONFIG_BASE + 0x18UL)              /*!< User Config 6 address              \hideinitializer */
#define FMC_USER_CONFIG_8       (FMC_CONFIG_BASE + 0x20UL)              /*!< User Config 6 address              \hideinitializer */
#define FMC_USER_CONFIG_9       (FMC_CONFIG_BASE + 0x24UL)              /*!< User Config 6 address              \hideinitializer */
#define FMC_USER_CONFIG_10      (FMC_CONFIG_BASE + 0x28UL)              /*!< User Config 6 address              \hideinitializer */
#define FMC_SCRLOCK_BASE        (FMC_CONFIG_BASE + 0x2CUL)              /*!< Secure Region Lock base address    \hideinitializer */
#define FMC_NSCBA_BASE          (FMC_CONFIG_BASE + 0x30UL)              /*!< Non-Secure base address            \hideinitializer */
#define FMC_ARLOCK_BASE         (FMC_CONFIG_BASE + 0x34UL)              /*!< All Region Lock base address       \hideinitializer */

#define FMC_OTP_BASE            0x0F310000UL                            /*!< OTP flash base address             \hideinitializer */
#define FMC_FLASH_PAGE_SIZE     0x2000UL                                /*!< Flash Page Size (8K bytes)         \hideinitializer */
#define FMC_VECMAP_SIZE         0x400UL                                 /*!< VECMAP Size (1024 bytes)           \hideinitializer */
#define FMC_PAGE_ADDR_MASK      0xFFFFE000UL                            /*!< Flash page address mask            \hideinitializer */
#define FMC_MULTI_WORD_PROG_LEN 512UL                                   /*!< Length of multi-word program.      \hideinitializer */
#define FMC_APWPROT_BLOCK_SIZE  0x8000UL                                /*!< APWPROT block size (32K bytes)     \hideinitializer */
#define FMC_OTP_ENTRY_CNT       256UL                                   /*!< OTP entry number                   \hideinitializer */
#define FMC_BANK_SIZE           FMC_APROM_BANK_SIZE                     /*!< For backward compatible            \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  XOM region number constant definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define XOMR0       0UL                             /*!< XOM region 0           */
#define XOMR1       1UL                             /*!< XOM region 1           */
#define XOMR2       2UL                             /*!< XOM region 2           */
#define XOMR3       3UL                             /*!< XOM region 3           */
#define XOMR_CNT    4UL                             /*!< Total XOM region count */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define IS_BOOT_FROM_LDROM      0x1UL           /*!< ISPCTL setting to select to boot from LDROM */
#define IS_BOOT_FROM_APROM      0x0UL           /*!< ISPCTL setting to select to boot from APROM */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_ISPCMD_READ         0x00UL          /*!< ISP Command: Read flash word         \hideinitializer */
#define FMC_ISPCMD_READ_UID     0x04UL          /*!< ISP Command: Read Unique ID          \hideinitializer */
#define FMC_ISPCMD_READ_ALL1    0x08UL          /*!< ISP Command: Read all-one result     \hideinitializer */
#define FMC_ISPCMD_READ_CID     0x0BUL          /*!< ISP Command: Read Company ID         \hideinitializer */
#define FMC_ISPCMD_READ_DID     0x0CUL          /*!< ISP Command: Read Device ID          \hideinitializer */
#define FMC_ISPCMD_READ_CKS     0x0DUL          /*!< ISP Command: Read checksum           \hideinitializer */
#define FMC_ISPCMD_PROGRAM      0x21UL          /*!< ISP Command: Write flash word        \hideinitializer */
#define FMC_ISPCMD_PAGE_ERASE   0x22UL          /*!< ISP Command: Page Erase Flash        \hideinitializer */
#define FMC_ISPCMD_BANK_ERASE   0x23UL          /*!< ISP Command: Erase Flash bank 0 or 1 \hideinitializer */
#define FMC_ISPCMD_CFG_ERASE    0x24UL          /*!< ISP Command: Erase config word       \hideinitializer */
#define FMC_ISPCMD_MASS_ERASE   0x26UL          /*!< ISP Command: Mass erase              \hideinitializer */
#define FMC_ISPCMD_PROGRAM_MUL  0x27UL          /*!< ISP Command: Multuple word program   \hideinitializer */
#define FMC_ISPCMD_RUN_ALL1     0x28UL          /*!< ISP Command: Run all-one verification \hideinitializer */
#define FMC_ISPCMD_BANK_REMAP   0x2CUL          /*!< ISP Command: Bank Remap              \hideinitializer */
#define FMC_ISPCMD_RUN_CKS      0x2DUL          /*!< ISP Command: Run checksum calculation \hideinitializer */
#define FMC_ISPCMD_VECMAP       0x2EUL          /*!< ISP Command: Vector Page Remap       \hideinitializer */
#define FMC_ISPCMD_READ_64      0x40UL          /*!< ISP Command: Read double flash word  \hideinitializer */
#define FMC_ISPCMD_PROGRAM_64   0x61UL          /*!< ISP Command: Write double flash word \hideinitializer */


#define READ_ALLONE_YES         0xA11FFFFFUL    /*!< Check-all-one result is all one.     \hideinitializer */
#define READ_ALLONE_NOT         0xA1100000UL    /*!< Check-all-one result is not all one. \hideinitializer */
#define READ_ALLONE_CMD_FAIL    0xFFFFFFFFUL    /*!< Check-all-one command failed.        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* FMC Time-out Handler Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_TIMEOUT_READ        (SystemCoreClock >> 3) /*!< Read command time-out 125 ms        \hideinitializer */
#define FMC_TIMEOUT_WRITE       (SystemCoreClock >> 3) /*!< Write command time-out 125 ms       \hideinitializer */
#define FMC_TIMEOUT_ERASE       (SystemCoreClock >> 2) /*!< Erase command time-out 250 ms       \hideinitializer */
#define FMC_TIMEOUT_CHKSUM      (SystemCoreClock << 1) /*!< Get checksum command time-out 2 s   \hideinitializer */
#define FMC_TIMEOUT_CHKALLONE   (SystemCoreClock << 1) /*!< Check-all-one command time-out 2 s  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* FMC Error Code Constant Definitions                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_OK                      (0UL)   /*!< FMC operation OK                         */
#define FMC_ERR_TIMEOUT             (-1UL)  /*!< FMC operation timeout                    */
#define FMC_ERR_READ_FAILED         (-2UL)  /*!< FMC read error                           */
#define FMC_ERR_PROG_FAILED         (-2UL)  /*!< FMC program error                        */
#define FMC_ERR_ERASE_FAILED        (-2UL)  /*!< FMC erase error                          */
#define FMC_ERR_INVALID_PARAM       (-3UL)  /*!< FMC operation timeout                    */
#define FMC_ERR_SC_ENABLED          (-4UL)  /*!< FMC secure conceal is enabled            */
#define FMC_ERR_SC_INVALID_BASE     (-5UL)  /*!< FMC invalid secure conceal base address (Must be page alignment and cannot set first page of APROM */
#define FMC_ERR_SC_INVALID_PAGECNT  (-6UL)  /*!< FMC invalid secure conceal page count    */
/** @} end of group FMC_EXPORTED_CONSTANTS */


/** @addtogroup FMC_EXPORTED_MACROS FMC Exported Macros
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define FMC_SET_APROM_BOOT()         (FMC->ISPCTL &= ~FMC_ISPCTL_BS_Msk)                       /*!< Select booting from APROM  \hideinitializer */
#define FMC_SET_LDROM_BOOT()         (FMC->ISPCTL |= FMC_ISPCTL_BS_Msk)                        /*!< Select booting from LDROM  \hideinitializer */
#define FMC_ENABLE_AP_UPDATE()       (FMC->ISPCTL |=  FMC_ISPCTL_APUEN_Msk)                    /*!< Enable APROM update        \hideinitializer */
#define FMC_DISABLE_AP_UPDATE()      (FMC->ISPCTL &= ~FMC_ISPCTL_APUEN_Msk)                    /*!< Disable APROM update       \hideinitializer */
#define FMC_ENABLE_CFG_UPDATE()      (FMC->ISPCTL |=  FMC_ISPCTL_CFGUEN_Msk)                   /*!< Enable User Config update  \hideinitializer */
#define FMC_DISABLE_CFG_UPDATE()     (FMC->ISPCTL &= ~FMC_ISPCTL_CFGUEN_Msk)                   /*!< Disable User Config update \hideinitializer */
#define FMC_ENABLE_LD_UPDATE()       (FMC->ISPCTL |=  FMC_ISPCTL_LDUEN_Msk)                    /*!< Enable LDROM update        \hideinitializer */
#define FMC_DISABLE_LD_UPDATE()      (FMC->ISPCTL &= ~FMC_ISPCTL_LDUEN_Msk)                    /*!< Disable LDROM update       \hideinitializer */
#define FMC_ENABLE_ISP()             (FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk)                    /*!< Enable ISP function        \hideinitializer */
#define FMC_DISABLE_ISP()            (FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk)                    /*!< Disable ISP function       \hideinitializer */
#define FMC_GET_FAIL_FLAG()          ((FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk) ? TRUE : FALSE)     /*!< Get ISP fail flag          \hideinitializer */
#define FMC_CLR_FAIL_FLAG()          (FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk)                     /*!< Clear ISP fail flag        \hideinitializer */
#define FMC_ENABLE_APWPROT(u32Bank, u32BlockMask)   (FMC->APWPROT[(u32Bank) & 0x1] |=  (u32BlockMask))    /*!< Enable APWPROT Block       \hideinitializer */
#define FMC_DISABLE_APWPROT(u32Bank, u32BlockMask)  (FMC->APWPROT[(u32Bank) & 0x1] &= ~(u32BlockMask))    /*!< Disable APWPROT Block      \hideinitializer */
#define FMC_IS_APWPROT(u32Bank, u32BlockMask)       (FMC->APWPROT[(u32Bank) & 0x1] & (u32BlockMask))      /*!< Is APWPROT Block enabled   \hideinitializer */
#define FMC_LOCK_APWPROT(u32Bank)      (FMC->APWPKEEP = (FMC->APWPKEEP & ~(0xFFFF << ((u32Bank) * 16))) | (0x55AA << ((u32Bank) * 16)))   /*!< Lock APWPROT (Need chip reset to unlock) \hideinitializer */
#define FMC_IS_APWPROT_LCOKED(u32Bank) ((((FMC->APWPKEEP >> ((u32Bank) * 16)) & 0xFFFF) == 0x55AA) ? TRUE: FALSE)                         /*!< Is APWPROT locked  \hideinitializer */
#define FMC_SET_SC_ACTIVE()            (FMC->SCACT  |= FMC_SCACT_SCACT_Msk)                          /*!< Activate secure conceal function (Need chip reset to deactivate) \hideinitializer */
#define FMC_GET_SC_ACTIVE()            ((FMC->SCACT & FMC_SCACT_SCACT_Msk) ? TRUE : FALSE)           /*!< Get secure conceal function active flag \hideinitializer */
/** @} end of group FMC_EXPORTED_MACROS */

extern int32_t  g_FMC_i32ErrCode;           /*!< FMC global error code                    */

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
  * @brief Enable FMC ISP function
  */
__STATIC_INLINE void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Disable FMC ISP function.
  */
__STATIC_INLINE void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Enable FMC ISP INT function.
  * @note  Workaround: Read ISPSTS once after trigger FMC_ISPTRG.ISPGO[0] to get ISPFF interrupt.
  */
__STATIC_INLINE void FMC_EnableINT(void)
{
    FMC->ISPCTL |= FMC_ISPCTL_INTEN_Msk;
}

/**
  * @brief Disable FMC ISP INT function.
  */
__STATIC_INLINE void FMC_DisableINT(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_INTEN_Msk;
}

/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT  Read time-out
  */
__STATIC_INLINE uint32_t FMC_ReadCID(void)
{
    int32_t i32TimeOutCnt = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    FMC->ISPADDR = 0x0u;                         /* Must keep 0x0 when read CID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)   /* Waiting for ISP Done */
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
  * @brief    Read product ID
  * @param    None
  * @return   The product ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  This function is used to read product ID.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           FMC_ERR_TIMEOUT  Read time-out
  */
__STATIC_INLINE uint32_t FMC_ReadPID(void)
{
    int32_t i32TimeOutCnt = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04u;                       /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)  /* Waiting for ISP Done */
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
 * @brief       Read Unique ID
 * @param[in]   u8Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 * @return      The 32-bit unique ID data of specified UID index. 0xFFFFFFFF means read failed.
 * @details     To read out 96-bit Unique ID.
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              FMC_ERR_TIMEOUT  Read time-out
 */
__STATIC_INLINE uint32_t FMC_ReadUID(uint8_t u8Index)
{
    int32_t i32TimeOutCnt = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;
    FMC->ISPADDR = ((uint32_t)u8Index << 2u);
    FMC->ISPDAT = 0u;
    FMC->ISPTRG = 0x1u;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)  /* Waiting for ISP Done */
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
  * @brief      To read UCID
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index
  * @details    This function is used to read unique chip ID (UCID). 0xFFFFFFFF means read failed.
  *
  * @note       Global error code g_FMC_i32ErrCode
  *             FMC_ERR_TIMEOUT  Read time-out
  */
__STATIC_INLINE uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t i32TimeOutCnt = FMC_TIMEOUT_READ;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = (0x04u * u32Index) + 0x10u;    /* The UCID is at offset 0x10 with word alignment. */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)    /* Waiting for ISP Done */
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}

/**
 * @brief       Set vector mapping address
 * @param[in]   u32PageAddr  The page address to remap to address 0x0. The address must be page alignment.
 * @return      To set VECMAP to remap specified page address to 0x0.
 * @details     This function is used to set VECMAP to map specified page to vector page (0x0).
 * @retval      0   Success
 * @retval      FMC_ERR_TIMEOUT  Failed
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              FMC_ERR_TIMEOUT  Command time-out
 */
__STATIC_INLINE int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t i32TimeOutCnt = FMC_TIMEOUT_WRITE;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD  = FMC_ISPCMD_VECMAP;  /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;       /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG  = 0x1u;               /* Trigger to start ISP procedure */

    while (FMC->ISPTRG)               /* Waiting for ISP Done */
    {
        if (i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            return FMC_ERR_TIMEOUT;
        }
    }

    return 0;
}

/**
 * @brief       Get current vector mapping address.
 * @param       None
 * @return      The current vector mapping address.
 * @details     To get VECMAP value which is the page address for remapping to vector page (0x0).
 * @note
 *              VECMAP only valid when new IAP function is enabled. (CBS = 10'b or 00'b)
 */
__STATIC_INLINE uint32_t FMC_GetVECMAP(void)
{
    return (FMC->ISPSTS & FMC_ISPSTS_VECMAP_Msk);
}

/**
 * @brief       Get selected flash bank index.
 * @param       None
 * @return      The selected flash bank index. 0: Bank0, 1: Bank1.
 */
__STATIC_INLINE uint32_t FMC_GetBankIdx(void)
{
    return ((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
}

/**
 * @brief       Check if bank remap function is enabled or not.
 * @param       None
 * @return      0: Bank remap function is disabled.
 *              1: Bank remap function is enabled.
 */
__STATIC_INLINE uint32_t FMC_IsBankRemapEnabled(void)
{
    return ((FMC->ISPSTS & FMC_ISPSTS_MIRBOUND_Msk) >> FMC_ISPSTS_MIRBOUND_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern uint32_t FMC_Read(uint32_t u32Addr);
extern int32_t  FMC_Read_64(uint32_t u32Addr, uint32_t *pu32Data0, uint32_t *pu32Data1);
extern int32_t  FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t  FMC_Write8Bytes(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1);
extern int32_t  FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32ByteLen);
extern int32_t  FMC_Erase(uint32_t u32PageAddr);
extern int32_t  FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t  FMC_WriteConfig(uint32_t u32ConfigAddr, uint32_t u32ConfigVal);
extern int32_t  FMC_EraseConfig(uint32_t u32ConfigAddr);
extern int32_t  FMC_RemapBank(uint32_t u32Bank);
extern int32_t  FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t  FMC_ConfigNPUXOM(uint32_t u32XomBase, uint8_t u8XomPageCnt, uint32_t u32DRBound);
extern int32_t  FMC_ConfigXOM(uint32_t u32XomNum, uint32_t u32XomBase, uint8_t u8XomPageCnt);
extern int32_t  FMC_GetXOMState(uint32_t u32XomNum);
extern int32_t  FMC_EraseXOM(uint32_t u32XomNum);
extern int32_t  FMC_GetBootSource(void);
extern int32_t  FMC_ReadOTP(uint32_t u32OtpNum, uint32_t *pu32LowWord, uint32_t *pu32HighWord);
extern int32_t  FMC_WriteOTP(uint32_t u32OtpNum, uint32_t u32LowWord, uint32_t u32HighWord);
extern int32_t  FMC_LockOTP(uint32_t u32OtpNum);
extern int32_t  FMC_IsOTPLocked(uint32_t u32OtpNum);
extern int32_t  FMC_ConfigSecureConceal(uint32_t u32Base, uint32_t u32PageCnt, uint32_t bActiveEnable);
extern uint32_t FMC_GetChkSum(uint32_t u32Addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32Addr, uint32_t u32count);

/** @} end of group FMC_EXPORTED_FUNCTIONS */
/** @} end of group FMC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif   /* __FMC_H__ */
