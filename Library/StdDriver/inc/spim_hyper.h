/**************************************************************************//**
 * @file    spim_hyper.h
 * @version V1.00
 * @brief   SPIM_HYPER driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __SPIM_HYPER_H__
#define __SPIM_HYPER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPIM_HYPER_Driver SPIM_HYPER Driver
  @{
*/

/** @addtogroup SPIM_HYPER_EXPORTED_CONSTANTS SPIM_HYPER Exported Constants
  @{
*/
// TESTCHIP_ONLY
#define SPIM_HYPER_DMM0_SADDR               (0x80000000UL)  /*!< SPIM0 DMM mode memory map base secure address */
#define SPIM_HYPER_DMM0_NSADDR              (0x90000000UL)  /*!< SPIM1 DMM mode memory map base non secure address */

//#define SPIM_HYPER_DMM0_SADDR               (0x82000000UL)  /*!< SPIM0 DMM mode memory map base secure address */
//#define SPIM_HYPER_DMM0_NSADDR              (0x92000000UL)  /*!< SPIM1 DMM mode memory map base non secure address */

// TESTCHIP_ONLY
#define SPIM_HYPER_DMM1_SADDR               (0x82000000UL)  /*!< SPIM1 DMM mode memory map base secure address */
#define SPIM_HYPER_DMM1_NSADDR              (0x92000000UL)  /*!< SPIM1 DMM mode memory map base non secure address */

#if defined (SCU_INIT_D0PNS2_VAL) && (SCU_INIT_D0PNS2_VAL & SCU_D0PNS2_SPIM0_Msk)
#define SPIM_HYPER_DMM0_ADDR                SPIM_HYPER_DMM0_NSADDR
#else
#define SPIM_HYPER_DMM0_ADDR                SPIM_HYPER_DMM0_SADDR
#endif

#if defined (SCU_INIT_D0PNS2_VAL) && (SCU_INIT_D0PNS2_VAL & SCU_D0PNS2_SPIM1_Msk) // TESTCHIP_ONLY
#define SPIM_HYPER_DMM1_ADDR                SPIM_HYPER_DMM1_NSADDR
#else
#define SPIM_HYPER_DMM1_ADDR                SPIM_HYPER_DMM1_SADDR
#endif

#define SPIM_HYPER_DMM_SIZE                 (0x2000000UL)       /*!< DMM mode memory mapping size */

#define SPIM_HYPER_MAX_LATENCY              (0x20)              /*!< Maximum DLL training number */

#define SPIM_HYPER_OP_ENABLE                (0x01UL)            /* SPIM_HYPER Operation Enable */
#define SPIM_HYPER_OP_DISABLE               (0x00UL)            /* SPIM_HYPER Operation Disable */

/* SPIM_HYPER Wait State Timeout Counter. */
#define SPIM_HYPER_TIMEOUT                  0xFFFF //SystemCoreClock     /*!< SPIM_HYPER time-out counter (1 second time-out) */

/* SPIM_HYPER Define Error Code */
#define SPIM_HYPER_OK                       ( 0L)           /*!< SPIM_HYPER operation OK */
#define SPIM_HYPER_ERR_FAIL                 (-1L)           /*!< SPIM_HYPER operation failed */
#define SPIM_HYPER_ERR_TIMEOUT              (-2L)           /*!< SPIM_HYPER operation abort due to timeout error */

/* SPIM Hyper Operation Mode */
#define SPIM_HYPER_OPMODE_IO                (0UL)           /*!< SPIM_CTL0: I/O Mode */
#define SPIM_HYPER_OPMODE_PAGEWRITE         (1UL)           /*!< SPIM_CTL0: Page Write Mode */
#define SPIM_HYPER_OPMODE_PAGEREAD          (2UL)           /*!< SPIM_CTL0: Page Read Mode */
#define SPIM_HYPER_OPMODE_DIRECTMAP         (3UL)           /*!< SPIM_CTL0: Direct Map Mode */

/* SPIM Hyper Mode Command */
#define SPIM_HYPER_CMD_IDLE                 (0x00000000)    /* Hyper Bus interface is Idle. */
#define SPIM_HYPER_CMD_RESET                (0x00000001)    /* Reset Hyper Bus Devices. */
#define SPIM_HYPER_CMD_READ_HRAM_REGISTER   (0x00000002)    /* Read Hyper RAM Regsiter (16-Bit, Read Data[15:0]. */
#define SPIM_HYPER_CMD_EXIT_HS_PD           (0x00000005)    /* Exit From Hybrid Sleep and deep power down. */
#define SPIM_HYPER_CMD_WRITE_HRAM_REGISTER  (0x00000007)    /* Write Hyper RAM Regsiter (16-Bit, Write Data[15:0]. */
#define SPIM_HYPER_CMD_READ_1_WORD          (0x00000008)    /* Read 1 word (Read Data[15:0]) from Hyper Bus Devices. */
#define SPIM_HYPER_CMD_READ_2_WORD          (0x00000009)    /* Read 2 word (Read Data[31:0]) from Hyper Bus Devices. */
#define SPIM_HYPER_CMD_WRITE_1_BYTE         (0x0000000C)    /* Write 1 Byte (Write Data[7:0]) to Hyper Bus Devices. */
#define SPIM_HYPER_CMD_WRITE_2_BYTE         (0x0000000D)    /* Write 2 Byte (Write Data[15:0]) to Hyper Bus Devices. */
#define SPIM_HYPER_CMD_WRITE_3_BYTE         (0x0000000E)    /* Write 3 Byte (Write Data[23:0]) to Hyper Bus Devices. */
#define SPIM_HYPER_CMD_WRITE_4_BYTE         (0x0000000F)    /* Write 4 Byte (Write Data[31:0]) to Hyper Bus Devices. */

/* HyperRAM Register Address */
#define HYPERRAM_ID_REG0                    (0x00000000)    /* Hyper RAM Identification Register 0. */
#define HYPERRAM_ID_REG1                    (0x00000002)    /* Hyper RAM Identification Register 1. */
#define HYPERRAM_CONFIG_REG0                (0x00001000)    /* Hyper RAM Configuration Register 0. */
#define HYPERRAM_CONFIG_REG1                (0x00001002)    /* Hyper RAM Configuration Register 1. */

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select Setup Time to Next CK Rising Edge
    10 = 3.5 HCLK cycles.
    11 = 4.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSST_3_5_HCLK    (0x2)
#define SPIM_HYPER_CSST_4_5_HCLK    (0x3)

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select Hold Time After CK Falling Edge
    00 = 0.5 HCLK cycles.
    01 = 1.5 HCLK cycles.
    10 = 2.5 HCLK cycles.
    11 = 3.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSH_0_5_HCLK    (0x0)
#define SPIM_HYPER_CSH_1_5_HCLK    (0x1)
#define SPIM_HYPER_CSH_2_5_HCLK    (0x2)
#define SPIM_HYPER_CSH_3_5_HCLK    (0x3)

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select High between Transaction
    10 = 0.5 HCLK cycles.
    ...
    1111 = 3.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSHI_2_HCLK    (0x02)
#define SPIM_HYPER_CSHI_3_HCLK    (0x03)
#define SPIM_HYPER_CSHI_4_HCLK    (0x04)
#define SPIM_HYPER_CSHI_5_HCLK    (0x05)
#define SPIM_HYPER_CSHI_6_HCLK    (0x06)
#define SPIM_HYPER_CSHI_7_HCLK    (0x07)
#define SPIM_HYPER_CSHI_8_HCLK    (0x08)
#define SPIM_HYPER_CSHI_9_HCLK    (0x09)
#define SPIM_HYPER_CSHI_10_HCLK   (0x0A)
#define SPIM_HYPER_CSHI_11_HCLK   (0x0B)
#define SPIM_HYPER_CSHI_12_HCLK   (0x0C)
#define SPIM_HYPER_CSHI_13_HCLK   (0x0D)
#define SPIM_HYPER_CSHI_14_HCLK   (0x0E)
#define SPIM_HYPER_CSHI_15_HCLK   (0x0F)

/** @} end of group SPIM_HYPER_EXPORTED_CONSTANTS */

/** @addtogroup SPIM_HYPER_EXPORTED_FUNCTIONS SPIM_HYPER Exported Functions
  @{
*/
/*----------------------------------------------------------------------------*/
/*  Define Macros and functions                                               */
/*----------------------------------------------------------------------------*/
/**
  * @brief   Enable cipher balance.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_BALEN(spim)     (spim->CTL0 |= SPIM_CTL0_BALEN_Msk)

/**
  * @brief   Disable cipher balance.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_DISABLE_BALEN(spim)    (spim->CTL0 &= ~(SPIM_CTL0_BALEN_Msk))

/**
  * @brief   Enable Hyper Device Mode.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_HYPMODE(spim) (spim->CTL0 |= (SPIM_CTL0_HYPER_EN_Msk))

/**
  * @brief   Set operation mode.
  * @param[in]   spim
  * @param[in]   x   SPI Function Operation Mode
  *                  - \ref SPIM_HYPER_OPMODE_IO
  *                  - \ref SPIM_HYPER_OPMODE_PAGEWRITE
  *                  - \ref SPIM_HYPER_OPMODE_PAGEREAD
  *                  - \ref SPIM_HYPER_OPMODE_DIRECTMAP
  * \hideinitializer
  */
#define SPIM_HYPER_SET_OPMODE(spim, x)    \
    (spim->CTL0 = (spim->CTL0 & ~(SPIM_CTL0_OPMODE_Msk)) | ((x) << SPIM_CTL0_OPMODE_Pos))

/**
  * @brief   Get operation mode.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_OPMODE(spim)   \
    ((spim->CTL0 & SPIM_CTL0_OPMODE_Msk) >> SPIM_CTL0_OPMODE_Pos)

/**
  * @brief   Start operation.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_SET_GO(spim)   (spim->CTL1 |= SPIM_CTL1_SPIMEN_Msk)

/**
  * @brief   Is engine busy.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_IS_BUSY(spim)  \
    ((spim->CTL1 & SPIM_CTL1_SPIMEN_Msk) >> SPIM_CTL1_SPIMEN_Pos)

#if (SPIM_REG_CACHE == 1) // TESTCHIP_ONLY not support
/**
  * @brief   Is cache enabled.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CACHE_EN(spim)    ((spim->CTL1 & SPIM_CTL1_CACHEOFF_Msk) >> SPIM_CTL1_CACHEOFF_Pos)

/**
  * @brief   Invalidate cache.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_INVALID_CACHE(spim)    (spim->CTL1 |= SPIM_CTL1_CDINVAL_Msk)

/**
  * @brief   Cache Write Through Enable.
  * @param[in]   spim
  * \hideinitializer
  */
//#define SPIM_HYPER_ENABLE_CAWRTHEN(spim)    (spim->CTL1 |= SPIM_CTL1_CAWRTHEN_Msk)

/**
  * @brief   Cache Write Through Disable.
  * @param[in]   spim
  * \hideinitializer
  */
//#define SPIM_HYPER_DISABLE_CAWRTHEN(spim)    (spim->CTL1 &= ~(SPIM_CTL1_CAWRTHEN_Msk))

/**
  * @brief   Cache Auto Selection Updated Cache Line Number Enable.
  * @param[in]   spim
  * \hideinitializer
  */
//#define SPIM_HYPER_ENABLE_AUTOSCLN(spim)    (spim->CTL1 |= SPIM_CTL1_AUTOSCLN_Msk)

/**
  * @brief   Cache Auto Selection Updated Cache Line Number Disable.
  * @param[in]   spim
  * \hideinitializer
  */
//#define SPIM_HYPER_DISABLE_AUTOSCLN(spim)    (spim->CTL1 &= ~(SPIM_CTL1_AUTOSCLN_Msk))

/**
  * @brief   Set Updated Cache Line Number per Cache Miss.
  * @param[in]   spim
  * @param[in]   x   SPI Function Operation Mode
  *                  - \ref 0x01 : Update one cache line per cache miss. (default)
  *                  - \ref 0x02 : Update two cache line per cache miss.
  *                  - \ref 0x03 : Update three cache line per cache miss.
  *                  - \ref 0x04 : Update four cache line per cache miss.
  *                  - \ref 0x05 : Update five cache line per cache miss.
  *                  - \ref 0x06 : Update siz cache line per cache miss.
  *                  - \ref 0x07 : Update seven cache line per cache miss.
  *                  - \ref 0x08 : Update eight cache line per cache miss.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_UPDCLNUM(spim, x)    \
    (spim->CTL1 = (spim->CTL1 & ~(SPIM_CTL1_UPDCLNUM_Msk)) | (x) << SPIM_CTL1_UPDCLNUM_Pos)

#define SPIM_HYPER_GET_UPDCLNUM(spim)   \
    ((spim->CTL1 & SPIM_CTL1_UPDCLNUM_Msk) >> SPIM_CTL1_UPDCLNUM_Pos)

/**
  * @brief   Reset Updated Cache Line Number per Cache Miss.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_RESET_UPDCLNUM(spim) (spim->CTL1 &= ~(SPIM_CTL1_UPDCLNUM_Msk))

#endif

/**
  * @brief   Set SPIM clock divider.
  * @param[in]   spim
  * @param[in]   x   Clock Divider Register, only support 1 or 2
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CLKDIV(spim, x) \
    (spim->CTL1 = (spim->CTL1 & ~(SPIM_CTL1_DIVIDER_Msk)) | ((x) << SPIM_CTL1_DIVIDER_Pos))

/**
  * @brief   Set SPIM clock divider.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CLKDIV(spim) \
    ((spim->CTL1 & SPIM_CTL1_DIVIDER_Msk) >> SPIM_CTL1_DIVIDER_Pos)

/**
  * @brief   Set DMM mode SPI flash deselect time.
  * @param[in]   spim
  * @param[in]   x   DMM mode SPI flash deselect time.
  *                  It could be 0 ~ 0xFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DMM_DESELTIM(spim, x)  \
    (spim->DMMCTL = (spim->DMMCTL & ~SPIM_DMMCTL_DESELTIM_Msk) | \
                    (((x) & 0x1FUL) << SPIM_DMMCTL_DESELTIM_Pos))

/**
  * @brief   Get current DMM mode SPI flash deselect time setting.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DMM_DESELTIM(spim) \
    ((spim->DMMCTL & SPIM_DMMCTL_DESELTIM_Msk) >> SPIM_DMMCTL_DESELTIM_Pos)

/**
  * @brief   Stop DMM mode Transfer.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DMMDONE(spim)   (spim->DMMCTL |= SPIM_DMMCTL_HYPDONE_Msk)

/**
  * @brief   Get DMM mode complete to stop TX/RX.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DMMDONE(spim)  \
    ((spim->DMMCTL & SPIM_DMMCTL_HYPDONE_Msk) >> SPIM_DMMCTL_HYPDONE_Pos)

/*----------------------------------------------------------------------------*/
/* SPIM_DLLx constant definitions                                            */
/*----------------------------------------------------------------------------*/
/**
  * @brief   Set DLL0 OLDO Enable Bit
  * @param[in]   spim
  * @param[in]   x is DLL circuit power mode.
  *              - \ref SPIM_HYPER_OP_ENABLE
  *              - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DLLOLDO(spim, x)  \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLLOLDO_Msk)) | ((x) << SPIM_DLL0_DLLOLDO_Pos))

/**
  * @brief   Set DLL0 Output Valid Counter Reset.
  * @param[in]   spim
  * @param[in]   x is starts to count from 0x0 to DLLOVNUM
  *              - \ref SPIM_HYPER_OP_ENABLE
  *              - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DLLOVRST(spim, x) \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLLOVRST_Msk)) | ((x) << SPIM_DLL0_DLLOVRST_Pos))

/**
  * @brief   Get DLL0 Output Valid Counter Reset Done.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLOVRST(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLOVRST_Msk) >> SPIM_DLL0_DLLOVRST_Pos)

/**
  * @brief   Get DLL0 Clock Divider Circuit Status Bit.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLCLKON(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLCLKON_Msk) >> SPIM_DLL0_DLLCLKON_Pos)

/**
  * @brief   Get DLL0 Lock Status Bit.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLLOCK(spim)   \
    ((spim->DLL0 & SPIM_DLL0_DLLLOCK_Msk) >> SPIM_DLL0_DLLLOCK_Pos)

/**
  * @brief   Get DLL0 Output Ready Status.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLREADY(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLREADY_Msk) >> SPIM_DLL0_DLLREADY_Pos)

/**
  * @brief   Get DLL0 Auto Trim Ready Status.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLATRDY(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLATRDY_Msk) >> SPIM_DLL0_DLLATRDY_Pos)

/**
  * @brief   Get DLL0 Refresh Status Bit.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLREF(spim)    \
    ((spim->DLL0 & SPIM_DLL0_DLL_REF_Msk) >> SPIM_DLL0_DLL_REF_Pos)

/**
  * @brief   Set DLL0 Delay Step Number.
  * @param[in]   spim
  * @param[in]   x   DLL0 Delay Step Number. It could be 0 ~ 0x1F.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLDLY_NUM(spim, x)    \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLL_DNUM_Msk)) | ((x) << SPIM_DLL0_DLL_DNUM_Pos))

/**
  * @brief   Get DLL0 Delay Step Number.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLDLY_NUM(spim)    \
    ((spim->DLL0 & SPIM_DLL0_DLL_DNUM_Msk) >> SPIM_DLL0_DLL_DNUM_Pos)

/**
  * @brief   Set DLL Auto Trim.
  * @param[in]   spim
  * @param[in]   x is starts to count from 0x0 to DLLOVNUM
  *              - \ref SPIM_HYPER_OP_ENABLE
  *              - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_SET_AUTO_TRIM_DLL(spim, x)   \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLLATEN_Msk)) | ((x) << SPIM_DLL0_DLLATEN_Pos))

/**
  * @brief   Set DLL0 Delay Step Number.
  * @param[in]   spim
  * @param[in]   x   DLL0 Delay Step Number.
  *                  It could be 0 ~ 0x1F.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLDIV(spim, x)    \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLLDIVER_Msk)) | ((x) << SPIM_DLL0_DLLDIVER_Pos))

/**
  * @brief   Set Cycle Number of between DLL Lock and DLL Output Valid.
  * @param[in]   spim
  * @param[in]   x   Cycle Number of between DLL Lock and DLL Output Valid.
  *                  It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLOV_NUM(spim, x)    \
    (spim->DLL1 = (spim->DLL1 & ~(SPIM_DLL1_DLLOVNUM_Msk)) | ((x) << SPIM_DLL1_DLLOVNUM_Pos))

/**
  * @brief   Get Cycle Number of between DLL Lock and DLL Output Valid.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLOV_NUM(spim) \
    ((spim->DLL1 & SPIM_DLL1_DLLOVNUM_Msk) >> SPIM_DLL1_DLLOVNUM_Pos)

/**
  * @brief   Set Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  * @param[in]   spim
  * @param[in]   x   Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  *                  It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLLOCK_NUM(spim, x)  \
    (spim->DLL1 = (spim->DLL1 & ~(SPIM_DLL1_DLLLKNUM_Msk)) | ((x) << SPIM_DLL1_DLLLKNUM_Pos))

/**
  * @brief   Get Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLLOCKNUM(spim) \
    ((spim->DLL1 & SPIM_DLL1_DLLLKNUM_Msk) >> SPIM_DLL1_DLLLKNUM_Pos)

/**
  * @brief   Set Cycle Number of between DLL Output Valid and DLL Auto Trim Enable Time.
  * @param[in]   spim
  * @param[in]   x   Cycle Number of between DLL Output Valid and DLL Auto Trim Enable Time.
  *                  It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLTRIM_NUM(spim, x) \
    (spim->DLL2 = (spim->DLL2 & ~(SPIM_DLL2_TRIMNUM_Msk)) | ((x) << SPIM_DLL2_TRIMNUM_Pos))

/**
  * @brief   Set Cycle Number of between DLL OLDO Enable and DLL Clock Divider Enable Time.
  * @param[in]   spim
  * @param[in]   x   Cycle Number of between DLL OLDO Enable and DLL Clock Divider Enable Time.
  *                  It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLCLKON_NUM(spim, x)    \
    (spim->DLL2 = (spim->DLL2 & ~(SPIM_DLL2_CLKONNUM_Msk)) | ((x) << SPIM_DLL2_CLKONNUM_Pos))

/**
  * @brief   Set Hyper Chip Select Setup Time to Next CK Rising Edge.
  * @param[in]  spim
  * @param[in]  x   Chip Select Setup Time to Next CK Rising Edge.
  *                  - \ref SPIM_HYPER_CSST_3_5_HCLK : 3.5 HCLK cycles
  *                  - \ref SPIM_HYPER_CSST_4_5_HCLK : 4.5 HCLK cycles
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSST(spim, x)    \
    (spim->HYPER_CONFIG1 = (spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSST_Msk)) | \
                           ((x) << SPIM_HYPER_CONFIG1_CSST_Pos))

/**
  * @brief   Set Hyper Chip Select Hold Time After CK Falling Edge.
  * @param[in]  spim
  * @param[in]  x   Chip Select Hold Time After CK Falling Edge.
  *                 - \ref SPIM_HYPER_CSH_0_5_HCLK : 0.5 HCLK cycles
  *                 - \ref SPIM_HYPER_CSH_1_5_HCLK : 1.5 HCLK cycles
  *                 - \ref SPIM_HYPER_CSH_2_5_HCLK : 2.5 HCLK cycles
  *                 - \ref SPIM_HYPER_CSH_3_5_HCLK : 3.5 HCLK cycles
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSH(spim, x) \
    (spim->HYPER_CONFIG1 = (spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSH_Msk)) | \
                           ((x) << SPIM_HYPER_CONFIG1_CSH_Pos))

/**
  * @brief   Set Hyper Chip Select High between Transaction.
  * @param[in]  spim
  * @param[in]  x   Set Chip Select High between Transaction as u8Value HCLK cycles.
  *                  It could be 2 ~ 16.
  *                 - \ref SPIM_HYPER_CSHI_2_HCLK  : 2 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_3_HCLK  : 3 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_4_HCLK  : 4 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_5_HCLK  : 5 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_6_HCLK  : 6 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_7_HCLK  : 7 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_8_HCLK  : 8 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_9_HCLK  : 9 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_10_HCLK : 10 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_11_HCLK : 11 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_12_HCLK : 12 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_13_HCLK : 13 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_14_HCLK : 14 HCLK cycles
  *                 - \ref SPIM_HYPER_CSHI_15_HCLK : 15 HCLK cycles
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSHI(spim, x)    \
    (spim->HYPER_CONFIG1 = (spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSHI_Msk)) | \
                           ((x) << SPIM_HYPER_CONFIG1_CSHI_Pos))

/**
  * @brief   Set Hyper Chip Select Maximum Low Time.
  * @param[in]  spim
  * @param[in]  u32CsMaxLT  Set Hyper Chip Select Maximum Low Time as u32CsMaxLT HCLK cycles.
  *                         It could be 1 ~ 2048.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSMAXLT(spim, u32CsMaxLT)   \
    (spim->HYPER_CONFIG1 = (spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSMAXLT_Msk)) | \
                           (((u32CsMaxLT) - 1) << SPIM_HYPER_CONFIG1_CSMAXLT_Pos))
/**
  * @brief   Get Hyper Chip Select Maximum Low Time.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CSMAXLT(spim)    \
    (((spim->HYPER_CONFIG1 & SPIM_HYPER_CONFIG1_CSMAXLT_Msk) >> SPIM_HYPER_CONFIG1_CSMAXLT_Pos) + 1UL)

/**
  * @brief   Set Hyper Chip Initial Read Access Time.
  * @param[in]  spim
  * @param[in]  x   Initial Access Time. It could be 1 ~ 31.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_ACCTWR(spim, x)  \
    (spim->HYPER_CONFIG2 = (spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_ACCTWR_Msk)) | \
                           ((x) << SPIM_HYPER_CONFIG2_ACCTWR_Pos))

/**
  * @brief   Get Hyper Chip Initial Read Access Time.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_ACCTWR(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_ACCTWR_Msk) >> SPIM_HYPER_CONFIG2_ACCTWR_Pos)

/**
  * @brief   Set Hyper Device RESETN Low Time.
  * @param[in]  spim
  * @param[in]  x Initial Device RESETN Low Time. It could be 0 ~ 255.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_RSTNLT(spim, x)    \
    (spim->HYPER_CONFIG2 = (spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_RSTNLT_Msk)) |\
                           ((x) << SPIM_HYPER_CONFIG2_RSTNLT_Pos))

/**
  * @brief   Get Hyper Device RESETN Low Time.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_RSTNLT(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_RSTNLT_Msk) >> SPIM_HYPER_CONFIG2_RSTNLT_Pos)

/**
  * @brief   Set Hyper Chip Initial Read Access Time.
  * @param[in]  spim
  * @param[in]  x   Initial Access Time. It could be 1 ~ 31.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_ACCTRD(spim, x)  \
    (spim->HYPER_CONFIG2 = (spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_ACCTRD_Msk)) |   \
                           ((x) << SPIM_HYPER_CONFIG2_ACCTRD_Pos))

/**
  * @brief   Get Hyper Chip Initial Read Access Time.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_ACCTRD(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_ACCTRD_Msk) >> SPIM_HYPER_CONFIG2_ACCTRD_Pos)

/**
  * @brief   Clear Hyper Bus Write DATA
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_CLEAR_WDATA(spim)    (spim->HYPER_WDATA &= ~(0xFFFFFFFF))

/**
  * @brief   Enable Hyper Chip Operation Done Interrupt.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_INT(spim) (spim->HYPER_INTEN |= SPIM_HYPER_INTEN_OPINTEN_Msk)

/**
  * @brief   Disable Hyper Chip Operation Done Interrupt.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_DISABLE_INT(spim)    (spim->HYPER_INTEN &= ~(SPIM_HYPER_INTEN_OPINTEN_Msk))

/**
  * @brief   Get Hyper Bus Operation Done Interrupt.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_INT(spim)    \
    ((spim->HYPER_INTEN & SPIM_HYPER_INTEN_OPINTEN_Msk) >> SPIM_HYPER_INTEN_OPINTEN_Pos)

/**
  * @brief   Get Hyper Chip Operation Done Interrupt.
  * @param[in]   spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_INTSTS(spim) \
    ((spim->HYPER_INTEN & SPIM_HYPER_INTEN_OPINTEN_Msk) >> SPIM_HYPER_INTEN_OPINTEN_Pos)

/*----------------------------------------------------------------------------*/
/* static inline functions                                                    */
/*----------------------------------------------------------------------------*/
__STATIC_INLINE void SPIM_HYPER_DISABLE_CIPHER(SPIM_T *spim);
__STATIC_INLINE void SPIM_HYPER_ENABLE_CIPHER(SPIM_T *spim);

__STATIC_INLINE void SPIM_HYPER_DISABLE_CACHE(SPIM_T *spim);
__STATIC_INLINE void SPIM_HYPER_ENABLE_CACHE(SPIM_T *spim);
__STATIC_INLINE uint32_t SPIM_HYPER_GetDMMAddress(SPIM_T *spim);

/**
  * @brief   Disable cipher.
  * @param[in]   spim
  * @note    When encryption/decryption of SPIM is disabled,
  *          please set SPIM_HYPER_SET_DMM_DESELTIM >= 0x8.
  */
__STATIC_INLINE void SPIM_HYPER_DISABLE_CIPHER(SPIM_T *spim)
{
    spim->CTL0 |= (SPIM_CTL0_BALEN_Msk | SPIM_CTL0_CIPHOFF_Msk);
    //spim->CTL0 = (spim->CTL0 & ~(SPIM_CTL0_BALEN_Msk)) | SPIM_CTL0_CIPHOFF_Msk;

    SPIM_HYPER_SET_DMM_DESELTIM(spim, 0x08);
}

/**
  * @brief   Enable cipher.
  * @param[in]   spim
  * @note    When encryption/decryption of SPIM is enabled,
  *          please set SPIM_HYPER_SET_DMM_DESELTIM() >= 0x10.
  */
__STATIC_INLINE void SPIM_HYPER_ENABLE_CIPHER(SPIM_T *spim)
{
    spim->CTL0 &= ~(SPIM_CTL0_CIPHOFF_Msk | SPIM_CTL0_BALEN_Msk);
    //spim->CTL0 = (spim->CTL0 & ~(SPIM_CTL0_CIPHOFF_Msk)) | SPIM_CTL0_BALEN_Msk;

    SPIM_HYPER_SET_DMM_DESELTIM(spim, 0x12);
}

/**
  * @brief   Disable cache.
  * @param[in]   spim
  * @note    Minimum time width of SPIM_SS
  *          deselect time = (DESELTIM + 1) * AHB clock cycle time.
  */
__STATIC_INLINE void SPIM_HYPER_DISABLE_CACHE(SPIM_T *spim)
{
#if (SPIM_REG_CACHE == 1) //TESTCHIP_ONLY not support
    (spim->CTL1 |= SPIM_CTL1_CACHEOFF_Msk);
#endif

    /* Cipher Disabled Set Deselect Time 0x01 */
    if (((spim->CTL0 & SPIM_CTL0_CIPHOFF_Msk) >> SPIM_CTL0_CIPHOFF_Pos) != SPIM_OP_DISABLE)
    {
        SPIM_HYPER_SET_DMM_DESELTIM(spim, 0x1);
    }
}

/**
  * @brief   Enable cache.
  * @param[in]   spim
  * @note    Minimum time width of SPIM_SS
  *          deselect time = (DESELTIM + 4) * AHB clock cycle time.
  */
__STATIC_INLINE void SPIM_HYPER_ENABLE_CACHE(SPIM_T *spim)
{
#if (SPIM_REG_CACHE == 1) // TESTCHIP_ONLY not support
    (spim->CTL1 &= ~(SPIM_CTL1_CACHEOFF_Msk));

    /* Cipher Disabled Set Deselect Time 0x04 */
    if (((spim->CTL0 & SPIM_CTL0_CIPHOFF_Msk) >> SPIM_CTL0_CIPHOFF_Pos) != SPIM_OP_DISABLE)
    {
        SPIM_HYPER_SET_DMM_DESELTIM(spim, 0x4);
    }

#endif
}

/**
  * @brief   Get Direct Map Address.
  * @param[in]  spim
  * @return Direct Mapping Address
  */
__STATIC_INLINE uint32_t SPIM_HYPER_GetDMMAddress(SPIM_T *spim)
{
    uint32_t u32DMMAddr = 0;

    if (spim == SPIM0)
    {
        u32DMMAddr = SPIM_HYPER_DMM0_ADDR;
    }
    else if (spim == SPIM1) // TESTCHIP_ONLY
    {
        u32DMMAddr = SPIM_HYPER_DMM1_ADDR;
    }

    return u32DMMAddr;
}

/* Octal SPI flash and hyper device training DLL API */
int32_t SPIM_HYPER_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum);

/* HyperRAM */
int32_t SPIM_HYPER_ExitHSAndDPD(SPIM_T *spim);
int32_t SPIM_HYPER_ReadHyperRAMReg(SPIM_T *spim, uint32_t u32Addr);
int32_t SPIM_HYPER_WriteHyperRAMReg(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Value);

/* Hyper Device API */
void SPIM_HYPER_Init(SPIM_T *spim, uint32_t u32Div);
int32_t SPIM_HYPER_Reset(SPIM_T *spim);
int16_t SPIM_HYPER_Read1Word(SPIM_T *spim, uint32_t u32Addr);
int32_t SPIM_HYPER_Read2Word(SPIM_T *spim, uint32_t u32Addr);
int32_t SPIM_HYPER_Write1Byte(SPIM_T *spim, uint32_t u32Addr, uint8_t u8Data);
int32_t SPIM_HYPER_Write2Byte(SPIM_T *spim, uint32_t u32Addr, uint16_t u16Data);
int32_t SPIM_HYPER_Write3Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data);
int32_t SPIM_HYPER_Write4Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data);

int32_t SPIM_HYPER_DMAWrite(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8WrBuf, uint32_t u32NTx);
int32_t SPIM_HYPER_DMARead(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8RdBuf, uint32_t u32NRx);

void SPIM_HYPER_EnterDirectMapMode(SPIM_T *spim);
void SPIM_HYPER_ExitDirectMapMode(SPIM_T *spim);
int32_t SPIM_HYPER_IsDMMDone(SPIM_T *spim);

/** @} end of group SPIM_HYPER_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_HYPER_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __SPIM_HYPER_H__ */
