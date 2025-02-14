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

#define SPIM_HYPER_DMM0_SADDR               (0x82000000UL)  /*!< SPIM0 DMM mode memory map base secure address */
#define SPIM_HYPER_DMM0_NSADDR              (0x92000000UL)  /*!< SPIM1 DMM mode memory map base non secure address */

#if defined (SCU_INIT_D0PNS2_VAL) && (SCU_INIT_D0PNS2_VAL & SCU_D0PNS2_SPIM0_Msk)
#define SPIM_HYPER_DMM0_ADDR                SPIM_HYPER_DMM0_NSADDR
#else
#define SPIM_HYPER_DMM0_ADDR                SPIM_HYPER_DMM0_SADDR
#endif

#define SPIM_HYPER_DMM_SIZE                 (0x2000000UL)       /*!< DMM mode memory mapping size */

#define SPIM_HYPER_MAX_LATENCY              (0x20)              /*!< Maximum DLL training number */

/* SPIM_HYPER Define Hyper Device Mode */
#define SPIM_HYPERRAM_MODE                  (0x01)
#define SPIM_HYPERFLASH_MODE                (0x02)

#define SPIM_HYPER_OP_ENABLE                (0x01UL)            /* SPIM_HYPER Operation Enable */
#define SPIM_HYPER_OP_DISABLE               (0x00UL)            /* SPIM_HYPER Operation Disable */

/* SPIM_HYPER Wait State Timeout Counter. */
#define SPIM_HYPER_TIMEOUT                  SystemCoreClock     /*!< SPIM_HYPER time-out counter (1 second time-out) */

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
#define SPIM_HYPER_HRAM_ID_REG0             (0x00000000)    /* Hyper RAM Identification Register 0. */
#define SPIM_HYPER_HRAM_ID_REG1             (0x00000002)    /* Hyper RAM Identification Register 1. */
#define SPIM_HYPER_HRAM_CONFIG_REG0         (0x00001000)    /* Hyper RAM Configuration Register 0. */
#define SPIM_HYPER_HRAM_CONFIG_REG1         (0x00001002)    /* Hyper RAM Configuration Register 1. */

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select Setup Time to Next CK Rising Edge
    0 = 3.5 HCLK cycles.
    1 = 4.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSST_3_5_HCLK    (0x0)
#define SPIM_HYPER_CSST_4_5_HCLK    (0x1)

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select Hold Time After CK Falling Edge
    10 = 2.5 HCLK cycles.
    11 = 3.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSH_2_5_HCLK    (0x2)
#define SPIM_HYPER_CSH_3_5_HCLK    (0x3)

/*----------------------------------------------------------------------------*/
/* SPIM_HYPER_CONFIG1: Chip Select High between Transaction
    10 = 0.5 HCLK cycles.
    ...
    1111 = 3.5 HCLK cycles.
*/
/*----------------------------------------------------------------------------*/
#define SPIM_HYPER_CSHI_4_HCLK    (0x03)
#define SPIM_HYPER_CSHI_5_HCLK    (0x04)
#define SPIM_HYPER_CSHI_6_HCLK    (0x05)
#define SPIM_HYPER_CSHI_7_HCLK    (0x06)
#define SPIM_HYPER_CSHI_8_HCLK    (0x07)
#define SPIM_HYPER_CSHI_9_HCLK    (0x08)
#define SPIM_HYPER_CSHI_10_HCLK   (0x09)
#define SPIM_HYPER_CSHI_11_HCLK   (0x0A)
#define SPIM_HYPER_CSHI_12_HCLK   (0x0B)
#define SPIM_HYPER_CSHI_13_HCLK   (0x0C)
#define SPIM_HYPER_CSHI_14_HCLK   (0x0D)
#define SPIM_HYPER_CSHI_15_HCLK   (0x0E)
#define SPIM_HYPER_CSHI_16_HCLK   (0x0F)

/** @} end of group SPIM_HYPER_EXPORTED_CONSTANTS */

/** @addtogroup SPIM_HYPER_EXPORTED_FUNCTIONS SPIM_HYPER Exported Functions
  @{
*/
/*----------------------------------------------------------------------------*/
/*  Define Macros and functions                                               */
/*----------------------------------------------------------------------------*/
/**
  * @brief  Get SPIM Cipher Mode.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CIPHER(spim) \
    ((spim->CTL0 & SPIM_CTL0_CIPHOFF_Msk) >> SPIM_CTL0_CIPHOFF_Pos)

/**
  * @brief  Enable Hyper Device Mode.
  * @param  spim
  * @param  x SPIM Hyper Device Mode
  *           - \ref SPIM_HYPERRAM_MODE
  *           - \ref SPIM_HYPERFLASH_MODE
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_HYPMODE(spim, x)                    \
    (spim->CTL0 = ((spim->CTL0 & ~(SPIM_CTL0_DEVMODE_Msk)) |  \
                   ((x) << SPIM_CTL0_DEVMODE_Pos)))

/**
  * @brief  Get operation mode.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_OPMODE(spim) \
    ((spim->CTL0 & SPIM_CTL0_OPMODE_Msk) >> SPIM_CTL0_OPMODE_Pos)

/**
  * @brief  Start operation.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_SET_GO(spim)   \
    (spim->CTL1 = ((spim->CTL1 & ~(SPIM_CTL1_SPIMEN_Msk)) | SPIM_CTL1_SPIMEN_Msk))

/**
  * @brief  Is engine busy.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_IS_BUSY(spim)  \
    ((spim->CTL1 & SPIM_CTL1_SPIMEN_Msk) >> SPIM_CTL1_SPIMEN_Pos)

/**
  * @brief  Set SPIM clock divider.
  * @param  spim
  * @param  x Clock Divider Register, only support 1 or 2
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CLKDIV(spim, x)                        \
    (spim->CTL1 = ((spim->CTL1 & ~(SPIM_CTL1_DIVIDER_Msk)) |  \
                   ((x) << SPIM_CTL1_DIVIDER_Pos)))

/**
  * @brief  Set SPIM clock divider.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CLKDIV(spim) \
    ((spim->CTL1 & SPIM_CTL1_DIVIDER_Msk) >> SPIM_CTL1_DIVIDER_Pos)

/**
  * @brief  Set DMM mode SPI flash deselect time.
  * @param  spim
  * @param  x DMM mode SPI flash deselect time. It could be 0 ~ 0xFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DMM_DESELTIM(spim, x)                        \
    (spim->DMMCTL = ((spim->DMMCTL & ~SPIM_DMMCTL_DESELTIM_Msk) |   \
                     (((x) & 0x1FUL) << SPIM_DMMCTL_DESELTIM_Pos)))

/**
  * @brief  Get current DMM mode SPI flash deselect time setting.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DMM_DESELTIM(spim) \
    ((spim->DMMCTL & SPIM_DMMCTL_DESELTIM_Msk) >> SPIM_DMMCTL_DESELTIM_Pos)

/**
  * @brief  Stop DMM mode Transfer.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DMMDONE(spim)                             \
    (spim->DMMCTL = ((spim->DMMCTL & ~(SPIM_DMMCTL_HYPDONE_Msk)) |  \
                     SPIM_DMMCTL_HYPDONE_Msk))

/**
  * @brief  Get DMM mode complete to stop TX/RX.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DMMDONE(spim)  \
    ((spim->DMMCTL & SPIM_DMMCTL_HYPDONE_Msk) >> SPIM_DMMCTL_HYPDONE_Pos)

/**
 * @brief Get DMM idle state.
 * @param spim
 * \hideinitializer
 */
#define SPIM_HYPER_GET_DMM_IDLE(spim) \
    ((spim->DMMCTL & SPIM_DMMCTL_DMMIDLE_Msk) >> SPIM_DMMCTL_DMMIDLE_Pos)

/**
 * @brief Set DMM time-out counter.
 * @param spim
 * \hideinitializer
 */
#define SPIM_HYPER_SET_DMM_TOCNTDMM(spim, x)  \
    (spim->DMM_TIMEOUT_INTERVAL = ((spim->DMM_TIMEOUT_INTERVAL & ~(SPIM_DMM_TIMEOUT_TOCNT_Msk)) | \
                                   ((x) <<SPIM_DMM_TIMEOUT_TOCNT_Pos)))

/**
 * @brief Get DMM time-out state flag.
 * @param spim
 * \hideinitializer
 */
#define SPIM_HYPER_GET_DMM_TIMEOUT_STS(spim)   \
    ((spim->DMM_TIMEOUT_FLAG_STS & SPIM_DMM_TIMEOUT_STS_TOF_Msk) >> SPIM_DMM_TIMEOUT_STS_TOF_Pos)

/**
 * @brief Clear DMM time-out state flag.
 * @param spim
 * \hideinitializer
 */
#define SPIM_HYPER_CLR_DMM_TIMEOUT_STS(spim)                                                                \
    (spim->DMM_TIMEOUT_FLAG_STS = ((spim->DMM_TIMEOUT_FLAG_STS & ~(SPIM_DMM_TIMEOUT_STS_TOF_Msk)) | \
                                   SPIM_DMM_TIMEOUT_STS_TOF_Msk))

/*----------------------------------------------------------------------------*/
/* SPIM_DLLx constant definitions                                            */
/*----------------------------------------------------------------------------*/
/**
  * @brief  Set DLL0 OLDO Enable Bit
  * @param  spim
  * @param  x is DLL circuit power mode.
  *           - \ref SPIM_HYPER_OP_ENABLE
  *           - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DLLOLDO(spim, x)  \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLLOLDO_Msk)) | ((x) << SPIM_DLL0_DLLOLDO_Pos)))

/**
  * @brief  Get DLL0 OLDO Enable Bit
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLOLDO(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLOLDO_Msk) >> SPIM_DLL0_DLLOLDO_Pos)

/**
  * @brief  Set DLL0 Output Valid Counter Reset.
  * @param  spim
  * @param  x is starts to count from 0x0 to DLLOVNUM
  *           - \ref SPIM_HYPER_OP_ENABLE
  *           - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_DLLOVRST(spim, x)                     \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLLOVRST_Msk)) |   \
                   ((x) << SPIM_DLL0_DLLOVRST_Pos)))

/**
  * @brief  Get DLL0 Output Valid Counter Reset Done.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLOVRST(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLOVRST_Msk) >> SPIM_DLL0_DLLOVRST_Pos)

/**
  * @brief  Get DLL0 Clock Divider Circuit Status Bit.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLCLKON(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLCLKON_Msk) >> SPIM_DLL0_DLLCLKON_Pos)

/**
  * @brief  Get DLL0 Lock Status Bit.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLLOCK(spim)   \
    ((spim->DLL0 & SPIM_DLL0_DLLLOCK_Msk) >> SPIM_DLL0_DLLLOCK_Pos)

/**
  * @brief  Get DLL0 Output Ready Status.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLREADY(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLREADY_Msk) >> SPIM_DLL0_DLLREADY_Pos)

/**
  * @brief  Get DLL0 Auto Trim Ready Status.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLATRDY(spim)  \
    ((spim->DLL0 & SPIM_DLL0_DLLATRDY_Msk) >> SPIM_DLL0_DLLATRDY_Pos)

/**
  * @brief  Get DLL0 Refresh Status Bit.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLREF(spim)    \
    ((spim->DLL0 & SPIM_DLL0_DLL_REF_Msk) >> SPIM_DLL0_DLL_REF_Pos)

/**
  * @brief  Set DLL0 Delay Step Number.
  * @param  spim
  * @param  x DLL0 Delay Step Number. It could be 0 ~ 0x1F.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLDLY_NUM(spim, x)                      \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLL_DNUM_Msk)) |   \
                   ((x) << SPIM_DLL0_DLL_DNUM_Pos)))

/**
  * @brief  Get DLL0 Delay Step Number.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLDLY_NUM(spim)    \
    ((spim->DLL0 & SPIM_DLL0_DLL_DNUM_Msk) >> SPIM_DLL0_DLL_DNUM_Pos)

/**
  * @brief  Set DLL Auto Trim.
  * @param  spim
  * @param  x is starts to count from 0x0 to DLLOVNUM
  *           - \ref SPIM_HYPER_OP_ENABLE
  *           - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_SET_AUTO_TRIM_DLL(spim, x)                   \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLLATEN_Msk)) |    \
                   ((x) << SPIM_DLL0_DLLATEN_Pos)))

/**
  * @brief  Set HYPERDLL Delay Time Self-Test Enable.
  * @param  spim
  * @param  x is starts to count from 0x0 to DLLOVNUM
  *           - \ref SPIM_HYPER_OP_ENABLE
  *           - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLL_SELF_TEST(spim, x)   \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_DLLLOOP_Msk)) | ((x) << SPIM_DLL0_DLLLOOP_Pos))

/**
  * @brief  Set Selection for Clock Source DQS/RWDS of HYPERDLL Open Loop Delay Line.
  * @param  spim
  * @param  x is starts to count from 0x0 to DLLOVNUM
  *           - \ref SPIM_HYPER_OP_ENABLE
  *           - \ref SPIM_HYPER_OP_DISABLE
  * \hideinitializer
  */
#define SPIM_HYPER_SET_INTERNAL_RWDS(spim, x)   \
    (spim->DLL0 = (spim->DLL0 & ~(SPIM_DLL0_SIGINTEN_Msk)) | ((x) << SPIM_DLL0_SIGINTEN_Pos))

/**
  * @brief  Set DLL0 Delay Step Number.
  * @param  spim
  * @param  x DLL0 Delay Step Number. It could be 0 ~ 0x1F.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLDIV(spim, x)                          \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLLDIVER_Msk)) |   \
                   ((x) << SPIM_DLL0_DLLDIVER_Pos)))

/**
 * @brief Set Band Selection of HYPERDLL Reference Clock.
 * @param spim
 * @param x is selection of HYPERDLL reference clock.
 *          - \ref SPIM_OP_ENABLE
 *          - \ref SPIM_OP_DISABLE
 * \hideinitializer
 */
#define SPIM_HYPER_SET_DLLFAST(spim, x)                         \
    (spim->DLL0 = ((spim->DLL0 & ~(SPIM_DLL0_DLLFAST_Msk)) |    \
                   ((x) << SPIM_DLL0_DLLFAST_Pos)))

/**
  * @brief  Set Cycle Number of between DLL Lock and DLL Output Valid.
  * @param  spim
  * @param  x Cycle Number of between DLL Lock and DLL Output Valid.
  *           It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLOV_NUM(spim, x)                       \
    (spim->DLL1 = ((spim->DLL1 & ~(SPIM_DLL1_DLLOVNUM_Msk)) |   \
                   ((x) << SPIM_DLL1_DLLOVNUM_Pos)))

/**
  * @brief  Get Cycle Number of between DLL Lock and DLL Output Valid.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLOV_NUM(spim) \
    ((spim->DLL1 & SPIM_DLL1_DLLOVNUM_Msk) >> SPIM_DLL1_DLLOVNUM_Pos)

/**
  * @brief  Set Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  * @param  spim
  * @param  x Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  *           It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLLOCK_NUM(spim, x)                     \
    (spim->DLL1 = ((spim->DLL1 & ~(SPIM_DLL1_DLLLKNUM_Msk)) |   \
                   ((x) << SPIM_DLL1_DLLLKNUM_Pos)))

/**
  * @brief  Get Cycle Number between DLL Clock Divider Enable and DLL Lock Valid.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_DLLLOCKNUM(spim) \
    ((spim->DLL1 & SPIM_DLL1_DLLLKNUM_Msk) >> SPIM_DLL1_DLLLKNUM_Pos)

/**
  * @brief  Set Cycle Number of between DLL Output Valid and DLL Auto Trim Enable Time.
  * @param  spim
  * @param  x Cycle Number of between DLL Output Valid and DLL Auto Trim Enable Time.
  *           It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLTRIM_NUM(spim, x)                     \
    (spim->DLL2 = ((spim->DLL2 & ~(SPIM_DLL2_TRIMNUM_Msk)) |    \
                   ((x) << SPIM_DLL2_TRIMNUM_Pos)))

/**
  * @brief  Set Cycle Number of between DLL OLDO Enable and DLL Clock Divider Enable Time.
  * @param  spim
  * @param  x Cycle Number of between DLL OLDO Enable and DLL Clock Divider Enable Time.
  *           It could be 0 ~ 0xFFFF.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_DLLCLKON_NUM(spim, x)                    \
    (spim->DLL2 = ((spim->DLL2 & ~(SPIM_DLL2_CLKONNUM_Msk)) |   \
                   ((x) << SPIM_DLL2_CLKONNUM_Pos)))

/**
  * @brief  Set Hyper Chip Select Setup Time to Next CK Rising Edge.
  * @param  spim
  * @param  x Chip Select Setup Time to Next CK Rising Edge.
  *           - \ref SPIM_HYPER_CSST_3_5_HCLK : 3.5 HCLK cycles
  *           - \ref SPIM_HYPER_CSST_4_5_HCLK : 4.5 HCLK cycles
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSST(spim, x)                                                \
    (spim->HYPER_CONFIG1 = ((spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSS_Msk)) | \
                            ((x) << SPIM_HYPER_CONFIG1_CSS_Pos)))

/**
  * @brief  Set Hyper Chip Select Hold Time After CK Falling Edge.
  * @param  spim
  * @param  x Chip Select Hold Time After CK Falling Edge.
  *           - \ref SPIM_HYPER_CSH_2_5_HCLK : 2.5 HCLK cycles
  *           - \ref SPIM_HYPER_CSH_3_5_HCLK : 3.5 HCLK cycles
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSH(spim, x)                                                 \
    (spim->HYPER_CONFIG1 = ((spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSH_Msk)) | \
                            ((x) << SPIM_HYPER_CONFIG1_CSH_Pos)))

/**
  * @brief  Set Hyper Chip Select Maximum Low Time.
  * @param  spim
  * @param  u32CsMaxLT  Set Hyper Chip Select Maximum Low Time as u32CsMaxLT HCLK cycles.
  *                     It could be 1 ~ 2048.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_CSMAXLT(spim, u32CsMaxLT)                                        \
    (spim->HYPER_CONFIG1 = ((spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSMAXLT_Msk)) | \
                            (((u32CsMaxLT) - 1) << SPIM_HYPER_CONFIG1_CSMAXLT_Pos)))
/**
  * @brief  Get Hyper Chip Select Maximum Low Time.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_CSMAXLT(spim)    \
    (((spim->HYPER_CONFIG1 & SPIM_HYPER_CONFIG1_CSMAXLT_Msk) >> SPIM_HYPER_CONFIG1_CSMAXLT_Pos) + 1UL)

/**
  * @brief  Set Hyper Chip Initial Read Access Time.
  * @param  spim
  * @param  x Initial Access Time. It could be 1 ~ 31.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_ACCTWR(spim, x)                                                  \
    (spim->HYPER_CONFIG2 = ((spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_ACCTWR_Msk)) |  \
                            ((x) << SPIM_HYPER_CONFIG2_ACCTWR_Pos)))

/**
  * @brief  Get Hyper Chip Initial Read Access Time.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_ACCTWR(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_ACCTWR_Msk) >> SPIM_HYPER_CONFIG2_ACCTWR_Pos)

/**
  * @brief  Set Hyper Device RESETN Low Time.
  * @param  spim
  * @param  x Initial Device RESETN Low Time. It could be 0 ~ 255.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_RSTNLT(spim, x)                                                  \
    (spim->HYPER_CONFIG2 = ((spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_RSTNLT_Msk)) |  \
                            ((x) << SPIM_HYPER_CONFIG2_RSTNLT_Pos)))

/**
  * @brief  Get Hyper Device RESETN Low Time.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_RSTNLT(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_RSTNLT_Msk) >> SPIM_HYPER_CONFIG2_RSTNLT_Pos)

/**
  * @brief  Set Hyper Chip Initial Read Access Time.
  * @param  spim
  * @param  x Initial Access Time. It could be 1 ~ 31.
  * \hideinitializer
  */
#define SPIM_HYPER_SET_ACCTRD(spim, x)                                                  \
    (spim->HYPER_CONFIG2 = ((spim->HYPER_CONFIG2 & ~(SPIM_HYPER_CONFIG2_ACCTRD_Msk)) |  \
                            ((x) << SPIM_HYPER_CONFIG2_ACCTRD_Pos)))

/**
  * @brief  Get Hyper Chip Initial Read Access Time.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_ACCTRD(spim) \
    ((spim->HYPER_CONFIG2 & SPIM_HYPER_CONFIG2_ACCTRD_Msk) >> SPIM_HYPER_CONFIG2_ACCTRD_Pos)

/**
  * @brief  Clear Hyper Bus Write DATA
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_CLEAR_WDATA(spim)    (spim->HYPER_WDATA &= ~(0xFFFFFFFF))

/**
  * @brief  Enable Hyper Chip Operation Done Interrupt.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_ENABLE_INT(spim)                                               \
    (spim->HYPER_INTEN = ((spim->HYPER_INTEN & ~(SPIM_HYPER_INTEN_OPINTEN_Msk)) | \
                          SPIM_HYPER_INTEN_OPINTEN_Msk))

/**
  * @brief  Disable Hyper Chip Operation Done Interrupt.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_DISABLE_INT(spim)    (spim->HYPER_INTEN &= ~(SPIM_HYPER_INTEN_OPINTEN_Msk))

/**
  * @brief  Get Hyper Bus Operation Done Interrupt.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_INT(spim)    \
    ((spim->HYPER_INTEN & SPIM_HYPER_INTEN_OPINTEN_Msk) >> SPIM_HYPER_INTEN_OPINTEN_Pos)

/**
  * @brief  Get Hyper Chip Operation Done Interrupt.
  * @param  spim
  * \hideinitializer
  */
#define SPIM_HYPER_GET_INTSTS(spim) \
    ((spim->HYPER_INTEN & SPIM_HYPER_INTEN_OPINTEN_Msk) >> SPIM_HYPER_INTEN_OPINTEN_Pos)

/*----------------------------------------------------------------------------*/
/* static inline functions                                                    */
/*----------------------------------------------------------------------------*/
__STATIC_INLINE int32_t SPIM_HYPER_WAIT_DMMDONE(SPIM_T *spim);
__STATIC_INLINE void SPIM_HYPER_DISABLE_CIPHER(SPIM_T *spim);
__STATIC_INLINE void SPIM_HYPER_ENABLE_CIPHER(SPIM_T *spim);
__STATIC_INLINE uint32_t SPIM_HYPER_GET_DMMADDR(SPIM_T *spim);
__STATIC_INLINE void SPIM_HYPER_SET_OPMODE(SPIM_T *spim, uint32_t x);

/**
 * @brief Set Hyper Chip Select High between Transaction.
 * @param spim
 * @param u32CSHI Set Chip Select High between Transaction as u8Value HCLK cycles.
 *                - \ref SPIM_HYPER_CSHI_4_HCLK  : 4 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_5_HCLK  : 5 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_6_HCLK  : 6 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_7_HCLK  : 7 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_8_HCLK  : 8 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_9_HCLK  : 9 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_10_HCLK  : 10 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_11_HCLK : 11 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_12_HCLK : 12 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_13_HCLK : 13 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_14_HCLK : 14 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_15_HCLK : 15 HCLK cycles
 *                - \ref SPIM_HYPER_CSHI_16_HCLK : 16 HCLK cycles
 * \hideinitializer
 */
__STATIC_INLINE void SPIM_HYPER_SET_CSHI(SPIM_T *spim, uint32_t u32CSHI)
{
    spim->HYPER_CONFIG1 = ((spim->HYPER_CONFIG1 & ~(SPIM_HYPER_CONFIG1_CSHI_Msk)) |
                           (((u32CSHI < SPIM_HYPER_CSHI_4_HCLK) ? SPIM_HYPER_CSHI_4_HCLK : u32CSHI) << SPIM_HYPER_CONFIG1_CSHI_Pos));
}

/**
  * @brief  Wait Hyper Direct Map Mode Read/Write Done.
  * @param  spim
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
__STATIC_INLINE int32_t SPIM_HYPER_WAIT_DMMDONE(SPIM_T *spim)
{
    volatile int32_t u32TimeOutCount = SPIM_HYPER_TIMEOUT;

    SPIM_HYPER_ENABLE_DMMDONE(spim);       /* HyperBus DMM Mode Done.  */

    while (SPIM_HYPER_GET_DMMDONE(spim))
    {
        if (--u32TimeOutCount <= 0)
        {
            break;
        }
    }

    u32TimeOutCount = SPIM_HYPER_TIMEOUT;

    /* Wait for DMM mode to be idle */
    while (SPIM_HYPER_GET_DMM_IDLE(spim) == SPIM_HYPER_OP_DISABLE)
    {
        if (--u32TimeOutCount <= 0)
        {
            break;
        }
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Disable cipher.
  * @param  spim
  * @note   When encryption/decryption of SPIM is disabled.
  */
__STATIC_INLINE void SPIM_HYPER_DISABLE_CIPHER(SPIM_T *spim)
{
    SPIM_HYPER_WAIT_DMMDONE(spim);

    spim->CTL0 |= (SPIM_CTL0_CIPHOFF_Msk);
}

/**
  * @brief  Enable cipher.
  * @param  spim
  * @note   When encryption/decryption of SPIM is enabled.
  */
__STATIC_INLINE void SPIM_HYPER_ENABLE_CIPHER(SPIM_T *spim)
{
    SPIM_HYPER_WAIT_DMMDONE(spim);

    spim->CTL0 &= ~(SPIM_CTL0_CIPHOFF_Msk);
}

/**
  * @brief  Get Direct Map Address.
  * @param  spim
  * @return Direct Mapping Address.
  */
__STATIC_INLINE uint32_t SPIM_HYPER_GET_DMMADDR(SPIM_T *spim)
{
    return ((spim == SPIM0) ? SPIM_HYPER_DMM0_ADDR : SPIM_HYPER_DMM0_ADDR);
}

/**
  * @brief  Set operation mode.
  * @param  spim
  * @param  x SPI Function Operation Mode.
  *           - \ref SPIM_HYPER_OPMODE_IO
  *           - \ref SPIM_HYPER_OPMODE_PAGEWRITE
  *           - \ref SPIM_HYPER_OPMODE_PAGEREAD
  *           - \ref SPIM_HYPER_OPMODE_DIRECTMAP
  * \hideinitializer
  */
__STATIC_INLINE void SPIM_HYPER_SET_OPMODE(SPIM_T *spim, uint32_t x)
{
    SPIM_HYPER_WAIT_DMMDONE(spim);

    (spim->CTL0 = ((spim->CTL0 & ~(SPIM_CTL0_OPMODE_Msk)) |
                   ((x) << SPIM_CTL0_OPMODE_Pos)));
}

/* Initialize SPIM Hyper DLL. */
int32_t SPIM_HYPER_INIT_DLL(SPIM_T *spim);
/* Octal SPI flash and hyper device training DLL API */
int32_t SPIM_HYPER_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum);

/* HyperRAM */
int32_t SPIM_HYPER_ExitHSAndDPD(SPIM_T *spim);
int32_t SPIM_HYPER_ReadHyperRAMReg(SPIM_T *spim, uint32_t u32Addr);
int32_t SPIM_HYPER_WriteHyperRAMReg(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Value);

/* Hyper Device API */
void SPIM_HYPER_Init(SPIM_T *spim, uint32_t u32HyperMode, uint32_t u32Div);
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

/** @} end of group SPIM_HYPER_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_HYPER_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __SPIM_HYPER_H__ */
