/****************************************************************************//**
 * @file     lpi2c.h
 * @version  V1.00
 * @brief    LPI2C driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __LPI2C_H__
#define __LPI2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPI2C_Driver I2C Driver
  @{
*/

/** @addtogroup LPI2C_EXPORTED_CONSTANTS I2C Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C_CTL constant definitions.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define LPI2C_CTL_STA_SI            0x28UL /*!< LPI2C_CTL setting for I2C control bits. It would set STA and SI bits          \hideinitializer */
#define LPI2C_CTL_STA_SI_AA         0x2CUL /*!< LPI2C_CTL setting for I2C control bits. It would set STA, SI and AA bits      \hideinitializer */
#define LPI2C_CTL_STO_SI            0x18UL /*!< LPI2C_CTL setting for I2C control bits. It would set STO and SI bits          \hideinitializer */
#define LPI2C_CTL_STO_SI_AA         0x1CUL /*!< LPI2C_CTL setting for I2C control bits. It would set STO, SI and AA bits      \hideinitializer */
#define LPI2C_CTL_SI                0x08UL /*!< LPI2C_CTL setting for I2C control bits. It would set SI bit                   \hideinitializer */
#define LPI2C_CTL_SI_AA             0x0CUL /*!< LPI2C_CTL setting for I2C control bits. It would set SI and AA bits           \hideinitializer */
#define LPI2C_CTL_STA               0x20UL /*!< LPI2C_CTL setting for I2C control bits. It would set STA bit                  \hideinitializer */
#define LPI2C_CTL_STO               0x10UL /*!< LPI2C_CTL setting for I2C control bits. It would set STO bit                  \hideinitializer */
#define LPI2C_CTL_AA                0x04UL /*!< LPI2C_CTL setting for I2C control bits. It would set AA bit                   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C GCMode constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define LPI2C_GCMODE_ENABLE           1    /*!< Enable  I2C GC Mode                                                         \hideinitializer */
#define LPI2C_GCMODE_DISABLE          0    /*!< Disable I2C GC Mode                                                         \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C AUTOMode constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define LPI2C_AUTO_TXPDMA             (1UL << LPI2C_AUTOCTL_AUTOMODE_Pos)    /*!< LPI2C working in auto TXPDMA mode                                      \hideinitializer */
#define LPI2C_AUTO_RXPDMA             (2UL << LPI2C_AUTOCTL_AUTOMODE_Pos)    /*!< LPI2C working in auto RXPDMA mode                                      \hideinitializer */
#define LPI2C_RANDOM_REPEAT_STA       (3UL << LPI2C_AUTOCTL_AUTOMODE_Pos)    /*!< LPI2C working in random read mode, repeat start between TX/RX          \hideinitializer */
#define LPI2C_RANDOM_STO_STA          (4UL << LPI2C_AUTOCTL_AUTOMODE_Pos)    /*!< LPI2C working in random read mode, stop and start between TX/RX        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C trigger source constant definitions.                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define LPI2C_TRGSRC_LPTMR0             0UL    /*!< LPI2C Auto-operation trigger source from LPTMR0                        \hideinitializer */
#define LPI2C_TRGSRC_LPTMR1             1UL    /*!< LPI2C Auto-operation trigger source from LPTMR1                        \hideinitializer */
#define LPI2C_TRGSRC_TTMR0              2UL    /*!< LPI2C Auto-operation trigger source from TTMR0                         \hideinitializer */
#define LPI2C_TRGSRC_TTMR1              3UL    /*!< LPI2C Auto-operation trigger source from TTMR1                         \hideinitializer */
#define LPI2C_TRGSRC_WKIOA              4UL    /*!< LPI2C Auto-operation trigger source from WKIOA                         \hideinitializer */
#define LPI2C_TRGSRC_WKIOB              5UL    /*!< LPI2C Auto-operation trigger source from WKIOB                         \hideinitializer */
#define LPI2C_TRGSRC_WKIOC              6UL    /*!< LPI2C Auto-operation trigger source from WKIOC                         \hideinitializer */
#define LPI2C_TRGSRC_WKIOD              7UL    /*!< LPI2C Auto-operation trigger source from WKIOD                         \hideinitializer */
#define LPI2C_TRGSRC_SWTRG              8UL    /*!< LPI2C Auto-operation trigger source from SWTRG                         \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define LPI2C_TIMEOUT     SystemCoreClock  /*!< LPI2C time-out counter (1 second time-out)                                 \hideinitializer */
#define LPI2C_OK          ( 0L)            /*!< LPI2C operation OK                                                         \hideinitializer */
#define LPI2C_ERR_FAIL    (-1L)            /*!< LPI2C operation failed                                                     \hideinitializer */
#define LPI2C_ERR_TIMEOUT (-2L)            /*!< LPI2C operation abort due to timeout error                                 \hideinitializer */

/** @} end of group LPI2C_EXPORTED_CONSTANTS */

extern int32_t g_LPI2C_i32ErrCode;

/** @addtogroup LPI2C_EXPORTED_FUNCTIONS I2C Exported Functions
  @{
*/
/**
 *    @brief        The macro is used to set I2C bus condition at One Time
 *
 *    @param[in]    i2c        Specify I2C port
 *    @param[in]    u8Ctrl     A byte writes to I2C control register
 *
 *    @details      Set LPI2C_CTL register to control I2C bus conditions of START, STOP, SI, ACK.
 *    \hideinitializer
 */
#define LPI2C_SET_CONTROL_REG(i2c, u8Ctrl) ((i2c)->CTL0 = ((i2c)->CTL0 & ~0x3c) | (u8Ctrl))

/**
 *    @brief        The macro is used to set START condition of I2C Bus
 *
 *    @param[in]    i2c        Specify I2C port
 *
 *    @details      Set the I2C bus START condition in LPI2C_CTL register.
 *    \hideinitializer
 */
#define LPI2C_START(i2c)  ((i2c)->CTL0 = ((i2c)->CTL0 & ~LPI2C_CTL0_SI_Msk) | LPI2C_CTL0_STA_Msk)

/**
 *    @brief        The macro is used to wait I2C bus status get ready
 *
 *    @param[in]    i2c        Specify I2C port
 *
 *    @details      When a new status is presented of I2C bus, the SI flag will be set in LPI2C_CTL register.
 *    \hideinitializer
 */
#define LPI2C_WAIT_READY(i2c)     while(!((i2c)->CTL0 & LPI2C_CTL0_SI_Msk))

/**
 *    @brief        The macro is used to Read I2C Bus Data Register
 *
 *    @param[in]    i2c        Specify I2C port
 *
 *    @return       A byte of I2C data register
 *
 *    @details      I2C controller read data from bus and save it in I2CDAT register.
 *    \hideinitializer
 */
#define LPI2C_GET_DATA(i2c)   ((i2c)->DAT)

/**
 *    @brief        Write a Data to I2C Data Register
 *
 *    @param[in]    i2c         Specify I2C port
 *    @param[in]    u8Data      A byte that writes to data register
 *
 *    @details      When write a data to LPI2C_DAT register, the I2C controller will shift it to I2C bus.
 *    \hideinitializer
 */
#define LPI2C_SET_DATA(i2c, u8Data) ((i2c)->DAT = (u8Data))

/**
 *    @brief        Get I2C Bus status code
 *
 *    @param[in]    i2c        Specify I2C port
 *
 *    @return       I2C status code
 *
 *    @details      To get this status code to monitor I2C bus event.
 *    \hideinitializer
 */
#define LPI2C_GET_STATUS(i2c) ((i2c)->STATUS0)

/**
 *    @brief        Get Time-out flag from I2C Bus
 *
 *    @param[in]    i2c     Specify I2C port
 *
 *    @retval       0       I2C Bus time-out is not happened
 *    @retval       1       I2C Bus time-out is happened
 *
 *    @details      When I2C bus occurs time-out event, the time-out flag will be set.
 *    \hideinitializer
 */
#define LPI2C_GET_TIMEOUT_FLAG(i2c)   ( ((i2c)->TOCTL & LPI2C_TOCTL_TOIF_Msk) == LPI2C_TOCTL_TOIF_Msk ? 1:0 )

/**
 *    @brief        To get wake-up flag from I2C Bus
 *
 *    @param[in]    i2c     Specify I2C port
 *
 *    @retval       0       Chip is not woken-up from power-down mode
 *    @retval       1       Chip is woken-up from power-down mode
 *
 *    @details      I2C bus occurs wake-up event, wake-up flag will be set.
 *    \hideinitializer
 */
#define LPI2C_GET_WAKEUP_FLAG(i2c) ( ((i2c)->WKSTS & LPI2C_WKSTS_WKIF_Msk) == LPI2C_WKSTS_WKIF_Msk ? 1:0  )

/**
 *    @brief        To clear wake-up flag
 *
 *    @param[in]    i2c     Specify I2C port
 *
 *    @details      If wake-up flag is set, use this macro to clear it.
 *    \hideinitializer
 */
#define LPI2C_CLEAR_WAKEUP_FLAG(i2c)  ((i2c)->WKSTS = LPI2C_WKSTS_WKIF_Msk)

/**
  * @brief      Enable RX PDMA function.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    Set RXPDMAEN bit of LPI2C_CTL1 register to enable RX PDMA transfer function.
  * \hideinitializer
  */
#define LPI2C_ENABLE_RX_PDMA(i2c)   ((i2c)->CTL1 |= LPI2C_CTL1_RXPDMAEN_Msk)

/**
  * @brief      Enable TX PDMA function.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    Set TXPDMAEN bit of LPI2C_CTL1 register to enable TX PDMA transfer function.
  * \hideinitializer
  */
#define LPI2C_ENABLE_TX_PDMA(i2c)   ((i2c)->CTL1 |= LPI2C_CTL1_TXPDMAEN_Msk)

/**
  * @brief      Disable RX PDMA transfer.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    Clear RXPDMAEN bit of LPI2C_CTL1 register to disable RX PDMA transfer function.
  * \hideinitializer
  */
#define LPI2C_DISABLE_RX_PDMA(i2c)   ((i2c)->CTL1 &= ~LPI2C_CTL1_RXPDMAEN_Msk)

/**
  * @brief      Disable TX PDMA transfer.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    Clear TXPDMAEN bit of LPI2C_CTL1 register to disable TX PDMA transfer function.
  * \hideinitializer
  */
#define LPI2C_DISABLE_TX_PDMA(i2c)   ((i2c)->CTL1 &= ~LPI2C_CTL1_TXPDMAEN_Msk)

/**
  * @brief      Enable PDMA stretch function.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    Enable this function is to stretch bus by hardware after PDMA transfer is done if SI is not cleared.
  * \hideinitializer
  */
#define LPI2C_ENABLE_PDMA_STRETCH(i2c)   ((i2c)->CTL1 |= LPI2C_CTL1_PDMASTR_Msk)

/**
  * @brief      Disable PDMA stretch function.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    I2C will send STOP after PDMA transfers done automatically.
  * \hideinitializer
  */
#define LPI2C_DISABLE_PDMA_STRETCH(i2c)   ((i2c)->CTL1 &= ~LPI2C_CTL1_PDMASTR_Msk)

/**
  * @brief      Reset PDMA function.
  *
  * @param[in]  i2c The pointer of the specified I2C module.
  *
  * @details    I2C PDMA engine will be reset after this function is called.
  * \hideinitializer
  */
#define LPI2C_DISABLE_RST_PDMA(i2c)   ((i2c)->CTL1 |= LPI2C_CTL1_PDMARST_Msk)

/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void LPI2C_STOP(LPI2C_T *i2c);

/**
 *    @brief        The macro is used to set STOP condition of I2C Bus
 *
 *    @param[in]    i2c        Specify I2C port
 *
 *    @details      Set the I2C bus STOP condition in LPI2C_CTL register.
 */
__STATIC_INLINE void LPI2C_STOP(LPI2C_T *i2c)
{

    (i2c)->CTL0 |= (LPI2C_CTL0_SI_Msk | LPI2C_CTL0_STO_Msk);

    while (i2c->CTL0 & LPI2C_CTL0_STO_Msk)
    {
    }
}

void LPI2C_ClearTimeoutFlag(LPI2C_T *i2c);
void LPI2C_Close(LPI2C_T *i2c);
void LPI2C_Trigger(LPI2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void LPI2C_DisableInt(LPI2C_T *i2c);
void LPI2C_EnableInt(LPI2C_T *i2c);
uint32_t LPI2C_GetBusClockFreq(LPI2C_T *i2c);
uint32_t LPI2C_GetIntFlag(LPI2C_T *i2c);
uint32_t LPI2C_GetStatus(LPI2C_T *i2c);
uint32_t LPI2C_Open(LPI2C_T *i2c, uint32_t u32BusClock);
uint8_t LPI2C_GetData(LPI2C_T *i2c);
void LPI2C_SetSlaveAddr(LPI2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void LPI2C_SetSlaveAddrMask(LPI2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
uint32_t LPI2C_SetBusClockFreq(LPI2C_T *i2c, uint32_t u32BusClock);
void LPI2C_EnableTimeout(LPI2C_T *i2c, uint8_t u8LongTimeout);
void LPI2C_DisableTimeout(LPI2C_T *i2c);
void LPI2C_EnableWakeup(LPI2C_T *i2c);
void LPI2C_DisableWakeup(LPI2C_T *i2c);
void LPI2C_SetData(LPI2C_T *i2c, uint8_t u8Data);
uint8_t LPI2C_WriteByte(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t LPI2C_WriteMultiBytes(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint8_t LPI2C_WriteByteOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t LPI2C_WriteMultiBytesOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t LPI2C_WriteByteTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t LPI2C_WriteMultiBytesTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t LPI2C_ReadByte(LPI2C_T *i2c, uint8_t u8SlaveAddr);
uint32_t LPI2C_ReadMultiBytes(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t LPI2C_ReadByteOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t LPI2C_ReadMultiBytesOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint8_t LPI2C_ReadByteTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t LPI2C_ReadMultiBytesTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);

/** @} end of group LPI2C_EXPORTED_FUNCTIONS */
/** @} end of group LPI2C_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif
