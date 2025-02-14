/**************************************************************************//**
 * @file     lpi2c.c
 * @version  V1.00
 * @brief    LPI2C driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPI2C_Driver I2C Driver
  @{
*/

int32_t g_LPI2C_i32ErrCode = 0;       /*!< LPI2C global error code */


/** @addtogroup LPI2C_EXPORTED_FUNCTIONS I2C Exported Functions
  @{
*/

/**
  * @brief      Enable specify I2C Controller and set Clock Divider
  *
  * @param[in]  i2c         Specify I2C port
  * @param[in]  u32BusClock The target I2C bus clock in Hz
  *
  * @return     Actual I2C bus clock frequency
  *
  * @details    The function enable the specify I2C Controller and set proper Clock Divider
  *             in I2C CLOCK DIVIDED REGISTER (I2CLK) according to the target I2C Bus clock.
  *             I2C Bus clock = PCLK / (4*(divider+1).
  *
  */
uint32_t LPI2C_Open(LPI2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32Div;
    uint32_t u32Pclk = CLK_GetPCLK4Freq();
    u32Div = (uint32_t)(((u32Pclk * 10U) / (u32BusClock * 4U) + 5U) / 10U - 1U); /* Compute proper divider for I2C clock */
    i2c->CLKDIV = u32Div;

    /* Enable I2C */
    i2c->CTL0 |= LPI2C_CTL0_LPI2CEN_Msk;

    return (u32Pclk / ((u32Div + 1U) << 2U));
}

/**
  * @brief      Disable specify I2C Controller
  *
  * @param[in]  i2c         Specify I2C port
  *
  * @details    Reset I2C Controller and disable specify I2C port.
  *
  */

void LPI2C_Close(LPI2C_T *i2c)
{
    uint32_t u32IsRegLocked;
    u32IsRegLocked = SYS_IsRegLocked();

    if (u32IsRegLocked)
    {
        SYS_UnlockReg();
    }

    /* Reset I2C Controller */
    SYS->LPI2CRST |= SYS_LPI2CRST_LPI2C0RST_Msk;
    SYS->LPI2CRST &= ~SYS_LPI2CRST_LPI2C0RST_Msk;

    /* Disable I2C */
    i2c->CTL0 &= ~LPI2C_CTL0_LPI2CEN_Msk;

    if (u32IsRegLocked)
    {
        SYS_LockReg();
    }
}

/**
  * @brief      Clear Time-out Counter flag
  *
  * @param[in]  i2c         Specify I2C port
  *
  * @details    When Time-out flag will be set, use this function to clear I2C Bus Time-out counter flag .
  *
  */
void LPI2C_ClearTimeoutFlag(LPI2C_T *i2c)
{
    i2c->TOCTL |= LPI2C_TOCTL_TOIF_Msk;
}

/**
  * @brief      Set Control bit of I2C Controller
  *
  * @param[in]  i2c         Specify I2C port
  * @param[in]  u8Start     Set I2C START condition
  * @param[in]  u8Stop      Set I2C STOP condition
  * @param[in]  u8Si        Clear SI flag
  * @param[in]  u8Ack       Set I2C ACK bit
  *
  * @details    The function set I2C Control bit of I2C Bus protocol.
  *
  */
void LPI2C_Trigger(LPI2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack)
{
    uint32_t u32Reg = 0U;

    if (u8Start)
    {
        u32Reg |= LPI2C_CTL_STA;
    }

    if (u8Stop)
    {
        u32Reg |= LPI2C_CTL_STO;
    }

    if (u8Si)
    {
        u32Reg |= LPI2C_CTL_SI;
    }

    if (u8Ack)
    {
        u32Reg |= LPI2C_CTL_AA;
    }

    i2c->CTL0 = (i2c->CTL0 & ~0x3CU) | u32Reg;
}

/**
  * @brief      Disable Interrupt of I2C Controller
  *
  * @param[in]  i2c         Specify I2C port
  *
  * @details    The function is used for disable I2C interrupt
  *
  */
void LPI2C_DisableInt(LPI2C_T *i2c)
{
    i2c->CTL0 &= ~LPI2C_CTL0_INTEN_Msk;
}

/**
  * @brief      Enable Interrupt of I2C Controller
  *
  * @param[in]  i2c         Specify I2C port
  *
  * @details    The function is used for enable I2C interrupt
  *
  */
void LPI2C_EnableInt(LPI2C_T *i2c)
{
    i2c->CTL0 |= LPI2C_CTL0_INTEN_Msk;
}

/**
 * @brief      Get I2C Bus Clock
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @return     The actual I2C Bus clock in Hz
 *
 * @details    To get the actual I2C Bus Clock frequency.
 */
uint32_t LPI2C_GetBusClockFreq(LPI2C_T *i2c)
{
    uint32_t u32Divider = i2c->CLKDIV;
    uint32_t u32Pclk = CLK_GetPCLK4Freq();
    return (u32Pclk / ((u32Divider + 1U) << 2U));
}

/**
 * @brief      Set I2C Bus Clock
 *
 * @param[in]  i2c          Specify I2C port
 * @param[in]  u32BusClock  The target I2C Bus Clock in Hz
 *
 * @return     The actual I2C Bus Clock in Hz
 *
 * @details    To set the actual I2C Bus Clock frequency.
 */
uint32_t LPI2C_SetBusClockFreq(LPI2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32Div;

    uint32_t u32Pclk = CLK_GetPCLK4Freq();
    u32Div = (uint32_t)(((u32Pclk * 10U) / (u32BusClock * 4U) + 5U) / 10U - 1U); /* Compute proper divider for I2C clock */
    i2c->CLKDIV = u32Div;
    return (u32Pclk / ((u32Div + 1U) << 2U));
}

/**
 * @brief      Get Interrupt Flag
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @return     I2C interrupt flag status
 *
 * @details    To get I2C Bus interrupt flag.
 */
uint32_t LPI2C_GetIntFlag(LPI2C_T *i2c)
{
    uint32_t u32Value;

    if ((i2c->CTL0 & LPI2C_CTL0_SI_Msk) == LPI2C_CTL0_SI_Msk)
    {
        u32Value = 1U;
    }
    else
    {
        u32Value = 0U;
    }

    return u32Value;
}

/**
 * @brief      Get I2C Bus Status Code
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @return     I2C Status Code
 *
 * @details    To get I2C Bus Status Code.
 */
uint32_t LPI2C_GetStatus(LPI2C_T *i2c)
{
    return (i2c->STATUS0);
}

/**
 * @brief      Read a Byte from I2C Bus
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @return     I2C Data
 *
 * @details    To read a bytes data from specify I2C port.
 */
uint8_t LPI2C_GetData(LPI2C_T *i2c)
{
    return (uint8_t)(i2c->DAT);
}

/**
 * @brief      Send a byte to I2C Bus
 *
 * @param[in]  i2c          Specify I2C port
 * @param[in]  u8Data       The data to send to I2C bus
 *
 * @details    This function is used to write a byte to specified I2C port
 */
void LPI2C_SetData(LPI2C_T *i2c, uint8_t u8Data)
{
    i2c->DAT = u8Data;
}

/**
 * @brief      Set 7-bit Slave Address and GC Mode
 *
 * @param[in]  i2c          Specify I2C port
 * @param[in]  u8SlaveNo    Set the number of I2C address register (0~3)
 * @param[in]  u8SlaveAddr  7-bit slave address
 * @param[in]  u8GCMode     Enable/Disable GC mode (LPI2C_GCMODE_ENABLE / LPI2C_GCMODE_DISABLE)
 *
 * @details    This function is used to set 7-bit slave addresses in I2C SLAVE ADDRESS REGISTER (I2CADDR0~3)
 *             and enable GC Mode.
 *
 */
void LPI2C_SetSlaveAddr(LPI2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode)
{
    switch (u8SlaveNo)
    {
        case 1:
            i2c->ADDR1  = ((uint32_t)u8SlaveAddr << 1U) | u8GCMode;
            break;

        case 2:
            i2c->ADDR2  = ((uint32_t)u8SlaveAddr << 1U) | u8GCMode;
            break;

        case 3:
            i2c->ADDR3  = ((uint32_t)u8SlaveAddr << 1U) | u8GCMode;
            break;

        case 0:
        default:
            i2c->ADDR0  = ((uint32_t)u8SlaveAddr << 1U) | u8GCMode;
            break;
    }
}

/**
 * @brief      Configure the mask bits of 7-bit Slave Address
 *
 * @param[in]  i2c              Specify I2C port
 * @param[in]  u8SlaveNo        Set the number of I2C address mask register (0~3)
 * @param[in]  u8SlaveAddrMask  A byte for slave address mask
 *
 * @details    This function is used to set 7-bit slave addresses.
 *
 */
void LPI2C_SetSlaveAddrMask(LPI2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask)
{
    switch (u8SlaveNo)
    {
        case 1:
            i2c->ADDRMSK1  = (uint32_t)u8SlaveAddrMask << 1U;
            break;

        case 2:
            i2c->ADDRMSK2  = (uint32_t)u8SlaveAddrMask << 1U;
            break;

        case 3:
            i2c->ADDRMSK3  = (uint32_t)u8SlaveAddrMask << 1U;
            break;

        case 0:
        default:
            i2c->ADDRMSK0  = (uint32_t)u8SlaveAddrMask << 1U;
            break;
    }
}

/**
 * @brief      Enable Time-out Counter Function and support Long Time-out
 *
 * @param[in]  i2c              Specify I2C port
 * @param[in]  u8LongTimeout    Configure DIV4 to enable Long Time-out (0/1)
 *
 * @details    This function enable Time-out Counter function and configure DIV4 to support Long
 *             Time-out.
 *
 */
void LPI2C_EnableTimeout(LPI2C_T *i2c, uint8_t u8LongTimeout)
{
    if (u8LongTimeout)
    {
        i2c->TOCTL |= LPI2C_TOCTL_TOCDIV4_Msk;
    }
    else
    {
        i2c->TOCTL &= ~LPI2C_TOCTL_TOCDIV4_Msk;
    }

    i2c->TOCTL |= LPI2C_TOCTL_TOCEN_Msk;
}

/**
 * @brief      Disable Time-out Counter Function
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @details    To disable Time-out Counter function in I2CTOC register.
 *
 */
void LPI2C_DisableTimeout(LPI2C_T *i2c)
{
    i2c->TOCTL &= ~LPI2C_TOCTL_TOCEN_Msk;
}

/**
 * @brief      Enable I2C Wake-up Function
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @details    To enable Wake-up function of I2C Wake-up control register.
 *
 */
void LPI2C_EnableWakeup(LPI2C_T *i2c)
{
    i2c->WKCTL |= LPI2C_WKCTL_WKEN_Msk;
}

/**
 * @brief      Disable I2C Wake-up Function
 *
 * @param[in]  i2c          Specify I2C port
 *
 * @details    To disable Wake-up function of I2C Wake-up control register.
 *
 */
void LPI2C_DisableWakeup(LPI2C_T *i2c)
{
    i2c->WKCTL &= ~LPI2C_WKCTL_WKEN_Msk;
}

/**
  * @brief      Write a byte to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  data            Write a byte data to Slave
  *
  * @retval     0               Write data success
  * @retval     1               Write data fail, or bus occurs error events
  *
  * @details    The function is used for I2C Master write a byte data to Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint8_t LPI2C_WriteByte(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }


        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));    /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                              /* Clear SI */
                break;

            case 0x18u:                                           /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, data);                          /* Write data to I2CDAT */
                break;

            case 0x20u:                                           /* Slave Address NACK */
            case 0x30u:                                           /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                          /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                u8Ctrl = LPI2C_CTL_STO_SI;                          /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                           /* Arbitration Lost */
            default:                                              /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);      /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                        /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return (u8Err | u8Xfering);                                  /* return (Success)/(Fail) status */
}

/**
  * @brief      Write multi bytes to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  *data           Pointer to array to write data to Slave
  * @param[in]  u32wLen         How many bytes need to write to Slave
  *
  * @return     A length of how many bytes have been transmitted.
  *
  * @details    The function is used for I2C Master write multi bytes data to Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint32_t LPI2C_WriteMultiBytes(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32txLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                              /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));    /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                           /* Clear SI */
                break;

            case 0x18u:                                           /* Slave Address ACK */
            case 0x28u:
                if (u32txLen < u32wLen)
                {
                    LPI2C_SET_DATA(i2c, data[u32txLen++]);                /* Write Data to I2CDAT */
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STO_SI;                   /* Clear SI and send STOP */
                    u8Xfering = 0u;
                }

                break;

            case 0x20u:                                           /* Slave Address NACK */
            case 0x30u:                                           /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                       /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x38u:                                           /* Arbitration Lost */
            default:                                             /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);      /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                        /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32txLen;                                             /* Return bytes length that have been transmitted */
}

/**
  * @brief      Specify a byte register address and write a byte to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u8DataAddr      Specify a address (1 byte) of data write to
  * @param[in]  data            A byte data to write it to Slave
  *
  * @retval     0               Write data success
  * @retval     1               Write data fail, or bus occurs error events
  *
  * @details    The function is used for I2C Master specify a address that data write to in Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint8_t LPI2C_WriteByteOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32txLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                              /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));    /* Send Slave address with write bit */
                u8Ctrl = LPI2C_CTL_SI;                           /* Clear SI */
                break;

            case 0x18u:                                           /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, u8DataAddr);                   /* Write Lo byte address of register */
                break;

            case 0x20u:                                           /* Slave Address NACK */
            case 0x30u:                                           /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                       /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u32txLen < 1u)
                {
                    LPI2C_SET_DATA(i2c, data);
                    u32txLen++;
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STO_SI;                   /* Clear SI and send STOP */
                    u8Xfering = 0u;
                }

                break;

            case 0x38u:                                           /* Arbitration Lost */
            default:                                             /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);      /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                        /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return (u8Err | u8Xfering);                                  /* return (Success)/(Fail) status */
}


/**
  * @brief      Specify a byte register address and write multi bytes to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u8DataAddr      Specify a address (1 byte) of data write to
  * @param[in]  *data           Pointer to array to write data to Slave
  * @param[in]  u32wLen         How many bytes need to write to Slave
  *
  * @return     A length of how many bytes have been transmitted.
  *
  * @details    The function is used for I2C Master specify a byte address that multi data bytes write to in Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint32_t LPI2C_WriteMultiBytesOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32txLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                              /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));    /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;
                break;

            case 0x18u:                                           /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, u8DataAddr);                   /* Write Lo byte address of register */
                break;

            case 0x20u:                                           /* Slave Address NACK */
            case 0x30u:                                           /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                       /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u32txLen < u32wLen)
                {
                    LPI2C_SET_DATA(i2c, data[u32txLen++]);
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STO_SI;                   /* Clear SI and send STOP */
                    u8Xfering = 0u;
                }

                break;

            case 0x38u:                                           /* Arbitration Lost */
            default:                                             /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);        /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                        /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32txLen;                                             /* Return bytes length that have been transmitted */
}

/**
  * @brief      Specify two bytes register address and Write a byte to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 byte) of data write to
  * @param[in]  data            Write a byte data to Slave
  *
  * @retval     0               Write data success
  * @retval     1               Write data fail, or bus occurs error events
  *
  * @details    The function is used for I2C Master specify two bytes address that data write to in Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint8_t LPI2C_WriteByteTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Addr = 1u, u8Ctrl = 0u;
    uint32_t u32txLen = 0u, u32TimeOutCount = 0U;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18u:                                                      /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00u) >> 8u));    /* Write Hi byte address of register */
                break;

            case 0x20u:                                                      /* Slave Address NACK */
            case 0x30u:                                                      /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u8Addr)
                {
                    LPI2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFu));       /* Write Lo byte address of register */
                    u8Addr = 0u;
                }
                else if ((u32txLen < 1u) && (u8Addr == 0u))
                {
                    LPI2C_SET_DATA(i2c, data);
                    u32txLen++;
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STO_SI;                              /* Clear SI and send STOP */
                    u8Xfering = 0u;
                }

                break;

            case 0x38u:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);                   /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return (u8Err | u8Xfering);                                             /* return (Success)/(Fail) status */
}


/**
  * @brief      Specify two bytes register address and write multi bytes to Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data write to
  * @param[in]  data[]          A data array for write data to Slave
  * @param[in]  u32wLen         How many bytes need to write to Slave
  *
  * @return     A length of how many bytes have been transmitted.
  *
  * @details    The function is used for I2C Master specify a byte address that multi data write to in Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */

uint32_t LPI2C_WriteMultiBytesTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Addr = 1u, u8Ctrl = 0u;
    uint32_t u32txLen = 0u, u32TimeOutCount = 0U;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18u:                                                      /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00u) >> 8u));    /* Write Hi byte address of register */
                break;

            case 0x20u:                                                      /* Slave Address NACK */
            case 0x30u:                                                      /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u8Addr)
                {
                    LPI2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFu));       /* Write Lo byte address of register */
                    u8Addr = 0u;
                }
                else if ((u32txLen < u32wLen) && (u8Addr == 0u))
                {
                    LPI2C_SET_DATA(i2c, data[u32txLen++]);                           /* Write data to Register I2CDAT*/
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STO_SI;                              /* Clear SI and send STOP */
                    u8Xfering = 0u;
                }

                break;

            case 0x38u:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);                   /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32txLen;                                                        /* Return bytes length that have been transmitted */
}

/**
  * @brief      Read a byte from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  *
  * @return     Read a byte data from Slave
  *
  * @details    The function is used for I2C Master to read a byte data from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint8_t LPI2C_ReadByte(LPI2C_T *i2c, uint8_t u8SlaveAddr)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, rdata = 0u, u8Ctrl = 0u;
    uint32_t u32TimeOutCount = 0U;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));    /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x40u:                                             /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x48u:                                             /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x58u:
                rdata = (unsigned char) LPI2C_GET_DATA(i2c);         /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                             /* Arbitration Lost */
            default:                                               /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);        /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                          /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    if (u8Err)
    {
        rdata = 0u;                                                 /* If occurs error, return 0 */
    }

    return rdata;                                                  /* Return read data */
}


/**
  * @brief      Read multi bytes from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[out] rdata[]         A data array to store data from Slave
  * @param[in]  u32rLen         How many bytes need to read from Slave
  *
  * @return     A length of how many bytes have been received
  *
  * @details    The function is used for I2C Master to read multi data bytes from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint32_t LPI2C_ReadMultiBytes(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32rxLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));    /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x40u:                                             /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI_AA;                          /* Clear SI and set ACK */
                break;

            case 0x48u:                                             /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x50u:
                rdata[u32rxLen++] = (unsigned char) LPI2C_GET_DATA(i2c);    /* Receive Data */

                if (u32rxLen < (u32rLen - 1u))
                {
                    u8Ctrl = LPI2C_CTL_SI_AA;                             /* Clear SI and set ACK */
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_SI;                                /* Clear SI */
                }

                break;

            case 0x58u:
                rdata[u32rxLen++] = (unsigned char) LPI2C_GET_DATA(i2c);    /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                                /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                                    /* Arbitration Lost */
            default:                                                      /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);               /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                                 /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32rxLen;                                                      /* Return bytes length that have been received */
}


/**
  * @brief      Specify a byte register address and read a byte from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u8DataAddr      Specify a address(1 byte) of data read from
  *
  * @return     Read a byte data from Slave
  *
  * @details    The function is used for I2C Master specify a byte address that a data byte read from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint8_t LPI2C_ReadByteOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, rdata = 0u, u8Ctrl = 0u;
    uint32_t u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));      /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x18u:                                             /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, u8DataAddr);                     /* Write Lo byte address of register */
                break;

            case 0x20u:                                             /* Slave Address NACK */
            case 0x30u:                                             /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                u8Ctrl = LPI2C_CTL_STA_SI;                         /* Send repeat START */
                break;

            case 0x10u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));    /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                               /* Clear SI */
                break;

            case 0x40u:                                             /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x48u:                                             /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x58u:
                rdata = (uint8_t) LPI2C_GET_DATA(i2c);               /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                             /* Arbitration Lost */
            default:                                               /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);        /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                          /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    if (u8Err)
    {
        rdata = 0u;                                                 /* If occurs error, return 0 */
    }

    return rdata;                                                  /* Return read data */
}

/**
  * @brief      Specify a byte register address and read multi bytes from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u8DataAddr      Specify a address (1 bytes) of data read from
  * @param[out] rdata[]         A data array to store data from Slave
  * @param[in]  u32rLen         How many bytes need to read from Slave
  *
  * @return     A length of how many bytes have been received
  *
  * @details    The function is used for I2C Master specify a byte address that multi data bytes read from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint32_t LPI2C_ReadMultiBytesOneReg(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Ctrl = 0u;
    uint32_t u32rxLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));      /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x18u:                                             /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, u8DataAddr);                     /* Write Lo byte address of register */
                break;

            case 0x20u:                                             /* Slave Address NACK */
            case 0x30u:                                             /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                u8Ctrl = LPI2C_CTL_STA_SI;                         /* Send repeat START */
                break;

            case 0x10u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));    /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                             /* Clear SI */
                break;

            case 0x40u:                                             /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI_AA;                          /* Clear SI and set ACK */
                break;

            case 0x48u:                                             /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x50u:
                rdata[u32rxLen++] = (uint8_t) LPI2C_GET_DATA(i2c);   /* Receive Data */

                if (u32rxLen < (u32rLen - 1u))
                {
                    u8Ctrl = LPI2C_CTL_SI_AA;                      /* Clear SI and set ACK */
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_SI;                         /* Clear SI */
                }

                break;

            case 0x58u:
                rdata[u32rxLen++] = (uint8_t) LPI2C_GET_DATA(i2c);   /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                         /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                             /* Arbitration Lost */
            default:                                               /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);        /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                          /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32rxLen;                                               /* Return bytes length that have been received */
}

/**
  * @brief      Specify two bytes register address and read a byte from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify an address(2 bytes) of data read from
  *
  * @return     Read a byte data from Slave
  *
  * @details    The function is used for I2C Master specify two bytes address that a data byte read from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint8_t LPI2C_ReadByteTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, rdata = 0u, u8Addr = 1u, u8Ctrl = 0u;
    uint32_t u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18u:                                                      /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00u) >> 8u));    /* Write Hi byte address of register */
                break;

            case 0x20u:                                                      /* Slave Address NACK */
            case 0x30u:                                                      /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u8Addr)
                {
                    LPI2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFu));       /* Write Lo byte address of register */
                    u8Addr = 0u;
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STA_SI;                              /* Clear SI and send repeat START */
                }

                break;

            case 0x10u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));             /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x40u:                                                      /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x48u:                                                      /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x58u:
                rdata = (unsigned char) LPI2C_GET_DATA(i2c);                  /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);                 /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    if (u8Err)
    {
        rdata = 0u;                                                          /* If occurs error, return 0 */
    }

    return rdata;                                                           /* Return read data */
}

/**
  * @brief      Specify two bytes register address and read multi bytes from Slave
  *
  * @param[in]  *i2c            Point to I2C peripheral
  * @param[in]  u8SlaveAddr     Access Slave address(7-bit)
  * @param[in]  u16DataAddr     Specify a address (2 bytes) of data read from
  * @param[out] rdata[]         A data array to store data from Slave
  * @param[in]  u32rLen         How many bytes need to read from Slave
  *
  * @return     A length of how many bytes have been received
  *
  * @details    The function is used for I2C Master specify two bytes address that multi data bytes read from Slave.
  *
  * @note       This function sets g_LPI2C_i32ErrCode to LPI2C_ERR_TIMEOUT if waiting I2C time-out.
  *
  */
uint32_t LPI2C_ReadMultiBytesTwoRegs(LPI2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen)
{
    uint8_t u8Xfering = 1u, u8Err = 0u, u8Addr = 1u, u8Ctrl = 0u;
    uint32_t u32rxLen = 0u, u32TimeOutCount = 0u;

    g_LPI2C_i32ErrCode = 0;

    LPI2C_START(i2c);                                                         /* Send START */

    while (u8Xfering && (u8Err == 0u))
    {
        u32TimeOutCount = LPI2C_TIMEOUT;
        LPI2C_WAIT_READY(i2c)
        {
            if (--u32TimeOutCount == 0)
            {
                g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
                u8Err = 1u;
                break;
            }
        }

        switch (LPI2C_GET_STATUS(i2c))
        {
            case 0x08u:
                LPI2C_SET_DATA(i2c, (uint8_t)(u8SlaveAddr << 1u | 0x00u));               /* Write SLA+W to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x18u:                                                      /* Slave Address ACK */
                LPI2C_SET_DATA(i2c, (uint8_t)((u16DataAddr & 0xFF00u) >> 8u));    /* Write Hi byte address of register */
                break;

            case 0x20u:                                                      /* Slave Address NACK */
            case 0x30u:                                                      /* Master transmit data NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x28u:
                if (u8Addr)
                {
                    LPI2C_SET_DATA(i2c, (uint8_t)(u16DataAddr & 0xFFu));       /* Write Lo byte address of register */
                    u8Addr = 0u;
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_STA_SI;                              /* Clear SI and send repeat START */
                }

                break;

            case 0x10u:
                LPI2C_SET_DATA(i2c, (uint8_t)((u8SlaveAddr << 1u) | 0x01u));             /* Write SLA+R to Register I2CDAT */
                u8Ctrl = LPI2C_CTL_SI;                                      /* Clear SI */
                break;

            case 0x40u:                                                      /* Slave Address ACK */
                u8Ctrl = LPI2C_CTL_SI_AA;                                   /* Clear SI and set ACK */
                break;

            case 0x48u:                                                      /* Slave Address NACK */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Err = 1u;
                break;

            case 0x50u:
                rdata[u32rxLen++] = (unsigned char) LPI2C_GET_DATA(i2c);      /* Receive Data */

                if (u32rxLen < (u32rLen - 1u))
                {
                    u8Ctrl = LPI2C_CTL_SI_AA;                               /* Clear SI and set ACK */
                }
                else
                {
                    u8Ctrl = LPI2C_CTL_SI;                                  /* Clear SI */
                }

                break;

            case 0x58u:
                rdata[u32rxLen++] = (unsigned char) LPI2C_GET_DATA(i2c);      /* Receive Data */
                u8Ctrl = LPI2C_CTL_STO_SI;                                  /* Clear SI and send STOP */
                u8Xfering = 0u;
                break;

            case 0x38u:                                                      /* Arbitration Lost */
            default:                                                        /* Unknow status */
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);                 /* Clear SI and send STOP */
                u8Ctrl = LPI2C_CTL_SI;
                u8Err = 1u;
                break;
        }

        LPI2C_SET_CONTROL_REG(i2c, u8Ctrl);                                   /* Write controlbit to LPI2C_CTL register */
    }

    u32TimeOutCount = LPI2C_TIMEOUT;

    while ((i2c)->CTL0 & LPI2C_CTL0_STO_Msk)
    {
        u32TimeOutCount--;

        if (u32TimeOutCount == 0)
        {
            g_LPI2C_i32ErrCode = LPI2C_ERR_TIMEOUT;
            u8Err = 1u;
            break;
        }
    }

    return u32rxLen;                                                        /* Return bytes length that have been received */
}

/** @} end of group LPI2C_EXPORTED_FUNCTIONS */
/** @} end of group LPI2C_Driver */
/** @} end of group Standard_Driver */
