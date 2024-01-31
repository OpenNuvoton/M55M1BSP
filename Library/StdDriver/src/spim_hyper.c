/**************************************************************************//**
 * @file    spim_hyper.c
 * @version V1.00
 * @brief   SPIM_HYPER driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPIM_HYPER_Driver SPIM_HYPER Driver
  @{
*/

/** @addtogroup SPIM_HYPER_EXPORTED_FUNCTIONS SPIM_HYPER Exported Functions
  @{
*/

/**
  * @brief      SPIM used to delay the read data strobe (DQS/RWDS) from Octal SPI Flash and Hyper bus device
  * @param      spim
  * @param      u32DelayNum DLL Delay Step Number
  * @return     SPIM_HYPER_OK          SPIM operation OK.
  *             SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note       This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  * @note       First fix DIVIDER to set the frequency of SPIM output bus clock.
  * @note       DLL default setting support
  *             1. Micron MT35XU02GCBA Octal SPI Flash.
  *             2. Winbond W958D8NBYA HyperRAM.
  *             3. Infineon S26KLxxS/S26KSxxS HyperFlash.
  *             Other device users must refer to the device specification set
  *             SPIM_HYPER_SET_DLLCLKON_NUM()
  *             SPIM_HYPER_SET_DLLTRIM_NUM()
  *             SPIM_HYPER_SET_DLLLOCK_NUM()
  *             SPIM_HYPER_SET_DLLOV_NUM()
  */
int32_t SPIM_HYPER_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum)
{
    volatile int i32Timeout = SPIM_HYPER_TIMEOUT;
    uint32_t u32Div = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable SPIM0/1 Module Clock */
    CLK_EnableModuleClock(SPIM0_MODULE);
    CLK_EnableModuleClock(SPIM1_MODULE);

    /* Lock protected registers */
    SYS_LockReg();

    /* SPIM starts to send DLL reference clock to DLL circuit
       that the frequency is the same as the SPIM output bus clock. */
    SPIM_HYPER_ENABLE_DLLOLDO(SPIM0, SPIM_HYPER_OP_ENABLE);
    SPIM_HYPER_ENABLE_DLLOLDO(SPIM1, SPIM_HYPER_OP_ENABLE);

    /* User asserts this control register to 0x1,
       the DLL circuit begins searching for lock with DLL reference clock. */
    SPIM_HYPER_ENABLE_DLLOVRST(SPIM0, SPIM_HYPER_OP_ENABLE);
    SPIM_HYPER_ENABLE_DLLOVRST(SPIM1, SPIM_HYPER_OP_ENABLE);

    u32Div = SPIM_HYPER_GET_CLKDIV(spim);

    if (u32Div >= 3)
    {
        u32Div = 3;
    }

    SPIM_HYPER_SET_DLLDIV(spim, u32Div);

    i32Timeout = SPIM_HYPER_TIMEOUT;

    /* Polling the DLL status register DLLCKON to 0x1,
       and the value 0x1 indicates that clock divider circuit inside DLL is enabled. */
    while (SPIM_HYPER_GET_DLLCLKON(spim) != SPIM_HYPER_OP_ENABLE)
    {
        if (--i32Timeout <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    i32Timeout = SPIM_HYPER_TIMEOUT;

    /* Polling the DLL status register DLLLOCK to 0x1,
       and the value 0x1 indicates that DLL circuit is in lock state */
    while (SPIM_HYPER_GET_DLLLOCK(spim) != SPIM_HYPER_OP_ENABLE)
    {
        if (--i32Timeout <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    i32Timeout = SPIM_HYPER_TIMEOUT;

    /* Polling the DLL status register DLLREADY to 0x1,
       and the value 0x1 indicates that output of DLL circuit is ready. */
    while (SPIM_HYPER_GET_DLLREADY(spim) != SPIM_HYPER_OP_ENABLE)
    {
        if (--i32Timeout <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    /* Set this valid delay number to control register DLL_DNUM. */
    SPIM_HYPER_SET_DLLDLY_NUM(spim, u32DelayNum);

    i32Timeout = SPIM_HYPER_TIMEOUT;

    /* Polling DLL status register DLL_REF to 1
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_HYPER_GET_DLLREF(spim) != SPIM_OP_DISABLE)
    {
        if (--i32Timeout <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief      SPIM Start Transfer And Wait Busy Status.
  * @param      spim
  * @param      u32IsSync   Wait Busy Status
  * @return     SPIM_HYPER_OK          SPIM write done.
  *             SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  */
int32_t SPIM_HYPER_WaitSPIMENDone(SPIM_T *spim, uint32_t u32IsSync)
{
    volatile int32_t i32TimeOutCount = SPIM_HYPER_TIMEOUT;

    SPIM_HYPER_SET_GO(spim);

    if (u32IsSync)
    {
        if (SPIM_HYPER_GET_INT(spim))
        {
            while (SPIM_HYPER_GET_INTSTS(spim))
            {
                if (--i32TimeOutCount <= 0)
                {
                    return SPIM_HYPER_ERR_TIMEOUT;
                }
            }
        }
        else
        {
            while (SPIM_HYPER_IS_BUSY(spim))
            {
                if (--i32TimeOutCount <= 0)
                {
                    return SPIM_HYPER_ERR_TIMEOUT;
                }
            }
        }
    }

    return SPIM_HYPER_OK;
}

/**
 * @brief   Wait Hyper Bus interface is Idle
 * @param   spim
 * @return  SPIM_HYPER_OK          SPIM operation OK.
 *          SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
 * @note    This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
 */
static int32_t SPIM_HYPER_IsCMDIdle(SPIM_T *spim)
{
    volatile int32_t i32TimeOutCnt = SPIM_HYPER_TIMEOUT;

    while (spim->HYPER_CMD != SPIM_HYPER_CMD_IDLE)
    {
        if (--i32TimeOutCnt <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    return SPIM_HYPER_OK;
}

/**
 * @brief   Set Hyper Bus Mode
 *
 * @param   spim
 * @param   u32Div Hyper bus device the setting values of DIVIDER are only 1 and 2.
 * @return  int
 */
void SPIM_HYPER_Init(SPIM_T *spim, uint32_t u32Div)
{
    /* Enable SPIM Hyper Bus Mode */
    SPIM_HYPER_ENABLE_HYPMODE(spim);

    SPIM_HYPER_SET_CLKDIV(spim, u32Div);
}

/**
  * @brief  Reset hyper chip function
  * @param  spim
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Reset(SPIM_T *spim)
{
    spim->HYPER_CMD = SPIM_HYPER_CMD_RESET;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Exit from Hybrid sleep and deep Power down function
  * @param  spim
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_ExitHSAndDPD(SPIM_T *spim)
{
    spim->HYPER_CMD = SPIM_HYPER_CMD_EXIT_HS_PD;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Read hyper chip register space
  * @param  spim
  * @param  u32Addr Address of hyper chip register space
  *                 - \ref HYPERRAM_ID_REG0       : 0x0000_0000 = Identification Register 0
  *                 - \ref HYPERRAM_ID_REG1       : 0x0000_0002 = Identification Register 1
  *                 - \ref HYPERRAM_CONFIG_REG0   : 0x0000_1000 = Configuration Register 0
  *                 - \ref HYPERRAM_CONFIG_REG1   : 0x0000_1002 = Configuration Register 1
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_FAIL    SPIM operation Fail.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_ReadHyperRAMReg(SPIM_T *spim, uint32_t u32Addr)
{
    if ((u32Addr != HYPERRAM_ID_REG0) &&
            (u32Addr != HYPERRAM_ID_REG1) &&
            (u32Addr != HYPERRAM_CONFIG_REG0) &&
            (u32Addr != HYPERRAM_CONFIG_REG1))
    {
        return SPIM_HYPER_ERR_FAIL;
    }

    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_HRAM_REGISTER;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return (spim->HYPER_RDATA & 0xFFFF);
}

/**
  * @brief  Write Hyper Chip register space
  * @param  spim
  * @param  u32Addr Address of Hyper Chip register space
  *                 - \ref HYPERRAM_ID_REG0     : 0x0000_0000 = Identification Register 0
  *                 - \ref HYPERRAM_ID_REG1     : 0x0000_0002 = Identification Register 1
  *                 - \ref HYPERRAM_CONFIG_REG0 : 0x0000_1000 = Configuration Register 0
  *                 - \ref HYPERRAM_CONFIG_REG1 : 0x0000_1002 = Configuration Register 1
  * @param  u32Value Configure HyperRAM Register Value
  * @return SPIM_HYPER_OK      SPIM operation OK.
  *         SPIM_HYPER_ERR_FAIL    SPIM operation Fail.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_WriteHyperRAMReg(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Value)
{
    if ((u32Addr != HYPERRAM_ID_REG0) &&
            (u32Addr != HYPERRAM_ID_REG1) &
            (u32Addr != HYPERRAM_CONFIG_REG0) &&
            (u32Addr != HYPERRAM_CONFIG_REG1))
    {
        return SPIM_HYPER_ERR_FAIL;
    }

    SPIM_HYPER_CLEAR_WDATA(spim);

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = u32Value;

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_HRAM_REGISTER;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Read 1 word from hyper chip space
  * @param  spim
  * @param  u32Addr  Address of hyper chip space
  * @return The 16 bit data of hyper chip space.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
int16_t SPIM_HYPER_Read1Word(SPIM_T *spim, uint32_t u32Addr)
{
    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_1_WORD;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return (spim->HYPER_RDATA & 0xFFFF);
}

/**
  * @brief  Read 2 word from hyper chip space
  * @param  spim
  * @param  u32Addr  Address of hyper chip space
  * @return The 32bit data of hyper chip space.
  */
int32_t SPIM_HYPER_Read2Word(SPIM_T *spim, uint32_t u32Addr)
{
    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_2_WORD;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return (spim->HYPER_RDATA & 0xFFFFFFFF);
}

/**
  * @brief  Write 1 byte to hyper chip space
  * @param  spim
  * @param  u32Addr  Address of hyper chip space
  * @param  u8Data   8 bits data to be written to hyper chip space
  * @return SPIM_HYPER_OK      SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write1Byte(SPIM_T *spim, uint32_t u32Addr, uint8_t u8Data)
{
    SPIM_HYPER_CLEAR_WDATA(spim);

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (uint8_t)(u8Data & 0xFF);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_1_BYTE;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Write 2 bytes to Hyper Chip space
  * @param  spim
  * @param  u32Addr  Address of Hyper Chip space
  * @param  u16Data  16 bits data to be written to Hyper Chip space
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write2Byte(SPIM_T *spim, uint32_t u32Addr, uint16_t u16Data)
{
    SPIM_HYPER_CLEAR_WDATA(spim);

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (uint16_t)(u16Data & 0xFFFF);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_2_BYTE;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Write 3 bytes to Hyper Chip space
  * @param  spim
  * @param  u32Addr  Address of Hyper Chip space
  * @param  u32Data  24 bits data to be written to Hyper Chip space
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write3Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data)
{
    SPIM_HYPER_CLEAR_WDATA(spim);

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (u32Data & 0xFFFFFF);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_3_BYTE;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Write 4 byte to hyper chip space
  * @param  spim
  * @param  u32Addr  Address of hyper chip space
  * @param  u32Data  32 bits data to be written to hyper chip space
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write4Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data)
{
    SPIM_HYPER_CLEAR_WDATA(spim);

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = u32Data;

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_4_BYTE;

    if (SPIM_HYPER_IsCMDIdle(spim) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Write data to HyperBus Module.
  * @param  spim
  * @param  u32Addr     Start address to write.
  * @param  pu8WrBuf    Transmit buffer.
  * @param  u32NTx      Number of bytes to write.
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_DMAWrite(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8WrBuf, uint32_t u32NTx)
{
    if (pu8WrBuf == NULL)
    {
        return SPIM_HYPER_ERR_FAIL;
    }

    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_PAGEWRITE);  /* Switch to DMA Write mode.   */

    spim->SRAMADDR = (uint32_t) pu8WrBuf;                /* SRAM u32Address.  */
    spim->DMACNT = u32NTx;                              /* Transfer length.  */
    spim->FADDR = u32Addr;                              /* Flash u32Address. */

    if (SPIM_HYPER_WaitSPIMENDone(spim, SPIM_HYPER_OP_ENABLE) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}

#if 0 //(TESTCHIP_ONLY)
/**
  * @brief  Read data from HyperBus Module.
  * @param  spim
  * @param  u32Addr     Start address to read.
  * @param  pu8RdBuf    Receive buffer.
  * @param  u32NRx      Number of bytes to read.
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_DMARead(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8RdBuf, uint32_t u32NRx)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_PAGEREAD);   /* Switch to DMA Write mode.   */

    spim->SRAMADDR = (uint32_t) pu8RdBuf;               /* SRAM u32Address. */
    spim->DMACNT = u32NRx;                              /* Transfer length. */
    spim->FADDR = u32Addr;                              /* Flash u32Address. */

    if (SPIM_HYPER_WaitSPIMENDone(spim, SPIM_HYPER_OP_ENABLE) != SPIM_HYPER_OK)
    {
        return SPIM_HYPER_ERR_TIMEOUT;
    }

    return SPIM_HYPER_OK;
}
#endif

/**
  * @brief  SPIM Hyper Mode Enter DMM Mode
  * @param  spim
  * @return None.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
void SPIM_HYPER_EnterDirectMapMode(SPIM_T *spim)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_DIRECTMAP);  /* Switch to DMA Write mode.   */
}

/**
  * @brief  SPIM Hyper Mode Exit DMM Mode
  * @param  spim
  * @return None.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
void SPIM_HYPER_ExitDirectMapMode(SPIM_T *spim)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_IO);       /* Switch back to Normal mode.  */
}

/**
  * @brief  Wait Hyper Direct Map Mode Read/Write Done.
  * @param  spim
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_IsDMMDone(SPIM_T *spim)
{
    volatile int32_t u32TimeOutCount = SPIM_HYPER_TIMEOUT;

    SPIM_HYPER_ENABLE_DMMDONE(spim);       /* HyperBus DMM Mode Done.  */

    while (SPIM_HYPER_GET_DMMDONE(spim))
    {
        if (--u32TimeOutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    return SPIM_HYPER_OK;
}

/** @} end of group SPIM_HYPER_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_HYPER_Driver */
/** @} end of group Standard_Driver */
