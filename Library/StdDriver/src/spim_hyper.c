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

#ifndef SPIM_HYPER_TRIM_HYPERDLL
    #define SPIM_HYPER_TRIM_HYPERDLL            (1)
#endif

#define SPIM_HYPER_ALTCTL0_DLL0TMEN_Pos         (8)
#define SPIM_HYPER_ALTCTL0_DLL0TMEN_Msk         (0x1UL << SPIM_HYPER_ALTCTL0_DLL0TMEN_Pos)

#define SPIM_HYPER_DLL0ATCTL0_TUDOFF_Pos        (9)
#define SPIM_HYPER_DLL0ATCTL0_TUDOFF_Msk        (0x1UL << SPIM_HYPER_DLL0ATCTL0_TUDOFF_Pos)

#define SPIM_HYPER_MLDOTCTL_MLDOPL0VT_Pos       (0)
#define SPIM_HYPER_MLDOTCTL_MLDOPL0VT_Msk       (0x3FUL << SPIM_HYPER_MLDOTCTL_MLDOPL0VT_Pos)

#define SPIM_HYPER_MLDOTCTL_WRBUSY_Pos          (31)
#define SPIM_HYPER_MLDOTCTL_WRBUSY_Msk          (0x1UL << SPIM_HYPER_MLDOTCTL_WRBUSY_Pos)

#define SPIM_HYPER_DLLTCTL_DLL0OLDOTRIM_Pos     (0)
#define SPIM_HYPER_DLLTCTL_DLL0OLDOTRIM_Msk     (0xFUL << SPIM_HYPER_DLLTCTL_DLL0OLDOTRIM_Pos)

#define SPIM_HYPER_ENABLE_SYSDLL0TMEN()                                                     \
    do{                                                                                     \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00UL)) | SPIM_HYPER_ALTCTL0_DLL0TMEN_Msk);   \
        outpw((SYS_BASE + 0xE00UL), u32Value);                                                \
    }while(0)

#define SPIM_HYPER_DISABLE_SYSDLL0TMEN()                                                    \
    do{                                                                                     \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00UL)) & ~SPIM_HYPER_ALTCTL0_DLL0TMEN_Msk);  \
        outpw((SYS_BASE + 0xE00UL), u32Value);                                                                                                \
    }while(0)

#define SPIM_HYPER_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                        \
    do{                                                                                     \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84UL)) & ~SPIM_HYPER_DLL0ATCTL0_TUDOFF_Msk); \
        outpw((SYS_BASE + 0xF84UL), u32Value); \
    }while(0)

#define SPIM_HYPER_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                       \
    do{                                                                                     \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84UL)) | SPIM_HYPER_DLL0ATCTL0_TUDOFF_Msk);  \
        outpw((SYS_BASE + 0xF84UL), u32Value);                                                \
    }while(0)

#define SPIM_HYPER_CEIL_DIV(x, y)               (((x) + (y) - 1U) / (y))

// Timing margin in permille (1000 = 100%). Set to 1100 to apply a +10% safety margin.
#define SPIM_HYPER_TRIM_MARGIN                  (1000U)

//------------------------------------------------------------------------------
/**
  * @brief      Wait Hyper Bus interface Idle
  * @param      spim    Pointer to the SPIM peripheral.
  * @retval     SPIM_HYPER_OK          Operation completed successfully.
  *             SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @details    This function is used to wait Hyper Bus interface idle.
  */
static int32_t spim_hyper_wait_cmdidle(const SPIM_T *spim)
{
    volatile int32_t i32TimeOutCnt = (int32_t)SPIM_HYPER_TIMEOUT;

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
  * @brief      SPIM Start Transfer And Wait Busy Status.
  * @param      spim        Pointer to the SPIM peripheral.
  * @param      u32IsSync   Wait Busy Status
  * @return     SPIM_HYPER_OK          SPIM write done.
  *             SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  */
static int32_t SPIM_HYPER_WaitSPIMENDone(SPIM_T *spim, uint32_t u32IsSync)
{
    volatile int32_t i32TimeOutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    /* Start SPIM Transfer, wait for operation complete according to u32IsSync */
    SPIM_HYPER_SET_GO(spim);

    if (u32IsSync)  /* Wait SPIM Busy status if u32IsSync is set */
    {
        while ((SPIM_HYPER_GET_INT(spim) ? SPIM_HYPER_GET_INTSTS(spim) : SPIM_HYPER_IS_BUSY(spim)))
        {
            if (--i32TimeOutCount <= 0)
            {
                return SPIM_HYPER_ERR_TIMEOUT;
            }
        }
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief  Wait Hyper Direct Map Mode Read/Write Done.
  * @param  spim
  * @return SPIM_HYPER_OK          SPIM operation OK.
  *         SPIM_HYPER_ERR_TIMEOUT SPIM operation abort due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_WAIT_DMMDONE(SPIM_T *spim)
{
    volatile int32_t i32TimeOutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    SPIM_HYPER_ENABLE_DMMDONE(spim);       /* HyperBus DMM Mode Done.  */

    while ((SPIM_HYPER_GET_DMMDONE(spim) != 0U) && (--i32TimeOutCount > 0)) {}

    i32TimeOutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    /* Wait for DMM mode to be idle */
    while ((SPIM_HYPER_GET_DMM_IDLE(spim) == SPIM_HYPER_OP_DISABLE) && (--i32TimeOutCount > 0)) {}

    return SPIM_HYPER_OK;
}

/**
  * @brief  Get Direct Map Address.
  * @param  spim
  * @return Direct Mapping Address.
  */
uint32_t SPIM_HYPER_GET_DMMADDR(const SPIM_T *spim)
{
    (void)spim;

    return SPIM_HYPER_DMM0_ADDR;
}

/**
  * @brief      Enable Hyper Bus Mode
  * @param      spim         Pointer to the SPIM peripheral.
  * @param[in]  u32HyperMode Select HyperRAM or HyperFlash Device
  *                         - \ref SPIM_HYPERRAM_MODE
  *                         - \ref SPIM_HYPERFLASH_MODE
  * @param[in]  u32Div       Hyper Bus device clock divider, the setting values of DIVIDER are only 1 and 2.
  * @return     None.
  * @details    This function is used to enable SPIM Hyper Bus Mode and set the clock divider.
  */
void SPIM_HYPER_Init(SPIM_T *spim, uint32_t u32HyperMode, uint32_t u32Div)
{
    /* Enable SPIM Hyper Bus Mode */
    SPIM_HYPER_ENABLE_HYPMODE(spim, u32HyperMode);

    /* Sets the clock divider for the SPIM hyper interface. */
    SPIM_HYPER_SET_CLKDIV(spim, (((u32Div != 1UL) && (u32Div != 2UL)) ? 1UL : u32Div));
}

/**
  * @brief  Reset hyper chip function
  * @param  spim    Pointer to the SPIM peripheral.
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Reset(SPIM_T *spim)
{
    volatile uint32_t u32Delay = 0;
    int32_t i32Ret = SPIM_HYPER_ERR_FAIL;

    spim->HYPER_CMD = SPIM_HYPER_CMD_RESET;

    i32Ret = spim_hyper_wait_cmdidle(spim);

    /* Delay for chip reset */
    for (u32Delay = 0; u32Delay < 0x3000UL; u32Delay++) {}

    return i32Ret;
}

/**
  * @brief  Exit from Hybrid sleep and deep Power down function
  * @param  spim    Pointer to the SPIM peripheral.
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_ExitHSAndDPD(SPIM_T *spim)
{
    spim->HYPER_CMD = SPIM_HYPER_CMD_EXIT_HS_PD;

    return spim_hyper_wait_cmdidle(spim);
}

/**
  * @brief  Read hyper chip register space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of hyper chip register space
  *                 - \ref SPIM_HYPER_HRAM_ID_REG0       : 0x0000_0000 = Identification Register 0
  *                 - \ref SPIM_HYPER_HRAM_ID_REG1       : 0x0000_0002 = Identification Register 1
  *                 - \ref SPIM_HYPER_HRAM_CONFIG_REG0   : 0x0000_1000 = Configuration Register 0
  *                 - \ref SPIM_HYPER_HRAM_CONFIG_REG1   : 0x0000_1002 = Configuration Register 1
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_FAIL    SPIM operation Fail.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_ReadHyperRAMReg(SPIM_T *spim, uint32_t u32Addr)
{
    if ((u32Addr != SPIM_HYPER_HRAM_ID_REG0) &&
            (u32Addr != SPIM_HYPER_HRAM_ID_REG1) &&
            (u32Addr != SPIM_HYPER_HRAM_CONFIG_REG0) &&
            (u32Addr != SPIM_HYPER_HRAM_CONFIG_REG1))
    {
        return SPIM_HYPER_ERR_FAIL;
    }

    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_HRAM_REGISTER;

    (void)spim_hyper_wait_cmdidle(spim);

    return (spim->HYPER_RDATA & 0xFFFFUL); // Return 16-bit value
}

/**
  * @brief  Write Hyper Chip register space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of Hyper Chip register space
  *                 - \ref SPIM_HYPER_HRAM_ID_REG0     : 0x0000_0000 = Identification Register 0
  *                 - \ref SPIM_HYPER_HRAM_ID_REG1     : 0x0000_0002 = Identification Register 1
  *                 - \ref SPIM_HYPER_HRAM_CONFIG_REG0 : 0x0000_1000 = Configuration Register 0
  *                 - \ref SPIM_HYPER_HRAM_CONFIG_REG1 : 0x0000_1002 = Configuration Register 1
  * @param  u32Value Configure HyperRAM Register Value
  * @return SPIM_HYPER_OK      Operation completed successfully.
  *         SPIM_HYPER_ERR_FAIL    SPIM operation Fail.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_WriteHyperRAMReg(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Value)
{
    if ((u32Addr != SPIM_HYPER_HRAM_ID_REG0) &&
            (u32Addr != SPIM_HYPER_HRAM_ID_REG1) &&
            (u32Addr != SPIM_HYPER_HRAM_CONFIG_REG0) &&
            (u32Addr != SPIM_HYPER_HRAM_CONFIG_REG1))
    {
        return SPIM_HYPER_ERR_FAIL;
    }

    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = u32Value;

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_HRAM_REGISTER;

    (void)spim_hyper_wait_cmdidle(spim);

    return SPIM_HYPER_OK;
}

/**
  * @brief  Read 1 word from hyper chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of hyper chip space
  * @return The 16 bit data of hyper chip space.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
uint16_t SPIM_HYPER_Read1Word(SPIM_T *spim, uint32_t u32Addr)
{
    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_1_WORD;

    (void)spim_hyper_wait_cmdidle(spim);

    return (uint16_t)(spim->HYPER_RDATA & 0xFFFFUL);
}

/**
  * @brief  Read 2 word from hyper chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of hyper chip space
  * @return The 32bit data of hyper chip space.
  */
uint32_t SPIM_HYPER_Read2Word(SPIM_T *spim, uint32_t u32Addr)
{
    spim->HYPER_ADR = u32Addr;

    spim->HYPER_CMD = SPIM_HYPER_CMD_READ_2_WORD;

    (void)spim_hyper_wait_cmdidle(spim);

    return (uint32_t)(spim->HYPER_RDATA & 0xFFFFFFFFUL);
}

/**
  * @brief  Write 1 byte to hyper chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of hyper chip space
  * @param  u8Data  8 bits data to be written to hyper chip space
  * @return SPIM_HYPER_OK      Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write1Byte(SPIM_T *spim, uint32_t u32Addr, uint8_t u8Data)
{
    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (uint8_t)(u8Data & 0xFFU);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_1_BYTE;

    return spim_hyper_wait_cmdidle(spim);
}

/**
  * @brief  Write 2 bytes to Hyper Chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of Hyper Chip space
  * @param  u16Data 16 bits data to be written to Hyper Chip space
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write2Byte(SPIM_T *spim, uint32_t u32Addr, uint16_t u16Data)
{
    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (uint16_t)(u16Data & 0xFFFFU);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_2_BYTE;

    return spim_hyper_wait_cmdidle(spim);
}

/**
  * @brief  Write 3 bytes to Hyper Chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of Hyper Chip space
  * @param  u32Data 24 bits data to be written to Hyper Chip space
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write3Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data)
{
    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (u32Data & 0xFFFFFFU);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_3_BYTE;

    return spim_hyper_wait_cmdidle(spim);
}

/**
  * @brief  Write 4 byte to hyper chip space
  * @param  spim    Pointer to the SPIM peripheral.
  * @param  u32Addr Address of hyper chip space
  * @param  u32Data 32 bits data to be written to hyper chip space
  * @return SPIM_HYPER_OK          Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT, if waiting Hyper Chip time-out.
  */
int32_t SPIM_HYPER_Write4Byte(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Data)
{
    spim->HYPER_ADR = u32Addr;
    spim->HYPER_WDATA = (u32Data & 0xFFFFFFFFUL);

    spim->HYPER_CMD = SPIM_HYPER_CMD_WRITE_4_BYTE;

    return spim_hyper_wait_cmdidle(spim);
}

/**
  * @brief  Write data to the HyperBus module using DMA.
  * @param  spim        Pointer to the SPIM peripheral.
  * @param  u32Addr     Start address to write to (HyperBus address).
  * @param  pu8WrBuf    Pointer to the transmit buffer.
  * @param  u32NTx      Number of bytes to write.
  * @return SPIM_HYPER_OK           Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT  Operation aborted due to timeout error.
  * @note   This function returns SPIM_HYPER_ERR_TIMEOUT if the HyperBus device times out.
  *         The number of bytes to write must be a multiple of 8 (for DMA alignment).
  */
int32_t SPIM_HYPER_DMAWrite(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8WrBuf, uint32_t u32NTx)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_PAGEWRITE);  /* Switch to DMA write mode. */

    spim->SRAMADDR = (uint32_t)pu8WrBuf;  /* Set the SRAM buffer address. */
    spim->DMACNT   = u32NTx;              /* Set the transfer length. */
    spim->FADDR    = u32Addr;             /* Set the HyperBus target address. */

    return SPIM_HYPER_WaitSPIMENDone(spim, SPIM_HYPER_OP_ENABLE);
}

/**
  * @brief  Read data from the HyperBus module using DMA.
  * @param  spim        Pointer to the SPIM peripheral.
  * @param  u32Addr     Start address to read from (HyperBus address).
  * @param  pu8RdBuf    Pointer to the receive buffer.
  * @param  u32NRx      Number of bytes to read.
  * @return SPIM_HYPER_OK           Operation completed successfully.
  *         SPIM_HYPER_ERR_TIMEOUT  Operation aborted due to timeout error.
  * @note   This function returns SPIM_HYPER_ERR_TIMEOUT if the HyperBus device times out.
  *         The number of bytes to read must be a multiple of 8 (for DMA alignment).
  */
int32_t SPIM_HYPER_DMARead(SPIM_T *spim, uint32_t u32Addr, uint8_t *pu8RdBuf, uint32_t u32NRx)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_PAGEREAD);  /* Switch to DMA read mode. */
    spim->SRAMADDR = (uint32_t)pu8RdBuf;                      /* Set the SRAM buffer address. */
    spim->DMACNT   = u32NRx;                                  /* Set the transfer length. */
    spim->FADDR    = u32Addr;                                 /* Set the HyperBus target address. */

    return SPIM_HYPER_WaitSPIMENDone(spim, SPIM_HYPER_OP_ENABLE);
}

/**
  * @brief  SPIM Hyper Mode Enter DMM Mode
  * @param  spim    Pointer to the SPIM peripheral.
  * @return None.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
void SPIM_HYPER_EnterDirectMapMode(SPIM_T *spim)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_DIRECTMAP);  /* Switch to DMA Write mode.   */
}

/**
  * @brief  SPIM Hyper Mode Exit DMM Mode
  * @param  spim    Pointer to the SPIM peripheral.
  * @return None.
  * @note   This function sets SPIM_HYPER_ERR_TIMEOUT if waiting Hyper Chip time-out.
  */
void SPIM_HYPER_ExitDirectMapMode(SPIM_T *spim)
{
    SPIM_HYPER_SET_OPMODE(spim, SPIM_HYPER_OPMODE_IO);       /* Switch back to Normal mode.  */
}

/**
  * @brief      Initialize SPIM Hyper DLL.
  * @param      spim    Pointer to the SPIM peripheral.
  * @return     SPIM_HYPER_OK          Operation completed successfully.
  *             SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @details    This function is used to initialize SPIM Hyper bus DLL.
  * @note       When system wake up reset, please calling this function reinintialize DLL.
  */
int32_t SPIM_HYPER_INIT_DLL(SPIM_T *spim)
{
    volatile int32_t i32TimeoutCount;
    uint32_t u32BusClkHz = CLK_GetSCLKFreq() / (SPIM_HYPER_GET_CLKDIV(spim) * 2UL);
    uint32_t u32ClkNs = 1000000000UL / u32BusClkHz;
    uint32_t u32DelayCycleCount = 0;
    const uint32_t u32DllLockUs  = 20;
    const uint32_t u32DllValidUs = 50;
    const uint32_t u32DllClkOnUs = 20;
    const uint32_t u32DllTrimUs  = 20;

    // Convert to cycles
    uint32_t u32DLLLKNUM = SPIM_HYPER_CEIL_DIV((u32DllLockUs * SPIM_HYPER_TRIM_MARGIN), u32ClkNs);
    uint32_t u32DLLOVNUM = SPIM_HYPER_CEIL_DIV((u32DllValidUs * SPIM_HYPER_TRIM_MARGIN), u32ClkNs);
    uint32_t u32CLKONNUM = SPIM_HYPER_CEIL_DIV((u32DllClkOnUs * SPIM_HYPER_TRIM_MARGIN), u32ClkNs);
    uint32_t u32TRIMNUM  = SPIM_HYPER_CEIL_DIV((u32DllTrimUs * SPIM_HYPER_TRIM_MARGIN), u32ClkNs);

    uint32_t u32Div = (SPIM_HYPER_GET_CLKDIV(spim) * 2UL);
    uint32_t u32FreqMHz = u32BusClkHz / 1000000UL;
    uint32_t u32FastEn = (u32FreqMHz <= 100UL) ? SPIM_HYPER_OP_DISABLE : SPIM_HYPER_OP_ENABLE;

    uint32_t u32DllDivCode = (u32Div <= 1UL) ? 0UL : (u32Div == 2UL) ? 1UL : (u32Div == 4UL) ? 2UL : 3UL;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    // DLL timing setup
    SPIM_HYPER_SET_DLLLOCK_NUM(spim, u32DLLLKNUM);
    SPIM_HYPER_SET_DLLOV_NUM(spim, u32DLLOVNUM);
    SPIM_HYPER_SET_DLLCLKON_NUM(spim, u32CLKONNUM);
    SPIM_HYPER_SET_DLLTRIM_NUM(spim, u32TRIMNUM);

#if (SPIM_HYPER_TRIM_HYPERDLL == 1)
    SPIM_HYPER_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF();
#endif

    SPIM_HYPER_SET_DLLDIV(spim, u32DllDivCode);
    SPIM_HYPER_SET_DLLFAST(spim, u32FastEn);
    SPIM_HYPER_ENABLE_DLLOLDO(spim, SPIM_HYPER_OP_ENABLE);

#if (SPIM_HYPER_TRIM_HYPERDLL == 1)
    SPIM_HYPER_SET_INTERNAL_RWDS(spim, SPIM_OP_ENABLE);
    SPIM_HYPER_SET_AUTO_TRIM_DLL(spim, SPIM_OP_ENABLE);
#endif

    // Start DLL output valid countdown
    SPIM_HYPER_ENABLE_DLLOVRST(spim, SPIM_HYPER_OP_ENABLE);

    // Wait DLLOVRST auto-clear
    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    while (SPIM_HYPER_GET_DLLOVRST(spim))
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    // Wait clock divider ON
    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    while (SPIM_HYPER_GET_DLLCLKON(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    // Wait DLL LOCK
    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    while (SPIM_HYPER_GET_DLLLOCK(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    // Wait DLL output READY
    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    while (SPIM_HYPER_GET_DLLREADY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

#if (SPIM_HYPER_TRIM_HYPERDLL == 1)
    // Wait auto-trim ready
    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    while (SPIM_HYPER_GET_DLLATRDY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    u32DelayCycleCount = (uint32_t)(u32BusClkHz / 1000UL) * 3UL;  // 3ms = 3 * (Hz / 1000)

    for (i32TimeoutCount = 0; i32TimeoutCount < (int32_t)u32DelayCycleCount; i32TimeoutCount++)
    {
    }

    SPIM_HYPER_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF();
    SPIM_HYPER_SET_INTERNAL_RWDS(spim, SPIM_HYPER_OP_DISABLE);
#endif

    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }

    return SPIM_HYPER_OK;
}

/**
  * @brief      Set SPIM DLL delay number.
  * @param      spim    Pointer to the SPIM peripheral.
  * @param      u32DelayNum The delay number of SPIM DLL.
  * @return     SPIM_HYPER_OK          Operation completed successfully.
  *             SPIM_HYPER_ERR_TIMEOUT Operation aborted due to timeout error.
  * @details    This function is used to set SPIM DLL delay number.
  *             The delay number is used to adjust the sampling edge of input clock
  *             to SPIM.
  */
int32_t SPIM_HYPER_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum)
{
    volatile int i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    if (SPIM_HYPER_GET_DLLOLDO(spim) != SPIM_HYPER_OP_ENABLE)
    {
        if (SPIM_HYPER_INIT_DLL(spim) != SPIM_HYPER_OK)
        {
            return SPIM_HYPER_ERR_FAIL;
        }
    }

    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    /* Polling DLL status register DLL_REF to 0
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_HYPER_GET_DLLREF(spim) != SPIM_HYPER_OP_DISABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    /* Set this valid delay number to control register DLL_DNUM. */
    SPIM_HYPER_SET_DLLDLY_NUM(spim, u32DelayNum);

    i32TimeoutCount = (int32_t)SPIM_HYPER_TIMEOUT;

    /* Polling DLL status register DLL_REF to 0
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_HYPER_GET_DLLREF(spim) != SPIM_HYPER_OP_DISABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_HYPER_ERR_TIMEOUT;
        }
    }

    return SPIM_HYPER_OK;
}

/** @} end of group SPIM_HYPER_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_HYPER_Driver */
/** @} end of group Standard_Driver */
