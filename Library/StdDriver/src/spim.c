/**************************************************************************//**
 * @file    spim.c
 * @version V1.00
 * @brief   SPIM driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPIM_Driver SPIM Driver
  @{
*/

/** @addtogroup SPIM_EXPORTED_FUNCTIONS SPIM Exported Functions
  @{
*/


/** @cond HIDDEN_SYMBOLS */


#define ENABLE_DEBUG                      0

#if ENABLE_DEBUG
    #define SPIM_DBGMSG   printf
#else
    #define SPIM_DBGMSG(...)   do { } while (0)      /* disable debug */
#endif

#ifndef SPIM_TRIM_HYPERDLL
    #define SPIM_TRIM_HYPERDLL            (1)
#endif

#define SPIM_ALTCTL0_DLL0TMEN_Pos         (8)
#define SPIM_ALTCTL0_DLL0TMEN_Msk         ((uint32_t)0x1U << SPIM_ALTCTL0_DLL0TMEN_Pos)

#define SPIM_DLL0ATCTL0_TUDOFF_Pos        (9)
#define SPIM_DLL0ATCTL0_TUDOFF_Msk        ((uint32_t)0x1U << SPIM_DLL0ATCTL0_TUDOFF_Pos)

#define SPIM_MLDOTCTL_MLDOPL0VT_Pos       (0)
#define SPIM_MLDOTCTL_MLDOPL0VT_Msk       ((uint32_t)0x3FU << SPIM_MLDOTCTL_MLDOPL0VT_Pos)

#define SPIM_MLDOTCTL_WRBUSY_Pos          (31)
#define SPIM_MLDOTCTL_WRBUSY_Msk          ((uint32_t)0x1U << SPIM_MLDOTCTL_WRBUSY_Pos)

#define SPIM_DLLTCTL_DLL0OLDOTRIM_Pos     (0)
#define SPIM_DLLTCTL_DLL0OLDOTRIM_Msk     ((uint32_t)0xFU << SPIM_DLLTCTL_DLL0OLDOTRIM_Pos)

#define SPIM_ENABLE_SYSDLL0TMEN()                                                       \
    do{                                                                                 \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00UL)) | SPIM_ALTCTL0_DLL0TMEN_Msk);     \
        outpw((SYS_BASE + 0xE00UL), u32Value);                                            \
    }while(0)

#define SPIM_DISABLE_SYSDLL0TMEN()                                                      \
    do{                                                                                 \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00UL)) & ~SPIM_ALTCTL0_DLL0TMEN_Msk);    \
        outpw((SYS_BASE + 0xE00UL), u32Value);                                            \
    }while(0)

#define SPIM_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                          \
    do{                                                                                 \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84UL)) & ~SPIM_DLL0ATCTL0_TUDOFF_Msk);   \
        outpw((SYS_BASE + 0xF84UL), u32Value);                                            \
    }while(0)

#define SPIM_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                         \
    do{                                                                                 \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84UL)) | SPIM_DLL0ATCTL0_TUDOFF_Msk);    \
        outpw((SYS_BASE + 0xF84UL), u32Value);                                            \
    }while(0)

#define SPIM_CEIL_DIV(x, y)               (((x) + (y) - 1U) / (y))

// Timing margin in permille (1000 = 100%). Set to 1100 to apply a +10% safety margin.
#define SPIM_TRIM_MARGIN                  (1000U)

//------------------------------------------------------------------------------
typedef struct
{
    uint32_t u32Saved;
    uint32_t u32Div;
    uint32_t u32RxClkDly;
} SPIM_DIV_CTX;

//------------------------------------------------------------------------------
static uint8_t gau8IDBuf[3] = {0};

static int32_t _SPIM_WriteData(SPIM_T *spim, const uint8_t *pu8TxBuf, uint32_t u32NTx, uint32_t u32NBit);
static int32_t _SPIM_ReadData(SPIM_T *spim, uint8_t *pu8RxBuf, uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_WriteStatusRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit);
static void _SPIM_ReadStatusRegister2(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_WriteStatusRegister2(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit);
static void _SPIM_ReadStatusRegister3(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_ReadSecurityRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit);
static int _SPIM_IsWriteDone(SPIM_T *spim, uint32_t u32NBit);
static int _SPIM_WaitWriteDone(SPIM_T *spim, uint32_t u32NBit);
static void _SPIM_EnableSpansionQuadMode(SPIM_T *spim, uint32_t u32IsEn);
static void _SPIM_EonSetQpiMode(SPIM_T *spim, int32_t i32IsEn);
static void _SPIM_SPANSION4BytesEnable(SPIM_T *spim, uint32_t u32IsEn, uint32_t u32NBit);
static void _SPIM_WriteInPageDataByIo(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, const uint8_t pu8TxBuf[], uint8_t u8WrCmd,
                                      uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, uint32_t u32IsSync);
static int32_t _SPIM_WriteInPageDataByPageWrite(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx,
                                                uint8_t *pu8TxBuf, uint8_t u8WrCmd, uint32_t u32IsSync);
static void _SPIM_ClearContReadPhase(SPIM_T *spim, uint32_t u32OPMode);
static void _SPIM_Sync4ByteAddrMode(SPIM_T *spim, uint32_t u32Is4ByteAddr, uint32_t u32CmdBit);
static uint32_t _SPIM_IsCrenAllowed(uint8_t u8Cmd);

//------------------------------------------------------------------------------
/**
 * @brief    Switch SPIM output to the specified bit mode.
 * @param[in]  spim     Pointer to the SPIM peripheral.
 * @param[in]  u32NBit  Bit width to set for output mode.
 *                      - \ref SPIM_BITMODE_1 : 1-bit mode
 *                      - \ref SPIM_BITMODE_2 : 2-bit mode (dual)
 *                      - \ref SPIM_BITMODE_4 : 4-bit mode (quad)
 *                      - \ref SPIM_BITMODE_8 : 8-bit mode (octal)
 * @return   None
 */
void SPIM_SwitchNBitOutput(SPIM_T *spim, uint32_t u32NBit)
{
    switch (u32NBit)
    {
        case SPIM_BITMODE_2:
            SPIM_ENABLE_DUAL_OUTPUT_MODE(spim);     /* 2-bit, Output. */
            break;

        case SPIM_BITMODE_4:
            SPIM_ENABLE_QUAD_OUTPUT_MODE(spim);     /* 4-bit, Output. */
            break;

        case SPIM_BITMODE_8:
            SPIM_ENABLE_OCTAL_OUTPUT_MODE(spim);    /* 8-bit, Output. */
            break;

        case SPIM_BITMODE_1:
        default:
            SPIM_ENABLE_SING_OUTPUT_MODE(spim);     /* 1-bit, Output. */
            break;
    }
}

/**
 * @brief    Switch SPIM input to the specified bit mode.
 * @param[in]  spim     Pointer to the SPIM peripheral.
 * @param[in]  u32NBit  Bit width to set for input mode.
 *                      - \ref SPIM_BITMODE_1 : 1-bit mode
 *                      - \ref SPIM_BITMODE_2 : 2-bit mode (dual)
 *                      - \ref SPIM_BITMODE_4 : 4-bit mode (quad)
 *                      - \ref SPIM_BITMODE_8 : 8-bit mode (octal)
 * @return     None
 */
void SPIM_SwitchNBitInput(SPIM_T *spim, uint32_t u32NBit)
{
    switch (u32NBit)
    {
        case SPIM_BITMODE_2:
            SPIM_ENABLE_DUAL_INPUT_MODE(spim);      /* 2-bit, Input.  */
            break;

        case SPIM_BITMODE_4:
            SPIM_ENABLE_QUAD_INPUT_MODE(spim);      /* 4-bit, Input.  */
            break;

        case SPIM_BITMODE_8:
            SPIM_ENABLE_OCTAL_INPUT_MODE(spim);     /* 8-bit, Input.  */
            break;

        case SPIM_BITMODE_1:
        default:
            SPIM_ENABLE_SING_INPUT_MODE(spim);      /* 1-bit, Input.  */
            break;
    }
}

/**
 * @brief      Check if the command is allowed to enable continuous read mode in DMM mode.
 * @param[in]  u8Cmd   Command opcode.
 * @return     SPIM_OP_ENABLE  : Allowed to enable continuous read mode.
 *             SPIM_OP_DISABLE : Not allowed to enable continuous read mode.
 */
static uint32_t _SPIM_IsCrenAllowed(uint8_t u8Cmd)
{
    uint32_t u32i;
    const uint8_t au8CrenAllowedCmds[] =
    {
        CMD_DMA_FAST_DUAL_READ,
        CMD_DMA_FAST_QUAD_READ,
        CMD_DMA_NORMAL_QUAD_READ,
        CMD_DMA_FAST_DUAL_DTR_READ,
        CMD_DMA_NORMAL_DTR_READ,
        CMD_DMA_FAST_QUAD_DTR_READ,
    };
    uint32_t u32CrenCmdCnt = (sizeof(au8CrenAllowedCmds) / sizeof(au8CrenAllowedCmds[0]));

    for (u32i = 0U; u32i < u32CrenCmdCnt; u32i++)
    {
        if (au8CrenAllowedCmds[u32i] == u8Cmd)
        {
            return SPIM_OP_ENABLE;
        }
    }

    return SPIM_OP_DISABLE;
}

/**
 * @brief      Set SPIM clock divider to 16 and RX clock delay number to 0.
 *             This configuration is used for SPI Flash read/write operations.
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[in]  u32Restore  Enable or disable restoring the original SPIM clock divider
 *                         and RX clock delay number after operation.
 *                         - \ref SPIM_OP_ENABLE
 *                         - \ref SPIM_OP_DISABLE
 * @return     None
 */
static void SPIM_SetupConfigRegDiv(SPIM_T *spim, uint32_t u32Restore)
{
    static SPIM_DIV_CTX sSPIMCtx = {0};

    if (u32Restore)
    {
        if (sSPIMCtx.u32Saved)
        {
            SPIM_SET_CLOCK_DIVIDER(spim, sSPIMCtx.u32Div);
            SPIM_SET_RXCLKDLY_RDDLYSEL(spim, sSPIMCtx.u32RxClkDly);
            SPIM_DBGMSG("SPIM%d restored: DIV=%u, RXDLY=%u\r\n", u32Idx, sSPIMCtx.u32Div, sSPIMCtx.u32RxClkDly);
            sSPIMCtx.u32Saved = 0;
        }
    }
    else
    {
        if (!sSPIMCtx.u32Saved)
        {
            sSPIMCtx.u32Div      = SPIM_GET_CLOCK_DIVIDER(spim);
            sSPIMCtx.u32RxClkDly = SPIM_GET_RXCLKDLY_RDDLYSEL(spim);
            sSPIMCtx.u32Saved    = 1U;
            SPIM_DBGMSG("SPIM%d saved: DIV=%u, RXDLY=%u\r\n", u32Idx, sSPIMCtx.u32Div, sSPIMCtx.u32RxClkDly);
        }

        SPIM_SET_CLOCK_DIVIDER(spim, 16UL);
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, 0UL);
    }
}

/**
 * @brief      Write data to SPI slave in specified bit mode.
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[in]  pu8TxBuf    Pointer to the transmit buffer.
 * @param[in]  u32NTx      Number of bytes to transmit.
 * @param[in]  u32NBit     Bit mode for data transmission.
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @retval     SPIM_OK             Write operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT    Write operation failed due to timeout.
 */
static int32_t _SPIM_WriteData(SPIM_T *spim, const uint8_t *pu8TxBuf, uint32_t u32NTx, uint32_t u32NBit)
{
    /* Write data to TX FIFO */
    uint32_t u32BufIdx = 0UL;   /* Transmit buffer index */
    uint32_t u32i;              /* Loop index */
    uint32_t u32Tmp;            /* Temporary variable */
    uint32_t u32BurstSize;      /* Burst data number */
    uint32_t u32NTxTmp = u32NTx;

    /* Switch between N-bit output mode */
    SPIM_SwitchNBitOutput(spim, u32NBit);

    while (u32NTxTmp)
    {
        /* Calculate the number of data to be transferred in one burst */
        uint32_t u32ChunkSize = (u32NTxTmp >= 16UL) ? 4UL :
                                (u32NTxTmp >= 12UL) ? 3UL :
                                (u32NTxTmp >= 8UL) ? 2UL :
                                (u32NTxTmp >= 4UL) ? 1UL :
                                0UL; /* Data chunk size */

        if (u32ChunkSize)
        {
            /* Transfer data in burst mode */
            u32i = u32ChunkSize;

            while (u32i)
            {
                (void)memcpy((uint8_t *)&u32Tmp, &pu8TxBuf[u32BufIdx], sizeof(uint32_t));
                u32i--;
                spim->TX[u32i] = u32Tmp;
                u32BufIdx += 4UL;
                u32NTxTmp -= 4UL;
            }

            u32BurstSize = u32ChunkSize;
            u32ChunkSize = 4UL;
        }
        else
        {
            u32ChunkSize = u32NTxTmp;
            /* Transfer data in single mode */
            (void)memcpy((uint8_t *)&u32Tmp, &pu8TxBuf[u32BufIdx], u32NTxTmp);
            u32BurstSize = 1UL;
            u32NTxTmp = 0UL;
            spim->TX[0] = u32Tmp;
        }

        /* Switch to Normal mode */
        SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
        /* Set data width */
        SPIM_SET_DATA_WIDTH(spim, (u32ChunkSize * 8UL));
        /* Set burst data number */
        SPIM_SET_BURST_DATA(spim, u32BurstSize);

        /* Wait until transfer complete */
        if (SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    return SPIM_OK;
}

/**
 * @brief      Read data from SPI slave using the specified bit mode.
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[out] pu8RxBuf    Pointer to the receive buffer.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode for SPI transmission/reception.
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @retval     SPIM_OK             Read operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT    Read operation aborted due to timeout.
 */
static int32_t _SPIM_ReadData(SPIM_T *spim, uint8_t *pu8RxBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /*
     * Read data in burst mode to improve performance.
     */
    uint32_t u32BufIdx = 0UL;       /* Buffer index */
    uint32_t u32NRxTmp = u32NRx;
    /* Configure SPIM to use N-bit input */
    SPIM_SwitchNBitInput(spim, u32NBit);

    while (u32NRxTmp)
    {
        /* Determine the number of data to be read in one burst */
        /* Number of data in one burst */
        uint32_t u32ChunkSize = (u32NRxTmp >= 16UL) ? 4UL : /* At least 16 bytes */
                                (u32NRxTmp >= 12UL) ? 3UL : /* 12 <= N < 16 */
                                (u32NRxTmp >= 8UL) ? 2UL : /* 8 <= N < 12 */
                                (u32NRxTmp >= 4UL) ? 1UL : /* 4 <= N < 8 */
                                0UL; /* N < 4 */
        uint32_t u32TmpChunkSize = u32ChunkSize;    /* Temporary value for chunk_size */
        uint32_t u32Tmp;                            /* Temporary variable for storing received data */
        uint32_t u32BurstSize = 0UL;                /* Number of data in one burst */

        if (u32ChunkSize)
        {
            /*
             * At least 2 data to be read in one burst, set burst size to
             * chunk_size.
             */
            u32BurstSize = u32ChunkSize;
            u32ChunkSize = 4UL;
        }
        else
        {
            /*
             * 1 data to be read, set burst size to 1 to read the data.
             */
            u32ChunkSize = u32NRxTmp;
            u32BurstSize = 1UL;
        }

        /* Configure SPIM to use Normal mode, N-bit data width and burst size */
        SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
        SPIM_SET_DATA_WIDTH(spim, (u32ChunkSize * 8UL));
        SPIM_SET_BURST_DATA(spim, u32BurstSize);

        /* Wait until transfer complete */
        if (SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
        {
            return SPIM_ERR_TIMEOUT;
        }

        u32ChunkSize = u32TmpChunkSize;

        /* Read received data */
        if (u32ChunkSize >= 2UL)
        {
            /* Read multiple data in one burst */
            while (u32ChunkSize)
            {
                u32Tmp = spim->RX[u32ChunkSize - 1UL];
                (void)memcpy(&pu8RxBuf[u32BufIdx], (uint8_t *)&u32Tmp, sizeof(uint32_t));
                u32ChunkSize--;
                u32BufIdx += 4UL;
                u32NRxTmp -= 4UL;
            }
        }
        else
        {
            /* Read 1 data */
            u32Tmp = spim->RX[0];
            (void)memcpy(&pu8RxBuf[u32BufIdx], (uint8_t *)&u32Tmp, u32NRxTmp);
            u32BufIdx += u32NRxTmp;
            u32NRxTmp = 0UL;
        }
    }

    return SPIM_OK;
}

/**
 * @brief       Set operation mode.
 * @param[in]   spim        The pointer of the specified SPIM module.
 * @param[in]   u32OPMode   SPI Function Operation Mode
 *              - \ref SPIM_CTL0_OPMODE_IO
 *              - \ref SPIM_CTL0_OPMODE_PAGEWRITE
 *              - \ref SPIM_CTL0_OPMODE_PAGEREAD
 *              - \ref SPIM_CTL0_OPMODE_DIRECTMAP
 * \hideinitializer
 */
void SPIM_SET_OPMODE(SPIM_T *spim, uint32_t u32OPMode)
{
    volatile int32_t i32TimeOutCount = (int32_t)SPIM_TIMEOUT;

    /* Wait for DMM mode to be idle. */
    while (SPIM_GET_DMM_IDLE(spim) == SPIM_OP_DISABLE)
    {
        if (--i32TimeOutCount <= 0)
        {
            break;
        }
    }

    ((spim)->CTL0 = (((spim)->CTL0 & ~(SPIM_CTL0_OPMODE_Msk)) |
                     ((u32OPMode) << SPIM_CTL0_OPMODE_Pos)));
}

/**
 * @brief      Issue the "Read Status Register #1" command to the SPI Flash.
 * @param[out] pu8DataBuf  Pointer to the buffer to store the received status register data.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode for SPI transmission/reception.
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @return     None
 */
void SPIM_ReadStatusRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_RDSR, OPCODE_RDSR};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8 : 0);

    (void)_SPIM_ReadData(spim, pu8DataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue the "Write Status Register #1" command to the SPI Flash.
 * @param[in]  pu8DataBuf  Pointer to the buffer containing data to be written.
 * @param[in]  u32NTx      Number of bytes to transmit.
 * @param[in]  u32NBit     Bit mode for SPI transmission.
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @return     None
 */
static void _SPIM_WriteStatusRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit)
{
    /* 1-byte Write Status Register #1 command + 1-byte data. */
    uint8_t au8CmdBuf[4] = {OPCODE_WRSR, 0x00U, 0x00U, 0x00U};
    uint32_t u32NTxTmp;

    (void)u32NTx;

    au8CmdBuf[1] = (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? au8CmdBuf[0] : pu8DataBuf[0];
    au8CmdBuf[2] = pu8DataBuf[0];
    au8CmdBuf[3] = pu8DataBuf[0];

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    u32NTxTmp = ((SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 4UL : 2UL);
    (void)_SPIM_WriteData(spim, au8CmdBuf, u32NTxTmp, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue the "Read Status Register #2" command.
 *
 * @param[in]  spim        Pointer to the SPIM instance.
 * @param[out] pu8DataBuf  Pointer to the buffer to store the received status register data.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode used for the command and data transfer.
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit mode (Dual)
 *                         - \ref SPIM_BITMODE_4 : 4-bit mode (Quad)
 *                         - \ref SPIM_BITMODE_8 : 8-bit mode (Octal)
 *
 * @return     None
 */
static void _SPIM_ReadStatusRegister2(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_RDSR2, OPCODE_RDSR2};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    (void)_SPIM_ReadData(spim, pu8DataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue the Winbond "Write Status Register" command.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral instance.
 * @param[in]  pu8DataBuf  Pointer to the transmit buffer containing the status register values to write.
 * @param[in]  u32NTx      Number of bytes to transmit.
 * @param[in]  u32NBit     Bit mode used for command and data transfer:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 */
static void _SPIM_WriteStatusRegister2(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit)
{
    uint8_t au8CmdBuf[6] = {OPCODE_WRSR, 0U, 0U, 0U, 0U, 0U};
    uint8_t u8DTREn = SPIM_GET_DTR_MODE(spim);
    uint32_t u32NTxTmp;

    (void)u32NTx;
    au8CmdBuf[1] = (u8DTREn == SPIM_OP_ENABLE) ? pu8DataBuf[0] : au8CmdBuf[0];
    au8CmdBuf[2] = (u8DTREn == SPIM_OP_ENABLE) ? pu8DataBuf[0] : pu8DataBuf[1];
    au8CmdBuf[3] = (u8DTREn == SPIM_OP_ENABLE) ? pu8DataBuf[0] : pu8DataBuf[1];
    au8CmdBuf[4] = pu8DataBuf[1];
    au8CmdBuf[5] = pu8DataBuf[1];

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    u32NTxTmp = (u8DTREn == SPIM_OP_ENABLE) ? 6UL : 3UL;
    (void)_SPIM_WriteData(spim, au8CmdBuf, u32NTxTmp, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

#if 0  /* not used */
/**
 * @brief      Issue the "Write Status Register #3" command.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[in]  pu8DataBuf  Pointer to the transmit buffer containing the data to write to Status Register #3.
 * @param[in]  u32NTx      Number of bytes to transmit.
 * @param[in]  u32NBit     Bit mode used for command and data transfer:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 */
static void SPIM_WriteStatusRegister3(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit)
{
    const uint8_t au8CmdBuf[] = {OPCODE_WRSR3, 0x00U};    /* 1-byte Write Status Register #2 command + 1-byte data. */
    au8CmdBuf[1] = pu8DataBuf[0];

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);        /* CS activated. */

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), u32NBit);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);        /* CS deactivated. */
}
#endif

/**
 * @brief      Issue the "Read Configuration Register #3" command.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[out] pu8DataBuf  Pointer to the receive buffer for the configuration register data.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode used for transmit/receive:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 */
static void _SPIM_ReadStatusRegister3(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_RDSR3, OPCODE_RDSR3};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);
    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    (void)_SPIM_ReadData(spim, pu8DataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

#if 0  /* not used */
/**
 * @brief      Issue the "Write Security Register" command.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[in]  pu8DataBuf  Pointer to the transmit buffer containing the data to write.
 * @param[in]  u32NTx      Number of bytes to transmit.
 * @param[in]  u32NBit     Bit mode used for transmit/receive:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 */
static void SPIM_WriteSecurityRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NTx, uint32_t u32NBit)
{
    /* 1-byte Write Status Register #2 command + 1-byte data. */
    uint8_t au8CmdBuf[] = {OPCODE_WRSCUR, 0x00U};

    au8CmdBuf[1] = pu8DataBuf[0];

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}
#endif

/**
 * @brief      Issue the "Read Security Register" command.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral.
 * @param[out] pu8DataBuf  Pointer to the receive buffer for storing the security register data.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode used for transmit/receive:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 */
static void _SPIM_ReadSecurityRegister(SPIM_T *spim, uint8_t *pu8DataBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_RDSCUR, OPCODE_RDSCUR};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    (void)_SPIM_ReadData(spim, pu8DataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue the "Read Flag Status Register" command for Micron MT35X Octal SPI Flash.
 *
 * @param[in]  spim        Pointer to the SPIM peripheral instance.
 * @param[out] pu8DataBuf  Pointer to the receive buffer for storing the FSR data.
 * @param[in]  u32NRx      Number of bytes to receive.
 * @param[in]  u32NBit     Bit mode used for transmit/receive:
 *                         - \ref SPIM_BITMODE_1 : 1-bit mode
 *                         - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                         - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                         - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @return     None
 *
 * @note       This command is used to check the current status of internal operations,
 *             such as program or erase progress.
 */
static void _SPIM_ReadMT35xFlagRegister(SPIM_T *spim, uint8_t *pu8DataBuf,
                                        uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_MICRON_RD_FLG, OPCODE_MICRON_RD_FLG};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8 : 0);

    (void)_SPIM_ReadData(spim, pu8DataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Check whether the SPI Flash erase or write operation has completed.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit  Bit mode used for transmit/receive:
 *                      - \ref SPIM_BITMODE_1 : 1-bit mode
 *                      - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                      - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                      - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @retval     1        Operation completed (Ready).
 * @retval     0        Operation still in progress (Busy).
 *
 * @note       Typically checks the WIP (Write-In-Progress) bit in the Flash status register.
 */
static int _SPIM_IsWriteDone(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t au8Status[2] = {0};
    SPIM_ReadStatusRegister(spim, au8Status, sizeof(au8Status), u32NBit);
    return !(au8Status[0] & SR_WIP);
}

/**
 * @brief      Wait until the SPI Flash erase or write operation completes.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit  Bit mode used for transmit/receive:
 *                      - \ref SPIM_BITMODE_1 : 1-bit mode
 *                      - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                      - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                      - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 *
 * @retval     SPIM_OK           Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT  Operation timed out (erase/write did not complete in time).
 *
 * @note       This function typically polls the WIP (Write-In-Progress) bit in the SPI Flash
 *             Status Register until it is cleared, or until a timeout occurs.
 */
static int _SPIM_WaitWriteDone(SPIM_T *spim, uint32_t u32NBit)
{
    volatile uint32_t u32Count;
    int i32Ret = (int32_t)SPIM_ERR_FAIL;

    for (u32Count = 0UL; u32Count < (SystemCoreClock / 1000UL); u32Count++)
    {
        if (_SPIM_IsWriteDone(spim, u32NBit))
        {
            i32Ret = SPIM_OK;
            break;
        }
    }

    if (i32Ret != SPIM_OK)
    {
        SPIM_DBGMSG("spim_wait_write_done time-out!!\n");
    }

    return i32Ret;
}

/**
 * @brief      Wait for the SPIM operation to complete and optionally check the busy status.
 *
 * @param[in]  spim       Pointer to the SPIM peripheral instance.
 * @param[in]  u32IsSync  Specify whether to wait until the operation completes:
 *                        - 0 : Do not wait (asynchronous mode).
 *                        - 1 : Wait for the busy flag to clear (synchronous mode).
 *
 * @retval     SPIM_OK           Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT  Operation failed due to timeout.
 *
 * @note       This function checks the busy status of the SPIM controller. If @p u32IsSync is set,
 *             it will poll the busy flag until cleared or timeout occurs.
 */
int32_t SPIM_WaitOpDone(SPIM_T *spim, uint32_t u32IsSync)
{
    volatile int32_t i32TimeOutCount = (int32_t)SPIM_TIMEOUT;

    /* Trigger SPIM operation. */
    SPIM_SET_GO(spim);

    if (u32IsSync)
    {
        /* Wait Busy Status. */
        while (SPIM_IS_BUSY(spim))
        {
            if (--i32TimeOutCount <= 0)
            {
                /* Time-out, return failed. */
                return SPIM_ERR_TIMEOUT;
            }
        }
    }

    return SPIM_OK;
}

/**
 * @brief      Issues a Write Enable or Write Disable command to the SPI Flash device.
 *
 * @param[in]  spim      Pointer to the SPIM peripheral instance.
 * @param[in]  i32IsEn   Specify whether to enable or disable write operations:
 *                       - \ref SPIM_OP_ENABLE : Send Write Enable command (e.g., 0x06)
 *                       - \ref SPIM_OP_DISABLE: Send Write Disable command (e.g., 0x04)
 * @param[in]  u32NBit   SPI bus I/O width (bit mode):
 *                       - \ref SPIM_BITMODE_1 : 1-bit mode
 *                       - \ref SPIM_BITMODE_2 : 2-bit mode
 *                       - \ref SPIM_BITMODE_4 : 4-bit mode
 *                       - \ref SPIM_BITMODE_8 : 8-bit mode
 *
 * @return     None.
 *
 * @note       This command is typically required before performing write, erase, or security register operations.
 */
void SPIM_SetWriteEnable(SPIM_T *spim, int32_t i32IsEn, uint32_t u32NBit)
{
    /* 1-byte Write Enable command. */
    uint8_t au8CmdBuf[2] = {0U, 0U};

    au8CmdBuf[0] = (i32IsEn ? OPCODE_WREN : OPCODE_WRDI);
    au8CmdBuf[1] = (i32IsEn ? OPCODE_WREN : OPCODE_WRDI);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated.   */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Enable or disable Wrap Around mode on the SPI Flash.
 *
 * @param[in]  spim    Pointer to the SPIM peripheral instance.
 * @param[in]  i32IsEn Specify whether to enable or disable Wrap mode:
 *                     - \ref SPIM_OP_ENABLE  : Enable Wrap Around mode
 *                     - \ref SPIM_OP_DISABLE : Disable Wrap Around mode
 *
 * @retval     SPIM_OK          Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT Operation aborted due to timeout.
 *
 * @note       Wrap Around mode is commonly used to optimize burst read performance
 *             by allowing continuous read within a fixed-length boundary.
 */
int32_t SPIM_SetWrapAroundEnable(SPIM_T *spim, int32_t i32IsEn)
{
    const uint32_t u32CmdBuf[2] = {0x00000000, 0x11011101};
    uint32_t u32Mask;
    uint32_t u32Val;
    uint32_t u32IsQuadEn;

    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, SPIM_BITMODE_1);

    if (u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);
    }

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    if ((uint32_t)i32IsEn == SPIM_OP_ENABLE)
    {
        u32Mask = (uint32_t)(1UL << 29);
    }
    else
    {
        u32Mask = (uint32_t)(1UL << 28);
    }

    u32Val = u32CmdBuf[0];
    u32Val |= u32Mask;

    spim->TX[0] = u32Val;
    spim->TX[1] = 0x11011101;

    SPIM_SwitchNBitOutput(spim, SPIM_BITMODE_4);

    /* Switch to Normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);

    SPIM_SET_DATA_WIDTH(spim, SPIM_DWIDTH_32);
    SPIM_SET_BURST_DATA(spim, SPIM_BURSTNUM_2);

    (void)SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    return SPIM_OK;
}

/**
 * @brief      Issue the command to exit continuous read mode on SPI Flash.
 *
 * @param[in]  spim    Pointer to the SPIM peripheral instance.
 *
 * @retval     0                  Operation completed successfully.
 * @retval     negative value     Operation failed (e.g., due to timeout or invalid configuration).
 *
 * @note       Some SPI Flash devices (e.g., Winbond, Macronix) enter continuous read mode after
 *             specific fast-read commands. This function sends a termination sequence (usually
 *             dummy clocks or specific exit command) to revert back to standby command mode.
 *
 * @note       Refer to the specific flash datasheet to determine if this operation is required.
 */
static int32_t _SPIM_SetContReadDisable(SPIM_T *spim)
{
    uint32_t u32IsQuadEn;

    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, SPIM_BITMODE_1);

    if (u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);
    }

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    spim->TX[0] = 0xFFFFFFFFUL;

    /* Quad output mode */
    SPIM_SwitchNBitOutput(spim, SPIM_BITMODE_4);

    /* Switch to normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
    SPIM_SET_DATA_WIDTH(spim, SPIM_DWIDTH_32);
    SPIM_SET_BURST_DATA(spim, SPIM_BURSTNUM_1);

    (void)SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    _SPIM_ClearContReadPhase(spim, SPIM_CTL0_OPMODE_DIRECTMAP);

    return SPIM_OK;
}

/**
 * @brief      Synchronize the 4-byte address mode state between the SPIM controller and the SPI Flash device.
 * @param[in]  spim           Pointer to the SPIM peripheral instance.
 * @param[in]  u32Is4ByteAddr Specify whether to enable (1) or disable (0) 4-byte address mode.
 * @param[in]  u32CmdBit      Bit mode used for the command transmission
 *                            - \ref SPIM_BITMODE_1 : 1-bit mode
 *                            - \ref SPIM_BITMODE_2 : 2-bit (Dual) mode
 *                            - \ref SPIM_BITMODE_4 : 4-bit (Quad) mode
 *                            - \ref SPIM_BITMODE_8 : 8-bit (Octal) mode
 * @return     SPIM_OK on success, or an error code on failure.
 *
 * @note       This function ensures that the SPIM controller's 4-byte address mode setting
 *             matches the SPI Flash device's mode by sending the appropriate command
 *             (EN4B or EX4B) if a change is needed.
 *
 */
static void _SPIM_Sync4ByteAddrMode(SPIM_T *spim, uint32_t u32Is4ByteAddr, uint32_t u32CmdBit)
{
    uint32_t u32Flash4BState = SPIM_Is4ByteModeEnable(spim, u32CmdBit);

    /* If the current state is the same, skip resending EN4B/EX4B to avoid redundant commands */
    if (u32Flash4BState != u32Is4ByteAddr)
    {
        (void)SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, u32CmdBit);
    }

    /* Sync SPIM mode regardless of flash result */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief      Exit Infineon Octal I/O (OPI) mode and revert to legacy 1S-1S-1S SPI mode.

 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 *
 * @note       After this operation, all future communication with the flash device should use
 *             standard SPI commands. Make sure the controller SPI configuration is reverted
 *             accordingly (e.g., bit width, clock polarity).
 */
void SPIM_ExitOPIMode_IFX(SPIM_T *spim)
{
    uint8_t u8CFR5Buf = 0xFFU;
    volatile uint32_t u32Delay = 0;

    // === Phase A: Try accessing CFR5V using Octal DDR (8D-8D-8D) ===
    SPIM_PHASE_T sPhase8D =
    {
        .u32CMDCode = OPCODE_IFX_RD_VCR,
        .u32CMDPhase = PHASE_OCTAL_MODE, .u32CMDWidth = PHASE_WIDTH_8, .u32CMDDTR = PHASE_ENABLE_DTR,
        .u32AddrPhase = PHASE_OCTAL_MODE, .u32AddrWidth = PHASE_WIDTH_32, .u32AddrDTR = PHASE_ENABLE_DTR,
        .u32DataPhase = PHASE_OCTAL_MODE, .u32ByteOrder = PHASE_ORDER_MODE0, .u32DataDTR = PHASE_ENABLE_DTR,
        .u32RDQS = SPIM_OP_DISABLE, .u32DcNum = 8
    };
    SPIM_PHASE_T sWrite8D = sPhase8D;
    SPIM_PHASE_T sPhase8S = {0};
    SPIM_PHASE_T sWrite8S = {0};

    sWrite8D.u32CMDCode = OPCODE_IFX_WR_VCR;
    sWrite8D.u32DcNum = 0;

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);
    SPIM_IO_ReadByPhase(spim, &sPhase8D, IFX_CFR5_ADDR, &u8CFR5Buf, 1);

    if ((u8CFR5Buf != 0xFFU) && (u8CFR5Buf != 0x00U))
    {
        u8CFR5Buf &= ~0x03;
        SPIM_IO_WriteByPhase(spim, &sWrite8D, IFX_CFR5_ADDR, &u8CFR5Buf, 1, SPIM_OP_DISABLE);
    }

    // === Phase B: Try accessing CFR5V using Octal SDR (8S-8S-8S) ===
    u8CFR5Buf = 0xFFU;
    sPhase8S = sPhase8D;
    sPhase8S.u32CMDDTR = PHASE_DISABLE_DTR;
    sPhase8S.u32AddrDTR = PHASE_DISABLE_DTR;
    sPhase8S.u32DataDTR = PHASE_DISABLE_DTR;
    sPhase8S.u32DcNum = 0;

    sWrite8S = sPhase8S;
    sWrite8S.u32CMDCode = OPCODE_IFX_WR_VCR;

    SPIM_IO_ReadByPhase(spim, &sPhase8S, IFX_CFR5_ADDR, &u8CFR5Buf, 1);

    if ((u8CFR5Buf != 0xFFU) && (u8CFR5Buf != 0x00U))
    {
        u8CFR5Buf &= ~0x03;
        SPIM_IO_WriteByPhase(spim, &sWrite8S, IFX_CFR5_ADDR, &u8CFR5Buf, 1, SPIM_OP_DISABLE);
    }

    // === Phase C: Optional delay and hardware reset ===
    for (u32Delay = 0; u32Delay < 50000UL; u32Delay++) {}

    SPIM_SET_RSTN_MODE(spim, SPIM_OP_ENABLE);

    for (u32Delay = 0; u32Delay < 50000UL; u32Delay++) {}

    SPIM_SET_RSTN_MODE(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Enter Infineon Octal I/O (OPI) mode in SDR or DDR.
 *
 * @param[in]  spim       Pointer to the SPIM peripheral instance.
 * @param[in]  i32IsDDR   DDR mode flag.
 *                        - \ref SPIM_OP_DISABLE : Enter OPI SDR mode (8S-8S-8S)
 *                        - \ref SPIM_OP_ENABLE  : Enter OPI DDR mode (8D-8D-8D)
 * @return     None.
 *
 * @note       Before calling this function, ensure that the Write Enable (WREN) command is issued.
 * @note       After entering OPI mode, all subsequent commands must use the corresponding
 *             8-line I/O protocol (SDR or DDR) with appropriate bit width configuration.
 */
void SPIM_EnterOPIMode_IFX(SPIM_T *spim, int i32IsDDR)
{
    uint8_t u8CFR5Buf = 0;
    volatile uint32_t u32Delay = 0;

    // Phase setting for initial 1S-1S-1S
    SPIM_PHASE_T sCmdRead =
    {
        .u32CMDCode   = OPCODE_IFX_RD_VCR, // 0x65: Read Any Register
        .u32CMDPhase  = PHASE_NORMAL_MODE,
        .u32CMDWidth  = PHASE_WIDTH_8,
        .u32CMDDTR    = PHASE_DISABLE_DTR,

        .u32AddrPhase = PHASE_NORMAL_MODE,
        .u32AddrWidth = PHASE_WIDTH_24,
        .u32AddrDTR   = PHASE_DISABLE_DTR,

        .u32DataPhase = PHASE_NORMAL_MODE,
        .u32ByteOrder = PHASE_ORDER_MODE0,
        .u32DataDTR   = PHASE_DISABLE_DTR,

        .u32RDQS      = SPIM_OP_DISABLE,
        .u32DcNum     = 0,
    };

    // Clone for write
    SPIM_PHASE_T sCmdWrite = sCmdRead;
    sCmdWrite.u32CMDCode = OPCODE_IFX_WR_VCR;

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    // Step 1: Read CFR5V from 1S mode
    SPIM_IO_ReadByPhase(spim, &sCmdRead, IFX_CFR5_ADDR, &u8CFR5Buf, sizeof(u8CFR5Buf));

    // Step 2: Set OPI-IT (bit 0) and SDRDDR (bit 1)
    u8CFR5Buf |= 0x01; // OPI-IT = 1 Octal mode enable

    if (i32IsDDR)
    {
        u8CFR5Buf |= 0x02; // SDRDDR = 1 DDR mode
    }
    else
    {
        u8CFR5Buf &= ~(0x02); // SDRDDR = 0 SDR mode
    }

    // Step 3: Write back updated value
    SPIM_IO_WriteByPhase(spim, &sCmdWrite, IFX_CFR5_ADDR, &u8CFR5Buf, sizeof(u8CFR5Buf), SPIM_OP_DISABLE);

    // Step 4: Wait for mode switch to take effect
    for (u32Delay = 0; u32Delay < 50000UL; u32Delay++) {}

    // Step 5: Update global DTR flag
    if (i32IsDDR)
    {
        SPIM_SET_DTR_MODE(spim, SPIM_OP_ENABLE);
    }
    else
    {
        SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
    }

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Exit Micron Octal I/O (OPI) mode and revert to 1S-1S-1S legacy SPI mode.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 *
 * @note       This operation typically requires a Write Enable (WREN) command beforehand.
 * @note       After exiting OPI mode, reconfigure SPIM controller to match legacy SPI timing and bit mode.
 */
void SPIM_ExitOPIMode_MICRON(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {OPCODE_MICRON_SDR_DQS};
    SPIM_PHASE_T sWrNVCRegCMD =
    {
        OPCODE_MICRON_WR_VCFG,                                                  //Command Code
        PHASE_OCTAL_MODE, PHASE_WIDTH_8, PHASE_ENABLE_DTR,                      //Command Phase
        PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_ENABLE_DTR,                     //Address Phase
        PHASE_OCTAL_MODE, PHASE_ORDER_MODE0, PHASE_ENABLE_DTR, SPIM_OP_ENABLE,  //Data Phase
        0,                                                                      //Dummy
        PHASE_DISABLE_CONT_READ, 0, 0, 0,                                      //Continue Phase
    };

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* Set non-volatile register exit octal DDR mode */
    SPIM_IO_WriteByPhase(spim, &sWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf), SPIM_OP_DISABLE);

    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);

    /* Disable 4-byte Address mode */
    _SPIM_Sync4ByteAddrMode(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Enter Micron Octal I/O (OPI) mode in either SDR or DDR.
 *
 * @param[in]  spim       Pointer to the SPIM peripheral instance.
 *
 * @note       A Write Enable (WREN) command is typically required before modifying VCR6.
 * @note       After enabling OPI mode, the SPIM controller must be reconfigured to match the selected
 *             OPI mode (DDR), including bit width, DQS, and dummy cycles.
 */
void SPIM_EnterOPIMode_MICRON(SPIM_T *spim)
{
    uint8_t u8CMDBuf[1] = {OPCODE_MICRON_OCTAL_DQS};
    SPIM_PHASE_T sWrNVCRegCMD =
    {
        OPCODE_MICRON_WR_VCFG,                                                      //Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                        //Command Phase
        PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
        0,
        PHASE_DISABLE_CONT_READ, 0, 0, 0,                                          //Continue Phase
    };

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* Enable 4-byte address mode */
    _SPIM_Sync4ByteAddrMode(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);

    /* Set non-volatile register enter octal DDR mode */
    SPIM_IO_WriteByPhase(spim, &sWrNVCRegCMD, 0x00, u8CMDBuf, sizeof(u8CMDBuf), SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);

    SPIM_SET_DTR_MODE(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Get the current SPIM serial clock frequency.
 *
 * @param[in]  spim    Pointer to the SPIM peripheral instance.
 *
 * @retval     uint32_t    SPI serial clock frequency in Hz.
 *
 * @note       Ensure the SPIM clock source and divider are properly configured before calling this function.
 */
uint32_t SPIM_GetSClkFreq(const SPIM_T *spim)
{
    uint32_t u32ClkDiv = SPIM_GET_CLOCK_DIVIDER(spim);

    return u32ClkDiv ? SystemCoreClock / (u32ClkDiv * 2U) : SystemCoreClock;
}

/**
 * @brief      Resets the connected SPI Flash device via SPIM.
 *
 * @param[in]  spim      Pointer to the SPIM instance.
 * @param[in]  u32NBit   N-bit transfer mode.
 *                       - \ref SPIM_BITMODE_1 : Standard SPI (1-bit)
 *                       - \ref SPIM_BITMODE_2 : Dual SPI (2-bit)
 *                       - \ref SPIM_BITMODE_4 : Quad SPI (4-bit)
 *                       - \ref SPIM_BITMODE_8 : Octal SPI (8-bit)
 * @return     None
 *
 * @note       Some Flash chips (e.g., Winbond, Micron, ISSI, Macronix) require reset before switching modes (e.g., exiting OPI).
 * @note       Ensure the SPIM interface is initialized and the clock is stable before calling this function.
 */
static void _SPIM_ResetDevice(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t au8CmdBuf[2] = {0UL, 0UL};

    /* Reset Enable */
    au8CmdBuf[0] = OPCODE_RSTEN;
    au8CmdBuf[1] = OPCODE_RSTEN;

    /* CS activated.    */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    /* Reset */
    au8CmdBuf[0] = OPCODE_RST;
    au8CmdBuf[1] = OPCODE_RST;

    /* CS activated.    */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Initialize the SPIM Flash interface and detect Flash device.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  i32ClrWP Write Protection control:
 *                      - `1` : Attempt to clear Flash write protection bits (if supported).
 *                      - `0` : Leave current write protection setting unchanged.
 *
 * @retval     SPIM_OK          Initialization completed successfully.
 * @retval     SPIM_ERR_FAIL    Initialization failed, possibly due to:
 *                              - Unable to read Manufacturer ID.
 *                              - Unknown or unsupported Flash vendor/device.
 *
 * @note       This function should be called after SPIM controller and clock are initialized.
 * @note       Clearing write protection may require prior write enable and manufacturer-specific handling.
 * @note       Only common vendors such as Winbond, Micron, Macronix, ISSI, etc., may be supported.
 */
int32_t SPIM_InitFlash(SPIM_T *spim, int32_t i32ClrWP)
{
    uint8_t au8IdBuf[3];
    uint32_t u32i = 0;
    int32_t i32Ret = (int32_t)SPIM_ERR_FAIL;
    const uint8_t  u8Supported_List[] =
    {
        MFGID_WINBOND,
        MFGID_MXIC,
        MFGID_EON,
        MFGID_ISSI,
        MFGID_SPANSION,
        MFGID_MICRON,
        MFGID_IFX,
    };

    /* Enable SPI Flash Mode */
    SPIM_SET_FLASH_MODE(spim);

    /* Enable DLL */
    (void)SPIM_INIT_DLL(spim);

    SPIM_SET_SS_ACTLVL(spim, SPIM_OP_DISABLE);

    SPIM_ExitOPIMode_IFX(spim);

    SPIM_ExitOPIMode_MICRON(spim);

    /*
     * Because not sure in SPI or QPI mode, do QPI reset and then SPI reset.
     */
    /* Quad Mode Reset */
    _SPIM_ResetDevice(spim, SPIM_BITMODE_4);

    /* Single Mode Reset */
    _SPIM_ResetDevice(spim, SPIM_BITMODE_1);

    if (i32ClrWP)
    {
        uint8_t dataBuf[] = {0x00U};

        /* Clear Block Protect. */
        SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);

        _SPIM_WriteStatusRegister(spim, dataBuf, sizeof(dataBuf), SPIM_BITMODE_1);

        (void)_SPIM_WaitWriteDone(spim, SPIM_BITMODE_1);
    }

    SPIM_ReadJedecId(spim, au8IdBuf, sizeof(au8IdBuf), SPIM_BITMODE_1);

    for (u32i = 0; u32i < sizeof(au8IdBuf); ++u32i)
    {
        gau8IDBuf[u32i] = au8IdBuf[u32i];
    }

    /* printf("ID: 0x%x, 0x%x, px%x\n", idBuf[0], idBuf[1], idBuf[2]); */

    for (u32i = 0UL; u32i < sizeof(u8Supported_List) / sizeof(u8Supported_List[0]); u32i++)
    {
        if (au8IdBuf[0] == u8Supported_List[u32i])
        {
            i32Ret = SPIM_OK;
            break;
        }
    }

    if (i32Ret != 0)
    {
        SPIM_DBGMSG("Flash initialize failed!! 0x%x\n", au8IdBuf[0]);
    }

    return i32Ret;
}

/**
 * @brief      Issue JEDEC ID (0x9F) command to read manufacturer and product ID.
 *
 * @param      spim       Pointer to the SPIM module.
 * @param      pu8IdBuf   Pointer to an ID buffer with space for 3 bytes:
 *                        - `pu8IdBuf[0]`: Manufacturer ID
 *                        - `pu8IdBuf[1]`: Memory Type / Product ID
 *                        - `pu8IdBuf[2]`: Capacity / Revision Code
 * @param      u32NRx     Number of bytes to read. Must be at least 3.
 * @param      u32NBit    N-bit transmit/receive mode. One of:
 *                        - \ref SPIM_BITMODE_1
 *                        - \ref SPIM_BITMODE_2
 *                        - \ref SPIM_BITMODE_4
 *                        - \ref SPIM_BITMODE_8
 *
 * @return     None.
 *
 * @note       Some vendors may require dummy cycles or specific bit modes to return valid ID.
 * @note       This function assumes the SPI Flash is in legacy 1S-1S-1S mode before OPI mode is entered.
 */
void SPIM_ReadJedecId(SPIM_T *spim, uint8_t *pu8IdBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte JEDEC ID command. */
    const uint8_t au8CmdBuf[2] = {OPCODE_RDID, OPCODE_RDID};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);
    (void)SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);
    (void)_SPIM_ReadData(spim, pu8IdBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/** @cond HIDDEN_SYMBOLS */

/**
 * @brief      Enable or disable Spansion Quad Mode.
 *
 * @param      spim     Pointer to the SPIM peripheral instance.
 * @param      u32IsEn  Operation mode:
 *                      - \ref SPIM_OP_ENABLE  : Enable Quad mode
 *                      - \ref SPIM_OP_DISABLE : Disable Quad mode
 *
 * @return     None.
 *
 * @note       This command sequence is specific to Spansion (Cypress) Flash devices.
 *             Make sure the flash device supports Quad Mode and follows the QE-bit definition
 *             before using this function.
 */
static void _SPIM_EnableSpansionQuadMode(SPIM_T *spim, uint32_t u32IsEn)
{
    uint8_t au8CmdBuf[3];
    uint8_t au8DataBuf[1];
    uint8_t u8Status1;
    volatile int32_t i32Delay = 10000L;

    /* Read Status Register-1 */
    au8CmdBuf[0] = OPCODE_RDSR;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

    (void)_SPIM_ReadData(spim, au8DataBuf, sizeof(au8DataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    /* SPIM_DBGMSG("SR1 = 0x%x\n", au8DataBuf[0]); */

    u8Status1 = au8DataBuf[0];

    /* Read Configuration Register-2 */
    au8CmdBuf[0] = OPCODE_RDSR2;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

    (void)_SPIM_ReadData(spim, au8DataBuf, sizeof(au8DataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    /* SPIM_DBGMSG("CR1 = 0x%x\n", au8DataBuf[0]); */

    SPIM_SetWriteEnable(spim, 1, 1UL);

    /* Write register */
    au8CmdBuf[0] = OPCODE_WRSR;
    au8CmdBuf[1] = u8Status1;

    if (u32IsEn)
    {
        /* set QUAD */
        au8CmdBuf[2] = au8DataBuf[0] | 0x2U;
    }
    else
    {
        /* clear QUAD */
        au8CmdBuf[2] = au8DataBuf[0] & ~0x2U;
    }

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetWriteEnable(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    /* Read Configuration Register-2 */
    au8CmdBuf[0] = OPCODE_RDSR2;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

    (void)_SPIM_ReadData(spim, au8DataBuf, sizeof(au8DataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    /* SPIM_DBGMSG("CR1 = 0x%x\n", au8DataBuf[0]); */
    while (--i32Delay >= 0) {}
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief      Enable or disable SPI Flash Quad Mode.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  u32IsEn  Desired Quad Mode state.
 *                      - \ref SPIM_OP_ENABLE  : Enable Quad mode
 *                      - \ref SPIM_OP_DISABLE : Disable Quad mode
 * @param[in]  u32NBit  SPI transfer bit mode for issuing the command.
 *                      - \ref SPIM_BITMODE_1  : Standard 1-bit SPI mode
 *                      - \ref SPIM_BITMODE_2  : Dual output mode
 *                      - \ref SPIM_BITMODE_4  : Quad output mode
 *                      - \ref SPIM_BITMODE_8  : Octal output mode
 *
 * @return     None.
 *
 * @note       Before enabling Quad mode, ensure that the Flash device supports it,
 *             and the QE bit location is correctly handled in vendor-specific logic.
 */
void SPIM_SetQuadEnable(SPIM_T *spim, uint32_t u32IsEn, uint32_t u32NBit)
{
    uint8_t au8IdBuf[3];
    uint8_t dataBuf[2];

    SPIM_ReadJedecId(spim, au8IdBuf, sizeof(au8IdBuf), u32NBit);

    SPIM_DBGMSG("SPIM_SetQuadEnable - Flash ID is 0x%x\n", au8IdBuf[0]);

    switch (au8IdBuf[0])
    {
        /* Winbond SPI flash */
        case MFGID_WINBOND:
            SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
            _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
            SPIM_DBGMSG("Status Register: 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);

            if (u32IsEn)
            {
                dataBuf[1] |= SR2_QE;
            }
            else
            {
                dataBuf[1] &= ~SR2_QE;
            }

            /* Write Enable. */
            SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);
            _SPIM_WriteStatusRegister2(spim, dataBuf, sizeof(dataBuf), u32NBit);
            (void)_SPIM_WaitWriteDone(spim, u32NBit);

            SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
            _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
            SPIM_DBGMSG("Status Register: 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);
            break;

        /* MXIC SPI flash. */
        case MFGID_MXIC:
        case MFGID_EON:

        /* ISSI SPI flash. */
        case MFGID_ISSI:
            /* Write Enable. */
            SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);

            dataBuf[0] = u32IsEn ? SR_QE : 0U;

            _SPIM_WriteStatusRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);

            (void)_SPIM_WaitWriteDone(spim, SPIM_BITMODE_1);
            break;

        case MFGID_SPANSION:
            _SPIM_EnableSpansionQuadMode(spim, u32IsEn);
            break;

        default:
            break;
    }
}

/**
 * @brief      Get the current Quad Mode status of the connected SPI Flash.
 *
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit  SPI transfer bit mode for issuing the command.
 *                      - \ref SPIM_BITMODE_1  : Standard 1-bit SPI mode
 *                      - \ref SPIM_BITMODE_2  : Dual output mode
 *                      - \ref SPIM_BITMODE_4  : Quad output mode
 *                      - \ref SPIM_BITMODE_8  : Octal output mode
 *
 * @retval     uint32_t    Current Quad Mode status:
 *                         - `1` : Quad Mode is enabled
 *                         - `0` : Quad Mode is disabled
 *
 * @note       This function reads the appropriate status/configuration
 * registers based on the detected Flash vendor to determine the Quad Mode
 * state.
 */
uint32_t SPIM_IsQuadEnabled(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t au8IdBuf[3] = {0U, 0U, 0U};
    uint8_t dataBuf[2] = {0U, 0U};
    uint32_t u32IsQuadMode = SPIM_OP_ENABLE;

    (void)memcpy(au8IdBuf, gau8IDBuf, sizeof(au8IdBuf));
    SPIM_DBGMSG("SPIM_IsQuadEnabled - Flash ID is 0x%x\n", au8IdBuf[0]);

    switch (au8IdBuf[0])
    {
        /* Winbond SPI flash */
        case MFGID_WINBOND:
        {
            /* QE 在 Status Register-2 的 SR2_QE bit */
            SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
            _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
            SPIM_DBGMSG("Status Register: 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);

            if ((dataBuf[1] & SR2_QE) != 0U)
            {
                u32IsQuadMode = SPIM_OP_DISABLE;
            }
            else
            {
                u32IsQuadMode = SPIM_OP_ENABLE;
            }

            break;
        }

        /* MXIC / EON SPI flash */
        case MFGID_MXIC:
        case MFGID_EON:
        case MFGID_ISSI:
        case MFGID_SPANSION:
        default:
            break;
    }

    return u32IsQuadMode;
}

/**
 * @brief      Enable or disable QPI (Quad Peripheral Interface) mode.
 *
 * @param[in]  spim    Pointer to the SPIM peripheral instance.
 * @param[in]  i32IsEn Set to 1 to enable QPI mode, or 0 to disable it.
 *                     - \ref SPIM_OP_ENABLE  : Enable QPI mode
 *                     - \ref SPIM_OP_DISABLE : Disable QPI mode
 * @return     None
 */
static void _SPIM_EonSetQpiMode(SPIM_T *spim, int32_t i32IsEn)
{
    /* 1-byte command.  */
    uint8_t au8CmdBuf[1];
    uint8_t au8Status[1];

    SPIM_ReadStatusRegister(spim, au8Status, sizeof(au8Status), SPIM_BITMODE_1);
    SPIM_DBGMSG("Status: 0x%x\n", status[0]);

    if (i32IsEn)
    {
        /* Assume in SPI mode. */
        au8CmdBuf[0] = OPCODE_ENQPI;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

        /* CS deactivated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    }
    else
    {
        /* Assume in QPI mode. */
        au8CmdBuf[0] = OPCODE_EXQPI;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_4);

        /* CS deactivated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    }

    SPIM_ReadStatusRegister(spim, au8Status, sizeof(au8Status), 1UL);
    SPIM_DBGMSG("Status: 0x%x\n", status[0]);
}

/**
 * @brief      Enable or disable 4-byte address mode for Spansion SPI Flash.
 * @param[in]  spim     Pointer to the SPIM peripheral instance.
 * @param[in]  u32IsEn  Enable or disable 4-byte address mode.
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 * @param[in]  u32NBit  Bit mode used for command transmission.
 *                      - \ref SPIM_BITMODE_1 : 1-bit mode
 *                      - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                      - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                      - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @return     None
 */
static void _SPIM_SPANSION4BytesEnable(SPIM_T *spim, uint32_t u32IsEn, uint32_t u32NBit)
{
    uint8_t au8CmdBuf[2];
    uint8_t au8DataBuf[1];

    au8CmdBuf[0] = OPCODE_BRRD;

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, 1UL, u32NBit);

    (void)_SPIM_ReadData(spim, au8DataBuf, 1UL, SPIM_BITMODE_1);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_DBGMSG("Bank Address register= 0x%x\n", au8DataBuf[0]);

    au8CmdBuf[0] = OPCODE_BRWR;

    if (u32IsEn)
    {
        /* set EXTADD */
        au8CmdBuf[1] = au8DataBuf[0] | 0x80U;
    }
    else
    {
        /* clear EXTADD */
        au8CmdBuf[1] = au8DataBuf[0] & ~0x80U;
    }

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, au8CmdBuf, sizeof(au8CmdBuf), SPIM_BITMODE_1);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

/** @cond HIDDEN_SYMBOLS */
/**
 * @brief      Query whether 4-byte address mode is enabled.
 * @param[in]  spim      Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit   Bit mode used for command transmission.
 *                       - \ref SPIM_BITMODE_1 : 1-bit mode
 *                       - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                       - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                       - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @retval     0         4-byte address mode is disabled.
 * @retval     1         4-byte address mode is enabled.
 */
uint32_t SPIM_Is4ByteModeEnable(SPIM_T *spim, uint32_t u32NBit)
{
    uint32_t isEn = 0;
    uint8_t idBuf[3];
    uint8_t dataBuf[2];

    memcpy(idBuf, gau8IDBuf, sizeof(idBuf));

    switch (idBuf[0])
    {
        case MFGID_WINBOND:
            _SPIM_ReadStatusRegister3(spim, dataBuf, sizeof(dataBuf), u32NBit);

            /* Before MX25L6406E, bit 6 of Status Register-3 is reserved. */
            //isEn = (idBuf[2] < 0x16U) ? !(int32_t)SPIM_OP_ENABLE : !!(dataBuf[0] & SR3_ADR);
            if (idBuf[2] < 0x16U)
            {
                /* Old device: bit reserved → treat as "not enabled". */
                isEn = (int32_t)SPIM_OP_DISABLE;
            }
            else
            {
                if ((dataBuf[0] & SR3_ADR) != 0U)
                {
                    isEn = (int32_t)SPIM_OP_ENABLE;
                }
                else
                {
                    isEn = (int32_t)SPIM_OP_DISABLE;
                }
            }

            break;

        case MFGID_MXIC:
        case MFGID_EON:
            _SPIM_ReadSecurityRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);

            /* Before MT25QL512ABA, bit 4 of Security Register is reserved. */
            //isEn = (idBuf[2] < 0x19U) ? !(int32_t)SPIM_OP_ENABLE : !!(dataBuf[0] & SCUR_4BYTE);
            if (idBuf[2] < 0x19U)
            {
                isEn = (int32_t)SPIM_OP_DISABLE;
            }
            else
            {
                if ((dataBuf[0] & SCUR_4BYTE) != 0U)
                {
                    isEn = (int32_t)SPIM_OP_ENABLE;
                }
                else
                {
                    isEn = (int32_t)SPIM_OP_DISABLE;
                }
            }

            break;

        case MFGID_ISSI:

            /* Before IS25LP016D, 4-byte address mode supported. */
            //isEn = (idBuf[2] < 0x49U) ? (int32_t)SPIM_OP_DISABLE : (int32_t)SPIM_OP_ENABLE;
            if (idBuf[2] < 0x49U)
            {
                isEn = (int32_t)SPIM_OP_DISABLE;
            }
            else
            {
                isEn = (int32_t)SPIM_OP_ENABLE;
            }

            break;

        case MFGID_MICRON:
            /* Before MT35XL01G1AA, bit 6 of Flag Register is reserved. */
            _SPIM_ReadMT35xFlagRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);

            // isEn = !!(dataBuf[0] & SR3_ADR);
            if ((dataBuf[0] & SR3_ADR) != 0U)
            {
                isEn = (int32_t)SPIM_OP_ENABLE;
            }
            else
            {
                isEn = (int32_t)SPIM_OP_DISABLE;
            }

            break;

        default:
            break;
    }

    return isEn;
}

/** @endcond HIDDEN_SYMBOLS  */

/**
 * @brief      Enter or exit 4-byte address mode on the SPI Flash device.
 * @param[in]  spim      Pointer to the SPIM peripheral instance.
 * @param[in]  u32IsEn   Enable or disable 4-byte address mode.
 *                       - \ref SPIM_OP_ENABLE  : Enter 4-byte address mode
 *                       - \ref SPIM_OP_DISABLE : Exit 4-byte address mode
 * @param[in]  u32NBit   Bit mode used for command transmission.
 *                       - \ref SPIM_BITMODE_1 : 1-bit mode
 *                       - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                       - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                       - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @retval     SPIM_OK         Operation completed successfully.
 * @retval     SPIM_ERR_FAIL   Operation failed (e.g., unsupported command or communication error).
 */
int32_t SPIM_Enable_4Bytes_Mode(SPIM_T *spim, uint32_t u32IsEn, uint32_t u32NBit)
{
    int32_t i32IsSupt = 0L;
    int32_t i32Ret = SPIM_ERR_FAIL;
    uint8_t au8IdBuf[3];

    SPIM_ReadJedecId(spim, au8IdBuf, sizeof(au8IdBuf), u32NBit);

    /* Based on Flash size, check if 4-byte address mode is supported. */
    switch (au8IdBuf[0])
    {
        case MFGID_WINBOND:
            i32IsSupt = (au8IdBuf[2] < 0x16U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_MXIC:
        case MFGID_EON:
            i32IsSupt = (au8IdBuf[2] < 0x19U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_ISSI:
            i32IsSupt = (au8IdBuf[2] < 0x49U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_SPANSION:
            _SPIM_SPANSION4BytesEnable(spim, u32IsEn, u32NBit);
            i32IsSupt = SPIM_OP_ENABLE;
            i32Ret = SPIM_OK;
            break;

        case MFGID_MICRON:
            //SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);
            i32IsSupt = SPIM_OP_ENABLE;
            break;

        default:
            break;
    }

    if ((i32IsSupt) && (au8IdBuf[0] != (uint8_t)MFGID_SPANSION))
    {
        /* 1-byte Enter/Exit 4-Byte Mode command. */
        uint8_t u8CmdBuf[2];
        uint8_t u8Opcode = (u32IsEn != 0U) ? OPCODE_EN4B : OPCODE_EX4B;

        u8CmdBuf[0] = u8Opcode;
        u8CmdBuf[1] = u8Opcode;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        (void)_SPIM_WriteData(spim, u8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

        /* CS deactivated.  */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

        /*
         * FIXME: Per test, 4BYTE Indicator bit doesn't set after EN4B, which
         * doesn't match spec(MX25L25635E), so skip the check below.
         */
        i32Ret = SPIM_OK;

        if (au8IdBuf[0] != (uint8_t)MFGID_MXIC)
        {
            /*
             *  About over 100 instrucsions executed, just want to give
             *  a time-out about 1 seconds to avoid infinite loop
             */
            volatile int32_t i32TimeOutCount = (SystemCoreClock) / 100U;

            while ((--i32TimeOutCount >= 0) && ((uint32_t)SPIM_Is4ByteModeEnable(spim, u32NBit) != u32IsEn)) {}

            if (i32TimeOutCount <= 0L)
            {
                i32Ret = SPIM_ERR_FAIL;
            }
        }
    }

    return i32Ret;
}

/**
 * @brief      Unlock Winbond flash chip with security bit enabled.
 * @param[in]  spim      Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit   Bit mode used for command transmission.
 *                       - \ref SPIM_BITMODE_1 : 1-bit mode
 *                       - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                       - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                       - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @return     None.
 * @note       This function issues the Winbond-specific unlock sequence
 *             to disable security features and allow full chip access.
 */
void SPIM_WinbondUnlock(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t au8IdBuf[3];
    uint8_t dataBuf[4];

    SPIM_ReadJedecId(spim, au8IdBuf, sizeof(au8IdBuf), u32NBit);

    if ((au8IdBuf[0] != (uint8_t)MFGID_WINBOND) ||
            (au8IdBuf[1] != 0x40U) ||
            (au8IdBuf[2] != 0x16U))
    {
        SPIM_DBGMSG("SPIM_WinbondUnlock - Not W25Q32, do nothing.\n");
        return;
    }

    SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
    _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
    SPIM_DBGMSG("Status Register: 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);

    /* clear Status Register-1 SEC bit */
    dataBuf[1] &= ~0x40;

    /* Write Enable.    */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);
    _SPIM_WriteStatusRegister2(spim, dataBuf, sizeof(dataBuf), u32NBit);
    (void)_SPIM_WaitWriteDone(spim, u32NBit);

    SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
    _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
    SPIM_DBGMSG("Status Register (after unlock): 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);
}

/**
 * @brief      Erase the entire SPI flash chip.
 * @param[in]  spim      Pointer to the SPIM peripheral instance.
 * @param[in]  u32NBit   Bit mode used for command transmission.
 *                       - \ref SPIM_BITMODE_1 : 1-bit mode
 *                       - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                       - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                       - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @param[in]  i32IsSync Whether to wait until the erase operation is completed.
 *                       - 1: Wait (blocking mode)
 *                       - 0: Return immediately (non-blocking)
 * @return     None.
 * @note       The chip erase operation may take several seconds to complete.
 *             Ensure the flash is write-enabled before calling this function.
 */
void SPIM_ChipErase(SPIM_T *spim, uint32_t u32NBit, int32_t i32IsSync)
{
    /* 1-byte Chip Erase command. */
    const uint8_t cmdBuf[2] = {OPCODE_CHIP_ERASE, OPCODE_CHIP_ERASE};

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (i32IsSync)
    {
        (void)_SPIM_WaitWriteDone(spim, u32NBit);
    }
}

/**
 * @brief      Erase a specific block in SPI flash memory.
 * @param[in]  spim            Pointer to the SPIM peripheral instance.
 * @param[in]  u32Addr         Target address within the block to erase.
 * @param[in]  u32Is4ByteAddr  Whether the flash uses a 4-byte address.
 *                             - 0: 3-byte address mode
 *                             - 1: 4-byte address mode
 * @param[in]  u32ErsCmd       Block erase command opcode.
 *                             - \ref OPCODE_SE_4K  : Sector Erase 4KB
 *                             - \ref OPCODE_BE_32K : Block Erase 32KB
 *                             - \ref OPCODE_BE_64K : Block Erase 64KB
 * @param[in]  u32NBit         Bit mode used for transmission.
 *                             - \ref SPIM_BITMODE_1 : 1-bit mode
 *                             - \ref SPIM_BITMODE_2 : 2-bit (dual) mode
 *                             - \ref SPIM_BITMODE_4 : 4-bit (quad) mode
 *                             - \ref SPIM_BITMODE_8 : 8-bit (octal) mode
 * @param[in]  i32IsSync       Whether to wait for the erase to complete.
 *                             - 1: Wait until erase is complete (blocking mode)
 *                             - 0: Return immediately (non-blocking mode)
 * @return     None.
 * @note       Ensure the flash is write-enabled before calling this function.
 *             The erase command will affect the entire block that contains the given address.
 */
void SPIM_EraseBlock(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32ErsCmd, uint32_t u32NBit, int32_t i32IsSync)
{
    uint8_t cmdBuf[6] = {0};
    uint32_t buf_idx = 0UL;
    uint32_t u32Is4ByteAddrTmp = u32Is4ByteAddr;

    cmdBuf[buf_idx] = (uint8_t)u32ErsCmd;
    buf_idx++;

    if ((SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) &&
            (u32NBit == SPIM_BITMODE_8))
    {
        cmdBuf[buf_idx] = (uint8_t)u32ErsCmd;
        buf_idx++;
        u32Is4ByteAddrTmp = SPIM_OP_ENABLE;
    }

    if (u32Is4ByteAddrTmp)
    {
        cmdBuf[buf_idx] = (uint8_t)((u32Addr >> 24U) & 0xFFU);
        buf_idx++;
    }

    cmdBuf[buf_idx] = (uint8_t)((u32Addr >> 16U) & 0xFFU);
    buf_idx++;
    cmdBuf[buf_idx] = (uint8_t)((u32Addr >> 8U) & 0xFFU);
    buf_idx++;
    cmdBuf[buf_idx] = (uint8_t)(u32Addr & 0xFFU);
    buf_idx++;

    /* Enable 4-byte address mode */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddrTmp, u32NBit);

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    (void)_SPIM_WriteData(spim, cmdBuf, buf_idx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (i32IsSync)
    {
        (void)_SPIM_WaitWriteDone(spim, u32NBit);
    }
}


/** @cond HIDDEN_SYMBOLS */

/**
 * @brief      Write data within a single page using the specified I/O mode.
 * @param[in]  u32Addr        Start address to write.
 * @param[in]  u32Is4ByteAddr Whether to use 4-byte addressing.
 *                             - 0: 3-byte address mode
 *                             - 1: 4-byte address mode
 * @param[in]  u32NTx         Number of bytes to write.
 * @param[in]  pu8TxBuf       Pointer to transmit buffer.
 * @param[in]  u8WrCmd        Write command opcode.
 * @param[in]  u32NBitCmd     Bit width for command phase.
 *                             - \ref SPIM_BITMODE_1
 *                             - \ref SPIM_BITMODE_2
 *                             - \ref SPIM_BITMODE_4
 *                             - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitAddr    Bit width for address phase.
 *                             - \ref SPIM_BITMODE_1
 *                             - \ref SPIM_BITMODE_2
 *                             - \ref SPIM_BITMODE_4
 *                             - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitDat     Bit width for data phase.
 *                             - \ref SPIM_BITMODE_1
 *                             - \ref SPIM_BITMODE_2
 *                             - \ref SPIM_BITMODE_4
 *                             - \ref SPIM_BITMODE_8
 * @param[in]  u32IsSync      Whether to wait until operation completes.
 *                             - 1: Blocking mode (wait for write to complete)
 *                             - 0: Non-blocking mode
 * @return     None.
 * @note       The data must not cross a page boundary. Caller must ensure write buffer stays within the same page.
 */
static void _SPIM_WriteInPageDataByIo(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, const uint8_t pu8TxBuf[], uint8_t u8WrCmd,
                                      uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, uint32_t u32IsSync)
{
    uint8_t au8CmdBuf[16] = {0};
    uint32_t u32BufIdx = 0;

    au8CmdBuf[0] = (uint8_t)u8WrCmd;
    au8CmdBuf[1] = (uint8_t)u8WrCmd;

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBitCmd);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    /* Write out command. */
    (void)_SPIM_WriteData(spim, au8CmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBitCmd);

    u32BufIdx = 0UL;

    if (u32Is4ByteAddr)
    {
        au8CmdBuf[u32BufIdx] = (uint8_t)(u32Addr >> 24U);
        u32BufIdx++;
    }

    au8CmdBuf[u32BufIdx] = (uint8_t)(u32Addr >> 16U);
    u32BufIdx++;
    au8CmdBuf[u32BufIdx] = (uint8_t)(u32Addr >> 8U);
    u32BufIdx++;
    au8CmdBuf[u32BufIdx] = (uint8_t) u32Addr;
    u32BufIdx++;

    /* Write out u32Address. */
    (void)_SPIM_WriteData(spim, au8CmdBuf, u32BufIdx, u32NBitAddr);

    /* Write out data. */
    (void)_SPIM_WriteData(spim, pu8TxBuf, u32NTx, u32NBitDat);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (u32IsSync)
    {
        (void)_SPIM_WaitWriteDone(spim, u32NBitCmd);
    }
}

/**
 * @brief      Write data within a single page using Page Write mode.
 * @param[in]  u32Addr        Start address to write.
 * @param[in]  u32Is4ByteAddr Whether to use 4-byte addressing.
 *                             - 0: 3-byte address mode
 *                             - 1: 4-byte address mode
 * @param[in]  u32NTx         Number of bytes to write.
 * @param[in]  pu8TxBuf       Pointer to transmit buffer.
 * @param[in]  u8WrCmd        Write command opcode.
 * @param[in]  u32IsSync         Whether to wait for the write to complete.
 *                             - 1: Blocking mode (wait for completion)
 *                             - 0: Non-blocking mode
 * @retval     SPIM_OK           Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT  Operation aborted due to timeout.
 * @note       If a timeout occurs while waiting for the operation to complete,
 *             this function sets @ref g_SPIM_i32ErrCode to @ref SPIM_TIMEOUT_ERR.
 * @warning    The write must not cross a page boundary. Caller is responsible for ensuring data fits within one page.
 */
static int32_t _SPIM_WriteInPageDataByPageWrite(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx,
                                                uint8_t pu8TxBuf[], uint8_t u8WrCmd, uint32_t u32IsSync)
{
    uint32_t u32WrCmdCode;

    (void)u32Is4ByteAddr;

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim)));

    if ((SPIM_GET_PHDMAW_CMD_DTR(spim) == SPIM_OP_ENABLE) &&
            (SPIM_GET_PHDMAW_CMD_WIDTH(spim) == PHASE_WIDTH_16))
    {
        /* Promote to uint32_t BEFORE shifting (MISRA compliant). */
        uint32_t u32Cmd = (uint32_t)u8WrCmd;
        u32WrCmdCode = (u32Cmd << 8U) | u32Cmd;
    }
    else
    {
        u32WrCmdCode = (uint32_t)(u8WrCmd);
    }

    /* Switch to Page Write mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_PAGEWRITE);
    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, u32WrCmdCode);

    /* SRAM u32Address. */
    spim->SRAMADDR = (uint32_t) pu8TxBuf;
    /* Transfer length. */
    spim->DMACNT = u32NTx;
    /* Flash u32Address. */
    spim->FADDR = u32Addr;

    (void)SPIM_WaitOpDone(spim, u32IsSync);

    (void)_SPIM_WaitWriteDone(spim, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim)));

    if (u8WrCmd == CMD_QUAD_PAGE_PROGRAM_EON)
    {
        /* Exit QPI mode. */
        _SPIM_EonSetQpiMode(spim, 0);
    }

    return SPIM_OK;
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief      Write data to SPI Flash using manual command sequence (I/O mode).
 * @param[in]  spim         Pointer to the SPIM instance.
 * @param[in]  u32Addr      Start address to write.
 * @param[in]  u32Is4ByteAddr
 *                          Specify address mode.
 *                          - 0: 3-byte address mode
 *                          - 1: 4-byte address mode
 * @param[in]  u32NTx       Number of bytes to write.
 * @param[in]  pu8TxBuf     Pointer to the transmit buffer.
 * @param[in]  u8WrCmd      Write command opcode.
 * @param[in]  u32NBitCmd   N-bit mode for transmitting the command.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitAddr  N-bit mode for transmitting the address.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitDat   N-bit mode for transmitting the data.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @return     None.
 *
 * @note       This function does not check for page boundary crossing. Caller must ensure
 *             data does not span across pages unless supported by the flash chip.
 *
 * @warning    This function assumes that the flash is already write-enabled. Be sure to
 *             call the appropriate Write Enable command before using this function.
 */
void SPIM_IO_Write(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t u8WrCmd,
                   uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat)
{
    /* Write out data to SPI Flash in small chunks (max 256 bytes per chunk) */
    /* index into tx buffer */
    uint32_t u32BufIdx = 0UL;
    uint32_t u32NTxTmp = u32NTx;
    uint32_t u32AddrTmp = u32Addr;

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));

    while (u32NTxTmp)
    {
        /* number of bytes to write in this chunk */
        uint32_t u32ToWr = (u32NTxTmp < SPIM_FLH_PAGE_SIZE) ? u32NTxTmp : SPIM_FLH_PAGE_SIZE;

        _SPIM_WriteInPageDataByIo(spim, u32AddrTmp, u32Is4ByteAddr,
                                  u32ToWr, &pu8TxBuf[u32BufIdx], u8WrCmd,
                                  u32NBitCmd, u32NBitAddr, u32NBitDat,
                                  SPIM_OP_ENABLE);
        u32AddrTmp += u32ToWr;
        u32NTxTmp -= u32ToWr;
        u32BufIdx += u32ToWr;
    }
}

/**
 * @brief      Read data from SPI Flash using manual command sequence (I/O mode).
 * @param[in]  spim         Pointer to the SPIM instance.
 * @param[in]  u32Addr      Start address to read.
 * @param[in]  u32Is4ByteAddr
 *                          Specify address mode.
 *                          - 0: 3-byte address mode
 *                          - 1: 4-byte address mode
 * @param[in]  u32NRx       Number of bytes to read.
 * @param[out] pu8RxBuf     Pointer to the receive buffer.
 * @param[in]  rdCmd        Read command opcode.
 * @param[in]  u32NBitCmd   N-bit mode for transmitting the command.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitAddr  N-bit mode for transmitting the address.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @param[in]  u32NBitDat   N-bit mode for transmitting/receiving data.
 *                          - \ref SPIM_BITMODE_1
 *                          - \ref SPIM_BITMODE_2
 *                          - \ref SPIM_BITMODE_4
 *                          - \ref SPIM_BITMODE_8
 * @param[in]  u32NDummy    Number of dummy cycles (clock cycles) after address phase.
 *
 * @return     None.
 *
 * @note       Caller must ensure that the read command is compatible with the selected bit
 *             modes and that the flash is ready for read operation.
 *
 * @warning    This function does not automatically handle status polling or chip select timing.
 *             Make sure the flash is not busy before calling this function.
 */
void SPIM_IO_Read(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint8_t rdCmd,
                  uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, uint32_t u32NDummy)
{
    uint8_t cmdBuf[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint32_t buf_idx = 0UL;

    cmdBuf[0] = rdCmd;
    cmdBuf[1] = rdCmd;

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    /* Write out command. */
    (void)_SPIM_WriteData(spim, cmdBuf, SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE ? 2UL : 1UL, u32NBitCmd);

    if (u32Is4ByteAddr)
    {
        cmdBuf[buf_idx] = (uint8_t)(u32Addr >> 24U);
        buf_idx++;
    }

    cmdBuf[buf_idx] = (uint8_t)(u32Addr >> 16U);
    buf_idx++;
    cmdBuf[buf_idx] = (uint8_t)(u32Addr >> 8U);
    buf_idx++;
    cmdBuf[buf_idx] = (uint8_t) u32Addr;
    buf_idx++;

    /* Add a byte for the address phase for dual or quad mode for read mode */
    buf_idx += ((u32NBitAddr == SPIM_BITMODE_2) || (u32NBitAddr == SPIM_BITMODE_4)) ? 1UL : 0UL;

    (void)_SPIM_WriteData(spim, cmdBuf, buf_idx, u32NBitAddr);    /* Write out u32Address. */

    /* Same bit mode as above. */
    (void)SPIM_IO_SendDummyByPhase(spim, u32NDummy);              /* Write out dummy bytes. */

    /* Read back data. */
    (void)_SPIM_ReadData(spim, pu8RxBuf, u32NRx, u32NBitDat);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Write data to SPI Flash using Page Write mode with PHDMAW.
 * @param[in]  spim           Pointer to the SPIM instance.
 * @param[in]  u32Addr        Start address to write.
 * @param[in]  u32Is4ByteAddr Specify whether 4-byte address mode is used.
 *                            - 0: 3-byte address mode
 *                            - 1: 4-byte address mode
 * @param[in]  u32NTx         Number of bytes to write.
 * @param[in]  pu8TxBuf       Pointer to the transmit buffer.
 * @param[in]  u8WrCmd        Write command opcode.
 *
 * @note       This function is optimized for aligned and page-boundary write operations.
 *             Make sure that the flash is write-enabled and ready before calling this function.
 *             The number of bytes to write must be a multiple of 8 (for DMA alignment).
 */
void SPIM_DMA_Write(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr,
                    uint32_t u32NTx, uint8_t *pu8TxBuf, uint32_t u8WrCmd)
{
    /* index into tx buffer */
    uint32_t u32BufIdx = 0UL;
    uint32_t u32NTxTmp = u32NTx;
    uint32_t u32AddrTmp = u32Addr;
    uint32_t u32CMDBits = SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim));
    uint32_t u32Target4B = (SPIM_GET_PHDMAW_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    uint32_t u32IsQuadEn;
    uint32_t u32QuadMode = (((SPIM_GET_PHDMAW_ADDR_BITMODE(spim) == PHASE_QUAD_MODE) ||
                             (SPIM_GET_PHDMAW_DATA_BITMODE(spim) == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    (void)u32Is4ByteAddr;

    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CMDBits);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Target4B, u32CMDBits);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CMDBits);
    }

    while (u32NTxTmp)
    {
        /* number of bytes to write in this chunk */
        uint32_t u32ToWr = (u32NTxTmp <= SPIM_FLH_PAGE_SIZE) ? u32NTxTmp : SPIM_FLH_PAGE_SIZE;

        (void)_SPIM_WriteInPageDataByPageWrite(spim, u32AddrTmp, u32Is4ByteAddr, u32ToWr, &pu8TxBuf[u32BufIdx], u8WrCmd, SPIM_OP_ENABLE);

        /* Advance indicator. */
        u32AddrTmp += u32ToWr;
        u32NTxTmp -= u32ToWr;
        u32BufIdx += u32ToWr;
    }
}

/**
 * @brief      Read data from SPI Flash using Page Read mode with PHDMAR.
 * @param[in]  spim           Pointer to the SPIM instance.
 * @param[in]  u32Addr        Start address to read.
 * @param[in]  u32Is4ByteAddr Specify whether 4-byte address mode is used.
 *                            - 0: 3-byte address mode
 *                            - 1: 4-byte address mode
 * @param[in]  u32NRx         Number of bytes to read.
 * @param[out] pu8RxBuf       Pointer to the receive buffer.
 * @param[in]  u8RdCmd        Read command opcode.
 * @param[in]  u32IsSync      Specify whether to use blocking mode.
 *                            - 0: Non-blocking (return immediately)
 *                            - 1: Blocking (wait for completion)
 *
 * @retval     SPIM_OK            Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT   Operation failed due to timeout.
 *
 * @note       Before calling this API, you must initialize the read phase by calling
 *             @ref SPIM_DMADMM_InitPhase with appropriate settings for PHDMAR (Page High-Speed DMA Read).
 */
int32_t SPIM_DMA_Read(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[],
                      uint8_t u8RdCmd, uint32_t u32IsSync)
{
    uint32_t u32Is4ByteAddrTmp = (SPIM_GET_PHDMAR_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    uint32_t u32RdCmdTmp;
    uint32_t u32CMDBits = SPIM_PhaseModeToNBit(SPIM_GET_PHDMAR_CMD_BITMODE(spim));
    uint32_t u32IsQuadEn;
    uint32_t u32QuadMode = (((SPIM_GET_PHDMAR_ADDR_BITMODE(spim) == PHASE_QUAD_MODE) ||
                             (SPIM_GET_PHDMAR_DATA_BITMODE(spim) == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    (void)u32Is4ByteAddr;

    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CMDBits);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddrTmp, u32CMDBits);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CMDBits);
    }

    if ((SPIM_GET_PHDMAR_MODE_DATAWIDTH(spim) != PHASE_WIDTH_0) &&
            (_SPIM_IsCrenAllowed(u8RdCmd) == SPIM_OP_ENABLE))
    {
        SPIM_SET_MODE_DATA(spim, SPIM_DMM_ENABLE_CREN);
        //SPIM_ENABLE_DMM_CREN(spim);
    }

    if ((SPIM_GET_PHDMAR_CMD_DTR(spim) == SPIM_OP_ENABLE) &&
            (SPIM_GET_PHDMAR_CMD_WIDTH(spim) == PHASE_WIDTH_16))
    {
        /* Promote to uint32_t BEFORE shifting (MISRA compliant). */
        uint32_t u32Cmd = (uint32_t)u8RdCmd;
        u32RdCmdTmp = (u32Cmd << 8U) | u32Cmd;
    }
    else
    {
        u32RdCmdTmp = (uint32_t)(u8RdCmd);
    }

    /* Switch to Page Read mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_PAGEREAD);
    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, u32RdCmdTmp);

    /* SRAM u32Address. */
    spim->SRAMADDR = (uint32_t) pu8RxBuf;
    /* Transfer length. */
    spim->DMACNT = u32NRx;
    /* Flash u32Address. */
    spim->FADDR = u32Addr;

    return SPIM_WaitOpDone(spim, u32IsSync);
}

/**
 * @brief      Enter Direct Map Mode (DMM) for memory-mapped SPI Flash access.
 *
 * @param[in]  spim            Pointer to the SPIM instance.
 * @param[in]  u32Is4ByteAddr  Specify whether 4-byte address mode is used.
 *                             - 0: Use 3-byte address mode.
 *                             - 1: Use 4-byte address mode.
 * @param[in]  u8RdCmd         SPI Flash read command used in DMM mode (e.g., 0x0B, 0x6B, 0xEB).
 * @param[in]  u32IdleIntvl    Idle interval between transactions, in SPIM clock cycles.
 *
 * @return     None.
 *
 * @note       Before entering DMM, ensure the Flash is ready and in the correct mode
 *             (e.g., QSPI, DTR). The command should match the expected Flash read protocol.
 */
void SPIM_EnterDirectMapMode(SPIM_T *spim, uint32_t u32Is4ByteAddr, uint8_t u8RdCmd, uint32_t u32IdleIntvl)
{
    uint32_t u32RdCmdTmp;
    uint32_t u32CMDBits = SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim));
    uint32_t u32Is4ByteAddrTmp = (SPIM_GET_PHDMM_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    uint32_t u32IsQuadEn;
    uint32_t u32QuadMode = (((SPIM_GET_PHDMM_ADDR_BITMODE(spim) == PHASE_QUAD_MODE) ||
                             (SPIM_GET_PHDMM_DATA_BITMODE(spim) == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    (void)u32Is4ByteAddr;

    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CMDBits);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddrTmp, u32CMDBits);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CMDBits);
    }

    if ((SPIM_GET_PHDMM_MODE_DATAWIDTH(spim) != PHASE_WIDTH_0) &&
            (_SPIM_IsCrenAllowed(u8RdCmd) == SPIM_OP_ENABLE))
    {
        SPIM_SET_MODE_DATA(spim, SPIM_DMM_ENABLE_CREN);
        SPIM_ENABLE_DMM_CREN(spim);
    }

    if ((SPIM_GET_PHDMM_CMD_WIDTH(spim) == PHASE_WIDTH_16) &&
            (SPIM_GET_PHDMM_CMD_DTR(spim) == SPIM_OP_ENABLE))
    {
        /* Promote to uint32_t BEFORE shifting (MISRA compliant). */
        uint32_t u32Cmd = (uint32_t)u8RdCmd;
        u32RdCmdTmp = (u32Cmd << 8U) | u32Cmd;
    }
    else
    {
        u32RdCmdTmp = (uint32_t)(u8RdCmd);
    }

    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, u32RdCmdTmp);
    /* Idle interval. */
    SPIM_SET_IDL_INTVL(spim, u32IdleIntvl);
    /* Switch to Direct Map mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_DIRECTMAP);
}

/**
 * @brief      Exit Direct Map Mode (DMM).
 * @param[in]  spim    Pointer to the SPIM instance.
 * @return     None.
 * @note       Call this function before switching to normal command-based access mode.
 */
void SPIM_ExitDirectMapMode(SPIM_T *spim)
{
    if (SPIM_GET_DMM_BWEN(spim) == SPIM_OP_ENABLE)
    {
        (void)SPIM_SetWrapAroundEnable(spim, 0UL);
    }

    if (SPIM_GET_MODE_DATA(spim) == SPIM_DMM_ENABLE_CREN)
    {
        (void)_SPIM_SetContReadDisable(spim);
    }
}

/**
 * @brief      Convert I/O phase mode to bit width size.
 *
 * @param[in]  u32Phase  I/O phase mode.
 *                       - \ref PHASE_NORMAL_MODE : 1-bit mode
 *                       - \ref PHASE_DUAL_MODE   : 2-bit mode
 *                       - \ref PHASE_QUAD_MODE   : 4-bit mode
 *
 * @return     Bit width of the specified phase mode (1, 2, or 4).
 */
uint32_t SPIM_PhaseModeToNBit(uint32_t u32Phase)
{
    return ((u32Phase == PHASE_NORMAL_MODE) ? SPIM_BITMODE_1 :
            (u32Phase == PHASE_DUAL_MODE) ? SPIM_BITMODE_2 :
            (u32Phase == PHASE_QUAD_MODE) ? SPIM_BITMODE_4 :
            (u32Phase == PHASE_OCTAL_MODE) ? SPIM_BITMODE_8 :
            SPIM_BITMODE_1);
}

/**
 * @brief      Get the address of the DMA/DMM Phase Setting Register based on operation mode.
 *
 * @param[in]  spim        Pointer to the SPIM instance.
 * @param[in]  u32OPMode   SPI functional operation mode.
 *                         - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA write mode (PHDMAW)
 *                         - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode  (PHDMAR)
 *                         - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Map mode (PHDMM)
 *
 * @return     Register address of the corresponding DMA/DMM phase setting.
 */
static volatile uint32_t *_SPIM_SwitchPhaseRegister(SPIM_T *spim, uint32_t u32OPMode)
{
    volatile uint32_t *pu32Addr = ((u32OPMode == SPIM_CTL0_OPMODE_PAGEWRITE) ? &spim->PHDMAW :
                                   (u32OPMode == SPIM_CTL0_OPMODE_PAGEREAD)  ? &spim->PHDMAR :
                                   (u32OPMode == SPIM_CTL0_OPMODE_DIRECTMAP) ? &spim->PHDMM :
                                   (uint32_t *)NULL);

    return (uint32_t *)(uintptr_t)pu32Addr;
}

/**
 * @brief      Set the command phase configuration for DMA or Direct Map operation.
 *
 * @param[in]  spim        Pointer to the SPIM instance.
 * @param[in]  u32OPMode   SPI operation mode.
 *                         - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA write mode
 *                         - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode
 *                         - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Map mode
 * @param[in]  u32NBit     Command bit mode.
 *                         - \ref PHASE_NORMAL_MODE : Standard mode (1-bit)
 *                         - \ref PHASE_DUAL_MODE   : Dual mode (2-bit)
 *                         - \ref PHASE_QUAD_MODE   : Quad mode (4-bit)
 *                         - \ref PHASE_OCTAL_MODE  : Octal mode (8-bit)
 * @param[in]  u32Width    Command width.
 *                         - \ref PHASE_WIDTH_8     : 8-bit command
 *                         - \ref PHASE_WIDTH_16    : 16-bit command
 *                         - \ref PHASE_WIDTH_24    : 24-bit command
 *                         - \ref PHASE_WIDTH_32    : 32-bit command
 * @param[in]  u32DTREn    Enable or disable DTR (Double Transfer Rate) mode.
 *                         - SPIM_OP_DISABLE : Disable DTR
 *                         - SPIM_OP_ENABLE  : Enable DTR
 *
 * @return     None.
 */
static int32_t _SPIM_SetCMDPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32DTREn)
{
    volatile uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == (uint32_t *)NULL)
    {
        return SPIM_ERR_FAIL;
    }

    /* clear Command phase setting. */
    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_CMD_Pos);

    *pu32PhaseReg |= ((u32Width << SPIM_PHDMAW_DW_CMD_Pos) |
                      (u32DTREn << SPIM_PHDMAR_DTR_CMD_Pos) |
                      (u32NBit << SPIM_PHDMAW_BM_CMD_Pos));

    return SPIM_OK;
}

/**
 * @brief      Set the address phase configuration for DMA or Direct Map operation.
 *
 * @param[in]  spim        Pointer to the SPIM instance.
 * @param[in]  u32OPMode   SPI operation mode.
 *                         - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA write mode
 *                         - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode
 *                         - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Map mode
 * @param[in]  u32NBit     Address bit mode.
 *                         - \ref PHASE_NORMAL_MODE : Standard mode (1-bit)
 *                         - \ref PHASE_DUAL_MODE   : Dual mode (2-bit)
 *                         - \ref PHASE_QUAD_MODE   : Quad mode (4-bit)
 *                         - \ref PHASE_OCTAL_MODE  : Octal mode (8-bit)
 * @param[in]  u32Width    Address width.
 *                         - \ref PHASE_WIDTH_8     : 8-bit address
 *                         - \ref PHASE_WIDTH_16    : 16-bit address
 *                         - \ref PHASE_WIDTH_24    : 24-bit address
 *                         - \ref PHASE_WIDTH_32    : 32-bit address
 * @param[in]  u32DTREn    Enable or disable DTR (Double Transfer Rate) mode.
 *                         - 0: Disable DTR
 *                         - 1: Enable DTR
 *
 * @retval     SPIM_OK          Address phase configured successfully.
 * @retval     SPIM_ERR_FAIL    Invalid parameters or unsupported configuration.
 */
static int32_t _SPIM_SetAddrPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32DTREn)
{
    volatile uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == (uint32_t *)NULL)
    {
        return SPIM_ERR_FAIL;
    }

    /* clear Address phase setting. */
    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_ADDR_Pos);

    *pu32PhaseReg |= ((u32Width << SPIM_PHDMAW_DW_ADDR_Pos) |
                      (u32NBit << SPIM_PHDMAW_BM_ADDR_Pos) |
                      (u32DTREn << SPIM_PHDMAR_DTR_ADDR_Pos));

    return SPIM_OK;
}


static void _SPIM_ClearContReadPhase(SPIM_T *spim, uint32_t u32OPMode)
{
    volatile uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_READMODE_Pos);
}

/**
 * @brief      Set the Continue Read phase for DMA or Direct Map operation.
 *
 * @param[in]  spim       Pointer to the SPIM instance.
 * @param[in]  u32OPMode  SPI operation mode.
 *                        - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode
 *                        - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Map mode
 * @param[in]  u32NBit    Bit mode used to transmit continue read command.
 *                        - \ref PHASE_NORMAL_MODE : Standard mode (1-bit)
 *                        - \ref PHASE_DUAL_MODE   : Dual mode (2-bit)
 *                        - \ref PHASE_QUAD_MODE   : Quad mode (4-bit)
 *                        - \ref PHASE_OCTAL_MODE  : Octal mode (8-bit)
 * @param[in]  u32Width   Command width.
 *                        - \ref PHASE_WIDTH_8     : 8-bit command
 *                        - \ref PHASE_WIDTH_16    : 16-bit command
 *                        - \ref PHASE_WIDTH_24    : 24-bit command
 *                        - \ref PHASE_WIDTH_32    : 32-bit command
 * @param[in]  u32ContEn  Enable or disable Continue Read mode.
 *                        - \ref PHASE_ENABLE_CONT_READ    : Enable Continue Read mode
 *                        - \ref PHASE_DISABLE_CONT_READ   : Disable Continue Read mode
 * @param[in]  u32DTREn   Enable or disable DTR (Double Transfer Rate) mode.
 *                        - 0: Disable DTR
 *                        - 1: Enable DTR
 *
 * @retval     SPIM_OK          Continue Read phase set successfully.
 * @retval     SPIM_ERR_FAIL    Invalid configuration or operation failed.
 */
static int32_t _SPIM_SetContReadPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32ContEn, uint32_t u32DTREn)
{
    volatile uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == (uint32_t *)NULL)
    {
        return SPIM_ERR_FAIL;
    }

    /* clear Read Mode phase setting. */
    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_READMODE_Pos);

    //if ((u32ContEn == PHASE_ENABLE_CONT_READ) &&
    //        (u32OPMode == SPIM_CTL0_OPMODE_DIRECTMAP))
    //{
    //    SPIM_ENABLE_DMM_CREN(spim);
    //}

    if ((u32ContEn == PHASE_ENABLE_CONT_READ) &&
            (u32OPMode != SPIM_CTL0_OPMODE_PAGEWRITE))
    {
        *pu32PhaseReg |= ((u32Width << SPIM_PHDMAR_DW_MODE_Pos) |
                          (u32NBit << SPIM_PHDMAR_BM_MODE_Pos) |
                          (u32DTREn << SPIM_PHDMAR_DTR_MODE_Pos));
        /* Enable Contiue Read Mode */
        //SPIM_SET_MODE_DATA(spim, SPIM_DMM_ENABLE_CREN);
    }
    else
    {
        *pu32PhaseReg &= ~(SPIM_PHDMAR_DW_MODE_Msk);
        SPIM_CLEAR_MODE_DATA(spim);
    }

    return SPIM_OK;
}

/**
 * @brief      Set the Data Phase configuration for DMA or Direct Map operation.
 *
 * @param[in]  spim          Pointer to the SPIM instance.
 * @param[in]  u32OPMode     SPI operation mode.
 *                           - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA write mode
 *                           - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode
 *                           - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Map mode
 * @param[in]  u32NBit       Bit mode used during the data phase.
 *                           - \ref PHASE_NORMAL_MODE : Standard mode (1-bit)
 *                           - \ref PHASE_DUAL_MODE   : Dual mode (2-bit)
 *                           - \ref PHASE_QUAD_MODE   : Quad mode (4-bit)
 *                           - \ref PHASE_OCTAL_MODE  : Octal mode (8-bit)
 * @param[in]  u32ByteOrder  Data byte order for Octal SPI Flash during read phase.
 *                           - \ref PHASE_ORDER_MODE0 : {byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7}
 *                           - \ref PHASE_ORDER_MODE1 : {byte7, byte6, byte5, byte4, byte3, byte2, byte1, byte0}
 *                           - \ref PHASE_ORDER_MODE2 : {byte1, byte0, byte3, byte2, byte5, byte4, byte7, byte6}
 *                           - \ref PHASE_ORDER_MODE3 : {byte6, byte7, byte4, byte5, byte2, byte3, byte0, byte1}
 * @param[in]  u32DTREn      Enable or disable DTR (Double Transfer Rate) mode.
 *                           - \ref SPIM_OP_ENABLE
 *                           - \ref SPIM_OP_DISABLE
 * @param[in]  u32RdDQS      Enable or disable Read DQS (Data Strobe).
 *                           - \ref SPIM_OP_ENABLE
 *                           - \ref SPIM_OP_DISABLE
 *
 * @retval     SPIM_OK       Configuration successful.
 * @retval     SPIM_ERR_FAIL Configuration failed.
 */
static int32_t _SPIM_SetDataPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit,
                                  uint32_t u32ByteOrder, uint32_t u32DTREn, uint32_t u32RdDQS)
{
    volatile uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == (uint32_t *)NULL)
    {
        return SPIM_ERR_FAIL;
    }

    /* clear Data phase setting. */
    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_DATA_Pos);

    *pu32PhaseReg |= ((u32RdDQS << SPIM_PHDMAR_RDQS_DATA_Pos)    |
                      (u32DTREn << SPIM_PHDMAR_DTR_DATA_Pos)     |
                      (u32ByteOrder << SPIM_PHDMAW_PBO_DATA_Pos) |
                      (u32NBit << SPIM_PHDMAR_BM_DATA_Pos));

    return SPIM_OK;
}

/**
 * @brief      Initialize command phase settings for DMA/DMM SPI Flash operations.
 *
 * @param[in]  spim           Pointer to the SPIM instance.
 * @param[in]  psPhaseTable   Pointer to the command phase table. This table must be
 *                            defined according to the supported command codes of the
 *                            target SPI Flash device.
 * @param[in]  u32OPMode      SPI operation mode:
 *                            - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA write mode
 *                            - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA read mode
 *                            - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 * @note  Invoke SPIM_DMADMM_InitPhase before any DMA read, DMA write, or
 *        Direct Map (DMM) access to configure command / address / data phases.
 *
 *        Reinvoke SPIM_DMADMM_InitPhase whenever any of the following change:
 *        - Command opcode
 *        - Operation mode (Page Write, Page Read, Direct Map)
 *
 *        Omitting reinitialization after such changes can lead to transfer
 *        errors or data corruption.
 *
 * @return     None.
 */
void SPIM_DMADMM_InitPhase(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable, uint32_t u32OPMode)
{
    uint32_t u32Is4ByteAddr = 0;
    uint32_t u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                             (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);
    uint32_t u32CmdBit = SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase);
    uint32_t u32IsQuadEn;

    SPIM_DISABLE_DMM_BWEN(spim);
    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CmdBit);

    /* Set SPIM DTR Mode */
    SPIM_SET_DTR_MODE(spim, psPhaseTable->u32CMDDTR);

    u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddr, u32CmdBit);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CmdBit);
    }

    /* Set Flash Command Phase */
    (void)_SPIM_SetCMDPhase(spim,
                            u32OPMode,
                            psPhaseTable->u32CMDPhase,
                            psPhaseTable->u32CMDWidth,
                            psPhaseTable->u32CMDDTR);

    /* Set Flash Address Phase */
    (void)_SPIM_SetAddrPhase(spim,
                             u32OPMode,
                             psPhaseTable->u32AddrPhase,
                             psPhaseTable->u32AddrWidth,
                             psPhaseTable->u32AddrDTR);

    /* Set DMA Read/DMM Continue Read Phase */
    (void)_SPIM_SetContReadPhase(spim,
                                 u32OPMode,
                                 psPhaseTable->u32RdModePhase,
                                 psPhaseTable->u32RdModeWidth,
                                 psPhaseTable->u32ContRdEn,
                                 psPhaseTable->u32RdModeDTR);

    /* Set Flash Data Phase */
    (void)_SPIM_SetDataPhase(spim,
                             u32OPMode,
                             psPhaseTable->u32DataPhase,
                             psPhaseTable->u32ByteOrder,
                             psPhaseTable->u32DataDTR,
                             psPhaseTable->u32RDQS);

    /* Set Dummy Cycle After Address. */
    if (u32OPMode == SPIM_CTL0_OPMODE_PAGEREAD)
    {
        SPIM_SET_DMAR_DC(spim, psPhaseTable->u32DcNum);
    }
    else if (u32OPMode == SPIM_CTL0_OPMODE_DIRECTMAP)
    {
        SPIM_SET_DMM_DC(spim, psPhaseTable->u32DcNum);
    }
    else
    {
        /* Do nothing for Page Write mode. */
    }
}

/*----------------------------------------------------------------------------*/
/* I/O Read/Write Operations by Phase Table                                   */
/*----------------------------------------------------------------------------*/
/**
 * @brief      Send command phase in Normal I/O mode.
 *
 * @param[in]  spim         Pointer to the SPIM instance.
 * @param[in]  u32OPMode    Normal I/O operation mode:
 *                          - \ref SPIM_IO_WRITE_PHASE : Write command phase
 *                          - \ref SPIM_IO_READ_PHASE  : Read command phase
 * @param[in]  u32OpCMD     SPI Flash command to send, defined by the SPI Flash specification.
 * @param[in]  u32CMDPhase  Command bit mode:
 *                          - \ref PHASE_NORMAL_MODE : Send command using Standard mode
 *                          - \ref PHASE_DUAL_MODE   : Send command using Dual mode
 *                          - \ref PHASE_QUAD_MODE   : Send command using Quad mode
 *                          - \ref PHASE_OCTAL_MODE  : Send command using Octal mode
 * @param[in]  u32DTREn     Enable or disable Double Transfer Rate (DTR) mode:
 *                          - \ref SPIM_OP_ENABLE
 *                          - \ref SPIM_OP_DISABLE
 * @return     None.
 */
void SPIM_IO_SendCommandByPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32OpCMD,
                                uint32_t u32CMDPhase, uint32_t u32DTREn)
{
    const uint8_t u8CmdBuf[2] = {(uint8_t)u32OpCMD, (uint8_t)u32OpCMD};

    /* DTR Activated. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    if (u32OPMode == SPIM_IO_WRITE_PHASE)
    {
        /* Write Enable. */
        SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, SPIM_PhaseModeToNBit(u32CMDPhase));
    }

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    /* Write out command. */
    (void)_SPIM_WriteData(spim, u8CmdBuf, (u32DTREn == SPIM_OP_ENABLE) ? 2UL : 1UL, SPIM_PhaseModeToNBit(u32CMDPhase));

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Send address phase in Normal I/O mode.
 *
 * @param[in]  spim             Pointer to the SPIM instance.
 * @param[in]  u32Is4ByteAddr   Specify whether to use 4-byte address mode (1: 4-byte, 0: 3-byte).
 * @param[in]  u32Addr          The starting read/write address to be sent.
 * @param[in]  u32AddrPhase     Address bit mode:
 *                              - \ref PHASE_NORMAL_MODE : Send address using Standard mode.
 *                              - \ref PHASE_DUAL_MODE   : Send address using Dual mode.
 *                              - \ref PHASE_QUAD_MODE   : Send address using Quad mode.
 *                              - \ref PHASE_OCTAL_MODE  : Send address using Octal mode.
 * @param[in]  u32DTREn         Enable or disable Double Transfer Rate (DTR) mode:
 *                              - \ref SPIM_OP_ENABLE
 *                              - \ref SPIM_OP_DISABLE
 * @return     None.
 */
void SPIM_IO_SendAddressByPhase(SPIM_T *spim,
                                uint32_t u32Is4ByteAddr,
                                uint32_t u32Addr,
                                uint32_t u32AddrPhase,
                                uint32_t u32DTREn)
{
    uint8_t u8CmdBuf[6] = {0};
    uint8_t u8BufIdx = 0;

    /* DTR Activated. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    if (u32Is4ByteAddr)
    {
        u8CmdBuf[u8BufIdx] = (uint8_t)(u32Addr >> 24);
        u8BufIdx++;
    }

    u8CmdBuf[u8BufIdx] = (uint8_t)(u32Addr >> 16);
    u8BufIdx++;
    u8CmdBuf[u8BufIdx] = (uint8_t)(u32Addr >> 8);
    u8BufIdx++;
    u8CmdBuf[u8BufIdx] = (uint8_t)u32Addr;
    u8BufIdx++;

    /* This field is for continuous read mode, but the I/O mode is not supported. */
    if ((gau8IDBuf[0] == (uint8_t)MFGID_WINBOND) &&
            ((SPIM_PhaseModeToNBit(u32AddrPhase) == 2UL) ||
             (SPIM_PhaseModeToNBit(u32AddrPhase) == 4UL)))
    {
        u8BufIdx++;
    }

    /* Write out u32Address. */
    (void)_SPIM_WriteData(spim, u8CmdBuf, u8BufIdx, SPIM_PhaseModeToNBit(u32AddrPhase));

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Send dummy cycles in Normal I/O mode.
 *
 * @param[in]  spim       Pointer to the SPIM instance.
 * @param[in]  u32NDummy  Number of dummy clock cycles to insert, typically defined by the SPI Flash command specification.
 *
 * @retval     SPIM_OK        Operation completed successfully.
 * @retval     SPIM_ERR_FAIL  Operation failed due to invalid configuration or hardware issue.
 */
int32_t SPIM_IO_SendDummyByPhase(SPIM_T *spim, uint32_t u32NDummy)
{
    uint32_t u32DTREn = 0;
    int32_t i32Ret = SPIM_OK;

    if (u32NDummy == 0UL)
    {
        return SPIM_OK;
    }

    u32DTREn = SPIM_GET_DTR_MODE(spim);

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    SPIM_SwitchNBitInput(spim, SPIM_BITMODE_1);
    spim->TX[0] = 0x00;

    /* Switch to Normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
    SPIM_SET_DATA_WIDTH(spim, u32NDummy);
    SPIM_SET_BURST_DATA(spim, (uint32_t)SPIM_BURSTNUM_1);

    i32Ret = SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);

    /* Restore DTR Mode. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    return i32Ret;
}

/**
 * @brief      Execute data phase in Normal I/O mode.
 *
 * @param[in]  spim           Pointer to the SPIM instance.
 * @param[in]  u32OPMode      Operation mode for Normal I/O.
 *                            - \ref SPIM_IO_WRITE_PHASE : Write mode.
 *                            - \ref SPIM_IO_READ_PHASE  : Read mode.
 * @param[in,out] pu8TRxBuf   Pointer to transmit or receive data buffer.
 *                            - In write mode: pointer to transmit data.
 *                            - In read mode: buffer for received data.
 * @param[in]  u32TRxSize     Size of data to transmit or receive in bytes.
 * @param[in]  u32DataPhase   Data phase bit mode.
 *                            - \ref PHASE_NORMAL_MODE : Standard mode.
 *                            - \ref PHASE_DUAL_MODE   : Dual mode.
 *                            - \ref PHASE_QUAD_MODE   : Quad mode.
 *                            - \ref PHASE_OCTAL_MODE  : Octal mode.
 * @param[in]  u32DTREn       Enable or disable DTR (Double Transfer Rate) mode.
 *                            - \ref SPIM_OP_ENABLE
 *                            - \ref SPIM_OP_DISABLE
 * @param[in]  u32RdDQS       Enable or disable DQS input during read.
 *                            - \ref SPIM_OP_ENABLE
 *                            - \ref SPIM_OP_DISABLE
 *
 * @retval     SPIM_OK        Operation successful.
 * @retval     SPIM_ERR_FAIL  Operation failed.
 */
void SPIM_IO_RWDataByPhase(SPIM_T *spim, uint32_t u32OPMode, uint8_t *pu8TRxBuf,
                           uint32_t u32TRxSize, uint32_t u32DataPhase, uint32_t u32DTREn, uint32_t u32RdDQS)
{
    /* DTR Activated. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    if (gau8IDBuf[0] == (uint8_t)MFGID_IFX)
    {
        SPIM_SET_RDQS_MODE(spim, u32RdDQS);
    }

    if (u32OPMode == SPIM_IO_WRITE_PHASE)
    {
        /* Write out data. */
        (void)_SPIM_WriteData(spim, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }
    else
    {
        /* Read back data. */
        (void)_SPIM_ReadData(spim, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }

    /* CS Deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SET_RDQS_MODE(spim, SPIM_OP_DISABLE);

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Write data to SPI Flash using command and phase settings in Normal I/O mode.
 *
 * @param[in]  spim            Pointer to the SPIM instance.
 * @param[in]  psPhaseTable    Pointer to the SPIM phase table, which contains command and phase configuration.
 * @param[in]  u32Addr         Target address in SPI Flash to write data.
 * @param[in]  u32Is4ByteAddr  Set to 1 for 4-byte addressing mode, 0 for 3-byte mode.
 * @param[in]  pu8TxBuf        Pointer to the data buffer to be written to SPI Flash.
 * @param[in]  u32RdSize       Number of bytes to write.
 * @param[in]  u32WrDone       Set to 1 to wait for write complete; 0 to return immediately.
 *
 * @return     None.
 */
static void SPIM_WriteInPageDataByPhaseIO(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable,
                                          uint32_t u32Addr, uint32_t u32Is4ByteAddr,
                                          uint8_t *pu8TxBuf, uint32_t u32RdSize,
                                          uint32_t u32WrDone)
{
    /* Send command and address. */
    SPIM_IO_SendCommandByPhase(spim, SPIM_IO_WRITE_PHASE, psPhaseTable->u32CMDCode,
                               psPhaseTable->u32CMDPhase, psPhaseTable->u32CMDDTR);

    SPIM_IO_SendAddressByPhase(spim,
                               u32Is4ByteAddr,
                               u32Addr,
                               psPhaseTable->u32AddrPhase,
                               psPhaseTable->u32AddrDTR);

    /* Write out dummy bytes. */
    (void)SPIM_IO_SendDummyByPhase(spim, psPhaseTable->u32DcNum);

    /* Write out data bytes. */
    SPIM_IO_RWDataByPhase(spim,
                          SPIM_IO_WRITE_PHASE,
                          pu8TxBuf,
                          u32RdSize,
                          psPhaseTable->u32DataPhase,
                          psPhaseTable->u32DataDTR,
                          psPhaseTable->u32RDQS);

    if (u32WrDone)
    {
        /* DTR Activated. */
        SPIM_SET_DTR_MODE(spim, psPhaseTable->u32CMDDTR);

        (void)_SPIM_WaitWriteDone(spim, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));

        /* DTR Deactivated. */
        SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
    }
}

/**
 * @brief      Write data to SPI Flash using specified phase settings (I/O mode).
 *
 * @param[in]  spim          Pointer to the SPIM instance.
 * @param[in]  psPhaseTable  Pointer to the phase table based on SPI Flash specifications.
 *                           This table defines the command and associated phase parameters.
 * @param[in]  u32Addr       Target address in SPI Flash to begin writing.
 * @param[in]  pu8TxBuf      Pointer to the data buffer to be written.
 * @param[in]  u32WrSize     Number of bytes to write.
 * @param[in]  u32WrDone     Set to 1 to wait for write completion; 0 to skip waiting.
 *
 * @return     None.
 */
void SPIM_IO_WriteByPhase(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable,
                          uint32_t u32Addr, uint8_t *pu8TxBuf, uint32_t u32WrSize, uint32_t u32WrDone)
{
    uint32_t u32DTREn = SPIM_GET_DTR_MODE(spim);
    /* Buffer index. */
    uint32_t u32BufIdx = 0UL;
    uint32_t u32AddrTmp = u32Addr;
    uint32_t u32WrSizeTmp = u32WrSize;
    /* 4-byte address mode or not. */
    uint32_t u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32)
                               ? SPIM_OP_ENABLE
                               : SPIM_OP_DISABLE);
    /* Quad mode or not. */
    uint32_t u32CMDBits = SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase);
    uint32_t u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                             (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);
    uint32_t u32IsQuadEn;

    SPIM_DISABLE_DMM_BWEN(spim);
    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CMDBits);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddr, u32CMDBits);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CMDBits);
    }

    while (u32WrSizeTmp)
    {
        /* The number of bytes to write*/
        uint32_t toWr = (u32WrSizeTmp < SPIM_FLH_PAGE_SIZE) ? u32WrSizeTmp : SPIM_FLH_PAGE_SIZE;

        SPIM_WriteInPageDataByPhaseIO(spim, psPhaseTable, u32AddrTmp, u32Is4ByteAddr, &pu8TxBuf[u32BufIdx], toWr, u32WrDone);
        /* Advance indicator. */
        u32AddrTmp += toWr;
        u32WrSizeTmp -= toWr;
        u32BufIdx += toWr;
    }

    SPIM_SET_DTR_MODE(spim, u32DTREn);
}

/**
 * @brief      Read data from SPI Flash using specified phase settings (I/O mode).
 *
 * @param[in]  spim          Pointer to the SPIM instance.
 * @param[in]  psPhaseTable  Pointer to the SPI Flash phase configuration table.
 *                           This table defines supported commands and transfer phases.
 * @param[in]  u32Addr       Starting address in SPI Flash to read from.
 * @param[out] pu8RxBuf      Pointer to the buffer to store read data.
 * @param[in]  u32RdSize     Number of bytes to read.
 *
 * @return     None.
 */
void SPIM_IO_ReadByPhase(SPIM_T *spim, const SPIM_PHASE_T *psPhaseTable, uint32_t u32Addr, uint8_t *pu8RxBuf, uint32_t u32RdSize)
{
    uint32_t u32DTREn = SPIM_GET_DTR_MODE(spim);
    uint32_t u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);
    uint32_t u32CMDBits = SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase);
    uint32_t u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                             (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);
    uint32_t u32IsQuadEn;

    SPIM_DISABLE_DMM_BWEN(spim);
    SPIM_DISABLE_DMM_CREN(spim);
    SPIM_CLEAR_MODE_DATA(spim);

    u32IsQuadEn = SPIM_IsQuadEnabled(spim, u32CMDBits);

    /* Enable/disable 4-Byte Address. */
    _SPIM_Sync4ByteAddrMode(spim, u32Is4ByteAddr, u32CMDBits);

    if (u32QuadMode && u32IsQuadEn)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, u32CMDBits);
    }

    /* send command, address, dummy, data */
    SPIM_IO_SendCommandByPhase(spim,
                               SPIM_IO_READ_PHASE,
                               psPhaseTable->u32CMDCode,
                               psPhaseTable->u32CMDPhase,
                               psPhaseTable->u32CMDDTR);

    SPIM_IO_SendAddressByPhase(spim,
                               u32Is4ByteAddr,
                               u32Addr,
                               psPhaseTable->u32AddrPhase,
                               psPhaseTable->u32AddrDTR);

    /* Write out dummy bytes. */
    (void)SPIM_IO_SendDummyByPhase(spim, psPhaseTable->u32DcNum);

    SPIM_IO_RWDataByPhase(spim,
                          SPIM_IO_READ_PHASE,
                          pu8RxBuf,
                          u32RdSize,
                          psPhaseTable->u32DataPhase,
                          psPhaseTable->u32DataDTR,
                          psPhaseTable->u32RDQS);

    SPIM_SET_DTR_MODE(spim, u32DTREn);
}

/**
 * @brief      Enable the SPIM DLL (Delay Locked Loop) circuit.
 *
 * @param[in]  spim      Pointer to the SPIM instance.
 *
 * @retval     SPIM_OK           DLL locked successfully.
 * @retval     SPIM_ERR_TIMEOUT  DLL lock failed due to timeout.
 *
 * @note       After a system wake-up or reset, this function must be called again
 *             to reinitialize the DLL.
 */
int32_t SPIM_INIT_DLL(SPIM_T *spim)
{
    volatile int i32TimeoutCount = (int32_t)SPIM_TIMEOUT;
    uint32_t u32BusClkHz = CLK_GetSCLKFreq() / (SPIM_GET_CLOCK_DIVIDER(spim) * 2UL);
    uint32_t u32ClkNs = 1000000000UL / u32BusClkHz;
    const uint32_t u32DllLockUs  = 20;
    const uint32_t u32DllValidUs = 50;
    const uint32_t u32DllClkOnUs = 20;
    const uint32_t u32DllTrimUs  = 20;
    uint32_t u32DelayCycleCount = 0;

    // Convert to cycles
    uint32_t u32DLLLKNUM = SPIM_CEIL_DIV((u32DllLockUs * SPIM_TRIM_MARGIN), u32ClkNs);
    uint32_t u32DLLOVNUM = SPIM_CEIL_DIV((u32DllValidUs * SPIM_TRIM_MARGIN), u32ClkNs);
    uint32_t u32CLKONNUM = SPIM_CEIL_DIV((u32DllClkOnUs * SPIM_TRIM_MARGIN), u32ClkNs);
    uint32_t u32TRIMNUM = SPIM_CEIL_DIV((u32DllTrimUs * SPIM_TRIM_MARGIN), u32ClkNs);

    uint32_t u32Div = (SPIM_GET_CLOCK_DIVIDER(spim) * 2UL);
    uint32_t u32FreqMHz = (u32BusClkHz / 1000000UL);
    uint32_t u32FastEn = (u32FreqMHz <= 100UL) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;

    uint32_t u32DllDivCode = (u32Div <= 1UL) ? 0UL : (u32Div == 2UL) ? 1UL : (u32Div == 4UL) ? 2UL : 3UL;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    // DLL timing setup
    SPIM_SET_DLLLOCK_NUM(spim, u32DLLLKNUM);
    SPIM_SET_DLLOV_NUM(spim, u32DLLOVNUM);
    SPIM_SET_DLLCLKON_NUM(spim, u32CLKONNUM);
    SPIM_SET_DLLTRIM_NUM(spim, u32TRIMNUM);

#if (SPIM_TRIM_HYPERDLL == 1)

    SPIM_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF();
#endif

    /* Set SPIM DLL clock divider */
    SPIM_SET_DLLDIV(spim, u32DllDivCode);

    SPIM_SET_DLLFAST(spim, u32FastEn);

    /* SPIM starts to send DLL reference clock to DLL circuit
       that the frequency is the same as the SPIM output bus clock. */
    /* Enable SPIM DLL output */
    SPIM_ENABLE_DLLOLDO(spim, SPIM_OP_ENABLE);

#if (SPIM_TRIM_HYPERDLL == 1)
    SPIM_SET_AUTO_TRIM_DLL(spim, SPIM_OP_ENABLE);

    SPIM_SET_INTERNAL_RWDS(spim, SPIM_OP_ENABLE);
#endif

    /* User asserts this control register to 0x1,
       the DLL circuit begins searching for lock with DLL reference clock. */
    /* Assert SPIM DLL reset */
    SPIM_ENABLE_DLLOVRST(spim, SPIM_OP_ENABLE);

    /* Polling the DLL status register DLLCKON to 0x1,
       and the value 0x1 indicates that clock divider circuit inside DLL is enabled. */
    while (SPIM_GET_DLLOVRST(spim) == SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling the DLL status register DLLCKON to 0x1,
       and the value 0x1 indicates that clock divider circuit inside DLL is enabled. */
    while (SPIM_GET_DLLCLKON(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling the DLL status register DLLLOCK to 0x1,
       and the value 0x1 indicates that DLL circuit is in lock state */
    while (SPIM_GET_DLLLOCK(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling the DLL status register DLLREADY to 0x1,
       and the value 0x1 indicates that output of DLL circuit is ready. */
    while (SPIM_GET_DLLREADY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

#if (SPIM_TRIM_HYPERDLL == 1)
    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling the DLL status register DLLREADY to 0x1,
    and the value 0x1 indicates that output of DLL circuit is ready. */
    while (SPIM_GET_DLLATRDY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    /* wait for auto trim setting */
    u32DelayCycleCount = ((u32BusClkHz / 1000UL) * 3UL);  // 3ms = 3 * (Hz / 1000)

    for (i32TimeoutCount = 0; i32TimeoutCount < (int32_t)u32DelayCycleCount; i32TimeoutCount++) {}

    SPIM_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF();

    SPIM_SET_INTERNAL_RWDS(spim, SPIM_OP_DISABLE);
#endif

    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }

    return SPIM_OK;
}

/**
 * @brief      Set the DLL delay number and update the DLL_DNUM register.
 *
 * @param[in]  spim        Pointer to the SPIM instance.
 * @param[in]  u32DelayNum DLL delay number to set.
 *
 * @retval     SPIM_OK            Operation completed successfully.
 * @retval     SPIM_ERR_TIMEOUT   Operation aborted due to timeout.
 *
 * @note       Ensure the SPIM clock divider is configured before calling this function,
 *             as the delay depends on the SPIM output bus frequency.
 * @note       Returns SPIM_ERR_TIMEOUT if the Hyper chip does not respond in time.
 */
int32_t SPIM_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum)
{
    volatile int i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    if (SPIM_GET_DLLOLDO(spim) != SPIM_OP_ENABLE)
    {
        (void)SPIM_INIT_DLL(spim);
    }

    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling DLL status register DLL_REF to 1
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_GET_DLLREF(spim) != SPIM_OP_DISABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    /* Set this valid delay number to control register DLL_DNUM. */
    SPIM_SET_DLLDLY_NUM(spim, u32DelayNum);

    i32TimeoutCount = (int32_t)SPIM_TIMEOUT;

    /* Polling DLL status register DLL_REF to 1
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_GET_DLLREF(spim) != SPIM_OP_DISABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            break;
        }
    }

    return SPIM_OK;
}

/** @} end of group SPIM_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_Driver */
/** @} end of group Standard_Driver */
