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


#define ENABLE_DEBUG    0

#if ENABLE_DEBUG
    #define SPIM_DBGMSG   printf
#else
    #define SPIM_DBGMSG(...)   do { } while (0)      /* disable debug */
#endif

#define SPIM_TRIM_HYPERDLL                (1)

#define SPIM_ALTCTL0_DLL0TMEN_Pos         (8)
#define SPIM_ALTCTL0_DLL0TMEN_Msk         (0x1ul << SPIM_ALTCTL0_DLL0TMEN_Pos)

#define SPIM_DLL0ATCTL0_TUDOFF_Pos        (9)
#define SPIM_DLL0ATCTL0_TUDOFF_Msk        (0x1ul << SPIM_DLL0ATCTL0_TUDOFF_Pos)

#define SPIM_CTL1_CACHEOFF_Pos            (1)
#define SPIM_CTL1_CACHEOFF_Msk            (0x1ul << SPIM_CTL1_CACHEOFF_Pos)

#define SPIM_ENABLE_SYSDLL0TMEN() \
    do{ \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00)) | SPIM_ALTCTL0_DLL0TMEN_Msk);\
        outpw((SYS_BASE + 0xE00), u32Value);\
    }while(0)

#define SPIM_DISABLE_SYSDLL0TMEN() \
    do{ \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xE00)) & ~SPIM_ALTCTL0_DLL0TMEN_Msk);\
        outpw((SYS_BASE + 0xE00), u32Value);\
    }while(0)

#define SPIM_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                \
    do{ \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84)) & ~SPIM_DLL0ATCTL0_TUDOFF_Msk);\
        outpw((SYS_BASE + 0xF84), u32Value);\
    }while(0)

#define SPIM_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF()                                \
    do{ \
        uint32_t u32Value = ((inpw(SYS_BASE + 0xF84)) | SPIM_DLL0ATCTL0_TUDOFF_Msk);\
        outpw((SYS_BASE + 0xF84), u32Value);\
    }while(0)

#define SPIM_DISABLE_CACHE(spim) (spim->CTL1 |= SPIM_CTL1_CACHEOFF_Msk)

//------------------------------------------------------------------------------
static volatile uint8_t  g_Supported_List[] =
{
    MFGID_WINBOND,
    MFGID_MXIC,
    MFGID_EON,
    MFGID_ISSI,
    MFGID_SPANSION,
    MFGID_MICRON,
};
static volatile uint8_t g_au8IDBuf[3] = {0};

static void _SPIM_SwitchNBitOutput(SPIM_T *spim, uint32_t u32NBit);
static void _SPIM_SwitchNBitInput(SPIM_T *spim, uint32_t u32NBit);
static int32_t _SPIM_WriteData(SPIM_T *spim, uint8_t pu8TxBuf[], uint32_t u32NTx, uint32_t u32NBit);
static int32_t _SPIM_ReadData(SPIM_T *spim, uint8_t pu8RxBuf[], uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_WriteStatusRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit);
static void _SPIM_ReadStatusRegister2(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_WriteStatusRegister2(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit);
static void _SPIM_ReadStatusRegister3(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit);
static void _SPIM_ReadSecurityRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit);
static int _SPIM_IsWriteDone(SPIM_T *spim, uint32_t u32NBit);
static int _SPIM_WaitWriteDone(SPIM_T *spim, uint32_t u32NBit);
static int32_t _SPIM_WaitOpDone(SPIM_T *spim, uint32_t u32IsSync);
static void _SPIM_EnableSpansionQuadMode(SPIM_T *spim, int isEn);
static void _SPIM_EonSetQpiMode(SPIM_T *spim, int isEn);
static void _SPIM_SPANSION4BytesEnable(SPIM_T *spim, int isEn, uint32_t u32NBit);
static void _SPIM_WriteInPageDataByIo(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t wrCmd,
                                      uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, int isSync);
static int32_t _SPIM_WriteInPageDataByPageWrite(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx,
                                                uint8_t pu8TxBuf[], uint32_t wrCmd, int isSync);
static void _SPIM_ClearContReadPhase(SPIM_T *spim, uint32_t u32OPMode);

//------------------------------------------------------------------------------
/**
 * @brief    Switch to the specified N-bit mode
 * @param    spim    SPIM port
 * @param    u32NBit  1, 2, 4, or 8 bits
 *                   - \ref SPIM_BITMODE_1
 *                   - \ref SPIM_BITMODE_2
 *                   - \ref SPIM_BITMODE_4
 *                   - \ref SPIM_BITMODE_8
 * @return   None
 */
static void _SPIM_SwitchNBitOutput(SPIM_T *spim, uint32_t u32NBit)
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
 * @brief    Switch to the specified N-bit input mode
 * @param    spim      SPIM port
 * @param    u32NBit   1, 2, 4, or 8 bits
 *                   - \ref SPIM_BITMODE_1
 *                   - \ref SPIM_BITMODE_2
 *                   - \ref SPIM_BITMODE_4
 *                   - \ref SPIM_BITMODE_8
 * @return   None
 */
static void _SPIM_SwitchNBitInput(SPIM_T *spim, uint32_t u32NBit)
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
 * @brief      Set SPIM clock divider to 16 and RX CLK Delay Number to 0.
 *             This function is used for SPI Flash read/write operation.
 * @param      spim        SPIM port
 * @param      u32Restore  Enable or disable restore SPIM clock divider and
 *                         RX CLK Delay Number.
 *                         - \ref SPIM_OP_ENABLE
 *                         - \ref SPIM_OP_DISABLE
 * @return     None.
 */
static void SPIM_SetupConfigRegDiv(SPIM_T *spim, uint32_t u32Restore)
{
    static uint32_t u32DivSave, u32RxClkDlySave;

    if (u32Restore)
    {
        SPIM_SET_CLOCK_DIVIDER(spim, u32DivSave);
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u32RxClkDlySave);
    }
    else
    {
        u32DivSave = SPIM_GET_CLOCK_DIVIDER(spim);
        u32RxClkDlySave = SPIM_GET_RXCLKDLY_RDDLYSEL(spim);

        SPIM_SET_CLOCK_DIVIDER(spim, 16);
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, 0);
    }
}

/**
 * @brief      Write data to SPI slave.
 * @param      spim
 * @param      pu8TxBuf    Transmit buffer.
 * @param      u32NTx      Number of bytes to transmit.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     SPIM_OK             SPIM write done.
 *             SPIM_ERR_TIMEOUT    SPIM operation abort due to timeout error.
 */
static int32_t _SPIM_WriteData(SPIM_T *spim, uint8_t pu8TxBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    /* Write data to TX FIFO */
    uint32_t u32BufIdx = 0UL;   /* Transmit buffer index */
    uint32_t u32i;              /* Loop index */
    uint32_t u32Tmp;            /* Temporary variable */
    uint32_t u32ChunkSize;      /* Data chunk size */
    uint32_t u32BurstSize;      /* Burst data number */

    /* Switch between N-bit output mode */
    _SPIM_SwitchNBitOutput(spim, u32NBit);

    while (u32NTx)
    {
        /* Calculate the number of data to be transferred in one burst */
        u32ChunkSize = (u32NTx >= 16) ? 4UL :
                       (u32NTx >= 12) ? 3UL :
                       (u32NTx >= 8) ? 2UL :
                       (u32NTx >= 4) ? 1UL :
                       0UL;

        if (u32ChunkSize)
        {
            /* Transfer data in burst mode */
            u32i = u32ChunkSize;

            while (u32i)
            {
                memcpy(&u32Tmp, &pu8TxBuf[u32BufIdx], 4UL);
                u32i--;
                spim->TX[u32i] = u32Tmp;
                u32BufIdx += 4UL;
                u32NTx -= 4UL;
            }

            u32BurstSize = u32ChunkSize;
            u32ChunkSize = 4UL;
        }
        else
        {
            u32ChunkSize = u32NTx;
            /* Transfer data in single mode */
            memcpy(&u32Tmp, &pu8TxBuf[u32BufIdx], u32NTx);
            u32BurstSize = 1UL;
            u32NTx = 0UL;
            spim->TX[0] = u32Tmp;
        }

        /* Switch to Normal mode */
        SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
        /* Set data width */
        SPIM_SET_DATA_WIDTH(spim, (u32ChunkSize * 8UL));
        /* Set burst data number */
        SPIM_SET_BURST_DATA(spim, u32BurstSize);

        /* Wait until transfer complete */
        if (_SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    return SPIM_OK;
}

/**
 * @brief      Read data from SPI slave.
 * @param      spim
 * @param      pu8RxBuf    Receive buffer.
 * @param      u32NRx      Size of receive buffer in bytes.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     SPIM_OK          SPIM write done.
 *             SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 */
static int32_t _SPIM_ReadData(SPIM_T *spim, uint8_t pu8RxBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /*
     * Read data in burst mode to improve performance.
     */
    uint32_t u32BufIdx = 0UL;       /* Buffer index */
    uint32_t u32Tmp = 0;            /* Temporary variable for storing received data */
    uint32_t u32ChunkSize = 0;      /* Number of data in one burst */
    uint32_t u32TmpChunkSize = 0;   /* Temporary value for chunk_size */
    uint32_t u32BurstSize = 0;      /* Number of data in one burst */

    /* Configure SPIM to use N-bit input */
    _SPIM_SwitchNBitInput(spim, u32NBit);

    while (u32NRx)
    {
        /* Determine the number of data to be read in one burst */
        u32ChunkSize = (u32NRx >= 16) ? 4UL : /* At least 16 bytes */
                       (u32NRx >= 12) ? 3UL : /* 12 <= N < 16 */
                       (u32NRx >= 8) ? 2UL : /* 8 <= N < 12 */
                       (u32NRx >= 4) ? 1UL : /* 4 <= N < 8 */
                       0UL; /* N < 4 */

        u32TmpChunkSize = u32ChunkSize;

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
            u32ChunkSize = u32NRx;
            u32BurstSize = 1UL;
        }

        /* Configure SPIM to use Normal mode, N-bit data width and burst size */
        SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
        SPIM_SET_DATA_WIDTH(spim, (u32ChunkSize * 8UL));
        SPIM_SET_BURST_DATA(spim, u32BurstSize);

        /* Wait until transfer complete */
        if (_SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
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
                memcpy(&pu8RxBuf[u32BufIdx], &u32Tmp, 4UL);
                u32ChunkSize--;
                u32BufIdx += 4UL;
                u32NRx -= 4UL;
            }
        }
        else
        {
            /* Read 1 data */
            u32Tmp = spim->RX[0];
            memcpy(&pu8RxBuf[u32BufIdx], &u32Tmp, u32NRx);
            u32BufIdx += u32NRx;
            u32NRx = 0UL;
        }
    }

    return SPIM_OK;
}

/**
 * @brief      Issue Read Status Register #1 command.
 * @param      dataBuf     Receive buffer.
 * @param      u32NRx      Size of receive buffer.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     None.
 */
void SPIM_ReadStatusRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    uint8_t cmdBuf[2] = {OPCODE_RDSR, OPCODE_RDSR};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8 : 0);

    _SPIM_ReadData(spim, dataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue Write Status Register #1 command.
 * @param      dataBuf     Transmit buffer.
 * @param      u32NTx      Size of transmit buffer.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     None.
 */
static void _SPIM_WriteStatusRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    /* 1-byte Write Status Register #1 command + 1-byte data. */
    uint8_t cmdBuf[4] = {OPCODE_WRSR, 0x00U, 0x00U, 0x00U};

    cmdBuf[1] = (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? cmdBuf[0] : dataBuf[0];
    cmdBuf[3] = cmdBuf[2] = dataBuf[0];

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 4UL : 2UL, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue Read Status Register #2 command.
 * @param      spim
 * @param      dataBuf     Receive buffer.
 * @param      u32NRx      Size of receive buffer.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     None.
 */
static void _SPIM_ReadStatusRegister2(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    uint8_t cmdBuf[2] = {OPCODE_RDSR2, OPCODE_RDSR2};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    _SPIM_ReadData(spim, dataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Issue Winbond Write Status Register command. This command write both Status Register-1
 *             and Status Register-2.
 * @param      spim
 * @param      dataBuf     Transmit buffer.
 * @param      u32NTx      Size of transmit buffer.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     None.
 */
static void _SPIM_WriteStatusRegister2(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    uint8_t cmdBuf[6] = {OPCODE_WRSR, 0U, 0U};
    uint8_t u8DTREn = SPIM_GET_DTR_MODE(spim);

    cmdBuf[1] = (u8DTREn == SPIM_OP_ENABLE) ? dataBuf[0] : cmdBuf[0];
    cmdBuf[2] = (u8DTREn == SPIM_OP_ENABLE) ? dataBuf[0] : dataBuf[1];
    cmdBuf[3] = (u8DTREn == SPIM_OP_ENABLE) ? dataBuf[0] : dataBuf[1];
    cmdBuf[4] = dataBuf[1];
    cmdBuf[5] = dataBuf[1];

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (u8DTREn == SPIM_OP_ENABLE) ? 6UL : 3UL, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

#if 0  /* not used */
/**
  * @brief      Issue Write Status Register #3 command.
  * @param      dataBuf     Transmit buffer.
  * @param      u32NTx      Size of transmit buffer.
  * @param      u32NBit     N-bit transmit/receive.
  * @return     None.
  */
static void SPIM_WriteStatusRegister3(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    uint8_t cmdBuf[] = {OPCODE_WRSR3, 0x00U};    /* 1-byte Write Status Register #2 command + 1-byte data. */
    cmdBuf[1] = dataBuf[0];

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);        /* CS activated. */

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), u32NBit);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);        /* CS deactivated. */
}
#endif

/**
  * @brief      Issue Read Config Register #3 command.
  * @param      spim
  * @param      dataBuf     Receive buffer.
  * @param      u32NRx      Size of receive buffer.
  * @param      u32NBit     N-bit transmit/receive.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
  * @return     None.
  */
static void _SPIM_ReadStatusRegister3(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    uint8_t cmdBuf[2] = {OPCODE_RDSR3, OPCODE_RDSR3};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);
    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    _SPIM_ReadData(spim, dataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

#if 0  /* not used */
/**
  * @brief      Issue Write Security Register command.
  * @param      dataBuf     Transmit buffer.
  * @param      u32NTx      Size of transmit buffer.
  * @param      u32NBit     N-bit transmit/receive.
  * @return     None.
  */
static void SPIM_WriteSecurityRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    /* 1-byte Write Status Register #2 command + 1-byte data. */
    uint8_t cmdBuf[] = {OPCODE_WRSCUR, 0x00U};

    cmdBuf[1] = dataBuf[0];

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}
#endif

/**
 * @brief      Issue Read Security Register command.
 * @param      dataBuf     Receive buffer.
 * @param      u32NRx      Size of receive buffer.
 * @param      u32NBit     N-bit transmit/receive.
 * @return     None.
 */
static void _SPIM_ReadSecurityRegister(SPIM_T *spim, uint8_t dataBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    uint8_t cmdBuf[2] = {OPCODE_RDSCUR, OPCODE_RDSCUR};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);

    _SPIM_ReadData(spim, dataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
  * @brief      Read Micron MT35X Octal SPI Flash Flag Status Register.
  * @param      spim
  * @param      dataBuf     Receive buffer.
  * @param      u32NRx      Receive Length.
  * @param      u32NBit     N-bit transmit/receive.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
  * @return     None.
  */
static void _SPIM_ReadMT35xFlagRegister(SPIM_T *spim, uint8_t dataBuf[],
                                        uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte Read Status Register #1 command. */
    uint8_t cmdBuf[2] = {OPCODE_RD_FLGREG, OPCODE_RD_FLGREG};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8 : 0);

    _SPIM_ReadData(spim, dataBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Check if Erase/Write is done.
 * @param      spim
 * @param      u32NBit N-bit transmit/receive.
 *                     - \ref SPIM_BITMODE_1
 *                     - \ref SPIM_BITMODE_2
 *                     - \ref SPIM_BITMODE_4
 *                     - \ref SPIM_BITMODE_8
 * @return     0: Not done. 1: Done.
 */
static int _SPIM_IsWriteDone(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t status[2] = {0};
    SPIM_ReadStatusRegister(spim, status, sizeof(status), u32NBit);
    return !(status[0] & SR_WIP);
}

/**
 * @brief      Wait until Erase/Write done.
 * @param      spim
 * @param      u32NBit N-bit transmit/receive.
 *                     - \ref SPIM_BITMODE_1
 *                     - \ref SPIM_BITMODE_2
 *                     - \ref SPIM_BITMODE_4
 *                     - \ref SPIM_BITMODE_8
 * @return     SPIM_OK          SPIM write done.
 *             SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 */
static int _SPIM_WaitWriteDone(SPIM_T *spim, uint32_t u32NBit)
{
    volatile uint32_t count;
    int i32Ret = SPIM_ERR_FAIL;

    for (count = 0UL; count < SystemCoreClock / 1000UL; count++)
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
 * @brief      Wait SPIM operation to complete and check busy status.
 * @param      spim        SPIM port
 * @param      u32IsSync   Wait Busy Status
 * @retval     SPIM_OK     SPIM operation OK.
 * @retval     SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 */
static int32_t _SPIM_WaitOpDone(SPIM_T *spim, uint32_t u32IsSync)
{
    volatile int32_t i32TimeOutCount = SPIM_TIMEOUT;

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
 * @brief      Issue Write Enable/disable command.
 * @param      spim
 * @param      isEn    Enable/disable.
 * @param      u32NBit N-bit transmit/receive.
 *                     - \ref SPIM_BITMODE_1
 *                     - \ref SPIM_BITMODE_2
 *                     - \ref SPIM_BITMODE_4
 *                     - \ref SPIM_BITMODE_8
 * @return     None.
 */
void SPIM_SetWriteEnable(SPIM_T *spim, int isEn, uint32_t u32NBit)
{
    /* 1-byte Write Enable command. */
    uint8_t cmdBuf[2] = {0U};
    cmdBuf[0] = cmdBuf[1] = (isEn ? OPCODE_WREN : OPCODE_WRDI);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated.   */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/**
  * @brief  Set Wrap Around Enable/disable.
  * @param  spim
  * @param  isEn    Wrap Mode.
  *                 - \ref SPIM_OP_ENABLE  : Enable
  *                 - \ref SPIM_OP_DISABLE : disable
  * @return SPIM_OK          SPIM operation OK.
  *         SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
  */
int32_t SPIM_SetWrapAroundEnable(SPIM_T *spim, uint32_t u32IsEn)
{
    uint32_t u32CmdBuf[2] = {0x00000000, 0x11011101};

    SPIM_SetQuadEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    spim->TX[0] = (u32IsEn == SPIM_OP_ENABLE) ?
                  (u32CmdBuf[0] | (1 << 29)) :
                  (u32CmdBuf[0] | (1 << 28));
    spim->TX[1] = 0x11011101;

    _SPIM_SwitchNBitOutput(spim, SPIM_BITMODE_4);

    /* Switch to Normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);

    SPIM_SET_DATA_WIDTH(spim, 32UL);
    SPIM_SET_BURST_DATA(spim, 2UL);

    if (_SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
    {
        return SPIM_ERR_TIMEOUT;
    }

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetQuadEnable(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    return SPIM_OK;
}

/**
  * @brief Set SPI flash exit continue read mode.
  * @param spim
  */
static int32_t _SPIM_SetContReadDisable(SPIM_T *spim)
{
    SPIM_SetQuadEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    spim->TX[0] = 0xFFFFFFFF;

    /* Quad output mode */
    _SPIM_SwitchNBitOutput(spim, SPIM_BITMODE_4);

    /* Switch to normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
    SPIM_SET_DATA_WIDTH(spim, 32UL);
    SPIM_SET_BURST_DATA(spim, 1UL);

    if (_SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
    {
        return SPIM_ERR_TIMEOUT;
    }

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetQuadEnable(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    _SPIM_ClearContReadPhase(spim, SPIM_CTL0_OPMODE_DIRECTMAP);

    return SPIM_OK;
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief      Get SPIM serial clock.
 * @param      spim
 * @return     SPI serial clock.
 * @details    This function calculates the serial clock of SPI in Hz.
 */
uint32_t SPIM_GetSClkFreq(SPIM_T *spim)
{
    uint32_t clkDiv = SPIM_GET_CLOCK_DIVIDER(spim);

    return clkDiv ? SystemCoreClock / (clkDiv * 2U) : SystemCoreClock;
}

/**
 * @brief      Reset SPIM device.
 * @param      spim
 * @param      u32NBit N-bit transfer.
 *                     - \ref SPIM_BITMODE_1
 *                     - \ref SPIM_BITMODE_2
 *                     - \ref SPIM_BITMODE_4
 *                     - \ref SPIM_BITMODE_8
 * @details    Send Reset Enable and Reset command to SPIM device to reset it.
 */
static void _SPIM_ResetDevice(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t cmdBuf[2] = {0UL};

    /* Reset Enable */
    cmdBuf[0] = cmdBuf[1] = OPCODE_RSTEN;
    /* CS activated.    */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    /* Reset */
    cmdBuf[0] = cmdBuf[1] = OPCODE_RST;
    /* CS activated.    */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Initialize SPIM flash.
 * @param      spim
 * @param      clrWP           Clear Write Protect or not.
 * @return     SPIM_OK         Success.
 *             SPIM_ERR_FAIL   Unrecognized manufacture ID or failed on reading manufacture ID.
 */
int32_t SPIM_InitFlash(SPIM_T *spim, int clrWP)
{
    uint8_t idBuf[3];
    uint32_t u32i = 0;
    int32_t ret = SPIM_ERR_FAIL;

    /* Enable SPI Flash Mode */
    SPIM_SET_FLASH_MODE(spim);

    /* Enable DLL */
    SPIM_INIT_DLL(spim);

    /* Workaround */
    SPIM_DISABLE_CACHE(spim);

    SPIM_SET_SS_ACTLVL(spim, SPIM_OP_DISABLE);

    /*
     * Because not sure in SPI or QPI mode, do QPI reset and then SPI reset.
     */
    /* Quad Mode Reset */
    _SPIM_ResetDevice(spim, SPIM_BITMODE_4);

    /* Single Mode Reset */
    _SPIM_ResetDevice(spim, SPIM_BITMODE_1);

    if (clrWP)
    {
        uint8_t dataBuf[] = {0x00U};

        /* Clear Block Protect. */
        SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, SPIM_BITMODE_1);

        _SPIM_WriteStatusRegister(spim, dataBuf, sizeof(dataBuf), SPIM_BITMODE_1);

        _SPIM_WaitWriteDone(spim, SPIM_BITMODE_1);
    }

    SPIM_ReadJedecId(spim, idBuf, sizeof(idBuf), SPIM_BITMODE_1);
    memcpy((uint8_t *)g_au8IDBuf, idBuf, sizeof(idBuf));

    /* printf("ID: 0x%x, 0x%x, px%x\n", idBuf[0], idBuf[1], idBuf[2]); */

    for (u32i = 0UL; u32i < sizeof(g_Supported_List) / sizeof(g_Supported_List[0]); u32i++)
    {
        if (idBuf[0] == g_Supported_List[u32i])
        {
            ret = SPIM_OK;
            break;
        }
    }

    if (ret != 0)
    {
        SPIM_DBGMSG("Flash initialize failed!! 0x%x\n", idBuf[0]);
    }

    return ret;
}

/**
 * @brief      Issue JEDEC ID command. Read manufacturer and product ID.
 * @param      spim       SPIM module.
 * @param      pu8IdBuf   ID buffer. It's an array of size 3. pu8IdBuf[0] stores
 *                        manufacturer ID, pu8IdBuf[1] stores product ID, and
 *                        pu8IdBuf[2] stores revision number.
 * @param      u32NRx     Size of ID buffer. It should be 3.
 * @param      u32NBit    N-bit transmit/receive. It should be
 *                        - \ref SPIM_BITMODE_1
 *                        - \ref SPIM_BITMODE_2
 *                        - \ref SPIM_BITMODE_4
 *                        - \ref SPIM_BITMODE_8
 * @return     none
 * @details    This function is used to read the JEDEC ID of flash device to
 *             identify the device.
 */
void SPIM_ReadJedecId(SPIM_T *spim, uint8_t *pu8IdBuf, uint32_t u32NRx, uint32_t u32NBit)
{
    /* 1-byte JEDEC ID command. */
    uint8_t cmdBuf[2] = {OPCODE_RDID, OPCODE_RDID};

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);
    SPIM_IO_SendDummyByPhase(spim, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 8UL : 0UL);
    _SPIM_ReadData(spim, pu8IdBuf, u32NRx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetupConfigRegDiv(spim, SPIM_OP_ENABLE);
}

/** @cond HIDDEN_SYMBOLS */

static void _SPIM_EnableSpansionQuadMode(SPIM_T *spim, int isEn)
{
    uint8_t cmdBuf[3];
    uint8_t dataBuf[1], status1;
    volatile int32_t i32Delay = 10000;

    /* Read Status Register-1 */
    cmdBuf[0] = OPCODE_RDSR;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

    _SPIM_ReadData(spim, dataBuf, sizeof(dataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    /* SPIM_DBGMSG("SR1 = 0x%x\n", dataBuf[0]); */

    status1 = dataBuf[0];

    /* Read Configuration Register-2 */
    cmdBuf[0] = OPCODE_RDSR2;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

    _SPIM_ReadData(spim, dataBuf, sizeof(dataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    /* SPIM_DBGMSG("CR1 = 0x%x\n", dataBuf[0]); */

    SPIM_SetWriteEnable(spim, 1, 1UL);

    /* Write register */
    cmdBuf[0] = OPCODE_WRSR;
    cmdBuf[1] = status1;

    if (isEn)
    {
        /* set QUAD */
        cmdBuf[2] = dataBuf[0] | 0x2U;
    }
    else
    {
        /* clear QUAD */
        cmdBuf[2] = dataBuf[0] & ~0x2U;
    }

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_SetWriteEnable(spim, SPIM_OP_DISABLE, SPIM_BITMODE_1);

    /* Read Configuration Register-2 */
    cmdBuf[0] = OPCODE_RDSR2;

    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

    _SPIM_ReadData(spim, dataBuf, sizeof(dataBuf), SPIM_BITMODE_1);

    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    /* SPIM_DBGMSG("CR1 = 0x%x\n", dataBuf[0]); */
    while (--i32Delay >= 0)
    {
        __NOP();
    }
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief  Set Quad Enable/disable.
 * @param  spim
 * @param  isEn    Quad Mode.
 *                 - \ref SPIM_OP_ENABLE
 *                 - \ref SPIM_OP_DISABLE
 * @param  u32NBit N-bit transmit/receive.
 *                 - \ref SPIM_BITMODE_1
 *                 - \ref SPIM_BITMODE_2
 *                 - \ref SPIM_BITMODE_4
 *                 - \ref SPIM_BITMODE_8
 * @return None.
 */
void SPIM_SetQuadEnable(SPIM_T *spim, int isEn, uint32_t u32NBit)
{
    uint8_t idBuf[3];
    uint8_t dataBuf[2];

    SPIM_ReadJedecId(spim, idBuf, sizeof(idBuf), u32NBit);

    SPIM_DBGMSG("SPIM_SetQuadEnable - Flash ID is 0x%x\n", idBuf[0]);

    switch (idBuf[0])
    {
        /* Winbond SPI flash */
        case MFGID_WINBOND:
            SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
            _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
            SPIM_DBGMSG("Status Register: 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);

            if (isEn)
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
            _SPIM_WaitWriteDone(spim, u32NBit);

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

            dataBuf[0] = isEn ? SR_QE : 0U;

            _SPIM_WriteStatusRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);

            _SPIM_WaitWriteDone(spim, u32NBit);
            break;

        case MFGID_SPANSION:
            _SPIM_EnableSpansionQuadMode(spim, isEn);
            break;

        default:
            break;
    }
}

/**
 * @brief      Enter/exit QPI mode.
 * @param      spim
 * @param      isEn        Enable/disable.
 * @return     None.
 */
static void _SPIM_EonSetQpiMode(SPIM_T *spim, int isEn)
{
    /* 1-byte command.  */
    uint8_t cmdBuf[1];
    uint8_t status[1];

    SPIM_ReadStatusRegister(spim, status, sizeof(status), SPIM_BITMODE_1);
    SPIM_DBGMSG("Status: 0x%x\n", status[0]);

    if (isEn)
    {
        /* Assume in SPI mode. */
        cmdBuf[0] = OPCODE_ENQPI;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

        /* CS deactivated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    }
    else
    {
        /* Assume in QPI mode. */
        cmdBuf[0] = OPCODE_EXQPI;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_4);

        /* CS deactivated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
    }

    SPIM_ReadStatusRegister(spim, status, sizeof(status), 1UL);
    SPIM_DBGMSG("Status: 0x%x\n", status[0]);
}

static void _SPIM_SPANSION4BytesEnable(SPIM_T *spim, int isEn, uint32_t u32NBit)
{
    uint8_t cmdBuf[2];
    uint8_t dataBuf[1];

    cmdBuf[0] = OPCODE_BRRD;

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, 1UL, u32NBit);

    _SPIM_ReadData(spim, dataBuf, 1UL, SPIM_BITMODE_1);

    /* CS deactivated.  */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    SPIM_DBGMSG("Bank Address register= 0x%x\n", dataBuf[0]);

    cmdBuf[0] = OPCODE_BRWR;

    if (isEn)
    {
        /* set EXTADD */
        cmdBuf[1] = dataBuf[0] | 0x80U;
    }
    else
    {
        /* clear EXTADD */
        cmdBuf[1] = dataBuf[0] & ~0x80U;
    }

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, sizeof(cmdBuf), SPIM_BITMODE_1);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);
}

/** @cond HIDDEN_SYMBOLS */
/**
  * @brief  Query 4-byte address mode enabled or not.
  * @param[in]  spim
  * @param[in]  u32NBit N-bit transmit/receive.
  *                     - \ref SPIM_BITMODE_1
  *                     - \ref SPIM_BITMODE_2
  *                     - \ref SPIM_BITMODE_4
  *                     - \ref SPIM_BITMODE_8
  * @return     0: 4-byte address mode disabled.
  *             1: 4-byte address mode enabled.
  * @details To check 4-byte address mode enabled or not for different flash vendor.
  */
int32_t SPIM_Is4ByteModeEnable(SPIM_T *spim, uint32_t u32NBit)
{
    int32_t isEn = 0;
    uint8_t idBuf[3];
    uint8_t dataBuf[2];

    SPIM_ReadJedecId(spim, idBuf, sizeof(idBuf), u32NBit);

    switch (idBuf[0])
    {
        case MFGID_WINBOND:
            _SPIM_ReadStatusRegister3(spim, dataBuf, sizeof(dataBuf), u32NBit);
            /* Before MX25L6406E, bit 6 of Status Register-3 is reserved. */
            isEn = (idBuf[2] < 0x16U) ? !SPIM_OP_ENABLE : !!(dataBuf[0] & SR3_ADR);
            break;

        case MFGID_MXIC:
        case MFGID_EON:
            _SPIM_ReadSecurityRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);
            /* Before MT25QL512ABA, bit 4 of Security Register is reserved. */
            isEn = (idBuf[2] < 0x19U) ? !SPIM_OP_ENABLE : !!(dataBuf[0] & SCUR_4BYTE);
            break;

        case MFGID_ISSI:
            /* Before IS25LP016D, 4-byte address mode supported. */
            isEn = (idBuf[2] < 0x49U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_MICRON:
            /* Before MT35XL01G1AA, bit 6 of Flag Register is reserved. */
            _SPIM_ReadMT35xFlagRegister(spim, dataBuf, sizeof(dataBuf), u32NBit);
            isEn = !!(dataBuf[0] & SR3_ADR);
            break;

        default:
            break;
    }

    return isEn;
}

/** @endcond HIDDEN_SYMBOLS  */


/**
 * @brief  Enter/Exit 4-byte address mode.
 * @param  spim
 * @param  isEn    Enable/disable.
 *                 - \ref SPIM_OP_ENABLE
 *                 - \ref SPIM_OP_DISABLE
 * @param  u32NBit N-bit transmit/receive.
 *                 - \ref SPIM_BITMODE_1
 *                 - \ref SPIM_BITMODE_2
 *                 - \ref SPIM_BITMODE_4
 *                 - \ref SPIM_BITMODE_8
 * @return SPIM_OK         SPIM operation OK.
 *         SPIM_ERR_FAIL   SPIM operation Fail.
 */
int32_t SPIM_Enable_4Bytes_Mode(SPIM_T *spim, int isEn, uint32_t u32NBit)
{
    int isSupt = 0L, ret = SPIM_ERR_FAIL;
    uint8_t idBuf[3];
    /* 1-byte Enter/Exit 4-Byte Mode command. */
    uint8_t cmdBuf[2];
    volatile int32_t i32TimeOutCount = 0;

    SPIM_ReadJedecId(spim, idBuf, sizeof(idBuf), u32NBit);

    /* Based on Flash size, check if 4-byte address mode is supported. */
    switch (idBuf[0])
    {
        case MFGID_WINBOND:
            isSupt = (idBuf[2] < 0x16U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_MXIC:
        case MFGID_EON:
            isSupt = (idBuf[2] < 0x19U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_ISSI:
            isSupt = (idBuf[2] < 0x49U) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE;
            break;

        case MFGID_SPANSION:
            _SPIM_SPANSION4BytesEnable(spim, isEn, u32NBit);
            isSupt = SPIM_OP_ENABLE;
            ret = SPIM_OK;
            break;

        case MFGID_MICRON:
            //SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);
            isSupt = SPIM_OP_ENABLE;
            break;

        default:
            break;
    }

    if ((isSupt) && (idBuf[0] != MFGID_SPANSION))
    {
        cmdBuf[0] = cmdBuf[1] = isEn ? OPCODE_EN4B : OPCODE_EX4B;

        /* CS activated. */
        SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

        _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

        /* CS deactivated.  */
        SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

        /*
         * FIXME: Per test, 4BYTE Indicator bit doesn't set after EN4B, which
         * doesn't match spec(MX25L25635E), so skip the check below.
         */
        ret = SPIM_OK;

        if (idBuf[0] != MFGID_MXIC)
        {
            /*
             *  About over 100 instrucsions executed, just want to give
             *  a time-out about 1 seconds to avoid infinite loop
             */
            i32TimeOutCount = (SystemCoreClock) / 100;

            while ((--i32TimeOutCount >= 0) && (SPIM_Is4ByteModeEnable(spim, u32NBit) != isEn)) {}

            if (i32TimeOutCount <= 0)
                ret = SPIM_ERR_FAIL;
        }
    }

    return ret;
}

/**
 * @brief  Unlock Winbond flash chip which has security bit set
 * @param  spim
 * @param  u32NBit N-bit transmit/receive.
 *                 - \ref SPIM_BITMODE_1
 *                 - \ref SPIM_BITMODE_2
 *                 - \ref SPIM_BITMODE_4
 *                 - \ref SPIM_BITMODE_8
 * @return None.
 */
void SPIM_WinbondUnlock(SPIM_T *spim, uint32_t u32NBit)
{
    uint8_t idBuf[3];
    uint8_t dataBuf[4];

    SPIM_ReadJedecId(spim, idBuf, sizeof(idBuf), u32NBit);

    if ((idBuf[0] != MFGID_WINBOND) || (idBuf[1] != 0x40) || (idBuf[2] != 0x16))
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
    _SPIM_WaitWriteDone(spim, u32NBit);

    SPIM_ReadStatusRegister(spim, &dataBuf[0], 1UL, u32NBit);
    _SPIM_ReadStatusRegister2(spim, &dataBuf[1], 1UL, u32NBit);
    SPIM_DBGMSG("Status Register (after unlock): 0x%x - 0x%x\n", dataBuf[0], dataBuf[1]);
}

/**
 * @brief  Erase whole chip.
 * @param  spim
 * @param  u32NBit N-bit transmit/receive.
 *                 - \ref SPIM_BITMODE_1
 *                 - \ref SPIM_BITMODE_2
 *                 - \ref SPIM_BITMODE_4
 *                 - \ref SPIM_BITMODE_8
 * @param  isSync  Block or not.
 * @return None.
 */
void SPIM_ChipErase(SPIM_T *spim, uint32_t u32NBit, int isSync)
{
    /* 1-byte Chip Erase command. */
    uint8_t cmdBuf[2] = {OPCODE_CHIP_ERASE, OPCODE_CHIP_ERASE};

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (isSync)
    {
        _SPIM_WaitWriteDone(spim, u32NBit);
    }
}


/**
 * @brief  Erase one block.
 * @param  spim
 * @param  u32Addr     Block to erase which contains the u32Addr.
 * @param  u32Is4ByteAddr 4-byte u32Address or not.
 * @param  u8ErsCmd    Erase command.
 *                     - ref OPCODE_SE_4K
 *                     - ref OPCODE_BE_32K
 *                     - ref OPCODE_BE_64K
 * @param  u32NBit     N-bit transmit/receive.
 *                     - \ref SPIM_BITMODE_1
 *                     - \ref SPIM_BITMODE_2
 *                     - \ref SPIM_BITMODE_4
 *                     - \ref SPIM_BITMODE_8
 * @param  isSync      Block or not.
 * @return None.
 */
void SPIM_EraseBlock(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint8_t u8ErsCmd, uint32_t u32NBit, int isSync)
{
    uint8_t cmdBuf[6] = {0};
    uint32_t buf_idx = 0UL;

    cmdBuf[buf_idx++] = u8ErsCmd;

    if ((SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) &&
            (u32NBit == SPIM_BITMODE_8))
    {
        cmdBuf[buf_idx++] = u8ErsCmd;
        u32Is4ByteAddr = SPIM_OP_ENABLE;
    }

    if (u32Is4ByteAddr)
    {
        cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 24);
    }

    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 16);
    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 8);
    cmdBuf[buf_idx++] = (uint8_t)(u32Addr & 0xFFUL);

    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, u32NBit);
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBit);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_WriteData(spim, cmdBuf, buf_idx, u32NBit);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (isSync)
    {
        _SPIM_WaitWriteDone(spim, u32NBit);
    }
}


/** @cond HIDDEN_SYMBOLS */

/**
 * @brief      Write data in the same page by I/O mode.
 * @param      u32Addr        Start u32Address to write.
 * @param      u32Is4ByteAddr 4-byte u32Address or not.
 * @param      u32NTx         Number of bytes to write.
 * @param      pu8TxBuf       Transmit buffer.
 * @param      wrCmd          Write command.
 * @param      u32NBitCmd     N-bit transmit command.
 * @param      u32NBitAddr    N-bit transmit u32Address.
 * @param      u32NBitDat     N-bit transmit/receive data.
 * @param      isSync         Block or not.
 * @return     None.
 */
static void _SPIM_WriteInPageDataByIo(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t wrCmd,
                                      uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, int isSync)
{
    uint8_t cmdBuf[16] = {wrCmd, wrCmd};
    uint32_t buf_idx;

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, u32NBitCmd);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    /* Write out command. */
    _SPIM_WriteData(spim, cmdBuf, (SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE) ? 2UL : 1UL, u32NBitCmd);

    buf_idx = 0UL;

    if (u32Is4ByteAddr)
    {
        cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 24);
    }

    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 16);
    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 8);
    cmdBuf[buf_idx++] = (uint8_t) u32Addr;

    /* Write out u32Address. */
    _SPIM_WriteData(spim, cmdBuf, buf_idx, u32NBitAddr);

    /* Write out data. */
    _SPIM_WriteData(spim, pu8TxBuf, u32NTx, u32NBitDat);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (isSync)
    {
        _SPIM_WaitWriteDone(spim, u32NBitCmd);
    }
}

/**
 * @brief      Write data in the same page by Page Write mode.
 * @param      u32Addr     Start u32Address to write.
 * @param      u32Is4ByteAddr 4-byte u32Address or not.
 * @param      u32NTx      Number of bytes to write.
 * @param      pu8TxBuf    Transmit buffer.
 * @param      wrCmd       Write command.
 * @param      isSync      Block or not.
 * @retval     SPIM_OK          SPIM operation OK.
 * @retval     SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 * @note       This function sets g_SPIM_i32ErrCode to SPIM_TIMEOUT_ERR if waiting SPIM time-out.
 */
static int32_t _SPIM_WriteInPageDataByPageWrite(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx,
                                                uint8_t pu8TxBuf[], uint32_t wrCmd, int isSync)
{
    u32Is4ByteAddr = (SPIM_GET_PHDMAW_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim)));

    /* Write Enable. */
    SPIM_SetWriteEnable(spim, SPIM_OP_ENABLE, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim)));

    wrCmd = (((SPIM_GET_PHDMAW_CMD_DTR(spim) == SPIM_OP_ENABLE) &&
              (SPIM_GET_PHDMAW_CMD_WIDTH(spim) == PHASE_WIDTH_16)) ?
             ((wrCmd << 8) | wrCmd) :
             wrCmd);

    /* Switch to Page Write mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_PAGEWRITE);
    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, wrCmd);

    /* Enable/disable 4-Byte Address. */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    /* SRAM u32Address. */
    spim->SRAMADDR = (uint32_t) pu8TxBuf;
    /* Transfer length. */
    spim->DMACNT = u32NTx;
    /* Flash u32Address. */
    spim->FADDR = u32Addr;

    if (_SPIM_WaitOpDone(spim, SPIM_OP_ENABLE) != SPIM_OK)
    {
        return SPIM_ERR_TIMEOUT;
    }

    _SPIM_WaitWriteDone(spim, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAW_CMD_BITMODE(spim)));

    if (wrCmd == CMD_QUAD_PAGE_PROGRAM_EON)
    {
        /* Exit QPI mode. */
        _SPIM_EonSetQpiMode(spim, 0);
    }

    return SPIM_OK;
}

/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief      Write data to SPI Flash by sending commands manually (I/O mode).
  * @param      spim
  * @param[in]  u32Addr     Start u32Address to write.
  * @param[in]  u32Is4ByteAddr 4-byte u32Address or not.
  * @param[in]  u32NTx      Number of bytes to write.
  * @param[in]  pu8TxBuf    Transmit buffer.
  * @param[in]  wrCmd       Write command.
  * @param[in]  u32NBitCmd  N-bit transmit command.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
  * @param[in]  u32NBitAddr N-bit transmit u32Address.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
  * @param[in]  u32NBitDat  N-bit transmit/receive data.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
 * @return     None.
  * @details    This function is used to write data to SPI Flash in small chunks
  *             (max 256 bytes per chunk) by sending commands manually.
 */
void SPIM_IO_Write(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NTx, uint8_t pu8TxBuf[], uint8_t wrCmd,
                   uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat)
{
    /* Write out data to SPI Flash in small chunks (max 256 bytes per chunk) */
    /* number of bytes to write in this chunk */
    uint32_t toWr;
    /* index into tx buffer */
    uint32_t buf_idx = 0UL;
    /* Query 4-byte address mode enabled or not. */
    uint32_t u32TempIs4ByteAddr = SPIM_Is4ByteModeEnable(spim, u32NBitCmd);

    if (u32Is4ByteAddr != u32TempIs4ByteAddr)
    {
        SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));
    }

    /* Set SPIM 4-Bytes Address Mode */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    while (u32NTx)
    {
        toWr = (u32NTx < SPIM_FLH_PAGE_SIZE) ? u32NTx : SPIM_FLH_PAGE_SIZE;

        _SPIM_WriteInPageDataByIo(spim, u32Addr, u32Is4ByteAddr,
                                  toWr, &pu8TxBuf[buf_idx], wrCmd,
                                  u32NBitCmd, u32NBitAddr, u32NBitDat,
                                  SPIM_OP_ENABLE);
        u32Addr += toWr;
        u32NTx -= toWr;
        buf_idx += toWr;
    }

    if (u32Is4ByteAddr != u32TempIs4ByteAddr)
    {
        SPIM_Enable_4Bytes_Mode(spim, u32TempIs4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));
    }

    /* Set SPIM 4-Bytes Address Mode */
    SPIM_SET_4BYTE_ADDR(spim, u32TempIs4ByteAddr);
}

/**
 * @brief      Read data from SPI Flash by sending commands manually (I/O mode).
 * @param      spim
 * @param      u32Addr     Start u32Address to read.
 * @param      u32Is4ByteAddr 4-byte u32Address or not.
 * @param      u32NRx      Number of bytes to read.
 * @param      pu8RxBuf    Receive buffer.
 * @param      rdCmd       Read command.
 * @param      u32NBitCmd  N-bit transmit command.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
 * @param      u32NBitAddr N-bit transmit u32Address.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
 * @param      u32NBitDat  N-bit transmit/receive data.
  *                         - \ref SPIM_BITMODE_1
  *                         - \ref SPIM_BITMODE_2
  *                         - \ref SPIM_BITMODE_4
  *                         - \ref SPIM_BITMODE_8
 * @param      u32NDummy   Number of dummy clock following address.
 * @return     None.
 */
void SPIM_IO_Read(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[], uint8_t rdCmd,
                  uint32_t u32NBitCmd, uint32_t u32NBitAddr, uint32_t u32NBitDat, int u32NDummy)
{
    uint8_t cmdBuf[8] = {rdCmd, rdCmd};
    uint32_t buf_idx = 0UL;
    /* Query 4-byte address mode enabled or not. */
    uint32_t u32TempIs4ByteAddr = SPIM_Is4ByteModeEnable(spim, u32NBitCmd);

    if (u32Is4ByteAddr != u32TempIs4ByteAddr)
    {
        SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));
    }

    /* Set SPIM 4-Bytes Address Mode */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    /* Write out command. */
    _SPIM_WriteData(spim, cmdBuf, SPIM_GET_DTR_MODE(spim) == SPIM_OP_ENABLE ? 2UL : 1UL, u32NBitCmd);

    if (u32Is4ByteAddr)
    {
        cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 24);
    }

    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 16);
    cmdBuf[buf_idx++] = (uint8_t)(u32Addr >> 8);
    cmdBuf[buf_idx++] = (uint8_t) u32Addr;

    /* Add a byte for the address phase for dual or quad mode for read mode */
    buf_idx += ((u32NBitAddr == SPIM_BITMODE_2) || (u32NBitAddr == SPIM_BITMODE_4)) ? 1UL : 0UL;

    _SPIM_WriteData(spim, cmdBuf, buf_idx, u32NBitAddr);    /* Write out u32Address. */

    /* Same bit mode as above. */
    SPIM_IO_SendDummyByPhase(spim, u32NDummy);              /* Write out dummy bytes. */

    /* Read back data. */
    _SPIM_ReadData(spim, pu8RxBuf, u32NRx, u32NBitDat);

    /* CS deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    if (u32Is4ByteAddr != u32TempIs4ByteAddr)
    {
        SPIM_Enable_4Bytes_Mode(spim, u32TempIs4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));
    }

    /* Set SPIM 4-Bytes Address Mode */
    SPIM_SET_4BYTE_ADDR(spim, u32TempIs4ByteAddr);
}

/**
 * @brief      Write data to SPI Flash by Page Write mode.
 * @param      spim        SPIM port.
 * @param      u32Addr     Start address to write.
 * @param      u32Is4ByteAddr 4-byte address or not.
 * @param      u32NTx      Number of bytes to write.
 * @param      pu8TxBuf    Transmit buffer.
 * @param      wrCmd       Write command.
 *
 * @details    This function use PHDMAW to write data to SPI flash in page write mode.
 *             Before calling this function, must first call SPIM_DMADMM_InitPhase to set PHDMAW.
 *             If the width of PHDMAW is 16-bit and DTR mode is enabled, the write command will
 *             be sent twice to complete the write operation.
 */
void SPIM_DMA_Write(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr,
                    uint32_t u32NTx, uint8_t *pu8TxBuf, uint32_t wrCmd)
{
    /* number of bytes to write in this chunk */
    uint32_t toWr;
    /* index into tx buffer */
    uint32_t buf_idx = 0UL;

    while (u32NTx)
    {
        toWr = (u32NTx <= SPIM_FLH_PAGE_SIZE) ? u32NTx : SPIM_FLH_PAGE_SIZE;

        _SPIM_WriteInPageDataByPageWrite(spim, u32Addr, u32Is4ByteAddr, toWr, &pu8TxBuf[buf_idx], wrCmd, SPIM_OP_ENABLE);

        /* Advance indicator. */
        u32Addr += toWr;
        u32NTx -= toWr;
        buf_idx += toWr;
    }
}

/**
 * @brief      Read data from SPI Flash by Page Read mode.
 * @param      spim
 * @param      u32Addr     Start address to read.
 * @param      u32Is4ByteAddr 4-byte u32Address or not.
 * @param      u32NRx      Number of bytes to read.
 * @param      pu8RxBuf    Receive buffer.
 * @param      u32RdCmd    Read command.
 * @param      isSync      Block or not.
 * @return     SPIM_OK          SPIM operation OK.
 *             SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 *             if waiting SPIM time-out.
 * @note       Before calling this API, must first call SPIM_DMADMM_InitPhase to set PHDMAR.
 */
int32_t SPIM_DMA_Read(SPIM_T *spim, uint32_t u32Addr, uint32_t u32Is4ByteAddr, uint32_t u32NRx, uint8_t pu8RxBuf[],
                      uint32_t u32RdCmd, int isSync)
{
    u32Is4ByteAddr = (SPIM_GET_PHDMAR_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMAR_CMD_BITMODE(spim)));

    u32RdCmd = (((SPIM_GET_PHDMAR_CMD_DTR(spim) == SPIM_OP_ENABLE) &&
                 (SPIM_GET_PHDMAR_CMD_WIDTH(spim) == PHASE_WIDTH_16)) ?
                ((u32RdCmd << 8) | u32RdCmd) :
                u32RdCmd);

    /* Switch to Page Read mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_PAGEREAD);
    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, u32RdCmd);
    /* Enable/disable 4-Byte Address. */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    /* SRAM u32Address. */
    spim->SRAMADDR = (uint32_t) pu8RxBuf;
    /* Transfer length. */
    spim->DMACNT = u32NRx;
    /* Flash u32Address. */
    spim->FADDR = u32Addr;

    return _SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);
}

/**
 * @brief      Enter Direct Map mode.
 * @param      spim
 * @param      u32Is4ByteAddr  4-byte u32Address or not.
 * @param      u32RdCmd        Read command.
 * @param      u32IdleIntvl    Idle interval.
 * @return     None.
 */
void SPIM_EnterDirectMapMode(SPIM_T *spim, uint32_t u32Is4ByteAddr, uint32_t u32RdCmd, uint32_t u32IdleIntvl)
{
    u32Is4ByteAddr = (SPIM_GET_PHDMM_ADDR_WIDTH(spim) == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE;
    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(SPIM_GET_PHDMM_CMD_BITMODE(spim)));

    /* Enable/disable 4-byte u32Address. */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    u32RdCmd = (((SPIM_GET_PHDMM_CMD_WIDTH(spim) == PHASE_WIDTH_16) &&
                 (SPIM_GET_PHDMM_CMD_DTR(spim) == SPIM_OP_ENABLE)) ?
                ((u32RdCmd << 8) | u32RdCmd) :
                u32RdCmd);

    /* SPIM mode. */
    SPIM_SET_CMD_CODE(spim, u32RdCmd);
    /* Idle interval. */
    SPIM_SET_IDL_INTVL(spim, u32IdleIntvl);
    SPIM_CLR_DMM_TIMEOUT_STS(spim);
    /* Switch to Direct Map mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_DIRECTMAP);
}

/**
 * @brief      Exit Direct Map mode.
 * @param      spim
 * @details    If enabled Continue Read Mode, it will be disabled.
 *             If enabled Wrap Around mode, it will be disabled.
 */
void SPIM_ExitDirectMapMode(SPIM_T *spim)
{
    if (SPIM_GET_DMM_BWEN(spim) == SPIM_OP_ENABLE)
    {
        SPIM_SetWrapAroundEnable(spim, 0);
    }

    if (SPIM_GET_MODE_DATA(spim) == SPIM_DMM_ENABLE_CREN)
    {
        _SPIM_SetContReadDisable(spim);
    }
}

/**
 * @brief Convert I/O phase bit mode size.
 * @param u32Phase
 *          - \ref PHASE_NORMAL_MODE : 1 Bit Mode
 *          - \ref PHASE_DUAL_MODE   : 2 Bit Mode
 *          - \ref PHASE_QUAD_MODE   : 4 Bit Mode
 * @return uint32_t Phase Size
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
  * @brief   Swich DMA/DMM Phase Setting Register
  * @param   spim
  * @param   u32OPMode   SPI Function Operation Mode
  *                      - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA Write mode
  *                      - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
  *                      - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
  * @return  PHDMAW/PHDMAR/PHDMM Register Address
  */
static void *_SPIM_SwitchPhaseRegister(SPIM_T *spim, uint32_t u32OPMode)
{
    return ((u32OPMode == SPIM_CTL0_OPMODE_PAGEWRITE) ? (uint32_t *)&spim->PHDMAW :
            (u32OPMode == SPIM_CTL0_OPMODE_PAGEREAD) ? (uint32_t *)&spim->PHDMAR :
            (u32OPMode == SPIM_CTL0_OPMODE_DIRECTMAP) ? (uint32_t *)&spim->PHDMM :
            NULL);
}


/**
 * @brief Set DMA/DMM Command Phase
 * @param spim
 * @param u32OPMode SPI Function Operation Mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA Write mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
 *                  - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 * @param u32NBit   Command Bit Mode
 *                  - \ref PHASE_NORMAL_MODE : Send Command Use Standard Mode
 *                  - \ref PHASE_DUAL_MODE   : Send Command Use Dual Mode
 *                  - \ref PHASE_QUAD_MODE   : Send Command Use Quad Mode
 *                  - \ref PHASE_OCTAL_MODE  : Send Command Use Octal Mode
 * @param u32Width  Command Width
 *                  - \ref PHASE_WIDTH_8    : Set Command Width 8bit
 *                  - \ref PHASE_WIDTH_16   : Set Command Width 16bit
 *                  - \ref PHASE_WIDTH_24   : Set Command Width 24bit
 *                  - \ref PHASE_WIDTH_32   : Set Command Width 32bit
 * @param u32DTREn  Set Enable/Disable DTR(Double Transfer Rate) Mode 0 or 1
 */
static int32_t _SPIM_SetCMDPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32DTREn)
{
    uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == NULL)
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
 * @brief Set DMA/DMM Address Phase
 * @param spim
 * @param u32OPMode SPI Function Operation Mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA Write mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
 *                  - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 * @param u32NBit   Command Bit Mode
 *                  - \ref PHASE_NORMAL_MODE : Send Command Use Standard Mode
 *                  - \ref PHASE_DUAL_MODE   : Send Command Use Dual Mode
 *                  - \ref PHASE_QUAD_MODE   : Send Command Use Quad Mode
 *                  - \ref PHASE_OCTAL_MODE  : Send Command Use Octal Mode
 * @param u32Width  Command Width
 *                  - \ref PHASE_WIDTH_8    : Set Command Width 8bit
 *                  - \ref PHASE_WIDTH_16   : Set Command Width 16bit
 *                  - \ref PHASE_WIDTH_24   : Set Command Width 24bit
 *                  - \ref PHASE_WIDTH_32   : Set Command Width 32bit
 * @param u32DTREn  Set Enable/Disable DTR(Double Transfer Rate) Mode 0 or 1
 * @return     SPIM_OK          SPIM operation OK.
 *             SPIM_ERR_FAIL    SPIM operation Fail.
 */
static int32_t _SPIM_SetAddrPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32DTREn)
{
    uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == NULL)
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
    uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_READMODE_Pos);
}

/**
 * @brief Set DMA/DMM Continue Read Mode Phase
 * @param spim
 * @param u32OPMode SPI Function Operation Mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
 *                  - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 * @param u32NBit   Command Bit Mode
 *                  - \ref PHASE_NORMAL_MODE : Send Command Use Standard Mode
 *                  - \ref PHASE_DUAL_MODE   : Send Command Use Dual Mode
 *                  - \ref PHASE_QUAD_MODE   : Send Command Use Quad Mode
 *                  - \ref PHASE_OCTAL_MODE  : Send Command Use Octal Mode
 * @param u32Width  Command Width
 *                  - \ref PHASE_WIDTH_8    : Set Command Width 8bit
 *                  - \ref PHASE_WIDTH_16   : Set Command Width 16bit
 *                  - \ref PHASE_WIDTH_24   : Set Command Width 24bit
 *                  - \ref PHASE_WIDTH_32   : Set Command Width 32bit
 * @param u32ContEn Enable Continue Read
 *                  - \ref PHASE_ENABLE_CONT_READ   : Enable Continue Read
 *                  - \ref PHASE_DISABLE_CONT_READM : Disable Continue Read
 * @param u32DTREn  Set Enable/Disable DTR(Double Transfer Rate) Mode 0 or 1
 * @return     SPIM_OK          SPIM operation OK.
 *             SPIM_ERR_FAIL    SPIM operation Fail.
 */
static int32_t _SPIM_SetContReadPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit, uint32_t u32Width, uint32_t u32ContEn, uint32_t u32DTREn)
{
    uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == NULL)
    {
        return SPIM_ERR_FAIL;
    }

    /* clear Read Mode phase setting. */
    *pu32PhaseReg &= ~(0xFFUL << PHASE_CLR_READMODE_Pos);

    *pu32PhaseReg |= ((u32Width << SPIM_PHDMAR_DW_MODE_Pos) |
                      (u32NBit << SPIM_PHDMAR_BM_MODE_Pos) |
                      (u32DTREn << SPIM_PHDMAR_DTR_MODE_Pos));

    if ((u32ContEn == PHASE_ENABLE_CONT_READ) /*&&
            (u32OPMode == SPIM_CTL0_OPMODE_DIRECTMAP)*/)
    {
        SPIM_ENABLE_DMM_CREN(spim);

        /* Enable Contiue Read Mode */
        SPIM_SET_MODE_DATA(spim, SPIM_DMM_ENABLE_CREN);
    }
    else
    {
        SPIM_CLEAR_MODE_DATA(spim);
    }

    return SPIM_OK;
}

/**
 * @brief Set DMA/DMM Data Phase
 * @param spim
 * @param u32OPMode SPI Function Operation Mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA Write mode
 *                  - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
 *                  - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 * @param u32NBit   Command Bit Mode
 *                  - \ref PHASE_NORMAL_MODE : Send Command Use Standard Mode
 *                  - \ref PHASE_DUAL_MODE   : Send Command Use Dual Mode
 *                  - \ref PHASE_QUAD_MODE   : Send Command Use Quad Mode
 *                  - \ref PHASE_OCTAL_MODE  : Send Command Use Octal Mode
 * @param u32ByteOrder Set Received Data Byte Order of Received Data Phase for Octal SPI Flash
 *                  - \ref PHASE_ORDER_MODE0 : Reveice Data Format Ref. {byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7}
 *                  - \ref PHASE_ORDER_MODE1 : Reveice Data Format Ref. {byte7, byte6, byte5, byte4, byte3, byte2, byte1, byte0}
 *                  - \ref PHASE_ORDER_MODE2 : Reveice Data Format Ref. {byte1, byte0, byte3, byte2, byte5, byte4, byte7, byte6}
 *                  - \ref PHASE_ORDER_MODE3 : Reveice Data Format Ref. {byte6, byte7, byte4, byte5, byte2, byte3, byte0, byte1}
 * @param u32DTREn Set Enable/Disable DTR(Double Transfer Rate) Mode
 *                  - \ref SPIM_OP_ENABLE
 *                  - \ref SPIM_OP_DISABLE
 * @param u32RdDQS Set Enable/Disable Read DQS
 *                  - \ref SPIM_OP_ENABLE
 *                  - \ref SPIM_OP_DISABLE
 * @return     SPIM_OK          SPIM operation OK.
 *             SPIM_ERR_FAIL    SPIM operation Fail.
 */
static int32_t _SPIM_SetDataPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32NBit,
                                  uint32_t u32ByteOrder, uint32_t u32DTREn, uint32_t u32RdDQS)
{
    uint32_t *pu32PhaseReg = (uint32_t *)_SPIM_SwitchPhaseRegister(spim, u32OPMode);

    if (pu32PhaseReg == NULL)
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
 * @brief Init DMA/DMM SPI Flash Read/Wirte Command Phase
 * @param spim
 * @param psPhaseTable  Check SPI Flash specifications for support command codes,
 *                      and create command phase table.
 * @param u32OPMode     SPI Function Operation Mode
 *                      - \ref SPIM_CTL0_OPMODE_PAGEWRITE : DMA Write mode
 *                      - \ref SPIM_CTL0_OPMODE_PAGEREAD  : DMA Read mode
 *                      - \ref SPIM_CTL0_OPMODE_DIRECTMAP : Direct Memory Mapping mode
 */
void SPIM_DMADMM_InitPhase(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable, uint32_t u32OPMode)
{
    uint32_t u32Is4ByteAddr = 0;
    uint32_t u32QuadMode = 0;
    uint32_t u32CmdBit = SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase);

    /* Set SPIM DTR Mode */
    SPIM_SET_DTR_MODE(spim, psPhaseTable->u32CMDDTR);

    u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, u32CmdBit);

    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                    (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    SPIM_SetQuadEnable(spim, u32QuadMode, u32CmdBit);

    SPIM_DISABLE_DMM_BWEN(spim);
    SPIM_DISABLE_DMM_CREN(spim);

    /* Set Flash Command Phase */
    _SPIM_SetCMDPhase(spim,
                      u32OPMode,
                      psPhaseTable->u32CMDPhase,
                      psPhaseTable->u32CMDWidth,
                      psPhaseTable->u32CMDDTR);

    /* Set Flash Address Phase */
    _SPIM_SetAddrPhase(spim,
                       u32OPMode,
                       psPhaseTable->u32AddrPhase,
                       psPhaseTable->u32AddrWidth,
                       psPhaseTable->u32AddrDTR);

    /* Set DMA Read/DMM Continue Read Phase */
    _SPIM_SetContReadPhase(spim,
                           u32OPMode,
                           psPhaseTable->u32RdModePhase,
                           psPhaseTable->u32RdModeWidth,
                           psPhaseTable->u32ContRdEn,
                           psPhaseTable->u32RdModeDTR);

    /* Set Flash Data Phase */
    _SPIM_SetDataPhase(spim,
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
}

/*----------------------------------------------------------------------------*/
/* I/O Read/Write Operations by Phase Table                                   */
/*----------------------------------------------------------------------------*/
/**
 * @brief Normal I/O mode sends commands phase.
 * @param spim
 * @param u32OPMode     Normal I/O read or wirte mode.
 *                      - \ref SPIM_IO_WRITE_PHASE
 *                      - \ref SPIM_IO_READ_PHASE
 * @param u32OpCMD      Read/Write Command reference SPI Flash specification.
 * @param u32CMDPhase   Command Bit Mode
 *                      - \ref PHASE_NORMAL_MODE : Send address use standard mode.
 *                      - \ref PHASE_DUAL_MODE   : Send address use dual mode.
 *                      - \ref PHASE_QUAD_MODE   : Send address use quad mode.
 *                      - \ref PHASE_OCTAL_MODE  : Send address use octal mode.
 * @param u32DTREn      Double data rate
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 */
void SPIM_IO_SendCommandByPhase(SPIM_T *spim, uint32_t u32OPMode, uint32_t u32OpCMD,
                                uint32_t u32CMDPhase, uint32_t u32DTREn)
{
    uint8_t u8CmdBuf[2] = {u32OpCMD, u32OpCMD};

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
    _SPIM_WriteData(spim, u8CmdBuf, (u32DTREn == SPIM_OP_ENABLE) ? 2UL : 1UL, SPIM_PhaseModeToNBit(u32CMDPhase));

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief   Normal I/O mode send address phase
 * @param   spim
 * @param   u32Is4ByteAddr    SPI Flash 4 Bytes Address
 * @param   u32Addr           Read/Write start addess
 * @param   u32AddrPhase      Address Bit Mode
 *                          - \ref PHASE_NORMAL_MODE : Send address use standard mode.
 *                          - \ref PHASE_DUAL_MODE   : Send address use dual mode.
 *                          - \ref PHASE_QUAD_MODE   : Send address use quad mode.
 *                          - \ref PHASE_OCTAL_MODE  : Send address use octal mode.
 * @param   u32DTREn          Double Data Rate
 *                          - \ref SPIM_OP_ENABLE
 *                          - \ref SPIM_OP_DISABLE
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
        u8CmdBuf[u8BufIdx++] = (uint8_t)(u32Addr >> 24);
    }

    u8CmdBuf[u8BufIdx++] = (uint8_t)(u32Addr >> 16);
    u8CmdBuf[u8BufIdx++] = (uint8_t)(u32Addr >> 8);
    u8CmdBuf[u8BufIdx++] = (uint8_t) u32Addr;

    /* This field is for continuous read mode, but the I/O mode is not supported. */
    if ((g_au8IDBuf[0] == MFGID_WINBOND) &&
            ((SPIM_PhaseModeToNBit(u32AddrPhase) == 2) ||
             (SPIM_PhaseModeToNBit(u32AddrPhase) == 4)))
    {
        u8BufIdx++;
    }

    /* Write out u32Address. */
    _SPIM_WriteData(spim, u8CmdBuf, u8BufIdx, SPIM_PhaseModeToNBit(u32AddrPhase));

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief Normal I/O mode send dummy cycle clocks.
 * @param spim
 * @param u32NDummy dummy clocks, reference SPI flash command specification
 * @return int32_t
 */
int32_t SPIM_IO_SendDummyByPhase(SPIM_T *spim, int u32NDummy)
{
    uint32_t u32DTREn = 0;
    int32_t i32Ret = SPIM_OK;

    if (u32NDummy == 0)
    {
        return SPIM_OK;
    }

    u32DTREn = SPIM_GET_DTR_MODE(spim);

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    _SPIM_SwitchNBitInput(spim, SPIM_BITMODE_1);
    spim->TX[0] = 0x00;

    /* Switch to Normal mode. */
    SPIM_SET_OPMODE(spim, SPIM_CTL0_OPMODE_IO);
    SPIM_SET_DATA_WIDTH(spim, u32NDummy);
    SPIM_SET_BURST_DATA(spim, SPIM_BITMODE_1);

    i32Ret = _SPIM_WaitOpDone(spim, SPIM_OP_ENABLE);

    /* Restore DTR Mode. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    return i32Ret;
}

/**
 * @brief Normal I/O mode data phase.
 * @param spim
 * @param u32OPMode     Normal I/O read or wirte mode.
 *                      - \ref SPIM_IO_WRITE_PHASE
 *                      - \ref SPIM_IO_READ_PHASE
 * @param pu8TRxBuf     Read/Write Data Buffer
 * @param u32TRxSize    Read/Write Data Size
 * @param u32DataPhase  Data Bit Mode
 *                      - \ref PHASE_NORMAL_MODE : Send data use standard mode.
 *                      - \ref PHASE_DUAL_MODE   : Send data use dual mode.
 *                      - \ref PHASE_QUAD_MODE   : Send data use quad mode.
 *                      - \ref PHASE_OCTAL_MODE  : Send data use octal mode.
 * @param u32DTREn      DTR mode
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 * @param u32RdDQS      Receive data from SPI Flash when read DQS
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 */
void SPIM_IO_RWDataByPhase(SPIM_T *spim, uint32_t u32OPMode, uint8_t *pu8TRxBuf,
                           uint32_t u32TRxSize, uint32_t u32DataPhase, uint32_t u32DTREn, uint32_t u32RdDQS)
{
    /* DTR Activated. */
    SPIM_SET_DTR_MODE(spim, u32DTREn);

    /* CS activated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_ENABLE);

    if (u32OPMode == SPIM_IO_WRITE_PHASE)
    {
        /* Write out data. */
        _SPIM_WriteData(spim, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }
    else
    {
        /* Read back data. */
        _SPIM_ReadData(spim, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }

    /* CS Deactivated. */
    SPIM_SET_SS_EN(spim, SPIM_OP_DISABLE);

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
}

/**
 * @brief      Write data to SPI Flash by command and phase in I/O mode.
 *
 * @param[in]  spim
 * @param[in]  psPhaseTable   The pointer of SPIM phase table.
 * @param[in]  u32Addr        The address of data to be programmed.
 * @param[in]  u32Is4ByteAddr Indicate if use 4-byte address mode.
 * @param[in]  pu8TxBuf       The pointer of data to be programmed.
 * @param[in]  u32RdSize      The length of data to be programmed.
 * @param[in]  u32WrDone      Indicate if wait write done or not.
 *
 * @return     None.
 */
static void SPIM_WriteInPageDataByPhaseIO(SPIM_T *spim,
                                          SPIM_PHASE_T *psPhaseTable,
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
    SPIM_IO_SendDummyByPhase(spim, psPhaseTable->u32DcNum);

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

        _SPIM_WaitWriteDone(spim, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));

        /* DTR Deactivated. */
        SPIM_SET_DTR_MODE(spim, SPIM_OP_DISABLE);
    }
}

/**
 * @brief   Write data to SPI Flash by Phase Setting (I/O mode).
 * @param   spim          Write data to SPI Flash by use phase table (I/O mode).
 * @param   psPhaseTable  Check SPI Flash specifications for support command codes,
 *                        and create command phase table.
 * @param   u32Addr       Write start address
 * @param   pu8TxBuf      Data buffer
 * @param   u32WrSize     Write data size
 * @param   u32WrDone     Enable write done function or not.
 */
void SPIM_IO_WriteByPhase(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable,
                          uint32_t u32Addr, uint8_t *pu8TxBuf, uint32_t u32WrSize, uint32_t u32WrDone)
{
    uint32_t u32DTREn = SPIM_GET_DTR_MODE(spim);
    /* Buffer index. */
    uint32_t u32BufIdx = 0UL;
    /* The number of bytes to write*/
    uint32_t toWr = 0UL;
    /* 4-byte address mode or not. */
    uint32_t u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32)
                               ? SPIM_OP_ENABLE
                               : SPIM_OP_DISABLE);
    /* Quad mode or not. */
    uint32_t u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                             (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    if (u32QuadMode)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
    }

    while (u32WrSize)
    {
        toWr = (u32WrSize < SPIM_FLH_PAGE_SIZE) ? u32WrSize : SPIM_FLH_PAGE_SIZE;

        SPIM_WriteInPageDataByPhaseIO(spim, psPhaseTable, u32Addr, u32Is4ByteAddr, &pu8TxBuf[u32BufIdx], toWr, u32WrDone);
        /* Advance indicator. */
        u32Addr += toWr;
        u32WrSize -= toWr;
        u32BufIdx += toWr;
    }

    if (u32QuadMode)
    {
        SPIM_SetQuadEnable(spim, SPIM_OP_DISABLE, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
    }

    SPIM_SET_DTR_MODE(spim, u32DTREn);
}

/**
 * @brief   Read data from SPI Flash by Phase Setting (I/O mode).
 * @param   spim          Read data from SPI Flash by use phase table (I/O mode).
 * @param   psPhaseTable  Check SPI Flash specifications for support command codes,
 *                        and create command phase table.
 * @param   u32Addr       Read start address
 * @param   pu8RxBuf      Data buffer
 * @param   u32RdSize     Read data size
 */
void SPIM_IO_ReadByPhase(SPIM_T *spim, SPIM_PHASE_T *psPhaseTable,
                         uint32_t u32Addr, uint8_t *pu8RxBuf, uint32_t u32RdSize)
{
    uint32_t u32DTREn = SPIM_GET_DTR_MODE(spim);
    uint32_t u32Is4ByteAddr = ((psPhaseTable->u32AddrWidth == PHASE_WIDTH_32) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);
    uint32_t u32QuadMode = (((psPhaseTable->u32AddrPhase == PHASE_QUAD_MODE) ||
                             (psPhaseTable->u32DataPhase == PHASE_QUAD_MODE)) ? SPIM_OP_ENABLE : SPIM_OP_DISABLE);

    /* Set Flash 4-Bytes Address Mode */
    SPIM_Enable_4Bytes_Mode(spim, u32Is4ByteAddr, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
    /* Set SPIM 4-Bytes Address Mode */
    SPIM_SET_4BYTE_ADDR(spim, u32Is4ByteAddr);

    if (u32QuadMode)
    {
        SPIM_SetQuadEnable(spim, u32QuadMode, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
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
    SPIM_IO_SendDummyByPhase(spim, psPhaseTable->u32DcNum);

    SPIM_IO_RWDataByPhase(spim,
                          SPIM_IO_READ_PHASE,
                          pu8RxBuf,
                          u32RdSize,
                          psPhaseTable->u32DataPhase,
                          psPhaseTable->u32DataDTR,
                          psPhaseTable->u32RDQS);

    if (u32QuadMode)
    {
        SPIM_SetQuadEnable(spim, SPIM_OP_DISABLE, SPIM_PhaseModeToNBit(psPhaseTable->u32CMDPhase));
    }

    SPIM_SET_DTR_MODE(spim, u32DTREn);
}

/**
 * @brief      Enable SPIM DLL.
 * @param      spim
 * @retval     SPIM_OK          DLL locked successfully.
 * @retval     SPIM_ERR_TIMEOUT DLL locked failed due to timeout.
 * @details    This function is used to enable DLL clock divider circuit,
 *             and send DLL reference clock to DLL circuit that the frequency is the same as the SPIM output bus clock.
 * @note       When system wake up reset, please calling this function reinintialize DLL.
 */
int32_t SPIM_INIT_DLL(SPIM_T *spim)
{
    volatile int i32TimeoutCount = SPIM_TIMEOUT;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint32_t u32FastEn = ((((CLK_GetSCLKFreq() / 1000000) / (SPIM_GET_CLOCK_DIVIDER(spim) * 2)) <= 100) ? SPIM_OP_DISABLE : SPIM_OP_ENABLE);

#if (SPIM_TRIM_HYPERDLL == 1)

    SPIM_ENABLE_SYSDLL0ATCTL0_TRIMUPDOFF();
#endif

    /* Set SPIM DLL clock divider */
    SPIM_SET_DLLDIV(spim, (u32Div > 3) ? 3 : u32Div);

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
            return SPIM_ERR_TIMEOUT;
        }
    }

    i32TimeoutCount = SPIM_TIMEOUT;

    /* Polling the DLL status register DLLCKON to 0x1,
       and the value 0x1 indicates that clock divider circuit inside DLL is enabled. */
    while (SPIM_GET_DLLCLKON(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    i32TimeoutCount = SPIM_TIMEOUT;

    /* Polling the DLL status register DLLLOCK to 0x1,
       and the value 0x1 indicates that DLL circuit is in lock state */
    while (SPIM_GET_DLLLOCK(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    i32TimeoutCount = SPIM_TIMEOUT;

    /* Polling the DLL status register DLLREADY to 0x1,
       and the value 0x1 indicates that output of DLL circuit is ready. */
    while (SPIM_GET_DLLREADY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

#if (SPIM_TRIM_HYPERDLL == 1)
    i32TimeoutCount = SPIM_TIMEOUT;

    /* Polling the DLL status register DLLREADY to 0x1,
    and the value 0x1 indicates that output of DLL circuit is ready. */
    while (SPIM_GET_DLLATRDY(spim) != SPIM_OP_ENABLE)
    {
        if (--i32TimeoutCount <= 0)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    /* wait for auto trim setting */
    for (i32TimeoutCount = 0; i32TimeoutCount < 0x200000; i32TimeoutCount++)
    {
        __NOP();
    }

    SPIM_DISABLE_SYSDLL0ATCTL0_TRIMUPDOFF();

    SPIM_SET_INTERNAL_RWDS(spim, SPIM_HYPER_OP_DISABLE);
#endif

    return SPIM_OK;
}

/**
 * @brief      Set the delay number of the DLL, and then update the delay step number
 *             of the DLL to control register DLL_DNUM.
 * @param      spim        SPIM base address
 * @param      u32DelayNum Delay number of the DLL
 * @retval     SPIM_OK             SPIM operation OK.
 *             SPIM_ERR_TIMEOUT    SPIM operation abort due to timeout error.
 * @note       This function sets SPIM_ERR_TIMEOUT, if waiting Hyper Chip time-out.
 * @note       First fix DIVIDER to set the frequency of SPIM output bus clock.
 */
int32_t SPIM_SetDLLDelayNum(SPIM_T *spim, uint32_t u32DelayNum)
{
    volatile int i32Timeout = SPIM_TIMEOUT;

    if (SPIM_GET_DLLOLDO(spim) != SPIM_OP_ENABLE)
    {
        SPIM_INIT_DLL(spim);
    }

    /* Set this valid delay number to control register DLL_DNUM. */
    SPIM_SET_DLLDLY_NUM(spim, u32DelayNum);

    /* Polling DLL status register DLL_REF to 1
       to know the updating flow of DLL delay step number is finish or not. */
    while (SPIM_GET_DLLREF(spim) != SPIM_OP_DISABLE)
    {
        if (--i32Timeout <= 0)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    return SPIM_OK;
}

/** @} end of group SPIM_EXPORTED_FUNCTIONS */
/** @} end of group SPIM_Driver */
/** @} end of group Standard_Driver */
