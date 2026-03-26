/**************************************************************************//**
 * @file     sdh.c
 * @version  V1.00
 * @brief    SDH driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SDH_Driver SDH Driver
  @{
*/

/** @addtogroup SDH_EXPORTED_FUNCTIONS SDH Exported Functions
  @{
*/

/** @cond HIDDEN_SYMBOLS */

/* global variables */
/* For response R3 (such as ACMD41, CRC-7 is invalid; but SD controller will still */
/* calculate CRC-7 and get an error result, software should ignore this error and clear SDISR [CRC_IF] flag */
/* _sd_uR3_CMD is the flag for it. 1 means software should ignore CRC-7 error */

#if (NVT_DCACHE_ON == 1)
/* Declare the SDH dedicated DMA buffer as non-cacheable. */
/* Note: Ensure that NVT_DCACHE_ON is enabled and the ARM MPU is initialized for the non-cacheable region in your project. */
NVT_NONCACHEABLE static uint8_t _SDH0_ucSDHCBuffer[SDH_BLOCK_SIZE] __attribute__((aligned(4)))  = {0} ;
NVT_NONCACHEABLE static uint8_t _SDH1_ucSDHCBuffer[SDH_BLOCK_SIZE] __attribute__((aligned(4)))  = {0} ;
#define DEF_ALIGNED_VALUE      DCACHE_LINE_SIZE
#else
static uint8_t _SDH0_ucSDHCBuffer[SDH_BLOCK_SIZE] __attribute__((aligned(4)))  = {0} ;
static uint8_t _SDH1_ucSDHCBuffer[SDH_BLOCK_SIZE] __attribute__((aligned(4)))  = {0} ;
#define DEF_ALIGNED_VALUE      4
#endif
SDH_INFO_T SD0;
SDH_INFO_T SD1;

//------------------------------------------------------------------------------
void *SDH_GetSDH0Buffer(void)
{
    return &_SDH0_ucSDHCBuffer[0];
}

void *SDH_GetSDH1Buffer(void)
{
    return &_SDH1_ucSDHCBuffer[0];
}

void *SDH_GetSDInfoMsg(const SDH_T *sdh)
{
    if (sdh == SDH0)
    {
        return &SD0;
    }
    else if (sdh == SDH1)
    {
        return &SD1;
    }
    else
    {
    }

    return NULL;
}

int32_t SDH_CheckRB(SDH_T *sdh)
{
    int32_t i32TimeOutCount2;
    uint32_t u32Done = 0U;
    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    i32TimeOutCount2 = (int32_t)SDH_TIMEOUT_CNT;

    while (u32Done == 0U)
    {
        int32_t i32TimeOutCount1 = (int32_t)SDH_TIMEOUT_CNT;
        sdh->CTL |= SDH_CTL_CLK8OEN_Msk;

        while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
        {
            if (--i32TimeOutCount1 == 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_DAT0STS_Msk) == SDH_INTSTS_DAT0STS_Msk)
        {
            u32Done = 1;
        }
        else
        {
            if (--i32TimeOutCount2 == 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                u32Done = 1;
            }
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return Successful;
}

uint32_t SDH_SDCommand(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg)
{
    volatile uint32_t u32Buf;
    volatile uint32_t u32Val = 0U;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    sdh->CMDARG = u32Arg;
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8U) | (SDH_CTL_COEN_Msk);
    sdh->CTL = u32Buf;

    while ((sdh->CTL & SDH_CTL_COEN_Msk) == SDH_CTL_COEN_Msk)
    {
        if (pSD->IsCardInsert == 0U)
        {
            u32Val = SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return u32Val;
}

uint32_t SDH_SDCmdAndRsp(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg, uint32_t u32NtickCount)
{
    volatile uint32_t u32Buf;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    sdh->CMDARG = u32Arg;
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8U) | (SDH_CTL_COEN_Msk | SDH_CTL_RIEN_Msk);
    sdh->CTL = u32Buf;

    if (u32NtickCount > 0U)
    {
        uint32_t u32NtickCountTmp = u32NtickCount;

        while ((sdh->CTL & SDH_CTL_RIEN_Msk) == SDH_CTL_RIEN_Msk)
        {
            u32NtickCountTmp--;

            if (u32NtickCountTmp == 0U)
            {
                sdh->CTL |= SDH_CTL_CTLRST_Msk; /* reset SD engine */
                return 2U;
            }

            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }
        }
    }
    else
    {
        while ((sdh->CTL & SDH_CTL_RIEN_Msk) == SDH_CTL_RIEN_Msk)
        {
            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    if (pSD->R7Flag)
    {
        uint32_t tmp0 = 0U;
        uint32_t tmp1 = 0U;
        tmp1 = sdh->RESP1 & 0xffU;
        tmp0 = sdh->RESP0 & 0xfU;

        if ((tmp1 != 0x55U) && (tmp0 != 0x01U))
        {
            pSD->R7Flag = 0U;
            return SDH_CMD8_ERROR;
        }
    }

    if (!pSD->R3Flag)
    {
        if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) == SDH_INTSTS_CRC7_Msk)     /* check CRC7 */
        {
            return Successful;
        }
        else
        {
            return SDH_CRC7_ERROR;
        }
    }
    else
    {
        /* ignore CRC error for R3 case */
        pSD->R3Flag = 0U;
        sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
        return Successful;
    }
}

static uint32_t SDH_Swap32(uint32_t u32Val)
{
    uint32_t u32B0 = (u32Val & 0x000000FFU) << 24;
    uint32_t u32B1 = (u32Val & 0x0000FF00U) << 8;
    uint32_t u32B2 = (u32Val & 0x00FF0000U) >> 8;
    uint32_t u32B3 = (u32Val & 0xFF000000U) >> 24;

    return (u32B0 | u32B1 | u32B2 | u32B3);
}

/* Get 16 bytes CID or CSD */
uint32_t SDH_SDCmdAndRsp2(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg, uint32_t pu32R2ptr[])
{
    uint32_t u32Buf;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    sdh->CMDARG = u32Arg;
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) |
             (u32Cmd << 8) | (SDH_CTL_COEN_Msk | SDH_CTL_R2EN_Msk);
    sdh->CTL = u32Buf;

    while ((sdh->CTL & SDH_CTL_R2EN_Msk) == SDH_CTL_R2EN_Msk)
    {
        if (pSD->IsCardInsert == FALSE)
        {
            return SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) == SDH_INTSTS_CRC7_Msk)
    {
        uint32_t u32i;
        uint32_t tmpBuf[5];

        for (u32i = 0U; u32i < 5U; u32i++)
        {
            tmpBuf[u32i] = SDH_Swap32(sdh->FB[u32i]);
        }

        for (u32i = 0U; u32i < 4U; u32i++)
        {
            pu32R2ptr[u32i] = ((tmpBuf[u32i] & 0x00ffffffU) << 8U) |
                              ((tmpBuf[u32i + 1U] & 0xff000000U) >> 24U);
        }
    }
    else
    {
        return SDH_CRC7_ERROR;
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return Successful;
}


static uint32_t SDH_SDCmdAndRspDataIn(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg)
{
    volatile uint32_t u32Buf;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    sdh->CMDARG = u32Arg;
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8U) |
             (SDH_CTL_COEN_Msk | SDH_CTL_RIEN_Msk | SDH_CTL_DIEN_Msk);

    sdh->CTL = u32Buf;

    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_RIEN_Msk) == SDH_CTL_RIEN_Msk)
    {
        if (pSD->IsCardInsert == FALSE)
        {
            return SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_DIEN_Msk) == SDH_CTL_DIEN_Msk)
    {
        if (pSD->IsCardInsert == FALSE)
        {
            return SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) != SDH_INTSTS_CRC7_Msk)
    {
        /* check CRC7 */
        return SDH_CRC7_ERROR;
    }

    if ((sdh->INTSTS & SDH_INTSTS_CRC16_Msk) != SDH_INTSTS_CRC16_Msk)
    {
        /* check CRC16 */
        return SDH_CRC16_ERROR;
    }

    return 0U;
}

/* there are 8 bits for divider0, maximum is 256 */
#define SDH_CLK_DIV0_MAX     256U

void SDH_Set_clock(const SDH_T *sdh, uint32_t u32SD_clk_khz)
{
    static uint32_t _SDH0_ReferenceClock;
    static uint32_t _SDH1_ReferenceClock;
    uint32_t u32Rate;
    uint32_t u32Div1;
    static uint32_t u32SD_ClkSrc = 0U;
    uint32_t u32SD_Clk_khzTemp = u32SD_clk_khz;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    /* initial state, clock source use HIRC */
    if (u32SD_Clk_khzTemp <= 400U)
    {
        static uint32_t u32SD_PwrCtl = 0U;

        u32SD_PwrCtl = CLK->SRCCTL;

        if ((u32SD_PwrCtl & CLK_SRCCTL_HIRCEN_Msk) != 0x4U)
        {
            CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;
        }

        if (sdh == SDH0)
        {
            u32SD_ClkSrc = (CLK->SDHSEL & CLK_SDHSEL_SDH0SEL_Msk);
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH0SEL_Msk) |
                          CLK_SDHSEL_SDH0SEL_HIRC;
            _SDH0_ReferenceClock = (__HIRC / 1000U);
        }
        else
        {
            u32SD_ClkSrc = (CLK->SDHSEL & CLK_SDHSEL_SDH1SEL_Msk);
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH1SEL_Msk) |
                          CLK_SDHSEL_SDH1SEL_HIRC;
            _SDH1_ReferenceClock = (__HIRC / 1000U);
        }
    }
    /* transfer state, clock source use sys_init() */
    else
    {
        //CLK->SRCCTL = u32SD_PwrCtl;

        if (sdh == SDH0)
        {
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH0SEL_Msk) | u32SD_ClkSrc;

            switch (u32SD_ClkSrc)
            {
                case CLK_SDHSEL_SDH0SEL_APLL1_DIV2:
                    _SDH0_ReferenceClock = ((CLK_GetAPLL1ClockFreq() >> 1U) / 1000U);
                    break;

                case CLK_SDHSEL_SDH0SEL_HCLK0:
                    _SDH0_ReferenceClock = (CLK_GetHCLK0Freq() / 1000U);
                    break;

                case CLK_SDHSEL_SDH0SEL_HIRC:
                    _SDH0_ReferenceClock = (__HIRC / 1000U);
                    break;

                case CLK_SDHSEL_SDH0SEL_HIRC48M_DIV4:
                    _SDH0_ReferenceClock = ((__HIRC48M / 1000U) / 4U);
                    break;

                case CLK_SDHSEL_SDH0SEL_HXT:
                default:
                    _SDH0_ReferenceClock = (CLK_GetHXTFreq() / 1000U);
                    break;
            }
        }
        else
        {
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH1SEL_Msk) | u32SD_ClkSrc;

            switch (u32SD_ClkSrc)
            {
                case CLK_SDHSEL_SDH1SEL_APLL1_DIV2:
                    _SDH1_ReferenceClock = ((CLK_GetAPLL1ClockFreq() >> 1U) / 1000U);
                    break;

                case CLK_SDHSEL_SDH1SEL_HCLK0:
                    _SDH1_ReferenceClock = (CLK_GetHCLK0Freq() / 1000U);
                    break;

                case CLK_SDHSEL_SDH1SEL_HIRC:
                    _SDH1_ReferenceClock = (__HIRC / 1000U);
                    break;

                case CLK_SDHSEL_SDH1SEL_HIRC48M_DIV4:
                    _SDH1_ReferenceClock = ((__HIRC48M / 1000U) / 4U);
                    break;

                case CLK_SDHSEL_SDH1SEL_HXT:
                default:
                    _SDH1_ReferenceClock = (CLK_GetHXTFreq() / 1000U);
                    break;
            }
        }

        if (u32SD_Clk_khzTemp >= 50000U)
        {
            u32SD_Clk_khzTemp = 50000U;
        }
    }

    if (sdh == SDH0)
    {
        u32Rate = _SDH0_ReferenceClock / u32SD_Clk_khzTemp;

        /* choose slower clock if system clock cannot divisible by wanted clock */
        if ((_SDH0_ReferenceClock % u32SD_Clk_khzTemp) != 0U)
        {
            u32Rate++;
        }
    }
    else
    {
        u32Rate = _SDH1_ReferenceClock / u32SD_Clk_khzTemp;

        /* choose slower clock if system clock cannot divisible by wanted clock */
        if ((_SDH1_ReferenceClock % u32SD_Clk_khzTemp) != 0U)
        {
            u32Rate++;
        }
    }

    if (u32Rate >= SDH_CLK_DIV0_MAX)
    {
        u32Rate = SDH_CLK_DIV0_MAX;
    }

    /*--- calculate the second divider CLKDIV0[SDHOST_N]*/
    u32Div1 = (u32Rate - 1U) & 0xFFU;

    /*--- setup register */
    if (sdh == SDH0)
    {
        CLK->SDHDIV &= ~CLK_SDHDIV_SDH0DIV_Msk;
        CLK->SDHDIV |= (u32Div1 << CLK_SDHDIV_SDH0DIV_Pos);
    }
    else
    {
        CLK->SDHDIV &= ~CLK_SDHDIV_SDH1DIV_Msk;
        CLK->SDHDIV |= (u32Div1 << CLK_SDHDIV_SDH1DIV_Pos);
    }

    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }

    return;
}

uint32_t SDH_CardDetection(SDH_T *sdh)
{
    uint32_t u32Val = TRUE;
    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    if ((sdh->INTEN & SDH_INTEN_CDSRC_Msk) == SDH_INTEN_CDSRC_Msk)   /* Card detect pin from GPIO */
    {
        sdh->CTL &= ~SDH_CTL_CLKKEEP_Msk;

        if ((sdh->INTSTS & SDH_INTSTS_CDSTS_Msk) == SDH_INTSTS_CDSTS_Msk)   /* Card remove */
        {
            pSD->IsCardInsert = (uint8_t)FALSE;
            u32Val = FALSE;
        }
        else
        {
            pSD->IsCardInsert = (uint8_t)TRUE;
        }
    }
    else if ((sdh->INTEN & SDH_INTEN_CDSRC_Msk) != SDH_INTEN_CDSRC_Msk)
    {
        volatile uint32_t u32i;
        sdh->CTL |= SDH_CTL_CLKKEEP_Msk;

        for (u32i = 0U; u32i < 5000U; u32i++)
        {
        }

        if ((sdh->INTSTS & SDH_INTSTS_CDSTS_Msk) == SDH_INTSTS_CDSTS_Msk)   /* Card insert */
        {
            pSD->IsCardInsert = (uint8_t)TRUE;
        }
        else
        {
            pSD->IsCardInsert = (uint8_t)FALSE;
            u32Val = FALSE;
        }

        sdh->CTL &= ~SDH_CTL_CLKKEEP_Msk;
    }
    else
    {

    }

    return u32Val;
}

static uint32_t SDH_Init(SDH_T *sdh)
{
    volatile uint32_t u32i = 0;
    uint32_t u32Resp;
    uint32_t volatile u32CmdTimeOut;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    /* set the clock to 300KHz */
    SDH_Set_clock(sdh, 300U);

    /* power ON 74 clock */
    sdh->CTL |= SDH_CTL_CLK74OEN_Msk;

    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK74OEN_Msk) == SDH_CTL_CLK74OEN_Msk)
    {
        if (pSD->IsCardInsert == FALSE)
        {
            return SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    (void)SDH_SDCommand(sdh, 0U, 0U);        /* reset all cards */

    for (u32i = 0; u32i < 0x1000U; u32i++) {}

    /* initial SDHC */
    pSD->R7Flag = 1U;
    u32CmdTimeOut = 0xFFFFFU;

    u32i = SDH_SDCmdAndRsp(sdh, 8U, 0x00000155U, u32CmdTimeOut);

    if (u32i == Successful)
    {
        /* SD 2.0 */
        (void)SDH_SDCmdAndRsp(sdh, 55U, 0x00U, u32CmdTimeOut);
        pSD->R3Flag = 1U;
        (void)SDH_SDCmdAndRsp(sdh, 41U, 0x40ff8000U, u32CmdTimeOut); /* 2.7v-3.6v */
        u32Resp = sdh->RESP0;

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while ((u32Resp & 0x00800000U) != 0x00800000U)        /* check if card is ready */
        {
            (void)SDH_SDCmdAndRsp(sdh, 55U, 0x00U, u32CmdTimeOut);
            pSD->R3Flag = 1U;
            (void)SDH_SDCmdAndRsp(sdh, 41U, 0x40ff8000U, u32CmdTimeOut); /* 3.0v-3.4v */
            u32Resp = sdh->RESP0;

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((u32Resp & 0x00400000U) == 0x00400000U)
        {
            pSD->CardType = SDH_TYPE_SD_HIGH;
        }
        else
        {
            pSD->CardType = SDH_TYPE_SD_LOW;
        }
    }
    else
    {
        /* SD 1.1 */
        (void)SDH_SDCommand(sdh, 0U, 0U);        /* reset all cards */

        for (u32i = 0; u32i < 0x100U; u32i++)
        {
        }

        u32i = SDH_SDCmdAndRsp(sdh, 55U, 0x00U, u32CmdTimeOut);

        if (u32i == 2U)     /* MMC memory */
        {
            (void)SDH_SDCommand(sdh, 0U, 0U);        /* reset */

            for (u32i = 0; u32i < 0x100U; u32i++)
            {
            }

            pSD->R3Flag = 1U;

            if (SDH_SDCmdAndRsp(sdh, 1U, 0x40ff8000U, u32CmdTimeOut) != 2U)    /* eMMC memory */
            {
                u32Resp = sdh->RESP0;
                i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

                while ((u32Resp & 0x00800000U) != 0x00800000U)
                {
                    /* check if card is ready */
                    pSD->R3Flag = 1U;

                    (void)SDH_SDCmdAndRsp(sdh, 1U, 0x40ff8000U, u32CmdTimeOut);      /* high voltage */
                    u32Resp = sdh->RESP0;

                    if (--i32TimeOutCount <= 0)
                    {
                        pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                        break;
                    }
                }

                if ((u32Resp & 0x00400000U) == 0x00400000U)
                {
                    pSD->CardType = SDH_TYPE_EMMC;
                }
                else
                {
                    pSD->CardType = SDH_TYPE_MMC;
                }
            }
            else
            {
                pSD->CardType = SDH_TYPE_UNKNOWN;
                return SDH_ERR_DEVICE;
            }
        }
        else if (u32i == 0U)     /* SD Memory */
        {
            pSD->R3Flag = 1U;
            (void)SDH_SDCmdAndRsp(sdh, 41U, 0x00ff8000U, u32CmdTimeOut); /* 3.0v-3.4v */
            u32Resp = sdh->RESP0;
            i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

            while ((u32Resp & 0x00800000U) != 0x00800000U)        /* check if card is ready */
            {
                (void)SDH_SDCmdAndRsp(sdh, 55U, 0x00U, u32CmdTimeOut);
                pSD->R3Flag = 1U;
                (void)SDH_SDCmdAndRsp(sdh, 41U, 0x00ff8000U, u32CmdTimeOut); /* 3.0v-3.4v */
                u32Resp = sdh->RESP0;

                if (--i32TimeOutCount <= 0)
                {
                    pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                    break;
                }
            }

            pSD->CardType = SDH_TYPE_SD_LOW;
        }
        else
        {
            pSD->CardType = SDH_TYPE_UNKNOWN;
            return SDH_INIT_ERROR;
        }
    }

    if (pSD->CardType != SDH_TYPE_UNKNOWN)
    {
        uint32_t u32Status;
        uint32_t au32CIDBuffer[4];
        (void)SDH_SDCmdAndRsp2(sdh, 2U, 0x00U, au32CIDBuffer);

        if ((pSD->CardType == SDH_TYPE_MMC) ||
                (pSD->CardType == SDH_TYPE_EMMC))
        {
            u32Status = SDH_SDCmdAndRsp(sdh, 3U, 0x10000U, 0U);

            if (u32Status != Successful)     /* set RCA */
            {
                return u32Status;
            }

            pSD->RCA = 0x10000U;
        }
        else
        {
            u32Status = SDH_SDCmdAndRsp(sdh, 3U, 0x00U, 0U);

            if (u32Status != Successful)       /* get RCA */
            {
                return u32Status;
            }
            else
            {
                pSD->RCA = ((sdh->RESP0 << 8U) & 0xffff0000U);
            }
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return Successful;
}

uint32_t SDH_SwitchToHighSpeed(SDH_T *sdh, SDH_INFO_T *pSD)
{
    uint32_t volatile u32Status = 0U;
    uint16_t current_comsumption;
    uint16_t busy_status0;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    pSD->i32ErrCode = 0;

    sdh->DMASA = (uint32_t)pSD->dmabuf;
    sdh->BLEN = 63U;

    u32Status = SDH_SDCmdAndRspDataIn(sdh, 6U, 0x00ffff01U);

    if (u32Status != Successful)
    {
        return Fail;
    }

    current_comsumption = ((uint16_t)(*pSD->dmabuf) << 8U);
    current_comsumption |= (uint16_t)(*(&pSD->dmabuf[1]));

    if (!current_comsumption)
    {
        return Fail;
    }

    busy_status0 = ((uint16_t)(*(&pSD->dmabuf[28])) << 8U);
    busy_status0 |= (uint16_t)(*(&pSD->dmabuf[29]));

    if (!busy_status0)   /* function ready */
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 63U;    /* 512 bit */

        u32Status = SDH_SDCmdAndRspDataIn(sdh, 6U, 0x80ffff01U);

        if (u32Status != Successful)
        {
            return Fail;
        }

        /* function change timing: 8 clocks */
        sdh->CTL |= SDH_CTL_CLK8OEN_Msk;

        while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
        {
            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if (pSD->i32ErrCode != 0)
        {
            return Fail;
        }

        current_comsumption = (uint16_t)((*pSD->dmabuf) << 8U);
        current_comsumption |= (uint16_t)(*(&pSD->dmabuf[1]));

        if (!current_comsumption)
        {
            return Fail;
        }

        return Successful;
    }
    else
    {
        return Fail;
    }
}

uint32_t SDH_SelectCardType(SDH_T *sdh)
{
    uint32_t volatile u32Status = 0U;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    u32Status = SDH_SDCmdAndRsp(sdh, 7U, pSD->RCA, 0U);

    if (u32Status != Successful)
    {
        return u32Status;
    }

    u32Status = SDH_CheckRB(sdh);

    if (u32Status != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    /* if SD card set 4bit */
    if (pSD->CardType == SDH_TYPE_SD_HIGH)
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 0x07U;  /* 64 bit */
        sdh->DMACTL |= SDH_DMACTL_DMARST_Msk;
        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while ((sdh->DMACTL & SDH_DMACTL_DMARST_Msk) == 0x2U)
        {
            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        u32Status = SDH_SDCmdAndRsp(sdh, 55U, pSD->RCA, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        u32Status = SDH_SDCmdAndRspDataIn(sdh, 51U, 0x00U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        if ((*pSD->dmabuf & 0xfU) == 0x2U)
        {
            u32Status = SDH_SwitchToHighSpeed(sdh, pSD);

            if (u32Status == Successful)
            {
                /* divider */
                SDH_Set_clock(sdh, SDHC_FREQ);
            }
        }

        u32Status = SDH_SDCmdAndRsp(sdh, 55U, pSD->RCA, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        u32Status = SDH_SDCmdAndRsp(sdh, 6U, 0x02U, 0U);

        if (u32Status != Successful)   /* set bus width */
        {
            return u32Status;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk;
    }
    else if (pSD->CardType == SDH_TYPE_SD_LOW)
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 0x07U;

        u32Status = SDH_SDCmdAndRsp(sdh, 55U, pSD->RCA, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        u32Status = SDH_SDCmdAndRspDataIn(sdh, 51U, 0x00U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        /* set data bus width. ACMD6 for SD card, SDCR_DBW for host. */
        u32Status = SDH_SDCmdAndRsp(sdh, 55U, pSD->RCA, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        u32Status = SDH_SDCmdAndRsp(sdh, 6U, 0x02U, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk;
    }
    else if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        uint32_t u32Param;

        if (pSD->CardType == SDH_TYPE_MMC)
        {
            sdh->CTL &= ~SDH_CTL_DBW_Msk;
        }

        /*--- sent CMD6 to MMC card to set bus width to 4 bits mode */
        /* set CMD6 argument Access field to 3, Index to 183, Value to 1 (4-bit mode) */
        u32Param = ((uint32_t)3U << 24U) | ((uint32_t)183U << 16U) | ((uint32_t)1U << 8U);

        u32Status = SDH_SDCmdAndRsp(sdh, 6U, u32Param, 0U);

        if (u32Status != Successful)
        {
            return u32Status;
        }

        u32Status = SDH_CheckRB(sdh);

        if (u32Status != Successful)
        {
            return SDH_ERR_TIMEOUT;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk; /* set bus width to 4-bit mode for SD host controller */
    }
    else
    {

    }

    u32Status = SDH_SDCmdAndRsp(sdh, 16U, SDH_BLOCK_SIZE, 0U);

    if (u32Status != Successful)
    {
        return u32Status;
    }

    sdh->BLEN = SDH_BLOCK_SIZE - 1U;

    (void)SDH_SDCommand(sdh, 7U, 0U);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    sdh->INTEN |= SDH_INTEN_BLKDIEN_Msk;

    return Successful;
}

void SDH_Get_SD_info(SDH_T *sdh)
{
    uint32_t u32R_LEN;
    uint32_t u32C_Size;
    uint32_t u32MULT;
    uint32_t u32Size;
    uint32_t au32Buffer[4];
    //unsigned char *ptr;
    SDH_INFO_T *pSD;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    (void)SDH_SDCmdAndRsp2(sdh, 9U, pSD->RCA, au32Buffer);

    if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        /* for MMC/eMMC card */
        if ((au32Buffer[0] & 0xc0000000U) == 0xc0000000U)
        {
            /* CSD_STRUCTURE [127:126] is 3 */
            /* CSD version depend on EXT_CSD register in eMMC v4.4 for card size > 2GB */
            (void)SDH_SDCmdAndRsp(sdh, 7U, pSD->RCA, 0U);

            //ptr = (uint8_t *)((uint32_t)_SDH_ucSDHCBuffer );
            sdh->DMASA = (uint32_t)pSD->dmabuf;
            sdh->BLEN = 511U;  /* read 512 bytes for EXT_CSD */

            if (SDH_SDCmdAndRspDataIn(sdh, 8U, 0x00U) == Successful)
            {
                (void)SDH_SDCommand(sdh, 7U, 0U);
                sdh->CTL |= SDH_CTL_CLK8OEN_Msk;

                while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
                {
                    if (--i32TimeOutCount <= 0)
                    {
                        pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                        break;
                    }
                }

                pSD->totalSectorN = ((uint32_t)(*(&pSD->dmabuf[215])) << 24U);
                pSD->totalSectorN |= ((uint32_t)(*(&pSD->dmabuf[214])) << 16U);
                pSD->totalSectorN |= ((uint32_t)(*(&pSD->dmabuf[213])) << 8U);
                pSD->totalSectorN |= (uint32_t)(*(&pSD->dmabuf[212]));
                pSD->diskSize = (pSD->totalSectorN / 2U);
            }
        }
        else
        {
            /* CSD version v1.0/1.1/1.2 in eMMC v4.4 spec for card size <= 2GB */
            u32R_LEN = ((au32Buffer[1] & 0x000f0000U) >> 16U);
            u32C_Size = (((au32Buffer[1] & 0x000003ffU) << 2U) |
                         ((au32Buffer[2] & 0xc0000000U) >> 30U));
            u32MULT = ((au32Buffer[2] & 0x00038000U) >> 15U);
            u32Size = ((u32C_Size + 1U) * (1U << (u32MULT + 2U)) * (1U << u32R_LEN));

            pSD->diskSize = (u32Size / 1024U);
            pSD->totalSectorN = (u32Size / 512U);
        }
    }
    else
    {
        if ((au32Buffer[0] & 0xc0000000U) != 0x0U)
        {
            u32C_Size = (((au32Buffer[1] & 0x0000003fU) << 16U) |
                         ((au32Buffer[2] & 0xffff0000U) >> 16U));
            u32Size = (u32C_Size + 1U) * 512U;  /* Kbytes */

            pSD->diskSize = u32Size;
            pSD->totalSectorN = (u32Size << 1U);
        }
        else
        {
            u32R_LEN = ((au32Buffer[1] & 0x000f0000U) >> 16U);
            u32C_Size = (((au32Buffer[1] & 0x000003ffU) << 2U) |
                         ((au32Buffer[2] & 0xc0000000U) >> 30U));
            u32MULT = ((au32Buffer[2] & 0x00038000U) >> 15U);
            u32Size = (u32C_Size + 1U) * (1U << (u32MULT + 2U)) * (1U << u32R_LEN);

            pSD->diskSize = u32Size / 1024U;
            pSD->totalSectorN = u32Size / 512U;
        }
    }

    pSD->sectorSize = (int)512;
}

static uint32_t SDH_ResetCard(SDH_T *sdh)
{
    SDH_INFO_T *pSD;
    uint32_t i32TimeOutCount;

    sdh->GINTEN = 0U;
    sdh->CTL &= ~SDH_CTL_SDNWR_Msk;
    sdh->CTL |= ((uint32_t)0x09U << SDH_CTL_SDNWR_Pos);    /* set SDNWR = 9 */
    sdh->CTL &= ~SDH_CTL_BLKCNT_Msk;
    sdh->CTL |= ((uint32_t)0x01U << SDH_CTL_BLKCNT_Pos);   /* set BLKCNT = 1 */
    sdh->CTL &= ~SDH_CTL_DBW_Msk;               /* SD 1-bit data bus */

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    /* set the clock to 300KHz */
    SDH_Set_clock(sdh, 300U);

    /* power ON 74 clock */
    sdh->CTL |= SDH_CTL_CLK74OEN_Msk;

    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK74OEN_Msk) == SDH_CTL_CLK74OEN_Msk)
    {
        if (--i32TimeOutCount == 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            return SDH_ERR_TIMEOUT;
        }
    }

    return SDH_SDCommand(sdh, 0U, 0U);
}

/** @endcond HIDDEN_SYMBOLS */

/**
 *  @brief  This function use to reset SD function and select card detection source and pin.
 *
 *  @param[in]  sdh Select SDH0 or SDH1.
 *  @param[in]  u32CardDetSrc Select card detection pin from GPIO or DAT3 pin.
 *                          - \ref CardDetect_From_GPIO
 *                          - \ref CardDetect_From_DAT3
 *
 *  @return None
 */
void SDH_Open(SDH_T *sdh, uint32_t u32CardDetSrc)
{
    int32_t i32TimeOutCount;
    volatile int u32i;
    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    sdh->DMACTL = SDH_DMACTL_DMARST_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->DMACTL & SDH_DMACTL_DMARST_Msk) == SDH_DMACTL_DMARST_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    sdh->DMACTL = SDH_DMACTL_DMAEN_Msk;

    sdh->GCTL = SDH_GCTL_GCTLRST_Msk | SDH_GCTL_SDEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->GCTL & SDH_GCTL_GCTLRST_Msk) == SDH_GCTL_GCTLRST_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (sdh == SDH0)
    {
        NVIC_EnableIRQ(SDH0_IRQn);
        (void)memset(&SD0, 0, sizeof(SDH_INFO_T));
        SD0.dmabuf = _SDH0_ucSDHCBuffer;
    }
    else if (sdh == SDH1)
    {
        NVIC_EnableIRQ(SDH1_IRQn);
        (void)memset(&SD1, 0, sizeof(SDH_INFO_T));
        SD1.dmabuf = _SDH1_ucSDHCBuffer;
    }
    else
    {
    }

    sdh->GCTL = SDH_GCTL_SDEN_Msk;

    if ((u32CardDetSrc & CardDetect_From_DAT3) == CardDetect_From_DAT3)
    {
        sdh->INTEN &= ~SDH_INTEN_CDSRC_Msk;
    }
    else
    {
        sdh->INTEN |= SDH_INTEN_CDSRC_Msk;
    }

    for (u32i = 0; u32i < 0x100; u32i++) {}

    sdh->INTSTS = SDH_INTSTS_CDIF_Msk;

    if ((u32CardDetSrc & CardDetect_From_DAT3) == CardDetect_From_DAT3)
    {
        /* Use polling mode. */
        sdh->INTEN &= ~SDH_INTEN_CDIEN_Msk;
    }
    else
    {
        sdh->INTEN |= SDH_INTEN_CDIEN_Msk;
    }

    sdh->CTL |= SDH_CTL_CTLRST_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CTLRST_Msk) == SDH_CTL_CTLRST_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if ((u32CardDetSrc & CardDetect_From_DAT3) == CardDetect_From_DAT3)
    {
        /* Forcefully reset the SD card state to idle to prevent DAT3 voltage in an unexpected level due to transmission terminated. */
        (void)SDH_ResetCard(sdh);
    }
}

void SDH_Close(SDH_T *sdh)
{
    sdh->GCTL &= ~(SDH_GCTL_GCTLRST_Msk | SDH_GCTL_SDEN_Msk);

    if (sdh == SDH0)
    {
        NVIC_DisableIRQ(SDH0_IRQn);
    }
    else if (sdh == SDH1)
    {
        NVIC_DisableIRQ(SDH1_IRQn);
    }
    else
    {
    }
}

/**
 *  @brief  This function use to initial SD card.
 *
 *  @param[in]    sdh    Select SDH0 or SDH1.
 *
 *  @return None
 *
 *  @details This function is used to initial SD card.
 *           SD initial state needs 400KHz clock output, driver will use HIRC for SD initial clock source.
 *           And then switch back to the user's setting.
 */
uint32_t SDH_Probe(SDH_T *sdh)
{
    uint32_t u32Val;

    sdh->GINTEN = 0U;
    sdh->CTL &= ~SDH_CTL_SDNWR_Msk;
    sdh->CTL |= ((uint32_t)0x09U << SDH_CTL_SDNWR_Pos);    /* set SDNWR = 9 */
    sdh->CTL &= ~SDH_CTL_BLKCNT_Msk;
    sdh->CTL |= ((uint32_t)0x01U << SDH_CTL_BLKCNT_Pos);   /* set BLKCNT = 1 */
    sdh->CTL &= ~SDH_CTL_DBW_Msk;               /* SD 1-bit data bus */

    if (!(SDH_CardDetection(sdh)))
    {
        return SDH_NO_SD_CARD;
    }

    u32Val = SDH_Init(sdh);

    if (u32Val != 0U)
    {
        return u32Val;
    }

    /* divider */
    if ((SD0.CardType == SDH_TYPE_MMC) || (SD1.CardType == SDH_TYPE_MMC))
    {
        SDH_Set_clock(sdh, MMC_FREQ);
    }
    else
    {
        SDH_Set_clock(sdh, SD_FREQ);
    }

    SDH_Get_SD_info(sdh);

    u32Val = SDH_SelectCardType(sdh);

    if (u32Val != 0U)
    {
        return u32Val;
    }

    return 0U;
}

static uint32_t _SDH_Read(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t volatile u32BIsSendCmd = FALSE;
    uint32_t volatile u32Reg;
    uint32_t volatile i;
    uint32_t volatile u32Loop;
    uint32_t volatile u32Status;
    uint32_t blksize = SDH_BLOCK_SIZE;
    int32_t i32TimeOutCount;

    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    if (u32SecCount == 0U)
    {
        return SDH_SELECT_ERROR;
    }

    u32Status = SDH_SDCmdAndRsp(sdh, 7U, pSD->RCA, 0U);

    if (u32Status  != Successful)
    {
        return u32Status;
    }

    u32Status = SDH_CheckRB(sdh);

    if (u32Status != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    sdh->BLEN = blksize - 1U;       /* the actual byte count is equal to (SDBLEN+1) */

    if ((pSD->CardType == SDH_TYPE_SD_HIGH) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        sdh->CMDARG = u32StartSec;
    }
    else
    {
        sdh->CMDARG = u32StartSec * blksize;
    }

    sdh->DMASA = (uint32_t)pu8BufAddr;

    u32Loop = u32SecCount / 255U;

    for (i = 0U; i < u32Loop; i++)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = sdh->CTL & ~SDH_CTL_CMDCODE_Msk;
        u32Reg = u32Reg | 0xff0000U;   /* set BLK_CNT to 255 */

        if (u32BIsSendCmd == FALSE)
        {
            sdh->CTL = (u32Reg |
                        ((uint32_t)18U << 8U) |
                        (SDH_CTL_COEN_Msk |
                         SDH_CTL_RIEN_Msk |
                         SDH_CTL_DIEN_Msk));
            u32BIsSendCmd = TRUE;
        }
        else
        {
            sdh->CTL = u32Reg | SDH_CTL_DIEN_Msk;
        }

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while (!pSD->DataReadyFlag)
        {
            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) != SDH_INTSTS_CRC7_Msk)      /* check CRC7 */
        {
            return SDH_CRC7_ERROR;
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRC16_Msk) != SDH_INTSTS_CRC16_Msk)     /* check CRC16 */
        {
            return SDH_CRC16_ERROR;
        }
    }

    u32Loop = u32SecCount % 255U;

    if (u32Loop != 0U)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = sdh->CTL & (~SDH_CTL_CMDCODE_Msk);
        u32Reg = u32Reg & (~SDH_CTL_BLKCNT_Msk);
        u32Reg |= (u32Loop << 16);    /* setup SDCR_BLKCNT */

        if (u32BIsSendCmd == FALSE)
        {
            sdh->CTL = (u32Reg |
                        ((uint32_t)18U << 8) |
                        (SDH_CTL_COEN_Msk |
                         SDH_CTL_RIEN_Msk |
                         SDH_CTL_DIEN_Msk));
            u32BIsSendCmd = TRUE;
        }
        else
        {
            sdh->CTL = u32Reg | SDH_CTL_DIEN_Msk;
        }

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while (!pSD->DataReadyFlag)
        {
            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) != SDH_INTSTS_CRC7_Msk)      /* check CRC7 */
        {
            return SDH_CRC7_ERROR;
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRC16_Msk) != SDH_INTSTS_CRC16_Msk)     /* check CRC16 */
        {
            return SDH_CRC16_ERROR;
        }
    }

    if (SDH_SDCmdAndRsp(sdh, 12U, 0U, 0U))      /* stop command */
    {
        return SDH_CRC7_ERROR;
    }

    u32Status = SDH_CheckRB(sdh);

    if (u32Status != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    (void)SDH_SDCommand(sdh, 7U, 0U);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return Successful;
}

/**
 *  @brief  This function use to read data from SD card.
 *
 *  @param[in]     sdh           Select SDH0 or SDH1.
 *  @param[out]    pu8BufAddr    The buffer to receive the data from SD card.
 *  @param[in]     u32StartSec   The start read sector address.
 *  @param[in]     u32SecCount   The the read sector number of data
 *
 *  @return   \ref SDH_SELECT_ERROR : u32SecCount is zero.
 *            \ref SDH_NO_SD_CARD : SD card be removed.
 *            \ref SDH_CRC_ERROR : CRC error happen.
 *            \ref SDH_CRC7_ERROR : CRC7 error happen.
 *            \ref Successful : Read data from SD card success.
 */
uint32_t SDH_Read(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t u32DrvRet;

    /* Check for alignment to word or cache line boundaries. */
    if ((uintptr_t)pu8BufAddr % (uintptr_t)DEF_ALIGNED_VALUE)
    {
        uint8_t *pu8NCBuffer;
        uint32_t u32i = 0;

        if (sdh == SDH0)
        {
            pu8NCBuffer = _SDH0_ucSDHCBuffer;
        }
        else if (sdh == SDH1)
        {
            pu8NCBuffer = _SDH1_ucSDHCBuffer;
        }
        else
        {
            return Fail;
        }

        while (u32i < u32SecCount)
        {
            /* Read data from SD card to the temporary buffer buffer. */
            u32DrvRet = _SDH_Read(sdh, pu8NCBuffer, u32StartSec + u32i, 1);

            if (u32DrvRet != Successful)
            {
                return u32DrvRet;
            }

            /* Copy temporary buffer to user-data buffer after reading. */
            (void)memcpy(&pu8BufAddr[(u32i * SDH_BLOCK_SIZE)], pu8NCBuffer, SDH_BLOCK_SIZE);

            u32i++;
        }
    }
    else
    {
        /* Read data from SD card to the destination buffer. */
        u32DrvRet = _SDH_Read(sdh, pu8BufAddr, u32StartSec, u32SecCount);

        if (u32DrvRet != Successful)
        {
            return u32DrvRet;
        }

#if (NVT_DCACHE_ON == 1)
        /* Invalidate data cache for data coherence */
        int32_t i32Size = (int32_t)u32SecCount * (int32_t)SDH_BLOCK_SIZE;  /* 保證在 int32_t 範圍內 */

        SCB_InvalidateDCache_by_Addr((void *)pu8BufAddr, i32Size);
#endif
    }

    return Successful;
}

static uint32_t _SDH_Write(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t volatile u32BIsSendCmd = FALSE;
    uint32_t volatile u32Reg;
    uint32_t volatile u32i;
    uint32_t volatile u32Loop;
    uint32_t volatile u32Status;
    int32_t i32TimeOutCount;

    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    if (u32SecCount == 0U)
    {
        return SDH_SELECT_ERROR;
    }

    u32Status = SDH_SDCmdAndRsp(sdh, 7U, pSD->RCA, 0U);

    if (u32Status != Successful)
    {
        return u32Status;
    }

    u32Status = SDH_CheckRB(sdh);

    if (u32Status != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    /* According to SD Spec v2.0, the write CMD block size MUST be 512, and the start address MUST be 512*n. */
    sdh->BLEN = SDH_BLOCK_SIZE - 1U;

    if ((pSD->CardType == SDH_TYPE_SD_HIGH) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        sdh->CMDARG = u32StartSec;
    }
    else
    {
        sdh->CMDARG = u32StartSec * SDH_BLOCK_SIZE;  /* set start address for SD CMD */
    }

    sdh->DMASA = (uint32_t)pu8BufAddr;
    u32Loop = u32SecCount / 255U;   /* the maximum block count is 0xFF=255 for register SDCR[BLK_CNT] */

    for (u32i = 0U; u32i < u32Loop; u32i++)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = (sdh->CTL & 0xff00c080U);
        u32Reg = u32Reg | 0xff0000U;   /* set BLK_CNT to 0xFF=255 */

        if (!u32BIsSendCmd)
        {
            sdh->CTL = (u32Reg |
                        ((uint32_t)25U << 8U) |
                        (SDH_CTL_COEN_Msk |
                         SDH_CTL_RIEN_Msk |
                         SDH_CTL_DOEN_Msk));
            u32BIsSendCmd = TRUE;
        }
        else
        {
            sdh->CTL = u32Reg | SDH_CTL_DOEN_Msk;
        }

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while (!pSD->DataReadyFlag)
        {
            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRCIF_Msk) != 0U)
        {
            sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
            return SDH_CRC_ERROR;
        }
    }

    u32Loop = u32SecCount % 255U;

    if (u32Loop != 0U)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = (sdh->CTL & 0xff00c080U) | (u32Loop << 16U);

        if (!u32BIsSendCmd)
        {
            sdh->CTL = (u32Reg |
                        ((uint32_t)25U << 8U) |
                        (SDH_CTL_COEN_Msk |
                         SDH_CTL_RIEN_Msk |
                         SDH_CTL_DOEN_Msk));
            u32BIsSendCmd = TRUE;
        }
        else
        {
            sdh->CTL = u32Reg | SDH_CTL_DOEN_Msk;
        }

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while (!pSD->DataReadyFlag)
        {
            if (pSD->IsCardInsert == FALSE)
            {
                return SDH_NO_SD_CARD;
            }

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRCIF_Msk) != 0U)
        {
            sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
            return SDH_CRC_ERROR;
        }
    }

    sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;

    if (SDH_SDCmdAndRsp(sdh, 12U, 0U, 0U))      /* stop command */
    {
        return SDH_CRC7_ERROR;
    }

    u32Status = SDH_CheckRB(sdh);

    if (u32Status != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    (void)SDH_SDCommand(sdh, 7U, 0U);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
    {
        return Fail;
    }

    return Successful;
}

/**
 *  @brief  This function use to write data to SD card.
 *
 *  @param[in]    sdh           Select SDH0 or SDH1.
 *  @param[in]    pu8BufAddr    The buffer to send the data to SD card.
 *  @param[in]    u32StartSec   The start write sector address.
 *  @param[in]    u32SecCount   The the write sector number of data.
 *
 *  @return   \ref SDH_SELECT_ERROR : u32SecCount is zero.
 *            \ref SDH_NO_SD_CARD : SD card be removed.
 *            \ref SDH_CRC_ERROR : CRC error happen.
 *            \ref SDH_CRC7_ERROR : CRC7 error happen.
 *            \ref Successful : Write data to SD card success.
 */
uint32_t SDH_Write(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t u32DrvRet;

    /* Check for alignment to word or cache line boundaries. */
    if ((uintptr_t)pu8BufAddr % (uintptr_t)DEF_ALIGNED_VALUE)
    {
        uint32_t u32i = 0;
        uint8_t *puNCBuffer;

        if (sdh == SDH0)
        {
            puNCBuffer = _SDH0_ucSDHCBuffer;
        }
        else if (sdh == SDH1)
        {
            puNCBuffer = _SDH1_ucSDHCBuffer;
        }
        else
        {
            return Fail;
        }

        while (u32i < u32SecCount)
        {
            /* Copy user data to the temporary buffer to prepare for writing to the SD card. */
            (void)memcpy(puNCBuffer, &pu8BufAddr[(u32i * SDH_BLOCK_SIZE)], SDH_BLOCK_SIZE);

            /* Flush the data cache for the temporary buffer before writing to the SD card.
             * This ensures that all cached data is written to memory before the write operation. */
            u32DrvRet = _SDH_Write(sdh, puNCBuffer, u32StartSec + u32i, 1);

            if (u32DrvRet != Successful)
            {
                return u32DrvRet;
            }

            u32i++;
        }
    }
    else
    {
#if (NVT_DCACHE_ON == 1)
        int32_t i32Size = (int32_t)u32SecCount * (int32_t)SDH_BLOCK_SIZE;

        /* Flush specific data cache address before writing to the SD card */
        SCB_CleanDCache_by_Addr((void *)pu8BufAddr, i32Size);
#endif

        /* Write data from the destination buffer to specific sector address. */
        u32DrvRet = _SDH_Write(sdh, pu8BufAddr, u32StartSec, u32SecCount);

        if (u32DrvRet != Successful)
        {
            return u32DrvRet;
        }
    }

    return Successful;
}

/** @} end of group SDH_EXPORTED_FUNCTIONS */
/** @} end of group SDH_Driver */
/** @} end of group Standard_Driver */
