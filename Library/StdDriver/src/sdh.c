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

static uint32_t _SDH0_ReferenceClock, _SDH1_ReferenceClock;

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
SDH_INFO_T SD0, SD1;

//------------------------------------------------------------------------------
void *SDH_GetSDH0Buffer(void)
{
    return &_SDH0_ucSDHCBuffer[0];
}

void *SDH_GetSDH1Buffer(void)
{
    return &_SDH1_ucSDHCBuffer[0];
}

void *SDH_GetSDInfoMsg(SDH_T *sdh)
{
    if (sdh == SDH0)
    {
        return &SD0;
    }
    else if (sdh == SDH1)
    {
        return &SD1;
    }

    return NULL;
}

int32_t SDH_CheckRB(SDH_T *sdh)
{
    int32_t i32TimeOutCount1, i32TimeOutCount2;
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

    while (1)
    {
        sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
        i32TimeOutCount1 = (int32_t)SDH_TIMEOUT_CNT;

        while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
        {
            if (--i32TimeOutCount1 == 0)
            {
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_DAT0STS_Msk) == SDH_INTSTS_DAT0STS_Msk)
        {
            break;
        }

        if (--i32TimeOutCount2 == 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

    return Successful;
}

uint32_t SDH_SDCommand(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg)
{
    volatile uint32_t u32Buf, u32Val = 0ul;
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
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8ul) | (SDH_CTL_COEN_Msk);
    sdh->CTL = u32Buf;

    while ((sdh->CTL & SDH_CTL_COEN_Msk) == SDH_CTL_COEN_Msk)
    {
        if (pSD->IsCardInsert == 0ul)
        {
            u32Val = SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

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
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8ul) | (SDH_CTL_COEN_Msk | SDH_CTL_RIEN_Msk);
    sdh->CTL = u32Buf;

    if (u32NtickCount > 0ul)
    {
        while ((sdh->CTL & SDH_CTL_RIEN_Msk) == SDH_CTL_RIEN_Msk)
        {
            if (u32NtickCount-- == 0ul)
            {
                sdh->CTL |= SDH_CTL_CTLRST_Msk; /* reset SD engine */
                return 2ul;
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
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

    if (pSD->R7Flag)
    {
        uint32_t tmp0 = 0ul, tmp1 = 0ul;
        tmp1 = sdh->RESP1 & 0xfful;
        tmp0 = sdh->RESP0 & 0xful;

        if ((tmp1 != 0x55ul) && (tmp0 != 0x01ul))
        {
            pSD->R7Flag = 0ul;
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
        pSD->R3Flag = 0ul;
        sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
        return Successful;
    }
}

static uint32_t SDH_Swap32(uint32_t u32Val)
{
    uint32_t u32Buf;

    u32Buf = u32Val;
    u32Val <<= 24;
    u32Val |= (u32Buf << 8) & 0xff0000ul;
    u32Val |= (u32Buf >> 8) & 0xff00ul;
    u32Val |= (u32Buf >> 24) & 0xfful;

    return u32Val;
}

/* Get 16 bytes CID or CSD */
uint32_t SDH_SDCmdAndRsp2(SDH_T *sdh, uint32_t u32Cmd, uint32_t u32Arg, uint32_t pu32R2ptr[])
{
    uint32_t u32i, u32Buf;
    uint32_t tmpBuf[5];
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
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8) | (SDH_CTL_COEN_Msk | SDH_CTL_R2EN_Msk);
    sdh->CTL = u32Buf;

    while ((sdh->CTL & SDH_CTL_R2EN_Msk) == SDH_CTL_R2EN_Msk)
    {
        if (pSD->IsCardInsert == FALSE)
        {
            return SDH_NO_SD_CARD;
        }

        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if ((sdh->INTSTS & SDH_INTSTS_CRC7_Msk) == SDH_INTSTS_CRC7_Msk)
    {
        for (u32i = 0ul; u32i < 5ul; u32i++)
        {
            tmpBuf[u32i] = SDH_Swap32(sdh->FB[u32i]);
        }

        for (u32i = 0ul; u32i < 4ul; u32i++)
        {
            pu32R2ptr[u32i] = ((tmpBuf[u32i] & 0x00fffffful) << 8) | ((tmpBuf[u32i + 1ul] & 0xff000000ul) >> 24);
        }
    }
    else
    {
        return SDH_CRC7_ERROR;
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

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
    u32Buf = (sdh->CTL & (~SDH_CTL_CMDCODE_Msk)) | (u32Cmd << 8ul) |
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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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

    return 0ul;
}

/* there are 8 bits for divider0, maximum is 256 */
#define SDH_CLK_DIV0_MAX     256ul

void SDH_Set_clock(SDH_T *sdh, uint32_t u32SD_clk_khz)
{
    uint32_t u32Rate, u32Div1;
    static uint32_t u32SD_ClkSrc = 0ul, u32SD_PwrCtl = 0ul;

    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    /* initial state, clock source use HIRC */
    if (u32SD_clk_khz <= 400ul)
    {
        u32SD_PwrCtl = CLK->SRCCTL;

        if ((u32SD_PwrCtl & CLK_SRCCTL_HIRCEN_Msk) != 0x4ul)
        {
            CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;
        }

        if (sdh == SDH0)
        {
            u32SD_ClkSrc = (CLK->SDHSEL & CLK_SDHSEL_SDH0SEL_Msk);
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH0SEL_Msk) | CLK_SDHSEL_SDH0SEL_HIRC;
            _SDH0_ReferenceClock = (__HIRC / 1000ul);
        }
        else
        {
            u32SD_ClkSrc = (CLK->SDHSEL & CLK_SDHSEL_SDH1SEL_Msk);
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH1SEL_Msk) | CLK_SDHSEL_SDH1SEL_HIRC;
            _SDH1_ReferenceClock = (__HIRC / 1000ul);
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
                    _SDH0_ReferenceClock = ((CLK_GetAPLL1ClockFreq() >> 1) / 1000ul);
                    break;

                case CLK_SDHSEL_SDH0SEL_HCLK0:
                    _SDH0_ReferenceClock = (CLK_GetHCLK0Freq() / 1000ul);
                    break;

                case CLK_SDHSEL_SDH0SEL_HIRC:
                    _SDH0_ReferenceClock = (__HIRC / 1000ul);
                    break;

                case CLK_SDHSEL_SDH0SEL_HIRC48M_DIV4:
                    _SDH0_ReferenceClock = ((__HIRC48M / 1000ul) / 4);
                    break;

                case CLK_SDHSEL_SDH0SEL_HXT:
                default:
                    _SDH0_ReferenceClock = (CLK_GetHXTFreq() / 1000ul);
                    break;
            }
        }
        else
        {
            CLK->SDHSEL = (CLK->SDHSEL & ~CLK_SDHSEL_SDH1SEL_Msk) | u32SD_ClkSrc;

            switch (u32SD_ClkSrc)
            {
                case CLK_SDHSEL_SDH1SEL_APLL1_DIV2:
                    _SDH1_ReferenceClock = ((CLK_GetAPLL1ClockFreq() >> 1) / 1000ul);
                    break;

                case CLK_SDHSEL_SDH1SEL_HCLK0:
                    _SDH1_ReferenceClock = (CLK_GetHCLK0Freq() / 1000ul);
                    break;

                case CLK_SDHSEL_SDH1SEL_HIRC:
                    _SDH1_ReferenceClock = (__HIRC / 1000ul);
                    break;

                case CLK_SDHSEL_SDH1SEL_HIRC48M_DIV4:
                    _SDH1_ReferenceClock = ((__HIRC48M / 1000ul) / 4);
                    break;

                case CLK_SDHSEL_SDH1SEL_HXT:
                default:
                    _SDH1_ReferenceClock = (CLK_GetHXTFreq() / 1000ul);
                    break;
            }
        }

        if (u32SD_clk_khz >= 50000ul)
        {
            u32SD_clk_khz = 50000ul;
        }
    }

    if (sdh == SDH0)
    {
        u32Rate = _SDH0_ReferenceClock / u32SD_clk_khz;

        /* choose slower clock if system clock cannot divisible by wanted clock */
        if ((_SDH0_ReferenceClock % u32SD_clk_khz) != 0ul)
        {
            u32Rate++;
        }
    }
    else
    {
        u32Rate = _SDH1_ReferenceClock / u32SD_clk_khz;

        /* choose slower clock if system clock cannot divisible by wanted clock */
        if ((_SDH1_ReferenceClock % u32SD_clk_khz) != 0ul)
        {
            u32Rate++;
        }
    }

    if (u32Rate >= SDH_CLK_DIV0_MAX)
    {
        u32Rate = SDH_CLK_DIV0_MAX;
    }

    /*--- calculate the second divider CLKDIV0[SDHOST_N]*/
    u32Div1 = (u32Rate - 1ul) & 0xFFul;

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
    volatile uint32_t u32i;
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
        sdh->CTL |= SDH_CTL_CLKKEEP_Msk;

        for (u32i = 0ul; u32i < 5000ul; u32i++)
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

    return u32Val;
}

static uint32_t SDH_Init(SDH_T *sdh)
{
    volatile uint32_t u32i = 0;
    uint32_t u32Status;
    uint32_t u32Resp;
    uint32_t au32CIDBuffer[4];
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
    SDH_Set_clock(sdh, 300ul);

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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    SDH_SDCommand(sdh, 0ul, 0ul);        /* reset all cards */

    for (u32i = 0; u32i < 0x1000ul; u32i++)
    {
    }

    /* initial SDHC */
    pSD->R7Flag = 1ul;
    u32CmdTimeOut = 0xFFFFFul;

    u32i = SDH_SDCmdAndRsp(sdh, 8ul, 0x00000155ul, u32CmdTimeOut);

    if (u32i == Successful)
    {
        /* SD 2.0 */
        SDH_SDCmdAndRsp(sdh, 55ul, 0x00ul, u32CmdTimeOut);
        pSD->R3Flag = 1ul;
        SDH_SDCmdAndRsp(sdh, 41ul, 0x40ff8000ul, u32CmdTimeOut); /* 2.7v-3.6v */
        u32Resp = sdh->RESP0;

        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while ((u32Resp & 0x00800000ul) != 0x00800000ul)        /* check if card is ready */
        {
            SDH_SDCmdAndRsp(sdh, 55ul, 0x00ul, u32CmdTimeOut);
            pSD->R3Flag = 1ul;
            SDH_SDCmdAndRsp(sdh, 41ul, 0x40ff8000ul, u32CmdTimeOut); /* 3.0v-3.4v */
            u32Resp = sdh->RESP0;

            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((u32Resp & 0x00400000ul) == 0x00400000ul)
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
        SDH_SDCommand(sdh, 0ul, 0ul);        /* reset all cards */

        for (u32i = 0; u32i < 0x100ul; u32i++)
        {
        }

        u32i = SDH_SDCmdAndRsp(sdh, 55ul, 0x00ul, u32CmdTimeOut);

        if (u32i == 2ul)     /* MMC memory */
        {
            SDH_SDCommand(sdh, 0ul, 0ul);        /* reset */

            for (u32i = 0; u32i < 0x100ul; u32i++)
            {
            }

            pSD->R3Flag = 1ul;

            if (SDH_SDCmdAndRsp(sdh, 1ul, 0x40ff8000ul, u32CmdTimeOut) != 2ul)    /* eMMC memory */
            {
                u32Resp = sdh->RESP0;
                i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

                while ((u32Resp & 0x00800000ul) != 0x00800000ul)
                {
                    /* check if card is ready */
                    pSD->R3Flag = 1ul;

                    SDH_SDCmdAndRsp(sdh, 1ul, 0x40ff8000ul, u32CmdTimeOut);      /* high voltage */
                    u32Resp = sdh->RESP0;

                    if (--i32TimeOutCount <= 0)
                    {
                        pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                        break;
                    }
                }

                if ((u32Resp & 0x00400000ul) == 0x00400000ul)
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
        else if (u32i == 0ul)     /* SD Memory */
        {
            pSD->R3Flag = 1ul;
            SDH_SDCmdAndRsp(sdh, 41ul, 0x00ff8000ul, u32CmdTimeOut); /* 3.0v-3.4v */
            u32Resp = sdh->RESP0;
            i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

            while ((u32Resp & 0x00800000ul) != 0x00800000ul)        /* check if card is ready */
            {
                SDH_SDCmdAndRsp(sdh, 55ul, 0x00ul, u32CmdTimeOut);
                pSD->R3Flag = 1ul;
                SDH_SDCmdAndRsp(sdh, 41ul, 0x00ff8000ul, u32CmdTimeOut); /* 3.0v-3.4v */
                u32Resp = sdh->RESP0;

                if (--i32TimeOutCount <= 0)
                {
                    pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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
        SDH_SDCmdAndRsp2(sdh, 2ul, 0x00ul, au32CIDBuffer);

        if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
        {
            if ((u32Status = SDH_SDCmdAndRsp(sdh, 3ul, 0x10000ul, 0ul)) != Successful)     /* set RCA */
            {
                return u32Status;
            }

            pSD->RCA = 0x10000ul;
        }
        else
        {
            if ((u32Status = SDH_SDCmdAndRsp(sdh, 3ul, 0x00ul, 0ul)) != Successful)       /* get RCA */
            {
                return u32Status;
            }
            else
            {
                pSD->RCA = (sdh->RESP0 << 8) & 0xffff0000;
            }
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

    return Successful;
}

uint32_t SDH_SwitchToHighSpeed(SDH_T *sdh, SDH_INFO_T *pSD)
{
    uint32_t volatile u32Status = 0ul;
    uint16_t current_comsumption, busy_status0;
    int32_t i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    pSD->i32ErrCode = 0;

    sdh->DMASA = (uint32_t)pSD->dmabuf;
    sdh->BLEN = 63ul;

    if ((u32Status = SDH_SDCmdAndRspDataIn(sdh, 6ul, 0x00ffff01ul)) != Successful)
    {
        return Fail;
    }

    current_comsumption = (uint16_t)((*pSD->dmabuf) << 8);
    current_comsumption |= (uint16_t)(*(pSD->dmabuf + 1));

    if (!current_comsumption)
    {
        return Fail;
    }

    busy_status0 = (uint16_t)((*(pSD->dmabuf + 28)) << 8);
    busy_status0 |= (uint16_t)(*(pSD->dmabuf + 29));

    if (!busy_status0)   /* function ready */
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 63ul;    /* 512 bit */

        if ((u32Status = SDH_SDCmdAndRspDataIn(sdh, 6ul, 0x80ffff01ul)) != Successful)
        {
            return Fail;
        }

        /* function change timing: 8 clocks */
        sdh->CTL |= SDH_CTL_CLK8OEN_Msk;

        while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
        {
            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if (pSD->i32ErrCode != 0)
            return Fail;

        current_comsumption = (uint16_t)((*pSD->dmabuf) << 8);
        current_comsumption |= (uint16_t)(*(pSD->dmabuf + 1));

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
    uint32_t volatile u32Status = 0ul;
    uint32_t u32Param;
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

    if ((u32Status = SDH_SDCmdAndRsp(sdh, 7ul, pSD->RCA, 0ul)) != Successful)
    {
        return u32Status;
    }

    if (SDH_CheckRB(sdh) != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    /* if SD card set 4bit */
    if (pSD->CardType == SDH_TYPE_SD_HIGH)
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 0x07ul;  /* 64 bit */
        sdh->DMACTL |= SDH_DMACTL_DMARST_Msk;
        i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

        while ((sdh->DMACTL & SDH_DMACTL_DMARST_Msk) == 0x2)
        {
            if (--i32TimeOutCount <= 0)
            {
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 55ul, pSD->RCA, 0ul)) != Successful)
        {
            return u32Status;
        }

        if ((u32Status = SDH_SDCmdAndRspDataIn(sdh, 51ul, 0x00ul)) != Successful)
        {
            return u32Status;
        }

        if ((*pSD->dmabuf & 0xful) == 0x2ul)
        {
            u32Status = SDH_SwitchToHighSpeed(sdh, pSD);

            if (u32Status == Successful)
            {
                /* divider */
                SDH_Set_clock(sdh, SDHC_FREQ);
            }
        }

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 55ul, pSD->RCA, 0ul)) != Successful)
        {
            return u32Status;
        }

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 6ul, 0x02ul, 0ul)) != Successful)   /* set bus width */
        {
            return u32Status;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk;
    }
    else if (pSD->CardType == SDH_TYPE_SD_LOW)
    {
        sdh->DMASA = (uint32_t)pSD->dmabuf;
        sdh->BLEN = 0x07ul;

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 55ul, pSD->RCA, 0ul)) != Successful)
        {
            return u32Status;
        }

        if ((u32Status = SDH_SDCmdAndRspDataIn(sdh, 51ul, 0x00ul)) != Successful)
        {
            return u32Status;
        }

        /* set data bus width. ACMD6 for SD card, SDCR_DBW for host. */
        if ((u32Status = SDH_SDCmdAndRsp(sdh, 55ul, pSD->RCA, 0ul)) != Successful)
        {
            return u32Status;
        }

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 6ul, 0x02ul, 0ul)) != Successful)
        {
            return u32Status;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk;
    }
    else if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
    {

        if (pSD->CardType == SDH_TYPE_MMC)
        {
            sdh->CTL &= ~SDH_CTL_DBW_Msk;
        }

        /*--- sent CMD6 to MMC card to set bus width to 4 bits mode */
        /* set CMD6 argument Access field to 3, Index to 183, Value to 1 (4-bit mode) */
        u32Param = (3ul << 24) | (183ul << 16) | (1ul << 8);

        if ((u32Status = SDH_SDCmdAndRsp(sdh, 6ul, u32Param, 0ul)) != Successful)
        {
            return u32Status;
        }

        if (SDH_CheckRB(sdh) != Successful)
        {
            return SDH_ERR_TIMEOUT;
        }

        sdh->CTL |= SDH_CTL_DBW_Msk; /* set bus width to 4-bit mode for SD host controller */

    }

    if ((u32Status = SDH_SDCmdAndRsp(sdh, 16ul, SDH_BLOCK_SIZE, 0ul)) != Successful)
    {
        return u32Status;
    }

    sdh->BLEN = SDH_BLOCK_SIZE - 1ul;

    SDH_SDCommand(sdh, 7ul, 0ul);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

    sdh->INTEN |= SDH_INTEN_BLKDIEN_Msk;

    return Successful;
}

void SDH_Get_SD_info(SDH_T *sdh)
{
    unsigned int u32R_LEN, u32C_Size, u32MULT, u32Size;
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

    SDH_SDCmdAndRsp2(sdh, 9ul, pSD->RCA, au32Buffer);

    if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        /* for MMC/eMMC card */
        if ((au32Buffer[0] & 0xc0000000) == 0xc0000000)
        {
            /* CSD_STRUCTURE [127:126] is 3 */
            /* CSD version depend on EXT_CSD register in eMMC v4.4 for card size > 2GB */
            SDH_SDCmdAndRsp(sdh, 7ul, pSD->RCA, 0ul);

            //ptr = (uint8_t *)((uint32_t)_SDH_ucSDHCBuffer );
            sdh->DMASA = (uint32_t)pSD->dmabuf;
            sdh->BLEN = 511ul;  /* read 512 bytes for EXT_CSD */

            if (SDH_SDCmdAndRspDataIn(sdh, 8ul, 0x00ul) == Successful)
            {
                SDH_SDCommand(sdh, 7ul, 0ul);
                sdh->CTL |= SDH_CTL_CLK8OEN_Msk;

                while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
                {
                    if (--i32TimeOutCount <= 0)
                    {
                        pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                        break;
                    }
                }

                pSD->totalSectorN = (uint32_t)(*(pSD->dmabuf + 215)) << 24;
                pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 214)) << 16;
                pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 213)) << 8;
                pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 212));
                pSD->diskSize = pSD->totalSectorN / 2ul;
            }
        }
        else
        {
            /* CSD version v1.0/1.1/1.2 in eMMC v4.4 spec for card size <= 2GB */
            u32R_LEN = (au32Buffer[1] & 0x000f0000ul) >> 16;
            u32C_Size = ((au32Buffer[1] & 0x000003fful) << 2) | ((au32Buffer[2] & 0xc0000000ul) >> 30);
            u32MULT = (au32Buffer[2] & 0x00038000ul) >> 15;
            u32Size = (u32C_Size + 1ul) * (1ul << (u32MULT + 2ul)) * (1ul << u32R_LEN);

            pSD->diskSize = u32Size / 1024ul;
            pSD->totalSectorN = u32Size / 512ul;
        }
    }
    else
    {
        if ((au32Buffer[0] & 0xc0000000) != 0x0ul)
        {
            u32C_Size = ((au32Buffer[1] & 0x0000003ful) << 16) | ((au32Buffer[2] & 0xffff0000ul) >> 16);
            u32Size = (u32C_Size + 1ul) * 512ul;  /* Kbytes */

            pSD->diskSize = u32Size;
            pSD->totalSectorN = u32Size << 1;
        }
        else
        {
            u32R_LEN = (au32Buffer[1] & 0x000f0000ul) >> 16;
            u32C_Size = ((au32Buffer[1] & 0x000003fful) << 2) | ((au32Buffer[2] & 0xc0000000ul) >> 30);
            u32MULT = (au32Buffer[2] & 0x00038000ul) >> 15;
            u32Size = (u32C_Size + 1ul) * (1ul << (u32MULT + 2ul)) * (1ul << u32R_LEN);

            pSD->diskSize = u32Size / 1024ul;
            pSD->totalSectorN = u32Size / 512ul;
        }
    }

    pSD->sectorSize = (int)512;
}

static uint32_t SDH_ResetCard(SDH_T *sdh)
{
    SDH_INFO_T *pSD;
    uint32_t i32TimeOutCount;

    sdh->GINTEN = 0ul;
    sdh->CTL &= ~SDH_CTL_SDNWR_Msk;
    sdh->CTL |=  0x09ul << SDH_CTL_SDNWR_Pos;   /* set SDNWR = 9 */
    sdh->CTL &= ~SDH_CTL_BLKCNT_Msk;
    sdh->CTL |=  0x01ul << SDH_CTL_BLKCNT_Pos;  /* set BLKCNT = 1 */
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
    SDH_Set_clock(sdh, 300ul);

    /* power ON 74 clock */
    sdh->CTL |= SDH_CTL_CLK74OEN_Msk;

    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK74OEN_Msk) == SDH_CTL_CLK74OEN_Msk)
    {
        if (--i32TimeOutCount == 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            return SDH_ERR_TIMEOUT;
        }
    }

    return SDH_SDCommand(sdh, 0ul, 0ul);
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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (sdh == SDH0)
    {
        NVIC_EnableIRQ(SDH0_IRQn);
        memset(&SD0, 0, sizeof(SDH_INFO_T));
        SD0.dmabuf = _SDH0_ucSDHCBuffer;
    }
    else if (sdh == SDH1)
    {
        NVIC_EnableIRQ(SDH1_IRQn);
        memset(&SD1, 0, sizeof(SDH_INFO_T));
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

    for (u32i = 0; u32i < 0x100; u32i++);

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
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if ((u32CardDetSrc & CardDetect_From_DAT3) == CardDetect_From_DAT3)
    {
        /* Forcefully reset the SD card state to idle to prevent DAT3 voltage in an unexpected level due to transmission terminated. */
        SDH_ResetCard(sdh);
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

    sdh->GINTEN = 0ul;
    sdh->CTL &= ~SDH_CTL_SDNWR_Msk;
    sdh->CTL |=  0x09ul << SDH_CTL_SDNWR_Pos;   /* set SDNWR = 9 */
    sdh->CTL &= ~SDH_CTL_BLKCNT_Msk;
    sdh->CTL |=  0x01ul << SDH_CTL_BLKCNT_Pos;  /* set BLKCNT = 1 */
    sdh->CTL &= ~SDH_CTL_DBW_Msk;               /* SD 1-bit data bus */

    if (!(SDH_CardDetection(sdh)))
    {
        return SDH_NO_SD_CARD;
    }

    if ((u32Val = SDH_Init(sdh)) != 0ul)
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

    if ((u32Val = SDH_SelectCardType(sdh)) != 0ul)
    {
        return u32Val;
    }

    return 0ul;
}

static uint32_t _SDH_Read(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t volatile u32BIsSendCmd = FALSE;
    uint32_t volatile u32Reg;
    uint32_t volatile i, u32Loop, status;
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

    if (u32SecCount == 0ul)
    {
        return SDH_SELECT_ERROR;
    }

    if ((status = SDH_SDCmdAndRsp(sdh, 7ul, pSD->RCA, 0ul)) != Successful)
    {
        return status;
    }

    if (SDH_CheckRB(sdh) != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    sdh->BLEN = blksize - 1ul;       /* the actual byte count is equal to (SDBLEN+1) */

    if ((pSD->CardType == SDH_TYPE_SD_HIGH) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        sdh->CMDARG = u32StartSec;
    }
    else
    {
        sdh->CMDARG = u32StartSec * blksize;
    }

    sdh->DMASA = (uint32_t)pu8BufAddr;

    u32Loop = u32SecCount / 255ul;

    for (i = 0ul; i < u32Loop; i++)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = sdh->CTL & ~SDH_CTL_CMDCODE_Msk;
        u32Reg = u32Reg | 0xff0000ul;   /* set BLK_CNT to 255 */

        if (u32BIsSendCmd == FALSE)
        {
            sdh->CTL = u32Reg | (18ul << 8) | (SDH_CTL_COEN_Msk |
                                               SDH_CTL_RIEN_Msk |
                                               SDH_CTL_DIEN_Msk);
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
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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

    u32Loop = u32SecCount % 255ul;

    if (u32Loop != 0ul)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = sdh->CTL & (~SDH_CTL_CMDCODE_Msk);
        u32Reg = u32Reg & (~SDH_CTL_BLKCNT_Msk);
        u32Reg |= (u32Loop << 16);    /* setup SDCR_BLKCNT */

        if (u32BIsSendCmd == FALSE)
        {
            sdh->CTL = u32Reg | (18ul << 8) | (SDH_CTL_COEN_Msk |
                                               SDH_CTL_RIEN_Msk |
                                               SDH_CTL_DIEN_Msk);
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
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
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

    if (SDH_SDCmdAndRsp(sdh, 12ul, 0ul, 0ul))      /* stop command */
    {
        return SDH_CRC7_ERROR;
    }

    if (SDH_CheckRB(sdh) != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    SDH_SDCommand(sdh, 7ul, 0ul);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

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
    if ((uint32_t)pu8BufAddr % DEF_ALIGNED_VALUE)
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
                return u32DrvRet;

            /* Copy temporary buffer to user-data buffer after reading. */
            memcpy(pu8BufAddr + (u32i * SDH_BLOCK_SIZE), pu8NCBuffer, SDH_BLOCK_SIZE);

            u32i++;
        }
    }
    else
    {
        /* Read data from SD card to the destination buffer. */
        u32DrvRet = _SDH_Read(sdh, pu8BufAddr, u32StartSec, u32SecCount);

        if (u32DrvRet != Successful)
            return u32DrvRet;

#if (NVT_DCACHE_ON == 1)
        /* Invalidate data cache for data coherence */
        SCB_InvalidateDCache_by_Addr((void *)pu8BufAddr, (int32_t)(u32SecCount * SDH_BLOCK_SIZE));
#endif
    }

    return Successful;
}

static uint32_t _SDH_Write(SDH_T *sdh, uint8_t *pu8BufAddr, uint32_t u32StartSec, uint32_t u32SecCount)
{
    uint32_t volatile u32BIsSendCmd = FALSE;
    uint32_t volatile u32Reg;
    uint32_t volatile u32i, u32Loop, u32Status;
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

    if (u32SecCount == 0ul)
    {
        return SDH_SELECT_ERROR;
    }

    if ((u32Status = SDH_SDCmdAndRsp(sdh, 7ul, pSD->RCA, 0ul)) != Successful)
    {
        return u32Status;
    }

    if (SDH_CheckRB(sdh) != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    /* According to SD Spec v2.0, the write CMD block size MUST be 512, and the start address MUST be 512*n. */
    sdh->BLEN = SDH_BLOCK_SIZE - 1ul;

    if ((pSD->CardType == SDH_TYPE_SD_HIGH) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        sdh->CMDARG = u32StartSec;
    }
    else
    {
        sdh->CMDARG = u32StartSec * SDH_BLOCK_SIZE;  /* set start address for SD CMD */
    }

    sdh->DMASA = (uint32_t)pu8BufAddr;
    u32Loop = u32SecCount / 255ul;   /* the maximum block count is 0xFF=255 for register SDCR[BLK_CNT] */

    for (u32i = 0ul; u32i < u32Loop; u32i++)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = sdh->CTL & 0xff00c080;
        u32Reg = u32Reg | 0xff0000ul;   /* set BLK_CNT to 0xFF=255 */

        if (!u32BIsSendCmd)
        {
            sdh->CTL = u32Reg | (25ul << 8) | (SDH_CTL_COEN_Msk |
                                               SDH_CTL_RIEN_Msk |
                                               SDH_CTL_DOEN_Msk);
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
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRCIF_Msk) != 0ul)
        {
            sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
            return SDH_CRC_ERROR;
        }
    }

    u32Loop = u32SecCount % 255ul;

    if (u32Loop != 0ul)
    {
        pSD->DataReadyFlag = (uint8_t)FALSE;
        u32Reg = (sdh->CTL & 0xff00c080) | (u32Loop << 16);

        if (!u32BIsSendCmd)
        {
            sdh->CTL = u32Reg | (25ul << 8) | (SDH_CTL_COEN_Msk |
                                               SDH_CTL_RIEN_Msk |
                                               SDH_CTL_DOEN_Msk);
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
                pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
                break;
            }
        }

        if ((sdh->INTSTS & SDH_INTSTS_CRCIF_Msk) != 0ul)
        {
            sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;
            return SDH_CRC_ERROR;
        }
    }

    sdh->INTSTS = SDH_INTSTS_CRCIF_Msk;

    if (SDH_SDCmdAndRsp(sdh, 12ul, 0ul, 0ul))      /* stop command */
    {
        return SDH_CRC7_ERROR;
    }

    if (SDH_CheckRB(sdh) != Successful)
    {
        return SDH_ERR_TIMEOUT;
    }

    SDH_SDCommand(sdh, 7ul, 0ul);
    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
    i32TimeOutCount = (int32_t)SDH_TIMEOUT_CNT;

    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
    {
        if (--i32TimeOutCount <= 0)
        {
            pSD->i32ErrCode = (int32_t)SDH_ERR_TIMEOUT;
            break;
        }
    }

    if (pSD->i32ErrCode != 0)
        return Fail;

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
    if ((uint32_t)pu8BufAddr % DEF_ALIGNED_VALUE)
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
            memcpy(puNCBuffer, pu8BufAddr + (u32i * SDH_BLOCK_SIZE), SDH_BLOCK_SIZE);

            /* Flush the data cache for the temporary buffer before writing to the SD card.
             * This ensures that all cached data is written to memory before the write operation. */
            u32DrvRet = _SDH_Write(sdh, puNCBuffer, u32StartSec + u32i, 1);

            if (u32DrvRet != Successful)
                return u32DrvRet;

            u32i++;
        }
    }
    else
    {
#if (NVT_DCACHE_ON == 1)
        /* Flush specific data cache address before writing to the SD card */
        SCB_CleanDCache_by_Addr((void *)pu8BufAddr, (int32_t)(u32SecCount * SDH_BLOCK_SIZE));
#endif

        /* Write data from the destination buffer to specific sector address. */
        u32DrvRet = _SDH_Write(sdh, pu8BufAddr, u32StartSec, u32SecCount);

        if (u32DrvRet != Successful)
            return u32DrvRet;
    }

    return Successful;
}

/** @} end of group SDH_EXPORTED_FUNCTIONS */
/** @} end of group SDH_Driver */
/** @} end of group Standard_Driver */
