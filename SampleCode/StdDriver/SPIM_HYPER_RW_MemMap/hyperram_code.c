/**************************************************************************//**
 * @file     hyperram_code.c
 * @version  V1.03
 * @brief    Collect of sub-routines running on SPIM flash.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "hyperram_code.h"

//------------------------------------------------------------------------------
#define TRIM_PAT_SIZE               32 /* Trim size must be a multiple of 8. */

#define SPIM_HYPER_DIV              1

#define HYPERRAM_CSM_TIME           4000 /* ns */
#define HYPERRAM_RD_LTCY            7
#define HYPERRAM_WR_LTCY            7
#define HYPERRAM_CSHI_CYCLE         4
#define HYPER_RAM_RST_CNT           0xFF
#define VERIFY_ERASE_PATTERN        0x0000
#define CSMAXLT_CIPHER_OFF          21
#define CSMAXLT_CIPHER_ON           54

//------------------------------------------------------------------------------
/**
  * @brief      SPIM Default Config HyperBus Access Module Parameters.
  * @param      spim
  * @param      u32CSM    Refer to the Hyper Device Specific Chip Select Maximum (tCSM) timing parameters.
  *                       The reference Winbond HyperRAM is 4000ns.
  * @param      u32AcctRD Initial Read Access Time 1 ~ 0x1F, Default Set 0x04
  * @param      u32AcctWR Initial Write Access Time 1 ~ 0x1F, Default Set 0x04
  * @return     None.
  */
void SPIM_Hyper_DefaultConfig(SPIM_T *spim, uint32_t u32CSM, uint32_t u32AcctRD, uint32_t u32AcctWR)
{
    uint32_t u32CoreFreq = (CLK_GetSCLKFreq() / 1000000);
    float fFreq = (float)((float)1000 / (float)u32CoreFreq);
    uint32_t u32DIV = SPIM_HYPER_GET_CLKDIV(spim);
    uint32_t u32CipherEn = SPIM_HYPER_GET_CIPHER(spim);
    uint32_t u32CSMAXLT = (uint32_t)((u32CSM / fFreq) -
                                     (2 * 8 * u32DIV) -
                                     (((!u32CipherEn) == SPIM_HYPER_OP_ENABLE) ? CSMAXLT_CIPHER_ON : CSMAXLT_CIPHER_OFF));

    /* Chip Select Setup Time 3.5 HCLK */
    SPIM_HYPER_SET_CSST(spim, SPIM_HYPER_CSST_3_5_HCLK);

    /* Chip Select Hold Time 3.5 HCLK */
    SPIM_HYPER_SET_CSH(spim, SPIM_HYPER_CSH_3_5_HCLK);

    /* Chip Select High between Transaction as 2 HCLK cycles */
    SPIM_HYPER_SET_CSHI(spim, HYPERRAM_CSHI_CYCLE);

    /* Chip Select Masximum low time HCLK */
    SPIM_HYPER_SET_CSMAXLT(spim, u32CSMAXLT);

    /* Initial Device RESETN Low Time 255 */
    SPIM_HYPER_SET_RSTNLT(spim, HYPER_RAM_RST_CNT);

    /* Initial Read Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTRD(spim, u32AcctRD);

    /* Initial Write Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTWR(spim, u32AcctWR);
}

/**
 * @brief    Erase HyperRAM
 *
 * @param    spim         SPIM device pointer
 * @param    u32StartAddr Erase start address
 * @param    u32EraseSize Erase size
 *
 * @return   None
 *
 * @note     This function is used to erase HyperRAM block
 */
void HyperRAM_Erase(SPIM_T *spim, uint32_t u32StartAddr, uint32_t u32EraseSize)
{
    uint32_t u32i;

    /* Erase Hyper RAM */
    for (u32i = 0; u32i < u32EraseSize; u32i += 2)
    {
        /* Erase Hyper RAM block */
        SPIM_HYPER_Write2Byte(spim, (u32StartAddr + u32i), VERIFY_ERASE_PATTERN);

        /* Read back check and erase fail */
        if (SPIM_HYPER_Read1Word(spim, u32StartAddr + u32i) != VERIFY_ERASE_PATTERN)
        {
            printf("Erase Hyper RAM fail!!\n");

            while (1);
        }
    }

    /* Check remain 1 byte */
    if (u32EraseSize % 2)
    {
        /* Erase remain 1 byte */
        SPIM_HYPER_Write1Byte(spim, (u32StartAddr + u32EraseSize - 1), 0x00);

        /* Read back check and erase fail */
        if ((SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32EraseSize - 1)) >> 8) & 0xFF)
        {
            printf("Erase Remain HyperRAM fail, Read Data = %x !!\n",
                   (SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32EraseSize - 1)) >> 8));

            while (1);
        }
    }
}

/**
 * @brief Check if the given array of values is consecutive.
 *
 * @param psDlyNumRange Pointer to the structure to store the range of consecutive values.
 * @param au8Src Array of values to be checked.
 * @param size Size of the array.
 */
uint8_t isConsecutive(uint8_t au8Src[], uint32_t size)
{
    uint8_t u8Find = 0, u8StartIdx = 0, u8MaxRang = 0;
    uint32_t u32i = 0, u32j = 1;

    // Check if the sequence is increasing or decreasing
    bool increasing = au8Src[1] > au8Src[0];

    // Iterate over the array
    for (u32i = 1; u32i < size; ++u32i)
    {
        // Check if the current element is consecutive to the previous one
        if ((increasing && au8Src[u32i] != au8Src[u32i - 1] + 1) ||
                (!increasing && au8Src[u32i] != au8Src[u32i - 1] - 1))
        {
            // Update the start and end indices of the consecutive range
            u8Find = u32i;
            u32j = 0;
        }

        // Increment the number of consecutive elements
        u32j++;

        // Update the range if the current range is longer than the previous one
        if (u32j >= u8MaxRang)
        {
            u8StartIdx = u8Find;
            u8MaxRang = u32j;
        }
    }

    return (u8MaxRang > 2) ?
           au8Src[((u8StartIdx + u8MaxRang / 2) + (((u8MaxRang % 2) != 0) ? 1 : 0)) - 1] :
           au8Src[u8StartIdx];
}

void HyperRAM_TrimDLLDelayNumber(SPIM_T *spim)
{
    if (spim == NULL)
    {
        return;
    }

    uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayRes[SPIM_HYPER_MAX_LATENCY] = {0};
    uint32_t u32PatternSize = TRIM_PAT_SIZE;
    uint32_t u32LoopAddr = 0;
    uint32_t u32Val = 0;
    uint32_t u32i = 0;
    uint32_t u32j = 0;
    uint32_t u32k = 0;
    uint32_t u32SrcAddr = 0;
    uint32_t u32ReTrimCnt = 0;
    uint32_t u32ReTrimMaxCnt = 6;
    uint64_t au64TrimPattern[(TRIM_PAT_SIZE * 2) / 8] = {0};
    uint64_t au64VerifyBuf[TRIM_PAT_SIZE / 8] = {0};
    uint8_t *pu8TrimPattern = (uint8_t *)au64TrimPattern;
    uint8_t *pu8VerfiyBuf = (uint8_t *)au64VerifyBuf;
    uint32_t u32DMMAddr = SPIM_HYPER_GET_DMMADDR(spim);

    /* Create Trim Pattern */
    for (u32k = 0; u32k < sizeof(au64TrimPattern); u32k++)
    {
        u32Val = (u32k & 0x0F) ^ (u32k >> 4) ^ (u32k >> 3);

        if (u32k & 0x01)
        {
            u32Val = ~u32Val;
        }

        pu8TrimPattern[u32k] = ~(uint8_t)(u32Val ^ (u32k << 3) ^ (u32k >> 2));
    }

    SPIM_HYPER_DMAWrite(spim, u32SrcAddr, pu8TrimPattern, sizeof(au64TrimPattern));

    for (u32ReTrimCnt = 0; u32ReTrimCnt < u32ReTrimMaxCnt; u32ReTrimCnt++)
    {
        for (u8RdDelay = 0; u8RdDelay < SPIM_HYPER_MAX_LATENCY; u8RdDelay++)
        {
            /* Set DLL calibration to select the valid delay step number */
            SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);

            memset(pu8VerfiyBuf, 0, sizeof(au64VerifyBuf));

#if (NVT_DCACHE_ON == 1)
            SCB_InvalidateDCache_by_Addr((volatile uint32_t *)((u32ReTrimCnt == 1) ? u32SrcAddr : (u32DMMAddr + u32SrcAddr)), (int32_t)TRIM_PAT_SIZE * 2);
#endif

            /* Calculate the pattern size based on the trim count */
            u32PatternSize =
                (((u32ReTrimCnt == 2) || (u32ReTrimCnt >= 3)) && (u8RdDelay == 0)) ?
                (TRIM_PAT_SIZE - 0x08) :
                TRIM_PAT_SIZE;

            /* Read data from the HyperRAM */
            u32LoopAddr = 0;

            for (u32k = 0; u32k < u32PatternSize; u32k += 0x08)
            {
                if (u32ReTrimCnt == 1)
                {
                    SPIM_HYPER_DMARead(spim, u32SrcAddr + u32LoopAddr, &pu8VerfiyBuf[u32k], 8);
                }
                else
                {
                    SPIM_HYPER_EnterDirectMapMode(spim);

                    /* Read 8 bytes of data from the HyperRAM */
                    *(volatile uint64_t *)&pu8VerfiyBuf[u32k] = *(volatile uint64_t *)(u32DMMAddr + u32SrcAddr + u32LoopAddr);
                }

                if ((u32i = memcmp(&pu8TrimPattern[u32LoopAddr], &pu8VerfiyBuf[u32k], 0x08)) != 0)
                {
                    break;
                }

                u32LoopAddr += (u32ReTrimCnt >= 3) ? 0x10 : 0x08;
            }

            u8RdDelayRes[u8RdDelay] += ((u32i == 0) ? 1 : 0);
        }
    }

    u32j = 0;

    for (u32i = 0; u32i < SPIM_HYPER_MAX_LATENCY; u32i++)
    {
        if (u8RdDelayRes[u32i] == u32ReTrimMaxCnt)
        {
            u8RdDelayRes[u32j++] = u32i;
        }
    }

    u8RdDelay = (u32j < 2) ? u8RdDelayRes[0] : isConsecutive(u8RdDelayRes, u32j);

    printf("Set DLL Delay Num : %d\r\n", u8RdDelay);
    /* Set the number of intermediate delay steps */
    SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);
}

void HyperRAM_Init(SPIM_T *spim)
{
    /* Enable SPIM Hyper Bus Mode */
    SPIM_HYPER_Init(spim, SPIM_HYPERRAM_MODE, SPIM_HYPER_DIV);

    /* SPIM Def. Enable Cipher, First Disable the test. */
    SPIM_HYPER_DISABLE_CIPHER(spim);

    /* Set R/W Latency Number */
    SPIM_Hyper_DefaultConfig(spim, HYPERRAM_CSM_TIME, HYPERRAM_RD_LTCY, HYPERRAM_WR_LTCY);

    /* Reset HyperRAM */
    SPIM_HYPER_Reset(spim);

    /* Trim DLL component delay stop number */
    HyperRAM_TrimDLLDelayNumber(spim);

    SPIM_HYPER_EnterDirectMapMode(spim);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
