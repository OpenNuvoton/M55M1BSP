/**************************************************************************//**
 * @file     hyperram_code.c
 * @version  V1.00
 * @brief    Collect of sub-routines running on SPIM flash.
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "hyperram_code.h"

#include "SPIM_PinConfig.h"

//------------------------------------------------------------------------------
__attribute__((aligned(32))) static uint8_t g_au8DestArray[BUFF_SIZE] = {0};
__attribute__((aligned(32))) static uint8_t g_au8TrimPatten[32] =
{
    0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
    0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
    0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
    0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
};

//------------------------------------------------------------------------------
/**
 * @brief Erase and check HyperRAM
 *
 * @param spim
 * @param u32StartAddr  erase start address
 * @param u32EraseSize  erase size
 */
void HyperRAM_Erase(SPIM_T *spim, uint32_t u32StartAddr, uint32_t u32EraseSize)
{
    uint16_t u16Data;
    uint32_t u32i = 0;
    uint32_t u32RemainSize = (u32EraseSize % 2);

    for (u32i = 0; u32i <= (u32EraseSize - u32RemainSize); u32i += 2)
    {
        SPIM_HYPER_Write2Byte(spim, (u32StartAddr + u32i), 0x0000);
        u16Data = SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32i));

        if (u16Data != 0x0000)
        {
            printf("Erase Hyper RAM fail!! Read address:0x%08x, data::0x%08x  expect: 0\n",
                   u32StartAddr + u32i,
                   u16Data);

            while (1);
        }
    }

    if (u32RemainSize != 0)
    {
        SPIM_HYPER_Write1Byte(spim, (u32StartAddr + (u32EraseSize - 1)), 0x00);
        u16Data = SPIM_HYPER_Read1Word(spim, (u32StartAddr + u32EraseSize));

        if ((u16Data & 0xFF) != 0)
        {
            printf("Erase Remain Hyper RAM fail!! Read address:0x%08x, data::0x%08x  expect: 0\n",
                   (u32StartAddr + (u32EraseSize - 1)),
                   u16Data);

            while (1);
        }
    }
}

#define DMM_MODE_TRIM

/**
 * @brief Training DLL component delay stop number
 *
 * @param spim
 */

void HyperRAM_TrainingDelayNumber(SPIM_T *spim)
{
    uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[SPIM_MAX_DLL_LATENCY] = {0};
    uint32_t u32SrcAddr = 0;
#ifdef DMM_MODE_TRIM
    uint32_t u32DMMAddr = SPIM_HYPER_GetDMMAddress(spim);
    uint32_t *pu32RdBuf = NULL;
    uint32_t u32RdDataCnt = 0;
    volatile uint32_t u32i = 0;
#endif

    /* Erase HyperRAM */
    HyperRAM_Erase(spim, u32SrcAddr, sizeof(g_au8TrimPatten));

    SCB_CleanDCache_by_Addr(g_au8TrimPatten, sizeof(g_au8TrimPatten));

    /* Write Data to HyperRAM */
    SPIM_HYPER_DMAWrite(spim, u32SrcAddr, g_au8TrimPatten, sizeof(g_au8TrimPatten));

#ifdef DMM_MODE_TRIM
    //SPIM_HYPER_EnterDirectMapMode(spim);
#endif

    for (u8RdDelay = 0; u8RdDelay <= SPIM_MAX_DLL_LATENCY; u8RdDelay++)
    {
        memset(g_au8DestArray, 0, sizeof(g_au8DestArray));

        /* Set DLL calibration to select the valid delay step number */
        SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);

        /* Read Data from HyperRAM */
#ifndef DMM_MODE_TRIM
        SPIM_HYPER_DMARead(spim, u32SrcAddr, g_au8DestArray, u32TestSize);
#else
        u32RdDataCnt = 0;
        pu32RdBuf = (uint32_t *)g_au8DestArray;

        for (u32i = u32SrcAddr; u32i < (u32SrcAddr + sizeof(g_au8TrimPatten)); u32i += 4)
        {
            //pu32RdBuf[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
            pu32RdBuf[u32RdDataCnt++] = SPIM_HYPER_Read2Word(spim, u32DMMAddr + u32i);
        }

#endif

        SCB_InvalidateDCache_by_Addr(g_au8DestArray, sizeof(g_au8TrimPatten));

        /* Verify the data and save the number of successful delay steps */
        if (memcmp(g_au8TrimPatten, g_au8DestArray, sizeof(g_au8TrimPatten)))
        {
            printf("!!!\tData compare failed at block 0x%x\n", u32SrcAddr);
        }
        else
        {
            printf("Delay Step Num : %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    SPIM_HYPER_ExitDirectMapMode(spim);

    if (u8RdDelayIdx <= 1)
    {
        u8RdDelayIdx = 0;
    }
    else
    {
        if (u8RdDelayIdx >= 2)
        {
            u8RdDelayIdx = (u8RdDelayIdx / 2) - 1;
        }
        else
        {
            u8RdDelayIdx = 1;
        }
    }

    printf("Set HyperRAM DLL number %d\r\n", u8RdDelayRes[u8RdDelayIdx]);
    /* Set the number of intermediate delay steps */
    SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelayRes[u8RdDelayIdx]);
}

/**
  * @brief      SPIM Default Config HyperBus Access Module Parameters.
  * @param      spim
  * @param      u32CSMaxLT Chip Select Maximum Low Time 0 ~ 0xFFFF, Default Set 0x02ED
  * @param      u32AcctRD Initial Read Access Time 1 ~ 0x1F, Default Set 0x04
  * @param      u32AcctWR Initial Write Access Time 1 ~ 0x1F, Default Set 0x04
  * @return     None.
  */
void SPIM_Hyper_DefaultConfig(SPIM_T *spim, uint32_t u32CSMaxLow, uint32_t u32AcctRD, uint32_t u32AcctWR)
{
    /* Chip Select Setup Time 2.5 */
    SPIM_HYPER_SET_CSST(spim, SPIM_HYPER_CSST_3_5_HCLK);

    /* Chip Select Hold Time 3.5 HCLK */
    SPIM_HYPER_SET_CSH(spim, SPIM_HYPER_CSH_3_5_HCLK);

    /* Chip Select High between Transaction as 2 HCLK cycles */
    SPIM_HYPER_SET_CSHI(spim, 2);

    /* Chip Select Masximum low time HCLK */
    SPIM_HYPER_SET_CSMAXLT(spim, u32CSMaxLow);

    /* Initial Device RESETN Low Time 255 */
    SPIM_HYPER_SET_RSTNLT(spim, 0xFF);

    /* Initial Read Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTRD(spim, u32AcctRD);

    /* Initial Write Access Time Clock cycle*/
    SPIM_HYPER_SET_ACCTWR(spim, u32AcctWR);
}

void HyperRAM_Init(SPIM_T *spim)
{
    /* Enable SPIM Hyper Bus Mode */
#if defined(TESTCHIP_ONLY)
    SPIM_HYPER_Init(spim, 1);
#else
    SPIM_HYPER_Init(spim, 1);
#endif

#if (SPIM_REG_CACHE == 1)
    /* Enable SPIM Cache */
    SPIM_ENABLE_CACHE(spim);
#endif //SPIM_CACHE_EN

    /* SPIM Def. Enable Cipher, First Disable the test. */
    SPIM_DISABLE_CIPHER(spim);

    /* Reset HyperRAM */
    SPIM_HYPER_Reset(spim);

    /* Set R/W Latency Number */
    SPIM_Hyper_DefaultConfig(spim, 780, 7, 7);

#if 0 //Set DLL directly
    /* Set the number of intermediate delay steps */
    SPIM_HYPER_SetDLLDelayNum(spim, 14);
#else
    /* Training DLL component delay stop number */
    HyperRAM_TrainingDelayNumber(spim);
#endif
}

void HyperRAM_PinConfig(SPIM_T *spim)
{
    if (spim == SPIM0)
    {
        //SPIM and OTFC clock was enabled on secure-domain code
        /* Enable SPIM0 module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);
#if defined (TESTCHIP_ONLY)
        CLK_EnableModuleClock(SPIM1_MODULE);
#endif

        /* Enable OTFC0 module clock */
        CLK_EnableModuleClock(OTFC0_MODULE);
        //printf("OTFCCTL = %d, addr = 0x%08X\r\n", CLK->OTFCCTL, &CLK->OTFCCTL);

        /* Init SPIM0 multi-function pins */
        SPIM0_CLK_PIN_INIT();
        SPIM0_CLKN_PIN_INIT();
        SPIM0_D2_PIN_INIT();
        SPIM0_D3_PIN_INIT();
        SPIM0_D4_PIN_INIT();
        SPIM0_D5_PIN_INIT();
        SPIM0_D6_PIN_INIT();
        SPIM0_D7_PIN_INIT();
        SPIM0_MISO_PIN_INIT();
        SPIM0_MOSI_PIN_INIT();
        SPIM0_SS_PIN_INIT();
        SPIM0_RWDS_PIN_INIT();
        SPIM0_RST_PIN_INIT();

        /* Set SPIM0 I/O pins as high slew rate up to 80 MHz. */
        SPIM0_PIN_HIGH_SLEW();
    }

#if defined (TESTCHIP_ONLY)
    else if (spim == SPIM1)
    {
        //SPIM and OTFC clock was enabled on secure-domain code
        /* Enable SPIM1 module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);
        CLK_EnableModuleClock(SPIM1_MODULE);
        /* Enable OTFC1 module clock */
        CLK_EnableModuleClock(OTFC1_MODULE);

        /* Init SPIM1 multi-function pins */
        SPIM1_CLK_PIN_INIT();
        SPIM1_CLKN_PIN_INIT();
        SPIM1_D2_PIN_INIT();
        SPIM1_D3_PIN_INIT();
        SPIM1_D4_PIN_INIT();
        SPIM1_D5_PIN_INIT();
        SPIM1_D6_PIN_INIT();
        SPIM1_D7_PIN_INIT();
        SPIM1_MISO_PIN_INIT();
        SPIM1_MOSI_PIN_INIT();
        SPIM1_SS_PIN_INIT();
        SPIM1_RWDS_PIN_INIT();
        SPIM1_RST_PIN_INIT();

        /* Set SPIM1 I/O pins as high slew rate up to 80 MHz. */
        SPIM1_PIN_HIGH_SLEW();
    }

#endif
}
