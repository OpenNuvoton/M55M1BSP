#include <string.h>
#include "NuMicro.h"

#define USE_4_BYTES_MODE            0   /* W25Q20 does not support 4-bytes address mode. */
#define SPIM_CIPHER_ON              0

/* User defined cipher key */
uint32_t gc_au32AESKey[8] =
{
    0x93484D6F, //Key0
    0x2F7A7F2A, //Key1
    0x063FF08A, //Key2
    0x7A29E38E, //Key3
    0x7A29E38E, //Scramble
    0x063FF08A, //NONCE0
    0x2F7A7F2A, //NONCE1
    0x93484D6F, //NONCE2
};

//------------------------------------------------------------------------------
/* 0x02h : CMD_NORMAL_PAGE_PROGRAM Command Phase Table */
SPIM_PHASE_T gsWb02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                                    //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    0,
};

/* 0x12h : CMD_NORMAL_PAGE_PROGRAM_4B Command Phase Table */
SPIM_PHASE_T gsWb12hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM_4B,                                                 //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    0,
};

/* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
SPIM_PHASE_T gsWb0BhRdCMD =
{
    CMD_DMA_FAST_READ,                                                          // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                        // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   // Data Phase
    8,                                                                          // Dummy Cycle Phase
};

/* 0xBB: CMD_DMA_FAST_DUAL_READ Command Phase Table */
SPIM_PHASE_T gsWbBBhRdCMD =
{
    CMD_DMA_FAST_DUAL_READ,                                                    // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_DUAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_DUAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,    // Data Phase
    0,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_DUAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xBC: CMD_DMA_FAST_DUAL_READ_4B Command Phase Table */
SPIM_PHASE_T gsWbBChRdCMD =
{
    CMD_DMA_FAST_DUAL_READ_4B,                                                 // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_DUAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_DUAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,    // Data Phase
    0,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_DUAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xEB: CMD_DMA_FAST_QUAD_READ Command Phase Table */
SPIM_PHASE_T gsWbEBhRdCMD =
{
    CMD_DMA_FAST_QUAD_READ,                                                    // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_QUAD_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_QUAD_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,    // Data Phase
    4,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_QUAD_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xEC: CMD_DMA_FAST_QUAD_READ_4B Command Phase Table */
SPIM_PHASE_T gsWbEChRdCMD =
{
    CMD_DMA_FAST_QUAD_READ_4B,                                                 // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_QUAD_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_QUAD_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,    // Data Phase
    4,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_QUAD_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

int32_t InitSPIM_OTFC(SPIM_T *psSPIM)
{
    SPIM_DISABLE_CIPHER(psSPIM);

    /* Set Cipher Key and protection region */
    OTFC_SetKeyFromKeyReg(OTFC0, gc_au32AESKey, OTFC_PR_0, 0, 0x200000);
    OTFC_ENABLE_PR(OTFC0, OTFC_PR_0);

    return 0;
}

int SPIM_TrimRxClkDlyNum(SPIM_T *spim, SPIM_PHASE_T *psWbRdCMD)
{
    volatile uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[0x0F] = {0};
    uint32_t u32SAddr = 0x10000;
    volatile uint32_t u32i = 0;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(spim);
    uint8_t au8TrimPatten[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
    uint8_t au8DestBuf[32] = {0};

#ifdef DMM_MODE_TRIM
    uint32_t u32RdDataCnt = 0;
    uint32_t u32DMMAddr = SPIM_GetDMMAddress(spim);
    uint32_t *pu32RdData = NULL;
#endif //

    SPIM_DMADMM_InitPhase(spim, &gsWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

    SPIM_SET_CLOCK_DIVIDER(spim, 8);

    SPIM_EraseBlock(spim, u32SAddr, SPIM_OP_DISABLE, OPCODE_BE_64K, 1, SPIM_OP_ENABLE);

    SPIM_DMA_Write(spim,
                   u32SAddr,
                   (gsWb02hWrCMD.u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL,
                   sizeof(au8TrimPatten),
                   au8TrimPatten,
                   gsWb02hWrCMD.u32CMDCode);

    SPIM_SET_CLOCK_DIVIDER(spim, u32Div);

#ifdef DMM_MODE_TRIM

    SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(spim,
                            (psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL,
                            psWbRdCMD->u32CMDCode,
                            1);
#endif

    for (u8RdDelay = 0; u8RdDelay <= 0xF; u8RdDelay++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelay);

        memset(au8DestBuf, 0, sizeof(au8DestBuf));

#ifndef DMM_MODE_TRIM

        SPIM_DMADMM_InitPhase(spim, psWbRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
        SPIM_DMA_Read(spim,
                      u32SAddr,
                      ((psWbRdCMD->u32AddrWidth == PHASE_WIDTH_32) ? 1UL : 0UL),
                      sizeof(au8TrimPatten),
                      au8DestBuf,
                      psWbRdCMD->u32CMDCode,
                      SPIM_OP_ENABLE);
#else
        u32RdDataCnt = 0;
        pu32RdData = (uint32_t *)au8DestBuf;

        for (u32i = u32SAddr; u32i < (u32SAddr + sizeof(au8TrimPatten)); u32i += 4)
        {
            pu32RdData[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
        }

#endif

        // Compare.
        if (memcmp(au8TrimPatten, au8DestBuf, sizeof(au8TrimPatten)) == 0)
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    if (u8RdDelayIdx >= 2)
    {
        u8RdDelayIdx = (u8RdDelayIdx / 2) - 1;
    }
    else
    {
        u8RdDelayIdx = 0;
    }

    printf("\r\nRX Delay = %d\r\n\r\n", u8RdDelayRes[u8RdDelayIdx]);
    SPIM_SET_RXCLKDLY_RDDLYSEL(spim, u8RdDelayRes[u8RdDelayIdx]);

    return u8RdDelay;
}

int32_t InitSPIM_DMM(SPIM_T *psSPIM)
{
    int32_t i32RetCode = 0;
    uint8_t au8JedecID[3]   = {0};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set SPIM clock as HCLK divided by 8 */
    SPIM_SET_CLOCK_DIVIDER(psSPIM, 1);

    SPIM_DISABLE_CIPHER(psSPIM);             /* Disable SPIM Cipher */

    SPIM_SET_RXCLKDLY_RDDLYSEL(psSPIM, 1);   /* Insert 3 delay cycle. Adjust the sampling clock of received data to latch the correct data. */

    if (SPIM_InitFlash(psSPIM, 1) != 0)      /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        return -1;
    }

    SPIM_ReadJedecId(psSPIM, au8JedecID, sizeof(au8JedecID), 1, 0);
    printf("Read JEDEC ID=0x%02X, 0x%02X, 0x%02X\n",
           au8JedecID[0], au8JedecID[1], au8JedecID[2]);

#if (SPIM_REG_CACHE == 1)
    SPIM_ENABLE_CACHE(psSPIM);
    psSPIM->CTL1 |= SPIM_CTL1_CDINVAL_Msk;        // invalid cache
#endif

    //SPIM_TrimRxClkDlyNum(psSPIM, &gsWb0BhRdCMD);
    SPIM_SET_RXCLKDLY_RDDLYSEL(psSPIM, 5);

    SPIM_DMADMM_InitPhase(psSPIM, &gsWb0BhRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);
    SPIM_EnterDirectMapMode(psSPIM, USE_4_BYTES_MODE, gsWb0BhRdCMD.u32CMDCode, 1);

    return 0;
}
