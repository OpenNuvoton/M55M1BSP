#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "NuMicro.h"
#include "hyperflash_code.h"
#include "SPIM_PinConfig.h"

//------------------------------------------------------------------------------
#define TC8263_WORK_AROUND

//------------------------------------------------------------------------------
static const uint32_t patterns[] =
{
    //0x00000000, 0xFFFFFFFF, 0x55aa55aa, 0xaa55aa55, 0x33cc33cc, 0xcc33cc33
    0x11111111, 0x22222222, 0x33333333, 0x44444444, 0x55555555, 0x66666666
    //0x11111111, 0x22222222, 0x33333333, 0x44444444
    //0x00112233, 0x44556677, 0x8899AABB, 0xCCDDEEFF
};

__attribute__((aligned(32))) uint8_t tstbuf[32] = {0};
__attribute__((aligned(32))) uint8_t tstbuf2[32] = {0};
__attribute__((aligned(32))) uint8_t tstbuf3[32] = {0};

// Key = 0x7A29E38E 063FF08A 2F7A7F2A 93484D6F 93484D6F 2F7A7F2A 063FF08A 7A29E38E
uint32_t gau32AESKey0[8] =
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
int32_t HyperFlash_WaitBusBusy(SPIM_T *pSPIMx);

//------------------------------------------------------------------------------
void dump_compare_error(uint32_t addr, uint8_t *buf_expect, uint8_t *buf_compare, int count)
{
    int  i, err_cnt = 0;

    for (i = 0; (i < count) && (err_cnt < 32); i++)
    {
        if (buf_expect[i] != buf_compare[i])
        {
            printf("addr 0x%x, expect: 0x%x, read: 0x%x\n",
                   addr + i,
                   buf_expect[i],
                   buf_compare[i]);
            err_cnt++;
        }
    }
}

void DumpBufferHex(uint8_t *pucBuff, int nSize)
{
    int     nIdx, i;

    nIdx = 0;

    while (nSize > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);

        printf("  ");

        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");

            nSize--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
}

void popDat(uint8_t buf[], uint32_t bufSize)
{
    uint8_t *bufInd = buf;
    uint32_t bufRmn = bufSize;

    while (bufRmn)
    {
        uint32_t nTx = sizeof(patterns);

        if (nTx > bufRmn)
        {
            nTx = bufRmn;
        }

        memcpy(bufInd, (uint8_t *) patterns, nTx);
        bufInd += nTx;                              // Advance indicator.
        bufRmn -= nTx;
    }
}

//------------------------------------------------------------------------------
// OTFC
//------------------------------------------------------------------------------
/**
  * @brief      Set OTFC AES key
  * @param[in]  u8Index is set index
  * @param[in]  u32Key is set key
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void SetAESKey0(uint8_t u8Index, uint32_t u32Key)
{
    gau32AESKey0[u8Index] = u32Key;
}

/**
  * @brief      Get OTFC AES key
  * @param[in]  u8Index is get key index
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void *GetAESKey0(uint8_t u8Index)
{
    return &gau32AESKey0[u8Index];
}

/**
  * @brief      OTFC Use Register Key Setting
  * @param[in]  pSPIMx SPIM Port number
  * @param[in]  u32PRBlock Protection Region 0 ~ 3
  * @param[in]  u32PRStartAddr Protection Region Start Address
  * @param[in]  u32PRSize Protection Region Size
  * @param[in]  pu32AESKey AES Key
  * @param[in]  DFAEn Differential Fault Attack ON/OFF
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void OTFC_CipherEnKeyFormReg(SPIM_T *pSPIMx,
                             uint32_t u32PRBlock,
                             uint32_t u32PRStartAddr,
                             uint32_t u32PRSize,
                             uint32_t *pu32AESKey,
                             uint32_t DFAEn)
{
    OTFC_T *pOTFCx = NULL;

    if (pSPIMx == SPIM0)
    {
        pOTFCx = OTFC0;
    }
    else
    {
        pOTFCx = OTFC1;
    }

    while (OTFC_GET_BUSY(pOTFCx, u32PRBlock) == 1);

    OTFC_RESET_PR(pOTFCx, u32PRBlock);

    pOTFCx->PR[u32PRBlock].SADDR = (u32PRStartAddr);
    pOTFCx->PR[u32PRBlock].EADDR = (u32PRStartAddr + u32PRSize);

    pOTFCx->PR[u32PRBlock].KEY0 = pu32AESKey[0];
    pOTFCx->PR[u32PRBlock].KEY1 = pu32AESKey[1];
    pOTFCx->PR[u32PRBlock].KEY2 = pu32AESKey[2];
    pOTFCx->PR[u32PRBlock].KEY3 = pu32AESKey[3];
    pOTFCx->PR[u32PRBlock].SCRAMBLE = pu32AESKey[4];
    pOTFCx->PR[u32PRBlock].NONCE0 = pu32AESKey[5];
    pOTFCx->PR[u32PRBlock].NONCE1 = pu32AESKey[6];
    pOTFCx->PR[u32PRBlock].NONCE2 = pu32AESKey[7];

    //OTFC_ENABLE_KSWAP(pOTFCx, u32PRBlock);

    OTFC_ENABLE_PR(pOTFCx, u32PRBlock);

    return;
}

//------------------------------------------------------------------------------
// HyperFlash
//------------------------------------------------------------------------------
#define TC8263_WORK_AROUND

//------------------------------------------------------------------------------
void HyperFlash_ResetModule(SPIM_T *spim)
{
    volatile uint32_t u32Delay = 0;

    SPIM_HYPER_Reset(spim);

    for (u32Delay = 0; u32Delay < 0x3000; u32Delay++) {}
}

/**
 * @brief Send Hyper Flash Operation Command
 *
 * @param spim
 * @param u32CMD
 * @param u32CMDData
 */
void HyperFlash_WriteOPCMD(SPIM_T *spim, uint32_t u32CMD, uint32_t u32Addr)
{
    SPIM_HYPER_Write2Byte(spim, (u32CMD * 2), u32Addr);
}

void HyperFlash_ClearECCError(SPIM_T *spim)
{
    HyperFlash_WriteOPCMD(spim, HF_CMD_NOOP_CODE, HF_CMD_NOOP_CODE);
}


void HyperFlash_WriteConfigRegister(SPIM_T *spim, uint32_t u32Reg, uint16_t u16WrData)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, u32Reg);

    SPIM_HYPER_Write2Byte(spim, HF_CMD_NOOP_CODE, u16WrData);

    HyperFlash_WaitBusBusy(spim);
}

/**
 * @brief Read HyperFlash Non-volatile config register
 *
 * @param spim
 * @return uint16_t Register value
 */
uint16_t HyperFlash_ReadConfigRegister(SPIM_T *spim, uint32_t u32Reg)
{
    volatile uint16_t u16RdData = 0;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, u32Reg);

    u16RdData = SPIM_HYPER_Read1Word(spim, HF_CMD_NOOP_CODE);

    return u16RdData;
}

/**
 * @brief Wait Hyper Flash Program Busy
 *
 * @param spim
 */
int32_t HyperFlash_WaitBusBusy(SPIM_T *spim)
{
    volatile int32_t i32Timeout = SPIM_TIMEOUT;
    volatile uint32_t u32Status = 0;

    while (u32Status != 0x80)
    {
#ifdef TC8263_WORK_AROUND
        HyperFlash_ClearECCError(spim);
#endif
        HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_70);

        u32Status = (SPIM_HYPER_Read1Word(spim, HF_CMD_NOOP_CODE) & 0x80);

        //if (u32Status == 0x80)
        //{
        //    log_printf("u32Status = %x\r\n", u32Status);
        //}

        //if (--i32Timeout <= 0)
        //{
        //    return SPIM_ERR_TIMEOUT;
        //}
    }

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    return SPIM_OK;
}

void HyperFlash_EraseSector(SPIM_T *spim, uint32_t u32SAddr)
{
    volatile int32_t i32Timeout = SPIM_TIMEOUT;
    volatile uint32_t u32Retry = 0;

    if ((u32SAddr != 0) && (u32SAddr >= 2))
    {
        u32SAddr /= 2;
    }

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_80);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, u32SAddr, HF_CMD_30);

    HyperFlash_WaitBusBusy(spim);
}

void HyperFlash_EraseChip(SPIM_T *spim)
{
    volatile int32_t i32Timeout = SPIM_TIMEOUT;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_80);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_10);

    HyperFlash_WaitBusBusy(spim);
}

void HyperFlash_DMMWrite(SPIM_T *pSPIMx)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(pSPIMx);
#endif

    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_555, HF_CMD_A0);

    if (((SPIM_GET_OPMODE(pSPIMx) & SPIM_CTL0_OPMODE_Msk) >> SPIM_CTL0_OPMODE_Pos) != 0x03)
    {
        SPIM_HYPER_EnterDirectMapMode(pSPIMx); // Hyper Mode Switch to Direct Map mode.
    }
}

void HyperFlash_DMMRead(SPIM_T *pSPIMx)
{
    //#ifdef TC8263_WORK_AROUND
    //    HyperFlash_ClearECCError(pSPIMx);
    //#endif

    //if (((SPIM_GET_OPMODE(pSPIMx) & SPIM_CTL0_OPMODE_Msk) >> SPIM_CTL0_OPMODE_Pos) != 0x03)
    //{
    SPIM_HYPER_EnterDirectMapMode(pSPIMx); // Hyper Mode Switch to Direct Map mode.
    //}
}

void HyperFlash_DMMWritePage(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvWrBuf, uint32_t u32NTx)
{
    int *pi32SrcAddr = NULL;
    uint8_t *pu8Data = (uint8_t *)pvWrBuf;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(pSPIMx);
#endif

    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(pSPIMx, HF_CMD_COMMON_555, HF_CMD_A0);

    SPIM_HYPER_EnterDirectMapMode(pSPIMx);

    pi32SrcAddr = (int *)(u32SAddr);

    memcpy(pi32SrcAddr, pu8Data, u32NTx);

    SPIM_HYPER_IsDMMDone(pSPIMx);

    HyperFlash_WaitBusBusy(pSPIMx);
}

void HyperFlash_DMMWriteData(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvWrBuf, uint32_t u32NTx)
{
    uint32_t   pageOffset, toWr;
    uint32_t   buf_idx = 0UL;
    uint8_t *pu8Data = (uint8_t *)pvWrBuf;

    pageOffset = (u32SAddr % HFLH_PAGE_SIZE);

    if ((pageOffset + u32NTx) <= HFLH_PAGE_SIZE)
    {
        /* Do all the bytes fit onto one page ? */
        HyperFlash_DMMWritePage(pSPIMx, u32SAddr, pvWrBuf, u32NTx);
        //printf("DMM W1\r\n");
    }
    else
    {
        toWr = HFLH_PAGE_SIZE - pageOffset;               /* Size of data remaining on the first page. */

        HyperFlash_DMMWritePage(pSPIMx, u32SAddr, (void *)&pu8Data[buf_idx], toWr);
        //printf("DMM W2\r\n");

        u32SAddr += toWr;                         /* Advance indicator. */
        u32NTx -= toWr;
        buf_idx += toWr;

        while (u32NTx)
        {
            toWr = HFLH_PAGE_SIZE;

            if (toWr > u32NTx)
            {
                toWr = u32NTx;
            }

            HyperFlash_DMMWritePage(pSPIMx, u32SAddr, (void *)&pu8Data[buf_idx], toWr);
            //printf("DMM W3\r\n");
            u32SAddr += toWr;                 /* Advance indicator. */
            u32NTx -= toWr;
            buf_idx += toWr;
        }
    }
}

void HyperFlash_DMMReadData(SPIM_T *pSPIMx, uint32_t u32SAddr, void *pvRdBuf, uint32_t u32NRx)
{
    int *pi32SrcAddr = NULL;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(pSPIMx);
#endif

    SPIM_HYPER_EnterDirectMapMode(pSPIMx);

    pi32SrcAddr = (int *)u32SAddr;
    memcpy(pvRdBuf, pi32SrcAddr, u32NRx);

    SPIM_HYPER_IsDMMDone(pSPIMx);
    HyperFlash_WaitBusBusy(pSPIMx);
}

void HyperFlash_DMM_Write(SPIM_T *spim)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_A0);

    SPIM_HYPER_EnterDirectMapMode(spim); // Hyper Mode Switch to Direct Map mode.
}

void HyperFlash_DMA_WriteByPage(SPIM_T *spim, uint32_t u32SAddr, uint8_t *pu8WrBuf, uint32_t u32NTx)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_A0);

    SPIM_HYPER_DMAWrite(spim, u32SAddr, pu8WrBuf, u32NTx);

    HyperFlash_WaitBusBusy(spim);
}

void HyperFlash_DMAWrite(SPIM_T *spim, uint32_t u32SAddr, uint8_t *pu8WrBuf, uint32_t u32NTx)
{
    uint32_t pageOffset = 0;

    pageOffset = u32SAddr % HFLH_PAGE_SIZE;

    if ((pageOffset + u32NTx) <= HFLH_PAGE_SIZE)
    {
        /* Do all the bytes fit onto one page ? */
        HyperFlash_DMA_WriteByPage(spim, u32SAddr, pu8WrBuf, u32NTx);
    }
    else
    {
        uint32_t toWr = 0;
        uint32_t buf_idx = 0;

        toWr = HFLH_PAGE_SIZE - pageOffset;               /* Size of data remaining on the first page. */

        HyperFlash_DMA_WriteByPage(spim, u32SAddr, &pu8WrBuf[buf_idx], toWr);

        u32SAddr += toWr;                         /* Advance indicator. */
        u32NTx -= toWr;
        buf_idx += toWr;

        while (u32NTx)
        {
            toWr = HFLH_PAGE_SIZE;

            if (toWr > u32NTx)
            {
                toWr = u32NTx;
            }

            HyperFlash_DMA_WriteByPage(spim, u32SAddr, &pu8WrBuf[buf_idx], toWr);

            u32SAddr += toWr;                 /* Advance indicator. */
            u32NTx -= toWr;
            buf_idx += toWr;
        }
    }
}

#ifdef DMM_MODE_TRIM

void HyperFlash_DMARead(SPIM_T *spim, uint32_t u32SAddr, uint8_t *pu8RdBuf, uint32_t u32NRx)
{
    SPIM_HYPER_DMARead(spim, u32SAddr, pu8RdBuf, u32NRx);

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WaitBusBusy(spim);
}
#endif

void HyperFlash_IO_Write2Byte(SPIM_T *spim, uint32_t u32WrAddr, uint16_t u16WrData)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_A0);

    SPIM_HYPER_Write2Byte(spim, u32WrAddr, u16WrData);

    HyperFlash_WaitBusBusy(spim);
}

void HyperFlash_IO_Write4Byte(SPIM_T *spim, uint32_t u32WrAddr, uint32_t u32WrData)
{
#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_COMMON_AA);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_2AA, HF_CMD_COMMON_55);
    HyperFlash_WriteOPCMD(spim, HF_CMD_COMMON_555, HF_CMD_A0);

    SPIM_HYPER_Write4Byte(spim, u32WrAddr, u32WrData);

    HyperFlash_WaitBusBusy(spim);
}

uint16_t HyperFlash_IO_Read2Byte(SPIM_T *spim, uint32_t u32SAddr)
{
    volatile uint16_t u16RdData = 0;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    u16RdData = SPIM_HYPER_Read1Word(spim, u32SAddr);

    return u16RdData;
}

uint32_t HyperFlash_IO_Read4Byte(SPIM_T *spim, uint32_t u32SAddr)
{
    volatile uint32_t u32RdData = 0;

#ifdef TC8263_WORK_AROUND
    HyperFlash_ClearECCError(spim);
#endif

    u32RdData = SPIM_HYPER_Read2Word(spim, u32SAddr);

    return u32RdData;
}

void HyperFlash_IO_Read(SPIM_T *spim, uint32_t u32SAddr, void *pvRdBuf, uint32_t u32NRx)
{
    uint8_t *pu8RxBuf = NULL;
    uint32_t u32DataCnt = 0;
    uint16_t *pu16RxBuf = (uint16_t *)pvRdBuf;
    uint32_t u32i = 0;
    uint32_t u32RemainSize = (u32NRx % 2);

    for (u32i = 0; u32i < (u32NRx - u32RemainSize); u32i += 2)
    {
        pu16RxBuf[u32DataCnt++] = HyperFlash_IO_Read2Byte(spim, u32SAddr + u32i);
    }

    if (u32RemainSize != 0)
    {
        uint8_t *pu8Temp = (void *)pvRdBuf;
        pu8RxBuf = (uint8_t *)&pu8Temp[(u32NRx - u32RemainSize)];
        *pu8RxBuf = ((HyperFlash_IO_Read2Byte(spim, ((u32SAddr + u32NRx) - u32RemainSize)) >> 8) & 0xFF);
    }
}

#define DMM_MODE_TRIM

/**
 * @brief Training DLL component delay stop number
 *
 * @param spim
 */
void HyperFlash_TrainingDLLDelayTime(SPIM_T *spim)
{
    volatile uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[SPIM_HYPER_MAX_LATENCY] = {0};
    uint32_t u32SrcAddr = 0;
    uint32_t u32TestSize = 32;
    volatile uint32_t u32i = 0;
    uint8_t au8TrimPatten[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
#ifdef DMM_MODE_TRIM
    uint32_t u32DMMAddr = SPIM_HYPER_GetDMMAddress(spim);
    uint32_t u32RdDataCnt = 0;
    uint32_t *pu32RdBuf = NULL;
#endif
    //popDat(au8TrimPatten, u32TestSize);

    /* Erase HyperFlash */
    HyperFlash_EraseSector(spim, 0); //one sector = 256KB

    HyperFlash_DMAWrite(spim, u32SrcAddr, au8TrimPatten, u32TestSize);

    //for (u8RdDelay = 0; u8RdDelay < u32TestSize; u8RdDelay += 4)
    //{
    //    memcpy(&u32WrData, &au8TrimPatten[u8RdDelay], sizeof(uint32_t));
    //    HyperFlash_IO_Write4Byte(spim, u32SrcAddr + u8RdDelay, u32WrData);
    //}

#ifdef DMM_MODE_TRIM
    /* Enter direct-mapped mode to run new applications */
    //SPIM_HYPER_EnterDirectMapMode(spim);
    pu32RdBuf = (uint32_t *)tstbuf2;
#endif

    for (u8RdDelay = 0; u8RdDelay <= SPIM_HYPER_MAX_LATENCY; u8RdDelay++)
    {
        /* Set DLL calibration to select the valid delay step number */
        SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelay);

        memset(tstbuf2, 0, sizeof(tstbuf2));

#ifndef DMM_MODE_TRIM
        /* Read Data from HyperFlash */
        HyperFlash_DMARead(spim, u32SrcAddr, tstbuf2, u32TestSize);
#else
        u32RdDataCnt = 0;

        //for (u32i = u32SrcAddr; u32i < (u32SrcAddr + u32TestSize); u32i += 4)
        //{
        //    pu32RdBuf[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
        //}

        for (u32i = 0; u32i < u32TestSize; u32i += 4)
        {
            pu32RdBuf[u32RdDataCnt++] = HyperFlash_IO_Read4Byte(spim, u32SrcAddr + u32i);
        }

#endif//

        /* Verify the data and save the number of successful delay steps */
        if (memcmp(au8TrimPatten, tstbuf2, u32TestSize))
        {
            printf("Data compare failed at block 0x%x\n", u32SrcAddr);
            //dump_compare_error(u32SrcAddr, au8TrimPatten, tstbuf2, u32TestSize);
        }
        else
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    //HyperFlash_WaitBusBusy(spim);

    if (u8RdDelayIdx <= 1)
    {
        u8RdDelayIdx = 0;
        u8RdDelayRes[u8RdDelayIdx] = 12;
    }
    else
    {
        if (u8RdDelayIdx >= 2)
        {
            u8RdDelayIdx = ((u8RdDelayIdx / 2));
        }
        else
        {
            u8RdDelayIdx = 0;
            u8RdDelayRes[u8RdDelayIdx] = 12;
        }
    }

    /* Set the number of intermediate delay steps */
    SPIM_HYPER_SetDLLDelayNum(spim, u8RdDelayRes[u8RdDelayIdx]);
    printf("Set Delay Step Num : %d\r\n", u8RdDelayRes[u8RdDelayIdx]);
}

/**
  * @brief      SPIM Default Config HyperBus Access Module Parameters.
  * @param      spim
  * @param      u32CSMaxLT Chip Select Maximum Low Time 0 ~ 0xFFFF, Default Set 0x02ED
  * @param      u32AcctRD Initial Read Access Time 1 ~ 0x1F, Default Set 0x04
  * @param      u32AcctWR Initial Write Access Time 1 ~ 0x1F, Default Set 0x04
  * @return     None.
  */
void SPIM_HyperFlash_DefaultConfig(SPIM_T *spim, uint32_t u32CSMaxLow,
                                   uint32_t u32AcctRD, uint32_t u32AcctWR)
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

void HyperFlash_SetReadLatency(SPIM_T *spim, uint32_t u32Latency)
{
    uint32_t u32RegValue = 0;

    /* HyperFlash default read latency is 16 and write is always 1 */
    SPIM_HyperFlash_DefaultConfig(spim, HFLH_MAX_CS_LOW, 16, HFLH_WR_ACCTIME);

    SPIM_HYPER_SetDLLDelayNum(spim, 12);

    HyperFlash_ResetModule(spim);

    u32RegValue = HyperFlash_ReadConfigRegister(spim, READ_VCR_REG);
    printf("1 VCReg = %x\r\n", u32RegValue);

    HyperFlash_WriteConfigRegister(spim, LOAD_VCR_REG, 0x8E4B);

    SPIM_HyperFlash_DefaultConfig(spim, HFLH_MAX_CS_LOW, 9, HFLH_WR_ACCTIME);

    u32RegValue = HyperFlash_ReadConfigRegister(spim, READ_VCR_REG);
    printf("2 VCReg = %x\r\n", u32RegValue);

    //HyperFlash_WriteConfigRegister(spim, WRITE_PWR_ON_TIME_REG, 0x0080);

    //log_printf("Pwr On Time = %x\r\n", HyperFlash_ReadConfigRegister(spim, READ_PWR_ON_TIME_REG));
    HyperFlash_WaitBusBusy(spim);

    HyperFlash_TrainingDLLDelayTime(spim);
}

void SPIM_NVIC_Disable(SPIM_T *pSPIMx)
{
    if (pSPIMx == SPIM0)
    {
        NVIC_DisableIRQ(SPIM0_IRQn);
    }
    else if (pSPIMx == SPIM1)
    {
        NVIC_DisableIRQ(SPIM1_IRQn);
    }
}

void HyperFlash_PinConfig(SPIM_T *pSPIMx)
{
    if (pSPIMx == SPIM0)
    {
        /* Enable SPIM0 module clock */
        CLK_EnableModuleClock(SPIM0_MODULE);
        /* Enable OTFC0 module clock */
        CLK_EnableModuleClock(OTFC0_MODULE);
        //printf("OTFCCTL = %d, addr = 0x%08X\r\n", CLK->OTFCCTL, &CLK->OTFCCTL);

        /* Init SPIM0 multi-function pins */
        SPIM0_RST_PIN_INIT();
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

        /* Set SPIM0 I/O pins as high slew rate up to 80 MHz. */
        SPIM0_PIN_HIGH_SLEW();
    }
    else if (pSPIMx == SPIM1)
    {
        /* Enable SPIM1 module clock */
        CLK_EnableModuleClock(SPIM1_MODULE);
        /* Enable OTFC1 module clock */
        CLK_EnableModuleClock(OTFC1_MODULE);

        /* Init SPIM1 multi-function pins */
        SPIM1_RST_PIN_INIT();
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

        /* Set SPIM1 I/O pins as high slew rate up to 80 MHz. */
        SPIM1_PIN_HIGH_SLEW();
    }
}

void HyperFlash_Init(SPIM_T *pSPIMx, uint32_t u32CacheOn)
{
    SPIM_NVIC_Disable(pSPIMx);

    /* Enable SPIM Hyper Bus Mode */
    SPIM_HYPER_Init(pSPIMx, 1);

    /* SPIM Def. Enable Cipher, First Disable the test. */
    SPIM_HYPER_DISABLE_CIPHER(pSPIMx);

    SPIM_HYPER_DISABLE_CACHE(pSPIMx);

    HyperFlash_SetReadLatency(pSPIMx, 9);

    if (u32CacheOn)
    {
#if (SPIM_CACHE_EN == 1)
        SPIM_ENABLE_CACHE(pSPIMx);
#endif //SPIM_CACHE_EN
    }
}
