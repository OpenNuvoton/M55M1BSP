/**************************************************************************//**
 * @file     CommandHandler.c
 * @version  V3.00
 * @brief    Secure ISP - Process commands
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "CommandHandler.h"

#define LIB_VERSION     (0x23551171UL) // 2355 + 'M' + 'DD' + IDX ... 1st released

uint32_t CMDLIB_VERSION(void)
{
    return LIB_VERSION;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Miscellaneous Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CURVE_P_SIZE        CURVE_P_256     /* ECC key length */
#define ECC_KEY_SIZE        256             /* 256-bits */
#define ALL_ONE_2PAGE_CHKS  0xB4293435      /* 2 page all 0xFF CRC32 checksum value */


//volatile ISP_INFO_T     g_ISPInfo = {0};

///* Declare Client's ECC Key pair */
//const char gacPrivKey[] = "d0ab2cb9eb88976e82f107598077ce50d8c7b67def7039ee5ba39ee0dd3be411";
//const char gacPubKey0[] = "d32438a1b4428541c564eeed79669b4bd3bf601c758469545e013c8fe8af7ef6";
//const char gacPubKey1[] = "476de8f3c6e6c48a8bacf1e1827cfb82501833c2bb816344f996533b1b031706";

void Hex2Reg(char input[], uint32_t volatile reg[]);
void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);

static uint32_t GetMaxAPROMSize(void)
{
    return FMC_APROM_SIZE;
}

static uint8_t B2C(uint8_t c)
{
    if (c < 10)
        return (c + '0');

    if (c < 16)
        return (c - 10 + 'a');

    return 0;
}

static uint32_t Swap32(uint32_t value)
{
    volatile uint32_t val;

    /* __REV(x), byte swap32 for keil project */

    val = (value << 24) | ((value << 8) & 0xff0000) | ((value >> 8) & 0xff00) | (value >> 24);
    return val;
}

void BytesSwap(char *buf, int32_t len)
{
    int32_t i;
    char    tmp;

    for (i = 0; i < (len / 2); i++)
    {
        tmp = buf[len - i - 1];
        buf[len - i - 1] = buf[i];
        buf[i] = tmp;
    }
}

/**
  * @brief      Open SHA encrypt function
  * @param[in]  start       SHA encrypt start address
  * @param[in]  end         SHA encrypt end address
  * @param[out] digest      The SHA encrypt output digest
  * @param[in]  mode        SHA operation mode, including:
  *                             - \ref SHA_ONESHOT
  *                             - \ref SHA_CONTI_START
  *                             - \ref SHA_CONTI_ING
  *                             - \ref SHA_CONTI_END
  * @retval     0           Success
  * @retval     -1          Failed
  */
int32_t CalculateSHA256(uint32_t start, uint32_t end, uint32_t digest[], E_SHA_OP_MODE mode, E_SHA_SRC src)
{
    volatile int32_t    i, bytes;
    uint32_t            *ptr, addr, data, Hash[8];

    bytes   = end - start;
    ptr     = (uint32_t *)start;
    addr    = (uint32_t)ptr;

    if ((mode == SHA_ONESHOT) || (mode == SHA_CONTI_START))
    {
        DBG("\n[Start SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)] (%s)\n", start, end, bytes, mode, (src == SHA_SRC_SRAM) ? "SRAM" : "Flash");

        //CRYPTO->HMAC_CTL = (SHA_MODE_SHA256 << CRYPTO_HMAC_CTL_OPMODE_Pos) | CRYPTO_HMAC_CTL_INSWAP_Msk;
        // Sync output byte order with PC
        CRYPTO->HMAC_CTL = (SHA_MODE_SHA256 << CRYPTO_HMAC_CTL_OPMODE_Pos) | CRYPTO_HMAC_CTL_INSWAP_Msk | CRYPTO_HMAC_CTL_OUTSWAP_Msk;
        CRYPTO->HMAC_DMACNT = 64;
        CRYPTO->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk;
    }
    else
    {
        DBG("[Continue SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)]\n", start, end, bytes, mode);
    }

    /* Start to calculate ... */
    while (bytes > 0)
    {
        if (bytes < 64)
            CRYPTO->HMAC_DMACNT = bytes;

        if (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_DATINREQ_Msk)
        {
            if (src == SHA_SRC_SRAM)
            {
                data = *ptr++;
                bytes -= 4;
            }
            else if (src == SHA_SRC_FLASH)
            {
                /* NOT Support calculate XOM region */
                if (addr < 512)
                    data = FMC_Read((addr & 0x0FFFFFFF)); /* First 512Byte using FMC ISP command */
                else
                    data = inpw((addr & 0x0FFFFFFF));

                addr += 4;
                bytes -= 4;
            }

            if (bytes <= 0)
                bytes = 0;

            /* bytes means remain byte counts */
            if (bytes != 0)
            {
                CRYPTO->HMAC_DATIN = data;
            }
            else
            {
                if ((mode == SHA_ONESHOT) || (mode == SHA_CONTI_END)) /* The last SHA operation */
                {
                    /* It's last word ... *-* */
                    CRYPTO->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk | CRYPTO_HMAC_CTL_DMALAST_Msk;
                    CRYPTO->HMAC_DATIN = data;

                    while (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_BUSY_Msk);

                    for (i = 0; i < 8; i++)
                    {
                        uint32_t u32DGST = CRYPTO->HMAC_DGST[i];

                        Hash[i] = u32DGST;
                    }

                    memcpy(digest, Hash, sizeof(Hash));
                    DBG("\t[SHA256 done]\n");
                    break;
                }
                else
                {
                    CRYPTO->HMAC_DATIN = data;
                    /* Exit to wait next SHA operation */
                }
            }
        }
    }

    return 0;
    /*
        Verify:
            Input:
                    String char -> "32107654" on PC tool
                    HEX byte    -> "3332313037363534" on PC tool and {0x30313233,0x34353637} in device
            Result:
                6952CF8EACE972CD4F10567331B46D85104E9E57402364F205876D13F84F7E42
                ==> Arraty {0x6952CF8E, 0xACE972CD, 0x4F105673....}
    */
}

/*
    * CCITT (0xFFFF)
    * mode: 0: calculate; 1: verify
*/
static uint16_t PACKET_ExecCCITT(uint32_t *pu32buf, uint16_t u32ByteCnt, uint8_t u8Mode)
{
    uint16_t   i;
    uint16_t   *pu16buf, u16OrgSum, u16CalSum;

    if (u32ByteCnt > 56)   // Invalid data byte count
        return -1;

    pu16buf = (uint16_t *)pu32buf;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC_Open(CRC_CCITT, 0, 0xFFFFul, CRC_CPU_WDATA_16);

    for (i = 1; i < (u32ByteCnt / 2); i++)
    {
#if (0)
        DBG("idx-%d: 0x%04x (CCITT)\n", i, pu16buf[i]);
#endif
        CRC->DAT = *(pu16buf + i);
    }

    u16OrgSum = pu16buf[0];
    u16CalSum = (CRC->CHECKSUM & 0xFFFFul);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    if (u8Mode == 0)
    {
        *(pu16buf + 0) = u16CalSum;
        return u16CalSum;
    }
    else if (u8Mode == 1)
    {
        /* Verify CCITT checksum */
        if (u16OrgSum == u16CalSum)
            return 0;   /* Verify CCITT Pass */
        else
        {
            //DBG("u16OrgSum: 0x%08X, u16CalSum: 0x%08X\n", u16OrgSum, u16CalSum);
            return -1;  /* Verify CCITT Fail */
        }
    }
    else
    {
        return -1;
    }
}

/*
    * CRC32
    * mode: 0: calculate; 1: verify
*/
static uint32_t PACKET_ExecCRC32(uint32_t *pu32buf, uint16_t len, uint8_t mode)
{
    uint16_t i;
    uint32_t u32OrgSum, u32CalSum;

    if (len > 60) // valid data byte count
        return -1;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_COM | CRC_CHECKSUM_RVS), 0xFFFFFFFFul, CRC_CPU_WDATA_32);

    for (i = 0; i < (len / 4) - 1; i++)
    {
#if (0)
        DBG("idx-%d: 0x%08x. (CRC32)\n", i, *(pu32buf + i));
#endif
        CRC->DAT = *(pu32buf + i);
    }

    u32OrgSum = *(pu32buf + i);
    u32CalSum = (CRC->CHECKSUM & 0xFFFFFFFFul);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    if (mode == 0)
    {
        *(pu32buf + i) = u32CalSum;
        return u32CalSum;
    }
    else if (mode == 1)
    {
        /* Verify CRC32 checksum */
        if (u32OrgSum == u32CalSum)
            return 0;   /* Verify CRC32 Pass */
        else
            return -1;  /* Verify CRC32 Fail */
    }
    else
    {
        return -1;
    }
}

/**
  * @brief      Perform AES-256 CFB NoPadding encrypt
  */
static int32_t PACKET_AES256Encrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    volatile int32_t    i;

    /* KEY and IV are byte order (32 bit) reversed, Swap32(x) and stored in ISP_INFO_T */
    memcpy((void *)&CRYPTO->AES_KEY[0], KEY, (4 * 8));
    memcpy((void *)&CRYPTO->AES_IV[0], IV, (4 * 4));

    CRYPTO->AES_SADDR = (uint32_t)in;
    CRYPTO->AES_DADDR = (uint32_t)out;
    CRYPTO->AES_CNT   = len;
    CRYPTO->AES_CTL = ((AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos));
    CRYPTO->AES_CTL |= (CRYPTO_AES_CTL_ENCRYPTO_Msk);
    CRYPTO->AES_CTL |= ((AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk);

    while (CRYPTO->AES_STS & CRYPTO_AES_STS_BUSY_Msk) {}

    return 0;
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
static int32_t PACKET_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    /* KEY and IV are byte order (32 bit) reversed, Swap32(x) and stored in ISP_INFO_T */
    memcpy((void *)&CRYPTO->AES_KEY[0], KEY, (4 * 8));
    memcpy((void *)&CRYPTO->AES_IV[0],  IV, (4 * 4));

    CRYPTO->AES_SADDR = (uint32_t)in;
    CRYPTO->AES_DADDR = (uint32_t)out;
    CRYPTO->AES_CNT   = len;
    CRYPTO->AES_CTL = ((AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos));
    CRYPTO->AES_CTL |= ((AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk | CRYPTO_AES_CTL_DMALAST_Msk);

    while (CRYPTO->AES_STS & CRYPTO_AES_STS_BUSY_Msk) {}

    /* [KL@2024.01.25] [Todo] Remove below delay to wait data ready. */
    CLK_SysTickLongDelay(100000);

    return 0;
}

int32_t CMD_GenRspPacket(CMD_PACKET_T *pCMD, ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i;

    /* Generate CCITT */
    PACKET_ExecCCITT((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 8, 0);

    for (i = 0; i < sizeof(pISPInfo->au32AESKey) / 4; i++)
    {
        if (pISPInfo->au32AESKey[i] != 0x0ul)
            break;
    }

    /* if i == 8, NO AES key, do not encrypt the cmd data */
    if (i != 8)
        PACKET_AES256Encrypt(pCMD->au32Data, pCMD->au32Data, sizeof(pCMD->au32Data), pISPInfo->au32AESKey, pISPInfo->au32AESIV);

#if (0)
    {
        uint32_t *pu32;
        pu32 = (uint32_t *)pCMD;
        DBG("AES KEY:\n");

        for (i = 0; i < sizeof(pISPInfo->au32AESKey) / 4; i++)
            DBG("   0x%08x", pISPInfo->au32AESKey[i]);

        DBG("\nAES IV:\n");

        for (i = 0; i < sizeof(pISPInfo->au32AESIV) / 4; i++)
            DBG("   0x%08x", pISPInfo->au32AESIV[i]);

        DBG("\nRSP data(encryption?):\n");

        for (i = 0; i < 2; i++)
            DBG("   0x%08x", pu32[i]);

        for (i = 2; i < sizeof(CMD_PACKET_T) / 4; i++)
        {
            if ((i % 4) == 2)
                DBG("\n");

            DBG("   0x%08x", pu32[i]);
        }

        DBG("\n");
    }
#endif

    /* Generate CRC32 */
    PACKET_ExecCRC32((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 4, 0);

#if (1)
    {
        uint32_t *pu32 = (uint32_t *)pCMD;

        NVT_UNUSED(pu32);

        if (i != 8)
        {
            DBG("AES KEY:\n");

            for (i = 0; i < sizeof(pISPInfo->au32AESKey) / 4; i++)
                DBG("   0x%08x", pISPInfo->au32AESKey[i]);

            DBG("\nAES IV:\n");

            for (i = 0; i < sizeof(pISPInfo->au32AESIV) / 4; i++)
                DBG("   0x%08x", pISPInfo->au32AESIV[i]);

            DBG("\n");
        }

        DBG("Send RSP data:\n");

        for (i = 0; i < 2; i++)
            DBG("   0x%08x", pu32[i]);

        for (i = 2; i < sizeof(CMD_PACKET_T) / 4; i++)
        {
            if ((i % 4) == 2)
                DBG("\n");

            DBG("   0x%08x", pu32[i]);
        }

        DBG("\n");
    }
#endif

    return 0;
}

int32_t CMD_ParseReqPacket(CMD_PACKET_T *pCMD, ISP_INFO_T *pISPInfo)
{
    volatile int32_t  i;

#if (0)
    {
        uint32_t *pu32 = (uint32_t *)pCMD;
        printf("Get REQ data:\n");

        for (i = 0; i < 2; i++)
            printf("   0x%08x", pu32[i]);

        for (i = 2; i < sizeof(CMD_PACKET_T) / 4; i++)
        {
            if ((i % 4) == 2)
                printf("\n");

            printf("   0x%08x", pu32[i]);
        }

        printf("\n");
    }
#endif

    /* verify CRC32 */
    if (PACKET_ExecCRC32((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 4, 1) != 0)
    {
        DBG("\n\tPacket CRC32 mismatch!\n");
        return -1;
    }

    for (i = 0; i < sizeof(pISPInfo->au32AESKey) / 4; i++)
    {
        if (pISPInfo->au32AESKey[i] != 0x0ul)
            break;
    }

    /* if i == 8, NO AES key, do not decrypt the cmd data */
    if (i != 8)
        PACKET_AES256Decrypt(pCMD->au32Data, pCMD->au32Data, sizeof(pCMD->au32Data), pISPInfo->au32AESKey, pISPInfo->au32AESIV);

#if (1)
    {
        uint32_t *pu32 = (uint32_t *)pCMD;

        NVT_UNUSED(pu32);

        if (i != 8)
        {
            DBG("AES KEY:\n");

            for (i = 0; i < sizeof(pISPInfo->au32AESKey) / 4; i++)
                DBG("   0x%08x", pISPInfo->au32AESKey[i]);

            DBG("\nAES IV:\n");

            for (i = 0; i < sizeof(pISPInfo->au32AESIV) / 4; i++)
                DBG("   0x%08x", pISPInfo->au32AESIV[i]);

            DBG("\n");
        }

        DBG("[Decrypt] Get REQ data:\n");

        for (i = 0; i < 2; i++)
            DBG("   0x%08x", pu32[i]);

        for (i = 2; i < sizeof(CMD_PACKET_T) / 4; i++)
        {
            if ((i % 4) == 2)
                DBG("\n");

            DBG("   0x%08x", pu32[i]);
        }

        DBG("\n");
    }
#endif

    /* verify CCITT */
    if (PACKET_ExecCCITT((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 8, 1) != 0)
    {
        DBG("\n\tPacket CCITT mismatch !\n");
        return -1;
    }

    DBG("Parse cmd Pass.\n\n");
    return 0;
}

void CommandHandlerInit(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP */
    FMC_ENABLE_ISP();
    FMC_DISABLE_AP_UPDATE();
    FMC_DISABLE_LD_UPDATE();
    FMC_DISABLE_CFG_UPDATE();

    /* Enable CRYPTO and TRNG clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);
    CLK_EnableModuleClock(TRNG0_MODULE);
    /* Enable ECC IRQ */
    ECC_ENABLE_INT(CRYPTO);
    NVIC_EnableIRQ(CRYPTO_IRQn);
    /* Initial Random Number Generator */
    RNG_Open();
}

static int32_t GenRandomIV(ISP_INFO_T *pISPInfo)
{
    uint32_t    au32Buf[8];

    if (RNG_Random(au32Buf, 8) == 0)
        return -1;

    memcpy(pISPInfo->au32AESIV, au32Buf, (128 / 8)); // size is 128-bit

    return 0;
}

/*
    Stage 1.
    Valid commands:
        "CMD_RESYNC"     "CMD_CONNECT"   "CMD_DISCONNECT"
*/
int32_t ParseCONNECT(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i, ret = 0, cmd_case;
    CMD_PACKET_T        cmd;
    uint32_t            u32Data;

    memset(&cmd,             0x0, sizeof(CMD_PACKET_T));
    memset(pISPInfo->rspbuf, 0x0, sizeof(pISPInfo->rspbuf));

    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    do
    {
        /* plaint text cmd */
        if (cmd.u16CmdID == CMD_RESYNC)
        {
            cmd.au32Data[0] = STS_OK;
            cmd.u16Len      = (4 * 1);
            ret = cmd.u16CmdID;
            pISPInfo->IsConnectOK = 0;
            break;
        }

        if (CMD_ParseReqPacket(&cmd, pISPInfo) != 0)
        {
            DBG("*** [Pasre error: 0x%x] ***\n", cmd.u16CmdID);
            memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

            cmd.au32Data[0] = ERR_CMD_CHECKSUM;
            cmd.u16Len      = (4 * 1);
            ret = -1;
            break;
        }
        else
        {
            cmd_case = cmd.u16CmdID;

            switch (cmd_case)
            {
                case CMD_DISCONNECT:
                    DBG("[CMD_DISCONNECT] (stage 1)\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    pISPInfo->IsConnectOK = 0;
                    break;

                case CMD_CONNECT:
                    DBG("[CMD_CONNECT] (stage 1)\n");

                    for (i = 0; i < 4; i++)
                    {
                        u32Data = Swap32(cmd.au32Data[i]);
                        pISPInfo->au32AESIV[i] = u32Data;
                    }

                    for (i = 0; i < 4; i++)
                    {
                        if (pISPInfo->au32AESIV[i] != 0)
                            break;
                    }

                    if (i == 4)
                    {
                        /* NO AESIV. Client will generate a new random AES IV */
                        GenRandomIV(pISPInfo);
                    }

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.au32Data[1] = SYS->PDID;

                    for (i = 0; i < 4; i++)
                    {
                        u32Data = Swap32(pISPInfo->au32AESIV[i]);
                        cmd.au32Data[2 + i] = u32Data;
                    }

                    cmd.au32Data[6] = pISPInfo->u32CmdMask;
                    cmd.au32Data[7] = 0x0;//FMC->KPKEYSTS; /* KPKEYSTS: bit-4:KEYFLAG(1:enable/0:disable), bit-3:FORBID */
                    cmd.au32Data[8] = 0x0;//(((FMC->KPKEYCNT&0xFFFF)<<16) | (FMC->KPCNT&0xFFFF));

                    DBG("PDID:  0x%08x.\n", cmd.au32Data[1]);

                    for (i = 0; i < 4; i++)
                        DBG("Target IV[%d]: 0x%08x.\n", i, pISPInfo->au32AESIV[i]);

                    cmd.u16Len      = (4 * 9);
                    ret = cmd.u16CmdID;
                    pISPInfo->IsConnectOK = 1;
                    break;

                default:
                    DBG("*** [Cmd error: 0x%x] ***\n", cmd.u16CmdID);
                    cmd.au32Data[0] = ERR_CMD_INVALID;
                    cmd.u16Len      = (4 * 1);
                    ret = -1;
                    break;
            }
        }

        if (ret == -1)
            pISPInfo->IsConnectOK = 0;
    } while (0);

    CMD_GenRspPacket(&cmd, pISPInfo);

    /* Prepare respone data done */
    memcpy(pISPInfo->rspbuf, (uint8_t *)&cmd, sizeof(CMD_PACKET_T));

    DBG("Repone.\n\n");
    return  ret;
}

static int32_t GenECDHKey(uint32_t *PrivKey, ECC_PUBKEY_T *PubKey, uint32_t *Key)
{
    volatile int32_t    ret = 0;
    char                d[70], Qx[70], Qy[70], z[70];
    uint32_t            tmp[8];

    memset(d,  0x0, sizeof(d));
    memset(Qx, 0x0, sizeof(Qx));
    memset(Qy, 0x0, sizeof(Qy));
    memset(z,  0x0, sizeof(z));

    /* Set d */
    /* B0, B1, B2 ... B31 to B31, B62, ... B0 */
    memcpy(tmp, (char *)PrivKey, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, d);
    memset(PrivKey, 0x0, (ECC_KEY_SIZE / 8));

    /* Set Qx */
    memcpy(tmp, (char *)PubKey->au32Key0, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, Qx);

    /* Set Qy */
    memcpy(tmp, (char *)PubKey->au32Key1, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, Qy);

    DBG("d:  %s\n", d);
    DBG("Qx: %s\n", Qx);
    DBG("Qy: %s\n", Qy);
    ret = ECC_GenerateSecretZ(CRYPTO, CURVE_P_SIZE, d, Qx, Qy, z);

    if (ret < 0)
    {
        memset(d,  0x0, sizeof(d));
        memset(Qx, 0x0, sizeof(Qx));
        memset(Qy, 0x0, sizeof(Qy));
        memset(z,  0x0, sizeof(z));
        return -1;
    }

    DBG("z:  %s\n", z);

    Hex2Reg(z, Key);
    BytesSwap((char *)Key, (ECC_KEY_SIZE / 8));

    memset(d,  0x0, sizeof(d));
    memset(Qx, 0x0, sizeof(Qx));
    memset(Qy, 0x0, sizeof(Qy));
    memset(z,  0x0, sizeof(z));

    DBG("\n");
    return 0;
}

static int32_t GenRandomPrivKey(char *d, int32_t nbits)
{
    volatile int32_t    i, j;
    uint32_t             au32Buf[8];
    uint8_t             *pu8Buf, u8Data;

    do
    {
        /* Generate random number for private key */
        if (RNG_Random(au32Buf, 8) == 0)
            return -1;

        pu8Buf = (uint8_t *)au32Buf;

        for (i = 0, j = 0; i < (nbits / 8); i++)
        {
            u8Data = B2C(pu8Buf[i] & 0xf);
            d[j++] = u8Data;
            u8Data = B2C(pu8Buf[i] >> 4);
            d[j++] = u8Data;
        }

        d[j] = 0; // NULL end

        /* Check if the private key valid */
        if (ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_SIZE, d))
        {
            return 0;
        }
        else
        {
            /* Invalid key */
            DBG("Current private key is not valid. Need a new one.\n");
            return -1;
        }
    } while (1);
}

static int32_t GenRandomECCKeyPair(uint32_t *priv, ECC_PUBKEY_T *pubkey)
{
    volatile int32_t    ret;
    char                d[70], k0[70], k1[70];

    memset(d,  0x0, sizeof(d));
    memset(k0, 0x0, sizeof(k0));
    memset(k1, 0x0, sizeof(k1));

    do
    {
        /* select random k */
        if ((ret = GenRandomPrivKey(d, ECC_KEY_SIZE)) != 0)
        {
            break;
        }

        if ((ret = ECC_GeneratePublicKey(CRYPTO, CURVE_P_SIZE, d, k0, k1)) != 0)
        {
            DBG("\nECC key generation failed!!\n\n");
            break;
        }

        DBG("d:  %s\n", d);
        DBG("k0: %s\n", k0);
        DBG("k1: %s\n", k1);
        Hex2Reg(d,  priv);
        Hex2Reg(k0, pubkey->au32Key0);
        Hex2Reg(k1, pubkey->au32Key1);
        BytesSwap((char *)priv,             sizeof(pubkey->au32Key0)); // 256-bits
        BytesSwap((char *)pubkey->au32Key0, sizeof(pubkey->au32Key0));
        BytesSwap((char *)pubkey->au32Key1, sizeof(pubkey->au32Key1));
    } while (0);

    memset(d,  0x0, sizeof(d));
    memset(k0, 0x0, sizeof(k0));
    memset(k1, 0x0, sizeof(k1));

    return ret;
}

/*
    Stage 2.
    Valid commands:
        "CMD_RESYNC"             "CMD_DISCONNECT"
        "CMD_ECDH_PUB0"          "CMD_ECDH_PUB1"
        "CMD_ECDH_GET_PUB0"      "CMD_ECDH_GET_PUB1"
        "CMD_ECDH_RAND_PUB0"     "CMD_ECDH_RAND_PUB1"
        "CMD_ECDH_GET_RAND_PUB0" "CMD_ECDH_GET_RAND_PUB1"
*/
int32_t ParseECDH(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i, ret = 0, cmd_case;
    CMD_PACKET_T        cmd;
    uint32_t            AESKey[8], u32Data;
    ECC_PUBKEY_T        randpub;
    uint32_t            priv[8];

    memset(&cmd,             0x0, sizeof(CMD_PACKET_T));
    memset(pISPInfo->rspbuf, 0x0, sizeof(pISPInfo->rspbuf));

    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    do
    {
        /* plaint text cmd */
        if (cmd.u16CmdID == CMD_RESYNC)
        {
            cmd.au32Data[0] = STS_OK;
            cmd.u16Len      = (4 * 1);
            ret = cmd.u16CmdID;
            pISPInfo->IsConnectOK = 0;
            break;
        }

        if (CMD_ParseReqPacket(&cmd, pISPInfo) != 0)
        {
            DBG("*** [Pasre error: 0x%x] ***\n", cmd.u16CmdID);
            memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

            cmd.au32Data[0] = ERR_CMD_CHECKSUM;
            cmd.u16Len      = (4 * 1);
            ret = -1;
            break;
        }
        else
        {
            cmd_case = cmd.u16CmdID;

            switch (cmd_case)
            {
                case CMD_DISCONNECT:
                    DBG("[CMD_DISCONNECT] (stage 2)\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    pISPInfo->IsConnectOK = 0;
                    break;

                case CMD_ECDH_PUB0:
                    /* Get Server public key 0 */
                    memcpy(pISPInfo->ServerPubKey.au32Key0, cmd.au32Data, cmd.u16Len);

                    for (i = 0; i < 8; i++)
                        DBG("Get pub0[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key0[i]);

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_PUB1:
                    /* Get Server public key 1 and generate ist ECDH AES key */
                    memcpy(pISPInfo->ServerPubKey.au32Key1, cmd.au32Data, cmd.u16Len);

                    for (i = 0; i < 8; i++)
                        DBG("Get pub1[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key1[i]);

                    /* TODO : user can identify server's public key here... */

                    if (pISPInfo->pfnGenKeyFunc == 0)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = -1;
                        break;
                    }

                    if (pISPInfo->pfnGenKeyFunc(&pISPInfo->ServerPubKey, AESKey) != 0)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = -1;
                        break;
                    }

                    //                    /* Generate 1st ECDH AES key */
                    //                    Hex2Reg((char *)gacPrivKey,  priv);
                    //                    BytesSwap((char *)priv, sizeof(priv)); // 256-bits
                    //                    if(GenECDHKey(priv, &pISPInfo->ServerPubKey, AESKey) != 0)
                    //                    {
                    //                        cmd.au32Data[0] = ERR_PARAMETER;
                    //                        cmd.u16Len      = (4 * 1);
                    //                        ret = -1;
                    //                        break;
                    //                    }
                    for (i = 0; i < 8; i++)
                    {
                        u32Data = Swap32(AESKey[i]);
                        AESKey[i] = u32Data;
                    }

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_GET_PUB0: // MUST be encrypted
                    /* Response encrypted Client public key 0 */
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    memcpy(&cmd.au32Data[1], pISPInfo->ClientPubKey.au32Key0, sizeof(pISPInfo->ClientPubKey.au32Key0));

                    for (i = 0; i < 8; i++)
                        DBG("Out pub0[%d]: 0x%08x.\n", i, cmd.au32Data[1 + i]);

                    cmd.u16Len      = (4 * (1 + 8));
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_GET_PUB1: // MUST be encrypted
                    /* Response encrypted Client public key 1 */
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    memcpy(&cmd.au32Data[1], pISPInfo->ClientPubKey.au32Key1, sizeof(pISPInfo->ClientPubKey.au32Key1));

                    for (i = 0; i < 8; i++)
                        DBG("Out pub1[%d]: 0x%08x.\n", i, cmd.au32Data[1 + i]);

                    cmd.u16Len      = (4 * (1 + 8));
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_RAND_PUB0: // MUST be encrypted
                    /* Get Server random public key 0 */
                    memcpy(pISPInfo->ServerPubKey.au32Key0, cmd.au32Data, cmd.u16Len);

                    for (i = 0; i < 8; i++)
                        DBG("Get rand_pub0[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key0[i]);

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_RAND_PUB1: // MUST be encrypted
                    /* Get Server random public key 1 */
                    memcpy(pISPInfo->ServerPubKey.au32Key1, cmd.au32Data, cmd.u16Len);

                    for (i = 0; i < 8; i++)
                        DBG("Get rand_pub1[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key1[i]);

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_GET_RAND_PUB0: // MUST be encrypted

                    /* Generate Client random ECC key pair */
                    if (GenRandomECCKeyPair(priv, &randpub) != 0)
                    {
                        memset(priv, 0x0, sizeof(priv));
                        memset(&randpub, 0x0, sizeof(ECC_PUBKEY_T));
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = -1;
                        break;
                    }

                    /* Generate 2nd ECDH AES key and update in CMD_ECDH_RSP_RAND_PUB1 cmd */
                    if (GenECDHKey(priv, &pISPInfo->ServerPubKey, AESKey) != 0)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = -1;
                        break;
                    }

                    for (i = 0; i < 8; i++)
                    {
                        u32Data = Swap32(AESKey[i]);
                        AESKey[i] = u32Data;
                    }

                    memcpy(pISPInfo->tmp0, AESKey, sizeof(AESKey));

                    memcpy(pISPInfo->ClientPubKey.au32Key0, randpub.au32Key0, sizeof(randpub.au32Key0));
                    memcpy(pISPInfo->ClientPubKey.au32Key1, randpub.au32Key1, sizeof(randpub.au32Key1));
                    memset(priv, 0x0, sizeof(priv));
                    memset(&randpub, 0x0, sizeof(ECC_PUBKEY_T));

                    /* Response encrypted Client random public key 0 */
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    memcpy(&cmd.au32Data[1], pISPInfo->ClientPubKey.au32Key0, sizeof(pISPInfo->ClientPubKey.au32Key0));

                    for (i = 0; i < 8; i++)
                        DBG("Out rand_pub0[%d]: 0x%08x.\n", i, cmd.au32Data[1 + i]);

                    cmd.u16Len      = (4 * (1 + 8));
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ECDH_GET_RAND_PUB1: // MUST be encrypted
                    /* Response encrypted Client random public key 1 */
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    memcpy(&cmd.au32Data[1], pISPInfo->ClientPubKey.au32Key1, sizeof(pISPInfo->ClientPubKey.au32Key1));

                    for (i = 0; i < 8; i++)
                        DBG("Out rand_pub1[%d]: 0x%08x.\n", i, cmd.au32Data[1 + i]);

                    cmd.u16Len      = (4 * (1 + 8));
                    ret = cmd.u16CmdID;
                    break;

                default:
                    DBG("*** [Cmd error: 0x%x] ***\n", cmd.u16CmdID);
                    cmd.au32Data[0] = ERR_CMD_INVALID;
                    cmd.u16Len      = (4 * 1);
                    ret = -1;
                    break;
            }
        }

        if (ret == -1)
            pISPInfo->IsConnectOK = 0;
    } while (0);

    CMD_GenRspPacket(&cmd, pISPInfo);

    memcpy(pISPInfo->rspbuf, (uint8_t *)&cmd, sizeof(CMD_PACKET_T));

    if ((ret == CMD_ECDH_PUB1) || (ret == CMD_ECDH_GET_RAND_PUB1))
    {
        if (ret == CMD_ECDH_PUB1)
        {
            /* Update 1st ECDH AES key to pISPInfo */
            memcpy(pISPInfo->au32AESKey, AESKey, sizeof(AESKey));

            for (i = 0; i < 8; i++)
                DBG("Gen 1st ECDH AES KEY[%d]: 0x%08x.\n", i, pISPInfo->au32AESKey[i]);
        }
        else
        {
            /* Update 2nd ECDH AES key to pISPInfo */
            memcpy(pISPInfo->au32AESKey, pISPInfo->tmp0, sizeof(AESKey));

            for (i = 0; i < 8; i++)
                DBG("Gen 2nd ECDH AES KEY[%d]: 0x%08x.\n", i, pISPInfo->au32AESKey[i]);

            memset(pISPInfo->tmp0, 0x0, sizeof(AESKey));
        }
    }

    memset(AESKey, 0x0, sizeof(AESKey));

    DBG("Repone.\n\n");
    return ret;
}

// [Begin] TESTCHIP_ONLY - This function should be removed in M55M1.
uint32_t FMC_GetChkSum(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32CRC32Checksum = 0xFFFFFFFF;

    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFFul, CRC_CPU_WDATA_32);
    CRC_SET_DMA_SADDR(CRC, u32StartAddr);
    CRC_SET_DMACNT_WORD(CRC, u32ByteSize / 4);
    CRC_DMA_START(CRC);

    while ((CRC_GET_STATUS(CRC) & CRC_DMASTS_FINISH_Msk) == 0)
        ;

    CRC->DMASTS = CRC_DMASTS_FINISH_Msk;
    u32CRC32Checksum = CRC_GetChecksum();

    return u32CRC32Checksum;
}
// [End] TESTCHIP_ONLY

int32_t _PageErase(uint32_t addr, uint32_t count, uint32_t u32CmdMask)
{
    volatile int32_t    i, ret = -1;

    DBG("[Page erase] addr: 0x%x, page counts: %d.\n", addr, count);

    addr &= ~NS_OFFSET;

    /* Check address and counts */
    if (((addr % FMC_FLASH_PAGE_SIZE) != 0) || (count == 0))
        return ERR_PAGE_ALIGN;

    /* Not use u32CmdMask */
    //    /* Check for erase LDROM or APROM */
    //    if(addr & FMC_LDROM_BASE)
    //    {
    //        if(u32CmdMask & MASK_UPDATE_LDROM)
    //            return CMD_IS_MASKED;
    //    }
    //    else
    //    {
    //        if(u32CmdMask & MASK_UPDATE_APROM)
    //            return CMD_IS_MASKED;
    //    }

    if ((addr < GetMaxAPROMSize()) && (addr >= FMC_APROM_BASE))
    {
        if (count > (GetMaxAPROMSize() / FMC_FLASH_PAGE_SIZE))
            return ERR_OVER_RANGE;

        if ((addr + (count * FMC_FLASH_PAGE_SIZE)) > GetMaxAPROMSize())
            return ERR_OVER_RANGE;

        FMC_ENABLE_AP_UPDATE();
    }
    else if ((addr < FMC_LDROM_END) && (addr >= FMC_LDROM_BASE))
    {
        if (count > (FMC_LDROM_SIZE / FMC_FLASH_PAGE_SIZE))
            return ERR_OVER_RANGE;

        if ((addr + (count * FMC_FLASH_PAGE_SIZE)) > FMC_LDROM_END)
            return ERR_OVER_RANGE;

        FMC_ENABLE_LD_UPDATE();
    }
    else
    {
        return ERR_OVER_RANGE;
    }

    for (i = 0; i < count; i++)
    {
        ret = ERR_ISP_ERASE;

        /* perform erase page */
        if (FMC_Erase(addr) < 0)
            break;

        /* Run 2 page checksum and verify its result */
        if (FMC_GetChkSum(addr, FMC_FLASH_PAGE_SIZE) != ALL_ONE_2PAGE_CHKS)
            break;

        addr += FMC_FLASH_PAGE_SIZE;
        ret = 0;
    }

    FMC_DISABLE_AP_UPDATE();
    FMC_DISABLE_LD_UPDATE();
    return ret;
}

static int32_t _IsValidFlashRegion(uint32_t addr, uint32_t size, uint32_t u32CmdMask)
{
    DBG("[Check flash region] addr: 0x%x, size: %d.\n", addr, size);

    /* Not use u32CmdMask */

    addr &= ~NS_OFFSET;

    /* Check address and length */
    if ((addr % 4) != 0)
        return ERR_INVALID_ADDRESS;

    if (((size % 4) != 0) || (size == 0))
        return ERR_INVALID_ADDRESS;

    /* Not use u32CmdMask */
    //    /* Check for erase LDROM or APROM */
    //    if(addr & FMC_LDROM_BASE)
    //    {
    //        if(u32CmdMask & MASK_UPDATE_LDROM)
    //            return CMD_IS_MASKED;
    //    }
    //    else
    //    {
    //        if(u32CmdMask & MASK_UPDATE_APROM)
    //            return CMD_IS_MASKED;
    //    }

    if ((addr < GetMaxAPROMSize()) && (addr >= FMC_APROM_BASE))
    {
        if (((addr + size) > GetMaxAPROMSize()) || ((addr + size) < addr))
            return ERR_OVER_RANGE;

        FMC_ENABLE_AP_UPDATE();
    }
    else if ((addr < FMC_LDROM_END) && (addr >= FMC_LDROM_BASE))
    {
        if (((addr + size) > FMC_LDROM_END) || ((addr + size) < addr))
            return ERR_OVER_RANGE;

        FMC_ENABLE_LD_UPDATE();
    }
    else
    {
        return ERR_OVER_RANGE;
    }

    return 0;
}

static int32_t _ISPWrite(uint32_t addr, uint32_t data)
{
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = addr;
    FMC->ISPDAT = data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    /* Clear ISPFF if cmd FAIL */
    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        return -1;
    }

    return 0;
}

static uint32_t _ISPRead(uint32_t addr)
{
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) { }

    /* Clear ISPFF if cmd FAIL */
    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        return -1;
    }

    return FMC->ISPDAT;
}

static int32_t _WriteFlash(uint32_t addr, uint32_t size, uint32_t *pu32Data, uint32_t u32CmdMask)
{
    volatile int32_t    i, ret = -1;

    DBG("[Write flash] addr: 0x%x, size: %d.\n", addr, size);

    if ((ret = _IsValidFlashRegion(addr, size, u32CmdMask)) != 0)
        return ret;

    addr &= ~NS_OFFSET;

    for (i = 0; i < (size / 4); i++)
    {
        /* write */
        if (_ISPWrite(addr, pu32Data[i]) != 0)
        {
            ret = ERR_ISP_WRITE;
            break;
        }

        /* verify */
        if (_ISPRead(addr) != pu32Data[i])
        {
            ret = ERR_ISP_WRITE;
            break;
        }

        addr += 4;
        ret = 0;
    }

    FMC_DISABLE_AP_UPDATE();
    FMC_DISABLE_LD_UPDATE();
    return ret;
}

/*
    Stage 3.
    Valid commands:
        "CMD_RESYNC"         "CMD_DISCONNECT"
        "CMD_GET_VERSION"    "CMD_RESET"
        "CMD_WRITE"          "CMD_ERASE"
        "CMD_SET_MASS_WRITE" "CMD_MASS_WRITE"
        "CMD_READ_CONFIG"    "CMD_UPDATE_CFG"
        "CMD_READ_OTP"       "CMD_WRITE_OTP" // Do NOT use "CMD_WRITE_OTP"
        "CMD_SET_REGION_LOCK"
        "CMD_XOM_SET"        "CMD_XOM_ERASE"
        "CMD_ERASE_KPROM"    "CMD_SET_KPROM"     "CMD_AUTH_KPROM"    "CMD_KPROM_STS"
        "CMD_GET_RAND_IV"    "CMD_SET_RAND_IV"   "CMD_GET_ID_SIGNATURE"
        "CMD_IDENTIFY_SERVER"
        "CMD_MASS_ERASE"
        "CMD_SET_NS"
*/
int32_t ParseCommands(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i, ret = 0, cmd_case, tmp;
    CMD_PACKET_T        cmd;
    uint32_t            addr, size;
    uint32_t            msg[8], R[8], S[8];
    ECC_PUBKEY_T        PubKey;

    memset(&cmd,             0x0, sizeof(CMD_PACKET_T));
    memset(pISPInfo->rspbuf, 0x0, sizeof(pISPInfo->rspbuf));
    memset(msg, 0x0, sizeof(msg));
    memset(R, 0x0, sizeof(R));
    memset(S, 0x0, sizeof(S));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));

    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    do
    {
        /* plaint text cmd */
        if (cmd.u16CmdID == CMD_RESYNC)
        {
            cmd.au32Data[0] = STS_OK;
            cmd.u16Len      = (4 * 1);
            ret = cmd.u16CmdID;
            pISPInfo->IsConnectOK = 0;
            break;
        }

        if (CMD_ParseReqPacket(&cmd, pISPInfo) != 0)
        {
            DBG("*** [Parse error: 0x%x] ***\n", cmd.u16CmdID);
            memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

            cmd.au32Data[0] = ERR_CMD_CHECKSUM;
            cmd.u16Len      = (4 * 1);
            ret = -1;
            break;
        }
        else
        {
            cmd_case = cmd.u16CmdID;

            switch (cmd_case)
            {
                case CMD_DISCONNECT:
                    DBG("[CMD_DISCONNECT] (stage 3)\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    pISPInfo->IsConnectOK = 0;
                    break;

                case CMD_GET_VERSION:
                    DBG("[CMD_GET_VERSION]\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.au32Data[1] = LIB_VERSION;
                    cmd.u16Len      = (4 * 2);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_ERASE:
                    DBG("[CMD_ERASE]\n");
                    ret = _PageErase(cmd.au32Data[0], cmd.au32Data[1], pISPInfo->u32CmdMask);
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = (ret == 0) ? STS_OK : ret;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_WRITE:
                    DBG("[CMD_WRITE] (0x%x, 0x%x, 0x%x)\n", cmd.au32Data[0], cmd.au32Data[1], cmd.au32Data[2]);
                    addr = cmd.au32Data[0];
                    size = cmd.au32Data[1];
                    ret = _WriteFlash(addr, size, &cmd.au32Data[2], pISPInfo->u32CmdMask);
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = (ret == 0) ? STS_OK : ret;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_SET_MASS_WRITE:
                    DBG("[CMD_SET_MASS_WRITE] (0x%x, 0x%x)\n", cmd.au32Data[0], cmd.au32Data[1]);
                    ret = _IsValidFlashRegion(cmd.au32Data[0], cmd.au32Data[1], pISPInfo->u32CmdMask);

                    if (ret == 0)
                    {
                        pISPInfo->tmp0[0] = cmd.au32Data[0]; // store write address
                        pISPInfo->tmp0[1] = cmd.au32Data[1]; // store write data length
                        FMC_DISABLE_AP_UPDATE();
                        FMC_DISABLE_LD_UPDATE();
                    }

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = (ret == 0) ? STS_OK : ret;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_MASS_WRITE:
                    DBG("[CMD_MASS_WRITE]\n");
                    addr = pISPInfo->tmp0[0] + (cmd.u16PacketID * 48); // maximum data length is 48
                    size = cmd.u16Len;
                    ret = _WriteFlash(addr, size, cmd.au32Data, pISPInfo->u32CmdMask);
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = (ret == 0) ? STS_OK : ret;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_GET_ID:
                    DBG("[CMD_GET_ID]\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.au32Data[1] = SYS->PDID;
                    cmd.au32Data[2] = FMC_ReadUID(0);
                    cmd.au32Data[3] = FMC_ReadUID(1);
                    cmd.au32Data[4] = FMC_ReadUID(2);
                    cmd.au32Data[5] = FMC_ReadUCID(0);
                    cmd.au32Data[6] = FMC_ReadUCID(1);
                    cmd.au32Data[7] = FMC_ReadUCID(2);
                    cmd.au32Data[8] = FMC_ReadUCID(3);
                    cmd.au32Data[9] = FMC_ReadCID();
                    cmd.au32Data[10] = 0x0ul;
                    cmd.u16Len      = (4 * 11);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_READ_CONFIG:
                    DBG("[CMD_READ_CONFIG]\n");
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = STS_OK;
                    cmd.au32Data[1] = FMC_Read(FMC_CONFIG_BASE + 0x0); // CFG0
                    cmd.au32Data[2] = (uint32_t)SCU->FNSADDR;
                    cmd.au32Data[3] = 0x0ul;        //((NuBL_ISPRead(FMC_SCRLOCK_BASE) & 0xFF) << 8) | (NuBL_ISPRead(FMC_ARLOCK_BASE) & 0xFF);
                    cmd.au32Data[4] = GetMaxAPROMSize();
                    cmd.au32Data[5] = 0x0ul;        //FMC->KPKEYSTS;
                    cmd.au32Data[6] = FMC_Read(FMC_CONFIG_BASE + 0xC); // CFG3
                    cmd.au32Data[7] = FMC->XOMSTS;
                    cmd.au32Data[8] = FMC->XOMR0STS;
                    cmd.au32Data[9] = FMC->XOMR1STS;
                    cmd.au32Data[10] = FMC->XOMR2STS;
                    cmd.au32Data[11] = FMC->XOMR3STS;
                    cmd.u16Len      = (4 * 12);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_RESET:
                    DBG("[CMD_RESET] (0x%x)\n", cmd.au32Data[0]);

                    if (cmd.au32Data[0] > 2)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = cmd.u16CmdID;
                        break;
                    }

                    if (cmd.au32Data[0] == 0)
                    {
                        SYS_ResetChip();
                    }
                    else if (cmd.au32Data[0] == 1)
                    {
                        NVIC_SystemReset();
                    }
                    else if (cmd.au32Data[0] == 2)
                    {
                        SYS_ResetCPU();
                    }

                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                case CMD_EXEC_VENDOR_FUNC:
                    DBG("[CMD_EXEC_VENDOR_FUNC] (Org ID: 0x%x; Vendor ID: 0x%x)\n", cmd_case, cmd.au32Data[1]);

                    if (cmd.u16Len < 4)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = cmd.u16CmdID;
                        break;
                    }

                    if ((cmd.u16Len = pISPInfo->pfnVendorFunc((uint32_t *)&cmd, 0)) == 0)
                    {
                        cmd.au32Data[0] = ERR_PARAMETER;
                        cmd.u16Len      = (4 * 1);
                        ret = cmd.u16CmdID;
                        break;
                    }
                    else
                    {
                        ret = cmd.u16CmdID;
                    }

                    break;

                case CMD_IS_MASKED:
                    DBG("[CMD_IS_MASKED] (0x%x)\n", cmd.u16CmdID);
                    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

                    cmd.au32Data[0] = CMD_IS_MASKED;
                    cmd.u16Len      = (4 * 1);
                    ret = cmd.u16CmdID;
                    break;

                default:
                    DBG("*** [Cmd error: 0x%x] ***\n", cmd.u16CmdID);
                    cmd.au32Data[0] = ERR_CMD_INVALID;
                    cmd.u16Len      = (4 * 1);
                    ret = -1;
                    break;
            }
        }

        if (ret == -1)
            pISPInfo->IsConnectOK = 0;
    } while (0);

    memset(msg, 0x0, sizeof(msg));
    memset(R, 0x0, sizeof(R));
    memset(S, 0x0, sizeof(S));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));

    CMD_GenRspPacket(&cmd, pISPInfo);

    memcpy(pISPInfo->rspbuf, (uint8_t *)&cmd, sizeof(CMD_PACKET_T));

    if ((ret == CMD_GET_RAND_IV) || (ret == CMD_SET_RAND_IV))
    {
        /* Update AESIV to pISPInfo */
        memcpy(pISPInfo->au32AESIV, pISPInfo->tmp0, (128 / 8)); // 128-bits
    }

    DBG("Repone.\n\n");
    return ret;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
