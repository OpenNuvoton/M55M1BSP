/**************************************************************************//**
 * @file     NuBL2_lib0.c
 * @version  V1.00
 * @brief    NuBL2 library source code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"

//#define printf(...)

#define DEBUG_CMD 0
void NuBL2_BytesSwap(char *buf, int32_t len);
void SetECCRegisters(int32_t i32Mode, uint32_t *pPriv, uint32_t *pPub);
void GetNuBL2AES256Key(uint32_t *key);
uint16_t cmd_CalCRC16Sum(uint32_t *pu32buf, uint32_t len);
int32_t cmd_VerifyCRC16Sum(uint32_t *pu32buf);
int32_t GenCmdSessionKey(uint32_t key[]);
int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t i32Mode);

static void SetNuBL2PrivKey(void)
{
    extern const char gc_strOTA_PrivKey[];
    int i;
    uint32_t au32PrivKey[8];

    // Set NuBL2 private key
    Hex2Reg((char *)gc_strOTA_PrivKey, au32PrivKey);

    for (i = 0; i < 8; i++)
    {
        CRYPTO->ECC_K[i] = au32PrivKey[i];
        DEBUG_MSG("NuBL2 PrivKey[%d]: 0x%08X\n", i, au32PrivKey[i]);
    }
}

void SetECCRegisters(int32_t i32Mode, uint32_t *pPriv, uint32_t *pPub)
{
    int32_t         i;
    ECC_PUBKEY_T    *pPubKey;

    if (i32Mode == 0x00020000)
    {
        pPubKey = (ECC_PUBKEY_T *)pPub;
        SetNuBL2PrivKey();
        NuBL2_BytesSwap((char *)pPubKey->au32Key0, sizeof(pPubKey->au32Key0));
        NuBL2_BytesSwap((char *)pPubKey->au32Key1, sizeof(pPubKey->au32Key1));

        for (i = 0; i < 8; i++)
            CRYPTO->ECC_X1[i] = pPubKey->au32Key0[i];

        for (i = 0; i < 8; i++)
            CRYPTO->ECC_Y1[i] = pPubKey->au32Key1[i];
    }
    else if (i32Mode == 0x000F000F)
    {
        pPubKey = (ECC_PUBKEY_T *)pPub;

        for (i = 0; i < 8; i++)
            CRYPTO->ECC_K[i] = pPriv[i];

        for (i = 0; i < 8; i++)
            CRYPTO->ECC_X1[i] = pPubKey->au32Key0[i];

        for (i = 0; i < 8; i++)
            CRYPTO->ECC_Y1[i] = pPubKey->au32Key1[i];
    }
}

//iv = "1000000000000000000000000000000a"
//CRYPTO->AES0_IV[0] = 0x10000000;
//CRYPTO->AES0_IV[1] = 0x00000000;
//CRYPTO->AES0_IV[2] = 0x00000000;
//CRYPTO->AES0_IV[3] = 0x0000000a;

/* These AES256 keys are used to encrypt/decrypt Public Key Storage in ECDSA verification mode */
void GetNuBL2AES256Key(uint32_t *key)
{
    key[0] = 0x12345678;
    key[1] = 0x22222222;
    key[2] = 0x33333333;
    key[3] = 0x44444444;
    key[4] = 0x55555555;
    key[5] = 0x66666666;
    key[6] = 0x77777777;
    key[7] = 0x89abcdef;
}

void NuBL2_BytesSwap(char *buf, int32_t len)
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

uint16_t cmd_CalCRC16Sum(uint32_t *pu32buf, uint32_t len)
{
    volatile uint32_t   i;
    uint16_t            *pu16buf;

    if (len > 56) // data byte count
        return (uint16_t) - 1;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL = (CRC_16 | CRC_CPU_WDATA_16) | CRC_CTL_CRCEN_Msk;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    pu16buf = (uint16_t *)pu32buf;
    CRC->DAT = *(pu16buf + 1);
    CRC->DAT = *(pu16buf + 2);
    CRC->DAT = *(pu16buf + 3);

    for (i = 0; i < (len / 2); i++)
        CRC->DAT = *(pu16buf + 4 + i);

    *(pu16buf + 0) = (CRC->CHECKSUM & 0xFFFF);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    return *(pu16buf + 0);
}

int32_t cmd_VerifyCRC16Sum(uint32_t *pu32buf)
{
    uint32_t   i, len;
    uint16_t            *pu16buf, ChkSum0, ChkSum1;

    pu16buf = (uint16_t *)pu32buf;
    len = pu32buf[1];

    if (len > 56) // data byte count
        return -1;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL = (CRC_16 | CRC_CPU_WDATA_16) | CRC_CTL_CRCEN_Msk;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    CRC->DAT = *(pu16buf + 1);
    CRC->DAT = *(pu16buf + 2);
    CRC->DAT = *(pu16buf + 3);

    for (i = 0; i < (len / 2); i++)
        CRC->DAT = *(pu16buf + 4 + i);

    ChkSum0 = *(pu16buf + 0);
    ChkSum1 = (CRC->CHECKSUM & 0xFFFF);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    /* Verify CRC16 checksum */
    if (ChkSum0 == ChkSum1)
        return 0;       /* Verify CRC16 Pass */
    else
        return ChkSum1; /* Verify CRC16 Fail */
}

int32_t GenCmdSessionKey(uint32_t key[])
{
    volatile int32_t    i32RetCode = 0;
    uint32_t            u32EccX1Addr, *pu32EccX1Addr;
    /* Public key from OTA Server APP */
    ECC_PUBKEY_T        PubKey =
    {
        { 0x29A11F4C, 0x58BAB922, 0xF6E95AE4, 0x6EE18D0C, 0x77945DE9, 0xE95BE628, 0xE3ACDECF, 0x952C4C42 },
        { 0x0EC490E3, 0x6DC89EE1, 0xF6063521, 0x8E17B4D0, 0xEB940D5F, 0xEAF7E7EC, 0xB7D294C4, 0x2671A4D3 }
    };
    uint32_t            AESkey[8];
    volatile uint32_t    i;

    NUBL_MSG("[%s] enter\n", __func__);
    memset(&AESkey, 0x0, sizeof(AESkey));

    /* Step 2. Calculate (NuBL2 * Host) ECDH key */
    /* Init ECC */
    if (NuBL_ECCInitCurve() != 0)
    {
        i32RetCode = -2001;
        goto _exit_GenCmdSessionKey;
    }

    /* Select NuBL2_priv * Host_pub */
    SetECCRegisters(0x00020000, NULL, (uint32_t *)&PubKey);

    /* Calculate ECDH shared key */
    if (NuBL_ECCGenerateECDHKey() != 0)
    {
        i32RetCode = -2002;
        goto _exit_GenCmdSessionKey;
    }

    /* Get ECDH shared key for follows AES encrypt/decrypt */
    u32EccX1Addr = (uint32_t)&CRYPTO->ECC_X1[0];
    pu32EccX1Addr = (uint32_t *)u32EccX1Addr;
    memcpy(key, pu32EccX1Addr, sizeof(AESkey));
    //memcpy(key, (void *)&CRPT->ECC_X1[0], sizeof(AESkey));
#if (DEBUG_CMD == 1)

    for (i = 0; i < 8; i++)
        NUBL_MSG("\tSharedKey: 0x%08x. (NuBL2_priv * Host_pub)\n", key[i]);

#endif

    i32RetCode = 0;

_exit_GenCmdSessionKey:
    SYS_ResetModule(SYS_CRYPTO0RST);

    memset(&AESkey, 0x0, sizeof(AESkey));
    memset(&PubKey, 0x0, sizeof(ECC_PUBKEY_T));

    return i32RetCode;
}

/*
    Identify public keys mached or mismatch
        mode:
            0: p32Buf means public keys
            1: p32Buf means public key Hash
    Return:
        bit-0:
            0: No NuBL32 pubic key;
            1: NuBL32 public key match
        bit-1:
            0: No NuBL33 pubic key;
            1: NuBL33 public key match
        bit-2:
            0: No Host pubic key;
            1: Host public key match
*/
int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t i32Mode)
{
    int32_t i32RetCode = 0;

    return i32RetCode;
}

/**
  * @brief      Perform NuBL2 to verify the hash value of Host public key
  * @param[in]  u32KeyHash  key hash of HOST
  * @retval     1,2,3       Success, Bit0:NuBL32, Bit1:NuBL33, Bit1|Bit0:both NuBL32 and NuBL33
  * @retval     -1      Failed
  * @details    This function is used to perform identify, authenticate and verify F/W integrity of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Decrypt(use NuBL2 AES Key )Key Storage to get NuBL32x public key \n
  *                 2. Decrypt(use NuBL2&NuBL3x ECDH key) NuBL3x info \n
  *                 3. Identify NuBL3x public key \n
  */
int32_t VerifyNuBL3xKeyHash(uint32_t *pu32KeyHash)
{
    int32_t i32RetCode = 0;
    NUBL_MSG("\nNuBL2 verify NuBL3x.\n\n");

    return i32RetCode;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
