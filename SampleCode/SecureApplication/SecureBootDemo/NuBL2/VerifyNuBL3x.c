/**************************************************************************//**
 * @file    VerifyNuBL3x.c
 * @version V1.00
 * @brief   This source file is used to authenticate the NuBL32 and NuBL33.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "partition_M55M1.h"
#include "NuBL2.h"

#define DBG_EN      0

void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);
int32_t Cal_SHA256_Flash(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest);
int32_t Cal_SHA256_SRAM(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest);

char Qx[ECDSA_EC256_KEY_LEN * 2 + 1] = "18ab8717ffe2bb2545238176bc3bd43d49fa036759979994a4eacbfe7be1c620",
                                       Qy[ECDSA_EC256_KEY_LEN * 2 + 1] = "724cb73609d0106a325b570016310f9b10c9738b5ce489a0efac88f2062d58da";


NVT_ITCM void CRYPTO_IRQHandler(void)
{
    ECC_Complete(CRYPTO);
}

static void BytesSwap(char *buf, int32_t len)
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

int32_t Cal_SHA256_Flash(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest)
{
    int32_t  i, i32StartAddr, i32ByteCnt, data;
    uint32_t u32TimeOutCnt;

    i32StartAddr = (int32_t)u32Addr;
    i32ByteCnt   = (int32_t)u32Bytes;

    CRYPTO->HMAC_CTL    = (SHA_MODE_SHA256 << CRYPTO_HMAC_CTL_OPMODE_Pos) | CRYPTO_HMAC_CTL_INSWAP_Msk | CRYPTO_HMAC_CTL_OUTSWAP_Msk;
    CRYPTO->HMAC_DMACNT = 64;
    CRYPTO->HMAC_CTL   |= CRYPTO_HMAC_CTL_START_Msk;

    /* Start to calculate ... */
    while (i32ByteCnt > 0)
    {
        if (i32ByteCnt < 64)
            CRYPTO->HMAC_DMACNT = (uint32_t)i32ByteCnt;

        if (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_DATINREQ_Msk)
        {
            data   = (int32_t)inpw(i32StartAddr);
            i32StartAddr += 4;
            i32ByteCnt   -= 4;

            if (i32ByteCnt <= 0)
                i32ByteCnt = 0;

            /* i32ByteCnt means remain byte counts */
            if (i32ByteCnt != 0)
            {
                CRYPTO->HMAC_DATIN = (uint32_t)data;
            }
            else
            {
                /* It's last word ... *-* */
                CRYPTO->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk | CRYPTO_HMAC_CTL_DMALAST_Msk;
                CRYPTO->HMAC_DATIN = (uint32_t)data;
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

                while (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_BUSY_Msk)
                {
                    if (--u32TimeOutCnt == 0)
                        return -1;
                }

                for (i = 0; i < 8; i++)
                    pu32Digest[i] = *(uint32_t *)((uint32_t) & (CRYPTO->HMAC_DGST[0]) + ((uint32_t)i * 4));
            }
        }
    }

#if (DBG_EN == 1) // enable for debugging
    printf("\nCal_SHA256_Flash\n");
    printf("    0x%08x\n", pu32Digest[0]);
    printf("    0x%08x\n", pu32Digest[1]);
    printf("    0x%08x\n", pu32Digest[2]);
    printf("    0x%08x\n", pu32Digest[3]);
    printf("    0x%08x\n", pu32Digest[4]);
    printf("    0x%08x\n", pu32Digest[5]);
    printf("    0x%08x\n", pu32Digest[6]);
    printf("    0x%08x\n", pu32Digest[7]);
#endif

    return 0;
}

int32_t Cal_SHA256_SRAM(uint32_t u32Addr, uint32_t u32Bytes, uint32_t *pu32Digest)
{
    uint32_t u32TimeOutCnt;

    /* Reset CRYPTO module */
    SYS_ResetModule(SYS_CRYPTO0RST);

    /*---------------------------------------
     *  SHA-256
     *---------------------------------------*/
    SHA_Open(CRYPTO, SHA_MODE_SHA256, SHA_IN_OUT_SWAP, 0);

    SHA_SetDMATransfer(CRYPTO, u32Addr, u32Bytes);

    SHA_Start(CRYPTO, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (SHA_GET_INT_FLAG(CRYPTO) == 0)
    {
        if (--u32TimeOutCnt == 0)
            return -1;
    }

    SHA_CLR_INT_FLAG(CRYPTO);

    SHA_Read(CRYPTO, pu32Digest);

#if (DBG_EN == 1) // enable for debugging
    printf("\nCal_SHA256_SRAM\n");
    printf("    0x%08x\n", pu32Digest[0]);
    printf("    0x%08x\n", pu32Digest[1]);
    printf("    0x%08x\n", pu32Digest[2]);
    printf("    0x%08x\n", pu32Digest[3]);
    printf("    0x%08x\n", pu32Digest[4]);
    printf("    0x%08x\n", pu32Digest[5]);
    printf("    0x%08x\n", pu32Digest[6]);
    printf("    0x%08x\n", pu32Digest[7]);
#endif

    return 0;
}

typedef enum
{
    eBOOT_ERROCDE_SUCCESS       = 0,
    eBOOT_ERRCODE_BADIMAGE      = -1,
    eBOOT_ERRCODE_BADHASH       = -2,
    eBOOT_ERRCODE_BADTLV        = -3,
    eBOOT_ERRCODE_BADSIGN       = -4,
} E_BOOT_ERRCODE;

int32_t VerifyNuBL3x(uint32_t u32ImgBaseAddr, uint32_t u32ImgByteSize, uint32_t *pu32ImgStartAddr)
{
    struct image_header     *psImgHdr = (struct image_header *)u32ImgBaseAddr;
    struct image_tlv_info   *psTlvInfo     = NULL,
                                 *psProtTlvInfo = NULL;
    struct image_tlv        *psTlv;
    uint32_t *pu32Hash, au32ImgHash[8];
    int32_t  i;
    E_ECC_CURVE ecc_curve = CURVE_P_256;
    char R[ECDSA_SIGN_LEN * 2 + 1] = { 0 }, S[ECDSA_SIGN_LEN * 2 + 1] = { 0 }, strDigest[IMAGE_HASH_LEN * 2 + 1] = { 0 };
    uint8_t au8Buf[32];

    if (psImgHdr->ih_magic != IMAGE_MAGIC)
    {
        return eBOOT_ERRCODE_BADIMAGE;
    }

    if (psImgHdr->ih_hdr_size + psImgHdr->ih_img_size > u32ImgByteSize)
        return eBOOT_ERRCODE_BADIMAGE;

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Enable crypto interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);
    ECC_ENABLE_INT(CRYPTO);

    if (Cal_SHA256_Flash(u32ImgBaseAddr, psImgHdr->ih_hdr_size + psImgHdr->ih_img_size + psImgHdr->ih_protect_tlv_size, au32ImgHash) != 0)
        return eBOOT_ERRCODE_BADHASH;

    psTlvInfo = (struct image_tlv_info *)(u32ImgBaseAddr + BOOT_TLV_OFF(psImgHdr));

    if (psTlvInfo->it_magic == IMAGE_TLV_PROT_INFO_MAGIC)
    {
        if (psImgHdr->ih_protect_tlv_size != psTlvInfo->it_tlv_tot)
        {
            return eBOOT_ERRCODE_BADTLV;
        }

        psProtTlvInfo = psTlvInfo;
        psTlvInfo     = (struct image_tlv_info *)((uint32_t)psProtTlvInfo + psProtTlvInfo->it_tlv_tot);
    }
    else if (psImgHdr->ih_protect_tlv_size != 0)
    {
        return eBOOT_ERRCODE_BADTLV;
    }

    if (psTlvInfo->it_magic != IMAGE_TLV_INFO_MAGIC)
    {
        return eBOOT_ERRCODE_BADTLV;
    }

    /*
     * Traverse through all of the TLVs, performing any checks we know
     * and are able to do.
     */
    psTlv = (struct image_tlv *)((uint32_t)psTlvInfo + sizeof(struct image_tlv_info));

    while ((uint32_t)psTlv < ((uint32_t)psTlvInfo + psTlvInfo->it_tlv_tot))
    {
        if (psTlv->it_type == IMAGE_TLV_SHA256)
        {
            if (psTlv->it_len != sizeof(au32ImgHash))
            {
                return eBOOT_ERRCODE_BADTLV;
            }

            pu32Hash = (uint32_t *)((uint32_t)psTlv + sizeof(struct image_tlv));

            for (i = 0; i < 8; i++)
            {
                if (au32ImgHash[i] != pu32Hash[i])
                {
                    printf("au32ImgHash[%d]: 0x%08X, saved pu32Hash[%d]: 0x%08X\n", i, au32ImgHash[i], i, pu32Hash[i]);
                    return eBOOT_ERRCODE_BADTLV;
                }
            }

            BytesSwap((char *)au32ImgHash, sizeof(au32ImgHash));
            Reg2Hex(IMAGE_HASH_LEN * 2, (uint32_t *)au32ImgHash, strDigest);
        }
        else if (psTlv->it_type == IMAGE_TLV_KEYHASH)
        {
            /* IMAGE_TLV_KEYHASH is hash of public key encoded in ANS.1 DER format. */
            /* Skip checking this attribute and check signature correctness directly in this sample code. */
        }
        else if (psTlv->it_type == IMAGE_TLV_PUBKEY)
        {
            printf("IMAGE_TLV_PUBKEY\n");
        }
        else if (psTlv->it_type == IMAGE_TLV_ECDSA_SIG)
        {
            uint8_t *pu8SignHead = (uint8_t *)((uint32_t)psTlv + sizeof(struct image_tlv));

            /* https://docs.mcuboot.com/ecdsa.html */
            /* ECDSA signatures are encoded as ASN.1, notably with the signature itself encoded as follows:
               ECDSA-Sig-Value ::= SEQUENCE (0x30) {
                   r  INTEGER (0x02),
                   s  INTEGER (0x02)
               }
            */
            if (pu8SignHead[0] != (MBEDTLS_ASN1_SEQUENCE | MBEDTLS_ASN1_CONSTRUCTED))
                return eBOOT_ERRCODE_BADSIGN;

            if (pu8SignHead[2] != MBEDTLS_ASN1_INTEGER)
                return eBOOT_ERRCODE_BADSIGN;

            /* Both r and s are 256-bit numbers.
               Because these are unsigned numbers that are being encoded in ASN.1 as signed values,
               if the high bit of the number is set, the DER-encoded representation will require 33 bytes instead of 32.
            */
            if (pu8SignHead[3] == (ECDSA_SIGN_LEN + 1))
                pu8SignHead += 5;
            else if (pu8SignHead[3] == ECDSA_SIGN_LEN)
                pu8SignHead += 4;
            else
                return eBOOT_ERRCODE_BADSIGN;

            for (i = 0; i < ECDSA_SIGN_LEN; i++)
                au8Buf[ECDSA_SIGN_LEN - i - 1] = pu8SignHead[i];

            Reg2Hex(ECDSA_SIGN_LEN * 2, (uint32_t *)au8Buf, R);

            pu8SignHead += ECDSA_SIGN_LEN;

            if (pu8SignHead[0] != MBEDTLS_ASN1_INTEGER)
                return eBOOT_ERRCODE_BADSIGN;

            if (pu8SignHead[1] == (ECDSA_SIGN_LEN + 1))
                pu8SignHead += 3;
            else if (pu8SignHead[1] == ECDSA_SIGN_LEN)
                pu8SignHead += 2;
            else
                return eBOOT_ERRCODE_BADSIGN;

            for (i = 0; i < ECDSA_SIGN_LEN; i++)
                au8Buf[ECDSA_SIGN_LEN - i - 1] = pu8SignHead[i];

            Reg2Hex(ECDSA_SIGN_LEN * 2, (uint32_t *)au8Buf, S);

            //printf("R: %s\n", R);
            //printf("S: %s\n", S);

            /* Verify the signature */
            if (ECC_VerifySignature(CRYPTO, ecc_curve, strDigest, Qx, Qy, R, S) < 0)
            {
                printf("ECC signature verification failed !\n");
                return eBOOT_ERRCODE_BADSIGN;
            }
        }

        psTlv = (struct image_tlv *)((uint32_t)psTlv + sizeof(struct image_tlv) + psTlv->it_len);
    }

    if (pu32ImgStartAddr != NULL)
        *pu32ImgStartAddr = u32ImgBaseAddr + psImgHdr->ih_hdr_size;

    return eBOOT_ERROCDE_SUCCESS;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
