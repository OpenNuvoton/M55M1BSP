/**************************************************************************//**
 * @file    NuBL2.h
 * @version V3.00
 * @brief   NuBL2 header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUBL2_H__
#define __NUBL2_H__

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "CommandHandler.h"
#include "../image.h"

#if defined (__DISABLE_MSG__)
    #define NUBL_MSG(...)
#else
    #define NUBL_MSG       printf  /* enable debug message */
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* FLASH memory layout definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define NUBL2_FW_BASE           (FMC_APROM_BASE + 0x00000000ul)     /* Secure code */
#define NUBL32_FW_BASE          (FMC_APROM_BASE + 0x00040000ul)     /* Secure code */
#define NUBL32_FW_SIZE          (0x20000ul)
#define NUBL33_FW_BASE          (FMC_NON_SECURE_BASE)               /* Non-secure code start address */
#define NUBL33_FW_SIZE          (0x20000ul)

#define NUBL_SIZE_ALIGN         (16)
#define NUBL_INVALID_ADDR       (uint32_t)(-1)
/*---------------------------------------------------------------------------------------------------------*/
/* Status and error code definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define NUBL_STS_OK             (0)
//#define ERR_XOM_NOT_ACTIVE      (-1)
#define ERR_XOM_SIZE            (-2)
#define ERR_XOM_ADDRESS         (-3)
#define ERR_INIT_ECC            (-1001)
#define ERR_GEN_ECDH            (-1002)
#define ERR_AES_DECRYPT         (-1003)
#define ERR_FILE_NOT_FOUND      (-1004)
#define ERR_INVALID_FW          (-1005)

typedef enum
{
    eIMG_IDX_NUBL32 = 0,
    eIMG_IDX_NUBL33 = 1,
    eIMG_IDX_CNT,
} E_IMG_IDX;

typedef struct
{
    uint32_t u32BaseAddr;
    uint32_t u32ByteSize;
    char     strImgName[32];
} S_NUBL3x_INFO;

#ifdef __cplusplus
extern "C"
{
#endif

extern const S_NUBL3x_INFO g_asImgInfo[eIMG_IDX_CNT];

int32_t NuBL_VerifySignature(E_ECC_CURVE ecc_curve, char *strDigest, char *R, char *S);
int32_t NuBL_VerifyNuBL3x(uint32_t u32ImgBaseAddr, uint32_t u32ImgByteSize, uint32_t *pu32ImgStartAddr);

#ifdef __cplusplus
}
#endif

#endif /* __NUBL2_H__ */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
