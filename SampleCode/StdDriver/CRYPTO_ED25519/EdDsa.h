/**************************************************************************//**
 * @file     EdDSA.h
 * @version  V1.00
 * @brief    Cryptographic Accelerator EdDSA header file
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __EDDSA_H__
#define __EDDSA_H__

enum ed_type
{
    EDDSA_ED25519,
    EDDSA_ED25519PH,
    EDDSA_ED25519CTX,
};

//From mmio.h
#define ptr_to_u32(x)   ((uint32_t)((uint32_t*)(x)))
#define u32_to_ptr32(x)   ((uint32_t *)((uint32_t)(x)))
#define u32_to_ptr8(x)   ((uint8_t *)((uint32_t)(x)))
#define readb   read8


int32_t  ECC_ED25519_SigGen(enum ed_type eddsa_type, uint8_t *priv_key,
                            uint8_t *message, int msg_len,
                            uint8_t *ctx, int ctx_len,
                            uint32_t *R, uint32_t *S);

int32_t  ECC_ED25519_Verify(enum ed_type eddsa_type,
                            uint8_t *message, int msg_len,
                            uint8_t *ctx, int ctx_len,
                            uint8_t *pub_key,
                            uint32_t *R, uint32_t *S);

void  dump_buff_hex(uint8_t *pucBuff, int nBytes);
#endif  /* __EDDSA_H__ */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

