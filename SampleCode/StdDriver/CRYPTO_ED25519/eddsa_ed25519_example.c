/**************************************************************************//**
 * @file    eddsa_ed25519_example.c
 * @version V1.00
 * @brief   ED25519 example code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "EdDsa.h"

#define MAX_KEY_LEN     512

struct test_vec
{
    enum ed_type  type;
    char    ed_name[16];
    int keylen;
    int msglen;
    char    message[8192];
    char    d[MAX_KEY_LEN];   /* secret key */
    char    p[MAX_KEY_LEN];   /* public key */
    char    sig[MAX_KEY_LEN * 2];   /* signature */
    char    context[8192];
};

uint8_t _msg[8192];
uint8_t _ctx[8192];

extern volatile uint32_t g_ECC_done, g_ECCERR_done;
/*
* More patterns can be found:
 * https://datatracker.ietf.org/doc/html/rfc8032
 */
struct test_vec ed25519_test_vector __attribute__((aligned(16))) =
{
    //DO_ED25519_TEST

    EDDSA_ED25519, "Ed25519", 256, 0,
    "",
    "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60",
    "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a",
    "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b",
    ""

};

#define assert(a) if( !( a ) )              \
    {                           \
        printf("Assertion Failed at	%s:%d -	%s\n",      \
               __FILE__, __LINE__, #a );  \
        while (1);                      \
    }

static inline uint8_t read8(const void *addr)
{
    return *(volatile uint8_t *)addr;
}

static volatile int g_HMAC_error;
static volatile int g_HMAC_done;

static volatile uint64_t  _start_time = 0;


void CRYPTO_IRQHandler()
{
    if (CRYPTO->INTSTS & CRYPTO_INTSTS_HMACEIF_Msk)
    {
        printf("SHAERRIF is set!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        g_HMAC_error = 1;
        CRYPTO->INTSTS = CRYPTO_INTSTS_HMACEIF_Msk;
    }

    if (CRYPTO->INTSTS & CRYPTO_INTSTS_HMACIF_Msk)
    {
        g_HMAC_done = 1;
        CRYPTO->INTSTS = CRYPTO_INTSTS_HMACIF_Msk;
    }

    ECC_Complete(CRYPTO);
}

void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    uint32_t  addr, end_addr;
    int   i;

    addr = ptr_to_u32(pucBuff);
    end_addr = addr + nBytes - 1;

    if ((addr % 16) != 0)
    {
        printf("0x%04x_%04x  ", (addr >> 16) & 0xffff, addr & 0xffff);

        for (i = 0; i < addr % 16; i++)
            printf(".. ");

        for (; (addr % 16) != 0; addr++)
            printf("%02x ", readb(u32_to_ptr8(addr)) & 0xff);

        printf("\n");
    }

    for (;  addr <= end_addr;)
    {
        printf("0x%04x_%04x  ", (addr >> 16) & 0xffff, addr & 0xffff);

        for (i = 0; i < 16; i++, addr++)
        {
            if (addr > end_addr)
                break;

            printf("%02x ", readb(u32_to_ptr8(addr)) & 0xff);
        }

        printf("\n");
    }

    printf("\n");
}


int unhexify(unsigned char *obuf, const char *ibuf)
{
    unsigned char c, c2;
    int len = strlen(ibuf) / 2;

    assert(strlen(ibuf) % 2 == 0);       /* must be even number of bytes */

    while (*ibuf !=  0)
    {
        c = *ibuf++;

        if (c >= '0' &&  c <= '9')
            c -= '0';
        else if (c >= 'a' && c <= 'f')
            c -= 'a' - 10;
        else if (c >= 'A' && c <= 'F')
            c -= 'A' - 10;
        else
            assert(0);

        c2 = *ibuf++;

        if (c2 >= '0' && c2 <= '9')
            c2 -= '0';
        else if (c2 >= 'a' && c2 <= 'f')
            c2 -= 'a' - 10;
        else if (c2 >= 'A' && c2 <= 'F')
            c2 -= 'A' - 10;
        else
            assert(0);

        *obuf++ = (c << 4) | c2;
    }

    return len;
}

void hexify(char *obuf, const unsigned char *ibuf, int len)
{
    unsigned char l, h;

    while (len != 0)
    {
        h = *ibuf / 16;
        l = *ibuf % 16;

        if (h <  10)
            *obuf++ = '0' + h;
        else
            *obuf++ = 'a' + h - 10;

        if (l <  10)
            *obuf++ = '0' + l;
        else
            *obuf++ = 'a' + l - 10;

        ++ibuf;
        len--;
    }
}


int ed25519_test()
{

    struct test_vec *sg_tv;
    uint8_t priv_key[128];
    uint8_t pub_key[128];
    uint8_t u8tmp[128];
    uint32_t R[8], S[8];
    int msg_len, ctx_len;

    printf("+-----------------------------------------------+\n");
    printf("|   Crypto ED25519 Signature Generation Test      |\n");
    printf("+-----------------------------------------------+\n");


    sg_tv = &ed25519_test_vector;
    printf("Run %s test, msglen = %d.\n", sg_tv->ed_name, sg_tv->msglen);
    printf("Message = %s\n", sg_tv->message);

    unhexify(priv_key, sg_tv->d);
    unhexify(pub_key, sg_tv->p);
    msg_len = unhexify(_msg, sg_tv->message);
    ctx_len = unhexify(_ctx, sg_tv->context);

    if (ECC_ED25519_SigGen(sg_tv->type, priv_key, _msg, msg_len, _ctx, ctx_len, R, S) != 0)
    {
        printf("ECC signature generation failed!!\n");

        while (1);
    }

    unhexify(u8tmp, sg_tv->sig);

    if (memcmp(u8tmp, R, 32) != 0)
    {
        printf("!!! Ed25519 sig_gen type %d test failed. R ==>\n", sg_tv->type);
        dump_buff_hex((uint8_t *)R, 32);
        printf("Expect =>\n");
        dump_buff_hex(u8tmp, 32);

        while (1);
    }

    printf("!!! Ed25519 sig_gen type %d test pass R !!\n", sg_tv->type);

    if (memcmp(&u8tmp[32], S, 32) != 0)
    {
        printf("!!! Ed25519 sig_gen type %d test failed. S ==>\n", sg_tv->type);
        dump_buff_hex((uint8_t *)S, 32);
        printf("Expect =>\n");
        dump_buff_hex(&u8tmp[32], 32);

        while (1);
    }

    printf("[ECC_ED25519_SigGen PASS]\n");

    printf("Ed25519 signature generation test passed.\n");
    printf("\n");

    if (ECC_ED25519_Verify(sg_tv->type, _msg, msg_len, _ctx, ctx_len, pub_key, R, S) != 0)
    {
        printf("Ed25519 sig_verify test failed!\n");

        while (1);
    }

    printf("[ECC_ED25519_Verify PASS]\n");
    printf("\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
