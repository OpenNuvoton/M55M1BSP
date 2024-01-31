/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_AES_GCM code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define GCM_MODE    (AES_MODE_GCM << CRYPTO_AES_CTL_OPMODE_Pos)
#define GHASH_MODE  (AES_MODE_GHASH << CRYPTO_AES_CTL_OPMODE_Pos)
#define CTR_MODE    (AES_MODE_CTR << CRYPTO_AES_CTL_OPMODE_Pos)

#define DMAEN       CRYPTO_AES_CTL_DMAEN_Msk
#define DMALAST     CRYPTO_AES_CTL_DMALAST_Msk
#define DMACC       CRYPTO_AES_CTL_DMACSCAD_Msk
#define START       CRYPTO_AES_CTL_START_Msk
#define FBIN        CRYPTO_AES_CTL_FBIN_Msk
#define FBOUT       CRYPTO_AES_CTL_FBOUT_Msk


#define GCM_PBLOCK_SIZE  128     /* NOTE: This value must be 16 bytes alignment. This value must > size of A */

#define MAX_GCM_BUF     4096
__ALIGNED(4) uint8_t g_au8Buf[MAX_GCM_BUF];
__ALIGNED(4) uint8_t g_au8Out[MAX_GCM_BUF];
__ALIGNED(4) uint8_t g_au8Out2[MAX_GCM_BUF];
__ALIGNED(4) uint8_t g_au8FeedBackBuf[72] = {0};

/* for the key and data in binary format */
__ALIGNED(4) uint8_t g_key[32] = { 0 };
__ALIGNED(4) uint8_t g_iv[32] = { 0 };
__ALIGNED(4) uint8_t g_A[265] = { 0 };
__ALIGNED(4) uint8_t g_P[256] = { 0 };
__ALIGNED(4) uint8_t g_C[256] = { 0 };
__ALIGNED(4) uint8_t g_T[256] = { 0 };


typedef struct
{
    char *pchKey;   /* The block cipher key */
    char *pchIV;    /* The initialization vector */
    char *pchA;     /* The additional authenticated data */
    char *pchP;     /* The plaintext */
    char *pchC;     /* The ciphertext */
    char *pchTag;   /* The authentication tag */
} GCM_TEST_T;

/* Test items */
const GCM_TEST_T sElements[] =
{
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "28bbbc081544db64c6a462ebfcc71a98"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e0123456789",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b01122",
        "cbc2a71a9a0eddd39ac1a4d430b48ab4e4689869794cb48a9743957740661f963c",
        "c623ffe47619a24c3120d2c8fa7c1a1e"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "9a4562c3b90f65e0f1dc715b58c2faf4"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "28bbbc081544db64c6a462ebfcc71a98"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "51344aee87ddbcb21743e7aafefca60a"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "8ad1b737122d19deb791b1177adf138f"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "fc7ad781e089086b2c716c7909f91bb0"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "a406086eb74c238de45f11e7ca798367"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "c20d88d8cd64af02d89e8a77cdb04d2f"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "87590156b26185b25b9a08a00c6a528a"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "803c9ceaf8e6548f20762763575aa6ee"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "7ae5f9e22822f20c3961bca075991cfc"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "ac8a28ba37fd1932af26bf62c80f7e04"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "a700e05709d41111beccf49c83f53326"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "370f43d44b02a070b97b0091cd60c369"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9da",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "c80bb70c28fa92bd2fbd0015ef5f6c24"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadb",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "677e65ebc9e6a7f2415ba01ee3096327"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdc",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "0d7391e0823cba650c6368c90bae9838"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdd",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "9cec944c67f5186fd4980239835100c6"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcddde",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "0f4fe0030df3949f5a9ad2b3976edde4"
    },


    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf30",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "4c128777842f6e78467350e5e8a6fc9b"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf3031",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "9ee055d91b23af632984f0fc98b4b317"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf303132",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "011ed384548b2c554f5e9a1f2c371f87"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf30313233",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "121cd16b9cbbcd72a331f5985689d2fb"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf3031323334",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "74555167ba0cffaf88e5d9bb802eba5d"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf303132333435",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "7155d1d272d29c8a9ea126d62f09334e"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf30313233343536",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "da20d1c7ab9463bb9fdb15c5a33e9e21"
    },


    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68df",
        "9a4562c3b90f65e0f1dc715b58c2faf4"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b01122",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe8",
        "bdcb22e0cfe0c956d146d64195f60e9a"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835",
        "0e4b009b6bf17a85f369c11a664c4762"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011223344",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe83505",
        "48168c9baa5f064ad38962841acf31e9"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b01122334455",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe8350536",
        "ea52a16817b3a10eb900c8ab739df724"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625",
        "56a3e422b3a5328ad532109ce685038b"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b011223344556677",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2",
        "b121a4c4cd2841158a796b4305e8fea5"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b01122334455667788",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f0",
        "93d5f8e738fcd8123740eb84f9641096"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c",
        "1b720c07b326322fd7bf808d83e72c04"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aa",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c23",
        "5a2c9e159443aacf91eea86ab9554937"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabb",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315",
        "101f5cb551009e13f4a67927847c789f"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbcc",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e4",
        "182661cfb3e8e8cd983e5c0429715873"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccdd",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e425",
        "e035b40fd2ed46c91861f1c73934295d"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddee",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557",
        "89bb7a59019b4dd84c00f40d106d5344"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f7",
        "149f39acbe7c2b9c38889520e45915bc"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff00",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f748",
        "3a7e4105193c2a638e1765fa6692259b"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff0011",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f748bb",
        "84064133d32b19fca8fcb2b642edc2c4"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff001122",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f748bbd8",
        "24e62f08f08ab7b236fbd810424f57a9"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff00112233",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f748bbd817",
        "665716fbe34c1829e81afddb8724d702"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0112233445566778899aabbccddeeff0011223344",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68dfe835053625d2f05c2315e42557f748bbd81748",
        "2247b91f3d0d8559d1f46d2d35a0aa60"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04b0",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be68",
        "41db204d39ee6fdb8e356855f6558503"
    },
    {
        "000102030405060708090A0B0C0D0E0F",
        "4d4d4d0000bc614e01234567",
        "30d0d1d2d3d4d5d6d7d8d9dadbdcdddedf",
        "01011000112233445566778899aabbccddeeff0000065f1f0400007e1f04",
        "801302ff8a7874133d414ced25b42534d28db0047720606b175bd52211be",
        "7cf6381098e683c1757723c7c3ff960d"
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "5F91D77123EF5EB9997913849B8DC1E9"
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "3247184B3C4F69A44DBCD22887BBB418",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091473F5985",
        "4D5C2AF327CD64A62CF35ABD2BA6FAB4",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "5F91D77123EF5EB9997913849B8DC1E9",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091473F5985",
        "64C0232904AF398A5B67C10B53A5024D",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091",
        "F07C2528EEA2FCA1211F905E1B6A881B",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "42831EC2217774244B7221B784D0D49CE3AA212F2C02A4E035C17E2329ACA12E21D514B25466931C7D8F6A5AAC84AA051BA30B396A0AAC973D58E091",
        "F07C2528EEA2FCA1211F905E",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "C835AA88AEBBC94F5A02E179FDCFC3E4",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710ACADE256",
        "9924A7C8587336BFB118024DB8674A14",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710ACADE256",
        "3B9153B4E7318A5F3BBEAC108F8A8EDB",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710",
        "93EA28C659E269902A80ACD208E7FC80",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "3980CA0B3C00E841EB06FAC4872A2757859E1CEAA6EFD984628593B40CA1E19C7D773D00C144C525AC619D18C84A3F4718E2448B2FE324D9CCDA2710",
        "93EA28C659E269902A80ACD2",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "",
        "",
        "FD2CAA16A5832E76AA132C1453EEDA7E",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B391AAFD255",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662898015AD",
        "B094DAC5D93471BDEC1A502270E3CC6C",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D58503B9699DE785895A96FDBAAF43B1CD7F598ECE23881B00E3ED0306887B0C785E27E8AD3F8223207104725DD4",
        "",
        "",
        "DE34B6DCD4CEE2FDBEC3CEA01AF1EE44",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662",
        "E097195F4532DA895FB917A5A55C6AA0",
    },
    {
        "FEFFE9928665731C6D6A8F9467308308FEFFE9928665731C6D6A8F9467308308",
        "CAFEBABEFACEDBADDECAF888",
        "3AD77BB40D7A3660A89ECAF32466EF97F5D3D585",
        "D9313225F88406E5A55909C5AFF5269A86A7A9531534F7DA2E4C303D8A318A721C3C0C95956809532FCF0E2449A6B525B16AEDF5AA0DE657BA637B39",
        "522DC1F099567D07F47F37A32A84427D643A8CDCBFE5C0C97598A2BD2555D1AA8CB08E48590DBB3DA7B08B1056828838C5F61E6393BA7A0ABCC9F662",
        "E097195F4532DA895FB917A5",
    },

    {
        "f8d6868a7250f76e85de2e9f813edfc2",
        "0a",
        "30307aef4c3b7fa25ac7b181999851717f703a481bf59b16546bf2df7fc7d81677de6989cc64140470ab8b86a42ae498",
        "81e562083769c8ae8dfda00f192396a504b70dcea2c25ed0b89012ab9ebffbdad8f227d98951e75685b16bac064ceebd6b1840",
        "d1c2fa6ba5b29cb95f7819b2e6f2a7dbc0d8a58828f7e8528451633385afe0730921d08b50b7e0fa3be469cc72ff0e3226fb54",
        "b96cb72d696ad2325c36a55634a21d0f"
    },
};

void DumpBuffHex(uint8_t *pucBuff, int nBytes)
{
    int32_t i32Idx, i, len;


    i32Idx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", i32Idx);

        len = (nBytes < 16) ? nBytes : 16;

        for (i = 0; i < len; i++)
            printf("%02x ", pucBuff[i32Idx + i]);

        for (; i < 16; i++)
        {
            printf("   ");
        }

        printf("  ");

        for (i = 0; i < len; i++)
        {
            if ((pucBuff[i32Idx + i] >= 0x20) && (pucBuff[i32Idx + i] < 127))
                printf("%c", pucBuff[i32Idx + i]);
            else
                printf(".");

            nBytes--;
        }

        i32Idx += len;
        printf("\n");
    }

    printf("\n");
}


volatile int  g_Crypto_Int_done = 0;

void CRYPTO_IRQHandler()
{
    if (AES_GET_INT_FLAG(CRYPTO))
    {
        g_Crypto_Int_done = 1;
        AES_CLR_INT_FLAG(CRYPTO);
    }
}



void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);


    /* Enable PLL0 200MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}


void str2bin(const char *pstr, uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t u8Ch;
    char c;

    for (i = 0; i < size; i++)
    {
        c = *pstr++;

        if (c == '\0')
            break;

        if ((c >= 'a') && (c <= 'f'))
            c -= ('a' - 10);
        else if ((c >= 'A') && (c <= 'F'))
            c -= ('A' - 10);
        else if ((c >= '0') && (c <= '9'))
            c -= '0';

        u8Ch = (uint8_t)c << 4;

        c = *pstr++;

        if (c == '\0')
        {
            buf[i] = u8Ch;
            break;
        }

        if ((c >= 'a') && (c <= 'f'))
            c -= ('a' - 10);
        else if ((c >= 'A') && (c <= 'F'))
            c -= ('A' - 10);
        else if ((c >= '0') && (c <= '9'))
            c -= '0';

        u8Ch += (uint8_t)c;

        buf[i] = u8Ch;
    }

}


void bin2str(uint8_t *buf, uint32_t size, char *pstr)
{
    int32_t i;
    uint8_t c;

    for (i = size - 1; i >= 0; i--)
    {
        c = buf[i] >> 4;
        *pstr++ = (c >= 10) ? c - 10 + 'a' : c + '0';
        c = buf[i] & 0xf;
        *pstr++ = (c >= 10) ? c - 10 + 'a' : c + '0';
    }

    *pstr = '\0';
}



int32_t ToBigEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if ((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while (u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if (u32Size > 0)
    {
        u32Tmp = 0;

        for (i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }

    return 0;
}



int32_t ToLittleEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if ((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while (u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if (u32Size > 0)
    {
        u32Tmp = 0;

        for (i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }

    return 0;
}

#define swap32(x) (((x) & 0xff) << 24 | ((x) & 0xff00) << 8 | ((x) & 0xff0000) >> 8 | ((x) >> 24) & 0xff)
static void swap64(uint8_t *p)
{
    uint8_t tmp;
    int32_t i;

    for (i = 0; i < 4; i++)
    {
        tmp = p[i];
        p[i] = p[7 - i];
        p[7 - i] = tmp;
    }
}


/*
NOTE: pbuf must be word alignment

    GCM input format must be block alignment. The block size is 16 bytes.
    {IV}{IV nbits}{A}{P/C}


*/

int32_t AES_GCMPacker(uint8_t *iv, uint32_t iv_len, uint8_t *A, uint32_t A_len, uint8_t *P, uint32_t P_len, uint8_t *pbuf, uint32_t *psize)
{
    uint32_t i;
    uint32_t iv_len_aligned, A_len_aligned, P_len_aligned;
    uint32_t u32Offset = 0;
    uint8_t *pu8;

    /* IV Section:

       if bitlen(IV) == 96
         IV section = IV || 31'bit 0 || 1

       if bitlen(IV) != 96
         IV section = 128'align(IV) || 64'bit 0 || 64'bitlen(IV)
    */
    if (iv_len > 0)
    {
        iv_len_aligned = iv_len;

        if (iv_len & 0xful)
            iv_len_aligned = ((iv_len + 16) >> 4) << 4;

        /* fill iv to output */
        for (i = 0; i < iv_len_aligned; i++)
        {
            if (i < iv_len)
                pbuf[i] = iv[i];
            else
                pbuf[i] = 0; // padding zero
        }

        /* fill iv len to putput */
        if (iv_len == 12)
        {
            pbuf[15] = 1;
            u32Offset += iv_len_aligned;
        }
        else
        {
            /* Padding zero. 64'bit 0 */
            memset(&pbuf[iv_len_aligned], 0, 8);

            /* 64'bitlen(IV) */
            pu8 = &pbuf[iv_len_aligned + 8];
            *((uint64_t *)pu8) = iv_len * 8;
            swap64(pu8);
            u32Offset += iv_len_aligned + 16;
        }
    }


    /* A Section = 128'align(A) */
    if (A_len > 0)
    {
        A_len_aligned = A_len;

        if (A_len & 0xful)
            A_len_aligned = ((A_len + 16) >> 4) << 4;

        for (i = 0; i < A_len_aligned; i++)
        {
            if (i < A_len)
                pbuf[u32Offset + i] = A[i];
            else
                pbuf[u32Offset + i] = 0; // padding zero
        }

        u32Offset += A_len_aligned;
    }

    /* P/C Section = 128'align(P/C) */
    if (P_len > 0)
    {
        P_len_aligned = P_len;

        if (P_len & 0xful)
            P_len_aligned = ((P_len + 16) >> 4) << 4;

        for (i = 0; i < P_len_aligned; i++)
        {
            if (i < P_len)
                pbuf[u32Offset + i] = P[i];
            else
                pbuf[u32Offset + i] = 0; // padding zero
        }

        u32Offset += P_len_aligned;
    }

    *psize = u32Offset;

    return 0;
}

void AES_Run(uint32_t u32Option)
{
    uint32_t u32TimeOutCnt;

    g_Crypto_Int_done = 0;
    CRYPTO->AES_CTL = u32Option | START;
    /* Waiting for AES calculation */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_Crypto_Int_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES calculation done time-out!\n");
            break;
        }
    }
}


int32_t AES_GCMEnc(uint8_t *key, uint32_t klen, uint8_t *iv, uint32_t ivlen, uint8_t *A, uint32_t alen, uint8_t *P, uint32_t plen, uint8_t *buf, uint32_t *size, uint32_t *plen_aligned)
{
    int32_t plen_cur;
    int32_t len;
    uint8_t *pin, *pout;
    int32_t i;
    uint32_t u32OptKeySize;
    uint32_t u32OptBasic;

    printf("\n");

    printf("key (%d):\n", klen);
    DumpBuffHex(key, klen);

    printf("IV (%d):\n", ivlen);
    DumpBuffHex(iv, ivlen);

    printf("A (%d):\n", alen);
    DumpBuffHex(A, alen);

    printf("P (%d):\n", plen);
    DumpBuffHex(P, plen);

    /* Prepare the key */
    memcpy(g_au8Buf, key, klen);
    ToBigEndian(g_au8Buf, klen);

    for (i = 0; i < klen / 4; i++)
    {
        CRYPTO->AES_KEY[i] = *((uint32_t *)&g_au8Buf[i * 4]);
    }

    /* Prepare key size option */
    i = klen >> 3;
    u32OptKeySize = (((i >> 2) << 1) | (i & 1)) << CRYPTO_AES_CTL_KEYSZ_Pos;

    /* Basic options for AES */
    u32OptBasic = CRYPTO_AES_CTL_ENCRYPTO_Msk | CRYPTO_AES_CTL_INSWAP_Msk | CRYPTO_AES_CTL_OUTSWAP_Msk | u32OptKeySize;


    /* Set byte count of IV */
    CRYPTO->AES_GCM_IVCNT[0] = ivlen;
    CRYPTO->AES_GCM_IVCNT[1] = 0;

    /* Set bytes count of A */
    CRYPTO->AES_GCM_ACNT[0] = alen;
    CRYPTO->AES_GCM_ACNT[1] = 0;
    /* Set bytes count of P */
    CRYPTO->AES_GCM_PCNT[0] = plen;
    CRYPTO->AES_GCM_PCNT[1] = 0;

    *plen_aligned = (plen & 0xful) ? ((plen + 16) / 16) * 16 : plen;

    if (plen <= GCM_PBLOCK_SIZE)
    {
        /* Just one shot */

        /* Prepare the blocked buffer for GCM */
        AES_GCMPacker(iv, ivlen, A, alen, P, plen, g_au8Buf, size);


        printf("input blocks (%d):\n", *size);
        DumpBuffHex(g_au8Buf, *size);

        AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);

        AES_Run(u32OptBasic | GCM_MODE | DMAEN);

        printf("output blocks (%d):\n", *size);
        DumpBuffHex(buf, *size);
    }
    else
    {

        /* Process P block by block, DMA casecade mode */

        /* inital DMA for AES-GCM casecade */

        /* Prepare the blocked buffer for GCM */
        AES_GCMPacker(iv, ivlen, A, alen, 0, 0, g_au8Buf, size);

        printf("input blocks for casecade 0 (%d):\n", *size);
        DumpBuffHex(g_au8Buf, *size);

        AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);
        /* feedback buffer is necessary for casecade mode */
        CRYPTO->AES_FBADDR = (uint32_t)&g_au8FeedBackBuf[0];

        AES_Run(u32OptBasic | GCM_MODE | FBOUT | DMAEN);

        /* Start to encrypt P data */
        printf("P Block size for casecade mode %d\n", GCM_PBLOCK_SIZE);
        plen_cur = plen;
        pin = P;
        pout = buf;

        while (plen_cur)
        {
            len = plen_cur;

            if (len > GCM_PBLOCK_SIZE)
            {
                len = GCM_PBLOCK_SIZE;
            }

            plen_cur -= len;

            /* Prepare the blocked buffer for GCM */
            memcpy(g_au8Buf, pin, len);

            /* padding 0 if necessary */
            if (len & 0xf)
            {
                memset(&g_au8Buf[len], 0, 16 - (len & 0xf));
                len = ((len + 16) >> 4) << 4;
            }

            printf("input blocks for casecade (%d):\n", len);
            DumpBuffHex(g_au8Buf, len);

            AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)pout, len);



            if (plen_cur)
            {
                /* casecade n */
                AES_Run(u32OptBasic | GCM_MODE | FBIN | FBOUT | DMAEN | DMACC);
            }
            else
            {
                /* last casecade */
                AES_Run(u32OptBasic | GCM_MODE | FBIN | FBOUT | DMAEN | DMACC | DMALAST);
            }

            printf("output blocks (%d):\n", len);
            DumpBuffHex(pout, len);

            pin += len;
            pout += len;
        }

        /* Total size is plen aligment size + tag size */
        *size = *plen_aligned + 16;

    }

    return 0;
}

int32_t AES_GCMDec(uint8_t *key, uint32_t klen, uint8_t *iv, uint32_t ivlen, uint8_t *A, uint32_t alen, uint8_t *P, uint32_t plen, uint8_t *buf, uint32_t *size, uint32_t *plen_aligned)
{
    int32_t i, len, plen_cur;
    uint8_t *pin, *pout;
    uint32_t u32OptBasic;
    uint32_t u32OptKeySize;

    /* Prepare key size option */
    i = klen >> 3;
    u32OptKeySize = (((i >> 2) << 1) | (i & 1)) << CRYPTO_AES_CTL_KEYSZ_Pos;

    /* Basic options for AES */
    u32OptBasic = CRYPTO_AES_CTL_INSWAP_Msk | CRYPTO_AES_CTL_OUTSWAP_Msk | u32OptKeySize;

    printf("\n");
    printf("key (%d):\n", klen);
    DumpBuffHex(key, klen);

    printf("IV (%d):\n", ivlen);
    DumpBuffHex(iv, ivlen);

    printf("A (%d):\n", alen);
    DumpBuffHex(A, alen);

    /* Set AES Key */
    memcpy(g_au8Buf, key, klen);
    ToBigEndian(g_au8Buf, klen);

    for (i = 0; i < klen / 4; i++)
    {
        CRYPTO->AES_KEY[i] = *((uint32_t *)&g_au8Buf[i * 4]);
    }

    /* Set byte count of IV */
    CRYPTO->AES_GCM_IVCNT[0] = ivlen;
    CRYPTO->AES_GCM_IVCNT[1] = 0;

    /* Set bytes count of A */
    CRYPTO->AES_GCM_ACNT[0] = alen;
    CRYPTO->AES_GCM_ACNT[1] = 0;

    /* Set bytes count of P/C */
    CRYPTO->AES_GCM_PCNT[0] = plen;
    CRYPTO->AES_GCM_PCNT[1] = 0;


    *plen_aligned = (plen & 0xful) ? ((plen + 16) >> 4) << 4 : plen;

    if (plen < GCM_PBLOCK_SIZE)
    {
        /* Small P/C size, just use DMA one shot */

        /* Prepare the blocked buffer for GCM */
        AES_GCMPacker(iv, ivlen, A, alen, P, plen, g_au8Buf, size);

        printf("input blocks (%d):\n", *size);
        DumpBuffHex(g_au8Buf, *size);

        AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);
        AES_Run(u32OptBasic | GCM_MODE | DMAEN);

        printf("output blocks (%d):\n", *size);
        DumpBuffHex(buf, *size);
    }
    else
    {
        /* Decrypt C data block by block. DMA cascade mode. */
        /* Process P block by block, DMA casecade mode */

        /* inital DMA for AES-GCM casecade */

        /* Prepare the blocked buffer for GCM */
        AES_GCMPacker(iv, ivlen, A, alen, 0, 0, g_au8Buf, size);

        printf("input blocks for casecade 0 (%d):\n", *size);
        DumpBuffHex(g_au8Buf, *size);

        AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)buf, *size);
        /* feedback buffer is necessary for casecade mode */
        CRYPTO->AES_FBADDR = (uint32_t)&g_au8FeedBackBuf[0];

        AES_Run(u32OptBasic | GCM_MODE | FBOUT | DMAEN);

        /* Start to decrypt P data */
        printf("P Block size for casecade mode %d\n", GCM_PBLOCK_SIZE);
        plen_cur = plen;
        pin = P;
        pout = buf;

        while (plen_cur)
        {
            len = plen_cur;

            if (len > GCM_PBLOCK_SIZE)
            {
                len = GCM_PBLOCK_SIZE;
            }

            plen_cur -= len;

            /* Prepare the blocked buffer for GCM */
            memcpy(g_au8Buf, pin, len);

            /* padding 0 if necessary */
            if (len & 0xf)
            {
                memset(&g_au8Buf[len], 0, 16 - (len & 0xf));
                len = ((len + 16) >> 4) << 4;
            }

            printf("input blocks for casecade (%d):\n", len);
            DumpBuffHex(g_au8Buf, len);

            AES_SetDMATransfer(CRYPTO, 0, (uint32_t)g_au8Buf, (uint32_t)pout, len);

            if (plen_cur)
            {
                /* casecade n */
                AES_Run(u32OptBasic | GCM_MODE | FBIN | FBOUT | DMAEN | DMACC);
            }
            else
            {
                /* last casecade */
                AES_Run(u32OptBasic | GCM_MODE | FBIN | FBOUT | DMAEN | DMACC | DMALAST);
            }

            printf("output blocks (%d):\n", len);
            DumpBuffHex(pout, len);

            pin += len;
            pout += len;
        }
    }


    return 0;
}


/*-----------------------------------------------------------------------------*/
int main(void)
{
    int i, n;
    uint32_t size, klen, plen, tlen, plen_aligned, alen, ivlen;

    // Enable ETM
    SET_TRACE_CLK_PE12();
    SET_TRACE_DATA0_PE11();
    SET_TRACE_DATA1_PE10();
    SET_TRACE_DATA2_PE9();
    SET_TRACE_DATA3_PE8();


    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+---------------------------------------+\n");
    printf("|               AES GCM Test            |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);

    n = sizeof(sElements) / sizeof(GCM_TEST_T);

    for (i = 0; i < n; i++)
    {

        printf("\n============================================================================\n");
        printf("Test iter %d/%d\n\n", i + 1, n);

        /* Reset CRYPTO module clock */
        SYS_ResetModule(SYS_CRYPTO0RST);

        AES_ENABLE_INT(CRYPTO);

        printf("key = %s\n", sElements[i].pchKey);
        printf("iv  = %s\n", sElements[i].pchIV);
        printf("A   = %s\n", sElements[i].pchA);
        printf("P   = %s\n", sElements[i].pchP);

        klen = strlen(sElements[i].pchKey) / 2;
        ivlen = strlen(sElements[i].pchIV) / 2;
        alen = strlen(sElements[i].pchA) / 2;
        plen = strlen(sElements[i].pchP) / 2;
        tlen = strlen(sElements[i].pchTag) / 2;

        /* Rule check */
        if ((klen != 16) && (klen != 24) && (klen != 32))
        {
            printf("klen = %d\n", klen);
            printf("Key size should 128, 192 or 256 bits\n");
            return -1;
        }

        if (alen > GCM_PBLOCK_SIZE)
        {
            printf("alen = %d\n", alen);
            printf("length of A should not larger than defined block size.\n");
            return -1;
        }

        if (GCM_PBLOCK_SIZE & 0xf)
        {
            printf("block size = %d\n", GCM_PBLOCK_SIZE);
            printf("Defined block size should be 16 bytes alignment.\n");
            return -1;
        }

        if (ivlen == 0)
        {
            printf("ivlen = %d\n", ivlen);
            printf("IV length should not be 0\n");
            return -1;
        }


        str2bin(sElements[i].pchKey, g_key, klen);
        str2bin(sElements[i].pchIV, g_iv, ivlen);
        str2bin(sElements[i].pchA, g_A, alen);
        str2bin(sElements[i].pchP, g_P, plen);

        AES_GCMEnc(g_key, klen, g_iv, ivlen, g_A, alen, g_P, plen, g_au8Out, &size, &plen_aligned);

        printf("C=%s\n", sElements[i].pchC);
        printf("T=%s\n", sElements[i].pchTag);

        str2bin(sElements[i].pchC, g_C, plen);
        str2bin(sElements[i].pchTag, g_T, tlen);

        if (memcmp(g_C, g_au8Out, plen))
        {
            printf("ERR: Encrypted data fail!\n");
            return -1;
        }

        if (memcmp(g_T, &g_au8Out[plen_aligned], tlen))
        {

            printf("ERR: Tag fail!\n");
            printf("tlen = %d\n", tlen);
            DumpBuffHex(&g_au8Out[plen_aligned], tlen);

            return -1;
        }

        AES_GCMDec(g_key, klen, g_iv, ivlen, g_A, alen, g_au8Out, plen, g_au8Out2, &size, &plen_aligned);

        if (memcmp(g_P, g_au8Out2, plen))
        {
            printf("ERR: Encrypted data fail!\n");
            return -1;
        }

        if (memcmp(g_T, &g_au8Out2[plen_aligned], tlen))
        {
            printf("ERR: Tag fail!");
            return -1;
        }

        printf("Test PASS!\n");

    }

    while (1) {}

}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
