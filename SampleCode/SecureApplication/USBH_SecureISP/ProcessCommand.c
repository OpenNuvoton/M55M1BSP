/**************************************************************************//**
 * @file     ProcessCommand.c
 * @version  V1.00
 * $Revision: 1 $
 * @brief    Transmit command to SecureISP USB command mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_hid.h"

//#define DBG     printf

#ifndef DBG
    #define DBG(...)
#endif

#define CMD_CONNECT                 0x80
#define CMD_RESET                   0x81
#define CMD_WRITE                   0x83
#define CMD_ERASE                   0x84
#define CMD_GET_ID                  0x85

#define CMD_ECDH_PUB0               0x8600
#define CMD_ECDH_PUB1               0x8601
#define CMD_ECDH_GET_PUB0           0x8602
#define CMD_ECDH_GET_PUB1           0x8603
#define CMD_ECDH_RAND_PUB0          0x8604
#define CMD_ECDH_RAND_PUB1          0x8605
#define CMD_ECDH_GET_RAND_PUB0      0x8606
#define CMD_ECDH_GET_RAND_PUB1      0x8607

#define CMD_VENDOR_FUNC             0x8F00
#define CMD_RESYNC                  0x8000


#define ERR_CMD_CHECKSUM            0x7D
#define ERR_CMD_PACKET_NUM          0x50

void Hex2Reg(char input[], uint32_t volatile reg[]);
void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);
/* These functions are for process USBH read and write function */
void int_read_callback(HID_DEV_T *hdev, uint16_t ep_addr, int status, uint8_t *rdata, uint32_t data_len);
void int_write_callback(HID_DEV_T *hdev, uint16_t ep_addr, int staus, uint8_t *wbuff, uint32_t *data_len);

volatile HID_DEV_T  *g_pHDEV;

volatile uint32_t   g_u32TmpBuf[1024]; // 4K byte temp buffer

static uint32_t sysGetNum(void);


/* Maximum 64-bytes */
typedef struct
{
    /* Word-0 */
    uint16_t        u16CCITT;       /* CCITT checksum of from u8CmdID to Word-13 */
    uint16_t        u16CmdID;       /* Command ID */

    /* Word-1 */
    uint16_t        u16PacketID;    /* Packet ID */
    uint16_t        u16Len;         /* Valid data length in command data field */

    /* Word-2 ~ 13 */
    uint32_t        au32Data[12];   /* Command data */

    /* Word-14 */
    uint32_t        u32CRC32;       /* CRC32 from Word-0 to Word-13 for check cmd integrity */

    /* Word-15 */
    uint32_t        RSVD;           /* Reserved */

} CMD_PACKET_T;

typedef struct
{
    uint32_t        au32Key0[8];    /* 256-bits */
    uint32_t        au32Key1[8];    /* 256-bits */
} ECC_PUBKEY_T;


typedef struct
{
    uint32_t        u32CmdMask;

    uint32_t        u32PID;
    uint32_t        u32CID;
    uint32_t        u32DID;
    uint32_t        au32UID[3];
    uint32_t        au32UCID[4];

    uint32_t        au32AESIV[4];       /* AES IV, 128-bits */
    uint32_t        au32AESKey[8];      /* AES-256 keys */

    uint32_t        u32KPKEYSTS;
    uint32_t        u32KPCNTSTS;

    ECC_PUBKEY_T    ServerPubKey;       /* Server's ECC public key, 64-bytes (256-bits + 256-bits) */
    ECC_PUBKEY_T    ClientPubKey;       /* Client's ECC public key, 64-bytes (256-bits + 256-bits) */

    volatile uint32_t u32IsRXReady;

    char            d[68];
    char            Qx[68];
    char            Qy[68];
    char            KeyZ[68];
    uint32_t        au32Tmp[FMC_FLASH_PAGE_SIZE / 4];
} CHIP_ISP_INFO_T;


__ALIGNED(4) CHIP_ISP_INFO_T g_ChipISPInfo;
__ALIGNED(4) CMD_PACKET_T    g_WriteCmd;
__ALIGNED(4) CMD_PACKET_T    g_ReturnData;


void  int_read_callback(HID_DEV_T *hdev, uint16_t ep_addr, int status, uint8_t *rdata, uint32_t data_len)
{
    /*
     *  USB host HID driver notify user the transfer status via <status> parameter. If the
     *  If <status> is 0, the USB transfer is fine. If <status> is not zero, this interrupt in
     *  transfer failed and HID driver will stop this pipe. It can be caused by USB transfer error
     *  or device disconnected.
     */
    if (status < 0)
    {
        DBG("Interrupt in transfer failed! status: %d\n", status);
        return;
    }

    memcpy((void *)&g_ReturnData, (void *)rdata, data_len);

    g_ChipISPInfo.u32IsRXReady = 0x5A5A;
}

void  int_write_callback(HID_DEV_T *hdev, uint16_t ep_addr, int staus, uint8_t *wbuff, uint32_t *data_len)
{
    //printf("[%s] ep_addr: 0x%X\n", __func__, ep_addr);
    memcpy(wbuff, (uint8_t *)&g_WriteCmd, sizeof(g_WriteCmd)); /* Fill data to be sent via interrupt out pipe     */
    *data_len = sizeof(g_WriteCmd);
}

void CRYPTO_IRQHandler()
{
    ECC_Complete(CRYPTO);
}

static uint32_t Swap32(uint32_t val)
{
    return (val << 24) | ((val << 8) & 0xff0000) | ((val >> 8) & 0xff00) | (val >> 24);
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

__attribute__((unused)) static int32_t _strkeycmp(char *str1, char *str2)
{
    int32_t len1, len2;
    int32_t i;
    char ch1, ch2;

    len1 = strlen(str1);
    len2 = strlen(str2);

    if (len1 != len2)
        return -1;

    for (i = 0; i < len1; i++)
    {
        ch1 = str1[i];
        ch2 = str2[i];
        ch1 = ((ch1 >= 'a') && (ch1 <= 'f')) ? (ch1 + 'A' - 'a') : (ch1);
        ch2 = ((ch2 >= 'a') && (ch2 <= 'f')) ? (ch2 + 'A' - 'a') : (ch2);

        if (ch1 != ch2)
            return -1;
    }

    return 0;

}

char My_GetChar(void)
{
    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }
}
static uint32_t sysGetNum(void)
{
    uint32_t i = 0;
    uint8_t InChar = 0x0, InString[16] = {0};

    while (InChar != 0x0D)
    {
        InChar = My_GetChar();

        if (InChar == 27)
        {
            return InChar;
        }

        if (InChar == 'x' || InChar == 'X' || InChar == 'f' ||
                InChar == 'F' || InChar == 'r' || InChar == 'R')
        {
            return InChar;
        }

        if (InChar == '-')
        {
            InString[i] = InChar;
            printf("%c", InChar);
            i++;
        }
        else if ((InChar >= '0') && (InChar <= '9'))
        {
            InString[i] = InChar;
            printf("%c", InChar);
            i++;
        }
    }

    printf("\n");

    return atoi((const char *)InString);
}

/*
    * CCITT (0xFFFF)
    * mode: 0: calculate; 1: verify
*/
static uint16_t _Perform_CCITT(uint32_t *pu32buf, uint16_t len, uint8_t mode)
{
    volatile uint16_t   i;
    uint16_t            *pu16buf, u32OrgSum, u32CalSum;

    if (len > 56) // valid data byte count
        return -1;

    pu16buf = (uint16_t *)pu32buf;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC_Open(CRC_CCITT, 0, 0xFFFFul, CRC_CPU_WDATA_16);

    for (i = 1; i < (len / 2); i++)
        CRC->DAT = *(pu16buf + i);

    u32OrgSum = *(pu16buf + 0);
    u32CalSum = (CRC->CHECKSUM & 0xFFFFul);

    /* Clear CRC checksum */
    CRC->SEED = 0xFFFFul;
    CRC->CTL |= CRC_CTL_CHKSINIT_Msk;

    if (mode == 0)
    {
        *(pu16buf + 0) = u32CalSum;
        return u32CalSum;
    }
    else if (mode == 1)
    {
        /* Verify CCITT checksum */
        if (u32OrgSum == u32CalSum)
            return 0;   /* Verify CCITT Pass */
        else
            return -1;  /* Verify CCITT Fail */
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
static uint32_t _Perform_CRC32(uint32_t *pu32buf, uint16_t len, uint8_t mode)
{
    uint16_t i;
    uint32_t u32OrgSum, u32CalSum;

    if (len > 60) // valid data byte count
        return -1;

    CLK_EnableModuleClock(CRC0_MODULE);
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_COM | CRC_CHECKSUM_RVS), 0xFFFFFFFFul, CRC_CPU_WDATA_32);

    for (i = 0; i < (len / 4) - 1; i++)
        CRC->DAT = *(pu32buf + i);

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

//    //key = "a000000000000000000000000000000000000000000000000000000000000001"
//    //CRYPTO->AES0_KEY[0] = 0xa0000000;
//    //CRYPTO->AES0_KEY[1] = 0x00000000;
//    //CRYPTO->AES0_KEY[2] = 0x00000000;
//    //CRYPTO->AES0_KEY[3] = 0x00000000;
//    //CRYPTO->AES0_KEY[4] = 0x00000000;
//    //CRYPTO->AES0_KEY[5] = 0x00000000;
//    //CRYPTO->AES0_KEY[6] = 0x00000000;
//    //CRYPTO->AES0_KEY[7] = 0x00000001;
//
//    //iv = "1000000000000000000000000000000a"
//    //CRYPTO->AES0_IV[0] = 0x10000000;
//    //CRYPTO->AES0_IV[1] = 0x00000000;
//    //CRYPTO->AES0_IV[2] = 0x00000000;
//    //CRYPTO->AES0_IV[3] = 0x0000000a;
/**
  * @brief      Perform AES-256 CFB NoPadding encrypt
  */
static int32_t _AES256Encrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    volatile int32_t    i;
    uint32_t u32TimeOutCnt;

    /* KEY and IV are byte order (32 bit) reversed, Swap32(x)) and stored in ISP_INFO_T */
    memcpy((void *)&CRYPTO->AES_KEY[0], KEY, (4 * 8));
    memcpy((void *)&CRYPTO->AES_IV[0], IV, (4 * 4));

    CRYPTO->AES_SADDR = (uint32_t)in;
    CRYPTO->AES_DADDR = (uint32_t)out;
    CRYPTO->AES_CNT   = len;
    CRYPTO->AES_CTL = ((AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos));
    CRYPTO->AES_CTL |= (CRYPTO_AES_CTL_ENCRYPTO_Msk);
    CRYPTO->AES_CTL |= ((AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (CRYPTO->AES_STS & CRYPTO_AES_STS_BUSY_Msk)
    {
        if (--u32TimeOutCnt == 0)
            return -1;
    }

    return 0;
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
static int32_t _AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    volatile int32_t    i;
    uint32_t u32TimeOutCnt;

    /* KEY and IV are byte order (32 bit) reversed, Swap32(x)) and stored in ISP_INFO_T */
    memcpy((void *)&CRYPTO->AES_KEY[0], KEY, (4 * 8));
    memcpy((void *)&CRYPTO->AES_IV[0], IV, (4 * 4));

    CRYPTO->AES_SADDR = (uint32_t)in;
    CRYPTO->AES_DADDR = (uint32_t)out;
    CRYPTO->AES_CNT   = len;
    CRYPTO->AES_CTL = ((AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos));
    CRYPTO->AES_CTL |= ((AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (CRYPTO->AES_STS & CRYPTO_AES_STS_BUSY_Msk)
    {
        if (--u32TimeOutCnt == 0)
            return -1;
    }

    return 0;
}

static int32_t _Perform_GenPacket(CMD_PACKET_T *pCMD)
{
    volatile int32_t    i;

#if (0)
    {
        uint32_t *pu32;
        pu32 = (uint32_t *)pCMD;
        DBG("Raw RSP data:\n");

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
    /* Generate CCITT */
    _Perform_CCITT((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 8, 0);

    for (i = 0; i < sizeof(g_ChipISPInfo.au32AESKey) / 4; i++)
    {
        if (g_ChipISPInfo.au32AESKey[i] != 0x0ul)
            break;
    }

    /* if i == 8, NO AES key, do not encrypt the cmd data */
    if (i == 8)
    {
        DBG("\nDo not encrypt cmd data. \n");
    }
    else
    {
        if (_AES256Encrypt(pCMD->au32Data, pCMD->au32Data, sizeof(pCMD->au32Data), g_ChipISPInfo.au32AESKey, g_ChipISPInfo.au32AESIV) < 0)
        {
            return -1;
        }
    }

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
    _Perform_CRC32((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 4, 0);

    return 0;
}

static int32_t _Perform_ParsePacket(CMD_PACKET_T *pCMD)
{
    volatile int32_t    i;

#if (0)
    {
        uint32_t *pu32;
        pu32 = (uint32_t *)pCMD;
        DBG("Raw REQ data:\n");

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

    /* verify CRC32 */
    if (_Perform_CRC32((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 4, 1) != 0)
    {
        DBG("\n\tPacket CRC32 mismatch!\n");
        return -1;
    }

    for (i = 0; i < sizeof(g_ChipISPInfo.au32AESKey) / 4; i++)
    {
        if (g_ChipISPInfo.au32AESKey[i] != 0x0ul)
            break;
    }

    /* if i == 8, NO AES key, do not decrypt the cmd data */
    if (i == 8)
    {
        DBG("\nDo not decrypt cmd data. \n");
    }
    else
    {
        if (_AES256Decrypt(pCMD->au32Data, pCMD->au32Data, sizeof(pCMD->au32Data), g_ChipISPInfo.au32AESKey, g_ChipISPInfo.au32AESIV) < 0)
        {
            return -1;
        }
    }

#if (0)
    {
        uint32_t *pu32;
        pu32 = (uint32_t *)pCMD;
        DBG("AES KEY:\n");

        for (i = 0; i < sizeof(g_ChipISPInfo.au32AESKey) / 4; i++)
            DBG("   0x%08x", g_ChipISPInfo.au32AESKey[i]);

        DBG("\nAES IV:\n");

        for (i = 0; i < sizeof(g_ChipISPInfo.au32AESIV) / 4; i++)
            DBG("   0x%08x", g_ChipISPInfo.au32AESIV[i]);

        DBG("\nREQ data(decryption?):\n");

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
    if (_Perform_CCITT((uint32_t *)pCMD, sizeof(CMD_PACKET_T) - 8, 1) != 0)
    {
        DBG("\n\tPacket CCITT mismatch!\n");
        return -1;
    }

    DBG("Parse cmd - Pass.\n\n");
    return 0;
}

/* Stage 1. */
static int32_t Process_Connect(void)
{
    uint32_t u32PacketID = 0x48;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_CONNECT;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    g_ChipISPInfo.u32PID        = g_ReturnData.au32Data[1];
    g_ChipISPInfo.au32AESIV[0]  = Swap32(g_ReturnData.au32Data[2]);
    g_ChipISPInfo.au32AESIV[1]  = Swap32(g_ReturnData.au32Data[3]);
    g_ChipISPInfo.au32AESIV[2]  = Swap32(g_ReturnData.au32Data[4]);
    g_ChipISPInfo.au32AESIV[3]  = Swap32(g_ReturnData.au32Data[5]);

    return 0;
}

/* ECC key pair */
char g_acPriv[] = "380a67fcfc01ca7073da7c2c54296a61327f77262a7d4674c3d8e29a63e3fa20";
char g_acPub0[] = "755b3819f05a3e9f32d4d599062834aac5220f75955378414a8f63716a152ce2";
char g_acPub1[] = "91c413f1915ed7b47473fd797647ba3d83e8224377909af5b30c530eaad79fd7";
static int32_t ECDH_0_SendPub0(void)
{
    uint32_t    i, u32PacketID = 0xE0;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_PUB0;
    g_WriteCmd.u16PacketID  = u32PacketID;
    memcpy(&g_WriteCmd.au32Data[0], g_ChipISPInfo.ServerPubKey.au32Key0, sizeof(g_ChipISPInfo.ServerPubKey.au32Key0));

    for (i = 0; i < 8; i++)
        DBG("Out pub0[%d]: 0x%08x.\n", i, g_WriteCmd.au32Data[i]);

    g_WriteCmd.u16Len       = sizeof(g_ChipISPInfo.ServerPubKey.au32Key0);

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    return 0;
}

/* The ECC public key pair of USB client, e.g SecureISPDemo */
char g_acClientPub0[] = "d32438a1b4428541c564eeed79669b4bd3bf601c758469545e013c8fe8af7ef6";
char g_acClientPub1[] = "476de8f3c6e6c48a8bacf1e1827cfb82501833c2bb816344f996533b1b031706";
static int32_t ECDH_1_SendPub1(void)
{
    uint32_t    i, u32PacketID = 0xE1;
    uint32_t    tmp[8];

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_PUB1;
    g_WriteCmd.u16PacketID  = u32PacketID;
    memcpy(&g_WriteCmd.au32Data[0], g_ChipISPInfo.ServerPubKey.au32Key1, sizeof(g_ChipISPInfo.ServerPubKey.au32Key1));

    for (i = 0; i < 8; i++)
        DBG("Out pub0[%d]: 0x%08x.\n", i, g_WriteCmd.au32Data[i]);

    g_WriteCmd.u16Len       = sizeof(g_ChipISPInfo.ServerPubKey.au32Key1);

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    /* Generate 1st ECDH key */
    ECC_ENABLE_INT(CRYPTO);

    if (ECC_GenerateSecretZ(CRYPTO, CURVE_P_256, (char *)g_acPriv, (char *)g_acClientPub0, (char *)g_acClientPub1, g_ChipISPInfo.KeyZ) < 0)
    {
        DBG("ECC ECDH share key calculation fail!!\n");
        return -1;
    }

    DBG("Generate 1st ECDH: %s\n\n", g_ChipISPInfo.KeyZ);

    Hex2Reg((char *)g_ChipISPInfo.KeyZ, tmp);
    BytesSwap((char *)tmp, sizeof(tmp));

    for (i = 0; i < 8; i++)
        g_ChipISPInfo.au32AESKey[i] = Swap32(tmp[i]);

    return 0;
}

static int32_t ECDH_2_GetPub0(void)
{
    uint32_t u32PacketID = 0xE2;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_GET_PUB0;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    memcpy(g_ChipISPInfo.ClientPubKey.au32Key0, (char *)(&g_ReturnData.au32Data[1]), sizeof(g_ChipISPInfo.ClientPubKey.au32Key0));

    return 0;
}

static int32_t ECDH_3_GetPub1(void)
{
    uint32_t u32PacketID = 0xE3;
    uint32_t tmp[8];

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_GET_PUB1;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];


    memcpy(g_ChipISPInfo.ClientPubKey.au32Key1, (char *)(&g_ReturnData.au32Data[1]), sizeof(g_ChipISPInfo.ClientPubKey.au32Key1));

    memset(g_ChipISPInfo.Qx, 0x0, sizeof(g_ChipISPInfo.Qx));
    memset(g_ChipISPInfo.Qy, 0x0, sizeof(g_ChipISPInfo.Qy));

    /* Set Qx */
    memcpy(tmp, (char *)g_ChipISPInfo.ClientPubKey.au32Key0, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, g_ChipISPInfo.Qx);

    /* Set Qy */
    memcpy(tmp, (char *)g_ChipISPInfo.ClientPubKey.au32Key1, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, g_ChipISPInfo.Qy);

    DBG("Qx: %s\n", g_ChipISPInfo.Qx);
    DBG("Qy: %s\n", g_ChipISPInfo.Qy);

#if 0 /* Identify SecureISPDemo's public key ? */

    if (_strkeycmp(g_ChipISPInfo.Qx, (char *)g_acClientPub0) != 0)
    {
        DBG("Qx [%s] is not matched with expected [%s]!\n", g_ChipISPInfo.Qx, g_acClientPub0);
        return -1;
    }

    if (_strkeycmp(g_ChipISPInfo.Qy, (char *)g_acClientPub1) != 0)
    {
        DBG("Qy [%s] is not matched with expected [%s]!\n", g_ChipISPInfo.Qx, g_acClientPub1);
        return -1;
    }

    DBG("\n");
#endif

    return 0;
}

static uint8_t Byte2Char(uint8_t c)
{
    if (c < 10)
        return (c + '0');

    if (c < 16)
        return (c - 10 + 'a');

    return 0;
}
static void Generate_RandECCKey(void)
{
    int32_t  i, j;
    int32_t  i32NBits;
    uint32_t au32Buf[8];
    uint8_t  *pu8Buf;

    memset(g_ChipISPInfo.d, 0x0, sizeof(g_ChipISPInfo.d));
    memset(g_ChipISPInfo.Qx, 0x0, sizeof(g_ChipISPInfo.Qx));
    memset(g_ChipISPInfo.Qy, 0x0, sizeof(g_ChipISPInfo.Qy));

    i32NBits = 256;

    ECC_ENABLE_INT(CRYPTO);

    do
    {
        /* Generate random number for private key */
        if (RNG_Random(au32Buf, 8) == 0)
            return ;

        pu8Buf = (uint8_t *)au32Buf;

        for (i = 0, j = 0; i < i32NBits / 8; i++)
        {
            g_ChipISPInfo.d[j++] = Byte2Char(pu8Buf[i] & 0xf);
            g_ChipISPInfo.d[j++] = Byte2Char(pu8Buf[i] >> 4);
        }

        g_ChipISPInfo.d[j] = 0; // NULL end

        DBG("Private key = %s\n", g_ChipISPInfo.d);

        /* Check if the private key valid */
        if (ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_256, g_ChipISPInfo.d))
        {
            //DBG("Private key check ok\n");
            break;
        }
        else
        {
            /* Invalid key */
            DBG("Current private key is not valid. Need a new one.\n");
        }
    } while (1);

    /* Generate public */
    if (ECC_GeneratePublicKey(CRYPTO, CURVE_P_256, g_ChipISPInfo.d, g_ChipISPInfo.Qx, g_ChipISPInfo.Qy) < 0)
    {
        printf("ECC key generation failed!!\n");

        while (1) {}
    }

    DBG("Public Qx is  %s\n", g_ChipISPInfo.Qx);
    DBG("Public Qy is  %s\n", g_ChipISPInfo.Qy);
    DBG("\n");

    Hex2Reg((char *)g_ChipISPInfo.Qx, g_ChipISPInfo.ServerPubKey.au32Key0);
    Hex2Reg((char *)g_ChipISPInfo.Qy, g_ChipISPInfo.ServerPubKey.au32Key1);
    BytesSwap((char *)g_ChipISPInfo.ServerPubKey.au32Key0, sizeof(g_ChipISPInfo.ServerPubKey.au32Key0));
    BytesSwap((char *)g_ChipISPInfo.ServerPubKey.au32Key1, sizeof(g_ChipISPInfo.ServerPubKey.au32Key1));
}

static int32_t ECDH_4_SendRandPub0(void)
{
    uint32_t    i, u32PacketID = 0xE4;

    Generate_RandECCKey();

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_RAND_PUB0;
    g_WriteCmd.u16PacketID  = u32PacketID;
    memcpy(&g_WriteCmd.au32Data[0], g_ChipISPInfo.ServerPubKey.au32Key0, sizeof(g_ChipISPInfo.ServerPubKey.au32Key0));

    for (i = 0; i < 8; i++)
        DBG("Out pub0[%d]: 0x%08x.\n", i, g_WriteCmd.au32Data[i]);

    g_WriteCmd.u16Len       = sizeof(g_ChipISPInfo.ServerPubKey.au32Key0);

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    return 0;
}

static int32_t ECDH_5_SendRandPub1(void)
{
    uint32_t    i, u32PacketID = 0xE5;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_RAND_PUB1;
    g_WriteCmd.u16PacketID  = u32PacketID;
    memcpy(&g_WriteCmd.au32Data[0], g_ChipISPInfo.ServerPubKey.au32Key1, sizeof(g_ChipISPInfo.ServerPubKey.au32Key1));

    for (i = 0; i < 8; i++)
        DBG("Out pub0[%d]: 0x%08x.\n", i, g_WriteCmd.au32Data[i]);

    g_WriteCmd.u16Len       = sizeof(g_ChipISPInfo.ServerPubKey.au32Key1);

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    return 0;
}

static int32_t ECDH_6_GetRandPub0(void)
{
    uint32_t u32PacketID = 0xE6;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_GET_RAND_PUB0;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    memcpy(g_ChipISPInfo.ClientPubKey.au32Key0, (char *)(&g_ReturnData.au32Data[1]), sizeof(g_ChipISPInfo.ClientPubKey.au32Key0));

    return 0;
}

static int32_t ECDH_7_GetRandPub1(void)
{
    uint32_t    i, u32PacketID = 0xE7;
    uint32_t    tmp[8];

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ECDH_GET_RAND_PUB1;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];



    /* Generate 2nd ECDH key */
    memcpy(g_ChipISPInfo.ClientPubKey.au32Key1, (char *)(&g_ReturnData.au32Data[1]), sizeof(g_ChipISPInfo.ClientPubKey.au32Key1));

    memset(g_ChipISPInfo.Qx,   0x0, sizeof(g_ChipISPInfo.Qx));
    memset(g_ChipISPInfo.Qy,   0x0, sizeof(g_ChipISPInfo.Qy));

    /* Set Qx */
    memcpy(tmp, (char *)g_ChipISPInfo.ClientPubKey.au32Key0, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, g_ChipISPInfo.Qx);

    /* Set Qy */
    memcpy(tmp, (char *)g_ChipISPInfo.ClientPubKey.au32Key1, sizeof(tmp));
    BytesSwap((char *)tmp, sizeof(tmp));
    Reg2Hex(64, tmp, g_ChipISPInfo.Qy);

    DBG("Qx: %s\n", g_ChipISPInfo.Qx);
    DBG("Qy: %s\n", g_ChipISPInfo.Qy);

    ECC_ENABLE_INT(CRYPTO);

    if (ECC_GenerateSecretZ(CRYPTO, CURVE_P_256, (char *)g_ChipISPInfo.d, (char *)g_ChipISPInfo.Qx, (char *)g_ChipISPInfo.Qy, g_ChipISPInfo.KeyZ) < 0)
    {
        DBG("ECC ECDH share key calculation fail!!\n");
        return -1;
    }

    DBG("Generate 2nd ECDH: %s\n\n", g_ChipISPInfo.KeyZ);

    Hex2Reg((char *)g_ChipISPInfo.KeyZ, tmp);
    BytesSwap((char *)tmp, sizeof(tmp));

    for (i = 0; i < 8; i++)
        g_ChipISPInfo.au32AESKey[i] = Swap32(tmp[i]);

    return 0;
}

static int32_t Process_ECDH(void)
{
    Hex2Reg((char *)g_acPub0, g_ChipISPInfo.ServerPubKey.au32Key0);
    Hex2Reg((char *)g_acPub1, g_ChipISPInfo.ServerPubKey.au32Key1);
    BytesSwap((char *)g_ChipISPInfo.ServerPubKey.au32Key0, sizeof(g_ChipISPInfo.ServerPubKey.au32Key0));
    BytesSwap((char *)g_ChipISPInfo.ServerPubKey.au32Key1, sizeof(g_ChipISPInfo.ServerPubKey.au32Key1));

    if (ECDH_0_SendPub0() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_0_SendPub0.\n");
        return -1;
    }

    if (ECDH_1_SendPub1() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_1_SendPub1.\n");
        return -1;
    }

    /* The following commands are encrypted by 1st ECDH key */
    if (ECDH_2_GetPub0() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_2_GetPub0.\n");
        return -1;
    }

    if (ECDH_3_GetPub1() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_3_GetPub1.\n");
        return -1;
    }

    if (ECDH_4_SendRandPub0() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_4_SendRandPub0.\n");
        return -1;
    }

    if (ECDH_5_SendRandPub1() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_5_SendRandPub1.\n");
        return -1;
    }

    if (ECDH_6_GetRandPub0() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_6_GetRandPub0.\n");
        return -1;
    }

    if (ECDH_7_GetRandPub1() != 0)
    {
        DBG("\n[FAIL] ==> ECDH_7_GetRandPub1.\n");
        return -1;
    }

    return 0;
}

static int32_t Process_GetID(void)
{
    uint32_t u32PacketID = 0x30;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_GET_ID;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 0;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];


    g_ChipISPInfo.u32PID        = g_ReturnData.au32Data[1];
    memcpy(g_ChipISPInfo.au32UID, &g_ReturnData.au32Data[2], sizeof(g_ChipISPInfo.au32UID));
    memcpy(g_ChipISPInfo.au32UCID, &g_ReturnData.au32Data[5], sizeof(g_ChipISPInfo.au32UCID));
    g_ChipISPInfo.u32CID        = g_ReturnData.au32Data[9];
    g_ChipISPInfo.u32DID        = g_ReturnData.au32Data[10];


    printf("* PID:  0x%08x\n", g_ChipISPInfo.u32PID);
    printf("* UID:  0x%08x, 0x%08x, 0x%08x\n",
           g_ChipISPInfo.au32UID[0], g_ChipISPInfo.au32UID[1], g_ChipISPInfo.au32UID[2]);
    printf("* UCID: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
           g_ChipISPInfo.au32UCID[0], g_ChipISPInfo.au32UCID[1], g_ChipISPInfo.au32UCID[2], g_ChipISPInfo.au32UCID[3]);
    printf("\n");

    return 0;
}

static int32_t Process_EraseFlash(uint32_t u32Addr, uint32_t u32PageCount)
{
    uint32_t u32PacketID = 0x31;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_ERASE;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 8;
    g_WriteCmd.au32Data[0]  = u32Addr;
    g_WriteCmd.au32Data[1]  = u32PageCount;

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    return 0;
}

static int32_t Process_WriteFlash(uint32_t u32Addr, uint32_t u32ByteCount, uint32_t *pu32Buf)
{
    uint32_t    i, u32PacketID = 0x32;

    if (u32ByteCount > 40)
        return -1; // valid data length can not large than 40 bytes

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_WRITE;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 4 + u32ByteCount;
    g_WriteCmd.au32Data[0]  = u32Addr;
    g_WriteCmd.au32Data[1]  = u32ByteCount;

    for (i = 0; i < (u32ByteCount / 4); i++)
        g_WriteCmd.au32Data[i + 2]  = pu32Buf[i];

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    if (g_ReturnData.au32Data[0] != 0)
        return g_ReturnData.au32Data[0];

    return 0;
}

static int32_t Process_VendorFunc(uint32_t *pu32Buf)
{
    uint32_t    i, u32PacketID = 0x99;

    if (pu32Buf[0] > 44)
        return -1; // valid data length can not large than 44 bytes

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_VENDOR_FUNC;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.au32Data[0]  = 0x0; // RESERVED

    /* Valid input data starting from g_WriteCmd.au32Data[1] */
    switch (pu32Buf[1])
    {
        case 0x1000:    // get ID via vendor command
        case 0x2000:    // read flash via vendor command
        case 0x3000:    // write flash via vendor command
            for (i = 0; i < (pu32Buf[0] / 4); i++)
                g_WriteCmd.au32Data[1 + i] = pu32Buf[1 + i];

            g_WriteCmd.u16Len = 4 + pu32Buf[0];
            break;

        default:
            DBG("\n[Invalid sub command] (L:%d)\n", __LINE__);
            return 0;
    }

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    /*
        Valid output data starting from g_ReturnData.au32Data[1]
        g_ReturnData.au32Data[1]  : byte counts
        g_ReturnData.au32Data[2~] : data buffer
    */
    switch (pu32Buf[1])
    {
        case 0x1000:    // get ID via vendor command
        case 0x2000:    // read flash data ID via vendor command
            for (i = 0; i < (g_ReturnData.au32Data[1] / 4); i++)
                printf("0x%08x, ", g_ReturnData.au32Data[2 + i]);

            printf("\n");
            break;

        case 0x3000:    // write flash via vendor command
            break;
    }

    return 0;
}

static int32_t Process_ReSyncDevice(void)
{
    uint32_t u32PacketID = 0x31;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_RESYNC;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 4;
    g_WriteCmd.au32Data[0]  = 0; // CHIP reset

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    while (g_ChipISPInfo.u32IsRXReady != 0x5A5A) {}

    if (_Perform_ParsePacket(&g_ReturnData) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_ParsePacket. (L:%d)\n", __LINE__);
        return -1;
    }

    if (g_ReturnData.u16PacketID != u32PacketID)
        return ERR_CMD_PACKET_NUM;

    return 0;
}

static int32_t Process_ResetDevice(void)
{
    uint32_t u32PacketID = 0x31;

    memset(&g_WriteCmd, 0x0, sizeof(CMD_PACKET_T));
    memset(&g_ReturnData, 0xFF, sizeof(CMD_PACKET_T));

    g_WriteCmd.u16CmdID     = CMD_RESET;
    g_WriteCmd.u16PacketID  = u32PacketID;
    g_WriteCmd.u16Len       = 4;
    g_WriteCmd.au32Data[0]  = 0; // CHIP reset

    if (_Perform_GenPacket(&g_WriteCmd) != 0)
    {
        DBG("\n[FAIL] ==> _Perform_GenPacket. (L:%d)\n", __LINE__);
        return -1;
    }

    g_ChipISPInfo.u32IsRXReady = 0;

    usbh_hid_stop_int_write((HID_DEV_T *)g_pHDEV, 0);

    if (usbh_hid_start_int_write((HID_DEV_T *)g_pHDEV, 0, int_write_callback) != HID_RET_OK)
    {
        DBG("\n[FAIL] ==> Interrupt out transfer started...\n");
        return -1;
    }

    return 0;
}

int32_t Process_USBHCommand(HID_DEV_T *hdev)
{
    uint32_t i, j;
    int32_t  i32RetCode;
    uint32_t u32Item, u32CmdID,
             u32FlashPageCnt   = 2,
             u32FlashStartAddr = (uint32_t)(FMC_APROM_BASE + 0x10000),
             u32FlashEndAddr   = (uint32_t)(FMC_APROM_BASE + 0x10000 + (FMC_FLASH_PAGE_SIZE * u32FlashPageCnt));

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Install USBH read function */
    if (usbh_hid_start_int_read(hdev, 0, int_read_callback) != HID_RET_OK)
    {
        printf("[FAIL] ==> Interrupt in transfer started...\n");

        while (1) {}
    }

    g_pHDEV = hdev;

    printf("\n\n****************************************\n\n");

    printf("Click any key to connect with SecureISP client. 'r' to reset Server.\n\n");

    if (My_GetChar() == 'r')
    {
        SYS_ResetChip();
    }

    Process_ReSyncDevice();
    delay_us(500000);

    /* Enable ECC IRQ */
    ECC_ENABLE_INT(CRYPTO);
    NVIC_EnableIRQ(CRYPTO_IRQn);
    /* Initial Random Number Generator */
    RNG_Open();

    memset(&g_ChipISPInfo, 0x0, sizeof(CHIP_ISP_INFO_T));

    /* Step 1. Wait connect command successfully */
    while (1)
    {
        printf("[Process connect ...]\n\n");

        if (Process_Connect() == 0)
            break;

        My_GetChar();
    }

    /* Step 2. Wait ECDH successfully */
    printf("[Processing ECDH key exchange ...]\n\n");

    if (Process_ECDH() != 0)
        while (1) {}


    /* Step 3. To process commands */
    while (1)
    {
        printf("\n");
        printf("=============================\n");
        printf("   [ 0] Read ID\n");
        printf("   [ 1] Erase flash from 0x%08X to 0x%08X\n", u32FlashStartAddr, u32FlashEndAddr);
        printf("   [ 2] Write flash from 0x%08X to 0x%08X\n", u32FlashStartAddr, u32FlashEndAddr);
        printf("        * Need to execute \"Erase flash\" command before \"Write flash\" command\n");
        printf("   [ 3] Vendor function - Read ID\n");
        printf("   [ 4] Vendor function - Read  flash from 0x%08X to 0x%08X\n", u32FlashStartAddr, u32FlashEndAddr);
        printf("   [ 5] Vendor function - Write flash from 0x%08X to 0x%08X\n", u32FlashStartAddr, u32FlashEndAddr);
        printf("        * Need to execute \"Erase flash\" command before \"Write flash\" command\n");
        printf("=============================\n");
        printf("   [90] Reset device\n");
        printf("=============================\n");

        u32Item = sysGetNum();
        printf("[Select: %d]\n\n", u32Item);

        if (u32Item == 0)
        {
            if ((i32RetCode = Process_GetID()) != 0)
            {
                printf("\n(%d@%s) Process_GetID FAIL (%d) !\n", __LINE__, __FILE__, i32RetCode);

                while (1) {}
            }
        }

        if (u32Item == 1)
        {
            if ((i32RetCode = Process_EraseFlash(u32FlashStartAddr, u32FlashPageCnt)) != 0)
            {
                printf("\n(%d@%s) Process_EraseFlash FAIL (%d) !\n", __LINE__, __FILE__, i32RetCode);

                while (1) {}
            }
        }

        if (u32Item == 2)
        {
            i = j = 0;

            do
            {
                for (i = 0; i < (32 / 4); i++)
                {
                    g_ChipISPInfo.au32Tmp[i] = 0x5a5a0000 + i + (j / 4);
                }

                if ((i32RetCode = Process_WriteFlash((u32FlashStartAddr + j), 32, g_ChipISPInfo.au32Tmp)) != 0)
                {
                    printf("\n(%d@%s) Process_WriteFlash FAIL (%d) !\n", __LINE__, __FILE__, i32RetCode);

                    while (1) {}
                }

                j += 32;
            } while ((u32FlashStartAddr + j) < u32FlashEndAddr);
        }

        if (u32Item == 3)
        {
            u32CmdID = 0x1000;
            g_ChipISPInfo.au32Tmp[0] = (4 * 1);     // vendor cmd: byte counts
            g_ChipISPInfo.au32Tmp[1] = u32CmdID;    // vendor cmd: command ID

            if ((i32RetCode = Process_VendorFunc(g_ChipISPInfo.au32Tmp)) != 0)
            {
                printf("\n(%d@%s) Vendor cmd:0x%x FAIL (%d) !\n", __LINE__, __FILE__, u32CmdID, i32RetCode);

                while (1) {}
            }
        }

        if (u32Item == 4)
        {
            u32CmdID = 0x2000;
            i = j = 0;

            do
            {
                g_ChipISPInfo.au32Tmp[0] = (4 * 3);                     // vendor cmd: byte counts
                g_ChipISPInfo.au32Tmp[1] = u32CmdID;                    // vendor cmd: command ID
                g_ChipISPInfo.au32Tmp[2] = (u32FlashStartAddr + j);     // vendor cmd: addr
                g_ChipISPInfo.au32Tmp[3] = 32;                          // vendor cmd: read byte size

                if ((i32RetCode = Process_VendorFunc(g_ChipISPInfo.au32Tmp)) != 0)
                {
                    printf("\n(%d@%s) Vendor cmd:0x%x FAIL (%d) ! (Addr:0x%x) \n", __LINE__, __FILE__, u32CmdID, i32RetCode, (uint32_t)(u32FlashStartAddr + j));

                    while (1) {}
                }

                j += 32;
            } while ((u32FlashStartAddr + j) < u32FlashEndAddr);
        }

        if (u32Item == 5)
        {
            u32CmdID = 0x3000;
            i = j = 0;

            do
            {
                g_ChipISPInfo.au32Tmp[0] = (4 * 11);                    // vendor cmd: byte counts
                g_ChipISPInfo.au32Tmp[1] = u32CmdID;                    // vendor cmd: command ID
                g_ChipISPInfo.au32Tmp[2] = (u32FlashStartAddr + j);     // vendor cmd: addr
                g_ChipISPInfo.au32Tmp[3] = 32;                          // vendor cmd: write byte size

                for (i = 0; i < (32 / 4); i++)
                {
                    g_ChipISPInfo.au32Tmp[4 + i] = 0x0000a5a5 + ((i + (j / 4)) << 16);
                }

                if ((i32RetCode = Process_VendorFunc(g_ChipISPInfo.au32Tmp)) != 0)
                {
                    printf("\n(%d@%s) Vendor cmd:0x%x FAIL (%d) ! (Addr:0x%x)\n", __LINE__, __FILE__, u32CmdID, i32RetCode, (uint32_t)(u32FlashStartAddr + j));


                    while (1) {}
                }

                j += 32;
            } while ((u32FlashStartAddr + j) < u32FlashEndAddr);
        }

        if (u32Item == 90)
        {
            if (Process_ResetDevice() == 0)
            {
                delay_us(1000000);
                SYS_ResetChip();
                break;
            }
        }
    }

    return 0;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
