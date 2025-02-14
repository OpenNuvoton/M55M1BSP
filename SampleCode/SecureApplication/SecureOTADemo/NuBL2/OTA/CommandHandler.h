/**************************************************************************//**
 * @file     CommandHandler.h
 * @version  V3.00
 * @brief    Secure ISP - Process commands
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __COMMAND_HANDLER_H__
#define __COMMAND_HANDLER_H__

#include "NuMicro.h"

#if defined (__DISABLE_MSG__)
    //#define printf(...)
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* Command ID Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define CMD_CONNECT                 0x80    /* connect cmd */
#define CMD_RESET                   0x81
#define CMD_READ_CONFIG             0x82
#define CMD_WRITE                   0x83
#define CMD_ERASE                   0x84
#define CMD_GET_PID                 0x85
#define CMD_GET_ID                  0x85
#define CMD_DH_KEY                  0x86
#define CMD_AUTH_KEY                0x87
#define CMD_AUTH_KPROM              0x87
#define CMD_CAL_ID_SIGNATURE        0x88
#define CMD_GET_ID_SIGNATURE        0x89    /* 0:get R; 1:get S */
#define CMD_READ_OTP                0x8D
#define CMD_DISCONNECT              0x8E    /* disconnect cmd */
#define CMD_GET_VERSION             0x8F

#define CMD_XOM_ERASE               0x90
#define CMD_XOM_SET                 0x91
#define CMD_SET_SBK                 0x96
#define CMD_SET_REGION_LOCK         0x97    /* 0:Secure region; 1:All region */
#define CMD_SET_KPROM               0x98    /* 0xA5: CFG not locked; 0xA4: CFG locked */
#define CMD_UPDATE_CFG              0x9A    /* CBS, MBS, EraseCFG */
#define CMD_SET_NS                  0x9B    /* Only available if no SCRLOCK and ARLOCK */
#define CMD_SET_IB_INFO             0x9C    /* Only available if no SCRLOCK and ARLOCK */
#define CMD_CAL_IB_INFO             0x9D    /* Only available if no SCRLOCK and ARLOCK */

#define CMD_ECDH_PUB0               0x8600  /* For Stage 2. only */
#define CMD_ECDH_PUB1               0x8601  /* For Stage 2. only */
#define CMD_ECDH_GET_PUB0           0x8602  /* For Stage 2. only */
#define CMD_ECDH_GET_PUB1           0x8603  /* For Stage 2. only */
#define CMD_ECDH_RAND_PUB0          0x8604  /* For Stage 2. only */
#define CMD_ECDH_RAND_PUB1          0x8605  /* For Stage 2. only */
#define CMD_ECDH_GET_RAND_PUB0      0x8606  /* For Stage 2. only */
#define CMD_ECDH_GET_RAND_PUB1      0x8607  /* For Stage 2. only */
#define CMD_GET_RAND_IV             0x8608
#define CMD_SET_RAND_IV             0x8609
#define CMD_SET_MASS_WRITE          0x8300
#define CMD_MASS_WRITE              0x8301
#define CMD_WRITE_OTP               0x8D00
#define CMD_IDENTIFY_SERVER         0x8700
#define CMD_EXEC_VENDOR_FUNC        0x8F00  /* 0x8F00 ~ 0x8FFF */
#define CMD_ERASE_KPROM             0x9801
#define CMD_KPROM_STS               0x9802
#define CMD_RESYNC                  0x8000
#define CMD_IS_MASKED               0x8888


/*---------------------------------------------------------------------------------------------------------*/
/*  Response Status Constant Definitions                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define STS_OK                      0x00
#define STS_XOM_ERASE_DONE          0x01
#define STS_NO_KPROM                0x02
#define STS_KPROM_MATCH             0x03
#define ERR_CMD_CONNECT             0x7F
#define ERR_CMD_INVALID             0x7E
#define ERR_CMD_CHECKSUM            0x7D
#define ERR_ISP_CONFIG              0x7C
#define ERR_ISP_WRITE               0x7B
#define ERR_INVALID_ADDRESS         0x7A
#define ERR_OVER_RANGE              0x79
#define ERR_PAGE_ALIGN              0x78
#define ERR_ISP_ERASE               0x77
#define ERR_DH_KEY                  0x76
#define ERR_DH_ARGUMENT             0x75
#define ERR_AUTH_KEY                0x74
#define ERR_AUTH_KEY_OVER           0x73
#define ERR_CMD_KEY_EXCHANGE        0x72
#define ERR_CMD_IDENTIFY            0x71
#define ERR_SPI_INVALID_PAGESIZE    0x70
#define ERR_TIMEOUT                 0x6F
#define ERR_CFG_MATCHED             0x6E
#define ERR_XOM_PARAMETER           0x6D
#define ERR_XOM_IS_ACTIVE           0x6C
#define ERR_XOM_NOT_ACTIVE          0x6B
#define ERR_SBK_EXIST               0x67
#define ERR_SET_SBK_FAIL            0x66
#define ERR_SREGION_LOCK            0x65
#define ERR_ALLREGION_LOCK          0x64
#define ERR_NS_BOUNDARY_EXIST       0x63
#define ERR_KPROM_IS_LOCKED         0x5F
#define ERR_PROGRAM_KPROM_FAIL      0x5E
#define ERR_ENABLE_KPROM_FAIL       0x5D
#define ERR_KPROM_LOCK_CFG_FAIL     0x5C
#define ERR_KPROM_KPMAX_FAIL        0x5B
#define ERR_KPROM_KEMAX_FAIL        0x5A
#define ERR_KPROM_IS_ENABLED        0x59
#define ERR_IB_INFO_NOT_READY       0x58
#define ERR_CAL_IB_INFO_FAIL        0x57
#define ERR_CAL_ID_SIGNATURE        0x56
#define ERR_VERIFY_ID_SIGNATURE     0x55
#define ERR_SET_ID_SIGNATURE        0x54
#define ERR_PARAMETER               0x53


/*---------------------------------------------------------------------------------------------------------*/
/*  typedef enum and struct definitions for CommandHandler                                                 */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    USB_MODE        = 0x1,
    UART_MODE       = 0x2,
    USB_UART_MODE   = 0x3,
    RESYNC_ISP      = 0x80,
} E_ISP_MODE;

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

#ifdef __cplusplus
extern "C"
{
#endif

typedef int32_t (*ISPCallback)(uint32_t *, uint32_t);
typedef void (*USBDEPFunc)(void);
typedef int32_t (*GenKeyFunc)(ECC_PUBKEY_T *, uint32_t *);
typedef struct
{
    uint32_t        u32CmdMask;
    /*
        BIT0: mask CMD_UPDATE_CFG
        BIT1: mask CMD_SET_REGION_LOCK
        BIT2: mask CMD_XOM_SET, CMD_XOM_ERASE
        BIT3: mask CMD_READ_OTP, CMD_WRITE_OTP
        BIT4: mask CMD_WRITE, CMD_ERASE, CMD_SET_MASS_WRITE, CMD_MASS_WRITE
        BIT5: mask CMD_ERASE_KPROM, CMD_SET_KPROM, CMD_AUTH_KPROM
        BIT6: mask CMD_RESET
        BIT7: mask CMD_MASS_ERASE

        BIT16: mask APROM update
        BIT17: mask LDROM update
    */

    uint32_t        au32AESKey[8];
    uint32_t        au32AESIV[4];

    ECC_PUBKEY_T    ClientPubKey;       /* 64-bytes (256-bits + 256-bits) */
    ECC_PUBKEY_T    ServerPubKey;       /* 64-bytes (256-bits + 256-bits) */

    uint32_t        IsConnectOK;
    uint32_t        timeout;

    uint8_t         rcvbuf[64];
    uint8_t         rspbuf[64];

    USBDEPFunc      pfnUSBDEP[USBD_MAX_EP];
    uint32_t        IsUSBDataReady;

    uint32_t        UARTClockFreq;
    uint32_t        UARTDataIdx;
    uint32_t        IsUARTDataReady;

    ISPCallback     pfnVendorFunc;

    GenKeyFunc      pfnGenKeyFunc;

    uint32_t        tmp0[8];
    uint32_t        tmp1[8];

} ISP_INFO_T;



/*---------------------------------------------------------------------------------------------------------*/
/*  Function API for CommandHandler                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void    CommandHandlerInit(void);
int32_t ParseCONNECT(ISP_INFO_T *pISPInfo);
int32_t ParseECDH(ISP_INFO_T *pISPInfo);
int32_t ParseCommands(ISP_INFO_T *pISPInfo);
void    BytesSwap(char *buf, int32_t len);

int32_t CMD_GenRspPacket(CMD_PACKET_T *pCMD, ISP_INFO_T *pISPInfo);
int32_t CMD_ParseReqPacket(CMD_PACKET_T *pCMD, ISP_INFO_T *pISPInfo);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_HANDLER_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
