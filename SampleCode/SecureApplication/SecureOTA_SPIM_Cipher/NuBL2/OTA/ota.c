/**************************************************************************//**
 * @file     ota.c
 * @version  V1.00
 * @brief    OTA demo code
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ota.h"

#include "NuBL_crypto.h"
#include "NuBL2.h"

#ifdef __DISABLE_MSG__
    #define printf(...)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t  g_u8IsFwUpgradeDone     = 0;
static volatile uint8_t  g_u8IsNuBL3xFwDone      = 0;
static volatile uint32_t g_u32SetFwUpgradeDone   = 0;
static volatile uint32_t g_u32LastSysFwWriteAddr = 0,
                         g_u32NewFwStartAddr     = 0,
                         g_u32NewFwByteSize      = 0;

static uint32_t g_au32RawData[12];              /* Decrypted Command data */
static uint32_t g_au32DecryptRawData[16];       /* Decrypted temp Command data */

int32_t cmd_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV);

//------------------------------------------------------------------------------------------------------------------

extern void SetECCRegisters(int32_t mode, uint32_t *pPriv, uint32_t *pPub);

extern int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t mode);
extern int32_t GenCmdSessionKey(uint32_t key[]);
extern int32_t _IsValidFlashRegion(uint32_t u32Addr, uint32_t u32ByteSize);

uint32_t Swap32(uint32_t val);
uint8_t EraseNewSysFwBlock(void);
uint8_t EraseNewAppFwBlock(void);
uint8_t OTA_WriteNewFW(uint32_t u32Address, uint8_t *pu8Buff, uint32_t u32Size);
int32_t NuBL2_CompareNuBL3xVer(uint32_t u32FwVer, int32_t i32Mode);
int32_t CheckNuBL3xWriteSpace(uint32_t u32FwSize, int32_t i32Mode);
int32_t OTA_GenRspPacket(CMD_PACKET_T *pCmd, uint16_t u16PacketID, ISP_INFO_T *pISPInfo, uint32_t u32Status);
int32_t ISP_InitInfo(ISP_INFO_T *pISPInfo);
int32_t MassWriteReqProcess(ISP_INFO_T *pISPInfo);
int32_t DisConnectReqProcess(void);
int32_t OTACmdReqProcess(ISP_INFO_T *pISPInfo);
//------------------------------------------------------------------------------------------------------------------

static ISP_INFO_T *s_psISPInfo = NULL;

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
int8_t OTA_TaskProcess(void)
{
    return OTA_API_TaskProcess();
}

uint32_t Swap32(uint32_t val)
{
    return (val << 24) | ((val << 8) & 0xff0000) | ((val >> 8) & 0xff00) | (val >> 24);
}

static void NuBL_BytesSwap(char *buf, int32_t len)
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
  * @brief Erase NuBL32 flash region
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Erase NuBL32 flash region
  */
/* TODO: erase size by FW INFO */
uint8_t EraseNewSysFwBlock(void)
{
    uint32_t i;
    uint32_t u32FlashPageSize;

    u32FlashPageSize = OTA_API_GetFlashPageSize();

    //printf("EraseNewSysFwBlock: u32FlashPageSize=0x%x(%d)\n", u32FlashPageSize, u32FlashPageSize);

    for (i = 0U; i < (uint32_t)SYS_NEW_FW_BLOCK_SIZE; i += u32FlashPageSize)
    {
        /* Erase page */
        if (OTA_API_EraseFlash((uint32_t)SYS_FW_BASE + i) != 0U)
        {
            printf("Erase fail(0x%x)\n", (uint32_t)SYS_FW_BASE + i);
            return STATUS_FAILED;
        }
    }

    printf("\nErase new system firmware block has done\n");

    return STATUS_SUCCESS;
}

/**
  * @brief Erase NuBL33 flash region
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Erase NuBL33 flash region
  */
uint8_t EraseNewAppFwBlock(void)
{
    uint32_t i;
    uint32_t u32FlashPageSize;

    u32FlashPageSize = OTA_API_GetFlashPageSize();

    //printf("EraseNewAppFwBlock: u32FlashPageSize=0x%x(%d)\n", u32FlashPageSize, u32FlashPageSize);

    for (i = 0U; i < APP_NEW_FW_BLOCK_SIZE; i += u32FlashPageSize)
    {
        // Erase page
        if (OTA_API_EraseFlash((uint32_t)APP_FW_BASE + i) != 0U)
        {
            printf("Erase fail(0x%x)\n", (uint32_t)APP_FW_BASE + i);
            return STATUS_FAILED;
        }
    }

    printf("\nErase new application firmware block has done\n");

    return STATUS_SUCCESS;
}

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_GetFwUpgradeDone(void)
{
    return g_u8IsFwUpgradeDone;
}


/**
  * @brief Write data to flash
  * @param[in]  u32Address    Flash address
  * @param[in]  pu8Buff       The pointer of data buffer
  * @param[in]  u32Size       Data size
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Write data to flash
  */
uint8_t OTA_WriteNewFW(uint32_t u32Address, uint8_t *pu8Buff, uint32_t u32Size)
{
    uint8_t u8Status = STATUS_FAILED;
    uint16_t u16Idx;

    DEBUG_MSG("OTA_WriteNewSysFW: addr 0x%x, size: %d\n", u32Address, u32Size);

    for (u16Idx = 0U; u16Idx < u32Size; u16Idx += 4U)
    {
        if (OTA_API_WriteFlash(u32Address, (uint32_t)((pu8Buff[u16Idx + 3] << 24) | (pu8Buff[u16Idx + 2] << 16) | (pu8Buff[u16Idx + 1] << 8) | (pu8Buff[u16Idx]))))
            u8Status = STATUS_FAILED;
        else
            u8Status = STATUS_SUCCESS;

        if (u8Status != STATUS_SUCCESS)
        {
            ERROR_MSG("OTA_API_WriteFlash write 0x%08X failed !\n", u32Address);
            break;
        }

        u32Address += 4U;
        g_u32LastSysFwWriteAddr = u32Address;
    }

    return u8Status;
}

/**
  * @brief Generate packet of response command and send out
  * @param[in]  pCmd           The pointer of response command
  * @param[in]  u16PacketID    The packet ID of response command
  * @param[in]  pISPInfo       The pointer of ISP Info
  * @param[in]  u32Status      Command process Status
  * @retval     0              Success
  * @retval     others         Failed
  * @details    Generate packet of response command include checksum calculation, and send it out.
  */
int32_t OTA_GenRspPacket(CMD_PACKET_T *pCmd, uint16_t u16PacketID, ISP_INFO_T *pISPInfo, uint32_t u32Status)
{
    CMD_PACKET_T        cmd;

    //memset(&cmd, 0x0, sizeof(CMD_PACKET_T));
    memcpy(&cmd, pCmd, sizeof(CMD_PACKET_T));

    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
    /* Fill command content by input parameters */
    cmd.u16PacketID = u16PacketID;
    cmd.au32Data[0] = u32Status;
    cmd.u16Len      = (4 * 1);
    DEBUG_MSG("OTA_GenRspPacket(PID:%d) Status: 0x%X\n", cmd.u16PacketID, u32Status);

    /* Checksum calculation for command packet */
    CMD_GenRspPacket(&cmd, pISPInfo);
    /* Copy packet to response buffer of ISP Info */
    memcpy(pISPInfo->rspbuf, &cmd, sizeof(CMD_PACKET_T));
    /* Send command packet */
    OTA_API_SendFrame((uint8_t *) pISPInfo->rspbuf, MAX_FRAME_SIZE);

    return 0;
}

/**
  * @brief Set public key of NuBL2
  * @param[in]  psPubKey   The pointer of public key
  * @return     None
  * @details    Set public key of NuBL2.
  */
static void SetPubKey(ECC_PUBKEY_T *psPubKey)
{
    extern const char gc_strOTA_PubKey0[], gc_strOTA_PubKey1[];

    Hex2Reg((char *)gc_strOTA_PubKey0, psPubKey->au32Key0);
    BytesSwap((char *)psPubKey->au32Key0, sizeof(psPubKey->au32Key0));

    Hex2Reg((char *)gc_strOTA_PubKey1, psPubKey->au32Key1);
    BytesSwap((char *)psPubKey->au32Key1, sizeof(psPubKey->au32Key1));
}

/**
  * @brief      Initial Secure ISP
  * @param      pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     -1          Failed
  * @details    This function initial the Secure ISP to update F/W.
  * @note       initial ClientPubKey and ServerPubKey is identity public key,
                but both key values will be changed to random public keys after ECDH key exchanged done.
  */
int32_t ISP_InitInfo(ISP_INFO_T *psISPInfo)
{
    /* Set the global ISP Info pointer */
    s_psISPInfo = psISPInfo;
    memset((void *)psISPInfo, 0x0, sizeof(ISP_INFO_T));

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_ENABLE_ISP();

    /* Enable all interrupt */
    __set_PRIMASK(0);
    /* Configure global SysInfo */
    SetPubKey(&psISPInfo->ClientPubKey);

    /* Generate IV for package encrypt */
    memset(psISPInfo->au32AESIV, 0x0, sizeof(psISPInfo->au32AESIV));
    psISPInfo->u32CmdMask = 0;

    return 0;
}

/**
  * @brief OTA process initialization
  * @param[in]  u32HSI      PLL Output Clock Frequency
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA process initialization
  */
__NONSECURE_ENTRY
int32_t OTA_Init(uint32_t u32HSI, ISP_INFO_T *psISPInfo)
{
    int32_t i32Ret;

    printf("OTA_Init\n");

    i32Ret = 0;
    SYS_UnlockReg();

    /* This global structure need to be initialized before re-connect, because client and server key was changed to random public key. */
    i32Ret = ISP_InitInfo(psISPInfo);

    if (i32Ret)
    {
        printf("OTA_Init: ISPInfo initial failed!\n");
        return i32Ret;
    }

    g_u32LastSysFwWriteAddr = NUBL_INVALID_ADDR;

    /* Init CyclesPerUs value for system tick */
    OTA_API_Init(u32HSI);

    return i32Ret;
}

/**
  * @brief OTA system tick interrupt process
  * @param[in]  u32Ticks    Tick value
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA system tick interrupt process
  */
__NONSECURE_ENTRY
uint8_t OTA_SysTickProcess(uint32_t u32Ticks)
{
    return OTA_API_SysTickProcess(u32Ticks);
}

/**
  * @brief OTA package process
  * @param      None
  * @return     None
  * @details    OTA package process
  */
__NONSECURE_ENTRY
void OTA_WiFiProcess(void)
{
    OTA_API_WiFiProcess();
}

/**
  * @brief      Get NuBL32 or NuBL33 F/W Version
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  u8Mode      F/W version of NuBL32 or NuBL33. bit-0: 0: NuBL32; 1: NuBL33  \n
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to get F/W version of NuBL32 or NuBL33. \n
  *             Flow: \n
  *                 1. Get NuBL3x info \n
  *                 2. Get NuBL3x F/W version (enclosed in F/W info) \n
  */
__NONSECURE_ENTRY
int32_t OTA_GetBLxFwVer(uint32_t *pu32FwVer, uint8_t u8Mode)
{
    volatile int32_t  ret = -1000;
    volatile uint32_t i;


    if (!((u8Mode == 0) || (u8Mode == 1)))
    {
        NUBL_MSG("\nGet NuBL3x Version FAIL. Invalid mode: 0x%x.\n\n", u8Mode);
        return ret;
    }

    NUBL_MSG("\nGet NuBL3%d. \n\n", ((u8Mode & BIT0) == 0) ? 2 : 3);

    return ret;
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
int32_t cmd_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    uint32_t u32TimeOutCnt;

    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* KEY and IV are byte order (32 bit) reversed, Swap32(x)) and stored in ISP_INFO_T */
    memcpy((void *)((uint32_t)&CRYPTO->AES_KEY[0]), KEY, (4 * 8));
    memcpy((void *)((uint32_t)&CRYPTO->AES_IV[0]), IV, (4 * 4));

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

/**
  * @brief      CMD_MASS_WRITE command process
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    CMD_MASS_WRITE command process
  */
int32_t MassWriteReqProcess(ISP_INFO_T *pISPInfo)
{
    int32_t i32Ret = 0;
    CMD_PACKET_T cmd;
    uint32_t u32RecvPackageSize;

    memset(&cmd, 0x0, sizeof(CMD_PACKET_T));
    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    DEBUG_MSG("[%s] PID:%d\n", __func__, cmd.u16PacketID);

    u32RecvPackageSize = (cmd.u16PacketID * 48); // maximum data length is 48
    NVT_UNUSED(u32RecvPackageSize);

    memcpy((uint8_t *)&g_au32RawData, cmd.au32Data, cmd.u16Len);
    memcpy((uint8_t *)&g_au32DecryptRawData + 16, cmd.au32Data, cmd.u16Len);

    if (g_u32LastSysFwWriteAddr == NUBL_INVALID_ADDR)
    {
        ERROR_MSG("[%s] Write new firmware to invalid flash address !\n", __func__);
        /* Generate response command and send it out */
        OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

        i32Ret = -1;
        goto _DisConn;
    }

    /* Check last write address is over FW1 start + FW1 size */
    // Because OTA Server APP sent data with 48 bytes alignment, fw image might not alignment with 48 bytes.
    // Check fw write start address instead of fw write start address + cmd.u16Len
    //if (((g_u32LastSysFwWriteAddr & ~NS_OFFSET) + cmd.u16Len) > (g_u32NewFwStartAddr + g_u32NewFwByteSize))
    if ((g_u32LastSysFwWriteAddr & ~NS_OFFSET) > (g_u32NewFwStartAddr + g_u32NewFwByteSize))
    {
        ERROR_MSG("Write new firmware to flash 0x%08X failed (Out of range) !\n", g_u32LastSysFwWriteAddr);
        /* Generate response command and send it out */
        OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

        i32Ret = -2;
        goto _DisConn;
    }
    else
    {
        /* Write new firmware data to flash */
        if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&g_au32RawData, cmd.u16Len) != STATUS_SUCCESS)
        {
            ERROR_MSG("Write new firmware to flash 0x%08X failed !\n", g_u32LastSysFwWriteAddr);
            /* Generate response command and send it out */
            OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

            i32Ret = -3;
            goto _DisConn;
        }
    }

    if (g_u32LastSysFwWriteAddr >= (g_u32NewFwStartAddr + g_u32NewFwByteSize))
    {
        g_u8IsNuBL3xFwDone = 1;
        // Forced to write last buffer to Flash
        OTA_API_WriteFlash(FORCE_WRITE_TO_FLASH, FORCE_WRITE_TO_FLASH);
    }

    /* Generate response command and send it out */
    OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, STS_OK);

    return 0;

_DisConn:
    /* NuBL32 or NuBL33 has update done. */

    /* Disconnect local Wi-Fi connection */
    OTA_API_TransferConnClose();

    return i32Ret;
}

/**
  * @brief      CMD_DISCONNECT command process
  * @param      None
  * @retval     0           Success
  * @retval     others      Failed
  * @details    CMD_DISCONNECT command process
  */
int32_t DisConnectReqProcess(void)
{
    uint32_t u32IsUpgradeDone;
    int32_t i32Ret;

    u32IsUpgradeDone = g_u32SetFwUpgradeDone;

    /* Disconnect local Wi-Fi connection */
    OTA_API_TransferConnClose();

    /* This global structure need to be initialized before re-connect, because client and server key was changed to random public key. */
    i32Ret = ISP_InitInfo(s_psISPInfo);

    if (u32IsUpgradeDone)
    {
        /* Set Reset flag for transfer task */
        OTA_API_SetResetFlag();
        printf("g_u8IsFwUpgradeDone\n");
        g_u8IsFwUpgradeDone = TRUE;
    }

    return i32Ret;
}

/**
  * @brief      OTA commands process
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA commands process
  */
int32_t OTACmdReqProcess(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i, u32Ret = 0;
    CMD_PACKET_T        cmd;

    memset(&cmd,             0x0, sizeof(CMD_PACKET_T));
    memset(pISPInfo->rspbuf, 0x0, sizeof(pISPInfo->rspbuf));
    /* Copy received content of command from receive buffer of ISP Info  */
    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    /* Parse received command packet, including decrypted data. */
    if (CMD_ParseReqPacket(&cmd, pISPInfo) != 0)
    {
        DEBUG_MSG("*** [Parse error: 0x%x] ***\n", cmd.u16CmdID);
        memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

        /* Response checksum error */
        cmd.au32Data[0] = ERR_CMD_CHECKSUM;
        cmd.u16Len      = (4 * 1);
        u32Ret = -1;
    }
    else
    {
        switch (cmd.u16CmdID)
        {
            case CMD_ECDH_PUB0:
                /* Get Server public key 0 */
                memcpy(pISPInfo->ServerPubKey.au32Key0, cmd.au32Data, cmd.u16Len);

                for (i = 0; i < 8; i++)
                    NUBL_MSG("Get pub0[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key0[i]);

                /* Response status ok */
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
                break;

            case CMD_ECDH_PUB1:
                /* Get Server public key 1 and generate ist ECDH AES key */
                memcpy(pISPInfo->ServerPubKey.au32Key1, cmd.au32Data, cmd.u16Len);

                for (i = 0; i < 8; i++)
                    NUBL_MSG("Get pub1[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key1[i]);

                /* Identify Host public key */
                if (((uint32_t)IdentifyPublicKey((uint32_t *)pISPInfo->ServerPubKey.au32Key0, 0)&BIT2) != BIT2)
                {
                    /* Response key authentication error */
                    cmd.au32Data[0] = ERR_AUTH_KEY;
                    cmd.u16Len      = (4 * 1);
                    u32Ret = -1;
                    break;
                }

                /* Response status ok */
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
                break;

            case CMD_AUTH_KEY:
                /* Compare NuBL3x pubKey from host */
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                /* verify pass */
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
                break;

            case CMD_MASS_WRITE:
                memcpy(pISPInfo->rcvbuf, &cmd, sizeof(cmd));
                MassWriteReqProcess(pISPInfo);
                return 0;

            //break;
            case CMD_DISCONNECT:
                /* get F/W upgrade done inform and keep to global structure. */
                g_u32SetFwUpgradeDone = cmd.au32Data[0];

                /* Response status ok */
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
                break;

            default:
                printf("Invalid command: 0x%x\n", cmd.u16CmdID);
                break;
        }
    }

    if (u32Ret == -1)
        pISPInfo->IsConnectOK = 0;

    /* Generate response command packet */
    CMD_GenRspPacket(&cmd, pISPInfo);

    /* Copy response command to response buffer of ISP Info */
    memcpy(pISPInfo->rspbuf, (uint8_t *)&cmd, sizeof(CMD_PACKET_T));

    return u32Ret;
}

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
void OTA_CallBackHandler(uint8_t *pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen)
{
    volatile int32_t i32Ret = 0;
    uint32_t u32Addr;

    NVT_UNUSED(u32StartIdx);
    NVT_UNUSED(u32ValidLen);
    NVT_UNUSED(u32Addr);

    /* Copy received packet to receive buffer of ISP Info */
    memcpy(s_psISPInfo->rcvbuf, (uint32_t *)((uint32_t)pu8Buff), u32Len);

    DEBUG_MSG("OTA_CallBackHandler(0x%x)\n", ((CMD_PACKET_T *)s_psISPInfo->rcvbuf)->u16CmdID);

    switch (((CMD_PACKET_T *)s_psISPInfo->rcvbuf)->u16CmdID)
    {
        //NuBL1 libary commands process ---------------------------------------------------------------
        case CMD_CONNECT:
            DEBUG_MSG("CMD_CONNECT_REQ\n");
            i32Ret = ParseCONNECT(s_psISPInfo);

            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) s_psISPInfo->rspbuf, MAX_FRAME_SIZE);
            break;

        case CMD_ECDH_GET_RAND_PUB0:
        case CMD_ECDH_GET_RAND_PUB1:
            ECC_Complete(CRYPTO);
            /* Enable CRYPTO clock */
            CLK_EnableModuleClock(CRYPTO0_MODULE);
            ECC_ENABLE_INT(CRYPTO);
            NVIC_EnableIRQ(CRYPTO_IRQn);

        case CMD_ECDH_GET_PUB0:
        case CMD_ECDH_GET_PUB1:
        case CMD_ECDH_RAND_PUB0:
        case CMD_ECDH_RAND_PUB1:

            i32Ret = ParseECDH(s_psISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) s_psISPInfo->rspbuf, MAX_FRAME_SIZE);
            break;

        case CMD_SET_MASS_WRITE:
        case CMD_RESET: //need ??(CHIP reset?System reset?CPU reset)
            i32Ret = ParseCommands(s_psISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) s_psISPInfo->rspbuf, MAX_FRAME_SIZE);

            if (i32Ret == CMD_SET_MASS_WRITE)
            {
                g_u32NewFwStartAddr = (s_psISPInfo->tmp0[0] & ~NS_OFFSET);
                g_u32NewFwByteSize  = (s_psISPInfo->tmp0[1] + (NUBL_SIZE_ALIGN - 1)) & ~(NUBL_SIZE_ALIGN - 1);
                NUBL_MSG("g_u32NewFwStartAddr: 0x%08X, g_u32NewFwByteSize: 0x%08X\n", g_u32NewFwStartAddr, g_u32NewFwByteSize);

                if (((g_u32NewFwStartAddr >= NUBL32_FW_BASE) && ((g_u32NewFwStartAddr + g_u32NewFwByteSize) < (NUBL32_FW_BASE + NUBL32_FW_SIZE))) ||
                        ((g_u32NewFwStartAddr >= (NUBL33_FW_BASE & ~NS_OFFSET)) && ((g_u32NewFwStartAddr + g_u32NewFwByteSize) < ((NUBL33_FW_BASE & ~NS_OFFSET) + NUBL33_FW_SIZE))))
                {
                    for (u32Addr = g_u32NewFwStartAddr; u32Addr < (g_u32NewFwStartAddr + g_u32NewFwByteSize); u32Addr += SPI_FLASH_PAGE_SIZE)
                        OTA_API_EraseFlash(u32Addr);

                    printf("New FW addr: 0x%08X, size: %d (%d)\n", g_u32NewFwStartAddr, g_u32NewFwByteSize, s_psISPInfo->tmp0[1]);
                    g_u32LastSysFwWriteAddr = g_u32NewFwStartAddr;
                }
                else
                {
                    g_u32LastSysFwWriteAddr = NUBL_INVALID_ADDR;
                    ERROR_MSG("Invalid firmware address 0x%08X !\n", g_u32NewFwStartAddr);
                }
            }

            break;

        //OTA customized process ----------------------------------------------------------------------
        case CMD_ECDH_PUB0:
        {
            uint32_t AESKey[8], i;

            /* Generate 1st ECDH AES key */
            /* Calculate (NuBL2 priv * Host pub) ECDH key. because NuBL2 has known the public key of host. */
            if (GenCmdSessionKey(AESKey) != 0)
            {
                /* Clear key buffer */
                memset(&AESKey, 0x0, sizeof(AESKey));
                return ;
            }

            for (i = 0; i < 8; i++)
                AESKey[i] = Swap32(AESKey[i]);

            NuBL_BytesSwap((char *)&AESKey, sizeof(AESKey));

            /* Copy session key to ISP Info */
            memcpy((void *)s_psISPInfo->au32AESKey, AESKey, sizeof(AESKey));

            for (i = 0; i < 8; i++)
                NUBL_MSG("Gen 1st KEY[%d]: 0x%08x.\n", i, s_psISPInfo->au32AESKey[i]);
        }

        case CMD_ECDH_PUB1:
        case CMD_AUTH_KEY:
        case CMD_MASS_WRITE:
        case CMD_DISCONNECT:
            /* OTA commands process */
            OTACmdReqProcess(s_psISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) s_psISPInfo->rspbuf, MAX_FRAME_SIZE);

            /* Disconnect Wi-Fi connection */
            if (((CMD_PACKET_T *)s_psISPInfo->rcvbuf)->u16CmdID == CMD_DISCONNECT)
                DisConnectReqProcess();

            break;

        default:
            printf("Invalid command: 0x%x\n", ((CMD_PACKET_T *)s_psISPInfo->rcvbuf)->u16CmdID);
            break;
    }
}


/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
