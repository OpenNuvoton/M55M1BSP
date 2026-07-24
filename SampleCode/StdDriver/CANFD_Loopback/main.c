/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CAN / CAN FD mode function to do internal loopback test.
 *
 *           Set OP_MODE to CANFD_OP_CAN_FD_MODE to enable CAN FD mode,
 *           or CANFD_OP_CAN_MODE to use classic CAN mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Mode selection: set OP_MODE to select operating mode                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define OP_MODE  CANFD_OP_CAN_FD_MODE  /* Options: CANFD_OP_CAN_FD_MODE  or  CANFD_OP_CAN_MODE */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t    g_u8RxCompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD0_TEST_HANDLE(void);
NVT_ITCM void CANFD00_IRQHandler(void);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    CANFD0_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN FD0 Callback function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD0_TEST_HANDLE(void)
{
    uint32_t u32IR = 0;

    u32IR = CANFD0->IR;

    printf("IR =0x%08X \n", u32IR);

    if (u32IR & CANFD_IR_RF1N_Msk)
    {
        printf("Received message in FIFO1 (CAN FD mode)\n");
        /* CAN FD mode: clear FIFO1 new message flag, read from FIFO1 */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
    }

    if (u32IR & CANFD_IR_RF0N_Msk)
    {
        printf("Received message in FIFO0 (CAN mode)\n");
        /* CAN mode: clear FIFO0 new message flag, read from FIFO0 */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
    }

    if (u32IR & CANFD_IR_TOO_Msk)
    {
        printf("Timeout Occurred\n");
        /* Clear Timeout Occurred flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk);
    }

    g_u8RxCompleteFlag = 1;
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

    /* Switch SCLK clock source to APLL0 and Enable APLL0 160MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_160MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select CAN FD0 clock source is APLL0/2 */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CANFDSEL_CANFD0SEL_APLL0_DIV2, CLK_CANFDDIV_CANFD0DIV(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /* Debug UART clock setting */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}

/*---------------------------------------------------------------------------------------------------------*/
/* Tx / Rx Test                                                                                            */
/*  u8LenParam: CAN FD mode - DLC index (0~7 maps to 8/12/16/20/24/32/48/64 bytes)                        */
/*              CAN mode    - DLC value directly (bytes)                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRx_Test(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eFrameIdType, uint32_t u32Id, uint8_t u8LenParam)
{
    uint8_t u8Cnt;

    /* Set the ID Number and ID Type -- common to both modes */
    psTxMsg->u32Id   = u32Id;
    psTxMsg->eIdType = eFrameIdType;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD mode: enable FD format and bitrate switching, map index to DLC */
    psTxMsg->bFDFormat      = 1;
    psTxMsg->bBitRateSwitch = 1;

    if (u8LenParam == 0) psTxMsg->u32DLC = 8;
    else if (u8LenParam == 1) psTxMsg->u32DLC = 12;
    else if (u8LenParam == 2) psTxMsg->u32DLC = 16;
    else if (u8LenParam == 3) psTxMsg->u32DLC = 20;
    else if (u8LenParam == 4) psTxMsg->u32DLC = 24;
    else if (u8LenParam == 5) psTxMsg->u32DLC = 32;
    else if (u8LenParam == 6) psTxMsg->u32DLC = 48;
    else if (u8LenParam == 7) psTxMsg->u32DLC = 64;

#else
    /* CAN mode: classic data frame, u8LenParam is the actual byte count */
    psTxMsg->eFrmType       = eCANFD_DATA_FRM;
    psTxMsg->bFDFormat      = 0;
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC         = u8LenParam;
#endif

    /* Fill payload */
    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxCompleteFlag = 0;

    /* Transmit */
    if (eFrameIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }

    /* Wait for reception complete */
    while (!g_u8RxCompleteFlag) {}

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Rx FIFO1 : Received message 0x%08X\n", g_sRxMsgFrame.u32Id);
#else
    printf("Rx FIFO0 : Received message 0x%08X\n", g_sRxMsgFrame.u32Id);
#endif

    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt < g_sRxMsgFrame.u32DLC; u8Cnt++)
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);

    printf("\n\n");

    memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
}

/*---------------------------------------------------------------------------------------------------------*/
/*                CAN / CAN FD Loopback Function Test                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Loopback(void)
{
    uint8_t u8Loop;
    uint8_t u8LoopStart;
    CANFD_FD_T sCANFD_Config;

    /* Common config initialisation */
    sCANFD_Config.sElemSize.u32UserDef = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD mode: FD operation, data bitrate 4 Mbps */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;
#else
    /* CAN mode: classic CAN operation, no data phase bitrate */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    /* Common: enable loopback, set nominal bitrate, open peripheral */
    sCANFD_Config.sBtConfig.bEnableLoopBack         = TRUE;
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
    printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
    printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

    /* Set SID / XID receive filters -- FIFO1 for CAN FD, FIFO0 for CAN */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
#else
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
#endif

    /* Reject non-matching frames -- common */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    /* Enable new-message interrupt for the appropriate FIFO */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF1NE_Msk), 0, 0, 0);
#else
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
#endif

    /* Common: enable IRQ and run */
    NVIC_EnableIRQ(CANFD00_IRQn);
    CANFD_RunToNormal(CANFD0, TRUE);

    /*
     * CAN FD mode starts from index 0 (DLC=8) to exercise all FD payload sizes.
     * CAN mode starts from index 1 (DLC=1) because index 0 would mean 0 bytes in CAN.
     */
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    u8LoopStart = 0;
#else
    u8LoopStart = 1;
#endif

    for (u8Loop = u8LoopStart; u8Loop < 8; u8Loop++)
        CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x110 + u8Loop, u8Loop);

    CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
    CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);

    for (u8Loop = u8LoopStart; u8Loop < 8; u8Loop++)
        CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x220 + u8Loop, u8Loop);

    CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x3333,  8);
    CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("\n CAN FD Mode Loopback example\r\n");
#else
    printf("\n CAN Mode Loopback example\r\n");
#endif

    /* CAN / CAN FD Loopback Test */
    CANFD_Loopback();

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("\n CAN FD Mode Loopback Test Done\r\n");
#else
    printf("\n CAN Mode Loopback Test Done\r\n");
#endif

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
