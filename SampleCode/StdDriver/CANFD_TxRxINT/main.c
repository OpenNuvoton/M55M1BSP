/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN / CAN FD bus communication.
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
volatile uint8_t    g_u8BusOffFlag = 0;
volatile uint32_t   g_u32BusOffRecoveryCounter = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_ShowRecvMessage(void);
void CANFD_RxTest(void);
void CANFD_TxTest(void);
void CANFD_TxRxINTTest(void);
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenParam);
uint8_t CANFD_BusOffRecovery(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD0 Line0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    uint32_t u32IntStatus;

    u32IntStatus = CANFD0->IR;
    printf("IR =0x%08X \n", u32IntStatus);

    /* Check Error Warning status */
    if (u32IntStatus & CANFD_IR_EW_Msk)
    {
        printf("Error warning flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EW_Msk);
    }

    /* Check Error Passive status */
    if (u32IntStatus & CANFD_IR_EP_Msk)
    {
        printf("Error passive flag is set.\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EP_Msk);
    }

    /* Check Bus-Off status */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected! Recovery will be triggered on next transmission attempt.\n");
            g_u8BusOffFlag = 1;
        }

        /* Always clear Bus-Off interrupt flag to prevent repeated interrupts */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
    }

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

    /* Check Rx FIFO1 New Message interrupt (CAN FD mode) */
    if (u32IntStatus & CANFD_IR_RF1N_Msk)
    {
        /* Receive the Rx FIFO1 buffer */
        CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
        g_u8RxCompleteFlag = 1;
        /* Clear Rx FIFO1 New Message interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }

#else

    /* Check Rx FIFO0 New Message interrupt (CAN mode) */
    if (u32IntStatus & CANFD_IR_RF0N_Msk)
    {
        /* Receive the Rx FIFO0 buffer */
        CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
        g_u8RxCompleteFlag = 1;
        /* Clear Rx FIFO0 New Message interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }

    /* Clear Timeout Occurred flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk);
#endif /* OP_MODE == CANFD_OP_CAN_FD_MODE */
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

    /* Set PJ multi-function pins for CANFD RXD and TXD */
    SET_CANFD0_RXD_PJ11();
    SET_CANFD0_TXD_PJ10();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function Test Menu (Master)                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TestItem(void)
{
    printf("\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("+-----------------------------------------------------------+\n");
    printf("|             CAN FD Tx Function Test (Master)              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111           ( Data lenght  8 bytes )|\n");
    printf("| [2] Standard ID = 0x113           ( Data lenght 12 bytes )|\n");
    printf("| [3] Standard ID = 0x22F           ( Data lenght 16 bytes )|\n");
    printf("| [4] Standard ID = 0x333           ( Data lenght 20 bytes )|\n");
    printf("| [5] Extended ID = 0x221           ( Data lenght 24 bytes )|\n");
    printf("| [6] Extended ID = 0x227           ( Data lenght 32 bytes )|\n");
    printf("| [7] Extended ID = 0x3333          ( Data lenght 48 bytes )|\n");
    printf("| [8] Extended ID = 0x44444         ( Data lenght 64 bytes )|\n");
    printf("| Select ID number and master will send message to slave ...|\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
#else
    printf("+-----------------------------------------------------------+\n");
    printf("|              CAN Tx Function Test (Master)                |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111    (Date Lenght 8bytes)           |\n");
    printf("| [2] Standard ID = 0x22F    (Date Lenght 8bytes)           |\n");
    printf("| [3] Standard ID = 0x333    (Date Lenght 8bytes)           |\n");
    printf("| [4] Extended ID = 0x221    (Date Lenght 8bytes)           |\n");
    printf("| [5] Extended ID = 0x3333   (Date Lenght 8bytes)           |\n");
    printf("| [6] Extended ID = 0x44444  (Date Lenght 8bytes)           |\n");
    printf("| Select ID number and master will send message to slave ...|\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function Tx Test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CANFD_TestItem();
        u8Item = getchar();

        switch (u8Item)
        {
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)

            case '1':
                /* Standard ID = 0x111, Data length 8 bytes (index 0) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 0);
                break;

            case '2':
                /* Standard ID = 0x113, Data length 12 bytes (index 1) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x113, 1);
                break;

            case '3':
                /* Standard ID = 0x22F, Data length 16 bytes (index 2) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 2);
                break;

            case '4':
                /* Standard ID = 0x333, Data length 20 bytes (index 3) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 3);
                break;

            case '5':
                /* Extended ID = 0x221, Data length 24 bytes (index 4) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 4);
                break;

            case '6':
                /* Extended ID = 0x227, Data length 32 bytes (index 5) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x227, 5);
                break;

            case '7':
                /* Extended ID = 0x3333, Data length 48 bytes (index 6) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 6);
                break;

            case '8':
                /* Extended ID = 0x44444, Data length 64 bytes (index 7) */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);
                break;
#else

            case '1':
                /* Standard ID = 0x111, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
                break;

            case '2':
                /* Standard ID = 0x22F, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
                break;

            case '3':
                /* Standard ID = 0x333, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
                break;

            case '4':
                /* Extended ID = 0x221, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
                break;

            case '5':
                /* Extended ID = 0x3333, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
                break;

            case '6':
                /* Extended ID = 0x44444, Data length 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
                break;
#endif /* OP_MODE == CANFD_OP_CAN_FD_MODE */

            default:
                break;
        }
    } while (u8Item != 27);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Send Message Function                                                                                   */
/*  u8LenParam: CAN FD mode - DLC index (0~7 maps to 8/12/16/20/24/32/48/64 bytes)                        */
/*              CAN mode    - DLC value directly (bytes)                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenParam)
{
    uint8_t u8Cnt;

    /* Check if bus is in Bus-Off state before transmitting */
    if (g_u8BusOffFlag)
    {
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
        printf("CAN FD is in Bus-Off state. Starting recovery process...\n");
#else
        printf("CAN is in Bus-Off state. Starting recovery process...\n");
#endif

        if (CANFD_BusOffRecovery())
        {
            printf("Bus-Off recovery successful. Proceeding with transmission.\n");
        }
        else
        {
            printf("Bus-Off recovery failed. Cannot transmit.\n");
            return;
        }
    }

    /* Set the ID Number and ID Type -- common to both modes */
    psTxMsg->u32Id   = u32Id;
    psTxMsg->eIdType = eIdType;

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

    /* Use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    printf("Data Message(%02u bytes) : ", psTxMsg->u32DLC);

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++)
    {
        psTxMsg->au8Data[u8Cnt] = u8Cnt;
        printf("%02u,", psTxMsg->au8Data[u8Cnt]);
    }

    printf("\n");

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function Rx Test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    uint8_t u8Cnt = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("Start CAN FD bus reception :\n");
#else
    printf("Start CAN bus reception :\n");
#endif

    do
    {
        while (!g_u8RxCompleteFlag) {}

        CANFD_ShowRecvMessage();
        g_u8RxCompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    } while (u8Cnt < 8);   /* CAN FD mode: receive 8 messages */

#else
    }

    while (u8Cnt < 6);   /* CAN mode: receive 6 messages */

#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Show Received Message Function                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02u bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt < g_sRxMsgFrame.u32DLC; u8Cnt++)
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);

    printf("\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Bus-Off Recovery Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_BusOffRecovery(void)
{
    printf("Starting Bus-Off recovery sequence...\n");

    /* Run to initial mode */
    CANFD_RunToNormal(CANFD0, FALSE);

    /* Cancel all transmit requests */
    CANFD0->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flags */
    CANFD_ClearStatusFlag(CANFD0, 0xFFFFFFFF);

    /* Run to normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);

    /* 50ms delay after recovery process */
    CLK_SysTickDelay(50000);

    /* Check if recovery was successful by verifying Bus-Off status */
    if (CANFD0->PSR & CANFD_PSR_BO_Msk)
    {
        /* Still in Bus-Off state, recovery failed */
        printf("Bus-Off recovery failed. Still in Bus-Off state.\n");
        return 0;
    }
    else
    {
        /* Recovery successful, clear Bus-Off flag */
        g_u8BusOffFlag = 0;
        g_u32BusOffRecoveryCounter++;
        printf("Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
        return 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Init CAN / CAN FD                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PJ10)                        CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PJ11)                        CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|  CAN0_TX |   CAN     |  CAN_H  |    CAN    |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|  CAN0_RX |           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /* Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
#else
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
    printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
    printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* receive 0x110~0x11F in CAN FD0 rx FIFO1 buffer by setting index 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    /* receive 0x22F in CAN FD0 rx FIFO1 buffer by setting index 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN FD0 rx FIFO1 buffer by setting index 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));
    /* receive 0x220~0x22F (29-bit id) in CAN FD0 rx FIFO1 buffer by setting index 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx FIFO1 buffer by setting index 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx FIFO1 buffer by setting index 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter (RX FIFO1) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX FIFO1 new message, Bus-Off, Error Warning and Error Passive interrupts */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF1NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#else
    /* receive 0x110~0x11F in CAN rx FIFO0 buffer by setting index 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* receive 0x22F in CAN rx FIFO0 buffer by setting index 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN rx FIFO0 buffer by setting index 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));
    /* receive 0x220~0x22F (29-bit id) in CAN rx FIFO0 buffer by setting index 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220),   CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN rx FIFO0 buffer by setting index 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333),  CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN rx FIFO0 buffer by setting index 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter (RX FIFO0) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX FIFO0 new message, Bus-Off, Error Warning and Error Passive interrupts */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_BOE_Msk | CANFD_IE_EWE_Msk | CANFD_IE_EPE_Msk), 0, 0, 0);
#endif /* OP_MODE == CANFD_OP_CAN_FD_MODE */

    /* Enable CANFD0 IRQ00 Handler */
    NVIC_EnableIRQ(CANFD00_IRQn);
    /* CAN FD0 Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CAN / CAN FD Tx Rx Interrupt Function Test                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN / CAN FD interface initialization */
    CANFD_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                              Function Test                                 |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(transmitter) and        |\n");
    printf("|    the other is slave(receiver). Master will send 8 messages with          |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 8 messages data.                 |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(transmitter)    [1] Slave(receiver)                            |\n");
    printf("+----------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        CANFD_TxTest();
    }
    else
    {
        CANFD_RxTest();
    }

    printf("Sample Code End.\n");

    if (g_u32BusOffRecoveryCounter > 0)
    {
        printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
    }
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
    printf("\n CAN FD mode transmission test\r\n");
#else
    printf("\n CAN bus communication example\r\n");
#endif

    CANFD_TxRxINTTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
