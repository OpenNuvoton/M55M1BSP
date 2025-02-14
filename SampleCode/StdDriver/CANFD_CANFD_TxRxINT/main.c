/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN FD bus communication.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFifo1CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_ShowRecvMessage(void);
void CANFD_RxTest(void);
void CANFD_TxTest(void);
void CANFD_TxRxINTTest(void);
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /* Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF1N_Msk);
    /* Receive the Rx Fifo1 buffer */
    CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxMsgFrame);
    g_u8RxFifo1CompleteFlag = 1;
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
/*        CAN FD Function Test Menu(Master)                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TestItem(void)
{
    printf("\n");
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
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN FD Function Tx Test                                                     */
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
            case '1':
                /* Standard ID =0x111,Data lenght 8 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 0);
                break;

            case '2':
                /* Standard ID =0x113,Data lenght 12 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x113, 1);
                break;

            case '3':
                /* Standard ID =0x22F,Data lenght 16 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 2);
                break;

            case '4':
                /* Standard ID =0x333,Data lenght 20 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 3);
                break;

            case '5':
                /* Extend ID =0x221,Data lenght 24 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 4);
                break;

            case '6':
                /* Extend ID =0x227,Data lenght 32 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x227, 5);
                break;

            case '7':
                /* Extend ID =0x3333,Data lenght 48 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 6);
                break;

            case '8':
                /* Extend ID =0x44444,Data lenght 64 bytes */
                CANFD_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);
                break;

            default:
                break;

        }
    } while (u8Item != 27);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                 Send CAN FD Message Function                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8LenType)
{
    uint8_t u8Cnt;

    /* Set the ID Number */
    psTxMsg->u32Id = u32Id;
    /* Set the frame type */
    psTxMsg->eIdType = eIdType;
    /* Set FD frame format attribute */
    psTxMsg->bFDFormat = 1;
    /* Set the bitrate switch attribute */
    psTxMsg->bBitRateSwitch = 1;

    /* Set data length */
    if (u8LenType == 0)      psTxMsg->u32DLC = 8;
    else if (u8LenType == 1) psTxMsg->u32DLC = 12;
    else if (u8LenType == 2) psTxMsg->u32DLC = 16;
    else if (u8LenType == 3) psTxMsg->u32DLC = 20;
    else if (u8LenType == 4) psTxMsg->u32DLC = 24;
    else if (u8LenType == 5) psTxMsg->u32DLC = 32;
    else if (u8LenType == 6) psTxMsg->u32DLC = 48;
    else if (u8LenType == 7) psTxMsg->u32DLC = 64;

    g_u8RxFifo1CompleteFlag = 0;

    /* use message buffer 0 */
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
/*                                   CAN FD Function Rx Test                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_RxTest(void)
{
    uint8_t u8Cnt = 0;

    printf("Start CAN FD bus reception :\n");

    do
    {
        while (!g_u8RxFifo1CompleteFlag) {}

        CANFD_ShowRecvMessage();
        g_u8RxFifo1CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    } while (u8Cnt < 8);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             Show the CAN FD Message Function                                            */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if (g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO1(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO1(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02u bytes) : ", g_sRxMsgFrame.u32DLC);

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                     Init CAN FD0                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|    CAN0_TXD(PJ10)                        CAN_TXD(Any board)   |\n");
    printf("|    CAN0_RXD(PJ11)                        CAN_RXD(Any board)   |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CANFD   | CANFD_H |    CANFD  |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           | CANFD_L |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /* Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /* Get the CAN FD configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
    /* Open the CAN FD feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* Note: About TLE9351VSJ High speed CAN FD transceiver */
    /* Due to TX/RX propagation delay, user should adjust the 'Transmitter Delay Compensation' setting */
    //CANFD0->DBTP |= CANFD_DBTP_TDC_Msk;
    //CANFD0->TDCR = 0x300;

    /* receive 0x110~0x11F in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    /* receive 0x220 in CAN FD0 rx fifo1 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN FD0 rx fifo1 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));

    /* receive 0x220~0x22f (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX fifo1) */
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX fifo1 new message interrupt using interrupt line 0 */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF1NE_Msk), 0, 0, 0);
    /* Enable CANFD0 IRQ00 Handler */
    NVIC_EnableIRQ(CANFD00_IRQn);
    /* CAN FD0 Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN FD Tx Rx Interrupt Function Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN FD interface initialization */
    CANFD_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                     CAN FD Function Test                                   |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter) and |\n");
    printf("|    the other is slave(CAN FD receiver). Master will send 8 messages with   |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 8 messages data.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)              |\n");
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

    printf("CAN FD Sample Code End.\n");
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

    /* print a note to terminal */
    printf("\n CAN FD mode transmission test\r\n");

    CANFD_TxRxINTTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
