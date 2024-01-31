/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of interrupt control using CAN bus communication.
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
volatile uint8_t   g_u8RxFifO0CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len);
void CAN_ShowRecvMessage(void);
void CAN_RxTest(void);
void CAN_TxTest(void);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                           */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
    /*Recieve the Rx Fifo0 message */
    CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
    g_u8RxFifO0CompleteFlag = 1;
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

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PJ multi-function pins for CAN RXD and TXD */
    SET_CAN0_RXD_PJ11();
    SET_CAN0_TXD_PJ10();
}


/*---------------------------------------------------------------------------------------------------------*/
/*              CAN Function Test Menu(Master)                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TestItem(void)
{
    printf("\n");
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
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Function Tx Test                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CAN_TestItem();
        u8Item = getchar();

        switch (u8Item)
        {
            case '1':
                /*Standard ID =0x111,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
                break;

            case '2':
                /*Standard ID =0x22F,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
                break;

            case '3':
                /*Standard ID =0x333,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
                break;

            case '4':
                /*Extend ID =0x111,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
                break;

            case '5':
                /*Extend ID =0x3333,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
                break;

            case '6':
                /*Extend ID =0x44444,Data lenght 8 bytes*/
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
                break;

            default:
                break;
        }

    } while (u8Item != 27);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Send Message Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;

    psTxMsg->u32Id = u32Id;
    psTxMsg->eIdType = eIdType;
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC = u8Len;

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFifO0CompleteFlag = 0;

    /* use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN CAN Function Rx Test                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_RxTest(void)
{
    uint8_t u8Cnt = 0;
    printf("Start CAN bus reception :\n");

    do
    {
        while (!g_u8RxFifO0CompleteFlag) {}

        CAN_ShowRecvMessage();
        g_u8RxFifO0CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    } while (u8Cnt < 6);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN Receive Message Function                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ShowRecvMessage(void)
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
/*                                    Init CAN                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PJ10)                        CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PJ11)                        CAN_RXD(Any board)     |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CAN     |  CAN_H  |    CAN    |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");
    /*Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /*Get the CAN configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;

    /*Open the CAN feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110~0x11F in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* receive 0x220 ~ 0x22F in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* receive 0x220 (29-bit id) in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x4444 (29-bit id) in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX fifo0)*/
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
    /* Enable CANFD0 IRQ00 Handler*/
    NVIC_EnableIRQ(CANFD00_IRQn);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Tx Rx Interrupt Function Test                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN interface initialization*/
    CAN_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                      CAN Function Test                                   |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and  |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with    |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if      |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                  |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        CAN_TxTest();
    }
    else
    {
        CAN_RxTest();
    }

    printf("CANFD Sample Code End.\n");
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
    printf("\nCAN bus communication example\r\n");

    CAN_TxRxINTTest();

    while (1) {}
}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
