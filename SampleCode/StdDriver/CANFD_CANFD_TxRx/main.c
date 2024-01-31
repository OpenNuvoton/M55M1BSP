/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive CAN FD messages through CAN interface.
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

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_TxRxTest(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif


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

    /* Select CAN FD0 clock source is APPL0/2 */
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
/*                                    Init CAN FD0                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PJ10)                        CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PJ11)                        CAN_RXD(Any board      |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CANFD   |  CAN_H  |   CANFD   |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");
    /*Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /*Get the CAN FD0 configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
    /*Open the CAN FD0 feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110 in CAN FD0 rx message buffer 0 by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    /* receive 0x220 in CAN FD0 rx message buffer 0 by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    /* receive 0x333 in CAN FD0 rx message buffer 0 by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* receive 0x222 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333, 1), CANFD_RX_BUFFER_EXT_HIGH(0x3333, 1));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  CAN FD Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxTest(void)
{
    uint8_t u8Item;
    uint8_t u8Cnt;
    CANFD_FD_MSG_T      sRxMsgFrame;
    CANFD_FD_MSG_T      sTxMsgFrame;

    /* CAN FD interface initialization*/
    CANFD_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                       CAN FD Function Test                               |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter)   |\n");
    printf("|    and the other is slave(CAN FD receiver). Master will send 6 messages  |\n");
    printf("|    with different sizes of data and ID to the slave. Slave will check if |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)            |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if (u8Item == '0')
    {
        uint8_t u8TxTestNum = 0;

        /* Send 6 messages with different ID and data size */
        for (u8TxTestNum = 0; u8TxTestNum < 6 ; u8TxTestNum++)
        {
            printf("Start to CAN FD Bus Transmitter :\n");

            /* Set the ID Number */
            if (u8TxTestNum == 0)      sTxMsgFrame.u32Id = 0x111;
            else if (u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if (u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if (u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if (u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if (u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /*Set the ID type*/
            if (u8TxTestNum < 3)
                sTxMsgFrame.eIdType = eCANFD_SID;
            else
                sTxMsgFrame.eIdType = eCANFD_XID;

            /*Set the frame type*/
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
            /*Set CAN FD frame format */
            sTxMsgFrame.bFDFormat = 1;
            /*Set the bitrate switch */
            sTxMsgFrame.bBitRateSwitch = 1;

            /*Set the data lenght */
            if (u8TxTestNum == 0  ||  u8TxTestNum == 3)     sTxMsgFrame.u32DLC = 16;
            else if (u8TxTestNum == 1 || u8TxTestNum == 4)  sTxMsgFrame.u32DLC = 32;
            else if (u8TxTestNum == 2 || u8TxTestNum == 5)  sTxMsgFrame.u32DLC = 64;

            if (u8TxTestNum < 3)
                printf("Send to transmit message 0x%08x (11-bit)\n", sTxMsgFrame.u32Id);
            else
                printf("Send to transmit message 0x%08x (29-bit)\n", sTxMsgFrame.u32Id);

            printf("Data Message : ");

            for (u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++)
            {
                sTxMsgFrame.au8Data[u8Cnt] = u8Cnt + u8TxTestNum;
                printf("%02u,", sTxMsgFrame.au8Data[u8Cnt]);
            }

            printf("\n\n");

            /* use message buffer 0 */
            if (CANFD_TransmitTxMsg(CANFD0, 0, &sTxMsgFrame) != eCANFD_TRANSMIT_SUCCESS)
            {
                printf("Failed to transmit message\n");
            }

        }

        printf("\n Transmit Done\n");
    }
    else
    {
        uint8_t u8ErrFlag = 0;
        uint8_t u8RxTestNum = 0;
        uint8_t u8RxTempLen = 0;

        printf("Start to CAN FD Bus Receiver :\n");

        /* Receive  6 messages with different ID and data size */
        do
        {
            /* check for any received messages on CAN FD0 message buffer 0 */
            if (CANFD_ReadRxBufMsg(CANFD0, 0, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {
                printf("Rx buf 0: Received message 0x%08X\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02u ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Standard ID number */
                if ((sRxMsgFrame.u32Id != 0x111) && (sRxMsgFrame.u32Id != 0x22F) && (sRxMsgFrame.u32Id != 0x333))
                {
                    u8ErrFlag = 1;
                }

                if (u8RxTestNum == 0)      u8RxTempLen = 16;
                else if (u8RxTestNum == 1) u8RxTempLen = 32;
                else if (u8RxTestNum == 2) u8RxTempLen = 64;

                /* Check Data lenght */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_SID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN FD STD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }

            /* check for any received messages on CAN FD0 message buffer 1 */
            if (CANFD_ReadRxBufMsg(CANFD0, 1, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {

                printf("Rx buf 1: Received message 0x%08X (29-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02u ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Extend ID number */
                if ((sRxMsgFrame.u32Id  != 0x222) && (sRxMsgFrame.u32Id  != 0x3333) && (sRxMsgFrame.u32Id != 0x44444))
                {
                    u8ErrFlag = 1;
                }

                if (u8RxTestNum == 3)      u8RxTempLen = 16;
                else if (u8RxTestNum == 4) u8RxTempLen = 32;
                else if (u8RxTestNum == 5) u8RxTempLen = 64;

                /* Check Data lenght */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_XID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN FD EXD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;

            }
        } while (u8RxTestNum < 6);

        printf("\n Receive OK & Check OK\n");
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

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|      CAN FD mode transmission test     |\n");
    printf("+----------------------------------------+\n");

    /* CANFD sample function */
    CANFD_TxRxTest();

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
