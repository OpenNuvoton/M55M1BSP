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
volatile uint8_t   g_u8BusOffFlag = 0;
volatile uint32_t  g_u32BusOffRecoveryCounter = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_TxRxTest(void);
uint8_t CANFD_BusOffRecovery(void);
uint8_t CANFD_CheckBusOffStatus(void);

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
    /* Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /* Get the CAN FD0 configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 2000000;
    /* Open the CAN FD0 feature */
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110 in CAN FD0 rx message buffer 0 by setting index 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    /* receive 0x220 in CAN FD0 rx message buffer 0 by setting index 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    /* receive 0x333 in CAN FD0 rx message buffer 0 by setting index 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* receive 0x222 (29-bit id) in CAN FD0 rx message buffer 1 by setting index 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx message buffer 1 by setting index 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333, 1), CANFD_RX_BUFFER_EXT_HIGH(0x3333, 1));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx message buffer 1 by setting index 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));
    /* CAN FD0 Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                 CAN FD Bus-Off Status Check Function                                   */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_CheckBusOffStatus(void)
{
    uint32_t u32IntStatus;

    /* Read interrupt status register */
    u32IntStatus = CANFD0->IR;

    /* Check Bus-Off status */
    if (u32IntStatus & CANFD_IR_BO_Msk)
    {
        if (CANFD0->PSR & CANFD_PSR_BO_Msk)
        {
            printf("Bus-Off detected!\n");
            g_u8BusOffFlag = 1;
        }

        /* Clear Bus-Off interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        /* Bus-Off detected */
        return 1;
    }

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

    /* No Bus-Off detected */
    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                 CAN FD Bus-Off Recovery Function                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CANFD_BusOffRecovery(void)
{
    printf("Starting CAN FD Bus-Off recovery sequence...\n");

    /* CAN FD0 run to initial mode */
    CANFD_RunToNormal(CANFD0, FALSE);

    /* Cancel all transmit requests */
    CANFD0->TXBCR = 0xFFFFFFFF;

    /* Clear all interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, 0xFFFFFFFF);

    /* CAN FD0 run to normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);

    /* 50ms delay after recovery process */
    CLK_SysTickDelay(50000);

    /* Check if recovery was successful by verifying Bus-Off status */
    if (CANFD0->PSR & CANFD_PSR_BO_Msk)
    {
        /* Still in Bus-Off state, recovery failed */
        printf("CAN FD Bus-Off recovery failed. Still in Bus-Off state.\n");
        /* Recovery failed */
        return 0;
    }
    else
    {
        /* Recovery successful, clear Bus-Off flag */
        g_u8BusOffFlag = 0;
        g_u32BusOffRecoveryCounter++;
        printf("CAN FD Bus-Off recovery completed. Recovery count: %u\n", g_u32BusOffRecoveryCounter);
        /* Recovery successful */
        return 1;
    }
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

    /* CAN FD interface initialization */
    CANFD_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                       CAN FD Function Test                               |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter)   |\n");
    printf("|    and the other is slave(CAN FD receiver). Master will send 6 messages  |\n");
    printf("|    with different sizes of data and ID to the slave. Slave will check if |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|    Bus-Off recovery feature is enabled for error handling.               |\n");
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
            /* Check for Bus-Off status before transmission */
            CANFD_CheckBusOffStatus();

            /* Check if CAN FD is in Bus-Off state before transmitting */
            if (g_u8BusOffFlag)
            {
                printf("CAN FD is in Bus-Off state. Starting recovery process...\n");

                if (CANFD_BusOffRecovery())
                {
                    printf("Bus-Off recovery successful. Proceeding with transmission.\n");
                }
                else
                {
                    printf("Bus-Off recovery failed. Skipping transmission.\n");
                    continue; /* Skip this transmission and try next */
                }
            }

            printf("Start to CAN FD Bus Transmitter :\n");

            /* Set the ID Number */
            if (u8TxTestNum == 0)      sTxMsgFrame.u32Id = 0x111;
            else if (u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if (u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if (u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if (u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if (u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /* Set the ID type */
            if (u8TxTestNum < 3)
                sTxMsgFrame.eIdType = eCANFD_SID;
            else
                sTxMsgFrame.eIdType = eCANFD_XID;

            /* Set the frame type */
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
            /* Set CAN FD frame format */
            sTxMsgFrame.bFDFormat = 1;
            /* Set the bitrate switch */
            sTxMsgFrame.bBitRateSwitch = 1;

            /* Set the data lenght */
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

                /* Check if failure was due to bus-off condition */
                if (CANFD_CheckBusOffStatus())
                {
                    printf("Transmission failed due to Bus-Off condition.\n");
                }
            }
            else
            {
                /* Check for any error conditions after successful transmission */
                CANFD_CheckBusOffStatus();
                printf("Message transmitted successfully.\n");
            }

        }

        printf("\n Transmit Done\n");

        if (g_u32BusOffRecoveryCounter > 0)
        {
            printf("Total Bus-Off recovery cycles: %u\n", g_u32BusOffRecoveryCounter);
        }
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
