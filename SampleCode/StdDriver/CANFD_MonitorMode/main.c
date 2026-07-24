/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CAN / CAN FD Monitor mode to listen to CAN bus communication test.
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
CANFD_FD_MSG_T   g_sRxFifo0MsgFrame[3];
CANFD_FD_MSG_T   g_sRxFifo1MsgFrame[3];
uint8_t g_u8RxFifo0RcvOk    = 0;
uint8_t g_u8RxFifo1RcvOk    = 0;
uint8_t g_u8RxFifo0MsgIndex = 0;
uint8_t g_u8RxFifo1MsgIndex = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate);
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD0 Line 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    /* Rx FIFO 0 New Message Interrupt */
    if (CANFD0->IR & CANFD_IR_RF0N_Msk)
    {
        g_u8RxFifo0RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0N_Msk);
    }

    /* Rx FIFO 1 New Message Interrupt */
    if (CANFD0->IR & CANFD_IR_RF1N_Msk)
    {
        g_u8RxFifo1RcvOk = 1;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1N_Msk);
    }

    /* Rx FIFO 0 Message Lost Interrupt */
    if (CANFD0->IR & CANFD_IR_RF0L_Msk)
    {
        printf("Rx FIFO 0 Message Lost(Standard ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF0L_Msk);
    }

    /* Rx FIFO 1 Message Lost Interrupt */
    if (CANFD0->IR & CANFD_IR_RF1L_Msk)
    {
        printf("Rx FIFO 1 Message Lost(Extended ID)\n");
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_RF1L_Msk);
    }
}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                     */
/*---------------------------------------------------------------------------*/
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg)
{
    uint8_t u8Cnt;

    /* Show the message information */
    if (sRxMsg->eIdType == eCANFD_SID)
        printf("Rx FIFO0: ID = 0x%08X(11-bit),DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);
    else
        printf("Rx FIFO1: ID = 0x%08X(29-bit),DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);

    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt < sRxMsg->u32DLC; u8Cnt++)
        printf("%02u ,", sRxMsg->au8Data[u8Cnt]);

    printf("\n\n");
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
/*  Init CAN / CAN FD Monitor Mode                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate)
{
    CANFD_FD_T sCANFD_Config;

    /* Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = u32DataBitRate;
#else
    (void)u32DataBitRate;
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
#endif

    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = u32NormBitRate;
    CANFD_Open(CANFD0, &sCANFD_Config);

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("CAN FD Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
    printf("CAN FD Data bit rate(bps): %d\n", CANFD_GetDataBitRate(CANFD0));
#else
    printf("CAN Nominal bit rate(bps): %d\n", CANFD_GetNominalBitRate(CANFD0));
#endif

    /* Enable the Bus Monitoring Mode */
    CANFD0->CCCR |= CANFD_CCCR_MON_Msk;

    /* Non-matching frames with Standard ID stored in Rx FIFO0, Extended ID stored in Rx FIFO1.
       Reject all remote frames with 11-bit standard IDs and 29-bit extended IDs. */
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, 1, 1);

    /* Enable RX FIFO new message and message lost interrupts using interrupt line 0 */
    CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_RF0LE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_RF1LE_Msk), 0, 0, 0);

    /* Enable CANFD0 IRQ00 Handler */
    NVIC_EnableIRQ(CANFD00_IRQn);

    /* CAN FD0 Run to Normal mode */
    CANFD_RunToNormal(CANFD0, TRUE);
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
    printf("+-----------------------------------------------------------------------------+\n");
#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    printf("|                      CAN FD Monitor Mode sample code                        |\n");
#else
    printf("|                        CAN Monitor Mode sample code                         |\n");
#endif
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CAN Monitor mode to listen to    |\n");
    printf("|    CAN Bus communication test.The sample code must be set up on the node of |\n");
    printf("|    CAN communication transmission. Users can use one of the sample codes    |\n");
    printf("|    ' CANFD_TxRx ' or ' CANFD_TxRxINT ' as the CAN communication             |\n");
    printf("|    transmission network.                                                    |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");


    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                              Pin Configure                                  |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                 CAN BUS                 CAN_RXD(Any board)  |\n");
    printf("|                                   ||    CAN_H   |-----------|               |\n");
    printf("|                                   || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|  CAN0_TXD(PJ10)                   ||    CAN_L   |Transceiver|               |\n");
    printf("|  CAN0_RXD(PJ11)                   || <--------->|           |------>        |\n");
    printf("|          |-----------|   CAN_H    ||            |           |CAN_RX         |\n");
    printf("|  ------> |           |<---------> ||            |-----------|               |\n");
    printf("|  CAN0_TX |   CAN     |            ||                                        |\n");
    printf("|          |Transceiver|            ||                    CAN_TXD(Any board)  |\n");
    printf("|  <------ |           |   CAN_L    ||                    CAN_RXD(Any board)  |\n");
    printf("|  CAN0_RX |           |<---------> ||    CAN_H   |-----------|               |\n");
    printf("|          |-----------|            || <--------->|           |<------        |\n");
    printf("|                                   ||            |    CAN    |CAN_TX         |\n");
    printf("|                                   ||    CAN_L   |Transceiver|               |\n");
    printf("|                                   || <--------->|           |------>        |\n");
    printf("|                                   ||            |           |CAN_RX         |\n");
    printf("|                                   ||            |-----------|               |\n");
    printf("|                                                                             |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

#if (OP_MODE == CANFD_OP_CAN_FD_MODE)
    /* CAN FD interface initialization: Nominal 1 Mbps, Data 2 Mbps */
    CANFD_MonitorMode_Init(1000000, 2000000);
#else
    /* CAN interface initialization: 1 Mbps (data bitrate unused in CAN mode) */
    CANFD_MonitorMode_Init(1000000, 0);
#endif

    while (1)
    {
        if (g_u8RxFifo0RcvOk == 1)
        {
            if (g_u8RxFifo0MsgIndex > 2)
                g_u8RxFifo0MsgIndex = 0;

            /* Receive the Rx FIFO0 message (Standard ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxFifo0MsgFrame[g_u8RxFifo0MsgIndex]);
            g_u8RxFifo0MsgIndex++;
            g_u8RxFifo0RcvOk = 0;
        }

        if (g_u8RxFifo0RcvOk == 0 && g_u8RxFifo0MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFifo0MsgFrame[g_u8RxFifo0MsgIndex - 1]);
            g_u8RxFifo0MsgIndex--;
        }

        if (g_u8RxFifo1RcvOk == 1)
        {
            if (g_u8RxFifo1MsgIndex > 2)
                g_u8RxFifo1MsgIndex = 0;

            /* Receive the Rx FIFO1 message (Extended ID) */
            CANFD_ReadRxFifoMsg(CANFD0, 1, &g_sRxFifo1MsgFrame[g_u8RxFifo1MsgIndex]);
            g_u8RxFifo1MsgIndex++;
            g_u8RxFifo1RcvOk = 0;
        }

        if (g_u8RxFifo1RcvOk == 0 && g_u8RxFifo1MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFifo1MsgFrame[g_u8RxFifo1MsgIndex - 1]);
            g_u8RxFifo1MsgIndex--;
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
