/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CAN mode function to do internal loopback test.
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
volatile uint8_t   g_u8RxFifo0CompleteFlag = 0;

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
    printf("IR =0x%08X \n", CANFD0->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
    CANFD_ReadRxFifoMsg(CANFD0, 0, &g_sRxMsgFrame);
    g_u8RxFifo0CompleteFlag = 1;
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

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

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

}

/*---------------------------------------------------------------------------------------------------------*/
/*                             CAN Tx Rx Function Test                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRx_Test(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eFrameIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;

    /*Set the ID Number*/
    psTxMsg->u32Id = u32Id;
    /*Set the ID Type*/
    psTxMsg->eIdType = eFrameIdType;
    /*Set the frame type*/
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    /*Set FD frame format attribute */
    psTxMsg->bFDFormat = 0;
    /*Set the bitrate switch attribute*/
    psTxMsg->bBitRateSwitch = 0;
    /*Set data length*/
    psTxMsg->u32DLC = u8Len;

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFifo0CompleteFlag = 0;

    /* use message buffer 0 */
    if (eFrameIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }

    /*Wait the Rx FIFO0 received message*/
    while (!g_u8RxFifo0CompleteFlag)
    {
    }

    printf("Rx buf 0: Received message 0x%08X\n", g_sRxMsgFrame.u32Id);
    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02u ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
    memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
}

/*---------------------------------------------------------------------------------------------------------*/
/*                CAN Function Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Loopback(void)
{
    uint8_t u8Loop;
    CANFD_FD_T sCANFD_Config;
    /*Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /*Get the CAN configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    /*Enable internal loopback mode*/
    sCANFD_Config.sBtConfig.bEnableLoopBack = TRUE;
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    /*Open the CAN FD feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110~0x11F in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* receive 0x22F in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* receive 0x220~0x22F (29-bit id) in CAN rx fifo0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN rx fifo0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN rx fifo0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX fifo0)*/
    CANFD_SetGFC(CANFD0, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
    /* Enable CANFD0 IRQ00 Handler*/
    NVIC_EnableIRQ(CANFD00_IRQn);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);

    for (u8Loop = 1 ; u8Loop < 8; u8Loop++)
    {
        CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x110 + u8Loop, u8Loop);
    }

    CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
    CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);

    for (u8Loop = 1 ; u8Loop < 8; u8Loop++)
    {
        CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x220 + u8Loop, u8Loop);
    }

    CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
    CAN_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
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
    printf("\n CAN Mode Loopback example\r\n");
    /* CAN Loopback Test */
    CAN_Loopback();
    printf("\n CAN Mode Loopback Test Done\r\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
