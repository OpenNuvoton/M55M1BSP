/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through CANFD interface.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip CANFD and assign update file
 *           of Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define CANFD_BAUD_RATE         500000
#define Master_ISP_ID           0x487
#define Device0_ISP_ID          0x784
#define CANFD_ISP_DataLength    0x08
#define CANFD_RETRY_COUNTS      0x1fffffff

#define CMD_READ_CONFIG         0xA2000000
#define CMD_RUN_APROM           0xAB000000
#define CMD_GET_DEVICEID        0xB1000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
/* Declare a CAN message structure */
typedef struct
{
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP_T;

static volatile CANFD_FD_MSG_T g_sRxMsgFrame;
static volatile uint8_t s_u8CANPackageFlag = 0, s_u8CANAckFlag = 0;

int32_t SYS_Init(void);
void CANFD_Package_ACK(CANFD_T *psCanfd);
void CANFD_Init(void);
void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(int ch);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void CANFD00_IRQHandler(void)
{
    uint32_t u32IIDRstatus;

    u32IIDRstatus = CANFD0->IR;

    /**************************/
    /* Status changed interrupt*/
    /**************************/
    if ((u32IIDRstatus & CANFD_IR_RF0N_Msk) == CANFD_IR_RF0N_Msk)
    {
        /* Clear interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 0, (CANFD_FD_MSG_T *)&g_sRxMsgFrame);
        s_u8CANPackageFlag = 1;
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32IIDRstatus = CANFD0->IR;
}

int32_t SYS_Init(void)
{
    /* Enable External HXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select SCLK to HIRC before APLL setting*/
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

    /* Enable APLL0 220MHz clock */
    CLK_DisableAPLL(CLK_APLL0_SELECT);
    /* Apply new PLL0 setting. */
    CLK->APLL0CTL = (CLK_APLLCTL_220MHz | CLK_APLLCTL_STBSEL_2460);
    /* Apply PLL0 Clock Source */
    CLK->APLL0SEL = (CLK->APLL0SEL & ~CLK_APLL0SEL_APLLSRC_Msk) | (CLK_APLLCTL_APLLSRC_HIRC << CLK_APLL0SEL_APLLSRC_Pos);
    /* Enable PLL0 */
    CLK->SRCCTL |= CLK_SRCCTL_APLL0EN_Msk;
    /* Wait for PLL clock stable */
    CLK_WaitClockReady(CLK_STATUS_APLL0STB_Msk);

    /* Set clock with limitations */
    CLK_SET_HCLK2DIV(2);
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Switch SCLK clock source to APLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CANFDSEL_CANFD0SEL_HXT, CLK_CANFDDIV_CANFD0DIV(1));
    /* Enable module clock */
    CLK_EnableModuleClock(ISP0_MODULE);
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SET_CANFD0_RXD_PJ11();
    SET_CANFD0_TXD_PJ10();

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Package_ACK(CANFD_T *psCanfd)
{
    CANFD_FD_MSG_T  sTxMsgFrame;

    s_u8CANAckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
    sTxMsgFrame.eIdType  = eCANFD_SID;
    sTxMsgFrame.u32Id    = Device0_ISP_ID;
    sTxMsgFrame.u32DLC   = CANFD_ISP_DataLength;

    sTxMsgFrame.au32Data[0] = g_sRxMsgFrame.au32Data[0];
    sTxMsgFrame.au32Data[1] = g_sRxMsgFrame.au32Data[1];

    if (CANFD_TransmitTxMsg(psCanfd, 0, &sTxMsgFrame) == FALSE)    // Configure Msg RAM and send the Msg in the RAM
    {
        return;
    }
}

void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    /*Use defined configuration */
    sCANFD_Config.sElemSize.u32UserDef = 0;
    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = CANFD_BAUD_RATE;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    /* Open the CAN feature */
    CANFD_Open(CANFD0, &sCANFD_Config);
    /* Set CAN reveive message */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(Master_ISP_ID, 0x7FF));
    /* receive 0x220~0x22f (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* Enable Standard ID and  Extended ID Filter as RX FOFI0*/
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk | CANFD_IE_TCE_Msk), 0, 0, 0);

    /* Enable CANFD0 IRQ */
    NVIC_EnableIRQ(CANFD00_IRQn);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeoutInMS = 300;

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Init CAN port */
    CANFD_Init();

    /* Unlock write-protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    while (u32TimeoutInMS > 0)
    {
        if (s_u8CANPackageFlag == 1)
        {
            goto _ISP;
        }

        CLK_SysTickDelay(1000);
        u32TimeoutInMS--;
    }

    /* Timeout then go to APROM */
    if (u32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    /* state of update program */
    while (1)
    {
        if (s_u8CANPackageFlag)
        {
            volatile STR_CANMSG_ISP_T *psISPCanMsg = (STR_CANMSG_ISP_T *)&g_sRxMsgFrame.au32Data[0];

            s_u8CANPackageFlag = 0;

            if (psISPCanMsg->Address == CMD_GET_DEVICEID)
            {
                psISPCanMsg->Data = SYS->PDID;
            }
            else if (psISPCanMsg->Address == CMD_READ_CONFIG)
            {
                uint32_t u32Data = psISPCanMsg->Data;
                psISPCanMsg->Data = FMC_Read(u32Data);
            }
            else if (psISPCanMsg->Address == CMD_RUN_APROM)
            {
                break;
            }
            else    // Update user config or APROM
            {
                uint32_t u32Addr = psISPCanMsg->Address;

                if ((u32Addr & FMC_CONFIG_BASE) == FMC_CONFIG_BASE)
                {
                    if (psISPCanMsg->Data != FMC_Read(u32Addr))
                        FMC_WriteConfig(u32Addr, psISPCanMsg->Data);

                    psISPCanMsg->Data = FMC_Read(u32Addr);
                }
                else if (u32Addr < FMC_APROM_SIZE)
                {
                    if ((u32Addr % FMC_FLASH_PAGE_SIZE) == 0)
                    {
                        FMC_Erase(FMC_APROM_BASE + u32Addr);
                    }

                    FMC_Write(FMC_APROM_BASE + u32Addr, psISPCanMsg->Data);
                    psISPCanMsg->Data = FMC_Read(FMC_APROM_BASE + u32Addr);
                }
            }

            /* Send CAN FD ISP Package (ACK) */
            CANFD_Package_ACK(CANFD0);
        }
    }

_APROM:
    /* CAN FD interface finalization */
    NVIC_DisableIRQ(CANFD00_IRQn);
    CANFD_Close(CANFD0);

    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
