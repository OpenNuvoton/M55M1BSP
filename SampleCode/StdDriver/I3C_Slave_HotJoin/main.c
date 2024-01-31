/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to initiate a Hot-Join request to an I3C Master.
 *          This sample code needs to work with I3C_MasterRW.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define DGBINT          printf
//#define DGBINT(...)

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define I3C0_SA        (0x68)
#define I3C0_MID       (0x8123UL)
#define I3C0_PID       (0xA13573C0UL)

volatile uint32_t   g_RespQ[I3C_DEVICE_RESP_QUEUE_CNT];
volatile uint32_t   g_RxBuf[I3C_DEVICE_RX_BUF_CNT], g_TxBuf[I3C_DEVICE_RX_BUF_CNT];
volatile uint32_t   g_u32IntSelMask = 0, g_u32IntOccurredMask = 0;
volatile uint32_t   g_u32RespStatus = I3C_STS_NO_ERR;

int32_t I3C_ProcessNoneRespInt(I3C_T *i3c, uint32_t i32IntMask);
int32_t I3C_ProcessRespError(I3C_T *i3c, uint32_t u32RespStatus);
NVT_ITCM void I3C0_IRQHandler(void);

/**
  * @brief  Process interrupt events except Response Ready interrupt.
  */
int32_t I3C_ProcessNoneRespInt(I3C_T *i3c, uint32_t u32IntMask)
{
    if (u32IntMask & I3C_INTEN_DA_ASSIGNED)
    {
        printf("[ Set I3C Dynamic Address 0x%02x ]\n\n", (uint32_t)I3C_GET_I3C_DA(i3c));
    }

    if (u32IntMask & I3C_INTEN_TRANSFER_ERR)
    {
        printf("[ Transfer error ]\n\n");
    }

    if (u32IntMask & I3C_INTEN_READ_REQUEST)
    {
        printf("[ Master read request event ]\n\n");
    }

    if (u32IntMask & I3C_INTEN_CCC_UPDATED)
    {
        printf("[ CCC Updated event ]\n");

        if (i3c->SLVEVNTS & I3C_SLVEVNTS_MWLUPD_Msk)
        {
            i3c->SLVEVNTS = I3C_SLVEVNTS_MWLUPD_Msk;
            printf("\t* Updated MWL to 0x%x\n\n", (uint32_t)((i3c->SLVMXLEN & I3C_SLVMXLEN_MWL_Msk) >> I3C_SLVMXLEN_MWL_Pos));
        }
        else if (i3c->SLVEVNTS & I3C_SLVEVNTS_MRLUPD_Msk)
        {
            i3c->SLVEVNTS = I3C_SLVEVNTS_MRLUPD_Msk;
            printf("\t* Updated MRL to 0x%x\n\n", (uint32_t)((i3c->SLVMXLEN & I3C_SLVMXLEN_MRL_Msk) >> I3C_SLVMXLEN_MRL_Pos));
            /* Reset TX FIFO and CMDQ FIFO -> apply resume */
            I3C_ResetAndResume(i3c, (I3C_RESET_TX_BUF | I3C_RESET_CMD_QUEUE), TRUE);
        }
        else
        {
            printf("\t* Updated - ENTAS%d\n", (uint32_t)((i3c->SLVEVNTS & I3C_SLVEVNTS_ACTSTATE_Msk) >> I3C_SLVEVNTS_ACTSTATE_Pos));
            printf("\t* Updated - HJEN %d\n", (uint32_t)((i3c->SLVEVNTS & I3C_SLVEVNTS_HJEN_Msk) >> I3C_SLVEVNTS_HJEN_Pos));
            printf("\t* Updated - SIREN %d\n", (uint32_t)((i3c->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk) >> I3C_SLVEVNTS_SIREN_Pos));
        }
    }

    return 0;
}

/**
  * @brief  Process response error event.
  */
int32_t I3C_ProcessRespError(I3C_T *i3c, uint32_t u32RespStatus)
{
    printf("[ Resp Error 0x%x] ", (u32RespStatus >> I3C_RESPQUE_ERRSTS_Pos));

    if (u32RespStatus == I3C_RESP_CRC_ERR)
    {
        printf("CRC error\n");
    }
    else if (u32RespStatus == I3C_RESP_PARITY_ERR)
    {
        printf("Parity error\n");
    }
    else if (u32RespStatus == I3C_RESP_FRAME_ERR)
    {
        printf("Frame error\n");
    }
    else if (u32RespStatus == I3C_RESP_FLOW_ERR)
    {
        printf("Underflow/Overflow error\n");
    }
    else if (u32RespStatus == I3C_RESP_MASTER_TERMINATE_ERR)
    {
        printf("Master early termination error\n");
    }
    else
    {
        printf("Unknow error\n");
    }

    if (I3C_IS_PROTOCOL_ERR(i3c))
    {
        printf("[ Device Protocol Error ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_UNDERFLOW_ERR(i3c))
    {
        printf("[ Device Underflow Error ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_OVERFLOW_ERR(i3c))
    {
        printf("[ Device Overflow Error ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_DATA_NOT_READY(i3c))
    {
        printf("[ Device Data Not Ready Status ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_BUFFER_NOT_AVAIL(i3c))
    {
        printf("[ Device Buffer Not Available Status ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_FRAME_ERR(i3c))
    {
        printf("[ Device Frame Error ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
    }

    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        printf("[ Device Slave Busy Status ] (0x%04x)\n\n", I3C_GET_DEVICE_STATUS(i3c));
        printf("\tPerform FIFO/Queue reset then wait RESUME complete ... ");
        I3C_RespErrorRecovery(i3c, g_u32RespStatus);
        printf("done.\n\n");
    }

    return 0;
}

/**
  * @brief  The I3C0 default IRQ, declared in startup_NUC1263.s.
  */
NVT_ITCM void I3C0_IRQHandler(void)
{
    DGBINT("\n");

    if (g_u32IntSelMask & I3C_INTEN_TX_EMPTY_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_TX_EMPTY_THLD))
        {
            DGBINT("[ INT ] TX_EMPTY_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_TX_EMPTY_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_RX_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_RX_THLD))
        {
            DGBINT("[ INT ] INTSTS_RX_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_RX_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_CMDQ_EMPTY_THLD)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_CMDQ_EMPTY_THLD))
        {
            DGBINT("[ INT ] CMDQ_EMPTY_THLD\n");
            g_u32IntOccurredMask |= I3C_INTSTS_CMDQ_EMPTY_THLD;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_RESPQ_READY)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_RESPQ_READY))
        {
            DGBINT("[ INT ] RESPQ_READY\n");
            g_u32IntOccurredMask |= I3C_INTSTS_RESPQ_READY;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_CCC_UPDATED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_CCC_UPDATED))
        {
            DGBINT("[ INT ] CCC_UPDATED\n");
            I3C_CLEAR_CCC_UPDATED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_CCC_UPDATED;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_DA_ASSIGNED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_DA_ASSIGNED))
        {
            DGBINT("[ INT ] DA_ASSIGNED\n");
            I3C_CLEAR_DA_ASSIGNED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_DA_ASSIGNED;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_TRANSFER_ERR)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_TRANSFER_ERR))
        {
            DGBINT("[ INT ] TRANSFER_ERR\n");
            I3C_CLEAR_TRANSFER_ERR_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_TRANSFER_ERR;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_READ_REQUEST)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_READ_REQUEST))
        {
            DGBINT("[ INT ] READ_REQUEST\n");
            I3C_CLEAR_READ_REQUEST_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_READ_REQUEST;
        }
    }

    if (g_u32IntSelMask & I3C_INTEN_IBI_UPDATED)
    {
        if (I3C_IS_INT_STATUS(I3C0, I3C_INTSTS_IBI_UPDATED))
        {
            DGBINT("[ INT ] IBI_UPDATED\n");
            I3C_CLEAR_IBI_UPDATED_STATUS(I3C0);
            g_u32IntOccurredMask |= I3C_INTSTS_IBI_UPDATED;
        }
    }

    if (g_u32IntOccurredMask & I3C_INTSTS_RESPQ_READY)
    {
        g_u32RespStatus = (uint32_t)I3C_ParseRespQueue(I3C0, (uint32_t *)(&g_RespQ[0]));
    }

    DGBINT("[ INT EXIT ] INTSTS: 0x%08x. Occurred: 0x%08x.\n\n", I3C0->INTSTS, g_u32IntOccurredMask);
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);
    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);
    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Set multi-function pins for I3C0 SDA and SCL */
    SET_I3C0_SCL_PB1();
    SET_I3C0_SDA_PB0();
    SYS_ResetModule(SYS_I3C0RST);
    /* Lock protected registers */
    SYS_LockReg();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t    i, u16Len;
    uint8_t     *pu8Data, u8TID;
    uint8_t     qn, u8RespQCnt;
    uint32_t    u32ActiveIntMask;
    int32_t     iErrCode = I3C_STS_NO_ERR;
    uint8_t     ch;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------+\n");
    printf("|    I3C0 Slave Hot-Join Request Sample Code    |\n");
    printf("+-----------------------------------------------+\n");
    printf("[I] Initiate a Hot-Join request immediately after I3C0 enabled.\n");
    printf("[A] Initiate a Hot-Join request when a 7'h7E header on the bus.\n");
    printf("Select: ");
    ch =  getchar();
    /* Initial I3C0 default settings */
    I3C0->SLVMID = I3C0_MID;
    I3C0->SLVPID = I3C0_PID;

    if ((ch == 'I') || (ch == 'i'))
    {
        printf("(I) I3C_SUPPORT_IMMEDIATE_HJ\n");
        I3C_Open(I3C0, I3C_SLAVE, I3C0_SA, I3C_SUPPORT_IMMEDIATE_HJ);
    }
    else
    {
        printf("(A) I3C_SUPPORT_ADAPTIVE_HJ\n");
        I3C_Open(I3C0, I3C_SLAVE, I3C0_SA, I3C_SUPPORT_ADAPTIVE_HJ);
    }

    /* Enable I3C0 interrupts */
    g_u32IntSelMask = (I3C_INTEN_RESPQ_READY | I3C_INTEN_CCC_UPDATED | I3C_INTEN_DA_ASSIGNED |
                       I3C_INTEN_TRANSFER_ERR | I3C_INTEN_READ_REQUEST);
    I3C_ENABLE_INT(I3C0, g_u32IntSelMask);
    NVIC_EnableIRQ(I3C0_IRQn);
    /* Enable I3C0 controller */
    I3C_Enable(I3C0);
    printf("# I3C0 Slave settings:\n");
    printf("    - SDA on PB.0\n");
    printf("    - SCL on PB.1\n");
    printf("    - I2C Static Address 0x%02x\n", I3C0_SA);
    printf("    - RespQ interrupt threshold %d\n", (uint32_t)(I3C_GET_RESPQ_THLD(I3C0) + 1));
    printf("    - The first operation of the I3C enable bit requires at least bus SCLx4 to become active\n");
    printf("# An I3C Master can write N-bytes data to Slave,\n");
    printf("  then perform a read request to receive the N-bytes data from Slave.\n");
    printf("    - The write data should be equal to the received data\n");
    printf("\n");
    /* Enable I3C0 controller */
    I3C_Enable(I3C0);
    printf("[ Wait for the I3C Slave to issue a Hot-Join request ]\n\n");

    while (1)
    {
        while (g_u32IntOccurredMask != 0)
        {
            /* Get active interrupt occurred events */
            u32ActiveIntMask = g_u32IntOccurredMask;
            g_u32IntOccurredMask = 0;

            if (u32ActiveIntMask & I3C_INTSTS_RESPQ_READY)
            {
                /* Process Response */
                if (g_u32RespStatus == I3C_STS_NO_ERR)
                {
                    /* Response no error */
                    u8RespQCnt = I3C_GET_RESPQ_THLD(I3C0) + 1;
                    qn = 0; // Queue number

                    do
                    {
                        if (I3C_IS_RESP_RX(g_RespQ[qn]))
                        {
                            /* Master write request */
                            u16Len = I3C_GET_RESP_DATA_LEN(g_RespQ[qn]);
                            printf("Slave receives %d-bytes:\n\thex: ", u16Len);

                            /* Read Rx data from data port */
                            for (i = 0; i < ((u16Len + 3) / 4); i++)
                            {
                                g_RxBuf[i] = I3C0->TXRXDAT;
                            }

                            pu8Data = (uint8_t *)(&g_RxBuf[0]);

                            for (i = 0; i < u16Len; i++)
                            {
                                printf("%02x ", pu8Data[i]);
                            }

                            printf("\n\n");
                            /* Clear CmdQ and Tx data for new transmit data */
                            I3C_ResetAndResume(I3C0, (I3C_RESET_CMD_QUEUE | I3C_RESET_TX_BUF), FALSE);
                            /* Set CmdQ and response data for a Master read request */
                            memcpy((uint8_t *)(&g_TxBuf[0]), (uint8_t *)(&g_RxBuf[0]), u16Len);
                            u8TID = (pu8Data[0] % 8);
                            iErrCode = I3C_SetCmdQueueAndData(I3C0, u8TID, (uint32_t *)&g_TxBuf[0], u16Len);

                            if (iErrCode != I3C_STS_NO_ERR)
                            {
                                printf("\tSet TX data error, %d.\n\n", iErrCode);
                            }
                            else
                            {
                                printf("[ Set TX %d-bytes and TID %d for Master read request ]\n\n", u16Len, u8TID);
                            }
                        }
                        else
                        {
                            /* Master read request -> Slave transmits data done */
                            printf("Slave transmits ID-%d done.\n\n", (uint32_t)I3C_GET_RESP_TID(g_RespQ[qn]));
                        }

                        qn++;
                        u8RespQCnt--;
                    } while (u8RespQCnt);
                }
                else
                {
                    /* Response has error */
                    I3C_ProcessRespError(I3C0, g_u32RespStatus);
                }

                g_u32RespStatus = I3C_STS_NO_ERR;
            }
            else
            {
                /* Process others interrupt event */
                I3C_ProcessNoneRespInt(I3C0, u32ActiveIntMask);
            }
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
