/******************************************************************************
 * @file     i2c_process.c
 * @version  V1.00
 * @brief    NAU88C22 I2C process source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/*!<Includes */
#include "i2c_process.h"
#include "usbd_audio.h"
#include "user_config.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
// I2C control
uint8_t g_u8SlaveAddr = 0x40;
volatile uint16_t g_u16I2CRxDataLen = 0;
volatile uint16_t g_u16I2CRxDataCnt = 0;
volatile uint16_t g_u16I2CTxDataLen = 0;
volatile uint16_t g_u16I2CTxDataCnt = 0;
volatile uint16_t g_u16I2CTimeOutCnt = 0;
volatile uint8_t g_u8I2CTimeOutTestflag = 0;
volatile uint8_t g_u8I2CTimeOut = 0;
volatile uint8_t g_u8I2CRead = 0;
volatile uint8_t g_u8CheckConnection = 0; // 0: no check, 1:under checking, 2: finish checking
volatile uint8_t g_u8NackCnt = 0;

void (*pI2CProtocolCallback)(void) = NULL;

// Codec Init
extern volatile uint8_t g_u8CodecInit;
extern volatile uint16_t g_u16CodecTimeOutCnt;
extern volatile uint8_t g_u8_7802Cntflag;
extern volatile uint8_t g_u8_7802TimeOut;
extern volatile uint16_t g_u16_7802TimeOutCnt;
extern volatile uint16_t g_u16SampleNum;
extern volatile uint16_t g_u16SampleNum_cnt;
extern volatile uint8_t g_u8read_process;
volatile uint16_t g_u16PowerOnDelay = 0;

volatile uint8_t g_u8EnterUnknownStatus = 0;

extern HIDTRANS_CMD_T g_sHidCmdH2D; // Host to device
extern HIDTRANS_CMD_T g_sHidCmdD2H; // Device to host
extern uint8_t g_au8ToHIDDataBuf[]; // Data to PC; [0][1]:I2C data length, [2]:command type, [3] feedback or not, the rest are data
extern uint8_t g_au8ToI2CDataBuf[]; // Data from PC; [0][1]:I2C data length, [2]:command type, [3] feedback or not, the rest are data
extern volatile uint8_t g_u8HIDCommandProcessing;

extern volatile uint8_t g_u8WriteFail;
extern volatile uint16_t g_u16I2CTxData_Fail_Cnt;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void I2C3_IRQHandler(void);
void SysTick_Handler(void);

/**
 * @brief       I2C Callback Function
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used as I2C callback funtion
 */
void I2C_Callback(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C_PORT);

    if (I2C_GET_TIMEOUT_FLAG(I2C_PORT))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(I2C_PORT);
    }
    else
    {
        switch (u32Status)
        {

            case 0x08:                               // START has been transmitted
                if (g_sHidCmdH2D.u8CommandType == 0) // Test Connection
                {
                    I2C_SET_DATA(I2C_PORT, (0x2F << 1)); // Write SLA+W to Register I2CDAT
                }
                else
                {
                    if (g_u8I2CRead)
                    {
                        g_u8I2CRead = 0;
                        I2C_SET_DATA(I2C_PORT, (g_u8SlaveAddr | 0x01)); // Write SLA+R to Register I2CDAT
                    }
                    else
                    {
                        I2C_SET_DATA(I2C_PORT, g_u8SlaveAddr); // Write SLA+W to Register I2CDAT
                    }
                }

                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI); // release SCL, not NACK it due to the ACK/NACK is determined by slave.
                break;

            // Master Write
            case 0x18: // SLA+W has been transmitted and ACK has been received
                if (g_u8CheckConnection == 1)
                {
                    g_u8CheckConnection = 2;
                    I2C_STOP(I2C_PORT);
                }
                else
                {
                    g_u8I2CTimeOut = 0;
                    g_u8I2CTimeOutTestflag = 1;
                    I2C_SET_DATA(I2C_PORT, g_au8ToI2CDataBuf[g_u16I2CTxDataCnt++]);
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI); // release SCL, not NACK it due to the ACK/NACK is determined by slave.
                }

                break;

            case 0x20: // SLA+W has been transmitted and NACK has been received
                I2C_STOP(I2C_PORT);

                if (++g_u8NackCnt >= 100)
                {
                    g_u8I2CTimeOut = 1;
                    g_u8NackCnt = 0;

                    if (g_u8CheckConnection == 1)
                    {
                        g_u8CheckConnection = 2;
                    }
                }
                else
                {
                    I2C_START(I2C_PORT);
                }

                break;

            case 0x28: // DATA has been transmitted and ACK has been received
                g_u16I2CTimeOutCnt = 0;

                if (g_u16I2CTxDataCnt < g_u16I2CTxDataLen)
                {
                    I2C_SET_DATA(I2C_PORT, g_au8ToI2CDataBuf[g_u16I2CTxDataCnt++]);
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }
                else // Write data finished
                {
                    g_u16I2CTxDataCnt = 0;
                    g_u16I2CTxDataLen = 0;
                    g_u8I2CTimeOutTestflag = 0;
                    g_u16I2CTimeOutCnt = 0;

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        // run I2C read flow
                        g_u8I2CRead = 1;
                        // Stop I2C and start again
                        I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_STO | I2C_CTL_SI | I2C_CTL_STA));
                    }
                    else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
                    {
                        g_u8HIDCommandProcessing = FINISHED;
                        I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                    }
                }

                break;

            case 0x30: // DATA has been transmitted and NACK has been received
                // resend data
                I2C_SET_DATA(I2C_PORT, g_au8ToI2CDataBuf[g_u16I2CTxDataCnt - 1]);
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;

            // Master Read, after Master Write tells the slave what to send, enter Master Read
            case 0x40: // SLA+R has been transmitted and ACK has been received
                g_u8I2CTimeOut = 0;
                g_u8I2CTimeOutTestflag = 1;
                I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_SI | I2C_CTL_AA));
                break;

            case 0x48: // SLA+R has been transmitted and NACK has been received

                // Stop I2C and start again
                if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON) || (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_BONGIOVI))
                {
                    g_u8I2CRead = 1;
                }

                I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_STO | I2C_CTL_SI | I2C_CTL_STA));
                break;

            case 0x50: // DATA has been received and ACK has been returned
                g_u16I2CTimeOutCnt = 0;
                g_au8ToHIDDataBuf[g_u16I2CRxDataCnt++] = I2C_GET_DATA(I2C_PORT);

                if (g_u16I2CRxDataCnt == I2C_PREFIX_LEN)
                {
                    g_u16I2CRxDataLen = (g_au8ToHIDDataBuf[1] << 8) | g_au8ToHIDDataBuf[0];
                }

                if ((g_u16I2CRxDataCnt > I2C_PREFIX_LEN) && g_u16I2CRxDataCnt == g_u16I2CRxDataLen)
                {
                    g_u16I2CRxDataCnt = 0;
                    g_u8HIDCommandProcessing = FINISHED;
                    g_u8I2CTimeOutTestflag = 0;
                    g_u16I2CTimeOutCnt = 0;
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                }
                else
                {
                    I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_SI | I2C_CTL_AA));
                }

                break;

            case 0x58: // DATA has been received and NACK has been returned
                // Do nothing, wait slave to re-send again
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;

            default:
                g_u8EnterUnknownStatus = 1;
                I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_SI | I2C_CTL_AA));
                break;
        }
    }
}

/**
 * @brief       Codec I2C Callback Function
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used as Codec I2C callback funtion
 */
void Codec_I2CCallback(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C_PORT);

    if (I2C_GET_TIMEOUT_FLAG(I2C_PORT))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(I2C_PORT);
    }
    else
    {
        switch (u32Status)
        {
            case 0x08:                               // START has been transmitted
                if (g_sHidCmdH2D.u8CommandType == 0) // Test Connection
                {
                    I2C_SET_DATA(I2C_PORT, (0x2F << 1)); // Write SLA+W to Register I2CDAT
                }
                else
                {
                    if (g_u8I2CRead)
                    {
                        g_u8I2CRead = 0;
                        I2C_SET_DATA(I2C_PORT, (g_u8SlaveAddr | 0x01)); // Write SLA+R to Register I2CDAT
                    }
                    else
                    {
                        I2C_SET_DATA(I2C_PORT, g_u8SlaveAddr); // Write SLA+W to Register I2CDAT
                    }
                }

                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;

            // Master Write
            case 0x18: // SLA+W has been transmitted and ACK has been received
                if (g_u8CheckConnection == 1)
                {
                    g_u8CheckConnection = 2;
                    I2C_STOP(I2C_PORT);
                }
                else
                {
                    g_u8I2CTimeOut = 0;
                    g_u8I2CTimeOutTestflag = 1;
                    I2C_SET_DATA(I2C_PORT, g_au8ToI2CDataBuf[g_u16I2CTxDataCnt++]);
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }

                break;

            case 0x20: // SLA+W has been transmitted and NACK has been received
                I2C_STOP(I2C_PORT);

                if (++g_u8NackCnt >= 100)
                {
                    g_u8I2CTimeOut = 1;
                    g_u8NackCnt = 0;

                    if (g_u8CheckConnection == 1)
                    {
                        g_u8CheckConnection = 2;
                    }
                }
                else
                {
                    I2C_START(I2C_PORT);
                }

                break;

            case 0x28: // DATA has been transmitted and ACK has been received
                g_u16I2CTimeOutCnt = 0;

                if (g_u16I2CTxDataCnt < g_u16I2CTxDataLen)
                {
                    I2C_SET_DATA(I2C_PORT, g_au8ToI2CDataBuf[g_u16I2CTxDataCnt++]);
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }
                else // Write data finished
                {
                    g_u16I2CTxDataCnt = 0;
                    g_u16I2CTxDataLen = 0;
                    g_u8I2CTimeOutTestflag = 0;
                    g_u16I2CTimeOutCnt = 0;

                    if (g_sHidCmdH2D.u8Header == HIDTRANS_READ)
                    {
                        // run I2C read flow
                        g_u8I2CRead = 1;
                        // Stop I2C and start again
                        I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_STO | I2C_CTL_SI | I2C_CTL_STA));
                    }
                    else // g_sHidCmdH2D.u8Header == HIDTRANS_WRITE
                    {
                        g_u8HIDCommandProcessing = FINISHED;
                        I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                    }
                }

                break;

            case 0x30: // DATA has been transmitted and NACK has been received

                // Reply write fail to PC
                if (g_u16I2CTxDataCnt >= 1)
                {
                    g_u8WriteFail = 1;

                    if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_16BIT)
                        g_u16I2CTxData_Fail_Cnt = g_u16I2CTxDataCnt - 2;
                    else if ((g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_9BIT) || (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_CODEC_BURST_DATA_8BIT))
                        g_u16I2CTxData_Fail_Cnt = g_u16I2CTxDataCnt - 1;

                    g_u16I2CTxDataCnt = 0;
                    g_u16I2CTxDataLen = 0;
                    g_u16I2CTimeOutCnt = 0;
                    g_u8I2CTimeOutTestflag = 0;
                    g_u8HIDCommandProcessing = FINISHED;
                }

                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                break;

            // Master Read, after Master Write tells the slave what to send, enter Master Read
            case 0x40: // SLA+R has been transmitted and ACK has been received
                g_u8I2CTimeOut = 0;
                g_u8I2CTimeOutTestflag = 1;

                // use NACK for end of I2C Read
                if (g_u16I2CRxDataLen <= 1)
                {
                    // NACK for end of read
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }
                else
                {
                    I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_SI | I2C_CTL_AA));
                }

                break;

            case 0x48: // SLA+R has been transmitted and NACK has been received

                // Stop I2C and start again
                if (g_sHidCmdH2D.u8CommandType == HIDTRANS_CMD_NUVOTON)
                {
                    g_u8I2CRead = 1;
                }

                I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_STO | I2C_CTL_SI | I2C_CTL_STA));
                break;

            case 0x50: // DATA has been received and ACK has been returned
                g_u16I2CTimeOutCnt = 0;
                g_au8ToHIDDataBuf[g_u16I2CRxDataCnt++] = I2C_GET_DATA(I2C_PORT);

                // use NACK for end of I2C Read
                if (g_u16I2CRxDataCnt == g_u16I2CRxDataLen - 1)
                {
                    // NACK for end of read
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }
                else
                {
                    I2C_SET_CONTROL_REG(I2C_PORT, (I2C_CTL_SI | I2C_CTL_AA));
                }

                break;

            case 0x58: // DATA has been received and NACK has been returned
                // Receive the last data
                g_au8ToHIDDataBuf[g_u16I2CRxDataCnt++] = I2C_GET_DATA(I2C_PORT);

                // For codec to stop I2C
                g_u16I2CRxDataCnt = 0;
                g_u8HIDCommandProcessing = FINISHED;
                g_u8I2CTimeOutTestflag = 0;
                g_u16I2CTimeOutCnt = 0;
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                break;

            default:
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;
        }
    }
}

/**
 * @brief       I2C IRQ Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is I2C IRQ Handler
 *
 */
void I2C3_IRQHandler(void)
{
    if (pI2CProtocolCallback != NULL)
    {
        pI2CProtocolCallback();
    }
}

/**
 * @brief       SysTick Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is SysTick Handler
 */
void SysTick_Handler(void)
{
    // Check the time interval between each data transfer to see whether timeout or not
    if (g_u8I2CTimeOutTestflag)
    {
        if (++g_u16I2CTimeOutCnt >= (uint16_t)1000) // 1s
        {
            g_u8I2CTimeOut = 1;
            g_u8I2CTimeOutTestflag = 0;
            g_u16I2CTimeOutCnt = 0;
            I2C_STOP(I2C_PORT);

            g_u8CheckConnection = 2;
        }
    }

    if (g_u8CodecInit)
    {
        g_u16CodecTimeOutCnt++;
    }

    if (g_u16PowerOnDelay < 2000)
    {
        g_u16PowerOnDelay++;
    }

    if (g_u8_7802Cntflag == 1)
    {
        g_u16_7802TimeOutCnt++;

        if (g_u16_7802TimeOutCnt >= 500)
        {
            g_u8_7802TimeOut = 1;
            g_u8_7802Cntflag = 0;
            g_u16_7802TimeOutCnt = 0;
            g_u16SampleNum = 0;
            g_u16SampleNum_cnt = 0;
            g_u8read_process = 0;
        }
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
