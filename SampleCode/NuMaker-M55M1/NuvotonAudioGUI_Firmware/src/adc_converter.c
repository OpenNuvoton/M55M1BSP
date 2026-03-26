/******************************************************************************
 * @file     adc_converter.c
 * @version  V1.00
 * @brief    NAU7802 ADC converter source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "usbd_audio.h"
#include "user_config.h"
#include "i2c_process.h"
#include "adc_converter.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern HIDTRANS_CMD_T g_sHidCmdH2D;     // Host to device
extern HIDTRANS_CMD_T g_sHidCmdD2H;     // Device to host
extern volatile uint8_t g_u8Qurey_mode;
extern volatile uint8_t g_u8Channel_mode;
extern volatile uint8_t g_u8Sample_Num_H;
extern volatile uint8_t g_u8Sample_Num_L;
extern volatile uint8_t g_u8T1_delay_H;
extern volatile uint8_t g_u8T1_delay_L;
extern volatile uint8_t g_u8T2_delay_H;
extern volatile uint8_t g_u8T2_delay_L;
extern volatile uint16_t g_u16SampleNum_cnt;
extern uint8_t g_au8ADCDataBuff[6144];  // max 1024(sample) * 3(byte) * 2 (ch)
extern uint8_t g_au8ToI2CDataBuf[];     // Data from PC
extern uint8_t g_au8ToHIDDataBuf[];     // Data to PC
volatile uint16_t g_u16SampleNum = 0;
volatile uint8_t g_u8_7802Cntflag = 0;
volatile uint8_t g_u8_7802TimeOut = 0;
volatile uint16_t g_u16_7802TimeOutCnt = 0;
volatile uint8_t g_u8_7802Singal_last_ch = 0;

/**
 * @brief       NAU7802_Conversion_Result
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     NAU7802 ADC read conversion result.
 *
 */
void NAU7802_Conversion_Result(void)
{
    g_sHidCmdH2D.u8Header = HIDTRANS_READ; // restore original state
    g_u8SlaveAddr = 0x54;
    g_u16I2CTxDataLen = 1; // Write 1 byte register
    g_u16I2CRxDataLen = 3; // Read data length is 3
    g_au8ToI2CDataBuf[0] = 0x12;
    pI2CProtocolCallback = Codec_I2CCallback;
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100); // wait I2C data write finish;
}

/**
 * @brief       NAU7802_Read_Ch
 *
 * @param[in]   None
 *
 * @return      channel value
 *
 * @details     NAU7802 ADC read channel.
 *
 */
int32_t NAU7802_Read_Ch(void)
{
    uint8_t Reg_Value;

    g_sHidCmdH2D.u8Header = HIDTRANS_READ; // restore original state
    g_u8SlaveAddr = 0x54;
    g_u16I2CTxDataLen = 1; // Write 1 byte register
    g_u16I2CRxDataLen = 1; // Read data length is 3
    g_au8ToI2CDataBuf[0] = 0x02;
    pI2CProtocolCallback = Codec_I2CCallback;
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100); // wait I2C data write finish
    Reg_Value = g_au8ToHIDDataBuf[0];
    return Reg_Value;
}

/**
 * @brief       NAU7802_Write_Ch
 *
 * @param[in]   ch    channel 0 or 1
 *
 * @return      None
 *
 * @details     NAU7802 ADC write channel.
 *
 */
void NAU7802_Write_Ch(uint8_t ch)
{
    g_sHidCmdH2D.u8Header = HIDTRANS_WRITE;
    g_u16I2CTxDataLen = 2; // register 1 byte and data 1 byte
    g_au8ToI2CDataBuf[0] = 0x02;

    if ((ch & 0x80) == 0x80)
        g_au8ToI2CDataBuf[1] = ch & 0x7F; // switch ch 0
    else
        g_au8ToI2CDataBuf[1] = ch | 0x80; // switch ch 1

    pI2CProtocolCallback = Codec_I2CCallback;

    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100); // wait I2C data write finish
}

/**
 * @brief       NAU7802_Switch_Ch
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     NAU7802 ADC switch channel.
 *
 */
void NAU7802_Switch_Ch()
{
    uint8_t Reg_Value;

    Reg_Value = NAU7802_Read_Ch();

    NAU7802_Write_Ch(Reg_Value);
}

/**
 * @brief       NAU7802_Switch_CS
 *
 * @param[in]   cs    0:cycle start 1:cycle stop
 *
 * @return      None
 *
 * @details     NAU7802 ADC switch CS mode.
 *
 */
void NAU7802_Switch_CS(uint8_t cs)
{
    uint8_t Reg_Value;

    (void)cs;

    g_sHidCmdH2D.u8Header = HIDTRANS_READ; // restore original state
    g_u8SlaveAddr = 0x54;
    g_u16I2CTxDataLen = 1; // Write 1 byte register
    g_u16I2CRxDataLen = 1; // Read data length is 3
    g_au8ToI2CDataBuf[0] = 0x00;
    pI2CProtocolCallback = Codec_I2CCallback;
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100); // wait I2C data write finish
    Reg_Value = g_au8ToHIDDataBuf[0];

    g_sHidCmdH2D.u8Header = HIDTRANS_WRITE;
    g_u16I2CTxDataLen = 2; // register 1 byte and data 1 byte
    g_au8ToI2CDataBuf[0] = 0x00;

    if ((Reg_Value & 0x10) == 0x10)
        g_au8ToI2CDataBuf[1] = Reg_Value & 0xEF; // Cycle start
    else
        g_au8ToI2CDataBuf[1] = Reg_Value | 0x10; // Cycle start

    pI2CProtocolCallback = Codec_I2CCallback;
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100); // wait I2C data write finish
}

/**
 * @brief       NAU7802_Check_CR
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     NAU7802 ADC check conversion ready.
 *
 */
int32_t NAU7802_Check_CR(void)
{
    g_sHidCmdH2D.u8Header = HIDTRANS_WRITE;
    g_u8SlaveAddr = 0x54;
    g_u16I2CTxDataLen = 1; // Write 1 byte register
    g_u16I2CRxDataLen = 1; // Read data length is 3
    g_au8ToI2CDataBuf[0] = 0x00;
    pI2CProtocolCallback = Codec_I2CCallback;
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STA); // Start I2C transmission

    while ((g_u16I2CTxDataCnt != g_u16I2CTxDataLen)) {}

    DelayUs(100);

    if ((g_au8ToHIDDataBuf[0] & 0x20) == 0x20)
        return 0;
    else
        return -1;
}

/**
 * @brief       NAU7802_Polling
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     NAU7802 ADC read data by polling mode.
 *
 */
void NAU7802_Polling(void)
{
    g_u8_7802Cntflag = 1;

    if (g_u8Channel_mode == ADC_Single)
    {
        while ((PD4 == 0) && (g_u8_7802TimeOut == 0)) {} // wait DRDY low to high & wait 500ms

        if (g_u8_7802TimeOut)
            return;
        else
            g_u16_7802TimeOutCnt = 0;

        NAU7802_Conversion_Result();

        if (g_u16SampleNum_cnt == 0)
            DelayUs(100);

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 0] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 1] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 2] = g_au8ToHIDDataBuf[2];
    }
    else if (g_u8Channel_mode == ADC_Dual)
    {
        // CH1
        while ((PD4 == 0) && (g_u8_7802TimeOut == 0)) {} // wait DRDY low to high & wait 500ms

        if (g_u8_7802TimeOut)
            return;
        else
            g_u16_7802TimeOutCnt = 0;

        if (g_u16SampleNum_cnt == 0)
        {
            g_u8_7802Singal_last_ch = NAU7802_Read_Ch();

            if ((g_u8_7802Singal_last_ch & 0x80) == 0x80) // check is ch2, yes is changed to ch1.
            {
                NAU7802_Write_Ch(g_u8_7802Singal_last_ch);
                DelayUs(1000);

                // Switch to CS state
                NAU7802_Switch_CS(1);
                DelayUs(5000);
                NAU7802_Switch_CS(0);
            }
        }

        NAU7802_Conversion_Result();
        DelayUs(1000);

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 0] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 1] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 2] = g_au8ToHIDDataBuf[2];

        // Switch to channel 2
        NAU7802_Switch_Ch();
        DelayUs(1000);

        // Switch to CS state
        NAU7802_Switch_CS(1);
        DelayUs(5000);
        NAU7802_Switch_CS(0);

        // CH2
        while ((PD4 == 0) && (g_u8_7802TimeOut == 0)) {} // wait DRDY low to high & wait 500ms

        if (g_u8_7802TimeOut)
            return;
        else
            g_u16_7802TimeOutCnt = 0;

        NAU7802_Conversion_Result();
        DelayUs(1000);

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 3] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 4] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 5] = g_au8ToHIDDataBuf[2];

        // Switch to channel 1
        NAU7802_Switch_Ch();
        DelayUs(1000);
        // Switch to CS state
        NAU7802_Switch_CS(1);
        DelayUs(5000);
        NAU7802_Switch_CS(0);

        if (((g_u8_7802Singal_last_ch & 0x80) == 0x80) && (g_u16SampleNum_cnt == (g_u16SampleNum - 1)))
        {
            NAU7802_Write_Ch(g_u8_7802Singal_last_ch & 0x7F); // Revert back to ch2.
            g_u8_7802Singal_last_ch = 0;
        }

        g_sHidCmdH2D.u8Header = HIDTRANS_READ; // restore original state
    }

    g_u8_7802Cntflag = 0;
}

/**
 * @brief       NAU7802_Time_Delay
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     NAU7802 ADC read data by time delay mode.
 *
 */
void NAU7802_Time_Delay(void)
{
    uint32_t i = 0, u32T1_delay_cnt = 0, u32T2_delay_cnt = 0;

    if (g_u8Channel_mode == ADC_Single)
    {
        u32T1_delay_cnt = (g_u8T1_delay_H << 8 | g_u8T1_delay_L);

        NAU7802_Conversion_Result();

        // while((PD4==0));    // wait DRDY low to high
        for (i = 0; i < 1000; i++) // us * 1000 =1 ms
        {
            DelayUs(u32T1_delay_cnt);
        }

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 0] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 1] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 3 + 2] = g_au8ToHIDDataBuf[2];
    }
    else if (g_u8Channel_mode == ADC_Dual)
    {
        u32T1_delay_cnt = (g_u8T1_delay_H << 8 | g_u8T1_delay_L);
        u32T2_delay_cnt = (g_u8T2_delay_H << 8 | g_u8T2_delay_L);

        if (g_u16SampleNum_cnt == 0)
        {
            g_u8_7802Singal_last_ch = NAU7802_Read_Ch();

            if ((g_u8_7802Singal_last_ch & 0x80) == 0x80) // check is ch2, yes is changed to ch1.
            {
                NAU7802_Write_Ch(g_u8_7802Singal_last_ch);

                for (i = 0; i < 1000; i++)
                {
                    DelayUs(u32T2_delay_cnt);
                }
            }
        }

        // CH1
        NAU7802_Conversion_Result();

        // while((PD4==0));    // wait DRDY low to high
        for (i = 0; i < 1000; i++)
        {
            DelayUs(u32T1_delay_cnt);
        }

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 0] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 1] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 2] = g_au8ToHIDDataBuf[2];

        // Switch to channel 2
        NAU7802_Switch_Ch();

        for (i = 0; i < 1000; i++)
        {
            DelayUs(u32T2_delay_cnt);
        }

        // CH2
        NAU7802_Conversion_Result();

        // while((PD4==0));    // wait DRDY low to high
        for (i = 0; i < 1000; i++)
        {
            DelayUs(u32T1_delay_cnt);
        }

        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 3] = g_au8ToHIDDataBuf[0];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 4] = g_au8ToHIDDataBuf[1];
        g_au8ADCDataBuff[g_u16SampleNum_cnt * 6 + 5] = g_au8ToHIDDataBuf[2];

        // Switch to channel 1
        NAU7802_Switch_Ch();

        for (i = 0; i < 1000; i++)
        {
            DelayUs(u32T2_delay_cnt);
        }

        if (((g_u8_7802Singal_last_ch & 0x80) == 0x80) && (g_u16SampleNum_cnt == (g_u16SampleNum - 1)))
        {
            NAU7802_Write_Ch(g_u8_7802Singal_last_ch & 0x7F); // Revert back to ch2.
            g_u8_7802Singal_last_ch = 0;
        }

        g_sHidCmdH2D.u8Header = HIDTRANS_READ; // restore original state
    }
}
/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
