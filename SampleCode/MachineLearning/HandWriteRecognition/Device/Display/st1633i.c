/**************************************************************************//**
 * @file     st1633i.c
 * @version  V1.00
 * @brief    st1633i touch controller driver.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "st1633i.h"

#define I2C_PORT        I2C1
#define ST1633_SLV_ADDR 0x55

//
//Variables Declaration
//
static S_ST_REGMAP sStRegMap;
static uint8_t pre_touch = 0;
static int8_t pre_id[ST_MAX_TOUCH];
static int16_t pre_x[ST_MAX_TOUCH];
static int16_t pre_y[ST_MAX_TOUCH];
static int16_t pre_w[ST_MAX_TOUCH];


//
//Function Prototype
//
uint8_t st1633i_write_reg(uint8_t addr, uint8_t value);
uint8_t st1633i_read_reg(uint8_t addr);
uint8_t st1633i_read_multi_reg(uint8_t addr, uint8_t *data, uint8_t len);

//
// st1633i_low_level_init
//
void st1633i_low_level_init(void)
{
    //
    // Reset
    //
    GPIO_SetMode(PD, BIT10, GPIO_MODE_OUTPUT);//CTP Reset Pin(XRES)
    PD10 = 0;

    //
    // INT
    //
    GPIO_SetMode(PF, BIT6, GPIO_MODE_INPUT);
    //GPIO_SetPullCtl(PG, BIT6, GPIO_PUSEL_PULL_UP);

    //
    // I2C1
    //
    SET_I2C1_SDA_PB10();
    SET_I2C1_SCL_PB11();

    I2C_Open(I2C_PORT, 400000); //Set I2C clock 100MHz
}

//
// st1633i_startup_sequence
//
void st1633i_startup_sequence(void)
{
    PD10 = 1;
    CLK_SysTickDelay(100);
    PD10 = 0;
    CLK_SysTickDelay(100);
    PD10 = 1;
}


//
// st1633i_handler
//
void st1663i_init(void)
{
    st1633i_write_reg(0x0, 0);//migrate from RTT port
}


//
// st1633i_handler
//
void st1633i_handler(void)
{
    uint8_t info[3];
    uint16_t tp_x, tp_y;

    //
    // low active
    //
    if (PG6 == 0)
    {
        st1633i_read_multi_reg(0x12, info, 3);

        //
        // valid?
        //
        if (info[0] & 0x80)
        {
            tp_x = (uint16_t)(((info[0] & 0x70) << 4) | info[1]);
            tp_y = (uint16_t)(((info[0] & 0x07) << 8) | info[2]);

            printf("x:%d, y:%d \r\n", tp_x, tp_y);
        }
    }
}

//
// st1633i_write_reg
//
uint8_t st1633i_write_reg(uint8_t addr, uint8_t value)
{
    return I2C_WriteByteOneReg(I2C_PORT, ST1633_SLV_ADDR, addr, value);
}


//
// st1633i_read_reg
//
uint8_t st1633i_read_reg(uint8_t addr)
{
    return I2C_ReadByteOneReg(I2C_PORT, ST1633_SLV_ADDR, addr);
}

//
// st1633i_read_multi_reg
//
uint8_t st1633i_read_multi_reg(uint8_t addr, uint8_t *data, uint8_t len)
{
    return I2C_ReadMultiBytesOneReg(I2C_PORT, ST1633_SLV_ADDR, addr, data, len);
}

static void st1663i_touch_up(void *buf, int8_t id)
{
    pre_x[id] = -1;  /* last point is none */
    pre_y[id] = -1;
    pre_w[id] = -1;

}

static void st1663i_touch_down(void *buf, int8_t id, int16_t x, int16_t y, int16_t w)
{
    pre_x[id] = x; /* save last point */
    pre_y[id] = y;
    pre_w[id] = w;
}



uint8_t st1663i_read_point(S_LVGL_TPINFO *pbuf, uint8_t read_num)
{
    int i;

    uint8_t len = 0;
    int32_t   touch_event, touchid;

    len = st1633i_read_multi_reg(0x10, (uint8_t *)&sStRegMap, sizeof(sStRegMap));

    if (!len)
    {
        printf("get touch data failed, err:%d\n", len);
        goto exit_read_point;
    }

    if (sStRegMap.u8Fingers > ST_MAX_TOUCH)
    {
        printf("FW report max point:%d > panel info. max:%d\n", sStRegMap.u8Fingers, ST_MAX_TOUCH);
        goto exit_read_point;
    }

    if (pre_touch > sStRegMap.u8Fingers)               /* point up */
    {
        for (i = 0; i < ST_MAX_TOUCH; i++)
        {
            uint8_t j;

            for (j = 0; j < sStRegMap.u8Fingers; j++)  /* this time touch num */
            {
                touchid = i;
#ifdef DEBUG_TP_MSG
                printf("pre_id[%d] = %d, touchid=%d \n", i, pre_id[i], touchid);
#endif

                if (pre_id[i] == touchid)                /* this id is not free */
                    break;
            }

            if ((j == sStRegMap.u8Fingers) && (pre_id[i] != -1))         /* free this node */
            {
                printf("touch up, free %d tid=%d\n", i, pre_id[i]);
                st1663i_touch_up(NULL, pre_id[i]);
                pbuf->event_touched =  0;
                pre_id[i] = -1;
            }
        }
    }

    for (i = 0; i < sStRegMap.u8Fingers; i++)
    {
        touch_event = sStRegMap.m_sTP[i].u8Valid;/*Valid event is report by TPC*/
        touchid = i;

        //printf("(i: %d, sStRegMap.u8Fingers: %d), touchid:%d, touch_event:%d,  pre_touch:%d\n", i, sStRegMap.u8Fingers, touchid, touch_event, pre_touch);

        pre_id[i] = touchid;

        if (touch_event)
        {
            uint16_t  x, y, w;

            x = ((uint16_t)sStRegMap.m_sTP[i].u8X0_H << 8) |  sStRegMap.m_sTP[i].m_u8X0_L;
            y = ((uint16_t)sStRegMap.m_sTP[i].u8Y0_H << 8) |  sStRegMap.m_sTP[i].m_u8Y0_L;
            w = sStRegMap.m_sTP[i].m_u8Z;
#ifdef DEBUG_TP_MSG
            printf("touch down,  [touch_event:%d], (touchid:%d, x:%d, y:%d, w:%d)\n", touch_event, touchid, x, y, w);
#endif
            pbuf->pos_x = x;
            pbuf->pos_y = y;
            pbuf->event_touched = 1;

            st1663i_touch_down(NULL, touchid, x, y, w);//Report to OS

        }
        else
        {
            // Up
            st1663i_touch_up(NULL, touchid);//report to OS
            pbuf->event_touched =  0;
            printf("touch up\r\n");
        }

    } // for (i = 0; i < sStRegMap.u8TDStatus; i++)

    pre_touch = sStRegMap.u8Fingers;

    return read_num;

exit_read_point:

    pre_touch = 0;

    return 0;
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/