/**************************************************************************//**
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

#define DBG_PRINTF(...)

/*******************************************************************************
 * Base on M55M1_UTCPD_BOARD_V1                                                *
 *                                                                             *
 *  VBUS_Source_Level --> Specified the VBUS level                             *
 *******************************************************************************/
/*******************************************************************************
 * VBUS Enable Output Active High
 * Control signal: SOURCE_DC/DC_EN (PE11)
 * The signal should be replaced by VBSRCEN
 *******************************************************************************/
static void VBUS_Enable_Output(int port)
{
    if (port == 0)
    {
        PE11 = 1;
        GPIO_SetMode(PE, BIT11, GPIO_MODE_OUTPUT);
    }
}
static void VBUS_Disable_Output(int port)
{
    if (port == 0)
    {
        PE11 = 0;
        GPIO_SetMode(PE, BIT11, GPIO_MODE_OUTPUT);
    }
}

/*******************************************************************************
 * VBUS_SRC_EN: PA2 active high                                                *
 * VBUS can be measure on JP27.1                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Source_VBus(int port)
{
    DBG_PRINTF("E SRC VBUS\n");
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SRC_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;
#endif
}
static void VBUS_CMD_Disable_Source_VBus(int port)
{
    DBG_PRINTF("D SRC VBUS\n");
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SRC_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 0;
#endif
}
/*******************************************************************************
 * VBUS_SINK_EN: PA3 active high                                               *
 * VBUS can be measure on JP27.2                                               *
 *******************************************************************************/
static void VBUS_CMD_Enable_Sink_VBus(int port)
{
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_ENABLE_SNK_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 1;
#endif
}
static void VBUS_CMD_Disable_Sink_VBus(int port)
{
#if 1
    outp32(UTCPD0_BASE + TCPC_REG_COMMAND, TCPC_REG_COMMAND_DISABLE_SNK_VBUS);
#else
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
    PA2 = 0;
#endif
}

void VBUS_Sink_Enable(int32_t port, bool bIsEnable)
{
    if (bIsEnable)
    {
        VBUS_CMD_Enable_Sink_VBus(port);
    }
    else
    {
        VBUS_CMD_Disable_Sink_VBus(port);
    }
}


void vbus_auto_discharge(uint32_t u32IsEnable)
{
    if (u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT));
}

void vbus_bleed_discharge(uint32_t u32IsEnable)
{
    if (u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_BLEED_DISCHARGE));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_BLEED_DISCHARGE));
}

void vbus_force_discharge(uint32_t u32IsEnable)
{
    if (u32IsEnable == 1)
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) | TCPC_REG_POWER_CTRL_FORCE_DISCHARGE));
    else
        outp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL, (inp32(UTCPD0_BASE + TCPC_REG_POWER_CTRL) & ~TCPC_REG_POWER_CTRL_FORCE_DISCHARGE));
}

static char i8RecLevel = 0;

void VBUS_Source_Level(int port, char i8Level)
{
    //printf("%s %d\n", __FUNCTION__, i8Level);
    VBUS_Sink_Enable(port, 0);      /* Disable VBSNNKEN pin */

    if (i8RecLevel == i8Level)
        return;

    if (i8Level  == 0)
    {
        //0V
        VBUS_Disable_Output(port);
        printf("Buck output disable\n");
        VBUS_CMD_Disable_Source_VBus(port);
        printf("VBSRCEN Disable\n");
        i8RecLevel = 0;
    }
    else if (i8Level  == 1)
    {
        //5V
        //VBUS_Disable_Output(port);
        GPIO_SetPullCtl(PE, BIT13, GPIO_PUSEL_DISABLE);
        GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(PE, BIT13, GPIO_MODE_INPUT);
        GPIO_SetMode(PE, BIT12, GPIO_MODE_INPUT);
        delay(3);
        VBUS_Enable_Output(port);
        DBG_PRINTF("Buck output 5V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 1;
    }
    else if (i8Level  == 2)
    {
        //9V
        //VBUS_Disable_Output(port);
        GPIO_SetPullCtl(PE, BIT13, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(PE, BIT13, GPIO_MODE_INPUT);

        GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_PULL_UP);
        GPIO_SetMode(PE, BIT12, GPIO_MODE_OUTPUT);
        PE12 = 0;

        VBUS_Enable_Output(port);
        DBG_PRINTF("Buck output 9V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        i8RecLevel = 2;
    }
    else if (i8Level  == 3)
    {
        //20V
        //VBUS_Disable_Output(port);
#if 1    /* Two steps adjust VBUS to 20V OK */
        GPIO_SetPullCtl(PE, BIT13, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(PE, BIT13, GPIO_MODE_INPUT);
        GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_PULL_UP);
        GPIO_SetMode(PE, BIT12, GPIO_MODE_OUTPUT);
        PE12 = 0;

        delay(3);   //if remove the delay, the protocol will generate error due to sink not to reply GoodCRC.

        GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);
        PE13 = 0;
        GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(PE, BIT12, GPIO_MODE_INPUT);
        VBUS_Enable_Output(port);
#else
        /* One step to adjust VBUS to 20V ==> Protocol error generate hard reset */
        //      GPIO_SetPullCtl(PE, BIT13, GPIO_PUSEL_DISABLE);
        //      GPIO_SetMode(PE, BIT13, GPIO_MODE_INPUT);
        //      GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_PULL_UP);
        //      GPIO_SetMode(PE, BIT12, GPIO_MODE_OUTPUT);

        //PE12 = 0;
        //delay(1); /* To avoid send GoodCrc back if rising 20V  */
        PE13 = 0;
        GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);

        GPIO_SetPullCtl(PE, BIT12, GPIO_PUSEL_DISABLE);
        GPIO_SetMode(PE, BIT12, GPIO_MODE_INPUT);
        VBUS_Enable_Output(port);
        //      VBUS_Disable_Output(port);
#endif
        DBG_PRINTF("Buck output 20V\n");
        VBUS_CMD_Enable_Source_VBus(port);
        DBG_PRINTF("VBSRCEN Enable\n");
        delay(10);
        i8RecLevel = 3;
    }
}
#if 0
void VBUS_Source_Level_Item(int port)
{
    uint8_t ch;

    VBUS_Disable_Output(port);  /* Disable VBUS output default */

    while (1)
    {
        do
        {
            DBG_PRINTF("[0] Disable VBUS Output                            \n");
            DBG_PRINTF("[1] VBUS Output 5V                                 \n");
            DBG_PRINTF("[2] VBUS Output 9V                                 \n");
            DBG_PRINTF("[3] VBUS Output 20V                                \n");
            ch = getchar();

        } while (((ch < '0') || (ch > '4')) && ((ch != 'q') && (ch != 'Q')));

        switch (ch)
        {
            case '0':
                VBUS_Source_Level(port, 0);
                break;

            case '1':
                VBUS_Source_Level(port, 1);
                break;

            case '2':
                VBUS_Source_Level(port, 2);
                break;

            case '3':
                VBUS_Source_Level(port, 3);
                break;

            case 'Q':
            case 'q':
                return;
        }
    }
}

/*******************************************************************************
 * M55M1_UTCPD BOARD V1                                                        *
 * If set VBUS_SOURCE = 20V, Even disable VB_SRC_EN                            *
 * The VBUS_S+ Still up to 15.88V                                              *
 * It seems not protect the VBUS will be = 0, if SNK device plug out           *
 *                                                                             *
 * Use VBSRC_EN to replace SOURC_DC/DC_EN signal should be better              *
 *******************************************************************************/

void VBUS_SRC_Control(int port)
{
    uint8_t ch;

    printf("Set SRC VBUS Level 20V\n");
    VBUS_Source_Level(port, 3);

    while (1)
    {
        do
        {
            DBG_PRINTF("[0] Disable Source VBUS                            \n");
            DBG_PRINTF("[1] Enable  Source VBUS                            \n");
            ch = getchar();

        } while (((ch < '0') || (ch > '1')) && ((ch != 'q') && (ch != 'Q')));

        switch (ch)
        {
            case '0':
                printf("VBUS on JP27.1 will be about 0V\n");
                VBUS_Disable_Output(port);
                VBUS_CMD_Disable_Source_VBus(port);

                /* The force discharge function will work if a Valid "Source-to-Sink connection" */
                SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(0xFFUL << (1 * 8))) | (17UL << (1 * 8));
                vbus_discharge_polarity_active_high();
                vbus_force_discharge(0);    //Force discharge bit won't be cleaned to 0 by hardware auto. Clear it first
                vbus_force_discharge(1);    //Enable force discharge

                break;

            case '1':
                printf("VBUS on JP27.1 will be about 20V\n");
                VBUS_Enable_Output(port);
                VBUS_CMD_Enable_Source_VBus(port);
                break;

            case 'Q':
            case 'q':
                return;
        }
    }
}

void VBUS_SNK_Control(void)
{
    uint8_t ch;
    int port  = 0;

    while (1)
    {
        do
        {
            DBG_PRINTF("[0] Disable Sink VBUS                            \n");
            DBG_PRINTF("[1] Enable  Sink VBUS                            \n");
            ch = getchar();

        } while (((ch < '0') || (ch > '1')) && ((ch != 'q') && (ch != 'Q')));

        switch (ch)
        {
            case '0':
                VBUS_CMD_Disable_Sink_VBus(port);
                break;

            case '1':
                VBUS_CMD_Enable_Sink_VBus(port);
                break;

            case 'Q':
            case 'q':
                return;
        }
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
// VCONN
//////////////////////////////////////////////////////////////////////////////////////////////

/* Enable/Disable Discharge Signal for UUTCPD0_DISCHG for VCONN */
void  vconn_disable_discharge(void)
{
    /* Disable VCONN discharge */
    outp32(UTCPD0_BASE + UTCPD_VCDGCTL, inp32(UTCPD0_BASE + UTCPD_VCDGCTL) & ~VCDGEN);
}
void  vconn_enable_discharge(void)
{
    /* Enable VCONN discharge */
    outp32(UTCPD0_BASE + UTCPD_VCDGCTL, inp32(UTCPD0_BASE + UTCPD_VCDGCTL) | VCDGEN);
}
void  vconn_discharge_polarity_active_low(void)
{
    /* VCONN discharge active low */
    outp32(UTCPD0_BASE + TCPC_REG_PINPL, inp32(UTCPD0_BASE + TCPC_REG_PINPL) & ~TCPC_REG_PINPL_VCDCHG);
}
void  vconn_discharge_polarity_active_high(void)
{
    /* VCONN discharge active low */
    outp32(UTCPD0_BASE + TCPC_REG_PINPL, inp32(UTCPD0_BASE + TCPC_REG_PINPL) | TCPC_REG_PINPL_VCDCHG);
}
