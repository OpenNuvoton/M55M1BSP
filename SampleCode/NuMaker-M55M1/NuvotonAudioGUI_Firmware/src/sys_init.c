/******************************************************************************
 * @file     sys_init.c
 * @version  V1.00
 * @brief    System clock configuration source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "sysClk_config.h"
#include "sys_init.h"
#include "user_config.h"

/**
 * @brief       SYS Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Initialize the system clock, multi-function I/O and module clock.
 */
uint32_t SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (SYSCLK_HXT != SYSCLK_HXT_DISABLE)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
#endif

    /* Switch SCLK clock source to APLL0 and Enable APLL0 192MHz clock */
    /* The system frequency needs to be set to a multiple of the 48Khz sampling rate, for example, it can be set to 192 MHz.*/
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_192MHZ);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C3_MODULE);

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);

    /* Select HSOTG PHY Reference clock frequency which is from HXT */
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    return CLK_GetSCLKFreq();
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
