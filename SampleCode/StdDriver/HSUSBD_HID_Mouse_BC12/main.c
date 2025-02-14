/**************************************************************************//**
 * @file     main.c
 * @version  V!.00
 * @brief    Demonstrate how to implement a USB mouse device with BC1.2 (Battery Charging).
 *           which shows different type of charging port after connected USB port.
 *           The mouse cursor will move automatically when this mouse device connecting to PC by USB.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mouse.h"

volatile S_HSUSBD_BC12_PD_STATUS g_sChargeStatus = HSUSBD_BC12_VBUS_OFF;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

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

    /* Enable TMR0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TMR0 clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
}

/**
 * @brief Utility for BC1.2 timing use (HSUSBD_BC_Detect)
 * @param[in] us
 * @return none
 */
int32_t SysTick_Delay(TIMER_T *dummy_for_compatible __attribute__((unused)), uint32_t us)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;
    int32_t i32ErrCode;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SysTick->CTRL = 0;
    SysTick->VAL = 0x00;

    SysTick->LOAD = us * (SystemCoreClock / 1000000); /* Depend on core clock */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    i32ErrCode = 0;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            i32ErrCode = -1;
            break;
        }
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0;

    /* Lock protected registers */
    SYS_LockReg();

    return i32ErrCode;
}

/**
 * @brief BC1.2 Charge Port Detection
 * @param[in] pu32TimerSrc  NULL: timing source is SysTick timer;
 *                          TIMER0 ~ TIMER3: timing source is peripheral H/W timer
 * @return status code (S_HSUSBD_BC12_PD_STATUS)
 * @details We recommend that the H/W timer for timing counting if H/W resource is sufficient
 *          because the BSP APIs handle the setting details.
 *          User has to take cares the setting details if SysTick is used for timing counting.
 */
S_HSUSBD_BC12_PD_STATUS HSUSBD_BC_Detect(TIMER_T *pu32TimerSrc)
{
    /* TDCD_TIMEOUT (BC1.2 SPEC): 300ms ~ 900ms */
#define DCD_TIMEOUT_PERIOD_US 349000UL

#define ENABLE_BC12_DBG_MSG 0
#if ENABLE_BC12_DBG_MSG
#define DBG_MSG printf
#else
#define DBG_MSG(...)
#endif

#define BC_DELAY(us) pfnBC_Delay(pu32TimerSrc, us)

    int32_t (*pfnBC_Delay)(TIMER_T * dummy, uint32_t us);

    if (pu32TimerSrc == NULL)
    {
        pfnBC_Delay = SysTick_Delay;
    }
    else if ((pu32TimerSrc == TIMER0) ||
             (pu32TimerSrc == TIMER1) ||
             (pu32TimerSrc == TIMER2) ||
             (pu32TimerSrc == TIMER3)
            )
    {
        pfnBC_Delay = TIMER_Delay;
    }
    else
    {
        DBG_MSG("Invalid delay timer source.\n");
        return HSUSBD_BC12_ERROR; // Invalid delay timer source
    }

    HSUSBD_ENABLE_PHY();

    while (!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk));

    if (HSUSBD_IS_ATTACHED() == 0)
    {
        HSUSBD->BCDC = 0;
        return HSUSBD_BC12_VBUS_OFF;
    }
    else
    {
        BC_DELAY(1000); // 1ms
        DBG_MSG("VBUS Plug\n");
    }

    HSUSBD_DISABLE_PHY();

    DBG_MSG("Check VBUS threshold voltage");
    HSUSBD->BCDC = HSUSBD_BCDC_BCDEN_Msk;

    BC_DELAY(30000); // 30 ms: wait PHY LDO stable
    HSUSBD->BCDC |= HSUSBD_BCDC_DETMOD_VBUS;

    while ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) == HSUSBD_BCDC_DETSTS_VBUS_UNREACH) {}

    DBG_MSG("\nCheck VBUS OK\n");
    DBG_MSG("Check data pin contact status\n");
    HSUSBD->BCDC = HSUSBD_BCDC_BCDEN_Msk | HSUSBD_BCDC_DETMOD_DCD;

    if (pu32TimerSrc == NULL)  /* Use SysTick timer */
    {
DCD_REPEAT_SYSTICK:
        SYS_UnlockReg();
        SysTick->CTRL = 0;
        SysTick->VAL = (0x00);

        SysTick->LOAD = DCD_TIMEOUT_PERIOD_US * (SystemCoreClock / 1000000);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while (1)
        {
            /* Using S/W debounce, TDCD_DBNC is 10 ms totally */
            if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) == HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
            {
                BC_DELAY(5000);

                if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(4000);

                if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(1000);

                if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    goto DCD_REPEAT_SYSTICK;
                }

                BC_DELAY(10000);
                DBG_MSG(" - DCD Data Contact\n");
                SYS_LockReg();
                break;
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                SysTick->CTRL = 0;
                SYS_LockReg();
                DBG_MSG(" - Timeout\n");
                break;
            }
        }
    }
    else   /* Use hardware timer */
    {
DCD_REPEAT_TIMER:
        {
            uint32_t u32Usec = DCD_TIMEOUT_PERIOD_US;
            uint32_t u32Clk = TIMER_GetModuleClock(pu32TimerSrc);
            uint32_t u32Prescale = 0UL, u32Delay = (SystemCoreClock / u32Clk) + 1UL;
            uint32_t u32Cmpr, u32NsecPerTick;

            /* Clear current timer configuration */
            pu32TimerSrc->CTL = 0UL;
            pu32TimerSrc->EXTCTL = 0UL;

            if (u32Clk <= 1000000UL) // Minimin delay is 1000 us if timer clock source is <= 1 MHz
            {
                if (u32Usec < 1000UL)
                    u32Usec = 1000UL;

                if (u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }
            else
            {
                if (u32Usec < 100UL)
                    u32Usec = 100UL;

                if (u32Usec > 1000000UL)
                    u32Usec = 1000000UL;
            }

            if (u32Clk <= 1000000UL)
            {
                u32Prescale = 0UL;
                u32NsecPerTick = 1000000000UL / u32Clk;
                u32Cmpr = (u32Usec * 1000UL) / u32NsecPerTick;
            }
            else
            {
                u32Cmpr = u32Usec * (u32Clk / 1000000UL);
                u32Prescale = (u32Cmpr >> 24); /* for 24 bits CMPDAT */

                if (u32Prescale > 0UL)
                    u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
            }

            pu32TimerSrc->CMP = u32Cmpr;
            pu32TimerSrc->CTL = TIMER_CTL_CNTEN_Msk | TIMER_ONESHOT_MODE | u32Prescale;

            // When system clock is faster than timer clock, it is possible timer active
            // bit cannot set in time while we check it. And the while loop below return
            // immediately, so put a tiny delay here allowing timer start counting and
            // raise active flag.
            for (; u32Delay > 0UL; u32Delay--)
            {
                __NOP();
            }

            while (1)
            {
                /* Using S/W debounce, TDCD_DBNC is 10 ms totally */
                if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) == HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                {
                    BC_DELAY(5000);

                    if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(2000);

                    if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    BC_DELAY(3000);

                    if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) != HSUSBD_BCDC_DETSTS_DCD_DATA_CONTACT)
                    {
                        goto DCD_REPEAT_TIMER;
                    }

                    DBG_MSG(" - DCD Data Contact\n");
                    break;
                }

                if (!(pu32TimerSrc->CTL & TIMER_CTL_ACTSTS_Msk))
                {
                    DBG_MSG(" - DCD Timeout\n");
                    break;
                }
            }
        }
    }

    HSUSBD->BCDC = HSUSBD_BCDC_BCDEN_Msk | HSUSBD_BCDC_DETMOD_PD;
    DBG_MSG("BC1.2 - Primary Detect: ");

    /* Delay 40ms */
    BC_DELAY(40000); // SPEC: TVDPSRC_ON
    BC_DELAY(1000);  // tune

    if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) == HSUSBD_BCDC_DETSTS_PD_SDP_NUSP)
    {
        HSUSBD->BCDC = 0; //important: to prevent next loop un-expected pulse
        return HSUSBD_BC12_SDP;
    }
    else
    {
        /* Port detect - Secondary detect */
        DBG_MSG("BC1.2 - Secondary Detect: ");
        HSUSBD->BCDC = HSUSBD_BCDC_BCDEN_Msk | HSUSBD_BCDC_DETMOD_SD;
        BC_DELAY(40000); // SPEC: TVDMSRC_ON
        BC_DELAY(5000);  // tune

        if ((HSUSBD->BCDC & HSUSBD_BCDC_DETSTS_Msk) == HSUSBD_BCDC_DETSTS_SD_CDP)
        {
            DBG_MSG("* CDP\n");
            HSUSBD->BCDC = 0; // important: to prevent next loop un-expected pulse
            return HSUSBD_BC12_CDP;
        }
        else
        {
            DBG_MSG("* DCP\n");
            HSUSBD->BCDC = 0; // important: to prevent next loop un-expected pulse
            return HSUSBD_BC12_DCP;
        }
    }

#undef ENABLE_BC12_DBG_MSG
#undef DBG_MSG
#undef DCD_TIMEOUT_PERIOD_US
#undef BC_DELAY
}

void PowerDown(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    HSUSBD->PHYCTL |= HSUSBD_PHYCTL_VBUSWKEN_Msk | HSUSBD_PHYCTL_LINESTATEWKEN_Msk;

    PMC_PowerDown();

    g_u8Suspend = 0;
    HSUSBD_ENABLE_USB();
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
        if (--u32TimeOutCnt == 0) break;

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("NuMicro HSUSBD HID with BC1.2\n");

restart:

    while (1)
    {
        g_sChargeStatus = HSUSBD_BC_Detect(TIMER0);

        if (HSUSBD_BC12_SDP == g_sChargeStatus)
        {
            printf("==>is SDP\n");
            break;
        }

        if (HSUSBD_BC12_CDP == g_sChargeStatus)
        {
            printf("==>is CDP\n");
            break;
        }

        if (HSUSBD_BC12_DCP == g_sChargeStatus)
        {
            printf("==>is DCP\n");
            break;
        }

        if (HSUSBD_BC12_VBUS_OFF == g_sChargeStatus)
        {
            continue;
        }

        if (HSUSBD_BC12_ERROR == g_sChargeStatus)
        {
            printf("parameter error\n");
            goto lexit;
        }
    }

    HSUSBD->BCDC = 0;

    HSUSBD_Open(&gsHSInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(HSUSBD_IRQn);

    /* Start transaction */
    HSUSBD_Start();

    while (1)
    {
        /* Enter power down when USB suspend */
        if (g_u8Suspend)
            PowerDown();

        if (HSUSBD_IS_ATTACHED() == FALSE)
        {
            printf("VBUS Un-Plug\n");
            HSUSBD_SET_SE0();
            goto restart;
        }

        HID_UpdateMouseData();
    }

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
