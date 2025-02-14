/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * @brief    CLK driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Disable clock divider output function
  * @param      None
  * @return     None
  * @details    This function disable clock divider output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK->CLKOCTL &= (~CLK_CLKOCTL_CLKOEN_Msk);
}

/**
  * @brief      This function is enable clock output module,
  *             enable clock output divider function and set frequency selection.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKOSEL_CLKOSEL_SYSCLK
  *             - \ref CLK_CLKOSEL_CLKOSEL_ACLK
  *             - \ref CLK_CLKOSEL_CLKOSEL_HCLK0
  *             - \ref CLK_CLKOSEL_CLKOSEL_HCLK1
  *             - \ref CLK_CLKOSEL_CLKOSEL_HCLK2
  *             - \ref CLK_CLKOSEL_CLKOSEL_APLL0_DIV2
  *             - \ref CLK_CLKOSEL_CLKOSEL_APLL1_DIV2
  *             - \ref CLK_CLKOSEL_CLKOSEL_HIRC48M
  *             - \ref CLK_CLKOSEL_CLKOSEL_HXT
  *             - \ref CLK_CLKOSEL_CLKOSEL_HIRC
  *             - \ref CLK_CLKOSEL_CLKOSEL_MIRC
  *             - \ref CLK_CLKOSEL_CLKOSEL_LXT
  *             - \ref CLK_CLKOSEL_CLKOSEL_LIRC
  * @param[in]  u32ClkDiv is divider output frequency selection. It could be 0~15.
  * @param[in]  u32ClkDivBy1En is clock divider setting. Including :
  *             - \ref CLK_CLKOCTL_DIV1EN_DIV_FREQSEL
  *             - \ref CLK_CLKOCTL_DIV1EN_DIV_1
  * @return     None
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv. \n
  *             The formula is: \n
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1) \n
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves. \n
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (u32ClkDiv) | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Select CKO clock source */
    CLK->CLKOSEL = (CLK->CLKOSEL & (~CLK_CLKOSEL_CLKOSEL_Msk)) | (u32ClkSrc);
}

/**
  * @brief      Get external high speed crystal clock frequency
  * @param      None
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    uint32_t u32Freq;

    if (CLK->SRCCTL & CLK_SRCCTL_HXTEN_Msk)
    {
        u32Freq = __HXT;
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Get external low speed crystal clock frequency
  * @param      None
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    uint32_t u32Freq;

    if (CLK->SRCCTL & CLK_SRCCTL_LXTEN_Msk)
    {
        u32Freq = __LXT;
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Get Internal Middle Speed RC Oscillator clock frequency
  * @param      None
  * @return     Internal Middle Speed RC Oscillator clock frequency
  * @details    This function get Internal Middle Speed RC Oscillator frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetMIRCFreq(void)
{
    uint32_t u32Freq;

    if ((CLK->SRCCTL & CLK_SRCCTL_MIRCEN_Msk) == CLK_SRCCTL_MIRCEN_Msk)
    {
        switch (CLK->MIRCCTL & CLK_MIRCCTL_MIRCFSEL_Msk)
        {
            case CLK_MIRCCTL_MIRCFSEL_1MHZ:
                u32Freq = FREQ_1MHZ;
                break;

            case CLK_MIRCCTL_MIRCFSEL_2MHZ:
                u32Freq = FREQ_2MHZ;
                break;

            case CLK_MIRCCTL_MIRCFSEL_4MHZ:
                u32Freq = FREQ_4MHZ;
                break;

            case CLK_MIRCCTL_MIRCFSEL_8MHZ:
                u32Freq = FREQ_8MHZ;
                break;

            default:
                u32Freq = __MIRC;
                break;
        }
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Disable MIRC
  * @param      None
  * @return     None
  * @details    This function disable MIRC clock. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableMIRC(void)
{
    CLK->SRCCTL &= ~CLK_SRCCTL_MIRCEN_Msk;
}

/**
  * @brief      Set MIRC frequency
  * @param[in]  u32MircFreq is MIRC clock frequency. Including :
  *             - \ref CLK_MIRCCTL_MIRCFSEL_1MHZ
  *             - \ref CLK_MIRCCTL_MIRCFSEL_2MHZ
  *             - \ref CLK_MIRCCTL_MIRCFSEL_4MHZ
  *             - \ref CLK_MIRCCTL_MIRCFSEL_8MHZ
  * @return     MIRC frequency
  * @details    This function is used to configure PWRCTL register to set specified MIRC frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnableMIRC(uint32_t u32MircFreq)
{
    /* Disable MIRC first to avoid unstable when setting MIRC */
    CLK_DisableMIRC();

    /* Select MIRC frequency */
    CLK->MIRCCTL = (CLK->MIRCCTL & ~CLK_MIRCCTL_MIRCFSEL_Msk) | u32MircFreq;

    /* Enable and apply new MIRC setting. */
    CLK->SRCCTL |= CLK_SRCCTL_MIRCEN_Msk;

    /* Wait for MIRC clock stable */
    CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);

    /* Return actual MIRC output clock frequency */
    return CLK_GetMIRCFreq();
}

/**
  * @brief      Get PCLK0 frequency
  * @param      None
  * @return     PCLK0 frequency
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t u32Freq, u32Hclk0Freq;

    u32Hclk0Freq = CLK_GetHCLK0Freq();

    u32Freq = u32Hclk0Freq / (((CLK->PCLKDIV & CLK_PCLKDIV_PCLK0DIV_Msk) >> CLK_PCLKDIV_PCLK0DIV_Pos) + 1UL);

    return u32Freq;
}

/**
  * @brief      Get PCLK1 frequency
  * @param      None
  * @return     PCLK1 frequency
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t u32Freq, u32Hclk1Freq;

    u32Hclk1Freq = CLK_GetHCLK0Freq();

    u32Freq = u32Hclk1Freq / (((CLK->PCLKDIV & CLK_PCLKDIV_PCLK1DIV_Msk) >> CLK_PCLKDIV_PCLK1DIV_Pos) + 1UL);

    return u32Freq;
}

/**
  * @brief      Get PCLK2 frequency
  * @param      None
  * @return     PCLK2 frequency
  * @details    This function get PCLK2 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK2Freq(void)
{
    uint32_t u32Freq, u32Hclk2Freq;

    u32Hclk2Freq = CLK_GetHCLK0Freq();

    u32Freq = u32Hclk2Freq / (((CLK->PCLKDIV & CLK_PCLKDIV_PCLK2DIV_Msk) >> CLK_PCLKDIV_PCLK2DIV_Pos) + 1UL);

    return u32Freq;
}

/**
  * @brief      Get PCLK3 frequency
  * @param      None
  * @return     PCLK3 frequency
  * @details    This function get PCLK3 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK3Freq(void)
{
    uint32_t u32Freq, u32Hclk3Freq;

    u32Hclk3Freq = CLK_GetHCLK0Freq();

    u32Freq = u32Hclk3Freq / (((CLK->PCLKDIV & CLK_PCLKDIV_PCLK3DIV_Msk) >> CLK_PCLKDIV_PCLK3DIV_Pos) + 1UL);

    return u32Freq;
}

/**
  * @brief      Get PCLK4 frequency
  * @param      None
  * @return     PCLK4 frequency
  * @details    This function get PCLK4 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK4Freq(void)
{
    uint32_t u32Freq, u32Hclk4Freq;

    u32Hclk4Freq = CLK_GetHCLK2Freq();

    u32Freq = u32Hclk4Freq / (((CLK->PCLKDIV & CLK_PCLKDIV_PCLK4DIV_Msk) >> CLK_PCLKDIV_PCLK4DIV_Pos) + 1UL);

    return u32Freq;
}

/**
  * @brief      Get PCLK5 frequency
  * @param      None
  * @return     PCLK5 frequency
  * @details    This function get PCLK5 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK5Freq(void)
{
    uint32_t u32Freq;

    u32Freq = CLK_GetACLKFreq();

    return u32Freq;
}

/**
  * @brief      Get SCLK frequency
  * @param      None
  * @return     SCLK frequency
  * @details    This function get SCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetSCLKFreq(void)
{
    CLK_SystemClockUpdate();

    return SystemCoreClock;
}

/**
  * @brief      Get ACLK frequency
  * @param      None
  * @return     ACLK frequency
  * @details    This function get ACLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetACLKFreq(void)
{
    SystemCoreClockUpdate();

    return SystemCoreClock;
}

/**
  * @brief      Get HCLK0 frequency
  * @param      None
  * @return     HCLK0 frequency
  * @details    This function get HCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLK0Freq(void)
{
    uint32_t u32Hclk0;

    u32Hclk0 = CLK_SystemClockUpdate();

    return u32Hclk0;
}

/**
  * @brief      Get HCLK1 frequency
  * @param      None
  * @return     HCLK1 frequency
  * @details    This function get HCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLK1Freq(void)
{
    uint32_t u32Hclk1;

    u32Hclk1 = CLK_SystemClockUpdate();

    return u32Hclk1;
}

/**
  * @brief      Get HCLK2 frequency
  * @param      None
  * @return     HCLK2 frequency
  * @details    This function get HCLK2 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLK2Freq(void)
{
    uint32_t u32SclkFreq, u32Hclk2;
    uint32_t u32Hclk2Div;

    u32SclkFreq = CLK_SystemClockUpdate();

    u32Hclk2Div = ((CLK->HCLKDIV & CLK_HCLKDIV_HCLK2DIV_Msk) >> CLK_HCLKDIV_HCLK2DIV_Pos) + 1UL;

    /* Update System Core Clock */
    u32Hclk2 = u32SclkFreq / u32Hclk2Div;

    return u32Hclk2;
}

/**
  * @brief      Set ACLK frequency
  * @param[in]  u32Aclk is ACLK frequency. The range of u32Hclk is 36MHz ~ 220MHz.
  * @return     ACLK frequency
  * @details    This function is used to set ACLK frequency by using PLL. The frequency unit is Hz. \n
  *             Power level and flash access cycle are also set according to ACLK frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Aclk)
{
    uint32_t u32HIRCSTB, u32TimeOutCount;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Check ACLK frequency range is 36MHz ~ 220MHz */
    if (u32Aclk > FREQ_220MHZ)
    {
        u32Aclk = FREQ_220MHZ;
    }
    else if (u32Aclk < FREQ_36MHZ)
    {
        u32Aclk = FREQ_36MHZ;
    }

    /* Switch SCLK clock source to HIRC clock for safe */
    CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->SCLKSEL = (CLK->SCLKSEL & (~CLK_SCLKSEL_SCLKSEL_Msk)) | CLK_SCLKSEL_SCLKSEL_HIRC;

    u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

    while (!(CLK->STATUS & CLK_STATUS_SCLKSWF_Msk))
    {
        if (u32TimeOutCount-- == 0) break;
    }

    /* Configure PLL setting if HXT clock is stable */
    if (CLK->STATUS & CLK_STATUS_HXTSTB_Msk)
    {
        u32Aclk = CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, u32Aclk, CLK_APLL0_SELECT);
    }
    /* Configure PLL setting if HXT clock is not stable */
    else
    {
        u32Aclk = CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, u32Aclk, CLK_APLL0_SELECT);

        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;
    }

    /* Select SCLK clock source to PLL,
       select ACLK clock source divider as 1,
       adjust power level, flash access cycle and update system core clock
    */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Disable HIRC if HIRC is disabled before setting core clock */
    if (u32HIRCSTB == 0UL)
    {
        CLK->SRCCTL &= ~CLK_SRCCTL_HIRCEN_Msk;
    }

    /* Return actually ACLK frequency is PLL frequency divide 1 */
    return u32Aclk;
}

/**
  * @brief      Set SCLK clock source
  * @param[in]  u32ClkSrc is SCLK clock source. Including :
  *             - \ref CLK_SCLKSEL_SCLKSEL_HIRC
  *             - \ref CLK_SCLKSEL_SCLKSEL_MIRC
  *             - \ref CLK_SCLKSEL_SCLKSEL_HIRC48M
  *             - \ref CLK_SCLKSEL_SCLKSEL_HXT
  *             - \ref CLK_SCLKSEL_SCLKSEL_APLL0
  * @return     None
  * @details    This function set SCLK clock source. \n
  *             Power level and flash access cycle are also set according to ACLK operation frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSCLK(uint32_t u32ClkSrc)
{
    uint32_t u32HIRCSTB, u32TimeOutCount, u32MaxFreq;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch to HIRC for safe. Avoid SCLK too high when applying new divider. */
    CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->SCLKSEL = (CLK->SCLKSEL & (~CLK_SCLKSEL_SCLKSEL_Msk)) | CLK_SCLKSEL_SCLKSEL_HIRC;

    u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

    while (!(CLK->STATUS & CLK_STATUS_SCLKSWF_Msk))
    {
        if (u32TimeOutCount-- == 0) break;
    }

    /* Switch to power level 0 for safe */
    PMC->PLCTL = (PMC->PLCTL & (~PMC_PLCTL_PLSEL_Msk)) | PMC_PLCTL_PLSEL_PL0;

    u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

    while (PMC->PLSTS & PMC_PLSTS_PLCBUSY_Msk)
    {
        if (u32TimeOutCount-- == 0) break;
    }

    if (u32ClkSrc == CLK_SCLKSEL_SCLKSEL_APLL0)
    {
        /* Get PLL Clock */
        PllClock = CLK_GetAPLL0ClockFreq();

        /* Assign the maximum frequency. */
        if (PllClock > FREQ_200MHZ)
            u32MaxFreq = PllClock;
        else
            u32MaxFreq = FREQ_200MHZ;

        /* Set clock with limitations */
        if (PllClock > (u32MaxFreq >> 1))
        {
            CLK_SET_HCLK2DIV(2);
            CLK_SET_PCLK0DIV(2);
            CLK_SET_PCLK1DIV(2);
            CLK_SET_PCLK2DIV(2);
            CLK_SET_PCLK3DIV(2);
            CLK_SET_PCLK4DIV(2);
        }
        else if (PllClock > (u32MaxFreq >> 2))
        {
            CLK_SET_HCLK2DIV(1);
            CLK_SET_PCLK0DIV(1);
            CLK_SET_PCLK1DIV(1);
            CLK_SET_PCLK2DIV(1);
            CLK_SET_PCLK3DIV(1);
            CLK_SET_PCLK4DIV(2);
        }
        else
        {
            CLK_SET_HCLK2DIV(1);
            CLK_SET_PCLK0DIV(1);
            CLK_SET_PCLK1DIV(1);
            CLK_SET_PCLK2DIV(1);
            CLK_SET_PCLK3DIV(1);
            CLK_SET_PCLK4DIV(1);
        }
    }

    /* Set Flash Access Cycle to 10 for safe */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (10);

    /* Switch to the new SCLK */
    CLK->SCLKSEL = (CLK->SCLKSEL & (~CLK_SCLKSEL_SCLKSEL_Msk)) | u32ClkSrc;

    u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

    while (!(CLK->STATUS & CLK_STATUS_SCLKSWF_Msk))
    {
        if (u32TimeOutCount-- == 0) break;
    }

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set power level according to the SCLK */
    if (SystemCoreClock > FREQ_200MHZ)
    {
        PMC->PLCTL = (PMC->PLCTL & (~PMC_PLCTL_PLSEL_Msk)) | PMC_PLCTL_PLSEL_PL0;
    }
    else
    {
        PMC->PLCTL = (PMC->PLCTL & (~PMC_PLCTL_PLSEL_Msk)) | PMC_PLCTL_PLSEL_PL1;
    }

    u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

    while (PMC->PLSTS & PMC_PLSTS_PLCBUSY_Msk)
    {
        if (u32TimeOutCount-- == 0) break;
    }

    /* Switch the flash access cycle to a suitable value based on SCLK */
    if (SystemCoreClock > FREQ_225MHZ)      //226~240
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (10);
    }
    else if (SystemCoreClock > FREQ_200MHZ) //201~225
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (9);
    }
    else if (SystemCoreClock > FREQ_175MHZ) //176~200
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (8);
    }
    else if (SystemCoreClock > FREQ_150MHZ) //151~175
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (7);
    }
    else if (SystemCoreClock > FREQ_125MHZ) //126~150
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (6);
    }
    else if (SystemCoreClock > FREQ_100MHZ) //101~125
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (5);
    }
    else if (SystemCoreClock > FREQ_75MHZ)  //76~100
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (4);
    }
    else if (SystemCoreClock > FREQ_50MHZ)  //51~75
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (3);
    }
    else if (SystemCoreClock > FREQ_25MHZ)  //26~50
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (2);
    }
    else /* SystemCoreClock <= FREQ_25MHZ */
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (1);
    }

    /* Disable HIRC if HIRC is disabled before switching SCLK source */
    if (u32HIRCSTB == 0UL)
    {
        CLK->SRCCTL &= ~CLK_SRCCTL_HIRCEN_Msk;
    }
}

/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u64ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index        |Clock source                              |Divider                         |
  * | :----------------- | :--------------------------------------- | :----------------------------- |
  * |\ref BPWM0_MODULE   |\ref CLK_BPWMSEL_BPWM0SEL_PCLK0           | x                              |
  * |\ref BPWM0_MODULE   |\ref CLK_BPWMSEL_BPWM0SEL_HCLK0           | x                              |
  * |\ref BPWM1_MODULE   |\ref CLK_BPWMSEL_BPWM1SEL_PCLK2           | x                              |
  * |\ref BPWM1_MODULE   |\ref CLK_BPWMSEL_BPWM1SEL_HCLK0           | x                              |
  * |\ref CANFD0_MODULE  |\ref CLK_CANFDSEL_CANFD0SEL_HXT           |\ref CLK_CANFDDIV_CANFD0DIV(x)  |
  * |\ref CANFD0_MODULE  |\ref CLK_CANFDSEL_CANFD0SEL_APLL0_DIV2    |\ref CLK_CANFDDIV_CANFD0DIV(x)  |
  * |\ref CANFD0_MODULE  |\ref CLK_CANFDSEL_CANFD0SEL_HCLK0         |\ref CLK_CANFDDIV_CANFD0DIV(x)  |
  * |\ref CANFD0_MODULE  |\ref CLK_CANFDSEL_CANFD0SEL_HIRC          |\ref CLK_CANFDDIV_CANFD0DIV(x)  |
  * |\ref CANFD0_MODULE  |\ref CLK_CANFDSEL_CANFD0SEL_HIRC48M_DIV4  |\ref CLK_CANFDDIV_CANFD0DIV(x)  |
  * |\ref CANFD1_MODULE  |\ref CLK_CANFDSEL_CANFD1SEL_HXT           |\ref CLK_CANFDDIV_CANFD1DIV(x)  |
  * |\ref CANFD1_MODULE  |\ref CLK_CANFDSEL_CANFD1SEL_APLL0_DIV2    |\ref CLK_CANFDDIV_CANFD1DIV(x)  |
  * |\ref CANFD1_MODULE  |\ref CLK_CANFDSEL_CANFD1SEL_HCLK0         |\ref CLK_CANFDDIV_CANFD1DIV(x)  |
  * |\ref CANFD1_MODULE  |\ref CLK_CANFDSEL_CANFD1SEL_HIRC          |\ref CLK_CANFDDIV_CANFD1DIV(x)  |
  * |\ref CANFD1_MODULE  |\ref CLK_CANFDSEL_CANFD1SEL_HIRC48M_DIV4  |\ref CLK_CANFDDIV_CANFD1DIV(x)  |
  * |\ref CCAP0_MODULE   |\ref CLK_CCAPSEL_CCAP0SEL_MIRC            | x                              |
  * |\ref CCAP0_MODULE   |\ref CLK_CCAPSEL_CCAP0SEL_HCLK2           | x                              |
  * |\ref CCAP0_MODULE   |\ref CLK_CCAPSEL_CCAP0SEL_HIRC            | x                              |
  * |\ref CCAP0_MODULE   |\ref CLK_CCAPSEL_CCAP0SEL_APLL0_DIV2      | x                              |
  * |\ref CCAP0_MODULE   |\ref CLK_CCAPSEL_CCAP0SEL_HXT             | x                              |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_HXT             |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_APLL1_DIV2      |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_MIRC            |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_HIRC            |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_HIRC48M         |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref DMIC0_MODULE   |\ref CLK_DMICSEL_DMIC0SEL_PCLK4           |\ref CLK_DMICDIV_DMIC0DIV(x)    |
  * |\ref EADC0_MODULE   |\ref CLK_EADCSEL_EADC0SEL_APLL1_DIV2      |\ref CLK_EADCDIV_EADC0DIV(x)    |
  * |\ref EADC0_MODULE   |\ref CLK_EADCSEL_EADC0SEL_APLL0_DIV2      |\ref CLK_EADCDIV_EADC0DIV(x)    |
  * |\ref EADC0_MODULE   |\ref CLK_EADCSEL_EADC0SEL_PCLK0           |\ref CLK_EADCDIV_EADC0DIV(x)    |
  * |\ref EPWM0_MODULE   |\ref CLK_EPWMSEL_EPWM0SEL_PCLK0           | x                              |
  * |\ref EPWM0_MODULE   |\ref CLK_EPWMSEL_EPWM0SEL_HCLK0           | x                              |
  * |\ref EPWM1_MODULE   |\ref CLK_EPWMSEL_EPWM1SEL_PCLK2           | x                              |
  * |\ref EPWM1_MODULE   |\ref CLK_EPWMSEL_EPWM1SEL_HCLK0           | x                              |
  * |\ref FMC0_MODULE    |\ref CLK_FMCSEL_FMC0SEL_HIRC              | x                              |
  * |\ref FMC0_MODULE    |\ref CLK_FMCSEL_FMC0SEL_HIRC48M_DIV4      | x                              |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_HXT               |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_APLL1_DIV2        |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_APLL0_DIV2        |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_PCLK1             |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_HIRC              |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S0_MODULE    |\ref CLK_I2SSEL_I2S0SEL_HIRC48M           |\ref CLK_I2SDIV_I2S0DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_HXT               |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_APLL1_DIV2        |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_APLL0_DIV2        |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_PCLK3             |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_HIRC              |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I2S1_MODULE    |\ref CLK_I2SSEL_I2S1SEL_HIRC48M           |\ref CLK_I2SDIV_I2S1DIV(x)      |
  * |\ref I3C0_MODULE    |\ref CLK_I3CSEL_I3C0SEL_HCLK0             | x                              |
  * |\ref I3C0_MODULE    |\ref CLK_I3CSEL_I3C0SEL_APLL1             | x                              |
  * |\ref KPI0_MODULE    |\ref CLK_KPISEL_KPI0SEL_HIRC48M_DIV4      |\ref CLK_KPIDIV_KPI0DIV(x)      |
  * |\ref KPI0_MODULE    |\ref CLK_KPISEL_KPI0SEL_HIRC              |\ref CLK_KPIDIV_KPI0DIV(x)      |
  * |\ref KPI0_MODULE    |\ref CLK_KPISEL_KPI0SEL_LIRC              |\ref CLK_KPIDIV_KPI0DIV(x)      |
  * |\ref KPI0_MODULE    |\ref CLK_KPISEL_KPI0SEL_HXT               |\ref CLK_KPIDIV_KPI0DIV(x)      |
  * |\ref LPADC0_MODULE  |\ref CLK_LPADCSEL_LPADC0SEL_PCLK4         |\ref CLK_LPADCDIV_LPADC0DIV(x)  |
  * |\ref LPADC0_MODULE  |\ref CLK_LPADCSEL_LPADC0SEL_LXT           |\ref CLK_LPADCDIV_LPADC0DIV(x)  |
  * |\ref LPADC0_MODULE  |\ref CLK_LPADCSEL_LPADC0SEL_MIRC          |\ref CLK_LPADCDIV_LPADC0DIV(x)  |
  * |\ref LPADC0_MODULE  |\ref CLK_LPADCSEL_LPADC0SEL_HIRC          |\ref CLK_LPADCDIV_LPADC0DIV(x)  |
  * |\ref LPSPI0_MODULE  |\ref CLK_LPSPISEL_LPSPI0SEL_PCLK4         | x                              |
  * |\ref LPSPI0_MODULE  |\ref CLK_LPSPISEL_LPSPI0SEL_MIRC          | x                              |
  * |\ref LPSPI0_MODULE  |\ref CLK_LPSPISEL_LPSPI0SEL_HIRC          | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_PCLK4         | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_LXT           | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_LIRC          | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_MIRC          | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_HIRC          | x                              |
  * |\ref LPTMR0_MODULE  |\ref CLK_LPTMRSEL_LPTMR0SEL_EXT           | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_PCLK4         | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_LXT           | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_LIRC          | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_MIRC          | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_HIRC          | x                              |
  * |\ref LPTMR1_MODULE  |\ref CLK_LPTMRSEL_LPTMR1SEL_EXT           | x                              |
  * |\ref LPUART0_MODULE |\ref CLK_LPUARTSEL_LPUART0SEL_PCLK4       |\ref CLK_LPUARTDIV_LPUART0DIV(x)|
  * |\ref LPUART0_MODULE |\ref CLK_LPUARTSEL_LPUART0SEL_LXT         |\ref CLK_LPUARTDIV_LPUART0DIV(x)|
  * |\ref LPUART0_MODULE |\ref CLK_LPUARTSEL_LPUART0SEL_MIRC        |\ref CLK_LPUARTDIV_LPUART0DIV(x)|
  * |\ref LPUART0_MODULE |\ref CLK_LPUARTSEL_LPUART0SEL_HIRC        |\ref CLK_LPUARTDIV_LPUART0DIV(x)|
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_LXT             |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_HXT             |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_LIRC            |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_HIRC            |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_HIRC48M_DIV4    |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_PCLK1           |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref PSIO0_MODULE   |\ref CLK_PSIOSEL_PSIO0SEL_APLL0_DIV2      |\ref CLK_PSIODIV_PSIO0DIV(x)    |
  * |\ref QSPI0_MODULE   |\ref CLK_QSPISEL_QSPI0SEL_HXT             | x                              |
  * |\ref QSPI0_MODULE   |\ref CLK_QSPISEL_QSPI0SEL_APLL0_DIV2      | x                              |
  * |\ref QSPI0_MODULE   |\ref CLK_QSPISEL_QSPI0SEL_PCLK0           | x                              |
  * |\ref QSPI0_MODULE   |\ref CLK_QSPISEL_QSPI0SEL_HIRC            | x                              |
  * |\ref QSPI0_MODULE   |\ref CLK_QSPISEL_QSPI0SEL_HIRC48M_DIV4    | x                              |
  * |\ref QSPI1_MODULE   |\ref CLK_QSPISEL_QSPI1SEL_HXT             | x                              |
  * |\ref QSPI1_MODULE   |\ref CLK_QSPISEL_QSPI1SEL_APLL0_DIV2      | x                              |
  * |\ref QSPI1_MODULE   |\ref CLK_QSPISEL_QSPI1SEL_PCLK2           | x                              |
  * |\ref QSPI1_MODULE   |\ref CLK_QSPISEL_QSPI1SEL_HIRC            | x                              |
  * |\ref QSPI1_MODULE   |\ref CLK_QSPISEL_QSPI1SEL_HIRC48M_DIV4    | x                              |
  * |\ref SC0_MODULE     |\ref CLK_SCSEL_SC0SEL_HXT                 |\ref CLK_SCDIV_SC0DIV(x)        |
  * |\ref SC0_MODULE     |\ref CLK_SCSEL_SC0SEL_APLL0_DIV2          |\ref CLK_SCDIV_SC0DIV(x)        |
  * |\ref SC0_MODULE     |\ref CLK_SCSEL_SC0SEL_PCLK1               |\ref CLK_SCDIV_SC0DIV(x)        |
  * |\ref SC0_MODULE     |\ref CLK_SCSEL_SC0SEL_HIRC                |\ref CLK_SCDIV_SC0DIV(x)        |
  * |\ref SC0_MODULE     |\ref CLK_SCSEL_SC0SEL_HIRC48M_DIV4        |\ref CLK_SCDIV_SC0DIV(x)        |
  * |\ref SC1_MODULE     |\ref CLK_SCSEL_SC1SEL_HXT                 |\ref CLK_SCDIV_SC1DIV(x)        |
  * |\ref SC1_MODULE     |\ref CLK_SCSEL_SC1SEL_APLL0_DIV2          |\ref CLK_SCDIV_SC1DIV(x)        |
  * |\ref SC1_MODULE     |\ref CLK_SCSEL_SC1SEL_PCLK3               |\ref CLK_SCDIV_SC1DIV(x)        |
  * |\ref SC1_MODULE     |\ref CLK_SCSEL_SC1SEL_HIRC                |\ref CLK_SCDIV_SC1DIV(x)        |
  * |\ref SC1_MODULE     |\ref CLK_SCSEL_SC1SEL_HIRC48M_DIV4        |\ref CLK_SCDIV_SC1DIV(x)        |
  * |\ref SC2_MODULE     |\ref CLK_SCSEL_SC2SEL_HXT                 |\ref CLK_SCDIV_SC2DIV(x)        |
  * |\ref SC2_MODULE     |\ref CLK_SCSEL_SC2SEL_APLL0_DIV2          |\ref CLK_SCDIV_SC2DIV(x)        |
  * |\ref SC2_MODULE     |\ref CLK_SCSEL_SC2SEL_PCLK1               |\ref CLK_SCDIV_SC2DIV(x)        |
  * |\ref SC2_MODULE     |\ref CLK_SCSEL_SC2SEL_HIRC                |\ref CLK_SCDIV_SC2DIV(x)        |
  * |\ref SC2_MODULE     |\ref CLK_SCSEL_SC2SEL_HIRC48M_DIV4        |\ref CLK_SCDIV_SC2DIV(x)        |
  * |\ref SDH0_MODULE    |\ref CLK_SDHSEL_SDH0SEL_HXT               |\ref CLK_SDHDIV_SDH0DIV(x)      |
  * |\ref SDH0_MODULE    |\ref CLK_SDHSEL_SDH0SEL_APLL1_DIV2        |\ref CLK_SDHDIV_SDH0DIV(x)      |
  * |\ref SDH0_MODULE    |\ref CLK_SDHSEL_SDH0SEL_HCLK0             |\ref CLK_SDHDIV_SDH0DIV(x)      |
  * |\ref SDH0_MODULE    |\ref CLK_SDHSEL_SDH0SEL_HIRC              |\ref CLK_SDHDIV_SDH0DIV(x)      |
  * |\ref SDH0_MODULE    |\ref CLK_SDHSEL_SDH0SEL_HIRC48M_DIV4      |\ref CLK_SDHDIV_SDH0DIV(x)      |
  * |\ref SDH1_MODULE    |\ref CLK_SDHSEL_SDH1SEL_HXT               |\ref CLK_SDHDIV_SDH1DIV(x)      |
  * |\ref SDH1_MODULE    |\ref CLK_SDHSEL_SDH1SEL_APLL1_DIV2        |\ref CLK_SDHDIV_SDH1DIV(x)      |
  * |\ref SDH1_MODULE    |\ref CLK_SDHSEL_SDH1SEL_HCLK0             |\ref CLK_SDHDIV_SDH1DIV(x)      |
  * |\ref SDH1_MODULE    |\ref CLK_SDHSEL_SDH1SEL_HIRC              |\ref CLK_SDHDIV_SDH1DIV(x)      |
  * |\ref SDH1_MODULE    |\ref CLK_SDHSEL_SDH1SEL_HIRC48M_DIV4      |\ref CLK_SDHDIV_SDH1DIV(x)      |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_HXT               | x                              |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_APLL1_DIV2        | x                              |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_APLL0_DIV2        | x                              |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_PCLK0             | x                              |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_HIRC              | x                              |
  * |\ref SPI0_MODULE    |\ref CLK_SPISEL_SPI0SEL_HIRC48M           | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_HXT               | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_APLL1_DIV2        | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_APLL0_DIV2        | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_PCLK2             | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_HIRC              | x                              |
  * |\ref SPI1_MODULE    |\ref CLK_SPISEL_SPI1SEL_HIRC48M           | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_HXT               | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_APLL1_DIV2        | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_APLL0_DIV2        | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_PCLK0             | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_HIRC              | x                              |
  * |\ref SPI2_MODULE    |\ref CLK_SPISEL_SPI2SEL_HIRC48M           | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_HXT               | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_APLL1_DIV2        | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_APLL0_DIV2        | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_PCLK2             | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_HIRC              | x                              |
  * |\ref SPI3_MODULE    |\ref CLK_SPISEL_SPI3SEL_HIRC48M           | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_HXT               | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_LXT               | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_PCLK1             | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_EXT               | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_LIRC              | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_HIRC              | x                              |
  * |\ref TMR0_MODULE    |\ref CLK_TMRSEL_TMR0SEL_HIRC48M_DIV4      | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_HXT               | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_LXT               | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_PCLK1             | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_EXT               | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_LIRC              | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_HIRC              | x                              |
  * |\ref TMR1_MODULE    |\ref CLK_TMRSEL_TMR1SEL_HIRC48M_DIV4      | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_HXT               | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_LXT               | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_PCLK3             | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_EXT               | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_LIRC              | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_HIRC              | x                              |
  * |\ref TMR2_MODULE    |\ref CLK_TMRSEL_TMR2SEL_HIRC48M_DIV4      | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_HXT               | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_LXT               | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_PCLK3             | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_EXT               | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_LIRC              | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_HIRC              | x                              |
  * |\ref TMR3_MODULE    |\ref CLK_TMRSEL_TMR3SEL_HIRC48M_DIV4      | x                              |
  * |\ref TTMR0_MODULE   |\ref CLK_TTMRSEL_TTMR0SEL_PCLK4           | x                              |
  * |\ref TTMR0_MODULE   |\ref CLK_TTMRSEL_TTMR0SEL_LXT             | x                              |
  * |\ref TTMR0_MODULE   |\ref CLK_TTMRSEL_TTMR0SEL_LIRC            | x                              |
  * |\ref TTMR0_MODULE   |\ref CLK_TTMRSEL_TTMR0SEL_MIRC            | x                              |
  * |\ref TTMR0_MODULE   |\ref CLK_TTMRSEL_TTMR0SEL_HIRC            | x                              |
  * |\ref TTMR1_MODULE   |\ref CLK_TTMRSEL_TTMR1SEL_PCLK4           | x                              |
  * |\ref TTMR1_MODULE   |\ref CLK_TTMRSEL_TTMR1SEL_LXT             | x                              |
  * |\ref TTMR1_MODULE   |\ref CLK_TTMRSEL_TTMR1SEL_LIRC            | x                              |
  * |\ref TTMR1_MODULE   |\ref CLK_TTMRSEL_TTMR1SEL_MIRC            | x                              |
  * |\ref TTMR1_MODULE   |\ref CLK_TTMRSEL_TTMR1SEL_HIRC            | x                              |
  * |\ref UART0_MODULE   |\ref CLK_UARTSEL0_UART0SEL_HXT            |\ref CLK_UARTDIV0_UART0DIV(x)   |
  * |\ref UART0_MODULE   |\ref CLK_UARTSEL0_UART0SEL_HIRC           |\ref CLK_UARTDIV0_UART0DIV(x)   |
  * |\ref UART0_MODULE   |\ref CLK_UARTSEL0_UART0SEL_LXT            |\ref CLK_UARTDIV0_UART0DIV(x)   |
  * |\ref UART0_MODULE   |\ref CLK_UARTSEL0_UART0SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART0DIV(x)   |
  * |\ref UART0_MODULE   |\ref CLK_UARTSEL0_UART0SEL_HIRC48M        |\ref CLK_UARTDIV0_UART0DIV(x)   |
  * |\ref UART1_MODULE   |\ref CLK_UARTSEL0_UART1SEL_HXT            |\ref CLK_UARTDIV0_UART1DIV(x)   |
  * |\ref UART1_MODULE   |\ref CLK_UARTSEL0_UART1SEL_HIRC           |\ref CLK_UARTDIV0_UART1DIV(x)   |
  * |\ref UART1_MODULE   |\ref CLK_UARTSEL0_UART1SEL_LXT            |\ref CLK_UARTDIV0_UART1DIV(x)   |
  * |\ref UART1_MODULE   |\ref CLK_UARTSEL0_UART1SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART1DIV(x)   |
  * |\ref UART1_MODULE   |\ref CLK_UARTSEL0_UART1SEL_HIRC48M        |\ref CLK_UARTDIV0_UART1DIV(x)   |
  * |\ref UART2_MODULE   |\ref CLK_UARTSEL0_UART2SEL_HXT            |\ref CLK_UARTDIV0_UART2DIV(x)   |
  * |\ref UART2_MODULE   |\ref CLK_UARTSEL0_UART2SEL_HIRC           |\ref CLK_UARTDIV0_UART2DIV(x)   |
  * |\ref UART2_MODULE   |\ref CLK_UARTSEL0_UART2SEL_LXT            |\ref CLK_UARTDIV0_UART2DIV(x)   |
  * |\ref UART2_MODULE   |\ref CLK_UARTSEL0_UART2SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART2DIV(x)   |
  * |\ref UART2_MODULE   |\ref CLK_UARTSEL0_UART2SEL_HIRC48M        |\ref CLK_UARTDIV0_UART2DIV(x)   |
  * |\ref UART3_MODULE   |\ref CLK_UARTSEL0_UART3SEL_HXT            |\ref CLK_UARTDIV0_UART3DIV(x)   |
  * |\ref UART3_MODULE   |\ref CLK_UARTSEL0_UART3SEL_HIRC           |\ref CLK_UARTDIV0_UART3DIV(x)   |
  * |\ref UART3_MODULE   |\ref CLK_UARTSEL0_UART3SEL_LXT            |\ref CLK_UARTDIV0_UART3DIV(x)   |
  * |\ref UART3_MODULE   |\ref CLK_UARTSEL0_UART3SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART3DIV(x)   |
  * |\ref UART3_MODULE   |\ref CLK_UARTSEL0_UART3SEL_HIRC48M        |\ref CLK_UARTDIV0_UART3DIV(x)   |
  * |\ref UART4_MODULE   |\ref CLK_UARTSEL0_UART4SEL_HXT            |\ref CLK_UARTDIV0_UART4DIV(x)   |
  * |\ref UART4_MODULE   |\ref CLK_UARTSEL0_UART4SEL_HIRC           |\ref CLK_UARTDIV0_UART4DIV(x)   |
  * |\ref UART4_MODULE   |\ref CLK_UARTSEL0_UART4SEL_LXT            |\ref CLK_UARTDIV0_UART4DIV(x)   |
  * |\ref UART4_MODULE   |\ref CLK_UARTSEL0_UART4SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART4DIV(x)   |
  * |\ref UART4_MODULE   |\ref CLK_UARTSEL0_UART4SEL_HIRC48M        |\ref CLK_UARTDIV0_UART4DIV(x)   |
  * |\ref UART5_MODULE   |\ref CLK_UARTSEL0_UART5SEL_HXT            |\ref CLK_UARTDIV0_UART5DIV(x)   |
  * |\ref UART5_MODULE   |\ref CLK_UARTSEL0_UART5SEL_HIRC           |\ref CLK_UARTDIV0_UART5DIV(x)   |
  * |\ref UART5_MODULE   |\ref CLK_UARTSEL0_UART5SEL_LXT            |\ref CLK_UARTDIV0_UART5DIV(x)   |
  * |\ref UART5_MODULE   |\ref CLK_UARTSEL0_UART5SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART5DIV(x)   |
  * |\ref UART5_MODULE   |\ref CLK_UARTSEL0_UART5SEL_HIRC48M        |\ref CLK_UARTDIV0_UART5DIV(x)   |
  * |\ref UART6_MODULE   |\ref CLK_UARTSEL0_UART6SEL_HXT            |\ref CLK_UARTDIV0_UART6DIV(x)   |
  * |\ref UART6_MODULE   |\ref CLK_UARTSEL0_UART6SEL_HIRC           |\ref CLK_UARTDIV0_UART6DIV(x)   |
  * |\ref UART6_MODULE   |\ref CLK_UARTSEL0_UART6SEL_LXT            |\ref CLK_UARTDIV0_UART6DIV(x)   |
  * |\ref UART6_MODULE   |\ref CLK_UARTSEL0_UART6SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART6DIV(x)   |
  * |\ref UART6_MODULE   |\ref CLK_UARTSEL0_UART6SEL_HIRC48M        |\ref CLK_UARTDIV0_UART6DIV(x)   |
  * |\ref UART7_MODULE   |\ref CLK_UARTSEL0_UART7SEL_HXT            |\ref CLK_UARTDIV0_UART7DIV(x)   |
  * |\ref UART7_MODULE   |\ref CLK_UARTSEL0_UART7SEL_HIRC           |\ref CLK_UARTDIV0_UART7DIV(x)   |
  * |\ref UART7_MODULE   |\ref CLK_UARTSEL0_UART7SEL_LXT            |\ref CLK_UARTDIV0_UART7DIV(x)   |
  * |\ref UART7_MODULE   |\ref CLK_UARTSEL0_UART7SEL_APLL0_DIV2     |\ref CLK_UARTDIV0_UART7DIV(x)   |
  * |\ref UART7_MODULE   |\ref CLK_UARTSEL0_UART7SEL_HIRC48M        |\ref CLK_UARTDIV0_UART7DIV(x)   |
  * |\ref UART8_MODULE   |\ref CLK_UARTSEL1_UART8SEL_HXT            |\ref CLK_UARTDIV1_UART8DIV(x)   |
  * |\ref UART8_MODULE   |\ref CLK_UARTSEL1_UART8SEL_HIRC           |\ref CLK_UARTDIV1_UART8DIV(x)   |
  * |\ref UART8_MODULE   |\ref CLK_UARTSEL1_UART8SEL_LXT            |\ref CLK_UARTDIV1_UART8DIV(x)   |
  * |\ref UART8_MODULE   |\ref CLK_UARTSEL1_UART8SEL_APLL0_DIV2     |\ref CLK_UARTDIV1_UART8DIV(x)   |
  * |\ref UART8_MODULE   |\ref CLK_UARTSEL1_UART8SEL_HIRC48M        |\ref CLK_UARTDIV1_UART8DIV(x)   |
  * |\ref UART9_MODULE   |\ref CLK_UARTSEL1_UART9SEL_HXT            |\ref CLK_UARTDIV1_UART9DIV(x)   |
  * |\ref UART9_MODULE   |\ref CLK_UARTSEL1_UART9SEL_HIRC           |\ref CLK_UARTDIV1_UART9DIV(x)   |
  * |\ref UART9_MODULE   |\ref CLK_UARTSEL1_UART9SEL_LXT            |\ref CLK_UARTDIV1_UART9DIV(x)   |
  * |\ref UART9_MODULE   |\ref CLK_UARTSEL1_UART9SEL_APLL0_DIV2     |\ref CLK_UARTDIV1_UART9DIV(x)   |
  * |\ref UART9_MODULE   |\ref CLK_UARTSEL1_UART9SEL_HIRC48M        |\ref CLK_UARTDIV1_UART9DIV(x)   |
  * |\ref USBD0_MODULE   |\ref CLK_USBSEL_USBSEL_HIRC48M            |\ref CLK_USBDIV_USBDIV(x)       |
  * |\ref USBD0_MODULE   |\ref CLK_USBSEL_USBSEL_APLL1_DIV2         |\ref CLK_USBDIV_USBDIV(x)       |
  * |\ref USBH0_MODULE   |\ref CLK_USBSEL_USBSEL_HIRC48M            |\ref CLK_USBDIV_USBDIV(x)       |
  * |\ref USBH0_MODULE   |\ref CLK_USBSEL_USBSEL_APLL1_DIV2         |\ref CLK_USBDIV_USBDIV(x)       |
  * |\ref WDT0_MODULE    |\ref CLK_WDTSEL_WDT0SEL_LXT               | x                              |
  * |\ref WDT0_MODULE    |\ref CLK_WDTSEL_WDT0SEL_LIRC              | x                              |
  * |\ref WDT1_MODULE    |\ref CLK_WDTSEL_WDT1SEL_LXT               | x                              |
  * |\ref WDT1_MODULE    |\ref CLK_WDTSEL_WDT1SEL_LIRC              | x                              |
  * |\ref WWDT0_MODULE   |\ref CLK_WWDTSEL_WWDT0SEL_LXT             | x                              |
  * |\ref WWDT0_MODULE   |\ref CLK_WWDTSEL_WWDT0SEL_LIRC            | x                              |
  * |\ref WWDT1_MODULE   |\ref CLK_WWDTSEL_WWDT1SEL_LXT             | x                              |
  * |\ref WWDT1_MODULE   |\ref CLK_WWDTSEL_WWDT1SEL_LIRC            | x                              |
  */
void CLK_SetModuleClock(uint64_t u64ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32Sel = 0UL, u32Div = 0UL;

    /* Configure clock source divider */
    if (MODULE_CLKDIV_Msk(u64ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32Div = (uint32_t)MODULE_CLKDIV_BASE + ((uint32_t)MODULE_CLKDIV(u64ModuleIdx) << 2);
        /* Apply new divider */
        M32(u32Div) = (M32(u32Div) & (~((uint32_t)MODULE_CLKDIV_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx)))) | u32ClkDiv;
    }

    /* Configure clock source */
    if (MODULE_CLKSEL_Msk(u64ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32Sel = (uint32_t)MODULE_CLKSEL_BASE + ((uint32_t)MODULE_CLKSEL(u64ModuleIdx) << 2);
        /* Set new clock selection setting */
        M32(u32Sel) = (M32(u32Sel) & (~((uint32_t)MODULE_CLKSEL_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx)))) | u32ClkSrc;
    }
}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_STSEL_ST0SEL_HXT
  *             - \ref CLK_STSEL_ST0SEL_HXT_DIV2
  *             - \ref CLK_STSEL_ST0SEL_ACLK_DIV2
  *             - \ref CLK_STSEL_ST0SEL_HIRC_DIV2
  * @return     None
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->STSEL = (CLK->STSEL & ~CLK_STSEL_ST0SEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_SRCCTL_HXTEN_Msk
  *             - \ref CLK_SRCCTL_LXTEN_Msk
  *             - \ref CLK_SRCCTL_MIRCEN_Msk
  *             - \ref CLK_SRCCTL_HIRCEN_Msk
  *             - \ref CLK_SRCCTL_LIRCEN_Msk
  *             - \ref CLK_SRCCTL_HIRC48MEN_Msk
  * @return     None
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->SRCCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_SRCCTL_HXTEN_Msk
  *             - \ref CLK_SRCCTL_LXTEN_Msk
  *             - \ref CLK_SRCCTL_MIRCEN_Msk
  *             - \ref CLK_SRCCTL_HIRCEN_Msk
  *             - \ref CLK_SRCCTL_LIRCEN_Msk
  *             - \ref CLK_SRCCTL_HIRC48MEN_Msk
  *             - \ref CLK_SRCCTL_APLL0EN_Msk
  *             - \ref CLK_SRCCTL_APLL1EN_Msk
  * @return     None
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->SRCCTL &= ~u32ClkMask;
}

/**
  * @brief      Enable module clock
  * @param[in]  u64ModuleIdx is module index. Including :
  *             - \ref ACMP01_MODULE
  *             - \ref ACMP23_MODULE
  *             - \ref AWF0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CANFD0_MODULE
  *             - \ref CANFD1_MODULE
  *             - \ref CCAP0_MODULE
  *             - \ref CRC0_MODULE
  *             - \ref CRYPTO0_MODULE
  *             - \ref DAC01_MODULE
  *             - \ref DMIC0_MODULE
  *             - \ref EADC0_MODULE
  *             - \ref EBI0_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref ECAP3_MODULE
  *             - \ref EMAC0_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EQEI0_MODULE
  *             - \ref EQEI1_MODULE
  *             - \ref EQEI2_MODULE
  *             - \ref EQEI3_MODULE
  *             - \ref FMC0_MODULE
  *             - \ref ISP0_MODULE
  *             - \ref GDMA0_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref GPIOG_MODULE
  *             - \ref GPIOH_MODULE
  *             - \ref GPIOI_MODULE
  *             - \ref GPIOJ_MODULE
  *             - \ref HSOTG0_MODULE
  *             - \ref HSUSBD0_MODULE
  *             - \ref HSUSBH0_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref I3C0_MODULE
  *             - \ref KDF0_MODULE
  *             - \ref KPI0_MODULE
  *             - \ref KS0_MODULE
  *             - \ref LPADC0_MODULE
  *             - \ref LPPDMA0_MODULE
  *             - \ref LPGPIO0_MODULE
  *             - \ref LPI2C0_MODULE
  *             - \ref LPSPI0_MODULE
  *             - \ref LPSRAM0_MODULE
  *             - \ref LPTMR0_MODULE
  *             - \ref LPTMR1_MODULE
  *             - \ref LPUART0_MODULE
  *             - \ref NPU0_MODULE
  *             - \ref OTFC0_MODULE
  *             - \ref OTG0_MODULE
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PSIO0_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref RTC0_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref SC2_MODULE
  *             - \ref SCU0_MODULE
  *             - \ref SDH0_MODULE
  *             - \ref SDH1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref SPIM0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref SRAM2_MODULE
  *             - \ref SRAM3_MODULE
  *             - \ref ST0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TRNG0_MODULE
  *             - \ref TTMR0_MODULE
  *             - \ref TTMR1_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref USBD0_MODULE
  *             - \ref USBH0_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref UTCPD0_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WWDT0_MODULE
  *             - \ref WWDT1_MODULE
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Get enable bit from module index */
    u32TmpVal = (1UL << (uint32_t)MODULE_CLKEN_Pos(u64ModuleIdx));
    /* Get address bit from module index */
    u32TmpAddr = (uint32_t)MODULE_CLKCTL_BASE + (uint32_t)(MODULE_CLKCTL(u64ModuleIdx) << 2);
    /* Enable module clock */
    *(volatile uint32_t *)u32TmpAddr |= u32TmpVal;
}

/**
  * @brief      Disable module clock
  * @param[in]  u64ModuleIdx is module index. Including :
  *             - \ref ACMP01_MODULE
  *             - \ref ACMP23_MODULE
  *             - \ref AWF0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CANFD0_MODULE
  *             - \ref CANFD1_MODULE
  *             - \ref CCAP0_MODULE
  *             - \ref CRC0_MODULE
  *             - \ref CRYPTO0_MODULE
  *             - \ref DAC01_MODULE
  *             - \ref DMIC0_MODULE
  *             - \ref EADC0_MODULE
  *             - \ref EBI0_MODULE
  *             - \ref ECAP0_MODULE
  *             - \ref ECAP1_MODULE
  *             - \ref ECAP2_MODULE
  *             - \ref ECAP3_MODULE
  *             - \ref EMAC0_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref EQEI0_MODULE
  *             - \ref EQEI1_MODULE
  *             - \ref EQEI2_MODULE
  *             - \ref EQEI3_MODULE
  *             - \ref FMC0_MODULE
  *             - \ref ISP0_MODULE
  *             - \ref GDMA0_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref GPIOG_MODULE
  *             - \ref GPIOH_MODULE
  *             - \ref GPIOI_MODULE
  *             - \ref GPIOJ_MODULE
  *             - \ref HSOTG0_MODULE
  *             - \ref HSUSBD0_MODULE
  *             - \ref HSUSBH0_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref I2C3_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref I3C0_MODULE
  *             - \ref KDF0_MODULE
  *             - \ref KPI0_MODULE
  *             - \ref KS0_MODULE
  *             - \ref LPADC0_MODULE
  *             - \ref LPPDMA0_MODULE
  *             - \ref LPGPIO0_MODULE
  *             - \ref LPI2C0_MODULE
  *             - \ref LPSPI0_MODULE
  *             - \ref LPSRAM0_MODULE
  *             - \ref LPTMR0_MODULE
  *             - \ref LPTMR1_MODULE
  *             - \ref LPUART0_MODULE
  *             - \ref NPU0_MODULE
  *             - \ref OTFC0_MODULE
  *             - \ref OTG0_MODULE
  *             - \ref PDMA0_MODULE
  *             - \ref PDMA1_MODULE
  *             - \ref PSIO0_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref RTC0_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref SC2_MODULE
  *             - \ref SCU0_MODULE
  *             - \ref SDH0_MODULE
  *             - \ref SDH1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref SPIM0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref SRAM2_MODULE
  *             - \ref SRAM3_MODULE
  *             - \ref ST0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TRNG0_MODULE
  *             - \ref TTMR0_MODULE
  *             - \ref TTMR1_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref USBD0_MODULE
  *             - \ref USBH0_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref UTCPD0_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WDT1_MODULE
  *             - \ref WWDT0_MODULE
  *             - \ref WWDT1_MODULE
  * @return     None
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    u32TmpVal = ~(1UL << (uint32_t)MODULE_CLKEN_Pos(u64ModuleIdx));

    u32TmpAddr = (uint32_t)MODULE_CLKCTL_BASE + (uint32_t)(MODULE_CLKCTL(u64ModuleIdx) << 2);

    *(volatile uint32_t *)u32TmpAddr &= u32TmpVal;
}

/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_APLLCTL_APLLSRC_HXT
  *             - \ref CLK_APLLCTL_APLLSRC_HXT_DIV2
  *             - \ref CLK_APLLCTL_APLLSRC_HIRC
  *             - \ref CLK_APLLCTL_APLLSRC_HIRC48_DIV4
  * @param[in]  u32PllFreq is PLL frequency. The range of u32PllFreq is 36 MHz ~ 500 MHz.
  * @param[in]  u32PllSelect is PLL selection. Including :
  *             - \ref CLK_APLL0_SELECT
  *             - \ref CLK_APLL1_SELECT
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnableAPLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq, uint32_t u32PllSelect)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32FRDIV, u32PllClk, u32StableSel;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Tmp4, u32Min, u32MinNF, u32MinNR, u32MinFRDIV, u32BestFreq;
    uint8_t au8NoTbl[4] = {1U, 2U, 2U, 4U};

    /* Disable PLL first to avoid unstable when setting PLL */
    if (u32PllSelect == CLK_APLL0_SELECT)
    {
        CLK_DisableAPLL(CLK_APLL0_SELECT);
    }
    else
    {
        CLK_DisableAPLL(CLK_APLL1_SELECT);
    }

    /* PLL source clock is from HXT */
    if (u32PllClkSrc == CLK_APLLCTL_APLLSRC_HXT)
    {
        /* Enable HXT clock */
        CLK->SRCCTL |= CLK_SRCCTL_HXTEN_Msk;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

        /* Select PLL source clock from HXT */
        u32PllSrcClk = __HXT;
    }
    /* PLL source clock is from HXT_DIV2 */
    else if (u32PllClkSrc == CLK_APLLCTL_APLLSRC_HXT_DIV2)
    {
        /* Enable HXT clock */
        CLK->SRCCTL |= CLK_SRCCTL_HXTEN_Msk;

        /* Wait for HXT clock ready */
        CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

        /* Select PLL source clock from HXT/2 */
        u32PllSrcClk = __HXT / 2;
    }
    /* PLL source clock is from HIRC */
    else if (u32PllClkSrc == CLK_APLLCTL_APLLSRC_HIRC)
    {
        /* Enable HIRC clock */
        CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;

        /* Wait for HIRC clock ready */
        CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

        /* Select PLL source clock from HIRC */
        u32PllSrcClk = __HIRC;
    }
    else /* PLL source clock is from HIRC48M */
    {
        /* Enable HIRC48M clock */
        CLK->SRCCTL |= CLK_SRCCTL_HIRC48MEN_Msk;

        /* Wait for HIRC48M clock ready */
        CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

        /* Select PLL source clock from HIRC48M/4 */
        u32PllSrcClk = __HIRC48M / 4;
    }

    /* Select APLL stable count */
    if (u32PllSrcClk == FREQ_4MHZ)
    {
        u32StableSel = CLK_APLLCTL_STBSEL_820;
    }
    else if ((u32PllSrcClk > FREQ_4MHZ) && (u32PllSrcClk <= FREQ_12MHZ))
    {
        u32StableSel = CLK_APLLCTL_STBSEL_2460;
    }
    else /* ((u32PllSrcClk > FREQ_12MHZ) && (u32PllSrcClk <= FREQ_24MHZ)) */
    {
        u32StableSel = CLK_APLLCTL_STBSEL_4920;
    }

    /* Check PLL frequency range */
    /* Constraint 1: 36MHz < FOUT < 500MHz */
    if ((u32PllFreq <= FREQ_500MHZ) && (u32PllFreq >= FREQ_36MHZ))
    {
        /* Select "NO" according to request frequency */
        if ((u32PllFreq < FREQ_72MHZ) && (u32PllFreq >= FREQ_36MHZ))
        {
            u32NO = 3UL;
        }
        else if ((u32PllFreq <= FREQ_220MHZ) && (u32PllFreq >= FREQ_72MHZ))
        {
            u32NO = 1UL;
        }
        else
        {
            u32NO = 0UL;
        }

        /* Find best solution */
        u32Min = (uint32_t) - 1;    /* initial u32Min to max value of uint32_t (0xFFFFFFFF) */
        u32MinNR = 0UL;
        u32MinNF = 0UL;
        u32MinFRDIV = 0UL;

        for (u32FRDIV = 0; u32FRDIV <= 4095; u32FRDIV++)  /* FRDIV = 0~4095 */
        {
            /* Break when get good results */
            if (u32Min == 0UL)
                break;

            for (u32NR = 1UL; u32NR <= 32UL; u32NR++)  /* max NR = 32 since NR = INDIV+1 and INDIV = 0~31 */
            {
                u32Tmp = u32PllSrcClk / u32NR;  /* FREF = FIN/NR */

                if ((u32Tmp >= FREQ_4MHZ) && (u32Tmp <= FREQ_8MHZ)) /* Constraint 2: 4MHz < FREF < 8MHz. */
                {
                    for (u32NF = 12UL; u32NF <= 255UL; u32NF++)     /* NF = 12~255 since NF = FBDIV+2 and FBDIV = 10~253 */
                    {
                        u32Tmp2 = (u32Tmp << 1) * u32NF;

                        if ((u32Tmp2 >= FREQ_144MHZ) && (u32Tmp2 <= FREQ_500MHZ))   /* Constraint 3: 144MHz < FVCO < 500MHz */
                        {
                            u32Tmp3 = u32Tmp2 / au8NoTbl[u32NO];

                            if (u32FRDIV)
                                u32Tmp3 = u32Tmp3 + ((u32Tmp >> 11) * u32FRDIV / au8NoTbl[u32NO]);

                            u32Tmp4 = u32Tmp3;

                            u32Tmp3 = (u32Tmp3 > u32PllFreq) ? u32Tmp3 - u32PllFreq : u32PllFreq - u32Tmp3;

                            if (u32Tmp3 < u32Min)
                            {
                                u32Min = u32Tmp3;
                                u32MinNR = u32NR;
                                u32MinNF = u32NF;
                                u32MinFRDIV = u32FRDIV;
                                u32BestFreq = u32Tmp4;

                                /* Break when get good results */
                                if (u32Min == 0UL)
                                    break;
                            }
                        }
                    }
                }
            }
        }

        if (u32PllSelect == CLK_APLL0_SELECT)
        {
            /* Apply new PLL0 setting. */
            CLK->APLL0CTL = (u32NO << CLK_APLL0CTL_OUTDIV_Pos) |
                            (u32StableSel) |
                            ((u32MinNR - 1UL) << CLK_APLL0CTL_INDIV_Pos) |
                            ((u32MinNF - 2UL) << CLK_APLL0CTL_FBDIV_Pos) |
                            ((u32MinFRDIV) << CLK_APLL0CTL_FRDIV_Pos);

            /* Apply PLL0 Clock Source */
            CLK->APLL0SEL = (CLK->APLL0SEL & ~CLK_APLL0SEL_APLLSRC_Msk) | u32PllClkSrc << CLK_APLL0SEL_APLLSRC_Pos;

            /* Enable PLL0 */
            CLK->SRCCTL |= CLK_SRCCTL_APLL0EN_Msk;
        }
        else
        {
            /* Apply new PLL1 setting. */
            CLK->APLL1CTL = (u32NO << CLK_APLL1CTL_OUTDIV_Pos) |
                            (u32StableSel) |
                            ((u32MinNR - 1UL) << CLK_APLL1CTL_INDIV_Pos) |
                            ((u32MinNF - 2UL) << CLK_APLL1CTL_FBDIV_Pos) |
                            ((u32MinFRDIV) << CLK_APLL1CTL_FRDIV_Pos);

            /* Apply PLL1 Clock Source */
            CLK->APLL1SEL = (CLK->APLL1SEL & ~CLK_APLL1SEL_APLLSRC_Msk) | u32PllClkSrc << CLK_APLL1SEL_APLLSRC_Pos;

            /* Enable PLL1 */
            CLK->SRCCTL |= CLK_SRCCTL_APLL1EN_Msk;
        }

        u32PllClk = u32BestFreq;
    }
    else
    {
        /* Apply default PLL setting(for 12MHz clock source) and return */
        if (u32PllSelect == CLK_APLL0_SELECT)
        {
            CLK->APLL0CTL = u32StableSel | CLK_APLLCTL_220MHz;

            /* Apply PLL0 Clock Source */
            CLK->APLL0SEL = (CLK->APLL0SEL & ~CLK_APLL0SEL_APLLSRC_Msk) | u32PllClkSrc << CLK_APLL0SEL_APLLSRC_Pos;

            /* Enable PLL0 */
            CLK->SRCCTL |= CLK_SRCCTL_APLL0EN_Msk;
        }
        else
        {
            CLK->APLL1CTL = u32StableSel | CLK_APLLCTL_220MHz;

            /* Apply PLL1 Clock Source */
            CLK->APLL1SEL = (CLK->APLL1SEL & ~CLK_APLL1SEL_APLLSRC_Msk) | u32PllClkSrc << CLK_APLL1SEL_APLLSRC_Pos;

            /* Enable PLL1 */
            CLK->SRCCTL |= CLK_SRCCTL_APLL1EN_Msk;
        }

        /* Actual PLL output clock frequency */
        u32PllClk = FREQ_220MHZ;
    }

    /* Wait for PLL clock stable */
    if (u32PllSelect == CLK_APLL0_SELECT)
    {
        CLK_WaitClockReady(CLK_STATUS_APLL0STB_Msk);
    }
    else
    {
        CLK_WaitClockReady(CLK_STATUS_APLL1STB_Msk);
    }

    /* Return actual PLL output clock frequency */
    return u32PllClk;
}

/**
  * @brief      Disable PLL
  * @param[in]  u32PllSelect is PLL selection. Including :
  *             - \ref CLK_APLL0_SELECT
  *             - \ref CLK_APLL1_SELECT
  * @return     None
  * @details    Disable PLL. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableAPLL(uint32_t u32PllSelect)
{
    if (u32PllSelect == CLK_APLL0_SELECT)
    {
        CLK->SRCCTL &= ~CLK_SRCCTL_APLL0EN_Msk;
    }
    else
    {
        CLK->SRCCTL &= ~CLK_SRCCTL_APLL1EN_Msk;
    }
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_MIRCSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_HIRC48MSTB_Msk
  *             - \ref CLK_STATUS_APLL0STB_Msk
  *             - \ref CLK_STATUS_APLL1STB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (>500ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1; /* 500ms time-out */
    uint32_t u32Ret = 1U;

    while ((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if (--u32TimeOutCnt == 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    return u32Ret;
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_MIRCSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_HIRC48MSTB_Msk
  *             - \ref CLK_STATUS_APLL0STB_Msk
  *             - \ref CLK_STATUS_APLL1STB_Msk
  * @retval     0  clock is timeout
  * @retval     1  clock is disable
  * @details    To wait for clock disable by specified clock source stable flag or timeout (>500ms)
  */
uint32_t CLK_WaitClockDisable(uint32_t u32ClkMask)
{
    uint32_t u32TimeOutCnt = SystemCoreClock >> 1;

    uint32_t u32Ret = 1U;

    while (CLK->STATUS & u32ClkMask)
    {
        if (--u32TimeOutCnt == 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    return u32Ret;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_STSEL_ST0SEL_HXT
  *             - \ref CLK_STSEL_ST0SEL_HXT_DIV2
  *             - \ref CLK_STSEL_ST0SEL_ACLK_DIV2
  *             - \ref CLK_STSEL_ST0SEL_HIRC_DIV2
  *             - \ref CLK_STSEL_ACLK
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;

    /* Set System Tick clock source */
    if (u32ClkSrc == CLK_STSEL_ACLK)
    {
        /* Select System Tick clock source from core clock */
        SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

        /* Disable System Tick clock source from external reference clock */
        CLK->STCTL &= ~CLK_STCTL_ST0CKEN_Msk;
    }
    else
    {
        /* Select System Tick clock source from external reference clock */
        SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;

        /* Enable System Tick clock source from external reference clock */
        CLK->STCTL |= CLK_STCTL_ST0CKEN_Msk;

        /* Select System Tick external reference clock source */
        CLK->STSEL = (CLK->STSEL & ~CLK_STSEL_ST0SEL_Msk) | u32ClkSrc;
    }

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0UL;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;
}

/**
  * @brief      Get PLL0 clock frequency
  * @param      None
  * @return     PLL0 frequency
  * @details    This function get PLL0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetAPLL0ClockFreq(void)
{
    uint32_t u32PllFreq = 0UL, u32PllReg, u32PllReg1;
    uint32_t u32FIN, u32NF, u32NR, u32NO, u32FRDIV;
    uint8_t au8NoTbl[4] = {1U, 2U, 2U, 4U};

    u32PllReg = CLK->APLL0CTL;
    u32PllReg1 = CLK->APLL0SEL;

    if (!(CLK->SRCCTL & CLK_SRCCTL_APLL0EN_Msk))
    {
        u32PllFreq = 0UL;       /* PLL is in power down mode or fix low */
    }
    else                        /* PLL is in normal mode */
    {
        /* PLL0 source clock */
        if ((u32PllReg1 & CLK_APLL0SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HIRC)
        {
            u32FIN = __HIRC;            /* PLL0 source clock from HIRC */
        }
        else if ((u32PllReg1 & CLK_APLL0SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HXT)
        {
            u32FIN = __HXT;             /* PLL0 source clock from HXT */
        }
        else if ((u32PllReg1 & CLK_APLL0SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HXT_DIV2)
        {
            u32FIN = __HXT / 2;         /* PLL0 source clock from HXT/2 */
        }
        else
        {
            u32FIN = __HIRC48M / 4;     /* PLL0 source clock from HIRC48M/4 */
        }

        /* Calculate PLL0 frequency */
        if (u32PllReg & CLK_APLL0CTL_BP_Msk)
        {
            u32PllFreq = u32FIN;  /* PLL0 is in bypass mode */
        }
        else
        {
            /* PLL0 is output enabled in normal work mode */
            u32NO = au8NoTbl[((u32PllReg & CLK_APLL0CTL_OUTDIV_Msk) >> CLK_APLL0CTL_OUTDIV_Pos)];
            u32NF = ((u32PllReg & CLK_APLL0CTL_FBDIV_Msk) >> CLK_APLL0CTL_FBDIV_Pos) + 2UL;
            u32NR = ((u32PllReg & CLK_APLL0CTL_INDIV_Msk) >> CLK_APLL0CTL_INDIV_Pos) + 1UL;
            u32FRDIV = ((u32PllReg & CLK_APLL0CTL_FRDIV_Msk) >> CLK_APLL0CTL_FRDIV_Pos);

            /* u32FIN is shifted 2 bits to avoid overflow */
            if (u32FRDIV)
                u32PllFreq = (((u32FIN >> 2) * (u32NF << 1)) / (u32NR * u32NO) << 2) + ((u32FIN >> 11) * u32FRDIV / (u32NR * u32NO));
            else
                u32PllFreq = (((u32FIN >> 2) * (u32NF << 1)) / (u32NR * u32NO) << 2);
        }
    }

    return u32PllFreq;
}

/**
  * @brief      Get PLL1 clock frequency
  * @param      None
  * @return     PLL1 frequency
  * @details    This function get PLL1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetAPLL1ClockFreq(void)
{
    uint32_t u32PllFreq = 0UL, u32PllReg, u32PllReg1;
    uint32_t u32FIN, u32NF, u32NR, u32NO, u32FRDIV;
    uint8_t au8NoTbl[4] = {1U, 2U, 2U, 4U};

    u32PllReg = CLK->APLL1CTL;
    u32PllReg1 = CLK->APLL1SEL;

    if (!(CLK->SRCCTL & CLK_SRCCTL_APLL1EN_Msk))
    {
        u32PllFreq = 0UL;           /* PLL1 is in power down mode or fix low */
    }
    else /* PLL1 is in normal mode */
    {
        /* PLL source clock */
        if ((u32PllReg1 & CLK_APLL1SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HIRC)
        {
            u32FIN = __HIRC;        /* PLL1 source clock from HIRC */
        }
        else if ((u32PllReg1 & CLK_APLL1SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HXT)
        {
            u32FIN = __HXT;         /* PLL1 source clock from HXT */
        }
        else if ((u32PllReg1 & CLK_APLL1SEL_APLLSRC_Msk) == CLK_APLLCTL_APLLSRC_HXT_DIV2)
        {
            u32FIN = __HXT / 2;         /* PLL1 source clock from HXT/2 */
        }
        else
        {
            u32FIN = __HIRC48M / 4;     /* PLL1 source clock from HIRC48M/4 */
        }

        /* Calculate PLL1 frequency */
        if (u32PllReg & CLK_APLL1CTL_BP_Msk)
        {
            u32PllFreq = u32FIN;  /* PLL1 is in bypass mode */
        }
        else
        {
            /* PLL1 is output enabled in normal work mode */
            u32NO = au8NoTbl[((u32PllReg & CLK_APLL1CTL_OUTDIV_Msk) >> CLK_APLL1CTL_OUTDIV_Pos)];
            u32NF = ((u32PllReg & CLK_APLL1CTL_FBDIV_Msk) >> CLK_APLL1CTL_FBDIV_Pos) + 2UL;
            u32NR = ((u32PllReg & CLK_APLL1CTL_INDIV_Msk) >> CLK_APLL1CTL_INDIV_Pos) + 1UL;
            u32FRDIV = ((u32PllReg & CLK_APLL1CTL_FRDIV_Msk) >> CLK_APLL1CTL_FRDIV_Pos);

            /* u32FIN is shifted 2 bits to avoid overflow */
            if (u32FRDIV)
                u32PllFreq = (((u32FIN >> 2) * (u32NF << 1)) / (u32NR * u32NO) << 2) + ((u32FIN >> 11) * u32FRDIV / (u32NR * u32NO));
            else
                u32PllFreq = (((u32FIN >> 2) * (u32NF << 1)) / (u32NR * u32NO) << 2);
        }
    }

    return u32PllFreq;
}

/**
  * @brief      Update system clock frequency
  * @param      None
  * @return     System clock frequency
  * @details    This function get system clock frequency. The frequency unit is Hz.
  */
uint32_t CLK_SystemClockUpdate(void)
{
    uint32_t SystemClock, u32ClkSrc, u32SclkDiv;
    uint32_t gau32ClkSrcTbl[] = {__HIRC, 0UL, __HIRC48M, __HXT, 0UL,};

    /* Update PLL Clock */
    PllClock = CLK_GetAPLL0ClockFreq();

    u32ClkSrc = CLK->SCLKSEL & CLK_SCLKSEL_SCLKSEL_Msk;

    if (u32ClkSrc == CLK_SCLKSEL_SCLKSEL_APLL0)
    {
        /* Use PLL clock */
        SystemClock = PllClock;
    }
    else if (u32ClkSrc == CLK_SCLKSEL_SCLKSEL_MIRC)
    {
        /* Use MIRC clock */
        SystemClock = CLK_GetMIRCFreq();
    }
    else
    {
        /* Use the clock sources directly */
        SystemClock = gau32ClkSrcTbl[u32ClkSrc];
    }

    /* Get SCLK divider setting */
    u32SclkDiv = ((CLK->SCLKDIV & CLK_SCLKDIV_SCLKDIV_Msk) >> CLK_SCLKDIV_SCLKDIV_Pos) + 1;

    SystemClock = SystemClock / u32SclkDiv;

    return SystemClock;
}

/**
  * @brief      Get selected module clock source
  * @param[in]  u64ModuleIdx is module index.
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CANFD0_MODULE
  *             - \ref CANFD1_MODULE
  *             - \ref CCAP0_MODULE
  *             - \ref DMIC0_MODULE
  *             - \ref EADC0_MODULE
  *             - \ref EPWM0_MODULE
  *             - \ref EPWM1_MODULE
  *             - \ref FMC0_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref KPI0_MODULE
  *             - \ref LPADC0_MODULE
  *             - \ref LPSPI0_MODULE
  *             - \ref LPTMR0_MODULE
  *             - \ref LPTMR1_MODULE
  *             - \ref LPUART0_MODULE
  *             - \ref PSIO0_MODULE
  *             - \ref QSPI0_MODULE
  *             - \ref QSPI1_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref SC2_MODULE
  *             - \ref SDH0_MODULE
  *             - \ref SDH1_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref SPI3_MODULE
  *             - \ref ST0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TTMR0_MODULE
  *             - \ref TTMR1_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref USBD0_MODULE
  *             - \ref USBH0_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WWDT0_MODULE
  *             - \ref WWDT1_MODULE
  * @return     Selected module clock source setting
  * @details    This function get selected module clock source.
  */
uint32_t CLK_GetModuleClockSource(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Get clock select control register address */
    u32TmpAddr = (uint32_t)MODULE_CLKSEL_BASE + (uint32_t)(MODULE_CLKSEL(u64ModuleIdx) << 2);

    /* Get clock source selection setting */
    u32TmpVal = (inpw((uint32_t *)u32TmpAddr) & (uint32_t)(MODULE_CLKSEL_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx))) >> (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx);

    return u32TmpVal;
}

/**
  * @brief      Get selected module clock divider number
  * @param[in]  u64ModuleIdx is module index.
  *             - \ref CANFD0_MODULE
  *             - \ref CANFD1_MODULE
  *             - \ref DMIC0_MODULE
  *             - \ref EADC0_MODULE
  *             - \ref I2S0_MODULE
  *             - \ref I2S1_MODULE
  *             - \ref KPI0_MODULE
  *             - \ref LPADC0_MODULE
  *             - \ref LPUART0_MODULE
  *             - \ref PSIO0_MODULE
  *             - \ref SC0_MODULE
  *             - \ref SC1_MODULE
  *             - \ref SC2_MODULE
  *             - \ref SDH0_MODULE
  *             - \ref SDH1_MODULE
  *             - \ref ST0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref UART5_MODULE
  *             - \ref UART6_MODULE
  *             - \ref UART7_MODULE
  *             - \ref UART8_MODULE
  *             - \ref UART9_MODULE
  *             - \ref USBD0_MODULE
  *             - \ref USBH0_MODULE
  * @return     Selected module clock divider number setting
  * @details    This function get selected module clock divider number.
  */
uint32_t CLK_GetModuleClockDivider(uint64_t u64ModuleIdx)
{
    uint32_t u32DivVal = 0UL, u32DivAddr = 0UL;

    /* Get clock divider control register address */
    u32DivAddr = (uint32_t)MODULE_CLKDIV_BASE + ((uint32_t)MODULE_CLKDIV(u64ModuleIdx) << 2);

    /* Get clock divider number setting */
    u32DivVal = (inpw((uint32_t *)u32DivAddr) & (uint32_t)(MODULE_CLKDIV_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx))) >> (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx);

    return u32DivVal;
}

/**
  * @brief      Set system and bus clock
  * @param[in]  u32SCLKSrc is SCLK clock source. Including :
  *             - \ref CLK_SCLKSEL_SCLKSEL_HIRC
  *             - \ref CLK_SCLKSEL_SCLKSEL_MIRC
  *             - \ref CLK_SCLKSEL_SCLKSEL_HIRC48M
  *             - \ref CLK_SCLKSEL_SCLKSEL_HXT
  *             - \ref CLK_SCLKSEL_SCLKSEL_APLL0
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_APLLCTL_APLLSRC_HXT
  *             - \ref CLK_APLLCTL_APLLSRC_HXT_DIV2
  *             - \ref CLK_APLLCTL_APLLSRC_HIRC
  *             - \ref CLK_APLLCTL_APLLSRC_HIRC48_DIV4
  *             u32PllClkSrc is ignored when u32SCLKSrc is not CLK_SCLKSEL_SCLKSEL_APLL0.
  * @param[in]  u32PllFreq is PLL frequency. The range of u32PllFreq is 50 MHz ~ 500 MHz.
  *             u32PllFreq is ignored when u32SCLKSrc is not CLK_SCLKSEL_SCLKSEL_APLL0.
  * @return     Current SCLK frequency
  * @details    This function is used to set the SCLK/HCLK/PCLK with clock limitations. \n
  *             The clock limitation as following below :
  *             The maximum SCLK is 220 MHz.
  *             The maximum HCLK0/HCLK1 is SCLK
  *             The maximum HCLK2 is SCLK/2
  *             The maximum PCLK0/PCLK1/PCLK2/PCLK3 is SCLK/2
  *             The maximum PCLK4 is HCLK2/2
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetBusClock(uint32_t u32SCLKSrc, uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    if (u32SCLKSrc == CLK_SCLKSEL_SCLKSEL_APLL0)
    {
        /* Select SCLK to HIRC before APLL setting*/
        CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

        /* Maximum frequency limit */
        if (u32PllFreq > FREQ_220MHZ)
            u32PllFreq = FREQ_220MHZ;

        /* Enable APLL0 clock */
        CLK_EnableAPLL(u32PllClkSrc, u32PllFreq, CLK_APLL0_SELECT);
    }
    else
    {
        if (u32SCLKSrc == CLK_SCLKSEL_SCLKSEL_MIRC)
        {
            /* Enable internal medium speed RC clock */
            CLK_EnableXtalRC(CLK_SRCCTL_MIRCEN_Msk);

            /* Waiting for internal medium speed clock ready */
            CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);
        }
        else if (u32SCLKSrc == CLK_SCLKSEL_SCLKSEL_HIRC48M)
        {
            /* Enable internal RC 48MHz clock */
            CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

            /* Waiting for internal RC 48MHz clock ready */
            CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);
        }
        else if (u32SCLKSrc == CLK_SCLKSEL_SCLKSEL_HXT)
        {
            /* Enable external high speed crystal clock */
            CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

            /* Waiting for external high speed crystal clock ready */
            CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
        }
        else
        {
            /* Enable Internal RC 12MHz clock */
            CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

            /* Waiting for Internal RC clock ready */
            CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
        }

        /* Release clock limitations */
        CLK_SET_HCLK2DIV(1);
        CLK_SET_PCLK0DIV(1);
        CLK_SET_PCLK1DIV(1);
        CLK_SET_PCLK2DIV(1);
        CLK_SET_PCLK3DIV(1);
        CLK_SET_PCLK4DIV(1);
    }

    /* Switch SCLK clock */
    CLK_SetSCLK(u32SCLKSrc);

    return SystemCoreClock;
}

/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             200MHz => 83886us, 180MHz => 93206us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the unit is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
void CLK_SysTickDelay(uint32_t us)
{
    uint32_t u32TargetValue, u32TargetInt, u32TargetRem, u32DelayCycles, SysTickValue;

    /* Systick function is using and clock source is core clock */
    if ((SysTick->CTRL & (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk)) == (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk))
    {
        u32DelayCycles = us * CyclesPerUs;

        if (u32DelayCycles > SysTick->LOAD)
        {
            /* Calculate re-load cycles with current SysTick->LOAD */
            u32TargetInt = u32DelayCycles / SysTick->LOAD;

            /* Calculate remainder delay cycles */
            u32TargetRem = u32DelayCycles % SysTick->LOAD;
        }
        else
        {
            u32TargetInt = 0;
            u32TargetRem = u32DelayCycles;
        }

        SysTickValue = SysTick->VAL;

        if (u32TargetRem > SysTickValue)
        {
            u32TargetValue = SysTick->LOAD;
            u32TargetValue = u32TargetValue - (u32TargetRem - SysTickValue);
            u32TargetInt++;
        }
        else
        {
            u32TargetValue = SysTickValue - u32TargetRem;
        }

        while (u32TargetInt > 0)
        {
            /* Wait for the countdown to reach zero */
            while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
            {
            }

            u32TargetInt--;
        }

        /* Wait for the countdown to reach the target value */
        while (SysTick->VAL > u32TargetValue)
        {
            /* Current value has been reloaded */
            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
                break;
        }
    }
    else
    {
        SysTick->LOAD = us * CyclesPerUs;
        SysTick->VAL  = 0x0UL;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
        {
        }

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;
    }
}

/**
  * @brief      This function execute long delay function.
  * @param[in]  us  Delay time.
  * @return     None
  * @details    Use the SysTick to generate the long delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t u32Delay;

    /* It should <= 65536us for each delay loop */
    u32Delay = 65536UL;

    do
    {
        if (us > u32Delay)
        {
            us -= u32Delay;
        }
        else
        {
            u32Delay = us;
            us = 0UL;
        }

        SysTick->LOAD = u32Delay * CyclesPerUs;
        SysTick->VAL  = (0x0UL);
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL);

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;
    } while (us > 0UL);
}

/** @} end of group CLK_EXPORTED_FUNCTIONS */
/** @} end of group CLK_Driver */
/** @} end of group Standard_Driver */
