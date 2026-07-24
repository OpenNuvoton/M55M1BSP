/**************************************************************************//**
 * @file     lpspi.c
 * @version  V1.00
 * @brief    LPSPI driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

#define LPSPI_CONFIG_TIMEOUT    (0x100000UL)

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPSPI_Driver LPSPI Driver
  @{
*/

/** @addtogroup LPSPI_EXPORTED_FUNCTIONS LPSPI Exported Functions
  @{
*/
static void LPSPI_WaitDisableReady(const LPSPI_T *lpspi)
{
    uint32_t u32Timeout = LPSPI_CONFIG_TIMEOUT;

    while ((((lpspi->STATUS & LPSPI_STATUS_SPIENSTS_Msk) != 0U) || ((lpspi->STATUS & LPSPI_STATUS_BUSY_Msk) != 0U)) && (u32Timeout != 0U))
    {
        u32Timeout--;
    }
}

static uint32_t LPSPI_EnterConfigMode(LPSPI_T *lpspi)
{
    uint32_t u32WasEnabled = lpspi->CTL & LPSPI_CTL_SPIEN_Msk;

    if (u32WasEnabled != 0U)
    {
        lpspi->CTL &= ~LPSPI_CTL_SPIEN_Msk;
        LPSPI_WaitDisableReady(lpspi);
    }

    return u32WasEnabled;
}

static void LPSPI_ExitConfigMode(LPSPI_T *lpspi, uint32_t u32WasEnabled)
{
    if (u32WasEnabled != 0U)
    {
        lpspi->CTL |= LPSPI_CTL_SPIEN_Msk;
    }
}

static void LPSPI_SetPCLKSrc(const LPSPI_T *lpspi)
{
    /* Select PCLK as the clock source of LPSPI */
    if (lpspi == LPSPI0)
    {
        CLK->LPSPISEL = (CLK->LPSPISEL & (~CLK_LPSPISEL_LPSPI0SEL_Msk)) | CLK_LPSPISEL_LPSPI0SEL_PCLK4;
    }
}

/**
 * @brief Select PCLK as the clock source of LPSPI
 *
 * @return Actual frequency of LPSPI bus clock.
 */
static uint32_t LPSPI_GetPCLKSrcFreq(const LPSPI_T *lpspi)
{
    uint32_t u32RetValue = 0;

    /* Select PCLK as the clock source of LPSPI */
    if (lpspi == LPSPI0)
    {
        /* Return slave peripheral clock rate */
        u32RetValue = CLK_GetPCLK4Freq();
    }

    return u32RetValue;
}

/**
 * @brief Get LPSPI Clock Source Frequency.
 *
 * @param lpspi     The pointer of the specified LPSPI module.
 * @return uint32_t Clock Frequency
 */
static uint32_t LPSPI_GetModuleClkSrcFreq(const LPSPI_T *lpspi)
{
    uint32_t u32LPSPIClkSrcSel = 0UL;
    uint32_t u32RetValue = 0UL;

    /* Get LPSPI clock source selection */
    if (lpspi == LPSPI0)
    {
        u32LPSPIClkSrcSel = ((CLK->LPSPISEL & CLK_LPSPISEL_LPSPI0SEL_Msk) >> CLK_LPSPISEL_LPSPI0SEL_Pos);
    }
    else
    {
        u32LPSPIClkSrcSel = LPSPI_CLKSEL_PCLK;
    }

    switch (u32LPSPIClkSrcSel)
    {
        case LPSPI_CLKSEL_PCLK:
            u32RetValue = LPSPI_GetPCLKSrcFreq(lpspi);      /* Clock source is PCLK4 */
            break;

        case LPSPI_CLKSEL_MIRC:
            u32RetValue = __MIRC;                           /* Clock source is MIRC */
            break;

        case LPSPI_CLKSEL_HIRC:
            u32RetValue = __HIRC;                           /* Clock source is HIRC */
            break;

        default:
            u32RetValue = LPSPI_GetPCLKSrcFreq(lpspi);      /* Clock source is PCLK4 */
            break;
    }

    return u32RetValue;
}

/**
  * @brief  This function make LPSPI module be ready to transfer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32MasterSlave Decides the LPSPI module is operating in master mode or in slave mode. (LPSPI_SLAVE, LPSPI_MASTER)
  * @param[in]  u32LPSPIMode Decides the transfer timing. (LPSPI_MODE_0, LPSPI_MODE_1, LPSPI_MODE_2, LPSPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a LPSPI transaction.
  * @param[in]  u32BusClock The expected frequency of LPSPI bus clock in Hz.
  * @return Actual frequency of LPSPI peripheral clock.
  * @details By default, the LPSPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
  *          slave selection function is disabled.
 *          In Slave mode, u32BusClock shall be 0 and the LPSPI clock divider setting will be 0.
  *          The actual clock rate may be different from the target LPSPI clock rate.
  *          For example, if the LPSPI source clock rate is 12 MHz and the target LPSPI bus clock rate is 7 MHz, the
  *          actual LPSPI clock rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, LPSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= LPSPI peripheral clock source, DIVIDER will be set to 0.
  * @note   In slave mode, the LPSPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t LPSPI_Open(LPSPI_T *lpspi, uint32_t u32MasterSlave, uint32_t u32LPSPIMode, uint32_t u32DataWidth, uint32_t u32BusClock)
{
    uint32_t u32RetValue = 0UL;

    lpspi->CTL &= ~LPSPI_CTL_SPIEN_Msk;

    LPSPI_WaitDisableReady(lpspi);

    /* Default setting: slave selection signal is active low.
    In Master mode, disable the automatic slave selection function. */
    lpspi->SSCTL = LPSPI_SS_ACTIVE_LOW;

    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    lpspi->CTL = (u32LPSPIMode | u32MasterSlave);
    LPSPI_SET_DATA_WIDTH(lpspi, u32DataWidth);

    if (u32MasterSlave == LPSPI_MASTER)
    {
        // Set the bus clock for the LPSPI module and store the actual frequency in u32RetValue
        u32RetValue = LPSPI_SetBusClock(lpspi, u32BusClock);
    }
    else     /* For slave mode, force the LPSPI peripheral clock rate to equal APB clock rate. */
    {
        /* Force slave clock source back to PCLK. */
        LPSPI_SetPCLKSrc(lpspi);

        /* Set DIVIDER = 0 */
        lpspi->CLKDIV = 0U;

        /* Select PCLK4 as the clock source of LPSPI */
        u32RetValue = LPSPI_GetPCLKSrcFreq(lpspi);
    }

    lpspi->CTL |= LPSPI_CTL_SPIEN_Msk;

    return u32RetValue;
}

/**
  * @brief  Disable LPSPI controller.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @details This function will reset LPSPI controller.
  */
void LPSPI_Close(const LPSPI_T *lpspi)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
    {
        /* Unlock protected registers */
        SYS_UnlockReg();
    }

    if (lpspi == LPSPI0)
    {
        /* Reset LPSPI */
        SYS->LPSPIRST |= SYS_LPSPIRST_LPSPI0RST_Msk;
        SYS->LPSPIRST &= ~SYS_LPSPIRST_LPSPI0RST_Msk;
    }
    else
    {

    }

    if (u32RegLockLevel)
    {
        /* Lock protected registers */
        SYS_LockReg();
    }
}

/**
  * @brief  Clear RX FIFO buffer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @details This function will clear LPSPI RX FIFO buffer. The RXEMPTY (LPSPI_STATUS[8]) will be set to 1.
  */
void LPSPI_ClearRxFIFO(LPSPI_T *lpspi)
{
    lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXFBCLR_Msk;
}

/**
  * @brief  Clear TX FIFO buffer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @details This function will clear LPSPI TX FIFO buffer. The TXEMPTY (LPSPI_STATUS[16]) will be set to 1.
  * @note The TX shift register will not be cleared.
  */
void LPSPI_ClearTxFIFO(LPSPI_T *lpspi)
{
    lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXFBCLR_Msk;
}

/**
  * @brief  Disable the automatic slave selection function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @details This function will disable the automatic slave selection function and set slave selection signal to inactive state.
  */
void LPSPI_DisableAutoSS(LPSPI_T *lpspi)
{
    uint32_t u32WasEnabled = LPSPI_EnterConfigMode(lpspi);

    lpspi->SSCTL &= ~(LPSPI_SSCTL_AUTOSS_Msk | LPSPI_SSCTL_SS_Msk);

    LPSPI_ExitConfigMode(lpspi, u32WasEnabled);
}

/**
  * @brief  Enable the automatic slave selection function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32SSPinMask Specifies slave selection pins. (LPSPI_SS)
  * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (LPSPI_SS_ACTIVE_HIGH, LPSPI_SS_ACTIVE_LOW)
  * @details This function will enable the automatic slave selection function. Only available in Master mode.
  *          The slave selection pin and the active level will be set in this function.
  */
void LPSPI_EnableAutoSS(LPSPI_T *lpspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    uint32_t u32WasEnabled = LPSPI_EnterConfigMode(lpspi);

    lpspi->SSCTL = (lpspi->SSCTL & (~(LPSPI_SSCTL_AUTOSS_Msk | LPSPI_SSCTL_SSACTPOL_Msk | LPSPI_SSCTL_SS_Msk))) | (u32SSPinMask | u32ActiveLevel | LPSPI_SSCTL_AUTOSS_Msk);

    LPSPI_ExitConfigMode(lpspi, u32WasEnabled);
}

/**
  * @brief  Set the LPSPI bus clock.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32BusClock The expected frequency of LPSPI bus clock in Hz.
  * @return Actual frequency of LPSPI bus clock.
  * @details This function is only available in Master mode. The actual clock rate may be different from the target SPI bus clock rate.
  *          For example, if the LPSPI source clock rate is 12 MHz and the target LPSPI bus clock rate is 7 MHz, the actual SPI bus clock
  *          rate will be 6 MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, LPSPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= LPSPI peripheral clock source, DIVIDER will be set to 0.
  */
uint32_t LPSPI_SetBusClock(LPSPI_T *lpspi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc = 0UL;
    uint32_t u32HCLKFreq = 0UL;
    uint32_t u32RetValue = 0UL;
    uint32_t u32WasEnabled = LPSPI_EnterConfigMode(lpspi);

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetSCLKFreq();

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Select PCLK4 as the clock source of LPSPI */
        LPSPI_SetPCLKSrc(lpspi);
    }

    /* Get clock source of SPI */
    u32ClkSrc = LPSPI_GetModuleClkSrcFreq(lpspi);

    if (u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        lpspi->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        lpspi->CLKDIV = 0U;
        /* Return master peripheral clock rate */
        u32RetValue = u32ClkSrc;
    }
    else if (u32BusClock == 0U)
    {
        /* Set DIVIDER to the maximum value. f_lpspi = f_lpspi_clk_src / (DIVIDER + 1) */
        lpspi->CLKDIV = (lpspi->CLKDIV & ~LPSPI_CLKDIV_DIVIDER_Msk) | LPSPI_CLKDIV_DIVIDER_Msk;

        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / ((LPSPI_CLKDIV_DIVIDER_Msk >> LPSPI_CLKDIV_DIVIDER_Pos) + 1U));
    }
    else
    {
        uint32_t u32Div = 0;

        u32Div = (uint32_t)((((uint64_t)u32ClkSrc + (uint64_t)u32BusClock - 1ULL) / (uint64_t)u32BusClock) - 1ULL);

        /* Ensure the selected LPSPI clock does not exceed the requested bus clock. */
        u32Div = ((u32Div > (LPSPI_CLKDIV_DIVIDER_Msk >> LPSPI_CLKDIV_DIVIDER_Pos)) ?
                  (LPSPI_CLKDIV_DIVIDER_Msk >> LPSPI_CLKDIV_DIVIDER_Pos) : u32Div);

        // Update the CLKDIV register with the new divider value
        lpspi->CLKDIV = (lpspi->CLKDIV & (~LPSPI_CLKDIV_DIVIDER_Msk)) | (u32Div << LPSPI_CLKDIV_DIVIDER_Pos);

        /* Return master peripheral clock rate */
        u32RetValue = (u32ClkSrc / (u32Div + 1U));
    }

    LPSPI_ExitConfigMode(lpspi, u32WasEnabled);

    return u32RetValue;
}

/**
  * @brief  Configure FIFO threshold setting.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32TxThreshold Decides the TX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @param[in]  u32RxThreshold Decides the RX FIFO threshold. It could be 0 ~ 3. If data width is 8~16 bits, it could be 0 ~ 7.
  * @details Set TX FIFO threshold and RX FIFO threshold configurations.
  */
void LPSPI_SetFIFO(LPSPI_T *lpspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    uint32_t u32WasEnabled = LPSPI_EnterConfigMode(lpspi);
    uint32_t u32TxTh = (u32TxThreshold & 0x7U);
    uint32_t u32RxTh = (u32RxThreshold & 0x7U);

    lpspi->FIFOCTL = (lpspi->FIFOCTL & ~(LPSPI_FIFOCTL_TXTH_Msk | LPSPI_FIFOCTL_RXTH_Msk)) |
                     (u32TxTh << LPSPI_FIFOCTL_TXTH_Pos) |
                     (u32RxTh << LPSPI_FIFOCTL_RXTH_Pos);

    LPSPI_ExitConfigMode(lpspi, u32WasEnabled);
}

/**
  * @brief  Get the actual frequency of LPSPI bus clock. Only available in Master mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return Actual LPSPI bus clock frequency in Hz.
  * @details This function will calculate the actual LPSPI bus clock rate according to the LPSPInSEL and DIVIDER settings. Only available in Master mode.
  */
uint32_t LPSPI_GetBusClock(const LPSPI_T *lpspi)
{
    uint32_t u32Div = 0;
    uint32_t u32ClkSrc = 0;

    /* Get DIVIDER setting */
    u32Div = (lpspi->CLKDIV & LPSPI_CLKDIV_DIVIDER_Msk) >> LPSPI_CLKDIV_DIVIDER_Pos;

    /* Get clock source of LPSPI */
    u32ClkSrc = LPSPI_GetModuleClkSrcFreq(lpspi);

    /* Return LPSPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1U));
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @details Enable LPSPI related interrupts specified by u32Mask parameter.
  */
void LPSPI_EnableInt(LPSPI_T *lpspi, uint32_t u32Mask)
{
    /* Enable unit transfer interrupt flag */
    if ((u32Mask & LPSPI_UNIT_INT_MASK) == LPSPI_UNIT_INT_MASK)
    {
        lpspi->CTL |= LPSPI_CTL_UNITIEN_Msk;
    }

    /* Enable slave selection signal active interrupt flag */
    if ((u32Mask & LPSPI_SSACT_INT_MASK) == LPSPI_SSACT_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SSACTIEN_Msk;
    }

    /* Enable slave selection signal inactive interrupt flag */
    if ((u32Mask & LPSPI_SSINACT_INT_MASK) == LPSPI_SSINACT_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SSINAIEN_Msk;
    }

    /* Enable slave TX under run interrupt flag */
    if ((u32Mask & LPSPI_SLVUR_INT_MASK) == LPSPI_SLVUR_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SLVURIEN_Msk;
    }

    /* Enable slave bit count error interrupt flag */
    if ((u32Mask & LPSPI_SLVBE_INT_MASK) == LPSPI_SLVBE_INT_MASK)
    {
        lpspi->SSCTL |= LPSPI_SSCTL_SLVBEIEN_Msk;
    }

    /* Enable slave TX underflow interrupt flag */
    if ((u32Mask & LPSPI_TXUF_INT_MASK) == LPSPI_TXUF_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXUFIEN_Msk;
    }

    /* Enable TX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) == LPSPI_FIFO_TXTH_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_TXTHIEN_Msk;
    }

    /* Enable RX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) == LPSPI_FIFO_RXTH_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXTHIEN_Msk;
    }

    /* Enable RX overrun interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) == LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXOVIEN_Msk;
    }

    /* Enable RX time-out interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) == LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->FIFOCTL |= LPSPI_FIFOCTL_RXTOIEN_Msk;
    }
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @details Disable LPSPI related interrupts specified by u32Mask parameter.
  */
void LPSPI_DisableInt(LPSPI_T *lpspi, uint32_t u32Mask)
{
    /* Disable unit transfer interrupt flag */
    if ((u32Mask & LPSPI_UNIT_INT_MASK) == LPSPI_UNIT_INT_MASK)
    {
        lpspi->CTL &= ~(LPSPI_CTL_UNITIEN_Msk);
    }

    /* Disable slave selection signal active interrupt flag */
    if ((u32Mask & LPSPI_SSACT_INT_MASK) == LPSPI_SSACT_INT_MASK)
    {
        lpspi->SSCTL &= ~(LPSPI_SSCTL_SSACTIEN_Msk);
    }

    /* Disable slave selection signal inactive interrupt flag */
    if ((u32Mask & LPSPI_SSINACT_INT_MASK) == LPSPI_SSINACT_INT_MASK)
    {
        lpspi->SSCTL &= ~(LPSPI_SSCTL_SSINAIEN_Msk);
    }

    /* Disable slave TX under run interrupt flag */
    if ((u32Mask & LPSPI_SLVUR_INT_MASK) == LPSPI_SLVUR_INT_MASK)
    {
        lpspi->SSCTL &= ~(LPSPI_SSCTL_SLVURIEN_Msk);
    }

    /* Disable slave bit count error interrupt flag */
    if ((u32Mask & LPSPI_SLVBE_INT_MASK) == LPSPI_SLVBE_INT_MASK)
    {
        lpspi->SSCTL &= ~(LPSPI_SSCTL_SLVBEIEN_Msk);
    }

    /* Disable slave TX underflow interrupt flag */
    if ((u32Mask & LPSPI_TXUF_INT_MASK) == LPSPI_TXUF_INT_MASK)
    {
        lpspi->FIFOCTL &= ~(LPSPI_FIFOCTL_TXUFIEN_Msk);
    }

    /* Disable TX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) == LPSPI_FIFO_TXTH_INT_MASK)
    {
        lpspi->FIFOCTL &= ~(LPSPI_FIFOCTL_TXTHIEN_Msk);
    }

    /* Disable RX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) == LPSPI_FIFO_RXTH_INT_MASK)
    {
        lpspi->FIFOCTL &= ~(LPSPI_FIFOCTL_RXTHIEN_Msk);
    }

    /* Disable RX overrun interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) == LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->FIFOCTL &= ~(LPSPI_FIFOCTL_RXOVIEN_Msk);
    }

    /* Disable RX time-out interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) == LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->FIFOCTL &= ~(LPSPI_FIFOCTL_RXTOIEN_Msk);
    }
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_TXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTH_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @return Interrupt flags of selected sources.
  * @details Get LPSPI related interrupt flags specified by u32Mask parameter.
  */
uint32_t LPSPI_GetIntFlag(const LPSPI_T *lpspi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0UL;
    uint32_t u32Status = lpspi->STATUS;

    /* Check unit transfer interrupt flag */
    if ((u32Mask & LPSPI_UNIT_INT_MASK) && ((u32Status & LPSPI_STATUS_UNITIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_UNIT_INT_MASK;
    }

    /* Check slave selection signal active interrupt flag */
    if ((u32Mask & LPSPI_SSACT_INT_MASK) && ((u32Status & LPSPI_STATUS_SSACTIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_SSACT_INT_MASK;
    }

    /* Check slave selection signal inactive interrupt flag */
    if ((u32Mask & LPSPI_SSINACT_INT_MASK) && ((u32Status & LPSPI_STATUS_SSINAIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_SSINACT_INT_MASK;
    }

    /* Check slave TX under run interrupt flag */
    if ((u32Mask & LPSPI_SLVUR_INT_MASK) && ((u32Status & LPSPI_STATUS_SLVURIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_SLVUR_INT_MASK;
    }

    /* Check slave bit count error interrupt flag */
    if ((u32Mask & LPSPI_SLVBE_INT_MASK) && ((u32Status & LPSPI_STATUS_SLVBEIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_SLVBE_INT_MASK;
    }

    /* Check slave TX underflow interrupt flag */
    if ((u32Mask & LPSPI_TXUF_INT_MASK) && ((u32Status & LPSPI_STATUS_TXUFIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_TXUF_INT_MASK;
    }

    /* Check TX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_TXTH_INT_MASK) && ((u32Status & LPSPI_STATUS_TXTHIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_FIFO_TXTH_INT_MASK;
    }

    /* Check RX threshold interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTH_INT_MASK) && ((u32Status & LPSPI_STATUS_RXTHIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_FIFO_RXTH_INT_MASK;
    }

    /* Check RX overrun interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXOV_INT_MASK) && ((u32Status & LPSPI_STATUS_RXOVIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_FIFO_RXOV_INT_MASK;
    }

    /* Check RX time-out interrupt flag */
    if ((u32Mask & LPSPI_FIFO_RXTO_INT_MASK) && ((u32Status & LPSPI_STATUS_RXTOIF_Msk) != 0U))
    {
        u32IntFlag |= LPSPI_FIFO_RXTO_INT_MASK;
    }

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
  *                       - \ref LPSPI_UNIT_INT_MASK
  *                       - \ref LPSPI_SSACT_INT_MASK
  *                       - \ref LPSPI_SSINACT_INT_MASK
  *                       - \ref LPSPI_SLVUR_INT_MASK
  *                       - \ref LPSPI_SLVBE_INT_MASK
  *                       - \ref LPSPI_TXUF_INT_MASK
  *                       - \ref LPSPI_FIFO_RXOV_INT_MASK
  *                       - \ref LPSPI_FIFO_RXTO_INT_MASK
  *
  * @details Clear LPSPI related interrupt flags specified by u32Mask parameter.
  */
void LPSPI_ClearIntFlag(LPSPI_T *lpspi, uint32_t u32Mask)
{
    if (u32Mask & LPSPI_UNIT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */
    }

    if (u32Mask & LPSPI_SSACT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */
    }

    if (u32Mask & LPSPI_SSINACT_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */
    }

    if (u32Mask & LPSPI_SLVUR_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */
    }

    if (u32Mask & LPSPI_SLVBE_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */
    }

    if (u32Mask & LPSPI_TXUF_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */
    }

    if (u32Mask & LPSPI_FIFO_RXOV_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */
    }

    if (u32Mask & LPSPI_FIFO_RXTO_INT_MASK)
    {
        lpspi->STATUS = LPSPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
    }
}

/**
  * @brief  Get LPSPI status.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a source.
  *                     This parameter decides which flags will be read. It is combination of:
  *                       - \ref LPSPI_BUSY_MASK
  *                       - \ref LPSPI_RX_EMPTY_MASK
  *                       - \ref LPSPI_RX_FULL_MASK
  *                       - \ref LPSPI_TX_EMPTY_MASK
  *                       - \ref LPSPI_TX_FULL_MASK
  *                       - \ref LPSPI_TXRX_RESET_MASK
  *                       - \ref LPSPI_SPIEN_STS_MASK
  *                       - \ref LPSPI_SSLINE_STS_MASK
  *
  * @return Flags of selected sources.
  * @details Get LPSPI related status specified by u32Mask parameter.
  */
uint32_t LPSPI_GetStatus(const LPSPI_T *lpspi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0UL;
    uint32_t u32Status = lpspi->STATUS;

    /* Check busy status */
    if ((u32Mask & LPSPI_BUSY_MASK) && ((u32Status & LPSPI_STATUS_BUSY_Msk) != 0U))
    {
        u32Flag |= LPSPI_BUSY_MASK;
    }

    /* Check RX empty flag */
    if ((u32Mask & LPSPI_RX_EMPTY_MASK) && ((u32Status & LPSPI_STATUS_RXEMPTY_Msk) != 0U))
    {
        u32Flag |= LPSPI_RX_EMPTY_MASK;
    }

    /* Check RX full flag */
    if ((u32Mask & LPSPI_RX_FULL_MASK) && ((u32Status & LPSPI_STATUS_RXFULL_Msk) != 0U))
    {
        u32Flag |= LPSPI_RX_FULL_MASK;
    }

    /* Check TX empty flag */
    if ((u32Mask & LPSPI_TX_EMPTY_MASK) && ((u32Status & LPSPI_STATUS_TXEMPTY_Msk) != 0U))
    {
        u32Flag |= LPSPI_TX_EMPTY_MASK;
    }

    /* Check TX full flag */
    if ((u32Mask & LPSPI_TX_FULL_MASK) && ((u32Status & LPSPI_STATUS_TXFULL_Msk) != 0U))
    {
        u32Flag |= LPSPI_TX_FULL_MASK;
    }

    /* Check TX/RX reset flag */
    if ((u32Mask & LPSPI_TXRX_RESET_MASK) && ((u32Status & LPSPI_STATUS_TXRXRST_Msk) != 0U))
    {
        u32Flag |= LPSPI_TXRX_RESET_MASK;
    }

    /* Check SPIEN flag */
    if ((u32Mask & LPSPI_SPIEN_STS_MASK) && ((u32Status & LPSPI_STATUS_SPIENSTS_Msk) != 0U))
    {
        u32Flag |= LPSPI_SPIEN_STS_MASK;
    }

    /* Check SPIx_SS line status */
    if ((u32Mask & LPSPI_SSLINE_STS_MASK) && ((u32Status & LPSPI_STATUS_SSLINE_Msk) != 0U))
    {
        u32Flag |= LPSPI_SSLINE_STS_MASK;
    }

    return u32Flag;
}

/** @} end of group LPSPI_EXPORTED_FUNCTIONS */
/** @} end of group LPSPI_Driver */
/** @} end of group Standard_Driver */
