/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * @brief    SYS driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup SYS_Driver SYS Driver
    @{
*/

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
    @{
*/


/**
  * @brief      Reset selected module
  * @param[in]  u32ModuleIndex is module index. Including :
  *             - \ref SYS_ACMP01RST
  *             - \ref SYS_ACMP23RST
  *             - \ref SYS_AWF0RST
  *             - \ref SYS_BPWM0RST
  *             - \ref SYS_BPWM1RST
  *             - \ref SYS_CANFD0RST
  *             - \ref SYS_CANFD1RST
  *             - \ref SYS_CCAP0RST
  *             - \ref SYS_CRC0RST
  *             - \ref SYS_CRYPTO0RST
  *             - \ref SYS_DAC01RST
  *             - \ref SYS_DMIC0RST
  *             - \ref SYS_EADC0RST
  *             - \ref SYS_EBI0RST
  *             - \ref SYS_ECAP0RST
  *             - \ref SYS_ECAP1RST
  *             - \ref SYS_ECAP2RST
  *             - \ref SYS_ECAP3RST
  *             - \ref SYS_EMAC0RST
  *             - \ref SYS_EPWM0RST
  *             - \ref SYS_EPWM1RST
  *             - \ref SYS_EQEI0RST
  *             - \ref SYS_EQEI1RST
  *             - \ref SYS_EQEI2RST
  *             - \ref SYS_EQEI3RST
  *             - \ref SYS_FMC0RST
  *             - \ref SYS_GDMA0RST
  *             - \ref SYS_GPIO0RST
  *             - \ref SYS_HSOTG0RST
  *             - \ref SYS_HSUSBD0RST
  *             - \ref SYS_HSUSBH0RST
  *             - \ref SYS_I2C0RST
  *             - \ref SYS_I2C1RST
  *             - \ref SYS_I2C2RST
  *             - \ref SYS_I2C3RST
  *             - \ref SYS_I2S0RST
  *             - \ref SYS_I2S1RST
  *             - \ref SYS_I3C0RST
  *             - \ref SYS_KDF0RST
  *             - \ref SYS_KPI0RST
  *             - \ref SYS_KS0RST
  *             - \ref SYS_LPADC0RST
  *             - \ref SYS_LPPDMA0RST
  *             - \ref SYS_LPGPIO0RST
  *             - \ref SYS_LPI2C0RST
  *             - \ref SYS_LPSPI0RST
  *             - \ref SYS_LPTMR0RST
  *             - \ref SYS_LPTMR1RST
  *             - \ref SYS_LPUART0RST
  *             - \ref SYS_NPURST
  *             - \ref SYS_OTFC0RST
  *             - \ref SYS_OTG0RST
  *             - \ref SYS_PDMA0RST
  *             - \ref SYS_PDMA1RST
  *             - \ref SYS_PSIO0RST
  *             - \ref SYS_QSPI0RST
  *             - \ref SYS_QSPI1RST
  *             - \ref SYS_RTC0RST
  *             - \ref SYS_SC0RST
  *             - \ref SYS_SC1RST
  *             - \ref SYS_SC2RST
  *             - \ref SYS_SCU0RST
  *             - \ref SYS_SDH0RST
  *             - \ref SYS_SDH1RST
  *             - \ref SYS_SPI0RST
  *             - \ref SYS_SPI1RST
  *             - \ref SYS_SPI2RST
  *             - \ref SYS_SPI3RST
  *             - \ref SYS_SPIM0RST
  *             - \ref SYS_TMR0RST
  *             - \ref SYS_TMR1RST
  *             - \ref SYS_TMR2RST
  *             - \ref SYS_TMR3RST
  *             - \ref SYS_TRNG0RST
  *             - \ref SYS_TTMR0RST
  *             - \ref SYS_TTMR1RST
  *             - \ref SYS_UART0RST
  *             - \ref SYS_UART1RST
  *             - \ref SYS_UART2RST
  *             - \ref SYS_UART3RST
  *             - \ref SYS_UART4RST
  *             - \ref SYS_UART5RST
  *             - \ref SYS_UART6RST
  *             - \ref SYS_UART7RST
  *             - \ref SYS_UART8RST
  *             - \ref SYS_UART9RST
  *             - \ref SYS_USBD0RST
  *             - \ref SYS_USBH0RST
  *             - \ref SYS_USCI0RST
  *             - \ref SYS_UTCPD0RST
  *             - \ref SYS_WWDT0RST
  *             - \ref SYS_WWDT1RST
  * @return     None
  * @details    This function reset selected module.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    /* Generate reset signal to the corresponding module */
    *(volatile uint32_t *)((uint32_t)SYS_BASE + (u32ModuleIndex >> 20UL))  |= 1UL << (u32ModuleIndex & 0x000fffffUL);

    /* Release corresponding module from reset state */
    *(volatile uint32_t *)((uint32_t)SYS_BASE + (u32ModuleIndex >> 20UL))  &= ~(1UL << (u32ModuleIndex & 0x000fffffUL));
}

/**
  * @brief      Enable and configure Brown-out detector function
  * @param[in]  i32Mode is reset or interrupt mode. Including :
  *             - \ref SYS_BODCTL_BOD_RST_EN
  *             - \ref SYS_BODCTL_BOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_1_6V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_6V
  *             - \ref SYS_BODCTL_BODVL_2_8V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  * @retval     SYS_OK          SYS operation OK.
  * @retval     SYS_ERR_TIMEOUT SYS operation abort due to timeout error.
  * @details    This function configure Brown-out detector reset or interrupt mode, enable Brown-out function and set Brown-out voltage level.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    /* Enable Brown-out Detector function */
    /* Enable Brown-out interrupt or reset function */
    /* Select Brown-out Detector threshold voltage */
    SYS_WAIT_BODCTL_WRBUSY();

    if ((SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) == 0)
        SYS->BODCTL = (SYS->BODCTL & ~(SYS_BODCTL_BODRSTEN_Msk | SYS_BODCTL_BODVL_Msk)) | \
                      ((uint32_t)i32Mode) | (u32BODLevel) | (SYS_BODCTL_BODEN_Msk);
    else return SYS_ERR_TIMEOUT;

    return SYS_OK;
}

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @retval     SYS_OK          SYS operation OK.
  * @retval     SYS_ERR_TIMEOUT SYS operation abort due to timeout error.
  * @details    This function disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_DisableBOD(void)
{
    SYS_WAIT_BODCTL_WRBUSY();

    if ((SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) == 0)
        SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;
    else return SYS_ERR_TIMEOUT;

    return SYS_OK;
}

/** @} end of group SYS_EXPORTED_FUNCTIONS */
/** @} end of group SYS_Driver */
/** @} end of group Standard_Driver */
