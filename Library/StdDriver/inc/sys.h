/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * @brief    SYS driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/

/** @addtogroup SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* SYS Timeout constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_TIMEOUT         SystemCoreClock     /*!< SYS time-out counter (1 second time-out) */

/*---------------------------------------------------------------------------------------------------------*/
/* SYS Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_OK              ( 0L)               /*!< SYS operation OK */
#define SYS_ERR_FAIL        (-1L)               /*!< SYS operation failed */
#define SYS_ERR_TIMEOUT     (-2L)               /*!< SYS operation abort due to timeout error */

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_ACMP01RST       ((0x200UL<<20) | SYS_ACMPRST_ACMP01RST_Pos)         /*!< Reset ACMP01 \hideinitializer*/
#define SYS_ACMP23RST       ((0x200UL<<20) | SYS_ACMPRST_ACMP23RST_Pos)         /*!< Reset ACMP23 \hideinitializer*/
#define SYS_AWF0RST         ((0x204UL<<20) | SYS_AWFRST_AWF0RST_Pos)            /*!< Reset AWF0 \hideinitializer*/
#define SYS_BPWM0RST        ((0x208UL<<20) | SYS_BPWMRST_BPWM0RST_Pos)          /*!< Reset BPWM0 \hideinitializer*/
#define SYS_BPWM1RST        ((0x208UL<<20) | SYS_BPWMRST_BPWM1RST_Pos)          /*!< Reset BPWM1 \hideinitializer*/
#define SYS_CANFD0RST       ((0x20CUL<<20) | SYS_CANFDRST_CANFD0RST_Pos)        /*!< Reset CANFD0RST \hideinitializer*/
#define SYS_CANFD1RST       ((0x20CUL<<20) | SYS_CANFDRST_CANFD1RST_Pos)        /*!< Reset CANFD1RST \hideinitializer*/
#define SYS_CCAP0RST        ((0x210UL<<20) | SYS_CCAPRST_CCAP0RST_Pos)          /*!< Reset CCAP0RST \hideinitializer*/
#define SYS_CRC0RST         ((0x214UL<<20) | SYS_CRCRST_CRC0RST_Pos)            /*!< Reset CRC0RST \hideinitializer*/
#define SYS_CRYPTO0RST      ((0x218UL<<20) | SYS_CRYPTORST_CRYPTO0RST_Pos)      /*!< Reset CRYPTO0RST \hideinitializer*/
#define SYS_DAC01RST        ((0x21CUL<<20) | SYS_DACRST_DAC01RST_Pos)           /*!< Reset DAC01RST \hideinitializer*/
#define SYS_DMIC0RST        ((0x220UL<<20) | SYS_DMICRST_DMIC0RST_Pos)          /*!< Reset DMIC0RST \hideinitializer*/
#define SYS_EADC0RST        ((0x224UL<<20) | SYS_EADCRST_EADC0RST_Pos)          /*!< Reset EADC0RST \hideinitializer*/
#define SYS_EBI0RST         ((0x228UL<<20) | SYS_EBIRST_EBI0RST_Pos)            /*!< Reset EBI0RST \hideinitializer*/
#define SYS_ECAP0RST        ((0x22CUL<<20) | SYS_ECAPRST_ECAP0RST_Pos)          /*!< Reset ECAP0RST \hideinitializer*/
#define SYS_ECAP1RST        ((0x22CUL<<20) | SYS_ECAPRST_ECAP1RST_Pos)          /*!< Reset ECAP1RST \hideinitializer*/
#define SYS_ECAP2RST        ((0x22CUL<<20) | SYS_ECAPRST_ECAP2RST_Pos)          /*!< Reset ECAP2RST \hideinitializer*/
#define SYS_ECAP3RST        ((0x22CUL<<20) | SYS_ECAPRST_ECAP3RST_Pos)          /*!< Reset ECAP3RST \hideinitializer*/
#define SYS_EMAC0RST        ((0x230UL<<20) | SYS_EMACRST_EMAC0RST_Pos)          /*!< Reset EMAC0RST \hideinitializer*/
#define SYS_EPWM0RST        ((0x234UL<<20) | SYS_EPWMRST_EPWM0RST_Pos)          /*!< Reset EPWM0RST \hideinitializer*/
#define SYS_EPWM1RST        ((0x234UL<<20) | SYS_EPWMRST_EPWM1RST_Pos)          /*!< Reset EPWM1RST \hideinitializer*/
#define SYS_EQEI0RST        ((0x238UL<<20) | SYS_EQEIRST_EQEI0RST_Pos)          /*!< Reset EQEI0RST \hideinitializer*/
#define SYS_EQEI1RST        ((0x238UL<<20) | SYS_EQEIRST_EQEI1RST_Pos)          /*!< Reset EQEI1RST \hideinitializer*/
#define SYS_EQEI2RST        ((0x238UL<<20) | SYS_EQEIRST_EQEI2RST_Pos)          /*!< Reset EQEI2RST \hideinitializer*/
#define SYS_EQEI3RST        ((0x238UL<<20) | SYS_EQEIRST_EQEI3RST_Pos)          /*!< Reset EQEI3RST \hideinitializer*/
#define SYS_FMC0RST         ((0x23CUL<<20) | SYS_FMCRST_FMC0RST_Pos)            /*!< Reset FMC0RST \hideinitializer*/
#define SYS_GDMA0RST        ((0x240UL<<20) | SYS_GDMARST_GDMA0RST_Pos)          /*!< Reset GDMA0RST \hideinitializer*/
#define SYS_GPIO0RST        ((0x244UL<<20) | SYS_GPIORST_GPIO0RST_Pos)          /*!< Reset GPIO0RST \hideinitializer*/
#define SYS_HSOTG0RST       ((0x248UL<<20) | SYS_HSOTGRST_HSOTG0RST_Pos)        /*!< Reset HSOTG0RST \hideinitializer*/
#define SYS_HSUSBD0RST      ((0x24CUL<<20) | SYS_HSUSBDRST_HSUSBD0RST_Pos)      /*!< Reset HSUSBD0RST \hideinitializer*/
#define SYS_HSUSBH0RST      ((0x250UL<<20) | SYS_HSUSBHRST_HSUSBH0RST_Pos)      /*!< Reset HSUSBH0RST \hideinitializer*/
#define SYS_I2C0RST         ((0x254UL<<20) | SYS_I2CRST_I2C0RST_Pos)            /*!< Reset I2C0RST \hideinitializer*/
#define SYS_I2C1RST         ((0x254UL<<20) | SYS_I2CRST_I2C1RST_Pos)            /*!< Reset I2C0RST \hideinitializer*/
#define SYS_I2C2RST         ((0x254UL<<20) | SYS_I2CRST_I2C2RST_Pos)            /*!< Reset I2C0RST \hideinitializer*/
#define SYS_I2C3RST         ((0x254UL<<20) | SYS_I2CRST_I2C3RST_Pos)            /*!< Reset I2C0RST \hideinitializer*/
#define SYS_I2S0RST         ((0x258UL<<20) | SYS_I2SRST_I2S0RST_Pos)            /*!< Reset I2S0RST \hideinitializer*/
#define SYS_I2S1RST         ((0x258UL<<20) | SYS_I2SRST_I2S1RST_Pos)            /*!< Reset I2S1RST \hideinitializer*/
#define SYS_I3C0RST         ((0x25CUL<<20) | SYS_I3CRST_I3C0RST_Pos)            /*!< Reset I3C0RST \hideinitializer*/
#define SYS_KDF0RST         ((0x260UL<<20) | SYS_KDFRST_KDF0RST_Pos)            /*!< Reset KDF0RST \hideinitializer*/
#define SYS_KPI0RST         ((0x264UL<<20) | SYS_KPIRST_KPI0RST_Pos)            /*!< Reset KPI0RST \hideinitializer*/
#define SYS_KS0RST          ((0x268UL<<20) | SYS_KSRST_KS0RST_Pos)              /*!< Reset KS0RST \hideinitializer*/
#define SYS_LPADC0RST       ((0x26CUL<<20) | SYS_LPADCRST_LPADC0RST_Pos)        /*!< Reset LPADC0RST \hideinitializer*/
#define SYS_LPPDMA0RST      ((0x270UL<<20) | SYS_LPPDMARST_LPPDMA0RST_Pos)      /*!< Reset LPDMA0RST \hideinitializer*/
#define SYS_LPGPIO0RST      ((0x274UL<<20) | SYS_LPGPIORST_LPGPIO0RST_Pos)      /*!< Reset LPGPIO0RST \hideinitializer*/
#define SYS_LPI2C0RST       ((0x278UL<<20) | SYS_LPI2CRST_LPI2C0RST_Pos)        /*!< Reset LPI2C0RST \hideinitializer*/
#define SYS_LPSPI0RST       ((0x27CUL<<20) | SYS_LPSPIRST_LPSPI0RST_Pos)        /*!< Reset LPSPI0RST \hideinitializer*/
#define SYS_LPTMR0RST       ((0x280UL<<20) | SYS_LPTMRRST_LPTMR0RST_Pos)        /*!< Reset LPTMR0RST \hideinitializer*/
#define SYS_LPTMR1RST       ((0x280UL<<20) | SYS_LPTMRRST_LPTMR1RST_Pos)        /*!< Reset LPTMR1RST \hideinitializer*/
#define SYS_LPUART0RST      ((0x284UL<<20) | SYS_LPUARTRST_LPUART0RST_Pos)      /*!< Reset LPUART0RST \hideinitializer*/
#define SYS_NPURST          ((0x004UL<<20) | SYS_RSTCTL_NPURST_Pos)             /*!< Reset NPURST \hideinitializer*/
#define SYS_OTFC0RST        ((0x288UL<<20) | SYS_OTFCRST_OTFC0RST_Pos)          /*!< Reset OTFC0RST \hideinitializer*/
#define SYS_OTG0RST         ((0x28CUL<<20) | SYS_OTGRST_OTG0RST_Pos)            /*!< Reset OTG0RST \hideinitializer*/
#define SYS_PDMA0RST        ((0x290UL<<20) | SYS_PDMARST_PDMA0RST_Pos)          /*!< Reset PDMA0RST \hideinitializer*/
#define SYS_PDMA1RST        ((0x290UL<<20) | SYS_PDMARST_PDMA1RST_Pos)          /*!< Reset PDMA1RST \hideinitializer*/
#define SYS_PSIO0RST        ((0x294UL<<20) | SYS_PSIORST_PSIO0RST_Pos)          /*!< Reset PSIO0RST \hideinitializer*/
#define SYS_QSPI0RST        ((0x298UL<<20) | SYS_QSPIRST_QSPI0RST_Pos)          /*!< Reset QSPI0RST \hideinitializer*/
#define SYS_QSPI1RST        ((0x298UL<<20) | SYS_QSPIRST_QSPI1RST_Pos)          /*!< Reset QSPI1RST \hideinitializer*/
#define SYS_RTC0RST         ((0x29CUL<<20) | SYS_RTCRST_RTC0RST_Pos)            /*!< Reset RTC0RST \hideinitializer*/
#define SYS_SC0RST          ((0x2A0UL<<20) | SYS_SCRST_SC0RST_Pos)              /*!< Reset SC0RST \hideinitializer*/
#define SYS_SC1RST          ((0x2A0UL<<20) | SYS_SCRST_SC1RST_Pos)              /*!< Reset SC1RST \hideinitializer*/
#define SYS_SC2RST          ((0x2A0UL<<20) | SYS_SCRST_SC2RST_Pos)              /*!< Reset SC2RST \hideinitializer*/
#define SYS_SCU0RST         ((0x2A4UL<<20) | SYS_SCURST_SCU0RST_Pos)            /*!< Reset SCU0RST \hideinitializer*/
#define SYS_SDH0RST         ((0x2A8UL<<20) | SYS_SDHRST_SDH0RST_Pos)            /*!< Reset SDH0RST \hideinitializer*/
#define SYS_SDH1RST         ((0x2A8UL<<20) | SYS_SDHRST_SDH1RST_Pos)            /*!< Reset SDH1RST \hideinitializer*/
#define SYS_SPI0RST         ((0x2ACUL<<20) | SYS_SPIRST_SPI0RST_Pos)            /*!< Reset SPI0RST \hideinitializer*/
#define SYS_SPI1RST         ((0x2ACUL<<20) | SYS_SPIRST_SPI1RST_Pos)            /*!< Reset SPI1RST \hideinitializer*/
#define SYS_SPI2RST         ((0x2ACUL<<20) | SYS_SPIRST_SPI2RST_Pos)            /*!< Reset SPI2RST \hideinitializer*/
#define SYS_SPI3RST         ((0x2ACUL<<20) | SYS_SPIRST_SPI3RST_Pos)            /*!< Reset SPI3RST \hideinitializer*/
#define SYS_SPIM0RST        ((0x2B0UL<<20) | SYS_SPIMRST_SPIM0RST_Pos)          /*!< Reset SPIM0RST \hideinitializer*/
#define SYS_TMR0RST         ((0x2C0UL<<20) | SYS_TMRRST_TMR0RST_Pos)            /*!< Reset TMR0RST \hideinitializer*/
#define SYS_TMR1RST         ((0x2C0UL<<20) | SYS_TMRRST_TMR1RST_Pos)            /*!< Reset TMR1RST \hideinitializer*/
#define SYS_TMR2RST         ((0x2C0UL<<20) | SYS_TMRRST_TMR2RST_Pos)            /*!< Reset TMR2RST \hideinitializer*/
#define SYS_TMR3RST         ((0x2C0UL<<20) | SYS_TMRRST_TMR3RST_Pos)            /*!< Reset TMR3RST \hideinitializer*/
#define SYS_TRNG0RST        ((0x2C4UL<<20) | SYS_TRNGRST_TRNG0RST_Pos)          /*!< Reset TRNG0RST \hideinitializer*/
#define SYS_TTMR0RST        ((0x2C8UL<<20) | SYS_TTMRRST_TTMR0RST_Pos)          /*!< Reset TTMR0RST \hideinitializer*/
#define SYS_TTMR1RST        ((0x2C8UL<<20) | SYS_TTMRRST_TTMR1RST_Pos)          /*!< Reset TTMR1RST \hideinitializer*/
#define SYS_UART0RST        ((0x2CCUL<<20) | SYS_UARTRST_UART0RST_Pos)          /*!< Reset UART0RST \hideinitializer*/
#define SYS_UART1RST        ((0x2CCUL<<20) | SYS_UARTRST_UART1RST_Pos)          /*!< Reset UART1RST \hideinitializer*/
#define SYS_UART2RST        ((0x2CCUL<<20) | SYS_UARTRST_UART2RST_Pos)          /*!< Reset UART2RST \hideinitializer*/
#define SYS_UART3RST        ((0x2CCUL<<20) | SYS_UARTRST_UART3RST_Pos)          /*!< Reset UART3RST \hideinitializer*/
#define SYS_UART4RST        ((0x2CCUL<<20) | SYS_UARTRST_UART4RST_Pos)          /*!< Reset UART4RST \hideinitializer*/
#define SYS_UART5RST        ((0x2CCUL<<20) | SYS_UARTRST_UART5RST_Pos)          /*!< Reset UART5RST \hideinitializer*/
#define SYS_UART6RST        ((0x2CCUL<<20) | SYS_UARTRST_UART6RST_Pos)          /*!< Reset UART6RST \hideinitializer*/
#define SYS_UART7RST        ((0x2CCUL<<20) | SYS_UARTRST_UART7RST_Pos)          /*!< Reset UART7RST \hideinitializer*/
#define SYS_UART8RST        ((0x2CCUL<<20) | SYS_UARTRST_UART8RST_Pos)          /*!< Reset UART8RST \hideinitializer*/
#define SYS_UART9RST        ((0x2CCUL<<20) | SYS_UARTRST_UART9RST_Pos)          /*!< Reset UART9RST \hideinitializer*/
#define SYS_USBD0RST        ((0x2D0UL<<20) | SYS_USBDRST_USBD0RST_Pos)          /*!< Reset USBD0RST \hideinitializer*/
#define SYS_USBH0RST        ((0x2D4UL<<20) | SYS_USBHRST_USBH0RST_Pos)          /*!< Reset USBH0RST \hideinitializer*/
#define SYS_USCI0RST        ((0x2D8UL<<20) | SYS_USCIRST_USCI0RST_Pos)          /*!< Reset USCI0RST \hideinitializer*/
#define SYS_UTCPD0RST       ((0x2DCUL<<20) | SYS_UTCPDRST_UTCPD0RST_Pos)        /*!< Reset UTCPD0RST \hideinitializer*/
#define SYS_WWDT0RST        ((0x2E0UL<<20) | SYS_WWDTRST_WWDT0RST_Pos)          /*!< Reset WWDT0RST \hideinitializer*/
#define SYS_WWDT1RST        ((0x2E0UL<<20) | SYS_WWDTRST_WWDT1RST_Pos)          /*!< Reset WWDT1RST \hideinitializer*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Threshold Voltage Selection constant definitions.                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BOD_INTERRUPT_EN     (0UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Interrupt Enable */
#define SYS_BODCTL_BOD_RST_EN           (1UL<<SYS_BODCTL_BODRSTEN_Pos)    /*!< Brown-out Reset Enable */
#define SYS_BODCTL_BODVL_1_6V           (0UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.6V */
#define SYS_BODCTL_BODVL_1_8V           (1UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 1.8V */
#define SYS_BODCTL_BODVL_2_0V           (2UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.0V */
#define SYS_BODCTL_BODVL_2_2V           (3UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.2V */
#define SYS_BODCTL_BODVL_2_4V           (4UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.4V */
#define SYS_BODCTL_BODVL_2_6V           (5UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.6V */
#define SYS_BODCTL_BODVL_2_8V           (6UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 2.8V */
#define SYS_BODCTL_BODVL_3_0V           (7UL<<SYS_BODCTL_BODVL_Pos)       /*!< Setting Brown Out Detector Threshold Voltage as 3.0V */

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector Wake-Up Selection constant definitions.                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BODWKEN_DISABLE      (0UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Wake-up Function Disable */
#define SYS_BODCTL_BODWKEN_RISE         (1UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Rise Wake-up Function Enable */
#define SYS_BODCTL_BODWKEN_DROP         (2UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Drop Wake-up Function Enable */
#define SYS_BODCTL_BODWKEN_BOTH         (3UL<<SYS_BODCTL_BODWKEN_Pos)    /*!< Brown Out Detector Power Rise and Drop Wake-up Function Enable*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown-out Detector Output De-glitch Time constant definitions.                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_BODCTL_BODDGSEL_LIRC        (0UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as LIRC*/
#define SYS_BODCTL_BODDGSEL_4_SCLK      (1UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 4 SCLK*/
#define SYS_BODCTL_BODDGSEL_8_SCLK      (2UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 8 SCLK*/
#define SYS_BODCTL_BODDGSEL_16_SCLK     (3UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 16 SCLK*/
#define SYS_BODCTL_BODDGSEL_32_SCLK     (4UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 32 SCLK*/
#define SYS_BODCTL_BODDGSEL_64_SCLK     (5UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 64 SCLK*/
#define SYS_BODCTL_BODDGSEL_128_SCLK    (6UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 128 SCLK*/
#define SYS_BODCTL_BODDGSEL_256_SCLK    (7UL<<SYS_BODCTL_BODDGSEL_Pos)    /*!< Setting Brown-out Detector Output De-glitch Time as 256 SCLK*/

/*---------------------------------------------------------------------------------------------------------*/
/*  VREFCTL constant definitions. (Write-Protection Register)                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define SYS_VREFCTL_VREF_PIN            (0x0UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = External pin */
#define SYS_VREFCTL_VREF_1_6V           (0x3UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 1.6V */
#define SYS_VREFCTL_VREF_2_048V         (0x7UL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.048V */
#define SYS_VREFCTL_VREF_2_5V           (0xBUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 2.5V */
#define SYS_VREFCTL_VREF_3_072V         (0xFUL<<SYS_VREFCTL_VREFCTL_Pos)    /*!< Vref = 3.072V */
#define SYS_VREFCTL_VREF_AVDD           (0x10UL<<SYS_VREFCTL_VREFCTL_Pos)   /*!< Vref = AVDD */
/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

/* PA.0 MFP */
#define SYS_GPA_MFP0_PA0MFP_GPIO              (0x0UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for GPIO               */
#define SYS_GPA_MFP0_PA0MFP_QSPI0_MOSI0       (0x3UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for QSPI0_MOSI0        */
#define SYS_GPA_MFP0_PA0MFP_SPI0_MOSI         (0x4UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for SPI0_MOSI          */
#define SYS_GPA_MFP0_PA0MFP_SD1_DAT0          (0x5UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for SD1_DAT0           */
#define SYS_GPA_MFP0_PA0MFP_SC0_CLK           (0x6UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for SC0_CLK            */
#define SYS_GPA_MFP0_PA0MFP_UART0_RXD         (0x7UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for UART0_RXD          */
#define SYS_GPA_MFP0_PA0MFP_UART1_nRTS        (0x8UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for UART1_nRTS         */
#define SYS_GPA_MFP0_PA0MFP_I2C2_SDA          (0x9UL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for I2C2_SDA           */
#define SYS_GPA_MFP0_PA0MFP_CCAP_DATA6        (0xAUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for CCAP_DATA6         */
#define SYS_GPA_MFP0_PA0MFP_I3C0_SDA          (0xBUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for I3C0_SDA           */
#define SYS_GPA_MFP0_PA0MFP_BPWM0_CH0         (0xCUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for BPWM0_CH0          */
#define SYS_GPA_MFP0_PA0MFP_EPWM0_CH5         (0xDUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for EPWM0_CH5          */
#define SYS_GPA_MFP0_PA0MFP_EQEI3_B           (0xEUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for EQEI3_B            */
#define SYS_GPA_MFP0_PA0MFP_DAC0_ST           (0xFUL<<SYS_GPA_MFP0_PA0MFP_Pos)    /*!< GPA_MFP0 PA0 setting for DAC0_ST            */
#define SYS_GPA_MFP0_PA0MFP_PSIO0_CH7         (0x11UL<<SYS_GPA_MFP0_PA0MFP_Pos)   /*!< GPA_MFP0 PA0 setting for PSIO0_CH7          */
#define SYS_GPA_MFP0_PA0MFP_UTCPD0_VCNEN1     (0x13UL<<SYS_GPA_MFP0_PA0MFP_Pos)   /*!< GPA_MFP0 PA0 setting for UTCPD0_VCNEN1      */
#define SYS_GPA_MFP0_PA0MFP_LPSPI0_MOSI       (0x14UL<<SYS_GPA_MFP0_PA0MFP_Pos)   /*!< GPA_MFP0 PA0 setting for LPSPI0_MOSI        */
#define SYS_GPA_MFP0_PA0MFP_LPUART0_RXD       (0x15UL<<SYS_GPA_MFP0_PA0MFP_Pos)   /*!< GPA_MFP0 PA0 setting for LPUART0_RXD        */
#define SYS_GPA_MFP0_PA0MFP_LPIO0             (0x17UL<<SYS_GPA_MFP0_PA0MFP_Pos)   /*!< GPA_MFP0 PA0 setting for LPIO0              */

/* PA.1 MFP */
#define SYS_GPA_MFP0_PA1MFP_GPIO              (0x0UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for GPIO               */
#define SYS_GPA_MFP0_PA1MFP_QSPI0_MISO0       (0x3UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for QSPI0_MISO0        */
#define SYS_GPA_MFP0_PA1MFP_SPI0_MISO         (0x4UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for SPI0_MISO          */
#define SYS_GPA_MFP0_PA1MFP_SD1_DAT1          (0x5UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for SD1_DAT1           */
#define SYS_GPA_MFP0_PA1MFP_SC0_DAT           (0x6UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for SC0_DAT            */
#define SYS_GPA_MFP0_PA1MFP_UART0_TXD         (0x7UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for UART0_TXD          */
#define SYS_GPA_MFP0_PA1MFP_UART1_nCTS        (0x8UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for UART1_nCTS         */
#define SYS_GPA_MFP0_PA1MFP_I2C2_SCL          (0x9UL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for I2C2_SCL           */
#define SYS_GPA_MFP0_PA1MFP_CCAP_DATA7        (0xAUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for CCAP_DATA7         */
#define SYS_GPA_MFP0_PA1MFP_I3C0_SCL          (0xBUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for I3C0_SCL           */
#define SYS_GPA_MFP0_PA1MFP_BPWM0_CH1         (0xCUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for BPWM0_CH1          */
#define SYS_GPA_MFP0_PA1MFP_EPWM0_CH4         (0xDUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for EPWM0_CH4          */
#define SYS_GPA_MFP0_PA1MFP_EQEI3_A           (0xEUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for EQEI3_A            */
#define SYS_GPA_MFP0_PA1MFP_DAC1_ST           (0xFUL<<SYS_GPA_MFP0_PA1MFP_Pos)    /*!< GPA_MFP0 PA1 setting for DAC1_ST            */
#define SYS_GPA_MFP0_PA1MFP_DMIC1_CLK         (0x10UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for DMIC1_CLK          */
#define SYS_GPA_MFP0_PA1MFP_PSIO0_CH6         (0x11UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for PSIO0_CH6          */
#define SYS_GPA_MFP0_PA1MFP_UTCPD0_DISCHG     (0x12UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for UTCPD0_DISCHG      */
#define SYS_GPA_MFP0_PA1MFP_UTCPD0_FRSTX1     (0x13UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for UTCPD0_FRSTX1      */
#define SYS_GPA_MFP0_PA1MFP_LPSPI0_MISO       (0x14UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for LPSPI0_MISO        */
#define SYS_GPA_MFP0_PA1MFP_LPUART0_TXD       (0x15UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for LPUART0_TXD        */
#define SYS_GPA_MFP0_PA1MFP_LPIO1             (0x17UL<<SYS_GPA_MFP0_PA1MFP_Pos)   /*!< GPA_MFP0 PA1 setting for LPIO1              */

/* PA.2 MFP */
#define SYS_GPA_MFP0_PA2MFP_GPIO              (0x0UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for GPIO               */
#define SYS_GPA_MFP0_PA2MFP_QSPI0_CLK         (0x3UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for QSPI0_CLK          */
#define SYS_GPA_MFP0_PA2MFP_SPI0_CLK          (0x4UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for SPI0_CLK           */
#define SYS_GPA_MFP0_PA2MFP_SD1_DAT2          (0x5UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for SD1_DAT2           */
#define SYS_GPA_MFP0_PA2MFP_SC0_RST           (0x6UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for SC0_RST            */
#define SYS_GPA_MFP0_PA2MFP_UART4_RXD         (0x7UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for UART4_RXD          */
#define SYS_GPA_MFP0_PA2MFP_UART1_RXD         (0x8UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for UART1_RXD          */
#define SYS_GPA_MFP0_PA2MFP_I2C1_SDA          (0x9UL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for I2C1_SDA           */
#define SYS_GPA_MFP0_PA2MFP_I2C0_SMBSUS       (0xAUL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for I2C0_SMBSUS        */
#define SYS_GPA_MFP0_PA2MFP_CCAP_DATA2        (0xBUL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for CCAP_DATA2         */
#define SYS_GPA_MFP0_PA2MFP_BPWM0_CH2         (0xCUL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for BPWM0_CH2          */
#define SYS_GPA_MFP0_PA2MFP_EPWM0_CH3         (0xDUL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for EPWM0_CH3          */
#define SYS_GPA_MFP0_PA2MFP_EQEI3_INDEX       (0xEUL<<SYS_GPA_MFP0_PA2MFP_Pos)    /*!< GPA_MFP0 PA2 setting for EQEI3_INDEX        */
#define SYS_GPA_MFP0_PA2MFP_DMIC1_DAT         (0x10UL<<SYS_GPA_MFP0_PA2MFP_Pos)   /*!< GPA_MFP0 PA2 setting for DMIC1_DAT          */
#define SYS_GPA_MFP0_PA2MFP_PSIO0_CH5         (0x11UL<<SYS_GPA_MFP0_PA2MFP_Pos)   /*!< GPA_MFP0 PA2 setting for PSIO0_CH5          */
#define SYS_GPA_MFP0_PA2MFP_I3C0_PUPEN        (0x12UL<<SYS_GPA_MFP0_PA2MFP_Pos)   /*!< GPA_MFP0 PA2 setting for I3C0_PUPEN         */
#define SYS_GPA_MFP0_PA2MFP_UTCPD0_VBSRCEN    (0x13UL<<SYS_GPA_MFP0_PA2MFP_Pos)   /*!< GPA_MFP0 PA2 setting for UTCPD0_VBSRCEN     */
#define SYS_GPA_MFP0_PA2MFP_LPSPI0_CLK        (0x14UL<<SYS_GPA_MFP0_PA2MFP_Pos)   /*!< GPA_MFP0 PA2 setting for LPSPI0_CLK         */

/* PA.3 MFP */
#define SYS_GPA_MFP0_PA3MFP_GPIO              (0x0UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for GPIO               */
#define SYS_GPA_MFP0_PA3MFP_QSPI0_SS          (0x3UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for QSPI0_SS           */
#define SYS_GPA_MFP0_PA3MFP_SPI0_SS           (0x4UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for SPI0_SS            */
#define SYS_GPA_MFP0_PA3MFP_SD1_DAT3          (0x5UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for SD1_DAT3           */
#define SYS_GPA_MFP0_PA3MFP_SC0_PWR           (0x6UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for SC0_PWR            */
#define SYS_GPA_MFP0_PA3MFP_UART4_TXD         (0x7UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for UART4_TXD          */
#define SYS_GPA_MFP0_PA3MFP_UART1_TXD         (0x8UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for UART1_TXD          */
#define SYS_GPA_MFP0_PA3MFP_I2C1_SCL          (0x9UL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for I2C1_SCL           */
#define SYS_GPA_MFP0_PA3MFP_I2C0_SMBAL        (0xAUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for I2C0_SMBAL         */
#define SYS_GPA_MFP0_PA3MFP_CCAP_DATA3        (0xBUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for CCAP_DATA3         */
#define SYS_GPA_MFP0_PA3MFP_BPWM0_CH3         (0xCUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for BPWM0_CH3          */
#define SYS_GPA_MFP0_PA3MFP_EPWM0_CH2         (0xDUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for EPWM0_CH2          */
#define SYS_GPA_MFP0_PA3MFP_EQEI0_B           (0xEUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for EQEI0_B            */
#define SYS_GPA_MFP0_PA3MFP_EPWM1_BRAKE1      (0xFUL<<SYS_GPA_MFP0_PA3MFP_Pos)    /*!< GPA_MFP0 PA3 setting for EPWM1_BRAKE1       */
#define SYS_GPA_MFP0_PA3MFP_DMIC0_CLKLP       (0x10UL<<SYS_GPA_MFP0_PA3MFP_Pos)   /*!< GPA_MFP0 PA3 setting for DMIC0_CLKLP        */
#define SYS_GPA_MFP0_PA3MFP_PSIO0_CH4         (0x11UL<<SYS_GPA_MFP0_PA3MFP_Pos)   /*!< GPA_MFP0 PA3 setting for PSIO0_CH4          */
#define SYS_GPA_MFP0_PA3MFP_UTCPD0_VBSNKEN    (0x13UL<<SYS_GPA_MFP0_PA3MFP_Pos)   /*!< GPA_MFP0 PA3 setting for UTCPD0_VBSNKEN     */
#define SYS_GPA_MFP0_PA3MFP_LPSPI0_SS         (0x14UL<<SYS_GPA_MFP0_PA3MFP_Pos)   /*!< GPA_MFP0 PA3 setting for LPSPI0_SS          */

/* PA.4 MFP */
#define SYS_GPA_MFP1_PA4MFP_GPIO              (0x0UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for GPIO               */
#define SYS_GPA_MFP1_PA4MFP_EBI_AD1           (0x2UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for EBI_AD1            */
#define SYS_GPA_MFP1_PA4MFP_QSPI0_MOSI1       (0x3UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for QSPI0_MOSI1        */
#define SYS_GPA_MFP1_PA4MFP_SPI0_I2SMCLK      (0x4UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for SPI0_I2SMCLK       */
#define SYS_GPA_MFP1_PA4MFP_SD1_CLK           (0x5UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for SD1_CLK            */
#define SYS_GPA_MFP1_PA4MFP_SC0_nCD           (0x6UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for SC0_nCD            */
#define SYS_GPA_MFP1_PA4MFP_UART0_nRTS        (0x7UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for UART0_nRTS         */
#define SYS_GPA_MFP1_PA4MFP_UART5_RXD         (0x8UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for UART5_RXD          */
#define SYS_GPA_MFP1_PA4MFP_I2C0_SDA          (0x9UL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for I2C0_SDA           */
#define SYS_GPA_MFP1_PA4MFP_CANFD0_RXD        (0xAUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for CANFD0_RXD         */
#define SYS_GPA_MFP1_PA4MFP_UART0_RXD         (0xBUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for UART0_RXD          */
#define SYS_GPA_MFP1_PA4MFP_BPWM0_CH4         (0xCUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for BPWM0_CH4          */
#define SYS_GPA_MFP1_PA4MFP_EPWM0_CH1         (0xDUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for EPWM0_CH1          */
#define SYS_GPA_MFP1_PA4MFP_EQEI0_A           (0xEUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for EQEI0_A            */
#define SYS_GPA_MFP1_PA4MFP_CCAP_SCLK         (0xFUL<<SYS_GPA_MFP1_PA4MFP_Pos)    /*!< GPA_MFP1 PA4 setting for CCAP_SCLK          */
#define SYS_GPA_MFP1_PA4MFP_DMIC0_CLK         (0x10UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for DMIC0_CLK          */
#define SYS_GPA_MFP1_PA4MFP_I3C0_SDA          (0x11UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for I3C0_SDA           */
#define SYS_GPA_MFP1_PA4MFP_UTCPD0_VBSRCEN    (0x13UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for UTCPD0_VBSRCEN     */
#define SYS_GPA_MFP1_PA4MFP_LPUART0_RXD       (0x14UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for LPUART0_RXD        */
#define SYS_GPA_MFP1_PA4MFP_LPUART0_nRTS      (0x15UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for LPUART0_nRTS       */
#define SYS_GPA_MFP1_PA4MFP_LPI2C0_SDA        (0x16UL<<SYS_GPA_MFP1_PA4MFP_Pos)   /*!< GPA_MFP1 PA4 setting for LPI2C0_SDA         */

/* PA.5 MFP */
#define SYS_GPA_MFP1_PA5MFP_GPIO              (0x0UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for GPIO               */
#define SYS_GPA_MFP1_PA5MFP_EBI_AD0           (0x2UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for EBI_AD0            */
#define SYS_GPA_MFP1_PA5MFP_QSPI0_MISO1       (0x3UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for QSPI0_MISO1        */
#define SYS_GPA_MFP1_PA5MFP_SPI1_I2SMCLK      (0x4UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for SPI1_I2SMCLK       */
#define SYS_GPA_MFP1_PA5MFP_SD1_CMD           (0x5UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for SD1_CMD            */
#define SYS_GPA_MFP1_PA5MFP_SC2_nCD           (0x6UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for SC2_nCD            */
#define SYS_GPA_MFP1_PA5MFP_UART0_nCTS        (0x7UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for UART0_nCTS         */
#define SYS_GPA_MFP1_PA5MFP_UART5_TXD         (0x8UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for UART5_TXD          */
#define SYS_GPA_MFP1_PA5MFP_I2C0_SCL          (0x9UL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for I2C0_SCL           */
#define SYS_GPA_MFP1_PA5MFP_CANFD0_TXD        (0xAUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for CANFD0_TXD         */
#define SYS_GPA_MFP1_PA5MFP_UART0_TXD         (0xBUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for UART0_TXD          */
#define SYS_GPA_MFP1_PA5MFP_BPWM0_CH5         (0xCUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for BPWM0_CH5          */
#define SYS_GPA_MFP1_PA5MFP_EPWM0_CH0         (0xDUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for EPWM0_CH0          */
#define SYS_GPA_MFP1_PA5MFP_EQEI0_INDEX       (0xEUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for EQEI0_INDEX        */
#define SYS_GPA_MFP1_PA5MFP_CCAP_PIXCLK       (0xFUL<<SYS_GPA_MFP1_PA5MFP_Pos)    /*!< GPA_MFP1 PA5 setting for CCAP_PIXCLK        */
#define SYS_GPA_MFP1_PA5MFP_DMIC0_DAT         (0x10UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for DMIC0_DAT          */
#define SYS_GPA_MFP1_PA5MFP_I3C0_SCL          (0x11UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for I3C0_SCL           */
#define SYS_GPA_MFP1_PA5MFP_UTCPD0_VBSNKEN    (0x13UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for UTCPD0_VBSNKEN     */
#define SYS_GPA_MFP1_PA5MFP_LPUART0_TXD       (0x14UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for LPUART0_TXD        */
#define SYS_GPA_MFP1_PA5MFP_LPUART0_nCTS      (0x15UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for LPUART0_nCTS       */
#define SYS_GPA_MFP1_PA5MFP_LPI2C0_SCL        (0x16UL<<SYS_GPA_MFP1_PA5MFP_Pos)   /*!< GPA_MFP1 PA5 setting for LPI2C0_SCL         */

/* PA.6 MFP */
#define SYS_GPA_MFP1_PA6MFP_GPIO              (0x0UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for GPIO               */
#define SYS_GPA_MFP1_PA6MFP_UTCPD0_DISCHG     (0x2UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for UTCPD0_DISCHG      */
#define SYS_GPA_MFP1_PA6MFP_EMAC0_RMII_RXERR  (0x3UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for EMAC0_RMII_RXERR   */
#define SYS_GPA_MFP1_PA6MFP_SPI1_SS           (0x4UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for SPI1_SS            */
#define SYS_GPA_MFP1_PA6MFP_SD1_nCD           (0x5UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for SD1_nCD            */
#define SYS_GPA_MFP1_PA6MFP_SC2_CLK           (0x6UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for SC2_CLK            */
#define SYS_GPA_MFP1_PA6MFP_UART0_RXD         (0x7UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for UART0_RXD          */
#define SYS_GPA_MFP1_PA6MFP_I2C1_SDA          (0x8UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for I2C1_SDA           */
#define SYS_GPA_MFP1_PA6MFP_QSPI1_MOSI1       (0x9UL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for QSPI1_MOSI1        */
#define SYS_GPA_MFP1_PA6MFP_EPWM1_CH5         (0xBUL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for EPWM1_CH5          */
#define SYS_GPA_MFP1_PA6MFP_BPWM1_CH3         (0xCUL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for BPWM1_CH3          */
#define SYS_GPA_MFP1_PA6MFP_ACMP1_WLAT        (0xDUL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for ACMP1_WLAT         */
#define SYS_GPA_MFP1_PA6MFP_TM3               (0xEUL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for TM3                */
#define SYS_GPA_MFP1_PA6MFP_INT0              (0xFUL<<SYS_GPA_MFP1_PA6MFP_Pos)    /*!< GPA_MFP1 PA6 setting for INT0               */
#define SYS_GPA_MFP1_PA6MFP_UTCPD0_VBSRCEN    (0x11UL<<SYS_GPA_MFP1_PA6MFP_Pos)   /*!< GPA_MFP1 PA6 setting for UTCPD0_VBSRCEN     */
#define SYS_GPA_MFP1_PA6MFP_KPI_COL0          (0x12UL<<SYS_GPA_MFP1_PA6MFP_Pos)   /*!< GPA_MFP1 PA6 setting for KPI_COL0           */
#define SYS_GPA_MFP1_PA6MFP_LPUART0_RXD       (0x15UL<<SYS_GPA_MFP1_PA6MFP_Pos)   /*!< GPA_MFP1 PA6 setting for LPUART0_RXD        */
#define SYS_GPA_MFP1_PA6MFP_LPIO4             (0x17UL<<SYS_GPA_MFP1_PA6MFP_Pos)   /*!< GPA_MFP1 PA6 setting for LPIO4              */

/* PA.7 MFP */
#define SYS_GPA_MFP1_PA7MFP_GPIO              (0x0UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for GPIO               */
#define SYS_GPA_MFP1_PA7MFP_EBI_AD7           (0x2UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for EBI_AD7            */
#define SYS_GPA_MFP1_PA7MFP_EMAC0_RMII_CRSDV  (0x3UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for EMAC0_RMII_CRSDV   */
#define SYS_GPA_MFP1_PA7MFP_SPI1_CLK          (0x4UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for SPI1_CLK           */
#define SYS_GPA_MFP1_PA7MFP_SC2_DAT           (0x6UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for SC2_DAT            */
#define SYS_GPA_MFP1_PA7MFP_UART0_TXD         (0x7UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for UART0_TXD          */
#define SYS_GPA_MFP1_PA7MFP_I2C1_SCL          (0x8UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for I2C1_SCL           */
#define SYS_GPA_MFP1_PA7MFP_QSPI1_MISO1       (0x9UL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for QSPI1_MISO1        */
#define SYS_GPA_MFP1_PA7MFP_EPWM1_CH4         (0xBUL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for EPWM1_CH4          */
#define SYS_GPA_MFP1_PA7MFP_BPWM1_CH2         (0xCUL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for BPWM1_CH2          */
#define SYS_GPA_MFP1_PA7MFP_ACMP0_WLAT        (0xDUL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for ACMP0_WLAT         */
#define SYS_GPA_MFP1_PA7MFP_TM2               (0xEUL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for TM2                */
#define SYS_GPA_MFP1_PA7MFP_INT1              (0xFUL<<SYS_GPA_MFP1_PA7MFP_Pos)    /*!< GPA_MFP1 PA7 setting for INT1               */
#define SYS_GPA_MFP1_PA7MFP_UTCPD0_VBSNKEN    (0x11UL<<SYS_GPA_MFP1_PA7MFP_Pos)   /*!< GPA_MFP1 PA7 setting for UTCPD0_VBSNKEN     */
#define SYS_GPA_MFP1_PA7MFP_KPI_COL1          (0x12UL<<SYS_GPA_MFP1_PA7MFP_Pos)   /*!< GPA_MFP1 PA7 setting for KPI_COL1           */
#define SYS_GPA_MFP1_PA7MFP_LPUART0_TXD       (0x15UL<<SYS_GPA_MFP1_PA7MFP_Pos)   /*!< GPA_MFP1 PA7 setting for LPUART0_TXD        */
#define SYS_GPA_MFP1_PA7MFP_LPIO5             (0x17UL<<SYS_GPA_MFP1_PA7MFP_Pos)   /*!< GPA_MFP1 PA7 setting for LPIO5              */

/* PA.8 MFP */
#define SYS_GPA_MFP2_PA8MFP_GPIO              (0x0UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for GPIO               */
#define SYS_GPA_MFP2_PA8MFP_EADC0_CH20        (0x1UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for EADC0_CH20         */
#define SYS_GPA_MFP2_PA8MFP_LPADC0_CH20       (0x1UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for LPADC0_CH20        */
#define SYS_GPA_MFP2_PA8MFP_EBI_ALE           (0x2UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for EBI_ALE            */
#define SYS_GPA_MFP2_PA8MFP_SC2_CLK           (0x3UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for SC2_CLK            */
#define SYS_GPA_MFP2_PA8MFP_SPI2_MOSI         (0x4UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for SPI2_MOSI          */
#define SYS_GPA_MFP2_PA8MFP_SD1_DAT0          (0x5UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for SD1_DAT0           */
#define SYS_GPA_MFP2_PA8MFP_USCI0_CTL1        (0x6UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for USCI0_CTL1         */
#define SYS_GPA_MFP2_PA8MFP_UART1_RXD         (0x7UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for UART1_RXD          */
#define SYS_GPA_MFP2_PA8MFP_UART7_RXD         (0x8UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for UART7_RXD          */
#define SYS_GPA_MFP2_PA8MFP_BPWM0_CH3         (0x9UL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for BPWM0_CH3          */
#define SYS_GPA_MFP2_PA8MFP_EQEI1_B           (0xAUL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for EQEI1_B            */
#define SYS_GPA_MFP2_PA8MFP_ECAP0_IC2         (0xBUL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for ECAP0_IC2          */
#define SYS_GPA_MFP2_PA8MFP_I2S1_DO           (0xCUL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for I2S1_DO            */
#define SYS_GPA_MFP2_PA8MFP_TM3_EXT           (0xDUL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for TM3_EXT            */
#define SYS_GPA_MFP2_PA8MFP_INT4              (0xFUL<<SYS_GPA_MFP2_PA8MFP_Pos)    /*!< GPA_MFP2 PA8 setting for INT4               */

/* PA.9 MFP */
#define SYS_GPA_MFP2_PA9MFP_GPIO              (0x0UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for GPIO               */
#define SYS_GPA_MFP2_PA9MFP_EADC0_CH21        (0x1UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for EADC0_CH21         */
#define SYS_GPA_MFP2_PA9MFP_LPADC0_CH21       (0x1UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for LPADC0_CH21        */
#define SYS_GPA_MFP2_PA9MFP_EBI_MCLK          (0x2UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for EBI_MCLK           */
#define SYS_GPA_MFP2_PA9MFP_SC2_DAT           (0x3UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for SC2_DAT            */
#define SYS_GPA_MFP2_PA9MFP_SPI2_MISO         (0x4UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for SPI2_MISO          */
#define SYS_GPA_MFP2_PA9MFP_SD1_DAT1          (0x5UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for SD1_DAT1           */
#define SYS_GPA_MFP2_PA9MFP_USCI0_DAT1        (0x6UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for USCI0_DAT1         */
#define SYS_GPA_MFP2_PA9MFP_UART1_TXD         (0x7UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for UART1_TXD          */
#define SYS_GPA_MFP2_PA9MFP_UART7_TXD         (0x8UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for UART7_TXD          */
#define SYS_GPA_MFP2_PA9MFP_BPWM0_CH2         (0x9UL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for BPWM0_CH2          */
#define SYS_GPA_MFP2_PA9MFP_EQEI1_A           (0xAUL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for EQEI1_A            */
#define SYS_GPA_MFP2_PA9MFP_ECAP0_IC1         (0xBUL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for ECAP0_IC1          */
#define SYS_GPA_MFP2_PA9MFP_I2S1_DI           (0xCUL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for I2S1_DI            */
#define SYS_GPA_MFP2_PA9MFP_TM2_EXT           (0xDUL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for TM2_EXT            */
#define SYS_GPA_MFP2_PA9MFP_SWDH_DAT          (0xFUL<<SYS_GPA_MFP2_PA9MFP_Pos)    /*!< GPA_MFP2 PA9 setting for SWDH_DAT           */

/* PA.10 MFP */
#define SYS_GPA_MFP2_PA10MFP_GPIO             (0x0UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for GPIO              */
#define SYS_GPA_MFP2_PA10MFP_ACMP1_P0         (0x1UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for ACMP1_P0          */
#define SYS_GPA_MFP2_PA10MFP_EADC0_CH22       (0x1UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for EADC0_CH22        */
#define SYS_GPA_MFP2_PA10MFP_LPADC0_CH22      (0x1UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for LPADC0_CH22       */
#define SYS_GPA_MFP2_PA10MFP_EBI_nWR          (0x2UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for EBI_nWR           */
#define SYS_GPA_MFP2_PA10MFP_SC2_RST          (0x3UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for SC2_RST           */
#define SYS_GPA_MFP2_PA10MFP_SPI2_CLK         (0x4UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for SPI2_CLK          */
#define SYS_GPA_MFP2_PA10MFP_SD1_DAT2         (0x5UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for SD1_DAT2          */
#define SYS_GPA_MFP2_PA10MFP_USCI0_DAT0       (0x6UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for USCI0_DAT0        */
#define SYS_GPA_MFP2_PA10MFP_I2C2_SDA         (0x7UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for I2C2_SDA          */
#define SYS_GPA_MFP2_PA10MFP_UART6_RXD        (0x8UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for UART6_RXD         */
#define SYS_GPA_MFP2_PA10MFP_BPWM0_CH1        (0x9UL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for BPWM0_CH1         */
#define SYS_GPA_MFP2_PA10MFP_EQEI1_INDEX      (0xAUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for EQEI1_INDEX       */
#define SYS_GPA_MFP2_PA10MFP_ECAP0_IC0        (0xBUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for ECAP0_IC0         */
#define SYS_GPA_MFP2_PA10MFP_I2S1_MCLK        (0xCUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for I2S1_MCLK         */
#define SYS_GPA_MFP2_PA10MFP_TM1_EXT          (0xDUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for TM1_EXT           */
#define SYS_GPA_MFP2_PA10MFP_DAC0_ST          (0xEUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for DAC0_ST           */
#define SYS_GPA_MFP2_PA10MFP_SWDH_CLK         (0xFUL<<SYS_GPA_MFP2_PA10MFP_Pos)   /*!< GPA_MFP2 PA10 setting for SWDH_CLK          */
#define SYS_GPA_MFP2_PA10MFP_KPI_ROW5         (0x12UL<<SYS_GPA_MFP2_PA10MFP_Pos)  /*!< GPA_MFP2 PA10 setting for KPI_ROW5          */
#define SYS_GPA_MFP2_PA10MFP_LPTM1_EXT        (0x17UL<<SYS_GPA_MFP2_PA10MFP_Pos)  /*!< GPA_MFP2 PA10 setting for LPTM1_EXT         */

/* PA.11 MFP */
#define SYS_GPA_MFP2_PA11MFP_GPIO             (0x0UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for GPIO              */
#define SYS_GPA_MFP2_PA11MFP_ACMP0_P0         (0x1UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for ACMP0_P0          */
#define SYS_GPA_MFP2_PA11MFP_EADC0_CH23       (0x1UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for EADC0_CH23        */
#define SYS_GPA_MFP2_PA11MFP_LPADC0_CH23      (0x1UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for LPADC0_CH23       */
#define SYS_GPA_MFP2_PA11MFP_EBI_nRD          (0x2UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for EBI_nRD           */
#define SYS_GPA_MFP2_PA11MFP_SC2_PWR          (0x3UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for SC2_PWR           */
#define SYS_GPA_MFP2_PA11MFP_SPI2_SS          (0x4UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for SPI2_SS           */
#define SYS_GPA_MFP2_PA11MFP_SD1_DAT3         (0x5UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for SD1_DAT3          */
#define SYS_GPA_MFP2_PA11MFP_USCI0_CLK        (0x6UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for USCI0_CLK         */
#define SYS_GPA_MFP2_PA11MFP_I2C2_SCL         (0x7UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for I2C2_SCL          */
#define SYS_GPA_MFP2_PA11MFP_UART6_TXD        (0x8UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for UART6_TXD         */
#define SYS_GPA_MFP2_PA11MFP_BPWM0_CH0        (0x9UL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for BPWM0_CH0         */
#define SYS_GPA_MFP2_PA11MFP_EPWM0_SYNC_OUT   (0xAUL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for EPWM0_SYNC_OUT    */
#define SYS_GPA_MFP2_PA11MFP_EPWM0_BRAKE1     (0xBUL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for EPWM0_BRAKE1      */
#define SYS_GPA_MFP2_PA11MFP_I2S1_BCLK        (0xCUL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for I2S1_BCLK         */
#define SYS_GPA_MFP2_PA11MFP_TM0_EXT          (0xDUL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for TM0_EXT           */
#define SYS_GPA_MFP2_PA11MFP_DAC1_ST          (0xEUL<<SYS_GPA_MFP2_PA11MFP_Pos)   /*!< GPA_MFP2 PA11 setting for DAC1_ST           */
#define SYS_GPA_MFP2_PA11MFP_KPI_ROW4         (0x12UL<<SYS_GPA_MFP2_PA11MFP_Pos)  /*!< GPA_MFP2 PA11 setting for KPI_ROW4          */
#define SYS_GPA_MFP2_PA11MFP_LPTM0_EXT        (0x17UL<<SYS_GPA_MFP2_PA11MFP_Pos)  /*!< GPA_MFP2 PA11 setting for LPTM0_EXT         */

/* PA.12 MFP */
#define SYS_GPA_MFP3_PA12MFP_GPIO             (0x0UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for GPIO              */
#define SYS_GPA_MFP3_PA12MFP_I2S0_BCLK        (0x2UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for I2S0_BCLK         */
#define SYS_GPA_MFP3_PA12MFP_UART4_TXD        (0x3UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for UART4_TXD         */
#define SYS_GPA_MFP3_PA12MFP_I2C1_SCL         (0x4UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for I2C1_SCL          */
#define SYS_GPA_MFP3_PA12MFP_SPI2_SS          (0x5UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for SPI2_SS           */
#define SYS_GPA_MFP3_PA12MFP_CANFD0_TXD       (0x6UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for CANFD0_TXD        */
#define SYS_GPA_MFP3_PA12MFP_SC2_PWR          (0x7UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for SC2_PWR           */
#define SYS_GPA_MFP3_PA12MFP_SD1_nCD          (0x8UL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for SD1_nCD           */
#define SYS_GPA_MFP3_PA12MFP_QSPI1_MISO0      (0xAUL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for QSPI1_MISO0       */
#define SYS_GPA_MFP3_PA12MFP_BPWM1_CH2        (0xBUL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for BPWM1_CH2         */
#define SYS_GPA_MFP3_PA12MFP_EQEI1_INDEX      (0xCUL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for EQEI1_INDEX       */
#define SYS_GPA_MFP3_PA12MFP_ECAP3_IC0        (0xDUL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for ECAP3_IC0         */
#define SYS_GPA_MFP3_PA12MFP_USB_VBUS         (0xEUL<<SYS_GPA_MFP3_PA12MFP_Pos)   /*!< GPA_MFP3 PA12 setting for USB_VBUS          */
#define SYS_GPA_MFP3_PA12MFP_PSIO0_CH4        (0x11UL<<SYS_GPA_MFP3_PA12MFP_Pos)  /*!< GPA_MFP3 PA12 setting for PSIO0_CH4         */
#define SYS_GPA_MFP3_PA12MFP_LPSPI0_SS        (0x14UL<<SYS_GPA_MFP3_PA12MFP_Pos)  /*!< GPA_MFP3 PA12 setting for LPSPI0_SS         */

/* PA.13 MFP */
#define SYS_GPA_MFP3_PA13MFP_GPIO             (0x0UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for GPIO              */
#define SYS_GPA_MFP3_PA13MFP_I2S0_MCLK        (0x2UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for I2S0_MCLK         */
#define SYS_GPA_MFP3_PA13MFP_UART4_RXD        (0x3UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for UART4_RXD         */
#define SYS_GPA_MFP3_PA13MFP_I2C1_SDA         (0x4UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for I2C1_SDA          */
#define SYS_GPA_MFP3_PA13MFP_SPI2_CLK         (0x5UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for SPI2_CLK          */
#define SYS_GPA_MFP3_PA13MFP_CANFD0_RXD       (0x6UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for CANFD0_RXD        */
#define SYS_GPA_MFP3_PA13MFP_SC2_RST          (0x7UL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for SC2_RST           */
#define SYS_GPA_MFP3_PA13MFP_QSPI1_MOSI0      (0xAUL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for QSPI1_MOSI0       */
#define SYS_GPA_MFP3_PA13MFP_BPWM1_CH3        (0xBUL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for BPWM1_CH3         */
#define SYS_GPA_MFP3_PA13MFP_EQEI1_A          (0xCUL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for EQEI1_A           */
#define SYS_GPA_MFP3_PA13MFP_ECAP3_IC1        (0xDUL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for ECAP3_IC1         */
#define SYS_GPA_MFP3_PA13MFP_USB_D_MINUS      (0xEUL<<SYS_GPA_MFP3_PA13MFP_Pos)   /*!< GPA_MFP3 PA13 setting for USB_D_MINUS       */
#define SYS_GPA_MFP3_PA13MFP_PSIO0_CH5        (0x11UL<<SYS_GPA_MFP3_PA13MFP_Pos)  /*!< GPA_MFP3 PA13 setting for PSIO0_CH5         */
#define SYS_GPA_MFP3_PA13MFP_LPSPI0_CLK       (0x14UL<<SYS_GPA_MFP3_PA13MFP_Pos)  /*!< GPA_MFP3 PA13 setting for LPSPI0_CLK        */

/* PA.14 MFP */
#define SYS_GPA_MFP3_PA14MFP_GPIO             (0x0UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for GPIO              */
#define SYS_GPA_MFP3_PA14MFP_I2S0_DI          (0x2UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for I2S0_DI           */
#define SYS_GPA_MFP3_PA14MFP_UART0_TXD        (0x3UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for UART0_TXD         */
#define SYS_GPA_MFP3_PA14MFP_EBI_AD5          (0x4UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for EBI_AD5           */
#define SYS_GPA_MFP3_PA14MFP_SPI2_MISO        (0x5UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for SPI2_MISO         */
#define SYS_GPA_MFP3_PA14MFP_I2C2_SCL         (0x6UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for I2C2_SCL          */
#define SYS_GPA_MFP3_PA14MFP_SC2_DAT          (0x7UL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for SC2_DAT           */
#define SYS_GPA_MFP3_PA14MFP_BPWM1_CH4        (0xBUL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for BPWM1_CH4         */
#define SYS_GPA_MFP3_PA14MFP_EQEI1_B          (0xCUL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for EQEI1_B           */
#define SYS_GPA_MFP3_PA14MFP_ECAP3_IC2        (0xDUL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for ECAP3_IC2         */
#define SYS_GPA_MFP3_PA14MFP_USB_D_PLUS       (0xEUL<<SYS_GPA_MFP3_PA14MFP_Pos)   /*!< GPA_MFP3 PA14 setting for USB_D_PLUS        */
#define SYS_GPA_MFP3_PA14MFP_PSIO0_CH6        (0x11UL<<SYS_GPA_MFP3_PA14MFP_Pos)  /*!< GPA_MFP3 PA14 setting for PSIO0_CH6         */
#define SYS_GPA_MFP3_PA14MFP_LPSPI0_MISO      (0x14UL<<SYS_GPA_MFP3_PA14MFP_Pos)  /*!< GPA_MFP3 PA14 setting for LPSPI0_MISO       */

/* PA.15 MFP */
#define SYS_GPA_MFP3_PA15MFP_GPIO             (0x0UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for GPIO              */
#define SYS_GPA_MFP3_PA15MFP_I2S0_DO          (0x2UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for I2S0_DO           */
#define SYS_GPA_MFP3_PA15MFP_UART0_RXD        (0x3UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for UART0_RXD         */
#define SYS_GPA_MFP3_PA15MFP_SPI2_MOSI        (0x5UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for SPI2_MOSI         */
#define SYS_GPA_MFP3_PA15MFP_I2C2_SDA         (0x6UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for I2C2_SDA          */
#define SYS_GPA_MFP3_PA15MFP_SC2_CLK          (0x7UL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for SC2_CLK           */
#define SYS_GPA_MFP3_PA15MFP_BPWM1_CH5        (0xBUL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for BPWM1_CH5         */
#define SYS_GPA_MFP3_PA15MFP_EPWM0_SYNC_IN    (0xCUL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for EPWM0_SYNC_IN     */
#define SYS_GPA_MFP3_PA15MFP_EQEI3_INDEX      (0xDUL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for EQEI3_INDEX       */
#define SYS_GPA_MFP3_PA15MFP_USB_OTG_ID       (0xEUL<<SYS_GPA_MFP3_PA15MFP_Pos)   /*!< GPA_MFP3 PA15 setting for USB_OTG_ID        */
#define SYS_GPA_MFP3_PA15MFP_PSIO0_CH7        (0x11UL<<SYS_GPA_MFP3_PA15MFP_Pos)  /*!< GPA_MFP3 PA15 setting for PSIO0_CH7         */
#define SYS_GPA_MFP3_PA15MFP_LPSPI0_MOSI      (0x14UL<<SYS_GPA_MFP3_PA15MFP_Pos)  /*!< GPA_MFP3 PA15 setting for LPSPI0_MOSI       */

/* PB.0 MFP */
#define SYS_GPB_MFP0_PB0MFP_GPIO              (0x0UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for GPIO               */
#define SYS_GPB_MFP0_PB0MFP_ACMP3_N           (0x1UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for ACMP3_N            */
#define SYS_GPB_MFP0_PB0MFP_EADC0_CH0         (0x1UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for EADC0_CH0          */
#define SYS_GPB_MFP0_PB0MFP_LPADC0_CH0        (0x1UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for LPADC0_CH0         */
#define SYS_GPB_MFP0_PB0MFP_EBI_ADR9          (0x2UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for EBI_ADR9           */
#define SYS_GPB_MFP0_PB0MFP_SD0_CMD           (0x3UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for SD0_CMD            */
#define SYS_GPB_MFP0_PB0MFP_SPI2_I2SMCLK      (0x4UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for SPI2_I2SMCLK       */
#define SYS_GPB_MFP0_PB0MFP_USCI0_CTL0        (0x6UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for USCI0_CTL0         */
#define SYS_GPB_MFP0_PB0MFP_UART2_RXD         (0x7UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for UART2_RXD          */
#define SYS_GPB_MFP0_PB0MFP_SPI0_I2SMCLK      (0x8UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for SPI0_I2SMCLK       */
#define SYS_GPB_MFP0_PB0MFP_I2C1_SDA          (0x9UL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for I2C1_SDA           */
#define SYS_GPB_MFP0_PB0MFP_I2S1_LRCK         (0xAUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for I2S1_LRCK          */
#define SYS_GPB_MFP0_PB0MFP_EPWM0_CH5         (0xBUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for EPWM0_CH5          */
#define SYS_GPB_MFP0_PB0MFP_EPWM1_CH5         (0xCUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for EPWM1_CH5          */
#define SYS_GPB_MFP0_PB0MFP_EPWM0_BRAKE1      (0xDUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for EPWM0_BRAKE1       */
#define SYS_GPB_MFP0_PB0MFP_ACMP3_O           (0xEUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for ACMP3_O            */
#define SYS_GPB_MFP0_PB0MFP_QSPI0_MOSI1       (0xFUL<<SYS_GPB_MFP0_PB0MFP_Pos)    /*!< GPB_MFP0 PB0 setting for QSPI0_MOSI1        */
#define SYS_GPB_MFP0_PB0MFP_I3C0_SDA          (0x10UL<<SYS_GPB_MFP0_PB0MFP_Pos)   /*!< GPB_MFP0 PB0 setting for I3C0_SDA           */
#define SYS_GPB_MFP0_PB0MFP_UTCPD0_VCNEN2     (0x11UL<<SYS_GPB_MFP0_PB0MFP_Pos)   /*!< GPB_MFP0 PB0 setting for UTCPD0_VCNEN2      */
#define SYS_GPB_MFP0_PB0MFP_KPI_ROW3          (0x12UL<<SYS_GPB_MFP0_PB0MFP_Pos)   /*!< GPB_MFP0 PB0 setting for KPI_ROW3           */
#define SYS_GPB_MFP0_PB0MFP_LPIO2             (0x17UL<<SYS_GPB_MFP0_PB0MFP_Pos)   /*!< GPB_MFP0 PB0 setting for LPIO2              */

/* PB.1 MFP */
#define SYS_GPB_MFP0_PB1MFP_GPIO              (0x0UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for GPIO               */
#define SYS_GPB_MFP0_PB1MFP_ACMP3_P0          (0x1UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for ACMP3_P0           */
#define SYS_GPB_MFP0_PB1MFP_EADC0_CH1         (0x1UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EADC0_CH1          */
#define SYS_GPB_MFP0_PB1MFP_LPADC0_CH1        (0x1UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for LPADC0_CH1         */
#define SYS_GPB_MFP0_PB1MFP_EBI_ADR8          (0x2UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EBI_ADR8           */
#define SYS_GPB_MFP0_PB1MFP_SD0_CLK           (0x3UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for SD0_CLK            */
#define SYS_GPB_MFP0_PB1MFP_EMAC0_RMII_RXERR  (0x4UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EMAC0_RMII_RXERR   */
#define SYS_GPB_MFP0_PB1MFP_SPI1_I2SMCLK      (0x5UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for SPI1_I2SMCLK       */
#define SYS_GPB_MFP0_PB1MFP_SPI3_I2SMCLK      (0x6UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for SPI3_I2SMCLK       */
#define SYS_GPB_MFP0_PB1MFP_UART2_TXD         (0x7UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for UART2_TXD          */
#define SYS_GPB_MFP0_PB1MFP_I2C1_SCL          (0x9UL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for I2C1_SCL           */
#define SYS_GPB_MFP0_PB1MFP_I2S0_LRCK         (0xAUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for I2S0_LRCK          */
#define SYS_GPB_MFP0_PB1MFP_EPWM0_CH4         (0xBUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EPWM0_CH4          */
#define SYS_GPB_MFP0_PB1MFP_EPWM1_CH4         (0xCUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EPWM1_CH4          */
#define SYS_GPB_MFP0_PB1MFP_EPWM0_BRAKE0      (0xDUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for EPWM0_BRAKE0       */
#define SYS_GPB_MFP0_PB1MFP_ACMP2_O           (0xEUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for ACMP2_O            */
#define SYS_GPB_MFP0_PB1MFP_QSPI0_MISO1       (0xFUL<<SYS_GPB_MFP0_PB1MFP_Pos)    /*!< GPB_MFP0 PB1 setting for QSPI0_MISO1        */
#define SYS_GPB_MFP0_PB1MFP_I3C0_SCL          (0x10UL<<SYS_GPB_MFP0_PB1MFP_Pos)   /*!< GPB_MFP0 PB1 setting for I3C0_SCL           */
#define SYS_GPB_MFP0_PB1MFP_UTCPD0_VBDCHG     (0x11UL<<SYS_GPB_MFP0_PB1MFP_Pos)   /*!< GPB_MFP0 PB1 setting for UTCPD0_VBDCHG      */
#define SYS_GPB_MFP0_PB1MFP_KPI_ROW2          (0x12UL<<SYS_GPB_MFP0_PB1MFP_Pos)   /*!< GPB_MFP0 PB1 setting for KPI_ROW2           */
#define SYS_GPB_MFP0_PB1MFP_LPIO3             (0x17UL<<SYS_GPB_MFP0_PB1MFP_Pos)   /*!< GPB_MFP0 PB1 setting for LPIO3              */

/* PB.2 MFP */
#define SYS_GPB_MFP0_PB2MFP_GPIO              (0x0UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for GPIO               */
#define SYS_GPB_MFP0_PB2MFP_ACMP0_P1          (0x1UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for ACMP0_P1           */
#define SYS_GPB_MFP0_PB2MFP_EADC0_CH2         (0x1UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for EADC0_CH2          */
#define SYS_GPB_MFP0_PB2MFP_LPADC0_CH2        (0x1UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for LPADC0_CH2         */
#define SYS_GPB_MFP0_PB2MFP_EBI_ADR3          (0x2UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for EBI_ADR3           */
#define SYS_GPB_MFP0_PB2MFP_SD0_DAT0          (0x3UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for SD0_DAT0           */
#define SYS_GPB_MFP0_PB2MFP_EMAC0_RMII_CRSDV  (0x4UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for EMAC0_RMII_CRSDV   */
#define SYS_GPB_MFP0_PB2MFP_SPI1_SS           (0x5UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for SPI1_SS            */
#define SYS_GPB_MFP0_PB2MFP_UART1_RXD         (0x6UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for UART1_RXD          */
#define SYS_GPB_MFP0_PB2MFP_UART5_nCTS        (0x7UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for UART5_nCTS         */
#define SYS_GPB_MFP0_PB2MFP_SC0_PWR           (0x9UL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for SC0_PWR            */
#define SYS_GPB_MFP0_PB2MFP_I2S0_DO           (0xAUL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for I2S0_DO            */
#define SYS_GPB_MFP0_PB2MFP_EPWM0_CH3         (0xBUL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for EPWM0_CH3          */
#define SYS_GPB_MFP0_PB2MFP_I2C1_SDA          (0xCUL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for I2C1_SDA           */
#define SYS_GPB_MFP0_PB2MFP_TM3               (0xEUL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for TM3                */
#define SYS_GPB_MFP0_PB2MFP_INT3              (0xFUL<<SYS_GPB_MFP0_PB2MFP_Pos)    /*!< GPB_MFP0 PB2 setting for INT3               */
#define SYS_GPB_MFP0_PB2MFP_DMIC1_CLK         (0x10UL<<SYS_GPB_MFP0_PB2MFP_Pos)   /*!< GPB_MFP0 PB2 setting for DMIC1_CLK          */
#define SYS_GPB_MFP0_PB2MFP_PSIO0_CH7         (0x11UL<<SYS_GPB_MFP0_PB2MFP_Pos)   /*!< GPB_MFP0 PB2 setting for PSIO0_CH7          */
#define SYS_GPB_MFP0_PB2MFP_KPI_ROW1          (0x12UL<<SYS_GPB_MFP0_PB2MFP_Pos)   /*!< GPB_MFP0 PB2 setting for KPI_ROW1           */
#define SYS_GPB_MFP0_PB2MFP_LPIO6             (0x17UL<<SYS_GPB_MFP0_PB2MFP_Pos)   /*!< GPB_MFP0 PB2 setting for LPIO6              */

/* PB.3 MFP */
#define SYS_GPB_MFP0_PB3MFP_GPIO              (0x0UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for GPIO               */
#define SYS_GPB_MFP0_PB3MFP_ACMP0_N           (0x1UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for ACMP0_N            */
#define SYS_GPB_MFP0_PB3MFP_EADC0_CH3         (0x1UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for EADC0_CH3          */
#define SYS_GPB_MFP0_PB3MFP_LPADC0_CH3        (0x1UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for LPADC0_CH3         */
#define SYS_GPB_MFP0_PB3MFP_EBI_ADR2          (0x2UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for EBI_ADR2           */
#define SYS_GPB_MFP0_PB3MFP_SD0_DAT1          (0x3UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for SD0_DAT1           */
#define SYS_GPB_MFP0_PB3MFP_EMAC0_RMII_RXD1   (0x4UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for EMAC0_RMII_RXD1    */
#define SYS_GPB_MFP0_PB3MFP_SPI1_CLK          (0x5UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for SPI1_CLK           */
#define SYS_GPB_MFP0_PB3MFP_UART1_TXD         (0x6UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for UART1_TXD          */
#define SYS_GPB_MFP0_PB3MFP_UART5_nRTS        (0x7UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for UART5_nRTS         */
#define SYS_GPB_MFP0_PB3MFP_SC0_RST           (0x9UL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for SC0_RST            */
#define SYS_GPB_MFP0_PB3MFP_I2S0_DI           (0xAUL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for I2S0_DI            */
#define SYS_GPB_MFP0_PB3MFP_EPWM0_CH2         (0xBUL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for EPWM0_CH2          */
#define SYS_GPB_MFP0_PB3MFP_I2C1_SCL          (0xCUL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for I2C1_SCL           */
#define SYS_GPB_MFP0_PB3MFP_TM2               (0xEUL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for TM2                */
#define SYS_GPB_MFP0_PB3MFP_INT2              (0xFUL<<SYS_GPB_MFP0_PB3MFP_Pos)    /*!< GPB_MFP0 PB3 setting for INT2               */
#define SYS_GPB_MFP0_PB3MFP_DMIC1_DAT         (0x10UL<<SYS_GPB_MFP0_PB3MFP_Pos)   /*!< GPB_MFP0 PB3 setting for DMIC1_DAT          */
#define SYS_GPB_MFP0_PB3MFP_PSIO0_CH6         (0x11UL<<SYS_GPB_MFP0_PB3MFP_Pos)   /*!< GPB_MFP0 PB3 setting for PSIO0_CH6          */
#define SYS_GPB_MFP0_PB3MFP_KPI_ROW0          (0x12UL<<SYS_GPB_MFP0_PB3MFP_Pos)   /*!< GPB_MFP0 PB3 setting for KPI_ROW0           */
#define SYS_GPB_MFP0_PB3MFP_LPIO7             (0x17UL<<SYS_GPB_MFP0_PB3MFP_Pos)   /*!< GPB_MFP0 PB3 setting for LPIO7              */

/* PB.4 MFP */
#define SYS_GPB_MFP1_PB4MFP_GPIO              (0x0UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for GPIO               */
#define SYS_GPB_MFP1_PB4MFP_ACMP1_P1          (0x1UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for ACMP1_P1           */
#define SYS_GPB_MFP1_PB4MFP_EADC0_CH4         (0x1UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for EADC0_CH4          */
#define SYS_GPB_MFP1_PB4MFP_LPADC0_CH4        (0x1UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for LPADC0_CH4         */
#define SYS_GPB_MFP1_PB4MFP_EBI_ADR1          (0x2UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for EBI_ADR1           */
#define SYS_GPB_MFP1_PB4MFP_SD0_DAT2          (0x3UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for SD0_DAT2           */
#define SYS_GPB_MFP1_PB4MFP_EMAC0_RMII_RXD0   (0x4UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for EMAC0_RMII_RXD0    */
#define SYS_GPB_MFP1_PB4MFP_SPI1_MOSI         (0x5UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for SPI1_MOSI          */
#define SYS_GPB_MFP1_PB4MFP_I2C0_SDA          (0x6UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for I2C0_SDA           */
#define SYS_GPB_MFP1_PB4MFP_UART5_RXD         (0x7UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for UART5_RXD          */
#define SYS_GPB_MFP1_PB4MFP_SC0_DAT           (0x9UL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for SC0_DAT            */
#define SYS_GPB_MFP1_PB4MFP_I2S0_MCLK         (0xAUL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for I2S0_MCLK          */
#define SYS_GPB_MFP1_PB4MFP_EPWM0_CH1         (0xBUL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for EPWM0_CH1          */
#define SYS_GPB_MFP1_PB4MFP_UART2_RXD         (0xCUL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for UART2_RXD          */
#define SYS_GPB_MFP1_PB4MFP_TM1               (0xEUL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for TM1                */
#define SYS_GPB_MFP1_PB4MFP_INT1              (0xFUL<<SYS_GPB_MFP1_PB4MFP_Pos)    /*!< GPB_MFP1 PB4 setting for INT1               */
#define SYS_GPB_MFP1_PB4MFP_DMIC0_CLK         (0x10UL<<SYS_GPB_MFP1_PB4MFP_Pos)   /*!< GPB_MFP1 PB4 setting for DMIC0_CLK          */
#define SYS_GPB_MFP1_PB4MFP_PSIO0_CH5         (0x11UL<<SYS_GPB_MFP1_PB4MFP_Pos)   /*!< GPB_MFP1 PB4 setting for PSIO0_CH5          */
#define SYS_GPB_MFP1_PB4MFP_KPI_COL7          (0x12UL<<SYS_GPB_MFP1_PB4MFP_Pos)   /*!< GPB_MFP1 PB4 setting for KPI_COL7           */
#define SYS_GPB_MFP1_PB4MFP_LPI2C0_SDA        (0x16UL<<SYS_GPB_MFP1_PB4MFP_Pos)   /*!< GPB_MFP1 PB4 setting for LPI2C0_SDA         */
#define SYS_GPB_MFP1_PB4MFP_LPTM1             (0x17UL<<SYS_GPB_MFP1_PB4MFP_Pos)   /*!< GPB_MFP1 PB4 setting for LPTM1              */

/* PB.5 MFP */
#define SYS_GPB_MFP1_PB5MFP_GPIO              (0x0UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for GPIO               */
#define SYS_GPB_MFP1_PB5MFP_ACMP1_N           (0x1UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for ACMP1_N            */
#define SYS_GPB_MFP1_PB5MFP_EADC0_CH5         (0x1UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for EADC0_CH5          */
#define SYS_GPB_MFP1_PB5MFP_LPADC0_CH5        (0x1UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for LPADC0_CH5         */
#define SYS_GPB_MFP1_PB5MFP_EBI_ADR0          (0x2UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for EBI_ADR0           */
#define SYS_GPB_MFP1_PB5MFP_SD0_DAT3          (0x3UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for SD0_DAT3           */
#define SYS_GPB_MFP1_PB5MFP_EMAC0_RMII_REFCLK (0x4UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for EMAC0_RMII_REFCLK  */
#define SYS_GPB_MFP1_PB5MFP_SPI1_MISO         (0x5UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for SPI1_MISO          */
#define SYS_GPB_MFP1_PB5MFP_I2C0_SCL          (0x6UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for I2C0_SCL           */
#define SYS_GPB_MFP1_PB5MFP_UART5_TXD         (0x7UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for UART5_TXD          */
#define SYS_GPB_MFP1_PB5MFP_SC0_CLK           (0x9UL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for SC0_CLK            */
#define SYS_GPB_MFP1_PB5MFP_I2S0_BCLK         (0xAUL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for I2S0_BCLK          */
#define SYS_GPB_MFP1_PB5MFP_EPWM0_CH0         (0xBUL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for EPWM0_CH0          */
#define SYS_GPB_MFP1_PB5MFP_UART2_TXD         (0xCUL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for UART2_TXD          */
#define SYS_GPB_MFP1_PB5MFP_TM0               (0xEUL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for TM0                */
#define SYS_GPB_MFP1_PB5MFP_INT0              (0xFUL<<SYS_GPB_MFP1_PB5MFP_Pos)    /*!< GPB_MFP1 PB5 setting for INT0               */
#define SYS_GPB_MFP1_PB5MFP_DMIC0_DAT         (0x10UL<<SYS_GPB_MFP1_PB5MFP_Pos)   /*!< GPB_MFP1 PB5 setting for DMIC0_DAT          */
#define SYS_GPB_MFP1_PB5MFP_PSIO0_CH4         (0x11UL<<SYS_GPB_MFP1_PB5MFP_Pos)   /*!< GPB_MFP1 PB5 setting for PSIO0_CH4          */
#define SYS_GPB_MFP1_PB5MFP_KPI_COL6          (0x12UL<<SYS_GPB_MFP1_PB5MFP_Pos)   /*!< GPB_MFP1 PB5 setting for KPI_COL6           */
#define SYS_GPB_MFP1_PB5MFP_LPI2C0_SCL        (0x16UL<<SYS_GPB_MFP1_PB5MFP_Pos)   /*!< GPB_MFP1 PB5 setting for LPI2C0_SCL         */
#define SYS_GPB_MFP1_PB5MFP_LPTM0             (0x17UL<<SYS_GPB_MFP1_PB5MFP_Pos)   /*!< GPB_MFP1 PB5 setting for LPTM0              */

/* PB.6 MFP */
#define SYS_GPB_MFP1_PB6MFP_GPIO              (0x0UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for GPIO               */
#define SYS_GPB_MFP1_PB6MFP_ACMP2_N           (0x1UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for ACMP2_N            */
#define SYS_GPB_MFP1_PB6MFP_EADC0_CH6         (0x1UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EADC0_CH6          */
#define SYS_GPB_MFP1_PB6MFP_LPADC0_CH6        (0x1UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for LPADC0_CH6         */
#define SYS_GPB_MFP1_PB6MFP_EBI_nWRH          (0x2UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EBI_nWRH           */
#define SYS_GPB_MFP1_PB6MFP_EMAC0_PPS         (0x3UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EMAC0_PPS          */
#define SYS_GPB_MFP1_PB6MFP_CANFD1_RXD        (0x5UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for CANFD1_RXD         */
#define SYS_GPB_MFP1_PB6MFP_UART1_RXD         (0x6UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for UART1_RXD          */
#define SYS_GPB_MFP1_PB6MFP_SD1_CLK           (0x7UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for SD1_CLK            */
#define SYS_GPB_MFP1_PB6MFP_EBI_nCS1          (0x8UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EBI_nCS1           */
#define SYS_GPB_MFP1_PB6MFP_SPI0_CLK          (0x9UL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for SPI0_CLK           */
#define SYS_GPB_MFP1_PB6MFP_BPWM1_CH5         (0xAUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for BPWM1_CH5          */
#define SYS_GPB_MFP1_PB6MFP_EPWM1_BRAKE1      (0xBUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EPWM1_BRAKE1       */
#define SYS_GPB_MFP1_PB6MFP_EPWM1_CH5         (0xCUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for EPWM1_CH5          */
#define SYS_GPB_MFP1_PB6MFP_INT4              (0xDUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for INT4               */
#define SYS_GPB_MFP1_PB6MFP_USB_VBUS_EN       (0xEUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for USB_VBUS_EN        */
#define SYS_GPB_MFP1_PB6MFP_ACMP1_O           (0xFUL<<SYS_GPB_MFP1_PB6MFP_Pos)    /*!< GPB_MFP1 PB6 setting for ACMP1_O            */
#define SYS_GPB_MFP1_PB6MFP_DMIC0_CLKLP       (0x10UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for DMIC0_CLKLP        */
#define SYS_GPB_MFP1_PB6MFP_EPWM0_SYNC_IN     (0x11UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for EPWM0_SYNC_IN      */
#define SYS_GPB_MFP1_PB6MFP_KPI_COL5          (0x12UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for KPI_COL5           */
#define SYS_GPB_MFP1_PB6MFP_SC1_nCD           (0x13UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for SC1_nCD            */
#define SYS_GPB_MFP1_PB6MFP_HSUSB_VBUS_EN     (0x14UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for HSUSB_VBUS_EN      */
#define SYS_GPB_MFP1_PB6MFP_TM1               (0x16UL<<SYS_GPB_MFP1_PB6MFP_Pos)   /*!< GPB_MFP1 PB6 setting for TM1                */

/* PB.7 MFP */
#define SYS_GPB_MFP1_PB7MFP_GPIO              (0x0UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for GPIO               */
#define SYS_GPB_MFP1_PB7MFP_ACMP2_P0          (0x1UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for ACMP2_P0           */
#define SYS_GPB_MFP1_PB7MFP_EADC0_CH7         (0x1UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EADC0_CH7          */
#define SYS_GPB_MFP1_PB7MFP_LPADC0_CH7        (0x1UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for LPADC0_CH7         */
#define SYS_GPB_MFP1_PB7MFP_EBI_nWRL          (0x2UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EBI_nWRL           */
#define SYS_GPB_MFP1_PB7MFP_EMAC0_RMII_TXEN   (0x3UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EMAC0_RMII_TXEN    */
#define SYS_GPB_MFP1_PB7MFP_CCAP_DATA0        (0x4UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for CCAP_DATA0         */
#define SYS_GPB_MFP1_PB7MFP_CANFD1_TXD        (0x5UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for CANFD1_TXD         */
#define SYS_GPB_MFP1_PB7MFP_UART1_TXD         (0x6UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for UART1_TXD          */
#define SYS_GPB_MFP1_PB7MFP_SD1_CMD           (0x7UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for SD1_CMD            */
#define SYS_GPB_MFP1_PB7MFP_EBI_nCS0          (0x8UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EBI_nCS0           */
#define SYS_GPB_MFP1_PB7MFP_SPI0_SS           (0x9UL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for SPI0_SS            */
#define SYS_GPB_MFP1_PB7MFP_BPWM1_CH4         (0xAUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for BPWM1_CH4          */
#define SYS_GPB_MFP1_PB7MFP_EPWM1_BRAKE0      (0xBUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EPWM1_BRAKE0       */
#define SYS_GPB_MFP1_PB7MFP_EPWM1_CH4         (0xCUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for EPWM1_CH4          */
#define SYS_GPB_MFP1_PB7MFP_INT5              (0xDUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for INT5               */
#define SYS_GPB_MFP1_PB7MFP_USB_VBUS_ST       (0xEUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for USB_VBUS_ST        */
#define SYS_GPB_MFP1_PB7MFP_ACMP0_O           (0xFUL<<SYS_GPB_MFP1_PB7MFP_Pos)    /*!< GPB_MFP1 PB7 setting for ACMP0_O            */
#define SYS_GPB_MFP1_PB7MFP_ETMC_TRACE_DATA3  (0x10UL<<SYS_GPB_MFP1_PB7MFP_Pos)   /*!< GPB_MFP1 PB7 setting for ETMC_TRACE_DATA3   */
#define SYS_GPB_MFP1_PB7MFP_KPI_COL4          (0x12UL<<SYS_GPB_MFP1_PB7MFP_Pos)   /*!< GPB_MFP1 PB7 setting for KPI_COL4           */
#define SYS_GPB_MFP1_PB7MFP_TRACE_SWO         (0x13UL<<SYS_GPB_MFP1_PB7MFP_Pos)   /*!< GPB_MFP1 PB7 setting for TRACE_SWO          */
#define SYS_GPB_MFP1_PB7MFP_HSUSB_VBUS_ST     (0x14UL<<SYS_GPB_MFP1_PB7MFP_Pos)   /*!< GPB_MFP1 PB7 setting for HSUSB_VBUS_ST      */

/* PB.8 MFP */
#define SYS_GPB_MFP2_PB8MFP_GPIO              (0x0UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for GPIO               */
#define SYS_GPB_MFP2_PB8MFP_ACMP2_P1          (0x1UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for ACMP2_P1           */
#define SYS_GPB_MFP2_PB8MFP_EADC0_CH8         (0x1UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for EADC0_CH8          */
#define SYS_GPB_MFP2_PB8MFP_LPADC0_CH8        (0x1UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for LPADC0_CH8         */
#define SYS_GPB_MFP2_PB8MFP_EBI_ADR19         (0x2UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for EBI_ADR19          */
#define SYS_GPB_MFP2_PB8MFP_EMAC0_RMII_TXD1   (0x3UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for EMAC0_RMII_TXD1    */
#define SYS_GPB_MFP2_PB8MFP_CCAP_DATA1        (0x4UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for CCAP_DATA1         */
#define SYS_GPB_MFP2_PB8MFP_UART0_RXD         (0x5UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for UART0_RXD          */
#define SYS_GPB_MFP2_PB8MFP_UART1_nRTS        (0x6UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for UART1_nRTS         */
#define SYS_GPB_MFP2_PB8MFP_I2C1_SMBSUS       (0x7UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for I2C1_SMBSUS        */
#define SYS_GPB_MFP2_PB8MFP_UART7_RXD         (0x8UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for UART7_RXD          */
#define SYS_GPB_MFP2_PB8MFP_I2C0_SDA          (0x9UL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for I2C0_SDA           */
#define SYS_GPB_MFP2_PB8MFP_BPWM1_CH3         (0xAUL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for BPWM1_CH3          */
#define SYS_GPB_MFP2_PB8MFP_SPI3_MOSI         (0xBUL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for SPI3_MOSI          */
#define SYS_GPB_MFP2_PB8MFP_INT6              (0xDUL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for INT6               */
#define SYS_GPB_MFP2_PB8MFP_USB_VBUS_ST       (0xEUL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for USB_VBUS_ST        */
#define SYS_GPB_MFP2_PB8MFP_USB_VBUS_EN       (0xFUL<<SYS_GPB_MFP2_PB8MFP_Pos)    /*!< GPB_MFP2 PB8 setting for USB_VBUS_EN        */
#define SYS_GPB_MFP2_PB8MFP_ETMC_TRACE_DATA2  (0x10UL<<SYS_GPB_MFP2_PB8MFP_Pos)   /*!< GPB_MFP2 PB8 setting for ETMC_TRACE_DATA2   */
#define SYS_GPB_MFP2_PB8MFP_LPUART0_RXD       (0x15UL<<SYS_GPB_MFP2_PB8MFP_Pos)   /*!< GPB_MFP2 PB8 setting for LPUART0_RXD        */

/* PB.9 MFP */
#define SYS_GPB_MFP2_PB9MFP_GPIO              (0x0UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for GPIO               */
#define SYS_GPB_MFP2_PB9MFP_ACMP2_P2          (0x1UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for ACMP2_P2           */
#define SYS_GPB_MFP2_PB9MFP_EADC0_CH9         (0x1UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for EADC0_CH9          */
#define SYS_GPB_MFP2_PB9MFP_LPADC0_CH9        (0x1UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for LPADC0_CH9         */
#define SYS_GPB_MFP2_PB9MFP_EBI_ADR18         (0x2UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for EBI_ADR18          */
#define SYS_GPB_MFP2_PB9MFP_EMAC0_RMII_TXD0   (0x3UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for EMAC0_RMII_TXD0    */
#define SYS_GPB_MFP2_PB9MFP_UART0_TXD         (0x5UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for UART0_TXD          */
#define SYS_GPB_MFP2_PB9MFP_UART1_nCTS        (0x6UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for UART1_nCTS         */
#define SYS_GPB_MFP2_PB9MFP_I2C1_SMBAL        (0x7UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for I2C1_SMBAL         */
#define SYS_GPB_MFP2_PB9MFP_UART7_TXD         (0x8UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for UART7_TXD          */
#define SYS_GPB_MFP2_PB9MFP_I2C0_SCL          (0x9UL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for I2C0_SCL           */
#define SYS_GPB_MFP2_PB9MFP_BPWM1_CH2         (0xAUL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for BPWM1_CH2          */
#define SYS_GPB_MFP2_PB9MFP_SPI3_MISO         (0xBUL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for SPI3_MISO          */
#define SYS_GPB_MFP2_PB9MFP_INT7              (0xDUL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for INT7               */
#define SYS_GPB_MFP2_PB9MFP_CCAP_HSYNC        (0xEUL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for CCAP_HSYNC         */
#define SYS_GPB_MFP2_PB9MFP_USB_VBUS_ST       (0xFUL<<SYS_GPB_MFP2_PB9MFP_Pos)    /*!< GPB_MFP2 PB9 setting for USB_VBUS_ST        */
#define SYS_GPB_MFP2_PB9MFP_ETMC_TRACE_DATA1  (0x10UL<<SYS_GPB_MFP2_PB9MFP_Pos)   /*!< GPB_MFP2 PB9 setting for ETMC_TRACE_DATA1   */
#define SYS_GPB_MFP2_PB9MFP_LPUART0_TXD       (0x15UL<<SYS_GPB_MFP2_PB9MFP_Pos)   /*!< GPB_MFP2 PB9 setting for LPUART0_TXD        */

/* PB.10 MFP */
#define SYS_GPB_MFP2_PB10MFP_GPIO             (0x0UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for GPIO              */
#define SYS_GPB_MFP2_PB10MFP_ACMP2_P3         (0x1UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for ACMP2_P3          */
#define SYS_GPB_MFP2_PB10MFP_EADC0_CH10       (0x1UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for EADC0_CH10        */
#define SYS_GPB_MFP2_PB10MFP_LPADC0_CH10      (0x1UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for LPADC0_CH10       */
#define SYS_GPB_MFP2_PB10MFP_EBI_ADR17        (0x2UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for EBI_ADR17         */
#define SYS_GPB_MFP2_PB10MFP_EMAC0_RMII_MDIO  (0x3UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for EMAC0_RMII_MDIO   */
#define SYS_GPB_MFP2_PB10MFP_UART0_nRTS       (0x5UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for UART0_nRTS        */
#define SYS_GPB_MFP2_PB10MFP_UART4_RXD        (0x6UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for UART4_RXD         */
#define SYS_GPB_MFP2_PB10MFP_I2C1_SDA         (0x7UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for I2C1_SDA          */
#define SYS_GPB_MFP2_PB10MFP_CANFD0_RXD       (0x8UL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for CANFD0_RXD        */
#define SYS_GPB_MFP2_PB10MFP_BPWM1_CH1        (0xAUL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for BPWM1_CH1         */
#define SYS_GPB_MFP2_PB10MFP_SPI3_SS          (0xBUL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for SPI3_SS           */
#define SYS_GPB_MFP2_PB10MFP_CCAP_VSYNC       (0xCUL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for CCAP_VSYNC        */
#define SYS_GPB_MFP2_PB10MFP_HSUSB_VBUS_EN    (0xEUL<<SYS_GPB_MFP2_PB10MFP_Pos)   /*!< GPB_MFP2 PB10 setting for HSUSB_VBUS_EN     */
#define SYS_GPB_MFP2_PB10MFP_ETMC_TRACE_DATA0 (0x10UL<<SYS_GPB_MFP2_PB10MFP_Pos)  /*!< GPB_MFP2 PB10 setting for ETMC_TRACE_DATA0  */
#define SYS_GPB_MFP2_PB10MFP_LPUART0_nRTS     (0x15UL<<SYS_GPB_MFP2_PB10MFP_Pos)  /*!< GPB_MFP2 PB10 setting for LPUART0_nRTS      */

/* PB.11 MFP */
#define SYS_GPB_MFP2_PB11MFP_GPIO             (0x0UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for GPIO              */
#define SYS_GPB_MFP2_PB11MFP_EADC0_CH11       (0x1UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for EADC0_CH11        */
#define SYS_GPB_MFP2_PB11MFP_LPADC0_CH11      (0x1UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for LPADC0_CH11       */
#define SYS_GPB_MFP2_PB11MFP_EBI_ADR16        (0x2UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for EBI_ADR16         */
#define SYS_GPB_MFP2_PB11MFP_EMAC0_RMII_MDC   (0x3UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for EMAC0_RMII_MDC    */
#define SYS_GPB_MFP2_PB11MFP_UART0_nCTS       (0x5UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for UART0_nCTS        */
#define SYS_GPB_MFP2_PB11MFP_UART4_TXD        (0x6UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for UART4_TXD         */
#define SYS_GPB_MFP2_PB11MFP_I2C1_SCL         (0x7UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for I2C1_SCL          */
#define SYS_GPB_MFP2_PB11MFP_CANFD0_TXD       (0x8UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for CANFD0_TXD        */
#define SYS_GPB_MFP2_PB11MFP_SPI0_I2SMCLK     (0x9UL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for SPI0_I2SMCLK      */
#define SYS_GPB_MFP2_PB11MFP_BPWM1_CH0        (0xAUL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for BPWM1_CH0         */
#define SYS_GPB_MFP2_PB11MFP_SPI3_CLK         (0xBUL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for SPI3_CLK          */
#define SYS_GPB_MFP2_PB11MFP_CCAP_SFIELD      (0xCUL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for CCAP_SFIELD       */
#define SYS_GPB_MFP2_PB11MFP_HSUSB_VBUS_ST    (0xEUL<<SYS_GPB_MFP2_PB11MFP_Pos)   /*!< GPB_MFP2 PB11 setting for HSUSB_VBUS_ST     */
#define SYS_GPB_MFP2_PB11MFP_ETMC_TRACE_CLK   (0x10UL<<SYS_GPB_MFP2_PB11MFP_Pos)  /*!< GPB_MFP2 PB11 setting for ETMC_TRACE_CLK    */
#define SYS_GPB_MFP2_PB11MFP_LPUART0_nCTS     (0x15UL<<SYS_GPB_MFP2_PB11MFP_Pos)  /*!< GPB_MFP2 PB11 setting for LPUART0_nCTS      */

/* PB.12 MFP */
#define SYS_GPB_MFP3_PB12MFP_GPIO             (0x0UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for GPIO              */
#define SYS_GPB_MFP3_PB12MFP_ACMP0_P2         (0x1UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for ACMP0_P2          */
#define SYS_GPB_MFP3_PB12MFP_ACMP1_P2         (0x1UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for ACMP1_P2          */
#define SYS_GPB_MFP3_PB12MFP_DAC0_OUT         (0x1UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for DAC0_OUT          */
#define SYS_GPB_MFP3_PB12MFP_EADC0_CH12       (0x1UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for EADC0_CH12        */
#define SYS_GPB_MFP3_PB12MFP_LPADC0_CH12      (0x1UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for LPADC0_CH12       */
#define SYS_GPB_MFP3_PB12MFP_EBI_AD15         (0x2UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for EBI_AD15          */
#define SYS_GPB_MFP3_PB12MFP_SC1_CLK          (0x3UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for SC1_CLK           */
#define SYS_GPB_MFP3_PB12MFP_SPI0_MOSI        (0x4UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for SPI0_MOSI         */
#define SYS_GPB_MFP3_PB12MFP_USCI0_CLK        (0x5UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for USCI0_CLK         */
#define SYS_GPB_MFP3_PB12MFP_UART0_RXD        (0x6UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for UART0_RXD         */
#define SYS_GPB_MFP3_PB12MFP_UART3_nCTS       (0x7UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for UART3_nCTS        */
#define SYS_GPB_MFP3_PB12MFP_I2C2_SDA         (0x8UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for I2C2_SDA          */
#define SYS_GPB_MFP3_PB12MFP_SD0_nCD          (0x9UL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for SD0_nCD           */
#define SYS_GPB_MFP3_PB12MFP_CCAP_SCLK        (0xAUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for CCAP_SCLK         */
#define SYS_GPB_MFP3_PB12MFP_EPWM1_CH3        (0xBUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for EPWM1_CH3         */
#define SYS_GPB_MFP3_PB12MFP_ETMC_TRACE_DATA3 (0xCUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for ETMC_TRACE_DATA3  */
#define SYS_GPB_MFP3_PB12MFP_TM3_EXT          (0xDUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for TM3_EXT           */
#define SYS_GPB_MFP3_PB12MFP_EBI_AD11         (0xEUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for EBI_AD11          */
#define SYS_GPB_MFP3_PB12MFP_SPI0_I2SMCLK     (0xFUL<<SYS_GPB_MFP3_PB12MFP_Pos)   /*!< GPB_MFP3 PB12 setting for SPI0_I2SMCLK      */
#define SYS_GPB_MFP3_PB12MFP_SWODEC_SWO       (0x10UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for SWODEC_SWO        */
#define SYS_GPB_MFP3_PB12MFP_PSIO0_CH3        (0x11UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for PSIO0_CH3         */
#define SYS_GPB_MFP3_PB12MFP_KPI_COL3         (0x12UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for KPI_COL3          */
#define SYS_GPB_MFP3_PB12MFP_QSPI0_CLK        (0x13UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for QSPI0_CLK         */
#define SYS_GPB_MFP3_PB12MFP_LPSPI0_MOSI      (0x14UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for LPSPI0_MOSI       */
#define SYS_GPB_MFP3_PB12MFP_LPUART0_RXD      (0x15UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for LPUART0_RXD       */
#define SYS_GPB_MFP3_PB12MFP_UTCPD0_VBSRCEN   (0x16UL<<SYS_GPB_MFP3_PB12MFP_Pos)  /*!< GPB_MFP3 PB12 setting for UTCPD0_VBSRCEN    */

/* PB.13 MFP */
#define SYS_GPB_MFP3_PB13MFP_GPIO             (0x0UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for GPIO              */
#define SYS_GPB_MFP3_PB13MFP_ACMP0_P3         (0x1UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for ACMP0_P3          */
#define SYS_GPB_MFP3_PB13MFP_ACMP1_P3         (0x1UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for ACMP1_P3          */
#define SYS_GPB_MFP3_PB13MFP_DAC1_OUT         (0x1UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for DAC1_OUT          */
#define SYS_GPB_MFP3_PB13MFP_EADC0_CH13       (0x1UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for EADC0_CH13        */
#define SYS_GPB_MFP3_PB13MFP_LPADC0_CH13      (0x1UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for LPADC0_CH13       */
#define SYS_GPB_MFP3_PB13MFP_EBI_AD14         (0x2UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for EBI_AD14          */
#define SYS_GPB_MFP3_PB13MFP_SC1_DAT          (0x3UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for SC1_DAT           */
#define SYS_GPB_MFP3_PB13MFP_SPI0_MISO        (0x4UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for SPI0_MISO         */
#define SYS_GPB_MFP3_PB13MFP_USCI0_DAT0       (0x5UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for USCI0_DAT0        */
#define SYS_GPB_MFP3_PB13MFP_UART0_TXD        (0x6UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for UART0_TXD         */
#define SYS_GPB_MFP3_PB13MFP_UART3_nRTS       (0x7UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for UART3_nRTS        */
#define SYS_GPB_MFP3_PB13MFP_I2C2_SCL         (0x8UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for I2C2_SCL          */
#define SYS_GPB_MFP3_PB13MFP_CLKO             (0x9UL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for CLKO              */
#define SYS_GPB_MFP3_PB13MFP_CCAP_PIXCLK      (0xAUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for CCAP_PIXCLK       */
#define SYS_GPB_MFP3_PB13MFP_EPWM1_CH2        (0xBUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for EPWM1_CH2         */
#define SYS_GPB_MFP3_PB13MFP_ETMC_TRACE_DATA2 (0xCUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for ETMC_TRACE_DATA2  */
#define SYS_GPB_MFP3_PB13MFP_TM2_EXT          (0xDUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for TM2_EXT           */
#define SYS_GPB_MFP3_PB13MFP_USB_VBUS_ST      (0xEUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for USB_VBUS_ST       */
#define SYS_GPB_MFP3_PB13MFP_HSUSB_VBUS_ST    (0xFUL<<SYS_GPB_MFP3_PB13MFP_Pos)   /*!< GPB_MFP3 PB13 setting for HSUSB_VBUS_ST     */
#define SYS_GPB_MFP3_PB13MFP_PSIO0_CH2        (0x11UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for PSIO0_CH2         */
#define SYS_GPB_MFP3_PB13MFP_KPI_COL2         (0x12UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for KPI_COL2          */
#define SYS_GPB_MFP3_PB13MFP_USCI0_CTL0       (0x13UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for USCI0_CTL0        */
#define SYS_GPB_MFP3_PB13MFP_LPSPI0_MISO      (0x14UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for LPSPI0_MISO       */
#define SYS_GPB_MFP3_PB13MFP_LPUART0_TXD      (0x15UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for LPUART0_TXD       */
#define SYS_GPB_MFP3_PB13MFP_UTCPD0_VBSNKEN   (0x16UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for UTCPD0_VBSNKEN    */
#define SYS_GPB_MFP3_PB13MFP_LPTM1            (0x17UL<<SYS_GPB_MFP3_PB13MFP_Pos)  /*!< GPB_MFP3 PB13 setting for LPTM1             */

/* PB.14 MFP */
#define SYS_GPB_MFP3_PB14MFP_GPIO             (0x0UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for GPIO              */
#define SYS_GPB_MFP3_PB14MFP_EADC0_CH14       (0x1UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for EADC0_CH14        */
#define SYS_GPB_MFP3_PB14MFP_LPADC0_CH14      (0x1UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for LPADC0_CH14       */
#define SYS_GPB_MFP3_PB14MFP_EBI_AD13         (0x2UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for EBI_AD13          */
#define SYS_GPB_MFP3_PB14MFP_SC1_RST          (0x3UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for SC1_RST           */
#define SYS_GPB_MFP3_PB14MFP_SPI0_CLK         (0x4UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for SPI0_CLK          */
#define SYS_GPB_MFP3_PB14MFP_USCI0_DAT1       (0x5UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for USCI0_DAT1        */
#define SYS_GPB_MFP3_PB14MFP_UART0_nRTS       (0x6UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for UART0_nRTS        */
#define SYS_GPB_MFP3_PB14MFP_UART3_RXD        (0x7UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for UART3_RXD         */
#define SYS_GPB_MFP3_PB14MFP_I2C2_SMBSUS      (0x8UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for I2C2_SMBSUS       */
#define SYS_GPB_MFP3_PB14MFP_CCAP_DATA0       (0x9UL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for CCAP_DATA0        */
#define SYS_GPB_MFP3_PB14MFP_EPWM1_CH1        (0xBUL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for EPWM1_CH1         */
#define SYS_GPB_MFP3_PB14MFP_ETMC_TRACE_DATA1 (0xCUL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for ETMC_TRACE_DATA1  */
#define SYS_GPB_MFP3_PB14MFP_TM1_EXT          (0xDUL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for TM1_EXT           */
#define SYS_GPB_MFP3_PB14MFP_CLKO             (0xEUL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for CLKO              */
#define SYS_GPB_MFP3_PB14MFP_USB_VBUS_ST      (0xFUL<<SYS_GPB_MFP3_PB14MFP_Pos)   /*!< GPB_MFP3 PB14 setting for USB_VBUS_ST       */
#define SYS_GPB_MFP3_PB14MFP_PSIO0_CH1        (0x11UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for PSIO0_CH1         */
#define SYS_GPB_MFP3_PB14MFP_KPI_COL1         (0x12UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for KPI_COL1          */
#define SYS_GPB_MFP3_PB14MFP_UTCPD0_VBSRCEN   (0x13UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for UTCPD0_VBSRCEN    */
#define SYS_GPB_MFP3_PB14MFP_LPSPI0_CLK       (0x14UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for LPSPI0_CLK        */
#define SYS_GPB_MFP3_PB14MFP_LPUART0_nRTS     (0x15UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for LPUART0_nRTS      */
#define SYS_GPB_MFP3_PB14MFP_LPTM1_EXT        (0x17UL<<SYS_GPB_MFP3_PB14MFP_Pos)  /*!< GPB_MFP3 PB14 setting for LPTM1_EXT         */

/* PB.15 MFP */
#define SYS_GPB_MFP3_PB15MFP_GPIO             (0x0UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for GPIO              */
#define SYS_GPB_MFP3_PB15MFP_EADC0_CH15       (0x1UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for EADC0_CH15        */
#define SYS_GPB_MFP3_PB15MFP_LPADC0_CH15      (0x1UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for LPADC0_CH15       */
#define SYS_GPB_MFP3_PB15MFP_EBI_AD12         (0x2UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for EBI_AD12          */
#define SYS_GPB_MFP3_PB15MFP_SC1_PWR          (0x3UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for SC1_PWR           */
#define SYS_GPB_MFP3_PB15MFP_SPI0_SS          (0x4UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for SPI0_SS           */
#define SYS_GPB_MFP3_PB15MFP_USCI0_CTL1       (0x5UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for USCI0_CTL1        */
#define SYS_GPB_MFP3_PB15MFP_UART0_nCTS       (0x6UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for UART0_nCTS        */
#define SYS_GPB_MFP3_PB15MFP_UART3_TXD        (0x7UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for UART3_TXD         */
#define SYS_GPB_MFP3_PB15MFP_I2C2_SMBAL       (0x8UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for I2C2_SMBAL        */
#define SYS_GPB_MFP3_PB15MFP_CCAP_DATA1       (0x9UL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for CCAP_DATA1        */
#define SYS_GPB_MFP3_PB15MFP_EPWM0_BRAKE1     (0xAUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for EPWM0_BRAKE1      */
#define SYS_GPB_MFP3_PB15MFP_EPWM1_CH0        (0xBUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for EPWM1_CH0         */
#define SYS_GPB_MFP3_PB15MFP_ETMC_TRACE_DATA0 (0xCUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for ETMC_TRACE_DATA0  */
#define SYS_GPB_MFP3_PB15MFP_TM0_EXT          (0xDUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for TM0_EXT           */
#define SYS_GPB_MFP3_PB15MFP_USB_VBUS_EN      (0xEUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for USB_VBUS_EN       */
#define SYS_GPB_MFP3_PB15MFP_HSUSB_VBUS_EN    (0xFUL<<SYS_GPB_MFP3_PB15MFP_Pos)   /*!< GPB_MFP3 PB15 setting for HSUSB_VBUS_EN     */
#define SYS_GPB_MFP3_PB15MFP_PSIO0_CH0        (0x11UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for PSIO0_CH0         */
#define SYS_GPB_MFP3_PB15MFP_KPI_COL0         (0x12UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for KPI_COL0          */
#define SYS_GPB_MFP3_PB15MFP_UTCPD0_VBSNKEN   (0x13UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for UTCPD0_VBSNKEN    */
#define SYS_GPB_MFP3_PB15MFP_LPSPI0_SS        (0x14UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for LPSPI0_SS         */
#define SYS_GPB_MFP3_PB15MFP_LPUART0_nCTS     (0x15UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for LPUART0_nCTS      */
#define SYS_GPB_MFP3_PB15MFP_LPTM0_EXT        (0x17UL<<SYS_GPB_MFP3_PB15MFP_Pos)  /*!< GPB_MFP3 PB15 setting for LPTM0_EXT         */

/* PC.0 MFP */
#define SYS_GPC_MFP0_PC0MFP_GPIO              (0x0UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for GPIO               */
#define SYS_GPC_MFP0_PC0MFP_EBI_AD0           (0x2UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for EBI_AD0            */
#define SYS_GPC_MFP0_PC0MFP_QSPI0_MOSI0       (0x4UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for QSPI0_MOSI0        */
#define SYS_GPC_MFP0_PC0MFP_SC1_CLK           (0x5UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for SC1_CLK            */
#define SYS_GPC_MFP0_PC0MFP_I2S0_LRCK         (0x6UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for I2S0_LRCK          */
#define SYS_GPC_MFP0_PC0MFP_SPI1_SS           (0x7UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for SPI1_SS            */
#define SYS_GPC_MFP0_PC0MFP_UART2_RXD         (0x8UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for UART2_RXD          */
#define SYS_GPC_MFP0_PC0MFP_I2C0_SDA          (0x9UL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for I2C0_SDA           */
#define SYS_GPC_MFP0_PC0MFP_EPWM1_CH5         (0xCUL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for EPWM1_CH5          */
#define SYS_GPC_MFP0_PC0MFP_CCAP_DATA0        (0xDUL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for CCAP_DATA0         */
#define SYS_GPC_MFP0_PC0MFP_ACMP1_O           (0xEUL<<SYS_GPC_MFP0_PC0MFP_Pos)    /*!< GPC_MFP0 PC0 setting for ACMP1_O            */
#define SYS_GPC_MFP0_PC0MFP_KPI_ROW5          (0x12UL<<SYS_GPC_MFP0_PC0MFP_Pos)   /*!< GPC_MFP0 PC0 setting for KPI_ROW5           */
#define SYS_GPC_MFP0_PC0MFP_UTCPD0_CC1        (0x13UL<<SYS_GPC_MFP0_PC0MFP_Pos)   /*!< GPC_MFP0 PC0 setting for UTCPD0_CC1         */
#define SYS_GPC_MFP0_PC0MFP_LPI2C0_SDA        (0x16UL<<SYS_GPC_MFP0_PC0MFP_Pos)   /*!< GPC_MFP0 PC0 setting for LPI2C0_SDA         */
#define SYS_GPC_MFP0_PC0MFP_LPIO4             (0x17UL<<SYS_GPC_MFP0_PC0MFP_Pos)   /*!< GPC_MFP0 PC0 setting for LPIO4              */

/* PC.1 MFP */
#define SYS_GPC_MFP0_PC1MFP_GPIO              (0x0UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for GPIO               */
#define SYS_GPC_MFP0_PC1MFP_EBI_AD1           (0x2UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for EBI_AD1            */
#define SYS_GPC_MFP0_PC1MFP_QSPI0_MISO0       (0x4UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for QSPI0_MISO0        */
#define SYS_GPC_MFP0_PC1MFP_SC1_DAT           (0x5UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for SC1_DAT            */
#define SYS_GPC_MFP0_PC1MFP_I2S0_DO           (0x6UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for I2S0_DO            */
#define SYS_GPC_MFP0_PC1MFP_SPI1_CLK          (0x7UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for SPI1_CLK           */
#define SYS_GPC_MFP0_PC1MFP_UART2_TXD         (0x8UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for UART2_TXD          */
#define SYS_GPC_MFP0_PC1MFP_I2C0_SCL          (0x9UL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for I2C0_SCL           */
#define SYS_GPC_MFP0_PC1MFP_EPWM1_CH4         (0xCUL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for EPWM1_CH4          */
#define SYS_GPC_MFP0_PC1MFP_CCAP_DATA1        (0xDUL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for CCAP_DATA1         */
#define SYS_GPC_MFP0_PC1MFP_ACMP0_O           (0xEUL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for ACMP0_O            */
#define SYS_GPC_MFP0_PC1MFP_EADC0_ST          (0xFUL<<SYS_GPC_MFP0_PC1MFP_Pos)    /*!< GPC_MFP0 PC1 setting for EADC0_ST           */
#define SYS_GPC_MFP0_PC1MFP_KPI_ROW4          (0x12UL<<SYS_GPC_MFP0_PC1MFP_Pos)   /*!< GPC_MFP0 PC1 setting for KPI_ROW4           */
#define SYS_GPC_MFP0_PC1MFP_UTCPD0_CC2        (0x13UL<<SYS_GPC_MFP0_PC1MFP_Pos)   /*!< GPC_MFP0 PC1 setting for UTCPD0_CC2         */
#define SYS_GPC_MFP0_PC1MFP_LPADC0_ST         (0x15UL<<SYS_GPC_MFP0_PC1MFP_Pos)   /*!< GPC_MFP0 PC1 setting for LPADC0_ST          */
#define SYS_GPC_MFP0_PC1MFP_LPI2C0_SCL        (0x16UL<<SYS_GPC_MFP0_PC1MFP_Pos)   /*!< GPC_MFP0 PC1 setting for LPI2C0_SCL         */
#define SYS_GPC_MFP0_PC1MFP_LPIO5             (0x17UL<<SYS_GPC_MFP0_PC1MFP_Pos)   /*!< GPC_MFP0 PC1 setting for LPIO5              */

/* PC.2 MFP */
#define SYS_GPC_MFP0_PC2MFP_GPIO              (0x0UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for GPIO               */
#define SYS_GPC_MFP0_PC2MFP_EBI_AD2           (0x2UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for EBI_AD2            */
#define SYS_GPC_MFP0_PC2MFP_QSPI0_CLK         (0x4UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for QSPI0_CLK          */
#define SYS_GPC_MFP0_PC2MFP_SC1_RST           (0x5UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for SC1_RST            */
#define SYS_GPC_MFP0_PC2MFP_I2S0_DI           (0x6UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for I2S0_DI            */
#define SYS_GPC_MFP0_PC2MFP_SPI1_MOSI         (0x7UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for SPI1_MOSI          */
#define SYS_GPC_MFP0_PC2MFP_UART2_nCTS        (0x8UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for UART2_nCTS         */
#define SYS_GPC_MFP0_PC2MFP_I2C0_SMBSUS       (0x9UL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for I2C0_SMBSUS        */
#define SYS_GPC_MFP0_PC2MFP_CANFD1_RXD        (0xAUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for CANFD1_RXD         */
#define SYS_GPC_MFP0_PC2MFP_UART3_RXD         (0xBUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for UART3_RXD          */
#define SYS_GPC_MFP0_PC2MFP_EPWM1_CH3         (0xCUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for EPWM1_CH3          */
#define SYS_GPC_MFP0_PC2MFP_CCAP_DATA2        (0xDUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for CCAP_DATA2         */
#define SYS_GPC_MFP0_PC2MFP_QSPI1_MOSI0       (0xEUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for QSPI1_MOSI0        */
#define SYS_GPC_MFP0_PC2MFP_I2C3_SDA          (0xFUL<<SYS_GPC_MFP0_PC2MFP_Pos)    /*!< GPC_MFP0 PC2 setting for I2C3_SDA           */
#define SYS_GPC_MFP0_PC2MFP_PSIO0_CH3         (0x11UL<<SYS_GPC_MFP0_PC2MFP_Pos)   /*!< GPC_MFP0 PC2 setting for PSIO0_CH3          */
#define SYS_GPC_MFP0_PC2MFP_KPI_ROW3          (0x12UL<<SYS_GPC_MFP0_PC2MFP_Pos)   /*!< GPC_MFP0 PC2 setting for KPI_ROW3           */
#define SYS_GPC_MFP0_PC2MFP_UTCPD0_CCDB1      (0x13UL<<SYS_GPC_MFP0_PC2MFP_Pos)   /*!< GPC_MFP0 PC2 setting for UTCPD0_CCDB1       */

/* PC.3 MFP */
#define SYS_GPC_MFP0_PC3MFP_GPIO              (0x0UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for GPIO               */
#define SYS_GPC_MFP0_PC3MFP_EBI_AD3           (0x2UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for EBI_AD3            */
#define SYS_GPC_MFP0_PC3MFP_QSPI0_SS          (0x4UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for QSPI0_SS           */
#define SYS_GPC_MFP0_PC3MFP_SC1_PWR           (0x5UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for SC1_PWR            */
#define SYS_GPC_MFP0_PC3MFP_I2S0_MCLK         (0x6UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for I2S0_MCLK          */
#define SYS_GPC_MFP0_PC3MFP_SPI1_MISO         (0x7UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for SPI1_MISO          */
#define SYS_GPC_MFP0_PC3MFP_UART2_nRTS        (0x8UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for UART2_nRTS         */
#define SYS_GPC_MFP0_PC3MFP_I2C0_SMBAL        (0x9UL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for I2C0_SMBAL         */
#define SYS_GPC_MFP0_PC3MFP_CANFD1_TXD        (0xAUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for CANFD1_TXD         */
#define SYS_GPC_MFP0_PC3MFP_UART3_TXD         (0xBUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for UART3_TXD          */
#define SYS_GPC_MFP0_PC3MFP_EPWM1_CH2         (0xCUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for EPWM1_CH2          */
#define SYS_GPC_MFP0_PC3MFP_CCAP_DATA3        (0xDUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for CCAP_DATA3         */
#define SYS_GPC_MFP0_PC3MFP_QSPI1_MISO0       (0xEUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for QSPI1_MISO0        */
#define SYS_GPC_MFP0_PC3MFP_I2C3_SCL          (0xFUL<<SYS_GPC_MFP0_PC3MFP_Pos)    /*!< GPC_MFP0 PC3 setting for I2C3_SCL           */
#define SYS_GPC_MFP0_PC3MFP_PSIO0_CH2         (0x11UL<<SYS_GPC_MFP0_PC3MFP_Pos)   /*!< GPC_MFP0 PC3 setting for PSIO0_CH2          */
#define SYS_GPC_MFP0_PC3MFP_KPI_ROW2          (0x12UL<<SYS_GPC_MFP0_PC3MFP_Pos)   /*!< GPC_MFP0 PC3 setting for KPI_ROW2           */
#define SYS_GPC_MFP0_PC3MFP_UTCPD0_CCDB2      (0x13UL<<SYS_GPC_MFP0_PC3MFP_Pos)   /*!< GPC_MFP0 PC3 setting for UTCPD0_CCDB2       */

/* PC.4 MFP */
#define SYS_GPC_MFP1_PC4MFP_GPIO              (0x0UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for GPIO               */
#define SYS_GPC_MFP1_PC4MFP_EBI_AD4           (0x2UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for EBI_AD4            */
#define SYS_GPC_MFP1_PC4MFP_QSPI0_MOSI1       (0x4UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for QSPI0_MOSI1        */
#define SYS_GPC_MFP1_PC4MFP_SC1_nCD           (0x5UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for SC1_nCD            */
#define SYS_GPC_MFP1_PC4MFP_I2S0_BCLK         (0x6UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for I2S0_BCLK          */
#define SYS_GPC_MFP1_PC4MFP_SPI1_I2SMCLK      (0x7UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for SPI1_I2SMCLK       */
#define SYS_GPC_MFP1_PC4MFP_UART2_RXD         (0x8UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for UART2_RXD          */
#define SYS_GPC_MFP1_PC4MFP_I2C1_SDA          (0x9UL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for I2C1_SDA           */
#define SYS_GPC_MFP1_PC4MFP_CANFD0_RXD        (0xAUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for CANFD0_RXD         */
#define SYS_GPC_MFP1_PC4MFP_UART4_RXD         (0xBUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for UART4_RXD          */
#define SYS_GPC_MFP1_PC4MFP_EPWM1_CH1         (0xCUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for EPWM1_CH1          */
#define SYS_GPC_MFP1_PC4MFP_CCAP_DATA4        (0xDUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for CCAP_DATA4         */
#define SYS_GPC_MFP1_PC4MFP_QSPI1_CLK         (0xEUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for QSPI1_CLK          */
#define SYS_GPC_MFP1_PC4MFP_I2C3_SMBSUS       (0xFUL<<SYS_GPC_MFP1_PC4MFP_Pos)    /*!< GPC_MFP1 PC4 setting for I2C3_SMBSUS        */
#define SYS_GPC_MFP1_PC4MFP_PSIO0_CH1         (0x11UL<<SYS_GPC_MFP1_PC4MFP_Pos)   /*!< GPC_MFP1 PC4 setting for PSIO0_CH1          */
#define SYS_GPC_MFP1_PC4MFP_KPI_ROW1          (0x12UL<<SYS_GPC_MFP1_PC4MFP_Pos)   /*!< GPC_MFP1 PC4 setting for KPI_ROW1           */
#define SYS_GPC_MFP1_PC4MFP_UTCPD0_FRSTX1     (0x13UL<<SYS_GPC_MFP1_PC4MFP_Pos)   /*!< GPC_MFP1 PC4 setting for UTCPD0_FRSTX1      */
#define SYS_GPC_MFP1_PC4MFP_UTCPD0_DISCHG     (0x14UL<<SYS_GPC_MFP1_PC4MFP_Pos)   /*!< GPC_MFP1 PC4 setting for UTCPD0_DISCHG      */

/* PC.5 MFP */
#define SYS_GPC_MFP1_PC5MFP_GPIO              (0x0UL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for GPIO               */
#define SYS_GPC_MFP1_PC5MFP_EBI_AD5           (0x2UL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for EBI_AD5            */
#define SYS_GPC_MFP1_PC5MFP_QSPI0_MISO1       (0x4UL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for QSPI0_MISO1        */
#define SYS_GPC_MFP1_PC5MFP_UART2_TXD         (0x8UL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for UART2_TXD          */
#define SYS_GPC_MFP1_PC5MFP_I2C1_SCL          (0x9UL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for I2C1_SCL           */
#define SYS_GPC_MFP1_PC5MFP_CANFD0_TXD        (0xAUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for CANFD0_TXD         */
#define SYS_GPC_MFP1_PC5MFP_UART4_TXD         (0xBUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for UART4_TXD          */
#define SYS_GPC_MFP1_PC5MFP_EPWM1_CH0         (0xCUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for EPWM1_CH0          */
#define SYS_GPC_MFP1_PC5MFP_CCAP_DATA5        (0xDUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for CCAP_DATA5         */
#define SYS_GPC_MFP1_PC5MFP_QSPI1_SS          (0xEUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for QSPI1_SS           */
#define SYS_GPC_MFP1_PC5MFP_I2C3_SMBAL        (0xFUL<<SYS_GPC_MFP1_PC5MFP_Pos)    /*!< GPC_MFP1 PC5 setting for I2C3_SMBAL         */
#define SYS_GPC_MFP1_PC5MFP_PSIO0_CH0         (0x11UL<<SYS_GPC_MFP1_PC5MFP_Pos)   /*!< GPC_MFP1 PC5 setting for PSIO0_CH0          */
#define SYS_GPC_MFP1_PC5MFP_KPI_ROW0          (0x12UL<<SYS_GPC_MFP1_PC5MFP_Pos)   /*!< GPC_MFP1 PC5 setting for KPI_ROW0           */
#define SYS_GPC_MFP1_PC5MFP_UTCPD0_FRSTX2     (0x13UL<<SYS_GPC_MFP1_PC5MFP_Pos)   /*!< GPC_MFP1 PC5 setting for UTCPD0_FRSTX2      */
#define SYS_GPC_MFP1_PC5MFP_UTCPD0_DISCHG     (0x14UL<<SYS_GPC_MFP1_PC5MFP_Pos)   /*!< GPC_MFP1 PC5 setting for UTCPD0_DISCHG      */
#define SYS_GPC_MFP1_PC5MFP_I3C0_PUPEN        (0x15UL<<SYS_GPC_MFP1_PC5MFP_Pos)   /*!< GPC_MFP1 PC5 setting for I3C0_PUPEN         */

/* PC.6 MFP */
#define SYS_GPC_MFP1_PC6MFP_GPIO              (0x0UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for GPIO               */
#define SYS_GPC_MFP1_PC6MFP_EBI_AD8           (0x2UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for EBI_AD8            */
#define SYS_GPC_MFP1_PC6MFP_EMAC0_RMII_RXD1   (0x3UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for EMAC0_RMII_RXD1    */
#define SYS_GPC_MFP1_PC6MFP_SPI1_MOSI         (0x4UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for SPI1_MOSI          */
#define SYS_GPC_MFP1_PC6MFP_UART4_RXD         (0x5UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for UART4_RXD          */
#define SYS_GPC_MFP1_PC6MFP_SC2_RST           (0x6UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for SC2_RST            */
#define SYS_GPC_MFP1_PC6MFP_UART0_nRTS        (0x7UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for UART0_nRTS         */
#define SYS_GPC_MFP1_PC6MFP_I2C1_SMBSUS       (0x8UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for I2C1_SMBSUS        */
#define SYS_GPC_MFP1_PC6MFP_UART6_RXD         (0x9UL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for UART6_RXD          */
#define SYS_GPC_MFP1_PC6MFP_ACMP3_WLAT        (0xAUL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for ACMP3_WLAT         */
#define SYS_GPC_MFP1_PC6MFP_EPWM1_CH3         (0xBUL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for EPWM1_CH3          */
#define SYS_GPC_MFP1_PC6MFP_BPWM1_CH1         (0xCUL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for BPWM1_CH1          */
#define SYS_GPC_MFP1_PC6MFP_TM1               (0xEUL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for TM1                */
#define SYS_GPC_MFP1_PC6MFP_INT2              (0xFUL<<SYS_GPC_MFP1_PC6MFP_Pos)    /*!< GPC_MFP1 PC6 setting for INT2               */
#define SYS_GPC_MFP1_PC6MFP_UTCPD0_FRSTX2     (0x11UL<<SYS_GPC_MFP1_PC6MFP_Pos)   /*!< GPC_MFP1 PC6 setting for UTCPD0_FRSTX2      */
#define SYS_GPC_MFP1_PC6MFP_KPI_COL2          (0x12UL<<SYS_GPC_MFP1_PC6MFP_Pos)   /*!< GPC_MFP1 PC6 setting for KPI_COL2           */
#define SYS_GPC_MFP1_PC6MFP_UTCPD0_DISCHG     (0x13UL<<SYS_GPC_MFP1_PC6MFP_Pos)   /*!< GPC_MFP1 PC6 setting for UTCPD0_DISCHG      */
#define SYS_GPC_MFP1_PC6MFP_LPUART0_nRTS      (0x15UL<<SYS_GPC_MFP1_PC6MFP_Pos)   /*!< GPC_MFP1 PC6 setting for LPUART0_nRTS       */
#define SYS_GPC_MFP1_PC6MFP_LPTM1             (0x17UL<<SYS_GPC_MFP1_PC6MFP_Pos)   /*!< GPC_MFP1 PC6 setting for LPTM1              */

/* PC.7 MFP */
#define SYS_GPC_MFP1_PC7MFP_GPIO              (0x0UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for GPIO               */
#define SYS_GPC_MFP1_PC7MFP_EBI_AD9           (0x2UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for EBI_AD9            */
#define SYS_GPC_MFP1_PC7MFP_EMAC0_RMII_RXD0   (0x3UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for EMAC0_RMII_RXD0    */
#define SYS_GPC_MFP1_PC7MFP_SPI1_MISO         (0x4UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for SPI1_MISO          */
#define SYS_GPC_MFP1_PC7MFP_UART4_TXD         (0x5UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for UART4_TXD          */
#define SYS_GPC_MFP1_PC7MFP_SC2_PWR           (0x6UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for SC2_PWR            */
#define SYS_GPC_MFP1_PC7MFP_UART0_nCTS        (0x7UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for UART0_nCTS         */
#define SYS_GPC_MFP1_PC7MFP_I2C1_SMBAL        (0x8UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for I2C1_SMBAL         */
#define SYS_GPC_MFP1_PC7MFP_UART6_TXD         (0x9UL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for UART6_TXD          */
#define SYS_GPC_MFP1_PC7MFP_ACMP2_WLAT        (0xAUL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for ACMP2_WLAT         */
#define SYS_GPC_MFP1_PC7MFP_EPWM1_CH2         (0xBUL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for EPWM1_CH2          */
#define SYS_GPC_MFP1_PC7MFP_BPWM1_CH0         (0xCUL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for BPWM1_CH0          */
#define SYS_GPC_MFP1_PC7MFP_TM0               (0xEUL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for TM0                */
#define SYS_GPC_MFP1_PC7MFP_INT3              (0xFUL<<SYS_GPC_MFP1_PC7MFP_Pos)    /*!< GPC_MFP1 PC7 setting for INT3               */
#define SYS_GPC_MFP1_PC7MFP_KPI_COL3          (0x12UL<<SYS_GPC_MFP1_PC7MFP_Pos)   /*!< GPC_MFP1 PC7 setting for KPI_COL3           */
#define SYS_GPC_MFP1_PC7MFP_LPUART0_nCTS      (0x15UL<<SYS_GPC_MFP1_PC7MFP_Pos)   /*!< GPC_MFP1 PC7 setting for LPUART0_nCTS       */
#define SYS_GPC_MFP1_PC7MFP_LPTM0             (0x17UL<<SYS_GPC_MFP1_PC7MFP_Pos)   /*!< GPC_MFP1 PC7 setting for LPTM0              */

/* PC.8 MFP */
#define SYS_GPC_MFP2_PC8MFP_GPIO              (0x0UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for GPIO               */
#define SYS_GPC_MFP2_PC8MFP_EBI_ADR16         (0x2UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for EBI_ADR16          */
#define SYS_GPC_MFP2_PC8MFP_EMAC0_RMII_REFCLK (0x3UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for EMAC0_RMII_REFCLK  */
#define SYS_GPC_MFP2_PC8MFP_I2C0_SDA          (0x4UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for I2C0_SDA           */
#define SYS_GPC_MFP2_PC8MFP_UART4_nCTS        (0x5UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for UART4_nCTS         */
#define SYS_GPC_MFP2_PC8MFP_UART1_RXD         (0x8UL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for UART1_RXD          */
#define SYS_GPC_MFP2_PC8MFP_EPWM1_CH1         (0xBUL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for EPWM1_CH1          */
#define SYS_GPC_MFP2_PC8MFP_BPWM1_CH4         (0xCUL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for BPWM1_CH4          */
#define SYS_GPC_MFP2_PC8MFP_SWODEC_SWO        (0xEUL<<SYS_GPC_MFP2_PC8MFP_Pos)    /*!< GPC_MFP2 PC8 setting for SWODEC_SWO         */
#define SYS_GPC_MFP2_PC8MFP_KPI_COL4          (0x12UL<<SYS_GPC_MFP2_PC8MFP_Pos)   /*!< GPC_MFP2 PC8 setting for KPI_COL4           */
#define SYS_GPC_MFP2_PC8MFP_LPI2C0_SDA        (0x16UL<<SYS_GPC_MFP2_PC8MFP_Pos)   /*!< GPC_MFP2 PC8 setting for LPI2C0_SDA         */

/* PC.9 MFP */
#define SYS_GPC_MFP2_PC9MFP_GPIO              (0x0UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for GPIO               */
#define SYS_GPC_MFP2_PC9MFP_ACMP3_P1          (0x1UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for ACMP3_P1           */
#define SYS_GPC_MFP2_PC9MFP_EBI_ADR7          (0x2UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for EBI_ADR7           */
#define SYS_GPC_MFP2_PC9MFP_UART6_nCTS        (0x5UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for UART6_nCTS         */
#define SYS_GPC_MFP2_PC9MFP_SPI3_SS           (0x6UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for SPI3_SS            */
#define SYS_GPC_MFP2_PC9MFP_UART3_RXD         (0x7UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for UART3_RXD          */
#define SYS_GPC_MFP2_PC9MFP_CANFD1_RXD        (0x9UL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for CANFD1_RXD         */
#define SYS_GPC_MFP2_PC9MFP_EPWM1_CH3         (0xCUL<<SYS_GPC_MFP2_PC9MFP_Pos)    /*!< GPC_MFP2 PC9 setting for EPWM1_CH3          */
#define SYS_GPC_MFP2_PC9MFP_I3C0_PUPEN        (0x12UL<<SYS_GPC_MFP2_PC9MFP_Pos)   /*!< GPC_MFP2 PC9 setting for I3C0_PUPEN         */

/* PC.10 MFP */
#define SYS_GPC_MFP2_PC10MFP_GPIO             (0x0UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for GPIO              */
#define SYS_GPC_MFP2_PC10MFP_ACMP3_P2         (0x1UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for ACMP3_P2          */
#define SYS_GPC_MFP2_PC10MFP_EBI_ADR6         (0x2UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for EBI_ADR6          */
#define SYS_GPC_MFP2_PC10MFP_UART6_nRTS       (0x5UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for UART6_nRTS        */
#define SYS_GPC_MFP2_PC10MFP_SPI3_CLK         (0x6UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for SPI3_CLK          */
#define SYS_GPC_MFP2_PC10MFP_UART3_TXD        (0x7UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for UART3_TXD         */
#define SYS_GPC_MFP2_PC10MFP_CANFD1_TXD       (0x9UL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for CANFD1_TXD        */
#define SYS_GPC_MFP2_PC10MFP_ECAP1_IC0        (0xBUL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for ECAP1_IC0         */
#define SYS_GPC_MFP2_PC10MFP_EPWM1_CH2        (0xCUL<<SYS_GPC_MFP2_PC10MFP_Pos)   /*!< GPC_MFP2 PC10 setting for EPWM1_CH2         */

/* PC.11 MFP */
#define SYS_GPC_MFP2_PC11MFP_GPIO             (0x0UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for GPIO              */
#define SYS_GPC_MFP2_PC11MFP_ACMP3_P3         (0x1UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for ACMP3_P3          */
#define SYS_GPC_MFP2_PC11MFP_EBI_ADR5         (0x2UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for EBI_ADR5          */
#define SYS_GPC_MFP2_PC11MFP_UART0_RXD        (0x3UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for UART0_RXD         */
#define SYS_GPC_MFP2_PC11MFP_I2C0_SDA         (0x4UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for I2C0_SDA          */
#define SYS_GPC_MFP2_PC11MFP_UART6_RXD        (0x5UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for UART6_RXD         */
#define SYS_GPC_MFP2_PC11MFP_SPI3_MOSI        (0x6UL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for SPI3_MOSI         */
#define SYS_GPC_MFP2_PC11MFP_ECAP1_IC1        (0xBUL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for ECAP1_IC1         */
#define SYS_GPC_MFP2_PC11MFP_EPWM1_CH1        (0xCUL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for EPWM1_CH1         */
#define SYS_GPC_MFP2_PC11MFP_ACMP1_O          (0xEUL<<SYS_GPC_MFP2_PC11MFP_Pos)   /*!< GPC_MFP2 PC11 setting for ACMP1_O           */
#define SYS_GPC_MFP2_PC11MFP_LPUART0_RXD      (0x15UL<<SYS_GPC_MFP2_PC11MFP_Pos)  /*!< GPC_MFP2 PC11 setting for LPUART0_RXD       */
#define SYS_GPC_MFP2_PC11MFP_LPI2C0_SDA       (0x16UL<<SYS_GPC_MFP2_PC11MFP_Pos)  /*!< GPC_MFP2 PC11 setting for LPI2C0_SDA        */

/* PC.12 MFP */
#define SYS_GPC_MFP3_PC12MFP_GPIO             (0x0UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for GPIO              */
#define SYS_GPC_MFP3_PC12MFP_EBI_ADR4         (0x2UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for EBI_ADR4          */
#define SYS_GPC_MFP3_PC12MFP_UART0_TXD        (0x3UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for UART0_TXD         */
#define SYS_GPC_MFP3_PC12MFP_I2C0_SCL         (0x4UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for I2C0_SCL          */
#define SYS_GPC_MFP3_PC12MFP_UART6_TXD        (0x5UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for UART6_TXD         */
#define SYS_GPC_MFP3_PC12MFP_SPI3_MISO        (0x6UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for SPI3_MISO         */
#define SYS_GPC_MFP3_PC12MFP_SC0_nCD          (0x9UL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for SC0_nCD           */
#define SYS_GPC_MFP3_PC12MFP_ECAP1_IC2        (0xBUL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for ECAP1_IC2         */
#define SYS_GPC_MFP3_PC12MFP_EPWM1_CH0        (0xCUL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for EPWM1_CH0         */
#define SYS_GPC_MFP3_PC12MFP_ACMP0_O          (0xEUL<<SYS_GPC_MFP3_PC12MFP_Pos)   /*!< GPC_MFP3 PC12 setting for ACMP0_O           */
#define SYS_GPC_MFP3_PC12MFP_LPUART0_TXD      (0x15UL<<SYS_GPC_MFP3_PC12MFP_Pos)  /*!< GPC_MFP3 PC12 setting for LPUART0_TXD       */
#define SYS_GPC_MFP3_PC12MFP_LPI2C0_SCL       (0x16UL<<SYS_GPC_MFP3_PC12MFP_Pos)  /*!< GPC_MFP3 PC12 setting for LPI2C0_SCL        */

/* PC.13 MFP */
#define SYS_GPC_MFP3_PC13MFP_GPIO             (0x0UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for GPIO              */
#define SYS_GPC_MFP3_PC13MFP_EADC0_CH19       (0x1UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for EADC0_CH19        */
#define SYS_GPC_MFP3_PC13MFP_LPADC0_CH19      (0x1UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for LPADC0_CH19       */
#define SYS_GPC_MFP3_PC13MFP_EBI_ADR10        (0x2UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for EBI_ADR10         */
#define SYS_GPC_MFP3_PC13MFP_SC2_nCD          (0x3UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for SC2_nCD           */
#define SYS_GPC_MFP3_PC13MFP_SPI2_I2SMCLK     (0x4UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for SPI2_I2SMCLK      */
#define SYS_GPC_MFP3_PC13MFP_CANFD1_TXD       (0x5UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for CANFD1_TXD        */
#define SYS_GPC_MFP3_PC13MFP_USCI0_CTL0       (0x6UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for USCI0_CTL0        */
#define SYS_GPC_MFP3_PC13MFP_UART2_TXD        (0x7UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for UART2_TXD         */
#define SYS_GPC_MFP3_PC13MFP_UART8_nCTS       (0x8UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for UART8_nCTS        */
#define SYS_GPC_MFP3_PC13MFP_BPWM0_CH4        (0x9UL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for BPWM0_CH4         */
#define SYS_GPC_MFP3_PC13MFP_CLKO             (0xDUL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for CLKO              */
#define SYS_GPC_MFP3_PC13MFP_EADC0_ST         (0xEUL<<SYS_GPC_MFP3_PC13MFP_Pos)   /*!< GPC_MFP3 PC13 setting for EADC0_ST          */
#define SYS_GPC_MFP3_PC13MFP_LPADC0_ST        (0x17UL<<SYS_GPC_MFP3_PC13MFP_Pos)  /*!< GPC_MFP3 PC13 setting for LPADC0_ST         */

/* PD.0 MFP */
#define SYS_GPD_MFP0_PD0MFP_GPIO              (0x0UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for GPIO               */
#define SYS_GPD_MFP0_PD0MFP_EBI_AD13          (0x2UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for EBI_AD13           */
#define SYS_GPD_MFP0_PD0MFP_USCI0_CLK         (0x3UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for USCI0_CLK          */
#define SYS_GPD_MFP0_PD0MFP_SPI0_MOSI         (0x4UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for SPI0_MOSI          */
#define SYS_GPD_MFP0_PD0MFP_UART3_RXD         (0x5UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for UART3_RXD          */
#define SYS_GPD_MFP0_PD0MFP_I2C2_SDA          (0x6UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for I2C2_SDA           */
#define SYS_GPD_MFP0_PD0MFP_SC2_CLK           (0x7UL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for SC2_CLK            */
#define SYS_GPD_MFP0_PD0MFP_I2S1_DO           (0xAUL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for I2S1_DO            */
#define SYS_GPD_MFP0_PD0MFP_EQEI2_A           (0xCUL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for EQEI2_A            */
#define SYS_GPD_MFP0_PD0MFP_ECAP2_IC1         (0xDUL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for ECAP2_IC1          */
#define SYS_GPD_MFP0_PD0MFP_TM2               (0xEUL<<SYS_GPD_MFP0_PD0MFP_Pos)    /*!< GPD_MFP0 PD0 setting for TM2                */
#define SYS_GPD_MFP0_PD0MFP_LPSPI0_MOSI       (0x14UL<<SYS_GPD_MFP0_PD0MFP_Pos)   /*!< GPD_MFP0 PD0 setting for LPSPI0_MOSI        */
#define SYS_GPD_MFP0_PD0MFP_LPIO6             (0x17UL<<SYS_GPD_MFP0_PD0MFP_Pos)   /*!< GPD_MFP0 PD0 setting for LPIO6              */

/* PD.1 MFP */
#define SYS_GPD_MFP0_PD1MFP_GPIO              (0x0UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for GPIO               */
#define SYS_GPD_MFP0_PD1MFP_EBI_AD12          (0x2UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for EBI_AD12           */
#define SYS_GPD_MFP0_PD1MFP_USCI0_DAT0        (0x3UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for USCI0_DAT0         */
#define SYS_GPD_MFP0_PD1MFP_SPI0_MISO         (0x4UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for SPI0_MISO          */
#define SYS_GPD_MFP0_PD1MFP_UART3_TXD         (0x5UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for UART3_TXD          */
#define SYS_GPD_MFP0_PD1MFP_I2C2_SCL          (0x6UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for I2C2_SCL           */
#define SYS_GPD_MFP0_PD1MFP_SC2_DAT           (0x7UL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for SC2_DAT            */
#define SYS_GPD_MFP0_PD1MFP_I2S1_DI           (0xAUL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for I2S1_DI            */
#define SYS_GPD_MFP0_PD1MFP_EQEI2_INDEX       (0xCUL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for EQEI2_INDEX        */
#define SYS_GPD_MFP0_PD1MFP_ECAP2_IC0         (0xDUL<<SYS_GPD_MFP0_PD1MFP_Pos)    /*!< GPD_MFP0 PD1 setting for ECAP2_IC0          */
#define SYS_GPD_MFP0_PD1MFP_LPSPI0_MISO       (0x14UL<<SYS_GPD_MFP0_PD1MFP_Pos)   /*!< GPD_MFP0 PD1 setting for LPSPI0_MISO        */
#define SYS_GPD_MFP0_PD1MFP_LPIO7             (0x17UL<<SYS_GPD_MFP0_PD1MFP_Pos)   /*!< GPD_MFP0 PD1 setting for LPIO7              */

/* PD.2 MFP */
#define SYS_GPD_MFP0_PD2MFP_GPIO              (0x0UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for GPIO               */
#define SYS_GPD_MFP0_PD2MFP_EBI_AD11          (0x2UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for EBI_AD11           */
#define SYS_GPD_MFP0_PD2MFP_USCI0_DAT1        (0x3UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for USCI0_DAT1         */
#define SYS_GPD_MFP0_PD2MFP_SPI0_CLK          (0x4UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for SPI0_CLK           */
#define SYS_GPD_MFP0_PD2MFP_UART3_nCTS        (0x5UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for UART3_nCTS         */
#define SYS_GPD_MFP0_PD2MFP_SC2_RST           (0x7UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for SC2_RST            */
#define SYS_GPD_MFP0_PD2MFP_UART0_RXD         (0x9UL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for UART0_RXD          */
#define SYS_GPD_MFP0_PD2MFP_I2S1_MCLK         (0xAUL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for I2S1_MCLK          */
#define SYS_GPD_MFP0_PD2MFP_EQEI3_B           (0xDUL<<SYS_GPD_MFP0_PD2MFP_Pos)    /*!< GPD_MFP0 PD2 setting for EQEI3_B            */
#define SYS_GPD_MFP0_PD2MFP_LPSPI0_CLK        (0x14UL<<SYS_GPD_MFP0_PD2MFP_Pos)   /*!< GPD_MFP0 PD2 setting for LPSPI0_CLK         */
#define SYS_GPD_MFP0_PD2MFP_LPUART0_RXD       (0x15UL<<SYS_GPD_MFP0_PD2MFP_Pos)   /*!< GPD_MFP0 PD2 setting for LPUART0_RXD        */

/* PD.3 MFP */
#define SYS_GPD_MFP0_PD3MFP_GPIO              (0x0UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for GPIO               */
#define SYS_GPD_MFP0_PD3MFP_EBI_AD10          (0x2UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for EBI_AD10           */
#define SYS_GPD_MFP0_PD3MFP_USCI0_CTL1        (0x3UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for USCI0_CTL1         */
#define SYS_GPD_MFP0_PD3MFP_SPI0_SS           (0x4UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for SPI0_SS            */
#define SYS_GPD_MFP0_PD3MFP_UART3_nRTS        (0x5UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for UART3_nRTS         */
#define SYS_GPD_MFP0_PD3MFP_SC2_PWR           (0x7UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for SC2_PWR            */
#define SYS_GPD_MFP0_PD3MFP_SC1_nCD           (0x8UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for SC1_nCD            */
#define SYS_GPD_MFP0_PD3MFP_UART0_TXD         (0x9UL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for UART0_TXD          */
#define SYS_GPD_MFP0_PD3MFP_I2S1_BCLK         (0xAUL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for I2S1_BCLK          */
#define SYS_GPD_MFP0_PD3MFP_EQEI3_A           (0xDUL<<SYS_GPD_MFP0_PD3MFP_Pos)    /*!< GPD_MFP0 PD3 setting for EQEI3_A            */
#define SYS_GPD_MFP0_PD3MFP_LPSPI0_SS         (0x14UL<<SYS_GPD_MFP0_PD3MFP_Pos)   /*!< GPD_MFP0 PD3 setting for LPSPI0_SS          */
#define SYS_GPD_MFP0_PD3MFP_LPUART0_TXD       (0x15UL<<SYS_GPD_MFP0_PD3MFP_Pos)   /*!< GPD_MFP0 PD3 setting for LPUART0_TXD        */

/* PD.4 MFP */
#define SYS_GPD_MFP1_PD4MFP_GPIO              (0x0UL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for GPIO               */
#define SYS_GPD_MFP1_PD4MFP_USCI0_CTL0        (0x3UL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for USCI0_CTL0         */
#define SYS_GPD_MFP1_PD4MFP_I2C1_SDA          (0x4UL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for I2C1_SDA           */
#define SYS_GPD_MFP1_PD4MFP_SPI1_SS           (0x5UL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for SPI1_SS            */
#define SYS_GPD_MFP1_PD4MFP_SC1_CLK           (0x8UL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for SC1_CLK            */
#define SYS_GPD_MFP1_PD4MFP_USB_VBUS_ST       (0xEUL<<SYS_GPD_MFP1_PD4MFP_Pos)    /*!< GPD_MFP1 PD4 setting for USB_VBUS_ST        */
#define SYS_GPD_MFP1_PD4MFP_PSIO0_CH7         (0x11UL<<SYS_GPD_MFP1_PD4MFP_Pos)   /*!< GPD_MFP1 PD4 setting for PSIO0_CH7          */

/* PD.5 MFP */
#define SYS_GPD_MFP1_PD5MFP_GPIO              (0x0UL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for GPIO               */
#define SYS_GPD_MFP1_PD5MFP_EBI_AD15          (0x2UL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for EBI_AD15           */
#define SYS_GPD_MFP1_PD5MFP_SD1_nCD           (0x3UL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for SD1_nCD            */
#define SYS_GPD_MFP1_PD5MFP_EBI_nCS0          (0x4UL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for EBI_nCS0           */
#define SYS_GPD_MFP1_PD5MFP_CLKO              (0xEUL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for CLKO               */
#define SYS_GPD_MFP1_PD5MFP_EADC0_ST          (0xFUL<<SYS_GPD_MFP1_PD5MFP_Pos)    /*!< GPD_MFP1 PD5 setting for EADC0_ST           */
#define SYS_GPD_MFP1_PD5MFP_LPADC0_ST         (0x17UL<<SYS_GPD_MFP1_PD5MFP_Pos)   /*!< GPD_MFP1 PD5 setting for LPADC0_ST          */

/* PD.6 MFP */
#define SYS_GPD_MFP1_PD6MFP_GPIO              (0x0UL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for GPIO               */
#define SYS_GPD_MFP1_PD6MFP_EBI_AD5           (0x2UL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for EBI_AD5            */
#define SYS_GPD_MFP1_PD6MFP_SD1_CLK           (0x3UL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for SD1_CLK            */
#define SYS_GPD_MFP1_PD6MFP_QSPI1_MOSI0       (0x5UL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for QSPI1_MOSI0        */
#define SYS_GPD_MFP1_PD6MFP_UART6_RXD         (0x6UL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for UART6_RXD          */
#define SYS_GPD_MFP1_PD6MFP_BPWM0_CH0         (0xCUL<<SYS_GPD_MFP1_PD6MFP_Pos)    /*!< GPD_MFP1 PD6 setting for BPWM0_CH0          */

/* PD.7 MFP */
#define SYS_GPD_MFP1_PD7MFP_GPIO              (0x0UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for GPIO               */
#define SYS_GPD_MFP1_PD7MFP_EBI_AD4           (0x2UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for EBI_AD4            */
#define SYS_GPD_MFP1_PD7MFP_SD1_CMD           (0x3UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for SD1_CMD            */
#define SYS_GPD_MFP1_PD7MFP_QSPI1_MISO0       (0x5UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for QSPI1_MISO0        */
#define SYS_GPD_MFP1_PD7MFP_UART6_TXD         (0x6UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for UART6_TXD          */
#define SYS_GPD_MFP1_PD7MFP_CCAP_HSYNC        (0x7UL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for CCAP_HSYNC         */
#define SYS_GPD_MFP1_PD7MFP_BPWM0_CH1         (0xCUL<<SYS_GPD_MFP1_PD7MFP_Pos)    /*!< GPD_MFP1 PD7 setting for BPWM0_CH1          */

/* PD.8 MFP */
#define SYS_GPD_MFP2_PD8MFP_GPIO              (0x0UL<<SYS_GPD_MFP2_PD8MFP_Pos)    /*!< GPD_MFP2 PD8 setting for GPIO               */
#define SYS_GPD_MFP2_PD8MFP_EBI_AD6           (0x2UL<<SYS_GPD_MFP2_PD8MFP_Pos)    /*!< GPD_MFP2 PD8 setting for EBI_AD6            */
#define SYS_GPD_MFP2_PD8MFP_I2C2_SDA          (0x3UL<<SYS_GPD_MFP2_PD8MFP_Pos)    /*!< GPD_MFP2 PD8 setting for I2C2_SDA           */
#define SYS_GPD_MFP2_PD8MFP_UART2_nRTS        (0x4UL<<SYS_GPD_MFP2_PD8MFP_Pos)    /*!< GPD_MFP2 PD8 setting for UART2_nRTS         */
#define SYS_GPD_MFP2_PD8MFP_UART7_RXD         (0x5UL<<SYS_GPD_MFP2_PD8MFP_Pos)    /*!< GPD_MFP2 PD8 setting for UART7_RXD          */
#define SYS_GPD_MFP2_PD8MFP_PSIO0_CH3         (0x11UL<<SYS_GPD_MFP2_PD8MFP_Pos)   /*!< GPD_MFP2 PD8 setting for PSIO0_CH3          */

/* PD.9 MFP */
#define SYS_GPD_MFP2_PD9MFP_GPIO              (0x0UL<<SYS_GPD_MFP2_PD9MFP_Pos)    /*!< GPD_MFP2 PD9 setting for GPIO               */
#define SYS_GPD_MFP2_PD9MFP_EBI_AD7           (0x2UL<<SYS_GPD_MFP2_PD9MFP_Pos)    /*!< GPD_MFP2 PD9 setting for EBI_AD7            */
#define SYS_GPD_MFP2_PD9MFP_I2C2_SCL          (0x3UL<<SYS_GPD_MFP2_PD9MFP_Pos)    /*!< GPD_MFP2 PD9 setting for I2C2_SCL           */
#define SYS_GPD_MFP2_PD9MFP_UART2_nCTS        (0x4UL<<SYS_GPD_MFP2_PD9MFP_Pos)    /*!< GPD_MFP2 PD9 setting for UART2_nCTS         */
#define SYS_GPD_MFP2_PD9MFP_UART7_TXD         (0x5UL<<SYS_GPD_MFP2_PD9MFP_Pos)    /*!< GPD_MFP2 PD9 setting for UART7_TXD          */
#define SYS_GPD_MFP2_PD9MFP_PSIO0_CH2         (0x11UL<<SYS_GPD_MFP2_PD9MFP_Pos)   /*!< GPD_MFP2 PD9 setting for PSIO0_CH2          */

/* PD.10 MFP */
#define SYS_GPD_MFP2_PD10MFP_GPIO             (0x0UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for GPIO              */
#define SYS_GPD_MFP2_PD10MFP_EADC0_CH16       (0x1UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for EADC0_CH16        */
#define SYS_GPD_MFP2_PD10MFP_LPADC0_CH16      (0x1UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for LPADC0_CH16       */
#define SYS_GPD_MFP2_PD10MFP_EBI_nCS2         (0x2UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for EBI_nCS2          */
#define SYS_GPD_MFP2_PD10MFP_UART1_RXD        (0x3UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for UART1_RXD         */
#define SYS_GPD_MFP2_PD10MFP_CANFD0_RXD       (0x4UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for CANFD0_RXD        */
#define SYS_GPD_MFP2_PD10MFP_UART8_RXD        (0x8UL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for UART8_RXD         */
#define SYS_GPD_MFP2_PD10MFP_EQEI0_B          (0xAUL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for EQEI0_B           */
#define SYS_GPD_MFP2_PD10MFP_ECAP3_IC2        (0xBUL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for ECAP3_IC2         */
#define SYS_GPD_MFP2_PD10MFP_INT7             (0xFUL<<SYS_GPD_MFP2_PD10MFP_Pos)   /*!< GPD_MFP2 PD10 setting for INT7              */

/* PD.11 MFP */
#define SYS_GPD_MFP2_PD11MFP_GPIO             (0x0UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for GPIO              */
#define SYS_GPD_MFP2_PD11MFP_EADC0_CH17       (0x1UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for EADC0_CH17        */
#define SYS_GPD_MFP2_PD11MFP_LPADC0_CH17      (0x1UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for LPADC0_CH17       */
#define SYS_GPD_MFP2_PD11MFP_EBI_nCS1         (0x2UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for EBI_nCS1          */
#define SYS_GPD_MFP2_PD11MFP_UART1_TXD        (0x3UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for UART1_TXD         */
#define SYS_GPD_MFP2_PD11MFP_CANFD0_TXD       (0x4UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for CANFD0_TXD        */
#define SYS_GPD_MFP2_PD11MFP_UART8_TXD        (0x8UL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for UART8_TXD         */
#define SYS_GPD_MFP2_PD11MFP_EQEI0_A          (0xAUL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for EQEI0_A           */
#define SYS_GPD_MFP2_PD11MFP_ECAP3_IC1        (0xBUL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for ECAP3_IC1         */
#define SYS_GPD_MFP2_PD11MFP_INT6             (0xFUL<<SYS_GPD_MFP2_PD11MFP_Pos)   /*!< GPD_MFP2 PD11 setting for INT6              */

/* PD.12 MFP */
#define SYS_GPD_MFP3_PD12MFP_GPIO             (0x0UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for GPIO              */
#define SYS_GPD_MFP3_PD12MFP_EADC0_CH18       (0x1UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for EADC0_CH18        */
#define SYS_GPD_MFP3_PD12MFP_LPADC0_CH18      (0x1UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for LPADC0_CH18       */
#define SYS_GPD_MFP3_PD12MFP_EBI_nCS0         (0x2UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for EBI_nCS0          */
#define SYS_GPD_MFP3_PD12MFP_CANFD1_RXD       (0x5UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for CANFD1_RXD        */
#define SYS_GPD_MFP3_PD12MFP_UART2_RXD        (0x7UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for UART2_RXD         */
#define SYS_GPD_MFP3_PD12MFP_UART8_nRTS       (0x8UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for UART8_nRTS        */
#define SYS_GPD_MFP3_PD12MFP_BPWM0_CH5        (0x9UL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for BPWM0_CH5         */
#define SYS_GPD_MFP3_PD12MFP_EQEI0_INDEX      (0xAUL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for EQEI0_INDEX       */
#define SYS_GPD_MFP3_PD12MFP_ECAP3_IC0        (0xBUL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for ECAP3_IC0         */
#define SYS_GPD_MFP3_PD12MFP_CLKO             (0xDUL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for CLKO              */
#define SYS_GPD_MFP3_PD12MFP_EADC0_ST         (0xEUL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for EADC0_ST          */
#define SYS_GPD_MFP3_PD12MFP_INT5             (0xFUL<<SYS_GPD_MFP3_PD12MFP_Pos)   /*!< GPD_MFP3 PD12 setting for INT5              */
#define SYS_GPD_MFP3_PD12MFP_LPADC0_ST        (0x17UL<<SYS_GPD_MFP3_PD12MFP_Pos)  /*!< GPD_MFP3 PD12 setting for LPADC0_ST         */

/* PD.13 MFP */
#define SYS_GPD_MFP3_PD13MFP_GPIO             (0x0UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for GPIO              */
#define SYS_GPD_MFP3_PD13MFP_EBI_AD10         (0x2UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for EBI_AD10          */
#define SYS_GPD_MFP3_PD13MFP_SD0_nCD          (0x3UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for SD0_nCD           */
#define SYS_GPD_MFP3_PD13MFP_SPI0_I2SMCLK     (0x4UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for SPI0_I2SMCLK      */
#define SYS_GPD_MFP3_PD13MFP_SPI1_I2SMCLK     (0x5UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for SPI1_I2SMCLK      */
#define SYS_GPD_MFP3_PD13MFP_QSPI1_MOSI0      (0x6UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for QSPI1_MOSI0       */
#define SYS_GPD_MFP3_PD13MFP_SC2_nCD          (0x7UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for SC2_nCD           */
#define SYS_GPD_MFP3_PD13MFP_SD1_CLK          (0x8UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for SD1_CLK           */
#define SYS_GPD_MFP3_PD13MFP_UART6_RXD        (0x9UL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for UART6_RXD         */
#define SYS_GPD_MFP3_PD13MFP_I2S1_LRCK        (0xAUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for I2S1_LRCK         */
#define SYS_GPD_MFP3_PD13MFP_BPWM0_CH0        (0xBUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for BPWM0_CH0         */
#define SYS_GPD_MFP3_PD13MFP_EQEI2_B          (0xCUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for EQEI2_B           */
#define SYS_GPD_MFP3_PD13MFP_ECAP2_IC2        (0xDUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for ECAP2_IC2         */
#define SYS_GPD_MFP3_PD13MFP_CLKO             (0xEUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for CLKO              */
#define SYS_GPD_MFP3_PD13MFP_EADC0_ST         (0xFUL<<SYS_GPD_MFP3_PD13MFP_Pos)   /*!< GPD_MFP3 PD13 setting for EADC0_ST          */
#define SYS_GPD_MFP3_PD13MFP_LPADC0_ST        (0x17UL<<SYS_GPD_MFP3_PD13MFP_Pos)  /*!< GPD_MFP3 PD13 setting for LPADC0_ST         */

/* PD.14 MFP */
#define SYS_GPD_MFP3_PD14MFP_GPIO             (0x0UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for GPIO              */
#define SYS_GPD_MFP3_PD14MFP_EBI_nCS0         (0x2UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for EBI_nCS0          */
#define SYS_GPD_MFP3_PD14MFP_SPI3_I2SMCLK     (0x3UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for SPI3_I2SMCLK      */
#define SYS_GPD_MFP3_PD14MFP_SC1_nCD          (0x4UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for SC1_nCD           */
#define SYS_GPD_MFP3_PD14MFP_SPI0_I2SMCLK     (0x5UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for SPI0_I2SMCLK      */
#define SYS_GPD_MFP3_PD14MFP_QSPI0_MOSI1      (0x6UL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for QSPI0_MOSI1       */
#define SYS_GPD_MFP3_PD14MFP_I2S1_BCLK        (0xAUL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for I2S1_BCLK         */
#define SYS_GPD_MFP3_PD14MFP_EPWM0_CH4        (0xBUL<<SYS_GPD_MFP3_PD14MFP_Pos)   /*!< GPD_MFP3 PD14 setting for EPWM0_CH4         */

/* PE.0 MFP */
#define SYS_GPE_MFP0_PE0MFP_GPIO              (0x0UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for GPIO               */
#define SYS_GPE_MFP0_PE0MFP_EBI_AD11          (0x2UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for EBI_AD11           */
#define SYS_GPE_MFP0_PE0MFP_QSPI0_MOSI0       (0x3UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for QSPI0_MOSI0        */
#define SYS_GPE_MFP0_PE0MFP_SC2_CLK           (0x4UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for SC2_CLK            */
#define SYS_GPE_MFP0_PE0MFP_I2S0_MCLK         (0x5UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for I2S0_MCLK          */
#define SYS_GPE_MFP0_PE0MFP_SPI1_MOSI         (0x6UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for SPI1_MOSI          */
#define SYS_GPE_MFP0_PE0MFP_UART3_RXD         (0x7UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for UART3_RXD          */
#define SYS_GPE_MFP0_PE0MFP_I2C1_SDA          (0x8UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for I2C1_SDA           */
#define SYS_GPE_MFP0_PE0MFP_UART4_nRTS        (0x9UL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for UART4_nRTS         */
#define SYS_GPE_MFP0_PE0MFP_UART8_RXD         (0xAUL<<SYS_GPE_MFP0_PE0MFP_Pos)    /*!< GPE_MFP0 PE0 setting for UART8_RXD          */
#define SYS_GPE_MFP0_PE0MFP_LPIO0             (0x17UL<<SYS_GPE_MFP0_PE0MFP_Pos)   /*!< GPE_MFP0 PE0 setting for LPIO0              */

/* PE.1 MFP */
#define SYS_GPE_MFP0_PE1MFP_GPIO              (0x0UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for GPIO               */
#define SYS_GPE_MFP0_PE1MFP_EBI_AD10          (0x2UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for EBI_AD10           */
#define SYS_GPE_MFP0_PE1MFP_QSPI0_MISO0       (0x3UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for QSPI0_MISO0        */
#define SYS_GPE_MFP0_PE1MFP_SC2_DAT           (0x4UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for SC2_DAT            */
#define SYS_GPE_MFP0_PE1MFP_I2S0_BCLK         (0x5UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for I2S0_BCLK          */
#define SYS_GPE_MFP0_PE1MFP_SPI1_MISO         (0x6UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for SPI1_MISO          */
#define SYS_GPE_MFP0_PE1MFP_UART3_TXD         (0x7UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for UART3_TXD          */
#define SYS_GPE_MFP0_PE1MFP_I2C1_SCL          (0x8UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for I2C1_SCL           */
#define SYS_GPE_MFP0_PE1MFP_UART4_nCTS        (0x9UL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for UART4_nCTS         */
#define SYS_GPE_MFP0_PE1MFP_UART8_TXD         (0xAUL<<SYS_GPE_MFP0_PE1MFP_Pos)    /*!< GPE_MFP0 PE1 setting for UART8_TXD          */
#define SYS_GPE_MFP0_PE1MFP_LPIO1             (0x17UL<<SYS_GPE_MFP0_PE1MFP_Pos)   /*!< GPE_MFP0 PE1 setting for LPIO1              */

/* PE.2 MFP */
#define SYS_GPE_MFP0_PE2MFP_GPIO              (0x0UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for GPIO               */
#define SYS_GPE_MFP0_PE2MFP_EBI_ALE           (0x2UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for EBI_ALE            */
#define SYS_GPE_MFP0_PE2MFP_SD0_DAT0          (0x3UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for SD0_DAT0           */
#define SYS_GPE_MFP0_PE2MFP_SPI3_MOSI         (0x5UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for SPI3_MOSI          */
#define SYS_GPE_MFP0_PE2MFP_SC0_CLK           (0x6UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for SC0_CLK            */
#define SYS_GPE_MFP0_PE2MFP_USCI0_CLK         (0x7UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for USCI0_CLK          */
#define SYS_GPE_MFP0_PE2MFP_UART6_nCTS        (0x8UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for UART6_nCTS         */
#define SYS_GPE_MFP0_PE2MFP_UART7_RXD         (0x9UL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for UART7_RXD          */
#define SYS_GPE_MFP0_PE2MFP_UART8_nRTS        (0xAUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for UART8_nRTS         */
#define SYS_GPE_MFP0_PE2MFP_EQEI0_B           (0xBUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for EQEI0_B            */
#define SYS_GPE_MFP0_PE2MFP_EPWM0_CH5         (0xCUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for EPWM0_CH5          */
#define SYS_GPE_MFP0_PE2MFP_BPWM0_CH0         (0xDUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for BPWM0_CH0          */
#define SYS_GPE_MFP0_PE2MFP_I2S0_MCLK         (0xEUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for I2S0_MCLK          */
#define SYS_GPE_MFP0_PE2MFP_SC2_CLK           (0xFUL<<SYS_GPE_MFP0_PE2MFP_Pos)    /*!< GPE_MFP0 PE2 setting for SC2_CLK            */

/* PE.3 MFP */
#define SYS_GPE_MFP0_PE3MFP_GPIO              (0x0UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for GPIO               */
#define SYS_GPE_MFP0_PE3MFP_EBI_MCLK          (0x2UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for EBI_MCLK           */
#define SYS_GPE_MFP0_PE3MFP_SD0_DAT1          (0x3UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for SD0_DAT1           */
#define SYS_GPE_MFP0_PE3MFP_SPI3_MISO         (0x5UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for SPI3_MISO          */
#define SYS_GPE_MFP0_PE3MFP_SC0_DAT           (0x6UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for SC0_DAT            */
#define SYS_GPE_MFP0_PE3MFP_USCI0_DAT0        (0x7UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for USCI0_DAT0         */
#define SYS_GPE_MFP0_PE3MFP_UART6_nRTS        (0x8UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for UART6_nRTS         */
#define SYS_GPE_MFP0_PE3MFP_UART7_TXD         (0x9UL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for UART7_TXD          */
#define SYS_GPE_MFP0_PE3MFP_UART8_nCTS        (0xAUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for UART8_nCTS         */
#define SYS_GPE_MFP0_PE3MFP_EQEI0_A           (0xBUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for EQEI0_A            */
#define SYS_GPE_MFP0_PE3MFP_EPWM0_CH4         (0xCUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for EPWM0_CH4          */
#define SYS_GPE_MFP0_PE3MFP_BPWM0_CH1         (0xDUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for BPWM0_CH1          */
#define SYS_GPE_MFP0_PE3MFP_I2S0_BCLK         (0xEUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for I2S0_BCLK          */
#define SYS_GPE_MFP0_PE3MFP_SC2_DAT           (0xFUL<<SYS_GPE_MFP0_PE3MFP_Pos)    /*!< GPE_MFP0 PE3 setting for SC2_DAT            */

/* PE.4 MFP */
#define SYS_GPE_MFP1_PE4MFP_GPIO              (0x0UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for GPIO               */
#define SYS_GPE_MFP1_PE4MFP_EBI_nWR           (0x2UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for EBI_nWR            */
#define SYS_GPE_MFP1_PE4MFP_SD0_DAT2          (0x3UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for SD0_DAT2           */
#define SYS_GPE_MFP1_PE4MFP_SPI3_CLK          (0x5UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for SPI3_CLK           */
#define SYS_GPE_MFP1_PE4MFP_SC0_RST           (0x6UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for SC0_RST            */
#define SYS_GPE_MFP1_PE4MFP_USCI0_DAT1        (0x7UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for USCI0_DAT1         */
#define SYS_GPE_MFP1_PE4MFP_UART6_RXD         (0x8UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for UART6_RXD          */
#define SYS_GPE_MFP1_PE4MFP_UART7_nCTS        (0x9UL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for UART7_nCTS         */
#define SYS_GPE_MFP1_PE4MFP_UART9_RXD         (0xAUL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for UART9_RXD          */
#define SYS_GPE_MFP1_PE4MFP_EQEI0_INDEX       (0xBUL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for EQEI0_INDEX        */
#define SYS_GPE_MFP1_PE4MFP_EPWM0_CH3         (0xCUL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for EPWM0_CH3          */
#define SYS_GPE_MFP1_PE4MFP_BPWM0_CH2         (0xDUL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for BPWM0_CH2          */
#define SYS_GPE_MFP1_PE4MFP_SPI1_MOSI         (0xFUL<<SYS_GPE_MFP1_PE4MFP_Pos)    /*!< GPE_MFP1 PE4 setting for SPI1_MOSI          */
#define SYS_GPE_MFP1_PE4MFP_PSIO0_CH3         (0x11UL<<SYS_GPE_MFP1_PE4MFP_Pos)   /*!< GPE_MFP1 PE4 setting for PSIO0_CH3          */

/* PE.5 MFP */
#define SYS_GPE_MFP1_PE5MFP_GPIO              (0x0UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for GPIO               */
#define SYS_GPE_MFP1_PE5MFP_EBI_nRD           (0x2UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for EBI_nRD            */
#define SYS_GPE_MFP1_PE5MFP_SD0_DAT3          (0x3UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for SD0_DAT3           */
#define SYS_GPE_MFP1_PE5MFP_SPI3_SS           (0x5UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for SPI3_SS            */
#define SYS_GPE_MFP1_PE5MFP_SC0_PWR           (0x6UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for SC0_PWR            */
#define SYS_GPE_MFP1_PE5MFP_USCI0_CTL1        (0x7UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for USCI0_CTL1         */
#define SYS_GPE_MFP1_PE5MFP_UART6_TXD         (0x8UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for UART6_TXD          */
#define SYS_GPE_MFP1_PE5MFP_UART7_nRTS        (0x9UL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for UART7_nRTS         */
#define SYS_GPE_MFP1_PE5MFP_UART9_TXD         (0xAUL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for UART9_TXD          */
#define SYS_GPE_MFP1_PE5MFP_EQEI1_B           (0xBUL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for EQEI1_B            */
#define SYS_GPE_MFP1_PE5MFP_EPWM0_CH2         (0xCUL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for EPWM0_CH2          */
#define SYS_GPE_MFP1_PE5MFP_BPWM0_CH3         (0xDUL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for BPWM0_CH3          */
#define SYS_GPE_MFP1_PE5MFP_SPI1_MISO         (0xFUL<<SYS_GPE_MFP1_PE5MFP_Pos)    /*!< GPE_MFP1 PE5 setting for SPI1_MISO          */
#define SYS_GPE_MFP1_PE5MFP_PSIO0_CH2         (0x11UL<<SYS_GPE_MFP1_PE5MFP_Pos)   /*!< GPE_MFP1 PE5 setting for PSIO0_CH2          */

/* PE.6 MFP */
#define SYS_GPE_MFP1_PE6MFP_GPIO              (0x0UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for GPIO               */
#define SYS_GPE_MFP1_PE6MFP_EBI_AD11          (0x2UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for EBI_AD11           */
#define SYS_GPE_MFP1_PE6MFP_SD0_CLK           (0x3UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for SD0_CLK            */
#define SYS_GPE_MFP1_PE6MFP_SPI3_I2SMCLK      (0x5UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for SPI3_I2SMCLK       */
#define SYS_GPE_MFP1_PE6MFP_SC0_nCD           (0x6UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for SC0_nCD            */
#define SYS_GPE_MFP1_PE6MFP_USCI0_CTL0        (0x7UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for USCI0_CTL0         */
#define SYS_GPE_MFP1_PE6MFP_UART5_RXD         (0x8UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for UART5_RXD          */
#define SYS_GPE_MFP1_PE6MFP_CANFD1_RXD        (0x9UL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for CANFD1_RXD         */
#define SYS_GPE_MFP1_PE6MFP_UART9_nRTS        (0xAUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for UART9_nRTS         */
#define SYS_GPE_MFP1_PE6MFP_EQEI1_A           (0xBUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for EQEI1_A            */
#define SYS_GPE_MFP1_PE6MFP_EPWM0_CH1         (0xCUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for EPWM0_CH1          */
#define SYS_GPE_MFP1_PE6MFP_BPWM0_CH4         (0xDUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for BPWM0_CH4          */
#define SYS_GPE_MFP1_PE6MFP_ACMP3_O           (0xEUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for ACMP3_O            */
#define SYS_GPE_MFP1_PE6MFP_QSPI0_MOSI0       (0xFUL<<SYS_GPE_MFP1_PE6MFP_Pos)    /*!< GPE_MFP1 PE6 setting for QSPI0_MOSI0        */
#define SYS_GPE_MFP1_PE6MFP_PSIO0_CH1         (0x11UL<<SYS_GPE_MFP1_PE6MFP_Pos)   /*!< GPE_MFP1 PE6 setting for PSIO0_CH1          */

/* PE.7 MFP */
#define SYS_GPE_MFP1_PE7MFP_GPIO              (0x0UL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for GPIO               */
#define SYS_GPE_MFP1_PE7MFP_EBI_AD10          (0x2UL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for EBI_AD10           */
#define SYS_GPE_MFP1_PE7MFP_SD0_CMD           (0x3UL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for SD0_CMD            */
#define SYS_GPE_MFP1_PE7MFP_UART5_TXD         (0x8UL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for UART5_TXD          */
#define SYS_GPE_MFP1_PE7MFP_CANFD1_TXD        (0x9UL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for CANFD1_TXD         */
#define SYS_GPE_MFP1_PE7MFP_UART9_nCTS        (0xAUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for UART9_nCTS         */
#define SYS_GPE_MFP1_PE7MFP_EQEI1_INDEX       (0xBUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for EQEI1_INDEX        */
#define SYS_GPE_MFP1_PE7MFP_EPWM0_CH0         (0xCUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for EPWM0_CH0          */
#define SYS_GPE_MFP1_PE7MFP_BPWM0_CH5         (0xDUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for BPWM0_CH5          */
#define SYS_GPE_MFP1_PE7MFP_ACMP2_O           (0xEUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for ACMP2_O            */
#define SYS_GPE_MFP1_PE7MFP_QSPI0_MISO0       (0xFUL<<SYS_GPE_MFP1_PE7MFP_Pos)    /*!< GPE_MFP1 PE7 setting for QSPI0_MISO0        */
#define SYS_GPE_MFP1_PE7MFP_PSIO0_CH0         (0x11UL<<SYS_GPE_MFP1_PE7MFP_Pos)   /*!< GPE_MFP1 PE7 setting for PSIO0_CH0          */

/* PE.8 MFP */
#define SYS_GPE_MFP2_PE8MFP_GPIO              (0x0UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for GPIO               */
#define SYS_GPE_MFP2_PE8MFP_EBI_ADR10         (0x2UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for EBI_ADR10          */
#define SYS_GPE_MFP2_PE8MFP_EMAC0_RMII_MDC    (0x3UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for EMAC0_RMII_MDC     */
#define SYS_GPE_MFP2_PE8MFP_I2S0_BCLK         (0x4UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for I2S0_BCLK          */
#define SYS_GPE_MFP2_PE8MFP_SPI2_CLK          (0x5UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for SPI2_CLK           */
#define SYS_GPE_MFP2_PE8MFP_UART2_TXD         (0x7UL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for UART2_TXD          */
#define SYS_GPE_MFP2_PE8MFP_EPWM0_CH0         (0xAUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for EPWM0_CH0          */
#define SYS_GPE_MFP2_PE8MFP_EPWM0_BRAKE0      (0xBUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for EPWM0_BRAKE0       */
#define SYS_GPE_MFP2_PE8MFP_ECAP0_IC0         (0xCUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for ECAP0_IC0          */
#define SYS_GPE_MFP2_PE8MFP_EQEI2_INDEX       (0xDUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for EQEI2_INDEX        */
#define SYS_GPE_MFP2_PE8MFP_TRACE_DATA3       (0xEUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for TRACE_DATA3        */
#define SYS_GPE_MFP2_PE8MFP_ECAP3_IC0         (0xFUL<<SYS_GPE_MFP2_PE8MFP_Pos)    /*!< GPE_MFP2 PE8 setting for ECAP3_IC0          */
#define SYS_GPE_MFP2_PE8MFP_DMIC0_DAT         (0x10UL<<SYS_GPE_MFP2_PE8MFP_Pos)   /*!< GPE_MFP2 PE8 setting for DMIC0_DAT          */

/* PE.9 MFP */
#define SYS_GPE_MFP2_PE9MFP_GPIO              (0x0UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for GPIO               */
#define SYS_GPE_MFP2_PE9MFP_EBI_ADR11         (0x2UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for EBI_ADR11          */
#define SYS_GPE_MFP2_PE9MFP_EMAC0_RMII_MDIO   (0x3UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for EMAC0_RMII_MDIO    */
#define SYS_GPE_MFP2_PE9MFP_I2S0_MCLK         (0x4UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for I2S0_MCLK          */
#define SYS_GPE_MFP2_PE9MFP_SPI2_MISO         (0x5UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for SPI2_MISO          */
#define SYS_GPE_MFP2_PE9MFP_UART2_RXD         (0x7UL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for UART2_RXD          */
#define SYS_GPE_MFP2_PE9MFP_EPWM0_CH1         (0xAUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for EPWM0_CH1          */
#define SYS_GPE_MFP2_PE9MFP_EPWM0_BRAKE1      (0xBUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for EPWM0_BRAKE1       */
#define SYS_GPE_MFP2_PE9MFP_ECAP0_IC1         (0xCUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for ECAP0_IC1          */
#define SYS_GPE_MFP2_PE9MFP_EQEI2_A           (0xDUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for EQEI2_A            */
#define SYS_GPE_MFP2_PE9MFP_TRACE_DATA2       (0xEUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for TRACE_DATA2        */
#define SYS_GPE_MFP2_PE9MFP_ECAP3_IC1         (0xFUL<<SYS_GPE_MFP2_PE9MFP_Pos)    /*!< GPE_MFP2 PE9 setting for ECAP3_IC1          */
#define SYS_GPE_MFP2_PE9MFP_DMIC0_CLK         (0x10UL<<SYS_GPE_MFP2_PE9MFP_Pos)   /*!< GPE_MFP2 PE9 setting for DMIC0_CLK          */

/* PE.10 MFP */
#define SYS_GPE_MFP2_PE10MFP_GPIO             (0x0UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for GPIO              */
#define SYS_GPE_MFP2_PE10MFP_EBI_ADR12        (0x2UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for EBI_ADR12         */
#define SYS_GPE_MFP2_PE10MFP_EMAC0_RMII_TXD0  (0x3UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for EMAC0_RMII_TXD0   */
#define SYS_GPE_MFP2_PE10MFP_I2S0_DI          (0x4UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for I2S0_DI           */
#define SYS_GPE_MFP2_PE10MFP_SPI2_MOSI        (0x5UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for SPI2_MOSI         */
#define SYS_GPE_MFP2_PE10MFP_UART3_TXD        (0x7UL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for UART3_TXD         */
#define SYS_GPE_MFP2_PE10MFP_EPWM0_CH2        (0xAUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for EPWM0_CH2         */
#define SYS_GPE_MFP2_PE10MFP_EPWM1_BRAKE0     (0xBUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for EPWM1_BRAKE0      */
#define SYS_GPE_MFP2_PE10MFP_ECAP0_IC2        (0xCUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for ECAP0_IC2         */
#define SYS_GPE_MFP2_PE10MFP_EQEI2_B          (0xDUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for EQEI2_B           */
#define SYS_GPE_MFP2_PE10MFP_TRACE_DATA1      (0xEUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for TRACE_DATA1       */
#define SYS_GPE_MFP2_PE10MFP_ECAP3_IC2        (0xFUL<<SYS_GPE_MFP2_PE10MFP_Pos)   /*!< GPE_MFP2 PE10 setting for ECAP3_IC2         */
#define SYS_GPE_MFP2_PE10MFP_DMIC0_CLKLP      (0x10UL<<SYS_GPE_MFP2_PE10MFP_Pos)  /*!< GPE_MFP2 PE10 setting for DMIC0_CLKLP       */

/* PE.11 MFP */
#define SYS_GPE_MFP2_PE11MFP_GPIO             (0x0UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for GPIO              */
#define SYS_GPE_MFP2_PE11MFP_EBI_ADR13        (0x2UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for EBI_ADR13         */
#define SYS_GPE_MFP2_PE11MFP_EMAC0_RMII_TXD1  (0x3UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for EMAC0_RMII_TXD1   */
#define SYS_GPE_MFP2_PE11MFP_I2S0_DO          (0x4UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for I2S0_DO           */
#define SYS_GPE_MFP2_PE11MFP_SPI2_SS          (0x5UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for SPI2_SS           */
#define SYS_GPE_MFP2_PE11MFP_UART3_RXD        (0x7UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for UART3_RXD         */
#define SYS_GPE_MFP2_PE11MFP_UART1_nCTS       (0x8UL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for UART1_nCTS        */
#define SYS_GPE_MFP2_PE11MFP_EPWM0_CH3        (0xAUL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for EPWM0_CH3         */
#define SYS_GPE_MFP2_PE11MFP_EPWM1_BRAKE1     (0xBUL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for EPWM1_BRAKE1      */
#define SYS_GPE_MFP2_PE11MFP_ECAP1_IC2        (0xDUL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for ECAP1_IC2         */
#define SYS_GPE_MFP2_PE11MFP_TRACE_DATA0      (0xEUL<<SYS_GPE_MFP2_PE11MFP_Pos)   /*!< GPE_MFP2 PE11 setting for TRACE_DATA0       */
#define SYS_GPE_MFP2_PE11MFP_DMIC1_DAT        (0x10UL<<SYS_GPE_MFP2_PE11MFP_Pos)  /*!< GPE_MFP2 PE11 setting for DMIC1_DAT         */
#define SYS_GPE_MFP2_PE11MFP_KPI_COL7         (0x12UL<<SYS_GPE_MFP2_PE11MFP_Pos)  /*!< GPE_MFP2 PE11 setting for KPI_COL7          */

/* PE.12 MFP */
#define SYS_GPE_MFP3_PE12MFP_GPIO             (0x0UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for GPIO              */
#define SYS_GPE_MFP3_PE12MFP_EBI_ADR14        (0x2UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for EBI_ADR14         */
#define SYS_GPE_MFP3_PE12MFP_EMAC0_RMII_TXEN  (0x3UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for EMAC0_RMII_TXEN   */
#define SYS_GPE_MFP3_PE12MFP_I2S0_LRCK        (0x4UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for I2S0_LRCK         */
#define SYS_GPE_MFP3_PE12MFP_SPI2_I2SMCLK     (0x5UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for SPI2_I2SMCLK      */
#define SYS_GPE_MFP3_PE12MFP_UART1_nRTS       (0x8UL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for UART1_nRTS        */
#define SYS_GPE_MFP3_PE12MFP_EPWM0_CH4        (0xAUL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for EPWM0_CH4         */
#define SYS_GPE_MFP3_PE12MFP_ECAP1_IC1        (0xDUL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for ECAP1_IC1         */
#define SYS_GPE_MFP3_PE12MFP_TRACE_CLK        (0xEUL<<SYS_GPE_MFP3_PE12MFP_Pos)   /*!< GPE_MFP3 PE12 setting for TRACE_CLK         */
#define SYS_GPE_MFP3_PE12MFP_DMIC1_CLK        (0x10UL<<SYS_GPE_MFP3_PE12MFP_Pos)  /*!< GPE_MFP3 PE12 setting for DMIC1_CLK         */
#define SYS_GPE_MFP3_PE12MFP_KPI_COL6         (0x12UL<<SYS_GPE_MFP3_PE12MFP_Pos)  /*!< GPE_MFP3 PE12 setting for KPI_COL6          */

/* PE.13 MFP */
#define SYS_GPE_MFP3_PE13MFP_GPIO             (0x0UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for GPIO              */
#define SYS_GPE_MFP3_PE13MFP_EBI_ADR15        (0x2UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for EBI_ADR15         */
#define SYS_GPE_MFP3_PE13MFP_EMAC0_PPS        (0x3UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for EMAC0_PPS         */
#define SYS_GPE_MFP3_PE13MFP_I2C0_SCL         (0x4UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for I2C0_SCL          */
#define SYS_GPE_MFP3_PE13MFP_UART4_nRTS       (0x5UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for UART4_nRTS        */
#define SYS_GPE_MFP3_PE13MFP_UART1_TXD        (0x8UL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for UART1_TXD         */
#define SYS_GPE_MFP3_PE13MFP_EPWM0_CH5        (0xAUL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for EPWM0_CH5         */
#define SYS_GPE_MFP3_PE13MFP_EPWM1_CH0        (0xBUL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for EPWM1_CH0         */
#define SYS_GPE_MFP3_PE13MFP_BPWM1_CH5        (0xCUL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for BPWM1_CH5         */
#define SYS_GPE_MFP3_PE13MFP_ECAP1_IC0        (0xDUL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for ECAP1_IC0         */
#define SYS_GPE_MFP3_PE13MFP_TRACE_SWO        (0xEUL<<SYS_GPE_MFP3_PE13MFP_Pos)   /*!< GPE_MFP3 PE13 setting for TRACE_SWO         */
#define SYS_GPE_MFP3_PE13MFP_KPI_COL5         (0x12UL<<SYS_GPE_MFP3_PE13MFP_Pos)  /*!< GPE_MFP3 PE13 setting for KPI_COL5          */
#define SYS_GPE_MFP3_PE13MFP_LPI2C0_SCL       (0x16UL<<SYS_GPE_MFP3_PE13MFP_Pos)  /*!< GPE_MFP3 PE13 setting for LPI2C0_SCL        */

/* PE.14 MFP */
#define SYS_GPE_MFP3_PE14MFP_GPIO             (0x0UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for GPIO              */
#define SYS_GPE_MFP3_PE14MFP_EBI_AD8          (0x2UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for EBI_AD8           */
#define SYS_GPE_MFP3_PE14MFP_UART2_TXD        (0x3UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for UART2_TXD         */
#define SYS_GPE_MFP3_PE14MFP_CANFD0_TXD       (0x4UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for CANFD0_TXD        */
#define SYS_GPE_MFP3_PE14MFP_SD1_nCD          (0x5UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for SD1_nCD           */
#define SYS_GPE_MFP3_PE14MFP_UART6_TXD        (0x6UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for UART6_TXD         */
#define SYS_GPE_MFP3_PE14MFP_UART3_TXD        (0x7UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for UART3_TXD         */
#define SYS_GPE_MFP3_PE14MFP_I2C1_SCL         (0x8UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for I2C1_SCL          */
#define SYS_GPE_MFP3_PE14MFP_UART4_nCTS       (0x9UL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for UART4_nCTS        */
#define SYS_GPE_MFP3_PE14MFP_UART8_TXD        (0xAUL<<SYS_GPE_MFP3_PE14MFP_Pos)   /*!< GPE_MFP3 PE14 setting for UART8_TXD         */
#define SYS_GPE_MFP3_PE14MFP_PSIO0_CH0        (0x11UL<<SYS_GPE_MFP3_PE14MFP_Pos)  /*!< GPE_MFP3 PE14 setting for PSIO0_CH0         */

/* PE.15 MFP */
#define SYS_GPE_MFP3_PE15MFP_GPIO             (0x0UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for GPIO              */
#define SYS_GPE_MFP3_PE15MFP_EBI_AD9          (0x2UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for EBI_AD9           */
#define SYS_GPE_MFP3_PE15MFP_UART2_RXD        (0x3UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for UART2_RXD         */
#define SYS_GPE_MFP3_PE15MFP_CANFD0_RXD       (0x4UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for CANFD0_RXD        */
#define SYS_GPE_MFP3_PE15MFP_UART6_RXD        (0x6UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for UART6_RXD         */
#define SYS_GPE_MFP3_PE15MFP_UART3_RXD        (0x7UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for UART3_RXD         */
#define SYS_GPE_MFP3_PE15MFP_I2C1_SDA         (0x8UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for I2C1_SDA          */
#define SYS_GPE_MFP3_PE15MFP_UART4_nRTS       (0x9UL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for UART4_nRTS        */
#define SYS_GPE_MFP3_PE15MFP_UART8_RXD        (0xAUL<<SYS_GPE_MFP3_PE15MFP_Pos)   /*!< GPE_MFP3 PE15 setting for UART8_RXD         */
#define SYS_GPE_MFP3_PE15MFP_PSIO0_CH1        (0x11UL<<SYS_GPE_MFP3_PE15MFP_Pos)  /*!< GPE_MFP3 PE15 setting for PSIO0_CH1         */

/* PF.0 MFP */
#define SYS_GPF_MFP0_PF0MFP_GPIO              (0x0UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for GPIO               */
#define SYS_GPF_MFP0_PF0MFP_UART1_TXD         (0x2UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for UART1_TXD          */
#define SYS_GPF_MFP0_PF0MFP_I2C1_SCL          (0x3UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for I2C1_SCL           */
#define SYS_GPF_MFP0_PF0MFP_UART0_TXD         (0x4UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for UART0_TXD          */
#define SYS_GPF_MFP0_PF0MFP_SC1_DAT           (0x5UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for SC1_DAT            */
#define SYS_GPF_MFP0_PF0MFP_I2S0_DO           (0x6UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for I2S0_DO            */
#define SYS_GPF_MFP0_PF0MFP_USCI0_CTL1        (0x7UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for USCI0_CTL1         */
#define SYS_GPF_MFP0_PF0MFP_UART2_TXD         (0x8UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for UART2_TXD          */
#define SYS_GPF_MFP0_PF0MFP_I2C0_SCL          (0x9UL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for I2C0_SCL           */
#define SYS_GPF_MFP0_PF0MFP_EPWM1_CH4         (0xBUL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for EPWM1_CH4          */
#define SYS_GPF_MFP0_PF0MFP_BPWM1_CH0         (0xCUL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for BPWM1_CH0          */
#define SYS_GPF_MFP0_PF0MFP_ACMP0_O           (0xDUL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for ACMP0_O            */
#define SYS_GPF_MFP0_PF0MFP_ICE_DAT           (0xEUL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for ICE_DAT            */
#define SYS_GPF_MFP0_PF0MFP_EADC0_ST          (0xFUL<<SYS_GPF_MFP0_PF0MFP_Pos)    /*!< GPF_MFP0 PF0 setting for EADC0_ST           */
#define SYS_GPF_MFP0_PF0MFP_I3C0_SCL          (0x10UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for I3C0_SCL           */
#define SYS_GPF_MFP0_PF0MFP_UTCPD0_FRSTX2     (0x12UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for UTCPD0_FRSTX2      */
#define SYS_GPF_MFP0_PF0MFP_UTCPD0_DISCHG     (0x13UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for UTCPD0_DISCHG      */
#define SYS_GPF_MFP0_PF0MFP_LPUART0_TXD       (0x15UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for LPUART0_TXD        */
#define SYS_GPF_MFP0_PF0MFP_LPADC0_ST         (0x16UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for LPADC0_ST          */
#define SYS_GPF_MFP0_PF0MFP_LPIO2             (0x17UL<<SYS_GPF_MFP0_PF0MFP_Pos)   /*!< GPF_MFP0 PF0 setting for LPIO2              */

/* PF.1 MFP */
#define SYS_GPF_MFP0_PF1MFP_GPIO              (0x0UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for GPIO               */
#define SYS_GPF_MFP0_PF1MFP_UART1_RXD         (0x2UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for UART1_RXD          */
#define SYS_GPF_MFP0_PF1MFP_I2C1_SDA          (0x3UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for I2C1_SDA           */
#define SYS_GPF_MFP0_PF1MFP_UART0_RXD         (0x4UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for UART0_RXD          */
#define SYS_GPF_MFP0_PF1MFP_SC1_CLK           (0x5UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for SC1_CLK            */
#define SYS_GPF_MFP0_PF1MFP_I2S0_LRCK         (0x6UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for I2S0_LRCK          */
#define SYS_GPF_MFP0_PF1MFP_USCI0_DAT1        (0x7UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for USCI0_DAT1         */
#define SYS_GPF_MFP0_PF1MFP_UART2_RXD         (0x8UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for UART2_RXD          */
#define SYS_GPF_MFP0_PF1MFP_I2C0_SDA          (0x9UL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for I2C0_SDA           */
#define SYS_GPF_MFP0_PF1MFP_EPWM1_CH5         (0xBUL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for EPWM1_CH5          */
#define SYS_GPF_MFP0_PF1MFP_BPWM1_CH1         (0xCUL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for BPWM1_CH1          */
#define SYS_GPF_MFP0_PF1MFP_ACMP1_O           (0xDUL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for ACMP1_O            */
#define SYS_GPF_MFP0_PF1MFP_ICE_CLK           (0xEUL<<SYS_GPF_MFP0_PF1MFP_Pos)    /*!< GPF_MFP0 PF1 setting for ICE_CLK            */
#define SYS_GPF_MFP0_PF1MFP_I3C0_SDA          (0x10UL<<SYS_GPF_MFP0_PF1MFP_Pos)   /*!< GPF_MFP0 PF1 setting for I3C0_SDA           */
#define SYS_GPF_MFP0_PF1MFP_UTCPD0_FRSTX1     (0x12UL<<SYS_GPF_MFP0_PF1MFP_Pos)   /*!< GPF_MFP0 PF1 setting for UTCPD0_FRSTX1      */
#define SYS_GPF_MFP0_PF1MFP_UTCPD0_DISCHG     (0x13UL<<SYS_GPF_MFP0_PF1MFP_Pos)   /*!< GPF_MFP0 PF1 setting for UTCPD0_DISCHG      */
#define SYS_GPF_MFP0_PF1MFP_LPUART0_RXD       (0x15UL<<SYS_GPF_MFP0_PF1MFP_Pos)   /*!< GPF_MFP0 PF1 setting for LPUART0_RXD        */
#define SYS_GPF_MFP0_PF1MFP_LPIO3             (0x17UL<<SYS_GPF_MFP0_PF1MFP_Pos)   /*!< GPF_MFP0 PF1 setting for LPIO3              */

/* PF.2 MFP */
#define SYS_GPF_MFP0_PF2MFP_GPIO              (0x0UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for GPIO               */
#define SYS_GPF_MFP0_PF2MFP_EBI_nCS1          (0x2UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for EBI_nCS1           */
#define SYS_GPF_MFP0_PF2MFP_UART0_RXD         (0x3UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for UART0_RXD          */
#define SYS_GPF_MFP0_PF2MFP_I2C0_SDA          (0x4UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for I2C0_SDA           */
#define SYS_GPF_MFP0_PF2MFP_QSPI0_CLK         (0x5UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for QSPI0_CLK          */
#define SYS_GPF_MFP0_PF2MFP_UART9_RXD         (0x7UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for UART9_RXD          */
#define SYS_GPF_MFP0_PF2MFP_I2C2_SMBSUS       (0x8UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for I2C2_SMBSUS        */
#define SYS_GPF_MFP0_PF2MFP_EPWM1_CH1         (0x9UL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for EPWM1_CH1          */
#define SYS_GPF_MFP0_PF2MFP_XT1_OUT           (0xAUL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for XT1_OUT            */
#define SYS_GPF_MFP0_PF2MFP_BPWM1_CH1         (0xBUL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for BPWM1_CH1          */
#define SYS_GPF_MFP0_PF2MFP_ACMP3_O           (0xDUL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for ACMP3_O            */
#define SYS_GPF_MFP0_PF2MFP_SC1_RST           (0xEUL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for SC1_RST            */
#define SYS_GPF_MFP0_PF2MFP_TM1_EXT           (0xFUL<<SYS_GPF_MFP0_PF2MFP_Pos)    /*!< GPF_MFP0 PF2 setting for TM1_EXT            */
#define SYS_GPF_MFP0_PF2MFP_LPUART0_RXD       (0x15UL<<SYS_GPF_MFP0_PF2MFP_Pos)   /*!< GPF_MFP0 PF2 setting for LPUART0_RXD        */
#define SYS_GPF_MFP0_PF2MFP_LPI2C0_SDA        (0x16UL<<SYS_GPF_MFP0_PF2MFP_Pos)   /*!< GPF_MFP0 PF2 setting for LPI2C0_SDA         */

/* PF.3 MFP */
#define SYS_GPF_MFP0_PF3MFP_GPIO              (0x0UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for GPIO               */
#define SYS_GPF_MFP0_PF3MFP_EBI_nCS0          (0x2UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for EBI_nCS0           */
#define SYS_GPF_MFP0_PF3MFP_UART0_TXD         (0x3UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for UART0_TXD          */
#define SYS_GPF_MFP0_PF3MFP_I2C0_SCL          (0x4UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for I2C0_SCL           */
#define SYS_GPF_MFP0_PF3MFP_UART9_TXD         (0x7UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for UART9_TXD          */
#define SYS_GPF_MFP0_PF3MFP_I2C2_SMBAL        (0x8UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for I2C2_SMBAL         */
#define SYS_GPF_MFP0_PF3MFP_EPWM1_CH0         (0x9UL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for EPWM1_CH0          */
#define SYS_GPF_MFP0_PF3MFP_XT1_IN            (0xAUL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for XT1_IN             */
#define SYS_GPF_MFP0_PF3MFP_BPWM1_CH0         (0xBUL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for BPWM1_CH0          */
#define SYS_GPF_MFP0_PF3MFP_ACMP2_O           (0xDUL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for ACMP2_O            */
#define SYS_GPF_MFP0_PF3MFP_SC1_PWR           (0xEUL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for SC1_PWR            */
#define SYS_GPF_MFP0_PF3MFP_TM0_EXT           (0xFUL<<SYS_GPF_MFP0_PF3MFP_Pos)    /*!< GPF_MFP0 PF3 setting for TM0_EXT            */
#define SYS_GPF_MFP0_PF3MFP_LPUART0_TXD       (0x15UL<<SYS_GPF_MFP0_PF3MFP_Pos)   /*!< GPF_MFP0 PF3 setting for LPUART0_TXD        */
#define SYS_GPF_MFP0_PF3MFP_LPI2C0_SCL        (0x16UL<<SYS_GPF_MFP0_PF3MFP_Pos)   /*!< GPF_MFP0 PF3 setting for LPI2C0_SCL         */

/* PF.4 MFP */
#define SYS_GPF_MFP1_PF4MFP_GPIO              (0x0UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for GPIO               */
#define SYS_GPF_MFP1_PF4MFP_UART2_TXD         (0x2UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for UART2_TXD          */
#define SYS_GPF_MFP1_PF4MFP_EBI_AD0           (0x3UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for EBI_AD0            */
#define SYS_GPF_MFP1_PF4MFP_UART2_nRTS        (0x4UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for UART2_nRTS         */
#define SYS_GPF_MFP1_PF4MFP_UART0_nRTS        (0x5UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for UART0_nRTS         */
#define SYS_GPF_MFP1_PF4MFP_UART3_RXD         (0x6UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for UART3_RXD          */
#define SYS_GPF_MFP1_PF4MFP_EPWM0_CH1         (0x7UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for EPWM0_CH1          */
#define SYS_GPF_MFP1_PF4MFP_BPWM0_CH5         (0x8UL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for BPWM0_CH5          */
#define SYS_GPF_MFP1_PF4MFP_X32_OUT           (0xAUL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for X32_OUT            */
#define SYS_GPF_MFP1_PF4MFP_EQEI2_B           (0xDUL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for EQEI2_B            */
#define SYS_GPF_MFP1_PF4MFP_PSIO0_CH1         (0xEUL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for PSIO0_CH1          */
#define SYS_GPF_MFP1_PF4MFP_KPI_COL1          (0xFUL<<SYS_GPF_MFP1_PF4MFP_Pos)    /*!< GPF_MFP1 PF4 setting for KPI_COL1           */
#define SYS_GPF_MFP1_PF4MFP_SWODEC_SWO        (0x10UL<<SYS_GPF_MFP1_PF4MFP_Pos)   /*!< GPF_MFP1 PF4 setting for SWODEC_SWO         */
#define SYS_GPF_MFP1_PF4MFP_UTCPD0_VBSRCEN    (0x11UL<<SYS_GPF_MFP1_PF4MFP_Pos)   /*!< GPF_MFP1 PF4 setting for UTCPD0_VBSRCEN     */

/* PF.5 MFP */
#define SYS_GPF_MFP1_PF5MFP_GPIO              (0x0UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for GPIO               */
#define SYS_GPF_MFP1_PF5MFP_UART2_RXD         (0x2UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for UART2_RXD          */
#define SYS_GPF_MFP1_PF5MFP_EBI_AD1           (0x3UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for EBI_AD1            */
#define SYS_GPF_MFP1_PF5MFP_UART2_nCTS        (0x4UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for UART2_nCTS         */
#define SYS_GPF_MFP1_PF5MFP_UART0_nCTS        (0x5UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for UART0_nCTS         */
#define SYS_GPF_MFP1_PF5MFP_UART3_TXD         (0x6UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for UART3_TXD          */
#define SYS_GPF_MFP1_PF5MFP_EPWM0_CH0         (0x7UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for EPWM0_CH0          */
#define SYS_GPF_MFP1_PF5MFP_BPWM0_CH4         (0x8UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for BPWM0_CH4          */
#define SYS_GPF_MFP1_PF5MFP_EPWM0_SYNC_OUT    (0x9UL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for EPWM0_SYNC_OUT     */
#define SYS_GPF_MFP1_PF5MFP_X32_IN            (0xAUL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for X32_IN             */
#define SYS_GPF_MFP1_PF5MFP_EADC0_ST          (0xBUL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for EADC0_ST           */
#define SYS_GPF_MFP1_PF5MFP_EQEI2_A           (0xDUL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for EQEI2_A            */
#define SYS_GPF_MFP1_PF5MFP_PSIO0_CH0         (0xEUL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for PSIO0_CH0          */
#define SYS_GPF_MFP1_PF5MFP_KPI_COL0          (0xFUL<<SYS_GPF_MFP1_PF5MFP_Pos)    /*!< GPF_MFP1 PF5 setting for KPI_COL0           */
#define SYS_GPF_MFP1_PF5MFP_UTCPD0_VBSNKEN    (0x11UL<<SYS_GPF_MFP1_PF5MFP_Pos)   /*!< GPF_MFP1 PF5 setting for UTCPD0_VBSNKEN     */
#define SYS_GPF_MFP1_PF5MFP_LPADC0_ST         (0x17UL<<SYS_GPF_MFP1_PF5MFP_Pos)   /*!< GPF_MFP1 PF5 setting for LPADC0_ST          */

/* PF.6 MFP */
#define SYS_GPF_MFP1_PF6MFP_GPIO              (0x0UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for GPIO               */
#define SYS_GPF_MFP1_PF6MFP_EBI_ADR19         (0x2UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for EBI_ADR19          */
#define SYS_GPF_MFP1_PF6MFP_SC0_CLK           (0x3UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for SC0_CLK            */
#define SYS_GPF_MFP1_PF6MFP_I2S0_LRCK         (0x4UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for I2S0_LRCK          */
#define SYS_GPF_MFP1_PF6MFP_SPI0_MOSI         (0x5UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for SPI0_MOSI          */
#define SYS_GPF_MFP1_PF6MFP_UART4_RXD         (0x6UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for UART4_RXD          */
#define SYS_GPF_MFP1_PF6MFP_EBI_nCS0          (0x7UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for EBI_nCS0           */
#define SYS_GPF_MFP1_PF6MFP_SPI3_I2SMCLK      (0x9UL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for SPI3_I2SMCLK       */
#define SYS_GPF_MFP1_PF6MFP_TAMPER0           (0xAUL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for TAMPER0            */
#define SYS_GPF_MFP1_PF6MFP_EQEI2_INDEX       (0xDUL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for EQEI2_INDEX        */
#define SYS_GPF_MFP1_PF6MFP_TRACE_SWO         (0xEUL<<SYS_GPF_MFP1_PF6MFP_Pos)    /*!< GPF_MFP1 PF6 setting for TRACE_SWO          */

/* PF.7 MFP */
#define SYS_GPF_MFP1_PF7MFP_GPIO              (0x0UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for GPIO               */
#define SYS_GPF_MFP1_PF7MFP_EBI_ADR18         (0x2UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for EBI_ADR18          */
#define SYS_GPF_MFP1_PF7MFP_SC0_DAT           (0x3UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for SC0_DAT            */
#define SYS_GPF_MFP1_PF7MFP_I2S0_DO           (0x4UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for I2S0_DO            */
#define SYS_GPF_MFP1_PF7MFP_SPI0_MISO         (0x5UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for SPI0_MISO          */
#define SYS_GPF_MFP1_PF7MFP_UART4_TXD         (0x6UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for UART4_TXD          */
#define SYS_GPF_MFP1_PF7MFP_CCAP_DATA0        (0x7UL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for CCAP_DATA0         */
#define SYS_GPF_MFP1_PF7MFP_TAMPER1           (0xAUL<<SYS_GPF_MFP1_PF7MFP_Pos)    /*!< GPF_MFP1 PF7 setting for TAMPER1            */

/* PF.8 MFP */
#define SYS_GPF_MFP2_PF8MFP_GPIO              (0x0UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for GPIO               */
#define SYS_GPF_MFP2_PF8MFP_EBI_ADR17         (0x2UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for EBI_ADR17          */
#define SYS_GPF_MFP2_PF8MFP_SC0_RST           (0x3UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for SC0_RST            */
#define SYS_GPF_MFP2_PF8MFP_I2S0_DI           (0x4UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for I2S0_DI            */
#define SYS_GPF_MFP2_PF8MFP_SPI0_CLK          (0x5UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for SPI0_CLK           */
#define SYS_GPF_MFP2_PF8MFP_UART5_nCTS        (0x6UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for UART5_nCTS         */
#define SYS_GPF_MFP2_PF8MFP_CCAP_DATA1        (0x7UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for CCAP_DATA1         */
#define SYS_GPF_MFP2_PF8MFP_CANFD1_RXD        (0x8UL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for CANFD1_RXD         */
#define SYS_GPF_MFP2_PF8MFP_TAMPER2           (0xAUL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for TAMPER2            */
#define SYS_GPF_MFP2_PF8MFP_UART9_RXD         (0xBUL<<SYS_GPF_MFP2_PF8MFP_Pos)    /*!< GPF_MFP2 PF8 setting for UART9_RXD          */

/* PF.9 MFP */
#define SYS_GPF_MFP2_PF9MFP_GPIO              (0x0UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for GPIO               */
#define SYS_GPF_MFP2_PF9MFP_EBI_ADR16         (0x2UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for EBI_ADR16          */
#define SYS_GPF_MFP2_PF9MFP_SC0_PWR           (0x3UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for SC0_PWR            */
#define SYS_GPF_MFP2_PF9MFP_I2S0_MCLK         (0x4UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for I2S0_MCLK          */
#define SYS_GPF_MFP2_PF9MFP_SPI0_SS           (0x5UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for SPI0_SS            */
#define SYS_GPF_MFP2_PF9MFP_UART5_nRTS        (0x6UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for UART5_nRTS         */
#define SYS_GPF_MFP2_PF9MFP_CCAP_DATA2        (0x7UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for CCAP_DATA2         */
#define SYS_GPF_MFP2_PF9MFP_CANFD1_TXD        (0x8UL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for CANFD1_TXD         */
#define SYS_GPF_MFP2_PF9MFP_TAMPER3           (0xAUL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for TAMPER3            */
#define SYS_GPF_MFP2_PF9MFP_UART9_TXD         (0xBUL<<SYS_GPF_MFP2_PF9MFP_Pos)    /*!< GPF_MFP2 PF9 setting for UART9_TXD          */

/* PF.10 MFP */
#define SYS_GPF_MFP2_PF10MFP_GPIO             (0x0UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for GPIO              */
#define SYS_GPF_MFP2_PF10MFP_EBI_ADR15        (0x2UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for EBI_ADR15         */
#define SYS_GPF_MFP2_PF10MFP_SC0_nCD          (0x3UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for SC0_nCD           */
#define SYS_GPF_MFP2_PF10MFP_I2S0_BCLK        (0x4UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for I2S0_BCLK         */
#define SYS_GPF_MFP2_PF10MFP_SPI0_I2SMCLK     (0x5UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for SPI0_I2SMCLK      */
#define SYS_GPF_MFP2_PF10MFP_UART5_RXD        (0x6UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for UART5_RXD         */
#define SYS_GPF_MFP2_PF10MFP_CCAP_DATA3       (0x7UL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for CCAP_DATA3        */
#define SYS_GPF_MFP2_PF10MFP_TAMPER4          (0xAUL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for TAMPER4           */
#define SYS_GPF_MFP2_PF10MFP_UART9_nRTS       (0xBUL<<SYS_GPF_MFP2_PF10MFP_Pos)   /*!< GPF_MFP2 PF10 setting for UART9_nRTS        */

/* PF.11 MFP */
#define SYS_GPF_MFP2_PF11MFP_GPIO             (0x0UL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for GPIO              */
#define SYS_GPF_MFP2_PF11MFP_EBI_ADR14        (0x2UL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for EBI_ADR14         */
#define SYS_GPF_MFP2_PF11MFP_SPI2_MOSI        (0x3UL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for SPI2_MOSI         */
#define SYS_GPF_MFP2_PF11MFP_UART5_TXD        (0x6UL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for UART5_TXD         */
#define SYS_GPF_MFP2_PF11MFP_CCAP_DATA4       (0x7UL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for CCAP_DATA4        */
#define SYS_GPF_MFP2_PF11MFP_TAMPER5          (0xAUL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for TAMPER5           */
#define SYS_GPF_MFP2_PF11MFP_UART9_nCTS       (0xBUL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for UART9_nCTS        */
#define SYS_GPF_MFP2_PF11MFP_TM3              (0xDUL<<SYS_GPF_MFP2_PF11MFP_Pos)   /*!< GPF_MFP2 PF11 setting for TM3               */

/* PG.0 MFP */
#define SYS_GPG_MFP0_PG0MFP_GPIO              (0x0UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for GPIO               */
#define SYS_GPG_MFP0_PG0MFP_EBI_ADR8          (0x2UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for EBI_ADR8           */
#define SYS_GPG_MFP0_PG0MFP_I2C0_SCL          (0x4UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for I2C0_SCL           */
#define SYS_GPG_MFP0_PG0MFP_I2C1_SMBAL        (0x5UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for I2C1_SMBAL         */
#define SYS_GPG_MFP0_PG0MFP_UART2_RXD         (0x6UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for UART2_RXD          */
#define SYS_GPG_MFP0_PG0MFP_CANFD1_TXD        (0x7UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for CANFD1_TXD         */
#define SYS_GPG_MFP0_PG0MFP_UART1_TXD         (0x8UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for UART1_TXD          */
#define SYS_GPG_MFP0_PG0MFP_I2C3_SCL          (0x9UL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for I2C3_SCL           */
#define SYS_GPG_MFP0_PG0MFP_I2S1_DO           (0xAUL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for I2S1_DO            */
#define SYS_GPG_MFP0_PG0MFP_EPWM0_CH1         (0xBUL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for EPWM0_CH1          */
#define SYS_GPG_MFP0_PG0MFP_CCAP_PIXCLK       (0xCUL<<SYS_GPG_MFP0_PG0MFP_Pos)    /*!< GPG_MFP0 PG0 setting for CCAP_PIXCLK        */

/* PG.1 MFP */
#define SYS_GPG_MFP0_PG1MFP_GPIO              (0x0UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for GPIO               */
#define SYS_GPG_MFP0_PG1MFP_EBI_ADR9          (0x2UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for EBI_ADR9           */
#define SYS_GPG_MFP0_PG1MFP_SPI2_I2SMCLK      (0x3UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for SPI2_I2SMCLK       */
#define SYS_GPG_MFP0_PG1MFP_I2C0_SDA          (0x4UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for I2C0_SDA           */
#define SYS_GPG_MFP0_PG1MFP_I2C1_SMBSUS       (0x5UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for I2C1_SMBSUS        */
#define SYS_GPG_MFP0_PG1MFP_UART2_TXD         (0x6UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for UART2_TXD          */
#define SYS_GPG_MFP0_PG1MFP_CANFD1_RXD        (0x7UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for CANFD1_RXD         */
#define SYS_GPG_MFP0_PG1MFP_UART1_RXD         (0x8UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for UART1_RXD          */
#define SYS_GPG_MFP0_PG1MFP_I2C3_SDA          (0x9UL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for I2C3_SDA           */
#define SYS_GPG_MFP0_PG1MFP_I2S1_LRCK         (0xAUL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for I2S1_LRCK          */
#define SYS_GPG_MFP0_PG1MFP_EPWM0_CH0         (0xBUL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for EPWM0_CH0          */
#define SYS_GPG_MFP0_PG1MFP_CCAP_SCLK         (0xCUL<<SYS_GPG_MFP0_PG1MFP_Pos)    /*!< GPG_MFP0 PG1 setting for CCAP_SCLK          */

/* PG.2 MFP */
#define SYS_GPG_MFP0_PG2MFP_GPIO              (0x0UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for GPIO               */
#define SYS_GPG_MFP0_PG2MFP_EBI_ADR11         (0x2UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for EBI_ADR11          */
#define SYS_GPG_MFP0_PG2MFP_SPI2_SS           (0x3UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for SPI2_SS            */
#define SYS_GPG_MFP0_PG2MFP_I2C0_SMBAL        (0x4UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for I2C0_SMBAL         */
#define SYS_GPG_MFP0_PG2MFP_I2C1_SCL          (0x5UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for I2C1_SCL           */
#define SYS_GPG_MFP0_PG2MFP_CCAP_DATA7        (0x7UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for CCAP_DATA7         */
#define SYS_GPG_MFP0_PG2MFP_I2C3_SMBAL        (0x9UL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for I2C3_SMBAL         */
#define SYS_GPG_MFP0_PG2MFP_SC1_nCD           (0xAUL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for SC1_nCD            */
#define SYS_GPG_MFP0_PG2MFP_SPI0_I2SMCLK      (0xBUL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for SPI0_I2SMCLK       */
#define SYS_GPG_MFP0_PG2MFP_TM0               (0xDUL<<SYS_GPG_MFP0_PG2MFP_Pos)    /*!< GPG_MFP0 PG2 setting for TM0                */
#define SYS_GPG_MFP0_PG2MFP_LPTM0             (0x17UL<<SYS_GPG_MFP0_PG2MFP_Pos)   /*!< GPG_MFP0 PG2 setting for LPTM0              */

/* PG.3 MFP */
#define SYS_GPG_MFP0_PG3MFP_GPIO              (0x0UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for GPIO               */
#define SYS_GPG_MFP0_PG3MFP_EBI_ADR12         (0x2UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for EBI_ADR12          */
#define SYS_GPG_MFP0_PG3MFP_SPI2_CLK          (0x3UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for SPI2_CLK           */
#define SYS_GPG_MFP0_PG3MFP_I2C0_SMBSUS       (0x4UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for I2C0_SMBSUS        */
#define SYS_GPG_MFP0_PG3MFP_I2C1_SDA          (0x5UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for I2C1_SDA           */
#define SYS_GPG_MFP0_PG3MFP_CCAP_DATA6        (0x7UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for CCAP_DATA6         */
#define SYS_GPG_MFP0_PG3MFP_I2C3_SMBSUS       (0x9UL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for I2C3_SMBSUS        */
#define SYS_GPG_MFP0_PG3MFP_UART4_RXD         (0xAUL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for UART4_RXD          */
#define SYS_GPG_MFP0_PG3MFP_UART0_RXD         (0xBUL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for UART0_RXD          */
#define SYS_GPG_MFP0_PG3MFP_TM1               (0xDUL<<SYS_GPG_MFP0_PG3MFP_Pos)    /*!< GPG_MFP0 PG3 setting for TM1                */
#define SYS_GPG_MFP0_PG3MFP_LPTM1             (0x17UL<<SYS_GPG_MFP0_PG3MFP_Pos)   /*!< GPG_MFP0 PG3 setting for LPTM1              */

/* PG.4 MFP */
#define SYS_GPG_MFP1_PG4MFP_GPIO              (0x0UL<<SYS_GPG_MFP1_PG4MFP_Pos)    /*!< GPG_MFP1 PG4 setting for GPIO               */
#define SYS_GPG_MFP1_PG4MFP_EBI_ADR13         (0x2UL<<SYS_GPG_MFP1_PG4MFP_Pos)    /*!< GPG_MFP1 PG4 setting for EBI_ADR13          */
#define SYS_GPG_MFP1_PG4MFP_SPI2_MISO         (0x3UL<<SYS_GPG_MFP1_PG4MFP_Pos)    /*!< GPG_MFP1 PG4 setting for SPI2_MISO          */
#define SYS_GPG_MFP1_PG4MFP_CCAP_DATA5        (0x7UL<<SYS_GPG_MFP1_PG4MFP_Pos)    /*!< GPG_MFP1 PG4 setting for CCAP_DATA5         */
#define SYS_GPG_MFP1_PG4MFP_TM2               (0xDUL<<SYS_GPG_MFP1_PG4MFP_Pos)    /*!< GPG_MFP1 PG4 setting for TM2                */

/* PG.5 MFP */
#define SYS_GPG_MFP1_PG5MFP_GPIO              (0x0UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for GPIO               */
#define SYS_GPG_MFP1_PG5MFP_EBI_nCS1          (0x2UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for EBI_nCS1           */
#define SYS_GPG_MFP1_PG5MFP_SPI3_SS           (0x3UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for SPI3_SS            */
#define SYS_GPG_MFP1_PG5MFP_SC1_PWR           (0x4UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for SC1_PWR            */
#define SYS_GPG_MFP1_PG5MFP_EBI_nWRL          (0x5UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for EBI_nWRL           */
#define SYS_GPG_MFP1_PG5MFP_I2C3_SMBAL        (0x8UL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for I2C3_SMBAL         */
#define SYS_GPG_MFP1_PG5MFP_I2S1_MCLK         (0xAUL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for I2S1_MCLK          */
#define SYS_GPG_MFP1_PG5MFP_EPWM0_CH3         (0xBUL<<SYS_GPG_MFP1_PG5MFP_Pos)    /*!< GPG_MFP1 PG5 setting for EPWM0_CH3          */

/* PG.6 MFP */
#define SYS_GPG_MFP1_PG6MFP_GPIO              (0x0UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for GPIO               */
#define SYS_GPG_MFP1_PG6MFP_EBI_nCS2          (0x2UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for EBI_nCS2           */
#define SYS_GPG_MFP1_PG6MFP_SPI3_CLK          (0x3UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for SPI3_CLK           */
#define SYS_GPG_MFP1_PG6MFP_SC1_RST           (0x4UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for SC1_RST            */
#define SYS_GPG_MFP1_PG6MFP_EBI_nWRH          (0x5UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for EBI_nWRH           */
#define SYS_GPG_MFP1_PG6MFP_I2C3_SMBSUS       (0x8UL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for I2C3_SMBSUS        */
#define SYS_GPG_MFP1_PG6MFP_I2S1_DI           (0xAUL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for I2S1_DI            */
#define SYS_GPG_MFP1_PG6MFP_EPWM0_CH2         (0xBUL<<SYS_GPG_MFP1_PG6MFP_Pos)    /*!< GPG_MFP1 PG6 setting for EPWM0_CH2          */

/* PG.9 MFP */
#define SYS_GPG_MFP2_PG9MFP_GPIO              (0x0UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for GPIO               */
#define SYS_GPG_MFP2_PG9MFP_EBI_AD0           (0x2UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for EBI_AD0            */
#define SYS_GPG_MFP2_PG9MFP_SD1_DAT3          (0x3UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for SD1_DAT3           */
#define SYS_GPG_MFP2_PG9MFP_QSPI1_MISO1       (0x5UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for QSPI1_MISO1        */
#define SYS_GPG_MFP2_PG9MFP_CCAP_PIXCLK       (0x7UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for CCAP_PIXCLK        */
#define SYS_GPG_MFP2_PG9MFP_ECAP2_IC0         (0x9UL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for ECAP2_IC0          */
#define SYS_GPG_MFP2_PG9MFP_BPWM0_CH5         (0xCUL<<SYS_GPG_MFP2_PG9MFP_Pos)    /*!< GPG_MFP2 PG9 setting for BPWM0_CH5          */

/* PG.10 MFP */
#define SYS_GPG_MFP2_PG10MFP_GPIO             (0x0UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for GPIO              */
#define SYS_GPG_MFP2_PG10MFP_EBI_AD1          (0x2UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for EBI_AD1           */
#define SYS_GPG_MFP2_PG10MFP_SD1_DAT2         (0x3UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for SD1_DAT2          */
#define SYS_GPG_MFP2_PG10MFP_QSPI1_MOSI1      (0x5UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for QSPI1_MOSI1       */
#define SYS_GPG_MFP2_PG10MFP_CCAP_SCLK        (0x7UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for CCAP_SCLK         */
#define SYS_GPG_MFP2_PG10MFP_ECAP2_IC1        (0x9UL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for ECAP2_IC1         */
#define SYS_GPG_MFP2_PG10MFP_BPWM0_CH4        (0xCUL<<SYS_GPG_MFP2_PG10MFP_Pos)   /*!< GPG_MFP2 PG10 setting for BPWM0_CH4         */

/* PG.11 MFP */
#define SYS_GPG_MFP2_PG11MFP_GPIO             (0x0UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for GPIO              */
#define SYS_GPG_MFP2_PG11MFP_EBI_AD2          (0x2UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for EBI_AD2           */
#define SYS_GPG_MFP2_PG11MFP_SD1_DAT1         (0x3UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for SD1_DAT1          */
#define SYS_GPG_MFP2_PG11MFP_QSPI1_SS         (0x5UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for QSPI1_SS          */
#define SYS_GPG_MFP2_PG11MFP_UART7_TXD        (0x6UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for UART7_TXD         */
#define SYS_GPG_MFP2_PG11MFP_CCAP_SFIELD      (0x7UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for CCAP_SFIELD       */
#define SYS_GPG_MFP2_PG11MFP_ECAP2_IC2        (0x9UL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for ECAP2_IC2         */
#define SYS_GPG_MFP2_PG11MFP_BPWM0_CH3        (0xCUL<<SYS_GPG_MFP2_PG11MFP_Pos)   /*!< GPG_MFP2 PG11 setting for BPWM0_CH3         */

/* PG.12 MFP */
#define SYS_GPG_MFP3_PG12MFP_GPIO             (0x0UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for GPIO              */
#define SYS_GPG_MFP3_PG12MFP_EBI_AD3          (0x2UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for EBI_AD3           */
#define SYS_GPG_MFP3_PG12MFP_SD1_DAT0         (0x3UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for SD1_DAT0          */
#define SYS_GPG_MFP3_PG12MFP_QSPI1_CLK        (0x5UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for QSPI1_CLK         */
#define SYS_GPG_MFP3_PG12MFP_UART7_RXD        (0x6UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for UART7_RXD         */
#define SYS_GPG_MFP3_PG12MFP_CCAP_VSYNC       (0x7UL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for CCAP_VSYNC        */
#define SYS_GPG_MFP3_PG12MFP_BPWM0_CH2        (0xCUL<<SYS_GPG_MFP3_PG12MFP_Pos)   /*!< GPG_MFP3 PG12 setting for BPWM0_CH2         */

/* PG.13 MFP */
#define SYS_GPG_MFP3_PG13MFP_GPIO             (0x0UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for GPIO              */
#define SYS_GPG_MFP3_PG13MFP_EBI_AD4          (0x2UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for EBI_AD4           */
#define SYS_GPG_MFP3_PG13MFP_UART1_TXD        (0x3UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for UART1_TXD         */
#define SYS_GPG_MFP3_PG13MFP_I2C0_SCL         (0x4UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for I2C0_SCL          */
#define SYS_GPG_MFP3_PG13MFP_SPI1_MISO        (0x5UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for SPI1_MISO         */
#define SYS_GPG_MFP3_PG13MFP_QSPI1_MISO0      (0x6UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for QSPI1_MISO0       */
#define SYS_GPG_MFP3_PG13MFP_CCAP_HSYNC       (0x7UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for CCAP_HSYNC        */
#define SYS_GPG_MFP3_PG13MFP_SC1_PWR          (0x8UL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for SC1_PWR           */
#define SYS_GPG_MFP3_PG13MFP_SPIM0_D6         (0xCUL<<SYS_GPG_MFP3_PG13MFP_Pos)   /*!< GPG_MFP3 PG13 setting for SPIM0_D6          */
#define SYS_GPG_MFP3_PG13MFP_PSIO0_CH4        (0x11UL<<SYS_GPG_MFP3_PG13MFP_Pos)  /*!< GPG_MFP3 PG13 setting for PSIO0_CH4         */
#define SYS_GPG_MFP3_PG13MFP_LPI2C0_SCL       (0x16UL<<SYS_GPG_MFP3_PG13MFP_Pos)  /*!< GPG_MFP3 PG13 setting for LPI2C0_SCL        */

/* PG.14 MFP */
#define SYS_GPG_MFP3_PG14MFP_GPIO             (0x0UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for GPIO              */
#define SYS_GPG_MFP3_PG14MFP_EBI_AD5          (0x2UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for EBI_AD5           */
#define SYS_GPG_MFP3_PG14MFP_UART1_RXD        (0x3UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for UART1_RXD         */
#define SYS_GPG_MFP3_PG14MFP_I2C0_SDA         (0x4UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for I2C0_SDA          */
#define SYS_GPG_MFP3_PG14MFP_SPI1_MOSI        (0x5UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for SPI1_MOSI         */
#define SYS_GPG_MFP3_PG14MFP_QSPI1_MOSI0      (0x6UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for QSPI1_MOSI0       */
#define SYS_GPG_MFP3_PG14MFP_SC1_RST          (0x8UL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for SC1_RST           */
#define SYS_GPG_MFP3_PG14MFP_SPIM0_D7         (0xCUL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for SPIM0_D7          */
#define SYS_GPG_MFP3_PG14MFP_ACMP0_O          (0xEUL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for ACMP0_O           */
#define SYS_GPG_MFP3_PG14MFP_EADC0_ST         (0xFUL<<SYS_GPG_MFP3_PG14MFP_Pos)   /*!< GPG_MFP3 PG14 setting for EADC0_ST          */
#define SYS_GPG_MFP3_PG14MFP_PSIO0_CH5        (0x11UL<<SYS_GPG_MFP3_PG14MFP_Pos)  /*!< GPG_MFP3 PG14 setting for PSIO0_CH5         */
#define SYS_GPG_MFP3_PG14MFP_LPI2C0_SDA       (0x16UL<<SYS_GPG_MFP3_PG14MFP_Pos)  /*!< GPG_MFP3 PG14 setting for LPI2C0_SDA        */

/* PG.15 MFP */
#define SYS_GPG_MFP3_PG15MFP_GPIO             (0x0UL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for GPIO              */
#define SYS_GPG_MFP3_PG15MFP_I2C1_SCL         (0x4UL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for I2C1_SCL          */
#define SYS_GPG_MFP3_PG15MFP_SPI1_CLK         (0x5UL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for SPI1_CLK          */
#define SYS_GPG_MFP3_PG15MFP_SC1_DAT          (0x8UL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for SC1_DAT           */
#define SYS_GPG_MFP3_PG15MFP_SPIM0_RWDS       (0xCUL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for SPIM0_RWDS        */
#define SYS_GPG_MFP3_PG15MFP_ACMP1_O          (0xEUL<<SYS_GPG_MFP3_PG15MFP_Pos)   /*!< GPG_MFP3 PG15 setting for ACMP1_O           */
#define SYS_GPG_MFP3_PG15MFP_PSIO0_CH6        (0x11UL<<SYS_GPG_MFP3_PG15MFP_Pos)  /*!< GPG_MFP3 PG15 setting for PSIO0_CH6         */

/* PH.0 MFP */
#define SYS_GPH_MFP0_PH0MFP_GPIO              (0x0UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for GPIO               */
#define SYS_GPH_MFP0_PH0MFP_EBI_ADR7          (0x2UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for EBI_ADR7           */
#define SYS_GPH_MFP0_PH0MFP_SPI3_MISO         (0x3UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for SPI3_MISO          */
#define SYS_GPH_MFP0_PH0MFP_UART5_TXD         (0x4UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for UART5_TXD          */
#define SYS_GPH_MFP0_PH0MFP_SC1_DAT           (0x5UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for SC1_DAT            */
#define SYS_GPH_MFP0_PH0MFP_I2C3_SCL          (0x8UL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for I2C3_SCL           */
#define SYS_GPH_MFP0_PH0MFP_TM0_EXT           (0xDUL<<SYS_GPH_MFP0_PH0MFP_Pos)    /*!< GPH_MFP0 PH0 setting for TM0_EXT            */

/* PH.1 MFP */
#define SYS_GPH_MFP0_PH1MFP_GPIO              (0x0UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for GPIO               */
#define SYS_GPH_MFP0_PH1MFP_EBI_ADR6          (0x2UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for EBI_ADR6           */
#define SYS_GPH_MFP0_PH1MFP_SPI3_MOSI         (0x3UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for SPI3_MOSI          */
#define SYS_GPH_MFP0_PH1MFP_UART5_RXD         (0x4UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for UART5_RXD          */
#define SYS_GPH_MFP0_PH1MFP_SC1_CLK           (0x5UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for SC1_CLK            */
#define SYS_GPH_MFP0_PH1MFP_I2C3_SDA          (0x8UL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for I2C3_SDA           */
#define SYS_GPH_MFP0_PH1MFP_TM1_EXT           (0xDUL<<SYS_GPH_MFP0_PH1MFP_Pos)    /*!< GPH_MFP0 PH1 setting for TM1_EXT            */

/* PH.2 MFP */
#define SYS_GPH_MFP0_PH2MFP_GPIO              (0x0UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for GPIO               */
#define SYS_GPH_MFP0_PH2MFP_EBI_ADR5          (0x2UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for EBI_ADR5           */
#define SYS_GPH_MFP0_PH2MFP_UART5_nRTS        (0x4UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for UART5_nRTS         */
#define SYS_GPH_MFP0_PH2MFP_UART4_TXD         (0x5UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for UART4_TXD          */
#define SYS_GPH_MFP0_PH2MFP_I2C0_SCL          (0x6UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for I2C0_SCL           */
#define SYS_GPH_MFP0_PH2MFP_UART9_RXD         (0x7UL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for UART9_RXD          */
#define SYS_GPH_MFP0_PH2MFP_TM2_EXT           (0xDUL<<SYS_GPH_MFP0_PH2MFP_Pos)    /*!< GPH_MFP0 PH2 setting for TM2_EXT            */

/* PH.3 MFP */
#define SYS_GPH_MFP0_PH3MFP_GPIO              (0x0UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for GPIO               */
#define SYS_GPH_MFP0_PH3MFP_EBI_ADR4          (0x2UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for EBI_ADR4           */
#define SYS_GPH_MFP0_PH3MFP_SPI1_I2SMCLK      (0x3UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for SPI1_I2SMCLK       */
#define SYS_GPH_MFP0_PH3MFP_UART5_nCTS        (0x4UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for UART5_nCTS         */
#define SYS_GPH_MFP0_PH3MFP_UART4_RXD         (0x5UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for UART4_RXD          */
#define SYS_GPH_MFP0_PH3MFP_I2C0_SDA          (0x6UL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for I2C0_SDA           */
#define SYS_GPH_MFP0_PH3MFP_TM3_EXT           (0xDUL<<SYS_GPH_MFP0_PH3MFP_Pos)    /*!< GPH_MFP0 PH3 setting for TM3_EXT            */

/* PH.4 MFP */
#define SYS_GPH_MFP1_PH4MFP_GPIO              (0x0UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for GPIO               */
#define SYS_GPH_MFP1_PH4MFP_EBI_ADR3          (0x2UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for EBI_ADR3           */
#define SYS_GPH_MFP1_PH4MFP_SPI1_MISO         (0x3UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for SPI1_MISO          */
#define SYS_GPH_MFP1_PH4MFP_UART7_nRTS        (0x4UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for UART7_nRTS         */
#define SYS_GPH_MFP1_PH4MFP_UART6_TXD         (0x5UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for UART6_TXD          */
#define SYS_GPH_MFP1_PH4MFP_SPI3_I2SMCLK      (0x9UL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for SPI3_I2SMCLK       */
#define SYS_GPH_MFP1_PH4MFP_EPWM0_CH5         (0xBUL<<SYS_GPH_MFP1_PH4MFP_Pos)    /*!< GPH_MFP1 PH4 setting for EPWM0_CH5          */

/* PH.5 MFP */
#define SYS_GPH_MFP1_PH5MFP_GPIO              (0x0UL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for GPIO               */
#define SYS_GPH_MFP1_PH5MFP_EBI_ADR2          (0x2UL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for EBI_ADR2           */
#define SYS_GPH_MFP1_PH5MFP_SPI1_MOSI         (0x3UL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for SPI1_MOSI          */
#define SYS_GPH_MFP1_PH5MFP_UART7_nCTS        (0x4UL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for UART7_nCTS         */
#define SYS_GPH_MFP1_PH5MFP_UART6_RXD         (0x5UL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for UART6_RXD          */
#define SYS_GPH_MFP1_PH5MFP_EPWM0_CH4         (0xBUL<<SYS_GPH_MFP1_PH5MFP_Pos)    /*!< GPH_MFP1 PH5 setting for EPWM0_CH4          */

/* PH.6 MFP */
#define SYS_GPH_MFP1_PH6MFP_GPIO              (0x0UL<<SYS_GPH_MFP1_PH6MFP_Pos)    /*!< GPH_MFP1 PH6 setting for GPIO               */
#define SYS_GPH_MFP1_PH6MFP_EBI_ADR1          (0x2UL<<SYS_GPH_MFP1_PH6MFP_Pos)    /*!< GPH_MFP1 PH6 setting for EBI_ADR1           */
#define SYS_GPH_MFP1_PH6MFP_SPI1_CLK          (0x3UL<<SYS_GPH_MFP1_PH6MFP_Pos)    /*!< GPH_MFP1 PH6 setting for SPI1_CLK           */
#define SYS_GPH_MFP1_PH6MFP_UART7_TXD         (0x4UL<<SYS_GPH_MFP1_PH6MFP_Pos)    /*!< GPH_MFP1 PH6 setting for UART7_TXD          */
#define SYS_GPH_MFP1_PH6MFP_UART9_nCTS        (0x7UL<<SYS_GPH_MFP1_PH6MFP_Pos)    /*!< GPH_MFP1 PH6 setting for UART9_nCTS         */

/* PH.7 MFP */
#define SYS_GPH_MFP1_PH7MFP_GPIO              (0x0UL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for GPIO               */
#define SYS_GPH_MFP1_PH7MFP_EBI_ADR0          (0x2UL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for EBI_ADR0           */
#define SYS_GPH_MFP1_PH7MFP_SPI1_SS           (0x3UL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for SPI1_SS            */
#define SYS_GPH_MFP1_PH7MFP_UART7_RXD         (0x4UL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for UART7_RXD          */
#define SYS_GPH_MFP1_PH7MFP_UART9_nRTS        (0x7UL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for UART9_nRTS         */
#define SYS_GPH_MFP1_PH7MFP_I2S1_BCLK         (0xBUL<<SYS_GPH_MFP1_PH7MFP_Pos)    /*!< GPH_MFP1 PH7 setting for I2S1_BCLK          */

/* PH.8 MFP */
#define SYS_GPH_MFP2_PH8MFP_GPIO              (0x0UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for GPIO               */
#define SYS_GPH_MFP2_PH8MFP_EBI_AD12          (0x2UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for EBI_AD12           */
#define SYS_GPH_MFP2_PH8MFP_QSPI0_CLK         (0x3UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for QSPI0_CLK          */
#define SYS_GPH_MFP2_PH8MFP_SC2_PWR           (0x4UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for SC2_PWR            */
#define SYS_GPH_MFP2_PH8MFP_I2S0_DI           (0x5UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for I2S0_DI            */
#define SYS_GPH_MFP2_PH8MFP_SPI1_CLK          (0x6UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for SPI1_CLK           */
#define SYS_GPH_MFP2_PH8MFP_UART3_nRTS        (0x7UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for UART3_nRTS         */
#define SYS_GPH_MFP2_PH8MFP_I2C1_SMBAL        (0x8UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for I2C1_SMBAL         */
#define SYS_GPH_MFP2_PH8MFP_I2C2_SCL          (0x9UL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for I2C2_SCL           */
#define SYS_GPH_MFP2_PH8MFP_UART1_TXD         (0xAUL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for UART1_TXD          */
#define SYS_GPH_MFP2_PH8MFP_UART9_nCTS        (0xDUL<<SYS_GPH_MFP2_PH8MFP_Pos)    /*!< GPH_MFP2 PH8 setting for UART9_nCTS         */
#define SYS_GPH_MFP2_PH8MFP_I3C0_SCL          (0x10UL<<SYS_GPH_MFP2_PH8MFP_Pos)   /*!< GPH_MFP2 PH8 setting for I3C0_SCL           */

/* PH.9 MFP */
#define SYS_GPH_MFP2_PH9MFP_GPIO              (0x0UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for GPIO               */
#define SYS_GPH_MFP2_PH9MFP_EBI_AD13          (0x2UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for EBI_AD13           */
#define SYS_GPH_MFP2_PH9MFP_QSPI0_SS          (0x3UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for QSPI0_SS           */
#define SYS_GPH_MFP2_PH9MFP_SC2_RST           (0x4UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for SC2_RST            */
#define SYS_GPH_MFP2_PH9MFP_I2S0_DO           (0x5UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for I2S0_DO            */
#define SYS_GPH_MFP2_PH9MFP_SPI1_SS           (0x6UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for SPI1_SS            */
#define SYS_GPH_MFP2_PH9MFP_UART3_nCTS        (0x7UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for UART3_nCTS         */
#define SYS_GPH_MFP2_PH9MFP_I2C1_SMBSUS       (0x8UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for I2C1_SMBSUS        */
#define SYS_GPH_MFP2_PH9MFP_I2C2_SDA          (0x9UL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for I2C2_SDA           */
#define SYS_GPH_MFP2_PH9MFP_UART1_RXD         (0xAUL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for UART1_RXD          */
#define SYS_GPH_MFP2_PH9MFP_UART9_nRTS        (0xDUL<<SYS_GPH_MFP2_PH9MFP_Pos)    /*!< GPH_MFP2 PH9 setting for UART9_nRTS         */
#define SYS_GPH_MFP2_PH9MFP_I3C0_SDA          (0x10UL<<SYS_GPH_MFP2_PH9MFP_Pos)   /*!< GPH_MFP2 PH9 setting for I3C0_SDA           */

/* PH.10 MFP */
#define SYS_GPH_MFP2_PH10MFP_GPIO             (0x0UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for GPIO              */
#define SYS_GPH_MFP2_PH10MFP_EBI_AD14         (0x2UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for EBI_AD14          */
#define SYS_GPH_MFP2_PH10MFP_QSPI0_MISO1      (0x3UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for QSPI0_MISO1       */
#define SYS_GPH_MFP2_PH10MFP_SC2_nCD          (0x4UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for SC2_nCD           */
#define SYS_GPH_MFP2_PH10MFP_I2S0_LRCK        (0x5UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for I2S0_LRCK         */
#define SYS_GPH_MFP2_PH10MFP_SPI1_I2SMCLK     (0x6UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for SPI1_I2SMCLK      */
#define SYS_GPH_MFP2_PH10MFP_UART4_TXD        (0x7UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for UART4_TXD         */
#define SYS_GPH_MFP2_PH10MFP_UART0_TXD        (0x8UL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for UART0_TXD         */
#define SYS_GPH_MFP2_PH10MFP_UART9_TXD        (0xDUL<<SYS_GPH_MFP2_PH10MFP_Pos)   /*!< GPH_MFP2 PH10 setting for UART9_TXD         */
#define SYS_GPH_MFP2_PH10MFP_I3C0_PUPEN       (0x10UL<<SYS_GPH_MFP2_PH10MFP_Pos)  /*!< GPH_MFP2 PH10 setting for I3C0_PUPEN        */
#define SYS_GPH_MFP2_PH10MFP_LPUART0_TXD      (0x15UL<<SYS_GPH_MFP2_PH10MFP_Pos)  /*!< GPH_MFP2 PH10 setting for LPUART0_TXD       */

/* PH.11 MFP */
#define SYS_GPH_MFP2_PH11MFP_GPIO             (0x0UL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for GPIO              */
#define SYS_GPH_MFP2_PH11MFP_EBI_AD15         (0x2UL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for EBI_AD15          */
#define SYS_GPH_MFP2_PH11MFP_QSPI0_MOSI1      (0x3UL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for QSPI0_MOSI1       */
#define SYS_GPH_MFP2_PH11MFP_UART4_RXD        (0x7UL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for UART4_RXD         */
#define SYS_GPH_MFP2_PH11MFP_UART0_RXD        (0x8UL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for UART0_RXD         */
#define SYS_GPH_MFP2_PH11MFP_EPWM0_CH5        (0xBUL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for EPWM0_CH5         */
#define SYS_GPH_MFP2_PH11MFP_UART9_RXD        (0xDUL<<SYS_GPH_MFP2_PH11MFP_Pos)   /*!< GPH_MFP2 PH11 setting for UART9_RXD         */
#define SYS_GPH_MFP2_PH11MFP_LPUART0_RXD      (0x15UL<<SYS_GPH_MFP2_PH11MFP_Pos)  /*!< GPH_MFP2 PH11 setting for LPUART0_RXD       */

/* PH.12 MFP */
#define SYS_GPH_MFP3_PH12MFP_GPIO             (0x0UL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for GPIO              */
#define SYS_GPH_MFP3_PH12MFP_EBI_AD0          (0x2UL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for EBI_AD0           */
#define SYS_GPH_MFP3_PH12MFP_UART9_TXD        (0x3UL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for UART9_TXD         */
#define SYS_GPH_MFP3_PH12MFP_QSPI1_MISO1      (0x6UL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for QSPI1_MISO1       */
#define SYS_GPH_MFP3_PH12MFP_CCAP_PIXCLK      (0x7UL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for CCAP_PIXCLK       */
#define SYS_GPH_MFP3_PH12MFP_SPIM0_CLKN       (0xCUL<<SYS_GPH_MFP3_PH12MFP_Pos)   /*!< GPH_MFP3 PH12 setting for SPIM0_CLKN        */

/* PH.13 MFP */
#define SYS_GPH_MFP3_PH13MFP_GPIO             (0x0UL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for GPIO              */
#define SYS_GPH_MFP3_PH13MFP_EBI_AD1          (0x2UL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for EBI_AD1           */
#define SYS_GPH_MFP3_PH13MFP_UART9_RXD        (0x3UL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for UART9_RXD         */
#define SYS_GPH_MFP3_PH13MFP_QSPI1_MOSI1      (0x6UL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for QSPI1_MOSI1       */
#define SYS_GPH_MFP3_PH13MFP_CCAP_SCLK        (0x7UL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for CCAP_SCLK         */
#define SYS_GPH_MFP3_PH13MFP_SPIM0_CLK        (0xCUL<<SYS_GPH_MFP3_PH13MFP_Pos)   /*!< GPH_MFP3 PH13 setting for SPIM0_CLK         */

/* PH.14 MFP */
#define SYS_GPH_MFP3_PH14MFP_GPIO             (0x0UL<<SYS_GPH_MFP3_PH14MFP_Pos)   /*!< GPH_MFP3 PH14 setting for GPIO              */
#define SYS_GPH_MFP3_PH14MFP_EBI_AD2          (0x2UL<<SYS_GPH_MFP3_PH14MFP_Pos)   /*!< GPH_MFP3 PH14 setting for EBI_AD2           */
#define SYS_GPH_MFP3_PH14MFP_QSPI1_SS         (0x6UL<<SYS_GPH_MFP3_PH14MFP_Pos)   /*!< GPH_MFP3 PH14 setting for QSPI1_SS          */
#define SYS_GPH_MFP3_PH14MFP_CCAP_SFIELD      (0x7UL<<SYS_GPH_MFP3_PH14MFP_Pos)   /*!< GPH_MFP3 PH14 setting for CCAP_SFIELD       */
#define SYS_GPH_MFP3_PH14MFP_SPIM0_D4         (0xCUL<<SYS_GPH_MFP3_PH14MFP_Pos)   /*!< GPH_MFP3 PH14 setting for SPIM0_D4          */

/* PH.15 MFP */
#define SYS_GPH_MFP3_PH15MFP_GPIO             (0x0UL<<SYS_GPH_MFP3_PH15MFP_Pos)   /*!< GPH_MFP3 PH15 setting for GPIO              */
#define SYS_GPH_MFP3_PH15MFP_EBI_AD3          (0x2UL<<SYS_GPH_MFP3_PH15MFP_Pos)   /*!< GPH_MFP3 PH15 setting for EBI_AD3           */
#define SYS_GPH_MFP3_PH15MFP_QSPI1_CLK        (0x6UL<<SYS_GPH_MFP3_PH15MFP_Pos)   /*!< GPH_MFP3 PH15 setting for QSPI1_CLK         */
#define SYS_GPH_MFP3_PH15MFP_CCAP_VSYNC       (0x7UL<<SYS_GPH_MFP3_PH15MFP_Pos)   /*!< GPH_MFP3 PH15 setting for CCAP_VSYNC        */
#define SYS_GPH_MFP3_PH15MFP_SPIM0_D5         (0xCUL<<SYS_GPH_MFP3_PH15MFP_Pos)   /*!< GPH_MFP3 PH15 setting for SPIM0_D5          */

/* PI.6 MFP */
#define SYS_GPI_MFP1_PI6MFP_GPIO              (0x0UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for GPIO               */
#define SYS_GPI_MFP1_PI6MFP_SC1_nCD           (0x5UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for SC1_nCD            */
#define SYS_GPI_MFP1_PI6MFP_I2S0_BCLK         (0x6UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for I2S0_BCLK          */
#define SYS_GPI_MFP1_PI6MFP_SPI1_I2SMCLK      (0x7UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for SPI1_I2SMCLK       */
#define SYS_GPI_MFP1_PI6MFP_UART2_TXD         (0x8UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for UART2_TXD          */
#define SYS_GPI_MFP1_PI6MFP_I2C1_SCL          (0x9UL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for I2C1_SCL           */
#define SYS_GPI_MFP1_PI6MFP_USB_VBUS_ST       (0xFUL<<SYS_GPI_MFP1_PI6MFP_Pos)    /*!< GPI_MFP1 PI6 setting for USB_VBUS_ST        */

/* PI.7 MFP */
#define SYS_GPI_MFP1_PI7MFP_GPIO              (0x0UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for GPIO               */
#define SYS_GPI_MFP1_PI7MFP_SC1_PWR           (0x5UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for SC1_PWR            */
#define SYS_GPI_MFP1_PI7MFP_I2S0_MCLK         (0x6UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for I2S0_MCLK          */
#define SYS_GPI_MFP1_PI7MFP_SPI1_MISO         (0x7UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for SPI1_MISO          */
#define SYS_GPI_MFP1_PI7MFP_UART2_RXD         (0x8UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for UART2_RXD          */
#define SYS_GPI_MFP1_PI7MFP_I2C1_SDA          (0x9UL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for I2C1_SDA           */
#define SYS_GPI_MFP1_PI7MFP_USB_VBUS_EN       (0xFUL<<SYS_GPI_MFP1_PI7MFP_Pos)    /*!< GPI_MFP1 PI7 setting for USB_VBUS_EN        */

/* PI.8 MFP */
#define SYS_GPI_MFP2_PI8MFP_GPIO              (0x0UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for GPIO               */
#define SYS_GPI_MFP2_PI8MFP_SC1_RST           (0x5UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for SC1_RST            */
#define SYS_GPI_MFP2_PI8MFP_I2S0_DI           (0x6UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for I2S0_DI            */
#define SYS_GPI_MFP2_PI8MFP_SPI1_MOSI         (0x7UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for SPI1_MOSI          */
#define SYS_GPI_MFP2_PI8MFP_UART2_nRTS        (0x8UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for UART2_nRTS         */
#define SYS_GPI_MFP2_PI8MFP_I2C0_SMBAL        (0x9UL<<SYS_GPI_MFP2_PI8MFP_Pos)    /*!< GPI_MFP2 PI8 setting for I2C0_SMBAL         */

/* PI.9 MFP */
#define SYS_GPI_MFP2_PI9MFP_GPIO              (0x0UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for GPIO               */
#define SYS_GPI_MFP2_PI9MFP_SC1_DAT           (0x5UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for SC1_DAT            */
#define SYS_GPI_MFP2_PI9MFP_I2S0_DO           (0x6UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for I2S0_DO            */
#define SYS_GPI_MFP2_PI9MFP_SPI1_CLK          (0x7UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for SPI1_CLK           */
#define SYS_GPI_MFP2_PI9MFP_UART2_nCTS        (0x8UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for UART2_nCTS         */
#define SYS_GPI_MFP2_PI9MFP_I2C0_SMBSUS       (0x9UL<<SYS_GPI_MFP2_PI9MFP_Pos)    /*!< GPI_MFP2 PI9 setting for I2C0_SMBSUS        */

/* PI.10 MFP */
#define SYS_GPI_MFP2_PI10MFP_GPIO             (0x0UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for GPIO              */
#define SYS_GPI_MFP2_PI10MFP_SC1_CLK          (0x5UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for SC1_CLK           */
#define SYS_GPI_MFP2_PI10MFP_I2S0_LRCK        (0x6UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for I2S0_LRCK         */
#define SYS_GPI_MFP2_PI10MFP_SPI1_SS          (0x7UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for SPI1_SS           */
#define SYS_GPI_MFP2_PI10MFP_UART2_TXD        (0x8UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for UART2_TXD         */
#define SYS_GPI_MFP2_PI10MFP_I2C0_SCL         (0x9UL<<SYS_GPI_MFP2_PI10MFP_Pos)   /*!< GPI_MFP2 PI10 setting for I2C0_SCL          */

/* PI.11 MFP */
#define SYS_GPI_MFP2_PI11MFP_GPIO             (0x0UL<<SYS_GPI_MFP2_PI11MFP_Pos)   /*!< GPI_MFP2 PI11 setting for GPIO              */
#define SYS_GPI_MFP2_PI11MFP_UART2_RXD        (0x8UL<<SYS_GPI_MFP2_PI11MFP_Pos)   /*!< GPI_MFP2 PI11 setting for UART2_RXD         */
#define SYS_GPI_MFP2_PI11MFP_I2C0_SDA         (0x9UL<<SYS_GPI_MFP2_PI11MFP_Pos)   /*!< GPI_MFP2 PI11 setting for I2C0_SDA          */

/* PI.12 MFP */
#define SYS_GPI_MFP3_PI12MFP_GPIO             (0x0UL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for GPIO              */
#define SYS_GPI_MFP3_PI12MFP_QSPI0_MISO1      (0x4UL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for QSPI0_MISO1       */
#define SYS_GPI_MFP3_PI12MFP_CANFD0_TXD       (0xAUL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for CANFD0_TXD        */
#define SYS_GPI_MFP3_PI12MFP_UART4_TXD        (0xBUL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for UART4_TXD         */
#define SYS_GPI_MFP3_PI12MFP_EPWM1_CH0        (0xCUL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for EPWM1_CH0         */
#define SYS_GPI_MFP3_PI12MFP_I2C3_SMBAL       (0xFUL<<SYS_GPI_MFP3_PI12MFP_Pos)   /*!< GPI_MFP3 PI12 setting for I2C3_SMBAL        */

/* PI.13 MFP */
#define SYS_GPI_MFP3_PI13MFP_GPIO             (0x0UL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for GPIO              */
#define SYS_GPI_MFP3_PI13MFP_QSPI0_MOSI1      (0x4UL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for QSPI0_MOSI1       */
#define SYS_GPI_MFP3_PI13MFP_CANFD0_RXD       (0xAUL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for CANFD0_RXD        */
#define SYS_GPI_MFP3_PI13MFP_UART4_RXD        (0xBUL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for UART4_RXD         */
#define SYS_GPI_MFP3_PI13MFP_EPWM1_CH1        (0xCUL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for EPWM1_CH1         */
#define SYS_GPI_MFP3_PI13MFP_I2C3_SMBSUS      (0xFUL<<SYS_GPI_MFP3_PI13MFP_Pos)   /*!< GPI_MFP3 PI13 setting for I2C3_SMBSUS       */

/* PI.14 MFP */
#define SYS_GPI_MFP3_PI14MFP_GPIO             (0x0UL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for GPIO              */
#define SYS_GPI_MFP3_PI14MFP_QSPI0_SS         (0x4UL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for QSPI0_SS          */
#define SYS_GPI_MFP3_PI14MFP_UART8_nCTS       (0x7UL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for UART8_nCTS        */
#define SYS_GPI_MFP3_PI14MFP_CANFD1_TXD       (0xAUL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for CANFD1_TXD        */
#define SYS_GPI_MFP3_PI14MFP_UART3_TXD        (0xBUL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for UART3_TXD         */
#define SYS_GPI_MFP3_PI14MFP_EPWM1_CH2        (0xCUL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for EPWM1_CH2         */
#define SYS_GPI_MFP3_PI14MFP_I2C3_SCL         (0xFUL<<SYS_GPI_MFP3_PI14MFP_Pos)   /*!< GPI_MFP3 PI14 setting for I2C3_SCL          */

/* PI.15 MFP */
#define SYS_GPI_MFP3_PI15MFP_GPIO             (0x0UL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for GPIO              */
#define SYS_GPI_MFP3_PI15MFP_QSPI0_CLK        (0x4UL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for QSPI0_CLK         */
#define SYS_GPI_MFP3_PI15MFP_UART8_nRTS       (0x7UL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for UART8_nRTS        */
#define SYS_GPI_MFP3_PI15MFP_CANFD1_RXD       (0xAUL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for CANFD1_RXD        */
#define SYS_GPI_MFP3_PI15MFP_UART3_RXD        (0xBUL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for UART3_RXD         */
#define SYS_GPI_MFP3_PI15MFP_EPWM1_CH3        (0xCUL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for EPWM1_CH3         */
#define SYS_GPI_MFP3_PI15MFP_I2C3_SDA         (0xFUL<<SYS_GPI_MFP3_PI15MFP_Pos)   /*!< GPI_MFP3 PI15 setting for I2C3_SDA          */

/* PJ.0 MFP */
#define SYS_GPJ_MFP0_PJ0MFP_GPIO              (0x0UL<<SYS_GPJ_MFP0_PJ0MFP_Pos)    /*!< GPJ_MFP0 PJ0 setting for GPIO               */
#define SYS_GPJ_MFP0_PJ0MFP_UTCPD0_DISCHG     (0x2UL<<SYS_GPJ_MFP0_PJ0MFP_Pos)    /*!< GPJ_MFP0 PJ0 setting for UTCPD0_DISCHG      */
#define SYS_GPJ_MFP0_PJ0MFP_QSPI0_MISO0       (0x4UL<<SYS_GPJ_MFP0_PJ0MFP_Pos)    /*!< GPJ_MFP0 PJ0 setting for QSPI0_MISO0        */
#define SYS_GPJ_MFP0_PJ0MFP_UART8_TXD         (0x7UL<<SYS_GPJ_MFP0_PJ0MFP_Pos)    /*!< GPJ_MFP0 PJ0 setting for UART8_TXD          */
#define SYS_GPJ_MFP0_PJ0MFP_EPWM1_CH4         (0xCUL<<SYS_GPJ_MFP0_PJ0MFP_Pos)    /*!< GPJ_MFP0 PJ0 setting for EPWM1_CH4          */

/* PJ.1 MFP */
#define SYS_GPJ_MFP0_PJ1MFP_GPIO              (0x0UL<<SYS_GPJ_MFP0_PJ1MFP_Pos)    /*!< GPJ_MFP0 PJ1 setting for GPIO               */
#define SYS_GPJ_MFP0_PJ1MFP_QSPI0_MOSI0       (0x4UL<<SYS_GPJ_MFP0_PJ1MFP_Pos)    /*!< GPJ_MFP0 PJ1 setting for QSPI0_MOSI0        */
#define SYS_GPJ_MFP0_PJ1MFP_UART8_RXD         (0x7UL<<SYS_GPJ_MFP0_PJ1MFP_Pos)    /*!< GPJ_MFP0 PJ1 setting for UART8_RXD          */
#define SYS_GPJ_MFP0_PJ1MFP_EPWM1_CH5         (0xCUL<<SYS_GPJ_MFP0_PJ1MFP_Pos)    /*!< GPJ_MFP0 PJ1 setting for EPWM1_CH5          */

/* PJ.2 MFP */
#define SYS_GPJ_MFP0_PJ2MFP_GPIO              (0x0UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for GPIO               */
#define SYS_GPJ_MFP0_PJ2MFP_EBI_AD5           (0x2UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for EBI_AD5            */
#define SYS_GPJ_MFP0_PJ2MFP_UART8_nCTS        (0x3UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for UART8_nCTS         */
#define SYS_GPJ_MFP0_PJ2MFP_I2C3_SMBAL        (0x5UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for I2C3_SMBAL         */
#define SYS_GPJ_MFP0_PJ2MFP_QSPI1_SS          (0x6UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for QSPI1_SS           */
#define SYS_GPJ_MFP0_PJ2MFP_CCAP_DATA5        (0x7UL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for CCAP_DATA5         */
#define SYS_GPJ_MFP0_PJ2MFP_CANFD0_TXD        (0xAUL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for CANFD0_TXD         */
#define SYS_GPJ_MFP0_PJ2MFP_SPIM0_RESETN      (0xCUL<<SYS_GPJ_MFP0_PJ2MFP_Pos)    /*!< GPJ_MFP0 PJ2 setting for SPIM0_RESETN       */

/* PJ.3 MFP */
#define SYS_GPJ_MFP0_PJ3MFP_GPIO              (0x0UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for GPIO               */
#define SYS_GPJ_MFP0_PJ3MFP_EBI_AD4           (0x2UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for EBI_AD4            */
#define SYS_GPJ_MFP0_PJ3MFP_UART8_nRTS        (0x3UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for UART8_nRTS         */
#define SYS_GPJ_MFP0_PJ3MFP_I2C3_SMBSUS       (0x5UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for I2C3_SMBSUS        */
#define SYS_GPJ_MFP0_PJ3MFP_QSPI1_CLK         (0x6UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for QSPI1_CLK          */
#define SYS_GPJ_MFP0_PJ3MFP_CCAP_DATA4        (0x7UL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for CCAP_DATA4         */
#define SYS_GPJ_MFP0_PJ3MFP_CANFD0_RXD        (0xAUL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for CANFD0_RXD         */
#define SYS_GPJ_MFP0_PJ3MFP_SPIM0_MOSI        (0xCUL<<SYS_GPJ_MFP0_PJ3MFP_Pos)    /*!< GPJ_MFP0 PJ3 setting for SPIM0_MOSI         */

/* PJ.4 MFP */
#define SYS_GPJ_MFP1_PJ4MFP_GPIO              (0x0UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for GPIO               */
#define SYS_GPJ_MFP1_PJ4MFP_EBI_AD3           (0x2UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for EBI_AD3            */
#define SYS_GPJ_MFP1_PJ4MFP_UART8_TXD         (0x3UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for UART8_TXD          */
#define SYS_GPJ_MFP1_PJ4MFP_I2C3_SCL          (0x5UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for I2C3_SCL           */
#define SYS_GPJ_MFP1_PJ4MFP_QSPI1_MISO0       (0x6UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for QSPI1_MISO0        */
#define SYS_GPJ_MFP1_PJ4MFP_CCAP_DATA3        (0x7UL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for CCAP_DATA3         */
#define SYS_GPJ_MFP1_PJ4MFP_CANFD1_TXD        (0xAUL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for CANFD1_TXD         */
#define SYS_GPJ_MFP1_PJ4MFP_SPIM0_MISO        (0xCUL<<SYS_GPJ_MFP1_PJ4MFP_Pos)    /*!< GPJ_MFP1 PJ4 setting for SPIM0_MISO         */

/* PJ.5 MFP */
#define SYS_GPJ_MFP1_PJ5MFP_GPIO              (0x0UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for GPIO               */
#define SYS_GPJ_MFP1_PJ5MFP_EBI_AD2           (0x2UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for EBI_AD2            */
#define SYS_GPJ_MFP1_PJ5MFP_UART8_RXD         (0x3UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for UART8_RXD          */
#define SYS_GPJ_MFP1_PJ5MFP_I2C3_SDA          (0x5UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for I2C3_SDA           */
#define SYS_GPJ_MFP1_PJ5MFP_QSPI1_MOSI0       (0x6UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for QSPI1_MOSI0        */
#define SYS_GPJ_MFP1_PJ5MFP_CCAP_DATA2        (0x7UL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for CCAP_DATA2         */
#define SYS_GPJ_MFP1_PJ5MFP_CANFD1_RXD        (0xAUL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for CANFD1_RXD         */
#define SYS_GPJ_MFP1_PJ5MFP_SPIM0_D2          (0xCUL<<SYS_GPJ_MFP1_PJ5MFP_Pos)    /*!< GPJ_MFP1 PJ5 setting for SPIM0_D2           */

/* PJ.6 MFP */
#define SYS_GPJ_MFP1_PJ6MFP_GPIO              (0x0UL<<SYS_GPJ_MFP1_PJ6MFP_Pos)    /*!< GPJ_MFP1 PJ6 setting for GPIO               */
#define SYS_GPJ_MFP1_PJ6MFP_EBI_AD1           (0x2UL<<SYS_GPJ_MFP1_PJ6MFP_Pos)    /*!< GPJ_MFP1 PJ6 setting for EBI_AD1            */
#define SYS_GPJ_MFP1_PJ6MFP_UART9_nCTS        (0x3UL<<SYS_GPJ_MFP1_PJ6MFP_Pos)    /*!< GPJ_MFP1 PJ6 setting for UART9_nCTS         */
#define SYS_GPJ_MFP1_PJ6MFP_CCAP_DATA1        (0x7UL<<SYS_GPJ_MFP1_PJ6MFP_Pos)    /*!< GPJ_MFP1 PJ6 setting for CCAP_DATA1         */
#define SYS_GPJ_MFP1_PJ6MFP_SPIM0_D3          (0xCUL<<SYS_GPJ_MFP1_PJ6MFP_Pos)    /*!< GPJ_MFP1 PJ6 setting for SPIM0_D3           */

/* PJ.7 MFP */
#define SYS_GPJ_MFP1_PJ7MFP_GPIO              (0x0UL<<SYS_GPJ_MFP1_PJ7MFP_Pos)    /*!< GPJ_MFP1 PJ7 setting for GPIO               */
#define SYS_GPJ_MFP1_PJ7MFP_EBI_AD0           (0x2UL<<SYS_GPJ_MFP1_PJ7MFP_Pos)    /*!< GPJ_MFP1 PJ7 setting for EBI_AD0            */
#define SYS_GPJ_MFP1_PJ7MFP_UART9_nRTS        (0x3UL<<SYS_GPJ_MFP1_PJ7MFP_Pos)    /*!< GPJ_MFP1 PJ7 setting for UART9_nRTS         */
#define SYS_GPJ_MFP1_PJ7MFP_CCAP_DATA0        (0x7UL<<SYS_GPJ_MFP1_PJ7MFP_Pos)    /*!< GPJ_MFP1 PJ7 setting for CCAP_DATA0         */
#define SYS_GPJ_MFP1_PJ7MFP_SPIM0_SS          (0xCUL<<SYS_GPJ_MFP1_PJ7MFP_Pos)    /*!< GPJ_MFP1 PJ7 setting for SPIM0_SS           */

/* PJ.8 MFP */
#define SYS_GPJ_MFP2_PJ8MFP_GPIO              (0x0UL<<SYS_GPJ_MFP2_PJ8MFP_Pos)    /*!< GPJ_MFP2 PJ8 setting for GPIO               */
#define SYS_GPJ_MFP2_PJ8MFP_EBI_nRD           (0x2UL<<SYS_GPJ_MFP2_PJ8MFP_Pos)    /*!< GPJ_MFP2 PJ8 setting for EBI_nRD            */
#define SYS_GPJ_MFP2_PJ8MFP_SD1_DAT3          (0x3UL<<SYS_GPJ_MFP2_PJ8MFP_Pos)    /*!< GPJ_MFP2 PJ8 setting for SD1_DAT3           */
#define SYS_GPJ_MFP2_PJ8MFP_UART7_TXD         (0x6UL<<SYS_GPJ_MFP2_PJ8MFP_Pos)    /*!< GPJ_MFP2 PJ8 setting for UART7_TXD          */
#define SYS_GPJ_MFP2_PJ8MFP_BPWM0_CH5         (0xCUL<<SYS_GPJ_MFP2_PJ8MFP_Pos)    /*!< GPJ_MFP2 PJ8 setting for BPWM0_CH5          */

/* PJ.9 MFP */
#define SYS_GPJ_MFP2_PJ9MFP_GPIO              (0x0UL<<SYS_GPJ_MFP2_PJ9MFP_Pos)    /*!< GPJ_MFP2 PJ9 setting for GPIO               */
#define SYS_GPJ_MFP2_PJ9MFP_EBI_nWR           (0x2UL<<SYS_GPJ_MFP2_PJ9MFP_Pos)    /*!< GPJ_MFP2 PJ9 setting for EBI_nWR            */
#define SYS_GPJ_MFP2_PJ9MFP_SD1_DAT2          (0x3UL<<SYS_GPJ_MFP2_PJ9MFP_Pos)    /*!< GPJ_MFP2 PJ9 setting for SD1_DAT2           */
#define SYS_GPJ_MFP2_PJ9MFP_UART7_RXD         (0x6UL<<SYS_GPJ_MFP2_PJ9MFP_Pos)    /*!< GPJ_MFP2 PJ9 setting for UART7_RXD          */
#define SYS_GPJ_MFP2_PJ9MFP_BPWM0_CH4         (0xCUL<<SYS_GPJ_MFP2_PJ9MFP_Pos)    /*!< GPJ_MFP2 PJ9 setting for BPWM0_CH4          */

/* PJ.10 MFP */
#define SYS_GPJ_MFP2_PJ10MFP_GPIO             (0x0UL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for GPIO              */
#define SYS_GPJ_MFP2_PJ10MFP_EBI_MCLK         (0x2UL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for EBI_MCLK          */
#define SYS_GPJ_MFP2_PJ10MFP_SD1_DAT1         (0x3UL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for SD1_DAT1          */
#define SYS_GPJ_MFP2_PJ10MFP_UART6_TXD        (0x6UL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for UART6_TXD         */
#define SYS_GPJ_MFP2_PJ10MFP_ECAP2_IC0        (0x9UL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for ECAP2_IC0         */
#define SYS_GPJ_MFP2_PJ10MFP_CANFD0_TXD       (0xBUL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for CANFD0_TXD        */
#define SYS_GPJ_MFP2_PJ10MFP_BPWM0_CH3        (0xCUL<<SYS_GPJ_MFP2_PJ10MFP_Pos)   /*!< GPJ_MFP2 PJ10 setting for BPWM0_CH3         */

/* PJ.11 MFP */
#define SYS_GPJ_MFP2_PJ11MFP_GPIO             (0x0UL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for GPIO              */
#define SYS_GPJ_MFP2_PJ11MFP_EBI_ALE          (0x2UL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for EBI_ALE           */
#define SYS_GPJ_MFP2_PJ11MFP_SD1_DAT0         (0x3UL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for SD1_DAT0          */
#define SYS_GPJ_MFP2_PJ11MFP_UART6_RXD        (0x6UL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for UART6_RXD         */
#define SYS_GPJ_MFP2_PJ11MFP_ECAP2_IC1        (0x9UL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for ECAP2_IC1         */
#define SYS_GPJ_MFP2_PJ11MFP_CANFD0_RXD       (0xBUL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for CANFD0_RXD        */
#define SYS_GPJ_MFP2_PJ11MFP_BPWM0_CH2        (0xCUL<<SYS_GPJ_MFP2_PJ11MFP_Pos)   /*!< GPJ_MFP2 PJ11 setting for BPWM0_CH2         */

/* PJ.12 MFP */
#define SYS_GPJ_MFP3_PJ12MFP_GPIO             (0x0UL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for GPIO              */
#define SYS_GPJ_MFP3_PJ12MFP_EBI_nCS0         (0x2UL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for EBI_nCS0          */
#define SYS_GPJ_MFP3_PJ12MFP_SD1_CMD          (0x3UL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for SD1_CMD           */
#define SYS_GPJ_MFP3_PJ12MFP_ECAP2_IC2        (0x9UL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for ECAP2_IC2         */
#define SYS_GPJ_MFP3_PJ12MFP_CANFD1_TXD       (0xBUL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for CANFD1_TXD        */
#define SYS_GPJ_MFP3_PJ12MFP_BPWM0_CH1        (0xCUL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for BPWM0_CH1         */
#define SYS_GPJ_MFP3_PJ12MFP_HSUSB_VBUS_ST    (0xFUL<<SYS_GPJ_MFP3_PJ12MFP_Pos)   /*!< GPJ_MFP3 PJ12 setting for HSUSB_VBUS_ST     */

/* PJ.13 MFP */
#define SYS_GPJ_MFP3_PJ13MFP_GPIO             (0x0UL<<SYS_GPJ_MFP3_PJ13MFP_Pos)   /*!< GPJ_MFP3 PJ13 setting for GPIO              */
#define SYS_GPJ_MFP3_PJ13MFP_SD1_CLK          (0x3UL<<SYS_GPJ_MFP3_PJ13MFP_Pos)   /*!< GPJ_MFP3 PJ13 setting for SD1_CLK           */
#define SYS_GPJ_MFP3_PJ13MFP_CANFD1_RXD       (0xBUL<<SYS_GPJ_MFP3_PJ13MFP_Pos)   /*!< GPJ_MFP3 PJ13 setting for CANFD1_RXD        */
#define SYS_GPJ_MFP3_PJ13MFP_BPWM0_CH0        (0xCUL<<SYS_GPJ_MFP3_PJ13MFP_Pos)   /*!< GPJ_MFP3 PJ13 setting for BPWM0_CH0         */
#define SYS_GPJ_MFP3_PJ13MFP_HSUSB_VBUS_EN    (0xFUL<<SYS_GPJ_MFP3_PJ13MFP_Pos)   /*!< GPJ_MFP3 PJ13 setting for HSUSB_VBUS_EN     */

#define SET_ACMP0_N_PB3()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_ACMP0_N)                       /*!< Set PB3 function to ACMP0_N             */
#define SET_ACMP0_O_PB7()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_ACMP0_O)                       /*!< Set PB7 function to ACMP0_O             */
#define SET_ACMP0_O_PC1()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_ACMP0_O)                       /*!< Set PC1 function to ACMP0_O             */
#define SET_ACMP0_O_PC12()          SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_ACMP0_O)                     /*!< Set PC12 function to ACMP0_O            */
#define SET_ACMP0_O_PF0()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_ACMP0_O)                       /*!< Set PF0 function to ACMP0_O             */
#define SET_ACMP0_O_PG14()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_ACMP0_O)                     /*!< Set PG14 function to ACMP0_O            */
#define SET_ACMP0_P0_PA11()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_ACMP0_P0)                    /*!< Set PA11 function to ACMP0_P0           */
#define SET_ACMP0_P1_PB2()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_ACMP0_P1)                      /*!< Set PB2 function to ACMP0_P1            */
#define SET_ACMP0_P2_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_ACMP0_P2)                    /*!< Set PB12 function to ACMP0_P2           */
#define SET_ACMP0_P3_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_ACMP0_P3)                    /*!< Set PB13 function to ACMP0_P3           */
#define SET_ACMP0_WLAT_PA7()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_ACMP0_WLAT)                    /*!< Set PA7 function to ACMP0_WLAT          */
#define SET_ACMP1_N_PB5()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_ACMP1_N)                       /*!< Set PB5 function to ACMP1_N             */
#define SET_ACMP1_O_PB6()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_ACMP1_O)                       /*!< Set PB6 function to ACMP1_O             */
#define SET_ACMP1_O_PC0()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_ACMP1_O)                       /*!< Set PC0 function to ACMP1_O             */
#define SET_ACMP1_O_PC11()          SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_ACMP1_O)                     /*!< Set PC11 function to ACMP1_O            */
#define SET_ACMP1_O_PF1()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_ACMP1_O)                       /*!< Set PF1 function to ACMP1_O             */
#define SET_ACMP1_O_PG15()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_ACMP1_O)                     /*!< Set PG15 function to ACMP1_O            */
#define SET_ACMP1_P0_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_ACMP1_P0)                    /*!< Set PA10 function to ACMP1_P0           */
#define SET_ACMP1_P1_PB4()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_ACMP1_P1)                      /*!< Set PB4 function to ACMP1_P1            */
#define SET_ACMP1_P2_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_ACMP1_P2)                    /*!< Set PB12 function to ACMP1_P2           */
#define SET_ACMP1_P3_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_ACMP1_P3)                    /*!< Set PB13 function to ACMP1_P3           */
#define SET_ACMP1_WLAT_PA6()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_ACMP1_WLAT)                    /*!< Set PA6 function to ACMP1_WLAT          */
#define SET_ACMP2_N_PB6()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_ACMP2_N)                       /*!< Set PB6 function to ACMP2_N             */
#define SET_ACMP2_O_PB1()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_ACMP2_O)                       /*!< Set PB1 function to ACMP2_O             */
#define SET_ACMP2_O_PE7()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_ACMP2_O)                       /*!< Set PE7 function to ACMP2_O             */
#define SET_ACMP2_O_PF3()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_ACMP2_O)                       /*!< Set PF3 function to ACMP2_O             */
#define SET_ACMP2_P0_PB7()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_ACMP2_P0)                      /*!< Set PB7 function to ACMP2_P0            */
#define SET_ACMP2_P1_PB8()          SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_ACMP2_P1)                      /*!< Set PB8 function to ACMP2_P1            */
#define SET_ACMP2_P2_PB9()          SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_ACMP2_P2)                      /*!< Set PB9 function to ACMP2_P2            */
#define SET_ACMP2_P3_PB10()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_ACMP2_P3)                    /*!< Set PB10 function to ACMP2_P3           */
#define SET_ACMP2_WLAT_PC7()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_ACMP2_WLAT)                    /*!< Set PC7 function to ACMP2_WLAT          */
#define SET_ACMP3_N_PB0()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_ACMP3_N)                       /*!< Set PB0 function to ACMP3_N             */
#define SET_ACMP3_O_PB0()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_ACMP3_O)                       /*!< Set PB0 function to ACMP3_O             */
#define SET_ACMP3_O_PE6()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_ACMP3_O)                       /*!< Set PE6 function to ACMP3_O             */
#define SET_ACMP3_O_PF2()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_ACMP3_O)                       /*!< Set PF2 function to ACMP3_O             */
#define SET_ACMP3_P0_PB1()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_ACMP3_P0)                      /*!< Set PB1 function to ACMP3_P0            */
#define SET_ACMP3_P1_PC9()          SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_ACMP3_P1)                      /*!< Set PC9 function to ACMP3_P1            */
#define SET_ACMP3_P2_PC10()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_ACMP3_P2)                    /*!< Set PC10 function to ACMP3_P2           */
#define SET_ACMP3_P3_PC11()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_ACMP3_P3)                    /*!< Set PC11 function to ACMP3_P3           */
#define SET_ACMP3_WLAT_PC6()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_ACMP3_WLAT)                    /*!< Set PC6 function to ACMP3_WLAT          */
#define SET_BPWM0_CH0_PA0()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_BPWM0_CH0)                     /*!< Set PA0 function to BPWM0_CH0           */
#define SET_BPWM0_CH0_PA11()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_BPWM0_CH0)                   /*!< Set PA11 function to BPWM0_CH0          */
#define SET_BPWM0_CH0_PD13()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_BPWM0_CH0)                   /*!< Set PD13 function to BPWM0_CH0          */
#define SET_BPWM0_CH0_PD6()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_BPWM0_CH0)                     /*!< Set PD6 function to BPWM0_CH0           */
#define SET_BPWM0_CH0_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_BPWM0_CH0)                     /*!< Set PE2 function to BPWM0_CH0           */
#define SET_BPWM0_CH0_PJ13()        SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ13MFP_Msk)) | SYS_GPJ_MFP3_PJ13MFP_BPWM0_CH0)                   /*!< Set PJ13 function to BPWM0_CH0          */
#define SET_BPWM0_CH1_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_BPWM0_CH1)                     /*!< Set PA1 function to BPWM0_CH1           */
#define SET_BPWM0_CH1_PA10()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_BPWM0_CH1)                   /*!< Set PA10 function to BPWM0_CH1          */
#define SET_BPWM0_CH1_PD7()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_BPWM0_CH1)                     /*!< Set PD7 function to BPWM0_CH1           */
#define SET_BPWM0_CH1_PE3()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_BPWM0_CH1)                     /*!< Set PE3 function to BPWM0_CH1           */
#define SET_BPWM0_CH1_PJ12()        SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_BPWM0_CH1)                   /*!< Set PJ12 function to BPWM0_CH1          */
#define SET_BPWM0_CH2_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_BPWM0_CH2)                     /*!< Set PA2 function to BPWM0_CH2           */
#define SET_BPWM0_CH2_PA9()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_BPWM0_CH2)                     /*!< Set PA9 function to BPWM0_CH2           */
#define SET_BPWM0_CH2_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_BPWM0_CH2)                     /*!< Set PE4 function to BPWM0_CH2           */
#define SET_BPWM0_CH2_PG12()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_BPWM0_CH2)                   /*!< Set PG12 function to BPWM0_CH2          */
#define SET_BPWM0_CH2_PJ11()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_BPWM0_CH2)                   /*!< Set PJ11 function to BPWM0_CH2          */
#define SET_BPWM0_CH3_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_BPWM0_CH3)                     /*!< Set PA3 function to BPWM0_CH3           */
#define SET_BPWM0_CH3_PA8()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_BPWM0_CH3)                     /*!< Set PA8 function to BPWM0_CH3           */
#define SET_BPWM0_CH3_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_BPWM0_CH3)                     /*!< Set PE5 function to BPWM0_CH3           */
#define SET_BPWM0_CH3_PG11()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_BPWM0_CH3)                   /*!< Set PG11 function to BPWM0_CH3          */
#define SET_BPWM0_CH3_PJ10()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_BPWM0_CH3)                   /*!< Set PJ10 function to BPWM0_CH3          */
#define SET_BPWM0_CH4_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_BPWM0_CH4)                     /*!< Set PA4 function to BPWM0_CH4           */
#define SET_BPWM0_CH4_PC13()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_BPWM0_CH4)                   /*!< Set PC13 function to BPWM0_CH4          */
#define SET_BPWM0_CH4_PE6()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_BPWM0_CH4)                     /*!< Set PE6 function to BPWM0_CH4           */
#define SET_BPWM0_CH4_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_BPWM0_CH4)                     /*!< Set PF5 function to BPWM0_CH4           */
#define SET_BPWM0_CH4_PG10()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_BPWM0_CH4)                   /*!< Set PG10 function to BPWM0_CH4          */
#define SET_BPWM0_CH4_PJ9()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ9MFP_Msk)) | SYS_GPJ_MFP2_PJ9MFP_BPWM0_CH4)                     /*!< Set PJ9 function to BPWM0_CH4           */
#define SET_BPWM0_CH5_PA5()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_BPWM0_CH5)                     /*!< Set PA5 function to BPWM0_CH5           */
#define SET_BPWM0_CH5_PD12()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_BPWM0_CH5)                   /*!< Set PD12 function to BPWM0_CH5          */
#define SET_BPWM0_CH5_PE7()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_BPWM0_CH5)                     /*!< Set PE7 function to BPWM0_CH5           */
#define SET_BPWM0_CH5_PF4()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_BPWM0_CH5)                     /*!< Set PF4 function to BPWM0_CH5           */
#define SET_BPWM0_CH5_PG9()         SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_BPWM0_CH5)                     /*!< Set PG9 function to BPWM0_CH5           */
#define SET_BPWM0_CH5_PJ8()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ8MFP_Msk)) | SYS_GPJ_MFP2_PJ8MFP_BPWM0_CH5)                     /*!< Set PJ8 function to BPWM0_CH5           */
#define SET_BPWM1_CH0_PB11()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_BPWM1_CH0)                   /*!< Set PB11 function to BPWM1_CH0          */
#define SET_BPWM1_CH0_PC7()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_BPWM1_CH0)                     /*!< Set PC7 function to BPWM1_CH0           */
#define SET_BPWM1_CH0_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_BPWM1_CH0)                     /*!< Set PF0 function to BPWM1_CH0           */
#define SET_BPWM1_CH0_PF3()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_BPWM1_CH0)                     /*!< Set PF3 function to BPWM1_CH0           */
#define SET_BPWM1_CH1_PB10()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_BPWM1_CH1)                   /*!< Set PB10 function to BPWM1_CH1          */
#define SET_BPWM1_CH1_PC6()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_BPWM1_CH1)                     /*!< Set PC6 function to BPWM1_CH1           */
#define SET_BPWM1_CH1_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_BPWM1_CH1)                     /*!< Set PF1 function to BPWM1_CH1           */
#define SET_BPWM1_CH1_PF2()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_BPWM1_CH1)                     /*!< Set PF2 function to BPWM1_CH1           */
#define SET_BPWM1_CH2_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_BPWM1_CH2)                   /*!< Set PA12 function to BPWM1_CH2          */
#define SET_BPWM1_CH2_PA7()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_BPWM1_CH2)                     /*!< Set PA7 function to BPWM1_CH2           */
#define SET_BPWM1_CH2_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_BPWM1_CH2)                     /*!< Set PB9 function to BPWM1_CH2           */
#define SET_BPWM1_CH3_PA13()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_BPWM1_CH3)                   /*!< Set PA13 function to BPWM1_CH3          */
#define SET_BPWM1_CH3_PA6()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_BPWM1_CH3)                     /*!< Set PA6 function to BPWM1_CH3           */
#define SET_BPWM1_CH3_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_BPWM1_CH3)                     /*!< Set PB8 function to BPWM1_CH3           */
#define SET_BPWM1_CH4_PA14()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_BPWM1_CH4)                   /*!< Set PA14 function to BPWM1_CH4          */
#define SET_BPWM1_CH4_PB7()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_BPWM1_CH4)                     /*!< Set PB7 function to BPWM1_CH4           */
#define SET_BPWM1_CH4_PC8()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_BPWM1_CH4)                     /*!< Set PC8 function to BPWM1_CH4           */
#define SET_BPWM1_CH5_PA15()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_BPWM1_CH5)                   /*!< Set PA15 function to BPWM1_CH5          */
#define SET_BPWM1_CH5_PB6()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_BPWM1_CH5)                     /*!< Set PB6 function to BPWM1_CH5           */
#define SET_BPWM1_CH5_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_BPWM1_CH5)                   /*!< Set PE13 function to BPWM1_CH5          */
#define SET_CANFD0_RXD_PA13()       SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_CANFD0_RXD)                  /*!< Set PA13 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PA4()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_CANFD0_RXD)                    /*!< Set PA4 function to CANFD0_RXD          */
#define SET_CANFD0_RXD_PB10()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_CANFD0_RXD)                  /*!< Set PB10 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PC4()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_CANFD0_RXD)                    /*!< Set PC4 function to CANFD0_RXD          */
#define SET_CANFD0_RXD_PD10()       SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_CANFD0_RXD)                  /*!< Set PD10 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PE15()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_CANFD0_RXD)                  /*!< Set PE15 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PI13()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_CANFD0_RXD)                  /*!< Set PI13 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PJ11()       SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_CANFD0_RXD)                  /*!< Set PJ11 function to CANFD0_RXD         */
#define SET_CANFD0_RXD_PJ3()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_CANFD0_RXD)                    /*!< Set PJ3 function to CANFD0_RXD          */
#define SET_CANFD0_TXD_PA12()       SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_CANFD0_TXD)                  /*!< Set PA12 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PA5()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_CANFD0_TXD)                    /*!< Set PA5 function to CANFD0_TXD          */
#define SET_CANFD0_TXD_PB11()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_CANFD0_TXD)                  /*!< Set PB11 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PC5()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_CANFD0_TXD)                    /*!< Set PC5 function to CANFD0_TXD          */
#define SET_CANFD0_TXD_PD11()       SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_CANFD0_TXD)                  /*!< Set PD11 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PE14()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_CANFD0_TXD)                  /*!< Set PE14 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PI12()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_CANFD0_TXD)                  /*!< Set PI12 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PJ10()       SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_CANFD0_TXD)                  /*!< Set PJ10 function to CANFD0_TXD         */
#define SET_CANFD0_TXD_PJ2()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_CANFD0_TXD)                    /*!< Set PJ2 function to CANFD0_TXD          */
#define SET_CANFD1_RXD_PB6()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_CANFD1_RXD)                    /*!< Set PB6 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PC2()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_CANFD1_RXD)                    /*!< Set PC2 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PC9()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_CANFD1_RXD)                    /*!< Set PC9 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PD12()       SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_CANFD1_RXD)                  /*!< Set PD12 function to CANFD1_RXD         */
#define SET_CANFD1_RXD_PE6()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_CANFD1_RXD)                    /*!< Set PE6 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PF8()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_CANFD1_RXD)                    /*!< Set PF8 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PG1()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_CANFD1_RXD)                    /*!< Set PG1 function to CANFD1_RXD          */
#define SET_CANFD1_RXD_PI15()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_CANFD1_RXD)                  /*!< Set PI15 function to CANFD1_RXD         */
#define SET_CANFD1_RXD_PJ13()       SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ13MFP_Msk)) | SYS_GPJ_MFP3_PJ13MFP_CANFD1_RXD)                  /*!< Set PJ13 function to CANFD1_RXD         */
#define SET_CANFD1_RXD_PJ5()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_CANFD1_RXD)                    /*!< Set PJ5 function to CANFD1_RXD          */
#define SET_CANFD1_TXD_PB7()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_CANFD1_TXD)                    /*!< Set PB7 function to CANFD1_TXD          */
#define SET_CANFD1_TXD_PC10()       SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_CANFD1_TXD)                  /*!< Set PC10 function to CANFD1_TXD         */
#define SET_CANFD1_TXD_PC13()       SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_CANFD1_TXD)                  /*!< Set PC13 function to CANFD1_TXD         */
#define SET_CANFD1_TXD_PC3()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_CANFD1_TXD)                    /*!< Set PC3 function to CANFD1_TXD          */
#define SET_CANFD1_TXD_PE7()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_CANFD1_TXD)                    /*!< Set PE7 function to CANFD1_TXD          */
#define SET_CANFD1_TXD_PF9()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_CANFD1_TXD)                    /*!< Set PF9 function to CANFD1_TXD          */
#define SET_CANFD1_TXD_PG0()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_CANFD1_TXD)                    /*!< Set PG0 function to CANFD1_TXD          */
#define SET_CANFD1_TXD_PI14()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_CANFD1_TXD)                  /*!< Set PI14 function to CANFD1_TXD         */
#define SET_CANFD1_TXD_PJ12()       SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_CANFD1_TXD)                  /*!< Set PJ12 function to CANFD1_TXD         */
#define SET_CANFD1_TXD_PJ4()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_CANFD1_TXD)                    /*!< Set PJ4 function to CANFD1_TXD          */
#define SET_CCAP_DATA0_PB14()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_CCAP_DATA0)                  /*!< Set PB14 function to CCAP_DATA0         */
#define SET_CCAP_DATA0_PB7()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_CCAP_DATA0)                    /*!< Set PB7 function to CCAP_DATA0          */
#define SET_CCAP_DATA0_PC0()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_CCAP_DATA0)                    /*!< Set PC0 function to CCAP_DATA0          */
#define SET_CCAP_DATA0_PF7()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_CCAP_DATA0)                    /*!< Set PF7 function to CCAP_DATA0          */
#define SET_CCAP_DATA0_PJ7()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ7MFP_Msk)) | SYS_GPJ_MFP1_PJ7MFP_CCAP_DATA0)                    /*!< Set PJ7 function to CCAP_DATA0          */
#define SET_CCAP_DATA1_PB15()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_CCAP_DATA1)                  /*!< Set PB15 function to CCAP_DATA1         */
#define SET_CCAP_DATA1_PB8()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_CCAP_DATA1)                    /*!< Set PB8 function to CCAP_DATA1          */
#define SET_CCAP_DATA1_PC1()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_CCAP_DATA1)                    /*!< Set PC1 function to CCAP_DATA1          */
#define SET_CCAP_DATA1_PF8()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_CCAP_DATA1)                    /*!< Set PF8 function to CCAP_DATA1          */
#define SET_CCAP_DATA1_PJ6()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ6MFP_Msk)) | SYS_GPJ_MFP1_PJ6MFP_CCAP_DATA1)                    /*!< Set PJ6 function to CCAP_DATA1          */
#define SET_CCAP_DATA2_PA2()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_CCAP_DATA2)                    /*!< Set PA2 function to CCAP_DATA2          */
#define SET_CCAP_DATA2_PC2()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_CCAP_DATA2)                    /*!< Set PC2 function to CCAP_DATA2          */
#define SET_CCAP_DATA2_PF9()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_CCAP_DATA2)                    /*!< Set PF9 function to CCAP_DATA2          */
#define SET_CCAP_DATA2_PJ5()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_CCAP_DATA2)                    /*!< Set PJ5 function to CCAP_DATA2          */
#define SET_CCAP_DATA3_PA3()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_CCAP_DATA3)                    /*!< Set PA3 function to CCAP_DATA3          */
#define SET_CCAP_DATA3_PC3()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_CCAP_DATA3)                    /*!< Set PC3 function to CCAP_DATA3          */
#define SET_CCAP_DATA3_PF10()       SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_CCAP_DATA3)                  /*!< Set PF10 function to CCAP_DATA3         */
#define SET_CCAP_DATA3_PJ4()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_CCAP_DATA3)                    /*!< Set PJ4 function to CCAP_DATA3          */
#define SET_CCAP_DATA4_PC4()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_CCAP_DATA4)                    /*!< Set PC4 function to CCAP_DATA4          */
#define SET_CCAP_DATA4_PF11()       SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_CCAP_DATA4)                  /*!< Set PF11 function to CCAP_DATA4         */
#define SET_CCAP_DATA4_PJ3()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_CCAP_DATA4)                    /*!< Set PJ3 function to CCAP_DATA4          */
#define SET_CCAP_DATA5_PC5()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_CCAP_DATA5)                    /*!< Set PC5 function to CCAP_DATA5          */
#define SET_CCAP_DATA5_PG4()        SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG4MFP_Msk)) | SYS_GPG_MFP1_PG4MFP_CCAP_DATA5)                    /*!< Set PG4 function to CCAP_DATA5          */
#define SET_CCAP_DATA5_PJ2()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_CCAP_DATA5)                    /*!< Set PJ2 function to CCAP_DATA5          */
#define SET_CCAP_DATA6_PA0()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_CCAP_DATA6)                    /*!< Set PA0 function to CCAP_DATA6          */
#define SET_CCAP_DATA6_PG3()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_CCAP_DATA6)                    /*!< Set PG3 function to CCAP_DATA6          */
#define SET_CCAP_DATA7_PA1()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_CCAP_DATA7)                    /*!< Set PA1 function to CCAP_DATA7          */
#define SET_CCAP_DATA7_PG2()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_CCAP_DATA7)                    /*!< Set PG2 function to CCAP_DATA7          */
#define SET_CCAP_HSYNC_PB9()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_CCAP_HSYNC)                    /*!< Set PB9 function to CCAP_HSYNC          */
#define SET_CCAP_HSYNC_PD7()        SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_CCAP_HSYNC)                    /*!< Set PD7 function to CCAP_HSYNC          */
#define SET_CCAP_HSYNC_PG13()       SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_CCAP_HSYNC)                  /*!< Set PG13 function to CCAP_HSYNC         */
#define SET_CCAP_PIXCLK_PA5()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_CCAP_PIXCLK)                   /*!< Set PA5 function to CCAP_PIXCLK         */
#define SET_CCAP_PIXCLK_PB13()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_CCAP_PIXCLK)                 /*!< Set PB13 function to CCAP_PIXCLK        */
#define SET_CCAP_PIXCLK_PG0()       SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_CCAP_PIXCLK)                   /*!< Set PG0 function to CCAP_PIXCLK         */
#define SET_CCAP_PIXCLK_PG9()       SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_CCAP_PIXCLK)                   /*!< Set PG9 function to CCAP_PIXCLK         */
#define SET_CCAP_PIXCLK_PH12()      SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_CCAP_PIXCLK)                 /*!< Set PH12 function to CCAP_PIXCLK        */
#define SET_CCAP_SCLK_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_CCAP_SCLK)                     /*!< Set PA4 function to CCAP_SCLK           */
#define SET_CCAP_SCLK_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_CCAP_SCLK)                   /*!< Set PB12 function to CCAP_SCLK          */
#define SET_CCAP_SCLK_PG1()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_CCAP_SCLK)                     /*!< Set PG1 function to CCAP_SCLK           */
#define SET_CCAP_SCLK_PG10()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_CCAP_SCLK)                   /*!< Set PG10 function to CCAP_SCLK          */
#define SET_CCAP_SCLK_PH13()        SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_CCAP_SCLK)                   /*!< Set PH13 function to CCAP_SCLK          */
#define SET_CCAP_SFIELD_PB11()      SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_CCAP_SFIELD)                 /*!< Set PB11 function to CCAP_SFIELD        */
#define SET_CCAP_SFIELD_PG11()      SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_CCAP_SFIELD)                 /*!< Set PG11 function to CCAP_SFIELD        */
#define SET_CCAP_SFIELD_PH14()      SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH14MFP_Msk)) | SYS_GPH_MFP3_PH14MFP_CCAP_SFIELD)                 /*!< Set PH14 function to CCAP_SFIELD        */
#define SET_CCAP_VSYNC_PB10()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_CCAP_VSYNC)                  /*!< Set PB10 function to CCAP_VSYNC         */
#define SET_CCAP_VSYNC_PG12()       SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_CCAP_VSYNC)                  /*!< Set PG12 function to CCAP_VSYNC         */
#define SET_CCAP_VSYNC_PH15()       SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH15MFP_Msk)) | SYS_GPH_MFP3_PH15MFP_CCAP_VSYNC)                  /*!< Set PH15 function to CCAP_VSYNC         */
#define SET_CLKO_PB13()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_CLKO)                        /*!< Set PB13 function to CLKO               */
#define SET_CLKO_PB14()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_CLKO)                        /*!< Set PB14 function to CLKO               */
#define SET_CLKO_PC13()             SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_CLKO)                        /*!< Set PC13 function to CLKO               */
#define SET_CLKO_PD12()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_CLKO)                        /*!< Set PD12 function to CLKO               */
#define SET_CLKO_PD13()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_CLKO)                        /*!< Set PD13 function to CLKO               */
#define SET_CLKO_PD5()              SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_CLKO)                          /*!< Set PD5 function to CLKO                */
#define SET_DAC0_OUT_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_DAC0_OUT)                    /*!< Set PB12 function to DAC0_OUT           */
#define SET_DAC0_ST_PA0()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_DAC0_ST)                       /*!< Set PA0 function to DAC0_ST             */
#define SET_DAC0_ST_PA10()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_DAC0_ST)                     /*!< Set PA10 function to DAC0_ST            */
#define SET_DAC1_OUT_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_DAC1_OUT)                    /*!< Set PB13 function to DAC1_OUT           */
#define SET_DAC1_ST_PA1()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_DAC1_ST)                       /*!< Set PA1 function to DAC1_ST             */
#define SET_DAC1_ST_PA11()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_DAC1_ST)                     /*!< Set PA11 function to DAC1_ST            */
#define SET_DMIC0_CLKLP_PA3()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_DMIC0_CLKLP)                   /*!< Set PA3 function to DMIC0_CLKLP         */
#define SET_DMIC0_CLKLP_PB6()       SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_DMIC0_CLKLP)                   /*!< Set PB6 function to DMIC0_CLKLP         */
#define SET_DMIC0_CLKLP_PE10()      SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_DMIC0_CLKLP)                 /*!< Set PE10 function to DMIC0_CLKLP        */
#define SET_DMIC0_CLK_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_DMIC0_CLK)                     /*!< Set PA4 function to DMIC0_CLK           */
#define SET_DMIC0_CLK_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_DMIC0_CLK)                     /*!< Set PB4 function to DMIC0_CLK           */
#define SET_DMIC0_CLK_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_DMIC0_CLK)                     /*!< Set PE9 function to DMIC0_CLK           */
#define SET_DMIC0_DAT_PA5()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_DMIC0_DAT)                     /*!< Set PA5 function to DMIC0_DAT           */
#define SET_DMIC0_DAT_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_DMIC0_DAT)                     /*!< Set PB5 function to DMIC0_DAT           */
#define SET_DMIC0_DAT_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_DMIC0_DAT)                     /*!< Set PE8 function to DMIC0_DAT           */
#define SET_DMIC1_CLK_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_DMIC1_CLK)                     /*!< Set PA1 function to DMIC1_CLK           */
#define SET_DMIC1_CLK_PB2()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_DMIC1_CLK)                     /*!< Set PB2 function to DMIC1_CLK           */
#define SET_DMIC1_CLK_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_DMIC1_CLK)                   /*!< Set PE12 function to DMIC1_CLK          */
#define SET_DMIC1_DAT_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_DMIC1_DAT)                     /*!< Set PA2 function to DMIC1_DAT           */
#define SET_DMIC1_DAT_PB3()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_DMIC1_DAT)                     /*!< Set PB3 function to DMIC1_DAT           */
#define SET_DMIC1_DAT_PE11()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_DMIC1_DAT)                   /*!< Set PE11 function to DMIC1_DAT          */
#define SET_EADC0_CH0_PB0()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_EADC0_CH0)                     /*!< Set PB0 function to EADC0_CH0           */
#define SET_EADC0_CH10_PB10()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_EADC0_CH10)                  /*!< Set PB10 function to EADC0_CH10         */
#define SET_EADC0_CH11_PB11()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_EADC0_CH11)                  /*!< Set PB11 function to EADC0_CH11         */
#define SET_EADC0_CH12_PB12()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_EADC0_CH12)                  /*!< Set PB12 function to EADC0_CH12         */
#define SET_EADC0_CH13_PB13()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_EADC0_CH13)                  /*!< Set PB13 function to EADC0_CH13         */
#define SET_EADC0_CH14_PB14()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_EADC0_CH14)                  /*!< Set PB14 function to EADC0_CH14         */
#define SET_EADC0_CH15_PB15()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_EADC0_CH15)                  /*!< Set PB15 function to EADC0_CH15         */
#define SET_EADC0_CH16_PD10()       SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_EADC0_CH16)                  /*!< Set PD10 function to EADC0_CH16         */
#define SET_EADC0_CH17_PD11()       SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_EADC0_CH17)                  /*!< Set PD11 function to EADC0_CH17         */
#define SET_EADC0_CH18_PD12()       SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_EADC0_CH18)                  /*!< Set PD12 function to EADC0_CH18         */
#define SET_EADC0_CH19_PC13()       SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_EADC0_CH19)                  /*!< Set PC13 function to EADC0_CH19         */
#define SET_EADC0_CH1_PB1()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EADC0_CH1)                     /*!< Set PB1 function to EADC0_CH1           */
#define SET_EADC0_CH20_PA8()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_EADC0_CH20)                    /*!< Set PA8 function to EADC0_CH20          */
#define SET_EADC0_CH21_PA9()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_EADC0_CH21)                    /*!< Set PA9 function to EADC0_CH21          */
#define SET_EADC0_CH22_PA10()       SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_EADC0_CH22)                  /*!< Set PA10 function to EADC0_CH22         */
#define SET_EADC0_CH23_PA11()       SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_EADC0_CH23)                  /*!< Set PA11 function to EADC0_CH23         */
#define SET_EADC0_CH2_PB2()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_EADC0_CH2)                     /*!< Set PB2 function to EADC0_CH2           */
#define SET_EADC0_CH3_PB3()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_EADC0_CH3)                     /*!< Set PB3 function to EADC0_CH3           */
#define SET_EADC0_CH4_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_EADC0_CH4)                     /*!< Set PB4 function to EADC0_CH4           */
#define SET_EADC0_CH5_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_EADC0_CH5)                     /*!< Set PB5 function to EADC0_CH5           */
#define SET_EADC0_CH6_PB6()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EADC0_CH6)                     /*!< Set PB6 function to EADC0_CH6           */
#define SET_EADC0_CH7_PB7()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EADC0_CH7)                     /*!< Set PB7 function to EADC0_CH7           */
#define SET_EADC0_CH8_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_EADC0_CH8)                     /*!< Set PB8 function to EADC0_CH8           */
#define SET_EADC0_CH9_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_EADC0_CH9)                     /*!< Set PB9 function to EADC0_CH9           */
#define SET_EADC0_ST_PC1()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_EADC0_ST)                      /*!< Set PC1 function to EADC0_ST            */
#define SET_EADC0_ST_PC13()         SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_EADC0_ST)                    /*!< Set PC13 function to EADC0_ST           */
#define SET_EADC0_ST_PD12()         SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_EADC0_ST)                    /*!< Set PD12 function to EADC0_ST           */
#define SET_EADC0_ST_PD13()         SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_EADC0_ST)                    /*!< Set PD13 function to EADC0_ST           */
#define SET_EADC0_ST_PD5()          SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_EADC0_ST)                      /*!< Set PD5 function to EADC0_ST            */
#define SET_EADC0_ST_PF0()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_EADC0_ST)                      /*!< Set PF0 function to EADC0_ST            */
#define SET_EADC0_ST_PF5()          SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_EADC0_ST)                      /*!< Set PF5 function to EADC0_ST            */
#define SET_EADC0_ST_PG14()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_EADC0_ST)                    /*!< Set PG14 function to EADC0_ST           */
#define SET_EBI_AD0_PA5()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EBI_AD0)                       /*!< Set PA5 function to EBI_AD0             */
#define SET_EBI_AD0_PC0()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_EBI_AD0)                       /*!< Set PC0 function to EBI_AD0             */
#define SET_EBI_AD0_PF4()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_EBI_AD0)                       /*!< Set PF4 function to EBI_AD0             */
#define SET_EBI_AD0_PG9()           SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_EBI_AD0)                       /*!< Set PG9 function to EBI_AD0             */
#define SET_EBI_AD0_PH12()          SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_EBI_AD0)                     /*!< Set PH12 function to EBI_AD0            */
#define SET_EBI_AD0_PJ7()           SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ7MFP_Msk)) | SYS_GPJ_MFP1_PJ7MFP_EBI_AD0)                       /*!< Set PJ7 function to EBI_AD0             */
#define SET_EBI_AD10_PD13()         SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_EBI_AD10)                    /*!< Set PD13 function to EBI_AD10           */
#define SET_EBI_AD10_PD3()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_EBI_AD10)                      /*!< Set PD3 function to EBI_AD10            */
#define SET_EBI_AD10_PE1()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_EBI_AD10)                      /*!< Set PE1 function to EBI_AD10            */
#define SET_EBI_AD10_PE7()          SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_EBI_AD10)                      /*!< Set PE7 function to EBI_AD10            */
#define SET_EBI_AD11_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_EBI_AD11)                    /*!< Set PB12 function to EBI_AD11           */
#define SET_EBI_AD11_PD2()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_EBI_AD11)                      /*!< Set PD2 function to EBI_AD11            */
#define SET_EBI_AD11_PE0()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_EBI_AD11)                      /*!< Set PE0 function to EBI_AD11            */
#define SET_EBI_AD11_PE6()          SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_EBI_AD11)                      /*!< Set PE6 function to EBI_AD11            */
#define SET_EBI_AD12_PB15()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_EBI_AD12)                    /*!< Set PB15 function to EBI_AD12           */
#define SET_EBI_AD12_PD1()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_EBI_AD12)                      /*!< Set PD1 function to EBI_AD12            */
#define SET_EBI_AD12_PH8()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_EBI_AD12)                      /*!< Set PH8 function to EBI_AD12            */
#define SET_EBI_AD13_PB14()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_EBI_AD13)                    /*!< Set PB14 function to EBI_AD13           */
#define SET_EBI_AD13_PD0()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_EBI_AD13)                      /*!< Set PD0 function to EBI_AD13            */
#define SET_EBI_AD13_PH9()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_EBI_AD13)                      /*!< Set PH9 function to EBI_AD13            */
#define SET_EBI_AD14_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_EBI_AD14)                    /*!< Set PB13 function to EBI_AD14           */
#define SET_EBI_AD14_PH10()         SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_EBI_AD14)                    /*!< Set PH10 function to EBI_AD14           */
#define SET_EBI_AD15_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_EBI_AD15)                    /*!< Set PB12 function to EBI_AD15           */
#define SET_EBI_AD15_PD5()          SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_EBI_AD15)                      /*!< Set PD5 function to EBI_AD15            */
#define SET_EBI_AD15_PH11()         SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_EBI_AD15)                    /*!< Set PH11 function to EBI_AD15           */
#define SET_EBI_AD1_PA4()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_EBI_AD1)                       /*!< Set PA4 function to EBI_AD1             */
#define SET_EBI_AD1_PC1()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_EBI_AD1)                       /*!< Set PC1 function to EBI_AD1             */
#define SET_EBI_AD1_PF5()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_EBI_AD1)                       /*!< Set PF5 function to EBI_AD1             */
#define SET_EBI_AD1_PG10()          SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_EBI_AD1)                     /*!< Set PG10 function to EBI_AD1            */
#define SET_EBI_AD1_PH13()          SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_EBI_AD1)                     /*!< Set PH13 function to EBI_AD1            */
#define SET_EBI_AD1_PJ6()           SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ6MFP_Msk)) | SYS_GPJ_MFP1_PJ6MFP_EBI_AD1)                       /*!< Set PJ6 function to EBI_AD1             */
#define SET_EBI_AD2_PC2()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_EBI_AD2)                       /*!< Set PC2 function to EBI_AD2             */
#define SET_EBI_AD2_PG11()          SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_EBI_AD2)                     /*!< Set PG11 function to EBI_AD2            */
#define SET_EBI_AD2_PH14()          SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH14MFP_Msk)) | SYS_GPH_MFP3_PH14MFP_EBI_AD2)                     /*!< Set PH14 function to EBI_AD2            */
#define SET_EBI_AD2_PJ5()           SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_EBI_AD2)                       /*!< Set PJ5 function to EBI_AD2             */
#define SET_EBI_AD3_PC3()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_EBI_AD3)                       /*!< Set PC3 function to EBI_AD3             */
#define SET_EBI_AD3_PG12()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_EBI_AD3)                     /*!< Set PG12 function to EBI_AD3            */
#define SET_EBI_AD3_PH15()          SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH15MFP_Msk)) | SYS_GPH_MFP3_PH15MFP_EBI_AD3)                     /*!< Set PH15 function to EBI_AD3            */
#define SET_EBI_AD3_PJ4()           SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_EBI_AD3)                       /*!< Set PJ4 function to EBI_AD3             */
#define SET_EBI_AD4_PC4()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_EBI_AD4)                       /*!< Set PC4 function to EBI_AD4             */
#define SET_EBI_AD4_PD7()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_EBI_AD4)                       /*!< Set PD7 function to EBI_AD4             */
#define SET_EBI_AD4_PG13()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_EBI_AD4)                     /*!< Set PG13 function to EBI_AD4            */
#define SET_EBI_AD4_PJ3()           SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_EBI_AD4)                       /*!< Set PJ3 function to EBI_AD4             */
#define SET_EBI_AD5_PA14()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_EBI_AD5)                     /*!< Set PA14 function to EBI_AD5            */
#define SET_EBI_AD5_PC5()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_EBI_AD5)                       /*!< Set PC5 function to EBI_AD5             */
#define SET_EBI_AD5_PD6()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_EBI_AD5)                       /*!< Set PD6 function to EBI_AD5             */
#define SET_EBI_AD5_PG14()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_EBI_AD5)                     /*!< Set PG14 function to EBI_AD5            */
#define SET_EBI_AD5_PJ2()           SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_EBI_AD5)                       /*!< Set PJ2 function to EBI_AD5             */
#define SET_EBI_AD6_PD8()           SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_EBI_AD6)                       /*!< Set PD8 function to EBI_AD6             */
#define SET_EBI_AD7_PA7()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_EBI_AD7)                       /*!< Set PA7 function to EBI_AD7             */
#define SET_EBI_AD7_PD9()           SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_EBI_AD7)                       /*!< Set PD9 function to EBI_AD7             */
#define SET_EBI_AD8_PC6()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_EBI_AD8)                       /*!< Set PC6 function to EBI_AD8             */
#define SET_EBI_AD8_PE14()          SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_EBI_AD8)                     /*!< Set PE14 function to EBI_AD8            */
#define SET_EBI_AD9_PC7()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_EBI_AD9)                       /*!< Set PC7 function to EBI_AD9             */
#define SET_EBI_AD9_PE15()          SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_EBI_AD9)                     /*!< Set PE15 function to EBI_AD9            */
#define SET_EBI_ADR0_PB5()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_EBI_ADR0)                      /*!< Set PB5 function to EBI_ADR0            */
#define SET_EBI_ADR0_PH7()          SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_EBI_ADR0)                      /*!< Set PH7 function to EBI_ADR0            */
#define SET_EBI_ADR10_PC13()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_EBI_ADR10)                   /*!< Set PC13 function to EBI_ADR10          */
#define SET_EBI_ADR10_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_EBI_ADR10)                     /*!< Set PE8 function to EBI_ADR10           */
#define SET_EBI_ADR11_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_EBI_ADR11)                     /*!< Set PE9 function to EBI_ADR11           */
#define SET_EBI_ADR11_PG2()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_EBI_ADR11)                     /*!< Set PG2 function to EBI_ADR11           */
#define SET_EBI_ADR12_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_EBI_ADR12)                   /*!< Set PE10 function to EBI_ADR12          */
#define SET_EBI_ADR12_PG3()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_EBI_ADR12)                     /*!< Set PG3 function to EBI_ADR12           */
#define SET_EBI_ADR13_PE11()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_EBI_ADR13)                   /*!< Set PE11 function to EBI_ADR13          */
#define SET_EBI_ADR13_PG4()         SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG4MFP_Msk)) | SYS_GPG_MFP1_PG4MFP_EBI_ADR13)                     /*!< Set PG4 function to EBI_ADR13           */
#define SET_EBI_ADR14_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_EBI_ADR14)                   /*!< Set PE12 function to EBI_ADR14          */
#define SET_EBI_ADR14_PF11()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_EBI_ADR14)                   /*!< Set PF11 function to EBI_ADR14          */
#define SET_EBI_ADR15_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_EBI_ADR15)                   /*!< Set PE13 function to EBI_ADR15          */
#define SET_EBI_ADR15_PF10()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_EBI_ADR15)                   /*!< Set PF10 function to EBI_ADR15          */
#define SET_EBI_ADR16_PB11()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_EBI_ADR16)                   /*!< Set PB11 function to EBI_ADR16          */
#define SET_EBI_ADR16_PC8()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_EBI_ADR16)                     /*!< Set PC8 function to EBI_ADR16           */
#define SET_EBI_ADR16_PF9()         SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_EBI_ADR16)                     /*!< Set PF9 function to EBI_ADR16           */
#define SET_EBI_ADR17_PB10()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_EBI_ADR17)                   /*!< Set PB10 function to EBI_ADR17          */
#define SET_EBI_ADR17_PF8()         SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_EBI_ADR17)                     /*!< Set PF8 function to EBI_ADR17           */
#define SET_EBI_ADR18_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_EBI_ADR18)                     /*!< Set PB9 function to EBI_ADR18           */
#define SET_EBI_ADR18_PF7()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_EBI_ADR18)                     /*!< Set PF7 function to EBI_ADR18           */
#define SET_EBI_ADR19_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_EBI_ADR19)                     /*!< Set PB8 function to EBI_ADR19           */
#define SET_EBI_ADR19_PF6()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_EBI_ADR19)                     /*!< Set PF6 function to EBI_ADR19           */
#define SET_EBI_ADR1_PB4()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_EBI_ADR1)                      /*!< Set PB4 function to EBI_ADR1            */
#define SET_EBI_ADR1_PH6()          SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH6MFP_Msk)) | SYS_GPH_MFP1_PH6MFP_EBI_ADR1)                      /*!< Set PH6 function to EBI_ADR1            */
#define SET_EBI_ADR2_PB3()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_EBI_ADR2)                      /*!< Set PB3 function to EBI_ADR2            */
#define SET_EBI_ADR2_PH5()          SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_EBI_ADR2)                      /*!< Set PH5 function to EBI_ADR2            */
#define SET_EBI_ADR3_PB2()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_EBI_ADR3)                      /*!< Set PB2 function to EBI_ADR3            */
#define SET_EBI_ADR3_PH4()          SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_EBI_ADR3)                      /*!< Set PH4 function to EBI_ADR3            */
#define SET_EBI_ADR4_PC12()         SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_EBI_ADR4)                    /*!< Set PC12 function to EBI_ADR4           */
#define SET_EBI_ADR4_PH3()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_EBI_ADR4)                      /*!< Set PH3 function to EBI_ADR4            */
#define SET_EBI_ADR5_PC11()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_EBI_ADR5)                    /*!< Set PC11 function to EBI_ADR5           */
#define SET_EBI_ADR5_PH2()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_EBI_ADR5)                      /*!< Set PH2 function to EBI_ADR5            */
#define SET_EBI_ADR6_PC10()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_EBI_ADR6)                    /*!< Set PC10 function to EBI_ADR6           */
#define SET_EBI_ADR6_PH1()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_EBI_ADR6)                      /*!< Set PH1 function to EBI_ADR6            */
#define SET_EBI_ADR7_PC9()          SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_EBI_ADR7)                      /*!< Set PC9 function to EBI_ADR7            */
#define SET_EBI_ADR7_PH0()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_EBI_ADR7)                      /*!< Set PH0 function to EBI_ADR7            */
#define SET_EBI_ADR8_PB1()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EBI_ADR8)                      /*!< Set PB1 function to EBI_ADR8            */
#define SET_EBI_ADR8_PG0()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_EBI_ADR8)                      /*!< Set PG0 function to EBI_ADR8            */
#define SET_EBI_ADR9_PB0()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_EBI_ADR9)                      /*!< Set PB0 function to EBI_ADR9            */
#define SET_EBI_ADR9_PG1()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_EBI_ADR9)                      /*!< Set PG1 function to EBI_ADR9            */
#define SET_EBI_ALE_PA8()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_EBI_ALE)                       /*!< Set PA8 function to EBI_ALE             */
#define SET_EBI_ALE_PE2()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_EBI_ALE)                       /*!< Set PE2 function to EBI_ALE             */
#define SET_EBI_ALE_PJ11()          SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_EBI_ALE)                     /*!< Set PJ11 function to EBI_ALE            */
#define SET_EBI_MCLK_PA9()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_EBI_MCLK)                      /*!< Set PA9 function to EBI_MCLK            */
#define SET_EBI_MCLK_PE3()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_EBI_MCLK)                      /*!< Set PE3 function to EBI_MCLK            */
#define SET_EBI_MCLK_PJ10()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_EBI_MCLK)                    /*!< Set PJ10 function to EBI_MCLK           */
#define SET_EBI_nCS0_PB7()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EBI_nCS0)                      /*!< Set PB7 function to EBI_nCS0            */
#define SET_EBI_nCS0_PD12()         SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_EBI_nCS0)                    /*!< Set PD12 function to EBI_nCS0           */
#define SET_EBI_nCS0_PD14()         SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_EBI_nCS0)                    /*!< Set PD14 function to EBI_nCS0           */
#define SET_EBI_nCS0_PD5()          SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_EBI_nCS0)                      /*!< Set PD5 function to EBI_nCS0            */
#define SET_EBI_nCS0_PF3()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_EBI_nCS0)                      /*!< Set PF3 function to EBI_nCS0            */
#define SET_EBI_nCS0_PF6()          SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_EBI_nCS0)                      /*!< Set PF6 function to EBI_nCS0            */
#define SET_EBI_nCS0_PJ12()         SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_EBI_nCS0)                    /*!< Set PJ12 function to EBI_nCS0           */
#define SET_EBI_nCS1_PB6()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EBI_nCS1)                      /*!< Set PB6 function to EBI_nCS1            */
#define SET_EBI_nCS1_PD11()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_EBI_nCS1)                    /*!< Set PD11 function to EBI_nCS1           */
#define SET_EBI_nCS1_PF2()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_EBI_nCS1)                      /*!< Set PF2 function to EBI_nCS1            */
#define SET_EBI_nCS1_PG5()          SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_EBI_nCS1)                      /*!< Set PG5 function to EBI_nCS1            */
#define SET_EBI_nCS2_PD10()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_EBI_nCS2)                    /*!< Set PD10 function to EBI_nCS2           */
#define SET_EBI_nCS2_PG6()          SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_EBI_nCS2)                      /*!< Set PG6 function to EBI_nCS2            */
#define SET_EBI_nRD_PA11()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_EBI_nRD)                     /*!< Set PA11 function to EBI_nRD            */
#define SET_EBI_nRD_PE5()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_EBI_nRD)                       /*!< Set PE5 function to EBI_nRD             */
#define SET_EBI_nRD_PJ8()           SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ8MFP_Msk)) | SYS_GPJ_MFP2_PJ8MFP_EBI_nRD)                       /*!< Set PJ8 function to EBI_nRD             */
#define SET_EBI_nWRH_PB6()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EBI_nWRH)                      /*!< Set PB6 function to EBI_nWRH            */
#define SET_EBI_nWRH_PG6()          SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_EBI_nWRH)                      /*!< Set PG6 function to EBI_nWRH            */
#define SET_EBI_nWRL_PB7()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EBI_nWRL)                      /*!< Set PB7 function to EBI_nWRL            */
#define SET_EBI_nWRL_PG5()          SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_EBI_nWRL)                      /*!< Set PG5 function to EBI_nWRL            */
#define SET_EBI_nWR_PA10()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_EBI_nWR)                     /*!< Set PA10 function to EBI_nWR            */
#define SET_EBI_nWR_PE4()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_EBI_nWR)                       /*!< Set PE4 function to EBI_nWR             */
#define SET_EBI_nWR_PJ9()           SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ9MFP_Msk)) | SYS_GPJ_MFP2_PJ9MFP_EBI_nWR)                       /*!< Set PJ9 function to EBI_nWR             */
#define SET_ECAP0_IC0_PA10()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_ECAP0_IC0)                   /*!< Set PA10 function to ECAP0_IC0          */
#define SET_ECAP0_IC0_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_ECAP0_IC0)                     /*!< Set PE8 function to ECAP0_IC0           */
#define SET_ECAP0_IC1_PA9()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_ECAP0_IC1)                     /*!< Set PA9 function to ECAP0_IC1           */
#define SET_ECAP0_IC1_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_ECAP0_IC1)                     /*!< Set PE9 function to ECAP0_IC1           */
#define SET_ECAP0_IC2_PA8()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_ECAP0_IC2)                     /*!< Set PA8 function to ECAP0_IC2           */
#define SET_ECAP0_IC2_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_ECAP0_IC2)                   /*!< Set PE10 function to ECAP0_IC2          */
#define SET_ECAP1_IC0_PC10()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_ECAP1_IC0)                   /*!< Set PC10 function to ECAP1_IC0          */
#define SET_ECAP1_IC0_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_ECAP1_IC0)                   /*!< Set PE13 function to ECAP1_IC0          */
#define SET_ECAP1_IC1_PC11()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_ECAP1_IC1)                   /*!< Set PC11 function to ECAP1_IC1          */
#define SET_ECAP1_IC1_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_ECAP1_IC1)                   /*!< Set PE12 function to ECAP1_IC1          */
#define SET_ECAP1_IC2_PC12()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_ECAP1_IC2)                   /*!< Set PC12 function to ECAP1_IC2          */
#define SET_ECAP1_IC2_PE11()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_ECAP1_IC2)                   /*!< Set PE11 function to ECAP1_IC2          */
#define SET_ECAP2_IC0_PD1()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_ECAP2_IC0)                     /*!< Set PD1 function to ECAP2_IC0           */
#define SET_ECAP2_IC0_PG9()         SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_ECAP2_IC0)                     /*!< Set PG9 function to ECAP2_IC0           */
#define SET_ECAP2_IC0_PJ10()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_ECAP2_IC0)                   /*!< Set PJ10 function to ECAP2_IC0          */
#define SET_ECAP2_IC1_PD0()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_ECAP2_IC1)                     /*!< Set PD0 function to ECAP2_IC1           */
#define SET_ECAP2_IC1_PG10()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_ECAP2_IC1)                   /*!< Set PG10 function to ECAP2_IC1          */
#define SET_ECAP2_IC1_PJ11()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_ECAP2_IC1)                   /*!< Set PJ11 function to ECAP2_IC1          */
#define SET_ECAP2_IC2_PD13()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_ECAP2_IC2)                   /*!< Set PD13 function to ECAP2_IC2          */
#define SET_ECAP2_IC2_PG11()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_ECAP2_IC2)                   /*!< Set PG11 function to ECAP2_IC2          */
#define SET_ECAP2_IC2_PJ12()        SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_ECAP2_IC2)                   /*!< Set PJ12 function to ECAP2_IC2          */
#define SET_ECAP3_IC0_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_ECAP3_IC0)                   /*!< Set PA12 function to ECAP3_IC0          */
#define SET_ECAP3_IC0_PD12()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_ECAP3_IC0)                   /*!< Set PD12 function to ECAP3_IC0          */
#define SET_ECAP3_IC0_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_ECAP3_IC0)                     /*!< Set PE8 function to ECAP3_IC0           */
#define SET_ECAP3_IC1_PA13()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_ECAP3_IC1)                   /*!< Set PA13 function to ECAP3_IC1          */
#define SET_ECAP3_IC1_PD11()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_ECAP3_IC1)                   /*!< Set PD11 function to ECAP3_IC1          */
#define SET_ECAP3_IC1_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_ECAP3_IC1)                     /*!< Set PE9 function to ECAP3_IC1           */
#define SET_ECAP3_IC2_PA14()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_ECAP3_IC2)                   /*!< Set PA14 function to ECAP3_IC2          */
#define SET_ECAP3_IC2_PD10()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_ECAP3_IC2)                   /*!< Set PD10 function to ECAP3_IC2          */
#define SET_ECAP3_IC2_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_ECAP3_IC2)                   /*!< Set PE10 function to ECAP3_IC2          */
#define SET_EMAC0_PPS_PB6()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EMAC0_PPS)                     /*!< Set PB6 function to EMAC0_PPS           */
#define SET_EMAC0_PPS_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_EMAC0_PPS)                   /*!< Set PE13 function to EMAC0_PPS          */
#define SET_EMAC0_RMII_CRSDV_PA7()  SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_EMAC0_RMII_CRSDV)              /*!< Set PA7 function to EMAC0_RMII_CRSDV    */
#define SET_EMAC0_RMII_CRSDV_PB2()  SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_EMAC0_RMII_CRSDV)              /*!< Set PB2 function to EMAC0_RMII_CRSDV    */
#define SET_EMAC0_RMII_MDC_PB11()   SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_EMAC0_RMII_MDC)              /*!< Set PB11 function to EMAC0_RMII_MDC     */
#define SET_EMAC0_RMII_MDC_PE8()    SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_EMAC0_RMII_MDC)                /*!< Set PE8 function to EMAC0_RMII_MDC      */
#define SET_EMAC0_RMII_MDIO_PB10()  SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_EMAC0_RMII_MDIO)             /*!< Set PB10 function to EMAC0_RMII_MDIO    */
#define SET_EMAC0_RMII_MDIO_PE9()   SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_EMAC0_RMII_MDIO)               /*!< Set PE9 function to EMAC0_RMII_MDIO     */
#define SET_EMAC0_RMII_REFCLK_PB5() SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_EMAC0_RMII_REFCLK)             /*!< Set PB5 function to EMAC0_RMII_REFCLK   */
#define SET_EMAC0_RMII_REFCLK_PC8() SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_EMAC0_RMII_REFCLK)             /*!< Set PC8 function to EMAC0_RMII_REFCLK   */
#define SET_EMAC0_RMII_RXD0_PB4()   SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_EMAC0_RMII_RXD0)               /*!< Set PB4 function to EMAC0_RMII_RXD0     */
#define SET_EMAC0_RMII_RXD0_PC7()   SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_EMAC0_RMII_RXD0)               /*!< Set PC7 function to EMAC0_RMII_RXD0     */
#define SET_EMAC0_RMII_RXD1_PB3()   SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_EMAC0_RMII_RXD1)               /*!< Set PB3 function to EMAC0_RMII_RXD1     */
#define SET_EMAC0_RMII_RXD1_PC6()   SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_EMAC0_RMII_RXD1)               /*!< Set PC6 function to EMAC0_RMII_RXD1     */
#define SET_EMAC0_RMII_RXERR_PA6()  SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_EMAC0_RMII_RXERR)              /*!< Set PA6 function to EMAC0_RMII_RXERR    */
#define SET_EMAC0_RMII_RXERR_PB1()  SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EMAC0_RMII_RXERR)              /*!< Set PB1 function to EMAC0_RMII_RXERR    */
#define SET_EMAC0_RMII_TXD0_PB9()   SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_EMAC0_RMII_TXD0)               /*!< Set PB9 function to EMAC0_RMII_TXD0     */
#define SET_EMAC0_RMII_TXD0_PE10()  SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_EMAC0_RMII_TXD0)             /*!< Set PE10 function to EMAC0_RMII_TXD0    */
#define SET_EMAC0_RMII_TXD1_PB8()   SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_EMAC0_RMII_TXD1)               /*!< Set PB8 function to EMAC0_RMII_TXD1     */
#define SET_EMAC0_RMII_TXD1_PE11()  SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_EMAC0_RMII_TXD1)             /*!< Set PE11 function to EMAC0_RMII_TXD1    */
#define SET_EMAC0_RMII_TXEN_PB7()   SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EMAC0_RMII_TXEN)               /*!< Set PB7 function to EMAC0_RMII_TXEN     */
#define SET_EMAC0_RMII_TXEN_PE12()  SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_EMAC0_RMII_TXEN)             /*!< Set PE12 function to EMAC0_RMII_TXEN    */
#define SET_EPWM0_BRAKE0_PB1()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EPWM0_BRAKE0)                  /*!< Set PB1 function to EPWM0_BRAKE0        */
#define SET_EPWM0_BRAKE0_PE8()      SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_EPWM0_BRAKE0)                  /*!< Set PE8 function to EPWM0_BRAKE0        */
#define SET_EPWM0_BRAKE1_PA11()     SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_EPWM0_BRAKE1)                /*!< Set PA11 function to EPWM0_BRAKE1       */
#define SET_EPWM0_BRAKE1_PB0()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_EPWM0_BRAKE1)                  /*!< Set PB0 function to EPWM0_BRAKE1        */
#define SET_EPWM0_BRAKE1_PB15()     SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_EPWM0_BRAKE1)                /*!< Set PB15 function to EPWM0_BRAKE1       */
#define SET_EPWM0_BRAKE1_PE9()      SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_EPWM0_BRAKE1)                  /*!< Set PE9 function to EPWM0_BRAKE1        */
#define SET_EPWM0_CH0_PA5()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EPWM0_CH0)                     /*!< Set PA5 function to EPWM0_CH0           */
#define SET_EPWM0_CH0_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_EPWM0_CH0)                     /*!< Set PB5 function to EPWM0_CH0           */
#define SET_EPWM0_CH0_PE7()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_EPWM0_CH0)                     /*!< Set PE7 function to EPWM0_CH0           */
#define SET_EPWM0_CH0_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_EPWM0_CH0)                     /*!< Set PE8 function to EPWM0_CH0           */
#define SET_EPWM0_CH0_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_EPWM0_CH0)                     /*!< Set PF5 function to EPWM0_CH0           */
#define SET_EPWM0_CH0_PG1()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_EPWM0_CH0)                     /*!< Set PG1 function to EPWM0_CH0           */
#define SET_EPWM0_CH1_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_EPWM0_CH1)                     /*!< Set PA4 function to EPWM0_CH1           */
#define SET_EPWM0_CH1_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_EPWM0_CH1)                     /*!< Set PB4 function to EPWM0_CH1           */
#define SET_EPWM0_CH1_PE6()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_EPWM0_CH1)                     /*!< Set PE6 function to EPWM0_CH1           */
#define SET_EPWM0_CH1_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_EPWM0_CH1)                     /*!< Set PE9 function to EPWM0_CH1           */
#define SET_EPWM0_CH1_PF4()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_EPWM0_CH1)                     /*!< Set PF4 function to EPWM0_CH1           */
#define SET_EPWM0_CH1_PG0()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_EPWM0_CH1)                     /*!< Set PG0 function to EPWM0_CH1           */
#define SET_EPWM0_CH2_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_EPWM0_CH2)                     /*!< Set PA3 function to EPWM0_CH2           */
#define SET_EPWM0_CH2_PB3()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_EPWM0_CH2)                     /*!< Set PB3 function to EPWM0_CH2           */
#define SET_EPWM0_CH2_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_EPWM0_CH2)                   /*!< Set PE10 function to EPWM0_CH2          */
#define SET_EPWM0_CH2_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_EPWM0_CH2)                     /*!< Set PE5 function to EPWM0_CH2           */
#define SET_EPWM0_CH2_PG6()         SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_EPWM0_CH2)                     /*!< Set PG6 function to EPWM0_CH2           */
#define SET_EPWM0_CH3_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_EPWM0_CH3)                     /*!< Set PA2 function to EPWM0_CH3           */
#define SET_EPWM0_CH3_PB2()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_EPWM0_CH3)                     /*!< Set PB2 function to EPWM0_CH3           */
#define SET_EPWM0_CH3_PE11()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_EPWM0_CH3)                   /*!< Set PE11 function to EPWM0_CH3          */
#define SET_EPWM0_CH3_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_EPWM0_CH3)                     /*!< Set PE4 function to EPWM0_CH3           */
#define SET_EPWM0_CH3_PG5()         SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_EPWM0_CH3)                     /*!< Set PG5 function to EPWM0_CH3           */
#define SET_EPWM0_CH4_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_EPWM0_CH4)                     /*!< Set PA1 function to EPWM0_CH4           */
#define SET_EPWM0_CH4_PB1()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EPWM0_CH4)                     /*!< Set PB1 function to EPWM0_CH4           */
#define SET_EPWM0_CH4_PD14()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_EPWM0_CH4)                   /*!< Set PD14 function to EPWM0_CH4          */
#define SET_EPWM0_CH4_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_EPWM0_CH4)                   /*!< Set PE12 function to EPWM0_CH4          */
#define SET_EPWM0_CH4_PE3()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_EPWM0_CH4)                     /*!< Set PE3 function to EPWM0_CH4           */
#define SET_EPWM0_CH4_PH5()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_EPWM0_CH4)                     /*!< Set PH5 function to EPWM0_CH4           */
#define SET_EPWM0_CH5_PA0()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_EPWM0_CH5)                     /*!< Set PA0 function to EPWM0_CH5           */
#define SET_EPWM0_CH5_PB0()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_EPWM0_CH5)                     /*!< Set PB0 function to EPWM0_CH5           */
#define SET_EPWM0_CH5_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_EPWM0_CH5)                   /*!< Set PE13 function to EPWM0_CH5          */
#define SET_EPWM0_CH5_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_EPWM0_CH5)                     /*!< Set PE2 function to EPWM0_CH5           */
#define SET_EPWM0_CH5_PH11()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_EPWM0_CH5)                   /*!< Set PH11 function to EPWM0_CH5          */
#define SET_EPWM0_CH5_PH4()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_EPWM0_CH5)                     /*!< Set PH4 function to EPWM0_CH5           */
#define SET_EPWM0_SYNC_IN_PA15()    SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_EPWM0_SYNC_IN)               /*!< Set PA15 function to EPWM0_SYNC_IN      */
#define SET_EPWM0_SYNC_IN_PB6()     SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EPWM0_SYNC_IN)                 /*!< Set PB6 function to EPWM0_SYNC_IN       */
#define SET_EPWM0_SYNC_OUT_PA11()   SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_EPWM0_SYNC_OUT)              /*!< Set PA11 function to EPWM0_SYNC_OUT     */
#define SET_EPWM0_SYNC_OUT_PF5()    SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_EPWM0_SYNC_OUT)                /*!< Set PF5 function to EPWM0_SYNC_OUT      */
#define SET_EPWM1_BRAKE0_PB7()      SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EPWM1_BRAKE0)                  /*!< Set PB7 function to EPWM1_BRAKE0        */
#define SET_EPWM1_BRAKE0_PE10()     SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_EPWM1_BRAKE0)                /*!< Set PE10 function to EPWM1_BRAKE0       */
#define SET_EPWM1_BRAKE1_PA3()      SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_EPWM1_BRAKE1)                  /*!< Set PA3 function to EPWM1_BRAKE1        */
#define SET_EPWM1_BRAKE1_PB6()      SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EPWM1_BRAKE1)                  /*!< Set PB6 function to EPWM1_BRAKE1        */
#define SET_EPWM1_BRAKE1_PE11()     SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_EPWM1_BRAKE1)                /*!< Set PE11 function to EPWM1_BRAKE1       */
#define SET_EPWM1_CH0_PB15()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_EPWM1_CH0)                   /*!< Set PB15 function to EPWM1_CH0          */
#define SET_EPWM1_CH0_PC12()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_EPWM1_CH0)                   /*!< Set PC12 function to EPWM1_CH0          */
#define SET_EPWM1_CH0_PC5()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_EPWM1_CH0)                     /*!< Set PC5 function to EPWM1_CH0           */
#define SET_EPWM1_CH0_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_EPWM1_CH0)                   /*!< Set PE13 function to EPWM1_CH0          */
#define SET_EPWM1_CH0_PF3()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_EPWM1_CH0)                     /*!< Set PF3 function to EPWM1_CH0           */
#define SET_EPWM1_CH0_PI12()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_EPWM1_CH0)                   /*!< Set PI12 function to EPWM1_CH0          */
#define SET_EPWM1_CH1_PB14()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_EPWM1_CH1)                   /*!< Set PB14 function to EPWM1_CH1          */
#define SET_EPWM1_CH1_PC11()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_EPWM1_CH1)                   /*!< Set PC11 function to EPWM1_CH1          */
#define SET_EPWM1_CH1_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_EPWM1_CH1)                     /*!< Set PC4 function to EPWM1_CH1           */
#define SET_EPWM1_CH1_PC8()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_EPWM1_CH1)                     /*!< Set PC8 function to EPWM1_CH1           */
#define SET_EPWM1_CH1_PF2()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_EPWM1_CH1)                     /*!< Set PF2 function to EPWM1_CH1           */
#define SET_EPWM1_CH1_PI13()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_EPWM1_CH1)                   /*!< Set PI13 function to EPWM1_CH1          */
#define SET_EPWM1_CH2_PB13()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_EPWM1_CH2)                   /*!< Set PB13 function to EPWM1_CH2          */
#define SET_EPWM1_CH2_PC10()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_EPWM1_CH2)                   /*!< Set PC10 function to EPWM1_CH2          */
#define SET_EPWM1_CH2_PC3()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_EPWM1_CH2)                     /*!< Set PC3 function to EPWM1_CH2           */
#define SET_EPWM1_CH2_PC7()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_EPWM1_CH2)                     /*!< Set PC7 function to EPWM1_CH2           */
#define SET_EPWM1_CH2_PI14()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_EPWM1_CH2)                   /*!< Set PI14 function to EPWM1_CH2          */
#define SET_EPWM1_CH3_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_EPWM1_CH3)                   /*!< Set PB12 function to EPWM1_CH3          */
#define SET_EPWM1_CH3_PC2()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_EPWM1_CH3)                     /*!< Set PC2 function to EPWM1_CH3           */
#define SET_EPWM1_CH3_PC6()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_EPWM1_CH3)                     /*!< Set PC6 function to EPWM1_CH3           */
#define SET_EPWM1_CH3_PC9()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_EPWM1_CH3)                     /*!< Set PC9 function to EPWM1_CH3           */
#define SET_EPWM1_CH3_PI15()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_EPWM1_CH3)                   /*!< Set PI15 function to EPWM1_CH3          */
#define SET_EPWM1_CH4_PA7()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_EPWM1_CH4)                     /*!< Set PA7 function to EPWM1_CH4           */
#define SET_EPWM1_CH4_PB1()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_EPWM1_CH4)                     /*!< Set PB1 function to EPWM1_CH4           */
#define SET_EPWM1_CH4_PB7()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_EPWM1_CH4)                     /*!< Set PB7 function to EPWM1_CH4           */
#define SET_EPWM1_CH4_PC1()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_EPWM1_CH4)                     /*!< Set PC1 function to EPWM1_CH4           */
#define SET_EPWM1_CH4_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_EPWM1_CH4)                     /*!< Set PF0 function to EPWM1_CH4           */
#define SET_EPWM1_CH4_PJ0()         SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ0MFP_Msk)) | SYS_GPJ_MFP0_PJ0MFP_EPWM1_CH4)                     /*!< Set PJ0 function to EPWM1_CH4           */
#define SET_EPWM1_CH5_PA6()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_EPWM1_CH5)                     /*!< Set PA6 function to EPWM1_CH5           */
#define SET_EPWM1_CH5_PB0()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_EPWM1_CH5)                     /*!< Set PB0 function to EPWM1_CH5           */
#define SET_EPWM1_CH5_PB6()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_EPWM1_CH5)                     /*!< Set PB6 function to EPWM1_CH5           */
#define SET_EPWM1_CH5_PC0()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_EPWM1_CH5)                     /*!< Set PC0 function to EPWM1_CH5           */
#define SET_EPWM1_CH5_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_EPWM1_CH5)                     /*!< Set PF1 function to EPWM1_CH5           */
#define SET_EPWM1_CH5_PJ1()         SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ1MFP_Msk)) | SYS_GPJ_MFP0_PJ1MFP_EPWM1_CH5)                     /*!< Set PJ1 function to EPWM1_CH5           */
#define SET_EQEI0_A_PA4()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_EQEI0_A)                       /*!< Set PA4 function to EQEI0_A             */
#define SET_EQEI0_A_PD11()          SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_EQEI0_A)                     /*!< Set PD11 function to EQEI0_A            */
#define SET_EQEI0_A_PE3()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_EQEI0_A)                       /*!< Set PE3 function to EQEI0_A             */
#define SET_EQEI0_B_PA3()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_EQEI0_B)                       /*!< Set PA3 function to EQEI0_B             */
#define SET_EQEI0_B_PD10()          SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_EQEI0_B)                     /*!< Set PD10 function to EQEI0_B            */
#define SET_EQEI0_B_PE2()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_EQEI0_B)                       /*!< Set PE2 function to EQEI0_B             */
#define SET_EQEI0_INDEX_PA5()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EQEI0_INDEX)                   /*!< Set PA5 function to EQEI0_INDEX         */
#define SET_EQEI0_INDEX_PD12()      SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_EQEI0_INDEX)                 /*!< Set PD12 function to EQEI0_INDEX        */
#define SET_EQEI0_INDEX_PE4()       SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_EQEI0_INDEX)                   /*!< Set PE4 function to EQEI0_INDEX         */
#define SET_EQEI1_A_PA13()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_EQEI1_A)                     /*!< Set PA13 function to EQEI1_A            */
#define SET_EQEI1_A_PA9()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_EQEI1_A)                       /*!< Set PA9 function to EQEI1_A             */
#define SET_EQEI1_A_PE6()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_EQEI1_A)                       /*!< Set PE6 function to EQEI1_A             */
#define SET_EQEI1_B_PA14()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_EQEI1_B)                     /*!< Set PA14 function to EQEI1_B            */
#define SET_EQEI1_B_PA8()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_EQEI1_B)                       /*!< Set PA8 function to EQEI1_B             */
#define SET_EQEI1_B_PE5()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_EQEI1_B)                       /*!< Set PE5 function to EQEI1_B             */
#define SET_EQEI1_INDEX_PA10()      SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_EQEI1_INDEX)                 /*!< Set PA10 function to EQEI1_INDEX        */
#define SET_EQEI1_INDEX_PA12()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_EQEI1_INDEX)                 /*!< Set PA12 function to EQEI1_INDEX        */
#define SET_EQEI1_INDEX_PE7()       SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_EQEI1_INDEX)                   /*!< Set PE7 function to EQEI1_INDEX         */
#define SET_EQEI2_A_PD0()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_EQEI2_A)                       /*!< Set PD0 function to EQEI2_A             */
#define SET_EQEI2_A_PE9()           SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_EQEI2_A)                       /*!< Set PE9 function to EQEI2_A             */
#define SET_EQEI2_A_PF5()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_EQEI2_A)                       /*!< Set PF5 function to EQEI2_A             */
#define SET_EQEI2_B_PD13()          SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_EQEI2_B)                     /*!< Set PD13 function to EQEI2_B            */
#define SET_EQEI2_B_PE10()          SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_EQEI2_B)                     /*!< Set PE10 function to EQEI2_B            */
#define SET_EQEI2_B_PF4()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_EQEI2_B)                       /*!< Set PF4 function to EQEI2_B             */
#define SET_EQEI2_INDEX_PD1()       SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_EQEI2_INDEX)                   /*!< Set PD1 function to EQEI2_INDEX         */
#define SET_EQEI2_INDEX_PE8()       SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_EQEI2_INDEX)                   /*!< Set PE8 function to EQEI2_INDEX         */
#define SET_EQEI2_INDEX_PF6()       SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_EQEI2_INDEX)                   /*!< Set PF6 function to EQEI2_INDEX         */
#define SET_EQEI3_A_PA1()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_EQEI3_A)                       /*!< Set PA1 function to EQEI3_A             */
#define SET_EQEI3_A_PD3()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_EQEI3_A)                       /*!< Set PD3 function to EQEI3_A             */
#define SET_EQEI3_B_PA0()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_EQEI3_B)                       /*!< Set PA0 function to EQEI3_B             */
#define SET_EQEI3_B_PD2()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_EQEI3_B)                       /*!< Set PD2 function to EQEI3_B             */
#define SET_EQEI3_INDEX_PA15()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_EQEI3_INDEX)                 /*!< Set PA15 function to EQEI3_INDEX        */
#define SET_EQEI3_INDEX_PA2()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_EQEI3_INDEX)                   /*!< Set PA2 function to EQEI3_INDEX         */
#define SET_ETMC_TRACE_CLK_PB11()   SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_ETMC_TRACE_CLK)              /*!< Set PB11 function to ETMC_TRACE_CLK     */
#define SET_ETMC_TRACE_DATA0_PB10() SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_ETMC_TRACE_DATA0)            /*!< Set PB10 function to ETMC_TRACE_DATA0   */
#define SET_ETMC_TRACE_DATA0_PB15() SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_ETMC_TRACE_DATA0)            /*!< Set PB15 function to ETMC_TRACE_DATA0   */
#define SET_ETMC_TRACE_DATA1_PB14() SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_ETMC_TRACE_DATA1)            /*!< Set PB14 function to ETMC_TRACE_DATA1   */
#define SET_ETMC_TRACE_DATA1_PB9()  SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_ETMC_TRACE_DATA1)              /*!< Set PB9 function to ETMC_TRACE_DATA1    */
#define SET_ETMC_TRACE_DATA2_PB13() SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_ETMC_TRACE_DATA2)            /*!< Set PB13 function to ETMC_TRACE_DATA2   */
#define SET_ETMC_TRACE_DATA2_PB8()  SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_ETMC_TRACE_DATA2)              /*!< Set PB8 function to ETMC_TRACE_DATA2    */
#define SET_ETMC_TRACE_DATA3_PB12() SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_ETMC_TRACE_DATA3)            /*!< Set PB12 function to ETMC_TRACE_DATA3   */
#define SET_ETMC_TRACE_DATA3_PB7()  SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_ETMC_TRACE_DATA3)              /*!< Set PB7 function to ETMC_TRACE_DATA3    */
#define SET_GPIO_PA0()              SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_GPIO)                          /*!< Set PA0 function to GPIO                */
#define SET_GPIO_PA1()              SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_GPIO)                          /*!< Set PA1 function to GPIO                */
#define SET_GPIO_PA10()             SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_GPIO)                        /*!< Set PA10 function to GPIO               */
#define SET_GPIO_PA11()             SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_GPIO)                        /*!< Set PA11 function to GPIO               */
#define SET_GPIO_PA12()             SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_GPIO)                        /*!< Set PA12 function to GPIO               */
#define SET_GPIO_PA13()             SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_GPIO)                        /*!< Set PA13 function to GPIO               */
#define SET_GPIO_PA14()             SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_GPIO)                        /*!< Set PA14 function to GPIO               */
#define SET_GPIO_PA15()             SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_GPIO)                        /*!< Set PA15 function to GPIO               */
#define SET_GPIO_PA2()              SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_GPIO)                          /*!< Set PA2 function to GPIO                */
#define SET_GPIO_PA3()              SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_GPIO)                          /*!< Set PA3 function to GPIO                */
#define SET_GPIO_PA4()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_GPIO)                          /*!< Set PA4 function to GPIO                */
#define SET_GPIO_PA5()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_GPIO)                          /*!< Set PA5 function to GPIO                */
#define SET_GPIO_PA6()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_GPIO)                          /*!< Set PA6 function to GPIO                */
#define SET_GPIO_PA7()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_GPIO)                          /*!< Set PA7 function to GPIO                */
#define SET_GPIO_PA8()              SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_GPIO)                          /*!< Set PA8 function to GPIO                */
#define SET_GPIO_PA9()              SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_GPIO)                          /*!< Set PA9 function to GPIO                */
#define SET_GPIO_PB0()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_GPIO)                          /*!< Set PB0 function to GPIO                */
#define SET_GPIO_PB1()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_GPIO)                          /*!< Set PB1 function to GPIO                */
#define SET_GPIO_PB10()             SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_GPIO)                        /*!< Set PB10 function to GPIO               */
#define SET_GPIO_PB11()             SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_GPIO)                        /*!< Set PB11 function to GPIO               */
#define SET_GPIO_PB12()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_GPIO)                        /*!< Set PB12 function to GPIO               */
#define SET_GPIO_PB13()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_GPIO)                        /*!< Set PB13 function to GPIO               */
#define SET_GPIO_PB14()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_GPIO)                        /*!< Set PB14 function to GPIO               */
#define SET_GPIO_PB15()             SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_GPIO)                        /*!< Set PB15 function to GPIO               */
#define SET_GPIO_PB2()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_GPIO)                          /*!< Set PB2 function to GPIO                */
#define SET_GPIO_PB3()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_GPIO)                          /*!< Set PB3 function to GPIO                */
#define SET_GPIO_PB4()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_GPIO)                          /*!< Set PB4 function to GPIO                */
#define SET_GPIO_PB5()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_GPIO)                          /*!< Set PB5 function to GPIO                */
#define SET_GPIO_PB6()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_GPIO)                          /*!< Set PB6 function to GPIO                */
#define SET_GPIO_PB7()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_GPIO)                          /*!< Set PB7 function to GPIO                */
#define SET_GPIO_PB8()              SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_GPIO)                          /*!< Set PB8 function to GPIO                */
#define SET_GPIO_PB9()              SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_GPIO)                          /*!< Set PB9 function to GPIO                */
#define SET_GPIO_PC0()              SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_GPIO)                          /*!< Set PC0 function to GPIO                */
#define SET_GPIO_PC1()              SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_GPIO)                          /*!< Set PC1 function to GPIO                */
#define SET_GPIO_PC10()             SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_GPIO)                        /*!< Set PC10 function to GPIO               */
#define SET_GPIO_PC11()             SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_GPIO)                        /*!< Set PC11 function to GPIO               */
#define SET_GPIO_PC12()             SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_GPIO)                        /*!< Set PC12 function to GPIO               */
#define SET_GPIO_PC13()             SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_GPIO)                        /*!< Set PC13 function to GPIO               */
#define SET_GPIO_PC2()              SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_GPIO)                          /*!< Set PC2 function to GPIO                */
#define SET_GPIO_PC3()              SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_GPIO)                          /*!< Set PC3 function to GPIO                */
#define SET_GPIO_PC4()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_GPIO)                          /*!< Set PC4 function to GPIO                */
#define SET_GPIO_PC5()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_GPIO)                          /*!< Set PC5 function to GPIO                */
#define SET_GPIO_PC6()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_GPIO)                          /*!< Set PC6 function to GPIO                */
#define SET_GPIO_PC7()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_GPIO)                          /*!< Set PC7 function to GPIO                */
#define SET_GPIO_PC8()              SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_GPIO)                          /*!< Set PC8 function to GPIO                */
#define SET_GPIO_PC9()              SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_GPIO)                          /*!< Set PC9 function to GPIO                */
#define SET_GPIO_PD0()              SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_GPIO)                          /*!< Set PD0 function to GPIO                */
#define SET_GPIO_PD1()              SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_GPIO)                          /*!< Set PD1 function to GPIO                */
#define SET_GPIO_PD10()             SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_GPIO)                        /*!< Set PD10 function to GPIO               */
#define SET_GPIO_PD11()             SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_GPIO)                        /*!< Set PD11 function to GPIO               */
#define SET_GPIO_PD12()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_GPIO)                        /*!< Set PD12 function to GPIO               */
#define SET_GPIO_PD13()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_GPIO)                        /*!< Set PD13 function to GPIO               */
#define SET_GPIO_PD14()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_GPIO)                        /*!< Set PD14 function to GPIO               */
#define SET_GPIO_PD2()              SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_GPIO)                          /*!< Set PD2 function to GPIO                */
#define SET_GPIO_PD3()              SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_GPIO)                          /*!< Set PD3 function to GPIO                */
#define SET_GPIO_PD4()              SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_GPIO)                          /*!< Set PD4 function to GPIO                */
#define SET_GPIO_PD5()              SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_GPIO)                          /*!< Set PD5 function to GPIO                */
#define SET_GPIO_PD6()              SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_GPIO)                          /*!< Set PD6 function to GPIO                */
#define SET_GPIO_PD7()              SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_GPIO)                          /*!< Set PD7 function to GPIO                */
#define SET_GPIO_PD8()              SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_GPIO)                          /*!< Set PD8 function to GPIO                */
#define SET_GPIO_PD9()              SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_GPIO)                          /*!< Set PD9 function to GPIO                */
#define SET_GPIO_PE0()              SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_GPIO)                          /*!< Set PE0 function to GPIO                */
#define SET_GPIO_PE1()              SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_GPIO)                          /*!< Set PE1 function to GPIO                */
#define SET_GPIO_PE10()             SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_GPIO)                        /*!< Set PE10 function to GPIO               */
#define SET_GPIO_PE11()             SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_GPIO)                        /*!< Set PE11 function to GPIO               */
#define SET_GPIO_PE12()             SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_GPIO)                        /*!< Set PE12 function to GPIO               */
#define SET_GPIO_PE13()             SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_GPIO)                        /*!< Set PE13 function to GPIO               */
#define SET_GPIO_PE14()             SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_GPIO)                        /*!< Set PE14 function to GPIO               */
#define SET_GPIO_PE15()             SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_GPIO)                        /*!< Set PE15 function to GPIO               */
#define SET_GPIO_PE2()              SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_GPIO)                          /*!< Set PE2 function to GPIO                */
#define SET_GPIO_PE3()              SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_GPIO)                          /*!< Set PE3 function to GPIO                */
#define SET_GPIO_PE4()              SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_GPIO)                          /*!< Set PE4 function to GPIO                */
#define SET_GPIO_PE5()              SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_GPIO)                          /*!< Set PE5 function to GPIO                */
#define SET_GPIO_PE6()              SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_GPIO)                          /*!< Set PE6 function to GPIO                */
#define SET_GPIO_PE7()              SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_GPIO)                          /*!< Set PE7 function to GPIO                */
#define SET_GPIO_PE8()              SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_GPIO)                          /*!< Set PE8 function to GPIO                */
#define SET_GPIO_PE9()              SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_GPIO)                          /*!< Set PE9 function to GPIO                */
#define SET_GPIO_PF0()              SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_GPIO)                          /*!< Set PF0 function to GPIO                */
#define SET_GPIO_PF1()              SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_GPIO)                          /*!< Set PF1 function to GPIO                */
#define SET_GPIO_PF10()             SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_GPIO)                        /*!< Set PF10 function to GPIO               */
#define SET_GPIO_PF11()             SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_GPIO)                        /*!< Set PF11 function to GPIO               */
#define SET_GPIO_PF2()              SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_GPIO)                          /*!< Set PF2 function to GPIO                */
#define SET_GPIO_PF3()              SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_GPIO)                          /*!< Set PF3 function to GPIO                */
#define SET_GPIO_PF4()              SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_GPIO)                          /*!< Set PF4 function to GPIO                */
#define SET_GPIO_PF5()              SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_GPIO)                          /*!< Set PF5 function to GPIO                */
#define SET_GPIO_PF6()              SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_GPIO)                          /*!< Set PF6 function to GPIO                */
#define SET_GPIO_PF7()              SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_GPIO)                          /*!< Set PF7 function to GPIO                */
#define SET_GPIO_PF8()              SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_GPIO)                          /*!< Set PF8 function to GPIO                */
#define SET_GPIO_PF9()              SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_GPIO)                          /*!< Set PF9 function to GPIO                */
#define SET_GPIO_PG0()              SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_GPIO)                          /*!< Set PG0 function to GPIO                */
#define SET_GPIO_PG1()              SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_GPIO)                          /*!< Set PG1 function to GPIO                */
#define SET_GPIO_PG10()             SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_GPIO)                        /*!< Set PG10 function to GPIO               */
#define SET_GPIO_PG11()             SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_GPIO)                        /*!< Set PG11 function to GPIO               */
#define SET_GPIO_PG12()             SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_GPIO)                        /*!< Set PG12 function to GPIO               */
#define SET_GPIO_PG13()             SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_GPIO)                        /*!< Set PG13 function to GPIO               */
#define SET_GPIO_PG14()             SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_GPIO)                        /*!< Set PG14 function to GPIO               */
#define SET_GPIO_PG15()             SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_GPIO)                        /*!< Set PG15 function to GPIO               */
#define SET_GPIO_PG2()              SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_GPIO)                          /*!< Set PG2 function to GPIO                */
#define SET_GPIO_PG3()              SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_GPIO)                          /*!< Set PG3 function to GPIO                */
#define SET_GPIO_PG4()              SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG4MFP_Msk)) | SYS_GPG_MFP1_PG4MFP_GPIO)                          /*!< Set PG4 function to GPIO                */
#define SET_GPIO_PG5()              SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_GPIO)                          /*!< Set PG5 function to GPIO                */
#define SET_GPIO_PG6()              SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_GPIO)                          /*!< Set PG6 function to GPIO                */
#define SET_GPIO_PG9()              SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_GPIO)                          /*!< Set PG9 function to GPIO                */
#define SET_GPIO_PH0()              SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_GPIO)                          /*!< Set PH0 function to GPIO                */
#define SET_GPIO_PH1()              SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_GPIO)                          /*!< Set PH1 function to GPIO                */
#define SET_GPIO_PH10()             SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_GPIO)                        /*!< Set PH10 function to GPIO               */
#define SET_GPIO_PH11()             SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_GPIO)                        /*!< Set PH11 function to GPIO               */
#define SET_GPIO_PH12()             SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_GPIO)                        /*!< Set PH12 function to GPIO               */
#define SET_GPIO_PH13()             SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_GPIO)                        /*!< Set PH13 function to GPIO               */
#define SET_GPIO_PH14()             SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH14MFP_Msk)) | SYS_GPH_MFP3_PH14MFP_GPIO)                        /*!< Set PH14 function to GPIO               */
#define SET_GPIO_PH15()             SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH15MFP_Msk)) | SYS_GPH_MFP3_PH15MFP_GPIO)                        /*!< Set PH15 function to GPIO               */
#define SET_GPIO_PH2()              SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_GPIO)                          /*!< Set PH2 function to GPIO                */
#define SET_GPIO_PH3()              SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_GPIO)                          /*!< Set PH3 function to GPIO                */
#define SET_GPIO_PH4()              SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_GPIO)                          /*!< Set PH4 function to GPIO                */
#define SET_GPIO_PH5()              SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_GPIO)                          /*!< Set PH5 function to GPIO                */
#define SET_GPIO_PH6()              SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH6MFP_Msk)) | SYS_GPH_MFP1_PH6MFP_GPIO)                          /*!< Set PH6 function to GPIO                */
#define SET_GPIO_PH7()              SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_GPIO)                          /*!< Set PH7 function to GPIO                */
#define SET_GPIO_PH8()              SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_GPIO)                          /*!< Set PH8 function to GPIO                */
#define SET_GPIO_PH9()              SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_GPIO)                          /*!< Set PH9 function to GPIO                */
#define SET_GPIO_PI10()             SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_GPIO)                        /*!< Set PI10 function to GPIO               */
#define SET_GPIO_PI11()             SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI11MFP_Msk)) | SYS_GPI_MFP2_PI11MFP_GPIO)                        /*!< Set PI11 function to GPIO               */
#define SET_GPIO_PI12()             SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_GPIO)                        /*!< Set PI12 function to GPIO               */
#define SET_GPIO_PI13()             SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_GPIO)                        /*!< Set PI13 function to GPIO               */
#define SET_GPIO_PI14()             SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_GPIO)                        /*!< Set PI14 function to GPIO               */
#define SET_GPIO_PI15()             SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_GPIO)                        /*!< Set PI15 function to GPIO               */
#define SET_GPIO_PI6()              SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_GPIO)                          /*!< Set PI6 function to GPIO                */
#define SET_GPIO_PI7()              SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_GPIO)                          /*!< Set PI7 function to GPIO                */
#define SET_GPIO_PI8()              SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_GPIO)                          /*!< Set PI8 function to GPIO                */
#define SET_GPIO_PI9()              SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_GPIO)                          /*!< Set PI9 function to GPIO                */
#define SET_GPIO_PJ0()              SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ0MFP_Msk)) | SYS_GPJ_MFP0_PJ0MFP_GPIO)                          /*!< Set PJ0 function to GPIO                */
#define SET_GPIO_PJ1()              SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ1MFP_Msk)) | SYS_GPJ_MFP0_PJ1MFP_GPIO)                          /*!< Set PJ1 function to GPIO                */
#define SET_GPIO_PJ10()             SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_GPIO)                        /*!< Set PJ10 function to GPIO               */
#define SET_GPIO_PJ11()             SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_GPIO)                        /*!< Set PJ11 function to GPIO               */
#define SET_GPIO_PJ12()             SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_GPIO)                        /*!< Set PJ12 function to GPIO               */
#define SET_GPIO_PJ13()             SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ13MFP_Msk)) | SYS_GPJ_MFP3_PJ13MFP_GPIO)                        /*!< Set PJ13 function to GPIO               */
#define SET_GPIO_PJ2()              SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_GPIO)                          /*!< Set PJ2 function to GPIO                */
#define SET_GPIO_PJ3()              SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_GPIO)                          /*!< Set PJ3 function to GPIO                */
#define SET_GPIO_PJ4()              SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_GPIO)                          /*!< Set PJ4 function to GPIO                */
#define SET_GPIO_PJ5()              SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_GPIO)                          /*!< Set PJ5 function to GPIO                */
#define SET_GPIO_PJ6()              SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ6MFP_Msk)) | SYS_GPJ_MFP1_PJ6MFP_GPIO)                          /*!< Set PJ6 function to GPIO                */
#define SET_GPIO_PJ7()              SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ7MFP_Msk)) | SYS_GPJ_MFP1_PJ7MFP_GPIO)                          /*!< Set PJ7 function to GPIO                */
#define SET_GPIO_PJ8()              SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ8MFP_Msk)) | SYS_GPJ_MFP2_PJ8MFP_GPIO)                          /*!< Set PJ8 function to GPIO                */
#define SET_GPIO_PJ9()              SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ9MFP_Msk)) | SYS_GPJ_MFP2_PJ9MFP_GPIO)                          /*!< Set PJ9 function to GPIO                */
#define SET_HSUSB_VBUS_EN_PB10()    SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_HSUSB_VBUS_EN)               /*!< Set PB10 function to HSUSB_VBUS_EN      */
#define SET_HSUSB_VBUS_EN_PB15()    SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_HSUSB_VBUS_EN)               /*!< Set PB15 function to HSUSB_VBUS_EN      */
#define SET_HSUSB_VBUS_EN_PB6()     SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_HSUSB_VBUS_EN)                 /*!< Set PB6 function to HSUSB_VBUS_EN       */
#define SET_HSUSB_VBUS_EN_PJ13()    SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ13MFP_Msk)) | SYS_GPJ_MFP3_PJ13MFP_HSUSB_VBUS_EN)               /*!< Set PJ13 function to HSUSB_VBUS_EN      */
#define SET_HSUSB_VBUS_ST_PB11()    SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_HSUSB_VBUS_ST)               /*!< Set PB11 function to HSUSB_VBUS_ST      */
#define SET_HSUSB_VBUS_ST_PB13()    SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_HSUSB_VBUS_ST)               /*!< Set PB13 function to HSUSB_VBUS_ST      */
#define SET_HSUSB_VBUS_ST_PB7()     SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_HSUSB_VBUS_ST)                 /*!< Set PB7 function to HSUSB_VBUS_ST       */
#define SET_HSUSB_VBUS_ST_PJ12()    SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_HSUSB_VBUS_ST)               /*!< Set PJ12 function to HSUSB_VBUS_ST      */
#define SET_I2C0_SCL_PA5()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_I2C0_SCL)                      /*!< Set PA5 function to I2C0_SCL            */
#define SET_I2C0_SCL_PB5()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_I2C0_SCL)                      /*!< Set PB5 function to I2C0_SCL            */
#define SET_I2C0_SCL_PB9()          SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_I2C0_SCL)                      /*!< Set PB9 function to I2C0_SCL            */
#define SET_I2C0_SCL_PC1()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_I2C0_SCL)                      /*!< Set PC1 function to I2C0_SCL            */
#define SET_I2C0_SCL_PC12()         SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_I2C0_SCL)                    /*!< Set PC12 function to I2C0_SCL           */
#define SET_I2C0_SCL_PE13()         SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_I2C0_SCL)                    /*!< Set PE13 function to I2C0_SCL           */
#define SET_I2C0_SCL_PF0()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_I2C0_SCL)                      /*!< Set PF0 function to I2C0_SCL            */
#define SET_I2C0_SCL_PF3()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_I2C0_SCL)                      /*!< Set PF3 function to I2C0_SCL            */
#define SET_I2C0_SCL_PG0()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_I2C0_SCL)                      /*!< Set PG0 function to I2C0_SCL            */
#define SET_I2C0_SCL_PG13()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_I2C0_SCL)                    /*!< Set PG13 function to I2C0_SCL           */
#define SET_I2C0_SCL_PH2()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_I2C0_SCL)                      /*!< Set PH2 function to I2C0_SCL            */
#define SET_I2C0_SCL_PI10()         SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_I2C0_SCL)                    /*!< Set PI10 function to I2C0_SCL           */
#define SET_I2C0_SDA_PA4()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_I2C0_SDA)                      /*!< Set PA4 function to I2C0_SDA            */
#define SET_I2C0_SDA_PB4()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_I2C0_SDA)                      /*!< Set PB4 function to I2C0_SDA            */
#define SET_I2C0_SDA_PB8()          SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_I2C0_SDA)                      /*!< Set PB8 function to I2C0_SDA            */
#define SET_I2C0_SDA_PC0()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_I2C0_SDA)                      /*!< Set PC0 function to I2C0_SDA            */
#define SET_I2C0_SDA_PC11()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_I2C0_SDA)                    /*!< Set PC11 function to I2C0_SDA           */
#define SET_I2C0_SDA_PC8()          SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_I2C0_SDA)                      /*!< Set PC8 function to I2C0_SDA            */
#define SET_I2C0_SDA_PF1()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_I2C0_SDA)                      /*!< Set PF1 function to I2C0_SDA            */
#define SET_I2C0_SDA_PF2()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_I2C0_SDA)                      /*!< Set PF2 function to I2C0_SDA            */
#define SET_I2C0_SDA_PG1()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_I2C0_SDA)                      /*!< Set PG1 function to I2C0_SDA            */
#define SET_I2C0_SDA_PG14()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_I2C0_SDA)                    /*!< Set PG14 function to I2C0_SDA           */
#define SET_I2C0_SDA_PH3()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_I2C0_SDA)                      /*!< Set PH3 function to I2C0_SDA            */
#define SET_I2C0_SDA_PI11()         SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI11MFP_Msk)) | SYS_GPI_MFP2_PI11MFP_I2C0_SDA)                    /*!< Set PI11 function to I2C0_SDA           */
#define SET_I2C0_SMBAL_PA3()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_I2C0_SMBAL)                    /*!< Set PA3 function to I2C0_SMBAL          */
#define SET_I2C0_SMBAL_PC3()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_I2C0_SMBAL)                    /*!< Set PC3 function to I2C0_SMBAL          */
#define SET_I2C0_SMBAL_PG2()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_I2C0_SMBAL)                    /*!< Set PG2 function to I2C0_SMBAL          */
#define SET_I2C0_SMBAL_PI8()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_I2C0_SMBAL)                    /*!< Set PI8 function to I2C0_SMBAL          */
#define SET_I2C0_SMBSUS_PA2()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_I2C0_SMBSUS)                   /*!< Set PA2 function to I2C0_SMBSUS         */
#define SET_I2C0_SMBSUS_PC2()       SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_I2C0_SMBSUS)                   /*!< Set PC2 function to I2C0_SMBSUS         */
#define SET_I2C0_SMBSUS_PG3()       SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_I2C0_SMBSUS)                   /*!< Set PG3 function to I2C0_SMBSUS         */
#define SET_I2C0_SMBSUS_PI9()       SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_I2C0_SMBSUS)                   /*!< Set PI9 function to I2C0_SMBSUS         */
#define SET_I2C1_SCL_PA12()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_I2C1_SCL)                    /*!< Set PA12 function to I2C1_SCL           */
#define SET_I2C1_SCL_PA3()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_I2C1_SCL)                      /*!< Set PA3 function to I2C1_SCL            */
#define SET_I2C1_SCL_PA7()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_I2C1_SCL)                      /*!< Set PA7 function to I2C1_SCL            */
#define SET_I2C1_SCL_PB1()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_I2C1_SCL)                      /*!< Set PB1 function to I2C1_SCL            */
#define SET_I2C1_SCL_PB11()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_I2C1_SCL)                    /*!< Set PB11 function to I2C1_SCL           */
#define SET_I2C1_SCL_PB3()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_I2C1_SCL)                      /*!< Set PB3 function to I2C1_SCL            */
#define SET_I2C1_SCL_PC5()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_I2C1_SCL)                      /*!< Set PC5 function to I2C1_SCL            */
#define SET_I2C1_SCL_PE1()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_I2C1_SCL)                      /*!< Set PE1 function to I2C1_SCL            */
#define SET_I2C1_SCL_PE14()         SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_I2C1_SCL)                    /*!< Set PE14 function to I2C1_SCL           */
#define SET_I2C1_SCL_PF0()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_I2C1_SCL)                      /*!< Set PF0 function to I2C1_SCL            */
#define SET_I2C1_SCL_PG15()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_I2C1_SCL)                    /*!< Set PG15 function to I2C1_SCL           */
#define SET_I2C1_SCL_PG2()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_I2C1_SCL)                      /*!< Set PG2 function to I2C1_SCL            */
#define SET_I2C1_SCL_PI6()          SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_I2C1_SCL)                      /*!< Set PI6 function to I2C1_SCL            */
#define SET_I2C1_SDA_PA13()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_I2C1_SDA)                    /*!< Set PA13 function to I2C1_SDA           */
#define SET_I2C1_SDA_PA2()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_I2C1_SDA)                      /*!< Set PA2 function to I2C1_SDA            */
#define SET_I2C1_SDA_PA6()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_I2C1_SDA)                      /*!< Set PA6 function to I2C1_SDA            */
#define SET_I2C1_SDA_PB0()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_I2C1_SDA)                      /*!< Set PB0 function to I2C1_SDA            */
#define SET_I2C1_SDA_PB10()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_I2C1_SDA)                    /*!< Set PB10 function to I2C1_SDA           */
#define SET_I2C1_SDA_PB2()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_I2C1_SDA)                      /*!< Set PB2 function to I2C1_SDA            */
#define SET_I2C1_SDA_PC4()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_I2C1_SDA)                      /*!< Set PC4 function to I2C1_SDA            */
#define SET_I2C1_SDA_PD4()          SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_I2C1_SDA)                      /*!< Set PD4 function to I2C1_SDA            */
#define SET_I2C1_SDA_PE0()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_I2C1_SDA)                      /*!< Set PE0 function to I2C1_SDA            */
#define SET_I2C1_SDA_PE15()         SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_I2C1_SDA)                    /*!< Set PE15 function to I2C1_SDA           */
#define SET_I2C1_SDA_PF1()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_I2C1_SDA)                      /*!< Set PF1 function to I2C1_SDA            */
#define SET_I2C1_SDA_PG3()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_I2C1_SDA)                      /*!< Set PG3 function to I2C1_SDA            */
#define SET_I2C1_SDA_PI7()          SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_I2C1_SDA)                      /*!< Set PI7 function to I2C1_SDA            */
#define SET_I2C1_SMBAL_PB9()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_I2C1_SMBAL)                    /*!< Set PB9 function to I2C1_SMBAL          */
#define SET_I2C1_SMBAL_PC7()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_I2C1_SMBAL)                    /*!< Set PC7 function to I2C1_SMBAL          */
#define SET_I2C1_SMBAL_PG0()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_I2C1_SMBAL)                    /*!< Set PG0 function to I2C1_SMBAL          */
#define SET_I2C1_SMBAL_PH8()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_I2C1_SMBAL)                    /*!< Set PH8 function to I2C1_SMBAL          */
#define SET_I2C1_SMBSUS_PB8()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_I2C1_SMBSUS)                   /*!< Set PB8 function to I2C1_SMBSUS         */
#define SET_I2C1_SMBSUS_PC6()       SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_I2C1_SMBSUS)                   /*!< Set PC6 function to I2C1_SMBSUS         */
#define SET_I2C1_SMBSUS_PG1()       SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_I2C1_SMBSUS)                   /*!< Set PG1 function to I2C1_SMBSUS         */
#define SET_I2C1_SMBSUS_PH9()       SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_I2C1_SMBSUS)                   /*!< Set PH9 function to I2C1_SMBSUS         */
#define SET_I2C2_SCL_PA1()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_I2C2_SCL)                      /*!< Set PA1 function to I2C2_SCL            */
#define SET_I2C2_SCL_PA11()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_I2C2_SCL)                    /*!< Set PA11 function to I2C2_SCL           */
#define SET_I2C2_SCL_PA14()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_I2C2_SCL)                    /*!< Set PA14 function to I2C2_SCL           */
#define SET_I2C2_SCL_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_I2C2_SCL)                    /*!< Set PB13 function to I2C2_SCL           */
#define SET_I2C2_SCL_PD1()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_I2C2_SCL)                      /*!< Set PD1 function to I2C2_SCL            */
#define SET_I2C2_SCL_PD9()          SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_I2C2_SCL)                      /*!< Set PD9 function to I2C2_SCL            */
#define SET_I2C2_SCL_PH8()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_I2C2_SCL)                      /*!< Set PH8 function to I2C2_SCL            */
#define SET_I2C2_SDA_PA0()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_I2C2_SDA)                      /*!< Set PA0 function to I2C2_SDA            */
#define SET_I2C2_SDA_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_I2C2_SDA)                    /*!< Set PA10 function to I2C2_SDA           */
#define SET_I2C2_SDA_PA15()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_I2C2_SDA)                    /*!< Set PA15 function to I2C2_SDA           */
#define SET_I2C2_SDA_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_I2C2_SDA)                    /*!< Set PB12 function to I2C2_SDA           */
#define SET_I2C2_SDA_PD0()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_I2C2_SDA)                      /*!< Set PD0 function to I2C2_SDA            */
#define SET_I2C2_SDA_PD8()          SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_I2C2_SDA)                      /*!< Set PD8 function to I2C2_SDA            */
#define SET_I2C2_SDA_PH9()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_I2C2_SDA)                      /*!< Set PH9 function to I2C2_SDA            */
#define SET_I2C2_SMBAL_PB15()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_I2C2_SMBAL)                  /*!< Set PB15 function to I2C2_SMBAL         */
#define SET_I2C2_SMBAL_PF3()        SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_I2C2_SMBAL)                    /*!< Set PF3 function to I2C2_SMBAL          */
#define SET_I2C2_SMBSUS_PB14()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_I2C2_SMBSUS)                 /*!< Set PB14 function to I2C2_SMBSUS        */
#define SET_I2C2_SMBSUS_PF2()       SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_I2C2_SMBSUS)                   /*!< Set PF2 function to I2C2_SMBSUS         */
#define SET_I2C3_SCL_PC3()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_I2C3_SCL)                      /*!< Set PC3 function to I2C3_SCL            */
#define SET_I2C3_SCL_PG0()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_I2C3_SCL)                      /*!< Set PG0 function to I2C3_SCL            */
#define SET_I2C3_SCL_PH0()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_I2C3_SCL)                      /*!< Set PH0 function to I2C3_SCL            */
#define SET_I2C3_SCL_PI14()         SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_I2C3_SCL)                    /*!< Set PI14 function to I2C3_SCL           */
#define SET_I2C3_SCL_PJ4()          SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_I2C3_SCL)                      /*!< Set PJ4 function to I2C3_SCL            */
#define SET_I2C3_SDA_PC2()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_I2C3_SDA)                      /*!< Set PC2 function to I2C3_SDA            */
#define SET_I2C3_SDA_PG1()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_I2C3_SDA)                      /*!< Set PG1 function to I2C3_SDA            */
#define SET_I2C3_SDA_PH1()          SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_I2C3_SDA)                      /*!< Set PH1 function to I2C3_SDA            */
#define SET_I2C3_SDA_PI15()         SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_I2C3_SDA)                    /*!< Set PI15 function to I2C3_SDA           */
#define SET_I2C3_SDA_PJ5()          SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_I2C3_SDA)                      /*!< Set PJ5 function to I2C3_SDA            */
#define SET_I2C3_SMBAL_PC5()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_I2C3_SMBAL)                    /*!< Set PC5 function to I2C3_SMBAL          */
#define SET_I2C3_SMBAL_PG2()        SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_I2C3_SMBAL)                    /*!< Set PG2 function to I2C3_SMBAL          */
#define SET_I2C3_SMBAL_PG5()        SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_I2C3_SMBAL)                    /*!< Set PG5 function to I2C3_SMBAL          */
#define SET_I2C3_SMBAL_PI12()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_I2C3_SMBAL)                  /*!< Set PI12 function to I2C3_SMBAL         */
#define SET_I2C3_SMBAL_PJ2()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_I2C3_SMBAL)                    /*!< Set PJ2 function to I2C3_SMBAL          */
#define SET_I2C3_SMBSUS_PC4()       SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_I2C3_SMBSUS)                   /*!< Set PC4 function to I2C3_SMBSUS         */
#define SET_I2C3_SMBSUS_PG3()       SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_I2C3_SMBSUS)                   /*!< Set PG3 function to I2C3_SMBSUS         */
#define SET_I2C3_SMBSUS_PG6()       SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_I2C3_SMBSUS)                   /*!< Set PG6 function to I2C3_SMBSUS         */
#define SET_I2C3_SMBSUS_PI13()      SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_I2C3_SMBSUS)                 /*!< Set PI13 function to I2C3_SMBSUS        */
#define SET_I2C3_SMBSUS_PJ3()       SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_I2C3_SMBSUS)                   /*!< Set PJ3 function to I2C3_SMBSUS         */
#define SET_I2S0_BCLK_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_I2S0_BCLK)                   /*!< Set PA12 function to I2S0_BCLK          */
#define SET_I2S0_BCLK_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_I2S0_BCLK)                     /*!< Set PB5 function to I2S0_BCLK           */
#define SET_I2S0_BCLK_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_I2S0_BCLK)                     /*!< Set PC4 function to I2S0_BCLK           */
#define SET_I2S0_BCLK_PE1()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_I2S0_BCLK)                     /*!< Set PE1 function to I2S0_BCLK           */
#define SET_I2S0_BCLK_PE3()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_I2S0_BCLK)                     /*!< Set PE3 function to I2S0_BCLK           */
#define SET_I2S0_BCLK_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_I2S0_BCLK)                     /*!< Set PE8 function to I2S0_BCLK           */
#define SET_I2S0_BCLK_PF10()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_I2S0_BCLK)                   /*!< Set PF10 function to I2S0_BCLK          */
#define SET_I2S0_BCLK_PI6()         SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_I2S0_BCLK)                     /*!< Set PI6 function to I2S0_BCLK           */
#define SET_I2S0_DI_PA14()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_I2S0_DI)                     /*!< Set PA14 function to I2S0_DI            */
#define SET_I2S0_DI_PB3()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_I2S0_DI)                       /*!< Set PB3 function to I2S0_DI             */
#define SET_I2S0_DI_PC2()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_I2S0_DI)                       /*!< Set PC2 function to I2S0_DI             */
#define SET_I2S0_DI_PE10()          SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_I2S0_DI)                     /*!< Set PE10 function to I2S0_DI            */
#define SET_I2S0_DI_PF8()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_I2S0_DI)                       /*!< Set PF8 function to I2S0_DI             */
#define SET_I2S0_DI_PH8()           SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_I2S0_DI)                       /*!< Set PH8 function to I2S0_DI             */
#define SET_I2S0_DI_PI8()           SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_I2S0_DI)                       /*!< Set PI8 function to I2S0_DI             */
#define SET_I2S0_DO_PA15()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_I2S0_DO)                     /*!< Set PA15 function to I2S0_DO            */
#define SET_I2S0_DO_PB2()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_I2S0_DO)                       /*!< Set PB2 function to I2S0_DO             */
#define SET_I2S0_DO_PC1()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_I2S0_DO)                       /*!< Set PC1 function to I2S0_DO             */
#define SET_I2S0_DO_PE11()          SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_I2S0_DO)                     /*!< Set PE11 function to I2S0_DO            */
#define SET_I2S0_DO_PF0()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_I2S0_DO)                       /*!< Set PF0 function to I2S0_DO             */
#define SET_I2S0_DO_PF7()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_I2S0_DO)                       /*!< Set PF7 function to I2S0_DO             */
#define SET_I2S0_DO_PH9()           SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_I2S0_DO)                       /*!< Set PH9 function to I2S0_DO             */
#define SET_I2S0_DO_PI9()           SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_I2S0_DO)                       /*!< Set PI9 function to I2S0_DO             */
#define SET_I2S0_LRCK_PB1()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_I2S0_LRCK)                     /*!< Set PB1 function to I2S0_LRCK           */
#define SET_I2S0_LRCK_PC0()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_I2S0_LRCK)                     /*!< Set PC0 function to I2S0_LRCK           */
#define SET_I2S0_LRCK_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_I2S0_LRCK)                   /*!< Set PE12 function to I2S0_LRCK          */
#define SET_I2S0_LRCK_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_I2S0_LRCK)                     /*!< Set PF1 function to I2S0_LRCK           */
#define SET_I2S0_LRCK_PF6()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_I2S0_LRCK)                     /*!< Set PF6 function to I2S0_LRCK           */
#define SET_I2S0_LRCK_PH10()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_I2S0_LRCK)                   /*!< Set PH10 function to I2S0_LRCK          */
#define SET_I2S0_LRCK_PI10()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_I2S0_LRCK)                   /*!< Set PI10 function to I2S0_LRCK          */
#define SET_I2S0_MCLK_PA13()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_I2S0_MCLK)                   /*!< Set PA13 function to I2S0_MCLK          */
#define SET_I2S0_MCLK_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_I2S0_MCLK)                     /*!< Set PB4 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PC3()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_I2S0_MCLK)                     /*!< Set PC3 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PE0()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_I2S0_MCLK)                     /*!< Set PE0 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_I2S0_MCLK)                     /*!< Set PE2 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_I2S0_MCLK)                     /*!< Set PE9 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PF9()         SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_I2S0_MCLK)                     /*!< Set PF9 function to I2S0_MCLK           */
#define SET_I2S0_MCLK_PI7()         SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_I2S0_MCLK)                     /*!< Set PI7 function to I2S0_MCLK           */
#define SET_I2S1_BCLK_PA11()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_I2S1_BCLK)                   /*!< Set PA11 function to I2S1_BCLK          */
#define SET_I2S1_BCLK_PD14()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_I2S1_BCLK)                   /*!< Set PD14 function to I2S1_BCLK          */
#define SET_I2S1_BCLK_PD3()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_I2S1_BCLK)                     /*!< Set PD3 function to I2S1_BCLK           */
#define SET_I2S1_BCLK_PH7()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_I2S1_BCLK)                     /*!< Set PH7 function to I2S1_BCLK           */
#define SET_I2S1_DI_PA9()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_I2S1_DI)                       /*!< Set PA9 function to I2S1_DI             */
#define SET_I2S1_DI_PD1()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_I2S1_DI)                       /*!< Set PD1 function to I2S1_DI             */
#define SET_I2S1_DI_PG6()           SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_I2S1_DI)                       /*!< Set PG6 function to I2S1_DI             */
#define SET_I2S1_DO_PA8()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_I2S1_DO)                       /*!< Set PA8 function to I2S1_DO             */
#define SET_I2S1_DO_PD0()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_I2S1_DO)                       /*!< Set PD0 function to I2S1_DO             */
#define SET_I2S1_DO_PG0()           SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_I2S1_DO)                       /*!< Set PG0 function to I2S1_DO             */
#define SET_I2S1_LRCK_PB0()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_I2S1_LRCK)                     /*!< Set PB0 function to I2S1_LRCK           */
#define SET_I2S1_LRCK_PD13()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_I2S1_LRCK)                   /*!< Set PD13 function to I2S1_LRCK          */
#define SET_I2S1_LRCK_PG1()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_I2S1_LRCK)                     /*!< Set PG1 function to I2S1_LRCK           */
#define SET_I2S1_MCLK_PA10()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_I2S1_MCLK)                   /*!< Set PA10 function to I2S1_MCLK          */
#define SET_I2S1_MCLK_PD2()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_I2S1_MCLK)                     /*!< Set PD2 function to I2S1_MCLK           */
#define SET_I2S1_MCLK_PG5()         SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_I2S1_MCLK)                     /*!< Set PG5 function to I2S1_MCLK           */
#define SET_I3C0_PUPEN_PA2()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_I3C0_PUPEN)                    /*!< Set PA2 function to I3C0_PUPEN          */
#define SET_I3C0_PUPEN_PC5()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_I3C0_PUPEN)                    /*!< Set PC5 function to I3C0_PUPEN          */
#define SET_I3C0_PUPEN_PC9()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_I3C0_PUPEN)                    /*!< Set PC9 function to I3C0_PUPEN          */
#define SET_I3C0_PUPEN_PH10()       SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_I3C0_PUPEN)                  /*!< Set PH10 function to I3C0_PUPEN         */
#define SET_I3C0_SCL_PA1()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_I3C0_SCL)                      /*!< Set PA1 function to I3C0_SCL            */
#define SET_I3C0_SCL_PA5()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_I3C0_SCL)                      /*!< Set PA5 function to I3C0_SCL            */
#define SET_I3C0_SCL_PB1()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_I3C0_SCL)                      /*!< Set PB1 function to I3C0_SCL            */
#define SET_I3C0_SCL_PF0()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_I3C0_SCL)                      /*!< Set PF0 function to I3C0_SCL            */
#define SET_I3C0_SCL_PH8()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_I3C0_SCL)                      /*!< Set PH8 function to I3C0_SCL            */
#define SET_I3C0_SDA_PA0()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_I3C0_SDA)                      /*!< Set PA0 function to I3C0_SDA            */
#define SET_I3C0_SDA_PA4()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_I3C0_SDA)                      /*!< Set PA4 function to I3C0_SDA            */
#define SET_I3C0_SDA_PB0()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_I3C0_SDA)                      /*!< Set PB0 function to I3C0_SDA            */
#define SET_I3C0_SDA_PF1()          SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_I3C0_SDA)                      /*!< Set PF1 function to I3C0_SDA            */
#define SET_I3C0_SDA_PH9()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_I3C0_SDA)                      /*!< Set PH9 function to I3C0_SDA            */
#define SET_ICE_CLK_PF1()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_ICE_CLK)                       /*!< Set PF1 function to ICE_CLK             */
#define SET_ICE_DAT_PF0()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_ICE_DAT)                       /*!< Set PF0 function to ICE_DAT             */
#define SET_INT0_PA6()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_INT0)                          /*!< Set PA6 function to INT0                */
#define SET_INT0_PB5()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_INT0)                          /*!< Set PB5 function to INT0                */
#define SET_INT1_PA7()              SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_INT1)                          /*!< Set PA7 function to INT1                */
#define SET_INT1_PB4()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_INT1)                          /*!< Set PB4 function to INT1                */
#define SET_INT2_PB3()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_INT2)                          /*!< Set PB3 function to INT2                */
#define SET_INT2_PC6()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_INT2)                          /*!< Set PC6 function to INT2                */
#define SET_INT3_PB2()              SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_INT3)                          /*!< Set PB2 function to INT3                */
#define SET_INT3_PC7()              SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_INT3)                          /*!< Set PC7 function to INT3                */
#define SET_INT4_PA8()              SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_INT4)                          /*!< Set PA8 function to INT4                */
#define SET_INT4_PB6()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_INT4)                          /*!< Set PB6 function to INT4                */
#define SET_INT5_PB7()              SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_INT5)                          /*!< Set PB7 function to INT5                */
#define SET_INT5_PD12()             SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_INT5)                        /*!< Set PD12 function to INT5               */
#define SET_INT6_PB8()              SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_INT6)                          /*!< Set PB8 function to INT6                */
#define SET_INT6_PD11()             SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_INT6)                        /*!< Set PD11 function to INT6               */
#define SET_INT7_PB9()              SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_INT7)                          /*!< Set PB9 function to INT7                */
#define SET_INT7_PD10()             SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_INT7)                        /*!< Set PD10 function to INT7               */
#define SET_KPI_COL0_PA6()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_KPI_COL0)                      /*!< Set PA6 function to KPI_COL0            */
#define SET_KPI_COL0_PB15()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_KPI_COL0)                    /*!< Set PB15 function to KPI_COL0           */
#define SET_KPI_COL0_PF5()          SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_KPI_COL0)                      /*!< Set PF5 function to KPI_COL0            */
#define SET_KPI_COL1_PA7()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_KPI_COL1)                      /*!< Set PA7 function to KPI_COL1            */
#define SET_KPI_COL1_PB14()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_KPI_COL1)                    /*!< Set PB14 function to KPI_COL1           */
#define SET_KPI_COL1_PF4()          SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_KPI_COL1)                      /*!< Set PF4 function to KPI_COL1            */
#define SET_KPI_COL2_PB13()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_KPI_COL2)                    /*!< Set PB13 function to KPI_COL2           */
#define SET_KPI_COL2_PC6()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_KPI_COL2)                      /*!< Set PC6 function to KPI_COL2            */
#define SET_KPI_COL3_PB12()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_KPI_COL3)                    /*!< Set PB12 function to KPI_COL3           */
#define SET_KPI_COL3_PC7()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_KPI_COL3)                      /*!< Set PC7 function to KPI_COL3            */
#define SET_KPI_COL4_PB7()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_KPI_COL4)                      /*!< Set PB7 function to KPI_COL4            */
#define SET_KPI_COL4_PC8()          SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_KPI_COL4)                      /*!< Set PC8 function to KPI_COL4            */
#define SET_KPI_COL5_PB6()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_KPI_COL5)                      /*!< Set PB6 function to KPI_COL5            */
#define SET_KPI_COL5_PE13()         SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_KPI_COL5)                    /*!< Set PE13 function to KPI_COL5           */
#define SET_KPI_COL6_PB5()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_KPI_COL6)                      /*!< Set PB5 function to KPI_COL6            */
#define SET_KPI_COL6_PE12()         SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_KPI_COL6)                    /*!< Set PE12 function to KPI_COL6           */
#define SET_KPI_COL7_PB4()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_KPI_COL7)                      /*!< Set PB4 function to KPI_COL7            */
#define SET_KPI_COL7_PE11()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_KPI_COL7)                    /*!< Set PE11 function to KPI_COL7           */
#define SET_KPI_ROW0_PB3()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_KPI_ROW0)                      /*!< Set PB3 function to KPI_ROW0            */
#define SET_KPI_ROW0_PC5()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_KPI_ROW0)                      /*!< Set PC5 function to KPI_ROW0            */
#define SET_KPI_ROW1_PB2()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_KPI_ROW1)                      /*!< Set PB2 function to KPI_ROW1            */
#define SET_KPI_ROW1_PC4()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_KPI_ROW1)                      /*!< Set PC4 function to KPI_ROW1            */
#define SET_KPI_ROW2_PB1()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_KPI_ROW2)                      /*!< Set PB1 function to KPI_ROW2            */
#define SET_KPI_ROW2_PC3()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_KPI_ROW2)                      /*!< Set PC3 function to KPI_ROW2            */
#define SET_KPI_ROW3_PB0()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_KPI_ROW3)                      /*!< Set PB0 function to KPI_ROW3            */
#define SET_KPI_ROW3_PC2()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_KPI_ROW3)                      /*!< Set PC2 function to KPI_ROW3            */
#define SET_KPI_ROW4_PA11()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_KPI_ROW4)                    /*!< Set PA11 function to KPI_ROW4           */
#define SET_KPI_ROW4_PC1()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_KPI_ROW4)                      /*!< Set PC1 function to KPI_ROW4            */
#define SET_KPI_ROW5_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_KPI_ROW5)                    /*!< Set PA10 function to KPI_ROW5           */
#define SET_KPI_ROW5_PC0()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_KPI_ROW5)                      /*!< Set PC0 function to KPI_ROW5            */
#define SET_LPADC0_CH0_PB0()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_LPADC0_CH0)                    /*!< Set PB0 function to LPADC0_CH0          */
#define SET_LPADC0_CH10_PB10()      SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_LPADC0_CH10)                 /*!< Set PB10 function to LPADC0_CH10        */
#define SET_LPADC0_CH11_PB11()      SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_LPADC0_CH11)                 /*!< Set PB11 function to LPADC0_CH11        */
#define SET_LPADC0_CH12_PB12()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_LPADC0_CH12)                 /*!< Set PB12 function to LPADC0_CH12        */
#define SET_LPADC0_CH13_PB13()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_LPADC0_CH13)                 /*!< Set PB13 function to LPADC0_CH13        */
#define SET_LPADC0_CH14_PB14()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_LPADC0_CH14)                 /*!< Set PB14 function to LPADC0_CH14        */
#define SET_LPADC0_CH15_PB15()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_LPADC0_CH15)                 /*!< Set PB15 function to LPADC0_CH15        */
#define SET_LPADC0_CH16_PD10()      SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_LPADC0_CH16)                 /*!< Set PD10 function to LPADC0_CH16        */
#define SET_LPADC0_CH17_PD11()      SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_LPADC0_CH17)                 /*!< Set PD11 function to LPADC0_CH17        */
#define SET_LPADC0_CH18_PD12()      SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_LPADC0_CH18)                 /*!< Set PD12 function to LPADC0_CH18        */
#define SET_LPADC0_CH19_PC13()      SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_LPADC0_CH19)                 /*!< Set PC13 function to LPADC0_CH19        */
#define SET_LPADC0_CH1_PB1()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_LPADC0_CH1)                    /*!< Set PB1 function to LPADC0_CH1          */
#define SET_LPADC0_CH20_PA8()       SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_LPADC0_CH20)                   /*!< Set PA8 function to LPADC0_CH20         */
#define SET_LPADC0_CH21_PA9()       SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_LPADC0_CH21)                   /*!< Set PA9 function to LPADC0_CH21         */
#define SET_LPADC0_CH22_PA10()      SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_LPADC0_CH22)                 /*!< Set PA10 function to LPADC0_CH22        */
#define SET_LPADC0_CH23_PA11()      SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_LPADC0_CH23)                 /*!< Set PA11 function to LPADC0_CH23        */
#define SET_LPADC0_CH2_PB2()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_LPADC0_CH2)                    /*!< Set PB2 function to LPADC0_CH2          */
#define SET_LPADC0_CH3_PB3()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_LPADC0_CH3)                    /*!< Set PB3 function to LPADC0_CH3          */
#define SET_LPADC0_CH4_PB4()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_LPADC0_CH4)                    /*!< Set PB4 function to LPADC0_CH4          */
#define SET_LPADC0_CH5_PB5()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_LPADC0_CH5)                    /*!< Set PB5 function to LPADC0_CH5          */
#define SET_LPADC0_CH6_PB6()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_LPADC0_CH6)                    /*!< Set PB6 function to LPADC0_CH6          */
#define SET_LPADC0_CH7_PB7()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_LPADC0_CH7)                    /*!< Set PB7 function to LPADC0_CH7          */
#define SET_LPADC0_CH8_PB8()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_LPADC0_CH8)                    /*!< Set PB8 function to LPADC0_CH8          */
#define SET_LPADC0_CH9_PB9()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_LPADC0_CH9)                    /*!< Set PB9 function to LPADC0_CH9          */
#define SET_LPADC0_ST_PC1()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_LPADC0_ST)                     /*!< Set PC1 function to LPADC0_ST           */
#define SET_LPADC0_ST_PC13()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_LPADC0_ST)                   /*!< Set PC13 function to LPADC0_ST          */
#define SET_LPADC0_ST_PD12()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_LPADC0_ST)                   /*!< Set PD12 function to LPADC0_ST          */
#define SET_LPADC0_ST_PD13()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_LPADC0_ST)                   /*!< Set PD13 function to LPADC0_ST          */
#define SET_LPADC0_ST_PD5()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_LPADC0_ST)                     /*!< Set PD5 function to LPADC0_ST           */
#define SET_LPADC0_ST_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_LPADC0_ST)                     /*!< Set PF0 function to LPADC0_ST           */
#define SET_LPADC0_ST_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_LPADC0_ST)                     /*!< Set PF5 function to LPADC0_ST           */
#define SET_LPI2C0_SCL_PA5()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_LPI2C0_SCL)                    /*!< Set PA5 function to LPI2C0_SCL          */
#define SET_LPI2C0_SCL_PB5()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_LPI2C0_SCL)                    /*!< Set PB5 function to LPI2C0_SCL          */
#define SET_LPI2C0_SCL_PC1()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_LPI2C0_SCL)                    /*!< Set PC1 function to LPI2C0_SCL          */
#define SET_LPI2C0_SCL_PC12()       SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_LPI2C0_SCL)                  /*!< Set PC12 function to LPI2C0_SCL         */
#define SET_LPI2C0_SCL_PE13()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_LPI2C0_SCL)                  /*!< Set PE13 function to LPI2C0_SCL         */
#define SET_LPI2C0_SCL_PF3()        SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_LPI2C0_SCL)                    /*!< Set PF3 function to LPI2C0_SCL          */
#define SET_LPI2C0_SCL_PG13()       SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_LPI2C0_SCL)                  /*!< Set PG13 function to LPI2C0_SCL         */
#define SET_LPI2C0_SDA_PA4()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_LPI2C0_SDA)                    /*!< Set PA4 function to LPI2C0_SDA          */
#define SET_LPI2C0_SDA_PB4()        SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_LPI2C0_SDA)                    /*!< Set PB4 function to LPI2C0_SDA          */
#define SET_LPI2C0_SDA_PC0()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_LPI2C0_SDA)                    /*!< Set PC0 function to LPI2C0_SDA          */
#define SET_LPI2C0_SDA_PC11()       SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_LPI2C0_SDA)                  /*!< Set PC11 function to LPI2C0_SDA         */
#define SET_LPI2C0_SDA_PC8()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_LPI2C0_SDA)                    /*!< Set PC8 function to LPI2C0_SDA          */
#define SET_LPI2C0_SDA_PF2()        SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_LPI2C0_SDA)                    /*!< Set PF2 function to LPI2C0_SDA          */
#define SET_LPI2C0_SDA_PG14()       SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_LPI2C0_SDA)                  /*!< Set PG14 function to LPI2C0_SDA         */
#define SET_LPIO0_PA0()             SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_LPIO0)                         /*!< Set PA0 function to LPIO0               */
#define SET_LPIO0_PE0()             SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_LPIO0)                         /*!< Set PE0 function to LPIO0               */
#define SET_LPIO1_PA1()             SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_LPIO1)                         /*!< Set PA1 function to LPIO1               */
#define SET_LPIO1_PE1()             SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_LPIO1)                         /*!< Set PE1 function to LPIO1               */
#define SET_LPIO2_PB0()             SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_LPIO2)                         /*!< Set PB0 function to LPIO2               */
#define SET_LPIO2_PF0()             SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_LPIO2)                         /*!< Set PF0 function to LPIO2               */
#define SET_LPIO3_PB1()             SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_LPIO3)                         /*!< Set PB1 function to LPIO3               */
#define SET_LPIO3_PF1()             SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_LPIO3)                         /*!< Set PF1 function to LPIO3               */
#define SET_LPIO4_PA6()             SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_LPIO4)                         /*!< Set PA6 function to LPIO4               */
#define SET_LPIO4_PC0()             SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_LPIO4)                         /*!< Set PC0 function to LPIO4               */
#define SET_LPIO5_PA7()             SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_LPIO5)                         /*!< Set PA7 function to LPIO5               */
#define SET_LPIO5_PC1()             SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_LPIO5)                         /*!< Set PC1 function to LPIO5               */
#define SET_LPIO6_PB2()             SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_LPIO6)                         /*!< Set PB2 function to LPIO6               */
#define SET_LPIO6_PD0()             SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_LPIO6)                         /*!< Set PD0 function to LPIO6               */
#define SET_LPIO7_PB3()             SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_LPIO7)                         /*!< Set PB3 function to LPIO7               */
#define SET_LPIO7_PD1()             SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_LPIO7)                         /*!< Set PD1 function to LPIO7               */
#define SET_LPSPI0_CLK_PA13()       SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_LPSPI0_CLK)                  /*!< Set PA13 function to LPSPI0_CLK         */
#define SET_LPSPI0_CLK_PA2()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_LPSPI0_CLK)                    /*!< Set PA2 function to LPSPI0_CLK          */
#define SET_LPSPI0_CLK_PB14()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_LPSPI0_CLK)                  /*!< Set PB14 function to LPSPI0_CLK         */
#define SET_LPSPI0_CLK_PD2()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_LPSPI0_CLK)                    /*!< Set PD2 function to LPSPI0_CLK          */
#define SET_LPSPI0_MISO_PA1()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_LPSPI0_MISO)                   /*!< Set PA1 function to LPSPI0_MISO         */
#define SET_LPSPI0_MISO_PA14()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_LPSPI0_MISO)                 /*!< Set PA14 function to LPSPI0_MISO        */
#define SET_LPSPI0_MISO_PB13()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_LPSPI0_MISO)                 /*!< Set PB13 function to LPSPI0_MISO        */
#define SET_LPSPI0_MISO_PD1()       SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_LPSPI0_MISO)                   /*!< Set PD1 function to LPSPI0_MISO         */
#define SET_LPSPI0_MOSI_PA0()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_LPSPI0_MOSI)                   /*!< Set PA0 function to LPSPI0_MOSI         */
#define SET_LPSPI0_MOSI_PA15()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_LPSPI0_MOSI)                 /*!< Set PA15 function to LPSPI0_MOSI        */
#define SET_LPSPI0_MOSI_PB12()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_LPSPI0_MOSI)                 /*!< Set PB12 function to LPSPI0_MOSI        */
#define SET_LPSPI0_MOSI_PD0()       SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_LPSPI0_MOSI)                   /*!< Set PD0 function to LPSPI0_MOSI         */
#define SET_LPSPI0_SS_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_LPSPI0_SS)                   /*!< Set PA12 function to LPSPI0_SS          */
#define SET_LPSPI0_SS_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_LPSPI0_SS)                     /*!< Set PA3 function to LPSPI0_SS           */
#define SET_LPSPI0_SS_PB15()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_LPSPI0_SS)                   /*!< Set PB15 function to LPSPI0_SS          */
#define SET_LPSPI0_SS_PD3()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_LPSPI0_SS)                     /*!< Set PD3 function to LPSPI0_SS           */
#define SET_LPTM0_EXT_PA11()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_LPTM0_EXT)                   /*!< Set PA11 function to LPTM0_EXT          */
#define SET_LPTM0_EXT_PB15()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_LPTM0_EXT)                   /*!< Set PB15 function to LPTM0_EXT          */
#define SET_LPTM0_PB5()             SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_LPTM0)                         /*!< Set PB5 function to LPTM0               */
#define SET_LPTM0_PC7()             SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_LPTM0)                         /*!< Set PC7 function to LPTM0               */
#define SET_LPTM0_PG2()             SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_LPTM0)                         /*!< Set PG2 function to LPTM0               */
#define SET_LPTM1_EXT_PA10()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_LPTM1_EXT)                   /*!< Set PA10 function to LPTM1_EXT          */
#define SET_LPTM1_EXT_PB14()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_LPTM1_EXT)                   /*!< Set PB14 function to LPTM1_EXT          */
#define SET_LPTM1_PB13()            SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_LPTM1)                       /*!< Set PB13 function to LPTM1              */
#define SET_LPTM1_PB4()             SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_LPTM1)                         /*!< Set PB4 function to LPTM1               */
#define SET_LPTM1_PC6()             SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_LPTM1)                         /*!< Set PC6 function to LPTM1               */
#define SET_LPTM1_PG3()             SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_LPTM1)                         /*!< Set PG3 function to LPTM1               */
#define SET_LPUART0_RXD_PA0()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_LPUART0_RXD)                   /*!< Set PA0 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PA4()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_LPUART0_RXD)                   /*!< Set PA4 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PA6()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_LPUART0_RXD)                   /*!< Set PA6 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PB12()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_LPUART0_RXD)                 /*!< Set PB12 function to LPUART0_RXD        */
#define SET_LPUART0_RXD_PB8()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_LPUART0_RXD)                   /*!< Set PB8 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PC11()      SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_LPUART0_RXD)                 /*!< Set PC11 function to LPUART0_RXD        */
#define SET_LPUART0_RXD_PD2()       SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_LPUART0_RXD)                   /*!< Set PD2 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PF1()       SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_LPUART0_RXD)                   /*!< Set PF1 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PF2()       SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_LPUART0_RXD)                   /*!< Set PF2 function to LPUART0_RXD         */
#define SET_LPUART0_RXD_PH11()      SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_LPUART0_RXD)                 /*!< Set PH11 function to LPUART0_RXD        */
#define SET_LPUART0_TXD_PA1()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_LPUART0_TXD)                   /*!< Set PA1 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PA5()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_LPUART0_TXD)                   /*!< Set PA5 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PA7()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_LPUART0_TXD)                   /*!< Set PA7 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PB13()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_LPUART0_TXD)                 /*!< Set PB13 function to LPUART0_TXD        */
#define SET_LPUART0_TXD_PB9()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_LPUART0_TXD)                   /*!< Set PB9 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PC12()      SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_LPUART0_TXD)                 /*!< Set PC12 function to LPUART0_TXD        */
#define SET_LPUART0_TXD_PD3()       SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_LPUART0_TXD)                   /*!< Set PD3 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PF0()       SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_LPUART0_TXD)                   /*!< Set PF0 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PF3()       SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_LPUART0_TXD)                   /*!< Set PF3 function to LPUART0_TXD         */
#define SET_LPUART0_TXD_PH10()      SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_LPUART0_TXD)                 /*!< Set PH10 function to LPUART0_TXD        */
#define SET_LPUART0_nCTS_PA5()      SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_LPUART0_nCTS)                  /*!< Set PA5 function to LPUART0_nCTS        */
#define SET_LPUART0_nCTS_PB11()     SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_LPUART0_nCTS)                /*!< Set PB11 function to LPUART0_nCTS       */
#define SET_LPUART0_nCTS_PB15()     SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_LPUART0_nCTS)                /*!< Set PB15 function to LPUART0_nCTS       */
#define SET_LPUART0_nCTS_PC7()      SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_LPUART0_nCTS)                  /*!< Set PC7 function to LPUART0_nCTS        */
#define SET_LPUART0_nRTS_PA4()      SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_LPUART0_nRTS)                  /*!< Set PA4 function to LPUART0_nRTS        */
#define SET_LPUART0_nRTS_PB10()     SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_LPUART0_nRTS)                /*!< Set PB10 function to LPUART0_nRTS       */
#define SET_LPUART0_nRTS_PB14()     SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_LPUART0_nRTS)                /*!< Set PB14 function to LPUART0_nRTS       */
#define SET_LPUART0_nRTS_PC6()      SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_LPUART0_nRTS)                  /*!< Set PC6 function to LPUART0_nRTS        */
#define SET_PSIO0_CH0_PB15()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_PSIO0_CH0)                   /*!< Set PB15 function to PSIO0_CH0          */
#define SET_PSIO0_CH0_PC5()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_PSIO0_CH0)                     /*!< Set PC5 function to PSIO0_CH0           */
#define SET_PSIO0_CH0_PE14()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_PSIO0_CH0)                   /*!< Set PE14 function to PSIO0_CH0          */
#define SET_PSIO0_CH0_PE7()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_PSIO0_CH0)                     /*!< Set PE7 function to PSIO0_CH0           */
#define SET_PSIO0_CH0_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_PSIO0_CH0)                     /*!< Set PF5 function to PSIO0_CH0           */
#define SET_PSIO0_CH1_PB14()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_PSIO0_CH1)                   /*!< Set PB14 function to PSIO0_CH1          */
#define SET_PSIO0_CH1_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_PSIO0_CH1)                     /*!< Set PC4 function to PSIO0_CH1           */
#define SET_PSIO0_CH1_PE15()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_PSIO0_CH1)                   /*!< Set PE15 function to PSIO0_CH1          */
#define SET_PSIO0_CH1_PE6()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_PSIO0_CH1)                     /*!< Set PE6 function to PSIO0_CH1           */
#define SET_PSIO0_CH1_PF4()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_PSIO0_CH1)                     /*!< Set PF4 function to PSIO0_CH1           */
#define SET_PSIO0_CH2_PB13()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_PSIO0_CH2)                   /*!< Set PB13 function to PSIO0_CH2          */
#define SET_PSIO0_CH2_PC3()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_PSIO0_CH2)                     /*!< Set PC3 function to PSIO0_CH2           */
#define SET_PSIO0_CH2_PD9()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_PSIO0_CH2)                     /*!< Set PD9 function to PSIO0_CH2           */
#define SET_PSIO0_CH2_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_PSIO0_CH2)                     /*!< Set PE5 function to PSIO0_CH2           */
#define SET_PSIO0_CH3_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_PSIO0_CH3)                   /*!< Set PB12 function to PSIO0_CH3          */
#define SET_PSIO0_CH3_PC2()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_PSIO0_CH3)                     /*!< Set PC2 function to PSIO0_CH3           */
#define SET_PSIO0_CH3_PD8()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_PSIO0_CH3)                     /*!< Set PD8 function to PSIO0_CH3           */
#define SET_PSIO0_CH3_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_PSIO0_CH3)                     /*!< Set PE4 function to PSIO0_CH3           */
#define SET_PSIO0_CH4_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_PSIO0_CH4)                   /*!< Set PA12 function to PSIO0_CH4          */
#define SET_PSIO0_CH4_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_PSIO0_CH4)                     /*!< Set PA3 function to PSIO0_CH4           */
#define SET_PSIO0_CH4_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_PSIO0_CH4)                     /*!< Set PB5 function to PSIO0_CH4           */
#define SET_PSIO0_CH4_PG13()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_PSIO0_CH4)                   /*!< Set PG13 function to PSIO0_CH4          */
#define SET_PSIO0_CH5_PA13()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_PSIO0_CH5)                   /*!< Set PA13 function to PSIO0_CH5          */
#define SET_PSIO0_CH5_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_PSIO0_CH5)                     /*!< Set PA2 function to PSIO0_CH5           */
#define SET_PSIO0_CH5_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_PSIO0_CH5)                     /*!< Set PB4 function to PSIO0_CH5           */
#define SET_PSIO0_CH5_PG14()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_PSIO0_CH5)                   /*!< Set PG14 function to PSIO0_CH5          */
#define SET_PSIO0_CH6_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_PSIO0_CH6)                     /*!< Set PA1 function to PSIO0_CH6           */
#define SET_PSIO0_CH6_PA14()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_PSIO0_CH6)                   /*!< Set PA14 function to PSIO0_CH6          */
#define SET_PSIO0_CH6_PB3()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_PSIO0_CH6)                     /*!< Set PB3 function to PSIO0_CH6           */
#define SET_PSIO0_CH6_PG15()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_PSIO0_CH6)                   /*!< Set PG15 function to PSIO0_CH6          */
#define SET_PSIO0_CH7_PA0()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_PSIO0_CH7)                     /*!< Set PA0 function to PSIO0_CH7           */
#define SET_PSIO0_CH7_PA15()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_PSIO0_CH7)                   /*!< Set PA15 function to PSIO0_CH7          */
#define SET_PSIO0_CH7_PB2()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_PSIO0_CH7)                     /*!< Set PB2 function to PSIO0_CH7           */
#define SET_PSIO0_CH7_PD4()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_PSIO0_CH7)                     /*!< Set PD4 function to PSIO0_CH7           */
#define SET_QSPI0_CLK_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_QSPI0_CLK)                     /*!< Set PA2 function to QSPI0_CLK           */
#define SET_QSPI0_CLK_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_QSPI0_CLK)                   /*!< Set PB12 function to QSPI0_CLK          */
#define SET_QSPI0_CLK_PC2()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_QSPI0_CLK)                     /*!< Set PC2 function to QSPI0_CLK           */
#define SET_QSPI0_CLK_PF2()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_QSPI0_CLK)                     /*!< Set PF2 function to QSPI0_CLK           */
#define SET_QSPI0_CLK_PH8()         SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_QSPI0_CLK)                     /*!< Set PH8 function to QSPI0_CLK           */
#define SET_QSPI0_CLK_PI15()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_QSPI0_CLK)                   /*!< Set PI15 function to QSPI0_CLK          */
#define SET_QSPI0_MISO0_PA1()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_QSPI0_MISO0)                   /*!< Set PA1 function to QSPI0_MISO0         */
#define SET_QSPI0_MISO0_PC1()       SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_QSPI0_MISO0)                   /*!< Set PC1 function to QSPI0_MISO0         */
#define SET_QSPI0_MISO0_PE1()       SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_QSPI0_MISO0)                   /*!< Set PE1 function to QSPI0_MISO0         */
#define SET_QSPI0_MISO0_PE7()       SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_QSPI0_MISO0)                   /*!< Set PE7 function to QSPI0_MISO0         */
#define SET_QSPI0_MISO0_PJ0()       SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ0MFP_Msk)) | SYS_GPJ_MFP0_PJ0MFP_QSPI0_MISO0)                   /*!< Set PJ0 function to QSPI0_MISO0         */
#define SET_QSPI0_MISO1_PA5()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_QSPI0_MISO1)                   /*!< Set PA5 function to QSPI0_MISO1         */
#define SET_QSPI0_MISO1_PB1()       SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_QSPI0_MISO1)                   /*!< Set PB1 function to QSPI0_MISO1         */
#define SET_QSPI0_MISO1_PC5()       SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_QSPI0_MISO1)                   /*!< Set PC5 function to QSPI0_MISO1         */
#define SET_QSPI0_MISO1_PH10()      SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_QSPI0_MISO1)                 /*!< Set PH10 function to QSPI0_MISO1        */
#define SET_QSPI0_MISO1_PI12()      SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_QSPI0_MISO1)                 /*!< Set PI12 function to QSPI0_MISO1        */
#define SET_QSPI0_MOSI0_PA0()       SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_QSPI0_MOSI0)                   /*!< Set PA0 function to QSPI0_MOSI0         */
#define SET_QSPI0_MOSI0_PC0()       SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_QSPI0_MOSI0)                   /*!< Set PC0 function to QSPI0_MOSI0         */
#define SET_QSPI0_MOSI0_PE0()       SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_QSPI0_MOSI0)                   /*!< Set PE0 function to QSPI0_MOSI0         */
#define SET_QSPI0_MOSI0_PE6()       SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_QSPI0_MOSI0)                   /*!< Set PE6 function to QSPI0_MOSI0         */
#define SET_QSPI0_MOSI0_PJ1()       SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ1MFP_Msk)) | SYS_GPJ_MFP0_PJ1MFP_QSPI0_MOSI0)                   /*!< Set PJ1 function to QSPI0_MOSI0         */
#define SET_QSPI0_MOSI1_PA4()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_QSPI0_MOSI1)                   /*!< Set PA4 function to QSPI0_MOSI1         */
#define SET_QSPI0_MOSI1_PB0()       SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_QSPI0_MOSI1)                   /*!< Set PB0 function to QSPI0_MOSI1         */
#define SET_QSPI0_MOSI1_PC4()       SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_QSPI0_MOSI1)                   /*!< Set PC4 function to QSPI0_MOSI1         */
#define SET_QSPI0_MOSI1_PD14()      SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_QSPI0_MOSI1)                 /*!< Set PD14 function to QSPI0_MOSI1        */
#define SET_QSPI0_MOSI1_PH11()      SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_QSPI0_MOSI1)                 /*!< Set PH11 function to QSPI0_MOSI1        */
#define SET_QSPI0_MOSI1_PI13()      SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_QSPI0_MOSI1)                 /*!< Set PI13 function to QSPI0_MOSI1        */
#define SET_QSPI0_SS_PA3()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_QSPI0_SS)                      /*!< Set PA3 function to QSPI0_SS            */
#define SET_QSPI0_SS_PC3()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_QSPI0_SS)                      /*!< Set PC3 function to QSPI0_SS            */
#define SET_QSPI0_SS_PH9()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_QSPI0_SS)                      /*!< Set PH9 function to QSPI0_SS            */
#define SET_QSPI0_SS_PI14()         SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_QSPI0_SS)                    /*!< Set PI14 function to QSPI0_SS           */
#define SET_QSPI1_CLK_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_QSPI1_CLK)                     /*!< Set PC4 function to QSPI1_CLK           */
#define SET_QSPI1_CLK_PG12()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_QSPI1_CLK)                   /*!< Set PG12 function to QSPI1_CLK          */
#define SET_QSPI1_CLK_PH15()        SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH15MFP_Msk)) | SYS_GPH_MFP3_PH15MFP_QSPI1_CLK)                   /*!< Set PH15 function to QSPI1_CLK          */
#define SET_QSPI1_CLK_PJ3()         SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_QSPI1_CLK)                     /*!< Set PJ3 function to QSPI1_CLK           */
#define SET_QSPI1_MISO0_PA12()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_QSPI1_MISO0)                 /*!< Set PA12 function to QSPI1_MISO0        */
#define SET_QSPI1_MISO0_PC3()       SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_QSPI1_MISO0)                   /*!< Set PC3 function to QSPI1_MISO0         */
#define SET_QSPI1_MISO0_PD7()       SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_QSPI1_MISO0)                   /*!< Set PD7 function to QSPI1_MISO0         */
#define SET_QSPI1_MISO0_PG13()      SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_QSPI1_MISO0)                 /*!< Set PG13 function to QSPI1_MISO0        */
#define SET_QSPI1_MISO0_PJ4()       SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_QSPI1_MISO0)                   /*!< Set PJ4 function to QSPI1_MISO0         */
#define SET_QSPI1_MISO1_PA7()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_QSPI1_MISO1)                   /*!< Set PA7 function to QSPI1_MISO1         */
#define SET_QSPI1_MISO1_PG9()       SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_QSPI1_MISO1)                   /*!< Set PG9 function to QSPI1_MISO1         */
#define SET_QSPI1_MISO1_PH12()      SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_QSPI1_MISO1)                 /*!< Set PH12 function to QSPI1_MISO1        */
#define SET_QSPI1_MOSI0_PA13()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_QSPI1_MOSI0)                 /*!< Set PA13 function to QSPI1_MOSI0        */
#define SET_QSPI1_MOSI0_PC2()       SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_QSPI1_MOSI0)                   /*!< Set PC2 function to QSPI1_MOSI0         */
#define SET_QSPI1_MOSI0_PD13()      SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_QSPI1_MOSI0)                 /*!< Set PD13 function to QSPI1_MOSI0        */
#define SET_QSPI1_MOSI0_PD6()       SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_QSPI1_MOSI0)                   /*!< Set PD6 function to QSPI1_MOSI0         */
#define SET_QSPI1_MOSI0_PG14()      SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_QSPI1_MOSI0)                 /*!< Set PG14 function to QSPI1_MOSI0        */
#define SET_QSPI1_MOSI0_PJ5()       SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_QSPI1_MOSI0)                   /*!< Set PJ5 function to QSPI1_MOSI0         */
#define SET_QSPI1_MOSI1_PA6()       SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_QSPI1_MOSI1)                   /*!< Set PA6 function to QSPI1_MOSI1         */
#define SET_QSPI1_MOSI1_PG10()      SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_QSPI1_MOSI1)                 /*!< Set PG10 function to QSPI1_MOSI1        */
#define SET_QSPI1_MOSI1_PH13()      SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_QSPI1_MOSI1)                 /*!< Set PH13 function to QSPI1_MOSI1        */
#define SET_QSPI1_SS_PC5()          SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_QSPI1_SS)                      /*!< Set PC5 function to QSPI1_SS            */
#define SET_QSPI1_SS_PG11()         SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_QSPI1_SS)                    /*!< Set PG11 function to QSPI1_SS           */
#define SET_QSPI1_SS_PH14()         SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH14MFP_Msk)) | SYS_GPH_MFP3_PH14MFP_QSPI1_SS)                    /*!< Set PH14 function to QSPI1_SS           */
#define SET_QSPI1_SS_PJ2()          SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_QSPI1_SS)                      /*!< Set PJ2 function to QSPI1_SS            */
#define SET_SC0_CLK_PA0()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_SC0_CLK)                       /*!< Set PA0 function to SC0_CLK             */
#define SET_SC0_CLK_PB5()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_SC0_CLK)                       /*!< Set PB5 function to SC0_CLK             */
#define SET_SC0_CLK_PE2()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_SC0_CLK)                       /*!< Set PE2 function to SC0_CLK             */
#define SET_SC0_CLK_PF6()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_SC0_CLK)                       /*!< Set PF6 function to SC0_CLK             */
#define SET_SC0_DAT_PA1()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_SC0_DAT)                       /*!< Set PA1 function to SC0_DAT             */
#define SET_SC0_DAT_PB4()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_SC0_DAT)                       /*!< Set PB4 function to SC0_DAT             */
#define SET_SC0_DAT_PE3()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_SC0_DAT)                       /*!< Set PE3 function to SC0_DAT             */
#define SET_SC0_DAT_PF7()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_SC0_DAT)                       /*!< Set PF7 function to SC0_DAT             */
#define SET_SC0_PWR_PA3()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_SC0_PWR)                       /*!< Set PA3 function to SC0_PWR             */
#define SET_SC0_PWR_PB2()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_SC0_PWR)                       /*!< Set PB2 function to SC0_PWR             */
#define SET_SC0_PWR_PE5()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_SC0_PWR)                       /*!< Set PE5 function to SC0_PWR             */
#define SET_SC0_PWR_PF9()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_SC0_PWR)                       /*!< Set PF9 function to SC0_PWR             */
#define SET_SC0_RST_PA2()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_SC0_RST)                       /*!< Set PA2 function to SC0_RST             */
#define SET_SC0_RST_PB3()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_SC0_RST)                       /*!< Set PB3 function to SC0_RST             */
#define SET_SC0_RST_PE4()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_SC0_RST)                       /*!< Set PE4 function to SC0_RST             */
#define SET_SC0_RST_PF8()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_SC0_RST)                       /*!< Set PF8 function to SC0_RST             */
#define SET_SC0_nCD_PA4()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_SC0_nCD)                       /*!< Set PA4 function to SC0_nCD             */
#define SET_SC0_nCD_PC12()          SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_SC0_nCD)                     /*!< Set PC12 function to SC0_nCD            */
#define SET_SC0_nCD_PE6()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_SC0_nCD)                       /*!< Set PE6 function to SC0_nCD             */
#define SET_SC0_nCD_PF10()          SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_SC0_nCD)                     /*!< Set PF10 function to SC0_nCD            */
#define SET_SC1_CLK_PB12()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_SC1_CLK)                     /*!< Set PB12 function to SC1_CLK            */
#define SET_SC1_CLK_PC0()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_SC1_CLK)                       /*!< Set PC0 function to SC1_CLK             */
#define SET_SC1_CLK_PD4()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_SC1_CLK)                       /*!< Set PD4 function to SC1_CLK             */
#define SET_SC1_CLK_PF1()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_SC1_CLK)                       /*!< Set PF1 function to SC1_CLK             */
#define SET_SC1_CLK_PH1()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_SC1_CLK)                       /*!< Set PH1 function to SC1_CLK             */
#define SET_SC1_CLK_PI10()          SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_SC1_CLK)                     /*!< Set PI10 function to SC1_CLK            */
#define SET_SC1_DAT_PB13()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_SC1_DAT)                     /*!< Set PB13 function to SC1_DAT            */
#define SET_SC1_DAT_PC1()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_SC1_DAT)                       /*!< Set PC1 function to SC1_DAT             */
#define SET_SC1_DAT_PF0()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_SC1_DAT)                       /*!< Set PF0 function to SC1_DAT             */
#define SET_SC1_DAT_PG15()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_SC1_DAT)                     /*!< Set PG15 function to SC1_DAT            */
#define SET_SC1_DAT_PH0()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_SC1_DAT)                       /*!< Set PH0 function to SC1_DAT             */
#define SET_SC1_DAT_PI9()           SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_SC1_DAT)                       /*!< Set PI9 function to SC1_DAT             */
#define SET_SC1_PWR_PB15()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_SC1_PWR)                     /*!< Set PB15 function to SC1_PWR            */
#define SET_SC1_PWR_PC3()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_SC1_PWR)                       /*!< Set PC3 function to SC1_PWR             */
#define SET_SC1_PWR_PF3()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_SC1_PWR)                       /*!< Set PF3 function to SC1_PWR             */
#define SET_SC1_PWR_PG13()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_SC1_PWR)                     /*!< Set PG13 function to SC1_PWR            */
#define SET_SC1_PWR_PG5()           SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_SC1_PWR)                       /*!< Set PG5 function to SC1_PWR             */
#define SET_SC1_PWR_PI7()           SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_SC1_PWR)                       /*!< Set PI7 function to SC1_PWR             */
#define SET_SC1_RST_PB14()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_SC1_RST)                     /*!< Set PB14 function to SC1_RST            */
#define SET_SC1_RST_PC2()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_SC1_RST)                       /*!< Set PC2 function to SC1_RST             */
#define SET_SC1_RST_PF2()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_SC1_RST)                       /*!< Set PF2 function to SC1_RST             */
#define SET_SC1_RST_PG14()          SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_SC1_RST)                     /*!< Set PG14 function to SC1_RST            */
#define SET_SC1_RST_PG6()           SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_SC1_RST)                       /*!< Set PG6 function to SC1_RST             */
#define SET_SC1_RST_PI8()           SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_SC1_RST)                       /*!< Set PI8 function to SC1_RST             */
#define SET_SC1_nCD_PB6()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_SC1_nCD)                       /*!< Set PB6 function to SC1_nCD             */
#define SET_SC1_nCD_PC4()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_SC1_nCD)                       /*!< Set PC4 function to SC1_nCD             */
#define SET_SC1_nCD_PD14()          SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_SC1_nCD)                     /*!< Set PD14 function to SC1_nCD            */
#define SET_SC1_nCD_PD3()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_SC1_nCD)                       /*!< Set PD3 function to SC1_nCD             */
#define SET_SC1_nCD_PG2()           SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_SC1_nCD)                       /*!< Set PG2 function to SC1_nCD             */
#define SET_SC1_nCD_PI6()           SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_SC1_nCD)                       /*!< Set PI6 function to SC1_nCD             */
#define SET_SC2_CLK_PA15()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_SC2_CLK)                     /*!< Set PA15 function to SC2_CLK            */
#define SET_SC2_CLK_PA6()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_SC2_CLK)                       /*!< Set PA6 function to SC2_CLK             */
#define SET_SC2_CLK_PA8()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_SC2_CLK)                       /*!< Set PA8 function to SC2_CLK             */
#define SET_SC2_CLK_PD0()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_SC2_CLK)                       /*!< Set PD0 function to SC2_CLK             */
#define SET_SC2_CLK_PE0()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_SC2_CLK)                       /*!< Set PE0 function to SC2_CLK             */
#define SET_SC2_CLK_PE2()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_SC2_CLK)                       /*!< Set PE2 function to SC2_CLK             */
#define SET_SC2_DAT_PA14()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_SC2_DAT)                     /*!< Set PA14 function to SC2_DAT            */
#define SET_SC2_DAT_PA7()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_SC2_DAT)                       /*!< Set PA7 function to SC2_DAT             */
#define SET_SC2_DAT_PA9()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_SC2_DAT)                       /*!< Set PA9 function to SC2_DAT             */
#define SET_SC2_DAT_PD1()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_SC2_DAT)                       /*!< Set PD1 function to SC2_DAT             */
#define SET_SC2_DAT_PE1()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_SC2_DAT)                       /*!< Set PE1 function to SC2_DAT             */
#define SET_SC2_DAT_PE3()           SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_SC2_DAT)                       /*!< Set PE3 function to SC2_DAT             */
#define SET_SC2_PWR_PA11()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_SC2_PWR)                     /*!< Set PA11 function to SC2_PWR            */
#define SET_SC2_PWR_PA12()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_SC2_PWR)                     /*!< Set PA12 function to SC2_PWR            */
#define SET_SC2_PWR_PC7()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_SC2_PWR)                       /*!< Set PC7 function to SC2_PWR             */
#define SET_SC2_PWR_PD3()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_SC2_PWR)                       /*!< Set PD3 function to SC2_PWR             */
#define SET_SC2_PWR_PH8()           SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_SC2_PWR)                       /*!< Set PH8 function to SC2_PWR             */
#define SET_SC2_RST_PA10()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_SC2_RST)                     /*!< Set PA10 function to SC2_RST            */
#define SET_SC2_RST_PA13()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_SC2_RST)                     /*!< Set PA13 function to SC2_RST            */
#define SET_SC2_RST_PC6()           SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_SC2_RST)                       /*!< Set PC6 function to SC2_RST             */
#define SET_SC2_RST_PD2()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_SC2_RST)                       /*!< Set PD2 function to SC2_RST             */
#define SET_SC2_RST_PH9()           SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_SC2_RST)                       /*!< Set PH9 function to SC2_RST             */
#define SET_SC2_nCD_PA5()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_SC2_nCD)                       /*!< Set PA5 function to SC2_nCD             */
#define SET_SC2_nCD_PC13()          SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_SC2_nCD)                     /*!< Set PC13 function to SC2_nCD            */
#define SET_SC2_nCD_PD13()          SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_SC2_nCD)                     /*!< Set PD13 function to SC2_nCD            */
#define SET_SC2_nCD_PH10()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_SC2_nCD)                     /*!< Set PH10 function to SC2_nCD            */
#define SET_SD0_CLK_PB1()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_SD0_CLK)                       /*!< Set PB1 function to SD0_CLK             */
#define SET_SD0_CLK_PE6()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_SD0_CLK)                       /*!< Set PE6 function to SD0_CLK             */
#define SET_SD0_CMD_PB0()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_SD0_CMD)                       /*!< Set PB0 function to SD0_CMD             */
#define SET_SD0_CMD_PE7()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_SD0_CMD)                       /*!< Set PE7 function to SD0_CMD             */
#define SET_SD0_DAT0_PB2()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_SD0_DAT0)                      /*!< Set PB2 function to SD0_DAT0            */
#define SET_SD0_DAT0_PE2()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_SD0_DAT0)                      /*!< Set PE2 function to SD0_DAT0            */
#define SET_SD0_DAT1_PB3()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_SD0_DAT1)                      /*!< Set PB3 function to SD0_DAT1            */
#define SET_SD0_DAT1_PE3()          SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_SD0_DAT1)                      /*!< Set PE3 function to SD0_DAT1            */
#define SET_SD0_DAT2_PB4()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_SD0_DAT2)                      /*!< Set PB4 function to SD0_DAT2            */
#define SET_SD0_DAT2_PE4()          SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_SD0_DAT2)                      /*!< Set PE4 function to SD0_DAT2            */
#define SET_SD0_DAT3_PB5()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_SD0_DAT3)                      /*!< Set PB5 function to SD0_DAT3            */
#define SET_SD0_DAT3_PE5()          SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_SD0_DAT3)                      /*!< Set PE5 function to SD0_DAT3            */
#define SET_SD0_nCD_PB12()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_SD0_nCD)                     /*!< Set PB12 function to SD0_nCD            */
#define SET_SD0_nCD_PD13()          SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_SD0_nCD)                     /*!< Set PD13 function to SD0_nCD            */
#define SET_SD1_CLK_PA4()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_SD1_CLK)                       /*!< Set PA4 function to SD1_CLK             */
#define SET_SD1_CLK_PB6()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_SD1_CLK)                       /*!< Set PB6 function to SD1_CLK             */
#define SET_SD1_CLK_PD13()          SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_SD1_CLK)                     /*!< Set PD13 function to SD1_CLK            */
#define SET_SD1_CLK_PD6()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_SD1_CLK)                       /*!< Set PD6 function to SD1_CLK             */
#define SET_SD1_CLK_PJ13()          SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ13MFP_Msk)) | SYS_GPJ_MFP3_PJ13MFP_SD1_CLK)                     /*!< Set PJ13 function to SD1_CLK            */
#define SET_SD1_CMD_PA5()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_SD1_CMD)                       /*!< Set PA5 function to SD1_CMD             */
#define SET_SD1_CMD_PB7()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_SD1_CMD)                       /*!< Set PB7 function to SD1_CMD             */
#define SET_SD1_CMD_PD7()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_SD1_CMD)                       /*!< Set PD7 function to SD1_CMD             */
#define SET_SD1_CMD_PJ12()          SYS->GPJ_MFP3 = ((SYS->GPJ_MFP3 & (~SYS_GPJ_MFP3_PJ12MFP_Msk)) | SYS_GPJ_MFP3_PJ12MFP_SD1_CMD)                     /*!< Set PJ12 function to SD1_CMD            */
#define SET_SD1_DAT0_PA0()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_SD1_DAT0)                      /*!< Set PA0 function to SD1_DAT0            */
#define SET_SD1_DAT0_PA8()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_SD1_DAT0)                      /*!< Set PA8 function to SD1_DAT0            */
#define SET_SD1_DAT0_PG12()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_SD1_DAT0)                    /*!< Set PG12 function to SD1_DAT0           */
#define SET_SD1_DAT0_PJ11()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_SD1_DAT0)                    /*!< Set PJ11 function to SD1_DAT0           */
#define SET_SD1_DAT1_PA1()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_SD1_DAT1)                      /*!< Set PA1 function to SD1_DAT1            */
#define SET_SD1_DAT1_PA9()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_SD1_DAT1)                      /*!< Set PA9 function to SD1_DAT1            */
#define SET_SD1_DAT1_PG11()         SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_SD1_DAT1)                    /*!< Set PG11 function to SD1_DAT1           */
#define SET_SD1_DAT1_PJ10()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_SD1_DAT1)                    /*!< Set PJ10 function to SD1_DAT1           */
#define SET_SD1_DAT2_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_SD1_DAT2)                    /*!< Set PA10 function to SD1_DAT2           */
#define SET_SD1_DAT2_PA2()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_SD1_DAT2)                      /*!< Set PA2 function to SD1_DAT2            */
#define SET_SD1_DAT2_PG10()         SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG10MFP_Msk)) | SYS_GPG_MFP2_PG10MFP_SD1_DAT2)                    /*!< Set PG10 function to SD1_DAT2           */
#define SET_SD1_DAT2_PJ9()          SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ9MFP_Msk)) | SYS_GPJ_MFP2_PJ9MFP_SD1_DAT2)                      /*!< Set PJ9 function to SD1_DAT2            */
#define SET_SD1_DAT3_PA11()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_SD1_DAT3)                    /*!< Set PA11 function to SD1_DAT3           */
#define SET_SD1_DAT3_PA3()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_SD1_DAT3)                      /*!< Set PA3 function to SD1_DAT3            */
#define SET_SD1_DAT3_PG9()          SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG9MFP_Msk)) | SYS_GPG_MFP2_PG9MFP_SD1_DAT3)                      /*!< Set PG9 function to SD1_DAT3            */
#define SET_SD1_DAT3_PJ8()          SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ8MFP_Msk)) | SYS_GPJ_MFP2_PJ8MFP_SD1_DAT3)                      /*!< Set PJ8 function to SD1_DAT3            */
#define SET_SD1_nCD_PA12()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_SD1_nCD)                     /*!< Set PA12 function to SD1_nCD            */
#define SET_SD1_nCD_PA6()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_SD1_nCD)                       /*!< Set PA6 function to SD1_nCD             */
#define SET_SD1_nCD_PD5()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD5MFP_Msk)) | SYS_GPD_MFP1_PD5MFP_SD1_nCD)                       /*!< Set PD5 function to SD1_nCD             */
#define SET_SD1_nCD_PE14()          SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_SD1_nCD)                     /*!< Set PE14 function to SD1_nCD            */
#define SET_SPI0_CLK_PA2()          SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_SPI0_CLK)                      /*!< Set PA2 function to SPI0_CLK            */
#define SET_SPI0_CLK_PB14()         SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_SPI0_CLK)                    /*!< Set PB14 function to SPI0_CLK           */
#define SET_SPI0_CLK_PB6()          SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_SPI0_CLK)                      /*!< Set PB6 function to SPI0_CLK            */
#define SET_SPI0_CLK_PD2()          SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_SPI0_CLK)                      /*!< Set PD2 function to SPI0_CLK            */
#define SET_SPI0_CLK_PF8()          SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_SPI0_CLK)                      /*!< Set PF8 function to SPI0_CLK            */
#define SET_SPI0_I2SMCLK_PA4()      SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_SPI0_I2SMCLK)                  /*!< Set PA4 function to SPI0_I2SMCLK        */
#define SET_SPI0_I2SMCLK_PB0()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_SPI0_I2SMCLK)                  /*!< Set PB0 function to SPI0_I2SMCLK        */
#define SET_SPI0_I2SMCLK_PB11()     SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_SPI0_I2SMCLK)                /*!< Set PB11 function to SPI0_I2SMCLK       */
#define SET_SPI0_I2SMCLK_PB12()     SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_SPI0_I2SMCLK)                /*!< Set PB12 function to SPI0_I2SMCLK       */
#define SET_SPI0_I2SMCLK_PD13()     SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_SPI0_I2SMCLK)                /*!< Set PD13 function to SPI0_I2SMCLK       */
#define SET_SPI0_I2SMCLK_PD14()     SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_SPI0_I2SMCLK)                /*!< Set PD14 function to SPI0_I2SMCLK       */
#define SET_SPI0_I2SMCLK_PF10()     SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_SPI0_I2SMCLK)                /*!< Set PF10 function to SPI0_I2SMCLK       */
#define SET_SPI0_I2SMCLK_PG2()      SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_SPI0_I2SMCLK)                  /*!< Set PG2 function to SPI0_I2SMCLK        */
#define SET_SPI0_MISO_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_SPI0_MISO)                     /*!< Set PA1 function to SPI0_MISO           */
#define SET_SPI0_MISO_PB13()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_SPI0_MISO)                   /*!< Set PB13 function to SPI0_MISO          */
#define SET_SPI0_MISO_PD1()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_SPI0_MISO)                     /*!< Set PD1 function to SPI0_MISO           */
#define SET_SPI0_MISO_PF7()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_SPI0_MISO)                     /*!< Set PF7 function to SPI0_MISO           */
#define SET_SPI0_MOSI_PA0()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_SPI0_MOSI)                     /*!< Set PA0 function to SPI0_MOSI           */
#define SET_SPI0_MOSI_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_SPI0_MOSI)                   /*!< Set PB12 function to SPI0_MOSI          */
#define SET_SPI0_MOSI_PD0()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_SPI0_MOSI)                     /*!< Set PD0 function to SPI0_MOSI           */
#define SET_SPI0_MOSI_PF6()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_SPI0_MOSI)                     /*!< Set PF6 function to SPI0_MOSI           */
#define SET_SPI0_SS_PA3()           SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_SPI0_SS)                       /*!< Set PA3 function to SPI0_SS             */
#define SET_SPI0_SS_PB15()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_SPI0_SS)                     /*!< Set PB15 function to SPI0_SS            */
#define SET_SPI0_SS_PB7()           SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_SPI0_SS)                       /*!< Set PB7 function to SPI0_SS             */
#define SET_SPI0_SS_PD3()           SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_SPI0_SS)                       /*!< Set PD3 function to SPI0_SS             */
#define SET_SPI0_SS_PF9()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_SPI0_SS)                       /*!< Set PF9 function to SPI0_SS             */
#define SET_SPI1_CLK_PA7()          SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_SPI1_CLK)                      /*!< Set PA7 function to SPI1_CLK            */
#define SET_SPI1_CLK_PB3()          SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_SPI1_CLK)                      /*!< Set PB3 function to SPI1_CLK            */
#define SET_SPI1_CLK_PC1()          SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_SPI1_CLK)                      /*!< Set PC1 function to SPI1_CLK            */
#define SET_SPI1_CLK_PG15()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_SPI1_CLK)                    /*!< Set PG15 function to SPI1_CLK           */
#define SET_SPI1_CLK_PH6()          SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH6MFP_Msk)) | SYS_GPH_MFP1_PH6MFP_SPI1_CLK)                      /*!< Set PH6 function to SPI1_CLK            */
#define SET_SPI1_CLK_PH8()          SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_SPI1_CLK)                      /*!< Set PH8 function to SPI1_CLK            */
#define SET_SPI1_CLK_PI9()          SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_SPI1_CLK)                      /*!< Set PI9 function to SPI1_CLK            */
#define SET_SPI1_I2SMCLK_PA5()      SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_SPI1_I2SMCLK)                  /*!< Set PA5 function to SPI1_I2SMCLK        */
#define SET_SPI1_I2SMCLK_PB1()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_SPI1_I2SMCLK)                  /*!< Set PB1 function to SPI1_I2SMCLK        */
#define SET_SPI1_I2SMCLK_PC4()      SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_SPI1_I2SMCLK)                  /*!< Set PC4 function to SPI1_I2SMCLK        */
#define SET_SPI1_I2SMCLK_PD13()     SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_SPI1_I2SMCLK)                /*!< Set PD13 function to SPI1_I2SMCLK       */
#define SET_SPI1_I2SMCLK_PH10()     SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_SPI1_I2SMCLK)                /*!< Set PH10 function to SPI1_I2SMCLK       */
#define SET_SPI1_I2SMCLK_PH3()      SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_SPI1_I2SMCLK)                  /*!< Set PH3 function to SPI1_I2SMCLK        */
#define SET_SPI1_I2SMCLK_PI6()      SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_SPI1_I2SMCLK)                  /*!< Set PI6 function to SPI1_I2SMCLK        */
#define SET_SPI1_MISO_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_SPI1_MISO)                     /*!< Set PB5 function to SPI1_MISO           */
#define SET_SPI1_MISO_PC3()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_SPI1_MISO)                     /*!< Set PC3 function to SPI1_MISO           */
#define SET_SPI1_MISO_PC7()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_SPI1_MISO)                     /*!< Set PC7 function to SPI1_MISO           */
#define SET_SPI1_MISO_PE1()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_SPI1_MISO)                     /*!< Set PE1 function to SPI1_MISO           */
#define SET_SPI1_MISO_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_SPI1_MISO)                     /*!< Set PE5 function to SPI1_MISO           */
#define SET_SPI1_MISO_PG13()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_SPI1_MISO)                   /*!< Set PG13 function to SPI1_MISO          */
#define SET_SPI1_MISO_PH4()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_SPI1_MISO)                     /*!< Set PH4 function to SPI1_MISO           */
#define SET_SPI1_MISO_PI7()         SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_SPI1_MISO)                     /*!< Set PI7 function to SPI1_MISO           */
#define SET_SPI1_MOSI_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_SPI1_MOSI)                     /*!< Set PB4 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PC2()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_SPI1_MOSI)                     /*!< Set PC2 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PC6()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_SPI1_MOSI)                     /*!< Set PC6 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PE0()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_SPI1_MOSI)                     /*!< Set PE0 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_SPI1_MOSI)                     /*!< Set PE4 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PG14()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_SPI1_MOSI)                   /*!< Set PG14 function to SPI1_MOSI          */
#define SET_SPI1_MOSI_PH5()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_SPI1_MOSI)                     /*!< Set PH5 function to SPI1_MOSI           */
#define SET_SPI1_MOSI_PI8()         SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_SPI1_MOSI)                     /*!< Set PI8 function to SPI1_MOSI           */
#define SET_SPI1_SS_PA6()           SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_SPI1_SS)                       /*!< Set PA6 function to SPI1_SS             */
#define SET_SPI1_SS_PB2()           SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_SPI1_SS)                       /*!< Set PB2 function to SPI1_SS             */
#define SET_SPI1_SS_PC0()           SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_SPI1_SS)                       /*!< Set PC0 function to SPI1_SS             */
#define SET_SPI1_SS_PD4()           SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_SPI1_SS)                       /*!< Set PD4 function to SPI1_SS             */
#define SET_SPI1_SS_PH7()           SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_SPI1_SS)                       /*!< Set PH7 function to SPI1_SS             */
#define SET_SPI1_SS_PH9()           SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_SPI1_SS)                       /*!< Set PH9 function to SPI1_SS             */
#define SET_SPI1_SS_PI10()          SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_SPI1_SS)                     /*!< Set PI10 function to SPI1_SS            */
#define SET_SPI2_CLK_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_SPI2_CLK)                    /*!< Set PA10 function to SPI2_CLK           */
#define SET_SPI2_CLK_PA13()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_SPI2_CLK)                    /*!< Set PA13 function to SPI2_CLK           */
#define SET_SPI2_CLK_PE8()          SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_SPI2_CLK)                      /*!< Set PE8 function to SPI2_CLK            */
#define SET_SPI2_CLK_PG3()          SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_SPI2_CLK)                      /*!< Set PG3 function to SPI2_CLK            */
#define SET_SPI2_I2SMCLK_PB0()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_SPI2_I2SMCLK)                  /*!< Set PB0 function to SPI2_I2SMCLK        */
#define SET_SPI2_I2SMCLK_PC13()     SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_SPI2_I2SMCLK)                /*!< Set PC13 function to SPI2_I2SMCLK       */
#define SET_SPI2_I2SMCLK_PE12()     SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_SPI2_I2SMCLK)                /*!< Set PE12 function to SPI2_I2SMCLK       */
#define SET_SPI2_I2SMCLK_PG1()      SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_SPI2_I2SMCLK)                  /*!< Set PG1 function to SPI2_I2SMCLK        */
#define SET_SPI2_MISO_PA14()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_SPI2_MISO)                   /*!< Set PA14 function to SPI2_MISO          */
#define SET_SPI2_MISO_PA9()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_SPI2_MISO)                     /*!< Set PA9 function to SPI2_MISO           */
#define SET_SPI2_MISO_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_SPI2_MISO)                     /*!< Set PE9 function to SPI2_MISO           */
#define SET_SPI2_MISO_PG4()         SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG4MFP_Msk)) | SYS_GPG_MFP1_PG4MFP_SPI2_MISO)                     /*!< Set PG4 function to SPI2_MISO           */
#define SET_SPI2_MOSI_PA15()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_SPI2_MOSI)                   /*!< Set PA15 function to SPI2_MOSI          */
#define SET_SPI2_MOSI_PA8()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_SPI2_MOSI)                     /*!< Set PA8 function to SPI2_MOSI           */
#define SET_SPI2_MOSI_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_SPI2_MOSI)                   /*!< Set PE10 function to SPI2_MOSI          */
#define SET_SPI2_MOSI_PF11()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_SPI2_MOSI)                   /*!< Set PF11 function to SPI2_MOSI          */
#define SET_SPI2_SS_PA11()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_SPI2_SS)                     /*!< Set PA11 function to SPI2_SS            */
#define SET_SPI2_SS_PA12()          SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_SPI2_SS)                     /*!< Set PA12 function to SPI2_SS            */
#define SET_SPI2_SS_PE11()          SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_SPI2_SS)                     /*!< Set PE11 function to SPI2_SS            */
#define SET_SPI2_SS_PG2()           SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_SPI2_SS)                       /*!< Set PG2 function to SPI2_SS             */
#define SET_SPI3_CLK_PB11()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_SPI3_CLK)                    /*!< Set PB11 function to SPI3_CLK           */
#define SET_SPI3_CLK_PC10()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_SPI3_CLK)                    /*!< Set PC10 function to SPI3_CLK           */
#define SET_SPI3_CLK_PE4()          SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_SPI3_CLK)                      /*!< Set PE4 function to SPI3_CLK            */
#define SET_SPI3_CLK_PG6()          SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG6MFP_Msk)) | SYS_GPG_MFP1_PG6MFP_SPI3_CLK)                      /*!< Set PG6 function to SPI3_CLK            */
#define SET_SPI3_I2SMCLK_PB1()      SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_SPI3_I2SMCLK)                  /*!< Set PB1 function to SPI3_I2SMCLK        */
#define SET_SPI3_I2SMCLK_PD14()     SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD14MFP_Msk)) | SYS_GPD_MFP3_PD14MFP_SPI3_I2SMCLK)                /*!< Set PD14 function to SPI3_I2SMCLK       */
#define SET_SPI3_I2SMCLK_PE6()      SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_SPI3_I2SMCLK)                  /*!< Set PE6 function to SPI3_I2SMCLK        */
#define SET_SPI3_I2SMCLK_PF6()      SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_SPI3_I2SMCLK)                  /*!< Set PF6 function to SPI3_I2SMCLK        */
#define SET_SPI3_I2SMCLK_PH4()      SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_SPI3_I2SMCLK)                  /*!< Set PH4 function to SPI3_I2SMCLK        */
#define SET_SPI3_MISO_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_SPI3_MISO)                     /*!< Set PB9 function to SPI3_MISO           */
#define SET_SPI3_MISO_PC12()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_SPI3_MISO)                   /*!< Set PC12 function to SPI3_MISO          */
#define SET_SPI3_MISO_PE3()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_SPI3_MISO)                     /*!< Set PE3 function to SPI3_MISO           */
#define SET_SPI3_MISO_PH0()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_SPI3_MISO)                     /*!< Set PH0 function to SPI3_MISO           */
#define SET_SPI3_MOSI_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_SPI3_MOSI)                     /*!< Set PB8 function to SPI3_MOSI           */
#define SET_SPI3_MOSI_PC11()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_SPI3_MOSI)                   /*!< Set PC11 function to SPI3_MOSI          */
#define SET_SPI3_MOSI_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_SPI3_MOSI)                     /*!< Set PE2 function to SPI3_MOSI           */
#define SET_SPI3_MOSI_PH1()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_SPI3_MOSI)                     /*!< Set PH1 function to SPI3_MOSI           */
#define SET_SPI3_SS_PB10()          SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_SPI3_SS)                     /*!< Set PB10 function to SPI3_SS            */
#define SET_SPI3_SS_PC9()           SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_SPI3_SS)                       /*!< Set PC9 function to SPI3_SS             */
#define SET_SPI3_SS_PE5()           SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_SPI3_SS)                       /*!< Set PE5 function to SPI3_SS             */
#define SET_SPI3_SS_PG5()           SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG5MFP_Msk)) | SYS_GPG_MFP1_PG5MFP_SPI3_SS)                       /*!< Set PG5 function to SPI3_SS             */
#define SET_SPIM0_CLKN_PH12()       SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_SPIM0_CLKN)                  /*!< Set PH12 function to SPIM0_CLKN         */
#define SET_SPIM0_CLK_PH13()        SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_SPIM0_CLK)                   /*!< Set PH13 function to SPIM0_CLK          */
#define SET_SPIM0_D2_PJ5()          SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_SPIM0_D2)                      /*!< Set PJ5 function to SPIM0_D2            */
#define SET_SPIM0_D3_PJ6()          SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ6MFP_Msk)) | SYS_GPJ_MFP1_PJ6MFP_SPIM0_D3)                      /*!< Set PJ6 function to SPIM0_D3            */
#define SET_SPIM0_D4_PH14()         SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH14MFP_Msk)) | SYS_GPH_MFP3_PH14MFP_SPIM0_D4)                    /*!< Set PH14 function to SPIM0_D4           */
#define SET_SPIM0_D5_PH15()         SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH15MFP_Msk)) | SYS_GPH_MFP3_PH15MFP_SPIM0_D5)                    /*!< Set PH15 function to SPIM0_D5           */
#define SET_SPIM0_D6_PG13()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_SPIM0_D6)                    /*!< Set PG13 function to SPIM0_D6           */
#define SET_SPIM0_D7_PG14()         SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_SPIM0_D7)                    /*!< Set PG14 function to SPIM0_D7           */
#define SET_SPIM0_MISO_PJ4()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_SPIM0_MISO)                    /*!< Set PJ4 function to SPIM0_MISO          */
#define SET_SPIM0_MOSI_PJ3()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_SPIM0_MOSI)                    /*!< Set PJ3 function to SPIM0_MOSI          */
#define SET_SPIM0_RESETN_PJ2()      SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_SPIM0_RESETN)                  /*!< Set PJ2 function to SPIM0_RESETN        */
#define SET_SPIM0_RWDS_PG15()       SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG15MFP_Msk)) | SYS_GPG_MFP3_PG15MFP_SPIM0_RWDS)                  /*!< Set PG15 function to SPIM0_RWDS         */
#define SET_SPIM0_SS_PJ7()          SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ7MFP_Msk)) | SYS_GPJ_MFP1_PJ7MFP_SPIM0_SS)                      /*!< Set PJ7 function to SPIM0_SS            */
#define SET_SWDH_CLK_PA10()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_SWDH_CLK)                    /*!< Set PA10 function to SWDH_CLK           */
#define SET_SWDH_DAT_PA9()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_SWDH_DAT)                      /*!< Set PA9 function to SWDH_DAT            */
#define SET_SWODEC_SWO_PB12()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_SWODEC_SWO)                  /*!< Set PB12 function to SWODEC_SWO         */
#define SET_SWODEC_SWO_PC8()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_SWODEC_SWO)                    /*!< Set PC8 function to SWODEC_SWO          */
#define SET_SWODEC_SWO_PF4()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_SWODEC_SWO)                    /*!< Set PF4 function to SWODEC_SWO          */
#define SET_TAMPER0_PF6()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_TAMPER0)                       /*!< Set PF6 function to TAMPER0             */
#define SET_TAMPER1_PF7()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_TAMPER1)                       /*!< Set PF7 function to TAMPER1             */
#define SET_TAMPER2_PF8()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_TAMPER2)                       /*!< Set PF8 function to TAMPER2             */
#define SET_TAMPER3_PF9()           SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_TAMPER3)                       /*!< Set PF9 function to TAMPER3             */
#define SET_TAMPER4_PF10()          SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_TAMPER4)                     /*!< Set PF10 function to TAMPER4            */
#define SET_TAMPER5_PF11()          SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_TAMPER5)                     /*!< Set PF11 function to TAMPER5            */
#define SET_TM0_EXT_PA11()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_TM0_EXT)                     /*!< Set PA11 function to TM0_EXT            */
#define SET_TM0_EXT_PB15()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_TM0_EXT)                     /*!< Set PB15 function to TM0_EXT            */
#define SET_TM0_EXT_PF3()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_TM0_EXT)                       /*!< Set PF3 function to TM0_EXT             */
#define SET_TM0_EXT_PH0()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_TM0_EXT)                       /*!< Set PH0 function to TM0_EXT             */
#define SET_TM0_PB5()               SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_TM0)                           /*!< Set PB5 function to TM0                 */
#define SET_TM0_PC7()               SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_TM0)                           /*!< Set PC7 function to TM0                 */
#define SET_TM0_PG2()               SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG2MFP_Msk)) | SYS_GPG_MFP0_PG2MFP_TM0)                           /*!< Set PG2 function to TM0                 */
#define SET_TM1_EXT_PA10()          SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_TM1_EXT)                     /*!< Set PA10 function to TM1_EXT            */
#define SET_TM1_EXT_PB14()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_TM1_EXT)                     /*!< Set PB14 function to TM1_EXT            */
#define SET_TM1_EXT_PF2()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_TM1_EXT)                       /*!< Set PF2 function to TM1_EXT             */
#define SET_TM1_EXT_PH1()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_TM1_EXT)                       /*!< Set PH1 function to TM1_EXT             */
#define SET_TM1_PB4()               SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_TM1)                           /*!< Set PB4 function to TM1                 */
#define SET_TM1_PB6()               SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_TM1)                           /*!< Set PB6 function to TM1                 */
#define SET_TM1_PC6()               SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_TM1)                           /*!< Set PC6 function to TM1                 */
#define SET_TM1_PG3()               SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_TM1)                           /*!< Set PG3 function to TM1                 */
#define SET_TM2_EXT_PA9()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_TM2_EXT)                       /*!< Set PA9 function to TM2_EXT             */
#define SET_TM2_EXT_PB13()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_TM2_EXT)                     /*!< Set PB13 function to TM2_EXT            */
#define SET_TM2_EXT_PH2()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_TM2_EXT)                       /*!< Set PH2 function to TM2_EXT             */
#define SET_TM2_PA7()               SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_TM2)                           /*!< Set PA7 function to TM2                 */
#define SET_TM2_PB3()               SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_TM2)                           /*!< Set PB3 function to TM2                 */
#define SET_TM2_PD0()               SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_TM2)                           /*!< Set PD0 function to TM2                 */
#define SET_TM2_PG4()               SYS->GPG_MFP1 = ((SYS->GPG_MFP1 & (~SYS_GPG_MFP1_PG4MFP_Msk)) | SYS_GPG_MFP1_PG4MFP_TM2)                           /*!< Set PG4 function to TM2                 */
#define SET_TM3_EXT_PA8()           SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_TM3_EXT)                       /*!< Set PA8 function to TM3_EXT             */
#define SET_TM3_EXT_PB12()          SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_TM3_EXT)                     /*!< Set PB12 function to TM3_EXT            */
#define SET_TM3_EXT_PH3()           SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_TM3_EXT)                       /*!< Set PH3 function to TM3_EXT             */
#define SET_TM3_PA6()               SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_TM3)                           /*!< Set PA6 function to TM3                 */
#define SET_TM3_PB2()               SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_TM3)                           /*!< Set PB2 function to TM3                 */
#define SET_TM3_PF11()              SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_TM3)                         /*!< Set PF11 function to TM3                */
#define SET_TRACE_CLK_PE12()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_TRACE_CLK)                   /*!< Set PE12 function to TRACE_CLK          */
#define SET_TRACE_DATA0_PE11()      SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_TRACE_DATA0)                 /*!< Set PE11 function to TRACE_DATA0        */
#define SET_TRACE_DATA1_PE10()      SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_TRACE_DATA1)                 /*!< Set PE10 function to TRACE_DATA1        */
#define SET_TRACE_DATA2_PE9()       SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_TRACE_DATA2)                   /*!< Set PE9 function to TRACE_DATA2         */
#define SET_TRACE_DATA3_PE8()       SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_TRACE_DATA3)                   /*!< Set PE8 function to TRACE_DATA3         */
#define SET_TRACE_SWO_PB7()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_TRACE_SWO)                     /*!< Set PB7 function to TRACE_SWO           */
#define SET_TRACE_SWO_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_TRACE_SWO)                   /*!< Set PE13 function to TRACE_SWO          */
#define SET_TRACE_SWO_PF6()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_TRACE_SWO)                     /*!< Set PF6 function to TRACE_SWO           */
#define SET_UART0_RXD_PA0()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_UART0_RXD)                     /*!< Set PA0 function to UART0_RXD           */
#define SET_UART0_RXD_PA15()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_UART0_RXD)                   /*!< Set PA15 function to UART0_RXD          */
#define SET_UART0_RXD_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_UART0_RXD)                     /*!< Set PA4 function to UART0_RXD           */
#define SET_UART0_RXD_PA6()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_UART0_RXD)                     /*!< Set PA6 function to UART0_RXD           */
#define SET_UART0_RXD_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_UART0_RXD)                   /*!< Set PB12 function to UART0_RXD          */
#define SET_UART0_RXD_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_UART0_RXD)                     /*!< Set PB8 function to UART0_RXD           */
#define SET_UART0_RXD_PC11()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_UART0_RXD)                   /*!< Set PC11 function to UART0_RXD          */
#define SET_UART0_RXD_PD2()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_UART0_RXD)                     /*!< Set PD2 function to UART0_RXD           */
#define SET_UART0_RXD_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_UART0_RXD)                     /*!< Set PF1 function to UART0_RXD           */
#define SET_UART0_RXD_PF2()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_UART0_RXD)                     /*!< Set PF2 function to UART0_RXD           */
#define SET_UART0_RXD_PG3()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_UART0_RXD)                     /*!< Set PG3 function to UART0_RXD           */
#define SET_UART0_RXD_PH11()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_UART0_RXD)                   /*!< Set PH11 function to UART0_RXD          */
#define SET_UART0_TXD_PA1()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_UART0_TXD)                     /*!< Set PA1 function to UART0_TXD           */
#define SET_UART0_TXD_PA14()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_UART0_TXD)                   /*!< Set PA14 function to UART0_TXD          */
#define SET_UART0_TXD_PA5()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_UART0_TXD)                     /*!< Set PA5 function to UART0_TXD           */
#define SET_UART0_TXD_PA7()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_UART0_TXD)                     /*!< Set PA7 function to UART0_TXD           */
#define SET_UART0_TXD_PB13()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_UART0_TXD)                   /*!< Set PB13 function to UART0_TXD          */
#define SET_UART0_TXD_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_UART0_TXD)                     /*!< Set PB9 function to UART0_TXD           */
#define SET_UART0_TXD_PC12()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_UART0_TXD)                   /*!< Set PC12 function to UART0_TXD          */
#define SET_UART0_TXD_PD3()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_UART0_TXD)                     /*!< Set PD3 function to UART0_TXD           */
#define SET_UART0_TXD_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_UART0_TXD)                     /*!< Set PF0 function to UART0_TXD           */
#define SET_UART0_TXD_PF3()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_UART0_TXD)                     /*!< Set PF3 function to UART0_TXD           */
#define SET_UART0_TXD_PH10()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_UART0_TXD)                   /*!< Set PH10 function to UART0_TXD          */
#define SET_UART0_nCTS_PA5()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_UART0_nCTS)                    /*!< Set PA5 function to UART0_nCTS          */
#define SET_UART0_nCTS_PB11()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_UART0_nCTS)                  /*!< Set PB11 function to UART0_nCTS         */
#define SET_UART0_nCTS_PB15()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_UART0_nCTS)                  /*!< Set PB15 function to UART0_nCTS         */
#define SET_UART0_nCTS_PC7()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_UART0_nCTS)                    /*!< Set PC7 function to UART0_nCTS          */
#define SET_UART0_nCTS_PF5()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_UART0_nCTS)                    /*!< Set PF5 function to UART0_nCTS          */
#define SET_UART0_nRTS_PA4()        SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_UART0_nRTS)                    /*!< Set PA4 function to UART0_nRTS          */
#define SET_UART0_nRTS_PB10()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_UART0_nRTS)                  /*!< Set PB10 function to UART0_nRTS         */
#define SET_UART0_nRTS_PB14()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_UART0_nRTS)                  /*!< Set PB14 function to UART0_nRTS         */
#define SET_UART0_nRTS_PC6()        SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_UART0_nRTS)                    /*!< Set PC6 function to UART0_nRTS          */
#define SET_UART0_nRTS_PF4()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_UART0_nRTS)                    /*!< Set PF4 function to UART0_nRTS          */
#define SET_UART1_RXD_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_UART1_RXD)                     /*!< Set PA2 function to UART1_RXD           */
#define SET_UART1_RXD_PA8()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_UART1_RXD)                     /*!< Set PA8 function to UART1_RXD           */
#define SET_UART1_RXD_PB2()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_UART1_RXD)                     /*!< Set PB2 function to UART1_RXD           */
#define SET_UART1_RXD_PB6()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_UART1_RXD)                     /*!< Set PB6 function to UART1_RXD           */
#define SET_UART1_RXD_PC8()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_UART1_RXD)                     /*!< Set PC8 function to UART1_RXD           */
#define SET_UART1_RXD_PD10()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_UART1_RXD)                   /*!< Set PD10 function to UART1_RXD          */
#define SET_UART1_RXD_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_UART1_RXD)                     /*!< Set PF1 function to UART1_RXD           */
#define SET_UART1_RXD_PG1()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_UART1_RXD)                     /*!< Set PG1 function to UART1_RXD           */
#define SET_UART1_RXD_PG14()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG14MFP_Msk)) | SYS_GPG_MFP3_PG14MFP_UART1_RXD)                   /*!< Set PG14 function to UART1_RXD          */
#define SET_UART1_RXD_PH9()         SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_UART1_RXD)                     /*!< Set PH9 function to UART1_RXD           */
#define SET_UART1_TXD_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_UART1_TXD)                     /*!< Set PA3 function to UART1_TXD           */
#define SET_UART1_TXD_PA9()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_UART1_TXD)                     /*!< Set PA9 function to UART1_TXD           */
#define SET_UART1_TXD_PB3()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_UART1_TXD)                     /*!< Set PB3 function to UART1_TXD           */
#define SET_UART1_TXD_PB7()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_UART1_TXD)                     /*!< Set PB7 function to UART1_TXD           */
#define SET_UART1_TXD_PD11()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_UART1_TXD)                   /*!< Set PD11 function to UART1_TXD          */
#define SET_UART1_TXD_PE13()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_UART1_TXD)                   /*!< Set PE13 function to UART1_TXD          */
#define SET_UART1_TXD_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_UART1_TXD)                     /*!< Set PF0 function to UART1_TXD           */
#define SET_UART1_TXD_PG0()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_UART1_TXD)                     /*!< Set PG0 function to UART1_TXD           */
#define SET_UART1_TXD_PG13()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG13MFP_Msk)) | SYS_GPG_MFP3_PG13MFP_UART1_TXD)                   /*!< Set PG13 function to UART1_TXD          */
#define SET_UART1_TXD_PH8()         SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_UART1_TXD)                     /*!< Set PH8 function to UART1_TXD           */
#define SET_UART1_nCTS_PA1()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_UART1_nCTS)                    /*!< Set PA1 function to UART1_nCTS          */
#define SET_UART1_nCTS_PB9()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_UART1_nCTS)                    /*!< Set PB9 function to UART1_nCTS          */
#define SET_UART1_nCTS_PE11()       SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_UART1_nCTS)                  /*!< Set PE11 function to UART1_nCTS         */
#define SET_UART1_nRTS_PA0()        SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_UART1_nRTS)                    /*!< Set PA0 function to UART1_nRTS          */
#define SET_UART1_nRTS_PB8()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_UART1_nRTS)                    /*!< Set PB8 function to UART1_nRTS          */
#define SET_UART1_nRTS_PE12()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE12MFP_Msk)) | SYS_GPE_MFP3_PE12MFP_UART1_nRTS)                  /*!< Set PE12 function to UART1_nRTS         */
#define SET_UART2_RXD_PB0()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_UART2_RXD)                     /*!< Set PB0 function to UART2_RXD           */
#define SET_UART2_RXD_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_UART2_RXD)                     /*!< Set PB4 function to UART2_RXD           */
#define SET_UART2_RXD_PC0()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_UART2_RXD)                     /*!< Set PC0 function to UART2_RXD           */
#define SET_UART2_RXD_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_UART2_RXD)                     /*!< Set PC4 function to UART2_RXD           */
#define SET_UART2_RXD_PD12()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_UART2_RXD)                   /*!< Set PD12 function to UART2_RXD          */
#define SET_UART2_RXD_PE15()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_UART2_RXD)                   /*!< Set PE15 function to UART2_RXD          */
#define SET_UART2_RXD_PE9()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE9MFP_Msk)) | SYS_GPE_MFP2_PE9MFP_UART2_RXD)                     /*!< Set PE9 function to UART2_RXD           */
#define SET_UART2_RXD_PF1()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_UART2_RXD)                     /*!< Set PF1 function to UART2_RXD           */
#define SET_UART2_RXD_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_UART2_RXD)                     /*!< Set PF5 function to UART2_RXD           */
#define SET_UART2_RXD_PG0()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG0MFP_Msk)) | SYS_GPG_MFP0_PG0MFP_UART2_RXD)                     /*!< Set PG0 function to UART2_RXD           */
#define SET_UART2_RXD_PI11()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI11MFP_Msk)) | SYS_GPI_MFP2_PI11MFP_UART2_RXD)                   /*!< Set PI11 function to UART2_RXD          */
#define SET_UART2_RXD_PI7()         SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_UART2_RXD)                     /*!< Set PI7 function to UART2_RXD           */
#define SET_UART2_TXD_PB1()         SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_UART2_TXD)                     /*!< Set PB1 function to UART2_TXD           */
#define SET_UART2_TXD_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_UART2_TXD)                     /*!< Set PB5 function to UART2_TXD           */
#define SET_UART2_TXD_PC1()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_UART2_TXD)                     /*!< Set PC1 function to UART2_TXD           */
#define SET_UART2_TXD_PC13()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_UART2_TXD)                   /*!< Set PC13 function to UART2_TXD          */
#define SET_UART2_TXD_PC5()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_UART2_TXD)                     /*!< Set PC5 function to UART2_TXD           */
#define SET_UART2_TXD_PE14()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_UART2_TXD)                   /*!< Set PE14 function to UART2_TXD          */
#define SET_UART2_TXD_PE8()         SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE8MFP_Msk)) | SYS_GPE_MFP2_PE8MFP_UART2_TXD)                     /*!< Set PE8 function to UART2_TXD           */
#define SET_UART2_TXD_PF0()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_UART2_TXD)                     /*!< Set PF0 function to UART2_TXD           */
#define SET_UART2_TXD_PF4()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_UART2_TXD)                     /*!< Set PF4 function to UART2_TXD           */
#define SET_UART2_TXD_PG1()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG1MFP_Msk)) | SYS_GPG_MFP0_PG1MFP_UART2_TXD)                     /*!< Set PG1 function to UART2_TXD           */
#define SET_UART2_TXD_PI10()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI10MFP_Msk)) | SYS_GPI_MFP2_PI10MFP_UART2_TXD)                   /*!< Set PI10 function to UART2_TXD          */
#define SET_UART2_TXD_PI6()         SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_UART2_TXD)                     /*!< Set PI6 function to UART2_TXD           */
#define SET_UART2_nCTS_PC2()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_UART2_nCTS)                    /*!< Set PC2 function to UART2_nCTS          */
#define SET_UART2_nCTS_PD9()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_UART2_nCTS)                    /*!< Set PD9 function to UART2_nCTS          */
#define SET_UART2_nCTS_PF5()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_UART2_nCTS)                    /*!< Set PF5 function to UART2_nCTS          */
#define SET_UART2_nCTS_PI9()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI9MFP_Msk)) | SYS_GPI_MFP2_PI9MFP_UART2_nCTS)                    /*!< Set PI9 function to UART2_nCTS          */
#define SET_UART2_nRTS_PC3()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_UART2_nRTS)                    /*!< Set PC3 function to UART2_nRTS          */
#define SET_UART2_nRTS_PD8()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_UART2_nRTS)                    /*!< Set PD8 function to UART2_nRTS          */
#define SET_UART2_nRTS_PF4()        SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_UART2_nRTS)                    /*!< Set PF4 function to UART2_nRTS          */
#define SET_UART2_nRTS_PI8()        SYS->GPI_MFP2 = ((SYS->GPI_MFP2 & (~SYS_GPI_MFP2_PI8MFP_Msk)) | SYS_GPI_MFP2_PI8MFP_UART2_nRTS)                    /*!< Set PI8 function to UART2_nRTS          */
#define SET_UART3_RXD_PB14()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_UART3_RXD)                   /*!< Set PB14 function to UART3_RXD          */
#define SET_UART3_RXD_PC2()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_UART3_RXD)                     /*!< Set PC2 function to UART3_RXD           */
#define SET_UART3_RXD_PC9()         SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_UART3_RXD)                     /*!< Set PC9 function to UART3_RXD           */
#define SET_UART3_RXD_PD0()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_UART3_RXD)                     /*!< Set PD0 function to UART3_RXD           */
#define SET_UART3_RXD_PE0()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_UART3_RXD)                     /*!< Set PE0 function to UART3_RXD           */
#define SET_UART3_RXD_PE11()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE11MFP_Msk)) | SYS_GPE_MFP2_PE11MFP_UART3_RXD)                   /*!< Set PE11 function to UART3_RXD          */
#define SET_UART3_RXD_PE15()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_UART3_RXD)                   /*!< Set PE15 function to UART3_RXD          */
#define SET_UART3_RXD_PF4()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_UART3_RXD)                     /*!< Set PF4 function to UART3_RXD           */
#define SET_UART3_RXD_PI15()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_UART3_RXD)                   /*!< Set PI15 function to UART3_RXD          */
#define SET_UART3_TXD_PB15()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_UART3_TXD)                   /*!< Set PB15 function to UART3_TXD          */
#define SET_UART3_TXD_PC10()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_UART3_TXD)                   /*!< Set PC10 function to UART3_TXD          */
#define SET_UART3_TXD_PC3()         SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_UART3_TXD)                     /*!< Set PC3 function to UART3_TXD           */
#define SET_UART3_TXD_PD1()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_UART3_TXD)                     /*!< Set PD1 function to UART3_TXD           */
#define SET_UART3_TXD_PE1()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_UART3_TXD)                     /*!< Set PE1 function to UART3_TXD           */
#define SET_UART3_TXD_PE10()        SYS->GPE_MFP2 = ((SYS->GPE_MFP2 & (~SYS_GPE_MFP2_PE10MFP_Msk)) | SYS_GPE_MFP2_PE10MFP_UART3_TXD)                   /*!< Set PE10 function to UART3_TXD          */
#define SET_UART3_TXD_PE14()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_UART3_TXD)                   /*!< Set PE14 function to UART3_TXD          */
#define SET_UART3_TXD_PF5()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_UART3_TXD)                     /*!< Set PF5 function to UART3_TXD           */
#define SET_UART3_TXD_PI14()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_UART3_TXD)                   /*!< Set PI14 function to UART3_TXD          */
#define SET_UART3_nCTS_PB12()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_UART3_nCTS)                  /*!< Set PB12 function to UART3_nCTS         */
#define SET_UART3_nCTS_PD2()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_UART3_nCTS)                    /*!< Set PD2 function to UART3_nCTS          */
#define SET_UART3_nCTS_PH9()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_UART3_nCTS)                    /*!< Set PH9 function to UART3_nCTS          */
#define SET_UART3_nRTS_PB13()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_UART3_nRTS)                  /*!< Set PB13 function to UART3_nRTS         */
#define SET_UART3_nRTS_PD3()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_UART3_nRTS)                    /*!< Set PD3 function to UART3_nRTS          */
#define SET_UART3_nRTS_PH8()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_UART3_nRTS)                    /*!< Set PH8 function to UART3_nRTS          */
#define SET_UART4_RXD_PA13()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_UART4_RXD)                   /*!< Set PA13 function to UART4_RXD          */
#define SET_UART4_RXD_PA2()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_UART4_RXD)                     /*!< Set PA2 function to UART4_RXD           */
#define SET_UART4_RXD_PB10()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB10MFP_Msk)) | SYS_GPB_MFP2_PB10MFP_UART4_RXD)                   /*!< Set PB10 function to UART4_RXD          */
#define SET_UART4_RXD_PC4()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_UART4_RXD)                     /*!< Set PC4 function to UART4_RXD           */
#define SET_UART4_RXD_PC6()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_UART4_RXD)                     /*!< Set PC6 function to UART4_RXD           */
#define SET_UART4_RXD_PF6()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF6MFP_Msk)) | SYS_GPF_MFP1_PF6MFP_UART4_RXD)                     /*!< Set PF6 function to UART4_RXD           */
#define SET_UART4_RXD_PG3()         SYS->GPG_MFP0 = ((SYS->GPG_MFP0 & (~SYS_GPG_MFP0_PG3MFP_Msk)) | SYS_GPG_MFP0_PG3MFP_UART4_RXD)                     /*!< Set PG3 function to UART4_RXD           */
#define SET_UART4_RXD_PH11()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_UART4_RXD)                   /*!< Set PH11 function to UART4_RXD          */
#define SET_UART4_RXD_PH3()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_UART4_RXD)                     /*!< Set PH3 function to UART4_RXD           */
#define SET_UART4_RXD_PI13()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI13MFP_Msk)) | SYS_GPI_MFP3_PI13MFP_UART4_RXD)                   /*!< Set PI13 function to UART4_RXD          */
#define SET_UART4_TXD_PA12()        SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_UART4_TXD)                   /*!< Set PA12 function to UART4_TXD          */
#define SET_UART4_TXD_PA3()         SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_UART4_TXD)                     /*!< Set PA3 function to UART4_TXD           */
#define SET_UART4_TXD_PB11()        SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB11MFP_Msk)) | SYS_GPB_MFP2_PB11MFP_UART4_TXD)                   /*!< Set PB11 function to UART4_TXD          */
#define SET_UART4_TXD_PC5()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_UART4_TXD)                     /*!< Set PC5 function to UART4_TXD           */
#define SET_UART4_TXD_PC7()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_UART4_TXD)                     /*!< Set PC7 function to UART4_TXD           */
#define SET_UART4_TXD_PF7()         SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF7MFP_Msk)) | SYS_GPF_MFP1_PF7MFP_UART4_TXD)                     /*!< Set PF7 function to UART4_TXD           */
#define SET_UART4_TXD_PH10()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_UART4_TXD)                   /*!< Set PH10 function to UART4_TXD          */
#define SET_UART4_TXD_PH2()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_UART4_TXD)                     /*!< Set PH2 function to UART4_TXD           */
#define SET_UART4_TXD_PI12()        SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI12MFP_Msk)) | SYS_GPI_MFP3_PI12MFP_UART4_TXD)                   /*!< Set PI12 function to UART4_TXD          */
#define SET_UART4_nCTS_PC8()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC8MFP_Msk)) | SYS_GPC_MFP2_PC8MFP_UART4_nCTS)                    /*!< Set PC8 function to UART4_nCTS          */
#define SET_UART4_nCTS_PE1()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_UART4_nCTS)                    /*!< Set PE1 function to UART4_nCTS          */
#define SET_UART4_nCTS_PE14()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_UART4_nCTS)                  /*!< Set PE14 function to UART4_nCTS         */
#define SET_UART4_nRTS_PE0()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_UART4_nRTS)                    /*!< Set PE0 function to UART4_nRTS          */
#define SET_UART4_nRTS_PE13()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE13MFP_Msk)) | SYS_GPE_MFP3_PE13MFP_UART4_nRTS)                  /*!< Set PE13 function to UART4_nRTS         */
#define SET_UART4_nRTS_PE15()       SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_UART4_nRTS)                  /*!< Set PE15 function to UART4_nRTS         */
#define SET_UART5_RXD_PA4()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_UART5_RXD)                     /*!< Set PA4 function to UART5_RXD           */
#define SET_UART5_RXD_PB4()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB4MFP_Msk)) | SYS_GPB_MFP1_PB4MFP_UART5_RXD)                     /*!< Set PB4 function to UART5_RXD           */
#define SET_UART5_RXD_PE6()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_UART5_RXD)                     /*!< Set PE6 function to UART5_RXD           */
#define SET_UART5_RXD_PF10()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_UART5_RXD)                   /*!< Set PF10 function to UART5_RXD          */
#define SET_UART5_RXD_PH1()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH1MFP_Msk)) | SYS_GPH_MFP0_PH1MFP_UART5_RXD)                     /*!< Set PH1 function to UART5_RXD           */
#define SET_UART5_TXD_PA5()         SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_UART5_TXD)                     /*!< Set PA5 function to UART5_TXD           */
#define SET_UART5_TXD_PB5()         SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB5MFP_Msk)) | SYS_GPB_MFP1_PB5MFP_UART5_TXD)                     /*!< Set PB5 function to UART5_TXD           */
#define SET_UART5_TXD_PE7()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_UART5_TXD)                     /*!< Set PE7 function to UART5_TXD           */
#define SET_UART5_TXD_PF11()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_UART5_TXD)                   /*!< Set PF11 function to UART5_TXD          */
#define SET_UART5_TXD_PH0()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH0MFP_Msk)) | SYS_GPH_MFP0_PH0MFP_UART5_TXD)                     /*!< Set PH0 function to UART5_TXD           */
#define SET_UART5_nCTS_PB2()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB2MFP_Msk)) | SYS_GPB_MFP0_PB2MFP_UART5_nCTS)                    /*!< Set PB2 function to UART5_nCTS          */
#define SET_UART5_nCTS_PF8()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_UART5_nCTS)                    /*!< Set PF8 function to UART5_nCTS          */
#define SET_UART5_nCTS_PH3()        SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH3MFP_Msk)) | SYS_GPH_MFP0_PH3MFP_UART5_nCTS)                    /*!< Set PH3 function to UART5_nCTS          */
#define SET_UART5_nRTS_PB3()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB3MFP_Msk)) | SYS_GPB_MFP0_PB3MFP_UART5_nRTS)                    /*!< Set PB3 function to UART5_nRTS          */
#define SET_UART5_nRTS_PF9()        SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_UART5_nRTS)                    /*!< Set PF9 function to UART5_nRTS          */
#define SET_UART5_nRTS_PH2()        SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_UART5_nRTS)                    /*!< Set PH2 function to UART5_nRTS          */
#define SET_UART6_RXD_PA10()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_UART6_RXD)                   /*!< Set PA10 function to UART6_RXD          */
#define SET_UART6_RXD_PC11()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC11MFP_Msk)) | SYS_GPC_MFP2_PC11MFP_UART6_RXD)                   /*!< Set PC11 function to UART6_RXD          */
#define SET_UART6_RXD_PC6()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_UART6_RXD)                     /*!< Set PC6 function to UART6_RXD           */
#define SET_UART6_RXD_PD13()        SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD13MFP_Msk)) | SYS_GPD_MFP3_PD13MFP_UART6_RXD)                   /*!< Set PD13 function to UART6_RXD          */
#define SET_UART6_RXD_PD6()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD6MFP_Msk)) | SYS_GPD_MFP1_PD6MFP_UART6_RXD)                     /*!< Set PD6 function to UART6_RXD           */
#define SET_UART6_RXD_PE15()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_UART6_RXD)                   /*!< Set PE15 function to UART6_RXD          */
#define SET_UART6_RXD_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_UART6_RXD)                     /*!< Set PE4 function to UART6_RXD           */
#define SET_UART6_RXD_PH5()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_UART6_RXD)                     /*!< Set PH5 function to UART6_RXD           */
#define SET_UART6_RXD_PJ11()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ11MFP_Msk)) | SYS_GPJ_MFP2_PJ11MFP_UART6_RXD)                   /*!< Set PJ11 function to UART6_RXD          */
#define SET_UART6_TXD_PA11()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_UART6_TXD)                   /*!< Set PA11 function to UART6_TXD          */
#define SET_UART6_TXD_PC12()        SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC12MFP_Msk)) | SYS_GPC_MFP3_PC12MFP_UART6_TXD)                   /*!< Set PC12 function to UART6_TXD          */
#define SET_UART6_TXD_PC7()         SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC7MFP_Msk)) | SYS_GPC_MFP1_PC7MFP_UART6_TXD)                     /*!< Set PC7 function to UART6_TXD           */
#define SET_UART6_TXD_PD7()         SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD7MFP_Msk)) | SYS_GPD_MFP1_PD7MFP_UART6_TXD)                     /*!< Set PD7 function to UART6_TXD           */
#define SET_UART6_TXD_PE14()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_UART6_TXD)                   /*!< Set PE14 function to UART6_TXD          */
#define SET_UART6_TXD_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_UART6_TXD)                     /*!< Set PE5 function to UART6_TXD           */
#define SET_UART6_TXD_PH4()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_UART6_TXD)                     /*!< Set PH4 function to UART6_TXD           */
#define SET_UART6_TXD_PJ10()        SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ10MFP_Msk)) | SYS_GPJ_MFP2_PJ10MFP_UART6_TXD)                   /*!< Set PJ10 function to UART6_TXD          */
#define SET_UART6_nCTS_PC9()        SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC9MFP_Msk)) | SYS_GPC_MFP2_PC9MFP_UART6_nCTS)                    /*!< Set PC9 function to UART6_nCTS          */
#define SET_UART6_nCTS_PE2()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_UART6_nCTS)                    /*!< Set PE2 function to UART6_nCTS          */
#define SET_UART6_nRTS_PC10()       SYS->GPC_MFP2 = ((SYS->GPC_MFP2 & (~SYS_GPC_MFP2_PC10MFP_Msk)) | SYS_GPC_MFP2_PC10MFP_UART6_nRTS)                  /*!< Set PC10 function to UART6_nRTS         */
#define SET_UART6_nRTS_PE3()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_UART6_nRTS)                    /*!< Set PE3 function to UART6_nRTS          */
#define SET_UART7_RXD_PA8()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_UART7_RXD)                     /*!< Set PA8 function to UART7_RXD           */
#define SET_UART7_RXD_PB8()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_UART7_RXD)                     /*!< Set PB8 function to UART7_RXD           */
#define SET_UART7_RXD_PD8()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD8MFP_Msk)) | SYS_GPD_MFP2_PD8MFP_UART7_RXD)                     /*!< Set PD8 function to UART7_RXD           */
#define SET_UART7_RXD_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_UART7_RXD)                     /*!< Set PE2 function to UART7_RXD           */
#define SET_UART7_RXD_PG12()        SYS->GPG_MFP3 = ((SYS->GPG_MFP3 & (~SYS_GPG_MFP3_PG12MFP_Msk)) | SYS_GPG_MFP3_PG12MFP_UART7_RXD)                   /*!< Set PG12 function to UART7_RXD          */
#define SET_UART7_RXD_PH7()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_UART7_RXD)                     /*!< Set PH7 function to UART7_RXD           */
#define SET_UART7_RXD_PJ9()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ9MFP_Msk)) | SYS_GPJ_MFP2_PJ9MFP_UART7_RXD)                     /*!< Set PJ9 function to UART7_RXD           */
#define SET_UART7_TXD_PA9()         SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_UART7_TXD)                     /*!< Set PA9 function to UART7_TXD           */
#define SET_UART7_TXD_PB9()         SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_UART7_TXD)                     /*!< Set PB9 function to UART7_TXD           */
#define SET_UART7_TXD_PD9()         SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD9MFP_Msk)) | SYS_GPD_MFP2_PD9MFP_UART7_TXD)                     /*!< Set PD9 function to UART7_TXD           */
#define SET_UART7_TXD_PE3()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_UART7_TXD)                     /*!< Set PE3 function to UART7_TXD           */
#define SET_UART7_TXD_PG11()        SYS->GPG_MFP2 = ((SYS->GPG_MFP2 & (~SYS_GPG_MFP2_PG11MFP_Msk)) | SYS_GPG_MFP2_PG11MFP_UART7_TXD)                   /*!< Set PG11 function to UART7_TXD          */
#define SET_UART7_TXD_PH6()         SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH6MFP_Msk)) | SYS_GPH_MFP1_PH6MFP_UART7_TXD)                     /*!< Set PH6 function to UART7_TXD           */
#define SET_UART7_TXD_PJ8()         SYS->GPJ_MFP2 = ((SYS->GPJ_MFP2 & (~SYS_GPJ_MFP2_PJ8MFP_Msk)) | SYS_GPJ_MFP2_PJ8MFP_UART7_TXD)                     /*!< Set PJ8 function to UART7_TXD           */
#define SET_UART7_nCTS_PE4()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_UART7_nCTS)                    /*!< Set PE4 function to UART7_nCTS          */
#define SET_UART7_nCTS_PH5()        SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH5MFP_Msk)) | SYS_GPH_MFP1_PH5MFP_UART7_nCTS)                    /*!< Set PH5 function to UART7_nCTS          */
#define SET_UART7_nRTS_PE5()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_UART7_nRTS)                    /*!< Set PE5 function to UART7_nRTS          */
#define SET_UART7_nRTS_PH4()        SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH4MFP_Msk)) | SYS_GPH_MFP1_PH4MFP_UART7_nRTS)                    /*!< Set PH4 function to UART7_nRTS          */
#define SET_UART8_RXD_PD10()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD10MFP_Msk)) | SYS_GPD_MFP2_PD10MFP_UART8_RXD)                   /*!< Set PD10 function to UART8_RXD          */
#define SET_UART8_RXD_PE0()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE0MFP_Msk)) | SYS_GPE_MFP0_PE0MFP_UART8_RXD)                     /*!< Set PE0 function to UART8_RXD           */
#define SET_UART8_RXD_PE15()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE15MFP_Msk)) | SYS_GPE_MFP3_PE15MFP_UART8_RXD)                   /*!< Set PE15 function to UART8_RXD          */
#define SET_UART8_RXD_PJ1()         SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ1MFP_Msk)) | SYS_GPJ_MFP0_PJ1MFP_UART8_RXD)                     /*!< Set PJ1 function to UART8_RXD           */
#define SET_UART8_RXD_PJ5()         SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ5MFP_Msk)) | SYS_GPJ_MFP1_PJ5MFP_UART8_RXD)                     /*!< Set PJ5 function to UART8_RXD           */
#define SET_UART8_TXD_PD11()        SYS->GPD_MFP2 = ((SYS->GPD_MFP2 & (~SYS_GPD_MFP2_PD11MFP_Msk)) | SYS_GPD_MFP2_PD11MFP_UART8_TXD)                   /*!< Set PD11 function to UART8_TXD          */
#define SET_UART8_TXD_PE1()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE1MFP_Msk)) | SYS_GPE_MFP0_PE1MFP_UART8_TXD)                     /*!< Set PE1 function to UART8_TXD           */
#define SET_UART8_TXD_PE14()        SYS->GPE_MFP3 = ((SYS->GPE_MFP3 & (~SYS_GPE_MFP3_PE14MFP_Msk)) | SYS_GPE_MFP3_PE14MFP_UART8_TXD)                   /*!< Set PE14 function to UART8_TXD          */
#define SET_UART8_TXD_PJ0()         SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ0MFP_Msk)) | SYS_GPJ_MFP0_PJ0MFP_UART8_TXD)                     /*!< Set PJ0 function to UART8_TXD           */
#define SET_UART8_TXD_PJ4()         SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ4MFP_Msk)) | SYS_GPJ_MFP1_PJ4MFP_UART8_TXD)                     /*!< Set PJ4 function to UART8_TXD           */
#define SET_UART8_nCTS_PC13()       SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_UART8_nCTS)                  /*!< Set PC13 function to UART8_nCTS         */
#define SET_UART8_nCTS_PE3()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_UART8_nCTS)                    /*!< Set PE3 function to UART8_nCTS          */
#define SET_UART8_nCTS_PI14()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI14MFP_Msk)) | SYS_GPI_MFP3_PI14MFP_UART8_nCTS)                  /*!< Set PI14 function to UART8_nCTS         */
#define SET_UART8_nCTS_PJ2()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ2MFP_Msk)) | SYS_GPJ_MFP0_PJ2MFP_UART8_nCTS)                    /*!< Set PJ2 function to UART8_nCTS          */
#define SET_UART8_nRTS_PD12()       SYS->GPD_MFP3 = ((SYS->GPD_MFP3 & (~SYS_GPD_MFP3_PD12MFP_Msk)) | SYS_GPD_MFP3_PD12MFP_UART8_nRTS)                  /*!< Set PD12 function to UART8_nRTS         */
#define SET_UART8_nRTS_PE2()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_UART8_nRTS)                    /*!< Set PE2 function to UART8_nRTS          */
#define SET_UART8_nRTS_PI15()       SYS->GPI_MFP3 = ((SYS->GPI_MFP3 & (~SYS_GPI_MFP3_PI15MFP_Msk)) | SYS_GPI_MFP3_PI15MFP_UART8_nRTS)                  /*!< Set PI15 function to UART8_nRTS         */
#define SET_UART8_nRTS_PJ3()        SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ3MFP_Msk)) | SYS_GPJ_MFP0_PJ3MFP_UART8_nRTS)                    /*!< Set PJ3 function to UART8_nRTS          */
#define SET_UART9_RXD_PE4()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_UART9_RXD)                     /*!< Set PE4 function to UART9_RXD           */
#define SET_UART9_RXD_PF2()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_UART9_RXD)                     /*!< Set PF2 function to UART9_RXD           */
#define SET_UART9_RXD_PF8()         SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF8MFP_Msk)) | SYS_GPF_MFP2_PF8MFP_UART9_RXD)                     /*!< Set PF8 function to UART9_RXD           */
#define SET_UART9_RXD_PH11()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH11MFP_Msk)) | SYS_GPH_MFP2_PH11MFP_UART9_RXD)                   /*!< Set PH11 function to UART9_RXD          */
#define SET_UART9_RXD_PH13()        SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH13MFP_Msk)) | SYS_GPH_MFP3_PH13MFP_UART9_RXD)                   /*!< Set PH13 function to UART9_RXD          */
#define SET_UART9_RXD_PH2()         SYS->GPH_MFP0 = ((SYS->GPH_MFP0 & (~SYS_GPH_MFP0_PH2MFP_Msk)) | SYS_GPH_MFP0_PH2MFP_UART9_RXD)                     /*!< Set PH2 function to UART9_RXD           */
#define SET_UART9_TXD_PE5()         SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_UART9_TXD)                     /*!< Set PE5 function to UART9_TXD           */
#define SET_UART9_TXD_PF3()         SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_UART9_TXD)                     /*!< Set PF3 function to UART9_TXD           */
#define SET_UART9_TXD_PF9()         SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF9MFP_Msk)) | SYS_GPF_MFP2_PF9MFP_UART9_TXD)                     /*!< Set PF9 function to UART9_TXD           */
#define SET_UART9_TXD_PH10()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH10MFP_Msk)) | SYS_GPH_MFP2_PH10MFP_UART9_TXD)                   /*!< Set PH10 function to UART9_TXD          */
#define SET_UART9_TXD_PH12()        SYS->GPH_MFP3 = ((SYS->GPH_MFP3 & (~SYS_GPH_MFP3_PH12MFP_Msk)) | SYS_GPH_MFP3_PH12MFP_UART9_TXD)                   /*!< Set PH12 function to UART9_TXD          */
#define SET_UART9_nCTS_PE7()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE7MFP_Msk)) | SYS_GPE_MFP1_PE7MFP_UART9_nCTS)                    /*!< Set PE7 function to UART9_nCTS          */
#define SET_UART9_nCTS_PF11()       SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF11MFP_Msk)) | SYS_GPF_MFP2_PF11MFP_UART9_nCTS)                  /*!< Set PF11 function to UART9_nCTS         */
#define SET_UART9_nCTS_PH6()        SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH6MFP_Msk)) | SYS_GPH_MFP1_PH6MFP_UART9_nCTS)                    /*!< Set PH6 function to UART9_nCTS          */
#define SET_UART9_nCTS_PH8()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH8MFP_Msk)) | SYS_GPH_MFP2_PH8MFP_UART9_nCTS)                    /*!< Set PH8 function to UART9_nCTS          */
#define SET_UART9_nCTS_PJ6()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ6MFP_Msk)) | SYS_GPJ_MFP1_PJ6MFP_UART9_nCTS)                    /*!< Set PJ6 function to UART9_nCTS          */
#define SET_UART9_nRTS_PE6()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_UART9_nRTS)                    /*!< Set PE6 function to UART9_nRTS          */
#define SET_UART9_nRTS_PF10()       SYS->GPF_MFP2 = ((SYS->GPF_MFP2 & (~SYS_GPF_MFP2_PF10MFP_Msk)) | SYS_GPF_MFP2_PF10MFP_UART9_nRTS)                  /*!< Set PF10 function to UART9_nRTS         */
#define SET_UART9_nRTS_PH7()        SYS->GPH_MFP1 = ((SYS->GPH_MFP1 & (~SYS_GPH_MFP1_PH7MFP_Msk)) | SYS_GPH_MFP1_PH7MFP_UART9_nRTS)                    /*!< Set PH7 function to UART9_nRTS          */
#define SET_UART9_nRTS_PH9()        SYS->GPH_MFP2 = ((SYS->GPH_MFP2 & (~SYS_GPH_MFP2_PH9MFP_Msk)) | SYS_GPH_MFP2_PH9MFP_UART9_nRTS)                    /*!< Set PH9 function to UART9_nRTS          */
#define SET_UART9_nRTS_PJ7()        SYS->GPJ_MFP1 = ((SYS->GPJ_MFP1 & (~SYS_GPJ_MFP1_PJ7MFP_Msk)) | SYS_GPJ_MFP1_PJ7MFP_UART9_nRTS)                    /*!< Set PJ7 function to UART9_nRTS          */
#define SET_USB_D_MINUS_PA13()      SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA13MFP_Msk)) | SYS_GPA_MFP3_PA13MFP_USB_D_MINUS)                 /*!< Set PA13 function to USB_D_MINUS        */
#define SET_USB_D_PLUS_PA14()       SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA14MFP_Msk)) | SYS_GPA_MFP3_PA14MFP_USB_D_PLUS)                  /*!< Set PA14 function to USB_D_PLUS         */
#define SET_USB_OTG_ID_PA15()       SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA15MFP_Msk)) | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID)                  /*!< Set PA15 function to USB_OTG_ID         */
#define SET_USB_VBUS_EN_PB15()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_USB_VBUS_EN)                 /*!< Set PB15 function to USB_VBUS_EN        */
#define SET_USB_VBUS_EN_PB6()       SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB6MFP_Msk)) | SYS_GPB_MFP1_PB6MFP_USB_VBUS_EN)                   /*!< Set PB6 function to USB_VBUS_EN         */
#define SET_USB_VBUS_EN_PB8()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_USB_VBUS_EN)                   /*!< Set PB8 function to USB_VBUS_EN         */
#define SET_USB_VBUS_EN_PI7()       SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI7MFP_Msk)) | SYS_GPI_MFP1_PI7MFP_USB_VBUS_EN)                   /*!< Set PI7 function to USB_VBUS_EN         */
#define SET_USB_VBUS_PA12()         SYS->GPA_MFP3 = ((SYS->GPA_MFP3 & (~SYS_GPA_MFP3_PA12MFP_Msk)) | SYS_GPA_MFP3_PA12MFP_USB_VBUS)                    /*!< Set PA12 function to USB_VBUS           */
#define SET_USB_VBUS_ST_PB13()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_USB_VBUS_ST)                 /*!< Set PB13 function to USB_VBUS_ST        */
#define SET_USB_VBUS_ST_PB14()      SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_USB_VBUS_ST)                 /*!< Set PB14 function to USB_VBUS_ST        */
#define SET_USB_VBUS_ST_PB7()       SYS->GPB_MFP1 = ((SYS->GPB_MFP1 & (~SYS_GPB_MFP1_PB7MFP_Msk)) | SYS_GPB_MFP1_PB7MFP_USB_VBUS_ST)                   /*!< Set PB7 function to USB_VBUS_ST         */
#define SET_USB_VBUS_ST_PB8()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB8MFP_Msk)) | SYS_GPB_MFP2_PB8MFP_USB_VBUS_ST)                   /*!< Set PB8 function to USB_VBUS_ST         */
#define SET_USB_VBUS_ST_PB9()       SYS->GPB_MFP2 = ((SYS->GPB_MFP2 & (~SYS_GPB_MFP2_PB9MFP_Msk)) | SYS_GPB_MFP2_PB9MFP_USB_VBUS_ST)                   /*!< Set PB9 function to USB_VBUS_ST         */
#define SET_USB_VBUS_ST_PD4()       SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_USB_VBUS_ST)                   /*!< Set PD4 function to USB_VBUS_ST         */
#define SET_USB_VBUS_ST_PI6()       SYS->GPI_MFP1 = ((SYS->GPI_MFP1 & (~SYS_GPI_MFP1_PI6MFP_Msk)) | SYS_GPI_MFP1_PI6MFP_USB_VBUS_ST)                   /*!< Set PI6 function to USB_VBUS_ST         */
#define SET_USCI0_CLK_PA11()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA11MFP_Msk)) | SYS_GPA_MFP2_PA11MFP_USCI0_CLK)                   /*!< Set PA11 function to USCI0_CLK          */
#define SET_USCI0_CLK_PB12()        SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_USCI0_CLK)                   /*!< Set PB12 function to USCI0_CLK          */
#define SET_USCI0_CLK_PD0()         SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD0MFP_Msk)) | SYS_GPD_MFP0_PD0MFP_USCI0_CLK)                     /*!< Set PD0 function to USCI0_CLK           */
#define SET_USCI0_CLK_PE2()         SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE2MFP_Msk)) | SYS_GPE_MFP0_PE2MFP_USCI0_CLK)                     /*!< Set PE2 function to USCI0_CLK           */
#define SET_USCI0_CTL0_PB0()        SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_USCI0_CTL0)                    /*!< Set PB0 function to USCI0_CTL0          */
#define SET_USCI0_CTL0_PB13()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_USCI0_CTL0)                  /*!< Set PB13 function to USCI0_CTL0         */
#define SET_USCI0_CTL0_PC13()       SYS->GPC_MFP3 = ((SYS->GPC_MFP3 & (~SYS_GPC_MFP3_PC13MFP_Msk)) | SYS_GPC_MFP3_PC13MFP_USCI0_CTL0)                  /*!< Set PC13 function to USCI0_CTL0         */
#define SET_USCI0_CTL0_PD4()        SYS->GPD_MFP1 = ((SYS->GPD_MFP1 & (~SYS_GPD_MFP1_PD4MFP_Msk)) | SYS_GPD_MFP1_PD4MFP_USCI0_CTL0)                    /*!< Set PD4 function to USCI0_CTL0          */
#define SET_USCI0_CTL0_PE6()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE6MFP_Msk)) | SYS_GPE_MFP1_PE6MFP_USCI0_CTL0)                    /*!< Set PE6 function to USCI0_CTL0          */
#define SET_USCI0_CTL1_PA8()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA8MFP_Msk)) | SYS_GPA_MFP2_PA8MFP_USCI0_CTL1)                    /*!< Set PA8 function to USCI0_CTL1          */
#define SET_USCI0_CTL1_PB15()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_USCI0_CTL1)                  /*!< Set PB15 function to USCI0_CTL1         */
#define SET_USCI0_CTL1_PD3()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD3MFP_Msk)) | SYS_GPD_MFP0_PD3MFP_USCI0_CTL1)                    /*!< Set PD3 function to USCI0_CTL1          */
#define SET_USCI0_CTL1_PE5()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE5MFP_Msk)) | SYS_GPE_MFP1_PE5MFP_USCI0_CTL1)                    /*!< Set PE5 function to USCI0_CTL1          */
#define SET_USCI0_CTL1_PF0()        SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_USCI0_CTL1)                    /*!< Set PF0 function to USCI0_CTL1          */
#define SET_USCI0_DAT0_PA10()       SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA10MFP_Msk)) | SYS_GPA_MFP2_PA10MFP_USCI0_DAT0)                  /*!< Set PA10 function to USCI0_DAT0         */
#define SET_USCI0_DAT0_PB13()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_USCI0_DAT0)                  /*!< Set PB13 function to USCI0_DAT0         */
#define SET_USCI0_DAT0_PD1()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD1MFP_Msk)) | SYS_GPD_MFP0_PD1MFP_USCI0_DAT0)                    /*!< Set PD1 function to USCI0_DAT0          */
#define SET_USCI0_DAT0_PE3()        SYS->GPE_MFP0 = ((SYS->GPE_MFP0 & (~SYS_GPE_MFP0_PE3MFP_Msk)) | SYS_GPE_MFP0_PE3MFP_USCI0_DAT0)                    /*!< Set PE3 function to USCI0_DAT0          */
#define SET_USCI0_DAT1_PA9()        SYS->GPA_MFP2 = ((SYS->GPA_MFP2 & (~SYS_GPA_MFP2_PA9MFP_Msk)) | SYS_GPA_MFP2_PA9MFP_USCI0_DAT1)                    /*!< Set PA9 function to USCI0_DAT1          */
#define SET_USCI0_DAT1_PB14()       SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_USCI0_DAT1)                  /*!< Set PB14 function to USCI0_DAT1         */
#define SET_USCI0_DAT1_PD2()        SYS->GPD_MFP0 = ((SYS->GPD_MFP0 & (~SYS_GPD_MFP0_PD2MFP_Msk)) | SYS_GPD_MFP0_PD2MFP_USCI0_DAT1)                    /*!< Set PD2 function to USCI0_DAT1          */
#define SET_USCI0_DAT1_PE4()        SYS->GPE_MFP1 = ((SYS->GPE_MFP1 & (~SYS_GPE_MFP1_PE4MFP_Msk)) | SYS_GPE_MFP1_PE4MFP_USCI0_DAT1)                    /*!< Set PE4 function to USCI0_DAT1          */
#define SET_USCI0_DAT1_PF1()        SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_USCI0_DAT1)                    /*!< Set PF1 function to USCI0_DAT1          */
#define SET_UTCPD0_CC1_PC0()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_UTCPD0_CC1)                    /*!< Set PC0 function to UTCPD0_CC1          */
#define SET_UTCPD0_CC2_PC1()        SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_UTCPD0_CC2)                    /*!< Set PC1 function to UTCPD0_CC2          */
#define SET_UTCPD0_CCDB1_PC2()      SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_UTCPD0_CCDB1)                  /*!< Set PC2 function to UTCPD0_CCDB1        */
#define SET_UTCPD0_CCDB2_PC3()      SYS->GPC_MFP0 = ((SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_UTCPD0_CCDB2)                  /*!< Set PC3 function to UTCPD0_CCDB2        */
#define SET_UTCPD0_DISCHG_PA1()     SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_UTCPD0_DISCHG)                 /*!< Set PA1 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PA6()     SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_UTCPD0_DISCHG)                 /*!< Set PA6 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PC4()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_UTCPD0_DISCHG)                 /*!< Set PC4 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PC5()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_UTCPD0_DISCHG)                 /*!< Set PC5 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PC6()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_UTCPD0_DISCHG)                 /*!< Set PC6 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PF0()     SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_UTCPD0_DISCHG)                 /*!< Set PF0 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PF1()     SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_UTCPD0_DISCHG)                 /*!< Set PF1 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_DISCHG_PJ0()     SYS->GPJ_MFP0 = ((SYS->GPJ_MFP0 & (~SYS_GPJ_MFP0_PJ0MFP_Msk)) | SYS_GPJ_MFP0_PJ0MFP_UTCPD0_DISCHG)                 /*!< Set PJ0 function to UTCPD0_DISCHG       */
#define SET_UTCPD0_FRSTX1_PA1()     SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_UTCPD0_FRSTX1)                 /*!< Set PA1 function to UTCPD0_FRSTX1       */
#define SET_UTCPD0_FRSTX1_PC4()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_UTCPD0_FRSTX1)                 /*!< Set PC4 function to UTCPD0_FRSTX1       */
#define SET_UTCPD0_FRSTX1_PF1()     SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF1MFP_Msk)) | SYS_GPF_MFP0_PF1MFP_UTCPD0_FRSTX1)                 /*!< Set PF1 function to UTCPD0_FRSTX1       */
#define SET_UTCPD0_FRSTX2_PC5()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_UTCPD0_FRSTX2)                 /*!< Set PC5 function to UTCPD0_FRSTX2       */
#define SET_UTCPD0_FRSTX2_PC6()     SYS->GPC_MFP1 = ((SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC6MFP_Msk)) | SYS_GPC_MFP1_PC6MFP_UTCPD0_FRSTX2)                 /*!< Set PC6 function to UTCPD0_FRSTX2       */
#define SET_UTCPD0_FRSTX2_PF0()     SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF0MFP_Msk)) | SYS_GPF_MFP0_PF0MFP_UTCPD0_FRSTX2)                 /*!< Set PF0 function to UTCPD0_FRSTX2       */
#define SET_UTCPD0_VBDCHG_PB1()     SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB1MFP_Msk)) | SYS_GPB_MFP0_PB1MFP_UTCPD0_VBDCHG)                 /*!< Set PB1 function to UTCPD0_VBDCHG       */
#define SET_UTCPD0_VBSNKEN_PA3()    SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_UTCPD0_VBSNKEN)                /*!< Set PA3 function to UTCPD0_VBSNKEN      */
#define SET_UTCPD0_VBSNKEN_PA5()    SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_UTCPD0_VBSNKEN)                /*!< Set PA5 function to UTCPD0_VBSNKEN      */
#define SET_UTCPD0_VBSNKEN_PA7()    SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA7MFP_Msk)) | SYS_GPA_MFP1_PA7MFP_UTCPD0_VBSNKEN)                /*!< Set PA7 function to UTCPD0_VBSNKEN      */
#define SET_UTCPD0_VBSNKEN_PB13()   SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB13MFP_Msk)) | SYS_GPB_MFP3_PB13MFP_UTCPD0_VBSNKEN)              /*!< Set PB13 function to UTCPD0_VBSNKEN     */
#define SET_UTCPD0_VBSNKEN_PB15()   SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB15MFP_Msk)) | SYS_GPB_MFP3_PB15MFP_UTCPD0_VBSNKEN)              /*!< Set PB15 function to UTCPD0_VBSNKEN     */
#define SET_UTCPD0_VBSNKEN_PF5()    SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_UTCPD0_VBSNKEN)                /*!< Set PF5 function to UTCPD0_VBSNKEN      */
#define SET_UTCPD0_VBSRCEN_PA2()    SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_UTCPD0_VBSRCEN)                /*!< Set PA2 function to UTCPD0_VBSRCEN      */
#define SET_UTCPD0_VBSRCEN_PA4()    SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_UTCPD0_VBSRCEN)                /*!< Set PA4 function to UTCPD0_VBSRCEN      */
#define SET_UTCPD0_VBSRCEN_PA6()    SYS->GPA_MFP1 = ((SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA6MFP_Msk)) | SYS_GPA_MFP1_PA6MFP_UTCPD0_VBSRCEN)                /*!< Set PA6 function to UTCPD0_VBSRCEN      */
#define SET_UTCPD0_VBSRCEN_PB12()   SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB12MFP_Msk)) | SYS_GPB_MFP3_PB12MFP_UTCPD0_VBSRCEN)              /*!< Set PB12 function to UTCPD0_VBSRCEN     */
#define SET_UTCPD0_VBSRCEN_PB14()   SYS->GPB_MFP3 = ((SYS->GPB_MFP3 & (~SYS_GPB_MFP3_PB14MFP_Msk)) | SYS_GPB_MFP3_PB14MFP_UTCPD0_VBSRCEN)              /*!< Set PB14 function to UTCPD0_VBSRCEN     */
#define SET_UTCPD0_VBSRCEN_PF4()    SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_UTCPD0_VBSRCEN)                /*!< Set PF4 function to UTCPD0_VBSRCEN      */
#define SET_UTCPD0_VCNEN1_PA0()     SYS->GPA_MFP0 = ((SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_UTCPD0_VCNEN1)                 /*!< Set PA0 function to UTCPD0_VCNEN1       */
#define SET_UTCPD0_VCNEN2_PB0()     SYS->GPB_MFP0 = ((SYS->GPB_MFP0 & (~SYS_GPB_MFP0_PB0MFP_Msk)) | SYS_GPB_MFP0_PB0MFP_UTCPD0_VCNEN2)                 /*!< Set PB0 function to UTCPD0_VCNEN2       */
#define SET_X32_IN_PF5()            SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF5MFP_Msk)) | SYS_GPF_MFP1_PF5MFP_X32_IN)                        /*!< Set PF5 function to X32_IN              */
#define SET_X32_OUT_PF4()           SYS->GPF_MFP1 = ((SYS->GPF_MFP1 & (~SYS_GPF_MFP1_PF4MFP_Msk)) | SYS_GPF_MFP1_PF4MFP_X32_OUT)                       /*!< Set PF4 function to X32_OUT             */
#define SET_XT1_IN_PF3()            SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF3MFP_Msk)) | SYS_GPF_MFP0_PF3MFP_XT1_IN)                        /*!< Set PF3 function to XT1_IN              */
#define SET_XT1_OUT_PF2()           SYS->GPF_MFP0 = ((SYS->GPF_MFP0 & (~SYS_GPF_MFP0_PF2MFP_Msk)) | SYS_GPF_MFP0_PF2MFP_XT1_OUT)                       /*!< Set PF2 function to XT1_OUT             */


/** @} end of group SYS_EXPORTED_CONSTANTS */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear Brown-out detector interrupt flag
  * @param      None
  * @return     None
  * @details    This macro clear Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_INT_FLAG() \
    do{ \
        SYS->BODSTS |= SYS_BODSTS_BODIF_Msk; \
    }while(0)

/**
* @brief      Set Brown-out detector function to low power mode
* @param      None
* @return     None
* @details    This macro set Brown-out detector to low power mode.
*             The register write-protection function should be disabled before using this macro.
* \hideinitializer
*/
#define SYS_SET_BOD_LPM() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODLPM_Msk; \
    }while(0)

/**
  * @brief      Set Brown-out detector function to normal mode
  * @param      None
  * @return     None
  * @details    This macro set Brown-out detector to normal mode.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_CLEAR_BOD_LPM() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODLPM_Msk; \
    }while(0)

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk; \
    }while(0)

/**
  * @brief      Enable Brown-out detector function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODEN_Msk; \
    }while(0)

/**
  * @brief      Get Brown-out detector interrupt flag
  * @param      None
  * @retval     0   Brown-out detect interrupt flag is not set.
  * @retval     >=1 Brown-out detect interrupt flag is set.
  * @details    This macro get Brown-out detector interrupt flag.
  * \hideinitializer
  */
#define SYS_GET_BOD_INT_FLAG()          (SYS->BODSTS & SYS_BODSTS_BODIF_Msk)

/**
  * @brief      Get Brown-out detector status
  * @param      None
  * @retval     0   System voltage is higher than BOD threshold voltage setting or BOD function is disabled.
  * @retval     >=1 System voltage is lower than BOD threshold voltage setting.
  * @details    This macro get Brown-out detector output status.
  *             If the BOD function is disabled, this function always return 0.
  * \hideinitializer
  */
#define SYS_GET_BOD_OUTPUT()            (SYS->BODSTS & SYS_BODSTS_BODOUT_Msk)

/**
  * @brief      Enable Brown-out detector interrupt function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detector interrupt function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_BOD_RST() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_BODRSTEN_Msk; \
    }while(0)

/**
  * @brief      Enable Brown-out detector reset function
  * @param      None
  * @return     None
  * @details    This macro enable Brown-out detect reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_BOD_RST() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_BODRSTEN_Msk; \
    }while(0)

/**
  * @brief      Set Brown-out detector voltage level
  * @param[in]  u32Level is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_1_6V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_2_2V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_6V
  *             - \ref SYS_BODCTL_BODVL_2_8V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  * @return     None
  * @details    This macro set Brown-out detector voltage level.
  *             The write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_LEVEL(u32Level) \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | (u32Level); \
    }while(0)

/**
  * @brief      Set Brown-out detector wake-up selection
  * @param[in]  u32Wakeup is Brown-out detector wake-up selection. Including :
  *             - \ref SYS_BODCTL_BODWKEN_DISABLE
  *             - \ref SYS_BODCTL_BODWKEN_RISE
  *             - \ref SYS_BODCTL_BODWKEN_DROP
  *             - \ref SYS_BODCTL_BODWKEN_BOTH
  * @return     None
  * @details    This macro set Brown-out detector wake-up selection.
  *             The write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_SET_BOD_WAKEUP(u32Wakeup) \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODWKEN_Msk) | (u32Wakeup); \
    }while(0)

/**
* @brief      Set Brown-out detector output de-glitch time selection
* @param[in]  u32Select is Brown-out detector wake-up selection. Including :
*             - \ref SYS_BODCTL_BODDGSEL_LIRC
*             - \ref SYS_BODCTL_BODDGSEL_4_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_8_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_16_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_32_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_64_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_128_SCLK
*             - \ref SYS_BODCTL_BODDGSEL_256_SCLK
* @return     None
* @details    This macro set Brown-out detector output de-glitch time selection.
*             The write-protection function should be disabled before using this macro.
* \hideinitializer
*/
#define SYS_SET_BODDGSEL(u32Select) \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODDGSEL_Msk) | (u32Select); \
    }while(0)

/**
  * @brief      Get reset source is from Brown-out detector reset
  * @param      None
  * @retval     0   Previous reset source is not from Brown-out detector reset
  * @retval     >=1 Previous reset source is from Brown-out detector reset
  * @details    This macro get previous reset source is from Brown-out detect reset or not.
  * \hideinitializer
  */
#define SYS_IS_BOD_RST()                (SYS->RSTSTS & SYS_RSTSTS_BODRF_Msk)

/**
  * @brief      Get reset source is from CPU reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU reset
  * @retval     >=1 Previous reset source is from CPU reset
  * @details    This macro get previous reset source is from CPU reset.
  * \hideinitializer
  */
#define SYS_IS_CPU_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPURF_Msk)

/**
  * @brief      Get reset source is from LVR Reset
  * @param      None
  * @retval     0   Previous reset source is not from Low-Voltage-Reset
  * @retval     >=1 Previous reset source is from Low-Voltage-Reset
  * @details    This macro get previous reset source is from Low-Voltage-Reset.
  * \hideinitializer
  */
#define SYS_IS_LVR_RST()                (SYS->RSTSTS & SYS_RSTSTS_LVRRF_Msk)

/**
  * @brief      Get reset source is from Power-on Reset
  * @param      None
  * @retval     0   Previous reset source is not from Power-on Reset
  * @retval     >=1 Previous reset source is from Power-on Reset
  * @details    This macro get previous reset source is from Power-on Reset.
  * \hideinitializer
  */
#define SYS_IS_POR_RST()                (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)

/**
  * @brief      Get reset source is from reset pin reset
  * @param      None
  * @retval     0   Previous reset source is not from reset pin reset
  * @retval     >=1 Previous reset source is from reset pin reset
  * @details    This macro get previous reset source is from reset pin reset.
  * \hideinitializer
  */
#define SYS_IS_RSTPIN_RST()             (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)

/**
  * @brief      Get reset source is from system reset
  * @param      None
  * @retval     0   Previous reset source is not from system reset
  * @retval     >=1 Previous reset source is from system reset
  * @details    This macro get previous reset source is from system reset.
  * \hideinitializer
  */
#define SYS_IS_SYSTEM_RST()             (SYS->RSTSTS & SYS_RSTSTS_SYSRF_Msk)

/**
  * @brief      Get reset source is from window watch dog reset
  * @param      None
  * @retval     0   Previous reset source is not from window watch dog reset
  * @retval     >=1 Previous reset source is from window watch dog reset
  * @details    This macro get previous reset source is from window watch dog reset.
  * \hideinitializer
  */
#define SYS_IS_WDT_RST()                (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)

/**
  * @brief      Get reset source is from CPU lockup reset
  * @param      None
  * @retval     0   Previous reset source is not from CPU lockup reset
  * @retval     >=1 Previous reset source is from CPU lockup reset
  * @details    This macro get previous reset source is from CPU lockup reset.
  * \hideinitializer
  */
#define SYS_IS_CPULK_RST()                (SYS->RSTSTS & SYS_RSTSTS_CPULKRF_Msk)


/**
  * @brief      Disable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_LVR() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL &= ~SYS_BODCTL_LVREN_Msk; \
    }while(0)

/**
  * @brief      Enable Low-Voltage-Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Low-Voltage-Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_LVR() \
    do{ \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk); \
        SYS->BODCTL |= SYS_BODCTL_LVREN_Msk; \
    }while(0)

/**
  * @brief      Disable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro disable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_DISABLE_POR()               (SYS->PORCTL = 0x5AA55AA5)

/**
  * @brief      Enable Power-on Reset function
  * @param      None
  * @return     None
  * @details    This macro enable Power-on Reset function.
  *             The register write-protection function should be disabled before using this macro.
  * \hideinitializer
  */
#define SYS_ENABLE_POR()                (SYS->PORCTL = 0)

/**
  * @brief      Wait SYS_BODCTL Write Busy Flag
  * @param      None
  * @return     None
  * @details    This macro waits SYS_BODCTL write busy flag is cleared and skips when time-out.
  */
#define SYS_WAIT_BODCTL_WRBUSY() \
    do{ \
        uint32_t u32TimeOutCnt = SYS_TIMEOUT; \
        while(SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) \
        { \
            if(--u32TimeOutCnt == 0) break; \
        } \
    }while(0)

/**
  * @brief      Wait SYS_VREFCTL Write Busy Flag
  * @param      None
  * @return     None
  * @details    This macro waits SYS_VREFCTL write busy flag is cleared and skips when time-out.
  */
#define SYS_WAIT_VREFCTL_WRBUSY() \
    do{ \
        uint32_t u32TimeOutCnt = SYS_TIMEOUT; \
        while(SYS->VREFCTL & SYS_VREFCTL_WRBUSY_Msk) \
        { \
            if(--u32TimeOutCnt == 0) break; \
        } \
    }while(0)

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void SYS_UnlockReg(void);
__STATIC_INLINE void SYS_LockReg(void);
__STATIC_INLINE void SYS_ClearResetSrc(uint32_t u32Src);
__STATIC_INLINE uint32_t SYS_GetBODStatus(void);
__STATIC_INLINE uint32_t SYS_GetResetSrc(void);
__STATIC_INLINE uint32_t SYS_IsRegLocked(void);
__STATIC_INLINE uint32_t SYS_ReadPDID(void);
__STATIC_INLINE void SYS_ResetChip(void);
__STATIC_INLINE void SYS_ResetCPU(void);
__STATIC_INLINE void SYS_SetVRef(uint32_t u32VRefCTL);

/**
  * @brief      Disable register write-protection function
  * @param      None
  * @return     None
  * @details    This function disable register write-protection function.
  *             To unlock the protected register to allow write access.
  */
__STATIC_INLINE void SYS_UnlockReg(void)
{
    uint32_t u32TimeOutCount = SYS_TIMEOUT;

    do
    {
        SYS->REGLCTL = 0x59UL;
        SYS->REGLCTL = 0x16UL;
        SYS->REGLCTL = 0x88UL;

        if (--u32TimeOutCount == 0) break;
    } while (SYS->REGLCTL == 0UL);
}

/**
  * @brief      Enable register write-protection function
  * @param      None
  * @return     None
  * @details    This function is used to enable register write-protection function.
  *             To lock the protected register to forbid write access.
  */
__STATIC_INLINE void SYS_LockReg(void)
{
    SYS->REGLCTL = 0UL;
}

/**
  * @brief      Clear reset source
  * @param[in]  u32Src is system reset source. Including :
  *             - \ref SYS_RSTSTS_PORF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_LVRRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS = u32Src;
}

/**
  * @brief      Get Brown-out detector output status
  * @param      None
  * @retval     0 System voltage is higher than BODVL setting or BODEN is 0.
  * @retval     1 System voltage is lower than BODVL setting.
  * @details    This function get Brown-out detector output status.
  */
__STATIC_INLINE uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODSTS & SYS_BODSTS_BODOUT_Msk) >> SYS_BODSTS_BODOUT_Pos);
}

/**
  * @brief      Get reset status register value
  * @param      None
  * @return     Reset source
  * @details    This function get the system reset status register value.
  */
__STATIC_INLINE uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief      Check if register is locked nor not
  * @param      None
  * @retval     0 Write-protection function is disabled.
  *             1 Write-protection function is enabled.
  * @details    This function check register write-protection bit setting.
  */
__STATIC_INLINE uint32_t SYS_IsRegLocked(void)
{
    return SYS->REGLCTL & 1UL ? 0UL : 1UL;
}

/**
  * @brief      Get product ID
  * @param      None
  * @return     Product ID
  * @details    This function get product ID.
  */
__STATIC_INLINE uint32_t SYS_ReadPDID(void)
{
    return SYS->PDID;
}

/**
  * @brief      Reset chip with chip reset
  * @param      None
  * @return     None
  * @details    This function reset chip with chip reset.
  *             The register write-protection function should be disabled before using this function.
  */
__STATIC_INLINE void SYS_ResetChip(void)
{
    SYS->RSTCTL |= SYS_RSTCTL_CHIPRST_Msk;
}

/**
  * @brief      Reset CPU with CPU reset
  * @param      None
  * @return     None
  * @details    This function reset CPU with CPU reset.
  *             The register write-protection function should be disabled before using this function.
  */
__STATIC_INLINE void SYS_ResetCPU(void)
{
    SYS->RSTCTL |= SYS_RSTCTL_CPURSTEN_Msk;

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    __WFI();
}

/**
  * @brief      Set Reference Voltage
  * @param[in]  u32VRefCTL is reference voltage setting. Including :
  *             - \ref SYS_VREFCTL_VREF_PIN
  *             - \ref SYS_VREFCTL_VREF_1_6V
  *             - \ref SYS_VREFCTL_VREF_2_048V
  *             - \ref SYS_VREFCTL_VREF_2_5V
  *             - \ref SYS_VREFCTL_VREF_3_072V
  *             - \ref SYS_VREFCTL_VREF_AVDD
  * @return     None
  * @details    This function select reference voltage.
  *             The register write-protection function should be disabled before using this function.
  */
__STATIC_INLINE void SYS_SetVRef(uint32_t u32VRefCTL)
{
    /* Set reference voltage */
    SYS->VREFCTL = (SYS->VREFCTL & (~SYS_VREFCTL_VREFCTL_Msk)) | (u32VRefCTL);
}

void SYS_ResetModule(uint32_t u32ModuleIndex);
int32_t SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
int32_t SYS_DisableBOD(void);

/** @} end of group SYS_EXPORTED_FUNCTIONS */
/** @} end of group SYS_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __SYS_H__ */
