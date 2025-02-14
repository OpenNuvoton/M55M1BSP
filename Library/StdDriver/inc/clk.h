/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * @brief    CLK driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Frequency constant definitions.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define FREQ_1MHZ          1000000UL    /*!< 1 MHz \hideinitializer */
#define FREQ_2MHZ          2000000UL    /*!< 2 MHz \hideinitializer */
#define FREQ_4MHZ          4000000UL    /*!< 4 MHz \hideinitializer */
#define FREQ_8MHZ          8000000UL    /*!< 8 MHz \hideinitializer */
#define FREQ_12MHZ         12000000UL   /*!< 12 MHz \hideinitializer */
#define FREQ_24MHZ         24000000UL   /*!< 24 MHz \hideinitializer */
#define FREQ_25MHZ         25000000UL   /*!< 25 MHz \hideinitializer */
#define FREQ_36MHZ         36000000UL   /*!< 36 MHz \hideinitializer */
#define FREQ_40MHZ         40000000UL   /*!< 40 MHz \hideinitializer */
#define FREQ_45MHZ         45000000UL   /*!< 45 MHz \hideinitializer */
#define FREQ_50MHZ         50000000UL   /*!< 50 MHz \hideinitializer */
#define FREQ_72MHZ         72000000UL   /*!< 72 MHz \hideinitializer */
#define FREQ_75MHZ         75000000UL   /*!< 75 MHz \hideinitializer */
#define FREQ_80MHZ         80000000UL   /*!< 80 MHz \hideinitializer */
#define FREQ_90MHZ         90000000UL   /*!< 90 MHz \hideinitializer */
#define FREQ_100MHZ        100000000UL  /*!< 100 MHz \hideinitializer */
#define FREQ_120MHZ        120000000UL  /*!< 120 MHz \hideinitializer */
#define FREQ_125MHZ        125000000UL  /*!< 125 MHz \hideinitializer */
#define FREQ_144MHZ        144000000UL  /*!< 144 MHz \hideinitializer */
#define FREQ_150MHZ        150000000UL  /*!< 150 MHz \hideinitializer */
#define FREQ_160MHZ        160000000UL  /*!< 160 MHz \hideinitializer */
#define FREQ_175MHZ        175000000UL  /*!< 175 MHz \hideinitializer */
#define FREQ_180MHZ        180000000UL  /*!< 180 MHz \hideinitializer */
#define FREQ_192MHZ        192000000UL  /*!< 192 MHz \hideinitializer */
#define FREQ_200MHZ        200000000UL  /*!< 200 MHz \hideinitializer */
#define FREQ_220MHZ        220000000UL  /*!< 220 MHz \hideinitializer */
#define FREQ_225MHZ        225000000UL  /*!< 225 MHz \hideinitializer */
#define FREQ_240MHZ        240000000UL  /*!< 240 MHz \hideinitializer */
#define FREQ_384MHZ        384000000UL  /*!< 384 MHz \hideinitializer */
#define FREQ_500MHZ        500000000UL  /*!< 500 MHz \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  MIRC constant definitions.                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_MIRCCTL_MIRCFSEL_1MHZ    (0x0UL << CLK_MIRCCTL_MIRCFSEL_Pos)          /*!< Select MIRC clock to 1 MHz \hideinitializer */
#define CLK_MIRCCTL_MIRCFSEL_2MHZ    (0x1UL << CLK_MIRCCTL_MIRCFSEL_Pos)          /*!< Select MIRC clock to 1 MHz \hideinitializer */
#define CLK_MIRCCTL_MIRCFSEL_4MHZ    (0x2UL << CLK_MIRCCTL_MIRCFSEL_Pos)          /*!< Select MIRC clock to 1 MHz \hideinitializer */
#define CLK_MIRCCTL_MIRCFSEL_8MHZ    (0x3UL << CLK_MIRCCTL_MIRCFSEL_Pos)          /*!< Select MIRC clock to 1 MHz \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL constant definitions.  (Write-protection)                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_SCLKSEL_SCLKSEL_HIRC            (0x0UL << CLK_SCLKSEL_SCLKSEL_Pos)          /*!< Select System clock source from HIRC \hideinitializer */
#define CLK_SCLKSEL_SCLKSEL_MIRC            (0x1UL << CLK_SCLKSEL_SCLKSEL_Pos)          /*!< Select System clock source from MIRC \hideinitializer */
#define CLK_SCLKSEL_SCLKSEL_HIRC48M         (0x2UL << CLK_SCLKSEL_SCLKSEL_Pos)          /*!< Select System clock source from HIRC48M \hideinitializer */
#define CLK_SCLKSEL_SCLKSEL_HXT             (0x3UL << CLK_SCLKSEL_SCLKSEL_Pos)          /*!< Select System clock source from HXT \hideinitializer */
#define CLK_SCLKSEL_SCLKSEL_APLL0           (0x4UL << CLK_SCLKSEL_SCLKSEL_Pos)          /*!< Select System clock source from APLL0 \hideinitializer */

#define CLK_BPWMSEL_BPWM0SEL_PCLK0          (0x0UL << CLK_BPWMSEL_BPWM0SEL_Pos)         /*!< Select BPWM0 clock source from PCLK0 \hideinitializer */
#define CLK_BPWMSEL_BPWM0SEL_HCLK0          (0x1UL << CLK_BPWMSEL_BPWM0SEL_Pos)         /*!< Select BPWM0 clock source from HCLK0 \hideinitializer */

#define CLK_BPWMSEL_BPWM1SEL_PCLK2          (0x0UL << CLK_BPWMSEL_BPWM1SEL_Pos)         /*!< Select BPWM1 clock source from PCLK2 \hideinitializer */
#define CLK_BPWMSEL_BPWM1SEL_HCLK0          (0x1UL << CLK_BPWMSEL_BPWM1SEL_Pos)         /*!< Select BPWM1 clock source from HCLK0 \hideinitializer */

#define CLK_CANFDSEL_CANFD0SEL_HXT          (0x0UL << CLK_CANFDSEL_CANFD0SEL_Pos)       /*!< Select CANFD0 clock source from HXT \hideinitializer */
#define CLK_CANFDSEL_CANFD0SEL_APLL0_DIV2   (0x1UL << CLK_CANFDSEL_CANFD0SEL_Pos)       /*!< Select CANFD0 clock source from APLL0/2 \hideinitializer */
#define CLK_CANFDSEL_CANFD0SEL_HCLK0        (0x2UL << CLK_CANFDSEL_CANFD0SEL_Pos)       /*!< Select CANFD0 clock source from HCLK0 \hideinitializer */
#define CLK_CANFDSEL_CANFD0SEL_HIRC         (0x3UL << CLK_CANFDSEL_CANFD0SEL_Pos)       /*!< Select CANFD0 clock source from HIRC \hideinitializer */
#define CLK_CANFDSEL_CANFD0SEL_HIRC48M_DIV4 (0x4UL << CLK_CANFDSEL_CANFD0SEL_Pos)       /*!< Select CANFD0 clock source from HIRC48M/4 \hideinitializer */

#define CLK_CANFDSEL_CANFD1SEL_HXT          (0x0UL << CLK_CANFDSEL_CANFD1SEL_Pos)       /*!< Select CANFD1 clock source from HXT \hideinitializer */
#define CLK_CANFDSEL_CANFD1SEL_APLL0_DIV2   (0x1UL << CLK_CANFDSEL_CANFD1SEL_Pos)       /*!< Select CANFD1 clock source from APLL0/2 \hideinitializer */
#define CLK_CANFDSEL_CANFD1SEL_HCLK0        (0x2UL << CLK_CANFDSEL_CANFD1SEL_Pos)       /*!< Select CANFD1 clock source from HCLK0 \hideinitializer */
#define CLK_CANFDSEL_CANFD1SEL_HIRC         (0x3UL << CLK_CANFDSEL_CANFD1SEL_Pos)       /*!< Select CANFD1 clock source from HIRC \hideinitializer */
#define CLK_CANFDSEL_CANFD1SEL_HIRC48M_DIV4 (0x4UL << CLK_CANFDSEL_CANFD1SEL_Pos)       /*!< Select CANFD1 clock source from HIRC48M/4 \hideinitializer */

#define CLK_CCAPSEL_CCAP0SEL_MIRC           (0x0UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from MIRC \hideinitializer */
#define CLK_CCAPSEL_CCAP0SEL_HCLK2          (0x1UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from HCLK2 \hideinitializer */
#define CLK_CCAPSEL_CCAP0SEL_HIRC           (0x2UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from HIRC \hideinitializer */
#define CLK_CCAPSEL_CCAP0SEL_APLL0_DIV2     (0x3UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from APLL0/2 \hideinitializer */
#define CLK_CCAPSEL_CCAP0SEL_HXT            (0x4UL << CLK_CCAPSEL_CCAP0SEL_Pos)         /*!< Select CCAP sensor clock source from HXT \hideinitializer */

#define CLK_CLKOSEL_CLKOSEL_SYSCLK          (0x0UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from SYSCLK \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_ACLK            (0x1UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from ACLK \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HCLK0           (0x2UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HCLK0 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HCLK1           (0x3UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HCLK1 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HCLK2           (0x4UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HCLK2 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_APLL0_DIV2      (0x5UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from APLL0/2 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_APLL1_DIV2      (0x6UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from APLL1/2 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HIRC48M         (0x7UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HIRC48M \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HXT             (0x8UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HXT \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HIRC            (0x9UL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from HIRC \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_MIRC            (0xaUL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from MIRC \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_LXT             (0xbUL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from LXT \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_LIRC            (0xcUL << CLK_CLKOSEL_CLKOSEL_Pos)          /*!< Select CLKO clock source from LIRC \hideinitializer */

#define CLK_DMICSEL_DMIC0SEL_HXT            (0x0UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from HXT \hideinitializer */
#define CLK_DMICSEL_DMIC0SEL_APLL1_DIV2     (0x1UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from APLL1/2 \hideinitializer */
#define CLK_DMICSEL_DMIC0SEL_MIRC           (0x2UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from MIRC \hideinitializer */
#define CLK_DMICSEL_DMIC0SEL_HIRC           (0x3UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from HIRC \hideinitializer */
#define CLK_DMICSEL_DMIC0SEL_HIRC48M        (0x4UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from HIRC48M \hideinitializer */
#define CLK_DMICSEL_DMIC0SEL_PCLK4          (0x5UL << CLK_DMICSEL_DMIC0SEL_Pos)         /*!< Select DMIC clock source from PCLK4 \hideinitializer */
#define CLK_DMICSEL_VAD0SEL_PCLK4           (0x0UL << CLK_DMICSEL_VAD0SEL_Pos)          /*!< Select VAD clock source from PCLK4 \hideinitializer */
#define CLK_DMICSEL_VAD0SEL_MIRC            (0x1UL << CLK_DMICSEL_VAD0SEL_Pos)          /*!< Select VAD clock source from MIRC \hideinitializer */
#define CLK_DMICSEL_VAD0SEL_HIRC            (0x2UL << CLK_DMICSEL_VAD0SEL_Pos)          /*!< Select VAD clock source from HIRC \hideinitializer */

#define CLK_EADCSEL_EADC0SEL_APLL1_DIV2     (0x0UL << CLK_EADCSEL_EADC0SEL_Pos)         /*!< Select EADC0 clock source from APLL1/2 \hideinitializer */
#define CLK_EADCSEL_EADC0SEL_APLL0_DIV2     (0x1UL << CLK_EADCSEL_EADC0SEL_Pos)         /*!< Select EADC0 clock source from APLL0/2 \hideinitializer */
#define CLK_EADCSEL_EADC0SEL_PCLK0          (0x2UL << CLK_EADCSEL_EADC0SEL_Pos)         /*!< Select EADC0 clock source from PCLK0 \hideinitializer */

#define CLK_EPWMSEL_EPWM0SEL_PCLK0          (0x0UL << CLK_EPWMSEL_EPWM0SEL_Pos)         /*!< Select EPWM0 clock source from PCLK0 \hideinitializer */
#define CLK_EPWMSEL_EPWM0SEL_HCLK0          (0x1UL << CLK_EPWMSEL_EPWM0SEL_Pos)         /*!< Select EPWM0 clock source from HCLK0 \hideinitializer */

#define CLK_EPWMSEL_EPWM1SEL_PCLK2          (0x0UL << CLK_EPWMSEL_EPWM1SEL_Pos)         /*!< Select EPWM1 clock source from PCLK2 \hideinitializer */
#define CLK_EPWMSEL_EPWM1SEL_HCLK0          (0x1UL << CLK_EPWMSEL_EPWM1SEL_Pos)         /*!< Select EPWM1 clock source from HCLK0 \hideinitializer */

#define CLK_FMCSEL_FMC0SEL_HIRC             (0x0UL << CLK_FMCSEL_FMC0SEL_Pos)           /*!< Select FMC clock source from HIRC \hideinitializer */
#define CLK_FMCSEL_FMC0SEL_HIRC48M_DIV4     (0x1UL << CLK_FMCSEL_FMC0SEL_Pos)           /*!< Select FMC clock source from HIRC48M/4 \hideinitializer */

#define CLK_I2SSEL_I2S0SEL_HXT              (0x0UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from HXT \hideinitializer */
#define CLK_I2SSEL_I2S0SEL_APLL1_DIV2       (0x1UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from APLL1/2 \hideinitializer */
#define CLK_I2SSEL_I2S0SEL_APLL0_DIV2       (0x2UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from APLL0/2 \hideinitializer */
#define CLK_I2SSEL_I2S0SEL_PCLK1            (0x3UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from PCLK1 \hideinitializer */
#define CLK_I2SSEL_I2S0SEL_HIRC             (0x4UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from HIRC \hideinitializer */
#define CLK_I2SSEL_I2S0SEL_HIRC48M          (0x5UL << CLK_I2SSEL_I2S0SEL_Pos)           /*!< Select I2S0 clock source from HIRC48M \hideinitializer */

#define CLK_I2SSEL_I2S1SEL_HXT              (0x0UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from HXT \hideinitializer */
#define CLK_I2SSEL_I2S1SEL_APLL1_DIV2       (0x1UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from APLL0/2 \hideinitializer */
#define CLK_I2SSEL_I2S1SEL_APLL0_DIV2       (0x2UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from APLL1/2 \hideinitializer */
#define CLK_I2SSEL_I2S1SEL_PCLK3            (0x3UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from PCLK3 \hideinitializer */
#define CLK_I2SSEL_I2S1SEL_HIRC             (0x4UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from HIRC \hideinitializer */
#define CLK_I2SSEL_I2S1SEL_HIRC48M          (0x5UL << CLK_I2SSEL_I2S1SEL_Pos)           /*!< Select I2S1 clock source from HIRC48M \hideinitializer */

#define CLK_I3CSEL_I3C0SEL_HCLK0            (0x0UL << CLK_I3CSEL_I3C0SEL_Pos)           /*!< Select I2S1 clock source from HCLK0 \hideinitializer */
#define CLK_I3CSEL_I3C0SEL_APLL1            (0x1UL << CLK_I3CSEL_I3C0SEL_Pos)           /*!< Select I2S1 clock source from APLL1 \hideinitializer */

#define CLK_KPISEL_KPI0SEL_HIRC48M_DIV4     (0x0UL << CLK_KPISEL_KPI0SEL_Pos)           /*!< Select KPI clock source from HIRC48M/4 \hideinitializer */
#define CLK_KPISEL_KPI0SEL_HIRC             (0x1UL << CLK_KPISEL_KPI0SEL_Pos)           /*!< Select KPI clock source from HIRC \hideinitializer */
#define CLK_KPISEL_KPI0SEL_LIRC             (0x2UL << CLK_KPISEL_KPI0SEL_Pos)           /*!< Select KPI clock source from LIRC \hideinitializer */
#define CLK_KPISEL_KPI0SEL_HXT              (0x3UL << CLK_KPISEL_KPI0SEL_Pos)           /*!< Select KPI clock source from HXT \hideinitializer */

#define CLK_LPADCSEL_LPADC0SEL_PCLK4        (0x0UL << CLK_LPADCSEL_LPADC0SEL_Pos)       /*!< Select LPADC0 clock source from PCLK2 \hideinitializer */
#define CLK_LPADCSEL_LPADC0SEL_LXT          (0x1UL << CLK_LPADCSEL_LPADC0SEL_Pos)       /*!< Select LPADC0 clock source from LXT \hideinitializer */
#define CLK_LPADCSEL_LPADC0SEL_MIRC         (0x2UL << CLK_LPADCSEL_LPADC0SEL_Pos)       /*!< Select LPADC0 clock source from MIRC \hideinitializer */
#define CLK_LPADCSEL_LPADC0SEL_HIRC         (0x3UL << CLK_LPADCSEL_LPADC0SEL_Pos)       /*!< Select LPADC0 clock source from HIRC \hideinitializer */

#define CLK_LPSPISEL_LPSPI0SEL_PCLK4        (0x0UL << CLK_LPSPISEL_LPSPI0SEL_Pos)       /*!< Select LPSPI0 clock source from PCLK4 \hideinitializer */
#define CLK_LPSPISEL_LPSPI0SEL_MIRC         (0x1UL << CLK_LPSPISEL_LPSPI0SEL_Pos)       /*!< Select LPSPI0 clock source from MIRC \hideinitializer */
#define CLK_LPSPISEL_LPSPI0SEL_HIRC         (0x2UL << CLK_LPSPISEL_LPSPI0SEL_Pos)       /*!< Select LPSPI0 clock source from HIRC \hideinitializer */

#define CLK_LPTMRSEL_LPTMR0SEL_PCLK4        (0x0UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from PCLK4 \hideinitializer */
#define CLK_LPTMRSEL_LPTMR0SEL_LXT          (0x1UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from LXT \hideinitializer */
#define CLK_LPTMRSEL_LPTMR0SEL_LIRC         (0x2UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from LIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR0SEL_MIRC         (0x3UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from MIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR0SEL_HIRC         (0x4UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from HIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR0SEL_EXT          (0x5UL << CLK_LPTMRSEL_LPTMR0SEL_Pos)       /*!< Select LPTMR0 clock source from external trigger \hideinitializer */

#define CLK_LPTMRSEL_LPTMR1SEL_PCLK4        (0x0UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from PCLK4 \hideinitializer */
#define CLK_LPTMRSEL_LPTMR1SEL_LXT          (0x1UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from LXT \hideinitializer */
#define CLK_LPTMRSEL_LPTMR1SEL_LIRC         (0x2UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from LIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR1SEL_MIRC         (0x3UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from MIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR1SEL_HIRC         (0x4UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from HIRC \hideinitializer */
#define CLK_LPTMRSEL_LPTMR1SEL_EXT          (0x5UL << CLK_LPTMRSEL_LPTMR1SEL_Pos)       /*!< Select LPTMR1 clock source from external trigger \hideinitializer */

#define CLK_LPUARTSEL_LPUART0SEL_PCLK4      (0x0UL << CLK_LPUARTSEL_LPUART0SEL_Pos)     /*!< Select LPUART0 clock source from PCLK4 \hideinitializer */
#define CLK_LPUARTSEL_LPUART0SEL_LXT        (0x1UL << CLK_LPUARTSEL_LPUART0SEL_Pos)     /*!< Select LPUART0 clock source from LXT \hideinitializer */
#define CLK_LPUARTSEL_LPUART0SEL_MIRC       (0x2UL << CLK_LPUARTSEL_LPUART0SEL_Pos)     /*!< Select LPUART0 clock source from MIRC \hideinitializer */
#define CLK_LPUARTSEL_LPUART0SEL_HIRC       (0x3UL << CLK_LPUARTSEL_LPUART0SEL_Pos)     /*!< Select LPUART0 clock source from HIRC \hideinitializer */

#define CLK_PSIOSEL_PSIO0SEL_LXT            (0x0UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from LXT \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_HXT            (0x1UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from HXT \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_LIRC           (0x2UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from LIRC \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_HIRC           (0x3UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from HIRC \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_HIRC48M_DIV4   (0x4UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from HIRC48M/4 \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_PCLK1          (0x5UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from PCLK1 \hideinitializer */
#define CLK_PSIOSEL_PSIO0SEL_APLL0_DIV2     (0x6UL << CLK_PSIOSEL_PSIO0SEL_Pos)         /*!< Select PSIO clock source from APLL0/2 \hideinitializer */

#define CLK_QSPISEL_QSPI0SEL_HXT            (0x0UL << CLK_QSPISEL_QSPI0SEL_Pos)         /*!< Select QSPI0 clock source from HXT \hideinitializer */
#define CLK_QSPISEL_QSPI0SEL_APLL0_DIV2     (0x1UL << CLK_QSPISEL_QSPI0SEL_Pos)         /*!< Select QSPI0 clock source from APLL0/2 \hideinitializer */
#define CLK_QSPISEL_QSPI0SEL_PCLK0          (0x2UL << CLK_QSPISEL_QSPI0SEL_Pos)         /*!< Select QSPI0 clock source from PCLK0 \hideinitializer */
#define CLK_QSPISEL_QSPI0SEL_HIRC           (0x3UL << CLK_QSPISEL_QSPI0SEL_Pos)         /*!< Select QSPI0 clock source from HIRC \hideinitializer */
#define CLK_QSPISEL_QSPI0SEL_HIRC48M_DIV4   (0x4UL << CLK_QSPISEL_QSPI0SEL_Pos)         /*!< Select QSPI0 clock source from HIRC48M/4 \hideinitializer */

#define CLK_QSPISEL_QSPI1SEL_HXT            (0x0UL << CLK_QSPISEL_QSPI1SEL_Pos)         /*!< Select QSPI1 clock source from HXT \hideinitializer */
#define CLK_QSPISEL_QSPI1SEL_APLL0_DIV2     (0x1UL << CLK_QSPISEL_QSPI1SEL_Pos)         /*!< Select QSPI1 clock source from APLL0/2 \hideinitializer */
#define CLK_QSPISEL_QSPI1SEL_PCLK2          (0x2UL << CLK_QSPISEL_QSPI1SEL_Pos)         /*!< Select QSPI1 clock source from PCLK2 \hideinitializer */
#define CLK_QSPISEL_QSPI1SEL_HIRC           (0x3UL << CLK_QSPISEL_QSPI1SEL_Pos)         /*!< Select QSPI1 clock source from HIRC \hideinitializer */
#define CLK_QSPISEL_QSPI1SEL_HIRC48M_DIV4   (0x4UL << CLK_QSPISEL_QSPI1SEL_Pos)         /*!< Select QSPI1 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SCSEL_SC0SEL_HXT                (0x0UL << CLK_SCSEL_SC0SEL_Pos)             /*!< Select SC0 clock source from HXT \hideinitializer */
#define CLK_SCSEL_SC0SEL_APLL0_DIV2         (0x1UL << CLK_SCSEL_SC0SEL_Pos)             /*!< Select SC0 clock source from APLL0/2 \hideinitializer */
#define CLK_SCSEL_SC0SEL_PCLK1              (0x2UL << CLK_SCSEL_SC0SEL_Pos)             /*!< Select SC0 clock source from PCLK1 \hideinitializer */
#define CLK_SCSEL_SC0SEL_HIRC               (0x3UL << CLK_SCSEL_SC0SEL_Pos)             /*!< Select SC0 clock source from HIRC \hideinitializer */
#define CLK_SCSEL_SC0SEL_HIRC48M_DIV4       (0x4UL << CLK_SCSEL_SC0SEL_Pos)             /*!< Select SC0 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SCSEL_SC1SEL_HXT                (0x0UL << CLK_SCSEL_SC1SEL_Pos)             /*!< Select SC1 clock source from HXT \hideinitializer */
#define CLK_SCSEL_SC1SEL_APLL0_DIV2         (0x1UL << CLK_SCSEL_SC1SEL_Pos)             /*!< Select SC1 clock source from APLL0/2 \hideinitializer */
#define CLK_SCSEL_SC1SEL_PCLK3              (0x2UL << CLK_SCSEL_SC1SEL_Pos)             /*!< Select SC1 clock source from PCLK3 \hideinitializer */
#define CLK_SCSEL_SC1SEL_HIRC               (0x3UL << CLK_SCSEL_SC1SEL_Pos)             /*!< Select SC1 clock source from HIRC \hideinitializer */
#define CLK_SCSEL_SC1SEL_HIRC48M_DIV4       (0x4UL << CLK_SCSEL_SC1SEL_Pos)             /*!< Select SC1 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SCSEL_SC2SEL_HXT                (0x0UL << CLK_SCSEL_SC2SEL_Pos)             /*!< Select SC2 clock source from HXT \hideinitializer */
#define CLK_SCSEL_SC2SEL_APLL0_DIV2         (0x1UL << CLK_SCSEL_SC2SEL_Pos)             /*!< Select SC2 clock source from APLL0/2 \hideinitializer */
#define CLK_SCSEL_SC2SEL_PCLK1              (0x2UL << CLK_SCSEL_SC2SEL_Pos)             /*!< Select SC2 clock source from PCLK1 \hideinitializer */
#define CLK_SCSEL_SC2SEL_HIRC               (0x3UL << CLK_SCSEL_SC2SEL_Pos)             /*!< Select SC2 clock source from HIRC \hideinitializer */
#define CLK_SCSEL_SC2SEL_HIRC48M_DIV4       (0x4UL << CLK_SCSEL_SC2SEL_Pos)             /*!< Select SC2 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SDHSEL_SDH0SEL_HXT              (0x0UL << CLK_SDHSEL_SDH0SEL_Pos)           /*!< Select SDH0 clock source from HXT \hideinitializer */
#define CLK_SDHSEL_SDH0SEL_APLL1_DIV2       (0x1UL << CLK_SDHSEL_SDH0SEL_Pos)           /*!< Select SDH0 clock source from APLL1/2 \hideinitializer */
#define CLK_SDHSEL_SDH0SEL_HCLK0            (0x2UL << CLK_SDHSEL_SDH0SEL_Pos)           /*!< Select SDH0 clock source from HCLK0 \hideinitializer */
#define CLK_SDHSEL_SDH0SEL_HIRC             (0x3UL << CLK_SDHSEL_SDH0SEL_Pos)           /*!< Select SDH0 clock source from HIRC \hideinitializer */
#define CLK_SDHSEL_SDH0SEL_HIRC48M_DIV4     (0x4UL << CLK_SDHSEL_SDH0SEL_Pos)           /*!< Select SDH0 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SDHSEL_SDH1SEL_HXT              (0x0UL << CLK_SDHSEL_SDH1SEL_Pos)           /*!< Select SDH1 clock source from HXT \hideinitializer */
#define CLK_SDHSEL_SDH1SEL_APLL1_DIV2       (0x1UL << CLK_SDHSEL_SDH1SEL_Pos)           /*!< Select SDH1 clock source from APLL1/2 \hideinitializer */
#define CLK_SDHSEL_SDH1SEL_HCLK0            (0x2UL << CLK_SDHSEL_SDH1SEL_Pos)           /*!< Select SDH1 clock source from HCLK0 \hideinitializer */
#define CLK_SDHSEL_SDH1SEL_HIRC             (0x3UL << CLK_SDHSEL_SDH1SEL_Pos)           /*!< Select SDH1 clock source from HIRC \hideinitializer */
#define CLK_SDHSEL_SDH1SEL_HIRC48M_DIV4     (0x4UL << CLK_SDHSEL_SDH1SEL_Pos)           /*!< Select SDH1 clock source from HIRC48M/4 \hideinitializer */

#define CLK_SPISEL_SPI0SEL_HXT              (0x0UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from HXT \hideinitializer */
#define CLK_SPISEL_SPI0SEL_APLL1_DIV2       (0x1UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from APLL1/2 \hideinitializer */
#define CLK_SPISEL_SPI0SEL_APLL0_DIV2       (0x2UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from APLL0/2 \hideinitializer */
#define CLK_SPISEL_SPI0SEL_PCLK0            (0x3UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from PCLK0 \hideinitializer */
#define CLK_SPISEL_SPI0SEL_HIRC             (0x4UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from HIRC \hideinitializer */
#define CLK_SPISEL_SPI0SEL_HIRC48M          (0x5UL << CLK_SPISEL_SPI0SEL_Pos)           /*!< Select SPI0 clock source from HIRC48M \hideinitializer */

#define CLK_SPISEL_SPI1SEL_HXT              (0x0UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from HXT \hideinitializer */
#define CLK_SPISEL_SPI1SEL_APLL1_DIV2       (0x1UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from APLL1/2 \hideinitializer */
#define CLK_SPISEL_SPI1SEL_APLL0_DIV2       (0x2UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from APLL0/2 \hideinitializer */
#define CLK_SPISEL_SPI1SEL_PCLK2            (0x3UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from PCLK2 \hideinitializer */
#define CLK_SPISEL_SPI1SEL_HIRC             (0x4UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from HIRC \hideinitializer */
#define CLK_SPISEL_SPI1SEL_HIRC48M          (0x5UL << CLK_SPISEL_SPI1SEL_Pos)           /*!< Select SPI1 clock source from HIRC48M \hideinitializer */

#define CLK_SPISEL_SPI2SEL_HXT              (0x0UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from HXT \hideinitializer */
#define CLK_SPISEL_SPI2SEL_APLL1_DIV2       (0x1UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from APLL1/2 \hideinitializer */
#define CLK_SPISEL_SPI2SEL_APLL0_DIV2       (0x2UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from APLL0/2 \hideinitializer */
#define CLK_SPISEL_SPI2SEL_PCLK0            (0x3UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from PCLK0 \hideinitializer */
#define CLK_SPISEL_SPI2SEL_HIRC             (0x4UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from HIRC \hideinitializer */
#define CLK_SPISEL_SPI2SEL_HIRC48M          (0x5UL << CLK_SPISEL_SPI2SEL_Pos)           /*!< Select SPI2 clock source from HIRC48M \hideinitializer */

#define CLK_SPISEL_SPI3SEL_HXT              (0x0UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from HXT \hideinitializer */
#define CLK_SPISEL_SPI3SEL_APLL1_DIV2       (0x1UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from APLL1/2 \hideinitializer */
#define CLK_SPISEL_SPI3SEL_APLL0_DIV2       (0x2UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from APLL0/2 \hideinitializer */
#define CLK_SPISEL_SPI3SEL_PCLK2            (0x3UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from PCLK0 \hideinitializer */
#define CLK_SPISEL_SPI3SEL_HIRC             (0x4UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from HIRC \hideinitializer */
#define CLK_SPISEL_SPI3SEL_HIRC48M          (0x5UL << CLK_SPISEL_SPI3SEL_Pos)           /*!< Select SPI3 clock source from HIRC48M \hideinitializer */

#define CLK_STSEL_ST0SEL_HXT                (0x0UL << CLK_STSEL_ST0SEL_Pos)             /*!< Select SysTick 0 clock source from HXT \hideinitializer */
#define CLK_STSEL_ST0SEL_HXT_DIV2           (0x1UL << CLK_STSEL_ST0SEL_Pos)             /*!< Select SysTick 0 clock source from HXT/2 \hideinitializer */
#define CLK_STSEL_ST0SEL_ACLK_DIV2          (0x2UL << CLK_STSEL_ST0SEL_Pos)             /*!< Select SysTick 0 clock source from ACLK/2 \hideinitializer */
#define CLK_STSEL_ST0SEL_HIRC_DIV2          (0x3UL << CLK_STSEL_ST0SEL_Pos)             /*!< Select SysTick 0 clock source from HIRC/2 \hideinitializer */
#define CLK_STSEL_ACLK                      (0x8UL)                                     /*!< Select SysTick 0 clock source from ACLK \hideinitializer */

#define CLK_TMRSEL_TMR0SEL_HXT              (0x0UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from HXT \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_LXT              (0x1UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from LXT \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_PCLK1            (0x2UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from PCLK1 \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_EXT              (0x3UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from external trigger \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_LIRC             (0x4UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from LIRC \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_HIRC             (0x5UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from HIRC \hideinitializer */
#define CLK_TMRSEL_TMR0SEL_HIRC48M_DIV4     (0x6UL << CLK_TMRSEL_TMR0SEL_Pos)           /*!< Select TMR0 clock source from HIRC48M/4 \hideinitializer */

#define CLK_TMRSEL_TMR1SEL_HXT              (0x0UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from HXT \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_LXT              (0x1UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from LXT \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_PCLK1            (0x2UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from PCLK1 \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_EXT              (0x3UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from external trigger \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_LIRC             (0x4UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from LIRC \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_HIRC             (0x5UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from HIRC \hideinitializer */
#define CLK_TMRSEL_TMR1SEL_HIRC48M_DIV4     (0x6UL << CLK_TMRSEL_TMR1SEL_Pos)           /*!< Select TMR1 clock source from HIRC48M/4 \hideinitializer */

#define CLK_TMRSEL_TMR2SEL_HXT              (0x0UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from HXT \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_LXT              (0x1UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from LXT \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_PCLK3            (0x2UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from PCLK3 \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_EXT              (0x3UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from external trigger \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_LIRC             (0x4UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from LIRC \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_HIRC             (0x5UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from HIRC \hideinitializer */
#define CLK_TMRSEL_TMR2SEL_HIRC48M_DIV4     (0x6UL << CLK_TMRSEL_TMR2SEL_Pos)           /*!< Select TMR2 clock source from HIRC48M/4 \hideinitializer */

#define CLK_TMRSEL_TMR3SEL_HXT              (0x0UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from HXT \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_LXT              (0x1UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from LXT \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_PCLK3            (0x2UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from PCLK3 \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_EXT              (0x3UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from external trigger \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_LIRC             (0x4UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from LIRC \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_HIRC             (0x5UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from HIRC \hideinitializer */
#define CLK_TMRSEL_TMR3SEL_HIRC48M_DIV4     (0x6UL << CLK_TMRSEL_TMR3SEL_Pos)           /*!< Select TMR3 clock source from HIRC48M/4 \hideinitializer */

#define CLK_TTMRSEL_TTMR0SEL_PCLK4          (0x0UL << CLK_TTMRSEL_TTMR0SEL_Pos)         /*!< Select TTMR0 clock source from PCLK4 \hideinitializer */
#define CLK_TTMRSEL_TTMR0SEL_LXT            (0x1UL << CLK_TTMRSEL_TTMR0SEL_Pos)         /*!< Select TTMR0 clock source from LXT \hideinitializer */
#define CLK_TTMRSEL_TTMR0SEL_LIRC           (0x2UL << CLK_TTMRSEL_TTMR0SEL_Pos)         /*!< Select TTMR0 clock source from LIRC \hideinitializer */
#define CLK_TTMRSEL_TTMR0SEL_MIRC           (0x3UL << CLK_TTMRSEL_TTMR0SEL_Pos)         /*!< Select TTMR0 clock source from MIRC \hideinitializer */
#define CLK_TTMRSEL_TTMR0SEL_HIRC           (0x4UL << CLK_TTMRSEL_TTMR0SEL_Pos)         /*!< Select TTMR0 clock source from HIRC \hideinitializer */

#define CLK_TTMRSEL_TTMR1SEL_PCLK4          (0x0UL << CLK_TTMRSEL_TTMR1SEL_Pos)         /*!< Select TTMR1 clock source from PCLK4 \hideinitializer */
#define CLK_TTMRSEL_TTMR1SEL_LXT            (0x1UL << CLK_TTMRSEL_TTMR1SEL_Pos)         /*!< Select TTMR1 clock source from LXT \hideinitializer */
#define CLK_TTMRSEL_TTMR1SEL_LIRC           (0x2UL << CLK_TTMRSEL_TTMR1SEL_Pos)         /*!< Select TTMR1 clock source from LIRC \hideinitializer */
#define CLK_TTMRSEL_TTMR1SEL_MIRC           (0x3UL << CLK_TTMRSEL_TTMR1SEL_Pos)         /*!< Select TTMR1 clock source from MIRC \hideinitializer */
#define CLK_TTMRSEL_TTMR1SEL_HIRC           (0x4UL << CLK_TTMRSEL_TTMR1SEL_Pos)         /*!< Select TTMR1 clock source from HIRC \hideinitializer */

#define CLK_UARTSEL0_UART0SEL_HXT           (0x0UL << CLK_UARTSEL0_UART0SEL_Pos)        /*!< Select UART0 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART0SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART0SEL_Pos)        /*!< Select UART0 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART0SEL_LXT           (0x2UL << CLK_UARTSEL0_UART0SEL_Pos)        /*!< Select UART0 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART0SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART0SEL_Pos)        /*!< Select UART0 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART0SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART0SEL_Pos)        /*!< Select UART0 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART1SEL_HXT           (0x0UL << CLK_UARTSEL0_UART1SEL_Pos)        /*!< Select UART1 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART1SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART1SEL_Pos)        /*!< Select UART1 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART1SEL_LXT           (0x2UL << CLK_UARTSEL0_UART1SEL_Pos)        /*!< Select UART1 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART1SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART1SEL_Pos)        /*!< Select UART1 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART1SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART1SEL_Pos)        /*!< Select UART1 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART2SEL_HXT           (0x0UL << CLK_UARTSEL0_UART2SEL_Pos)        /*!< Select UART2 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART2SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART2SEL_Pos)        /*!< Select UART2 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART2SEL_LXT           (0x2UL << CLK_UARTSEL0_UART2SEL_Pos)        /*!< Select UART2 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART2SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART2SEL_Pos)        /*!< Select UART2 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART2SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART2SEL_Pos)        /*!< Select UART2 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART3SEL_HXT           (0x0UL << CLK_UARTSEL0_UART3SEL_Pos)        /*!< Select UART3 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART3SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART3SEL_Pos)        /*!< Select UART3 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART3SEL_LXT           (0x2UL << CLK_UARTSEL0_UART3SEL_Pos)        /*!< Select UART3 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART3SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART3SEL_Pos)        /*!< Select UART3 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART3SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART3SEL_Pos)        /*!< Select UART3 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART4SEL_HXT           (0x0UL << CLK_UARTSEL0_UART4SEL_Pos)        /*!< Select UART4 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART4SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART4SEL_Pos)        /*!< Select UART4 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART4SEL_LXT           (0x2UL << CLK_UARTSEL0_UART4SEL_Pos)        /*!< Select UART4 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART4SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART4SEL_Pos)        /*!< Select UART4 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART4SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART4SEL_Pos)        /*!< Select UART4 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART5SEL_HXT           (0x0UL << CLK_UARTSEL0_UART5SEL_Pos)        /*!< Select UART5 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART5SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART5SEL_Pos)        /*!< Select UART5 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART5SEL_LXT           (0x2UL << CLK_UARTSEL0_UART5SEL_Pos)        /*!< Select UART5 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART5SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART5SEL_Pos)        /*!< Select UART5 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART5SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART5SEL_Pos)        /*!< Select UART5 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART6SEL_HXT           (0x0UL << CLK_UARTSEL0_UART6SEL_Pos)        /*!< Select UART6 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART6SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART6SEL_Pos)        /*!< Select UART6 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART6SEL_LXT           (0x2UL << CLK_UARTSEL0_UART6SEL_Pos)        /*!< Select UART6 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART6SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART6SEL_Pos)        /*!< Select UART6 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART6SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART6SEL_Pos)        /*!< Select UART6 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL0_UART7SEL_HXT           (0x0UL << CLK_UARTSEL0_UART7SEL_Pos)        /*!< Select UART7 clock source from HXT \hideinitializer */
#define CLK_UARTSEL0_UART7SEL_HIRC          (0x1UL << CLK_UARTSEL0_UART7SEL_Pos)        /*!< Select UART7 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL0_UART7SEL_LXT           (0x2UL << CLK_UARTSEL0_UART7SEL_Pos)        /*!< Select UART7 clock source from LXT \hideinitializer */
#define CLK_UARTSEL0_UART7SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL0_UART7SEL_Pos)        /*!< Select UART7 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL0_UART7SEL_HIRC48M       (0x4UL << CLK_UARTSEL0_UART7SEL_Pos)        /*!< Select UART7 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL1_UART8SEL_HXT           (0x0UL << CLK_UARTSEL1_UART8SEL_Pos)        /*!< Select UART8 clock source from HXT \hideinitializer */
#define CLK_UARTSEL1_UART8SEL_HIRC          (0x1UL << CLK_UARTSEL1_UART8SEL_Pos)        /*!< Select UART8 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL1_UART8SEL_LXT           (0x2UL << CLK_UARTSEL1_UART8SEL_Pos)        /*!< Select UART8 clock source from LXT \hideinitializer */
#define CLK_UARTSEL1_UART8SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL1_UART8SEL_Pos)        /*!< Select UART8 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL1_UART8SEL_HIRC48M       (0x4UL << CLK_UARTSEL1_UART8SEL_Pos)        /*!< Select UART8 clock source from HIRC48M \hideinitializer */

#define CLK_UARTSEL1_UART9SEL_HXT           (0x0UL << CLK_UARTSEL1_UART9SEL_Pos)        /*!< Select UART9 clock source from HXT \hideinitializer */
#define CLK_UARTSEL1_UART9SEL_HIRC          (0x1UL << CLK_UARTSEL1_UART9SEL_Pos)        /*!< Select UART9 clock source from HIRC \hideinitializer */
#define CLK_UARTSEL1_UART9SEL_LXT           (0x2UL << CLK_UARTSEL1_UART9SEL_Pos)        /*!< Select UART9 clock source from LXT \hideinitializer */
#define CLK_UARTSEL1_UART9SEL_APLL0_DIV2    (0x3UL << CLK_UARTSEL1_UART9SEL_Pos)        /*!< Select UART9 clock source from APLL0/2 \hideinitializer */
#define CLK_UARTSEL1_UART9SEL_HIRC48M       (0x4UL << CLK_UARTSEL1_UART9SEL_Pos)        /*!< Select UART9 clock source from HIRC48M \hideinitializer */

#define CLK_USBSEL_USBSEL_HIRC48M           (0x0UL << CLK_USBSEL_USBSEL_Pos)            /*!< Select USB clock source from HIRC48M \hideinitializer */
#define CLK_USBSEL_USBSEL_APLL1_DIV2        (0x1UL << CLK_USBSEL_USBSEL_Pos)            /*!< Select USB clock source from APLL1/2 \hideinitializer */

#define CLK_WDTSEL_WDT0SEL_LXT              (0x0UL << CLK_WDTSEL_WDT0SEL_Pos)           /*!< Select WDT0 clock source from LXT \hideinitializer */
#define CLK_WDTSEL_WDT0SEL_LIRC             (0x1UL << CLK_WDTSEL_WDT0SEL_Pos)           /*!< Select WDT0 clock source from LIRC \hideinitializer */

#define CLK_WDTSEL_WDT1SEL_LXT              (0x0UL << CLK_WDTSEL_WDT1SEL_Pos)           /*!< Select WDT1 clock source from LXT \hideinitializer */
#define CLK_WDTSEL_WDT1SEL_LIRC             (0x1UL << CLK_WDTSEL_WDT1SEL_Pos)           /*!< Select WDT1 clock source from LIRC \hideinitializer */

#define CLK_WWDTSEL_WWDT0SEL_LIRC           (0x0UL << CLK_WWDTSEL_WWDT0SEL_Pos)         /*!< Select WWDT0 clock source from LIRC \hideinitializer */
#define CLK_WWDTSEL_WWDT0SEL_LXT            (0x1UL << CLK_WWDTSEL_WWDT0SEL_Pos)         /*!< Select WWDT0 clock source from LXT \hideinitializer */

#define CLK_WWDTSEL_WWDT1SEL_LIRC           (0x0UL << CLK_WWDTSEL_WWDT1SEL_Pos)         /*!< Select WWDT1 clock source from LIRC \hideinitializer */
#define CLK_WWDTSEL_WWDT1SEL_LXT            (0x1UL << CLK_WWDTSEL_WWDT1SEL_Pos)         /*!< Select WWDT1 clock source from LXT \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_SCLKDIV_SCLKDIV(x)              (((x) - 1UL) << CLK_SCLKDIV_SCLKDIV_Pos)        /*!< SCLKDIV Setting for SCLK clock divider. It could be 1~16 \hideinitializer */
#define CLK_HCLKDIV_HCLK2DIV(x)             (((x) - 1UL) << CLK_HCLKDIV_HCLK2DIV_Pos)       /*!< HCLKDIV Setting for HCLK2 clock divider. It could be 1~16 \hideinitializer */
#define CLK_PCLKDIV_PCLK0DIV(x)             (((x) - 1UL) << CLK_PCLKDIV_PCLK0DIV_Pos)       /*!< PCLKDIV Setting for PCLK0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_PCLKDIV_PCLK1DIV(x)             (((x) - 1UL) << CLK_PCLKDIV_PCLK1DIV_Pos)       /*!< PCLKDIV Setting for PCLK1 clock divider. It could be 1~16 \hideinitializer */
#define CLK_PCLKDIV_PCLK2DIV(x)             (((x) - 1UL) << CLK_PCLKDIV_PCLK2DIV_Pos)       /*!< PCLKDIV Setting for PCLK2 clock divider. It could be 1~16 \hideinitializer */
#define CLK_PCLKDIV_PCLK3DIV(x)             (((x) - 1UL) << CLK_PCLKDIV_PCLK3DIV_Pos)       /*!< PCLKDIV Setting for PCLK3 clock divider. It could be 1~16 \hideinitializer */
#define CLK_PCLKDIV_PCLK4DIV(x)             (((x) - 1UL) << CLK_PCLKDIV_PCLK4DIV_Pos)       /*!< PCLKDIV Setting for PCLK4 clock divider. It could be 1~16 \hideinitializer */

#define CLK_STDIV_ST0DIV(x)                 (((x) - 1UL) << CLK_STDIV_ST0DIV_Pos)           /*!< STDIV Setting for ST0 clock divider. It could be 1~256 \hideinitializer */

#define CLK_CANFDDIV_CANFD0DIV(x)           (((x) - 1UL) << CLK_CANFDDIV_CANFD0DIV_Pos)     /*!< CANFDDIV Setting for CANFD0 clock divider. It could be 1~256 \hideinitializer */
#define CLK_CANFDDIV_CANFD1DIV(x)           (((x) - 1UL) << CLK_CANFDDIV_CANFD1DIV_Pos)     /*!< CANFDDIV Setting for CANFD1 clock divider. It could be 1~256 \hideinitializer */

#define CLK_DMICDIV_DMIC0DIV(x)            (((x) - 1UL) << CLK_DMICDIV_DMIC0DIV_Pos)        /*!< DMICDIV Setting for DMIC0 clock divider. It could be 1~256 \hideinitializer */

#define CLK_EADCDIV_EADC0DIV(x)             (((x) - 1UL) << CLK_EADCDIV_EADC0DIV_Pos)       /*!< CANFDDIV Setting for EADC0 clock divider. It could be 1~256 \hideinitializer */

#define CLK_I2SDIV_I2S0DIV(x)               (((x) - 1UL) << CLK_I2SDIV_I2S0DIV_Pos)         /*!< I2SDIV Setting for I2S0 clock divider. It could be 1~256 \hideinitializer */
#define CLK_I2SDIV_I2S1DIV(x)               (((x) - 1UL) << CLK_I2SDIV_I2S1DIV_Pos)         /*!< I2SDIV Setting for I2S1 clock divider. It could be 1~256 \hideinitializer */

#define CLK_KPIDIV_KPI0DIV(x)               (((x) - 1UL) << CLK_KPIDIV_KPI0DIV_Pos)         /*!< KPIDIV Setting for KPI clock divider. It could be 1~256 \hideinitializer */

#define CLK_LPADCDIV_LPADC0DIV(x)           (((x) - 1UL) << CLK_LPADCDIV_LPADC0DIV_Pos)     /*!< LPADCDIV Setting for LPADC clock divider. It could be 1~16 \hideinitializer */

#define CLK_LPUARTDIV_LPUART0DIV(x)         (((x) - 1UL) << CLK_LPUARTDIV_LPUART0DIV_Pos)   /*!< LPUARTDIV Setting for LPUART clock divider. It could be 1~16 \hideinitializer */

#define CLK_PSIODIV_PSIO0DIV(x)             (((x) - 1UL) << CLK_PSIODIV_PSIO0DIV_Pos)       /*!< PSIODIV Setting for PSIO clock divider. It could be 1~16 \hideinitializer */

#define CLK_SCDIV_SC0DIV(x)                 (((x) - 1UL) << CLK_SCDIV_SC0DIV_Pos)           /*!< SCDIV Setting for SC0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_SCDIV_SC1DIV(x)                 (((x) - 1UL) << CLK_SCDIV_SC1DIV_Pos)           /*!< SCDIV Setting for SC1 clock divider. It could be 1~16 \hideinitializer */
#define CLK_SCDIV_SC2DIV(x)                 (((x) - 1UL) << CLK_SCDIV_SC2DIV_Pos)           /*!< SCDIV Setting for SC2 clock divider. It could be 1~16 \hideinitializer */

#define CLK_SDHDIV_SDH0DIV(x)               (((x) - 1UL) << CLK_SDHDIV_SDH0DIV_Pos)         /*!< SDHDIV Setting for SDH0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_SDHDIV_SDH1DIV(x)               (((x) - 1UL) << CLK_SDHDIV_SDH1DIV_Pos)         /*!< SDHDIV Setting for SDH1 clock divider. It could be 1~16 \hideinitializer */

#define CLK_UARTDIV0_UART0DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART0DIV_Pos)      /*!< UARTDIV0 Setting for UART0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART1DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART1DIV_Pos)      /*!< UARTDIV0 Setting for UART1 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART2DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART2DIV_Pos)      /*!< UARTDIV0 Setting for UART2 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART3DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART3DIV_Pos)      /*!< UARTDIV0 Setting for UART3 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART4DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART4DIV_Pos)      /*!< UARTDIV0 Setting for UART4 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART5DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART5DIV_Pos)      /*!< UARTDIV0 Setting for UART5 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART6DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART6DIV_Pos)      /*!< UARTDIV0 Setting for UART6 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV0_UART7DIV(x)            (((x) - 1UL) << CLK_UARTDIV0_UART7DIV_Pos)      /*!< UARTDIV0 Setting for UART7 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV1_UART8DIV(x)            (((x) - 1UL) << CLK_UARTDIV1_UART8DIV_Pos)      /*!< UARTDIV1 Setting for UART8 clock divider. It could be 1~16 \hideinitializer */
#define CLK_UARTDIV1_UART9DIV(x)            (((x) - 1UL) << CLK_UARTDIV1_UART9DIV_Pos)      /*!< UARTDIV1 Setting for UART9 clock divider. It could be 1~16 \hideinitializer */

#define CLK_USBDIV_USBDIV(x)                (((x) - 1UL) << CLK_USBDIV_USBDIV_Pos)          /*!< USBDIV Setting for USB clock divider. It could be 1~16 \hideinitializer */

#define CLK_VSENSEDIV_VSENSEDIV(x)          (((x) - 1UL) << CLK_VSENSEDIV_VSENSEDIV_Pos)    /*!< VSENSEDIV Setting for VSENSE clock divider. It could be 1~256 \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  APLLCTL constant definitions. APLL = FIN * 2 * NF / NR / NO                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_APLL0_SELECT     0UL
#define CLK_APLL1_SELECT     1UL

#define CLK_APLLCTL_APLLSRC_HXT           0x00000000UL    /*!< For APLL clock source is HXT.       4MHz < FIN/NR < 8MHz \hideinitializer */
#define CLK_APLLCTL_APLLSRC_HXT_DIV2      0x00000001UL    /*!< For APLL clock source is HXT/2.     4MHz < FIN/NR < 8MHz \hideinitializer */
#define CLK_APLLCTL_APLLSRC_HIRC          0x00000002UL    /*!< For APLL clock source is HIRC.      4MHz < FIN/NR < 8MHz \hideinitializer */
#define CLK_APLLCTL_APLLSRC_HIRC48_DIV4   0x00000003UL    /*!< For APLL clock source is HIRC48/4.  4MHz < FIN/NR < 8MHz \hideinitializer */

#define CLK_APLLCTL_NF(x)        (((x)-2UL))       /*!< x must be constant and 2 <= x <= 513.    200MHz < FIN*2*NF/NR < 500MHz. \hideinitializer */
#define CLK_APLLCTL_NR(x)        (((x)-1UL)<<9)    /*!< x must be constant and 1 <= x <= 32.     4MHz < FIN/NR < 8MHz \hideinitializer */

#define CLK_APLLCTL_NO_1         0x0000UL        /*!< For output divider is 1 \hideinitializer */
#define CLK_APLLCTL_NO_2         0x4000UL        /*!< For output divider is 2 \hideinitializer */
#define CLK_APLLCTL_NO_4         0xC000UL        /*!< For output divider is 4 \hideinitializer */

#define CLK_APLLCTL_72MHz           (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 36UL) | CLK_APLLCTL_NO_4) /*!< Predefined APLLCTL setting for 72MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_80MHz           (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 40UL) | CLK_APLLCTL_NO_4) /*!< Predefined APLLCTL setting for 80MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_144MHz          (CLK_APLLCTL_NR(2UL) | CLK_APLLCTL_NF( 24UL) | CLK_APLLCTL_NO_2) /*!< Predefined APLLCTL setting for 144MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_160MHz          (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 40UL) | CLK_APLLCTL_NO_2) /*!< Predefined APLLCTL setting for 160MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_180MHz          (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 45UL) | CLK_APLLCTL_NO_2) /*!< Predefined APLLCTL setting for 180MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_192MHz          (CLK_APLLCTL_NR(2UL) | CLK_APLLCTL_NF( 32UL) | CLK_APLLCTL_NO_2) /*!< Predefined APLLCTL setting for 192MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_200MHz          (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 25UL) | CLK_APLLCTL_NO_1) /*!< Predefined APLLCTL setting for 200MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_220MHz          (CLK_APLLCTL_NR(3UL) | CLK_APLLCTL_NF( 55UL) | CLK_APLLCTL_NO_2) /*!< Predefined APLLCTL setting for 200MHz APLL output with 12MHz \hideinitializer */
#define CLK_APLLCTL_384MHz          (CLK_APLLCTL_NR(2UL) | CLK_APLLCTL_NF( 32UL) | CLK_APLLCTL_NO_1) /*!< Predefined APLLCTL setting for 384MHz APLL output with 12MHz \hideinitializer */

#define CLK_APLLCTL_STBSEL_820     (0UL << CLK_APLL0CTL_STBSEL_Pos)    /*!< Select APLL stable time is 820 APLL source clock \hideinitializer */
#define CLK_APLLCTL_STBSEL_2460    (1UL << CLK_APLL0CTL_STBSEL_Pos)    /*!< Select APLL stable time is 2460 APLL source clock \hideinitializer */
#define CLK_APLLCTL_STBSEL_4920    (2UL << CLK_APLL0CTL_STBSEL_Pos)    /*!< Select APLL stable time is 4920 APLL source clock \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_CLKCTL_BASE      (0x40001200UL)
#define MODULE_CLKSEL_BASE      (0x40001400UL)
#define MODULE_CLKDIV_BASE      (0x40001500UL)

#define MODULE_CLKCTL(x)        (((x) >> 47) & 0xffUL)   /*!< Calculate CLKCTL offset \hideinitializer */
#define MODULE_CLKSEL(x)        (((x) >> 39) & 0xffUL)   /*!< Calculate CLKSEL offset \hideinitializer */
#define MODULE_CLKDIV(x)        (((x) >> 31) & 0xffUL)   /*!< Calculate CLKDIV offset \hideinitializer */
#define MODULE_CLKSEL_Msk(x)    (((x) >> 23) & 0xffUL)   /*!< Calculate CLKSEL mask offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Msk(x)    (((x) >> 15) & 0xffUL)   /*!< Calculate CLKDIV mask offset on MODULE index \hideinitializer */
#define MODULE_CLKEN_Pos(x)     (((x) >> 10) & 0x1fUL)   /*!< Calculate CLKEN offset on MODULE index \hideinitializer */
#define MODULE_CLKSEL_Pos(x)    (((x) >>  5) & 0x1fUL)   /*!< Calculate CLKSEL position offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Pos(x)    (((x) >>  0) & 0x1fUL)   /*!< Calculate CLKDIV position offset on MODULE index \hideinitializer */

#define MODULE_NoMsk            0x0ULL                   /*!< Not mask on MODULE index \hideinitializer */
#define NA                      MODULE_NoMsk             /*!< Not Available \hideinitializer */

#define MODULE_CLKCTL_ENC(x)        (((x) & 0xffUL) << 47)   /*!< CLKCTL offset \hideinitializer */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0xffUL) << 39)   /*!< CLKSEL offset \hideinitializer */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0xffUL) << 31)   /*!< CLKDIV offset \hideinitializer */
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0xffUL) << 23)   /*!< CLKSEL mask offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xffUL) << 15)   /*!< CLKDIV mask offset on MODULE index \hideinitializer */
#define MODULE_CLKEN_Pos_ENC(x)     (((x) & 0x1fUL) << 10)   /*!< CLKEN offset on MODULE index \hideinitializer */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1fUL) <<  5)   /*!< CLKSEL position offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1fUL) <<  0)   /*!< CLKDIV position offset on MODULE index \hideinitializer */

#define ACMP01_MODULE   (MODULE_CLKCTL_ENC( 0ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ACMPCTL_ACMP01CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ACMP01 Module */

#define ACMP23_MODULE   (MODULE_CLKCTL_ENC( 0ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ACMPCTL_ACMP23CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ACMP23 Module */

#define AWF0_MODULE     (MODULE_CLKCTL_ENC( 1ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_AWFCTL_AWF0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< AWF0 Module */

#define BPWM0_MODULE    (MODULE_CLKCTL_ENC( 2ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_BPWMCTL_BPWM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 1ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_BPWMSEL_BPWM0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< BPWM0 Module */

#define BPWM1_MODULE    (MODULE_CLKCTL_ENC( 2ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_BPWMCTL_BPWM1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 1ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_BPWMSEL_BPWM1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< BPWM1 Module */

#define CANFD0_MODULE   (MODULE_CLKCTL_ENC( 3ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CANFDCTL_CANFD0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 2ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_CANFDSEL_CANFD0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 4ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_CANFDDIV_CANFD0DIV_Pos))      /*!< CANFD0 Module */

#define CANFD1_MODULE   (MODULE_CLKCTL_ENC( 3ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CANFDCTL_CANFD1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 2ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_CANFDSEL_CANFD1SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 4ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_CANFDDIV_CANFD1DIV_Pos))      /*!< CANFD1 Module */

#define CCAP0_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CCAPCTL_CCAP0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_CCAPSEL_CCAP0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< CCAP0 Module */

#define CRC0_MODULE     (MODULE_CLKCTL_ENC( 5ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CRCCTL_CRC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< CRC0 Module */

#define CRYPTO0_MODULE  (MODULE_CLKCTL_ENC( 6ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CRYPTOCTL_CRYPTO0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< CRYPTO0 Module */

#define DAC01_MODULE    (MODULE_CLKCTL_ENC( 7ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_DACCTL_DAC01CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< DAC01 Module */

#define DMIC0_MODULE    (MODULE_CLKCTL_ENC( 8ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_DMICCTL_DMIC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_DMICSEL_DMIC0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_DMICDIV_DMIC0DIV_Pos))        /*!< DMIC0 Module */

#define VAD0SEL_MODULE  (MODULE_CLKCTL_ENC( 8ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_DMICCTL_DMIC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_DMICSEL_VAD0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(NA)|MODULE_CLKDIV_Pos_ENC(NA))                                             /*!< VAD0SEL Module */

#define EADC0_MODULE    (MODULE_CLKCTL_ENC( 9ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EADCCTL_EADC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 6ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_EADCSEL_EADC0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 6ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_EADCDIV_EADC0DIV_Pos))        /*!< EADC0 Module */

#define EBI0_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EBICTL_EBI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EBI0 Module */

#define ECAP0_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ECAPCTL_ECAP0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ECAP0 Module */

#define ECAP1_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ECAPCTL_ECAP1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ECAP1 Module */

#define ECAP2_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ECAPCTL_ECAP2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ECAP2 Module */

#define ECAP3_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ECAPCTL_ECAP3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ECAP3 Module */

#define EMAC0_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EMACCTL_EMAC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EMAC0 Module */

#define EPWM0_MODULE    (MODULE_CLKCTL_ENC(13ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EPWMCTL_EPWM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 7ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_EPWMSEL_EPWM0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EPWM0 Module */

#define EPWM1_MODULE    (MODULE_CLKCTL_ENC(13ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EPWMCTL_EPWM1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 7ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_EPWMSEL_EPWM1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EPWM1 Module */

#define EQEI0_MODULE    (MODULE_CLKCTL_ENC(14ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EQEICTL_EQEI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EQEI0 Module */

#define EQEI1_MODULE    (MODULE_CLKCTL_ENC(14ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EQEICTL_EQEI1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EQEI1 Module */

#define EQEI2_MODULE    (MODULE_CLKCTL_ENC(14ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EQEICTL_EQEI2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EQEI2 Module */

#define EQEI3_MODULE    (MODULE_CLKCTL_ENC(14ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_EQEICTL_EQEI3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< EQEI3 Module */

#define FMC0_MODULE     (MODULE_CLKCTL_ENC(15ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_FMCCTL_FMC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 8ULL)|MODULE_CLKSEL_Msk_ENC(   1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_FMCSEL_FMC0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< FMC0 Module */

#define ISP0_MODULE     (MODULE_CLKCTL_ENC(15ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_FMCCTL_ISP0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ISP0 Module */

#define GDMA0_MODULE    (MODULE_CLKCTL_ENC(16ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GDMACTL_GDMA0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GDMA0 Module */

#define GPIOA_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOACKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOA Module */

#define GPIOB_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOBCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOB Module */

#define GPIOC_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOCCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOC Module */

#define GPIOD_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIODCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOD Module */

#define GPIOE_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOECKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOE Module */

#define GPIOF_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOFCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOF Module */

#define GPIOG_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOGCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOG Module */

#define GPIOH_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOHCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOH Module */

#define GPIOI_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOICKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOI Module */

#define GPIOJ_MODULE    (MODULE_CLKCTL_ENC(17ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOJCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< GPIOJ Module */

#define HSOTG0_MODULE   (MODULE_CLKCTL_ENC(18ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_HSOTGCTL_HSOTG0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< HSOTG0 Module */

#define HSUSBD0_MODULE  (MODULE_CLKCTL_ENC(19ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_HSUSBDCTL_HSUSBD0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< HSUSBD0 Module */

#define HSUSBH0_MODULE  (MODULE_CLKCTL_ENC(20ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_HSUSBHCTL_HSUSBH0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< HSUSBH0 Module */

#define I2C0_MODULE     (MODULE_CLKCTL_ENC(21ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< I2C0 Module */

#define I2C1_MODULE     (MODULE_CLKCTL_ENC(21ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< I2C1 Module */

#define I2C2_MODULE     (MODULE_CLKCTL_ENC(21ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< I2C2 Module */

#define I2C3_MODULE     (MODULE_CLKCTL_ENC(21ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< I2C3 Module */

#define I2S0_MODULE     (MODULE_CLKCTL_ENC(22ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2SCTL_I2S0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 9ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_I2SSEL_I2S0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 7ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_I2SDIV_I2S0DIV_Pos))          /*!< I2S0 Module */

#define I2S1_MODULE     (MODULE_CLKCTL_ENC(22ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2SCTL_I2S1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 9ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_I2SSEL_I2S1SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 7ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_I2SDIV_I2S1DIV_Pos))          /*!< I2S1 Module */

#define I3C0_MODULE     (MODULE_CLKCTL_ENC(23ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I3CCTL_I3C0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(10ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_I3CSEL_I3C0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< I3C0 Module */

#define KDF0_MODULE     (MODULE_CLKCTL_ENC(24ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_KDFCTL_KDF0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< KDF0 Module */

#define KPI0_MODULE     (MODULE_CLKCTL_ENC(25ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_KPICTL_KPI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(11ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_KPISEL_KPI0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 8ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_KPIDIV_KPI0DIV_Pos))          /*!< KPI0 Module */

#define KS0_MODULE      (MODULE_CLKCTL_ENC(26ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_KSCTL_KS0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< KS0 Module */

#define LPADC0_MODULE   (MODULE_CLKCTL_ENC(27ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPADCCTL_LPADC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(12ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_LPADCSEL_LPADC0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 9ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_LPADCDIV_LPADC0DIV_Pos))      /*!< LPADC0 Module */

#define LPPDMA0_MODULE  (MODULE_CLKCTL_ENC(28ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPPDMACTL_LPPDMA0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPPDMA0 Module */

#define LPGPIO0_MODULE  (MODULE_CLKCTL_ENC(29ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPGPIOCTL_LPGPIO0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPGPIO0 Module */

#define LPI2C0_MODULE   (MODULE_CLKCTL_ENC(30ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPI2CCTL_LPI2C0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPI2C0 Module */

#define LPSPI0_MODULE   (MODULE_CLKCTL_ENC(31ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPSPICTL_LPSPI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(13ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_LPSPISEL_LPSPI0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPSPI0 Module */

#define LPSRAM0_MODULE  (MODULE_CLKCTL_ENC(32ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPSRAMCTL_LPSRAM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPSRAM0 Module */

#define LPTMR0_MODULE   (MODULE_CLKCTL_ENC(33ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPTMRCTL_LPTMR0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(14ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_LPTMRSEL_LPTMR0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPTMR0 Module */

#define LPTMR1_MODULE   (MODULE_CLKCTL_ENC(33ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPTMRCTL_LPTMR1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(14ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_LPTMRSEL_LPTMR1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< LPTMR1 Module */

#define LPUART0_MODULE  (MODULE_CLKCTL_ENC(34ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_LPUARTCTL_LPUART0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(15ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_LPUARTSEL_LPUART0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(10ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_LPUARTDIV_LPUART0DIV_Pos))    /*!< LPUART0 Module */

#define NPU0_MODULE     (MODULE_CLKCTL_ENC(35ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_NPUCTL_NPU0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< NPU0 Module */

#define OTFC0_MODULE    (MODULE_CLKCTL_ENC(37ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_OTFCCTL_OTFC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< OTFC0 Module */

#define OTG0_MODULE     (MODULE_CLKCTL_ENC(38ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_OTGCTL_OTG0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< OTG0 Module */

#define PDMA0_MODULE    (MODULE_CLKCTL_ENC(39ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_PDMACTL_PDMA0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< PDMA0 Module */

#define PDMA1_MODULE    (MODULE_CLKCTL_ENC(39ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_PDMACTL_PDMA1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< PDMA1 Module */

#define PSIO0_MODULE    (MODULE_CLKCTL_ENC(40ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_PSIOCTL_PSIO0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(16ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_PSIOSEL_PSIO0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(11ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_PSIODIV_PSIO0DIV_Pos))        /*!< PSIO0 Module */

#define QSPI0_MODULE    (MODULE_CLKCTL_ENC(41ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_QSPICTL_QSPI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(17ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_QSPISEL_QSPI0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< QSPI0 Module */

#define QSPI1_MODULE    (MODULE_CLKCTL_ENC(41ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_QSPICTL_QSPI1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(17ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_QSPISEL_QSPI1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< QSPI1 Module */

#define RTC0_MODULE     (MODULE_CLKCTL_ENC(42ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_RTCCTL_RTC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< RTC0 Module */

#define SC0_MODULE      (MODULE_CLKCTL_ENC(43ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SCCTL_SC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(18ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SCSEL_SC0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(12ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_SCDIV_SC0DIV_Pos))            /*!< SC0 Module */

#define SC1_MODULE      (MODULE_CLKCTL_ENC(43ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SCCTL_SC1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(18ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SCSEL_SC1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(12ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_SCDIV_SC1DIV_Pos))            /*!< SC1 Module */

#define SC2_MODULE      (MODULE_CLKCTL_ENC(43ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SCCTL_SC2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(18ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SCSEL_SC2SEL_Pos)|\
                         MODULE_CLKDIV_ENC(12ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_SCDIV_SC2DIV_Pos))            /*!< SC2 Module */

#define SCU0_MODULE     (MODULE_CLKCTL_ENC(44ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SCUCTL_SCU0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SCU0 Module */

#define SDH0_MODULE     (MODULE_CLKCTL_ENC(45ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SDHCTL_SDH0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(19ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SDHSEL_SDH0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(13ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_SDHDIV_SDH0DIV_Pos))          /*!< SDH0 Module */

#define SDH1_MODULE     (MODULE_CLKCTL_ENC(45ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SDHCTL_SDH1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(19ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SDHSEL_SDH1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(13ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_SDHDIV_SDH1DIV_Pos))          /*!< SDH1 Module */

#define SPI0_MODULE     (MODULE_CLKCTL_ENC(46ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SPICTL_SPI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(20ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SPISEL_SPI0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SPI0 Module */

#define SPI1_MODULE     (MODULE_CLKCTL_ENC(46ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SPICTL_SPI1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(20ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SPISEL_SPI1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SPI1 Module */

#define SPI2_MODULE     (MODULE_CLKCTL_ENC(46ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SPICTL_SPI2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(20ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SPISEL_SPI2SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SPI2 Module */

#define SPI3_MODULE     (MODULE_CLKCTL_ENC(46ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SPICTL_SPI3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(20ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_SPISEL_SPI3SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SPI3 Module */

#define SPIM0_MODULE    (MODULE_CLKCTL_ENC(47ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SPIMCTL_SPIM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SPIM0 Module */

#define SRAM0_MODULE    (MODULE_CLKCTL_ENC(48ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SRAM0 Module */

#define SRAM1_MODULE    (MODULE_CLKCTL_ENC(48ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SRAM1 Module */

#define SRAM2_MODULE    (MODULE_CLKCTL_ENC(48ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SRAM2 Module */

#define SRAM3_MODULE    (MODULE_CLKCTL_ENC(48ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< SRAM3 Module */

#define ST0_MODULE      (MODULE_CLKCTL_ENC(51ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_STCTL_ST0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< ST0 Module */

#define TMR0_MODULE     (MODULE_CLKCTL_ENC(53ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(22ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL_TMR0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TMR0 Module */

#define TMR1_MODULE     (MODULE_CLKCTL_ENC(53ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(22ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL_TMR1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TMR1 Module */

#define TMR2_MODULE     (MODULE_CLKCTL_ENC(53ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(22ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL_TMR2SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TMR2 Module */

#define TMR3_MODULE     (MODULE_CLKCTL_ENC(53ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(22ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL_TMR3SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TMR3 Module */

#define TRNG0_MODULE    (MODULE_CLKCTL_ENC(54ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TRNGCTL_TRNG0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TRNG0 Module */

#define TTMR0_MODULE    (MODULE_CLKCTL_ENC(55ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TTMRCTL_TTMR0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(23ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TTMRSEL_TTMR0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TTMR0 Module */

#define TTMR1_MODULE    (MODULE_CLKCTL_ENC(55ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TTMRCTL_TTMR1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(23ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TTMRSEL_TTMR1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< TTMR1 Module */

#define UART0_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART0DIV_Pos))       /*!< UART0 Module */

#define UART1_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART1DIV_Pos))       /*!< UART1 Module */

#define UART2_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART2SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART2DIV_Pos))       /*!< UART2 Module */

#define UART3_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART3SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART3DIV_Pos))       /*!< UART3 Module */

#define UART4_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART4CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART4SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART4DIV_Pos))       /*!< UART4 Module */

#define UART5_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART5CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART5SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART5DIV_Pos))       /*!< UART5 Module */

#define UART6_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART6CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART6SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART6DIV_Pos))       /*!< UART6 Module */

#define UART7_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART7CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(24ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL0_UART7SEL_Pos)|\
                         MODULE_CLKDIV_ENC(15ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV0_UART7DIV_Pos))       /*!< UART7 Module */

#define UART8_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART8CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(25ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL1_UART8SEL_Pos)|\
                         MODULE_CLKDIV_ENC(16ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV1_UART8DIV_Pos))       /*!< UART8 Module */

#define UART9_MODULE    (MODULE_CLKCTL_ENC(56ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART9CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(25ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL1_UART9SEL_Pos)|\
                         MODULE_CLKDIV_ENC(16ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV1_UART9DIV_Pos))       /*!< UART9 Module */

#define USBD0_MODULE    (MODULE_CLKCTL_ENC(57ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USBDCTL_USBD0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(26ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_USBSEL_USBSEL_Pos)|\
                         MODULE_CLKDIV_ENC(17ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_USBDIV_USBDIV_Pos))           /*!< USBD0 Module */

#define USBH0_MODULE    (MODULE_CLKCTL_ENC(58ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USBHCTL_USBH0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(26ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_USBSEL_USBSEL_Pos)|\
                         MODULE_CLKDIV_ENC(17ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_USBDIV_USBDIV_Pos))           /*!< USBH0 Module */

#define USCI0_MODULE    (MODULE_CLKCTL_ENC(59ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< USCI0 Module */

#define UTCPD0_MODULE   (MODULE_CLKCTL_ENC(60ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UTCPDCTL_UTCPD0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< UTCPD0 Module */

#define WDT0_MODULE     (MODULE_CLKCTL_ENC(61ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WDTCTL_WDT0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(27ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WDTSEL_WDT0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< WDT0 Module */

#define WDT1_MODULE     (MODULE_CLKCTL_ENC(61ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WDTCTL_WDT1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(27ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WDTSEL_WDT1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< WDT1 Module */

#define WWDT0_MODULE    (MODULE_CLKCTL_ENC(62ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WWDTCTL_WWDT0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(31ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WWDTSEL_WWDT0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< WWDT0 Module */

#define WWDT1_MODULE    (MODULE_CLKCTL_ENC(62ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WWDTCTL_WWDT1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(31ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WWDTSEL_WWDT1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))                                        /*!< WWDT1 Module */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKO constant definitions.                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKOCTL_DIV1EN_DIV_FREQSEL      0UL
#define CLK_CLKOCTL_DIV1EN_DIV_1            1UL


/** @} end of group CLK_EXPORTED_CONSTANTS */

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief      Set SCLK Divider
  * @param      u32SclkDiv is SCLK clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set SCLK clock divider
  */
#define CLK_SET_SCLKDIV(u32SclkDiv)        (CLK->SCLKDIV = (CLK->SCLKDIV & (~CLK_SCLKDIV_SCLKDIV_Msk)) | CLK_SCLKDIV_SCLKDIV(u32SclkDiv))

/**
  * @brief      Set HCLK2 Divider
  * @param      u32Hclk2Div is HCLK2 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set HCLK2 clock divider
  */
#define CLK_SET_HCLK2DIV(u32Hclk2Div)        (CLK->HCLKDIV = (CLK->HCLKDIV & (~CLK_HCLKDIV_HCLK2DIV_Msk)) | CLK_HCLKDIV_HCLK2DIV(u32Hclk2Div))

/**
  * @brief      Set PCLK0 Divider
  * @param      u32PCLK0Div is PCLK0 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set PCLK0 clock divider
  */
#define CLK_SET_PCLK0DIV(u32PCLK0Div)        (CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_PCLK0DIV_Msk)) | CLK_PCLKDIV_PCLK0DIV(u32PCLK0Div))

/**
  * @brief      Set PCLK1 Divider
  * @param      u32PCLK1Div is PCLK1 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set PCLK1 clock divider
  */
#define CLK_SET_PCLK1DIV(u32PCLK1Div)        (CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_PCLK1DIV_Msk)) | CLK_PCLKDIV_PCLK1DIV(u32PCLK1Div))

/**
  * @brief      Set PCLK2 Divider
  * @param      u32PCLK2Div is PCLK2 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set PCLK2 clock divider
  */
#define CLK_SET_PCLK2DIV(u32PCLK2Div)        (CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_PCLK2DIV_Msk)) | CLK_PCLKDIV_PCLK2DIV(u32PCLK2Div))

/**
  * @brief      Set PCLK3 Divider
  * @param      u32PCLK3Div is PCLK3 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set PCLK3 clock divider
  */
#define CLK_SET_PCLK3DIV(u32PCLK3Div)        (CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_PCLK3DIV_Msk)) | CLK_PCLKDIV_PCLK3DIV(u32PCLK3Div))

/**
  * @brief      Set PCLK4 Divider
  * @param      u32PCLK4Div is PCLK4 clock divider.  It could be 1~16
  * @return     None
  * @details    This macro set PCLK4 clock divider
  */
#define CLK_SET_PCLK4DIV(u32PCLK4Div)        (CLK->PCLKDIV = (CLK->PCLKDIV & (~CLK_PCLKDIV_PCLK4DIV_Msk)) | CLK_PCLKDIV_PCLK4DIV(u32PCLK4Div))

/**
  * @brief      Set System Tick Divider
  * @param      u32ST0Div is system tick clock divider.  It could be 1~256
  * @return     None
  * @details    This macro set system tick clock divider
  */
#define CLK_SET_STDIV(u32ST0Div)        (CLK->STDIV = (CLK->STDIV & (~CLK_STDIV_ST0DIV_Msk)) | CLK_STDIV_ST0DIV(u32ST0Div))

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_DisableMIRC(void);
uint32_t CLK_EnableMIRC(uint32_t u32MircFreq);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetHXTFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetLXTFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetMIRCFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetSCLKFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetACLKFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetHCLK0Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetHCLK1Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetHCLK2Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK0Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK1Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK2Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK3Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK4Freq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetPCLK5Freq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Aclk);
void CLK_SetSCLK(uint32_t u32ClkSrc);
uint32_t CLK_SetBusClock(uint32_t u32SCLKSrc, uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_SetModuleClock(uint64_t u64ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint64_t u64ModuleIdx);
void CLK_DisableModuleClock(uint64_t u64ModuleIdx);
uint32_t CLK_EnableAPLL(uint32_t u32APLLClkSrc, uint32_t u32APLLFreq, uint32_t u32APLLSelect);
void CLK_DisableAPLL(uint32_t u32APLLSelect);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
uint32_t CLK_WaitClockDisable(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetAPLL0ClockFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetAPLL1ClockFreq(void);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetModuleClockSource(uint64_t u64ModuleIdx);
__NONSECURE_ENTRY_WEAK uint32_t CLK_GetModuleClockDivider(uint64_t u64ModuleIdx);
__NONSECURE_ENTRY_WEAK uint32_t CLK_SystemClockUpdate(void);
void CLK_SysTickDelay(uint32_t us);
void CLK_SysTickLongDelay(uint32_t us);

/** @} end of group CLK_EXPORTED_FUNCTIONS */

/** @} end of group CLK_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __CLK_H__ */
