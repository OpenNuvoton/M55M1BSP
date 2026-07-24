/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M55M1
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__

/* -------------------  Interrupt Number Definition  ------------------------------ */

typedef enum IRQn
{
    /* -------------------  Processor Exceptions Numbers  ----------------------------- */
    NonMaskableInt_IRQn           = -14,      /*  2 Non Maskable Interrupt                  */
    HardFault_IRQn                = -13,      /*  3 HardFault Interrupt                     */
    MemoryManagement_IRQn         = -12,      /*  4 Memory Management Interrupt             */
    BusFault_IRQn                 = -11,      /*  5 Bus Fault Interrupt                     */
    UsageFault_IRQn               = -10,      /*  6 Usage Fault Interrupt                   */
    SecureFault_IRQn              =  -9,      /*  7 Secure Fault Interrupt                  */
    SVCall_IRQn                   =  -5,      /* 11 SVC Interrupt                           */
    DebugMonitor_IRQn             =  -4,      /* 12 Debug Monitor Interrupt                 */
    PendSV_IRQn                   =  -2,      /* 14 Pend SV Interrupt                       */
    SysTick_IRQn                  =  -1,      /* 15 System Tick Interrupt                   */

    /* -------------------  Processor Interrupt Numbers  ------------------------------ */
    BODOUT_IRQn                   =   0,      /*!< Brown-Out Low Voltage Detected Interrupt */
    IRC_IRQn                      =   1,      /*!< IRC TRIM Interrupt                       */
    PMC_IRQn                      =   2,      /*!< Power Mode Controller Interrupt          */
    SRAMPERR_IRQn                 =   3,      /*!< SRAM Parity Check Error Interrupt        */
    CKFAIL_IRQn                   =   4,      /*!< Clock Fail Detected Interrupt            */
    ISP_IRQn                      =   5,      /*!< FMC ISP Interrupt                        */
    RTC_IRQn                      =   6,      /*!< Real Time Clock Interrupt                */
    RTCTAMPER_IRQn                =   7,      /*!< Backup Register Tamper Interrupt         */
    WDT0_IRQn                     =   8,      /*!< Watchdog Timer 0 Interrupt               */
    WWDT0_IRQn                    =   9,      /*!< Window Watchdog Timer 0 Interrupt        */

    /*!< Reserved                                                                           */
    WDT1_IRQn                     =  11,      /*!< Watchdog Timer 1 Interrupt               */
    WWDT1_IRQn                    =  12,      /*!< Window Watchdog Timer 1 Interrupt        */
    NPU_IRQn                      =  13,      /*!< Neural Network Processor Interrupt       */
    EINT0_IRQn                    =  14,      /*!< External Input 0 Interrupt               */
    EINT1_IRQn                    =  15,      /*!< External Input 1 Interrupt               */
    EINT2_IRQn                    =  16,      /*!< External Input 2 Interrupt               */
    EINT3_IRQn                    =  17,      /*!< External Input 3 Interrupt               */
    EINT4_IRQn                    =  18,      /*!< External Input 4 Interrupt               */
    EINT5_IRQn                    =  19,      /*!< External Input 5 Interrupt               */

    GPA_IRQn                      =  20,      /*!< GPIO Port A Interrupt                    */
    GPB_IRQn                      =  21,      /*!< GPIO Port B Interrupt                    */
    GPC_IRQn                      =  22,      /*!< GPIO Port C Interrupt                    */
    GPD_IRQn                      =  23,      /*!< GPIO Port D Interrupt                    */
    GPE_IRQn                      =  24,      /*!< GPIO Port E Interrupt                    */
    GPF_IRQn                      =  25,      /*!< GPIO Port F Interrupt                    */
    GPG_IRQn                      =  26,      /*!< GPIO Port G Interrupt                    */
    GPH_IRQn                      =  27,      /*!< GPIO Port H Interrupt                    */
    GPI_IRQn                      =  28,      /*!< GPIO Port I Interrupt                    */
    GPJ_IRQn                      =  29,      /*!< GPIO Port J Interrupt                    */

    BRAKE0_IRQn                   =  30,      /*!< EPWM0 Brake Interrupt                    */
    EPWM0P0_IRQn                  =  31,      /*!< EPWM0 Pair 0 Interrupt                   */
    EPWM0P1_IRQn                  =  32,      /*!< EPWM0 Pair 1 Interrupt                   */
    EPWM0P2_IRQn                  =  33,      /*!< EPWM0 Pair 2 Interrupt                   */
    BRAKE1_IRQn                   =  34,      /*!< EPWM1 Brake Interrupt                    */
    EPWM1P0_IRQn                  =  35,      /*!< EPWM1 Pair 0 Interrupt                   */
    EPWM1P1_IRQn                  =  36,      /*!< EPWM1 Pair 1 Interrupt                   */
    EPWM1P2_IRQn                  =  37,      /*!< EPWM1 Pair 2 Interrupt                   */
    BPWM0_IRQn                    =  38,      /*!< BPWM0 Interrupt                          */
    BPWM1_IRQn                    =  39,      /*!< BPWM1 Interrupt                          */

    /*!< Reserved                                                                           */
    PDMA0_IRQn                    =  41,      /*!< PDMA0 Interrupt                          */
    PDMA1_IRQn                    =  42,      /*!< PDMA1 Interrupt                          */
    LPPDMA_IRQn                   =  43,      /*!< Low Power PDMA Interrupt                 */
    SCU_IRQn                      =  44,      /*!< SCU Interrupt                            */
    /*!< Reserved                                                                           */
    KS_IRQn                       =  46,      /*!< Key Store Interrupt                      */
    TIMER0_IRQn                   =  47,      /*!< Timer0 Interrupt                         */
    TIMER1_IRQn                   =  48,      /*!< Timer1 Interrupt                         */
    TIMER2_IRQn                   =  49,      /*!< Timer2 Interrupt                         */

    TIMER3_IRQn                   =  50,      /*!< Timer3 Interrupt                         */
    LPTMR0_IRQn                   =  51,      /*!< Low Power Timer 0 Interrupt              */
    LPTMR1_IRQn                   =  52,      /*!< Low Power Timer 1 Interrupt              */
    /*!< Reserved                                                                           */
    TTMR0_IRQn                    =  54,      /*!< Tick Timer 0 Interrupt                   */
    TTMR1_IRQn                    =  55,      /*!< Tick Timer 1 Interrupt                   */
    USBH0_IRQn                    =  56,      /*!< USB Host 0 Interrupt                     */
    USBH1_IRQn                    =  57,      /*!< USB Host 1 Interrupt                     */
    USBD_IRQn                     =  58,      /*!< USB Device Interrupt                     */
    USBOTG_IRQn                   =  59,      /*!< USB OTG Interrupt                        */

    HSUSBH_IRQn                   =  60,      /*!< High Speed USB Host Interrupt            */
    HSUSBD_IRQn                   =  61,      /*!< High Speed USB Device Interrupt          */
    HSOTG_IRQn                    =  62,      /*!< High Speed OTG Interrupt                 */
    EMAC0_IRQn                    =  63,      /*!< EMAC0 Interrupt                          */
    QSPI0_IRQn                    =  64,      /*!< QSPI0 Interrupt                          */
    QSPI1_IRQn                    =  65,      /*!< QSPI1 Interrupt                          */
    SPI0_IRQn                     =  66,      /*!< SPI0 Interrupt                           */
    SPI1_IRQn                     =  67,      /*!< SPI1 Interrupt                           */
    SPI2_IRQn                     =  68,      /*!< SPI2 Interrupt                           */
    SPI3_IRQn                     =  69,      /*!< SPI3 Interrupt                           */

    /*!< Reserved                                                                           */
    LPSPI0_IRQn                   =  71,      /*!< Low Power SPI 0 Interrupt                */
    /*!< Reserved                                                                           */
    SPIM0_IRQn                    =  73,      /*!< SPIM0 Interrupt                          */
    /*!< Reserved                                                                           */
    UART0_IRQn                    =  75,      /*!< UART0 Interrupt                          */
    UART1_IRQn                    =  76,      /*!< UART1 Interrupt                          */
    UART2_IRQn                    =  77,      /*!< UART2 Interrupt                          */
    UART3_IRQn                    =  78,      /*!< UART3 Interrupt                          */
    UART4_IRQn                    =  79,      /*!< UART4 Interrupt                          */

    UART5_IRQn                    =  80,      /*!< UART5 Interrupt                          */
    UART6_IRQn                    =  81,      /*!< UART6 Interrupt                          */
    UART7_IRQn                    =  82,      /*!< UART7 Interrupt                          */
    UART8_IRQn                    =  83,      /*!< UART8 Interrupt                          */
    UART9_IRQn                    =  84,      /*!< UART9 Interrupt                          */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    EINT6_IRQn                    =  88,      /*!< External Input 6 Interrupt               */
    EINT7_IRQn                    =  89,      /*!< External Input 7 Interrupt               */

    LPUART0_IRQn                  =  90,      /*!< Low Power UART 0 Interrupt               */
    /*!< Reserved                                                                           */
    I2C0_IRQn                     =  92,      /*!< I2C0 Interrupt                           */
    I2C1_IRQn                     =  93,      /*!< I2C1 Interrupt                           */
    I2C2_IRQn                     =  94,      /*!< I2C2 Interrupt                           */
    I2C3_IRQn                     =  95,      /*!< I2C3 Interrupt                           */
    LPI2C0_IRQn                   =  96,      /*!< Low Power I2C 0 Interrupt                */
    USCI0_IRQn                    =  97,      /*!< USCI0 Interrupt                          */
    /*!< Reserved                                                                           */
    SC0_IRQn                      =  99,      /*!< Smart Card Host 0 Interrupt              */

    SC1_IRQn                      = 100,      /*!< Smart Card Host 1 Interrupt              */
    SC2_IRQn                      = 101,      /*!< Smart Card Host 2 Interrupt              */
    PSIO_IRQn                     = 102,      /*!< PSIO Interrupt                           */
    DMIC0_IRQn                    = 103,      /*!< DMIC0 Interrupt                          */
    DMIC0VAD_IRQn                 = 104,      /*!< DMIC0 VAD Interrupt                      */
    I2S0_IRQn                     = 105,      /*!< I2S0 Interrupt                           */
    I2S1_IRQn                     = 106,      /*!< I2S1 Interrupt                           */
    TRNG_IRQn                     = 107,      /*!< TRNG Interrupt                           */
    I3C0_IRQn                     = 108,      /*!< I3C0 Interrupt                           */
    /*!< Reserved                                                                           */

    OTFC0_IRQn                    = 110,      /*!< OTFC0 Interrupt                          */
    /*!< Reserved                                                                           */
    KPI_IRQn                      = 112,      /*!< KPI Interrupt                            */
    SDH0_IRQn                     = 113,      /*!< SD Host 0 Interrupt                      */
    SDH1_IRQn                     = 114,      /*!< SD Host 1 Interrupt                      */
    CCAP_IRQn                     = 115,      /*!< CCAP Interrupt                           */
    CRYPTO_IRQn                   = 116,      /*!< CRYPTO Interrupt                         */
    CANFD00_IRQn                  = 117,      /*!< CANFD00 Interrupt                        */
    CANFD01_IRQn                  = 118,      /*!< CANFD01 Interrupt                        */
    CANFD10_IRQn                  = 119,      /*!< CANFD10 Interrupt                        */

    CANFD11_IRQn                  = 120,      /*!< CANFD11 Interrupt                        */
    ACMP01_IRQn                   = 121,      /*!< ACMP0 and ACMP1 Interrupt                */
    ACMP23_IRQn                   = 122,      /*!< ACMP2 and ACMP3 Interrupt                */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    CRC_IRQn                      = 125,      /*!< CRC Interrupt                            */
    EADC00_IRQn                   = 126,      /*!< EADC0 Interrupt 0                        */
    EADC01_IRQn                   = 127,      /*!< EADC0 Interrupt 1                        */
    EADC02_IRQn                   = 128,      /*!< EADC0 Interrupt 2                        */
    EADC03_IRQn                   = 129,      /*!< EADC0 Interrupt 3                        */

    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    LPADC0_IRQn                   = 134,      /*!< Low Power ADC 0 Interrupt                */
    DAC01_IRQn                    = 135,      /*!< DAC0 and DAC1 Interrupt                  */
    /*!< Reserved                                 */
    EQEI0_IRQn                    = 137,      /*!< EQEI0 Interrupt                          */
    EQEI1_IRQn                    = 138,      /*!< EQEI1 Interrupt                          */
    EQEI2_IRQn                    = 139,      /*!< EQEI2 Interrupt                          */

    EQEI3_IRQn                    = 140,      /*!< EQEI3 Interrupt                          */
    ECAP0_IRQn                    = 141,      /*!< ECAP0 Interrupt                          */
    ECAP1_IRQn                    = 142,      /*!< ECAP1 Interrupt                          */
    ECAP2_IRQn                    = 143,      /*!< ECAP2 Interrupt                          */
    ECAP3_IRQn                    = 144,      /*!< ECAP3 Interrupt                          */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    AWF_IRQn                      = 149,      /*!< AWF Interrupt                            */

    UTCPD_IRQn                    = 150,      /*!< UTCPD Interrupt                          */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */
    /*!< Reserved                                                                           */

    GDMACH0_IRQn                  = 160,      /*!< GDMA Channel 0 Interrupt                 */
    GDMACH1_IRQn                  = 161,      /*!< GDMA Channel 1 Interrupt                 */

    /** @cond DOXYGEN_IGNORE */
    IRQ_OFFSET                    = 16,       /* To omit MISRA_C Rule 10.4                  */
    TOTAL_IRQn_CNT                = GDMACH1_IRQn + IRQ_OFFSET,
    /** @endcond */
} IRQn_Type;

#endif /* __IRQN_H__ */