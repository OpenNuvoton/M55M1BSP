/**************************************************************************//**
 * @file     startup_M55M1.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M55M1
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"


/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);


/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Reset_Handler_Main(void);
void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SecureFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

void ACMP01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ACMP23_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void AWF_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BODOUT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BPWM0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BPWM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BRAKE0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BRAKE1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD00_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CCAP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CKFAIL_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRYPTO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DAC01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMIC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMIC0VAD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC00_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC02_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC03_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECAP0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECAP1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECAP2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECAP3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EMAC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM0P0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM0P1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM0P2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM1P0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM1P1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EPWM1P2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ETI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

void GDMACH0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GDMACH1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPF_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPJ_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSOTG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSUSBD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void HSUSBH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2S1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I3C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void IRC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ISP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void KPI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void KS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPADC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPI2C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPPDMA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPSPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTMR0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTMR1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPUART0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

void NPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void OTFC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PDMA0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PDMA1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PSIO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void QSPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void QSPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTCTAMPER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

void SC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SC1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SC2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SCU_IRQHandler(void) __attribute__((weak));
void SDH0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SDH1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPIM0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SRAMPERR_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TRNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TTMR0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TTMR1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBH0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBH1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBOTG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USCI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UTCPD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WDT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WDT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WWDT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WWDT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

/* Static vector table
 * For performance, M55M1 places vector table in DTCM by default.
 * User can define NVT_VECTOR_ON_FLASH to place vector table in Flash.
 *
 * If NVT_VECTOR_ON_FLASH is defined and use IAR,
 *   IRQ handlers referenced in __VECTOR_TABLE (__vector_table) are protected and
 *   not affected by 'initialize by copy'. It means IRQ handler must placed in Flash.
 */
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
#ifndef NVT_VECTOR_ON_FLASH
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*       Initial Stack Pointer                            */
    Reset_Handler,                            /*       Reset Handler                                    */
    NMI_Handler,                              /*   -14 NMI Handler                                      */
    HardFault_Handler,                        /*   -13 Hard Fault Handler                               */
    MemManage_Handler,                        /*   -12 MPU Fault Handler                                */
    BusFault_Handler,                         /*   -11 Bus Fault Handler                                */
    UsageFault_Handler,                       /*   -10 Usage Fault Handler                              */
    SecureFault_Handler,                      /*    -9 Secure Fault Handler                             */
};

/* Declare new vector table placed in DTCM */
const VECTOR_TABLE_Type DTCM_VECTOR_TABLE[] NVT_DTCM_VTOR =
#endif
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*       Initial Stack Pointer                            */
    Reset_Handler,                            /*       Reset Handler                                    */
    NMI_Handler,                              /*   -14 NMI Handler                                      */
    HardFault_Handler,                        /*   -13 Hard Fault Handler                               */
    MemManage_Handler,                        /*   -12 MPU Fault Handler                                */
    BusFault_Handler,                         /*   -11 Bus Fault Handler                                */
    UsageFault_Handler,                       /*   -10 Usage Fault Handler                              */
    SecureFault_Handler,                      /*    -9 Secure Fault Handler                             */
    0,                                        /*       Reserved                                         */
    0,                                        /*       Reserved                                         */
    0,                                        /*       Reserved                                         */
    SVC_Handler,                              /*    -5 SVC Handler                                      */
    DebugMon_Handler,                         /*    -4 Debug Monitor Handler                            */
    0,                                        /*       Reserved                                         */
    PendSV_Handler,                           /*    -2 PendSV Handler Handler                           */
    SysTick_Handler,                          /*    -1 SysTick Handler                                  */

    /* Interrupts */
    BODOUT_IRQHandler,                        /*!<   0 Brown-Out Low Voltage Detected Interrupt Handler */
    IRC_IRQHandler,                           /*!<   1 IRC TRIM Interrupt Handler                       */
    PMC_IRQHandler,                           /*!<   2 Power Mode Controller Interrupt Handler          */
    SRAMPERR_IRQHandler,                      /*!<   3 SRAM Parity Check Error Interrupt Handler        */
    CKFAIL_IRQHandler,                        /*!<   4 Clock Fail Detected Interrupt Handler            */
    ISP_IRQHandler,                           /*!<   5 FMC ISP Interrupt Handler                        */
    RTC_IRQHandler,                           /*!<   6 Real Time Clock Interrupt Handler                */
    RTCTAMPER_IRQHandler,                     /*!<   7 Backup Register Tamper Interrupt Handler         */
    WDT0_IRQHandler,                          /*!<   8 Watchdog Timer 0 Interrupt Handler               */
    WWDT0_IRQHandler,                         /*!<   9 Window Watchdog Timer 0 Interrupt Handler        */

    0,                                        /*       Reserved                                         */
    WDT1_IRQHandler,                          /*!<  11 Watchdog Timer 1 Interrupt Handler               */
    WWDT1_IRQHandler,                         /*!<  12 Window Watchdog Timer 1 Interrupt Handler        */
    NPU_IRQHandler,                           /*!<  13 Neural Network Processor Interrupt Handler       */
    EINT0_IRQHandler,                         /*!<  14 External Input 0 Interrupt Handler               */
    EINT1_IRQHandler,                         /*!<  15 External Input 1 Interrupt Handler               */
    EINT2_IRQHandler,                         /*!<  16 External Input 2 Interrupt Handler               */
    EINT3_IRQHandler,                         /*!<  17 External Input 3 Interrupt Handler               */
    EINT4_IRQHandler,                         /*!<  18 External Input 4 Interrupt Handler               */
    EINT5_IRQHandler,                         /*!<  19 External Input 5 Interrupt Handler               */

    GPA_IRQHandler,                           /*!<  20 GPIO Port A Interrupt Handler                    */
    GPB_IRQHandler,                           /*!<  21 GPIO Port B Interrupt Handler                    */
    GPC_IRQHandler,                           /*!<  22 GPIO Port C Interrupt Handler                    */
    GPD_IRQHandler,                           /*!<  23 GPIO Port D Interrupt Handler                    */
    GPE_IRQHandler,                           /*!<  24 GPIO Port E Interrupt Handler                    */
    GPF_IRQHandler,                           /*!<  25 GPIO Port F Interrupt Handler                    */
    GPG_IRQHandler,                           /*!<  26 GPIO Port G Interrupt Handler                    */
    GPH_IRQHandler,                           /*!<  27 GPIO Port H Interrupt Handler                    */
    GPI_IRQHandler,                           /*!<  28 GPIO Port I Interrupt Handler                    */
    GPJ_IRQHandler,                           /*!<  29 GPIO Port J Interrupt Handler                    */

    BRAKE0_IRQHandler,                        /*!<  30 EPWM0 Brake Interrupt Handler                    */
    EPWM0P0_IRQHandler,                       /*!<  31 EPWM0 Pair 0 Interrupt Handler                   */
    EPWM0P1_IRQHandler,                       /*!<  32 EPWM0 Pair 1 Interrupt Handler                   */
    EPWM0P2_IRQHandler,                       /*!<  33 EPWM0 Pair 2 Interrupt Handler                   */
    BRAKE1_IRQHandler,                        /*!<  34 EPWM1 Brake Interrupt Handler                    */
    EPWM1P0_IRQHandler,                       /*!<  35 EPWM1 Pair 0 Interrupt Handler                   */
    EPWM1P1_IRQHandler,                       /*!<  36 EPWM1 Pair 1 Interrupt Handler                   */
    EPWM1P2_IRQHandler,                       /*!<  37 EPWM1 Pair 2 Interrupt Handler                   */
    BPWM0_IRQHandler,                         /*!<  38 BPWM0 Interrupt Handler                          */
    BPWM1_IRQHandler,                         /*!<  39 BPWM1 Interrupt Handler                          */

    0,                                        /*!<     Reserved                                         */
    PDMA0_IRQHandler,                         /*!<  41 PDMA0 Interrupt Handler                          */
    PDMA1_IRQHandler,                         /*!<  42 PDMA1 Interrupt Handler                          */
    LPPDMA_IRQHandler,                        /*!<  43 Low Power PDMA Interrupt Handler                 */
    SCU_IRQHandler,                           /*!<  44 SCU Interrupt Handler                            */
    0,                                        /*!<     Reserved                                         */
    KS_IRQHandler,                            /*!<  46 Key Store Interrupt Handler                      */
    TIMER0_IRQHandler,                        /*!<  47 Timer0 Interrupt Handler                         */
    TIMER1_IRQHandler,                        /*!<  48 Timer1 Interrupt Handler                         */
    TIMER2_IRQHandler,                        /*!<  49 Timer2 Interrupt Handler                         */

    TIMER3_IRQHandler,                        /*!<  50 Timer3 Interrupt Handler                         */
    LPTMR0_IRQHandler,                        /*!<  51 Low Power Timer 0 Interrupt Handler              */
    LPTMR1_IRQHandler,                        /*!<  52 Low Power Timer 1 Interrupt Handler              */
    0,                                        /*!<     Reserved                                         */
    TTMR0_IRQHandler,                         /*!<  54 Tick Timer 0 Interrupt Handler                   */
    TTMR1_IRQHandler,                         /*!<  55 Tick Timer 1 Interrupt Handler                   */
    USBH0_IRQHandler,                         /*!<  56 USB Host 0 Interrupt Handler                     */
    USBH1_IRQHandler,                         /*!<  57 USB Host 1 Interrupt Handler                     */
    USBD_IRQHandler,                          /*!<  58 USB Device Interrupt Handler                     */
    USBOTG_IRQHandler,                        /*!<  59 USB OTG Interrupt Handler                        */

    HSUSBH_IRQHandler,                        /*!<  60 High Speed USB Host Interrupt Handler            */
    HSUSBD_IRQHandler,                        /*!<  61 High Speed USB Device Interrupt Handler          */
    HSOTG_IRQHandler,                         /*!<  62 High Speed OTG Interrupt Handler                 */
    EMAC0_IRQHandler,                         /*!<  63 EMAC0 Interrupt Handler                          */
    QSPI0_IRQHandler,                         /*!<  64 QSPI0 Interrupt Handler                          */
    QSPI1_IRQHandler,                         /*!<  65 QSPI1 Interrupt Handler                          */
    SPI0_IRQHandler,                          /*!<  66 SPI0 Interrupt Handler                           */
    SPI1_IRQHandler,                          /*!<  67 SPI1 Interrupt Handler                           */
    SPI2_IRQHandler,                          /*!<  68 SPI2 Interrupt Handler                           */
    SPI3_IRQHandler,                          /*!<  69 SPI3 Interrupt Handler                           */

    0,                                        /*!<     Reserved                                         */
    LPSPI0_IRQHandler,                        /*!<  71 Low Power SPI 0 Interrupt Handler                */
    0,                                        /*!<     Reserved                                         */
    SPIM0_IRQHandler,                         /*!<  73 SPIM0 Interrupt Handler                          */
    0,                                        /*!<     Reserved                                         */
    UART0_IRQHandler,                         /*!<  75 UART0 Interrupt Handler                          */
    UART1_IRQHandler,                         /*!<  76 UART1 Interrupt Handler                          */
    UART2_IRQHandler,                         /*!<  77 UART2 Interrupt Handler                          */
    UART3_IRQHandler,                         /*!<  78 UART3 Interrupt Handler                          */
    UART4_IRQHandler,                         /*!<  79 UART4 Interrupt Handler                          */

    UART5_IRQHandler,                         /*!<  80 UART5 Interrupt Handler                          */
    UART6_IRQHandler,                         /*!<  81 UART6 Interrupt Handler                          */
    UART7_IRQHandler,                         /*!<  82 UART7 Interrupt Handler                          */
    UART8_IRQHandler,                         /*!<  83 UART8 Interrupt Handler                          */
    UART9_IRQHandler,                         /*!<  84 UART9 Interrupt Handler                          */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    EINT6_IRQHandler,                         /*!<  88 External Input 6 Interrupt Handler               */
    EINT7_IRQHandler,                         /*!<  89 External Input 7 Interrupt Handler               */

    LPUART0_IRQHandler,                       /*!<  90 Low Power UART 0 Interrupt Handler               */
    0,                                        /*!<     Reserved                                         */
    I2C0_IRQHandler,                          /*!<  92 I2C0 Interrupt Handler                           */
    I2C1_IRQHandler,                          /*!<  93 I2C1 Interrupt Handler                           */
    I2C2_IRQHandler,                          /*!<  94 I2C2 Interrupt Handler                           */
    I2C3_IRQHandler,                          /*!<  95 I2C3 Interrupt Handler                           */
    LPI2C0_IRQHandler,                        /*!<  96 Low Power I2C 0 Interrupt Handler                */
    USCI0_IRQHandler,                         /*!<  97 USCI0 Interrupt Handler                          */
    0,                                        /*!<     Reserved                                         */
    SC0_IRQHandler,                           /*!<  99 Smart Card Host 0 Interrupt Handler              */

    SC1_IRQHandler,                           /*!< 100 Smart Card Host 1 Interrupt Handler              */
    SC2_IRQHandler,                           /*!< 101 Smart Card Host 2 Interrupt Handler              */
    PSIO_IRQHandler,                          /*!< 102 PSIO Interrupt Handler                           */
    DMIC0_IRQHandler,                         /*!< 103 DMIC0 Interrupt Handler                          */
    DMIC0VAD_IRQHandler,                      /*!< 104 DMIC0 VAD Interrupt Handler                      */
    I2S0_IRQHandler,                          /*!< 105 I2S0 Interrupt Handler                           */
    I2S1_IRQHandler,                          /*!< 106 I2S1 Interrupt Handler                           */
    TRNG_IRQHandler,                          /*!< 107 TRNG Interrupt Handler                           */
    I3C0_IRQHandler,                          /*!< 108 I3C0 Interrupt Handler                           */
    0,                                        /*!<     Reserved                                         */

    OTFC0_IRQHandler,                         /*!< 110 OTFC0 Interrupt Handler                          */
    0,                                        /*!<     Reserved                                         */
    KPI_IRQHandler,                           /*!< 112 KPI Interrupt Handler                            */
    SDH0_IRQHandler,                          /*!< 113 SD Host 0 Interrupt Handler                      */
    SDH1_IRQHandler,                          /*!< 114 SD Host 1 Interrupt Handler                      */
    CCAP_IRQHandler,                          /*!< 115 CCAP Interrupt Handler                           */
    CRYPTO_IRQHandler,                        /*!< 116 CRYPTO Interrupt Handler                         */
    CANFD00_IRQHandler,                       /*!< 117 CANFD0 Interrupt 0 Handler                       */
    CANFD01_IRQHandler,                       /*!< 118 CANFD0 Interrupt 1 Handler                       */
    CANFD10_IRQHandler,                       /*!< 119 CANFD1 Interrupt 0 Handler                       */

    CANFD11_IRQHandler,                       /*!< 120 CANFD1 Interrupt 1 Handler                       */
    ACMP01_IRQHandler,                        /*!< 121 ACMP0 and ACMP1 Interrupt Handler                */
    ACMP23_IRQHandler,                        /*!< 122 ACMP2 and ACMP3 Interrupt Handler                */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    CRC_IRQHandler,                           /*!< 125 CRC Interrupt Handler                            */
    EADC00_IRQHandler,                        /*!< 126 EADC0 Interrupt 0 Handler                        */
    EADC01_IRQHandler,                        /*!< 127 EADC0 Interrupt 1 Handler                        */
    EADC02_IRQHandler,                        /*!< 128 EADC0 Interrupt 2 Handler                        */
    EADC03_IRQHandler,                        /*!< 129 EADC0 Interrupt 3 Handler                        */

    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    LPADC0_IRQHandler,                        /*!< 134 Low Power ADC 0 Interrupt Handler                */
    // DAC1 is not support in TESTCHIP_ONLY
    DAC01_IRQHandler,                         /*!< 135 DAC0 and DAC1 Interrupt Handler                  */
    0,                                        /*!<     Reserved                                         */
    EQEI0_IRQHandler,                         /*!< 137 EQEI0 Interrupt Handler                          */
    EQEI1_IRQHandler,                         /*!< 138 EQEI1 Interrupt Handler                          */
    EQEI2_IRQHandler,                         /*!< 139 EQEI2 Interrupt Handler                          */

    EQEI3_IRQHandler,                         /*!< 140 EQEI3 Interrupt Handler                          */
    ECAP0_IRQHandler,                         /*!< 141 ECAP0 Interrupt Handler                          */
    ECAP1_IRQHandler,                         /*!< 142 ECAP1 Interrupt Handler                          */
    ECAP2_IRQHandler,                         /*!< 143 ECAP2 Interrupt Handler                          */
    ECAP3_IRQHandler,                         /*!< 144 ECAP3 Interrupt Handler                          */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    AWF_IRQHandler,                           /*!< 149 AWF Interrupt Handler                            */

    UTCPD_IRQHandler,                         /*!< 150 UTCPD Interrupt Handler                          */
    ETI_IRQHandler,                           /*!< 151 ETI Interrupt Handler                            */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */
    0,                                        /*!<     Reserved                                         */

    GDMACH0_IRQHandler,                       /*!< 160 GDMA Channel 0 Interrupt Handler                 */
    GDMACH1_IRQHandler,                       /*!< 161 GDMA Channel 1 Interrupt Handler                 */
};

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

/* If some peripherals (e.g. Hyper RAM) must be initialized before startup routine
 *   user can implement its own Reset_Handler_PreInit to do early initialization.
 */
__WEAK void Reset_Handler_PreInit(void)
{
    /* Prevent using C runtime library functions (e.g. printf)
     * or global variables in this function.
     */
}

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    // Copy __set_PSP/__set_MSPLIM/__set_PSPLIM from cmsis_armclang.h
    __ASM volatile("MSR psp, %0" : : "r"((uint32_t)(&__INITIAL_SP)) :);
    __ASM volatile("MSR msplim, %0" : : "r"((uint32_t)(&__STACK_LIMIT)));
    __ASM volatile("MSR psplim, %0" : : "r"((uint32_t)(&__STACK_LIMIT)));

    // Move other code to Reset_Handler_Main to prevent compiler generate stack access code.
    // Move stack pointer before setting MSPLIM might cause hard fault due to MSPLIM violation.
    Reset_Handler_Main();
}

__NO_RETURN void Reset_Handler_Main(void)
{
    if ((__PC() & NS_OFFSET) == 0)
    {
        // Unlock protected registers
        do
        {
            SYS->REGLCTL = 0x59UL;
            SYS->REGLCTL = 0x16UL;
            SYS->REGLCTL = 0x88UL;
        } while (SYS->REGLCTL == 0UL);

#ifdef TESTCHIP_ONLY

        if (!(SYS->RSTSTS & SYS_RSTSTS_PORF_Msk))
        {
            SYS_ResetChip();
        }

#endif

        // Switch SRAM0 to normal power mode
        if (PMC->SYSRB0PC != 0)
        {
            PMC->SYSRB0PC = 0;

            while (PMC->SYSRB0PC & PMC_SYSRB0PC_PCBUSY_Msk)
                ;
        }

        // Enable SRAM0 clock
        CLK->SRAMCTL |= CLK_SRAMCTL_SRAM0CKEN_Msk;

        // Switch SRAM1 to normal power mode
        if (PMC->SYSRB1PC != 0)
        {
            PMC->SYSRB1PC = 0;

            while (PMC->SYSRB1PC & PMC_SYSRB1PC_PCBUSY_Msk)
                ;
        }

        // Enable SRAM1 clock
        CLK->SRAMCTL |= CLK_SRAMCTL_SRAM1CKEN_Msk;
    }

#if (defined (__FPU_USED) && (__FPU_USED == 1U)) || \
    (defined (__ARM_FEATURE_MVE) && (__ARM_FEATURE_MVE > 0U))
    SCB->CPACR |= ((3U << 10U * 2U) |         /* Enable CP10 Full Access */
                   (3U << 11U * 2U));         /* Enable CP11 Full Access */

    /* Set low-power state for PDEPU                */
    /*  0b00  | ON, PDEPU is not in low-power state */
    /*  0b01  | ON, but the clock is off            */
    /*  0b10  | RET(ention)                         */
    /*  0b11  | OFF                                 */

    /* Clear ELPSTATE, value is 0b11 on Cold reset */
    PWRMODCTL->CPDLPSTATE &= ~(PWRMODCTL_CPDLPSTATE_ELPSTATE_Msk << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos);

    /* Favor best FP/MVE performance by default, avoid EPU switch-ON delays */
    /* PDEPU ON, Clock OFF */
    PWRMODCTL->CPDLPSTATE |= 0x1 << PWRMODCTL_CPDLPSTATE_ELPSTATE_Pos;
#endif

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    __TZ_set_STACKSEAL_S((uint32_t *)(&__STACK_SEAL));
#endif

    SCB_InvalidateICache();
    SCB_EnableICache();

#ifdef NVT_DCACHE_ON
    SCB_InvalidateDCache();
    SCB_EnableDCache();
#endif  // NVT_DCACHE_ON

    Reset_Handler_PreInit();
    __PROGRAM_START();      // Enter PreMain (C library entry point)
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif


/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
extern void ProcessHardFault(uint32_t *pu32StackFrame);

struct StackFrame
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xPsr;
};

void HardFault_Handler(void)
{
    uint32_t u32IRQ;
    struct StackFrame *psStackFrame = NULL;

    (void)u32IRQ;
    __ASM volatile("mrs %0, ipsr            \n" // Read IPSR (Exception number)
                   "sub %0, #16             \n" // Get it into IRQn_Type range
                   "tst lr, #4              \n" // Select the stack which was in use
                   "ite eq                  \n"
                   "mrseq %1, msp           \n"
                   "mrsne %1, psp           \n"
                   : "=r"(u32IRQ), "=r"(psStackFrame));

    // Get the instruction caused the hardfault
    ProcessHardFault((uint32_t *)psStackFrame);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
