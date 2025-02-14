/**************************************************************************//**
 * @file     M55M1.h
 * @version  V1.00
 * @brief    Peripheral Access Layer Header File for NuMicro M55M1
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage NuMicro M55M1 Series CMSIS BSP Driver Reference
  *
  * <b>Introduction</b>
  *
  * This user manual describes the usage of M55M1/M55M1 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Important Notice</b>
  *
  * Nuvoton Products are neither intended nor warranted for usage in systems or equipment,
  * any malfunction or failure of which may cause loss of human life, bodily injury or severe
  * property damage. Such applications are deemed, "Insecure Usage".
  *
  * Insecure usage includes, but is not limited to: equipment for surgical implementation,
  * atomic energy control instruments, airplane or spaceship instruments, the control or
  * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
  * instruments, all types of safety devices, and other applications intended to support or
  * sustain life.
  *
  * All Insecure Usage shall be made at customer's risk, and in the event that third parties
  * lay claims to Nuvoton as a result of customer's Insecure Usage, customer shall indemnify
  * the damages and liabilities thus incurred by Nuvoton.
  *
  * Please note that all data and specifications are subject to change without notice. All the
  * trademarks of products and companies mentioned in this datasheet belong to their respective
  * owners.
  *
  * <b>Copyright Notice</b>
  *
  * SPDX-License-Identifier: Apache-2.0<br/>
  * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
  */


#ifndef __M55M1_H__
#define __M55M1_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                Processor and Core peripheral                              */
/******************************************************************************/
/** @addtogroup CMSIS_Device Device CMSIS Definitions
    Configuration of the Cortex-M4 Processor and Core peripheral
    @{
*/

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

    /*!< Reserved                                 */
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

    /*!< Reserved                                 */
    PDMA0_IRQn                    =  41,      /*!< PDMA0 Interrupt                          */
    PDMA1_IRQn                    =  42,      /*!< PDMA1 Interrupt                          */
    LPPDMA_IRQn                   =  43,      /*!< Low Power PDMA Interrupt                 */
    SCU_IRQn                      =  44,      /*!< SCU Interrupt                            */
    /*!< Reserved                                 */
    KS_IRQn                       =  46,      /*!< Key Store Interrupt                      */
    TIMER0_IRQn                   =  47,      /*!< Timer0 Interrupt                         */
    TIMER1_IRQn                   =  48,      /*!< Timer1 Interrupt                         */
    TIMER2_IRQn                   =  49,      /*!< Timer2 Interrupt                         */

    TIMER3_IRQn                   =  50,      /*!< Timer3 Interrupt                         */
    LPTMR0_IRQn                   =  51,      /*!< Low Power Timer 0 Interrupt              */
    LPTMR1_IRQn                   =  52,      /*!< Low Power Timer 1 Interrupt              */
    /*!< Reserved                                 */
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

    /*!< Reserved                                 */
    LPSPI0_IRQn                   =  71,      /*!< Low Power SPI 0 Interrupt                */
    /*!< Reserved                                 */
    SPIM0_IRQn                    =  73,      /*!< SPIM0 Interrupt                          */
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
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    EINT6_IRQn                    =  88,      /*!< External Input 6 Interrupt               */
    EINT7_IRQn                    =  89,      /*!< External Input 7 Interrupt               */

    LPUART0_IRQn                  =  90,      /*!< Low Power UART 0 Interrupt               */
    /*!< Reserved                                 */
    I2C0_IRQn                     =  92,      /*!< I2C0 Interrupt                           */
    I2C1_IRQn                     =  93,      /*!< I2C1 Interrupt                           */
    I2C2_IRQn                     =  94,      /*!< I2C2 Interrupt                           */
    I2C3_IRQn                     =  95,      /*!< I2C3 Interrupt                           */
    LPI2C0_IRQn                   =  96,      /*!< Low Power I2C 0 Interrupt                */
    USCI0_IRQn                    =  97,      /*!< USCI0 Interrupt                          */
    /*!< Reserved                                 */
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
    /*!< Reserved                                 */

    OTFC0_IRQn                    = 110,      /*!< OTFC0 Interrupt                          */
    /*!< Reserved                                 */
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
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    CRC_IRQn                      = 125,      /*!< CRC Interrupt                            */
    EADC00_IRQn                   = 126,      /*!< EADC0 Interrupt 0                        */
    EADC01_IRQn                   = 127,      /*!< EADC0 Interrupt 1                        */
    EADC02_IRQn                   = 128,      /*!< EADC0 Interrupt 2                        */
    EADC03_IRQn                   = 129,      /*!< EADC0 Interrupt 3                        */

    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
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
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    AWF_IRQn                      = 149,      /*!< AWF Interrupt                            */

    UTCPD_IRQn                    = 150,      /*!< UTCPD Interrupt                          */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */
    /*!< Reserved                                 */

    GDMACH0_IRQn                  = 160,      /*!< GDMA Channel 0 Interrupt                 */
    GDMACH1_IRQn                  = 161,      /*!< GDMA Channel 1 Interrupt                 */
    TOTAL_IRQn_CNT                = GDMACH1_IRQn + 16,
} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined (__ICCARM__)
#pragma language=extended
#elif defined(__AFMCC_VERSION) && (__AFMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning 586
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif


/* --------  Configuration of Core peripheral  ----------------------------------- */
#define __CM55_REV                0x0001U     /* Core revision r0p1 */
#define __NVIC_PRIO_BITS          3U          /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U          /* Set to 1 if different SysTick Config is used */
#define __VTOR_PRESENT            1U          /* VTOR present */
#define __MPU_PRESENT             1U          /* MPU present */
#define __FPU_PRESENT             1U          /* FPU present */
#define __FPU_DP                  1U          /* double precision FPU */
#define __DSP_PRESENT             1U          /* DSP extension present */
#define __SAUREGION_PRESENT       1U          /* SAU regions present */
#define __PMU_PRESENT             1U          /* PMU present */
#define __PMU_NUM_EVENTCNT        8U          /* PMU Event Counters */
#define __ICACHE_PRESENT          1U          /* Instruction Cache present */
#define __DCACHE_PRESENT          1U          /* Data Cache present */

/** @} end of group CMSIS_Device */


#include "core_cm55.h"                        /* Processor and core peripheral */
#include "system_M55M1.h"                     /* System header */
#if __has_include("partition_M55M1.h")
#include "partition_M55M1.h"                  /* User defined setup for Secure/Non-Secure Zones */
#else
#include "partition_M55M1_template.h"         /* Default setup for Secure/Non-Secure Zones */
#endif

#if __has_include("mpu_config_M55M1.h")
#include "mpu_config_M55M1.h"                  /* User defined setup for MPU regions */
#else
#include "mpu_config_M55M1_template.h"         /* Default setup for MPU regions */
#endif

/******************************************************************************/
/*                        Peripheral Register Structures                      */
/******************************************************************************/

/** @addtogroup REGISTER Control Register
    @{
*/

#include "acmp_reg.h"
#include "awf_reg.h"
#include "bpwm_reg.h"
#include "canfd_reg.h"
#include "ccap_reg.h"
#include "clk_reg.h"
#include "crc_reg.h"
#include "crypto_reg.h"
#include "dac_reg.h"
#include "dmic_reg.h"
#include "eadc_reg.h"
#include "ebi_reg.h"
#include "ecap_reg.h"
#include "epwm_reg.h"
#include "eqei_reg.h"
#include "fmc_reg.h"
#include "gpio_reg.h"
#include "hsotg_reg.h"
#include "hsusbd_reg.h"
#include "hsusbh_reg.h"
#include "i2c_reg.h"
#include "i2s_reg.h"
#include "i3c_reg.h"
#include "kdf_reg.h"
#include "kpi_reg.h"
#include "ks_reg.h"
#include "lpadc_reg.h"
#include "lpgpio_reg.h"
#include "lpi2c_reg.h"
#include "lppdma_reg.h"
#include "lpspi_reg.h"
#include "lptmr_reg.h"
#include "lpuart_reg.h"
#include "otfc_reg.h"
#include "otg_reg.h"
#include "pdma_reg.h"
#include "pmc_reg.h"
#include "psio_reg.h"
#include "qspi_reg.h"
#include "rtc_reg.h"
#include "sc_reg.h"
#include "scu_reg.h"
#include "sdh_reg.h"
#include "spi_reg.h"
#include "spim_reg.h"
#include "sys_reg.h"
#include "timer_reg.h"
#include "trng_reg.h"
#include "ttmr_reg.h"
#include "uart_reg.h"
#include "utcpd_reg.h"
#include "ui2c_reg.h"
#include "usbd_reg.h"
#include "usbh_reg.h"
#include "uspi_reg.h"
#include "uuart_reg.h"
#include "wdt_reg.h"
#include "wwdt_reg.h"

/** @} end of REGISTER group */


/******************************************************************************/
/*                         Peripheral Memory Map                              */
/******************************************************************************/

/** @addtogroup PERIPHERAL_MEMMAP Peripheral Memory Base
    Memory Mapped Structure for peripheral
    @{
 */

/* Peripheral and SRAM base address */
#define ITCM_BASE                 ((uint32_t)     0x00000000UL)  /*!< ITCM Base Address         */
#define FLASH_BASE                ((uint32_t)     0x00100000UL)  /*!< Flash Base Address        */
#define DTCM_BASE                 ((uint32_t)     0x20000000UL)  /*!< DTCM Base Address         */
#define SRAM_BASE                 ((uint32_t)     0x20100000UL)  /*!< SRAM0~2 Base Address      */
#define SRAM3_BASE                ((uint32_t)     0x20300000UL)  /*!< SRAM3 Base Address        */
#define LPSRAM_BASE               ((uint32_t)     0x20310000UL)  /*!< LPSRAM Base Address       */
#define PERIPH_BASE               ((uint32_t)     0x40000000UL)  /*!< Peripheral Base Address   */

/* Peripheral memory map */
#define AXIPERIPH_BASE            (PERIPH_BASE +  0x00000000UL)  /*!< AXI Base Address          */

#define AHB0PERIPH_BASE           (PERIPH_BASE +  0x00200000UL)  /*!< AHB0 Base Address         */
#define AHB1PERIPH_BASE           (PERIPH_BASE +  0x00040000UL)  /*!< AHB1 Base Address         */
#define AHB2PERIPH_BASE           (PERIPH_BASE +  0x00400000UL)  /*!< AHB2 Base Address         */

#define APB0PERIPH_BASE           (PERIPH_BASE +  0x00240000UL)  /*!< APB0 Base Address         */
#define APB1PERIPH_BASE           (PERIPH_BASE +  0x00250000UL)  /*!< APB1 Base Address         */
#define APB2PERIPH_BASE           (PERIPH_BASE +  0x00280000UL)  /*!< APB2 Base Address         */
#define APB3PERIPH_BASE           (PERIPH_BASE +  0x00290000UL)  /*!< APB3 Base Address         */
#define APB4PERIPH_BASE           (PERIPH_BASE +  0x00440000UL)  /*!< APB4 Base Address         */

/* AXI peripheral (PCLK5 clock domain) */
#define SYS_BASE                  (AXIPERIPH_BASE + 0x00000UL)
#define CLK_BASE                  (AXIPERIPH_BASE + 0x01000UL)
#define PMC_BASE                  (AXIPERIPH_BASE + 0x02000UL)
#define NPU_BASE                  (AXIPERIPH_BASE + 0x03000UL)
#define GDMA_BASE                 (AXIPERIPH_BASE + 0x04000UL)
#define SRAM0MPC_BASE             (AXIPERIPH_BASE + 0x08000UL)
#define SRAM1MPC_BASE             (AXIPERIPH_BASE + 0x09000UL)
#define SRAM2MPC_BASE             (AXIPERIPH_BASE + 0x0A000UL)
#define SRAM3MPC_BASE             (AXIPERIPH_BASE + 0x0B000UL)
#define SPIM0MPC_BASE             (AXIPERIPH_BASE + 0x0D000UL)
#define FMC_BASE                  (AXIPERIPH_BASE + 0x44000UL)  /*!< ACLK clock domain         */

/* AHB0 peripheral (HCLK0 clock domain) */
#define PDMA0_BASE                (AHB0PERIPH_BASE + 0x00000UL)
#define PDMA1_BASE                (AHB0PERIPH_BASE + 0x01000UL)
#define USBH0_BASE                (AHB0PERIPH_BASE + 0x02000UL)
#define USBH1_BASE                (AHB0PERIPH_BASE + 0x03000UL)
#define HSUSBH_BASE               (AHB0PERIPH_BASE + 0x04000UL)
#define HSUSBD_BASE               (AHB0PERIPH_BASE + 0x05000UL)
#define SDH0_BASE                 (AHB0PERIPH_BASE + 0x06000UL)
#define SDH1_BASE                 (AHB0PERIPH_BASE + 0x07000UL)
#define EMAC0_BASE                (AHB0PERIPH_BASE + 0x08000UL)
#define CRYPTO_BASE               (AHB0PERIPH_BASE + 0x0A000UL)
#define CRC_BASE                  (AHB0PERIPH_BASE + 0x0B000UL)

#define KS_BASE                   (AHB0PERIPH_BASE + 0x20000UL)
#define KDF_BASE                  (AHB0PERIPH_BASE + 0x21000UL)
#define CANFD0_BASE               (AHB0PERIPH_BASE + 0x22000UL)
#define CANFD1_BASE               (AHB0PERIPH_BASE + 0x24000UL)
#define GPIOA_BASE                (AHB0PERIPH_BASE + 0x29000UL)
#define GPIOB_BASE                (AHB0PERIPH_BASE + 0x29040UL)
#define GPIOC_BASE                (AHB0PERIPH_BASE + 0x29080UL)
#define GPIOD_BASE                (AHB0PERIPH_BASE + 0x290C0UL)
#define GPIOE_BASE                (AHB0PERIPH_BASE + 0x29100UL)
#define GPIOF_BASE                (AHB0PERIPH_BASE + 0x29140UL)
#define GPIOG_BASE                (AHB0PERIPH_BASE + 0x29180UL)
#define GPIOH_BASE                (AHB0PERIPH_BASE + 0x291C0UL)
#define GPIOI_BASE                (AHB0PERIPH_BASE + 0x29200UL)
#define GPIOJ_BASE                (AHB0PERIPH_BASE + 0x29240UL)
#define GPIO_INT_BASE             (AHB0PERIPH_BASE + 0x29440UL)
#define GPIO_PIN_DATA_BASE        (AHB0PERIPH_BASE + 0x29800UL)
#define EBI_BASE                  (AHB0PERIPH_BASE + 0x30000UL)

/* AHB1 peripheral (HCLK1 clock domain) */
#define OTFC0_BASE                (AHB1PERIPH_BASE + 0x00000UL)
#define SPIM0_BASE                (AHB1PERIPH_BASE + 0x02000UL)
/* AHB2 peripheral (HCLK2 clock domain) */
#define LPPDMA_BASE               (AHB2PERIPH_BASE + 0x00000UL)
#define CCAP_BASE                 (AHB2PERIPH_BASE + 0x01000UL)
#define SCU_BASE                  (AHB2PERIPH_BASE + 0x02000UL)
#define FVC_BASE                  (AHB2PERIPH_BASE + 0x02500UL)
#define DPM_BASE                  (AHB2PERIPH_BASE + 0x02600UL)
#define PLM_BASE                  (AHB2PERIPH_BASE + 0x02700UL)
#define LPGPIO_BASE               (AHB2PERIPH_BASE + 0x03000UL)

/* APB0 peripheral (PCLK0 clock domain) */
#define WWDT0_BASE                (APB0PERIPH_BASE + 0x00000UL)
#define EADC0_BASE                (APB0PERIPH_BASE + 0x01000UL)
#define EPWM0_BASE                (APB0PERIPH_BASE + 0x02000UL)
#define BPWM0_BASE                (APB0PERIPH_BASE + 0x03000UL)
#define EQEI0_BASE                (APB0PERIPH_BASE + 0x04000UL)
#define EQEI2_BASE                (APB0PERIPH_BASE + 0x05000UL)
#define ECAP0_BASE                (APB0PERIPH_BASE + 0x06000UL)
#define ECAP2_BASE                (APB0PERIPH_BASE + 0x07000UL)
#define I2C0_BASE                 (APB0PERIPH_BASE + 0x08000UL)
#define I2C2_BASE                 (APB0PERIPH_BASE + 0x09000UL)
#define QSPI0_BASE                (APB0PERIPH_BASE + 0x0A000UL)
#define SPI0_BASE                 (APB0PERIPH_BASE + 0x0B000UL)
#define SPI2_BASE                 (APB0PERIPH_BASE + 0x0C000UL)
#define UART0_BASE                (APB0PERIPH_BASE + 0x0D000UL)
#define UART2_BASE                (APB0PERIPH_BASE + 0x0E000UL)
#define UART4_BASE                (APB0PERIPH_BASE + 0x0F000UL)

/* APB1 peripheral (PCLK1 clock domain) */
#define UART6_BASE                (APB1PERIPH_BASE + 0x00000UL)
#define UART8_BASE                (APB1PERIPH_BASE + 0x01000UL)
#define USCI0_BASE                (APB1PERIPH_BASE + 0x02000UL)
#define SC0_BASE                  (APB1PERIPH_BASE + 0x03000UL)
#define SC2_BASE                  (APB1PERIPH_BASE + 0x04000UL)
#define PSIO_BASE                 (APB1PERIPH_BASE + 0x05000UL)
#define TIMER0_BASE               (APB1PERIPH_BASE + 0x06000UL)
#define TIMER1_BASE               (APB1PERIPH_BASE + 0x06100UL)
#define DAC0_BASE                 (APB1PERIPH_BASE + 0x07000UL)
#define DAC1_BASE                 (APB1PERIPH_BASE + 0x07040UL)
#define HSOTG_BASE                (APB1PERIPH_BASE + 0x09000UL)
#define I2S0_BASE                 (APB1PERIPH_BASE + 0x0A000UL)
#define ACMP01_BASE               (APB1PERIPH_BASE + 0x0B000UL)
#define USBD_BASE                 (APB1PERIPH_BASE + 0x0C000UL)

/* APB2 peripheral (PCLK2 clock domain) */
#define WWDT1_BASE                (APB2PERIPH_BASE + 0x00000UL)
#define EPWM1_BASE                (APB2PERIPH_BASE + 0x02000UL)
#define BPWM1_BASE                (APB2PERIPH_BASE + 0x03000UL)
#define EQEI1_BASE                (APB2PERIPH_BASE + 0x04000UL)
#define EQEI3_BASE                (APB2PERIPH_BASE + 0x05000UL)
#define ECAP1_BASE                (APB2PERIPH_BASE + 0x06000UL)
#define ECAP3_BASE                (APB2PERIPH_BASE + 0x07000UL)
#define I2C1_BASE                 (APB2PERIPH_BASE + 0x08000UL)
#define I2C3_BASE                 (APB2PERIPH_BASE + 0x09000UL)
#define QSPI1_BASE                (APB2PERIPH_BASE + 0x0A000UL)
#define SPI1_BASE                 (APB2PERIPH_BASE + 0x0B000UL)
#define SPI3_BASE                 (APB2PERIPH_BASE + 0x0C000UL)
#define UART1_BASE                (APB2PERIPH_BASE + 0x0D000UL)
#define UART3_BASE                (APB2PERIPH_BASE + 0x0E000UL)
#define UART5_BASE                (APB2PERIPH_BASE + 0x0F000UL)

/* APB3 peripheral (PCLK3 clock domain) */
#define UART7_BASE                (APB3PERIPH_BASE + 0x00000UL)
#define UART9_BASE                (APB3PERIPH_BASE + 0x01000UL)
#define SC1_BASE                  (APB3PERIPH_BASE + 0x02000UL)
#define OTG_BASE                  (APB3PERIPH_BASE + 0x03000UL)
#define KPI_BASE                  (APB3PERIPH_BASE + 0x04000UL)
#define TIMER2_BASE               (APB3PERIPH_BASE + 0x05000UL)
#define TIMER3_BASE               (APB3PERIPH_BASE + 0x05100UL)
#define TRNG_BASE                 (APB3PERIPH_BASE + 0x06000UL)
#define RTC_BASE                  (APB3PERIPH_BASE + 0x07000UL)
#define I2S1_BASE                 (APB3PERIPH_BASE + 0x08000UL)
#define ACMP23_BASE               (APB3PERIPH_BASE + 0x09000UL)
#define I3C0_BASE                 (APB3PERIPH_BASE + 0x0A000UL)
#define UTCPD_BASE                (APB3PERIPH_BASE + 0x0B000UL)

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UTCPD_Msk)
#define UTCPD0_BASE               (APB3PERIPH_BASE + 0x0B000UL+ NS_OFFSET)
#define UTCPD1_BASE               (APB3PERIPH_BASE + 0x0B000UL+ NS_OFFSET)
#else
#define UTCPD0_BASE               (APB3PERIPH_BASE + 0x0B000UL)
#define UTCPD1_BASE               (APB3PERIPH_BASE + 0x0B000UL)
#endif

/* APB4 peripheral (PCLK4 clock domain) */
#define LPTMR0_BASE               (APB4PERIPH_BASE + 0x00000UL)
#define LPTMR1_BASE               (APB4PERIPH_BASE + 0x00100UL)
#define TTMR0_BASE                (APB4PERIPH_BASE + 0x01000UL)
#define TTMR1_BASE                (APB4PERIPH_BASE + 0x01100UL)
#define LPADC0_BASE               (APB4PERIPH_BASE + 0x02000UL)
#define LPI2C0_BASE               (APB4PERIPH_BASE + 0x03000UL)
#define LPSPI0_BASE               (APB4PERIPH_BASE + 0x04000UL)
#define DMIC0_BASE                (APB4PERIPH_BASE + 0x05000UL)
#define VAD0_BASE                 (APB4PERIPH_BASE + 0x05300UL)
#define LPUART0_BASE              (APB4PERIPH_BASE + 0x06000UL)
#define WDT0_BASE                 (APB4PERIPH_BASE + 0x07000UL)
#define WDT1_BASE                 (APB4PERIPH_BASE + 0x08000UL)
#define AWF_BASE                  (APB4PERIPH_BASE + 0x09000UL)
#define LPSRAMMPC_BASE            (APB4PERIPH_BASE + 0x10000UL)

/** @} end of group PERIPHERAL_MEMMAP */


/******************************************************************************/
/*                         Peripheral Declaration                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL_DECLARATION Peripheral Pointer
  The Declaration of Peripheral Pointer
  @{
 */

/** @addtogroup PERIPHERAL_S Secure Peripheral Pointer
    The Declaration of Secure Peripheral Pointer
    @{
*/

#define ACMP01_S                  ((ACMP_T *)     ACMP01_BASE)
#define ACMP23_S                  ((ACMP_T *)     ACMP23_BASE)
#define AWF_S                     ((AWF_T *)      AWF_BASE)
#define BPWM0_S                   ((BPWM_T *)     BPWM0_BASE)
#define BPWM1_S                   ((BPWM_T *)     BPWM1_BASE)
#define CANFD0_S                  ((CANFD_T *)    CANFD0_BASE)
#define CANFD1_S                  ((CANFD_T *)    CANFD1_BASE)
#define CCAP_S                    ((CCAP_T *)     CCAP_BASE)
#define CLK_S                     ((CLK_T *)      CLK_BASE)
#define CRC_S                     ((CRC_T *)      CRC_BASE)
#define CRYPTO_S                  ((CRYPTO_T *)   CRYPTO_BASE)
#define DAC0_S                    ((DAC_T *)      DAC0_BASE)
#define DAC1_S                    ((DAC_T *)      DAC1_BASE)
#define DMIC0_S                   ((DMIC_T *)     DMIC0_BASE)
#define DPM_S                     ((DPM_T *)      DPM_BASE)
#define EADC0_S                   ((EADC_T *)     EADC0_BASE)
#define ECAP0_S                   ((ECAP_T *)     ECAP0_BASE)
#define ECAP1_S                   ((ECAP_T *)     ECAP1_BASE)
#define ECAP2_S                   ((ECAP_T *)     ECAP2_BASE)
#define ECAP3_S                   ((ECAP_T *)     ECAP3_BASE)
#define EBI_S                     ((EBI_T *)      EBI_BASE)
#define EMAC0_S                   ((EMAC_T *)     EMAC0_BASE)
#define EPWM0_S                   ((EPWM_T *)     EPWM0_BASE)
#define EPWM1_S                   ((EPWM_T *)     EPWM1_BASE)
#define EQEI0_S                   ((EQEI_T *)     EQEI0_BASE)
#define EQEI1_S                   ((EQEI_T *)     EQEI1_BASE)
#define EQEI2_S                   ((EQEI_T *)     EQEI2_BASE)
#define EQEI3_S                   ((EQEI_T *)     EQEI3_BASE)
#define FMC_S                     ((FMC_T *)      FMC_BASE)
#define FVC_S                     ((FVC_T *)      FVC_BASE)

#define GDMA_S                                   (GDMA_BASE)
#define GPIO_S                    ((GPIO_INT_T *) GPIO_INT_BASE)
#define HSOTG_S                   ((HSOTG_T *)    HSOTG_BASE)
#define HSUSBD_S                  ((HSUSBD_T *)   HSUSBD_BASE)
#define HSUSBH_S                  ((HSUSBH_T *)   HSUSBH_BASE)
#define I2C0_S                    ((I2C_T *)      I2C0_BASE)
#define I2C1_S                    ((I2C_T *)      I2C1_BASE)
#define I2C2_S                    ((I2C_T *)      I2C2_BASE)
#define I2C3_S                    ((I2C_T *)      I2C3_BASE)
#define I2S0_S                    ((I2S_T *)      I2S0_BASE)
#define I2S1_S                    ((I2S_T *)      I2S1_BASE)
#define I3C0_S                    ((I3C_T *)      I3C0_BASE)
#define KDF_S                     ((KDF_T *)      KDF_BASE)
#define KPI_S                     ((KPI_T *)      KPI_BASE)
#define KS_S                      ((KS_T *)       KS_BASE)
#define LPADC0_S                  ((LPADC_T *)    LPADC0_BASE)
#define LPGPIO_S                  ((LPGPIO_T *)   LPGPIO_BASE)
#define LPI2C0_S                  ((LPI2C_T *)    LPI2C0_BASE)
#define LPPDMA_S                  ((LPPDMA_T *)   LPPDMA_BASE)
#define LPSPI0_S                  ((LPSPI_T *)    LPSPI0_BASE)
#define LPTMR0_S                  ((LPTMR_T *)    LPTMR0_BASE)
#define LPTMR1_S                  ((LPTMR_T *)    LPTMR1_BASE)
#define LPUART0_S                 ((LPUART_T *)   LPUART0_BASE)

#define NPU_S                                    (NPU_BASE)
#define OTFC0_S                   ((OTFC_T *)     OTFC0_BASE)
#define OTG_S                     ((OTG_T *)      OTG_BASE)
#define PA_S                      ((GPIO_T *)     GPIOA_BASE)
#define PB_S                      ((GPIO_T *)     GPIOB_BASE)
#define PC_S                      ((GPIO_T *)     GPIOC_BASE)
#define PD_S                      ((GPIO_T *)     GPIOD_BASE)
#define PE_S                      ((GPIO_T *)     GPIOE_BASE)
#define PF_S                      ((GPIO_T *)     GPIOF_BASE)
#define PG_S                      ((GPIO_T *)     GPIOG_BASE)
#define PH_S                      ((GPIO_T *)     GPIOH_BASE)
#define PI_S                      ((GPIO_T *)     GPIOI_BASE)
#define PJ_S                      ((GPIO_T *)     GPIOJ_BASE)
#define PDMA0_S                   ((PDMA_T *)     PDMA0_BASE)
#define PDMA1_S                   ((PDMA_T *)     PDMA1_BASE)
#define PLM_S                     ((PLM_T *)      PLM_BASE)
#define PMC_S                     ((PMC_T *)      PMC_BASE)
#define PSIO_S                    ((PSIO_T *)     PSIO_BASE)
#define QSPI0_S                   ((QSPI_T *)     QSPI0_BASE)
#define QSPI1_S                   ((QSPI_T *)     QSPI1_BASE)
#define RTC_S                     ((RTC_T *)      RTC_BASE)

#define SC0_S                     ((SC_T *)       SC0_BASE)
#define SC1_S                     ((SC_T *)       SC1_BASE)
#define SC2_S                     ((SC_T *)       SC2_BASE)
#define SCU_S                     ((SCU_T *)      SCU_BASE)
#define SDH0_S                    ((SDH_T *)      SDH0_BASE)
#define SDH1_S                    ((SDH_T *)      SDH1_BASE)
#define SPI0_S                    ((SPI_T *)      SPI0_BASE)
#define SPI1_S                    ((SPI_T *)      SPI1_BASE)
#define SPI2_S                    ((SPI_T *)      SPI2_BASE)
#define SPI3_S                    ((SPI_T *)      SPI3_BASE)
#define SPIM0_S                   ((SPIM_T *)     SPIM0_BASE)
#define SYS_S                     ((SYS_T *)      SYS_BASE)
#define TIMER0_S                  ((TIMER_T *)    TIMER0_BASE)
#define TIMER1_S                  ((TIMER_T *)    TIMER1_BASE)
#define TIMER2_S                  ((TIMER_T *)    TIMER2_BASE)
#define TIMER3_S                  ((TIMER_T *)    TIMER3_BASE)
#define TRNG_S                    ((TRNG_T *)     TRNG_BASE)
#define TTMR0_S                   ((TTMR_T *)     TTMR0_BASE)
#define TTMR1_S                   ((TTMR_T *)     TTMR1_BASE)
#define UART0_S                   ((UART_T *)     UART0_BASE)
#define UART1_S                   ((UART_T *)     UART1_BASE)
#define UART2_S                   ((UART_T *)     UART2_BASE)
#define UART3_S                   ((UART_T *)     UART3_BASE)
#define UART4_S                   ((UART_T *)     UART4_BASE)
#define UART5_S                   ((UART_T *)     UART5_BASE)
#define UART6_S                   ((UART_T *)     UART6_BASE)
#define UART7_S                   ((UART_T *)     UART7_BASE)
#define UART8_S                   ((UART_T *)     UART8_BASE)
#define UART9_S                   ((UART_T *)     UART9_BASE)
#define USBH0_S                   ((USBH_T *)     USBH0_BASE)
#define USBH1_S                   ((USBH1_T *)    USBH1_BASE)
#define USBD_S                    ((USBD_T *)     USBD_BASE)
#define UI2C0_S                   ((UI2C_T *)     USCI0_BASE)
#define USPI0_S                   ((USPI_T *)     USCI0_BASE)
#define UTCPD_S                   ((UTCPD_T *)    UTCPD_BASE)
#define UUART0_S                  ((UUART_T *)    USCI0_BASE)
#define VAD0_S                    ((VAD_T *)      VAD0_BASE)
#define WDT0_S                    ((WDT_T *)      WDT0_BASE)
#define WWDT0_S                   ((WWDT_T *)     WWDT0_BASE)
#define WWDT1_S                   ((WWDT_T *)     WWDT1_BASE)

/** @} end of group PERIPHERAL_S */

/** @addtogroup PERIPHERAL_NS Non-Secure Peripheral Pointer
    The Declaration of Non-Secure Peripheral Pointer
    @{
*/

#define NS_OFFSET                 ((uint32_t)     0x10000000UL)   /*!< Non-Secure Address Offset */

#define ACMP01_NS                 ((ACMP_T *)     (ACMP01_BASE    + NS_OFFSET))
#define ACMP23_NS                 ((ACMP_T *)     (ACMP23_BASE    + NS_OFFSET))
#define AWF_NS                    ((AWF_T *)      (AWF_BASE       + NS_OFFSET))
#define BPWM0_NS                  ((BPWM_T *)     (BPWM0_BASE     + NS_OFFSET))
#define BPWM1_NS                  ((BPWM_T *)     (BPWM1_BASE     + NS_OFFSET))
#define CANFD0_NS                 ((CANFD_T *)    (CANFD0_BASE    + NS_OFFSET))
#define CANFD1_NS                 ((CANFD_T *)    (CANFD1_BASE    + NS_OFFSET))
#define CCAP_NS                   ((CCAP_T *)     (CCAP_BASE      + NS_OFFSET))
#define CRC_NS                    ((CRC_T *)      (CRC_BASE       + NS_OFFSET))
#define CRYPTO_NS                 ((CRYPTO_T *)   (CRYPTO_BASE    + NS_OFFSET))
#define DAC0_NS                   ((DAC_T *)      (DAC0_BASE      + NS_OFFSET))
#define DAC1_NS                   ((DAC_T *)      (DAC1_BASE      + NS_OFFSET))
#define DMIC0_NS                  ((DMIC_T *)     (DMIC0_BASE     + NS_OFFSET))
#define EADC0_NS                  ((EADC_T *)     (EADC0_BASE     + NS_OFFSET))
#define ECAP0_NS                  ((ECAP_T *)     (ECAP0_BASE     + NS_OFFSET))
#define ECAP1_NS                  ((ECAP_T *)     (ECAP1_BASE     + NS_OFFSET))
#define ECAP2_NS                  ((ECAP_T *)     (ECAP2_BASE     + NS_OFFSET))
#define ECAP3_NS                  ((ECAP_T *)     (ECAP3_BASE     + NS_OFFSET))
#define EBI_NS                    ((EBI_T *)      (EBI_BASE       + NS_OFFSET))
#define EMAC0_NS                  ((EMAC_T *)     (EMAC0_BASE     + NS_OFFSET))
#define EPWM0_NS                  ((EPWM_T *)     (EPWM0_BASE     + NS_OFFSET))
#define EPWM1_NS                  ((EPWM_T *)     (EPWM1_BASE     + NS_OFFSET))
#define EQEI0_NS                  ((EQEI_T *)     (EQEI0_BASE     + NS_OFFSET))
#define EQEI1_NS                  ((EQEI_T *)     (EQEI1_BASE     + NS_OFFSET))
#define EQEI2_NS                  ((EQEI_T *)     (EQEI2_BASE     + NS_OFFSET))
#define EQEI3_NS                  ((EQEI_T *)     (EQEI3_BASE     + NS_OFFSET))
#define FMC_NS                    ((FMC_T *)      (FMC_BASE       + NS_OFFSET))

#define GDMA_NS                                   (GDMA_BASE      + NS_OFFSET)
#define GPIO_NS                   ((GPIO_INT_T *) (GPIO_INT_BASE  + NS_OFFSET))
#define HSOTG_NS                  ((HSOTG_T *)    (HSOTG_BASE     + NS_OFFSET))
#define HSUSBD_NS                 ((HSUSBD_T *)   (HSUSBD_BASE    + NS_OFFSET))
#define HSUSBH_NS                 ((HSUSBH_T *)   (HSUSBH_BASE    + NS_OFFSET))
#define I2C0_NS                   ((I2C_T *)      (I2C0_BASE      + NS_OFFSET))
#define I2C1_NS                   ((I2C_T *)      (I2C1_BASE      + NS_OFFSET))
#define I2C2_NS                   ((I2C_T *)      (I2C2_BASE      + NS_OFFSET))
#define I2C3_NS                   ((I2C_T *)      (I2C3_BASE      + NS_OFFSET))
#define I2S0_NS                   ((I2S_T *)      (I2S0_BASE      + NS_OFFSET))
#define I2S1_NS                   ((I2S_T *)      (I2S1_BASE      + NS_OFFSET))
#define I3C0_NS                   ((I3C_T *)      (I3C0_BASE      + NS_OFFSET))
#define KDF_NS                    ((KDF_T *)      (KDF_BASE       + NS_OFFSET))
#define KPI_NS                    ((KPI_T *)      (KPI_BASE       + NS_OFFSET))
#define LPADC0_NS                 ((LPADC_T *)    (LPADC0_BASE    + NS_OFFSET))
#define LPGPIO_NS                 ((LPGPIO_T *)   (LPGPIO_BASE    + NS_OFFSET))
#define LPI2C0_NS                 ((LPI2C_T *)    (LPI2C0_BASE    + NS_OFFSET))
#define LPPDMA_NS                 ((LPPDMA_T *)   (LPPDMA_BASE    + NS_OFFSET))
#define LPSPI0_NS                 ((LPSPI_T *)    (LPSPI0_BASE    + NS_OFFSET))
#define LPTMR0_NS                 ((LPTMR_T *)    (LPTMR0_BASE    + NS_OFFSET))
#define LPTMR1_NS                 ((LPTMR_T *)    (LPTMR1_BASE    + NS_OFFSET))
#define LPUART0_NS                ((LPUART_T *)   (LPUART0_BASE   + NS_OFFSET))

#define NPU_NS                                    (NPU_BASE       + NS_OFFSET)
#define OTG_NS                    ((OTG_T *)      (OTG_BASE       + NS_OFFSET))
#define PA_NS                     ((GPIO_T *)     (GPIOA_BASE     + NS_OFFSET))
#define PB_NS                     ((GPIO_T *)     (GPIOB_BASE     + NS_OFFSET))
#define PC_NS                     ((GPIO_T *)     (GPIOC_BASE     + NS_OFFSET))
#define PD_NS                     ((GPIO_T *)     (GPIOD_BASE     + NS_OFFSET))
#define PE_NS                     ((GPIO_T *)     (GPIOE_BASE     + NS_OFFSET))
#define PF_NS                     ((GPIO_T *)     (GPIOF_BASE     + NS_OFFSET))
#define PG_NS                     ((GPIO_T *)     (GPIOG_BASE     + NS_OFFSET))
#define PH_NS                     ((GPIO_T *)     (GPIOH_BASE     + NS_OFFSET))
#define PI_NS                     ((GPIO_T *)     (GPIOI_BASE     + NS_OFFSET))
#define PJ_NS                     ((GPIO_T *)     (GPIOJ_BASE     + NS_OFFSET))
#define PDMA0_NS                  ((PDMA_T *)     (PDMA0_BASE     + NS_OFFSET))
#define PDMA1_NS                  ((PDMA_T *)     (PDMA1_BASE     + NS_OFFSET))
#define PSIO_NS                   ((PSIO_T *)     (PSIO_BASE      + NS_OFFSET))
#define QSPI0_NS                  ((QSPI_T *)     (QSPI0_BASE     + NS_OFFSET))
#define QSPI1_NS                  ((QSPI_T *)     (QSPI1_BASE     + NS_OFFSET))

#define SC0_NS                    ((SC_T *)       (SC0_BASE       + NS_OFFSET))
#define SC1_NS                    ((SC_T *)       (SC1_BASE       + NS_OFFSET))
#define SC2_NS                    ((SC_T *)       (SC2_BASE       + NS_OFFSET))
#define DPM_NS                    ((DPM_T *)      (DPM_BASE       + NS_OFFSET))
#define SDH0_NS                   ((SDH_T *)      (SDH0_BASE      + NS_OFFSET))
#define SDH1_NS                   ((SDH_T *)      (SDH1_BASE      + NS_OFFSET))
#define SPI0_NS                   ((SPI_T *)      (SPI0_BASE      + NS_OFFSET))
#define SPI1_NS                   ((SPI_T *)      (SPI1_BASE      + NS_OFFSET))
#define SPI2_NS                   ((SPI_T *)      (SPI2_BASE      + NS_OFFSET))
#define SPI3_NS                   ((SPI_T *)      (SPI3_BASE      + NS_OFFSET))
#define SPIM0_NS                  ((SPIM_T *)     (SPIM0_BASE     + NS_OFFSET))
#define SYS_NS                    ((SYS_T *)      (SYS_BASE       + NS_OFFSET))
#define TIMER0_NS                 ((TIMER_T *)    (TIMER0_BASE    + NS_OFFSET))
#define TIMER1_NS                 ((TIMER_T *)    (TIMER1_BASE    + NS_OFFSET))
#define TIMER2_NS                 ((TIMER_T *)    (TIMER2_BASE    + NS_OFFSET))
#define TIMER3_NS                 ((TIMER_T *)    (TIMER3_BASE    + NS_OFFSET))
#define TRNG_NS                   ((TRNG_T *)     (TRNG_BASE      + NS_OFFSET))
#define TTMR0_NS                  ((TTMR_T *)     (TTMR0_BASE     + NS_OFFSET))
#define TTMR1_NS                  ((TTMR_T *)     (TTMR1_BASE     + NS_OFFSET))
#define UART0_NS                  ((UART_T *)     (UART0_BASE     + NS_OFFSET))
#define UART1_NS                  ((UART_T *)     (UART1_BASE     + NS_OFFSET))
#define UART2_NS                  ((UART_T *)     (UART2_BASE     + NS_OFFSET))
#define UART3_NS                  ((UART_T *)     (UART3_BASE     + NS_OFFSET))
#define UART4_NS                  ((UART_T *)     (UART4_BASE     + NS_OFFSET))
#define UART5_NS                  ((UART_T *)     (UART5_BASE     + NS_OFFSET))
#define UART6_NS                  ((UART_T *)     (UART6_BASE     + NS_OFFSET))
#define UART7_NS                  ((UART_T *)     (UART7_BASE     + NS_OFFSET))
#define UART8_NS                  ((UART_T *)     (UART8_BASE     + NS_OFFSET))
#define UART9_NS                  ((UART_T *)     (UART9_BASE     + NS_OFFSET))
#define USBH0_NS                  ((USBH_T *)     (USBH0_BASE     + NS_OFFSET))
#define USBH1_NS                  ((USBH1_T *)    (USBH1_BASE     + NS_OFFSET))
#define USBD_NS                   ((USBD_T *)     (USBD_BASE      + NS_OFFSET))
#define UI2C0_NS                  ((UI2C_T *)     (USCI0_BASE     + NS_OFFSET))
#define USPI0_NS                  ((USPI_T *)     (USCI0_BASE     + NS_OFFSET))
#define UTCPD_NS                  ((UTCPD_T *)    (UTCPD_BASE     + NS_OFFSET))
#define UUART0_NS                 ((UUART_T *)    (USCI0_BASE     + NS_OFFSET))
#define VAD0_NS                   ((VAD_T *)      (VAD0_BASE      + NS_OFFSET))
#define WDT1_NS                   ((WDT_T *)      (WDT1_BASE      + NS_OFFSET))
#define WWDT0_NS                  ((WWDT_T *)     (WWDT0_BASE     + NS_OFFSET))
#define WWDT1_NS                  ((WWDT_T *)     (WWDT1_BASE     + NS_OFFSET))

/** @} end of group PERIPHERAL_NS */

/** @addtogroup PERIPHERAL_SNS Peripheral Pointer
    The Declaration of Peripheral Pointer
    @{
*/

# define PA       ((__PC() & NS_OFFSET) ? PA_NS : PA_S)
# define PB       ((__PC() & NS_OFFSET) ? PB_NS : PB_S)
# define PC       ((__PC() & NS_OFFSET) ? PC_NS : PC_S)
# define PD       ((__PC() & NS_OFFSET) ? PD_NS : PD_S)
# define PE       ((__PC() & NS_OFFSET) ? PE_NS : PE_S)
# define PF       ((__PC() & NS_OFFSET) ? PF_NS : PF_S)
# define PG       ((__PC() & NS_OFFSET) ? PG_NS : PG_S)
# define PH       ((__PC() & NS_OFFSET) ? PH_NS : PH_S)
# define PI       ((__PC() & NS_OFFSET) ? PI_NS : PI_S)
# define PJ       ((__PC() & NS_OFFSET) ? PJ_NS : PJ_S)

/* Always Secure peripheral */
#define CLK       CLK_S
#define DPM       DPM_S
#define FVC       FVC_S
#define FMC       FMC_S
#define GPIO      GPIO_S
#define KS        KS_S
#define OTFC0     OTFC0_S
#define PLM       PLM_S
#define PMC       PMC_S
#define SCU       SCU_S
#define SYS       SYS_S
#define WDT0      WDT0_S

/* Always Non-Secure peripheral */
#define WDT1      WDT1_NS

/* SCU Configured peripheral */
/* SCU_D0PNS0 */
#if defined (SCU_INIT_D0PNS0_VAL) && (SCU_INIT_D0PNS0_VAL & SCU_D0PNS0_NPU_Msk)
#define NPU          NPU_NS
#else
#define NPU          NPU_S
#endif

/* SCU_D0PNS2 */
#if defined (SCU_INIT_D0PNS2_VAL) && (SCU_INIT_D0PNS2_VAL & SCU_D0PNS2_SPIM0_Msk)
#define SPIM0        SPIM0_NS
#else
#define SPIM0        SPIM0_S
#endif

/* SCU_D1PNS0 */
#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_PDMA0_Msk)
#define PDMA0        PDMA0_NS
#else
#define PDMA0        PDMA0_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_PDMA1_Msk)
#define PDMA1        PDMA1_NS
#else
#define PDMA1        PDMA1_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_USBH0_Msk)
#define USBH0        USBH0_NS
#else
#define USBH0        USBH0_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_USBH1_Msk)
#define USBH1        USBH1_NS
#else
#define USBH1        USBH1_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_HSUSBH_Msk)
#define HSUSBH       HSUSBH_NS
#else
#define HSUSBH       HSUSBH_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_HSUSBD_Msk)
#define HSUSBD       HSUSBD_NS
#else
#define HSUSBD       HSUSBD_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_SDH0_Msk)
#define SDH0         SDH0_NS
#else
#define SDH0         SDH0_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_SDH1_Msk)
#define SDH1         SDH1_NS
#else
#define SDH1         SDH1_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_EMAC0_Msk)
#define EMAC0        EMAC0_NS
#else
#define EMAC0        EMAC0_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_CRYPTO_Msk)
#define CRYPTO       CRYPTO_NS
#else
#define CRYPTO       CRYPTO_S
#endif

#if defined (SCU_INIT_D1PNS0_VAL) && (SCU_INIT_D1PNS0_VAL & SCU_D1PNS0_CRC_Msk)
#define CRC          CRC_NS
#else
#define CRC          CRC_S
#endif

/* SCU_D1PNS1 */
#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_KDF_Msk)
#define KDF          KDF_NS
#else
#define KDF          KDF_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_CANFD0_Msk)
#define CANFD0       CANFD0_NS
#else
#define CANFD0       CANFD0_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_CANFD1_Msk)
#define CANFD1       CANFD1_NS
#else
#define CANFD1       CANFD1_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_ETMC_Msk)
#define ETMC         ETMC_NS
#else
#define ETMC         ETMC_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_SWDH_Msk)
#define SWDH         SWDH_NS
#else
#define SWDH         SWDH_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_SWODEC_Msk)
#define SWODEC       SWODEC_NS
#else
#define SWODEC       SWODEC_S
#endif

#if defined (SCU_INIT_D1PNS1_VAL) && (SCU_INIT_D1PNS1_VAL & SCU_D1PNS1_EBI_Msk)
#define EBI          EBI_NS
#else
#define EBI          EBI_S
#endif

/* SCU_D1PNS2 */
#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_WWDT0_Msk)
#define WWDT0        WWDT0_NS
#else
#define WWDT0        WWDT0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_EADC0_Msk)
#define EADC0        EADC0_NS
#else
#define EADC0        EADC0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_EPWM0_Msk)
#define EPWM0        EPWM0_NS
#else
#define EPWM0        EPWM0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_BPWM0_Msk)
#define BPWM0        BPWM0_NS
#else
#define BPWM0        BPWM0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_EQEI0_Msk)
#define EQEI0        EQEI0_NS
#else
#define EQEI0        EQEI0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_EQEI2_Msk)
#define EQEI2        EQEI2_NS
#else
#define EQEI2        EQEI2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_ECAP0_Msk)
#define ECAP0        ECAP0_NS
#else
#define ECAP0        ECAP0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_ECAP2_Msk)
#define ECAP2        ECAP2_NS
#else
#define ECAP2        ECAP2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_I2C0_Msk)
#define I2C0         I2C0_NS
#else
#define I2C0         I2C0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_I2C2_Msk)
#define I2C2         I2C2_NS
#else
#define I2C2         I2C2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_QSPI0_Msk)
#define QSPI0        QSPI0_NS
#else
#define QSPI0        QSPI0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_SPI0_Msk)
#define SPI0         SPI0_NS
#else
#define SPI0         SPI0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_SPI2_Msk)
#define SPI2         SPI2_NS
#else
#define SPI2         SPI2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_UART0_Msk)
#define UART0        UART0_NS
#else
#define UART0        UART0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_UART2_Msk)
#define UART2        UART2_NS
#else
#define UART2        UART2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_UART4_Msk)
#define UART4        UART4_NS
#else
#define UART4        UART4_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_UART6_Msk)
#define UART6        UART6_NS
#else
#define UART6        UART6_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_UART8_Msk)
#define UART8        UART8_NS
#else
#define UART8        UART8_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_USCI0_Msk)
#define USCI0        USCI0_NS
#define UI2C0        UI2C0_NS
#define USPI0        USPI0_NS
#define UUART0       UUART0_NS
#else
#define USCI0        USCI0_S
#define UI2C0        UI2C0_S
#define USPI0        USPI0_S
#define UUART0       UUART0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_SC0_Msk)
#define SC0          SC0_NS
#else
#define SC0          SC0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_SC2_Msk)
#define SC2          SC2_NS
#else
#define SC2          SC2_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_PSIO_Msk)
#define PSIO         PSIO_NS
#else
#define PSIO         PSIO_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_TMR01_Msk)
#define TIMER0       TIMER0_NS
#define TIMER1       TIMER1_NS
#else
#define TIMER0       TIMER0_S
#define TIMER1       TIMER1_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_DAC01_Msk)
#define DAC0         DAC0_NS
#define DAC1         DAC1_NS
#else
#define DAC0         DAC0_S
#define DAC1         DAC1_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_HSOTG_Msk)
#define HSOTG        HSOTG_NS
#else
#define HSOTG        HSOTG_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_I2S0_Msk)
#define I2S0         I2S0_NS
#else
#define I2S0         I2S0_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_ACMP01_Msk)
#define ACMP01       ACMP01_NS
#else
#define ACMP01       ACMP01_S
#endif

#if defined (SCU_INIT_D1PNS2_VAL) && (SCU_INIT_D1PNS2_VAL & SCU_D1PNS2_USBD_Msk)
#define USBD         USBD_NS
#else
#define USBD         USBD_S
#endif

/* SCU_D1PNS4 */
#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_WWDT1_Msk)
#define WWDT1        WWDT1_NS
#else
#define WWDT1        WWDT1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_EPWM1_Msk)
#define EPWM1        EPWM1_NS
#else
#define EPWM1        EPWM1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_BPWM1_Msk)
#define BPWM1        BPWM1_NS
#else
#define BPWM1        BPWM1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_EQEI1_Msk)
#define EQEI1        EQEI1_NS
#else
#define EQEI1        EQEI1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_EQEI3_Msk)
#define EQEI3        EQEI3_NS
#else
#define EQEI3        EQEI3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_ECAP1_Msk)
#define ECAP1        ECAP1_NS
#else
#define ECAP1        ECAP1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_ECAP3_Msk)
#define ECAP3        ECAP3_NS
#else
#define ECAP3        ECAP3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_I2C1_Msk)
#define I2C1         I2C1_NS
#else
#define I2C1         I2C1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_I2C3_Msk)
#define I2C3         I2C3_NS
#else
#define I2C3         I2C3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_QSPI1_Msk)
#define QSPI1        QSPI1_NS
#else
#define QSPI1        QSPI1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_SPI1_Msk)
#define SPI1         SPI1_NS
#else
#define SPI1         SPI1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_SPI3_Msk)
#define SPI3         SPI3_NS
#else
#define SPI3         SPI3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UART1_Msk)
#define UART1        UART1_NS
#else
#define UART1        UART1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UART3_Msk)
#define UART3        UART3_NS
#else
#define UART3        UART3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UART5_Msk)
#define UART5        UART5_NS
#else
#define UART5        UART5_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UART7_Msk)
#define UART7        UART7_NS
#else
#define UART7        UART7_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UART9_Msk)
#define UART9        UART9_NS
#else
#define UART9        UART9_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_SC1_Msk)
#define SC1          SC1_NS
#else
#define SC1          SC1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_OTG_Msk)
#define OTG          OTG_NS
#else
#define OTG          OTG_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_KPI_Msk)
#define KPI          KPI_NS
#else
#define KPI          KPI_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_TMR23_Msk)
#define TIMER2       TIMER2_NS
#define TIMER3       TIMER3_NS
#else
#define TIMER2       TIMER2_S
#define TIMER3       TIMER3_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_TRNG_Msk)
#define TRNG         TRNG_NS
#else
#define TRNG         TRNG_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_RTC_Msk)
#define RTC          RTC_NS
#else
#define RTC          RTC_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_I2S1_Msk)
#define I2S1         I2S1_NS
#else
#define I2S1         I2S1_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_ACMP23_Msk)
#define ACMP23       ACMP23_NS
#else
#define ACMP23       ACMP23_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_I3C0_Msk)
#define I3C0         I3C0_NS
#else
#define I3C0         I3C0_S
#endif

#if defined (SCU_INIT_D1PNS4_VAL) && (SCU_INIT_D1PNS4_VAL & SCU_D1PNS4_UTCPD_Msk)
#define UTCPD        UTCPD_NS
#else
#define UTCPD        UTCPD_S
#endif

/* SCU_D2PNS0 */
#if defined (SCU_INIT_D2PNS0_VAL) && (SCU_INIT_D2PNS0_VAL & SCU_D2PNS0_LPPDMA_Msk)
#define LPPDMA       LPPDMA_NS
#else
#define LPPDMA       LPPDMA_S
#endif

#if defined (SCU_INIT_D2PNS0_VAL) && (SCU_INIT_D2PNS0_VAL & SCU_D2PNS0_CCAP_Msk)
#define CCAP         CCAP_NS
#else
#define CCAP         CCAP_S
#endif

#if defined (SCU_INIT_D2PNS0_VAL) && (SCU_INIT_D2PNS0_VAL & SCU_D2PNS0_LPGPIO_Msk)
#define LPGPIO       LPGPIO_NS
#else
#define LPGPIO       LPGPIO_S
#endif

/* SCU_D2PNS2 */
#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_LPTMR01_Msk)
#define LPTMR0       LPTMR0_NS
#define LPTMR1       LPTMR1_NS
#else
#define LPTMR0       LPTMR0_S
#define LPTMR1       LPTMR1_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_TTMR01_Msk)
#define TTMR0        TTMR0_NS
#define TTMR1        TTMR1_NS
#else
#define TTMR0        TTMR0_S
#define TTMR1        TTMR1_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_LPADC0_Msk)
#define LPADC0       LPADC0_NS
#else
#define LPADC0       LPADC0_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_LPI2C0_Msk)
#define LPI2C0       LPI2C0_NS
#else
#define LPI2C0       LPI2C0_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_LPSPI0_Msk)
#define LPSPI0       LPSPI0_NS
#else
#define LPSPI0       LPSPI0_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_DMIC0_Msk)
#define DMIC0        DMIC0_NS
#define VAD0         VAD0_NS
#else
#define DMIC0        DMIC0_S
#define VAD0         VAD0_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_LPUART0_Msk)
#define LPUART0      LPUART0_NS
#else
#define LPUART0      LPUART0_S
#endif

#if defined (SCU_INIT_D2PNS2_VAL) && (SCU_INIT_D2PNS2_VAL & SCU_D2PNS2_AWF_Msk)
#define AWF          AWF_NS
#else
#define AWF          AWF_S
#endif

/** @} end of group PERIPHERAL_SNS */
/** @} end of group PERIPHERAL_DECLARATION */


/* --------------------  End of section using anonymous unions  ------------------- */
#if   defined (__CC_ARM)
#pragma pop
#elif defined (__ICCARM__)
/* leave anonymous unions enabled */
#elif (__AFMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning restore
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif


/*=============================================================================*/

/** @addtogroup IO_ROUTINE I/O Routines
    The Declaration of I/O Routines
    @{
 */

typedef volatile uint8_t  vu8;    ///< Define  8-bit unsigned volatile data type
typedef volatile uint16_t vu16;   ///< Define 16-bit unsigned volatile data type
typedef volatile uint32_t vu32;   ///< Define 32-bit unsigned volatile data type
typedef volatile uint64_t vu64;   ///< Define 64-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)            (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr)           (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr)           (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)   (*((volatile unsigned int *)(port))=(value))

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)          ((*((volatile unsigned int *)(port))))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)   (*((volatile unsigned short *)(port))=(value))

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)          ((*((volatile unsigned short *)(port))))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)   (*((volatile unsigned char *)(port))=(value))

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)          ((*((volatile unsigned char *)(port))))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)  (*((volatile unsigned int *)(port))=(value))

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)         ((*((volatile unsigned int *)(port))))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)  (*((volatile unsigned short *)(port))=(value))

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)         ((*((volatile unsigned short *)(port))))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)   (*((volatile unsigned char *)(port))=(value))

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)          ((*((volatile unsigned char *)(port))))

/** @} end of group IO_ROUTINE */


/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/

/** @addtogroup Legacy_Constants Legacy Constants
    Legacy Constants
    @{
*/

#define E_SUCCESS           (0)

#ifndef NULL
#define NULL                (0)                 ///< NULL pointer
#endif

#define TRUE                (1UL)               ///< Boolean true, define to use in API parameters or return value
#define FALSE               (0UL)               ///< Boolean false, define to use in API parameters or return value

#define ENABLE              (1UL)               ///< Enable, define to use in API parameters
#define DISABLE             (0UL)               ///< Disable, define to use in API parameters

/* Define one bit mask */
#define BIT0                (0x00000001UL)      ///< Bit 0 mask of an 32 bit integer
#define BIT1                (0x00000002UL)      ///< Bit 1 mask of an 32 bit integer
#define BIT2                (0x00000004UL)      ///< Bit 2 mask of an 32 bit integer
#define BIT3                (0x00000008UL)      ///< Bit 3 mask of an 32 bit integer
#define BIT4                (0x00000010UL)      ///< Bit 4 mask of an 32 bit integer
#define BIT5                (0x00000020UL)      ///< Bit 5 mask of an 32 bit integer
#define BIT6                (0x00000040UL)      ///< Bit 6 mask of an 32 bit integer
#define BIT7                (0x00000080UL)      ///< Bit 7 mask of an 32 bit integer
#define BIT8                (0x00000100UL)      ///< Bit 8 mask of an 32 bit integer
#define BIT9                (0x00000200UL)      ///< Bit 9 mask of an 32 bit integer
#define BIT10               (0x00000400UL)      ///< Bit 10 mask of an 32 bit integer
#define BIT11               (0x00000800UL)      ///< Bit 11 mask of an 32 bit integer
#define BIT12               (0x00001000UL)      ///< Bit 12 mask of an 32 bit integer
#define BIT13               (0x00002000UL)      ///< Bit 13 mask of an 32 bit integer
#define BIT14               (0x00004000UL)      ///< Bit 14 mask of an 32 bit integer
#define BIT15               (0x00008000UL)      ///< Bit 15 mask of an 32 bit integer
#define BIT16               (0x00010000UL)      ///< Bit 16 mask of an 32 bit integer
#define BIT17               (0x00020000UL)      ///< Bit 17 mask of an 32 bit integer
#define BIT18               (0x00040000UL)      ///< Bit 18 mask of an 32 bit integer
#define BIT19               (0x00080000UL)      ///< Bit 19 mask of an 32 bit integer
#define BIT20               (0x00100000UL)      ///< Bit 20 mask of an 32 bit integer
#define BIT21               (0x00200000UL)      ///< Bit 21 mask of an 32 bit integer
#define BIT22               (0x00400000UL)      ///< Bit 22 mask of an 32 bit integer
#define BIT23               (0x00800000UL)      ///< Bit 23 mask of an 32 bit integer
#define BIT24               (0x01000000UL)      ///< Bit 24 mask of an 32 bit integer
#define BIT25               (0x02000000UL)      ///< Bit 25 mask of an 32 bit integer
#define BIT26               (0x04000000UL)      ///< Bit 26 mask of an 32 bit integer
#define BIT27               (0x08000000UL)      ///< Bit 27 mask of an 32 bit integer
#define BIT28               (0x10000000UL)      ///< Bit 28 mask of an 32 bit integer
#define BIT29               (0x20000000UL)      ///< Bit 29 mask of an 32 bit integer
#define BIT30               (0x40000000UL)      ///< Bit 30 mask of an 32 bit integer
#define BIT31               (0x80000000UL)      ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk           (0x000000FFUL)      ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk           (0x0000FF00UL)      ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk           (0x00FF0000UL)      ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk           (0xFF000000UL)      ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param) (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param) (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param) (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param) (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/** @} end of group Legacy_Constants */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/

#include "acmp.h"
#include "awf.h"
#include "bpwm.h"
#include "canfd.h"
#include "ccap.h"
#include "clk.h"
#include "crc.h"
#include "dac.h"
#include "dmic.h"
#include "eadc.h"
#include "ebi.h"
#include "ecap.h"
#include "epwm.h"
#include "eqei.h"
#include "fmc.h"
#include "gdma/dma350_lib.h"
#include "gdma/dma350_drv.h"
#include "gpio.h"
#include "hsotg.h"
#include "hsusbd.h"
#include "i2c.h"
#include "i2s.h"
#include "i3c.h"
#include "ks.h"
#include "kdf.h"
#include "crypto.h"
#include "kpi.h"
#include "lpadc.h"
#include "lpgpio.h"
#include "lpi2c.h"
#include "lppdma.h"
#include "lpspi.h"
#include "lptmr.h"
#include "lptmr_pwm.h"
#include "lpuart.h"
#include "otfc.h"
#include "otg.h"
#include "pdma.h"
#include "pmc.h"
#include "psio.h"
#include "qspi.h"
#include "rng.h"
#include "rtc.h"
#include "sc.h"
#include "scu/scu.h"
#include "scu/dpm.h"
#include "scu/plm.h"
#include "scu/mpc_sie_reg_map.h"
#include "scu/mpc_sie_drv.h"
#include "scuart.h"
#include "sdh.h"
#include "spi.h"
#include "spim.h"
#include "spim_hyper.h"
#include "sys.h"
#include "timer.h"
#include "timer_pwm.h"
#include "trng.h"
#include "ttmr.h"
#include "uart.h"
//#include "usbd.h"
#include "usbd_User.h"
#include "usci_i2c.h"
#include "usci_spi.h"
#include "usci_uart.h"
#include "wdt.h"
#include "wwdt.h"


#ifdef __cplusplus
}
#endif

#endif  /* __M55M1_H__ */
