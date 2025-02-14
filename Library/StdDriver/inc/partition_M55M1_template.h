/**************************************************************************//**
 * @file     partition_M55M1_template.h
 * @version  V1.00
 * @brief    Default Initial Setup for Secure / Non-secure Attribution Zones for M55M1
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef PARTITION_M55M1_TEMPLATE_H
#define PARTITION_M55M1_TEMPLATE_H

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
//    <o> Secure SRAM Size <0x8000-0x150000:0x8000>
*/
#define SCU_SECURE_SRAM_SIZE        0x100000
#define NON_SECURE_SRAM_BASE        (0x20100000 + 0x10000000 + SCU_SECURE_SRAM_SIZE)

/*
// <h> Non-secure Base Address (NSCBA)
*/
#define FMC_INIT_NSCBA              1
/*
//    <o> Secure APROM Flash Size <0x2000-0x200000:0x2000>
*/

#define FMC_SECURE_REGION_SIZE      0x80000
#define FMC_SECURE_END              (FMC_APROM_BASE + FMC_SECURE_REGION_SIZE)
#define FMC_NON_SECURE_BASE         (FMC_SECURE_END + NS_OFFSET)

/*
// <q>Enable Mirror Boundary (Bank Remap Function)
*/
#define FMC_INIT_MIRROR_BOUND       0x0
/*
// </h>
*/

/*
//    <o> Secure SPI Flash Size <0x0-0x2000000:0x20000>
//    <i> 0x0: Whole SPI Flash is secure
*/
#define SCU_SECURE_SPIFLASH_SIZE    0x0

/*----------------------------------------------------------------------------*/

/*
// <h> Peripheral Secure Attribution
*/

/*
// <h> Domain 0 Peripheral Secure Attribution
*/

//   <o0.03>  NPU            <0=> Secure <1=> Non-secure
//   <o1.02>  SPIM0          <0=> Secure <1=> Non-secure

#define SCU_INIT_D0PNS0_VAL         0x0
#define SCU_INIT_D0PNS2_VAL         0x0

/*
// </h>
*/

/*
// <h> Domain 1 Peripheral Secure Attribution
*/
//   <o0.00>  PDMA0          <0=> Secure <1=> Non-secure
//   <o0.01>  PDMA1          <0=> Secure <1=> Non-secure
//   <o0.02>  USBH0          <0=> Secure <1=> Non-secure
//   <o0.04>  HSUSBH         <0=> Secure <1=> Non-secure
//   <o0.05>  HSUSBD         <0=> Secure <1=> Non-secure
//   <o0.06>  SDH0           <0=> Secure <1=> Non-secure
//   <o0.07>  SDH1           <0=> Secure <1=> Non-secure
//   <o0.08>  EMAC0          <0=> Secure <1=> Non-secure
//   <o0.10>  CRYPTO         <0=> Secure <1=> Non-secure
//   <o0.11>  CRC            <0=> Secure <1=> Non-secure
//   <o1.01>  KDF            <0=> Secure <1=> Non-secure
//   <o1.02>  CANFD0         <0=> Secure <1=> Non-secure
//   <o1.04>  CANFD1         <0=> Secure <1=> Non-secure
//   <o1.16>  EBI            <0=> Secure <1=> Non-secure
//   <o2.00>  WWDT0          <0=> Secure <1=> Non-secure
//   <o3.00>  WWDT1          <0=> Secure <1=> Non-secure
//   <o2.01>  EADC0          <0=> Secure <1=> Non-secure
//   <o2.02>  EPWM0          <0=> Secure <1=> Non-secure
//   <o3.02>  EPWM1          <0=> Secure <1=> Non-secure
//   <o2.03>  BPWM0          <0=> Secure <1=> Non-secure
//   <o3.03>  BPWM1          <0=> Secure <1=> Non-secure
//   <h> EQEI
//   <o2.04>  EQEI0          <0=> Secure <1=> Non-secure
//   <o3.04>  EQEI1          <0=> Secure <1=> Non-secure
//   <o2.05>  EQEI2          <0=> Secure <1=> Non-secure
//   <o3.05>  EQEI3          <0=> Secure <1=> Non-secure
//   </h>
//   <h> ECAP
//   <o2.06>  ECAP0          <0=> Secure <1=> Non-secure
//   <o3.06>  ECAP1          <0=> Secure <1=> Non-secure
//   <o2.07>  ECAP2          <0=> Secure <1=> Non-secure
//   <o3.07>  ECAP3          <0=> Secure <1=> Non-secure
//   </h>
//   <h> I2C
//   <o2.08>  I2C0           <0=> Secure <1=> Non-secure
//   <o3.08>  I2C1           <0=> Secure <1=> Non-secure
//   <o2.09>  I2C2           <0=> Secure <1=> Non-secure
//   <o3.09>  I2C3           <0=> Secure <1=> Non-secure
//   </h>
//   <o2.10>  QSPI0          <0=> Secure <1=> Non-secure
//   <o3.10>  QSPI1          <0=> Secure <1=> Non-secure
//   <h> SPI
//   <o2.11>  SPI0           <0=> Secure <1=> Non-secure
//   <o3.11>  SPI1           <0=> Secure <1=> Non-secure
//   <o2.12>  SPI2           <0=> Secure <1=> Non-secure
//   <o3.12>  SPI3           <0=> Secure <1=> Non-secure
//   </h>
//   <h> UART
//   <o2.13>  UART0          <0=> Secure <1=> Non-secure
//   <o3.13>  UART1          <0=> Secure <1=> Non-secure
//   <o2.14>  UART2          <0=> Secure <1=> Non-secure
//   <o3.14>  UART3          <0=> Secure <1=> Non-secure
//   <o2.15>  UART4          <0=> Secure <1=> Non-secure
//   <o3.15>  UART5          <0=> Secure <1=> Non-secure
//   <o2.16>  UART6          <0=> Secure <1=> Non-secure
//   <o3.16>  UART7          <0=> Secure <1=> Non-secure
//   <o2.17>  UART8          <0=> Secure <1=> Non-secure
//   <o3.17>  UART9          <0=> Secure <1=> Non-secure
//   </h>
//   <o2.18>  USCI0          <0=> Secure <1=> Non-secure
//   <o2.19>  SC0            <0=> Secure <1=> Non-secure
//   <o3.18>  SC1            <0=> Secure <1=> Non-secure
//   <o2.20>  SC2            <0=> Secure <1=> Non-secure
//   <o2.21>  PSIO           <0=> Secure <1=> Non-secure
//   <o2.22>  TMR01          <0=> Secure <1=> Non-secure
//   <o3.21>  TMR23          <0=> Secure <1=> Non-secure
//   <o2.23>  DAC01          <0=> Secure <1=> Non-secure
//   <o2.25>  HSOTG          <0=> Secure <1=> Non-secure
//   <o2.26>  I2S0           <0=> Secure <1=> Non-secure
//   <o3.24>  I2S1           <0=> Secure <1=> Non-secure
//   <o2.27>  ACMP01         <0=> Secure <1=> Non-secure
//   <o3.25>  ACMP23         <0=> Secure <1=> Non-secure
//   <o2.28>  USBD           <0=> Secure <1=> Non-secure
//   <o3.19>  OTG            <0=> Secure <1=> Non-secure
//   <o3.20>  KPI            <0=> Secure <1=> Non-secure
//   <o3.22>  TRNG           <0=> Secure <1=> Non-secure
//   <o3.23>  RTC            <0=> Secure <1=> Non-secure
//   <o3.26>  I3C0           <0=> Secure <1=> Non-secure
//   <o3.27>  UTCPD          <0=> Secure <1=> Non-secure

#define SCU_INIT_D1PNS0_VAL         0x0
#define SCU_INIT_D1PNS1_VAL         0x0
#define SCU_INIT_D1PNS2_VAL         0x0
#define SCU_INIT_D1PNS4_VAL         0x0

/*
// </h>
*/

/*
// <h> Domain 2 (Low Power) Peripheral Secure Attribution
*/

//   <o0.00>  LPDMA          <0=> Secure <1=> Non-secure
//   <o0.01>  CCAP           <0=> Secure <1=> Non-secure
//   <o0.03>  LPGPIO         <0=> Secure <1=> Non-secure
//   <o1.00>  LPTMR01        <0=> Secure <1=> Non-secure
//   <o1.01>  TTMR01         <0=> Secure <1=> Non-secure
//   <o1.02>  LPADC0         <0=> Secure <1=> Non-secure
//   <o1.03>  LPI2C0         <0=> Secure <1=> Non-secure
//   <o1.04>  LPSPI0         <0=> Secure <1=> Non-secure
//   <o1.05>  DMIC0          <0=> Secure <1=> Non-secure
//   <o1.06>  LPUART0        <0=> Secure <1=> Non-secure
//   <o1.09>  AWF            <0=> Secure <1=> Non-secure
#define SCU_INIT_D2PNS0_VAL         0x0
#define SCU_INIT_D2PNS2_VAL         0x0

/*
// </h>
*/

/*
// </h>
*/

/*
// <h> GPIO Pin Secure Attribution
*/

/*
    IONSx
*/

/*
// Bit 0..31
// <h> PA
//   <o.0 >  PA0       <0=> Secure <1=> Non-secure
//   <o.1 >  PA1       <0=> Secure <1=> Non-secure
//   <o.2 >  PA2       <0=> Secure <1=> Non-secure
//   <o.3 >  PA3       <0=> Secure <1=> Non-secure
//   <o.4 >  PA4       <0=> Secure <1=> Non-secure
//   <o.5 >  PA5       <0=> Secure <1=> Non-secure
//   <o.6 >  PA6       <0=> Secure <1=> Non-secure
//   <o.7 >  PA7       <0=> Secure <1=> Non-secure
//   <o.8 >  PA8       <0=> Secure <1=> Non-secure
//   <o.9 >  PA9       <0=> Secure <1=> Non-secure
//   <o.10>  PA10      <0=> Secure <1=> Non-secure
//   <o.11>  PA11      <0=> Secure <1=> Non-secure
//   <o.12>  PA12      <0=> Secure <1=> Non-secure
//   <o.13>  PA13      <0=> Secure <1=> Non-secure
//   <o.14>  PA14      <0=> Secure <1=> Non-secure
//   <o.15>  PA15      <0=> Secure <1=> Non-secure
// </h>

*/
#define SCU_INIT_IONS0_VAL          0x00000000

/*
// Bit 0..31
// <h> PB
//   <o.0 >  PB0       <0=> Secure <1=> Non-secure
//   <o.1 >  PB1       <0=> Secure <1=> Non-secure
//   <o.2 >  PB2       <0=> Secure <1=> Non-secure
//   <o.3 >  PB3       <0=> Secure <1=> Non-secure
//   <o.4 >  PB4       <0=> Secure <1=> Non-secure
//   <o.5 >  PB5       <0=> Secure <1=> Non-secure
//   <o.6 >  PB6       <0=> Secure <1=> Non-secure
//   <o.7 >  PB7       <0=> Secure <1=> Non-secure
//   <o.8 >  PB8       <0=> Secure <1=> Non-secure
//   <o.9 >  PB9       <0=> Secure <1=> Non-secure
//   <o.10>  PB10      <0=> Secure <1=> Non-secure
//   <o.11>  PB11      <0=> Secure <1=> Non-secure
//   <o.12>  PB12      <0=> Secure <1=> Non-secure
//   <o.13>  PB13      <0=> Secure <1=> Non-secure
//   <o.14>  PB14      <0=> Secure <1=> Non-secure
//   <o.15>  PB15      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS1_VAL          0x00000000


/*
// Bit 0..31
// <h> PC
//   <o.0 >  PC0       <0=> Secure <1=> Non-secure
//   <o.1 >  PC1       <0=> Secure <1=> Non-secure
//   <o.2 >  PC2       <0=> Secure <1=> Non-secure
//   <o.3 >  PC3       <0=> Secure <1=> Non-secure
//   <o.4 >  PC4       <0=> Secure <1=> Non-secure
//   <o.5 >  PC5       <0=> Secure <1=> Non-secure
//   <o.6 >  PC6       <0=> Secure <1=> Non-secure
//   <o.7 >  PC7       <0=> Secure <1=> Non-secure
//   <o.8 >  PC8       <0=> Secure <1=> Non-secure
//   <o.9 >  PC9       <0=> Secure <1=> Non-secure
//   <o.10>  PC10      <0=> Secure <1=> Non-secure
//   <o.11>  PC11      <0=> Secure <1=> Non-secure
//   <o.12>  PC12      <0=> Secure <1=> Non-secure
//   <o.13>  PC13      <0=> Secure <1=> Non-secure
//   <o.14>  PC14      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS2_VAL          0x00000000

/*
// Bit 0..31
// <h> PD
//   <o.0 >  PD0       <0=> Secure <1=> Non-secure
//   <o.1 >  PD1       <0=> Secure <1=> Non-secure
//   <o.2 >  PD2       <0=> Secure <1=> Non-secure
//   <o.3 >  PD3       <0=> Secure <1=> Non-secure
//   <o.4 >  PD4       <0=> Secure <1=> Non-secure
//   <o.5 >  PD5       <0=> Secure <1=> Non-secure
//   <o.6 >  PD6       <0=> Secure <1=> Non-secure
//   <o.7 >  PD7       <0=> Secure <1=> Non-secure
//   <o.8 >  PD8       <0=> Secure <1=> Non-secure
//   <o.9 >  PD9       <0=> Secure <1=> Non-secure
//   <o.10>  PD10      <0=> Secure <1=> Non-secure
//   <o.11>  PD11      <0=> Secure <1=> Non-secure
//   <o.12>  PD12      <0=> Secure <1=> Non-secure
//   <o.13>  PD13      <0=> Secure <1=> Non-secure
//   <o.14>  PD14      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS3_VAL          0x00000000


/*
// Bit 0..31
// <h> PE
//   <o.0 >  PE0       <0=> Secure <1=> Non-secure
//   <o.1 >  PE1       <0=> Secure <1=> Non-secure
//   <o.2 >  PE2       <0=> Secure <1=> Non-secure
//   <o.3 >  PE3       <0=> Secure <1=> Non-secure
//   <o.4 >  PE4       <0=> Secure <1=> Non-secure
//   <o.5 >  PE5       <0=> Secure <1=> Non-secure
//   <o.6 >  PE6       <0=> Secure <1=> Non-secure
//   <o.7 >  PE7       <0=> Secure <1=> Non-secure
//   <o.8 >  PE8       <0=> Secure <1=> Non-secure
//   <o.9 >  PE9       <0=> Secure <1=> Non-secure
//   <o.10>  PE10      <0=> Secure <1=> Non-secure
//   <o.11>  PE11      <0=> Secure <1=> Non-secure
//   <o.12>  PE12      <0=> Secure <1=> Non-secure
//   <o.13>  PE13      <0=> Secure <1=> Non-secure
//   <o.14>  PE14      <0=> Secure <1=> Non-secure
//   <o.15>  PE15      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS4_VAL          0x00000000

/*
// Bit 0..31
// <h> PF
//   <o.0 >  PF0       <0=> Secure <1=> Non-secure
//   <o.1 >  PF1       <0=> Secure <1=> Non-secure
//   <o.2 >  PF2       <0=> Secure <1=> Non-secure
//   <o.3 >  PF3       <0=> Secure <1=> Non-secure
//   <o.4 >  PF4       <0=> Secure <1=> Non-secure
//   <o.5 >  PF5       <0=> Secure <1=> Non-secure
//   <o.6 >  PF6       <0=> Secure <1=> Non-secure
//   <o.7 >  PF7       <0=> Secure <1=> Non-secure
//   <o.8 >  PF8       <0=> Secure <1=> Non-secure
//   <o.9 >  PF9       <0=> Secure <1=> Non-secure
//   <o.10>  PF10      <0=> Secure <1=> Non-secure
//   <o.11>  PF11      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS5_VAL      0x00000000


/*
// Bit 0..31
// <h> PG
//   <o.0 >  PG0       <0=> Secure <1=> Non-secure
//   <o.1 >  PG1       <0=> Secure <1=> Non-secure
//   <o.2 >  PG2       <0=> Secure <1=> Non-secure
//   <o.3 >  PG3       <0=> Secure <1=> Non-secure
//   <o.4 >  PG4       <0=> Secure <1=> Non-secure
//   <o.5 >  PG5       <0=> Secure <1=> Non-secure
//   <o.6 >  PG6       <0=> Secure <1=> Non-secure
//   <o.7 >  PG7       <0=> Secure <1=> Non-secure
//   <o.8 >  PG8       <0=> Secure <1=> Non-secure
//   <o.9 >  PG9       <0=> Secure <1=> Non-secure
//   <o.10>  PG10      <0=> Secure <1=> Non-secure
//   <o.11>  PG11      <0=> Secure <1=> Non-secure
//   <o.12>  PG12      <0=> Secure <1=> Non-secure
//   <o.13>  PG13      <0=> Secure <1=> Non-secure
//   <o.14>  PG14      <0=> Secure <1=> Non-secure
//   <o.15>  PG15      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS6_VAL      0x00000000

/*
// Bit 0..31
// <h> PH
//   <o.0 >  PH0       <0=> Secure <1=> Non-secure
//   <o.1 >  PH1       <0=> Secure <1=> Non-secure
//   <o.2 >  PH2       <0=> Secure <1=> Non-secure
//   <o.3 >  PH3       <0=> Secure <1=> Non-secure
//   <o.4 >  PH4       <0=> Secure <1=> Non-secure
//   <o.5 >  PH5       <0=> Secure <1=> Non-secure
//   <o.6 >  PH6       <0=> Secure <1=> Non-secure
//   <o.7 >  PH7       <0=> Secure <1=> Non-secure
//   <o.8 >  PH8       <0=> Secure <1=> Non-secure
//   <o.9 >  PH9       <0=> Secure <1=> Non-secure
//   <o.10>  PH10      <0=> Secure <1=> Non-secure
//   <o.11>  PH11      <0=> Secure <1=> Non-secure
//   <o.12>  PH12      <0=> Secure <1=> Non-secure
//   <o.13>  PH13      <0=> Secure <1=> Non-secure
//   <o.14>  PH14      <0=> Secure <1=> Non-secure
//   <o.15>  PH15      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS7_VAL      0x00000000

/*
// Bit 0..31
// <h> PI
//   <o.6 >  PI6       <0=> Secure <1=> Non-secure
//   <o.7 >  PI7       <0=> Secure <1=> Non-secure
//   <o.8 >  PI8       <0=> Secure <1=> Non-secure
//   <o.9 >  PI9       <0=> Secure <1=> Non-secure
//   <o.10>  PI10      <0=> Secure <1=> Non-secure
//   <o.11>  PI11      <0=> Secure <1=> Non-secure
//   <o.12>  PI12      <0=> Secure <1=> Non-secure
//   <o.13>  PI13      <0=> Secure <1=> Non-secure
//   <o.14>  PI14      <0=> Secure <1=> Non-secure
//   <o.15>  PI15      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS8_VAL      0x00000000

/*
// Bit 0..31
// <h> PJ
//   <o.0 >  PJ0       <0=> Secure <1=> Non-secure
//   <o.1 >  PJ1       <0=> Secure <1=> Non-secure
//   <o.2 >  PJ2       <0=> Secure <1=> Non-secure
//   <o.3 >  PJ3       <0=> Secure <1=> Non-secure
//   <o.4 >  PJ4       <0=> Secure <1=> Non-secure
//   <o.5 >  PJ5       <0=> Secure <1=> Non-secure
//   <o.6 >  PJ6       <0=> Secure <1=> Non-secure
//   <o.7 >  PJ7       <0=> Secure <1=> Non-secure
//   <o.8 >  PJ8       <0=> Secure <1=> Non-secure
//   <o.9 >  PJ9       <0=> Secure <1=> Non-secure
//   <o.10>  PJ10      <0=> Secure <1=> Non-secure
//   <o.11>  PJ11      <0=> Secure <1=> Non-secure
//   <o.12>  PJ12      <0=> Secure <1=> Non-secure
//   <o.13>  PJ13      <0=> Secure <1=> Non-secure
// </h>
*/
#define SCU_INIT_IONS9_VAL      0x00000000

/*
// </h>
*/

/*
// <h> GPIO and EINT Interrupt Secure Attribution
*/

/*
//    Initialize GPIO/EINT ITNS (Interrupts 0..31)
*/

/*
// Bit 0..31
//   <o.0 >  GPA        <0=> Secure <1=> Non-secure
//   <o.1 >  GPB        <0=> Secure <1=> Non-secure
//   <o.2 >  GPC        <0=> Secure <1=> Non-secure
//   <o.3 >  GPD        <0=> Secure <1=> Non-secure
//   <o.4 >  GPE        <0=> Secure <1=> Non-secure
//   <o.5 >  GPF        <0=> Secure <1=> Non-secure
//   <o.6 >  GPG        <0=> Secure <1=> Non-secure
//   <o.7 >  GPH        <0=> Secure <1=> Non-secure
//   <o.8 >  GPI        <0=> Secure <1=> Non-secure
//   <o.9 >  GPJ        <0=> Secure <1=> Non-secure
//   <o.10>  EINT0      <0=> Secure <1=> Non-secure
//   <o.11>  EINT1      <0=> Secure <1=> Non-secure
//   <o.12>  EINT2      <0=> Secure <1=> Non-secure
//   <o.13>  EINT3      <0=> Secure <1=> Non-secure
//   <o.14>  EINT4      <0=> Secure <1=> Non-secure
//   <o.15>  EINT5      <0=> Secure <1=> Non-secure
//   <o.16>  EINT6      <0=> Secure <1=> Non-secure
//   <o.17>  EINT7      <0=> Secure <1=> Non-secure
*/
#define SCU_INIT_IO_ITNS_VAL      0x0000
/*
// </h>
*/

/* ---------------------------------------------------------------------------------------------------- */

/*
// <e>Initialize Security Attribution Unit (SAU) CTRL register
*/
#define SAU_INIT_CTRL 1

/*
//   <q> Enable SAU
//   <i> To enable Secure Attribute Unit (SAU).
*/
#define SAU_INIT_CTRL_ENABLE 1

/*
//   <o> All Memory Attribute When SAU is disabled
//     <0=> All Memory is Secure
//     <1=> All Memory is Non-secure
//   <i> To set the ALLNS bit in SAU CTRL.
//   <i> When all Memory is Non-secure (ALLNS is 1), IDAU can override memory map configuration.
*/
#define SAU_INIT_CTRL_ALLNS  0

/*
// </e>
*/


/*
// <h>Initialize Security Attribution Unit (SAU) Address Regions
// <i>SAU configuration specifies regions to be one of:
// <i> - Secure and Non-secure Callable
// <i> - Non-secure
// <i>Note: All memory regions not configured by SAU are Secure
*/
#define SAU_REGIONS_MAX   8                 /* Max. number of SAU regions */

/*
//   <e>Initialize SAU Region 0
//   <i> Setup SAU Region 0 memory attributes
*/
#define SAU_INIT_REGION0    0
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START0     0x0003F000      /* start address of SAU region 0 */
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END0       0x0003FFFF      /* end address of SAU region 0 */
/*
//     <o>Region is
//         <0=>Non-secure
//         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC0       1
/*
//   </e>
*/

/*
//   <e>Initialize SAU Region 1
//   <i> Setup SAU Region 1 memory attributes
*/
#define SAU_INIT_REGION1    0
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START1     0x10040000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END1       0x1007FFFF
/*
//     <o>Region is
//         <0=>Non-secure
//         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC1       0
/*
//   </e>
*/

/*
//   <e>Initialize SAU Region 2
//   <i> Setup SAU Region 2 memory attributes
*/
#define SAU_INIT_REGION2    0
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START2     0x2000F000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END2       0x2000FFFF
/*
//     <o>Region is
//         <0=>Non-secure
//         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC2       1
/*
//   </e>
*/

/*
//   <e>Initialize SAU Region 3
//   <i> Setup SAU Region 3 memory attributes
*/
#define SAU_INIT_REGION3    0
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START3     0x13F000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END3       0x13FFFF
/*
//     <o>Region is
//         <0=>Non-secure
//         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC3       1
/*
//   </e>
*/

/*
   <e>Initialize SAU Region 4 (Config Non-secure Callable in system_M55M1.c)
   <i> Setup SAU Region 4 memory attributes
*/
#define SAU_INIT_REGION4    0
/*
     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START4     0      /* start address of SAU region 4 */

/*
     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END4       0      /* end address of SAU region 4 */

/*
     <o>Region is
         <0=>Non-secure
         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC4       1
/*
   </e>
*/

/*
   <e>Initialize SAU Region 5 (Non-secure APROM)
   <i> Setup SAU Region 5 memory attributes
*/
#define SAU_INIT_REGION5    1
/*
     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START5     FMC_NON_SECURE_BASE     /* start address of SAU region 5 */

/*
     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END5       0x102FFFFF              /* end address of SAU region 5 */

/*
     <o>Region is
         <0=>Non-secure
         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC5       0
/*
   </e>
*/

/*
   <e>Initialize SAU Region 6 (Non-secure SRAM)
   <i> Setup SAU Region 6 memory attributes
*/
#define SAU_INIT_REGION6    1

/*
     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START6     NON_SECURE_SRAM_BASE    /* start address of SAU region 6 */

/*
     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END6       0x3024FFFF              /* end address of SAU region 6 */

/*
     <o>Region is
         <0=>Non-secure
         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC6       0
/*
   </e>
*/

/*
   <e>Initialize SAU Region 7 (Non-secure IP region)
   <i> Setup SAU Region 7 memory attributes
*/
#define SAU_INIT_REGION7    1

/*
     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START7     0x50000000      /* start address of SAU region 7 */

/*
     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END7       0x5FFFFFFF      /* end address of SAU region 7 */

/*
     <o>Region is
         <0=>Non-secure
         <1=>Secure, Non-secure Callable
*/
#define SAU_INIT_NSC7       0
/*
   </e>
*/

/*
// </h>
*/

/*
// <e>Setup behavior of Sleep and Exception Handling
*/
#define SCB_CSR_AIRCR_INIT  1

/*
//   <o> Deep Sleep can be enabled by
//     <0=>Secure and Non-secure state
//     <1=>Secure state only
//   <i> Value for SCB->CSR register bit DEEPSLEEPS
*/
#define SCB_CSR_DEEPSLEEPS_VAL  0

/*
//   <o>System reset request accessible from
//     <0=> Secure and Non-secure state
//     <1=> Secure state only
//   <i> Value for SCB->AIRCR register bit SYSRESETREQS
*/
#define SCB_AIRCR_SYSRESETREQS_VAL  0

/*
//   <o>Priority of Non-secure exceptions is
//     <0=> Not altered
//     <1=> Lowered to 0x80-0xFF
//   <i> Value for SCB->AIRCR register bit PRIS
*/
#define SCB_AIRCR_PRIS_VAL      0

/* Assign HardFault to be always Secure for safe */
#define SCB_AIRCR_BFHFNMINS_VAL 0

/*
// </e>
*/

/*
// <e>Setup behaviour of Floating Point and Vector Unit (FPU/MVE)
*/
#define TZ_FPU_NS_USAGE 1

/*
// <o>Floating Point and Vector Unit usage
//     <0=> Secure state only
//     <3=> Secure and Non-secure state
//   <i> Value for SCB->NSACR register bits CP10, CP11
*/
#define SCB_NSACR_CP10_11_VAL       3

/*
// <o>Treat floating-point registers as Secure
//     <0=> Disabled
//     <1=> Enabled
//   <i> Value for FPU->FPCCR register bit TS
*/
#define FPU_FPCCR_TS_VAL            0

/*
// <o>Clear on return (CLRONRET) accessibility
//     <0=> Secure and Non-secure state
//     <1=> Secure state only
//   <i> Value for FPU->FPCCR register bit CLRONRETS
*/
#define FPU_FPCCR_CLRONRETS_VAL     0

/*
// <o>Clear floating-point caller saved registers on exception return
//     <0=> Disabled
//     <1=> Enabled
//   <i> Value for FPU->FPCCR register bit CLRONRET
*/
#define FPU_FPCCR_CLRONRET_VAL      1

/*
// </e>
*/

/*
    max 128 SAU regions.
    SAU regions are defined in partition.h
 */

#define SAU_INIT_REGION(n) \
    SAU->RNR  =  (n                                     & SAU_RNR_REGION_Msk); \
    SAU->RBAR =  (SAU_INIT_START##n                     & SAU_RBAR_BADDR_Msk); \
    SAU->RLAR =  (SAU_INIT_END##n                       & SAU_RLAR_LADDR_Msk) | \
                 ((SAU_INIT_NSC##n << SAU_RLAR_NSC_Pos)  & SAU_RLAR_NSC_Msk)   | 1U

#endif  /* PARTITION_M55M1_TEMPLATE_H */
