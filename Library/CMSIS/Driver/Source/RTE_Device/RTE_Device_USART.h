
/**************************************************************************//**
 * @file     RTE_Device_USART.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M55M1 UART/SCUART/LPUART/UUART
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

/*
  CMSIS Driver Instance | Hardware Resource
  :---------------------|:-----------------------
  Driver_USART0         | UART0
  Driver_USART1         | UART1
  Driver_USART2         | UART2
  Driver_USART3         | UART3
  Driver_USART4         | UART4
  Driver_USART5         | UART5
  Driver_USART6         | UART6
  Driver_USART7         | UART7
  Driver_USART8         | UART8
  Driver_USART9         | UART9
  Driver_USART10        | SCUART0
  Driver_USART11        | SCUART1
  Driver_USART12        | SCUART2
  Driver_USART13        | LPUART0
  Driver_USART14        | UUART0
*/

#ifndef __RTE_DEVICE_USART_H
    #define __RTE_DEVICE_USART_H

    #include "Driver_USART.h"

    // <e> USART0 (UART0)
    // <i> Configuration settings for Driver_USART0 in component ::CMSIS Driver:USART
    #define RTE_USART0                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART0_RX_PDMA               1
    #define RTE_USART0_RX_PDMA_NUMBER        0
    #define RTE_USART0_RX_PDMA_CHANNEL       0

    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART0_TX_PDMA               1
    #define RTE_USART0_TX_PDMA_NUMBER        1
    #define RTE_USART0_TX_PDMA_CHANNEL       0
    // </e>

    // <e> USART1 (UART1)
    // <i> Configuration settings for Driver_USART1 in component ::CMSIS Driver:USART
    #define RTE_USART1                       1

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART1_RX_PDMA               0
    #define RTE_USART1_RX_PDMA_NUMBER        0
    #define RTE_USART1_RX_PDMA_CHANNEL       1
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART1_TX_PDMA               0
    #define RTE_USART1_TX_PDMA_NUMBER        1
    #define RTE_USART1_TX_PDMA_CHANNEL       1
    // </e>

    // <e> USART2 (UART2)
    // <i> Configuration settings for Driver_USART2 in component ::CMSIS Driver:USART
    #define RTE_USART2                       1

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART2_RX_PDMA               1
    #define RTE_USART2_RX_PDMA_NUMBER        0
    #define RTE_USART2_RX_PDMA_CHANNEL       2
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART2_TX_PDMA               1
    #define RTE_USART2_TX_PDMA_NUMBER        1
    #define RTE_USART2_TX_PDMA_CHANNEL       2
    // </e>
    // <e> USART3 (UART3)
    // <i> Configuration settings for Driver_USART3 in component ::CMSIS Driver:USART
    #define RTE_USART3                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART3_RX_PDMA               1
    #define RTE_USART3_RX_PDMA_NUMBER        0
    #define RTE_USART3_RX_PDMA_CHANNEL       3
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART3_TX_PDMA               1
    #define RTE_USART3_TX_PDMA_NUMBER        1
    #define RTE_USART3_TX_PDMA_CHANNEL       3
    // </e>
    // <e> USART4 (UART4)
    // <i> Configuration settings for Driver_USART4 in component ::CMSIS Driver:USART
    #define RTE_USART4                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART4_RX_PDMA               1
    #define RTE_USART4_RX_PDMA_NUMBER        0
    #define RTE_USART4_RX_PDMA_CHANNEL       4
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART4_TX_PDMA               1
    #define RTE_USART4_TX_PDMA_NUMBER        1
    #define RTE_USART4_TX_PDMA_CHANNEL       4
    // </e>

    // <e> USART5 (UART5)
    // <i> Configuration settings for Driver_USART5 in component ::CMSIS Driver:USART
    #define RTE_USART5                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART5_RX_PDMA               1
    #define RTE_USART5_RX_PDMA_NUMBER        0
    #define RTE_USART5_RX_PDMA_CHANNEL       5
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART5_TX_PDMA               1
    #define RTE_USART5_TX_PDMA_NUMBER        1
    #define RTE_USART5_TX_PDMA_CHANNEL       5
    // </e>

    // <e> USART6 (UART6)
    // <i> Configuration settings for Driver_USART6 in component ::CMSIS Driver:USART
    #define RTE_USART6                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART6_RX_PDMA               1
    #define RTE_USART6_RX_PDMA_NUMBER        0
    #define RTE_USART6_RX_PDMA_CHANNEL       6
    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART6_TX_PDMA               1
    #define RTE_USART6_TX_PDMA_NUMBER        1
    #define RTE_USART6_TX_PDMA_CHANNEL       6
    // </e>

    // <e> USART7 (UART7)
    // <i> Configuration settings for Driver_USART7 in component ::CMSIS Driver:USART
    #define RTE_USART7                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART7_RX_PDMA               1
    #define RTE_USART7_RX_PDMA_NUMBER        0
    #define RTE_USART7_RX_PDMA_CHANNEL       7

    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART7_TX_PDMA               1
    #define RTE_USART7_TX_PDMA_NUMBER        1
    #define RTE_USART7_TX_PDMA_CHANNEL       7
    // </e>

    // <e> USART8 (UART8)
    // <i> Configuration settings for Driver_USART8 in component ::CMSIS Driver:USART
    #define RTE_USART8                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART8_RX_PDMA               1
    #define RTE_USART8_RX_PDMA_NUMBER        0
    #define RTE_USART8_RX_PDMA_CHANNEL       8

    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART8_TX_PDMA               1
    #define RTE_USART8_TX_PDMA_NUMBER        1
    #define RTE_USART8_TX_PDMA_CHANNEL       8
    // </e>

    // <e> USART9 (UART9)
    // <i> Configuration settings for Driver_USART9 in component ::CMSIS Driver:USART
    #define RTE_USART9                       0

    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART9_RX_PDMA               1
    #define RTE_USART9_RX_PDMA_NUMBER        0
    #define RTE_USART9_RX_PDMA_CHANNEL       9

    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART9_TX_PDMA               1
    #define RTE_USART9_TX_PDMA_NUMBER        1
    #define RTE_USART9_TX_PDMA_CHANNEL       9
    // </e>

    // <o0.0> USART10 (SCUART0)
    // <i> Configuration settings for Driver_USART10 in component ::CMSIS Driver:USART
    #define RTE_USART10                      1

    // <o0.0> USART11 (SCUART1)
    // <i> Configuration settings for Driver_USART11 in component ::CMSIS Driver:USART
    #define RTE_USART11                      1

    // <o0.0> USART12 (SCUART2)
    // <i> Configuration settings for Driver_USART12 in component ::CMSIS Driver:USART
    #define RTE_USART12                      1

    // <e> USART13 (LPUART0)
    // <i> Configuration settings for Driver_USART13 in component ::CMSIS Driver:USART
    #define RTE_USART13                      1

    //   <e> LPPDMA RX
    //     <o1> Number <0=>0
    //     <i>  Selects LPPDMA Number
    //     <o2> Channel(0~4) <0-4>
    //     <i>  Selects LPPDMA Channel
    //   </e>
    #define RTE_USART13_RX_LPPDMA                  1
    #define RTE_USART13_RX_LPPDMA_NUMBER           0
    #define RTE_USART13_RX_LPPDMA_CHANNEL          0

    //   <e> LPPDMA TX
    //     <o1> Number <0=>0
    //     <i>  Selects LPPDMA Number
    //     <o2> Channel(0~4) <0-4>
    //     <i>  Selects LPPDMA Channel
    //   </e>
    #define RTE_USART13_TX_LPPDMA                  1
    #define RTE_USART13_TX_LPPDMA_NUMBER           0
    #define RTE_USART13_TX_LPPDMA_CHANNEL          1
    // </e>

    // <e> USART14 (UUART0)
    // <i> Configuration settings for Driver_USART14 in component ::CMSIS Driver:USART
    #define RTE_USART14                       1
    //   <e> PDMA Rx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART14_RX_PDMA               1
    #define RTE_USART14_RX_PDMA_NUMBER        0
    #define RTE_USART14_RX_PDMA_CHANNEL       14

    //   <e> PDMA Tx
    //     <o1> Number <0=>0 <1=>1
    //     <i>  Selects PDMA Number
    //     <o2> Channel(0~15) <0-15>
    //     <i>  Selects PDMA Channel
    //   </e>
    #define RTE_USART14_TX_PDMA               1
    #define RTE_USART14_TX_PDMA_NUMBER        1
    #define RTE_USART14_TX_PDMA_CHANNEL       14
    // </e>
#endif


// Global driver structures ****************************************************
#if RTE_USART0
    extern  ARM_DRIVER_USART Driver_USART0;
#endif
#if RTE_USART1
    extern  ARM_DRIVER_USART Driver_USART1;
#endif
#if RTE_USART2
    extern  ARM_DRIVER_USART Driver_USART2;
#endif
#if RTE_USART3
    extern  ARM_DRIVER_USART Driver_USART3;
#endif
#if RTE_USART4
    extern  ARM_DRIVER_USART Driver_USART4;
#endif
#if RTE_USART5
    extern  ARM_DRIVER_USART Driver_USART5;
#endif
#if RTE_USART6
    extern  ARM_DRIVER_USART Driver_USART6;
#endif
#if RTE_USART7
    extern  ARM_DRIVER_USART Driver_USART7;
#endif
#if RTE_USART8
    extern  ARM_DRIVER_USART Driver_USART8;
#endif
#if RTE_USART9
    extern  ARM_DRIVER_USART Driver_USART9;
#endif
#if RTE_USART10
    extern  ARM_DRIVER_USART Driver_USART10;
#endif
#if RTE_USART11
    extern  ARM_DRIVER_USART Driver_USART11;
#endif
#if RTE_USART12
    extern  ARM_DRIVER_USART Driver_USART12;
#endif
#if RTE_USART13
    extern  ARM_DRIVER_USART Driver_USART13;
#endif
#if RTE_USART14
    extern  ARM_DRIVER_USART Driver_USART14;
#endif