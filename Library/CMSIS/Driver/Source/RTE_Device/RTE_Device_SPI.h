/**************************************************************************//**
 * @file     RTE_Device_SPI.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M55M1 SPI
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------
/*
  CMSIS Driver Instance | Nuvoton Hardware Resource
  :---------------------|:--------------------------
  Driver_SPI0           | SPI0
  Driver_SPI1           | SPI1
  Driver_SPI2           | SPI2
  Driver_SPI3           | SPI3
  Driver_SPI4           | QSPI0
  Driver_SPI5           | QSPI1
  Driver_SPI6           | LPSPI0
  Driver_SPI7           | USCI_SPI0
*/

#ifndef __RTE_DEVICE_SPI_H
#define __RTE_DEVICE_SPI_H

#define RTE_SPI_FULL_XFER_MODE              1
#define RTE_SPI_HALF_XFER_MODE              2
#define RTE_SPI_3WIRE_XFER_MODE             3
#define RTE_QSPI_DUAL_XFER_MODE             4
#define RTE_QSPI_QUAD_XFER_MODE             5
#define RTE_LPSPI_AUTO_OP_XFER_MODE         6

// <e> SPI0 [SPI0]
// <i> Configuration settings for Driver_SPI0 in component ::CMSIS Driver:SPI
#define RTE_SPI0                            1

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_SPI0_XFER_MODE_ID               0
#if    (RTE_SPI0_XFER_MODE_ID == 0)
    #define RTE_SPI0_XFER_MODE              RTE_SPI_FULL_XFER_MODE
#elif (RTE_SPI0_XFER_MODE_ID == 1)
    #define RTE_SPI0_XFER_MODE              RTE_SPI_HALF_XFER_MODE
#elif  (RTE_SPI0_XFER_MODE_ID == 2)
    #define RTE_SPI0_XFER_MODE              RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_SPI0_XFER_MODE_ID > 2)
    #error "Invalid SPI0 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI0_RX_PDMA                    0
#define RTE_SPI0_RX_PDMA_PORT               0
#define RTE_SPI0_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI0_TX_PDMA                    0
#define RTE_SPI0_TX_PDMA_PORT               0
#define RTE_SPI0_TX_PDMA_CHANNEL            1

// </e>


// <e> SPI1 [SPI1]
// <i> Configuration settings for Driver_SPI1 in component ::CMSIS Driver:SPI
#define RTE_SPI1                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_SPI1_XFER_MODE_ID               1
#if    (RTE_SPI1_XFER_MODE_ID == 0)
    #define RTE_SPI1_XFER_MODE              RTE_SPI_FULL_XFER_MODE
#elif (RTE_SPI1_XFER_MODE_ID == 1)
    #define RTE_SPI1_XFER_MODE              RTE_SPI_HALF_XFER_MODE
#elif  (RTE_SPI1_XFER_MODE_ID == 2)
    #define RTE_SPI1_XFER_MODE              RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_SPI1_XFER_MODE_ID > 2)
    #error "Invalid SPI1 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI1_RX_PDMA                    0
#define RTE_SPI1_RX_PDMA_PORT               0
#define RTE_SPI1_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI1_TX_PDMA                    0
#define RTE_SPI1_TX_PDMA_PORT               0
#define RTE_SPI1_TX_PDMA_CHANNEL            1

// </e>


// <e> SPI2 [SPI2]
// <i> Configuration settings for Driver_SPI2 in component ::CMSIS Driver:SPI
#define RTE_SPI2                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_SPI2_XFER_MODE_ID               1
#if    (RTE_SPI2_XFER_MODE_ID == 0)
    #define RTE_SPI2_XFER_MODE              RTE_SPI_FULL_XFER_MODE
#elif (RTE_SPI2_XFER_MODE_ID == 1)
    #define RTE_SPI2_XFER_MODE              RTE_SPI_HALF_XFER_MODE
#elif  (RTE_SPI2_XFER_MODE_ID == 2)
    #define RTE_SPI2_XFER_MODE              RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_SPI2_XFER_MODE_ID > 2)
    #error "Invalid SPI2 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI2_RX_PDMA                    0
#define RTE_SPI2_RX_PDMA_PORT               0
#define RTE_SPI2_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI2_TX_PDMA                    0
#define RTE_SPI2_TX_PDMA_PORT               0
#define RTE_SPI2_TX_PDMA_CHANNEL            1

// </e>


// <e> SPI3 [SPI3]
// <i> Configuration settings for Driver_SPI3 in component ::CMSIS Driver:SPI
#define RTE_SPI3                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_SPI3_XFER_MODE_ID               1
#if    (RTE_SPI3_XFER_MODE_ID == 0)
    #define RTE_SPI3_XFER_MODE              RTE_SPI_FULL_XFER_MODE
#elif (RTE_SPI3_XFER_MODE_ID == 1)
    #define RTE_SPI3_XFER_MODE              RTE_SPI_HALF_XFER_MODE
#elif  (RTE_SPI3_XFER_MODE_ID == 2)
    #define RTE_SPI3_XFER_MODE              RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_SPI3_XFER_MODE_ID > 2)
    #error "Invalid SPI3 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI3_RX_PDMA                    0
#define RTE_SPI3_RX_PDMA_PORT               0
#define RTE_SPI3_RX_PDMA_CHANNEL            0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_SPI3_TX_PDMA                    0
#define RTE_SPI3_TX_PDMA_PORT               0
#define RTE_SPI3_TX_PDMA_CHANNEL            1

// </e>

// <e> SPI4 [QSPI0]
// <i> Configuration settings for Driver_SPI4 in component ::CMSIS Driver:SPI
#define RTE_SPI4                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_QSPI0_XFER_MODE_ID              0
#if    (RTE_QSPI0_XFER_MODE_ID == 0)
    #define RTE_QSPI0_XFER_MODE             RTE_SPI_FULL_XFER_MODE
#elif (RTE_QSPI0_XFER_MODE_ID == 1)
    #define RTE_QSPI0_XFER_MODE             RTE_SPI_HALF_XFER_MODE
#elif  (RTE_QSPI0_XFER_MODE_ID == 2)
    #define RTE_QSPI0_XFER_MODE             RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_QSPI0_XFER_MODE_ID == 3)
    #define RTE_QSPI0_XFER_MODE             RTE_QSPI_DUAL_XFER_MODE
#elif  (RTE_QSPI0_XFER_MODE_ID == 4)
    #define RTE_QSPI0_XFER_MODE             RTE_QSPI_QUAD_XFER_MODE
#elif  (RTE_QSPI0_XFER_MODE_ID > 4)
    #error "Invalid QSPI0 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_QSPI0_RX_PDMA                   1
#define RTE_QSPI0_RX_PDMA_PORT              0
#define RTE_QSPI0_RX_PDMA_CHANNEL           0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_QSPI0_TX_PDMA                   1
#define RTE_QSPI0_TX_PDMA_PORT              0
#define RTE_QSPI0_TX_PDMA_CHANNEL           1

// </e>


// <e> SPI5 [QSPI1]
// <i> Configuration settings for Driver_SPI5 in component ::CMSIS Driver:SPI
#define RTE_SPI5                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_QSPI1_XFER_MODE_ID              0
#if    (RTE_QSPI1_XFER_MODE_ID == 0)
    #define RTE_QSPI1_XFER_MODE             RTE_SPI_FULL_XFER_MODE
#elif (RTE_QSPI1_XFER_MODE_ID == 1)
    #define RTE_QSPI1_XFER_MODE             RTE_SPI_HALF_XFER_MODE
#elif  (RTE_QSPI1_XFER_MODE_ID == 2)
    #define RTE_QSPI1_XFER_MODE             RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_QSPI1_XFER_MODE_ID == 3)
    #define RTE_QSPI1_XFER_MODE             RTE_QSPI_DUAL_XFER_MODE
#elif  (RTE_QSPI1_XFER_MODE_ID == 4)
    #define RTE_QSPI1_XFER_MODE             RTE_QSPI_QUAD_XFER_MODE
#elif  (RTE_QSPI1_XFER_MODE_ID > 4)
    #error "Invalid QSPI1 Transfer Mode Configuration!"
#endif

//   <e> PDMA RX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_QSPI1_RX_PDMA                   0
#define RTE_QSPI1_RX_PDMA_PORT              0
#define RTE_QSPI1_RX_PDMA_CHANNEL           0

//   <e> PDMA TX
//     <o1> Port <0=>0 <1=>1
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_QSPI1_TX_PDMA                   0
#define RTE_QSPI1_TX_PDMA_PORT              0
#define RTE_QSPI1_TX_PDMA_CHANNEL           1

// </e>

// <e> SPI6 [LPSPI0]
// <i> Configuration settings for Driver_SPI6 in component ::CMSIS Driver:SPI
#define RTE_SPI6                            0

//   <o> Transfer Mode <0=>Full-Duplex
#define RTE_LPSPI0_XFER_MODE_ID             0
#if    (RTE_LPSPI0_XFER_MODE_ID == 0)
    #define RTE_LPSPI0_XFER_MODE            RTE_SPI_FULL_XFER_MODE
#elif (RTE_LPSPI0_XFER_MODE_ID == 1)
    #define RTE_LPSPI0_XFER_MODE            RTE_SPI_HALF_XFER_MODE
#elif  (RTE_LPSPI0_XFER_MODE_ID == 2)
    #define RTE_LPSPI0_XFER_MODE            RTE_SPI_3WIRE_XFER_MODE
#elif  (RTE_LPSPI0_XFER_MODE_ID == 3)
    #define RTE_LPSPI0_XFER_MODE            RTE_LPSPI_AUTO_OP_XFER_MODE
#elif  (RTE_LPSPI0_XFER_MODE_ID > 3)
    #error "Invalid LPSPI0 Transfer Mode Configuration!"
#endif

//   <e> LPPDMA RX
//     <o1> Port <0=>0
//     <i>  Selects LPPDMA Port
//     <o2> Channel(0~4) <0-4>
//     <i>  Selects LPPDMA Channel
//   </e>
#define RTE_LPSPI0_RX_PDMA                  1
#define RTE_LPSPI0_RX_PDMA_PORT             0
#define RTE_LPSPI0_RX_PDMA_CHANNEL          0

//   <e> LPPDMA TX
//     <o1> Port <0=>0
//     <i>  Selects LPPDMA Port
//     <o2> Channel(0~4) <0-4>
//     <i>  Selects LPPDMA Channel
//   </e>
#define RTE_LPSPI0_TX_PDMA                  1
#define RTE_LPSPI0_TX_PDMA_PORT             0
#define RTE_LPSPI0_TX_PDMA_CHANNEL          1

// </e>

// <e> SPI7 [USCI_SPI0]
// <i> Configuration settings for Driver_SPI7 in component ::CMSIS Driver:SPI
#define RTE_SPI7                            0

//   <e> PDMA RX
//     <o1> Port <0=>0
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_USPI0_RX_PDMA                   0
#define RTE_USPI0_RX_PDMA_PORT              0
#define RTE_USPI0_RX_PDMA_CHANNEL           0

//   <e> PDMA TX
//     <o1> Port <0=>0
//     <i>  Selects PDMA Port
//     <o2> Channel(0~15) <0-15>
//     <i>  Selects PDMA Channel
//   </e>
#define RTE_USPI0_TX_PDMA                   0
#define RTE_USPI0_TX_PDMA_PORT              0
#define RTE_USPI0_TX_PDMA_CHANNEL 1

// </e>

// <e> SPI8 [SPIM0]
// <i> Configuration settings for Driver_SPI8 in component ::CMSIS Driver:SPI
#define RTE_SPI8                            1

//   <e> PDMA RX
//   </e>
#define RTE_SPIM0_RX_PDMA                   1

//   <e> PDMA TX
//   </e>
#define RTE_SPIM0_TX_PDMA                   1

// </e>

#endif /* __RTE_DEVICE_SPI_H */
