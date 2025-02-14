/**************************************************************************//**
 * @file     scu.h
 * @version  V1.00
 * @brief    Security Configuration Unit driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SCU_H__
#define __SCU_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup SCU_Driver SCU Driver
    @{
*/

/** @addtogroup SCU_EXPORTED_CONSTANTS SCU Exported Constants
    @{
*/

#define SRAM0_BASE          (SRAM_BASE)
#define SRAM0_SIZE          ((uint32_t) 0x00080000UL)
#define SRAM1_BASE          (SRAM0_BASE + SRAM0_SIZE)
#define SRAM1_SIZE          ((uint32_t) 0x00080000UL)
#define SRAM2_BASE          (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE          ((uint32_t) 0x00050000UL)
#define SRAM3_SIZE          ((uint32_t) 0x00002000UL)
#define LPSRAM_SIZE         ((uint32_t) 0x00002000UL)
#define SPIM0_MEM_BASE      ((uint32_t) 0x80000000UL)
#define SPIM0_MEM_SIZE      ((uint32_t) 0x02000000UL)
/* EBI memory secure/non-secure is decided by SCU_D1PNS1 and did not support MPC config */

#define SRAM0MPC_BLK        (0x8000UL)        // Block size: 32 KB
#define SRAM1MPC_BLK        (0x8000UL)        // Block size: 32 KB
#define SRAM2MPC_BLK        (0x8000UL)        // Block size: 32 KB
#define SRAM3MPC_BLK        (0x1000UL)        // Block size: 4 KB
#define LPSRAMMPC_BLK       (0x1000UL)        // Block size: 4 KB
#define SPIM0MPC_BLK        (0x20000UL)       // Block size: 128 KB

typedef enum
{
    eSCU_MASTER_ID_CPU      = 0,
    eSCU_MASTER_ID_PDMA0    = 1,
    eSCU_MASTER_ID_PDMA1    = 2,
    eSCU_MASTER_ID_USBH0    = 3,
    eSCU_MASTER_ID_HSUSBH   = 5,
    eSCU_MASTER_ID_HSUSBD   = 6,
    eSCU_MASTER_ID_SDH0     = 7,
    eSCU_MASTER_ID_SDH1     = 8,
    eSCU_MASTER_ID_EMAC     = 9,
    eSCU_MASTER_ID_CRYPTO   = 10,
    eSCU_MASTER_ID_CRC      = 11,
    eSCU_MASTER_ID_GDMA     = 12,
    eSCU_MASTER_ID_NPU      = 13,
    eSCU_MASTER_ID_LPPDMA   = 14,
    eSCU_MASTER_ID_CCAP     = 15,
    eSCU_MASTER_ID_SPIM0    = 16,
} E_SCU_MASTER_ID;

typedef enum
{
    eSCU_INT_IDX_APB0       = 0,
    eSCU_INT_IDX_APB1       = 1,
    eSCU_INT_IDX_APB2       = 2,
    eSCU_INT_IDX_APB3       = 3,
    eSCU_INT_IDX_APB4       = 4,
    eSCU_INT_IDX_APB5       = 5,
    eSCU_INT_IDX_D0PPC0     = 8,
    eSCU_INT_IDX_D1PPC0     = 9,
    eSCU_INT_IDX_D1PPC1     = 10,
    eSCU_INT_IDX_D2PPC0     = 11,
    eSCU_INT_IDX_EBI        = 16,
    eSCU_INT_IDX_FLASH      = 17,
    eSCU_INT_IDX_GDMA       = 32 +  0,
    eSCU_INT_IDX_PDMA0      = 32 +  1,
    eSCU_INT_IDX_PDMA1      = 32 +  2,
    eSCU_INT_IDX_USBH0      = 32 +  3,
    eSCU_INT_IDX_HSUSBH     = 32 +  5,
    eSCU_INT_IDX_HSUSBD     = 32 +  6,
    eSCU_INT_IDX_SDH0       = 32 +  7,
    eSCU_INT_IDX_SDH1       = 32 +  8,
    eSCU_INT_IDX_EMAC       = 32 +  9,
    eSCU_INT_IDX_CRYPTO     = 32 + 10,
    eSCU_INT_IDX_CRC        = 32 + 11,
    eSCU_INT_IDX_LPPDMA     = 32 + 12,
    eSCU_INT_IDX_CCAP       = 32 + 13,
    eSCU_INT_IDX_NPUIF1     = 32 + 14,
    eSCU_INT_IDX_NPUIF0     = 32 + 15,
    eSCU_INT_IDX_SPIM0      = 32 + 16,
    eSCU_INT_IDX_SRAM0_MPC  = 64 +  0,
    eSCU_INT_IDX_SRAM1_MPC  = 64 +  1,
    eSCU_INT_IDX_SRAM2_MPC  = 64 +  2,
    eSCU_INT_IDX_SRAM3_MPC  = 64 +  3,
    eSCU_INT_IDX_LPSRAM_MPC = 64 +  4,
    eSCU_INT_IDX_SPIM0_MPC  = 64 +  5,
} E_SCU_INT_IDX;

/**
 *  @details  Non-secure Attribution Definition.
 */
typedef enum
{
    /******  SCU_D0PNS0 **********************************************************************************/
    eSCU_PERI_IDX_NPU            = 3,

    /******  SCU_D0PNS2 **********************************************************************************/
    eSCU_PERI_IDX_SPIM0          = 64 + 2,

    /******  SCU_D1PNS0 **********************************************************************************/
    eSCU_PERI_IDX_PDMA0          = 256 +  0,
    eSCU_PERI_IDX_PDMA1          = 256 +  1,
    eSCU_PERI_IDX_USBH0          = 256 +  2,
    eSCU_PERI_IDX_HSUSBH         = 256 +  4,
    eSCU_PERI_IDX_HSUSBD         = 256 +  5,
    eSCU_PERI_IDX_SDH0           = 256 +  6,
    eSCU_PERI_IDX_SDH1           = 256 +  7,
    eSCU_PERI_IDX_EMAC0          = 256 +  8,
    eSCU_PERI_IDX_CRYPTO         = 256 + 10,
    eSCU_PERI_IDX_CRC            = 256 + 11,

    /******  SCU_D1PNS1 **********************************************************************************/
    eSCU_PERI_IDX_KDF            = 288 +  1,
    eSCU_PERI_IDX_CANFD0         = 288 +  2,
    eSCU_PERI_IDX_CANFD1         = 288 +  4,
    eSCU_PERI_IDX_EBI            = 288 + 16,

    /******  SCU_D1PNS2 **********************************************************************************/
    eSCU_PERI_IDX_WWDT0          = 320 +  0,
    eSCU_PERI_IDX_EADC0          = 320 +  1,
    eSCU_PERI_IDX_EPWM0          = 320 +  2,
    eSCU_PERI_IDX_BPWM0          = 320 +  3,
    eSCU_PERI_IDX_EQEI0          = 320 +  4,
    eSCU_PERI_IDX_EQEI2          = 320 +  5,
    eSCU_PERI_IDX_ECAP0          = 320 +  6,
    eSCU_PERI_IDX_ECAP2          = 320 +  7,
    eSCU_PERI_IDX_I2C0           = 320 +  8,
    eSCU_PERI_IDX_I2C2           = 320 +  9,
    eSCU_PERI_IDX_QSPI0          = 320 + 10,
    eSCU_PERI_IDX_SPI0           = 320 + 11,
    eSCU_PERI_IDX_SPI2           = 320 + 12,
    eSCU_PERI_IDX_UART0          = 320 + 13,
    eSCU_PERI_IDX_UART2          = 320 + 14,
    eSCU_PERI_IDX_UART4          = 320 + 15,
    eSCU_PERI_IDX_UART6          = 320 + 16,
    eSCU_PERI_IDX_UART8          = 320 + 17,
    eSCU_PERI_IDX_USCI0          = 320 + 18,
    eSCU_PERI_IDX_SC0            = 320 + 19,
    eSCU_PERI_IDX_SC2            = 320 + 20,
    eSCU_PERI_IDX_PSIO           = 320 + 21,
    eSCU_PERI_IDX_TMR01          = 320 + 22,
    eSCU_PERI_IDX_DAC01          = 320 + 23,
    eSCU_PERI_IDX_HSOTG          = 320 + 25,
    eSCU_PERI_IDX_I2S0           = 320 + 26,
    eSCU_PERI_IDX_ACMP01         = 320 + 27,
    eSCU_PERI_IDX_USBD           = 320 + 28,

    /******  SCU_D1PNS4 **********************************************************************************/
    eSCU_PERI_IDX_WWDT1          = 384 +  0,
    eSCU_PERI_IDX_EPWM1          = 384 +  2,
    eSCU_PERI_IDX_BPWM1          = 384 +  3,
    eSCU_PERI_IDX_EQEI1          = 384 +  4,
    eSCU_PERI_IDX_EQEI3          = 384 +  5,
    eSCU_PERI_IDX_ECAP1          = 384 +  6,
    eSCU_PERI_IDX_ECAP3          = 384 +  7,
    eSCU_PERI_IDX_I2C1           = 384 +  8,
    eSCU_PERI_IDX_I2C3           = 384 +  9,
    eSCU_PERI_IDX_QSPI1          = 384 + 10,
    eSCU_PERI_IDX_SPI1           = 384 + 11,
    eSCU_PERI_IDX_SPI3           = 384 + 12,
    eSCU_PERI_IDX_UART1          = 384 + 13,
    eSCU_PERI_IDX_UART3          = 384 + 14,
    eSCU_PERI_IDX_UART5          = 384 + 15,
    eSCU_PERI_IDX_UART7          = 384 + 16,
    eSCU_PERI_IDX_UART9          = 384 + 17,
    eSCU_PERI_IDX_SC1            = 384 + 18,
    eSCU_PERI_IDX_OTG            = 384 + 19,
    eSCU_PERI_IDX_KPI            = 384 + 20,
    eSCU_PERI_IDX_TMR23          = 384 + 21,
    eSCU_PERI_IDX_TRNG           = 384 + 22,
    eSCU_PERI_IDX_RTC            = 384 + 23,
    eSCU_PERI_IDX_I2S1           = 384 + 24,
    eSCU_PERI_IDX_ACMP23         = 384 + 25,
    eSCU_PERI_IDX_I3C0           = 384 + 26,
    eSCU_PERI_IDX_UTCPD          = 384 + 27,

    /******  SCU_D2PNS0 **********************************************************************************/
    eSCU_PERI_IDX_LPPDMA         = 512 + 0,
    eSCU_PERI_IDX_CCAP           = 512 + 1,
    eSCU_PERI_IDX_LPGPIO         = 512 + 3,

    /******  SCU_D2PNS2 **********************************************************************************/
    eSCU_PERI_IDX_LPTMR01        = 576 + 0,
    eSCU_PERI_IDX_TTMR01         = 576 + 1,
    eSCU_PERI_IDX_LPADC0         = 576 + 2,
    eSCU_PERI_IDX_LPI2C0         = 576 + 3,
    eSCU_PERI_IDX_LPSPI0         = 576 + 4,
    eSCU_PERI_IDX_DMIC0          = 576 + 5,
    eSCU_PERI_IDX_LPUART0        = 576 + 6,
    eSCU_PERI_IDX_AWF            = 576 + 9,
} E_SCU_PERI_IDX;

/** @} end of group SCU_EXPORTED_CONSTANTS */


/** @addtogroup SCU_EXPORTED_FUNCTIONS SCU Exported Functions
    @{
*/

/**
  * @brief      Set peripheral to non-secure state
  *
  * @param[in]  ePeriIdx   The secure/non-secure attribution of specified module.
                           The possible value could be refer to \ref E_SCU_PERI_IDX.
  *
  * @return     None
  *
  * @details    This macro is used to set a peripheral to be non-secure peripheral.
  *
  */
#define SCU_SET_PERI_NS(ePeriIdx)     (SCU->DxPNSy[(ePeriIdx) / 32] |= (1 << ((ePeriIdx) & 0x1Ful)))

/**
  * @brief      Set peripheral to secure state
  *
  * @param[in]  ePeriIdx   The secure/non-secure attribution of specified module.
                           The possible value could be refer to \ref E_SCU_PERI_IDX.
  *
  * @return     None
  *
  * @details    This macro is used to set a peripheral to be secure peripheral.
  *
  */
#define SCU_SET_PERI_S(ePeriIdx)     (SCU->DxPNSy[(ePeriIdx) / 32] &= ~(1 << ((ePeriIdx) & 0x1Ful)))

/**
 * @brief       Get peripheral secure/non-secure attribution
 *
 * @param[in]   ePeriIdx   The secure/non-secure attribution of specified module.
                           The possible value could be refer to \ref E_SCU_PERI_IDX.
 *
 * @return      The secure/non-secure attribution of specified peripheral.
 * @retval      0 The peripheral is secure
 * @retval      1 The peripheral is non-secure
 *
 * @details     This macro gets the peripheral secure/non-secure attribution.
 */
#define SCU_IS_PERI_NS(ePeriIdx)      ((SCU->DxPNSy[(ePeriIdx) / 32] >> ((ePeriIdx) & 0x1Ful)) & 1ul)


/**
 * @brief       Set secure/non-secure attribution of specified GPIO pin
 *
 * @param[in]   u32Port        GPIO Port. It could be PA, PB, PC, PD, PE, PF, PG and PH.
 * @param[in]   u32Bitmask     Bit mask of each bit. 0 is secure. 1 is non-secure.
 *
 * @return      None
 *
 * @details     This macro sets GPIO pin secure/non-secure attribution.
 */
#define SCU_SET_GPIO_NS(u32Port, u32Bitmask)    (SCU->IONS[((uint32_t)(u32Port) - (GPIOA_BASE)) / 0x40] = (u32Bitmask))


/**
 * @brief       Get secure/non-secure attribution of specified GPIO port
 *
 * @param[in]   u32Port        GPIO Port. It could be PA, PB, PC, PD, PE, PF, PG and PH.
 *
 * @return      The secure/non-secure attribution of the port.
 * @retval      0 The relative bit of specified IO port is secure
 * @retval      1 The relative bit of specified IO port is non-secure
 *
 * @details     This macro gets IO secure/non-secure attribution of specified IO port.
 */
#define SCU_GET_GPIO_NS(u32Port)                (SCU->IONS[((uint32_t)(u32Port) - (GPIOA_BASE)) / 0x40])


/**
 * @brief       Enable slave secure violation interrupts
 *
 * @param[in]   eIntIdx    The mask of each secure violation interrupt source
 *              - \ref SCU_SVIEN0_APB0IEN_Msk
 *              - \ref SCU_SVIEN0_APB1IEN_Msk
 *              - \ref SCU_SVIEN0_APB2IEN_Msk
 *              - \ref SCU_SVIEN0_APB3IEN_Msk
 *              - \ref SCU_SVIEN0_APB4IEN_Msk
 *              - \ref SCU_SVIEN0_APB5IEN_Msk
 *              - \ref SCU_SVIEN0_D0PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_D1PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_D1PPC1IEN_Msk
 *              - \ref SCU_SVIEN0_D2PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_EBIIEN_Msk
 *              - \ref SCU_SVIEN0_FLASHIEN_Msk
 *
 * @return      None
 *
 * @details     This macro is used to enable slave secure violation interrupt of SCU.
 *              The secure violation interrupt could be used to detect attack of secure elements.
 */
#define SCU_ENABLE_INT(eIntIdx)       (SCU->SVIEN[(eIntIdx) / 32] |= (1 << ((eIntIdx) & 0x1Ful)))


/**
 * @brief       Disable slave secure violation interrupts
 *
 * @param[in]   eIntIdx    The mask of each secure violation interrupt source
 *              - \ref SCU_SVIEN0_APB0IEN_Msk
 *              - \ref SCU_SVIEN0_APB1IEN_Msk
 *              - \ref SCU_SVIEN0_APB2IEN_Msk
 *              - \ref SCU_SVIEN0_APB3IEN_Msk
 *              - \ref SCU_SVIEN0_APB4IEN_Msk
 *              - \ref SCU_SVIEN0_APB5IEN_Msk
 *              - \ref SCU_SVIEN0_D0PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_D1PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_D1PPC1IEN_Msk
 *              - \ref SCU_SVIEN0_D2PPC0IEN_Msk
 *              - \ref SCU_SVIEN0_EBIIEN_Msk
 *              - \ref SCU_SVIEN0_FLASHIEN_Msk
 *
 * @return      None
 *
 * @details     This macro is used to disable slave secure violation interrupt of SCU.
 *
 */
#define SCU_DISABLE_INT(eIntIdx)      (SCU->SVIEN[(eIntIdx) / 32] &= ~(1 << ((eIntIdx) & 0x1Ful)))


/**
  * @brief      Get secure violation interrupt status
  *
  * @param[in]  eIntIdx     Available interrupt flag status
  *           - \ref eSCU_INT_IDX_APB0    \ref eSCU_INT_IDX_APB1   \ref eSCU_INT_IDX_APB2
  *           - \ref eSCU_INT_IDX_APB3    \ref eSCU_INT_IDX_APB4   \ref eSCU_INT_IDX_APB5
  *           - \ref eSCU_INT_IDX_D0PPC0  \ref eSCU_INT_IDX_D1PPC0
  *           - \ref eSCU_INT_IDX_D1PPC1  \ref eSCU_INT_IDX_D2PPC0
  *           - \ref eSCU_INT_IDX_EBI     \ref eSCU_INT_IDX_FLASH
  *           - \ref eSCU_INT_IDX_GDMA
  *           - \ref eSCU_INT_IDX_PDMA0   \ref eSCU_INT_IDX_PDMA1
  *           - \ref eSCU_INT_IDX_USBH0   \ref eSCU_INT_IDX_HSUSBH
  *           - \ref eSCU_INT_IDX_HSUSBD
  *           - \ref eSCU_INT_IDX_SDH0
  *           - \ref eSCU_INT_IDX_SDH1
  *           - \ref eSCU_INT_IDX_EMAC
  *           - \ref eSCU_INT_IDX_CRYPTO
  *           - \ref eSCU_INT_IDX_CRC
  *           - \ref eSCU_INT_IDX_LPPDMA
  *           - \ref eSCU_INT_IDX_CCAP
  *           - \ref eSCU_INT_IDX_NPUIF1
  *           - \ref eSCU_INT_IDX_NPUIF0
  *           - \ref eSCU_INT_IDX_SPIM0
  *           - \ref eSCU_INT_IDX_SRAM0_MPC
  *           - \ref eSCU_INT_IDX_SRAM1_MPC
  *           - \ref eSCU_INT_IDX_SRAM2_MPC
  *           - \ref eSCU_INT_IDX_SRAM3_MPC
  *           - \ref eSCU_INT_IDX_LPSRAM_MPC
  *           - \ref eSCU_INT_IDX_SPIM0_MPC
  * @return   The value of interrupt flag status
  */
#define SCU_GET_INT_FLAG(eIntIdx)     ((SCU->SVINTSTS[(eIntIdx) / 32] >> ((eIntIdx) & 0x1Ful)) & 0x1ul)

/**
  * @brief      Clear secure violation interrupt flag
  *
  *             The combination of the specified interrupt flags.
  *             Each bit corresponds to a interrupt source.
  *             This parameter decides which interrupt flags will be cleared.
  *             - \ref eSCU_INT_IDX_APB0    \ref eSCU_INT_IDX_APB1   \ref eSCU_INT_IDX_APB2
  *             - \ref eSCU_INT_IDX_APB3    \ref eSCU_INT_IDX_APB4   \ref eSCU_INT_IDX_APB5
  *             - \ref eSCU_INT_IDX_D0PPC0  \ref eSCU_INT_IDX_D1PPC0
  *             - \ref eSCU_INT_IDX_D1PPC1  \ref eSCU_INT_IDX_D2PPC0
  *             - \ref eSCU_INT_IDX_EBI     \ref eSCU_INT_IDX_FLASH
  *             - \ref eSCU_INT_IDX_GDMA
  *             - \ref eSCU_INT_IDX_PDMA0   \ref eSCU_INT_IDX_PDMA1
  *             - \ref eSCU_INT_IDX_USBH0   \ref eSCU_INT_IDX_HSUSBH
  *             - \ref eSCU_INT_IDX_HSUSBD
  *             - \ref eSCU_INT_IDX_SDH0
  *             - \ref eSCU_INT_IDX_SDH1
  *             - \ref eSCU_INT_IDX_EMAC
  *             - \ref eSCU_INT_IDX_CRYPTO
  *             - \ref eSCU_INT_IDX_CRC
  *             - \ref eSCU_INT_IDX_LPPDMA
  *             - \ref eSCU_INT_IDX_CCAP
  *             - \ref eSCU_INT_IDX_NPUIF1
  *             - \ref eSCU_INT_IDX_NPUIF0
  *             - \ref eSCU_INT_IDX_SPIM0
  *             - \ref eSCU_INT_IDX_SRAM0_MPC
  *             - \ref eSCU_INT_IDX_SRAM1_MPC
  *             - \ref eSCU_INT_IDX_SRAM2_MPC
  *             - \ref eSCU_INT_IDX_SRAM3_MPC
  *             - \ref eSCU_INT_IDX_LPSRAM_MPC
  *             - \ref eSCU_INT_IDX_SPIM0_MPC
  *
  * @return     None
  *
  * @details    Clear SCU related interrupt flags specified by flag parameter.
  *
  */
#define SCU_CLR_INT_FLAG(eIntIdx)     (SCU->SVINTSTS[(eIntIdx) / 32] = (1 << ((eIntIdx) & 0x1Ful)))

/**
  * @brief      Control the behavior of non-secure monitor when CPU is in idle state.
  *
  * @param[in]  opt Option for behavior control of non-secure monitor when CPU in idle.
  *              - true     The counter keeps counting when CPU is in idle.
                 - false    The counter will stop when CPU is in idle.
  *
  * @return     None
  *
  * @details    To control non-secure monitor counter when CPU is in idle.
  *
  */
#define SCU_NSM_IDLE_ON(opt)    ((opt)?(SCU->NSMCTL |= SCU_NSMCTL_IDLEON_Msk):(SCU->NSMCTL &= ~SCU_NSMCTL_IDLEON_Msk))

/**
  * @brief      Control the behavior of non-secure monitor when CPU is in debug state.
  *
  * @param[in]  opt Option for behavior control of non-secure monitor when CPU in debug.
  *              - true     The counter keeps counting when CPU is in debug.
                 - false    The counter will stop when CPU is in debug.
  *
  * @return     None
  *
  * @details    To control non-secure monitor counter when CPU is in debug.
  *
  */
#define SCU_NSM_DBG_ON(opt)    ((opt)?(SCU->NSMCTL |= SCU_NSMCTL_DBGON_Msk):(SCU->NSMCTL &= ~SCU_NSMCTL_DBGON_Msk))

/**
 * @brief       Check CPU is in secure or non-secure state
 *
 * @param[in]   psSCU   SCU or SCU_NS
 *
 * @return      The secure/non-secure state of CPU.
 * @retval      0 CPU is secure
 * @retval      1 CPU is non-secure
 *
 * @details     This macro returns CPU is in secure or non-secure state.
 */
#define SCU_IS_CPU_NS(psSCU)        ((psSCU->NSMSTS & SCU_NSMSTS_CURRNS_Msk) >> SCU_NSMSTS_CURRNS_Pos)

/**
 * @brief Retrieve the non-secure flash base address.
 * @return The non-secure flash base address.
 */
#define SCU_GET_FLASH_NS_BASE()     (SCU->FNSADDR)


/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void SCU_NSMConfig(uint32_t u32Ticks, uint32_t u32Prescale);
__STATIC_INLINE void SCU_TimerConfig(uint32_t u32Ticks, uint32_t u32Prescale);


/**
  * @brief      Config non-secure monitor to detect timeout in non-secure state.
  *
  * @param[in]  u32Ticks       A specified period for timeout in non-secure state
  * @param[in]  u32Prescale    A pre-scale divider to non-secure monitor clock

  *
  * @return     None
  *
  * @details    This function is used to configure non-secure monitor. If the CPU state stay in non-secure state for
  *             a specified period. The non-secure monitor will timeout and assert an interrupt. Otherwise, the
  *             non-secure monitor will auto clear whenever returning to secure state. This could be used to avoid
  *             CPU state in non-secure state too long time for security purpose. User must enable SCU_IRQn if interrupt
  *             is necessary.
  *
  */
__STATIC_INLINE void SCU_NSMConfig(uint32_t u32Ticks, uint32_t u32Prescale)
{

    SCU->NSMLOAD = u32Ticks;
    SCU->NSMVAL  = 0ul;
    SCU->NSMCTL  = SCU_NSMCTL_AUTORLD_Msk | SCU_NSMCTL_NSMIEN_Msk | (u32Prescale & 0xFFul);
}

/**
  * @brief      Config non-secure monitor to be a timer.
  *
  * @param[in]  u32Ticks       A specified period for timer interrupt.
  * @param[in]  u32Prescale    A pre-scale divider to timer clock source.

  *
  * @return     None
  *
  * @details    This function is used to configure non-secure monitor as a timer. In other words, the timer counter
  *             keeps counting even CPU is in secure state.
  *
  */
__STATIC_INLINE void SCU_TimerConfig(uint32_t u32Ticks, uint32_t u32Prescale)
{

    SCU->NSMLOAD = u32Ticks;
    SCU->NSMVAL  = 0ul;
    SCU->NSMCTL  = SCU_NSMCTL_AUTORLD_Msk | SCU_NSMCTL_NSMIEN_Msk | SCU_NSMCTL_TMRMOD_Msk | (u32Prescale & 0xFFul);
}

/** @} end of group SCU_EXPORTED_FUNCTIONS */
/** @} end of group SCU_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __SCU_H__ */
