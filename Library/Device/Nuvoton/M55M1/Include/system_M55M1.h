/**************************************************************************//**
 * @file     system_M55M1.h
 * @version  V1.00
 * @brief    CMSIS Device System Header File for NuMicro M55M1
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#ifndef __SYSTEM_M55M1_H__
#define __SYSTEM_M55M1_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__ICCARM__)
//#pragma diag_suppress=Be006
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Macro Definition                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef NVT_DCACHE_ON
#define NVT_DCACHE_ON               1    /*!< Set 1 to enable data cache or 0 to disable data cache */
#endif

#ifndef CHECK_SCU_VIOLATION
#define CHECK_SCU_VIOLATION         0    /*!< Set default SCU violation check disabled */
#endif

#if !defined(DEBUG_PORT_UART_IDX)
#define DEBUG_PORT_UART_IDX         0    /*!< Set UART port index to specify console port. */
#endif

#if !defined(DEBUG_PORT_UART_GRP_IDX)
#define DEBUG_PORT_UART_GRP_IDX     0    /*!< Set UART port group number of specified console port. */
#endif

#define PASTER(a, b, c)         a##b##c
#define EVAL_MACRO(a, b, c)     PASTER(a, b, c)
#define _UARTn(n)               EVAL_MACRO(UART, n, )
#define _UARTn_MODULE(n)        EVAL_MACRO(UART, n, _MODULE)
#define _UARTn_IRQn(n)          EVAL_MACRO(UART, n, _IRQn)
#define _UARTn_IRQHandler(n)    EVAL_MACRO(UART, n, _IRQHandler)
#define _UARTn_FIFO_SIZE(n)     EVAL_MACRO(UART, n, _FIFO_SIZE)
#define _UARTn_RST(n)           EVAL_MACRO(SYS_, EVAL_MACRO(UART, n, RST), )
#define _UARTn_CLKSEL(m, n)     EVAL_MACRO(CLK_, EVAL_MACRO(UARTSEL, m, _), EVAL_MACRO(UART, n, SEL_HIRC))
#define _UARTn_CLKDIV(m, n)     EVAL_MACRO(CLK_, EVAL_MACRO(UARTDIV, m, _), EVAL_MACRO(UART, n, DIV(1)))

#if !defined(DEBUG_PORT)
#define DEBUG_PORT              _UARTn(DEBUG_PORT_UART_IDX)                                    /*!< UARTn                       */
#endif
#define DEBUG_PORT_MODULE       _UARTn_MODULE(DEBUG_PORT_UART_IDX)                             /*!< UARTn_MODULE                */
#define DEBUG_PORT_IRQn         _UARTn_IRQn(DEBUG_PORT_UART_IDX)                               /*!< UARTn_IRQn                  */
#define DEBUG_PORT_IRQHandler   _UARTn_IRQHandler(DEBUG_PORT_UART_IDX)                         /*!< UARTn_IRQHandler            */
#define DEBUG_PORT_FIFO_SIZE    _UARTn_FIFO_SIZE(DEBUG_PORT_UART_IDX)                          /*!< UARTn_FIFO_SIZE             */
#define DEBUG_PORT_RST          _UARTn_RST(DEBUG_PORT_UART_IDX)                                /*!< UARTn_RST                   */
#define DEBUG_PORT_CLKSEL       _UARTn_CLKSEL(DEBUG_PORT_UART_GRP_IDX, DEBUG_PORT_UART_IDX)    /*!< CLK_UARTSELm_UARTnSEL_HIRC  */
#define DEBUG_PORT_CLKDIV       _UARTn_CLKDIV(DEBUG_PORT_UART_GRP_IDX, DEBUG_PORT_UART_IDX)    /*!< CLK_UARTDIVm_UARTnDIV(1)    */

#define ICACHE_LINE_SIZE                        (__SCB_ICACHE_LINE_SIZE)    /*!< ICache line byte size              */
#define DCACHE_LINE_SIZE                        (__SCB_DCACHE_LINE_SIZE)    /*!< DCache line byte size              */
#define DCACHE_ALIGN_LINE_SIZE(u32ByteSize)     (((u32ByteSize) + (DCACHE_LINE_SIZE) - 1) & ~((DCACHE_LINE_SIZE) - 1))  /* Align to DCache line size */

#define NVT_UNUSED(x)                           (void)(x)                                           /*!< To suppress the unused parameter warnings      */
#define NVT_ITCM                                __attribute__((section("ITCM")))                    /*!< Placed declaration code in ITCM region         */
#define NVT_DTCM_INIT                           __attribute__((section("DTCM.Init")))               /*!< Placed declaration data in DTCM region         */
#define NVT_NONCACHEABLE_INIT                   __attribute__((section("NonCacheable.Init")))       /*!< Placed declaration data in NonCacheable region */
#define NVT_DTCM_VTOR                           __attribute__((section("DTCM.VTOR")))               /*!< Placed vector table in DTCM region             */

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
#if (NVT_DCACHE_ON == 1)
/* If D-Cache is enabled, placed NVT_NONCACHEABLE in predefined non-cacheable section. */
#define NVT_NONCACHEABLE                __attribute__((section(".NonCacheable.ZeroInit")))          /*!< Placed declaration data in NonCacheable region */
#else   // (NVT_DCACHE_ON == 0)
/* If D-Cache is disabled, placed NVT_NONCACHEABLE in bss section. */
#define NVT_NONCACHEABLE                __attribute__((section(".bss.NonCacheable.ZeroInit")))      /*!< Placed declaration data in bss region          */
#endif

#define NVT_DTCM                            __attribute__((section(".DTCM.ZeroInit")))              /*!< Placed declaration data in DTCM region         */
#define NVT_NOINIT                          __attribute__((section(".noinit")))                     /*!< Placed variables in the UNINIT region          */
#else
/* https://developer.arm.com/documentation/ka003046 */
/* Arm Compiler 6 only creates ZI sections, if the section name starts with ".bss". */
#define NVT_NONCACHEABLE                    __attribute__((section(".bss.NonCacheable.ZeroInit")))  /*!< Placed declaration data in NonCacheable region */
#define NVT_DTCM                            __attribute__((section(".bss.DTCM.ZeroInit")))          /*!< Placed declaration data in DTCM region         */
#define NVT_NOINIT                          __attribute__((section(".bss.NoInit")))                 /*!< Placed variables in the UNINIT region          */
#endif

#define __PC()                                                \
    __extension__({                                           \
        register unsigned int current_pc;                     \
        __asm volatile("mov %0, pc" : "=r"(current_pc) : : ); \
        current_pc;                                           \
    })    /*!< Current program counter            */

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/

#ifndef __HSI
#define __HSI       (220000000UL)         /*!< PLL default output is 220 MHz */
#endif

#ifndef __HXT
#define __HXT       (24000000UL)          /*!< External Crystal Clock Frequency */
#endif

#ifndef __LXT
#define __LXT       (32768UL)             /*!< External Crystal Clock Frequency 32.768 KHz */
#endif

#define __HIRC      (12000000UL)          /*!< Internal 12 MHz RC Oscillator Frequency */
#define __HIRC48M   (48000000UL)          /*!< Internal 48 MHz RC Oscillator Frequency */
#define __MIRC      (1000000UL)           /*!< Internal 1 MHz RC Oscillator Frequency */
#define __LIRC      (32000UL)             /*!< Internal 32 KHz RC Oscillator Frequency */

#define __SYS_OSC_CLK     (    ___HSI)    /*!< Main oscillator frequency */
#define __SYSTEM_CLOCK    (1UL*__HXT)

extern uint32_t CyclesPerUs;              /*!< Cycles per micro second              */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock)  */
extern uint32_t PllClock;                 /*!< PLL Output Clock Frequency           */

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3L)
#if defined (__ICCARM__)
#define __NONSECURE_ENTRY       __cmse_nonsecure_entry
#define __NONSECURE_ENTRY_WEAK  __cmse_nonsecure_entry //__weak
#define __NONSECURE_CALL        __cmse_nonsecure_call
#else
#define __NONSECURE_ENTRY       __attribute__ ((cmse_nonsecure_entry))
#define __NONSECURE_ENTRY_WEAK  __attribute__ ((cmse_nonsecure_entry, weak))
#define __NONSECURE_CALL        __attribute__ ((cmse_nonsecure_call))
#endif
#else
#define __NONSECURE_ENTRY
#define __NONSECURE_ENTRY_WEAK
#define __NONSECURE_CALL
#endif

/**
  \brief Exception / Interrupt Handler Function Prototype
*/
typedef void(*VECTOR_TABLE_Type)(void);

/**
  \brief System Clock Frequency (Core Clock)
*/
extern uint32_t SystemCoreClock;

/**
  \brief Setup the microcontroller system.

   Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit(void);


/**
  \brief  Update SystemCoreClock variable.

   Updates the SystemCoreClock with current core Clock retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate(void);

#ifndef NVT_DBG_UART_OFF
/**
 * \brief  Set debug UART multi-function pins
 */
extern void SetDebugUartMFP(void);

/**
 * \brief  Set debug UART clock source and enable module clock.
 */
extern void SetDebugUartCLK(void);

/**
 * \brief  Initialize debug UART to 115200-8n1
 */
extern void InitDebugUart(void);

#endif /* NVT_DBG_UART_OFF */

/**
 * \brief  Initialize MPU Region according to mpu_config_M55M1.h and user defined table
 */
extern int32_t InitPreDefMPURegion(const ARM_MPU_Region_t *psMPURegion, uint32_t u32RegionCnt);

/**
 * \brief    Setup MPC configuration
 */
extern int32_t SetupMPC(
    const uint32_t u32MPCBaseAddr,
    const uint32_t u32MemBaseAddr,    const uint32_t u32MemByteSize,
    const uint32_t u32MemBaseAddr_S,  const uint32_t u32MemByteSize_S,
    const uint32_t u32MemBaseAddr_NS, const uint32_t u32MemByteSize_NS
);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_M55M1_H__ */
