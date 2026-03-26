/******************************************************************************
 * @file     sys_init.h
 * @version  V1.00
 * @brief    System clock configuration header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __SYSCLK_H__
#define __SYSCLK_H__

#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @brief   Provide user selections of HIRC's frequency.
  * @details
  * SYSCLK_HIRC_CLK_49152 : HIRC's frequency is 49.152 MHz.
  * SYSCLK_HIRC_CLK_48000 : HIRC's frequency is 48 MHz.
  */
#define SYSCLK_HIRC_CLK_49152    (0)
#define SYSCLK_HIRC_CLK_48000    (1)

/**
  * @brief   Provide user selections of HXT's frequency.
  * @details
  * SYSCLK_HXT_DISABE    : Disable HXT.
  * SYSCLK_HXT_CLK_12000 : Enable HXT, and HXT's frequency is 12 MHz
  * SYSCLK_HXT_CLK_12288 : Enable HXT, and HXT's frequency is 12.288 MHz
  */
#define SYSCLK_HXT_DISABLE       (0)
#define SYSCLK_HXT_CLK_12000     (1)
#define SYSCLK_HXT_CLK_12288     (2)

/**
  * @brief   Provide user selections of HCLK's clock source.
  * @details
  * SYSCLK_HCLK_CLK_HIRC     : HCLK's clock source is from HIRC.
  * SYSCLK_HCLK_CLK_HXT      : HCLK's clock source is from HXT.
  * SYSCLK_HCLK_CLK_PLL_HIRC : HCLK's clock source is from PLL, and PLL's clock source is from HIRC.
  * SYSCLK_HCLK_CLK_PLL_HXT  : HCLK's clock source is from PLL, and PLL's clock source is from HXT.
  */
#define SYSCLK_HCLK_CLK_HIRC     (0)
#define SYSCLK_HCLK_CLK_HXT      (1)
#define SYSCLK_HCLK_CLK_PLL_HIRC (2)
#define SYSCLK_HCLK_CLK_PLL_HXT  (3)

/**
  * @brief  This function generates desired system clock via the configuration defined in sysClk_config.h.
  * @return The actually system clock
  */
uint32_t SYS_Init(void);

#ifdef __cplusplus
}
#endif

#endif // __SYSCLK_H__
