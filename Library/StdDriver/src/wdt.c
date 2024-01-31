/**************************************************************************//**
 * @file     wdt.c
 * @version  V1.00
 * @brief    WDT driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup WDT_Driver WDT Driver
  @{
*/

/** @addtogroup WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
  * @brief      Initialize WDT and start counting
  *
  * @param[in] wdt                 wdt The pointer of the specified WDT module.
  * @param[in] u32TimeoutInterval  Time-out interval period of WDT module. Valid values are:
  *                                 - \ref WDT_TIMEOUT_2POW4
  *                                 - \ref WDT_TIMEOUT_2POW6
  *                                 - \ref WDT_TIMEOUT_2POW8
  *                                 - \ref WDT_TIMEOUT_2POW10
  *                                 - \ref WDT_TIMEOUT_2POW12
  *                                 - \ref WDT_TIMEOUT_2POW14
  *                                 - \ref WDT_TIMEOUT_2POW16
  *                                 - \ref WDT_TIMEOUT_2POW18
  *                                 - \ref WDT_TIMEOUT_2POW20
  * @param[in]  u32ResetDelay       Configure WDT time-out reset delay period. Valid values are:
  *                                 - \ref WDT_RESET_DELAY_1026CLK
  *                                 - \ref WDT_RESET_DELAY_130CLK
  *                                 - \ref WDT_RESET_DELAY_18CLK
  *                                 - \ref WDT_RESET_DELAY_3CLK
  * @param[in]  u32EnableReset      Enable WDT time-out reset system function. Valid values are TRUE and FALSE.
  * @param[in]  u32EnableWakeup     Enable WDT time-out wake-up system function. Valid values are TRUE and FALSE.
  *
  * @retval     WDT_OK              WDT operation OK.
  * @retval     WDT_ERR_TIMEOUT     WDT operation abort due to timeout error
  *
  * @details    This function makes WDT module start counting with different time-out interval, reset delay period and choose to \n
  *             enable or disable WDT time-out reset system or wake-up system.
  * @note       Please make sure that Register Write-Protection Function has been disabled before using this function.
  */
int32_t WDT_Open(WDT_T   *wdt,
                 uint32_t u32TimeoutInterval,
                 uint32_t u32ResetDelay,
                 uint32_t u32EnableReset,
                 uint32_t u32EnableWakeup)
{
    uint32_t u32TimeOutCnt = WDT_TIMEOUT;

    wdt->ALTCTL = u32ResetDelay;

    wdt->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |
               (u32EnableReset << WDT_CTL_RSTEN_Pos) |
               (u32EnableWakeup << WDT_CTL_WKEN_Pos);

    while ((wdt->CTL & WDT_CTL_SYNC_Msk) == WDT_CTL_SYNC_Msk) /* Wait enable WDTEN bit completed, it needs 4 * WDT_CLK. */
    {
        if (--u32TimeOutCnt == 0) return WDT_ERR_TIMEOUT;
    }

    return WDT_OK;

}

/** @} end of group WDT_EXPORTED_FUNCTIONS */
/** @} end of group WDT_Driver */
/** @} end of group Standard_Driver */
