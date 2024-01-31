/**************************************************************************//**
 * @file     lpgpio.c
 * @version  V1.00
 * @brief    LPGPIO driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPGPIO_Driver LPGPIO Driver
  @{
*/

/** @addtogroup LPGPIO_EXPORTED_FUNCTIONS LPGPIO Exported Functions
  @{
*/

/**
 * @brief       Set LPGPIO operation mode
 *
 * @param[in]   port        LPGPIO port. It could be LPGPIO.
 * @param[in]   u32PinMask  The single or multiple pins of specified LPGPIO port.
 *                          It could be BIT0 ~ BIT7 for LPGPIO pin 0 ~ pin 7.
 * @param[in]   u32Mode     Operation mode.  It could be \n
 *                          LPGPIO_MODE_INPUT, LPGPIO_MODE_OUTPUT.
 *
 * @details     This function is used to set specified LPGPIO operation mode.
 */
void LPGPIO_SetMode(LPGPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for (i = 0ul; i < LPGPIO_PIN_MAX; i++)
    {
        if ((u32PinMask & (BIT0 << i)) == (BIT0 << i))
        {
            port->MODE = (port->MODE & ~(BIT0 << i)) | (u32Mode << i);
        }
    }
}

/** @} end of group LPGPIO_EXPORTED_FUNCTIONS */
/** @} end of group LPGPIO_Driver */
/** @} end of group Standard_Driver */
