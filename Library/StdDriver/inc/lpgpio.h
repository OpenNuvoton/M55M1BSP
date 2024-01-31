/**************************************************************************//**
 * @file     lpgpio.h
 * @version  V1.00
 * @brief    LPGPIO driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __LPGPIO_H__
#define __LPGPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPGPIO_Driver LPGPIO Driver
  @{
*/

/** @addtogroup LPGPIO_EXPORTED_CONSTANTS LPGPIO Exported Constants
  @{
*/

#define LPGPIO_PIN_MAX          (8UL)   /*!< Specify Maximum Pins of Each LPGPIO Port \hideinitializer */

/*------------------------------------------------------------------*/
/*  LPGPIO_MODE Constant Definitions                                */
/*------------------------------------------------------------------*/
#define LPGPIO_MODE_INPUT       (0x0UL) /*!< Input Mode \hideinitializer */
#define LPGPIO_MODE_OUTPUT      (0x1UL) /*!< Output Mode \hideinitializer */

/** @} end of group LPGPIO_EXPORTED_CONSTANTS */

/** @addtogroup LPGPIO_EXPORTED_FUNCTIONS LPGPIO Exported Functions
  @{
*/

/**
 * @brief       Get LPGPIO Port IN Data
 *
 * @param[in]   port        LPGPIO port. It could be LPGPIO.
 *
 * @return      The specified port data
 *
 * @details     Get the PIN register of specified LPGPIO port.
 * \hideinitializer
 */
#define LPGPIO_GET_IN_DATA(port)  ((port)->PIN)

/**
 * @brief       Set LPGPIO Port OUT Data
 *
 * @param[in]   port        LPGPIO port. It could be LPGPIO.
 * @param[in]   u32Data     LPGPIO port data.
 *
 * @details     Set the Data into specified LPGPIO port.
 * \hideinitializer
 */
#define LPGPIO_SET_OUT_DATA(port, u32Data)    ((port)->DOUT = (u32Data))

void LPGPIO_SetMode(LPGPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);

/** @} end of group LPGPIO_EXPORTED_FUNCTIONS */
/** @} end of group LPGPIO_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __LPGPIO_H__ */
