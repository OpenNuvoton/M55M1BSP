/**************************************************************************//**
 * @file     Board.hpp
 * @version  V1.00
 * @brief    Target board releated function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __BOARD_INIT_HPP__
#define __BOARD_INIT_HPP__

/**
  * @brief Initiate the hardware resources
  * @return 0: Success, <0: Fail
  * @details Initiate clock, UART, NPU
  * \hideinitializer
  */
int BoardInit(void);

#endif

