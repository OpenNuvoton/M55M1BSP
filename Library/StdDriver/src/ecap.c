/**************************************************************************//**
 * @file     ecap.c
 * @version  V1.00
 * @brief    Enhanced Input Capture Timer (ECAP) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup ECAP_Driver ECAP Driver
  @{
*/

/** @addtogroup ECAP_EXPORTED_FUNCTIONS ECAP Exported Functions
  @{
*/

/**
  * @brief      Enable ECAP function
  * @param[in]  ecap        The pointer of the specified ECAP module.
  * @param[in]  u32FuncMask Input capture function select
  *                         - \ref ECAP_DISABLE_COMPARE_RELOAD
  *                         - \ref ECAP_COMPARE_FUNCTION
  *                         - \ref ECAP_RELOAD_FUNCTION
  *                         - \ref ECAP_RELOAD_COMPARE_FUNCTION
  * @return     None
  * @details    This macro enable input capture function and select compare and reload function.
  */
void ECAP_Open(ECAP_T *ecap, uint32_t u32FuncMask)
{
    /* Clear Input capture mode*/
    ecap->CTL0 = ecap->CTL0 & ~(ECAP_CTL0_CMPEN_Msk);
    ecap->CTL1 = ecap->CTL1 & ~(ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk | ECAP_CTL1_CAP2RLDEN_Msk);

    /* Enable Input Capture and set mode */
    ecap->CTL0 |= ECAP_CTL0_CAPEN_Msk;

    switch (u32FuncMask)
    {
        case ECAP_DISABLE_COMPARE_RELOAD:/*!< Input capture compare and reload function disable*/
            ecap->CTL0 = ecap->CTL0 & ~(ECAP_CTL0_CMPEN_Msk);
            ecap->CTL1 = ecap->CTL1 & ~(ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk | ECAP_CTL1_CAP2RLDEN_Msk);
            break;

        case ECAP_RELOAD_FUNCTION:/*!< Input capture reload function                    */
            ecap->CTL0 = ecap->CTL0 & ~(ECAP_CTL0_CMPEN_Msk);
            ecap->CTL1 = ecap->CTL1 | (ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk | ECAP_CTL1_CAP2RLDEN_Msk);
            break;

        case ECAP_COMPARE_FUNCTION:/*!< Input capture compare function                   */
            ecap->CTL0 = ecap->CTL0 | (ECAP_CTL0_CMPEN_Msk);
            ecap->CTL1 = ecap->CTL1 & ~(ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk | ECAP_CTL1_CAP2RLDEN_Msk);
            break;

        case ECAP_RELOAD_COMPARE_FUNCTION:/*!< Input capture reload & compare function          */
            ecap->CTL0 = ecap->CTL0 | (ECAP_CTL0_CMPEN_Msk);
            ecap->CTL1 = ecap->CTL1 | (ECAP_CTL1_CAP0RLDEN_Msk | ECAP_CTL1_CAP1RLDEN_Msk | ECAP_CTL1_CAP2RLDEN_Msk);
            break;

        default:
            break;
    }
}

/**
  * @brief      Disable ECAP function
  * @param[in]  ecap        The pointer of the specified ECAP module.
  * @return     None
  * @details    This macro disable input capture function.
  */
void ECAP_Close(ECAP_T *ecap)
{
    /* Disable Input Capture*/
    ecap->CTL0 &= ~ECAP_CTL0_CAPEN_Msk;
}

/**
  * @brief This macro is used to enable input channel interrupt
  * @param[in] ecap      Specify ECAP port
  * @param[in] u32Mask  The input channel Mask
  *                  - \ref ECAP_CTL0_CAPIEN0_Msk
  *                  - \ref ECAP_CTL0_CAPIEN1_Msk
  *                  - \ref ECAP_CTL0_CAPIEN2_Msk
  *                  - \ref ECAP_CTL0_OVIEN_Msk
  *                  - \ref ECAP_CTL0_CMPIEN_Msk
  * @return None
  * @details This macro will enable the input channel_n interrupt.
  */
void ECAP_EnableINT(ECAP_T *ecap, uint32_t u32Mask)
{
    /* Enable input channel interrupt */
    ecap->CTL0 |= (u32Mask);
}

/**
  * @brief This macro is used to disable input channel interrupt
  * @param[in] ecap      Specify ECAP port
  * @param[in] u32Mask  The input channel number
  *                  - \ref ECAP_CTL0_CAPIEN0_Msk
  *                  - \ref ECAP_CTL0_CAPIEN1_Msk
  *                  - \ref ECAP_CTL0_CAPIEN2_Msk
  *                  - \ref ECAP_CTL0_OVIEN_Msk
  *                  - \ref ECAP_CTL0_CMPIEN_Msk
  * @return None
  * @details This macro will disable the input channel_n interrupt.
  */
void ECAP_DisableINT(ECAP_T *ecap, uint32_t u32Mask)
{
    /* Disable input channel interrupt */
    ecap->CTL0 &= ~(u32Mask);
}

/** @} end of group ECAP_EXPORTED_FUNCTIONS */
/** @} end of group ECAP_Driver */
/** @} end of group Standard_Driver */
