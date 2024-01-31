/**************************************************************************//**
 * @file     awf.c
 * @version  V1.00
 * @brief    AWF driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup AWF_Driver AWF Driver
    @{
*/

/** @addtogroup AWF_EXPORTED_FUNCTIONS AWF Exported Functions
    @{
*/



/**
  * @brief      This API is used to enable awf function
  * @param[in]  u32IntEn is interrupt selection.
  *             - \ref AWF_HTINT_ENABLE
  *             - \ref AWF_LTINT_ENABLE
  *             - \ref AWF_BOTHINT_ENABLE
  *             - \ref AWF_BOTHINT_DISABLE
  * @param[in]  u32WakeupEn
  *             - \ref AWF_HTWK_ENABLE
  *             - \ref AWF_LTWK_ENABLE
  *             - \ref AWF_BOTHWK_ENABLE
  *             - \ref AWF_BOTHWK_DISABLE
  * @param[in]  u32HTH is HTH Value. It could be 0~524288.
  * @param[in]  u32LTH is LTH Value. It could be 0~524288.
  * @param[in]  u32WBINIT is WBINIT Value. It could be 0~65535.
  * @param[in]  u32ACCCount is Accumulation count. It could be 2~8.
  * @details    Determine interrupt and wake-up enable or disable. \n
  *             Set suitable threshold and word buffer initial value in this API.
  */
void AWF_Open(uint32_t u32IntEn, uint32_t u32WakeupEn, uint32_t u32HTH, uint32_t u32LTH, uint32_t u32WBINIT, uint32_t u32ACCCount)
{
    /* Set high threshold value*/
    AWF_SET_HTH(u32HTH);

    /* Set low threshold value*/
    AWF_SET_LTH(u32LTH);

    /* Set accumulate count*/
    AWF->CTL = ((AWF->CTL & ~AWF_CTL_ACUCNT_Msk) | (u32ACCCount << AWF_CTL_ACUCNT_Pos));

    /* Set word buffer initial value*/
    AWF_SET_WBINIT(u32WBINIT);

    /* Clear interrupt status */
    AWF->STATUS = AWF->STATUS;

    /* Set interrupt function*/
    AWF->CTL = ((AWF->CTL & ~(AWF_CTL_HTIEN_Msk | AWF_CTL_LTIEN_Msk | AWF_CTL_HTWKEN_Msk | AWF_CTL_LTWKEN_Msk)) | u32IntEn | u32WakeupEn);
}

/**
  * @brief      This API is used to disable awf function
  */
void AWF_Close(void)
{
    /* Clear all setting value */
    AWF->CTL = AWF_CLOSE;
    AWF_SET_WBINIT(AWF_CLOSE);
    AWF_SET_HTH(AWF_CLOSE);
    AWF_SET_LTH(AWF_CLOSE);
    AWF_CLEAR_HTH_INTFLAG();
    AWF_CLEAR_LTH_INTFLAG();
}

/** @} end of group AWF_EXPORTED_FUNCTIONS */
/** @} end of group AWF_Driver */
/** @} end of group Standard_Driver */
