/**************************************************************************//**
 * @file     plm.h
 * @version  V1.00
 * @brief    Product life cycle management driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PLM_H__
#define __PLM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PLM_Driver PLM Driver
  @{
*/

/** @addtogroup PLM_EXPORTED_CONSTANTS PLM Exported Constants
  @{
*/

typedef enum
{
    PLM_VENDOR   = 0,
    PLM_OEM      = 1,
    PLM_DEPLOYED = 3,
    PLM_RMA      = 7
} PLM_STAGE_T;

#define PLM_WVCODE   (0x475AUL << PLM_CTL_WVCODE_Pos)  /*!< The key code for PLM_CTL write. */

/** @} end of group FVC_EXPORTED_CONSTANTS */


/** @addtogroup FVC_EXPORTED_FUNCTIONS FVC Exported Functions
  @{
*/


/**
  * @brief      Get product life-cycle stage
  * @return     Current stage of PLM
  * @details    This function is used to Get PLM stage.
  */
#define PLM_GetStage()  ((PLM->STS & PLM_STS_STAGE_Msk) >> PLM_STS_STAGE_Pos)


/**
  * @brief      Set product lifecycle stage
  * @param[in]  eStage  Product life-cycle stage. It could be:
  *                     \ref PLM_VENDOR
  *                     \ref PLM_OEM
  *                     \ref PLM_DEPLOYED
  *                     \ref PLM_RMA
  * @retval     0   Successful
  * @retval     -1  Failed
  * @details    This function is used to set product lifecycle stage. The stage cannot rollback.
  *             It could be only be ePLM_STAGE_OEM, ePLM_STAGE_DEPLOYED and ePLM_STAGE_RMA.
  *             PLM provides some hardware-based control based on these stages. (Refer to TRM for detail descriptions)
  *             It needs cold reset to take effect.
  */
__STATIC_INLINE int32_t PLM_SetStage(PLM_STAGE_T eStage)
{

    /* Do nothing when stage is not changed */
    if (PLM_GetStage() == eStage)
        return 0;

    /* If dirty bit is set, it means previous update need cold reset to take effect */
    if (PLM->STS & PLM_STS_DIRTY_Msk)
        return -1;

    PLM->CTL = PLM_WVCODE | (eStage);

    /* The dirty flag should be set when PLM stage set successfully. */
    if (PLM->STS & PLM_STS_DIRTY_Msk)
        return -1;

    return 0;
}

/** @} end of group PLM_EXPORTED_FUNCTIONS */
/** @} end of group PLM_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __PLM_H__ */

