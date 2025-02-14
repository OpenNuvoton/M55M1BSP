/**************************************************************************//**
 * @file     dpm.c
 * @version  V1.00
 * @brief    Debug Protection Mechanism driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DPM_Driver DPM Driver
  @{
*/

int32_t g_DPM_i32ErrCode = 0; /*!< DPM global error code */

/** @addtogroup DPM_EXPORTED_FUNCTIONS DPM Exported Functions
  @{
*/

/**
  * @brief      Set Debug Disable
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     None
  * @details    This function sets Secure or Non-secure DPM debug disable.
  *             The debug disable function works after reset (chip reset or pin reset).
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
void DPM_SetDebugDisable(DPM_T *psDPM)
{
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        psDPM->CTL = (psDPM->CTL & (~DPM_CTL_RWVCODE_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_DBGDIS_Msk);
}

/**
  * @brief      Set Debug Lock
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                     - \ref DPM
  *                     - \ref DPM_NS
  * @return     None
  * @details    This function sets Secure or Non-secure DPM debug lock.
  *             The debug lock function works after reset (chip reset or pin reset).
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
void DPM_SetDebugLock(DPM_T *psDPM)
{
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        psDPM->CTL = (psDPM->CTL & (~DPM_CTL_RWVCODE_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_LOCK_Msk);
}

/**
  * @brief      Get Debug Disable
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @retval     0   Debug is not in disable status.
  * @retval     1   Debug is in disable status.
  * @retval     -1  DPM time-our error
  * @details    This function gets Secure or Non-secure DPM debug disable status.
  *             If Secure debug is disabled, debugger cannot access Secure region and can access Non-secure region only.
  *             If Non-secure debug is disabled, debugger cannot access all Secure and Non-secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_GetDebugDisable(DPM_T *psDPM)
{
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        i32RetVal = (psDPM->STS & DPM_STS_DBGDIS_Msk) >> DPM_STS_DBGDIS_Pos;

    return i32RetVal;
}

/**
  * @brief      Get Debug Lock
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @retval     0   Debug is not in lock status.
  * @retval     1   Debug is in lock status.
  * @retval     -1  DPM time-our error
  * @details    This function gets Secure or Non-secure DPM debug disable status.
  *             If Secure debug is locked, debugger cannot access Secure region and can access Non-secure region only.
  *             If Non-secure debug is locked, debugger cannot access all Secure and Non-secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_GetDebugLock(DPM_T *psDPM)
{
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        i32RetVal = (psDPM->STS & DPM_STS_LOCK_Msk) >> DPM_STS_LOCK_Pos;

    return i32RetVal;
}

/**
  * @brief      Update DPM Password
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @param[in]  au32Pwd  Password with 256 bits length.
  * @retval     0   No password is updated. The password update count has reached the maximum value.
  * @retval     1   Password update is successful.
  * @retval     -1  DPM time-our error
  * @details    This function updates Secure or Non-secure DPM password.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_SetPasswordUpdate(DPM_T *psDPM, uint32_t au32Pwd[])
{
    uint32_t u32i;
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    /* Set Secure DPM password */
    for (u32i = 0; u32i < 4; u32i++)
    {
        DPM_WAIT_STS_BUSY(psDPM);

        if (g_DPM_i32ErrCode != 0)
            break;

        psDPM->SPW[u32i] = au32Pwd[u32i];
    }

    /* Set Secure DPM password update */
    if (g_DPM_i32ErrCode == 0)
    {
        DPM_WAIT_STS_BUSY(psDPM);

        if (g_DPM_i32ErrCode == 0)
            psDPM->CTL = (psDPM->CTL & (~DPM_CTL_RWVCODE_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_PWUPD_Msk);
    }

    /* Check Secure DPM password update flag */
    if (g_DPM_i32ErrCode == 0)
    {
        DPM_WAIT_STS_BUSY(psDPM);

        if (g_DPM_i32ErrCode == 0)
            i32RetVal = (psDPM->STS & DPM_STS_PWUOK_Msk) >> DPM_STS_PWUOK_Pos;
    }

    /* Clear Secure DPM password update flag */
    if (g_DPM_i32ErrCode == 0)
    {
        DPM_WAIT_STS_BUSY(psDPM);

        if ((i32RetVal == 1) && (g_DPM_i32ErrCode == 0))
            psDPM->STS = DPM_STS_PWUOK_Msk;
    }

    return i32RetVal;
}

/**
  * @brief      Compare DPM Password
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @param[in]  au32Pwd  Compared password with 256 bits length.
  * @retval     0   The password comparison can be proccessed.
  * @retval     1   No more password comparison can be proccessed. \n
  *                 The password comparison fail times has reached the maximum value.
  * @retval     -1  DPM time-our error
  * @details    This function sets Secure or Non-secure DPM password comparison. \n
  *             The comparison result is checked by DPM_GetPasswordErrorFlag().
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_SetPasswordCompare(DPM_T *psDPM, uint32_t au32Pwd[])
{
    uint32_t u32i;
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    /* Check Secure DPM password compare fail times maximum flag */
    if (psDPM->STS & DPM_STS_PWFMAX_Msk)
    {
        i32RetVal = 1;
    }
    else
    {
        /* Set Secure DPM password */
        for (u32i = 0; u32i < 4; u32i++)
        {
            DPM_WAIT_STS_BUSY(psDPM);

            if (g_DPM_i32ErrCode != 0)
                break;

            psDPM->SPW[u32i] = au32Pwd[u32i];
        }

        /* Set Secure DPM password cpmpare */
        if (g_DPM_i32ErrCode == 0)
        {
            DPM_WAIT_STS_BUSY(psDPM);

            if (g_DPM_i32ErrCode == 0)
            {
                psDPM->CTL = (psDPM->CTL & (~DPM_CTL_RWVCODE_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_PWCMP_Msk);
                i32RetVal = 0;
            }
        }
    }

    return i32RetVal;
}

/**
  * @brief      Get DPM Password Error Flag
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @retval     0   Specified DPM module password compare error flag is 0.
  * @retval     1   Specified DPM module password compare error flag is 1.
  * @retval     -1  DPM time-our error
  * @details    This function returns Secure or Non-secure DPM password compare error flag.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_GetPasswordErrorFlag(DPM_T *psDPM)
{
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    /* Check Secure DPM password compare error flag */
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        i32RetVal = (psDPM->STS & DPM_STS_PWCERR_Msk) >> DPM_STS_PWCERR_Pos;

    return i32RetVal;
}

/**
  * @brief      Get DPM Interrupt Flag (Secure Only)
  * @param      None
  * @retval     0   Secure DPM interrupt flag is 0.
  * @retval     1   Secure DPM interrupt flag is 1.
  * @retval     -1  DPM time-our error
  * @details    This function returns Secure DPM interrupt flag.
  *             Secure DPM interrupt flag includes Secure and Non-secure DPM password compare error flag.
  *             This function is for Secure DPM and Secure region only.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
int32_t DPM_GetIntFlag(void)
{
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    DPM_WAIT_STS_BUSY(DPM);

    if (g_DPM_i32ErrCode == 0)
        i32RetVal = (DPM->STS & DPM_STS_INT_Msk) >> DPM_STS_INT_Pos;

    return i32RetVal;
}


/**
  * @brief      Clear DPM Password Error Flag
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     None
  * @details    This function clears Secure or Non-secure DPM password compare error flag.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
void DPM_ClearPasswordErrorFlag(DPM_T *psDPM)
{
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
    {
        psDPM->STS = DPM_STS_PWCERR_Msk;
    }
}

/**
  * @brief      Enable Debugger Write Access
  ** @param[in] psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     None
  * @details    This function enables external debugger to write Secure or Non-secure DPM registers.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
void DPM_EnableDebuggerWriteAccess(DPM_T *psDPM)
{
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        psDPM->CTL = (psDPM->CTL & (~(DPM_CTL_RWVCODE_Msk | DPM_CTL_DACCWDIS_Msk))) | DPM_CTL_WVCODE;
}

/**
  * @brief      Disable Debugger Write Access
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     None.
  * @details    This function disables external debugger to write Secure or Non-secure DPM registers.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
void DPM_DisableDebuggerWriteAccess(DPM_T *psDPM)
{
    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        psDPM->CTL = (psDPM->CTL & (~DPM_CTL_RWVCODE_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_DACCWDIS_Msk);
}

/** @} end of group DPM_EXPORTED_FUNCTIONS */
/** @} end of group DPM_Driver */
/** @} end of group Standard_Driver */
