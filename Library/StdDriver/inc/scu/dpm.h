/**************************************************************************//**
 * @file     dpm.h
 * @version  V1.00
 * @brief    Debug Protection Mechanism driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DPM_H__
#define __DPM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup DPM_Driver DPM Driver
    @{
*/

/** @addtogroup DPM_EXPORTED_CONSTANTS DPM Exported Constants
    @{
*/

typedef enum
{
    eDPM_STS_DEFAULT  = 0,
    eDPM_STS_CLOSE    = 1,
    eDPM_STS_LOCKED   = 2,
    eDPM_STS_OPEN     = 5
} E_DPM_STS;

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Control Register Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DPM_CTL_WVCODE      (0x5AUL << DPM_CTL_RWVCODE_Pos)    /*!< Secure DPM control register write verify code */
#define DPM_CTL_RVCODE      (0xA5UL << DPM_CTL_RWVCODE_Pos)    /*!< Secure DPM control register read verify code */
#define DPM_NSCTL_WVCODE    (0x5AUL << DPM_CTL_RWVCODE_Pos)    /*!< Non-secure DPM control register write verify code */
#define DPM_NSCTL_RVCODE    (0xA5UL << DPM_CTL_RWVCODE_Pos)    /*!< Non-secure DPM control register read verify code */

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Time-out Handler Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DPM_TIMEOUT         (SystemCoreClock)   /*!< 1 second time-out */

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Error Code Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DPM_TIMEOUT_ERR     (-1L)               /*!< DPM time-out error value */

/** @}*/ /* end of group DPM_EXPORTED_CONSTANTS */

extern int32_t g_DPM_i32ErrCode;

/** @addtogroup DPM_EXPORTED_FUNCTIONS DPM Exported Functions
    @{
*/

/**
  * @brief      Wait DPM_STS Busy Flag
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     None
  * @details    This macro waits DPM_STS busy flag is cleared and skips when time-out.
  * @note       This macro sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
__STATIC_INLINE void DPM_WAIT_STS_BUSY(DPM_T *psDPM)
{
    uint32_t u32TimeOutCnt = DPM_TIMEOUT;

    g_DPM_i32ErrCode = 0;

    while (psDPM->STS & DPM_STS_BUSY_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            g_DPM_i32ErrCode = DPM_TIMEOUT_ERR;
            break;
        }
    }
}

/**
  * @brief      Enable DPM Interrupt
  * @param      None
  * @return     None
  * @details    This function enables DPM interrupt and only allowed to execute from Secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
__STATIC_INLINE void DPM_ENABLE_INT(void)
{
    DPM_WAIT_STS_BUSY(DPM);

    if (g_DPM_i32ErrCode == 0)
        DPM->CTL = (DPM->CTL & ~(DPM_CTL_RWVCODE_Msk | DPM_CTL_INTEN_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_INTEN_Msk);
}

/**
  * @brief      Disable DPM Interrupt (Secure Only)
  * @param      None
  * @return     None
  * @details    This function disables DPM interrupt and only allowed to execute from Secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
__STATIC_INLINE void DPM_DISABLE_INT(void)
{
    DPM_WAIT_STS_BUSY(DPM);

    if (g_DPM_i32ErrCode == 0)
        DPM->CTL = (DPM->CTL & ~(DPM_CTL_RWVCODE_Msk | DPM_CTL_INTEN_Msk)) | DPM_CTL_WVCODE;
}

/**
  * @brief      Enable Debugger to Access DPM Registers (Secure Only)
  * @param      None
  * @return     None
  * @details    This function enables debugger to access Secure and Non-secure DPM registers
  *             and only allowed to execute from Secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
__STATIC_INLINE void DPM_ENABLE_DBG_ACCESS(void)
{
    DPM_WAIT_STS_BUSY(DPM);

    if (g_DPM_i32ErrCode == 0)
        DPM->CTL = (DPM->CTL & ~(DPM_CTL_RWVCODE_Msk | DPM_CTL_DACCDIS_Msk)) | DPM_CTL_WVCODE;
}

/**
  * @brief      Disable Debugger to Access DPM Registers (Secure Only)
  * @param      None
  * @return     None
  * @details    This function disables debugger to access Secure and Non-secure DPM registers
  *             and only allowed to execute from Secure region.
  * @note       This function sets g_DPM_i32ErrCode to DPM_TIMEOUT_ERR if waiting DPM time-out.
  */
__STATIC_INLINE void DPM_DISABLE_DBG_ACCESS(void)
{
    DPM_WAIT_STS_BUSY(DPM);

    if (g_DPM_i32ErrCode == 0)
        DPM->CTL = (DPM->CTL & ~(DPM_CTL_RWVCODE_Msk | DPM_CTL_DACCDIS_Msk)) | (DPM_CTL_WVCODE | DPM_CTL_DACCDIS_Msk);
}

/**
  * @brief      Get DPM Password Update Count
  * @param[in]  psDPM  The pointer of the specified DPM module
  *                    - \ref DPM is for Secure DPM
  *                    - \ref DPM_NS is for Non-secure DPM
  * @return     DPM Password Update Count
  */
__STATIC_INLINE int32_t DPM_GetPasswordUpdateCnt(DPM_T *psDPM)
{
    int32_t i32RetVal = DPM_TIMEOUT_ERR;

    DPM_WAIT_STS_BUSY(psDPM);

    if (g_DPM_i32ErrCode == 0)
        i32RetVal = (psDPM->STS & DPM_STS_PWUCNT_Msk) >> DPM_STS_PWUCNT_Pos;

    return i32RetVal;
}

void DPM_SetDebugDisable(DPM_T *psDPM);
void DPM_SetDebugLock(DPM_T *psDPM);
int32_t DPM_GetDebugDisable(DPM_T *psDPM);
int32_t DPM_GetDebugLock(DPM_T *psDPM);
int32_t DPM_SetPasswordUpdate(DPM_T *psDPM, uint32_t au32Pwd[]);
int32_t DPM_SetPasswordCompare(DPM_T *psDPM, uint32_t au32Pwd[]);
int32_t DPM_GetPasswordErrorFlag(DPM_T *psDPM);
int32_t DPM_GetIntFlag(void);
void DPM_ClearPasswordErrorFlag(DPM_T *psDPM);
void DPM_EnableDebuggerWriteAccess(DPM_T *psDPM);
void DPM_DisableDebuggerWriteAccess(DPM_T *psDPM);


/** @} end of group DPM_EXPORTED_FUNCTIONS */
/** @} end of group DPM_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __DPM_H__ */
