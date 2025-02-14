/**************************************************************************//**
 * @file     acmp.c
 * @version  V1.00
 * @brief    M55M1 series ACMP driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup ACMP_Driver ACMP Driver
  @{
*/

int32_t g_ACMP_i32ErrCode = 0;  /*!< ACMP global error code */

/** @addtogroup ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief  Configure the specified ACMP module
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  * @param[in]  u32NegSrc Comparator negative input selection.  Including:
  *                  - \ref ACMP_CTL_NEGSEL_PIN
  *                  - \ref ACMP_CTL_NEGSEL_CRV
  *                  - \ref ACMP_CTL_NEGSEL_VBG
  *                  - \ref ACMP_CTL_NEGSEL_DAC0
  *                  - \ref ACMP_CTL_NEGSEL_DAC1
  * @param[in]  u32HysSel The hysteresis function option. Including:
  *                  - \ref ACMP_CTL_HYSTERESIS_40MV
  *                  - \ref ACMP_CTL_HYSTERESIS_20MV
  *                  - \ref ACMP_CTL_HYSTERESIS_DISABLE
  *
  * @return     None
  *
  * @details    Configure hysteresis function, select the source of negative input and enable analog comparator.
  * @note       This API will reset and calibrate ACMP if ACMP never be calibrated after chip power on.
  */
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysSel)
{

    acmp->CTL[u32ChNum] = (acmp->CTL[u32ChNum] & (~(ACMP_CTL_NEGSEL_Msk | ACMP_CTL_HYSSEL_Msk))) | (u32NegSrc | u32HysSel | ACMP_CTL_ACMPEN_Msk);
}

/**
  * @brief  Close analog comparator
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  *
  * @return     None
  *
  * @details  This function will clear ACMPEN bit of ACMP_CTL register to disable analog comparator.
  */
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum)
{
    acmp->CTL[u32ChNum] &= (~ACMP_CTL_ACMPEN_Msk);
}

/**
  * @brief  Calibration the specified ACMP module
  *
  * @param[in]  acmp The pointer of the specified ACMP module
  *
  * @return     None
  *
  * @details    The comparators have its own trim bits which can be used to calibrate the offset voltage..
  * @note       The ACMP calibration function must be completed before configuring ACMP..
  */
void ACMP_Calibration(ACMP_T *acmp)
{
    g_ACMP_i32ErrCode = 0;

    /* Do calibration for ACMP to decrease the effect of electrical random noise. */
    if (((acmp->CALSR & ACMP_CALSR_DONE0_Msk) == 0) || ((acmp->CALSR & ACMP_CALSR_DONE1_Msk) == 0))
    {
        uint32_t u32Delay = 0;
        /* Unlock protected registers */
        SYS_UnlockReg();

        if (acmp == ACMP01)
        {
            /* Must reset ACMP before ACMP calibration */
            SYS_ResetModule(SYS_ACMP01RST);
        }
        else
        {
            /* Must reset ACMP before ACMP calibration */
            SYS_ResetModule(SYS_ACMP23RST);
        }

        /* Lock protected registers */
        SYS_LockReg();


        /* Calibration ACMP0/ACMP2*/
        acmp->CTL[0] |= ACMP_CTL_ACMPEN_Msk;

        /* MUST enable CRV and set NEGSEL to CRV for ACMP calibration. */
        acmp->VREF = (ACMP_VREF_CRV0EN_Msk | ACMP_VREF_CRV0SSEL_Msk);
        acmp->CTL[0] = (acmp->CTL[0] & ~(ACMP_CTL_NEGSEL_Msk)) | (ACMP_CTL_NEGSEL_CRV);

        acmp->CALCTL |= ACMP_CALCTL_CALTRG0_Msk;            /* Start to calibration */
        u32Delay = SystemCoreClock;   /* 1 second */

        while ((acmp->CALSR & ACMP_CALSR_DONE0_Msk) == 0)   /* Wait calibration finish */
        {
            if (--u32Delay == 0)
            {
                g_ACMP_i32ErrCode = ACMP_TIMEOUT_ERR;
                break;
            }
        }

        /* Calibration ACMP1/ACMP3 */
        acmp->CTL[1] |= ACMP_CTL_ACMPEN_Msk;

        /* MUST enable CRV and set NEGSEL to CRV for ACMP calibration. */
        acmp->VREF = (ACMP_VREF_CRV1EN_Msk | ACMP_VREF_CRV1SSEL_Msk);
        acmp->CTL[1] = (acmp->CTL[1] & ~(ACMP_CTL_NEGSEL_Msk)) | (ACMP_CTL_NEGSEL_CRV);

        acmp->CALCTL |= ACMP_CALCTL_CALTRG1_Msk;            /* Start to calibration */
        u32Delay = SystemCoreClock; /* 1 second */

        while ((acmp->CALSR & ACMP_CALSR_DONE1_Msk) == 0)  /* Wait calibration finish */
        {
            if (--u32Delay == 0)
            {
                g_ACMP_i32ErrCode = ACMP_TIMEOUT_ERR;
                break;
            }
        }

    }

}
/** @} end of group ACMP_EXPORTED_FUNCTIONS */

/** @} end of group ACMP_Driver */

/** @} end of group Standard_Driver */
