/**************************************************************************//**
 * @file    ACI_SinCos.c
 * @version V1.00
 * @brief   Helper function for Nuvoton sin/cos ACI feature
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "ACI_CDE.h"
#include "arm_math.h"

typedef int32_t fixed_point_t;

static fixed_point_t float_to_fixed(float fInput, uint32_t u32FractionalBits)
{
    uint64_t u64Temp;
    u64Temp = (1ULL << u32FractionalBits);
    return (fixed_point_t)(roundf(fInput * (float)u64Temp));
}

static float fixed_to_float(fixed_point_t qInput, uint32_t u32FractionalBits)
{
    uint64_t temp;
    temp = (1ULL << u32FractionalBits);
    return ((float) qInput / (float)(temp));
}


/**
  * @brief      This function return the sine of fRadians
  * @param[in]  fRadians
  * @return     sinf of fRadians
  */
float sinf_aci(float fRadians)
{
    fixed_point_t qCordic2Sine;
    fixed_point_t qCordicRadinas;

    //Workaround: hardware limitation on cordic sin(0).
    if (fRadians == 0)
    {
        //return 0 directly
        return 0;
    }

    //Cordic using Q29(-4.0 ~ 4.0) to represent -PI ~ PI (-180~180 degress)
    qCordicRadinas = float_to_fixed(fRadians, 29);
    qCordic2Sine = CDE_CORDIC2_SIN(qCordicRadinas);

    return fixed_to_float(qCordic2Sine, 31);
}

/**
  * @brief      This function return the cosine of fRadians
  * @param[in]  fRadians
  * @return     cosf of fRadians
  */
float cosf_aci(float fRadians)
{
    fixed_point_t qCordic2Cosine;
    fixed_point_t qCordicRadinas;

    //Cordic using Q29(-4.0 ~ 4.0) to represent -PI ~ PI (-180~180 degress)
    qCordicRadinas = float_to_fixed(fRadians, 29);
    qCordic2Cosine = CDE_CORDIC2_COS(qCordicRadinas);

    return fixed_to_float(qCordic2Cosine, 31);
}

