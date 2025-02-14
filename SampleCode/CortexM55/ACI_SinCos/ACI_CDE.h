#ifndef __ACI_CDE_H__
#define __ACI_CDE_H__

#include "arm_cde.h"

// Cordic1 approximation
#define CDE_CORDIC1_SIN(angle)  __arm_cx2(2, (uint32_t)angle, 0b010)
#define CDE_CORDIC1_COS(angle)  __arm_cx2(2, (uint32_t)angle, 0b011)

// Cordic1 approximation
#define CDE_CORDIC2_SIN(angle)  __arm_cx2(2, (uint32_t)angle, 0b100)
#define CDE_CORDIC2_COS(angle)  __arm_cx2(2, (uint32_t)angle, 0b101)

#endif
