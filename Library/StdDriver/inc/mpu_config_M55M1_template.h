/**************************************************************************//**
 * @file     mpu_config_M55M1_template.h
 * @version  V1.00
 * @brief    Initial Setup for MPU region for M55M1
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __MPU_CONFIG_M55M1_H__
#define __MPU_CONFIG_M55M1_H__

/*
 * Copy from Library\StdDriver\inc\mpu_config_M55M1_template.h
 * Rename to mpu_config_M55M1.h
 * Add mpu_config_M55M1.h folder to include path
 */

#define MPU_REGIONS_MAX     8UL                 /* Max. number of MPU regions */
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <h>Memory Protection Unit (MPU) Regions
// <h>Pre-defined MPU attribute
*/
/*
// <q.0> 0: eMPU_ATTR_DEV_nGnRnE
// <i> Device memory type non Gathering, non Re-ordering, non Early Write Acknowledgement
// <q.1> 1: eMPU_ATTR_DEV_nGnRE
// <i> Device memory type non Gathering, non Re-ordering, Early Write Acknowledgement
// <q.2> 2: eMPU_ATTR_DEV_nGRE
// <i> Device memory type non Gathering, Re-ordering, Early Write Acknowledgement
// <q.3> 3: eMPU_ATTR_DEV_GRE
// <i> Device memory type Gathering, Re-ordering, Early Write Acknowledgement
// <q.4> 4: eMPU_ATTR_NON_CACHEABLE
// <i> Attribute for non-cacheable normal memory
// <q.5> 5: eMPU_ATTR_CACHEABLE_WTRA
// <i> Attribute for cacheable normal memory with Non-transient, Write-Through, Read-allocate, Not Write-allocate
// <q.6> 6: eMPU_ATTR_CACHEABLE_WBWARA
// <i> Attribute for cacheable normal memory with Non-transient, Write-Back, Read-allocate, Write-allocate
*/
#define MPU_INIT_MEM_ATTRS  0x7FUL
/*
// </h>
*/
/*
//   <e> MPU Region 0 (Lowest priority)
//   <i> Setup MPU Region 0 Attributes
//   <i> Default Set to EBI Memory Space
*/
#define MPU_INIT_REGION0    1UL
/*
//     <o>Base Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE0      0x60000000UL       /* Base address of MPU region 0 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE0      0x00300000UL       /* Size of MPU region 0 */
#define MPU_INIT_LIMIT0     (MPU_INIT_BASE0 + MPU_INIT_SIZE0 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR0       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR0   0xCUL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION0 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE0 < 32UL) || ((MPU_INIT_SIZE0 % 32UL) != 0UL) || ((MPU_INIT_BASE0 % 32UL) != 0UL))
        #error "MPU region 0 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR0) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 0 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 1
//   <i> Setup MPU Region 1 Attributes
//   <i> Default Set to Aliased SRAM + HyperRAM Memory Space
*/
#define MPU_INIT_REGION1    1UL
/*
//     <o>Base Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE1      0x81F00000UL       /* Base address of MPU region 1 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE1      0x02100000UL       /* Size of MPU region 1 */
#define MPU_INIT_LIMIT1     (MPU_INIT_BASE1 + MPU_INIT_SIZE1 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR1       0x6UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR1   0x4UL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION1 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE1 < 32UL) || ((MPU_INIT_SIZE1 % 32UL) != 0UL) || ((MPU_INIT_BASE1 % 32UL) != 0UL))
        #error "MPU region 1 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR1) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 1 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 2
//   <i> Setup MPU Region 2 Attributes
*/
#define MPU_INIT_REGION2    0UL
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE2      0x00000000UL       /* Base address of MPU region 2 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE2      0x00100000UL       /* Size of MPU region 2 */
#define MPU_INIT_LIMIT2     (MPU_INIT_BASE2 + MPU_INIT_SIZE2 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR2       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR2   0xCUL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION2 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE2 < 32UL) || ((MPU_INIT_SIZE2 % 32UL) != 0UL) || ((MPU_INIT_BASE2 % 32UL) != 0UL))
        #error "MPU region 2 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR2) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 2 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 3
//   <i> Setup MPU Region 3 Attributes
*/
#define MPU_INIT_REGION3    0UL
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE3      0x00000000UL       /* Base address of MPU region 3 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE3      0x00010000UL       /* Size of MPU region 3 */
#define MPU_INIT_LIMIT3     (MPU_INIT_BASE3 + MPU_INIT_SIZE3 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR3       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR3   0x0UL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION3 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE3 < 32UL) || ((MPU_INIT_SIZE3 % 32UL) != 0UL) || ((MPU_INIT_BASE3 % 32UL) != 0UL))
        #error "MPU region 3 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR3) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 3 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 4
//   <i> Setup MPU Region 4 Attributes
*/
#define MPU_INIT_REGION4    0UL
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE4      0x00000000UL       /* Base address of MPU region 4 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE4      0x00010000UL       /* Size of MPU region 4 */
#define MPU_INIT_LIMIT4     (MPU_INIT_BASE4 + MPU_INIT_SIZE4 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR4       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR4   0x0UL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION4 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE4 < 32UL) || ((MPU_INIT_SIZE4 % 32UL) != 0UL) || ((MPU_INIT_BASE4 % 32UL) != 0UL))
        #error "MPU region 4 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR4) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 4 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 5
//   <i> Setup MPU Region 5 Attributes
*/
#define MPU_INIT_REGION5    0UL
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE5      0x00000000UL       /* Base address of MPU region 5 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE5      0x00010000UL       /* Size of MPU region 5 */
#define MPU_INIT_LIMIT5     (MPU_INIT_BASE5 + MPU_INIT_SIZE5 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR5       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR5   0x0UL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION5 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE5 < 32UL) || ((MPU_INIT_SIZE5 % 32UL) != 0UL) || ((MPU_INIT_BASE5 % 32UL) != 0UL))
        #error "MPU region 5 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR5) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 5 has invalid memory attribute !"
    #endif
#endif

/*
//   <e> MPU Region 6 (Highest priority)
//   <i> Setup MPU Region 6 Attributes
*/
#define MPU_INIT_REGION6    0UL
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define MPU_INIT_BASE6      0x00000000UL       /* Base address of MPU region 6 */
/*
//     <o>Region Size <0x20-0xFFFFFFFF:0x20>
*/
#define MPU_INIT_SIZE6      0x00010000UL       /* Size of MPU region 6 */
#define MPU_INIT_LIMIT6     (MPU_INIT_BASE6 + MPU_INIT_SIZE6 - 1UL)
/*
//    <o>  Memory attribute
//      <0=> Device-nGnRnE
//      <1=> Device-nGnRE
//      <2=> Device-nGRE
//      <3=> Device-GRE
//      <4=> Non-cacheable
//      <5=> Cacheable-WTRA
//      <6=> Cacheable-WBWARA
*/
#define MPU_MEM_ATTR6       0x0UL
/*
//  <h> Access attribute
//   <q.1> Read-Only
//   <q.2> Non-Privileged
//   <q.3> Non-executable
*/
#define MPU_MEM_ACC_ATTR6   0x0UL
/*
//   </h>
*/
/*
//   </e>
*/
#if (MPU_INIT_REGION6 == 1UL) /* Check region config */
    #if ((MPU_INIT_SIZE6 < 32UL) || ((MPU_INIT_SIZE6 % 32UL) != 0UL) || ((MPU_INIT_BASE6 % 32UL) != 0UL))
        #error "MPU region 6 has invalid base address or size config !"
    #endif

    #if (((1UL << MPU_MEM_ATTR6) & (MPU_INIT_MEM_ATTRS)) == 0UL)
        #error "MPU region 6 has invalid memory attribute !"
    #endif
#endif

//   <n> MPU Region 7 is reserved for Non-cacheable region in M55M1 BSP
/*
// </h>
*/

typedef enum
{
    eMPU_ATTR_DEV_nGnRnE,
    eMPU_ATTR_DEV_nGnRE,
    eMPU_ATTR_DEV_nGRE,
    eMPU_ATTR_DEV_GRE,
    eMPU_ATTR_NON_CACHEABLE,
    eMPU_ATTR_CACHEABLE_WTRA,
    eMPU_ATTR_CACHEABLE_WBWARA,
} E_MPU_ATTR;

typedef enum
{
    eMPU_RBAR_SH,
    eMPU_RBAR_RO,
    eMPU_RBAR_NP,
    eMPU_RBAR_XN,
} E_MPU_RBAR;

#define MPU_INIT_REGIONS            ((MPU_INIT_REGION0 << 0UL) | (MPU_INIT_REGION1 << 1UL) | (MPU_INIT_REGION2 << 2UL) | (MPU_INIT_REGION3 << 3UL) | \
                                     (MPU_INIT_REGION4 << 4UL) | (MPU_INIT_REGION5 << 5UL) | (MPU_INIT_REGION6 << 6UL))
#define MPU_INIT_REGION(i)          (MPU_INIT_REGION##i)
#define MPU_INIT_FIXED(i)           (MPU_INIT_FIXED##i)
#define MPU_INIT_BASE(i)            (MPU_INIT_BASE##i)
#define MPU_INIT_SIZE(i)            (MPU_INIT_SIZE##i)
#define MPU_INIT_LIMIT(i)           (MPU_INIT_LIMIT##i)
#define MPU_MEM_ATTR(i)             (MPU_MEM_ATTR##i)
#define MPU_MEM_ACC_ATTR(i, index)  (((MPU_MEM_ACC_ATTR##i) >> (uint32_t)(index)) & 0x1UL)
#define MPU_INIT_RBAR(i, u32Base)   ((uint32_t)(ARM_MPU_RBAR(u32Base, ARM_MPU_SH_NON, MPU_MEM_ACC_ATTR(i, eMPU_RBAR_RO), MPU_MEM_ACC_ATTR(i, eMPU_RBAR_NP), MPU_MEM_ACC_ATTR(i, eMPU_RBAR_XN))))

#endif  // __MPU_CONFIG_M55M1_H__
