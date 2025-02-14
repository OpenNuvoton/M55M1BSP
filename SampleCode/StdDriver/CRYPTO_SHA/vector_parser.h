/******************************************************************************
 * @file     #include "vector_parser.h".h
 * @brief
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __VECTOR_PARSER_H__
#define __VECTOR_PARSER_H__

extern uint8_t *g_au8ShaData;
extern uint8_t g_au8ShaDigest[64];
extern int32_t g_i32DataLen;

#if(NVT_DCACHE_ON == 1)
    void NVT_SCB_CleanDCache_ShaDataPool(void);
    void NVT_SCB_InvalidateDCache_ShaDataPool(void);
#endif
#endif  /* __VECTOR_PARSER_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
