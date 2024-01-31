/******************************************************************************
 * @file     BUFCTRL.h
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 17/07/28 10:04a $
 * @brief   This header file contain macro define for buffer control.
 *          Include this header file in the main.c file.
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//#include "Platform.h"

#ifndef __BUFCTRL_H__
#define __BUFCTRL_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Define structure                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    uint16_t  u16DataCount;
    uint16_t  u16BufCount;
    uint16_t  u16WriteIdx;
    uint16_t  u16ReadIdx;
    int32_t  *pai32Buf;
} S_BUFCTRL;

/**
 * @brief       Check if the buffer control handler is full or not.
 * @param[in]   psBuf the pointer of control handler.
 * @return      Handler is full or not.
 */
#define BUFCTRL_IS_FULL(psBuf)                ((psBuf->u16DataCount>=psBuf->u16BufCount)?1:0)

/**
 * @brief       Check if the buffer control handler is empty or not.
 * @param[in]   psBuf the pointer of control handler.
 * @return      Handler is empty or not.
 */
#define BUFCTRL_IS_EMPTY(psBuf)               ((psBuf->u16DataCount==0)?1:0)

/**
 * @brief       Get the current data count in control handler.
 * @param[in]   psBuf the pointer of control handler.
 * @return      The data count of handler.
 */
#define BUFCTRL_GET_COUNT(psBuf)              (psBuf->u16DataCount)

/**
 * @brief       Initialize the buffer control handler.
 * @param[in]   psBuf the pointer of control handler.
 * @return      None
 */
__STATIC_INLINE void BUFCTRL_CFG(volatile S_BUFCTRL *psBuf, int32_t *pi32Data, uint16_t u16Count)
{
    psBuf->u16DataCount = 0;
    psBuf->u16WriteIdx = 0;
    psBuf->u16ReadIdx = 0;
    psBuf->u16BufCount = u16Count;
    psBuf->pai32Buf = pi32Data;
}
/**
 * @brief       Write data to buffer.
 * @param[in]   psBuf the pointer of control handler.
 * @param[in]   u32Data the 32-bit data to write.
 * @return      None
 */
__STATIC_INLINE void BUFCTRL_WRITE(volatile S_BUFCTRL *psBuf, int32_t i32Data)
{
    if (psBuf->u16DataCount < psBuf->u16BufCount)
    {
        psBuf->pai32Buf[psBuf->u16WriteIdx] = i32Data;
        psBuf->u16DataCount++;

        if ((psBuf->u16WriteIdx += 1) >= psBuf->u16BufCount)
        {
            psBuf->u16WriteIdx = 0;
        }
    }
}
/**
 * @brief       Read data from buffer to the designated pointer.
 * @param[in]   psBuf the pointer of control handler.
 * @param[in]   pu32Data the pointer to store the data.
 * @return      None
 */
__STATIC_INLINE void BUFCTRL_READ(volatile S_BUFCTRL *psBuf, int32_t *pi32Data)
{
    if (psBuf->u16DataCount > 0)
    {
        *pi32Data = psBuf->pai32Buf[psBuf->u16ReadIdx];
        psBuf->u16DataCount--;

        if ((psBuf->u16ReadIdx += 1) >= psBuf->u16BufCount)
        {
            psBuf->u16ReadIdx = 0;
        }
    }
}
#endif
