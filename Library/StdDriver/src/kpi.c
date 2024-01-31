/**************************************************************************//**
 * @file     kpi.c
 * @version  V1.00
 * @brief    KPI driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup KPI_Driver KPI Driver
  @{
*/

/** @addtogroup KPI_EXPORTED_FUNCTIONS KPI Exported Functions
  @{
*/

static KPI_KEY_T *s_pKeyQueue = 0;
static volatile uint32_t s_u32MaxKeyCnt = 0;
static volatile uint32_t s_u32FirstKey = 0;
static volatile uint32_t s_u32LastKey = 0;
static volatile uint32_t s_u32RowCnt = 0;
static volatile uint32_t s_u32ColCnt = 0;

__WEAK void KPI_KeyHandler(KPI_KEY_T key)

{
    uint32_t u32Next;
    /* Move last to next available space */
    u32Next = s_u32LastKey + 1;

    if (u32Next >= s_u32MaxKeyCnt)
    {
        u32Next = 0;    // buffer wrap
    }

    if (u32Next == s_u32FirstKey)
    {
        return;    // Queue full
    }

    /* Push key to the queue */
    s_pKeyQueue[s_u32LastKey] = key;
    s_u32LastKey = u32Next;
}

NVT_ITCM void KPI_IRQHandler()
{
    int32_t i, j, idx, r;
    uint32_t u32KeyPress[2], u32KeyRelease[2], status;
    uint32_t row, col, mask;
    KPI_KEY_T key;
    /* cache key events ASAP */
    status = KPI->STATUS;
    u32KeyPress[0] = KPI->KPF[0];
    u32KeyPress[1] = KPI->KPF[1];
    u32KeyRelease[0] = KPI->KRF[0];
    u32KeyRelease[1] = KPI->KRF[1];

    if (status & KPI_STATUS_KIF_Msk)
    {
        /* Get current row/column setting */
        row = s_u32RowCnt;
        col = s_u32ColCnt;

        /* Deal with the key evernts */
        for (i = 0; i < row; i++)
        {
            for (j = 0; j < col; j++)
            {
                /* Identify the specified key bit */
                idx = (i < 4) ? 0 : 1;
                r   = i - idx * 4;
                mask = 1ul << (r * 8 + j);

                /* Key Release */
                if (status & KPI_STATUS_KRIF_Msk)
                {
                    if (u32KeyRelease[idx] & mask)
                    {
                        /* Clean event */
                        KPI->KRF[idx] = mask;
                        /* Record the key */
                        key.x = i;
                        key.y = j;
                        key.st = KPI_RELEASE;
                        /* call handler */
                        KPI_KeyHandler(key);
                    }
                }
            }
        }

        /* Deal with the key evernts */
        for (i = 0; i < row; i++)
        {
            for (j = 0; j < col; j++)
            {
                /* Identify the specified key bit */
                idx = (i < 4) ? 0 : 1;
                r   = i - idx * 4;
                mask = 1ul << (r * 8 + j);

                /* Key Press */
                if (status & KPI_STATUS_KPIF_Msk)
                {
                    if (u32KeyPress[idx] & mask)
                    {
                        /* Clean event */
                        KPI->KPF[idx] = mask;
                        /* Record the key */
                        key.x = i;
                        key.y = j;
                        key.st = KPI_PRESS;
                        /* call handler */
                        KPI_KeyHandler(key);
                    }
                }
            }
        }
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    status = KPI->STATUS;
}


/**
 *    @brief        Open Keypad interface
 *
 *    @param[in]    u32Rows         The number of key rows for key scan. it could be 2 ~ 6.
 *    @param[in]    u32Columns      The number of key columns for key scan. it could be 1 ~ 8.
 *    @param[in]    pkeyQueue       The FIFO queue of the key press/release status.
 *    @param[in]    u32MaxKeyCnt    Maximum key counts in the key queue.
 *
 *    @retval       0   Sucessful
 *    @retval       -1  Failure
 *
 *    @details      The function is used to set row and column of keypad and start to key scan.
 */
int32_t KPI_Open(uint32_t u32Rows, uint32_t u32Columns, KPI_KEY_T *pkeyQueue, uint32_t u32MaxKeyCnt)
{
    /* Key ROW limitation */
    if ((u32Rows < 2) || (u32Rows > 6))
    {
        return -1;
    }

    /* Key COLUMN limitation */
    if ((u32Columns < 1) || (u32Columns > 8))
    {
        return -1;
    }

    s_u32RowCnt = u32Rows;
    s_u32ColCnt = u32Columns;
    /* Set KPI */
    KPI->CTL = ((u32Rows - 1) << KPI_CTL_KROW_Pos) | ((u32Columns - 1) << KPI_CTL_KCOL_Pos) |
               KPI_CTL_KIEN_Msk | KPI_CTL_KPIEN_Msk | KPI_CTL_KRIEN_Msk |
               KPI_ROW_SCAN_DELAY4CLK | KPI_COL_SAMPLE_8CLK |
               KPI_CTL_KPEN_Msk;
    /* Set up the queue of key */
    s_pKeyQueue = pkeyQueue;
    s_u32MaxKeyCnt = u32MaxKeyCnt;
    s_u32FirstKey = 0;
    s_u32LastKey = 0;
    return 0;
}

/**
 *    @brief        Close Keypad interface
 *
 *    @details      The function is used to stop and close key pad.
 */

void KPI_Close()
{
    /* Disable Keypad */
    KPI->CTL = 0;
}

/**
 *    @brief        Config KPI scan key timing.
 *
 *    @param[in]    u32PreScale    Row Scan Cycle Pre-scale Value. The divided number is from 1 to 256
 *    @param[in]    u32Debounce    De-bounce Sampling Cycle.
 *                  \ref KPI_COL_SAMPLE_8CLK
 *                  \ref KPI_COL_SAMPLE_16CLK
 *                  \ref KPI_COL_SAMPLE_32CLK
 *                  \ref KPI_COL_SAMPLE_64CLK
 *                  \ref KPI_COL_SAMPLE_128CLK
 *                  \ref KPI_COL_SAMPLE_256CLK
 *                  \ref KPI_COL_SAMPLE_512CLK
 *                  \ref KPI_COL_SAMPLE_1024CLK
 *                  \ref KPI_COL_SAMPLE_2048CLK
 *                  \ref KPI_COL_SAMPLE_4096CLK
 *                  \ref KPI_COL_SAMPLE_8192CLK
 *    @param[in]    u32ScanDelay   Setting delay cycle when row change:
 *                  \ref KPI_ROW_SCAN_DELAY4CLK
 *                  \ref KPI_ROW_SCAN_DELAY8CLK
 *                  \ref KPI_ROW_SCAN_DELAY16CLK
 *                  \ref KPI_ROW_SCAN_DELAY32CLK
 *
 */
void KPI_ConfigKeyScanTiming(uint32_t u32PreScale, uint32_t u32Debounce, uint32_t u32ScanDelay)
{
    KPI->CTL &= ~(KPI_CTL_PSC_Msk | KPI_CTL_DBCLKSEL_Msk | KPI_CTL_ROWDLY_Msk);
    KPI->CTL |= ((u32PreScale - 1) << KPI_CTL_PSC_Pos) | u32Debounce | u32ScanDelay;
}

/**
 *    @brief        Detect any key press
 *
 *
 *    @retval       1   Key pressed
 *    @retval       0   No key pressed
 *
 *    @details      The function is used to check if any key pressed.
 */
int32_t KPI_kbhit()
{
    if (s_u32FirstKey != s_u32LastKey)
    {
        return 1;
    }

    return 0;
}


/**
 *    @brief        Get pressed/released key
 *
 *    @return       return the pressed key information. If no key pressed, return key index is 0xff, 0xff.
 *
 *    @details      The function is get the key pressed or key released.
 */
KPI_KEY_T KPI_GetKey()
{
    KPI_KEY_T key = {0xff, 0xff, 0xffff};

    /* Check if queue is empty */
    if (s_u32FirstKey != s_u32LastKey)
    {
        /* Pop the key from queue */
        key = s_pKeyQueue[s_u32FirstKey++];

        /* Wrap around check */
        if (s_u32FirstKey >= s_u32MaxKeyCnt)
        {
            s_u32FirstKey = 0;
        }
    }

    return key;
}

/** @} end of group KPI_EXPORTED_FUNCTIONS */
/** @} end of group KPI_Driver */
/** @} end of group Standard_Driver */
