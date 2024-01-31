/**************************************************************************//**
 * @file     kpi.h
 * @version  V1.00
 * @brief    KPI driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __KPI_H__
#define __KPI_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup KPI_Driver KPI Driver
  @{
*/

/** @addtogroup KPI_EXPORTED_CONSTANTS KPI Exported Constants
  @{
*/

#define KPI_MAX_ROW     6
#define KPI_MAX_COL     8
#define KPI_MAX_KEYS    (KPI_MAX_ROW * KPI_MAX_COL)

#define KPI_PRESS   0
#define KPI_RELEASE 1

typedef struct
{
    uint8_t     x;
    uint8_t     y;
    uint16_t    st;
} KPI_KEY_T;


/*---------------------------------------------------------------------------------------------------------*/
/*  KPI_CTL constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define KPI_ROW_SCAN_DELAY4CLK      (0x0 << KPI_CTL_ROWDLY_Pos) /*!< Delay cycle when row change */
#define KPI_ROW_SCAN_DELAY8CLK      (0x1 << KPI_CTL_ROWDLY_Pos) /*!< Delay cycle when row change */
#define KPI_ROW_SCAN_DELAY16CLK     (0x2 << KPI_CTL_ROWDLY_Pos) /*!< Delay cycle when row change */
#define KPI_ROW_SCAN_DELAY32CLK     (0x3 << KPI_CTL_ROWDLY_Pos) /*!< Delay cycle when row change */

#define KPI_COL_SAMPLE_8CLK         (0x3 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_16CLK        (0x4 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_32CLK        (0x5 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_64CLK        (0x6 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_128CLK       (0x7 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_256CLK       (0x8 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_512CLK       (0x9 << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_1024CLK      (0xA << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_2048CLK      (0xB << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_4096CLK      (0xC << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */
#define KPI_COL_SAMPLE_8192CLK      (0xD << KPI_CTL_DBCLKSEL_Pos) /*!< Scan in De-bounce Sampling Cycle Selection */

/** @} end of group KPI_EXPORTED_CONSTANTS */

/** @addtogroup KPI_EXPORTED_FUNCTIONS KPI Exported Functions
  @{
*/

int32_t KPI_Open(uint32_t u32Rows, uint32_t u32Columns, KPI_KEY_T *pkeyQueue, uint32_t u32MaxKeyCnt);
void KPI_Close(void);
void KPI_ConfigKeyScanTiming(uint32_t u32PreScale, uint32_t u32Debounce, uint32_t u32ScanDelay);
int32_t KPI_kbhit(void);
KPI_KEY_T KPI_GetKey(void);

/** @} end of group KPI_EXPORTED_FUNCTIONS */
/** @} end of group KPI_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __KPI_H__ */
