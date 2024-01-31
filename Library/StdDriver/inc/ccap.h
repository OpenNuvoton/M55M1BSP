/**************************************************************************//**
 * @file     ccap.h
 * @version  V1.00
 * @brief    CCAP driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __CCAP_H__
#define __CCAP_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup CCAP_Driver CCAP Driver
    @{
*/

/** @addtogroup CCAP_EXPORTED_CONSTANTS CCAP Exported Constants
    @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* CTL constant definitions                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define CCAP_CTL_CCAPEN             (1ul << CCAP_CTL_CCAPEN_Pos)        /*!< CCAP CTL setting for enabling Camera Capture Interface         \hideinitializer */
#define CCAP_CTL_PKTEN              (1ul << CCAP_CTL_PKTEN_Pos)         /*!< CCAP CTL setting for enabling packet output mode               \hideinitializer */
#define CCAP_CTL_SHUTTER            (1ul << CCAP_CTL_SHUTTER_Pos)       /*!< CCAP CTL setting for enabling shutter mode                     \hideinitializer */
#define CCAP_CTL_UPDATE             (1ul << CCAP_CTL_UPDATE_Pos)        /*!< CCAP CTL setting for enabling update register at new frame     \hideinitializer */
#define CCAP_CTL_RESET              (1ul << CCAP_CTL_VPRST_Pos)         /*!< CCAP CTL setting for capture reset                             \hideinitializer */
#define CCAP_CTL_MY8_MY4            (0ul << CCAP_CTL_MY8_MY4_Pos)       /*!< CCAP CTL setting for 4-bit data I/O interface with Mono        \hideinitializer */
#define CCAP_CTL_MY8_MY4_SWAP       (1ul << CCAP_CTL_MY4_SWAP_Pos)      /*!< CCAP CTL setting for 4-bit data swap I/O interface with Mono   \hideinitializer */
#define CCAP_CTL_MY8_MY8            (1ul << CCAP_CTL_MY8_MY4_Pos)       /*!< CCAP CTL setting for 8-bit data I/O interface with Mono        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* PAR constant definitions                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define CCAP_PAR_INFMT_YUV422           (0ul << CCAP_PAR_INFMT_Pos)         /*!< CCAP PAR setting for Sensor Input Data YUV422 Format       \hideinitializer */
#define CCAP_PAR_INFMT_YUV422_          (2ul << CCAP_PAR_INFMT_Pos)         /*!< CCAP PAR setting for Sensor Input Data YUV422 Format       \hideinitializer */
#define CCAP_PAR_INFMT_RGB565           (1ul << CCAP_PAR_INFMT_Pos)         /*!< CCAP PAR setting for Sensor Input Data RGB565 Format       \hideinitializer */
#define CCAP_PAR_INFMT_RGB888           (3ul << CCAP_PAR_INFMT_Pos)         /*!< CCAP PAR setting for Sensor Input Data RGB888 Format       \hideinitializer */

#define CCAP_PAR_SENTYPE_CCIR601        (0ul << CCAP_PAR_SENTYPE_Pos)       /*!< CCAP PAR setting for Sensor Input CCIR601 Type             \hideinitializer */
#define CCAP_PAR_SENTYPE_CCIR656        (1ul << CCAP_PAR_SENTYPE_Pos)       /*!< CCAP PAR setting for Sensor Input CCIR656 Type             \hideinitializer */

#define CCAP_PAR_INDATORD_YUYV          (0x0ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, YUYV         \hideinitializer */
#define CCAP_PAR_INDATORD_YVYU          (0x1ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, YVYU         \hideinitializer */
#define CCAP_PAR_INDATORD_UYVY          (0x2ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, UYVY         \hideinitializer */
#define CCAP_PAR_INDATORD_VYUY          (0x3ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, VYUY         \hideinitializer */

#define CCAP_PAR_INDATORD_RGGB          (0x0ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, Byte0: R[4:0] G[5:3], Byte1 G[2:0] B[4:0]    \hideinitializer */
#define CCAP_PAR_INDATORD_BGGR          (0x1ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, Byte0: B[4:0] G[5:3], Byte1 G[2:0] R[4:0]    \hideinitializer */
#define CCAP_PAR_INDATORD_GBRG          (0x2ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, Byte0: G[2:0] B[4:0], Byte1 R[4:0] G[5:3]    \hideinitializer */
#define CCAP_PAR_INDATORD_GRBG          (0x3ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, Byte0: G[2:0] R[4:0], Byte1 B[4:0] G[5:3]    \hideinitializer */

#define CCAP_PAR_INDATORD_RGB888_RGB    (0x0ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 RGB   \hideinitializer */
#define CCAP_PAR_INDATORD_RGB888_RBG    (0x1ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 RBG   \hideinitializer */
#define CCAP_PAR_INDATORD_RGB888_GRB    (0x2ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 GRB   \hideinitializer */
#define CCAP_PAR_INDATORD_RGB888_GBR    (0x3ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 GBR   \hideinitializer */
#define CCAP_PAR_INDATORD_RGB888_BRG    (0x4ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 BRG   \hideinitializer */
#define CCAP_PAR_INDATORD_RGB888_BGR    (0x5ul << CCAP_PAR_INDATORD_Pos)    /*!< CCAP PAR setting for Sensor Input Data Order, RGB888 BGR   \hideinitializer */

#define CCAP_PAR_OUTFMT_YUV422          (0x0ul << CCAP_PAR_OUTFMT_Pos)      /*!< CCAP PAR setting for Image Data YUV422 Format Output to System Memory                      \hideinitializer */
#define CCAP_PAR_OUTFMT_ONLY_Y          (0x1ul << CCAP_PAR_OUTFMT_Pos)      /*!< CCAP PAR setting for Image Data ONLY_Y Format Output to System Memory                      \hideinitializer */
#define CCAP_PAR_OUTFMT_RGB555          (0x2ul << CCAP_PAR_OUTFMT_Pos)      /*!< CCAP PAR setting for Image Data RGB555 Format Output to System Memory                      \hideinitializer */
#define CCAP_PAR_OUTFMT_RGB565          (0x3ul << CCAP_PAR_OUTFMT_Pos)      /*!< CCAP PAR setting for Image Data RGB565 Format Output to System Memory                      \hideinitializer */
/*!< CCAP PAR/PARM setting for Image Data RGB888 UINT8 Format Output to System Memory                \hideinitializer */
#define CCAP_PAR_OUTFMT_RGB888_U8       ((1 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x0ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_BGR888_U8       ((0 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x0ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_ARGB888_U8      ((3 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x0ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_BGRA888_U8      ((2 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x0ul << CCAP_PAR_OUTFMT_Pos))
/*!< CCAP PAR/PARM setting for Image Data RGB888 INT8  Format Output to System Memory                \hideinitializer */
#define CCAP_PAR_OUTFMT_RGB888_I8       ((1 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x1ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_BGR888_I8       ((0 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x1ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_ARGB888_I8      ((3 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x1ul << CCAP_PAR_OUTFMT_Pos))
#define CCAP_PAR_OUTFMT_BGRA888_I8      ((2 << CCAP_PARM_RGB888OUTORD_Pos) | CCAP_PARM_OUTFMTH_Msk | (0x1ul << CCAP_PAR_OUTFMT_Pos))

#define CCAP_PAR_VSP_LOW                (0x0ul << CCAP_PAR_VSP_Pos)         /*!< CCAP PAR setting for Sensor Vsync Polarity                 \hideinitializer */
#define CCAP_PAR_VSP_HIGH               (0x1ul << CCAP_PAR_VSP_Pos)         /*!< CCAP PAR setting for Sensor Vsync Polarity                 \hideinitializer */
#define CCAP_PAR_HSP_LOW                (0x0ul << CCAP_PAR_HSP_Pos)         /*!< CCAP PAR setting for Sensor Hsync Polarity                 \hideinitializer */
#define CCAP_PAR_HSP_HIGH               (0x1ul << CCAP_PAR_HSP_Pos)         /*!< CCAP PAR setting for Sensor Hsync Polarity                 \hideinitializer */
#define CCAP_PAR_PCLKP_LOW              (0x0ul << CCAP_PAR_PCLKP_Pos)       /*!< CCAP PAR setting for Sensor Pixel Clock Polarity           \hideinitializer */
#define CCAP_PAR_PCLKP_HIGH             (0x1ul << CCAP_PAR_PCLKP_Pos)       /*!< CCAP PAR setting for Sensor Pixel Clock Polarity           \hideinitializer */
#define CCAP_PAR_CCIR656_NO_BLANK       (0x1ul << CCAP_PAR_FBB_Pos)         /*!< CCAP PAR setting for Sensor Pixel Clock Polarity           \hideinitializer */

#define CCAP_MD_TRIG_LPTMR0             (0x0ul << CCAP_MDTRG_WK_TRGSEL_Pos) /*!< CCAP Motion Detection trigger source for LPTMR0            \hideinitializer */
#define CCAP_MD_TRIG_LPTMR1             (0x1ul << CCAP_MDTRG_WK_TRGSEL_Pos) /*!< CCAP Motion Detection trigger source for LPTMR1            \hideinitializer */
#define CCAP_MD_TRIG_TTMR0              (0x2ul << CCAP_MDTRG_WK_TRGSEL_Pos) /*!< CCAP Motion Detection trigger source for TTMR0             \hideinitializer */
#define CCAP_MD_TRIG_TTMR1              (0x3ul << CCAP_MDTRG_WK_TRGSEL_Pos) /*!< CCAP Motion Detection trigger source for TTMR1             \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* CCAP INT constant definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CCAP_INT_VIEN_ENABLE            CCAP_INTEN_VIEN_Msk                   /*!< CCAP INT setting for Video Frame End Interrupt enable            \hideinitializer */
#define CCAP_INT_MEIEN_ENABLE           CCAP_INTEN_MEIEN_Msk                  /*!< CCAP INT setting for Bus Master Transfer Error Interrupt enable  \hideinitializer */
#define CCAP_INT_ADDRMIEN_ENABLE        CCAP_INTEN_ADDRMIEN_Msk               /*!< CCAP INT setting for Memory Address Match Interrupt enable       \hideinitializer */
#define CCAP_INT_MDIEN_MODE1_ENABLE     (1ul << CCAP_INTEN_MDIEN_Pos)         /*!< CCAP INT setting for Motion Detection Modes Interrupt enable     \hideinitializer */
#define CCAP_INT_MDIEN_MODE2_ENABLE     (2ul << CCAP_INTEN_MDIEN_Pos)         /*!< CCAP INT setting for Motion Detection Modes Interrupt enable     \hideinitializer */

#define CCAP_INTSTS_MDINTF_MODE1_Msk    (1ul << CCAP_INTSTS_MDINTF_Pos)       /*!< CCAP INT status for Motion Detection Modes 1                     \hideinitializer */
#define CCAP_INTSTS_MDINTF_MODE2_Msk    (2ul << CCAP_INTSTS_MDINTF_Pos)       /*!< CCAP INT status for Motion Detection Modes 2                     \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* CCAP Motion Detection definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define CCAP_MD_CELL_WIDTH              (80ul)
#define CCAP_MD_CELL_HEIGHT             (60ul)
#define CCAP_MD_WIDTH                   (320ul)
#define CCAP_MD_HEIGHT                  (240ul)
#define CCAP_MD_WINDOW_CNT              (16ul)
#define CCAP_MD_MAX_WINDOW_SAD          (0x12AD4ul)
#define CCAP_MD_MAX_TOTAL_SAD           (CCAP_MD_MAX_WINDOW_SAD * CCAP_MD_WINDOW_CNT)

/*---------------------------------------------------------------------------------------------------------*/
/* CCAP Error Code definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CCAP_OK                         ( 0L)   /*!< CCAP operation OK                         */
#define CCAP_ERR_FAIL                   (-1L)   /*!< CCAP operation failed                     */
#define CCAP_ERR_TIMEOUT                (-2L)   /*!< CCAP operation abort due to timeout error */
#define CCAP_ERR_INVALID_MD_REGION      (-3L)   /*!< CCAP invalid motion detection region      */
#define CCAP_ERR_INVALID_PARAM          (-4L)   /*!< CCAP invalid parameter                    */

/** @} end of group CCAP_EXPORTED_CONSTANTS */

/** @addtogroup CCAP_EXPORTED_FUNCTIONS CCAP Exported Functions
    @{
*/

/**
 * @brief     Is CCAP module stopped
 *
 * @param     None
 *
 * @retval    0   CCAP module is enabled
 * @retval    1   CCAP module is disabled(stopped)
 *
 * @details   Check Camera Capture Interface module Enable or Disable(stopped)
 */
#define CCAP_IS_STOPPED()  ((CCAP->CTL & CCAP_CTL_CCAPEN_Msk) ? 0 : 1)

/**
 * @brief     Clear CCAP interrupt flag
 *
 * @param[in] u32IntMask    Interrupt flags settings. It could be
 *                          - \ref CCAP_INTSTS_VINTF_Msk
 *                          - \ref CCAP_INTSTS_MEINTF_Msk
 *                          - \ref CCAP_INTSTS_ADDRMINTF_Msk
 *                          - \ref CCAP_INTSTS_MDINTF_MODE1_Msk
 *                          - \ref CCAP_INTSTS_MDINTF_MODE2_Msk
 *
 * @details   Clear Camera Capture Interface interrupt flag
 */
#define CCAP_CLR_INT_FLAG(u32IntMask) (CCAP->INTSTS = (u32IntMask))

/**
 * @brief     Get CCAP Interrupt Flag
 *
 * @return    CCAP interrupt flag
 */
#define CCAP_GET_INT_STS() (CCAP->INTSTS)

/**
 * @brief     Get CCAP Wake-up Flag
 *
 * @return    CCAP Wake-up flag
 */
#define CCAP_GET_WAKEUP_FLAG()  ((CCAP->MDTRG_WK >> CCAP_MDTRG_WK_WKF_Pos) & CCAP_MDTRG_WK_WKF_Msk)

/**
 * @brief     Clear CCAP Wake-up Flag
 */
#define CCAP_CLR_WAKEUP_FLAG()  (CCAP->MDTRG_WK = CCAP_MDTRG_WK_WKF_Msk)

/**
 * @brief     Get CCAP Wake-up Source
 *
 * @return    CCAP Wake-up Source
 *            0: LPTMR0
 *            1: LPTMR1
 *            2: TTMR0
 *            3: TTMR1
 */
#define CCAP_GET_WAKEUP_SRC()   ((CCAP->MDTRG_WK >> CCAP_MDTRG_WK_TRGSEL_Pos) & CCAP_MDTRG_WK_TRGSEL_Msk)

/**
 * @brief     Get CCAP Wake-up Function
 *
 * @return    CCAP Wake-up Function Enable State
 *            0: Disabled, 1: Enabled
 */
#define CCAP_GET_WAKEUP_FUNC()  ((CCAP->MDTRG_WK >> CCAP_MDTRG_WK_WKEN_Pos) & CCAP_MDTRG_WK_WKEN_Msk)

/**
 * @brief     Enable CCAP Motion Detection Window
 *
 * @param     u32WindowMsk          Motion Detection Window Bitmask
 *                                  It could be 0x0 ~ 0xFFFF.
 *                                  Bit n is 0 = Motion Detection Window n is disabled.
 *                                  Bit n is 1 = Motion Detection Window n is enabled.
 *                                  (0,  0)----(80,  0)----(160,  0)----(240,  0)---------
 *                                    |           |            |            |            |
 *                                    |     W0     |      W1     |      W2     |      W3     |
 *                                    |           |            |            |            |
 *                                  (0, 60)----(80, 60)----(160, 60)----(240, 60)---------
 *                                    |           |            |            |            |
 *                                    |     4     |      5     |      6     |      7     |
 *                                    |           |            |            |            |
 *                                  (0,120)----(80,120)----(160,120)----(240,120)---------
 *                                    |           |            |            |            |
 *                                    |     8     |      9     |     10     |     11     |
 *                                    |           |            |            |            |
 *                                  (0,180)----(80,180)----(160,180)----(240,180)---------
 *                                    |           |            |            |            |
 *                                    |    12     |     13     |     14     |     15     |
 *                                    |           |            |            |            |
 *                                    -----------------------------------------------(319,239)
 *
 * @details   Enable Camera Capture Interface motion detection window
 */
#define CCAP_MD_ENABLE_WINDOW(u32WindowMsk) (CCAP->MDCTL = (CCAP->MDCTL & ~0xFFFF) | (u32WindowMsk))

/**
 * @brief     Disable CCAP Motion Detection Window
 *
 * @param     u32WindowMsk          Motion Detection Window Bitmask
 *                                  It could be 0x0 ~ 0xFFFF.
 *
 * @details   Disable Camera Capture Interface motion detection window
 */
#define CCAP_MD_DISABLE_WINDOW(u32WindowMsk) (CCAP->MDCTL = (CCAP->MDCTL & ~(u32WindowMsk)))

/**
 * @brief     Set CCAP Motion Detection Trigger Source
 *
 * @param[in] u32TrigSrc            Motion Detection Trigger Source. It could be
 *                                  - \ref CCAP_MD_TRIG_LPTMR0
 *                                  - \ref CCAP_MD_TRIG_LPTMR1
 *                                  - \ref CCAP_MD_TRIG_TTMR0
 *                                  - \ref CCAP_MD_TRIG_TTMR1
 * @param[in] bWakeUp               CCAP interrupt signal will generate a wake-up trigger event to CPU
 *
 * @details   Set Camera Capture Interface motion detection trigger source
 *
 */
#define CCAP_MD_SET_TRIG_SRC(u32TrigSrc, bWakeUp) \
    (CCAP->MDTRG_WK = (CCAP->MDTRG_WK & ~(CCAP_MDTRG_WK_WKEN_Msk | CCAP_MDTRG_WK_TRGSEL_Msk)) | ((u32TrigSrc) | ((bWakeUp) << CCAP_MDTRG_WK_WKEN_Pos)))

/**
 * @brief     Set CCAP Motion Detection Total Threshold (Summary of all windows)
 *
 * @param[in] u32TotalThreshold     Motion Detection Total Threshold
 *                                  It could be 0 (Most sensitive) ~ 0x12AD40 (Mode 1 interrupt is disabled).
 *
 * @details   Set Camera Capture Interface motion detection total threshold
 *            When Motion Detection Total SAD (Sum of Absolute Differences) (CCAP_MDTSAD) is greater than
 *            Motion Detection Total Threshold (CCAP_MDTTH),
 *            CCAP will trigger Motion Detection Mode 1 interrupt.
 *
 */
#define CCAP_MD_SET_TOTAL_THRESHOLD(u32TotalThreshold) (CCAP->MDTTH = (u32TotalThreshold))

/**
 * @brief     Get CCAP Motion Detection Total Sum of Absolute Differences (Summary of all windows)
 */
#define CCAP_MD_GET_TOTAL_SAD()         (CCAP->MDTSAD)

/**
 * @brief     Set CCAP Motion Detection Window Overflow Count Threshold
 *
 * @param[in] u32WinOverflowCnt     Motion Detection Total Window Overflow Count Threshold
 *                                  It could be 0x0 (Most sensitive) ~ 0xF (Less sensitive).
 *
 *
 * @details   Set motion detection window overflow count threshold
 *            When Motion Detection Window Overflow counter (CCAP_MDWOC) is greater than
 *            Motion Detection Window Overflow Counter Threshold (CCAP_MDWOCTH),
 *            CCAP will trigger Motion Detection Mode 2 interrupt.
 */
#define CCAP_MD_SET_OVERFLOW_WIN_THRESHOLD(u32WinOverflowCnt) (CCAP->MDWOCTH = (u32WinOverflowCnt))

/**
 * @brief     Get CCAP Motion Detection Window Overflow Count
 */
#define CCAP_MD_GET_OVERFLOW_WIN_CNT()  (CCAP->MDWOC)

/**
 * @brief     Set CCAP Motion Detection Window Threshold (Sensitivity level of single window)
 *
 * @param[in] u32WinIdx:        Motion Detection Window Index. It could be 0x0 ~ 0xF.
 * @param[in] u32Threshold:     Motion Detection Window Threshold
 *                              It could be 0 (Most sensitive) ~ 0x12AD4 (This window is disabled).
 *
 * @details   Set Camera Capture Interface motion detection total threshold
 */
#define CCAP_MD_SET_WIN_THRESHOLD(u32WinIdx, u32Threshold) (CCAP->MDWTH[u32WinIdx] = (u32Threshold))

/**
 * @brief     Get CCAP Motion Detection Sum of Absolute Differences of sigle window
 *
 * @param[in] u32WinIdx:        Motion Detection Window Index. It could be 0x0 ~ 0xF.
 */
#define CCAP_MD_GET_WIN_SAD(u32WinIdx)  (CCAP->MDWSAD[u32WinIdx])

/**
 * @brief     Set Camera Capture Interface Frame Rate Scale Down Divider
 *
 * @param[in] u8Num:            Scale down numerator.   It could be 0x1 ~ 0x63.
 * @param[in] u8Den:            Scale down denominator. It could be 0x1 ~ 0x63 and u8Num must <= u8Den.
 * @details   The output image frame rate will be equal to input image frame rate * (u8Num/u8Den)
 */
#define CCAP_FR_SET_SCALEDOWN_RATE(u8Num, u8Den)     (CCAP->FRCTL = (((u8Num) & 0x3F) << CCAP_FRCTL_FRN_Pos) | (((u8Den) & 0x3F) << CCAP_FRCTL_FRM_Pos))

/**
 * @brief     Get Camera Capture Interface Frame Rate Scale Down Rate (1 ~ 100 %)
 */
#define CCAP_FR_GET_SCALEDOWN_RATE()                (((((CCAP->FRCTL >> CCAP_FRCTL_FRN_Pos) & 0x3F) * 100) / ((CCAP->FRCTL >> CCAP_FRCTL_FRM_Pos) & 0x3F)))

/**
 * @brief      Set System Memory Packet Base Address
 *
 * @param[in]  u32Address: Set CCAP_PKTBA0 register. It should be 0x0 ~ 0xFFFFFFFF.
 *
 * @details    This function is used to set System Memory Packet Base Address 0 Register.
 */
__STATIC_INLINE void CCAP_SetPacketBuf(uint32_t u32Address)
{
    CCAP->PKTBA0 = u32Address;
    CCAP->CTL |= CCAP_CTL_UPDATE_Msk;
}

/**
 * @brief      Close Camera Capture Interface
 */
__STATIC_INLINE void CCAP_Close(void)
{
    CCAP->CTL &= ~CCAP_CTL_CCAPEN;
}

/**
 * @brief      Enable CCAP Interrupt
 *
 * @param[in]  u32IntMask  Interrupt settings. It could be
 *                         - \ref CCAP_INT_VIEN_ENABLE
 *                         - \ref CCAP_INT_MEIEN_ENABLE
 *                         - \ref CCAP_INT_ADDRMIEN_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE1_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE2_ENABLE
 *
 * @details    This function is used to enable Video Frame End Interrupt,
 *             Bus Master Transfer Error Interrupt, Memory Address Match Interrupt
 *             and Motion Detection Modes Interrupt.
 */
__STATIC_INLINE void CCAP_EnableInt(uint32_t u32IntMask)
{
    CCAP->INTEN = (CCAP->INTEN & ~(CCAP_INTEN_VIEN_Msk | CCAP_INTEN_MEIEN_Msk | CCAP_INTEN_ADDRMIEN_Msk | CCAP_INTEN_MDIEN_Msk))
                  | u32IntMask;
}

/**
 * @brief      Disable CCAP Interrupt
 *
 * @param[in]  u32IntMask  Interrupt settings. It could be
 *                         - \ref CCAP_INT_VIEN_ENABLE
 *                         - \ref CCAP_INT_MEIEN_ENABLE
 *                         - \ref CCAP_INT_ADDRMIEN_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE1_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE2_ENABLE
 *
 * @details    This function is used to disable Video Frame End Interrupt,
 *             Bus Master Transfer Error Interrupt, Memory Address Match Interrupt
 *             and Motion Detection Modes Interrupt.
 */
__STATIC_INLINE void CCAP_DisableInt(uint32_t u32IntMask)
{
    CCAP->INTEN = (CCAP->INTEN & ~(u32IntMask));
}

/**
 * @brief      Get CCAP Interrupt Enable State
 *
 * @param[in]  u32IntMask  Interrupt settings. It could be
 *                         - \ref CCAP_INT_VIEN_ENABLE
 *                         - \ref CCAP_INT_MEIEN_ENABLE
 *                         - \ref CCAP_INT_ADDRMIEN_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE1_ENABLE
 *                         - \ref CCAP_INT_MDIEN_MODE2_ENABLE
 *
 * @details    This function is used to get CCAP interrupt is enabled or not.
 */
__STATIC_INLINE uint32_t CCAP_IsIntEnabled(uint32_t u32IntMask)
{
    return (CCAP->INTEN & u32IntMask);
}

/**
 * @brief      Enable Monochrome CMOS Sensor
 *
 * @param[in]  u32Interface  Data I/O interface setting. It could be
 *                           - \ref CCAP_CTL_MY8_MY4
 *                           - \ref CCAP_CTL_MY8_MY4_SWAP
 *                           - \ref CCAP_CTL_MY8_MY8
 *
 * @details    This function is used to select monochrome CMOS sensor and set data width.
 */
__STATIC_INLINE void CCAP_EnableMono(uint32_t u32Interface)
{
    CCAP->CTL = (CCAP->CTL & ~(CCAP_CTL_MY8_MY4_Msk | CCAP_CTL_MY4_SWAP_Msk)) | CCAP_CTL_MONO_Msk | u32Interface;
}

/**
 * @brief      Disable Monochrome CMOS Sensor selection
 */
__STATIC_INLINE void CCAP_DisableMono(void)
{
    CCAP->CTL &= ~CCAP_CTL_MONO_Msk;
}

/**
 * @brief      Enable update register at new frame
 */
__STATIC_INLINE void CCAP_EnableUpdate(void)
{
    CCAP->CTL |= CCAP_CTL_UPDATE;
}

/**
 * @brief      Enable Luminance 8-bit Y to 1-bit Y Conversion
 *
 * @param[in]  u32Threshold   Luminance Y8 to Y1 Threshold Value. It should be 0 ~ 255.
 *
 * @details    This function is used to enable luminance Y8 to Y1 function and set its threshold value.
 */
__STATIC_INLINE void CCAP_EnableLumaYOne(uint32_t u32Threshold)
{
    CCAP->CTL |= CCAP_CTL_Luma_Y_One_Msk;
    CCAP->LUMA_Y1_THD = u32Threshold & 0xff;
}

/**
 * @brief      Disable Luminance 8-bit Y to 1-bit Y Conversion
 */
__STATIC_INLINE void CCAP_DisableLumaYOne(void)
{
    CCAP->CTL &= ~CCAP_CTL_Luma_Y_One_Msk;
}

/**
 * @brief      Start Camera Capture Interface function
 */
__STATIC_INLINE void CCAP_Start(void)
{
    CCAP->CTL |= CCAP_CTL_CCAPEN;
}

void CCAP_Open(uint32_t u32SensorProp, uint32_t u32InFormat, uint32_t u32OutFormat);
void CCAP_Close(void);
void CCAP_SetCroppingWindow(uint32_t u32VStart, uint32_t u32HStart, uint32_t u32Height, uint32_t u32Width);
void CCAP_SetPacketBuf(uint32_t u32Address);
void CCAP_SetPacketScaling(uint32_t u32VNumerator, uint32_t u32VDenominator, uint32_t u32HNumerator, uint32_t u32HDenominator);
void CCAP_SetPacketStride(uint32_t u32Stride);
void CCAP_EnableInt(uint32_t u32IntMask);
void CCAP_DisableInt(uint32_t u32IntMask);
void CCAP_Start(void);
int32_t CCAP_Stop(uint32_t u32FrameComplete);
void CCAP_EnableMono(uint32_t u32Interface);
void CCAP_DisableMono(void);
void CCAP_EnableLumaYOne(uint32_t u32Threshold);
void CCAP_DisableLumaYOne(void);
int32_t CCAP_MD_SetRegionSensitivity(uint32_t u32Y, uint32_t u32X, uint32_t u32Height, uint32_t u32Width, uint32_t u32Sensitivity);
int32_t CCAP_MD_SetGlobalSensitivity(uint32_t u32Sensitivity);

/** @} end of group CCAP_EXPORTED_FUNCTIONS */
/** @} end of group CCAP_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CCAP_H__
