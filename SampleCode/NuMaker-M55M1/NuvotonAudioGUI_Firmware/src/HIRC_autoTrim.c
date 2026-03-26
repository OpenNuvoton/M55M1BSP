/******************************************************************************
 * @file     HIRC_autoTrim.c
 * @version  V1.00
 * @brief    Auto HIRC Trim source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "user_config.h"
#include "audio_class.h"

/*----------------------------------------------------------------------------*/
/* Global Variables                                                           */
/*----------------------------------------------------------------------------*/
// For HIRC trim automatically
volatile uint8_t g_u8USBPlayEn = 0;
volatile uint8_t g_u8USBRecEn = 0;
volatile uint8_t g_u8HIRCAutoTrimDone = 0;
volatile uint8_t g_u8HIRCAutoTrimEnable = 0;
volatile uint8_t g_u8HIRCAutoTrimCnt = 0;

// For HIRC trim manually
volatile uint8_t u8USBPlayCnt = 0;
volatile uint8_t u8PDMAPlayCnt = 0;

//------------------------------------------------------------------------------
extern volatile uint16_t g_u16PlayBack_Ptrs_Distance;
extern volatile uint16_t g_u16PlayBack_Read_Ptr;
extern volatile uint16_t g_u16PlayBack_Write_Ptr;
extern volatile uint8_t g_u8AudioSpeakerState;
extern volatile uint8_t g_bUnderFlow; // No RingBuffer for reading
extern volatile uint8_t g_bOverFlow;  // No RingBuffer for Writting
extern volatile uint16_t g_u16PlayBack_MAX_USB_BUFFER_LEN;
extern volatile uint16_t g_u16PlayBack_I2S_BUFF_LEN;
extern volatile uint16_t g_u16PlayBack_USB_BUFFER_THRE_BASE;
extern volatile uint16_t g_u16PlayBack_USB_BUFF_UPPER_THRE;
extern volatile uint16_t g_u16PlayBack_USB_BUFF_LOWER_THRE;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/**
 * @brief       Enable HIRC Auto Trim function
 *
 * @param[in]   u32FreqSel   The target frequency of HIRC trim. Valid values are:
 *                       - \ref SYS_HIRCTCTL_FREQSEL_48M
 *
 * @return      None
 *
 * @details     This function is used to enable HIRC Auto Trim function.
 */
void SYS_EnableTrimHIRC(uint32_t u32FreqSel)
{
    //SYS->HIRCTISTS |= SYS_HIRCTISTS_FREQLOCK_Msk;
    //SYS->HIRCTCTL = (SYS->HIRCTCTL & ~SYS_HIRCTCTL_FREQSEL_Msk) | u32FreqSel;
    SYS->TISTS48M |= SYS_TISTS48M_FREQLOCK_Msk;
    SYS->TCTL48M = (SYS->TCTL48M & ~SYS_TCTL48M_FREQSEL_Msk) | u32FreqSel;
}

/**
 * @brief       Enable HIRC Auto Trim function
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to enable HIRC Auto Trim function.
 */
void HIRC_AutoTrim_Enable(void)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (g_u8HIRCAutoTrimDone == 0)
    {
        if (u32RegLockLevel) SYS_UnlockReg();

        if (g_u8HIRCAutoTrimEnable == 0) // HIRC auto trim initial & enable
        {
            g_u8HIRCAutoTrimEnable = 1;
            SYS_SET_TRIMHIRC_LOOPSEL(SYS_HIRCTCTL_LOOPSEL_16);
            SYS_SET_TRIMHIRC_RETRYCNT(SYS_HIRCTCTL_RETRYCNT_64);
            SYS_ENABLE_TRIMHIRC_CLKERRSTOP();
            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_HIRCTISTS_TRIMFAIL_INT_FLAG | SYS_HIRCTISTS_CLKERROR_INT_FLAG);
            SYS_EnableTrimHIRC(SYS_HIRCTCTL_FREQSEL_48M);
        }

        if (SYS_IS_TRIMHIRC_DONE())
        {
            g_u8HIRCAutoTrimCnt++;
            g_u8HIRCAutoTrimEnable = 0;
            //SYS->HIRCTCTL &= ~SYS_HIRCTCTL_FREQSEL_Msk;
            //SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_HIRCTISTS_FREQLOCK_Msk);
            SYS->TCTL48M &= ~SYS_TCTL48M_FREQSEL_Msk;
            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_TISTS48M_FREQLOCK_Msk);

            if (g_u8HIRCAutoTrimCnt == 5)
            {
                g_u8HIRCAutoTrimDone = 1;
                g_u8HIRCAutoTrimCnt = 0;
                g_u8HIRCAutoTrimEnable = 0;
            }
        }
        //else if (SYS->HIRCTISTS & (SYS_HIRCTISTS_TRIMFAIL_INT_FLAG | SYS_HIRCTISTS_CLKERROR_INT_FLAG))
        else if (SYS->TISTS48M & (SYS_HIRCTISTS_TRIMFAIL_INT_FLAG | SYS_HIRCTISTS_CLKERROR_INT_FLAG))
        {
            g_u8HIRCAutoTrimCnt = 0;
            g_u8HIRCAutoTrimEnable = 0;

            //SYS->HIRCTCTL &= ~SYS_HIRCTCTL_FREQSEL_Msk;
            SYS->TCTL48M &= ~SYS_TCTL48M_FREQSEL_Msk;

            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_HIRCTISTS_TRIMFAIL_INT_FLAG | SYS_HIRCTISTS_CLKERROR_INT_FLAG);
            SYS_EnableTrimHIRC(SYS_HIRCTCTL_FREQSEL_48M);
        }

        if (u32RegLockLevel) SYS_LockReg();
    }
}

/**
 * @brief       Reset HIRC Auto Trim function
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to reset HIRC Auto Trim function.
 */
void HIRC_AutoTrim_Reset(void)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel) SYS_UnlockReg();

    //SYS->HIRCTCTL &= ~SYS_HIRCTCTL_FREQSEL_Msk;
    SYS->TCTL48M &= ~SYS_TCTL48M_FREQSEL_Msk;

    g_u8HIRCAutoTrimDone = 0;
    g_u8HIRCAutoTrimCnt = 0;
    g_u8HIRCAutoTrimEnable = 0;

    if (u32RegLockLevel) SYS_LockReg();
}

/**
 * @brief       Enable HIRC Trim function manually
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to fine-tune HIRC frequency manually.
 *              It will adjust HIRC frequency by the distance of USB playback buffer read and write pointers.
 */
void HIRC_ManualTrim_Enable(void)
{
    if (g_u8USBPlayEn)
    {
        if (g_u16PlayBack_Write_Ptr >= g_u16PlayBack_Read_Ptr)
        {
            if (g_u16PlayBack_Write_Ptr == g_u16PlayBack_Read_Ptr)
            {
                if (g_bOverFlow == TRUE)
                    g_u16PlayBack_Ptrs_Distance = g_u16PlayBack_MAX_USB_BUFFER_LEN;
                else
                    g_u16PlayBack_Ptrs_Distance = 0;
            }
            else
                g_u16PlayBack_Ptrs_Distance = g_u16PlayBack_Write_Ptr - g_u16PlayBack_Read_Ptr;
        }
        else
        {
            g_u16PlayBack_Ptrs_Distance = (g_u16PlayBack_MAX_USB_BUFFER_LEN - g_u16PlayBack_Read_Ptr) + g_u16PlayBack_Write_Ptr;
        }

        if (g_u16PlayBack_Ptrs_Distance >= g_u16PlayBack_USB_BUFF_UPPER_THRE)
        {
            outpw((SYS_BASE + 0x10c), (inpw(SYS_BASE + 0x10c) - 1));
        }
        else if (g_u16PlayBack_Ptrs_Distance <= (g_u16PlayBack_USB_BUFF_LOWER_THRE / 4 * 3))
        {
            outpw((SYS_BASE + 0x10c), (inpw(SYS_BASE + 0x10c) + 1));
        }
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
