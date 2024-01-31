/**************************************************************************//**
 * @file     Display.c
 * @version  V1.00
 * @brief    Display functions
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "LCD.h"

#include "pmu_counter.h"

S_LCD_INFO *s_psLCD = &g_s_WQVGA_FSA506;

static void Configure_EBI_16BIT_Pins(void)
{
    /* AD0 ~ AD15*/
    SET_EBI_AD0_PC0();
    SET_EBI_AD1_PC1();
    SET_EBI_AD2_PC2();
    SET_EBI_AD3_PC3();
    SET_EBI_AD4_PC4();
    SET_EBI_AD5_PC5();
    SET_EBI_AD6_PD8();
    SET_EBI_AD7_PD9();
    SET_EBI_AD8_PE14();
    SET_EBI_AD9_PE15();
    SET_EBI_AD10_PE1();
    SET_EBI_AD11_PE0();
    SET_EBI_AD12_PH8();
    SET_EBI_AD13_PH9();
    SET_EBI_AD14_PH10();
    SET_EBI_AD15_PH11();

    /* EBI nWR and nRD pins on PA.10 and PA.11 */
    SET_EBI_nWR_PJ9();
    SET_EBI_nRD_PJ8();

    /* EBI nCS1 pin on PD.11 */
    SET_EBI_nCS0_PD14();
}


int Display_Init(void)
{
    pmu_reset_counters();

    //TODO: EBI bus init
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable EBI clock */
    CLK_EnableModuleClock(EBI0_MODULE);


    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();


    GPIO_SetSlewCtl(PC, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT14 | BIT15), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT0 | BIT1), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, (BIT8 | BIT9 | BIT10 | BIT11), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, BIT14, GPIO_SLEWCTL_HIGH);

    GPIO_SetMode(CONFIG_LCD_BACKLIGHT_PORT, 1 << CONFIG_LCD_BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
    GPIO_SetMode(CONFIG_LCD_DC_PORT, 1 << CONFIG_LCD_DC_PIN, GPIO_MODE_OUTPUT);
    GPIO_SetMode(CONFIG_LCD_RESET_PORT, 1 << CONFIG_LCD_RESET_PIN, GPIO_MODE_OUTPUT);

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(CONFIG_LCD_EBI, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, 0, EBI_CS_ACTIVE_LOW);

    /* lock protected registers */
    SYS_LockReg();

    //Init LCD
    s_psLCD->m_pfnInit();
    return 0;
}

void Display_FillRect(uint16_t *pu16Pixels, const S_DISP_RECT *psRect)
{
    int32_t w = psRect->u32BottonRightX - psRect->u32TopLeftX;
    int32_t h = psRect->u32BottonRightY - psRect->u32TopLeftY;

    s_psLCD->m_pfnSetColumn(psRect->u32TopLeftX, psRect->u32BottonRightX);
    s_psLCD->m_pfnSetPage(psRect->u32TopLeftY, psRect->u32BottonRightY);
    s_psLCD->m_pfnSentPixel(pu16Pixels, 0, h * w * sizeof(uint16_t));
}

void Display_Delay(uint32_t u32MilliSec)
{
    uint64_t u64WaiteCycles = pmu_get_systick_Count();

    u64WaiteCycles += (SystemCoreClock / 1000) * u32MilliSec;

    while (pmu_get_systick_Count() <= u64WaiteCycles)
    {
        __NOP();
    }
}

int Display_PutText(
    const char *szText,
    const uint32_t u32TextSize,
    const uint32_t u32PosX,
    const uint32_t u32PosY,
    const uint32_t u32FontColor,
    const uint32_t u32BackgroundColor,
    const bool bMultipleLines
)
{
    /* We use a font which is 9x15. */
    const uint32_t x_span =  8; /* Each character is this  8 pixels "wide". */
    const uint32_t y_span = 16; /* Each character is this 16 pixels "high". */

    if (u32TextSize == 0)
    {
        return 1;
    }

    /* If not within the LCD bounds, return error. */
    if (u32PosX + x_span > s_psLCD->m_u16Width || u32PosY + y_span > s_psLCD->m_u16Height)
    {
        return 1;
    }
    else
    {

        const uint32_t col = u32PosX / x_span;
        const uint32_t max_cols = s_psLCD->m_u16Width / x_span - 1;
        const uint32_t max_lines = s_psLCD->m_u16Height / y_span - 1;

        uint32_t i = 0;
        uint32_t current_line = u32PosY / y_span;
        uint32_t current_col = col;

        /* Display the string on the LCD. */
        for (i = 0; i < u32TextSize; ++i)
        {

            if (bMultipleLines)
            {

                /* If the next character won't fit. */
                if (current_col > max_cols)
                {
                    current_col = col;

                    /* If the next line won't fit. */
                    if (++current_line  > max_lines)
                    {
                        return 1;
                    }
                }
            }

            s_psLCD->m_pfnPutChar(current_col * x_span, current_line * y_span, szText[i], u32FontColor, u32BackgroundColor);
            current_col++;
        }
    }

    return 0;
}


void Display_ClearRect(uint32_t u32Color, const S_DISP_RECT *psRect)
{
    int32_t w = psRect->u32BottonRightX - psRect->u32TopLeftX;
    int32_t h = psRect->u32BottonRightY - psRect->u32TopLeftY;

    s_psLCD->m_pfnSetColumn(psRect->u32TopLeftX, psRect->u32BottonRightX);
    s_psLCD->m_pfnSetPage(psRect->u32TopLeftY, psRect->u32BottonRightY);
    s_psLCD->m_pfnSentPixel(NULL, u32Color, h * w * sizeof(uint16_t));
}

void Display_ClearLCD(uint32_t u32Color)
{
    int32_t w = s_psLCD->m_u16Width;
    int32_t h = s_psLCD->m_u16Height;

    s_psLCD->m_pfnSetColumn(0, w - 1);
    s_psLCD->m_pfnSetPage(0, h - 1);
    s_psLCD->m_pfnSentPixel(NULL, u32Color, h * w * sizeof(uint16_t));
}

uint32_t Disaplay_GetLCDWidth(void)
{
    return s_psLCD->m_u16Width;
}

uint32_t Disaplay_GetLCDHeight(void)
{
    return s_psLCD->m_u16Height;
}
