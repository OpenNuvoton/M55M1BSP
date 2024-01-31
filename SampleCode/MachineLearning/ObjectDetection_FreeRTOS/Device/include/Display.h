/**************************************************************************//**
 * @file     Display.h
 * @version  V1.00
 * @brief    Display function
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_LCD_EBI              EBI_BANK0
//#define CONFIG_LCD_EBI_USE_PDMA     1
#define CONFIG_LCD_EBI_ADDR         (EBI_BANK0_BASE_ADDR+(CONFIG_LCD_EBI*EBI_MAX_SIZE))

#define C_WHITE     0xFFFF
#define C_BLACK     0x0000
#define C_BLUE      0x001F
#define C_BLUE2     0x051F
#define C_RED       0xF800
#define C_MAGENTA   0xF81F
#define C_GREEN     0x07E0
#define C_CYAN      0x7FFF
#define C_YELLOW    0xFFE0

typedef struct
{
    uint32_t u32TopLeftX;
    uint32_t u32TopLeftY;
    uint32_t u32BottonRightX;
    uint32_t u32BottonRightY;
} S_DISP_RECT;

int Display_Init(void);
void Display_FillRect(uint16_t *pu16Pixels, const S_DISP_RECT *psRect);
void Display_Delay(uint32_t u32MilliSec);
int Display_PutText(
    const char *szText,
    const uint32_t u32TextSize,
    const uint32_t u32PosX,
    const uint32_t u32PosY,
    const uint32_t u32FontColor,
    const uint32_t u32BackgroundColor,
    const bool bMultipleLines
);

void Display_ClearRect(uint32_t u32Color, const S_DISP_RECT *psRect);
void Display_ClearLCD(uint32_t u32Color);
uint32_t Disaplay_GetLCDWidth(void);
uint32_t Disaplay_GetLCDHeight(void);

#define FONT_WIDTH  8
#define FONT_HTIGHT 16

extern uint8_t Font8x16[];

#ifdef __cplusplus
}
#endif

#endif
