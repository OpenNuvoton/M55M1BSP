/**************************************************************************//**
 * @file     LCD.h
 * @version  V1.00
 * @brief    LCD EBI driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __LCD_H__
#define __LCD_H__

#include "NuMicro.h"
#include "Display.h"


/* LCD EBI */

#define CONFIG_LCD_BACKLIGHT_PORT   PG//NU_GET_PININDEX(evGG, 5)   //101
#define CONFIG_LCD_BACKLIGHT_PIN    5//NU_GET_PININDEX(evGG, 5)   //101

#define CONFIG_LCD_DC_PORT          PH//NU_GET_PININDEX(evGG, 5)   //101
#define CONFIG_LCD_DC_PIN           7//NU_GET_PININDEX(evGG, 5)   //101

#define CONFIG_LCD_RESET_PORT       PH//NU_GET_PININDEX(evGG, 5)   //101
#define CONFIG_LCD_RESET_PIN        6//NU_GET_PININDEX(evGG, 5)   //101

#define CONFIG_LCD_PIN_BACKLIGHT    PG5//NU_GET_PININDEX(evGG, 5)   //101
#define CONFIG_LCD_PIN_DC           PH7//NU_GET_PININDEX(evGH, 7)   //119
#define CONFIG_LCD_PIN_RESET        PH6//NU_GET_PININDEX(evGG, 7)   //103

#define SET_RS                   CONFIG_LCD_PIN_DC = 1
#define CLR_RS                   CONFIG_LCD_PIN_DC = 0
#define SET_RST                  CONFIG_LCD_PIN_RESET = 1
#define CLR_RST                  CONFIG_LCD_PIN_RESET = 0
#define SET_BACKLIGHT_ON         CONFIG_LCD_PIN_BACKLIGHT = 1
#define SET_BACKLIGHT_OFF        CONFIG_LCD_PIN_BACKLIGHT = 0

#define LCD_Delay_MilliSec   Display_Delay

typedef int32_t (*PFN_LCD_INIT)(void);
typedef void (*PFN_LCD_WRITE_REG)(uint16_t u16Reg, uint16_t u16Data);
typedef void (*PFN_LCD_SET_COLUMN)(uint16_t u16StartCol, uint16_t u16EndCol);
typedef void (*PFN_LCD_SET_PAGE)(uint16_t u16StartPage, uint16_t u16EndPage);
typedef void (*PFN_LCD_SENT_PIXEL)(uint16_t *pu16Pixels, uint32_t u32FixedColor, int32_t i32ByteLen);
typedef void (*PFN_LCD_PUT_CHAR)(uint16_t x, uint16_t y, uint8_t c, uint32_t fColor, uint32_t bColor);

typedef enum
{
    eLCD_INPUT_RGB565,
    eLCD_INPUT_RGB888,
} E_LCD_INPUT_FORMAT;

typedef struct s_lcd_info
{
    char                m_strName[16];
    E_LCD_INPUT_FORMAT  m_eInputFormat;
    uint16_t            m_u16Width;
    uint16_t            m_u16Height;
    PFN_LCD_INIT        m_pfnInit;
    PFN_LCD_WRITE_REG   m_pfnWriteReg;
    PFN_LCD_SET_COLUMN  m_pfnSetColumn;
    PFN_LCD_SET_PAGE    m_pfnSetPage;
    PFN_LCD_SENT_PIXEL  m_pfnSentPixel;
    PFN_LCD_PUT_CHAR    m_pfnPutChar;
} S_LCD_INFO;

extern S_LCD_INFO g_s_WQVGA_FSA506;

#endif
