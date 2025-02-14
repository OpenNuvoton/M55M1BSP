/**************************************************************************//**
 * @file     fsa506.h
 * @version  V1.00
 * @brief    fsa506 display driver header
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __FSA506_H__
#define __FSA506_H__

//
// Includes
//
#include <stdint.h>
#include "fonts.h"
//#include "ili9341/ili9341.h"

//
// Defines
//
typedef struct rect_info
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} rect_info_t;

//
// Functions
//
#ifdef  __cplusplus
extern  "C" {
#endif
void fsa506_low_level_init(void);
void fsa506_startup_sequence(void);

void lcd_write_reg(uint8_t addr, uint8_t value);
void lcd_write_ram(uint16_t val);

void fsa506_send_cmd(uint16_t cmd);
void fsa506_send_cmd_parameter(uint16_t data);
void fsa506_send_cmd_done(void);
void fsa506_write_reg(uint16_t reg, uint16_t data);
void fsa506_send_pixel_data(uint16_t color);
void fsa506_send_pixels(uint16_t *pixels, int len);
void fsa506_set_column(uint16_t StartCol, uint16_t EndCol);
void fsa506_set_page(uint16_t StartPage, uint16_t EndPage);
void fsa506_fillrect(uint16_t *pixels, struct rect_info *pRectInfo);
void fsa506_fillscreen(uint16_t color);
void fsa506_send_fixpixel(uint16_t pixels, int len);
void fsa506_fillrect_color(uint16_t color, struct rect_info *pRectInfo);
void fsa506_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);

#ifdef  __cplusplus
}
#endif
#endif  /* __FSA506_H__ */
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/