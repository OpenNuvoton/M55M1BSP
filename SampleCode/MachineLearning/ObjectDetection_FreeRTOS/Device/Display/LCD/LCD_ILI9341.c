#include <stdio.h>
#include "NuMicro.h"
#include "../LCD.h"

#if defined(__SPI_LCD_PANEL__)

//#define SWAPAXIS

#define LCD_RES_X 320
#define LCD_RES_Y 240

static void _write_command(uint8_t temp)
{
    CLR_RS;
    CLR_SS;

    SPI_WRITE_TX(CONFIG_LCD_SPI_PORT, temp);

    while (SPI_IS_BUSY(CONFIG_LCD_SPI_PORT));

    SET_SS;
}

static void _write_data(uint8_t temp)
{
    SET_RS;
    CLR_SS;

    SPI_WRITE_TX(CONFIG_LCD_SPI_PORT, temp);

    while (SPI_IS_BUSY(CONFIG_LCD_SPI_PORT));

    SET_SS;
}

static void set_color(uint16_t color)
{
    _write_data(color >> 8);
    _write_data(color);
}

static int32_t ili9341_init(void)
{
    CLR_RS;
    CLR_RST;
    SET_BACKLIGHT_OFF;

    SET_RS;
    CLR_RST;
    LCD_Delay_MilliSec(200);
    SET_RST;
    LCD_Delay_MilliSec(400);

    _write_command(0xCF);
    _write_data(0x00);
    _write_data(0xD9);
    _write_data(0X30);

    _write_command(0xED);
    _write_data(0x64);
    _write_data(0x03);
    _write_data(0X12);
    _write_data(0X81);

    _write_command(0xE8);
    _write_data(0x85);
    _write_data(0x10);
    _write_data(0x78);

    _write_command(0xCB);
    _write_data(0x39);
    _write_data(0x2C);
    _write_data(0x00);
    _write_data(0x34);
    _write_data(0x02);

    _write_command(0xF7);
    _write_data(0x20);

    _write_command(0xEA);
    _write_data(0x00);
    _write_data(0x00);

    _write_command(0xC0);    //Power control
    _write_data(0x21);   //VRH[5:0]

    _write_command(0xC1);    //Power control
    _write_data(0x12);   //SAP[2:0];BT[3:0]

    _write_command(0xC5);    //VCM control
    _write_data(0x32);
    _write_data(0x3C);

    _write_command(0xC7);    //VCM control2
    _write_data(0XC1);

    _write_command(0x36);    // Memory Access Control
    //_write_data(0xA8);
    //_write_data(0xe8);
    _write_data(0x68); //左右
    //_write_data(0x28);

    _write_command(0x3A);
    _write_data(0x55);

    _write_command(0xB1);
    _write_data(0x00);
    _write_data(0x18);

    _write_command(0xB6);    // Display Function Control
    _write_data(0x0A);
    _write_data(0xA2);

    _write_command(0x51);    // Display brightness
    _write_data(0xFF);

    _write_command(0xF2);    // 3Gamma Function Disable
    _write_data(0x00);

    _write_command(0x26);    //Gamma curve selected
    _write_data(0x01);

    _write_command(0xE0);    //Set Gamma
    _write_data(0x0F);
    _write_data(0x20);
    _write_data(0x1E);
    _write_data(0x09);
    _write_data(0x12);
    _write_data(0x0B);
    _write_data(0x50);
    _write_data(0XBA);
    _write_data(0x44);
    _write_data(0x09);
    _write_data(0x14);
    _write_data(0x05);
    _write_data(0x23);
    _write_data(0x21);
    _write_data(0x00);

    _write_command(0XE1);    //Set Gamma
    _write_data(0x00);
    _write_data(0x19);
    _write_data(0x19);
    _write_data(0x00);
    _write_data(0x12);
    _write_data(0x07);
    _write_data(0x2D);
    _write_data(0x28);
    _write_data(0x3F);
    _write_data(0x02);
    _write_data(0x0A);
    _write_data(0x08);
    _write_data(0x25);
    _write_data(0x2D);
    _write_data(0x0F);

    _write_command(0x11);    //Exit Sleep
    LCD_Delay_MilliSec(120);
    _write_command(0x29);    //Display on

    SET_BACKLIGHT_ON;
    return 0;
}

static void ili9341_set_column(uint16_t StartCol, uint16_t EndCol)
{
#if defined(SWAPAXIS)
    _write_command(0x2b);
    _write_data(StartCol >> 8);
    _write_data(StartCol);
    _write_data(EndCol >> 8);
    _write_data(EndCol);
    _write_command(0x2c);
#else
    _write_command(0x2a);
    _write_data(StartCol >> 8);
    _write_data(StartCol);
    _write_data(EndCol >> 8);
    _write_data(EndCol);
    _write_command(0x2c);
#endif
}

static void ili9341_set_page(uint16_t StartPage, uint16_t EndPage)
{
#if defined(SWAPAXIS)
    _write_command(0x2a);
    _write_data(StartPage >> 8);
    _write_data(StartPage);
    _write_data(EndPage >> 8);
    _write_data(EndPage);
    _write_command(0x2c);
#else
    _write_command(0x2b);
    _write_data(StartPage >> 8);
    _write_data(StartPage);
    _write_data(EndPage >> 8);
    _write_data(EndPage);
    _write_command(0x2c);
#endif
}

static void ili9341_send_pixels(uint16_t *pixels, uint32_t destWidth, uint32_t destHeight, uint32_t fixedColor, int32_t byteLen, int32_t scaleUpFactory)
{
    uint32_t w, h, r_x, r_y;
    uint16_t *pixels_y = NULL;
    uint16_t color = fixedColor;

    uint32_t srcWidth = destWidth / scaleUpFactory;

    pixels_y = pixels;

    for (h = 0; h < destHeight; h += scaleUpFactory)
    {
        for (r_y = 0; r_y < scaleUpFactory; r_y++)
        {
            for (w = 0; w < destWidth; w += scaleUpFactory)
            {
                if (pixels_y)
                    color = pixels_y[w / scaleUpFactory];

                for (r_x = 0; r_x < scaleUpFactory; r_x++)
                {
                    set_color(color);
                }
            }
        }

        if (pixels_y)
        {
            pixels_y += srcWidth;
        }
    }
}

void ili9341_put_char8x16(uint16_t x, uint16_t y, uint8_t c, uint32_t fColor, uint32_t bColor, int32_t scaleUpFactory)
{
    uint32_t i, j;
    uint8_t m;

    ili9341_set_column(x, x + (8 * scaleUpFactory) - 1);
    ili9341_set_page(y, y + (16 * scaleUpFactory) - 1);

    int r_x, r_y;

    for (i = 0; i < 16; i++)
    {
        for (r_y = 0; r_y < scaleUpFactory; r_y++)
        {
            m = Font8x16[c * 16 + i];

            for (j = 0; j < 8; j++)
            {
                for (r_x = 0; r_x < scaleUpFactory; r_x++)
                {
                    if ((m & 0x80) == 0x80)
                    {
                        set_color(fColor);
                    }
                    else
                    {
                        set_color(bColor);
                    }
                }

                m <<= 1;
            }
        }
    }
}


S_LCD_INFO g_s_QVGA_ILI9341 =
{
    .m_strName      = "ILI9341",
    .m_eInputFormat = eLCD_INPUT_RGB565,
    .m_u16Width     = LCD_RES_X,
    .m_u16Height    = LCD_RES_Y,
    .m_pfnInit      = ili9341_init,
    .m_pfnWriteReg  = NULL,
    .m_pfnSetColumn = ili9341_set_column,
    .m_pfnSetPage   = ili9341_set_page,
    .m_pfnSentPixel = ili9341_send_pixels,
    .m_pfnPutChar   = ili9341_put_char8x16,
};

#endif
