/**************************************************************************//**
 * @file     fsa506.c
 * @version  V1.00
 * @brief    fsa506 display driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
//
// Includes
//
#include <stdio.h>
#include "NuMicro.h"
#include "fsa506.h"
#include "board_m55m1.h"
extern  uint8_t au8DestArray[1024];
extern uint32_t volatile g_u32IsTestOver;
//
// Defines
//
#define FSA506_ADDR_CMD  0x0
#define FSA506_ADDR_DATA 0x2
#define fsa506_write_cmd(Cmd)       (*((volatile uint16_t *)(EBI_BANK0_BASE_ADDR+(FSA506_ADDR_CMD))) = (Cmd))
#define fsa506_write_data(Data)       (*((volatile uint16_t *)(EBI_BANK0_BASE_ADDR+(FSA506_ADDR_DATA))) = (Data))
#define fsa506_read_data()              (*((volatile uint16_t *)(EBI_BANK0_BASE_ADDR+(FSA506_ADDR_DATA))))
#define XSIZE_PHYS                                     (480)
#define YSIZE_PHYS                                     (272)

#define SET_RST()       PH6 = 1
#define CLR_RST()       PH6 = 0

#define SET_RS()        PH7 = 1
#define CLR_RS()        PH7 = 0

#define SET_BL_ON()     PG5 = 1
#define CLR_BL_ON()     PG5 = 0


#define FSA506_CMD_SENDCMDDONE 0x80

#define  PDMA_MAXCNT 65535


//
// Functions
//
void nuFlushEBI_PDMA(int len, uint16_t *pixels);
void nuFlushEBI_PDMAScatterGather(int len, uint16_t *pixels);
//
// fsa506_low_level_init
//
void fsa506_low_level_init(void)
{
#if defined(__NUMAKER_M55M1_BETA__)
    //Selelct EBI multifunction pins 16 bit
    SYS->GPC_MFP0 &= ~(SYS_GPC_MFP0_PC3MFP_Msk | SYS_GPC_MFP0_PC2MFP_Msk | SYS_GPC_MFP0_PC1MFP_Msk | SYS_GPC_MFP0_PC0MFP_Msk);
    SYS->GPC_MFP0 |= (SYS_GPC_MFP0_PC3MFP_EBI_AD3 | SYS_GPC_MFP0_PC2MFP_EBI_AD2 | SYS_GPC_MFP0_PC1MFP_EBI_AD1 | SYS_GPC_MFP0_PC0MFP_EBI_AD0);
    SYS->GPC_MFP1 &= ~(SYS_GPC_MFP1_PC5MFP_Msk | SYS_GPC_MFP1_PC4MFP_Msk);
    SYS->GPC_MFP1 |= (SYS_GPC_MFP1_PC5MFP_EBI_AD5 | SYS_GPC_MFP1_PC4MFP_EBI_AD4);
    SYS->GPD_MFP2 &= ~(SYS_GPD_MFP2_PD9MFP_Msk | SYS_GPD_MFP2_PD8MFP_Msk);
    SYS->GPD_MFP2 |= (SYS_GPD_MFP2_PD9MFP_EBI_AD7 | SYS_GPD_MFP2_PD8MFP_EBI_AD6);
    SYS->GPD_MFP3 &= ~(SYS_GPD_MFP3_PD14MFP_Msk);
    SYS->GPD_MFP3 |= (SYS_GPD_MFP3_PD14MFP_EBI_nCS0);
    SYS->GPE_MFP0 &= ~(SYS_GPE_MFP0_PE1MFP_Msk | SYS_GPE_MFP0_PE0MFP_Msk);
    SYS->GPE_MFP0 |= (SYS_GPE_MFP0_PE1MFP_EBI_AD10 | SYS_GPE_MFP0_PE0MFP_EBI_AD11);
    SYS->GPE_MFP3 &= ~(SYS_GPE_MFP3_PE15MFP_Msk | SYS_GPE_MFP3_PE14MFP_Msk);
    SYS->GPE_MFP3 |= (SYS_GPE_MFP3_PE15MFP_EBI_AD9 | SYS_GPE_MFP3_PE14MFP_EBI_AD8);
    SYS->GPH_MFP2 &= ~(SYS_GPH_MFP2_PH11MFP_Msk | SYS_GPH_MFP2_PH10MFP_Msk | SYS_GPH_MFP2_PH9MFP_Msk | SYS_GPH_MFP2_PH8MFP_Msk);
    SYS->GPH_MFP2 |= (SYS_GPH_MFP2_PH11MFP_EBI_AD15 | SYS_GPH_MFP2_PH10MFP_EBI_AD14 | SYS_GPH_MFP2_PH9MFP_EBI_AD13 | SYS_GPH_MFP2_PH8MFP_EBI_AD12);
    SYS->GPJ_MFP2 &= ~(SYS_GPJ_MFP2_PJ9MFP_Msk | SYS_GPJ_MFP2_PJ8MFP_Msk);
    SYS->GPJ_MFP2 |= (SYS_GPJ_MFP2_PJ9MFP_EBI_nWR | SYS_GPJ_MFP2_PJ8MFP_EBI_nRD);

    GPIO_SetSlewCtl(PC, (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT14 | BIT15), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT0 | BIT1), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, (BIT8 | BIT9 | BIT10 | BIT11), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, BIT14, GPIO_SLEWCTL_HIGH);

    GPIO_SetSlewCtl(PD, BIT14, GPIO_SLEWCTL_HIGH);
#else//__NUMAKER_M55M1__
    SET_EBI_AD0_PA5();
    SET_EBI_AD1_PA4();
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
    SET_EBI_nWR_PJ9();
    SET_EBI_nRD_PJ8();
    SET_EBI_nCS0_PD14();
    SET_EBI_ADR0_PH7();

    GPIO_SetSlewCtl(PH, (BIT7 | BIT8 | BIT9 | BIT10 | BIT11), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PA, (BIT4 | BIT5), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PC, (BIT2 | BIT3 | BIT4 | BIT5), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT14 | BIT15), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PE, (BIT0 | BIT1), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PJ, (BIT8 | BIT9), GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PD, BIT14, GPIO_SLEWCTL_HIGH);
#endif
    //
    //LED_RS(PH7)
    //
    GPIO_SetMode(PH,  BIT7, GPIO_MODE_OUTPUT);
    //
    //LED_RST(PG5,7,8)
    //
    GPIO_SetMode(PG,  BIT5 | BIT7 | BIT8, GPIO_MODE_OUTPUT);


    //
    // LED_A/PWM (100Hz - 1KHz, Adjust brightness by duty cycle)
    //
    SET_EPWM0_CH3_PG5();
    EPWM_ConfigOutputChannel(EPWM0, 3, 100, 30);
    EPWM_EnableOutput(EPWM0, (0x01 << 3));
    EPWM_Start(EPWM0, (0x01 << 3));
    //
    // Initialize EBI
    //
    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOW, EBI_OPMODE_CACCESS | EBI_OPMODE_ADSEPARATE, EBI_CS_ACTIVE_LOW);

    SET_RS();
}

//
// fsa506_startup_sequence
//
void fsa506_startup_sequence(void)
{
    CLR_BL_ON();
    CLR_RST();
    CLK_SysTickDelay(100000);
    SET_RST();
    CLK_SysTickDelay(100000);

    fsa506_write_reg(0x40, 0x12);   // [5]:PLL control 20~100MHz [2:1]:Output Driving 8mA, [0]:Output slew Fast
    fsa506_write_reg(0x41, 0x05);   // PLL Programmable pre-divider: 5
    fsa506_write_reg(0x42, 0x06);   // PLL Programmable loop divider: 6

    /* Set the panel X size */
    fsa506_write_reg(0x08, (XSIZE_PHYS >> 8)); //Set the panel X size H[1.0]
    fsa506_write_reg(0x09, (XSIZE_PHYS));    //Set the panel X size L[7:0]

    /* Memory write start address */
    fsa506_write_reg(0x0a, 0x00); //[17:16] bits of memory write start address
    fsa506_write_reg(0x0b, 0x00); //[15:8] bits of memory write start address
    fsa506_write_reg(0x0c, 0x00); //[7:0] bits of memory write start address

    /* Clock & format */
    fsa506_write_reg(0x10, 0x0D); //[0-1] : 20MHz, [2]: Parallel panel, [3]: Normal operation
    fsa506_write_reg(0x11, 0x05); //[3-5]: RGB, [0-2]BGR
    //rtt = fsa506_read_data(0x10);

    /* For TFT output timing adjust */
    fsa506_write_reg(0x12, 0x00);                      //Hsync start position H-Byte
    fsa506_write_reg(0x13, 0x00);                      //Hsync start position L-Byte
    fsa506_write_reg(0x14, (41 >> 8));                //Hsync pulse width H-Byte
    fsa506_write_reg(0x15, (41));                        //Hsync pulse width L-Byte

    fsa506_write_reg(0x16, (43 >> 8));                 //DE pulse start position H-Byte
    fsa506_write_reg(0x17, (43));                         //DE pulse start position L-Byte
    fsa506_write_reg(0x18, (XSIZE_PHYS >> 8)); //DE pulse width H-Byte
    fsa506_write_reg(0x19, (XSIZE_PHYS));         //DE pulse width L-Byte
    fsa506_write_reg(0x1a, (525 >> 8));               //Hsync total clocks H-Byte
    fsa506_write_reg(0x1b, (525));                       //Hsync total clocks H-Byte
    fsa506_write_reg(0x1c, 0x00);                       //Vsync start position H-Byte
    fsa506_write_reg(0x1d, 0x00);                       //Vsync start position L-Byte
    fsa506_write_reg(0x1e, (10 >> 8));                 //Vsync pulse width H-Byte
    fsa506_write_reg(0x1f, (10));                          //Vsync pulse width L-Byte
    fsa506_write_reg(0x20, (12 >> 8));                 //Vertical DE pulse start position H-Byte
    fsa506_write_reg(0x21, (12));                         //Vertical DE pulse start position L-Byte
    fsa506_write_reg(0x22, (YSIZE_PHYS >> 8)); //Vertical Active width H-Byte
    fsa506_write_reg(0x23, (YSIZE_PHYS));         //Vertical Active width H-Byte
    fsa506_write_reg(0x24, (286 >> 8));                //Vertical total width H-Byte
    fsa506_write_reg(0x25, (286));                        //Vertical total width L-Byte

    fsa506_write_reg(0x26, 0x00);                        //Memory read start address
    fsa506_write_reg(0x27, 0x00);                        //Memory read start address
    fsa506_write_reg(0x28, 0x00);                        //Memory read start address
    fsa506_write_reg(0x29, 0x01);                        //[0] Load output timing related setting (H sync., V sync. and DE) to take effect

    //[7:4] Reserved
    //[3]   Output pin X_DCON level control
    //[2]   Output clock inversion     0: Normal 1: Inverse
    //[1:0] Image rotate
    //      00: 0? 01: 90? 10: 270?11: 180?
    fsa506_write_reg(0x2d, (1 << 2) | 0x08);

    /* Set the Horizontal offset  */
    fsa506_write_reg(0x30, 0x00);                        //_H byte H-Offset[3:0]
    fsa506_write_reg(0x31, 0x00);                        //_L byte H-Offset[7:0]
    fsa506_write_reg(0x32, 0x00);                        //_H byte V-Offset[3:0]
    fsa506_write_reg(0x33, 0x00);                        //_L byte V-Offset[7:0]
    fsa506_write_reg(0x34, (XSIZE_PHYS >> 8));  //H byte H-def[3:0]
    fsa506_write_reg(0x35, (XSIZE_PHYS));       //_L byte H-def[7:0]
    fsa506_write_reg(0x36, ((2 * YSIZE_PHYS) >> 8)); //[3:0] MSB of image vertical physical resolution in memory
    fsa506_write_reg(0x37, (2 * YSIZE_PHYS));   //[7:0] LSB of image vertical physical resolution in memory

    fsa506_fillscreen(0);
    SET_BL_ON();
}


void fsa506_send_cmd(uint16_t cmd)
{
    CLR_RS();
    fsa506_write_cmd(cmd);
    __DSB();
    SET_RS();
}


void fsa506_send_cmd_parameter(uint16_t data)
{
    fsa506_write_data(data);
    __DSB();
}


void fsa506_send_cmd_done(void)
{
    CLR_RS();
    fsa506_write_cmd(0x80);
    __DSB();
    SET_RS();
}


void fsa506_write_reg(uint16_t reg, uint16_t data)
{
    fsa506_send_cmd(reg & 0xFF);


    fsa506_send_cmd_parameter(data & 0xFF);
    __DSB();
    fsa506_send_cmd_done();
}


void fsa506_send_pixel_data(uint16_t color)
{

    fsa506_write_data(color);
    __DSB();
}


void fsa506_send_pixels(uint16_t *pixels, int len)
{

    //len: number of bytes of raw image, e.g: QGVA, RGB565, len = 320*240*2 = 153600 bytes
    //count : since EBI is 16-bit parallel data path, count is the number of bus transections, so count = len/2.
    // QVGA, RGB565, count = 76800
    int count = len / sizeof(uint16_t);
    {
        // CPU feed
        int i = 0;

        while (i < count)
        {
            fsa506_send_pixel_data(pixels[i]);
            //printf("pixels:%04x\r\n", pixels[0]);
            i++;
        }
    }

}

void fsa506_send_fixpixel(uint16_t pixels, int len)
{

    //len: number of bytes of raw image, e.g: QGVA, RGB565, len = 320*240*2 = 153600 bytes
    //count : since EBI is 16-bit parallel data path, count is the number of bus transections, so count = len/2.
    // QVGA, RGB565, count = 76800
    int count = len / sizeof(uint16_t);
    {
        // CPU feed
        int i = 0;

        while (i < count)
        {
            fsa506_send_pixel_data(pixels);
            //printf("pixels:%04x\r\n", pixels[0]);
            i++;
        }
    }

}
void fsa506_set_column(uint16_t StartCol, uint16_t EndCol)
{
    fsa506_write_reg(0x0, (StartCol >> 8) & 0xFF);
    __DSB();
    fsa506_write_reg(0x1, StartCol & 0xFF);
    __DSB();
    fsa506_write_reg(0x2, (EndCol >> 8) & 0xFF);
    __DSB();
    fsa506_write_reg(0x3, EndCol & 0xFF);
    __DSB();
}

void fsa506_set_page(uint16_t StartPage, uint16_t EndPage)
{
    fsa506_write_reg(0x4, (StartPage >> 8) & 0xFF);
    __DSB();
    fsa506_write_reg(0x5, StartPage & 0xFF);
    __DSB();
    fsa506_write_reg(0x6, (EndPage >> 8) & 0xFF);
    __DSB();
    fsa506_write_reg(0x7, EndPage & 0xFF);
    __DSB();
}



//migrate from  lcd_fsa506c.
void fsa506_fillrect(uint16_t *pixels, struct rect_info *pRectInfo)
{
    fsa506_set_column(pRectInfo->x, pRectInfo->x + pRectInfo->width - 1);
    fsa506_set_page(pRectInfo->y, pRectInfo->y + pRectInfo->height - 1);

    fsa506_send_cmd(0xC1);
    __DSB();
    fsa506_send_pixels(pixels, pRectInfo->height * pRectInfo->width * 2);
    __DSB();
    fsa506_send_cmd_done();
    __DSB();
}

void fsa506_fillrect_color(uint16_t color, struct rect_info *pRectInfo)
{
    fsa506_set_column(pRectInfo->x, pRectInfo->x + pRectInfo->width - 1);
    fsa506_set_page(pRectInfo->y, pRectInfo->y + pRectInfo->height - 1);

    fsa506_send_cmd(0xC1);
    __DSB();
    fsa506_send_fixpixel(color, pRectInfo->height * pRectInfo->width * 2);
    __DSB();
    fsa506_send_cmd_done();
    __DSB();
}

void fsa506_fillscreen(uint16_t color)
{

    fsa506_set_column(0, (480 - 1));
    fsa506_set_page(0, (272 - 1));

    fsa506_send_cmd(0xC1);

    for (int i = 0; i < (480 * 272); i++)
        fsa506_send_pixel_data(color);

    __DSB();
    fsa506_send_cmd_done();

}


static void fsa506_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
    uint32_t i, b, j;

    fsa506_set_column(x, x + font.width - 1);
    fsa506_set_page(y, y + font.height - 1);

    fsa506_send_cmd(0xC1);

    for (i = 0; i < font.height; i++)
    {
        b = font.data[(ch - 32) * font.height + i];

        for (j = 0; j < font.width; j++)
        {
            if ((b << j) & 0x8000)
            {
                uint8_t data[] = { color & 0xFF, color >> 8 };
                fsa506_send_pixels((uint16_t *)(data), sizeof(data));
            }
            else
            {
                uint8_t data[] = { bgcolor & 0xFF, bgcolor >> 8 };
                fsa506_send_pixels((uint16_t *)(data), sizeof(data));
            }
        }
    }

    __DSB();
    fsa506_send_cmd_done();
}

void fsa506_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{


    while (*str)
    {
        if (x + font.width >= FSA506_WIDTH)
        {
            break;


            if (*str == ' ')
            {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        fsa506_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }


}