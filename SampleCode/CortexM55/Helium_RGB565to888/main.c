/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   RG565toRGB888 with Helium.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

//Use Helium
#include <arm_mve.h>


/****************************************************************************
 * User Define Macro
 ****************************************************************************/
#define R_RGBIDX 0
#define G_RGBIDX 1
#define B_RGBIDX 2
#define OFFSET_R_LOW    0x15120F0C09060300
#define OFFSET_R_HIGH    0x2D2A2724211E1B18
#define OFFSET_G_LOW    0x1613100D0A070401
#define OFFSET_G_HIGH    0x2E2B2825221F1C19
#define OFFSET_B_LOW    0x1714110E0B080502
#define OFFSET_B_HIGH    0x2F2C292623201D1A
#define OFFSET_565_STRIDEONE_LOW    0x0E0C0A0806040200
#define OFFSET_565_STRIDEONE_HIGH    0x1E1C1A1816141210
#define OFFSET_565_STRIDETWO_LOW    0x0F0D0B0907050301
#define OFFSET_565_STRIDETWO_HIGH    0x1F1D1B1917151311

#define OFFSET_565HIGHBYTE_LOW     0x0E0C0A0806040200
#define OFFSET_565HIGHBYTE_HIGH    0x1E1C1A1816141210
#define OFFSET_565LOWBYTE_LOW    0x0F0D0B0907050301
#define OFFSET_565LOWBYTE_HIGH   0x1F1D1B1917151311
#define BYTENUMBER_RGB565  2
#define BYTENUMBER_RGB888  3
#define PIXELS_LOOP       16
#define PIXELS_CONVERT_EACHLOOP       16

#define SRC_IMAGE_LEN            16
#define SRC_IMAGE_WIDTH           4
#define SRC_IMAGE_HEIGHT          4

#define DST_IMAGE_LEN            (SRC_IMAGE_LEN*BYTENUMBER_RGB888)

#define CHECK_888RESULT
#define SW_SCALEUP_TEST
/****************************************************************************
 * Global Variables
 ****************************************************************************/
int iPMU_TickInit;
uint64_t sPMU_TickCounter;
uint16_t image_rgb565_src[SRC_IMAGE_LEN] = { 0xDFF2, 0xB234, 0xA2A4, 0x923A, 0xC2C4, 0xA2A4, 0x62A4, 0xA234, 0xA235, 0x1C74, 0xF234, 0x3234, 0x4234, 0x5234, 0x6234, 0x7277};
uint8_t image_dst[DST_IMAGE_LEN] = {0};
uint8_t vimage_dst[DST_IMAGE_LEN] = {0};
volatile uint32_t g_u32Ticks = 0;
/****************************************************************************
 * Function Proto Type
 ****************************************************************************/
void RGB565to888(uint8_t *src, uint8_t *dst, uint8_t len);
void VRGB565to888_TP(uint8_t *src, uint8_t *dst, uint16_t size_h, uint16_t size_w);
void SYS_Init(void);

void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    g_u32Ticks += SysTick->LOAD + 1;
}


int Init_SysTick_Export(void)
{
    const uint32_t ticks_10ms = SystemCoreClock / 100 + 1;
    int err = 0;

    /* Reset CPU cycle count value. */
    g_u32Ticks = 0;

    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);

    /* SysTick init - this will enable interrupt too. */
    err = SysTick_Config(ticks_10ms);

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!err && !SysTick->VAL)
    {
        __NOP();
    }

    return err;
}

/**
 * Gets the current SysTick derived counter value
 */
static uint64_t Get_SysTick_Cycle_Count(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return g_u32Ticks + (SysTick->LOAD - systick_val);
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);


    /* Enable PLL0 220MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeVal;

    uint32_t u32TimeVal_Vec;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+-----------------------------------------+\n");
    printf("|   Helium RGB565 to 888 Sample Code KEIL |\n");
    printf("+-----------------------------------------+\n");

    /* Use systick to measure inference time */


    while (1)
    {
        printf("\n Execute RGB565to888\n");
        Init_SysTick_Export();
        RGB565to888((uint8_t *)(image_rgb565_src), image_dst, SRC_IMAGE_LEN);
        u32TimeVal = Get_SysTick_Cycle_Count();
        printf("\n Tick Counters are:%d!\n", u32TimeVal);

        printf("\n Execute VRGB565to888_TP\n");
        Init_SysTick_Export();
        VRGB565to888_TP((uint8_t *)(image_rgb565_src), vimage_dst, SRC_IMAGE_HEIGHT, SRC_IMAGE_WIDTH);
        u32TimeVal_Vec = Get_SysTick_Cycle_Count();
        printf("\n Tick Counters are:%d!\n", u32TimeVal_Vec);

        printf("\n Compare RGB565 Results \n");

        for (int ii = 0; ii < DST_IMAGE_LEN; ii++)
        {
            if (image_dst[ii] == vimage_dst[ii])
            {
                printf("\n Match: image_dst[%d] = 0x%02x \n", ii, image_dst[ii]);
            }
            else
            {
                printf("\n Error: image_dst[%d] = 0x%02x, vimage_dst[%d] = 0x%02x \n", ii, image_dst[ii], ii, vimage_dst[ii]);
            }
        }

        printf("\n RGB565 Speedup %1.2f X\n", (float)(u32TimeVal) / (float)(u32TimeVal_Vec));

        printf("\n Test Done\n");

        while (1);
    }

}

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
void RGB565to888(uint8_t *src, uint8_t *dst, uint8_t len)
{
    uint8_t buf888_high, buf888_low;
    uint8_t dst888_r, dst888_g, dst888_b;
    int count = len;

    while (count >= 0)
    {
        //RGB565_IN_UINT8
        buf888_high = *src;
        buf888_low = *(src + 1);

        //Separate RGB//
        dst888_r  = (buf888_high & 0xF8) ;

        dst888_g  = ((buf888_high & 0x07) << 5) | ((buf888_low & 0xE0) >> 3) ;

        dst888_b  = (buf888_low << 3) ;

        //Store to Memory
        *(dst++) = dst888_r;
        *(dst++) = dst888_g;
        *(dst++) = dst888_b;

        src += BYTENUMBER_RGB565;
        count --;
    }
}

void VRGB565to888_TP(uint8_t *src, uint8_t *dst, uint16_t size_h, uint16_t size_w)
{
    int8_t count;
    uint8x16_t vsrc_8_16_lo;
    uint8x16_t vsrc_8_16_hi;
    uint8x16_t vdst_8_16_r;
    uint8x16_t vdst_8_16_g;
    uint8x16_t vdst_8_16_b;
    uint8x16_t offset_8_16;

    count = size_h * size_w;

    while (count > 0)
    {
        //RGB565_IN_UINT8

        //Include tail predication to handle improper tailing.
        mve_pred16_t p = vctp8q(count);

        offset_8_16 = vcreateq_u8(OFFSET_565LOWBYTE_LOW, OFFSET_565LOWBYTE_HIGH);

        vsrc_8_16_lo = vldrbq_gather_offset_z_u8(src, offset_8_16, p);

        offset_8_16 = vcreateq_u8(OFFSET_565HIGHBYTE_LOW, OFFSET_565HIGHBYTE_HIGH);

        vsrc_8_16_hi = vldrbq_gather_offset_z_u8(src, offset_8_16, p);

        //Separate RGB//
        vdst_8_16_r  =  vshlq_n(vshrq(vsrc_8_16_hi, 3), 3);

        vdst_8_16_g =  vorrq_u8(vshlq_n(vsrc_8_16_hi, 5), vshlq_n(vshrq(vsrc_8_16_lo, 5), 2));

        vdst_8_16_b =  vshlq_n(vsrc_8_16_lo, 3);

        //Scatter Store//
        offset_8_16 = vcreateq_u8(OFFSET_R_LOW, OFFSET_R_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_r, p);

        offset_8_16 = vcreateq_u8(OFFSET_G_LOW, OFFSET_G_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_g, p);

        offset_8_16 = vcreateq_u8(OFFSET_B_LOW, OFFSET_B_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_b, p);

        //Update data source and destination pointer

        //Source: shift one RGB566 pixel size(2 byte)
        src += BYTENUMBER_RGB565 * PIXELS_CONVERT_EACHLOOP;

        //Destination: shift one RGB888 pixel size(3 byte)
        dst += BYTENUMBER_RGB888 * PIXELS_CONVERT_EACHLOOP;

        count -= PIXELS_CONVERT_EACHLOOP;

    }// while (count > 0)
}
#else

void RGB565to888(uint8_t *src, uint8_t *dst, uint8_t len)
{
    uint8_t buf888_high, buf888_low;
    uint8_t dst888_r, dst888_g, dst888_b;
    int count = len;

    while (count >= 0)
    {
        //RGB565_IN_UINT8
        buf888_high = *src;
        buf888_low = *(src + 1);

        //Separate RGB//
        dst888_r  = (buf888_high & 0xF8) ;

        dst888_g  = ((buf888_high & 0x07) << 5) | ((buf888_low & 0xE0) >> 3) ;

        dst888_b  = (buf888_low << 3) ;

#ifdef PADDING_RGB888_LSB
        //Separate RGB with Padding//
        dst888_r  = (buf888_high & 0xF8) | (buf888_high >> 5) ;

        dst888_g  = (((buf888_high & 0x07) << 5) | ((buf888_high & 0x07) >> 1)) | ((buf888_low & 0xE0) >> 3) ;

        dst888_b  = (buf888_low << 3) | ((buf888_low & 0x1F) >> 2);
#endif
        //Store to Memory
        *(dst++) = dst888_r;
        *(dst++) = dst888_g;
        *(dst++) = dst888_b;

        src += BYTENUMBER_RGB565;
        count --;
    }
}


void VRGB565to888_TP(uint8_t *src, uint8_t *dst, uint16_t size_h, uint16_t size_w)
{
    int8_t count;
    uint8x16_t vsrc_8_16_lo;
    uint8x16_t vsrc_8_16_hi;
    uint8x16_t vdst_8_16_r;
    uint8x16_t vdst_8_16_g;
    uint8x16_t vdst_8_16_b;
    uint8x16_t offset_8_16;

    count = size_h * size_w;

    while (count > 0)
    {
        //RGB565_IN_UINT8

        //Include tail predication to handle improper tailing.
        mve_pred16_t p = vctp8q(count);

        offset_8_16 = vcreateq_u8(OFFSET_565LOWBYTE_LOW, OFFSET_565LOWBYTE_HIGH);

        vsrc_8_16_lo = vldrbq_gather_offset_z_u8(src, offset_8_16, p);

        offset_8_16 = vcreateq_u8(OFFSET_565HIGHBYTE_LOW, OFFSET_565HIGHBYTE_HIGH);

        vsrc_8_16_hi = vldrbq_gather_offset_z_u8(src, offset_8_16, p);

        //Separate RGB//
        vdst_8_16_r  =  vrshlq(vshrq(vsrc_8_16_hi, 3), 3);

        //[2022-07-12] :Note vrshrq vs vshrq. vrshrq results in round. That is 0x07 rounds to 0x08.
        vdst_8_16_g =  vorrq_u8(vrshlq(vsrc_8_16_hi, 5), vrshlq(vshrq(vsrc_8_16_lo, 5), 2));

        vdst_8_16_b =  vrshlq(vsrc_8_16_lo, 3);

#ifdef PADDING_RGB888_LSB
        //Padding LSB with MSB

        vdst_8_16_r  =  vorrq_u8(vrshlq(vshrq(vsrc_8_16_hi, 3), 3), vshrq(vsrc_8_16_hi, 5));

        vdst_8_16_g =  vorrq_u8(vorrq_u8(vrshlq(vsrc_8_16_hi, 5), vrshlq(vshrq(vsrc_8_16_lo, 5), 2)),  vshrq(vrshlq(vsrc_8_16_hi, 5), 6));

        vdst_8_16_b =  vorrq_u8(vrshlq(vsrc_8_16_lo, 3), vshrq(vrshlq(vsrc_8_16_lo, 3), 5));
#endif
        //Scatter Store//
        offset_8_16 = vcreateq_u8(OFFSET_R_LOW, OFFSET_R_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_r, p);

        offset_8_16 = vcreateq_u8(OFFSET_G_LOW, OFFSET_G_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_g, p);

        offset_8_16 = vcreateq_u8(OFFSET_B_LOW, OFFSET_B_HIGH);

        vstrbq_scatter_offset_p(dst, offset_8_16, vdst_8_16_b, p);

        //Update data source and destination pointer

        //Source: shift one RGB566 pixel size(2 byte)
        src += BYTENUMBER_RGB565 * PIXELS_CONVERT_EACHLOOP;

        //Destination: shift one RGB888 pixel size(3 byte)
        dst += BYTENUMBER_RGB888 * PIXELS_CONVERT_EACHLOOP;

        count -= PIXELS_CONVERT_EACHLOOP;

    }
}
#endif
