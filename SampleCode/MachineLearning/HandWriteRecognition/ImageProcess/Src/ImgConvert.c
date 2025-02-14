/**************************************************************************//**
 * @file     ImgConvert.c
 * @version  V0.10
 * @brief    image resize and color transfer function
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "ImgConvert.h"
//#include "log_macros.h"

#include "imlib.h"
#include "framebuffer.h"

//Used by omv library
__attribute__((section(".sram.data"), aligned(16))) static char fb_array[OMV_FB_SIZE + OMV_FB_ALLOC_SIZE];
__attribute__((section(".sram.data"), aligned(16))) static char jpeg_array[OMV_JPEG_BUF_SIZE];

char *_fb_base = NULL;
char *_fb_end = NULL;
char *_jpeg_buf = NULL;
char *_fballoc = NULL;

static bool s_bImgLibInit = false;

int ImgConvert_Init()
{
    if (s_bImgLibInit == false)
    {
        _fb_base = fb_array;
        _fb_end =  fb_array + OMV_FB_SIZE - 1;
        _fballoc = _fb_base + OMV_FB_SIZE + OMV_FB_ALLOC_SIZE;
        _jpeg_buf = jpeg_array;

        fb_alloc_init0();
        framebuffer_init0();

        s_bImgLibInit = true;
    }

    return 0;
}

