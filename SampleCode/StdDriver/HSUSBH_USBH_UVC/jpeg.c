/**************************************************************************//**
 * @file     jepg.c
 * @version  V1.00
 * @brief    JPEG driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "jpeglib.h"
#include "NuMicro.h"

/* To eliminate Warning[Pe188]: enumerated type mixed with another type.
 * TRUE and FALSE are emulation and defined in jmorecfg.h */
#ifdef TRUE
    #undef TRUE
#endif

#ifdef FALSE
    #undef FALSE
#endif

/* To decode packet JPEG to RGB888 directly to a memory buffer
 * libJEPG will malloc() the buffer so the caller must free() it when
 * they are finished with it.
 *
 * image    - the input JPEG image.
 * jpegSize - output, the number of bytes in the JPEG buffer
 * jpegBuf  - output, a pointer to the output RGB888 buffer.
 *
 * return int     Returns 1 (true) if the JPEG decode done, 0 (false) otherwise.
 */
int decode_jpeg_to_rgb888(unsigned char *image, unsigned long jpegSize, unsigned char *rgb888Buf)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    uint32_t row_stride;
    int ret = FALSE;

    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_decompress(&cinfo);

    // jpegBuf in this sample is declared in main.c
    // Config libjpeg to encode to specified memory buffer and its buffer size
    jpeg_mem_src(&cinfo, image, jpegSize);
    /* jpeg_read_header() */
    ret = jpeg_read_header(&cinfo, TRUE);

    if (ret == FALSE)
        return (ret);

    cinfo.out_color_space = JCS_RGB;

    ret = jpeg_start_decompress(&cinfo);

    if (ret == FALSE)
        return (ret);

    row_stride = cinfo.output_width * 3 ;

    while (cinfo.output_scanline < cinfo.output_height)
    {
        unsigned char *buffer_array[1];
        buffer_array[0] = rgb888Buf + \
                          (cinfo.output_scanline) * row_stride;

        ret = jpeg_read_scanlines(&cinfo, buffer_array, 1);

        if (ret == FALSE)
            return (ret);

    }

    ret = jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    return (ret);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/