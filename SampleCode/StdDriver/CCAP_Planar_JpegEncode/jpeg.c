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

enum
{
    ePLANE_Y = 0,
    ePLANE_U = 1,
    ePLANE_V = 2,
};

/*
 * To encode planar image to JPEG directly to a memory buffer
 * libJEPG will malloc() the buffer so the caller must free() it when
 * they are finished with it.
 *
 * pu8ImgBufY      - the Y plane data of image
 * pu8ImgBufU      - the U plane data of image
 * pu8ImgBufV      - the V plane data of image
 * u32Width        - the width of the input image
 * u32Height       - the height of the input image
 * u32Quality      - target JPEG 'quality' factor (max 100)
 * strComment      - optional JPEG NULL-termoinated strComment, pass NULL for no comment.
 * pulJpegByteSize - output, the number of bytes in the output JPEG buffer
 * ppu8JpegBuf     - output, a pointer to the output JPEG buffer, must call free() when finished with it.
 */
void JpegEncodePlanarToMem(
    uint8_t *pu8ImgBufY, uint8_t *pu8ImgBufU, uint8_t *pu8ImgBufV,
    uint32_t u32PlanarFmt, uint32_t u32Width, uint32_t u32Height, uint32_t u32Quality,
    const char *strComment, unsigned long *pulJpegByteSize, uint8_t **ppu8JpegBuf)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPARRAY sRawDataBuf[3];
    JSAMPROW   asDataY[16], asDataU[16], asDataV[16];
    int32_t    i, i32HSampFactor, i32VSampFactor;

    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfo);
    cinfo.image_width      = u32Width;
    cinfo.image_height     = u32Height;
    cinfo.input_components = 2;
    cinfo.in_color_space   = JCS_YCbCr;

    jpeg_set_defaults(&cinfo);
    jpeg_set_colorspace(&cinfo, JCS_YCbCr);
    cinfo.raw_data_in = TRUE;
    cinfo.do_fancy_downsampling = FALSE;

    if (u32PlanarFmt == CCAP_PLN_OUTFMT_YUV422P)
    {
        cinfo.comp_info[ePLANE_Y].h_samp_factor = 2;   // For Y
        cinfo.comp_info[ePLANE_Y].v_samp_factor = 1;
    }
    else if (u32PlanarFmt == CCAP_PLN_OUTFMT_YUV420P)
    {
        cinfo.comp_info[ePLANE_Y].h_samp_factor = 2;   // For Y
        cinfo.comp_info[ePLANE_Y].v_samp_factor = 2;
    }
    else
    {
        printf("Not suuport planar format !\n");
        return ;
    }

    cinfo.comp_info[ePLANE_U].h_samp_factor = 1;   // For Cb
    cinfo.comp_info[ePLANE_U].v_samp_factor = 1;
    cinfo.comp_info[ePLANE_V].h_samp_factor = 1;   // For Cr
    cinfo.comp_info[ePLANE_V].v_samp_factor = 1;
    i32HSampFactor = cinfo.comp_info[ePLANE_Y].h_samp_factor;
    i32VSampFactor = cinfo.comp_info[ePLANE_Y].v_samp_factor;

    jpeg_set_quality(&cinfo, u32Quality, (boolean)TRUE);

    // ppu8JpegBuf in this sample is declared in main.c
    // Config libjpeg to encode to specified memory buffer and its buffer size
    jpeg_mem_dest(&cinfo, ppu8JpegBuf, pulJpegByteSize);

    jpeg_start_compress(&cinfo, (boolean)TRUE);

    // Add comment section if any ...
    if (strComment)
    {
        jpeg_write_marker(&cinfo, JPEG_COM, (const JOCTET *)strComment, strlen(strComment));
    }

    sRawDataBuf[ePLANE_Y] = asDataY;
    sRawDataBuf[ePLANE_U] = asDataU;
    sRawDataBuf[ePLANE_V] = asDataV;

    // Encode
    while (cinfo.next_scanline < cinfo.image_height)
    {
        for (i = 0; i < (cinfo.block_size * i32VSampFactor); i++)
        {
            asDataY[i] = &pu8ImgBufY[(cinfo.next_scanline + i) * u32Width];

            if ((i % i32VSampFactor) == 0)
            {
                asDataU[i / i32VSampFactor] = &pu8ImgBufU[(cinfo.next_scanline + i) * (u32Width / i32HSampFactor / i32VSampFactor)];
                asDataV[i / i32VSampFactor] = &pu8ImgBufV[(cinfo.next_scanline + i) * (u32Width / i32HSampFactor / i32VSampFactor)];
            }
        }

        jpeg_write_raw_data(&cinfo, sRawDataBuf, (cinfo.block_size * i32VSampFactor));
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/