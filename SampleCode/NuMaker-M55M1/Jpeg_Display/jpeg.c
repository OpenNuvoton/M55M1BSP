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
#include "ff.h"
#include "imlib.h"

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
    #include <strings.h>
#endif




#include "main.h"
#include "Display.h"
#undef PI
#include "NuMicro.h"//to check: must after jpeglib.h
/* For libjpeg FatFs source usage*/
#define INPUT_BUF_SIZE  4096

/* For list all jpeg files. */
#define MAX_PHOTOS 32          // Max 100 files
#define MAX_PATH_SIZE 64       // Max path is 64 Byte



typedef struct
{
    struct jpeg_source_mgr pub;
    FIL *infile;
    JOCTET *buffer;
    boolean start_of_file;
} my_source_mgr;

typedef my_source_mgr *my_src_ptr;

typedef struct
{
    char file_paths[MAX_PHOTOS][MAX_PATH_SIZE]; // array of path
    uint16_t total_count;                       // file count
    uint16_t current_index;                     // current file index
} Playlist_t;


typedef struct
{
    unsigned int img_w; /* nominal image width (from SOF marker) */
    unsigned int img_h; /* nominal image height */
    unsigned int num;     /* scaling num */
    unsigned int denom; /* nominal denom */
} my_cinfo;

my_cinfo decode_info;

Playlist_t myPlaylist = { .total_count = 0, .current_index = 0 };




uint16_t jpeg_get_best_denom_factor(uint16_t img_w, uint16_t img_h, uint16_t *pscaling_denom);

int is_jpg(const char *filename)
{
    // 1. Find the position of last '.'
    const char *ext = strrchr(filename, '.');

    // 2. If no '.' present or '.' is the last char, no subname
    if (!ext || ext == filename)
    {
        return 0;
    }

    // 3. Check if the subname is .jpg ? .jpeg (upper or little cases are ok)
    // Use strcasecmp to take compatibility to SD cards
    // formatted in Windows.(subname such as .JPG or .Jpg)
    if (strcasecmp(ext, ".jpg") == 0 || strcasecmp(ext, ".jpeg") == 0)
    {
        return 1; // found JPEG file
    }

    return 0; // not a JPEG file
}

FRESULT collect_photos(char *path)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;

    res = f_opendir(&dir, path);

    if (res != FR_OK) return res;

    for (;;)
    {
        res = f_readdir(&dir, &fno);

        if (res != FR_OK || fno.fname[0] == 0) break;

        // Skip folder name start with '.'
        if (fno.fname[0] == '.') continue;

        char full_path[MAX_PATH_SIZE];
        /*
            todo: check full path length check, must smaller than MAX_PATH_SIZE
        */
        snprintf(full_path, sizeof(full_path), "%s/%s", path, fno.fname);

        if (fno.fattrib & AM_DIR)
        {
            // enter child folder
            collect_photos(full_path);
        }
        else
        {
            // Check if subname is JPG
            if (is_jpg(fno.fname) && myPlaylist.total_count < MAX_PHOTOS)
            {
                strncpy(myPlaylist.file_paths[myPlaylist.total_count],
                        full_path, MAX_PATH_SIZE - 1);
                myPlaylist.total_count++;
            }
        }
    }

    f_closedir(&dir);
    return res;
}


// Get the file path of current file to display
char *Playlist_GetCurrent()
{
    DBG_msg("total_count =%d \r\n", myPlaylist.total_count);
    DBG_msg("current_index =%d \r\n", myPlaylist.current_index);

    if (myPlaylist.total_count == 0) return NULL;

    return myPlaylist.file_paths[myPlaylist.current_index];
}

// Move file index to consequent file
int8_t Playlist_Next()
{
    if (myPlaylist.total_count > 0)
    {
        myPlaylist.current_index = (myPlaylist.current_index + 1) % myPlaylist.total_count;

        if (myPlaylist.current_index == 0)
        {
            DBG_msg("loop overflow\r\n");
            return -1;
        }

        return myPlaylist.current_index;
    }

    return 0;
}

// Move file index to next random file
void Playlist_Shuffle()
{
    if (myPlaylist.total_count > 0)
    {
        myPlaylist.current_index = rand() % myPlaylist.total_count;
    }
}

// A. Init
static void init_source(j_decompress_ptr cinfo)
{
    my_src_ptr src = (my_src_ptr) cinfo->src;
    src->start_of_file = TRUE;
}

// B. Fill the buffer
static boolean fill_input_buffer(j_decompress_ptr cinfo)
{
    my_src_ptr src = (my_src_ptr) cinfo->src;
    UINT br;

    // Read from SD and send to buffer
    FRESULT res = f_read(src->infile, src->buffer, INPUT_BUF_SIZE, &br);

    if (res != FR_OK || br == 0)
    {
        // If EOF or error occurs, pad it with(EOI)
        src->buffer[0] = (JOCTET) 0xFF;
        src->buffer[1] = (JOCTET) JPEG_EOI;
        br = 2;
    }

    src->pub.next_input_byte = src->buffer;
    src->pub.bytes_in_buffer = br;
    src->start_of_file = FALSE;

    return TRUE;
}

// C. skip header/tag (useless)
static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_src_ptr src = (my_src_ptr) cinfo->src;

    if (num_bytes > 0)
    {
        while (num_bytes > (long) src->pub.bytes_in_buffer)
        {
            num_bytes -= (long) src->pub.bytes_in_buffer;
            (void) fill_input_buffer(cinfo);
        }

        src->pub.next_input_byte += (size_t) num_bytes;
        src->pub.bytes_in_buffer -= (size_t) num_bytes;
    }
}

// D. Handle the end
static void term_source(j_decompress_ptr cinfo)
{
    /* Left Nothing */
}


void jpeg_fatfs_src(j_decompress_ptr cinfo, FIL *infile)
{
    my_src_ptr src;

    // source manager
    if (cinfo->src == NULL)
    {
        cinfo->src = (struct jpeg_source_mgr *)
                     (*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
        src = (my_src_ptr) cinfo->src;
        src->buffer = (JOCTET *)
                      (*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, INPUT_BUF_SIZE * sizeof(JOCTET));
    }

    src = (my_src_ptr) cinfo->src;
    src->pub.init_source = init_source;
    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart; // default of libjpeg
    src->pub.term_source = term_source;
    src->infile = infile; // hook to FatFs' FIL pointer
    src->pub.bytes_in_buffer = 0;
    src->pub.next_input_byte = NULL;
}


/* To encode packet image to JPEG directly to a memory buffer
 * libJEPG will malloc() the buffer so the caller must free() it when
 * they are finished with it.
 *
 * image    - the input greyscale image, 1 byte is 1 pixel.
 * width    - the width of the input image
 * height   - the height of the input image
 * quality  - target JPEG 'quality' factor (max 100)
 * comment  - optional JPEG NULL-termoinated comment, pass NULL for no comment.
 * jpegSize - output, the number of bytes in the output JPEG buffer
 * jpegBuf  - output, a pointer to the output JPEG buffer, must call free() when finished with it.
 */
void encode_jpeg_to_memory(unsigned char *image, int width, int height, int quality,
                           const char *comment, unsigned long *jpegSize, unsigned char **jpegBuf)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    JSAMPROW row_pointer[1];
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfo);
    cinfo.image_width = width;
    cinfo.image_height = height;

#if (TEST_GRAYSCALE == 1)
    // Input is GRAYSCALE, 1 byte per pixel
    cinfo.input_components = 1;
    cinfo.in_color_space   = JCS_GRAYSCALE;
#else
    // Input is RGB888, 3 byte per pixel
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;
#endif

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, (boolean)TRUE);

    // jpegBuf in this sample is declared in main.c
    // Config libjpeg to encode to specified memory buffer and its buffer size
    jpeg_mem_dest(&cinfo, jpegBuf, jpegSize);

    jpeg_start_compress(&cinfo, (boolean)TRUE);

    // Add comment section if any..
    if (comment)
    {
        jpeg_write_marker(&cinfo, JPEG_COM, (const JOCTET *)comment, strlen(comment));
    }

    // raw stride = width * byte per pixel
    row_stride = width * cinfo.input_components;

    // Encode
    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = &image[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
}

void JpegEncode(unsigned char *image, unsigned char *jBuf, unsigned long *jSize, int width, int height)
{
    encode_jpeg_to_memory(image, width, height, 85, "Nuvoton", jSize, &jBuf);

    DBG_msg("JPEG image buffer range: 0x%08x to 0x%08x, JPEG size (bytes): %ld\n",
            (unsigned int)jBuf,
            (unsigned int)jBuf + (unsigned int)*jSize,
            *jSize);
}


/* To decode JPEG file to RGB888 image buffer.
 *
 * jfile    - the input jpeg file name.
 * imgBuf   - output, a pointer to the decoded image buffer.
 */
//width and height of the Jpeg file image are in jpeg file header
void decode_jpeg_to_memory(unsigned char *jfile, unsigned char *imgBuf)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;

    JSAMPROW row_pointer[1];

    int row_stride;

    FIL file;

    if (f_open(&file, (const char *)(jfile), FA_READ) != FR_OK)
    {
        DBG_msg("f_open %s fails!!\r\n", jfile);
        return;
    }

    DBG_msg("f_open %s pass!!\r\n", jfile);


    cinfo.err = jpeg_std_error(&jerr);

    jpeg_create_decompress(&cinfo);


    // jpegBuf in this sample is declared in main.c
    // Config libjpeg to decode from specified memory buffer and its buffer size.
    jpeg_fatfs_src(&cinfo, &file);

    //Read jpeg file header
    jpeg_read_header(&cinfo, TRUE);

#if (TEST_GRAYSCALE_DECODE == 1)
    // Support GRAYSCALE Output?
    /*
    todo
            cinfo.output_components = 1;
    cinfo.out_color_space   = JCS_GRAYSCALE;
    */
    //Default as follow, based on jpeg file header
    //Target Out is RGB888, 3 byte per pixel
    //cinfo.output_components = 3;
    //cinfo.out_color_space    = JCS_RGB;
#endif



    //Use scaling in libjpeg
    printf("Jpg file dimension: %d x %d\n", cinfo.image_width, cinfo.image_height);
    decode_info.num = jpeg_get_best_denom_factor(cinfo.image_width, cinfo.image_height, (uint16_t *)(&decode_info.denom));

    cinfo.scale_num = decode_info.num;
    cinfo.scale_denom = SCALING_DENOM;

    //      float fscale = (float)(num)/ (float)(SCALING_DENOM);
    //      DBG_msg("Suitable Scaling: num=%d , denom=%d, fscale=%f \n", cinfo.scale_num, cinfo.scale_denom, fscale);

    jpeg_start_decompress(&cinfo);

    DBG_msg("Decoded output dimension: %d x %d\n", cinfo.output_width, cinfo.output_height);

    decode_info.img_w = cinfo.output_width;
    decode_info.img_h = cinfo.output_height;

    // row stride = width * byte per pixel
    row_stride = cinfo.output_width * cinfo.output_components;

    // Decode
    while (cinfo.output_scanline < cinfo.output_height)
    {

        row_pointer[0] = imgBuf + (row_stride * cinfo.output_scanline);
        jpeg_read_scanlines(&cinfo, row_pointer, 1);

    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

}

void JpegDecode(unsigned char *jFile, unsigned char *image)
{
    decode_jpeg_to_memory(jFile, image);
    DBG_msg("Decoded JPEG image buffer range: 0x%08x \n", (unsigned int)image);
}

//Fixed Scaling_Denom to 8, then try num from 1 to 8.
//The return value here is the scaling num.
uint16_t jpeg_get_best_denom_factor(uint16_t img_w, uint16_t img_h, uint16_t *pscaling_denom)
{

    //uint16_t ii;
    *pscaling_denom = SCALING_DENOM;
    float ratio_w = (float)img_w / PANEL_DISPLAY_WIDTH_MAX;
    float ratio_h = (float)img_h / PANEL_DISPLAY_HEIGHT_MAX;

    if (ratio_w < 0) return 0;

    if (ratio_h < 0) return 0;

    if (ratio_w < ratio_h)
    {
        // Hight limited
        DBG_msg("jpeg denom height scaling: %d / %d  \r\n", (img_h), PANEL_DISPLAY_HEIGHT_MAX);

        for (uint16_t ii = SCALING_DENOM; ii > 0; ii--)
        {
            if ((((float)(ii) / SCALING_DENOM) * img_h) <= PANEL_DISPLAY_HEIGHT_MAX)
            {
                DBG_msg("height scaling: %d/8 is fine. \r\n", ii);
                return ii;

            }

        }



    }
    else
    {
        // Width limited
        DBG_msg("jpeg denom width scaling: %d / %d  \r\n", (img_w), PANEL_DISPLAY_WIDTH_MAX);

        for (uint16_t ii = SCALING_DENOM; ii > 0; ii--)
        {
            if ((((float)(ii) / SCALING_DENOM) * img_w) <= PANEL_DISPLAY_WIDTH_MAX)
            {
                DBG_msg("width scaling: %d/8 is fine. \r\n", ii);
                return ii;

            }

        }


    }

    return 0;
}

void jpeg_get_output_size(uint16_t *img_w, uint16_t *img_h)
{
    *img_w =  decode_info.img_w;
    *img_h =  decode_info.img_h;
}

void jpeg_convert_RGB888toRGB565_SW(uint8_t *pu8Data, uint32_t u32PixelCount)
{
    // Read ptr 3 bytes each(R, G, B)
    uint8_t *pu8Read = pu8Data;

    // Write ptr 2 bytes rach (uint16_t)
    uint16_t *pu16Write = (uint16_t *)pu8Data;

    for (uint32_t i = 0; i < u32PixelCount; i++)
    {
        // Get R, G, B
        uint8_t r = pu8Read[0];
        uint8_t g = pu8Read[1];
        uint8_t b = pu8Read[2];

        // Convert 888 to 565
        *pu16Write = ((uint16_t)(r >> 3) << 11) |
                     ((uint16_t)(g >> 2) << 5)  |
                     ((uint16_t)(b >> 3));

        // update ptr
        pu8Read += 3;
        pu16Write += 1;
    }
}

void jpeg_compute_roi_centering(S_DISP_RECT *pDispRect, uint16_t u16_w, uint16_t u16_h)
{
    pDispRect->u32TopLeftX     = (PANEL_WIDTH  - u16_w) / 2;
    pDispRect->u32TopLeftY     = (PANEL_HEIGHT - u16_h) / 2;
    pDispRect->u32BottonRightX = (pDispRect->u32TopLeftX) + u16_w - 1;
    pDispRect->u32BottonRightY = (pDispRect->u32TopLeftY) + u16_h - 1;

    DBG_msg("jpeg_compute_roi_centering, (%d,%d), (%d,%d) \r\n",
            pDispRect->u32TopLeftX, pDispRect->u32TopLeftY,
            pDispRect->u32BottonRightX, pDispRect->u32BottonRightY);
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
