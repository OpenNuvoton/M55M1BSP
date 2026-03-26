#ifndef __MAIN_H__
#define __MAIN_H__
// Includes

#include <stdint.h>
#include "imlib.h"
#include "ff.h"
#include "Display.h"
// Macros
#if ENABLE_DEBUG
    #define DBG_msg   printf
#else
    #define DBG_msg(...)   do { } while (0)       /* disable debug */
#endif


// Used by omv library
#if defined(__USE_UVC__)
    //UVC only support QVGA, QQVGA
    #define GLCD_WIDTH  320
    #define GLCD_HEIGHT 240
#else
    #define GLCD_WIDTH  160
    #define GLCD_HEIGHT 120
#endif

//RGB565
#define IMAGE_FB_SIZE   (GLCD_WIDTH * GLCD_HEIGHT * 2)

#undef OMV_FB_SIZE
#define OMV_FB_SIZE (IMAGE_FB_SIZE + 1024/4)

//Panel Size Setting
#define PANEL_WIDTH              800U
#define PANEL_HEIGHT             480U

#define TEXT_OMV_OFFSET_W          8U
#define TEXT_OMV_OFFSET_H          8U

//Panel Resolution Setting
#if defined(PANEL_DISPLAY_480X480)
    #define PANEL_DISPLAY_WIDTH_MAX  480U
    #define PANEL_DISPLAY_HEIGHT_MAX 480U
#elif defined(PANEL_DISPLAY_320X240)
    #define PANEL_DISPLAY_WIDTH_MAX  320U
    #define PANEL_DISPLAY_HEIGHT_MAX 240U
#else
    #define PANEL_DISPLAY_WIDTH_MAX  600U
    #define PANEL_DISPLAY_HEIGHT_MAX 480U
#endif


//Image Scaling Parameter for libjpeg
#define SCALING_DENOM              8U


//Enumeration


/* Structure Definition*/

typedef enum
{
    eFRAMEBUF_EMPTY,
    eFRAMEBUF_FULL,
    eFRAMEBUF_INF
} E_FRAMEBUF_STATE;

typedef struct
{
    E_FRAMEBUF_STATE eState;
    image_t frameImage;
} S_FRAMEBUF;

//Struct

// Functions

#ifdef  __cplusplus
extern  "C" {
#endif
extern FRESULT collect_photos(char *path) ;
extern char *Playlist_GetCurrent();
extern int8_t Playlist_Next();
extern void Playlist_Shuffle();

extern void jpeg_get_output_size(uint16_t *img_w, uint16_t *img_h);
extern void jpeg_convert_RGB888toRGB565_SW(uint8_t *pu8Data, uint32_t u32PixelCount);
extern void jpeg_compute_roi_centering(S_DISP_RECT *pDispRect, uint16_t u16_w, uint16_t u16_h);

void JpegEncode(unsigned char *image, unsigned char *jBuf, unsigned long *jSize, int width, int height);
void JpegDecode(unsigned char *jFile, unsigned char *image);
#ifdef  __cplusplus
}
#endif
#endif  /* __BOARD_M55M1_H__ */
