/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This example demonstrates how to use the USBH library to
 *           connect to a USB Video Class Device, capture the MJPEG or YUYV bit
 *           stream, and use the JPEGLibrary decoder to decode
 *           the JPEG image or YUYV conversion RGB565 image
 *           and output it to display.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "jpeglib.h"
#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_uvc.h"
#include "Display.h"


#ifdef DEBUG_ENABLE_SEMIHOST
    #error This sample cannot execute with semihost enabled
#endif

#define DISPALY     1

#define USE_USB_APLL1_CLOCK         1

#define IMAGE_DISP_UPSCALE_FACTOR 1
#define FONT_DISP_UPSCALE_FACTOR 2

#define START_CAPTURE           0x01
#define END_CAPTURE             0x00

#if 0
    #define DEMO_YUYV_FORMAT
#else
    #define DEMO_MJPEG_FORMAT
#endif
/*----------------------------------------------------------------------
 * USB UVC
 */
#define SELECT_RES_WIDTH     (640)
#define SELECT_RES_HEIGHT    (480)
#ifdef DEMO_YUYV_FORMAT
    #define IMAGE_MAX_SIZE       (SELECT_RES_WIDTH * SELECT_RES_HEIGHT * 2)
#else
    #define IMAGE_MAX_SIZE       (SELECT_RES_WIDTH * SELECT_RES_HEIGHT)
#endif

#define IMAGE_BUFF_CNT       2
#define RGB_IMAGE_MAX_SIZE   (SELECT_RES_WIDTH * SELECT_RES_HEIGHT)

enum
{
    IMAGE_BUFF_FREE,
    IMAGE_BUFF_USB,
    IMAGE_BUFF_READY,
    IMAGE_BUFF_POST
};

struct img_buff_t
{
    uint8_t   *buff;
    int       len;
    int       state;
    int       sti;
};


//YUV or MJEPG Format
struct img_buff_t  _imgs[IMAGE_BUFF_CNT];
#ifdef DEMO_YUYV_FORMAT
    //Because the internal SRAM cannot store two 640*480 images.
    uint8_t  image_buff_pool[IMAGE_BUFF_CNT - 1][IMAGE_MAX_SIZE] __attribute__((aligned(32)));
#else
    uint8_t  image_buff_pool[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE] __attribute__((aligned(32)));
#endif
volatile int   _idx_usb = 0, _idx_post = 0;
int   _total_frame_count = 0;

#define STRIP_LINES          (40)
#define STRIP_PIXELS         (SELECT_RES_WIDTH * STRIP_LINES)
#define NUM_STRIPS           (SELECT_RES_HEIGHT / STRIP_LINES)

uint16_t rgb565_strip_buff[STRIP_PIXELS] __attribute__((aligned(32)));

/* Use a full-frame rgb888 buffer (still needed for full-frame decoder) */
/* This requires enough memory - consider using non-cacheable SRAM */
static uint8_t rgb888_strip_buff[SELECT_RES_WIDTH * STRIP_LINES * 3] __attribute__((aligned(32)));

static volatile uint32_t s_u32TickCnt;

extern int kbhit(void);                        /* function in retarget.c                 */
void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);

void SYS_Init(void);
/**
*/
// limit 0-255 for RGB
static inline uint8_t clamp(int value)
{
    if (value < 0) return 0;

    if (value > 255) return 255;

    return (uint8_t)value;
}

/**
 * @brief Converts a strip of YUV422 (YUYV) image to RGB565 format.
 *
 * @param yuv_buffer Pointer to the start of this strip in the YUV422 image.
 * @param rgb_buffer Pointer to the output RGB565 strip buffer.
 * @param width      Width of the image in pixels.
 * @param lines      Number of lines in this strip.
 */
void yuv422_to_rgb565_strip(uint8_t *yuv_buffer, uint16_t *rgb_buffer, int width, int lines)
{
    int strip_size = width * lines * 2; /* YUV422: 2 bytes per pixel */
    int rgb_index = 0;

    for (int i = 0; i < strip_size; i += 4)
    {
        uint8_t Y0 = yuv_buffer[i];
        uint8_t U  = yuv_buffer[i + 1];
        uint8_t Y1 = yuv_buffer[i + 2];
        uint8_t V  = yuv_buffer[i + 3];

        int C = Y0;
        int D = U - 128;
        int E = V - 128;

        int R = clamp(C + ((179 * E) >> 7));
        int G = clamp(C - ((44 * D + 91 * E) >> 7));
        int B = clamp(C + ((227 * D) >> 7));

        rgb_buffer[rgb_index++] = ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);

        C = Y1;

        R = clamp(C + ((179 * E) >> 7));
        G = clamp(C - ((44 * D + 91 * E) >> 7));
        B = clamp(C + ((227 * D) >> 7));

        rgb_buffer[rgb_index++] = ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
    }
}

/**
 * @brief Converts a strip of RGB888 image to RGB565 format.
 *
 * @param src    Pointer to the input RGB888 strip data.
 * @param dst    Pointer to the output RGB565 strip data.
 * @param pixels The total number of pixels in this strip.
 */
void rgb888_to_rgb565_strip(const uint8_t *src, uint16_t *dst, int pixels)
{
    for (int i = 0; i < pixels; i++)
    {
        uint8_t r = src[3 * i + 0];
        uint8_t g = src[3 * i + 1];
        uint8_t b = src[3 * i + 2];

        uint16_t r5 = (r >> 3) & 0x1F;
        uint16_t g6 = (g >> 2) & 0x3F;
        uint16_t b5 = (b >> 3) & 0x1F;
        dst[i] = (r5 << 11) | (g6 << 5) | b5;
    }
}

/* To eliminate Warning[Pe188]: enumerated type mixed with another type.
 * TRUE and FALSE are emulation and defined in jmorecfg.h */
#ifdef TRUE
    #undef TRUE
#endif

#ifdef FALSE
    #undef FALSE
#endif

/**
 * @brief  Decode MJPEG frame using libjpeg scanline API, strip-by-strip.
 *
 * @param  jpeg_buf    Pointer to the complete MJPEG frame data.
 * @param  jpeg_size   Size of the MJPEG frame in bytes.
 *
 * @return 0 on success, -1 on decode failure.
 *
 * @details
 * Uses jpeg_read_scanlines() to read STRIP_LINES lines at a time,
 * convert rgb888 -> rgb565, and display each strip immediately.
 *
 * Memory usage:
 *   rgb888_strip_buff: STRIP_LINES * width * 3 bytes (e.g., 40*640*3 = 75KB)
 *   rgb565_strip_buff: STRIP_LINES * width * 2 bytes (e.g., 40*640*2 = 50KB)
 *   No full-frame rgb888 buffer needed!
 *
 * This replaces the old decode_jpeg_to_rgb888() + full-frame approach.
 */
int mjpeg_decode_strip_display(uint8_t *jpeg_buf, int jpeg_size)
{
    static struct jpeg_decompress_struct cinfo;
    static struct jpeg_error_mgr jerr;
    static int initialized = 0;
    int      lines_in_strip;
    int      strip_y_start;
#if (DISPALY == 1)
    S_DISP_RECT sDispRect;
#endif

    /* Check JPEG SOI marker */
    if (jpeg_size < 2 || jpeg_buf[0] != 0xFF || jpeg_buf[1] != 0xD8)
    {
        printf("Invalid JPEG SOI! 0x%x/0x%x/%d\n", jpeg_buf[0], jpeg_buf[1], jpeg_size);
        return -1;
    }

#if 0
    /*--- Setup error handler (must be before jpeg_create_decompress) ---*/
    cinfo.err = jpeg_std_error(&jerr);

    /*--- Create decompressor and set source ---*/
    jpeg_create_decompress(&cinfo);
#else

    if (!initialized)
    {
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_decompress(&cinfo);
        initialized = 1;
    }

#endif
    jpeg_mem_src(&cinfo, jpeg_buf, (unsigned long)jpeg_size);

    /*--- Read JPEG header ---*/
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK)
    {
        printf("JPEG header error!\n");
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }

    /*--- Configure output: RGB888, no scaling ---*/
    cinfo.out_color_space = JCS_RGB;
    cinfo.dct_method = JDCT_IFAST;
    cinfo.do_fancy_upsampling = FALSE;
    cinfo.do_block_smoothing = FALSE;
    cinfo.dither_mode = JDITHER_NONE;


    /*--- Start decompression ---*/
    if (!jpeg_start_decompress(&cinfo))
    {
        printf("jpeg_start_decompress failed!\n");
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }

    /*
     *  Verify dimensions match our expected resolution.
     *  output_width / output_height are valid after jpeg_start_decompress().
     */
    if (cinfo.output_width != SELECT_RES_WIDTH ||
            cinfo.output_height != SELECT_RES_HEIGHT ||
            cinfo.output_components != 3)
    {
        printf("JPEG dimension mismatch: %dx%d (expected %dx%d)\n",
               cinfo.output_width, cinfo.output_height,
               SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
        jpeg_finish_decompress(&cinfo);
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }

    /*
     *  Read scanlines strip-by-strip.
     *
     *  jpeg_read_scanlines() reads up to max_lines scanlines per call.
     *  cinfo.output_scanline tracks the current row (0 .. output_height-1).
     *  Each scanline is output_width * output_components bytes of RGB888.
     */
    /* Use a static buffer large enough for STRIP_LINES of RGB888 */

    strip_y_start = 0;
    lines_in_strip = 0;

    while (cinfo.output_scanline < cinfo.output_height)
    {
        /*
         *  Build an array of row pointers for the entire strip.
         *  jpeg_read_scanlines can read multiple lines at once,
         *  which is MUCH faster than reading 1 line at a time
         *  (avoids per-call overhead * 360 calls -> only 9 calls).
         */
        int lines_to_read = STRIP_LINES;

        /* Handle last strip if image height is not perfectly divisible */
        if (cinfo.output_scanline + lines_to_read > cinfo.output_height)
            lines_to_read = cinfo.output_height - cinfo.output_scanline;

        /* Build row pointer array for this strip */
        JSAMPROW row_pointers[STRIP_LINES];

        for (int r = 0; r < lines_to_read; r++)
        {
            row_pointers[r] = (JSAMPROW)(rgb888_strip_buff +
                                         (r * SELECT_RES_WIDTH * 3));
        }

        /* Read all lines of this strip in one call */
        lines_in_strip = 0;

        while (lines_in_strip < lines_to_read)
        {
            JDIMENSION rows_read = jpeg_read_scanlines(
                                       &cinfo,
                                       &row_pointers[lines_in_strip],
                                       lines_to_read - lines_in_strip);

            if (rows_read == 0)
                break;

            lines_in_strip += (int)rows_read;
        }

        if (lines_in_strip == 0)
            break;

        /* Convert RGB888 -> RGB565 for this strip */
        rgb888_to_rgb565_strip(rgb888_strip_buff,
                               rgb565_strip_buff,
                               SELECT_RES_WIDTH * lines_in_strip);

#if (DISPALY == 1)
        sDispRect.u32TopLeftX     = 0;
        sDispRect.u32TopLeftY     = strip_y_start * IMAGE_DISP_UPSCALE_FACTOR;
        sDispRect.u32BottonRightX = (SELECT_RES_WIDTH * IMAGE_DISP_UPSCALE_FACTOR) - 1;
        sDispRect.u32BottonRightY = ((strip_y_start + lines_in_strip) * IMAGE_DISP_UPSCALE_FACTOR) - 1;

        Display_FillRect((uint16_t *)rgb565_strip_buff,
                         &sDispRect, IMAGE_DISP_UPSCALE_FACTOR);
#endif

        strip_y_start += lines_in_strip;
    }

    /*--- Finish decompression ---*/
    jpeg_finish_decompress(&cinfo);
    //jpeg_destroy_decompress(&cinfo);

    return 0;
}

/**
 * @brief  Process YUYV frame strip-by-strip to RGB565 and display.
 *
 * @param  yuv_buf   Pointer to the complete YUYV frame data.
 * @param  yuv_size  Size of the YUYV frame in bytes.
 *
 * @details
 * YUYV data is sequential scanline-based, so true strip processing
 * works perfectly. Each strip is independent - no decoder state needed.
 */
void yuyv_strip_display(uint8_t *yuv_buf, int yuv_size)
{
#if (DISPALY == 1)
    S_DISP_RECT sDispRect;
#endif
    int strip_yuv_size = SELECT_RES_WIDTH * STRIP_LINES * 2;

    (void)yuv_size;

    for (int strip = 0; strip < NUM_STRIPS; strip++)
    {
        int y_start = strip * STRIP_LINES;
        uint8_t *yuv_src = yuv_buf + (strip * strip_yuv_size);

        /* Convert this strip: YUV422 -> RGB565 */
        yuv422_to_rgb565_strip(yuv_src, rgb565_strip_buff, SELECT_RES_WIDTH, STRIP_LINES);

#if (DISPALY == 1)
        /* Display this strip at the correct Y position */
        sDispRect.u32TopLeftX     = 0;
        sDispRect.u32TopLeftY     = y_start * IMAGE_DISP_UPSCALE_FACTOR;
        sDispRect.u32BottonRightX = (SELECT_RES_WIDTH * IMAGE_DISP_UPSCALE_FACTOR) - 1;
        sDispRect.u32BottonRightY = ((y_start + STRIP_LINES) * IMAGE_DISP_UPSCALE_FACTOR) - 1;

        Display_FillRect((uint16_t *)rgb565_strip_buff, &sDispRect, IMAGE_DISP_UPSCALE_FACTOR);
#endif
    }
}

/**
 * @brief    Check any char input from UART
 *
 * @param    None
 *
 * @retval   1: No any char input
 * @retval   0: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */

int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U);
}

NVT_ITCM void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;

    if (SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks(void)
{
    return s_u32TickCnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC_12M. Prescale 12
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    TIMER_SET_PRESCALE_VALUE(TIMER0, (12 - 1));
    /* stop timer0 */
    TIMER_Stop(TIMER0);
    /* write 1 to clear for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* set timer cmp value */
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    /* Timer0 config to oneshot mode */
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* start timer0*/
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

#if (USE_USB_APLL1_CLOCK)
    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);
#endif

    /* Enable GPIOA module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT*/
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

#if (USE_USB_APLL1_CLOCK)
    /* USB Host desired input clock is 48 MHz. Set as APLL1 divided by 2 (96/2 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));
#else
    /* USB Host desired input clock is 48 MHz. Set as HIRC48M divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH0_MODULE);
    CLK_EnableModuleClock(USBD0_MODULE);
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x1ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;
    //delay_us(20);

    /* Set Debug Uart CLK*/
    SetDebugUartCLK();
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}

void  init_image_buffers(void)
{
    int   i;

    for (i = 0; i < IMAGE_BUFF_CNT; i++)
    {
#ifdef DEMO_YUYV_FORMAT
        //Because the internal SRAM cannot store two 640*480 images, so reuse image_buff_pool.
        _imgs[i].buff = image_buff_pool[0];
#else
        _imgs[i].buff = image_buff_pool[i];
#endif
        _imgs[i].len = 0;
        _imgs[i].state = IMAGE_BUFF_FREE;
        _imgs[i].sti = 0;
    }

    _idx_usb = 0;
    _idx_post = 0;
}

int  uvc_rx_callbak(UVC_DEV_T *vdev, uint8_t *data, int len)
{
    int  next_idx = 0;

    NVT_UNUSED(data);

    //printf("RX: %d\n", len);
    _total_frame_count++;

    next_idx = (_idx_usb + 1) % IMAGE_BUFF_CNT;

    if (_imgs[next_idx].state != IMAGE_BUFF_FREE)
    {
        /*
         *  Next image buffer is in used.
         *  Just drop this newly received image and reuse the same image buffer.
         */
        // sysprintf("Drop!\n");
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
    }
    else
    {
        _imgs[_idx_usb].state = IMAGE_BUFF_READY;   /* mark the current buffer as ready for decode/display */
        _imgs[_idx_usb].len   = len;                /* length of this newly received image   */
        _imgs[_idx_usb].sti   = vdev->img_sti;
        /* proceed to the next image buffer */
        _idx_usb = next_idx;
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;     /* mark the next image as used by USB    */

        /* assign the next image buffer to receive next image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
    }

    return 0;
}

void show_menu()
{
    printf("\n\n+---------------------------------------------+\n");
    printf("|  Operation menu                             |\n");
    printf("+---------------------------------------------+\n");
    printf("|  [1] Stop video streaming                   |\n");
    printf("|  [2] Start video streaming                  |\n");
    printf("|  [3] Start video Still Image Capture        |\n");
    printf("+---------------------------------------------+\n\n");
    usbh_memory_used();
}

UVC_DEV_T *uvc_conn_check(UVC_DEV_T *cur_vdev)
{
    UVC_DEV_T *vdev = cur_vdev;
    IMAGE_FORMAT_E  format;
    int width, height;
    int i, ret;

    if (usbh_pooling_hubs())       /* USB Host port detect polling and management */
    {
        /*
         *  Has hub port event.
         */
        vdev = usbh_uvc_get_device_list();

        if (vdev == NULL)
        {
            cur_vdev = NULL;
            printf("\n[No UVC device connected]\n\n");
            return NULL;
        }

        if (cur_vdev == vdev)
        {
            printf("\n\n\nWaiting for UVC device connected...\n");
            return vdev;
        }

        if (vdev->next != NULL)
        {
            printf("\n\nWarning!! Multiple UVC device is not supported!!\n\n");
            getchar();
            return cur_vdev;
        }

        /*----------------------------------------------------------------------------*/
        /*  New UVC device connected.                                                 */
        /*----------------------------------------------------------------------------*/
        cur_vdev = vdev;
        printf("\n\n----------------------------------------------------------\n");
        printf("[Video format list]\n");

        for (i = 0; ; i++)
        {
            ret = usbh_get_video_still_format(cur_vdev, i, &format, &width, &height);

            if (ret != 0)
                break;

            printf("[Still][%d] %s, %d x %d\n", i, (format == UVC_FORMAT_MJPEG ? "MJPEG" : "YUYV"), width, height);
        }

        printf("\n");

        for (i = 0; ; i++)
        {
            ret = usbh_get_video_format(cur_vdev, i, &format, &width, &height);

            if (ret != 0)
                break;

            printf("[%d] %s, %d x %d\n", i, (format == UVC_FORMAT_MJPEG ? "MJPEG" : "YUYV"), width, height);
        }

        printf("\n\n");
#ifdef DEMO_YUYV_FORMAT
        ret = usbh_set_video_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
        ret = usbh_set_video_still_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else
        ret = usbh_set_video_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
        ret = usbh_set_video_still_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#endif

        if (ret != 0)
            printf("usbh_set_video_format failed! - 0x%x\n", ret);

        init_image_buffers();

        delay_us(200);//wait UVC Device command effective.

        /* assign the first image buffer to receive the image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;


        ret = usbh_uvc_start_streaming(vdev, uvc_rx_callbak);

        if (ret < 0)
        {
            printf("Start still straming faill..\n");
        }

        show_menu();
    }

    return cur_vdev;
}


int32_t main(void)
{
    UVC_DEV_T       *vdev = NULL;
    int             command, ret;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    InitDebugUart();                   /* Init DeubgUART for printf */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
#if defined (DEMO_YUYV_FORMAT)
    printf("|        USB Host UVC demo YUYV Format       |\n");
#else
    printf("|        USB Host UVC Demo MJPEG Format      |\n");
#endif
    printf("|            Video Streaming                 |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");
#if (DISPALY == 1)
    //LCD init
    Display_Init();
    printf("LCD Display init done\n");
    Display_ClearLCD(C_WHITE);
    printf("ClearLCD done\n");
#endif
    //USB host init
    usbh_core_init();
    //init UVC Class Driver
    usbh_uvc_init();
    usbh_memory_used();
    //waiting 2 Sec for UVC Class Device Stable
    delay_us(2000000);

    printf("start.. \n");

    uint32_t t_last = get_ticks();
    uint32_t cnt_last = 0;

    while (1)
    {
        vdev = uvc_conn_check(vdev);

        if (vdev == NULL)
        {
            t_last = get_ticks();
            cnt_last = 0;
            _total_frame_count = 0;
            continue;
        }

        if (!kbhit())
        {
            printf("get - ");
            command = getchar();

            printf("comand : 0x%x\n", command);

            switch (command)
            {
                case '1':
                    printf("Stop vdev->is_streaming:%d\n", vdev->is_streaming);

                    if (!vdev->is_streaming)
                        break;

                    ret = usbh_uvc_stop_streaming(vdev);

                    if (ret != 0)
                        printf("\nusbh_uvc_stop_streaming failed! - %d\n", ret);

                    init_image_buffers();

                    //wait UVC Device command effective.

                    /* assign the first image buffer to receive the image from USB */
                    usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
                    _imgs[_idx_usb].state = IMAGE_BUFF_USB;

                    Display_ClearLCD(C_WHITE);
                    break;

                case '2':
                    if (vdev->is_streaming)
                    {
                        printf("uvc is streaming..\n");
                        break;
                    }

#ifdef DEMO_YUYV_FORMAT
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
                    ret = usbh_set_video_still_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
                    ret = usbh_set_video_still_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#endif
                    ret = usbh_uvc_start_streaming(vdev, uvc_rx_callbak);

                    if (ret != 0)
                        printf("\nusbh_uvc_start_streaming failed! - %d\n", ret);

                    break;

                case '3':
                    printf("Triggre Still Image Test...\n");

                    //Clear LCD to WHITE;
                    Display_ClearLCD(C_WHITE);

                    //stop video streaming if video is streaming.
                    if (vdev->is_streaming)
                    {
                        ret = usbh_uvc_stop_streaming(vdev);

                        if (ret != 0)
                            printf("\nusbh_uvc_stop_streaming failed! - %d\n", ret);
                    }

                    //Init Image buffer..
                    init_image_buffers();
                    delay_us(20 * 1000); //wait 20ms
                    /* assign the first image buffer to receive the image from USB */
                    usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
                    _imgs[_idx_usb].state = IMAGE_BUFF_USB;

#ifdef DEMO_YUYV_FORMAT
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
                    ret = usbh_set_video_still_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
                    ret = usbh_set_video_still_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#endif

                    if (ret != 0)
                    {
                        printf("usbh_set_video_format failed! - 0x%x\n", ret);
                    }

                    delay_us(20 * 1000); //wait 20ms

                    ret = usbh_uvc_still_image_trigger_control(vdev, START_CAPTURE);

                    if (ret < 0)
                    {
                        printf("still image trigger fail ..\n");
                    }

                    delay_us(20 * 1000); //wait 20ms

                    ret = usbh_uvc_start_streaming(vdev, uvc_rx_callbak);

                    if (ret < 0)
                    {
                        printf("Start still straming faill..\n");
                    }

                    while (_imgs[_idx_post].sti != 1)
                    {
                        usbh_uvc_still_image_trigger_control(vdev, START_CAPTURE);
                        delay_us(200 * 1000); //wait

                        if (_imgs[_idx_post].state == IMAGE_BUFF_READY)
                        {
                            if (_imgs[_idx_post].sti == 1)
                            {
#ifdef DEMO_YUYV_FORMAT
                                /* YUYV: true strip processing - no full-frame RGB buffer needed */
                                yuyv_strip_display(_imgs[_idx_post].buff, _imgs[_idx_post].len);
#else
                                mjpeg_decode_strip_display(_imgs[_idx_post].buff, _imgs[_idx_post].len);
#endif

                                _imgs[_idx_post].state = IMAGE_BUFF_FREE;
                                _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;

                                break;
                            }

                            _imgs[_idx_post].state = IMAGE_BUFF_FREE;
                            _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;
                        }

                    };

                    delay_us(20 * 1000); //wait 20ms

                    //Stop Video Streaming.
                    ret = usbh_uvc_stop_streaming(vdev);

                    if (ret != 0)
                    {
                        printf("\nusbh_uvc_stop_streaming failed! - %d\n", ret);
                    }

                    delay_us(20 * 1000); //wait 20ms

                    usbh_uvc_still_image_trigger_control(vdev, END_CAPTURE);
                    printf("Test Done\n");
                    printf("Please reopen Video Streaming command[2]..\n");
                    break;
            }
        }

        if (_imgs[_idx_post].state == IMAGE_BUFF_READY)
        {
#if (DISPALY == 1)
#ifdef DEMO_YUYV_FORMAT
            /* YUYV: true strip processing - no full-frame RGB buffer needed */
            yuyv_strip_display(_imgs[_idx_post].buff, _imgs[_idx_post].len);
#else
            /* MJPEG: strip decode + display */
            mjpeg_decode_strip_display(_imgs[_idx_post].buff, _imgs[_idx_post].len);
#endif
#endif
            _imgs[_idx_post].state = IMAGE_BUFF_POST;
            _imgs[_idx_post].state = IMAGE_BUFF_FREE;
            _imgs[_idx_post].sti = 0;
            _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;
        }

        if (get_ticks() - t_last > 100)
        {
            cnt_last = _total_frame_count - cnt_last;

            printf("Frame rate: %d, Total: %d \n", (cnt_last * 100) / (get_ticks() - t_last), _total_frame_count);

            t_last = get_ticks();
            cnt_last = _total_frame_count;
        }
    }
}
