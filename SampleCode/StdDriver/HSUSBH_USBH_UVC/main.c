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

#if 0
    #define DEMO_YUYV_FORMAT
#else
    #define DEMO_MJPEG_FORMAT
#endif
/*----------------------------------------------------------------------
 * USB UVC
 */
#define SELECT_RES_WIDTH     (320)
#define SELECT_RES_HEIGHT    (240)

#define IMAGE_MAX_SIZE       (SELECT_RES_WIDTH * SELECT_RES_HEIGHT * 2)
#define IMAGE_BUFF_CNT       4

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
};


//YUV or MJEPG Format
struct img_buff_t  _imgs[IMAGE_BUFF_CNT];
uint8_t  image_buff_pool[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE] __attribute__((aligned(32)));
volatile int   _idx_usb = 0, _idx_post = 0;
int   _total_frame_count = 0;
uint8_t rgb888_image_buff[SELECT_RES_WIDTH * SELECT_RES_HEIGHT * 3]  __attribute__((aligned(32)));
//RGB565
uint16_t  rgb_image_buff_pool[RGB_IMAGE_MAX_SIZE] __attribute__((aligned(32)));

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t s_u32TickCnt;

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

//YUV422 (YUYV) to RGB565
/**
* @brief Converts a YUV422 (specifically YUYV format) image to RGB565 format.
*
* @param yuv_buffer Pointer to the input YUV422 image data.
*                   The YUYV format stores data as Y0, U, Y1, V for every two pixels.
* @param rgb_buffer Pointer to the output RGB565 image data.
* @param width      Width of the image in pixels.
* @param height     Height of the image in pixels.
*
* @details
* This function iterates through the YUV422 data, taking 4 bytes at a time (Y0, U, Y1, V),
* which represent two pixels. It then converts these two YUV pixels to two RGB565 pixels.
* The conversion uses an integer-approximated version of the BT.601 standard formula.
*
* YUV to RGB conversion formula (approximated for integer arithmetic):
*   R = Y + 1.402 * (V - 128)  => R = Y + (179 * (V-128)) >> 7
*   G = Y - 0.344 * (U - 128) - 0.714 * (V - 128) => G = Y - (44 * (U-128) + 91 * (V-128)) >> 7
*   B = Y + 1.772 * (U - 128)  => B = Y + (227 * (U-128)) >> 7
*
* RGB565 format packs R, G, B components into a 16-bit value:
*   Bits 15-11: Red (5 bits)
*   Bits 10-5:  Green (6 bits)
*   Bits 4-0:   Blue (5 bits)
*/
void yuv422_to_rgb565(uint8_t *yuv_buffer, uint16_t *rgb_buffer, int width, int height)
{
    int frame_size = width * height * 2; // YUV422
    int rgb_index = 0;

    for (int i = 0; i < frame_size; i += 4)
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
 * @brief Converts an RGB888 image to RGB565 format.
 *
 * @param src    Pointer to the input RGB888 image data.
 *               Each pixel is represented by 3 bytes (R, G, B).
 * @param dst    Pointer to the output RGB565 image data.
 *               Each pixel is represented by 2 bytes (16 bits).
 * @param pixels The total number of pixels in the image.
 *
 * @details
 * This function iterates through each pixel of the input RGB888 image,
 * extracts the 8-bit Red, Green, and Blue components, and then
 * converts them to their 5-bit (Red, Blue) and 6-bit (Green)
 * representations for the RGB565 format.
 *
 * RGB888 format:
 *   Red:   8 bits
 *   Green: 8 bits
 *   Blue:  8 bits
 *
 * RGB565 format (16 bits per pixel):
 *   Bits 15-11: Red   (5 bits) - Takes the most significant 5 bits of the 8-bit Red.
 *   Bits 10-5:  Green (6 bits) - Takes the most significant 6 bits of the 8-bit Green.
 *   Bits 4-0:   Blue  (5 bits) - Takes the most significant 5 bits of the 8-bit Blue.
 */
void rgb888_to_rgb565(const uint8_t *src, uint16_t *dst, int pixels)
{
    for (int i = 0; i < pixels; i++)
    {
        uint8_t r = src[3 * i + 0];
        uint8_t g = src[3 * i + 1];
        uint8_t b = src[3 * i + 2];
        /*
         * RGB565
         *   R[4:0] (High 5 bit) <- R[7:3]
         *   G[5:0] (High 6 bit) <- G[7:2]
         *   B[4:0] (High 5 bit) <- B[7:3]
         */
        uint16_t r5 = (r >> 3) & 0x1F;
        uint16_t g6 = (g >> 2) & 0x3F;
        uint16_t b5 = (b >> 3) & 0x1F;
        dst[i] = (r5 << 11) | (g6 << 5) | b5;
    }
}

extern int decode_jpeg_to_rgb888(unsigned char *image, unsigned long jpegSize, unsigned char *rgb888Buf);


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
#if 1
void  init_image_buffers(void)
{
    int   i;

    for (i = 0; i < IMAGE_BUFF_CNT; i++)
    {
        //nc_ptr
        //_imgs[i].buff = nc_ptr(&image_buff_pool[i]);
        _imgs[i].buff = image_buff_pool[i];
        _imgs[i].len = 0;
        _imgs[i].state = IMAGE_BUFF_FREE;
    }

    _idx_usb = 0;
    _idx_post = 0;
}

int  uvc_rx_callbak(UVC_DEV_T *vdev, uint8_t *data, int len)
{
    int  next_idx;

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

        /* proceed to the next image buffer */
        _idx_usb = next_idx;
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;     /* mark the next image as used by USB    */

        /* assign the next image buffer to receive next image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
    }

    return 0;
}
#endif
void show_menu()
{
    printf("\n\n+---------------------------------------------+\n");
    printf("|  Operation menu                             |\n");
    printf("+---------------------------------------------+\n");
    printf("|  [1] Stop video streaming                   |\n");
    printf("|  [2] Start video streaming                  |\n");
    printf("+---------------------------------------------+\n\n");
    usbh_memory_used();
    //printf("UVC is_streaming = %d,\n",cur_vdev->is_streaming);
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
            ret = usbh_get_video_format(cur_vdev, i, &format, &width, &height);

            if (ret != 0)
                break;

            printf("[%d] %s, %d x %d\n", i, (format == UVC_FORMAT_MJPEG ? "MJPEG" : "YUYV"), width, height);
        }

        printf("\n\n");
#ifdef DEMO_YUYV_FORMAT
        ret = usbh_set_video_format(cur_vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else
        ret = usbh_set_video_format(cur_vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#endif

        if (ret != 0)
            printf("usbh_set_video_format failed! - 0x%x\n", ret);

        init_image_buffers();

        delay_us(200);//wait UVC Device command effective.

        /* assign the first image buffer to receive the image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;

        ret = usbh_uvc_start_streaming(cur_vdev, uvc_rx_callbak);

        if (ret != 0)
        {
            printf("usbh_uvc_start_streaming failed! - %d\n", ret);
            printf("Please re-connect UVC device...\n");
        }
        else
            show_menu();

    }

    return cur_vdev;
}

int32_t main(void)
{
    UVC_DEV_T       *vdev = NULL;
    S_DISP_RECT     sDispRect;
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

                    break;

                case '2':
#ifdef DEMO_YUYV_FORMAT
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_YUY2, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else
                    ret = usbh_set_video_format(vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#endif

                    if (ret != 0)
                        printf("usbh_set_video_format failed! - 0x%x\n", ret);

                    printf("Start vdev->is_streaming:%d\n", vdev->is_streaming);

                    if (vdev->is_streaming)
                        break;

                    ret = usbh_uvc_start_streaming(vdev, uvc_rx_callbak);

                    if (ret != 0)
                        printf("\nusbh_uvc_start_streaming failed! - %d\n", ret);

                    break;
            }
        }

        if (_imgs[_idx_post].state == IMAGE_BUFF_READY)
        {
#if (DISPALY == 1)

#ifdef  DEMO_YUYV_FORMAT
            //YUV422 to rgb565
            yuv422_to_rgb565((uint8_t *)_imgs[_idx_post].buff, (uint16_t *)rgb_image_buff_pool, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);
#else

            //Check it is JPEG File and MJPEG Decode -> rgb888
            if (decode_jpeg_to_rgb888((uint8_t *)_imgs[_idx_post].buff, IMAGE_MAX_SIZE, rgb888_image_buff) == TRUE)
            {
                //rgb888 -> rgb565
                rgb888_to_rgb565(rgb888_image_buff, rgb_image_buff_pool, RGB_IMAGE_MAX_SIZE);
            }
            else
            {

                goto Next_Image;
            }

#endif
            //Display image on LCD
            sDispRect.u32TopLeftX = 0;
            sDispRect.u32TopLeftY = 0;
            sDispRect.u32BottonRightX = ((SELECT_RES_WIDTH * IMAGE_DISP_UPSCALE_FACTOR) - 1);
            sDispRect.u32BottonRightY = ((SELECT_RES_HEIGHT * IMAGE_DISP_UPSCALE_FACTOR) - 1);

            Display_FillRect((uint16_t *)rgb_image_buff_pool, &sDispRect, IMAGE_DISP_UPSCALE_FACTOR);
#endif
#ifndef  DEMO_YUYV_FORMAT
Next_Image:
#endif
            _imgs[_idx_post].state = IMAGE_BUFF_POST;
            _imgs[_idx_post].state = IMAGE_BUFF_FREE;
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
