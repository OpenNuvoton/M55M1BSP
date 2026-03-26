/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Decode and display Jpeg file on LCD panel.
 *
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>


#include "diskio.h"
#include "ff.h"

#include "board_config.h"

#include "imlib.h"          /* Image processing */
#include "framebuffer.h"
#include "Display.h"

#include "main.h"
#undef PI
#include "NuMicro.h"
/* Macros */

#define DEF_CARD_DETECT_SOURCE       CardDetect_From_GPIO
//#define DEF_CARD_DETECT_SOURCE       CardDetect_From_DAT3

#define BUFF_SIZE       (8*1024)/4


#define PATH_IMAGE_RESOLUTION_WIDTH    PANEL_DISPLAY_WIDTH_MAX
#define PATH_IMAGE_RESOLUTION_HEIGHT   PANEL_DISPLAY_HEIGHT_MAX


#define IMAGE_DISP_UPSCALE_FACTOR 1
#if defined(LT7381_LCD_PANEL)
    #define FONT_DISP_UPSCALE_FACTOR 2
#else
    #define FONT_DISP_UPSCALE_FACTOR 1
#endif




/* Variables*/
DWORD acc_size;                         /* Work register for fs command */
WORD acc_files, acc_dirs;
FILINFO Finfo;

uint8_t  *Buff;
uint64_t volatile gSec = 0;
uint32_t volatile gSdInit = 0;

#if _USE_LFN
    char Lfname[512];
#endif

#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t Buff_Pool[BUFF_SIZE] ;       /* Working buffer */
#else
    uint8_t Buff_Pool[BUFF_SIZE] __attribute__((aligned(4)));       /* Working buffer */
#endif

//Used by libjpeg, decode
__attribute__((section(".bss.sram.data"), aligned(32))) static uint8_t g_au8RGB888ImageBuffer [PATH_IMAGE_RESOLUTION_WIDTH * PATH_IMAGE_RESOLUTION_HEIGHT * 3 + 512];


// Used by omv library
char *_fb_base = NULL;
char *_fb_end = NULL;
char *_jpeg_buf = NULL;
char *_fballoc = NULL;
__attribute__((section(".bss.vram.data"), aligned(32))) static char fb_array[OMV_FB_SIZE + OMV_FB_ALLOC_SIZE];
__attribute__((section(".bss.vram.data"), aligned(32))) static char jpeg_array[OMV_JPEG_BUF_SIZE];





NVT_ITCM void TIMER0_IRQHandler(void)
{
    gSec++;//gSec is used for Display_Delay
    //printf("gSec=%d \r\n", gSec);
    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
}

void timer_init()
{
    // Set timer frequency to 1HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);


    // Start Timer 0
    TIMER_Start(TIMER0);
}

uint32_t get_timer_value()
{
    //printf("get_timer_value() To do...\n");
    return gSec;
}

/*----------------------------------------------*/
/* Scan for picfiles                            */
/*----------------------------------------------*/
FRESULT scan_picfiles(char *path)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;

    res = f_findfirst(&dir, &fno, path, "*.jpg");

    while (res == FR_OK && fno.fname[0])
    {
        printf("find: %s\n", fno.fname);
        res = f_findnext(&dir, &fno);
    }

    return res;
}
NVT_ITCM void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;
    volatile uint32_t u32CDState;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    ier = SDH0->INTEN;

    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
        //printf("SD block down\r\n");
    }

    if ((ier & SDH_INTEN_CDIEN_Msk) &&
            (isr & SDH_INTSTS_CDIF_Msk))    // card detect
    {
        //----- SD interrupt status
        // delay 50 us to sync the GPIO and SDH
        {
            (void)SDH0->INTSTS;

            CLK_SysTickDelay(50);

            isr = SDH0->INTSTS;
        }

        u32CDState = (((SDH0->INTEN & SDH_INTEN_CDSRC_Msk) >> SDH_INTEN_CDSRC_Pos) == 0) ?
                     (!(SDH0->INTSTS & SDH_INTSTS_CDSTS_Msk)) : (SDH0->INTSTS & SDH_INTSTS_CDSTS_Msk);

        if (u32CDState)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            //SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }

        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable SDH0 module clock source as HCLK and SDH0 module clock divider as 4 */
    CLK_EnableModuleClock(SDH0_MODULE);
    CLK_SetModuleClock(SDH0_MODULE, CLK_SDHSEL_SDH0SEL_HCLK0, CLK_SDHDIV_SDH0DIV(4));

    /* Enable Tiemr 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select Timer 0 module clock source as HXT */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HXT, 0);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable all GPIO clock */
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

    /* Enable FMC0 module clock to keep FMC clock when CPU idle but NPU running*/
    //CLK_EnableModuleClock(FMC0_MODULE);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable EBI0 module clock */
    //CLK_EnableModuleClock(EBI0_MODULE);

    /* Enable EPWM0 module clock */
    //CLK_EnableModuleClock(EPWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pin for SDH */
    /* CD: PB12(9), PD13(3) */
    //SET_SD0_nCD_PB12();
    SET_SD0_nCD_PD13();

    /* CLK: PB1(3), PE6(3) */
    //SET_SD0_CLK_PB1();
    SET_SD0_CLK_PE6();

    /* CMD: PB0(3), PE7(3) */
    //SET_SD0_CMD_PB0();
    SET_SD0_CMD_PE7();

    /* D0: PB2(3), PE2(3) */
    //SET_SD0_DAT0_PB2();
    SET_SD0_DAT0_PE2();

    /* D1: PB3(3), PE3(3) */
    //SET_SD0_DAT1_PB3();
    SET_SD0_DAT1_PE3();

    /* D2: PB4(3), PE4(3) */
    //SET_SD0_DAT2_PB4();
    SET_SD0_DAT2_PE4();

    /* D3: PB5(3)-, PE5(3) */
    //SET_SD0_DAT3_PB5();
    SET_SD0_DAT3_PE5();
}


/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

DWORD get_fattime(void)
{
    DWORD tmr;

    tmr = 0x00000;

    return tmr;
}

static void omv_init()
{
    image_t frameBuffer;

    frameBuffer.w = GLCD_WIDTH;
    frameBuffer.h = GLCD_HEIGHT;
    frameBuffer.size = GLCD_WIDTH * GLCD_HEIGHT * 3;
    frameBuffer.pixfmt = PIXFORMAT_RGB565;


    _fb_base = fb_array;
    _fb_end =  fb_array + OMV_FB_SIZE - 1;
    _fballoc = _fb_base + OMV_FB_SIZE + OMV_FB_ALLOC_SIZE;
    _jpeg_buf = jpeg_array;

    fb_alloc_init0();

    framebuffer_init0();
    framebuffer_init_from_image(&frameBuffer);
}


int32_t main(void)
{

    TCHAR       sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    //Display image on LCD
    S_DISP_RECT sDispRect;

    //For Jpeg image check
    uint16_t w, h;

    SYS_UnlockReg();

    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    timer_init();

    /*
        SD initial state needs 300KHz clock output, driver will use HIRC for SD initial clock source.
        And then switch back to the user's setting.
    */
    gSdInit = (SDH_Open_Disk(SDH0, DEF_CARD_DETECT_SOURCE) == 0) ? 1 : 0;

    SYS_LockReg();


    //display framebuffer
    image_t frameBuffer;

    //omv library init
    omv_init();
    framebuffer_init_image(&frameBuffer);

    Display_Init();
    Display_ClearLCD(C_BLACK);


    printf("\n");
    printf("====================================\n");
    printf("          JPEG DISPLAY DEMO         \n");
    printf("====================================\n");

#if (DEF_CARD_DETECT_SOURCE == CardDetect_From_DAT3)
    printf("You enabled card detection source from DAT3 mode.\n");
    printf("Please remove pull-up resistor of DAT3 pin and add a pull-down 100Kohm resistor on DAT3 pin.\n");
    printf("Please also check your SD card is with an internal pull-up circuit on DAT3 pin.\n");
#endif

    f_chdrive(sd_path);          /* set default path */

    for (;;)
    {
        if (!(SDH_CardDetection(SDH0)))
        {
            gSdInit = 0;
            printf("No card!!\n");
            continue;
        }

        if (!gSdInit)
        {
            gSdInit = (SDH_Open_Disk(SDH0, DEF_CARD_DETECT_SOURCE) == 0) ? 1 : 0;
        }


        /*Test list all files*/
        collect_photos("0:/");

        while (1)
        {
            char *target_file = Playlist_GetCurrent();

            if (target_file != NULL)
            {
                DBG_msg("Find: %s\n", target_file);

                JpegDecode((unsigned char *)target_file, (uint8_t *)(g_au8RGB888ImageBuffer));

                //Get the output width and size for centering the pic
                jpeg_get_output_size(&w, &h);
                DBG_msg("decode output size() : w(%d), h(%d) \r\n", w, h);

                //Color format conversion to fit panel requirement.
                jpeg_convert_RGB888toRGB565_SW((uint8_t *)(g_au8RGB888ImageBuffer), w * h);

                //Find coordinate that centering the image
                jpeg_compute_roi_centering(&sDispRect, w, h);

                //Flush black and update iamge
                Display_ClearLCD(C_BLACK);
                Display_FillRect((uint16_t *)g_au8RGB888ImageBuffer, &sDispRect, IMAGE_DISP_UPSCALE_FACTOR);

                //Update file name on omv framebuffer
                char szDisplayText[64];
                sprintf(szDisplayText, "%s", &target_file[4]);
                Display_PutText(szDisplayText, strlen(szDisplayText),
                                TEXT_OMV_OFFSET_W, TEXT_OMV_OFFSET_H, C_WHITE, C_BLACK,
                                false, FONT_DISP_UPSCALE_FACTOR);
                //Delay some time if requied
                Display_Delay(1000);
            }

            if (Playlist_Next() < 0)
            {
                //Delay for 1000ms if all images are visited.
                Display_Delay(1000);

                //break;
            };

            //break;
        }



        //while(1);

    }

}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
