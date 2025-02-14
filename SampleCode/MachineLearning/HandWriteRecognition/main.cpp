/******************************************************************************
 * @file     main.cpp
 * @version  V1.00
 * @brief    Handwrite recognition with MNIST datasets and fdMobileNet
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "ethosu_npu_init.h"
#include "BufAttributes.h"
#include "NNClassify/MainClassify.h"
#include "imlib.h"
#include "ImgConvert.h"
#include "board_m55m1.h"
#include "numbering_10.c"
#include "mpu_config_M55M1.h"
using namespace std;

/****************************************************************************
 * User Define Macro
 ****************************************************************************/
#define BUFF_LEN        8000
#define INTERVAL_UPDATEPANEL        (5)
#define INTERVAL_DOINFERENCE        (50)

/****************************************************************************
 * Global Variables
 ****************************************************************************/
//Arena tensor
extern uint8_t  _tensor_arena[] ;

//Systick Related
extern uint32_t SystemCoreClock;        /* Expected to come from the cmsis-device lib */

volatile int32_t g_u32Ticks = 0;
uint64_t sPMU_TickCounter;
static uint64_t cpu_cycle_count = 0;    /* 64-bit cpu cycle counter */

//For whole system timing
volatile uint32_t g_u32Ticks_start = 0;
volatile uint32_t g_u32Ticks_end = 0;

//Inference event related counter and flag
volatile uint8_t  g_u8InfCounts = 0;
volatile uint8_t u8DoInfFlag = 0;

//Clear drawing/update inference result related flag
volatile uint8_t u8ClearDrawFlag = 0;
volatile uint8_t u8NotDrawYetFlag = 1;

//For EBI Panel
rect_info  rect_preview;

//For TP Info
S_LVGL_TPINFO  sctp_info;

//For  drawing  line with openmv
image_t srcImg;
image_t displayImg;


uint8_t u8SpeedString[32];

extern const char *g_NumberCategory[];

//Data Buffer for ML inference
static __attribute__((aligned))int8_t gNNBuffer[MNIST_HANDWRITE_WIDTH * MNIST_HANDWRITE_HEIGHT] = {0};

//Frame Buffer for OMV
static __attribute__((aligned))uint8_t u8DrawBuffer[MNIST_HANDWRITE_HEIGHT * MNIST_HANDWRITE_WIDTH] = {0};
static __attribute__((aligned))uint8_t u8FrameBuffer[TP_POS_X_MAX * TP_POS_Y_MAX * 2] = {0};


/****************************************************************************
 * User Code
 ****************************************************************************/
#ifdef  __cplusplus
extern  "C" {
#endif
void SysTick_Handler(void);
void TIMER0_IRQHandler(void);
uint8_t dbuffer_update(S_LVGL_TPINFO tp_info);
void sensor_timer_run(void);
static eUserIntent draw_check_intent(S_LVGL_TPINFO tp_info);
#ifdef  __cplusplus
}
#endif


void sensor_timer_run(void)
{
    // Set timer frequency to 1HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);//Sets TP sample rate to 100Hz

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start Timer 0
    TIMER_Start(TIMER0);

}

/**
 * Update the touch positions to update the buffer image
 */
uint8_t dbuffer_update(S_LVGL_TPINFO tp_info)
{
    static uint16_t scale_posx, scale_posy, scale_posx_old, scale_posy_old, posx, posy, posx_old, posy_old;
    static uint8_t cur_state = 0;//Undefine_State;
    static uint8_t next_state = 0;//Undefine_State;


    if (tp_info.event_touched == 1) //touched
    {

        //Scaling the coordinate points to fit input model dimension
        posx = tp_info.pos_x;
        posy = tp_info.pos_y;

        //Check is pos is in desired range
        if (posx > TP_POS_X_MAX)   posx = TP_POS_X_MAX;

        if (posy > TP_POS_Y_MAX)   posy = TP_POS_Y_MAX;

        scale_posx = (uint16_t)((float)(posx) * MNIST_HANDWRITE_WIDTH / TP_POS_X_MAX);
        scale_posy = (uint16_t)((float)(posy) * MNIST_HANDWRITE_HEIGHT / TP_POS_Y_MAX);

    }
    else//up
    {
        //printf("release\r\n");
    }

    //FSM

    switch (cur_state)
    {
        case Undefine_State:
        {

            if (tp_info.event_touched == 1)
            {
                scale_posx_old = scale_posx;
                scale_posy_old = scale_posy;
                imlib_draw_line(&srcImg, scale_posx_old,  scale_posy_old, scale_posx,  scale_posy, 0xFF, 3);

                posx_old = posx;
                posy_old = posy;
                imlib_draw_line(&displayImg, posx_old,  posy_old, posx, posy, 0xFF, 22);


                next_state = Draw_State;
            }
            else//released, add for completeness[2023-08-18]
            {
                next_state = Undefine_State;
            }
        }
        break;

        case Draw_State:
        {

            if (tp_info.event_touched == 1)
            {
                imlib_draw_line(&srcImg, scale_posx_old,  scale_posy_old, scale_posx,  scale_posy, 0xFF, 3);

                scale_posx_old = scale_posx;
                scale_posy_old = scale_posy;

                imlib_draw_line(&displayImg, posx_old,  posy_old, posx, posy, 0xFF, 22);

                posx_old = posx;
                posy_old = posy;
                next_state = Draw_State;
            }
            else//released
            {
                next_state = Undefine_State;
            }
        }

        break;

        default:

            break;
    }

    cur_state = next_state;
    //printf("cur_state:%d \r\n", cur_state);
    return cur_state;
}



static eUserIntent draw_check_intent(S_LVGL_TPINFO tp_info)
{
    if ((sctp_info.pos_x > CLEARDRAW_BUTTON_X_THRESH) && (sctp_info.pos_y > CLEARDRAW_BUTTON_Y_THRESH))
    {
        u8ClearDrawFlag = 1;
        return ButtonPress_Intent;
    }
    else if ((sctp_info.pos_x > TP_POS_X_MAX) || (sctp_info.pos_y > TP_POS_Y_MAX))
    {
        return NoIntent_Intent;
    }
    else
    {
        return DrawSomething_Intent;
    }

}


void TIMER0_IRQHandler(void)
{
    uint8_t ret_estate;
    eUserIntent ret_eIntent;

    st1663i_read_point(&sctp_info, 1);

    ret_eIntent = draw_check_intent(sctp_info);

    ret_estate = dbuffer_update(sctp_info);

    if ((u8NotDrawYetFlag == 1) && (ret_estate == Draw_State) && (ret_eIntent == DrawSomething_Intent))
        u8NotDrawYetFlag = 0;

    if (g_u32Ticks < INTERVAL_UPDATEPANEL) //50ms
    {
        g_u32Ticks++;
    }
    else
    {
        g_u32Ticks = 0;
        fsa506_fillrect((uint16_t *)(&u8FrameBuffer[0]), &rect_preview);

    }

    if (g_u8InfCounts < INTERVAL_DOINFERENCE) //500ms
    {
        g_u8InfCounts++;
    }
    else
    {
        g_u8InfCounts = 0;
        u8DoInfFlag = 1;

    }

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);
    //LED_TIMER = 0;
}

void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    cpu_cycle_count += SysTick->LOAD + 1;
}

uint64_t Get_SysTick_Cycle_Count_Export(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return cpu_cycle_count + (SysTick->LOAD - systick_val);
}

int Init_SysTick_Export(void)
{
    const uint32_t ticks_10ms = SystemCoreClock / 100 + 1;
    int err = 0;

    /* Reset CPU cycle count value. */
    cpu_cycle_count = 0;

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

int32_t nvtGetCurrentTimeTicks()
{
    return (int32_t)(Get_SysTick_Cycle_Count_Export());
}


void SYS_Init(void)
{
    // Init System Clock                                                                                       */
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable NPU module clock */
    CLK_EnableModuleClock(NPU0_MODULE);

    /* Enable GPIO module clock */
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
    CLK_EnableModuleClock(FMC0_MODULE);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Enable EBI0 module clock */
    CLK_EnableModuleClock(EBI0_MODULE);

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable NPU module clock */
    CLK_EnableModuleClock(NPU0_MODULE);

    /* Enable TMR module clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_TMRSEL_TMR1SEL_PCLK1, 0);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);


    /* Select UART6 module clock source as HIRC and UART6 module clock divider as 1 */
    SetDebugUartCLK();

    // Init I/O Multi-function
    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
}

/* Cache policy function */
enum { NonCache_index, WTRA_index, WBWARA_index };

static void InitLCD(void)
{
    fsa506_low_level_init();
    fsa506_startup_sequence();
}

static void LCD_DisplayUIDescription(void)
{
    fsa506_WriteString(300, 0,   "MNIST Demo(U55)", Font_11x18, FSA506_WHITE,  FSA506_BLACK);
    fsa506_WriteString(300, 200,   "             ", Font_11x18, FSA506_WHITE,  FSA506_BLUE);
    fsa506_WriteString(300, 218,   "  ClearDraw  ", Font_11x18, FSA506_WHITE,  FSA506_BLUE);
    fsa506_WriteString(300, 236,   "             ", Font_11x18, FSA506_WHITE,  FSA506_BLUE);
}


static void LCD_DisplayResult(char *pch, uint16_t index)
{
    fsa506_WriteString(300, 64,  "                 ",                         Font_11x18, FSA506_WHITE,  FSA506_BLACK);//add extra 3 word space
    fsa506_WriteString(300, 64,  g_NumberCategory[index], Font_11x18, FSA506_WHITE,  FSA506_BLACK);

    fsa506_WriteString(300, 164,  "                 ",                         Font_11x18, FSA506_WHITE,  FSA506_BLACK);//add extra 3 word space
    fsa506_WriteString(300, 164, (char *)(pch), Font_11x18, FSA506_WHITE,  FSA506_BLACK);
}


static void InitTouchDriver(void)
{
    st1633i_low_level_init();
    st1633i_startup_sequence();
}

static void ImgConvert_BufConfiguration(void)
{
    srcImg.w = MNIST_HANDWRITE_WIDTH;
    srcImg.h =  MNIST_HANDWRITE_HEIGHT;
    srcImg.size = MNIST_HANDWRITE_WIDTH * MNIST_HANDWRITE_HEIGHT;
    srcImg.pixfmt = PIXFORMAT_GRAYSCALE;
    srcImg.data = u8DrawBuffer;

    displayImg.w = TP_POS_Y_MAX;
    displayImg.h =  TP_POS_X_MAX;
    displayImg.size = TP_POS_Y_MAX * TP_POS_X_MAX ;
    displayImg.pixfmt = PIXFORMAT_RGB565;
    displayImg.data = u8FrameBuffer;
}

static void config_showresultwindow(rect_info_t *pRect, uint16_t h, uint16_t w)
{
    // Set panel fill rect-window parameter
    pRect->x = 0;
    pRect->y = 0;
    pRect->width = w;
    pRect->height = h;

}

static void ResetTouchPositions(void)
{
    sctp_info.pos_x = 0;
    sctp_info.pos_y = 0;
    sctp_info.event_touched = 0;
}

int main(void)
{

    uint16_t cc_index;
    float cc[340];
    float cc_max = -1;
    g_u32Ticks = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* UART init - will enable valid use of printf (stdout
    * re-directed at this UART (UART6) */
    InitDebugUart();

    SYS_LockReg();                   /* Unlock register lock protect */

    printf("SystemCoreClock=%u\n", SystemCoreClock);

#if defined(ARM_NPU)

    int state;

    /* If Arm Ethos-U NPU is to be used, we initialise it here */
    if (0 != (state = arm_ethosu_npu_init()))
    {
        return state;
    }

    /* Setup cache poicy of tensor arean buffer */
    printf("Set tesnor arena cache policy to WTRA \n");
    const std::vector<ARM_MPU_Region_t> mpuConfig =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)_tensor_arena),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)_tensor_arena) + 0x50000 - 1),        // Limit
                         WTRA_index) // Attribute index - Write-Through, Read-allocate
        }
    };

    // Setup MPU configuration
    InitPreDefMPURegion(&mpuConfig[0], mpuConfig.size());
#endif /* ARM_NPU */

    /* Initial and startup flow for LCD */
    InitLCD();

    /* Configure the size anf format of tflite image buffer and display image buffer */
    ImgConvert_BufConfiguration();

    /* Init omv lib for processing(draw line) image buffer */
    ImgConvert_Init();

    MainClassify mainclassify(gNNBuffer);

    /* Origins from (0,0) */
    config_showresultwindow(&rect_preview, TP_POS_Y_MAX, TP_POS_X_MAX);

    /* Initial and startup flow for touch driver */
    InitTouchDriver();

    /* Clear touch info. structure content */
    ResetTouchPositions();

    /* Start timer to drive*/
    sensor_timer_run();

    fsa506_fillrect_color(FSA506_WHITE, &rect_preview);
    memset(u8FrameBuffer, 0xFF, sizeof(u8FrameBuffer));

    /* Display UI description text, button */
    LCD_DisplayUIDescription();

    while (1)
    {
        if (u8DoInfFlag == 1)
        {

            mainclassify.QuantizeInputData(u8DrawBuffer);
            mainclassify.FillInTensorData();

            Init_SysTick_Export();
            mainclassify.Classify();  // Classify the extracted features
            g_u32Ticks_end = Get_SysTick_Cycle_Count_Export();
            g_u32Ticks_end = (uint32_t)(180000000 / (float)(g_u32Ticks_end));

            std::sprintf((char *)(u8SpeedString), "fps=%d", g_u32Ticks_end);

            cc_max = -1;

            for (uint16_t i = 0; i < MNIST_HANDWRITE_TYPE; i++)
            {
                cc[i]  = mainclassify.outQuantParams.scale *
                         (float)(((int8_t)(mainclassify.output[i])) - mainclassify.outQuantParams.offset);

                //printf("cc[%d] =%f, cc_max=%f\n", i, cc[i], cc_max);
                if (cc[i] > cc_max)
                {
                    cc_max = cc[i];
                    cc_index = i;
                    printf(" current cc_max=%f, index=%d\n", cc_max, i);
                }
            }

            printf(" Most possible is %dth class %s\n", (cc_index + 1), g_NumberCategory[cc_index]);

            if (!u8NotDrawYetFlag)
            {
                LCD_DisplayResult((char *)(u8SpeedString), cc_index);
            }

            u8DoInfFlag = 0;
        }


        if (u8ClearDrawFlag)
        {
            /* Stop Timer 0 */
            TIMER_Stop(TIMER0);

            memset(u8FrameBuffer, 0xFF, sizeof(u8FrameBuffer));
            memset(u8DrawBuffer,   0x00, sizeof(u8DrawBuffer));
            fsa506_WriteString(300, 64,  "                 ",                         Font_11x18, FSA506_WHITE,  FSA506_BLACK);
            fsa506_fillrect_color(FSA506_WHITE, &rect_preview);

            /* Start Timer */
            sctp_info.pos_x = 0;
            sctp_info.pos_y = 0;
            u8ClearDrawFlag = 0;
            u8NotDrawYetFlag = 1;
            TIMER_Start(TIMER0);
        }

    }

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
