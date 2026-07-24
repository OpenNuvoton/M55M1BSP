/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    The sample receives audio data from UAC device, and immediately send
 *           back to that UAC device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_uac.h"

#ifdef DEBUG_ENABLE_SEMIHOST
    #error This sample cannot execute with semihost enabled
#endif

#define USE_USB_APLL1_CLOCK         1

//Please, check your UAC device's supported sample rates and adjust this value if necessary to avoid streaming issue due to unsupported sample rate.
//Ensure the microphone supports the required sample rates, as the speaker (host) sample rate must match the microphone.
#define UAC_SAMPLE_FREQUENCY  48000
#define UAC2_SAMPLE_PAKET(i)   ((i)/8000)      // Sample packet count per second, e.g. 48KHz/8000 = 6 packets per second for 48KHz sample rate with 1ms USB frame.

#if (NVT_DCACHE_ON == 1)
    //for usbh_ctrl_xfer data block using, but data size (uint16_t) < one cache line size (32B) => Non-cacheable should be ok.
    NVT_NONCACHEABLE static uint16_t s_u16VolMax, s_u16VolMin, s_u16VolRes, s_u16VolCur;
#else
    static uint16_t s_u16VolMax, s_u16VolMin, s_u16VolRes, s_u16VolCur;
#endif

extern int kbhit(void);                        /* function in retarget.c                 */

extern volatile int8_t g_i8MicIsMono;
extern volatile uint32_t g_u32UacRecCnt;       /* Counter of UAC record data             */
extern volatile uint32_t g_u32UacPlayCnt;      /* Counter of UAC playback data           */

extern volatile uint8_t g_u8MicSubslotSize;
extern volatile uint8_t g_u8SpkSubslotSize;
extern volatile uint8_t g_u8MicChannels;
extern volatile uint8_t g_u8SpkChannels;
extern volatile uint16_t g_u16SampleClockFreq;

extern void ResetAudioLoopBack(void);
extern int audio_1_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
extern int audio_1_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
extern int audio_2_0_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
extern int audio_2_0_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);

static volatile uint32_t s_u32TickCnt;
static volatile uint32_t s_u32StreamStartTick;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
void uac_control_example(UAC_DEV_T *uac_dev);
void SYS_Init(void);
static int start_streaming(UAC_DEV_T *uac_dev);
static void stop_streaming(UAC_DEV_T *uac_dev);

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

void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes)
{
    int i8Idx, i8Cnt;

    i8Idx = 0;

    while (i8Bytes > 0)
    {
        printf("0x%04X  ", i8Idx);

        for (i8Cnt = 0; (i8Cnt < 16) && (i8Bytes > 0); i8Cnt++)
        {
            printf("%02x ", pu8Buff[i8Idx + i8Cnt]);
            i8Bytes--;
        }

        i8Idx += 16;
        printf("\n");
    }

    printf("\n");
}

void uac_control_example(UAC_DEV_T *uac_dev)
{
    uint16_t u16Val;
    uint32_t au32SRate[16];
    uint8_t u8Val;
    uint8_t au8Data[8];
    int i8Cnt, i8Ret;
    uint32_t u32Val;

    s_u16VolMax = s_u16VolMin = s_u16VolRes = 0;

    printf("UAC version: 0x%04x (%s)\n", uac_dev->acif.bcdADC,
           (uac_dev->acif.bcdADC >= 0x0200) ? "UAC 2.0" : "UAC 1.0");

    if (uac_dev->acif.bcdADC >= 0x0200)
    {
        printf("Clock Source ID: 0x%02x\n", uac_dev->acif.clk_src_id);
        printf("Microphone Clock Source ID: 0x%02x\n", uac_dev->acif.mic_clk_src_id);
    }

    printf("\nGet channel information ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get channel number information                             */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_SPEAKER);

    if (i8Ret < 0)
        printf("    Failed to get speaker's channel number.\n");
    else
    {
        printf("    Speaker: %d\n", i8Ret);
        g_u8SpkChannels = (uint8_t)i8Ret;
    }

    i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_MICROPHONE);

    if (i8Ret < 0)
        printf("    Failed to get microphone's channel number.\n");
    else
    {
        printf("    Microphone: %d\n", i8Ret);
        g_u8MicChannels = (uint8_t)i8Ret;

        if (i8Ret == 1)
            g_i8MicIsMono = 1;
        else
            g_i8MicIsMono = 0;
    }

    printf("\nGet subframe bit resolution ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_SPEAKER, &u8Val);

    if (i8Ret < 0)
        printf("    Failed to get speaker's bit resoltion.\n");
    else
    {
        printf("    Speaker audio subframe size: %d bytes\n", u8Val);
        printf("    Speaker subframe bit resolution: %d\n", i8Ret);
        g_u8SpkSubslotSize = u8Val;
    }

    i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_MICROPHONE, &u8Val);

    if (i8Ret < 0)
        printf("    Failed to get microphone's bit resoltion.\n");
    else
    {
        printf("    Microphone audio subframe size: %d bytes\n", u8Val);
        printf("    Microphone subframe bit resolution: %d\n", i8Ret);
        g_u8MicSubslotSize = u8Val;
    }

    printf("\nGet sampling rate list ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_SPEAKER, (uint32_t *)&au32SRate[0], 16, &u8Val);

    if (i8Ret < 0)
        printf("    Failed to get speaker's sampling rate.\n");
    else
    {
        if (u8Val == 0)
            printf("    Speaker sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
        else
        {
            for (i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                printf("    Speaker sampling rate: %d\n", au32SRate[i8Cnt]);
        }
    }

    i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_MICROPHONE, (uint32_t *)&au32SRate[0], 16, &u8Val);

    if (i8Ret < 0)
        printf("    Failed to get microphone's sampling rate.\n");
    else
    {
        if (u8Val == 0)
            printf("    Microphone sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
        else
        {
            for (i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                printf("    Microphone sampling rate: %d\n", au32SRate[i8Cnt]);
        }
    }

    /*-------------------------------------------------------------*/
    /*  UAC 2.0: Clock Source direct API                           */
    /*-------------------------------------------------------------*/
    if (uac_dev->acif.bcdADC >= 0x0200)
    {
        UAC2_FREQ_SUBRANGE_T ranges[16];
        uint16_t num_ranges;
        uint32_t freq;

        printf("\n[UAC 2.0] Clock Source direct API tests ===>\n");

        /* Get current clock frequency */
        if (usbh_uac2_clock_get_freq(uac_dev, UAC_SPEAKER, &freq) == UAC_RET_OK)
            printf("    Current clock frequency: %d Hz\n", freq);
        else
            printf("    Failed to get current clock frequency!\n");

        /* Get supported frequency range */
        i8Ret = usbh_uac2_clock_get_freq_range(uac_dev, ranges, 16, &num_ranges, UAC_SPEAKER);

        if (i8Ret == UAC_RET_OK)
        {
            printf("    Supported frequency ranges (%d):\n", num_ranges);

            for (i8Cnt = 0; i8Cnt < (int)num_ranges && i8Cnt < 16; i8Cnt++)
                printf("      [%d] %d - %d Hz (res=%d)\n", i8Cnt,
                       ranges[i8Cnt].dMIN, ranges[i8Cnt].dMAX, ranges[i8Cnt].dRES);
        }
        else
            printf("    Failed to get frequency range! (ret=%d)\n", i8Ret);

        /* Set frequency to 48000 Hz */
        if (usbh_uac2_clock_set_freq(uac_dev, UAC_SPEAKER, UAC_SAMPLE_FREQUENCY) == UAC_RET_OK)
            printf("    Set clock frequency: OK\n");
        else
            printf("    Set clock frequency: FAIL\n");

        /* Verify frequency after set */
        if (usbh_uac2_clock_get_freq(uac_dev, UAC_SPEAKER, &freq) == UAC_RET_OK)
            printf("    Verify clock frequency: %d Hz\n", freq);
        else
            printf("    Failed to verify clock frequency!\n");

        //Test Mircophone clock source if supported
        /* Get current clock frequency */
        if (usbh_uac2_clock_get_freq(uac_dev, UAC_MICROPHONE, &freq) == UAC_RET_OK)
            printf("    Current clock frequency: %d Hz\n", freq);
        else
            printf("    Failed to get current clock frequency!\n");

        /* Get supported frequency range */
        i8Ret = usbh_uac2_clock_get_freq_range(uac_dev, ranges, 16, &num_ranges, UAC_MICROPHONE);

        if (i8Ret == UAC_RET_OK)
        {
            printf("    Supported frequency ranges (%d):\n", num_ranges);

            for (i8Cnt = 0; i8Cnt < (int)num_ranges && i8Cnt < 16; i8Cnt++)
                printf("      [%d] %d - %d Hz (res=%d)\n", i8Cnt,
                       ranges[i8Cnt].dMIN, ranges[i8Cnt].dMAX, ranges[i8Cnt].dRES);
        }
        else
            printf("    Failed to get frequency range! (ret=%d)\n", i8Ret);

        /* Set frequency to 48000 Hz */
        if (usbh_uac2_clock_set_freq(uac_dev, UAC_MICROPHONE, UAC_SAMPLE_FREQUENCY) == UAC_RET_OK)
            printf("    Mic    Set clock frequency: OK\n");
        else
            printf("    Mic    Set clock frequency: FAIL\n");

        /* Verify frequency after set */
        if (usbh_uac2_clock_get_freq(uac_dev, UAC_MICROPHONE, &freq) == UAC_RET_OK)
            printf("    Mic    Verify clock frequency: %d Hz\n", freq);
        else
            printf("    Mic    Failed to verify clock frequency!\n");
    }

    printf("\nSpeaker mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if (usbh_uac_mute_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
    {
        printf("    Speaker mute state is %d.\n", au8Data[0]);
    }
    else
        printf("    Failed to get speaker mute state!\n");

    printf("\nSpeaker L(F) volume control ===>\n");

#if 0

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get seaker L(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) minimum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) maximum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) volume resolution!\n");

    printf("\nSpeaker R(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) minimum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) maximum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker right channel.            */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) volume resolution!\n");

#endif

    printf("\nSpeaker volume control (L/R channels) ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*  Note: DAC2 has no Master volume, use Left/Right channels.               */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker minimum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker maximum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker volume resolution!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L current volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L current volume!\n");

    if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R current volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R current volume!\n");

#if 0
    printf("\nMixer master volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's microphone.         */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone mute control ===>\n");

    if (usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Microphone mute state is %d.\n", au8Data[0]);
    else
        printf("    Failed to get microphone mute state!\n");

#endif

    printf("\nMicrophone volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current volume value of UAC device's microphone.       */
    /*  Note: DAC2 has no Master volume, use Left channel.         */
    /*-------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &s_u16VolCur) == UAC_RET_OK)
        printf("    Microphone current volume is 0x%x.\n", s_u16VolCur);
    else
        printf("    Failed to get microphone current volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &s_u16VolMin) == UAC_RET_OK)
        printf("    Microphone minimum volume is 0x%x.\n", s_u16VolMin);
    else
        printf("    Failed to get microphone minimum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &s_u16VolMax) == UAC_RET_OK)
        printf("    Microphone maximum volume is 0x%x.\n", s_u16VolMax);
    else
        printf("    Failed to get microphone maximum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get resolution of UAC device's microphone volume value.    */
    /*-------------------------------------------------------------*/
    if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_LEFT_FRONT, &s_u16VolRes) == UAC_RET_OK)
        printf("    Microphone volume resolution is 0x%x.\n", s_u16VolRes);
    else
        printf("    Failed to get microphone volume resolution!\n");

#if 0
    /*-------------------------------------------------------------*/
    /*  Get current auto-gain setting of UAC device's microphone.  */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone automatic gain control ===>\n");

    if (UAC_AutoGainControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Microphone auto gain is %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");

#endif

    printf("\nSampling rate control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's speaker.   */
    /*-------------------------------------------------------------*/
    if (usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's speaker.       */
    /*-------------------------------------------------------------*/
    u32Val = UAC_SAMPLE_FREQUENCY;

    if (usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
        printf("    Failed to set Speaker's current sampling rate %d.\n", u32Val);

    if (usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's microphone.*/
    /*-------------------------------------------------------------*/
    if (usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get microphone's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's microphone.    */
    /*-------------------------------------------------------------*/
    u32Val = UAC_SAMPLE_FREQUENCY;

    if (usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
        printf("    Failed to set microphone's current sampling rate!\n");

    if (usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get microphone's current sampling rate!\n");
}

/*---------------------------------------------------------------------------*/
/*  Start/Stop streaming                                                     */
/*---------------------------------------------------------------------------*/
static int start_streaming(UAC_DEV_T *uac_dev)
{
    int in_ret, out_ret;
    uint32_t srate;

    /*-------------------------------------------------------------*/
    /*  UAC 2.0: Set Clock Source frequency BEFORE streaming       */
    /*-------------------------------------------------------------*/
    if (uac_dev->acif.bcdADC >= 0x0200)
    {
        srate = UAC_SAMPLE_FREQUENCY;
        printf("\nUAC 2.0 Set clock to %d Hz before streaming...\n", srate);

        if (usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &srate) != UAC_RET_OK)
            printf("    WARNING: Failed to set speaker sampling rate!\n");

        if (usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &srate) == UAC_RET_OK)
            printf("    Speaker sampling rate confirmed: %d Hz\n", srate);

        g_u16SampleClockFreq = UAC2_SAMPLE_PAKET(srate);

        printf("g_u16SampleClockFreq=>%d\n", g_u16SampleClockFreq);
    }

    ResetAudioLoopBack();

    if (uac_dev->acif.bcdADC >= 0x0200)
    {
        in_ret = usbh_uac_start_audio_in(uac_dev, audio_2_0_in_callback);
        out_ret = usbh_uac_start_audio_out(uac_dev, audio_2_0_out_callback);
    }
    else
    {
        in_ret = usbh_uac_start_audio_in(uac_dev, audio_1_0_in_callback);
        out_ret = usbh_uac_start_audio_out(uac_dev, audio_1_0_out_callback);
    }

    printf("Starting audio OUT (speaker)...\n");
    printf("Audio OUT start: %s (ret=%d)\n", (out_ret == 0) ? "OK" : "FAIL", out_ret);

    printf("Starting audio IN (microphone)...\n");
    printf("Audio IN start: %s (ret=%d)\n", (in_ret == 0) ? "OK" : "FAIL", in_ret);

    if ((in_ret == 0) && (out_ret == 0))
    {
        s_u32StreamStartTick = s_u32TickCnt;
        printf("Full duplex loopback streaming started. Tick=%d\n", s_u32StreamStartTick);
    }
    else
    {
        s_u32StreamStartTick = 0;
    }

    return (in_ret == 0 && out_ret == 0) ? 0 : -1;
}

static void stop_streaming(UAC_DEV_T *uac_dev)
{
    uint32_t elapsed;

    printf("\nStopping audio IN...\n");
    usbh_uac_stop_audio_in(uac_dev);
    printf("Stopping audio OUT...\n");
    usbh_uac_stop_audio_out(uac_dev);

    if (s_u32StreamStartTick != 0)
    {
        elapsed = (s_u32TickCnt - s_u32StreamStartTick) / 100;
        printf("Streaming stopped. Duration: %d sec, IN: %d, OUT: %d\n",
               elapsed, g_u32UacRecCnt, g_u32UacPlayCnt);
    }
    else
    {
        printf("Streaming stopped. IN: %d, OUT: %d\n",
               g_u32UacRecCnt, g_u32UacPlayCnt);
    }

    s_u32StreamStartTick = 0;
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

int32_t main(void)
{
    UAC_DEV_T *uac_dev = NULL;
    UAC_DEV_T *uac_dev_save = NULL;
    int i8Ch;
    uint16_t u16Val;
    uint32_t u32ConnectCnt = 0;
    int i8Streaming = 0;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    InitDebugUart();                   /* Init DeubgUART for printf */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+----------------------------------------------------+\n");
    printf("|                                                    |\n");
    printf("|  USB Host UAC Loopback Demo                        |\n");
    printf("|                                                    |\n");
    printf("+----------------------------------------------------+\n");
    printf("Commands:\n");
    printf("  +/-  : Mic volume up/down\n");
    printf("  0    : Read mic current volume\n");
    printf("  s    : Stop streaming\n");
    printf("  r    : Restart streaming\n");
    printf("  other: Show IN/OUT counters & memory\n\n");

    usbh_core_init();
    usbh_uac_init();
    usbh_memory_used();

    while (1)
    {
        if (usbh_pooling_hubs())              /* USB Host port detect polling and management */
        {
            /*
             *  Has hub port event.
             */

            uac_dev = usbh_uac_get_device_list();

            if (uac_dev == NULL)
            {
                if (uac_dev_save != NULL)
                {
                    printf("\n<<<< Device disconnected. (connect count: %d) >>>>\n", u32ConnectCnt);
                    i8Streaming = 0;
                    s_u32StreamStartTick = 0;
                    ResetAudioLoopBack();
                }

                uac_dev_save = uac_dev;
                continue;
            }

            if (uac_dev != uac_dev_save)              /* should be newly connected UAC device */
            {
                u32ConnectCnt++;
                printf("\n====> Device connected [%p] (connect #%d)\n", uac_dev, u32ConnectCnt);
                usbh_uac_open(uac_dev);

                uac_control_example(uac_dev);

                /*---------------------------------------------------------*/
                /*  Set speaker volume to 70% use L/R channels           */
                /*  (DAC2 does NOT support Master volume control)           */
                /*---------------------------------------------------------*/
                {
                    uint16_t u16VolMin, u16VolMax, u16VolSet;

                    if ((usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &u16VolMin) == UAC_RET_OK) &&
                            (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &u16VolMax) == UAC_RET_OK))
                    {
                        /* Set to 70% of volume range */
                        u16VolSet = (uint16_t)((int16_t)u16VolMin + (((int16_t)u16VolMax - (int16_t)u16VolMin) * 70) / 100);
                        printf("Setting speaker volume to 70%%: 0x%04x (min=0x%04x, max=0x%04x)\n",
                               u16VolSet, u16VolMin, u16VolMax);

                        if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_LEFT_FRONT, &u16VolSet) != UAC_RET_OK)
                            printf("    WARNING: Failed to set speaker L volume!\n");

                        if (usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_RIGHT_FRONT, &u16VolSet) != UAC_RET_OK)
                            printf("    WARNING: Failed to set speaker R volume!\n");
                    }
                }

                if (start_streaming(uac_dev) == 0)
                    i8Streaming = 1;
                else
                    i8Streaming = 0;

                uac_dev_save = uac_dev;
            }
        }

        if (uac_dev == NULL)
        {
            ResetAudioLoopBack();

            if (!kbhit())
            {
                i8Ch = getchar();
                usbh_memory_used();
            }

            uac_dev_save = uac_dev;

            continue;
        }

        /*-------------------------------------------------------------*/
        /*  Keyboard commands                                          */
        /*-------------------------------------------------------------*/
        if (!kbhit())
        {
            i8Ch = getchar();

            if ((i8Ch == '+') && ((int16_t)(s_u16VolCur + s_u16VolRes) <= (int16_t)s_u16VolMax))
            {
                printf("+");
                u16Val = s_u16VolCur + s_u16VolRes;

                if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
                {
                    printf("    Microphone set volume 0x%x success.\n", u16Val);
                    s_u16VolCur = u16Val;
                    usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_RIGHT_FRONT, &u16Val);
                }
                else
                    printf("    Failed to set microphone volume 0x%x!\n", u16Val);
            }
            else if ((i8Ch == '-') && ((int16_t)(s_u16VolCur - s_u16VolRes) >= (int16_t)s_u16VolMin))
            {
                printf("-");
                u16Val = s_u16VolCur - s_u16VolRes;

                if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
                {
                    printf("    Microphone set volume 0x%x success.\n", u16Val);
                    s_u16VolCur = u16Val;
                    usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_RIGHT_FRONT, &u16Val);
                }
                else
                    printf("    Failed to set microphone volume 0x%x!\n", u16Val);
            }
            else if (i8Ch == '0')
            {
                if (usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &s_u16VolCur) == UAC_RET_OK)
                    printf("    Microphone current volume is 0x%x.\n", s_u16VolCur);
                else
                    printf("    Failed to get microphone current volume!\n");
            }
            else if (i8Ch == 's' || i8Ch == 'S')
            {
                /*---------------------------------------------------------*/
                /*Stop streaming                                           */
                /*---------------------------------------------------------*/
                if (i8Streaming)
                {
                    stop_streaming(uac_dev);
                    i8Streaming = 0;
                    printf("Streaming stopped. Press 'r' to restart.\n");
                }
                else
                {
                    printf("Not streaming.\n");
                }
            }
            else if (i8Ch == 'r' || i8Ch == 'R')
            {
                /*---------------------------------------------------------*/
                /*  Restart streaming test                                 */
                /*---------------------------------------------------------*/
                if (!i8Streaming)
                {
                    printf("Restarting streaming...\n");

                    if (start_streaming(uac_dev) == 0)
                    {
                        i8Streaming = 1;
                        printf("Streaming restarted OK.\n");
                    }
                    else
                    {
                        printf("Streaming restart FAILED!\n");
                    }
                }
                else
                {
                    printf("Already streaming.\n");
                }
            }
            else
            {
                if (i8Streaming && s_u32StreamStartTick)
                {
                    uint32_t elapsed = (s_u32TickCnt - s_u32StreamStartTick) / 100;
                    printf("Time %3ds | IN: %d, OUT: %d\n",
                           elapsed, g_u32UacRecCnt, g_u32UacPlayCnt);
                }
                else
                {
                    printf("IN: %d, OUT: %d (not streaming)\n", g_u32UacRecCnt, g_u32UacPlayCnt);
                }

                usbh_memory_used();
            }

        }  /* end of kbhit() */
    }
}
