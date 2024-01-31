/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This is an UAC1.0 sample and used to plays the sound send from PC
 *           through the USB interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    uint32_t volatile i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
#if defined(TESTCHIP_ONLY)
    /* Switch SCLK clock source to APLL0 and Enable APLL0 144MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_144MHZ);
#else
    /* Switch SCLK clock source to APLL0 and Enable APLL0 192MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_192MHZ);
#endif

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

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Enable HSOTG0_ module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);

    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSOTGPHYEN_Msk;

    for (i = 0; i < 0x1000; i++);  // delay > 10 us

    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);


    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

#if defined(ALIGN_AF_PINS)
    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);
#else
    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

#if defined(ALIGN_AF_PINS)
    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;
#else
    /* Set I2C3 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
#endif

}

/* Init I2C interface */
void I2C_Init(void)
{
    /* Open I2C_PORT and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);

    /* Get I2C Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C_PORT));
}

int32_t main(void)
{
    uint32_t volatile i = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    InitDebugUart();

    printf("\n");
    printf("+---------------------------------------------------------+\n");
    printf("|           NuMicro HSUSB UAC1.0 Sample Code              |\n");
    printf("+---------------------------------------------------------+\n");
    printf("HXT clock %d Hz\n", CLK_GetHXTFreq());
    printf("CPU clock %d Hz\n", CLK_GetSCLKFreq());

    /* Init I2C to access codec */
    I2C_Init();

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, MODULE_NoMsk);

    /* Open I2S0 interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

    /* Lock protected registers */
    SYS_LockReg();

    /* Set JK-EN low to enable phone jack on NuMaker board. */
#if defined(ALIGN_AF_PINS)
    SET_GPIO_PB12();
    GPIO_SetMode(PB, BIT12, GPIO_MODE_OUTPUT);
    PB12 = 0;
#else
    SET_GPIO_PD4();
    GPIO_SetMode(PD, BIT4, GPIO_MODE_OUTPUT);
    PD4 = 0;
#endif

#if NAU8822
    NAU8822_Setup();
#else
    NAU88L25_Reset();
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(10000);
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    /* Configure PDMA */
    PDMA_Init();

    /* Configure TIMER0 for adjusting codec's PLL */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 100);
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TIMER0_IRQn, 3);
    NVIC_EnableIRQ(TIMER0_IRQn);

    HSUSBD_Open(&gsHSInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    NVIC_EnableIRQ(HSUSBD_IRQn);
    HSUSBD_CLR_SE0();

    while (1)
    {
        if (g_usbd_rxflag)
        {
            UAC_GetPlayData();
        }
        else if (u8AudioPlaying && (u8TxDataCntInBuffer < 1))
        {
            UAC_DeviceDisable(1);
        }

        if ((g_usbd_UsbAudioState == UAC_START_AUDIO_RECORD) && g_usbd_txflag)
        {
            UAC_SendRecData();
        }

        if ((u8TxDataCntInBuffer != 0) && ((++i % 0x50000) == 0))
        {
            printf("%d <-> %d\n", u8TxDataCntInBuffer, u8RxDataCntInBuffer);
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
