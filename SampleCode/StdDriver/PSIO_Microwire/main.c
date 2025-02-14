/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to implement Microwire protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "AT93C46D_driver_EEPROM.h"

volatile uint32_t *pu32ChipSelectPin, *pu32InputPin;

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO0_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 6 */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(6));

    CLK_EnableModuleClock(GPIOE_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* PSIO multi-function pin CH0(PE.14), CH1(PE.15), CH2(PE.5) and CH3(PE.4) */
    SET_PSIO0_CH0_PE14();
    SET_PSIO0_CH1_PE15();
    SET_PSIO0_CH2_PE5();
    SET_PSIO0_CH3_PE4();
}

void SetCSPinToPSIO(void)
{
    SET_PSIO0_CH0_PE14();
}

void SetCSPinToGPIO(void)
{
    SET_GPIO_PE14();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_PSIO_AT93C46D sConfig;
    uint8_t u8TxData = 0;
    uint8_t u8RxData = 0;
    uint8_t u8Address = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|      AT93C46D Microwire EEPROM  Test Code            | \n");
    printf("| Please connected PSIO_CH0(PE.14), PSIO_CH1(PE.15)    | \n");
    printf("| ,PSIO_CH2(PE.5), PSIO_CH4(PE.4)to device.            | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0, pin 0, pin1, pin 2, pin 3 */
    sConfig.u8SlotCtrl      = PSIO_SC0;
    sConfig.u8ChipSelectPin = PSIO_PIN0;
    sConfig.u8ClockPin      = PSIO_PIN1;
    sConfig.u8DO            = PSIO_PIN2;
    sConfig.u8DI            = PSIO_PIN3;

    /* Initialize PIN config */
    pu32ChipSelectPin   = &PE14;
    pu32InputPin        = &PE5;

    /* Set CS pin output mode when GPIO function */
    GPIO_SetMode(PE, BIT14, GPIO_MODE_OUTPUT);

    /* Initialize PSIO setting for AT93C46D */
    PSIO_AT93C46D_Init(&sConfig);

    /* Send write enable command */
    PSIO_AT93C46D_EraseWrite_Enable(&sConfig);

    printf("Read/Write data to AT93C46D\n");

    for (u8Address = 0; u8Address < EEPROM_SIZE / DATA_WIDTH; u8Address++, u8TxData = u8Address + 2)
    {
        /* Send erase command */
        PSIO_AT93C46D_Erase(&sConfig, u8Address);

        /* Write data to AT93C46D */
        PSIO_AT93C46D_Write(&sConfig, u8Address, &u8TxData);

        /* Read data from AT93C46D */
        PSIO_AT93C46D_Read(&sConfig, u8Address, &u8RxData);

        /* Compare data is correct */
        if (u8TxData != u8RxData)
        {
            printf("[Error] TxData:0x%x, RxData:0x%x\n", u8TxData, u8RxData);

            while (1);
        }
        else
        {
            printf(".");
        }
    }

    printf("PASS!\n");

    while (1);
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
