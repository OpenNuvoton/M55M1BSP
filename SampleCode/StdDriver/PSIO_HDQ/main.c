/**************************************************************************//**
 * @file    main.c
 * @version V3.00
 * @brief   Demonstrate how to implement HDQ protocol by PSIO.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "BQ2028_driver_EEPROM.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
PSIO_BQ2028_CFG_T g_Config;

typedef void (*PSIO_FUNC)(PSIO_BQ2028_CFG_T *pConfig);
PSIO_FUNC s_pfnPSIOHandler = NULL;

//------------------------------------------------------------------------------
NVT_ITCM void PSIO_IRQHandler(void)
{
    /* Get slot controller done interrupt flag */
    if (PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_SC0IF_Msk))
    {
        /* Clear slot controller done interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_SC0IF_Msk);

        if (s_pfnPSIOHandler != NULL)
        {
            s_pfnPSIOHandler(&g_Config);
        }
    }
    else
    {
        printf("Invalid interrupt occur \n");
    }
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

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

    /* Set PCLK1 to HCLK/8 */
    CLK_SET_PCLK1DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO0_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 0x7Du */
    CLK_SetModuleClock(PSIO0_MODULE, CLK_PSIOSEL_PSIO0SEL_HIRC, CLK_PSIODIV_PSIO0DIV(0x7Du));

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PSIO multi-function pin CH0(PE.14) */
    SET_PSIO0_CH0_PE14();
}

int main()
{
    uint8_t u8RxData = 0, u8DataCnt = 0;
    uint8_t u8Col, u8Row, u8Page, u8CRC;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    /* If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register. */
    SYS_LockReg();

    printf("******************************************************\n");
    printf("|           HDQ TI BQ2028 EEPROM Test Code           |\n");
    printf("|      Please connected PSIO_CH0(PE.14) to device    |\n");
    printf("******************************************************\n");

    /* Use slot controller 0 and pin 0 */
    g_Config.u8SlotCtrl   = PSIO_SC0;
    g_Config.u8Data0Pin   = PSIO_PIN0;

    /* Initialize PSIO setting for BQ2028 */
    PSIO_BQ2028_Open(&g_Config);

    /* Send "Read ID" command and read device ID */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_READID, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    printf("BQ2028 ID is %x.\n", u8RxData);

    /* Send "Read ID" command and read device revision */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_READREV, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    printf("REV is %x.\n", u8RxData);

    /* Send "Read device status" command and read device status */
    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_STATUS, &u8RxData);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Clear reset flag if reset flag is 1 */
    if (u8RxData & STATUS_RST_MSK)
    {
        /* Send "Read control 0 register" command and read control 0 register value */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_CTL0, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Send "Write control 0 register" command and write STATUS_RST_MSK to clear reset flag */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CTL0, u8RxData | STATUS_RST_MSK);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }
    }

    printf("/***********************************/\n");
    printf("|       Buffer R/W Test             |\n");
    printf("/***********************************/\n");

    printf("Buffer 0 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 0 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 0 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF0, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 0 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF0, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check write data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 1 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 1 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 1 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF1, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 1 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF1, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check write data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 2 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 2 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 2 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF2, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 2 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF2, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("Buffer 3 R/W Testing...");

    /* Write 0x0 ~ 0xFF to buffer 3 */
    for (u8DataCnt = 0; u8DataCnt < 0xFF; u8DataCnt++)
    {
        /* Write 1 Byte data to buffer 3 */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_BUF3, u8DataCnt);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Read 1 Byte from buffer 3 */
        PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_BUF3, &u8RxData);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        /* Check data correct or not */
        if (u8DataCnt != u8RxData)
        {
            printf("[Error] Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

            while (1);
        }
    }

    printf("PASS!\n");

    printf("/***********************************/\n");
    printf("|       EEPROM R/W Test             |\n");
    printf("/***********************************/\n");

    /* Enable to write manufacturer area registers */
    PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CTL2, 0x01);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Enable page */
    PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE_EN, 0xFF);

    while (PSIO_BQ2028_BUSY())
    {
        /* Do something here */
    }

    /* Update EEPROM page 0 ~ page 7 */
    for (u8Page = 0; u8Page < 8; u8Page++)
    {
        /* Setting current page which we want to access */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE, u8Page);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        printf("Page %d write pattern\n", u8Page);

        /* Access row 0x0 ~0xF */
        for (u8Row = 0; u8Row < 0x10; u8Row++)
        {
            /* Access column 0x0 ~0x3 */
            for (u8Col = 0; u8Col < 4; u8Col++)
            {
                /* Prepare data and calculating the CRC value */
                u8DataCnt = (u8Page * 0x10 * 4) + (0x10 * u8Row) + u8Col;
                u8CRC   =   PSIO_BQ2028_CRC8(u8DataCnt);

                /* Send "Write and EEPROM address" command and write data to EEPROM */
                PSIO_BQ2028_Write_OneByte(&g_Config, (u8Row << 2) | u8Col | HDQ_W | HDQ_MAP, u8DataCnt);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Send "Check CRC" command and write CRC value */
                PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_CRCT, u8CRC);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Check EEPROM status is normal */
                while (1)
                {
                    PSIO_BQ2028_Read_OneByte(&g_Config, HDQ_CMD_STATUS, &u8RxData);

                    while (PSIO_BQ2028_BUSY())
                    {
                        /* Do something here */
                    }

                    if (u8RxData & (STATUS_PAGEER_MSK | STATUS_MEMER_MSK | STATUS_CRCER_MSK))
                    {
                        printf("status error 0x%x", u8RxData);
                    }
                    else if (!(u8RxData & STATUS_BUSY_MSK))
                        break;
                }
            }
        }
    }

    /* Verify EEPROM page 0 ~ page 7 */
    for (u8Page = 0; u8Page < 8; u8Page++)
    {
        /* Setting current page which we want to access */
        PSIO_BQ2028_Write_OneByte(&g_Config, HDQ_CMD_PAGE, u8Page);

        while (PSIO_BQ2028_BUSY())
        {
            /* Do something here */
        }

        printf("Page %d read pattern\n", u8Page);

        /* Access row 0x0 ~0xF */
        for (u8Row = 0; u8Row < 0x10; u8Row++)
        {
            /* Access column 0x0 ~0x3 */
            for (u8Col = 0; u8Col < 4; u8Col++)
            {
                u8DataCnt = (u8Page * 0x10 * 4) + (0x10 * u8Row) + u8Col;

                /* Send "Read and EEPROM address" command and read data from EEPROM */
                PSIO_BQ2028_Read_OneByte(&g_Config, (u8Row << 2) | u8Col | HDQ_R | HDQ_MAP, &u8RxData);

                while (PSIO_BQ2028_BUSY())
                {
                    /* Do something here */
                }

                /* Check data correct or not */
                if (u8DataCnt != u8RxData)
                {
                    printf("[Error]Write Data=0x%x, Read Data=0x%x\n", u8DataCnt, u8RxData);

                    while (1);
                }
            }
        }
    }

    printf("PASS...\n");

    while (1);
}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
