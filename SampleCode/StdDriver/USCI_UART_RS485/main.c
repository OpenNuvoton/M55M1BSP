/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive data in UART RS485 mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define RS485_ADDRSS1     0xC0
#define RS485_ADDRSS2     0xA2
#define RS485_ADDRSS3     0xB1
#define RS485_ADDRSS4     0xD3


#define ADDR_NUM 4
#define DATA_NUM 10


/*---------------------------------------------------------------------------------------------------------*/
/* Global variable                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_au32AddrBuffer[ADDR_NUM];
static volatile uint32_t g_au32DataBuffer[ADDR_NUM][DATA_NUM];
static uint8_t g_u8AddrIndex = 0;
static uint8_t g_u8DataIndex = 0;
static volatile uint8_t g_u8ReceiveDone = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void RS485_HANDLE(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void USCI0_Init(void);
void SYS_Init(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle USCI interrupt event                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void USCI0_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE(void)
{
    volatile uint32_t u32ProtSts = UUART_GET_PROT_STATUS(UUART0);
    volatile uint32_t u32BufSts = UUART_GET_BUF_STATUS(UUART0);
    uint32_t u32Data;

    if (u32ProtSts & UUART_PROTSTS_RXENDIF_Msk)     /* Receive end interrupt */
    {
        /* Handle received data */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);
        u32Data = UUART_READ(UUART0);

        if (u32Data & 0x100)
            g_au32AddrBuffer[g_u8AddrIndex++] = u32Data;
        else
        {
            g_au32DataBuffer[g_u8AddrIndex - 1][g_u8DataIndex++] = u32Data;

            if (g_u8DataIndex == DATA_NUM)
            {
                if (g_u8AddrIndex == ADDR_NUM)
                    g_u8ReceiveDone = 1;
                else
                    g_u8DataIndex = 0;
            }
        }
    }
    else if (u32BufSts & UUART_BUFSTS_RXOVIF_Msk)     /* Receive buffer over-run error interrupt */
    {
        UUART_CLR_BUF_INT_FLAG(UUART0, UUART_BUFSTS_RXOVIF_Msk);
        printf("\nBuffer Error...\n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave(void)
{
    /* Set UART line configuration and control signal output inverse */
    UUART_SetLine_Config(UUART0, 0, UUART_WORD_LEN_9, UUART_PARITY_NONE, UUART_STOP_BIT_1);
    UUART0->LINECTL |= UUART_LINECTL_CTLOINV_Msk;

    /* Enable RTS auto direction function */
    UUART0->PROTCTL |= UUART_PROTCTL_RTSAUDIREN_Msk;

    printf("+-----------------------------------------------------------+\n");
    printf("|    RS485 Mode                                             |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Receive address and data byte.                            |\n");
    printf("+-----------------------------------------------------------+\n");

    /* Enable USCI receive end and receive buffer over-run error Interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    UUART_ENABLE_BUF_INT(UUART0, UUART_BUFCTL_RXOVIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);

    printf("Ready to receive data...\n");

    /* Wait receive complete */
    while (g_u8ReceiveDone == 0);

    for (g_u8AddrIndex = 0; g_u8AddrIndex < ADDR_NUM; g_u8AddrIndex++)
    {
        printf("\nAddr=0x%x,Get:", (g_au32AddrBuffer[g_u8AddrIndex] & 0xFF));

        for (g_u8DataIndex = 0; g_u8DataIndex < DATA_NUM; g_u8DataIndex++)
            printf("%d,", (g_au32DataBuffer[g_u8AddrIndex][g_u8DataIndex] & 0xFF));
    }

    /* Disable USCI interrupt */
    UUART_DISABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    UUART_DISABLE_BUF_INT(UUART0, UUART_BUFCTL_RXOVIEN_Msk);
    NVIC_DisableIRQ(USCI0_IRQn);

    printf("\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    UUART_WRITE(UUART0, (0x100 | u8data));
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    for (u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while (UUART_GET_TX_FULL(UUART0));          /* Wait if Tx is full */

        UUART_WRITE(UUART0, pu8TxBuf[u32Count]);    /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster(void)
{
    int32_t i32Idx;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data     |\n");
    printf("| bytes to test RS485 9-bit mode. Please connect TX/RX to   |\n");
    printf("| another board and wait its ready to receive.              |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    getchar();

    /* Set UART line configuration and control signal output inverse */
    UUART_SetLine_Config(UUART0, 0, UUART_WORD_LEN_9, UUART_PARITY_NONE, UUART_STOP_BIT_1);
    UUART0->LINECTL |= UUART_LINECTL_CTLOINV_Msk;

    /* Enable RTS auto direction function */
    UUART0->PROTCTL |= UUART_PROTCTL_RTSAUDIREN_Msk;

    /* Prepare data to transmit */
    for (i32Idx = 0; i32Idx < 10; i32Idx++)
    {
        g_u8SendDataGroup1[i32Idx] = (uint8_t)i32Idx;
        g_u8SendDataGroup2[i32Idx] = (uint8_t)i32Idx + 10;
        g_u8SendDataGroup3[i32Idx] = (uint8_t)i32Idx + 20;
        g_u8SendDataGroup4[i32Idx] = (uint8_t)i32Idx + 30;
    }

    /* Send different address and data for test */
    printf("Send Address %x and data 0~9\n", RS485_ADDRSS1);
    RS485_SendAddressByte(RS485_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n", RS485_ADDRSS2);
    RS485_SendAddressByte(RS485_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);

    printf("Send Address %x and data 20~29\n", RS485_ADDRSS3);
    RS485_SendAddressByte(RS485_ADDRSS3);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);

    printf("Send Address %x and data 30~39\n", RS485_ADDRSS4);
    RS485_SendAddressByte(RS485_ADDRSS4);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);

    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|                                          |Slave| |\n");
    printf("| |    TX|--USCI0_DAT1(PA.9)     USCI0_DAT0(PA.10)--|RX   | |\n");
    printf("| |   RTS|--USCI0_CTL1(PA.8)     USCI0_CTL1(PA.8)---|RTS  | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       RS485 Function Test                                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.

        Master:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'.
            4.Data bytes : the parity bit should be '0'.
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set RTS auto direction enabled and HW will control RTS pin. CTLOINV is set to '1'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".

        Note: User can measure transmitted data waveform on TX and RX pin.
              RTS pin is used for RS485 transceiver to control transmission direction.
              RTS pin is low in idle state. When master is sending data, RTS pin will be pull high.
              The connection to RS485 transceiver is as following figure for reference.
               __________     ___________      ___________      __________
              |          |   |           |    |           |    |          |
              |Master    |   |RS485      |    |RS485      |    |Slave     |
              | UART_TX  |---|Transceiver|<==>|Transceiver|----| UART_RX  |
              | UART_RTS |---|           |    |           |----| UART_RTS |
              |__________|   |___________|    |___________|    |__________|
    */

    if (u32Item == '0')
        RS485_9bitModeMaster();
    else
        RS485_9bitModeSlave();

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable USCI0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set multi-function pins for USCI0_DAT0(PA.10), USCI0_DAT1(PA.9) and USCI0_CTL1(PA.8) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
    SET_USCI0_CTL1_PA8();

}



void USCI0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS_ResetModule(SYS_USCI0RST);

    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Init USCI0 */
    USCI0_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART RS485 sample function */
    RS485_FunctionTest();

    printf("\nUSCI UART Sample Program End\n");

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
