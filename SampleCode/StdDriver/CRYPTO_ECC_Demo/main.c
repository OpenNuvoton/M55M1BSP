/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_ECC_Demo code for M55M1 series MCU
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          192  /* Select ECC P-192 curve, 192-bits key length */
#define PRNG_KEY_SIZE       PRNG_KEY_SIZE_256
#define CURVE_P_SIZE        CURVE_P_192

static char d[168];                         /* private key */
static char Qx[168], Qy[168];               /* temporary buffer used to keep output public keys */
static char k[168];                         /* random integer k form [1, n-1]                */
static char msg[] = "This is a message. It could be encypted.";
static char R[168], S[168];                 /* temporary buffer used to keep digital signature (R,S) pair */




#define ENDIAN(x)   ((((x)>>24)&0xff) | (((x)>>8)&0xff00) | (((x)<<8)&0xff0000) | ((x)<<24))

uint8_t Byte2Char(uint8_t c);
void CRYPTO_IRQHandler(void);
void  dump_buff_hex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

uint8_t Byte2Char(uint8_t c)
{
    if (c < 10)
        return (c + '0');

    if (c < 16)
        return (c - 10 + 'a');

    return 0;
}


void CRYPTO_IRQHandler()
{
    ECC_Complete(CRYPTO);
}


void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);

        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);

        printf("  ");

        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");

            nBytes--;
        }

        nIdx += 16;
        printf("\n");
    }

    printf("\n");
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


    /* Enable PLL0 200MHz clock */
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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t j, m, err;
    uint32_t time, i, nbits;
    uint32_t au32r[(KEY_LENGTH + 31) / 32];
    uint8_t *au8r;


    SYS_UnlockReg();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+---------------------------------------+\n");
    printf("|     Crypto ECC Demo Sample Code       |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);
    ECC_ENABLE_INT(CRYPTO);

    nbits = KEY_LENGTH;

    /* Initial TRNG */
    RNG_Open();

    /* Init Timer */
    SysTick->LOAD  = 0xfffffful;                                              /* set reload register */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;    /* Enable SysTick IRQ and SysTick Timer */

    au8r = (uint8_t *)&au32r[0];

    do
    {

        /* Generate random number for private key */
        RNG_Random(au32r, (nbits + 31) / 32);

        for (i = 0, j = 0; i < nbits / 8; i++)
        {
            d[j++] = Byte2Char(au8r[i] & 0xf);
            d[j++] = Byte2Char(au8r[i] >> 4);
        }

        d[j] = 0; // NULL end


        printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if (ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_SIZE, d))
        {
            break;
        }
        else
        {
            /* Invalid key */
            printf("Current private key is not valid. Need a new one.\n");
        }

    } while (1);

    /* Reset SysTick to measure time */
    SysTick->VAL = 0;

    if (ECC_GeneratePublicKey(CRYPTO, CURVE_P_SIZE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");

        while (1);
    }

    time = 0xffffff - SysTick->VAL;

    printf("Public Qx is %s\n", Qx);
    printf("Public Qy is %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    /*
        Try to generate signature serveral times with private key and verificate them with the same
        public key.

    */
    for (m = 0; m < 3; m++)
    {
        printf("//-------------------------------------------------------------------------//\n");

        /* Generate random number k */
        RNG_Random(au32r, (nbits + 31) / 32);

        for (i = 0, j = 0; i < nbits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }

        k[j] = 0; // NULL End

        printf("  k = %s\n", k);

        if (ECC_IsPrivateKeyValid(CRYPTO, CURVE_P_SIZE, k))
        {
            /* Private key check ok */
        }
        else
        {
            /* Invalid key */
            printf("Current k is not valid\n");
            return -1;

        }

        SysTick->VAL = 0;

        if (ECC_GenerateSignature(CRYPTO, CURVE_P_SIZE, msg, d, k, R, S) < 0)
        {
            printf("ECC signature generation failed!!\n");
            return -1;
        }

        time = 0xffffff - SysTick->VAL;

        printf("  R  = %s\n", R);
        printf("  S  = %s\n", S);
        printf("  msg= %s\n", msg);
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

        SysTick->VAL = 0;
        err = ECC_VerifySignature(CRYPTO, CURVE_P_SIZE, msg, Qx, Qy, R, S);
        time = 0xffffff - SysTick->VAL;

        if (err < 0)
        {
            printf("ECC signature verification failed!!\n");
            return -1;
        }
        else
        {
            printf("ECC digital signature verification OK.\n");
        }

        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);
    }

    printf("Demo Done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
