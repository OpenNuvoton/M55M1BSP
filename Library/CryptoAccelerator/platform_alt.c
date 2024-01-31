/*
 *  Platform abstraction layer
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "common.h"

#if defined(MBEDTLS_PLATFORM_C)

#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include "NuMicro.h"


#if defined(MBEDTLS_PLATFORM_EXIT_ALT)
#if defined(MBEDTLS_PLATFORM_STD_EXIT)
/*
 * Make dummy function to prevent NULL pointer dereferences
 */
static void platform_exit_uninit(int status)
{
    ((void)status);

    for (;;) {}
}


#endif /* !MBEDTLS_PLATFORM_STD_EXIT */

void (*mbedtls_exit)(int status) = MBEDTLS_PLATFORM_STD_EXIT;

int mbedtls_platform_set_exit(void (*exit_func)(int status))
{
    mbedtls_exit = exit_func;
    return (0);
}
#endif /* MBEDTLS_PLATFORM_EXIT_ALT */

#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
/*
 * Placeholder platform setup that does nothing by default
 */
int mbedtls_platform_setup(mbedtls_platform_context *ctx)
{
    (void)ctx;

    //    SYS_UnlockReg();

    //    /*---------------------------------------------------------------------------------------------------------*/
    //    /* Init System Clock                                                                                       */
    //    /*---------------------------------------------------------------------------------------------------------*/

    //    /* Enable HIRC, HXT and LXT clock */
    //    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    //    /* Wait for HIRC, HXT and LXT clock ready */
    //    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    //    /* Set PCLK0 and PCLK1 to HCLK/2 */
    //    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    //    /* Set core clock to 200MHz */
    //    CLK_SetCoreClock(200000000);

    //    /* Enable all GPIO clock */
    //    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
    //        CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    //    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    //    /* Enable UART0 module clock */
    //    CLK_EnableModuleClock(UART0_MODULE);

    //    /* Enable CRYPTO module clock */
    //    CLK_EnableModuleClock(CRPT_MODULE);


    //    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    //    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    //    /*---------------------------------------------------------------------------------------------------------*/
    //    /* Init I/O Multi-function                                                                                 */
    //    /*---------------------------------------------------------------------------------------------------------*/

    //    /* Set multi-function pins for UART RXD and TXD */
    //    SET_UART0_RXD_PB12();
    //    SET_UART0_TXD_PB13();


    //    SYS_ResetModule(UART0_RST);
    //    UART_Open(UART0, 115200);

    return (0);
}

/*
 * Placeholder platform teardown that does nothing by default
 */
void mbedtls_platform_teardown(mbedtls_platform_context *ctx)
{
    (void)ctx;
}
#endif /* MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT */

#endif /* MBEDTLS_PLATFORM_C */
