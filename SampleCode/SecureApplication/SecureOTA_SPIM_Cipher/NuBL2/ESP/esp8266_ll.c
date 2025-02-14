/**
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software,
 * | and to permit persons to whom the Software is furnished to do so,
 * | subject to the following conditions:
 * |
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * |
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "esp8266_ll.h"
#include "esp8266.h"

/* Include platform dependant libraries */
#include "NuMicro.h"
#include "ota_transfer.h"

#if ESP_RTOS
    osMutexId id;
#endif /* ESP_RTOS */

uint8_t ESP_LL_Callback(ESP_LL_Control_t ctrl, void *param, void *result)
{
    switch (ctrl)
    {
        case ESP_LL_Control_Init:                   /* Initialize low-level part of communication */
        {
            ESP_LL_t *LL = (ESP_LL_t *)param;       /* Get low-level value from callback */

            NVT_UNUSED(LL);
            /************************************/
            /*  Device specific initialization  */
            /************************************/
            //UART_Open(ESP_WIFI_UART_PORT, LL->Baudrate);
            UART_ENABLE_INT(ESP_WIFI_UART_PORT, (UART_INTEN_RDAIEN_Msk | UART_INTEN_BUFERRIEN_Msk));
            NVIC_EnableIRQ(ESP_WIFI_UART_IRQn);

#if defined(ESP_WIFI_RESET_PIN)
            ESP_WIFI_RESET_PIN = 1;
#endif

            if (result)
            {
                *(uint8_t *)result = 0;             /* Successfully initialized */
            }

            return 1;                               /* Return 1 = command was processed */
        }

        case ESP_LL_Control_Send:
        {
            ESP_LL_Send_t *send = (ESP_LL_Send_t *)param;   /* Get send parameters */
            /* Send actual data to UART */
            UART_Write(ESP_WIFI_UART_PORT, (uint8_t *)send->Data, send->Count);   /* Send actual data */

            if (result)
            {
                *(uint8_t *)result = 0;             /* Successfully send */
            }

            return 1;                               /* Command processed */
        }

#if defined(ESP_WIFI_RESET_PIN)

        case ESP_LL_Control_SetReset:               /* Set reset value */
        {
            uint8_t state = *(uint8_t *)param;      /* Get state packed in uint8_t variable */

            if (state == ESP_RESET_SET)             /* Check state value */
            {
                ESP_WIFI_RESET_PIN = 0;
            }
            else
            {
                ESP_WIFI_RESET_PIN = 1;
            }

            return 1;                               /* Command has been processed */
        }

#endif /* defined(ESP_WIFI_RESET_PORT) && defined(ESP_WIFI_RESET_PIN) */

#if ESP_RTOS

        case ESP_LL_Control_SYS_Create:             /* Create system synchronization object */
        {
            ESP_RTOS_SYNC_t *Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            id = osMutexCreate(Sync);               /* Create mutex */

            if (result)
            {
                *(uint8_t *)result = id == NULL;    /*!< Set result value */
            }

            return 1;                               /* Command processed */
        }

        case ESP_LL_Control_SYS_Delete:             /* Delete system synchronization object */
        {
            ESP_RTOS_SYNC_t *Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            osMutexDelete(Sync);                    /* Delete mutex object */

            if (result)
            {
                *(uint8_t *)result = id == NULL;    /*!< Set result value */
            }

            return 1;                               /* Command processed */
        }

        case ESP_LL_Control_SYS_Request:            /* Request system synchronization object */
        {
            ESP_RTOS_SYNC_t *Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            (void)Sync;                             /* Prevent compiler warnings */

            *(uint8_t *)result = osMutexWait(id, 1000) == osOK ? 0 : 1; /* Set result according to response */
            return 1;                               /* Command processed */
        }

        case ESP_LL_Control_SYS_Release:            /* Release system synchronization object */
        {
            ESP_RTOS_SYNC_t *Sync = (ESP_RTOS_SYNC_t *)param;   /* Get pointer to sync object */
            (void)Sync;                             /* Prevent compiler warnings */

            *(uint8_t *)result = osMutexRelease(id) == osOK ? 0 : 1;    /* Set result according to response */
            return 1;                               /* Command processed */
        }

#endif /* ESP_RTOS */

        default:
            return 0;
    }
}

/* USART receive interrupt handler */
void WiFi_ReceiveHandler(uint8_t ch)
{
#if ENABLE_DBG_RX_MSG
    printf("%c", ch);
    //printf("0x%X",ch);
#endif
    /* Send received character to ESP stack */
    ESP_DataReceived(&ch, 1);
}
