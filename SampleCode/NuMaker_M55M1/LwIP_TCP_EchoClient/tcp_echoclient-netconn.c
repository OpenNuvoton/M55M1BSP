

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "string.h"
#include "tcp_echoclient-netconn.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TCPECHOCLIENT_THREAD_PRIO    ( tskIDLE_PRIORITY + 2UL )
#define TCPECHOCLIENT_THREAD_STACKSIZE  200
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  TCP echo client thread
  * @param arg pointer on argument(not used here)
  * @retval None
  */
static void tcp_echoclient_netconn_thread(void *arg)
{
    struct netconn *conn;
    err_t err;
    ip_addr_t server_ip;
    unsigned short server_port;
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    char string[] = "nuvoton";

    IP4_ADDR(&server_ip, 192, 168, 1, 2);
    server_port = 80;

    /* Create a new TCP connection handle */
    conn = netconn_new(NETCONN_TCP);

    if (conn != NULL)
    {
        /* Bind to port 5168 with default IP address */
        err = netconn_bind(conn, NULL, 5168);

        if (err == ERR_OK)
        {
            err = netconn_connect(conn, &server_ip, server_port);

            if (err == ERR_OK)
            {
                printf("Connect to server succeed !\n");

                while (1)
                {
                    printf("Send for TCP packet ...");
                    netconn_write(conn, (const unsigned char *)string, (size_t)strlen(string), NETCONN_NOCOPY);

                    /* Read the data from the port, blocking if nothing yet there.
                     We assume the request is in one netbuf */
                    if (netconn_recv(conn, &inbuf) != ERR_OK)
                    {
                        netbuf_delete(inbuf);
                        printf("### perform netconn_recv fail.\n");
                        netconn_delete(conn);
                        break;
                    }

                    if (inbuf != NULL)
                    {
                        if (netconn_err(conn) == ERR_OK)
                        {
                            netbuf_data(inbuf, (void **)&buf, &buflen);
                            printf("Recv: %s\n", buf);
                        }
                    }

                    /* Delete the buffer (netconn_recv gives us ownership,
                     so we have to make sure to deallocate the buffer) */
                    netbuf_delete(inbuf);

                    vTaskDelay(1000);
                }
            }
            else
                printf("Connect to server fail !\n");
        }
        else
        {
            printf("can not bind netconn");
            netconn_close(conn);
        }
    }
    else
    {
        printf("can not create netconn");
    }
}

/**
  * @brief  Initialize the TCP client (start its thread)
  * @param  none
  * @retval None
  */
void tcp_echoclient_netconn_init()
{
    sys_thread_new("TCPECHO", tcp_echoclient_netconn_thread, NULL, TCPECHOCLIENT_THREAD_STACKSIZE, TCPECHOCLIENT_THREAD_PRIO);
}
