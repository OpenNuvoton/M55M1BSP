/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A sample provides command line interface for reading files from
 *           USB disk and writing to SPIM flash. It also provide functions of
 *           dump SPIM flash, compare USB disk file with SPIM flash,
 *           and branch to run code on SPIM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "ff.h"
#include "diskio.h"

#define USE_USB_APLL1_CLOCK       1
#define BUFF_SIZE                 2048      /* Working buffer size                        */
#define SPIM_FLASH_MAX_SIZE       0x8000000 /* Assumed maximum flash size 128 MB          */
#define SPIM_FLASH_PAGE_SIZE      0x10000   /* SPIM flash page size, depend on flash      */
#define IS_4BYTES_ADDR            0         /* W25Q20 does not support 4-bytes address mode. */

/* 0x02h : CMD_NORMAL_PAGE_PROGRAM Command Phase Table */
SPIM_PHASE_T sWb02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                    //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    0,
};

/* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
SPIM_PHASE_T sWb03hRdCMD =
{
    CMD_DMA_FAST_READ,                                          // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,        // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,       // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR, SPIM_OP_DISABLE,    // Data Phase
    8,                                                          // Dummy Cycle Phase
};


typedef void (FUNC_PTR)(void);

char      usbh_path[] = { '3', ':', 0 };    /* USB drive started from 3                   */
uint8_t   idBuf[3];

#ifdef __ICCARM__
    #pragma data_alignment=32
    uint8_t  Buff1[BUFF_SIZE] ;                 /* Working buffer                             */
    uint8_t  Buff2[BUFF_SIZE] ;                 /* Working buffer                             */
#else
    uint8_t  Buff1[BUFF_SIZE] __attribute__((aligned(32)));     /* Working buffer                             */
    uint8_t  Buff2[BUFF_SIZE] __attribute__((aligned(32)));     /* Working buffer                             */
#endif

char      Line[128];                        /* Console input buffer                       */

FILINFO   Finfo;
FIL       file;


volatile uint32_t  g_tick_cnt;              /* SYSTICK timer counter                      */

NVT_ITCM void SysTick_Handler(void)
{
    g_tick_cnt++;                           /* timer tick counting 100 per second         */
}

void enable_sys_tick(int ticks_per_second)
{
    g_tick_cnt = 0;

    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks()
{
    return g_tick_cnt;
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

/*
 *  Set stack base address to SP register.
 */
#ifdef __ARMCC_VERSION                 /* for Keil compiler */
void __set_SP(uint32_t _sp)
{
    __ASM volatile("msr msp, r0");
    __ASM volatile("bx  lr");
}
#endif


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

    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

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
    CLK_EnableModuleClock(SPIM0_MODULE);

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
    //delay_us(20);

    /* Set Debug Uart CLK*/
    SetDebugUartCLK();
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Init SPIM multi-function pins, MOSI(PJ.1), MISO(PI.13), CLK(PJ.0), SS(PI.12), D3(PI.15), and D2(PI.14) */
    /* Init SPIM multi-function pins */
    SET_SPIM0_CLK_PC4();
    SET_SPIM0_MISO_PG12();
    SET_SPIM0_MOSI_PG11();
    SET_SPIM0_D2_PC0();
    SET_SPIM0_D3_PG10();
    SET_SPIM0_SS_PC3();

    PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk |
                  GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk);

    PG->SMTEN |= (GPIO_SMTEN_SMTEN10_Msk |
                  GPIO_SMTEN_SMTEN11_Msk |
                  GPIO_SMTEN_SMTEN12_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PC, BIT0, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PC, BIT3, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PC, BIT4, GPIO_SLEWCTL_HIGH);

    GPIO_SetSlewCtl(PG, BIT10, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PG, BIT11, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PG, BIT12, GPIO_SLEWCTL_HIGH);

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.8     */
    SET_USB_VBUS_EN_PB8();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.9   */
    SET_USB_VBUS_ST_PB9();

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

/*----------------------------------------------*/
/* Get a line from the input                    */
/*----------------------------------------------*/
void get_line(char *buff, int len)
{
    char    c;
    int     idx = 0;

    for (;;)
    {
        c = getchar();
        putchar(c);

        if (c == '\r') break;

        if ((c == '\b') && idx) idx--;

        if ((c >= ' ') && (idx < len - 1)) buff[idx++] = c;
    }

    buff[idx] = 0;
    putchar('\n');
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
}

void put_rc(FRESULT rc)
{
    const char *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    //FRESULT i;
    uint32_t i;

    for (i = 0; (i != (UINT)rc) && *p; i++)
    {
        while (*p++) ;
    }

    printf(_T("rc=%d FR_%s\n"), (UINT)rc, p);
}

int xatoi(                                  /* 0:Failed, 1:Successful                     */
    char       **str,                       /* Pointer to pointer to the string           */
    uint32_t   *res                         /* Pointer to a variable to store the value   */
)
{
    uint32_t   val;
    uint8_t    r, s = 0;
    char       c;


    *res = 0;

    while ((c = **str) == ' ')(*str)++;     /* Skip leading spaces */

    if (c == '-')       /* negative? */
    {
        s = 1;
        c = *(++(*str));
    }

    if (c == '0')
    {
        c = *(++(*str));

        switch (c)
        {
            case 'x':       /* hexadecimal */
                r = 16;
                c = *(++(*str));
                break;

            case 'b':       /* binary */
                r = 2;
                c = *(++(*str));
                break;

            default:
                if (c <= ' ') return 1; /* single zero */

                if (c < '0' || c > '9') return 0;   /* invalid char */

                r = 8;      /* octal */
        }
    }
    else
    {
        if (c < '0' || c > '9') return 0;   /* EOL or invalid char */

        r = 10;         /* decimal */
    }

    val = 0;

    while (c > ' ')
    {
        if (c >= 'a') c -= 0x20;

        c -= '0';

        if (c >= 17)
        {
            c -= 7;

            if (c <= 9) return 0;   /* invalid char */
        }

        if (c >= r) return 0;       /* invalid char for current radix */

        val = val * r + c;
        c = *(++(*str));
    }

    if (s) val = 0 - val;           /* apply sign if needed */

    *res = val;
    return 1;
}

static void  dump_buff_hex(uint32_t addr, uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;

    while (nBytes > 0)
    {
        printf("0x%08X  ", addr + nIdx);

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

int  show_root_dir()
{
    long    p1;                             /* total file size counter                    */
    FRESULT res;                            /* FATFS operation return code                */

    DIR dir;                                /* FATFS directory object                     */
    UINT s1, s2;                            /* file and directory counter                 */

    if (f_opendir(&dir, usbh_path))         /* try to open USB drive root directory       */
        return -1;                          /* open failed                                */

    p1 = s1 = s2 = 0;                       /* initialize counters                        */

    for (; ;)                               /* loop until reached end of root directory   */
    {
        res = f_readdir(&dir, &Finfo);      /* read directory entry                       */

        if ((res != FR_OK) || !Finfo.fname[0]) break;  /* no more entries                 */

        if (Finfo.fattrib & AM_DIR)         /* is a directory?                            */
        {
            s2++;                           /* increase directory counter                 */
        }
        else                                /* should be a file                           */
        {
            s1++;                           /* increase file counter                      */
            p1 += Finfo.fsize;              /* increase total file size counter           */
        }

        /* print file entry information               */
        printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
               (Finfo.fattrib & AM_DIR) ? 'D' : '-',    /* is a directory?                */
               (Finfo.fattrib & AM_RDO) ? 'R' : '-',    /* is read-only?                  */
               (Finfo.fattrib & AM_HID) ? 'H' : '-',    /* is hidden?                     */
               (Finfo.fattrib & AM_SYS) ? 'S' : '-',    /* is system file/directory?      */
               (Finfo.fattrib & AM_ARC) ? 'A' : '-',    /* is an archive?                 */
               (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,       /* date */
               (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, Finfo.fsize, Finfo.fname);    /* time */
#if _USE_LFN

        for (p2 = strlen(Finfo.fname); p2 < 14; p2++)   /* print the long file name       */
            putchar(' ');

        printf("%s\n", Lfname);
#else
        putchar('\n');
#endif
    }

    /* print the statistic information            */
    printf("%4u File(s),%10lu bytes total\n%4u Dir(s)\n\n", s1, p1, s2);
    return 0;
}

/*
 *  w <file> <addr>
 */
int  write_file_to_flash(char *cmdline)
{
    char        *ptr = cmdline;             /* command string pointer                     */
    char        fname[64];                  /* file name string pointer                   */
    uint32_t    faddr, page_addr, addr;     /* flash address                              */
    UINT        len;                        /* data length                                */
    FRESULT     res;                        /* FATFS operation return code                */

    while (*ptr == ' ') ptr++;              /* skip space characters                      */

    for (len = 0; len < 60; len++)
    {
        if ((*ptr == ' ') || (*ptr == 0))
            break;

        fname[len] = *ptr++;
    }

    fname[len] = 0;

    if (!xatoi(&ptr, &faddr))               /* get <addr> parameter (SPIM flash address)  */
    {
        printf("Usage:  w <file> <addr>\n");
        return -1;
    }

    faddr = faddr - (faddr % SPIM_FLASH_PAGE_SIZE);  /* force page alignment              */

    printf("Write file [%s] to SPIM address 0x%x...\n", fname, faddr);
    res = f_open(&file, fname, FA_OPEN_EXISTING | FA_READ);   /* Open file                */

    if (res)
    {
        put_rc(res);                        /* Open failed, print error message           */
        return -1;                          /* Abort...                                   */
    }

    SPIM_ReadJedecId(SPIM0, idBuf, sizeof(idBuf), 1, 0);
    printf("Flash ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    //SPIM_Enable_4Bytes_Mode(SPIM0, IS_4BYTES_ADDR, 1);  /* Enable 4-bytes address mode?          */

    /*
     *  Erase SPIM flash page and program...
     */
    for (page_addr = faddr; page_addr < SPIM_FLASH_MAX_SIZE; page_addr += SPIM_FLASH_PAGE_SIZE)
    {
        if (f_eof(&file))
        {
            printf("OK [%d]\n", res);
            f_close(&file);             /* close file                                 */
            return 0;                   /* done                                       */
        }

        /* Erase SPIM flash */
        printf("Erase flash page 0x%x...\n", page_addr);
        SPIM_EraseBlock(SPIM0, page_addr, IS_4BYTES_ADDR, OPCODE_BE_64K, 1, 1);

        memset(Buff2, 0xff, BUFF_SIZE);     /* prepared compare buffer                    */

        /* Verify erased page */
        printf("Verify erased page...");

        for (addr = page_addr; addr < page_addr + SPIM_FLASH_PAGE_SIZE; addr += BUFF_SIZE)
        {
            memset(Buff1, 0x11, BUFF_SIZE); /* fill buffer with non-0xFF                  */

            /* DMA read SPIM flash                                                        */
            SPIM_DMA_Read(SPIM0, addr, IS_4BYTES_ADDR, BUFF_SIZE, Buff1, sWb03hRdCMD.u32CMDCode, 1);

            if (memcmp(Buff1, Buff2, BUFF_SIZE) != 0)
            {
                printf("Verify address 0x%x failed!\n", addr);
                f_close(&file);             /* close file                                 */
                return -1;                  /* non-0xFF data found, erase failed          */
            }
        }

        printf("OK.\n");

        printf("Program page and verify...");

        for (addr = page_addr; addr < page_addr + SPIM_FLASH_PAGE_SIZE; addr += BUFF_SIZE)
        {
            res = f_read(&file, Buff1, BUFF_SIZE, &len);

            if (res || (len == 0))
            {
                printf("OK [%d]\n", res);
                f_close(&file);             /* close file                                 */
                return 0;                   /* done                                       */
            }

            /* DMA write SPIM flash                       */
            SPIM_DMA_Write(SPIM0, addr, IS_4BYTES_ADDR, BUFF_SIZE, Buff1, sWb02hWrCMD.u32CMDCode);

            memset(Buff2, 0, BUFF_SIZE);
            /* DMA read SPIM flash                        */
            SPIM_DMA_Read(SPIM0, addr, IS_4BYTES_ADDR, BUFF_SIZE, Buff2, sWb03hRdCMD.u32CMDCode, 1);

            if (memcmp(Buff1, Buff2, BUFF_SIZE) != 0)
            {
                printf("Failed at address 0x%x!\n", addr);
                f_close(&file);             /* close file                                 */
                return -1;
            }

            printf(".");
        }

        printf("OK\n");
    }

    return 0;
}

/*
 *  c <file> <addr>
 */
int  compare_file_with_flash(char *cmdline)
{
    char        *ptr = cmdline;             /* command string pointer                     */
    char        fname[64];                  /* file name string pointer                   */
    uint32_t    faddr, page_addr, addr;     /* flash address                              */
    UINT        i, len;                     /* data length                                */
    FRESULT     res;                        /* FATFS operation return code                */

    while (*ptr == ' ') ptr++;              /* skip space characters                      */

    for (len = 0; len < 60; len++)
    {
        if ((*ptr == ' ') || (*ptr == 0))
            break;

        fname[len] = *ptr++;
    }

    fname[len] = 0;

    if (!xatoi(&ptr, &faddr))               /* get <addr> parameter (SPIM flash address)  */
    {
        printf("Usage:  c <file> <addr>\n");
        return -1;
    }

    faddr = faddr - (faddr % SPIM_FLASH_PAGE_SIZE);  /* force page alignment              */

    printf("Compare file [%s] with SPIM address 0x%x...\n", fname, faddr);
    res = f_open(&file, fname, FA_OPEN_EXISTING | FA_READ);   /* Open file                */

    if (res)
    {
        put_rc(res);                        /* Open failed, print error message           */
        return -1;                          /* Abort...                                   */
    }

    SPIM_ReadJedecId(SPIM0, idBuf, sizeof(idBuf), 1, 0);
    printf("Flash ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    //SPIM_Enable_4Bytes_Mode(SPIM0, IS_4BYTES_ADDR, 1);  /* Enable 4-bytes address mode?          */

    /*
     *  Compare ...
     */
    for (page_addr = faddr; page_addr < SPIM_FLASH_MAX_SIZE; page_addr += SPIM_FLASH_PAGE_SIZE)
    {
        printf("Comparing...");

        for (addr = page_addr; addr < page_addr + SPIM_FLASH_PAGE_SIZE; addr += BUFF_SIZE)
        {
            memset(Buff1, 0xff, BUFF_SIZE); /* fill 0xff to clear buffer                  */
            memset(Buff2, 0, BUFF_SIZE);    /* fill 0x00 to clear buffer                  */

            res = f_read(&file, Buff1, BUFF_SIZE, &len);

            if (res || (len == 0))
            {
                printf("Compare OK.\n");
                f_close(&file);             /* close file                                 */
                return 0;                   /* done                                       */
            }

            /* DMA read SPIM flash                        */
            SPIM_DMA_Read(SPIM0, addr, IS_4BYTES_ADDR, BUFF_SIZE, Buff2, sWb03hRdCMD.u32CMDCode, 1);

            if (memcmp(Buff1, Buff2, len) != 0)
            {
                for (i = 0; i < len; i++)
                    printf("0x%04x: 0x%02x  0x%02x\n", addr + i, Buff1[i], Buff2[i]);

                printf("Compare failed!\n");
                f_close(&file);             /* close file                                 */
                return -1;
            }

            printf(".");
        }

        printf("OK\n");
    }

    return 0;
}

/*
 *  d <addr> <len>
 */
int  dump_spim_flash(char *cmdline)
{
    char        *ptr = cmdline;             /* command string pointer                     */
    uint32_t    faddr, addr;                /* flash address                              */
    uint32_t    dump_len;                   /* dump length                                */
    UINT        len;                        /* data length                                */

    if (!xatoi(&ptr, &faddr))               /* get <addr> parameter (SPIM flash address)  */
    {
        printf("Usage:  d <addr> <len>\n");
        return -1;
    }

    faddr = faddr - (faddr % BUFF_SIZE);    /* force block alignment                      */

    if (!xatoi(&ptr, &dump_len))            /* get <len> parameter (data dump length)     */
    {
        printf("Usage:  d <addr> <len>\n");
        return -1;
    }

    SPIM_ReadJedecId(SPIM0, idBuf, sizeof(idBuf), 1, 0);
    printf("Flash ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    //SPIM_Enable_4Bytes_Mode(SPIM0, IS_4BYTES_ADDR, 1);  /* Enable 4-bytes address mode?          */

    /*
     *  Read and dump ...
     */
    for (addr = faddr; dump_len > 0; addr += BUFF_SIZE)
    {
        memset(Buff1, 0, BUFF_SIZE);        /* fill 0x00 to clear buffer                  */
        /* DMA read SPIM flash                        */
        SPIM_DMA_Read(SPIM0, addr, IS_4BYTES_ADDR, BUFF_SIZE, Buff1, sWb03hRdCMD.u32CMDCode, 1);

        if (dump_len < BUFF_SIZE)
            len = dump_len;
        else
            len = BUFF_SIZE;

        dump_buff_hex(addr, Buff1, len);    /* dump data                                  */
        dump_len -= len;
    }

    return 0;
}

/*
 *  g <addr>
 *
 *  Example:
 *  Assumed the image located on SPIM flash offset address 0x2000. It's RO_BASE must
 *  be (SPIM_DMM_MAP_ADDR+0x2000). To branch and run this image, user must issue
 *  command "g 0x2000".
 */
int  go_to_flash(char *cmdline)
{
    char        *ptr = cmdline;             /* command string pointer                     */
    uint32_t    faddr;                      /* flash address                              */
    FUNC_PTR    *func;                      /* function pointer                           */

    if (!xatoi(&ptr, &faddr))               /* get <addr> parameter (SPIM flash address)  */
    {
        printf("Usage:  g <addr>\n");
        return -1;
    }

    SPIM_ReadJedecId(SPIM0, idBuf, sizeof(idBuf), 1, 0);
    printf("Flash ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);
    //TestOnly
#ifndef TESTCHIP_ONLY
    SPIM_ENABLE_CACHE(SPIM0);
#endif
    //SPIM_Enable_4Bytes_Mode(SPIM0, IS_4BYTES_ADDR, 1);  /* Enable 4-bytes address mode?          */
#ifndef TESTCHIP_ONLY
    SPIM0->CTL1 |= SPIM_CTL1_CDINVAL_Msk;    /* invalid cache                              */
#endif
    /* Enable DMM mode                            */
    SPIM_EnterDirectMapMode(SPIM0, IS_4BYTES_ADDR, sWb03hRdCMD.u32CMDCode, 1);

    func = (FUNC_PTR *)(SPIM_DMM0_SADDR + faddr + 1);

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;   /* disable SYSTICK (prevent interrupt)   */

    //#ifdef __GNUC__                        /* for GNU C compiler */
    //    __ASM volatile("msr msp, r0");
    //    __ASM volatile("bx  lr");
    //#else
    //    __set_SP(inpw(SPIM_DMM0_SADDR + faddr));
    //#endif

    func();                                      /* branch to SPIM                        */

    return 0;
}

int SPIM_TrimRXClkDlyNum(SPIM_T *pSPIMx, SPIM_PHASE_T *psWbRdCMD)
{
    volatile uint8_t u8RdDelay = 0;
    uint8_t u8RdDelayIdx = 0;
    uint8_t u8RdDelayRes[0x0F] = {0};
    uint32_t u32SAddr = 0x0;
    volatile uint32_t u32i = 0;
    uint32_t u32Div = SPIM_GET_CLOCK_DIVIDER(pSPIMx);
    uint8_t au8TrimPatten[32] =
    {
        0xff, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
        0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
        0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
        0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
    };
    uint8_t au8CmpBuf[32] = {0};
#ifdef DMM_MODE_TRIM
    uint32_t u32RdDataCnt = 0;
    uint32_t u32DMMAddr = SPIM_GetDMMAddress(pSPIMx);
    uint32_t *pu32RdData = NULL;
#endif //

    SPIM_SET_CLOCK_DIVIDER(pSPIMx, 8);

    SPIM_EraseBlock(pSPIMx, u32SAddr, SPIM_OP_DISABLE, OPCODE_BE_64K, 1, SPIM_OP_ENABLE);

    SPIM_DMA_Write(pSPIMx, u32SAddr, SPIM_OP_DISABLE, sizeof(au8TrimPatten), au8TrimPatten, sWb02hWrCMD.u32CMDCode);

    SPIM_SET_CLOCK_DIVIDER(pSPIMx, u32Div);

#ifdef DMM_MODE_TRIM
    SPIM_DMM_ReadPhase(pSPIMx, psWbRdCMD, 1);
#endif

    for (u8RdDelay = 0; u8RdDelay <= 0xF; u8RdDelay++)
    {
        SPIM_SET_RXCLKDLY_RDDLYSEL(pSPIMx, u8RdDelay);

        memset(au8CmpBuf, 0, sizeof(au8TrimPatten));

#ifndef DMM_MODE_TRIM
        SPIM_DMA_Read(pSPIMx, u32SAddr, SPIM_OP_DISABLE,  sizeof(au8TrimPatten), au8CmpBuf, sWb03hRdCMD.u32CMDCode, SPIM_OP_ENABLE);
#else
        u32RdDataCnt = 0;
        pu32RdData = (uint32_t *)tstbuf2;

        //SPIM_IO_ReadPhase(pSPIMx, pMT0BhRdCMD, u32SAddr, tstbuf2, sizeof(au8TrimPatten));

        for (u32i = u32SAddr; u32i < (u32SAddr + sizeof(au8TrimPatten)); u32i += 4)
        {
            pu32RdData[u32RdDataCnt++] = inpw(u32DMMAddr + u32i);
        }

#endif

        // Compare.
        if (memcmp(au8TrimPatten, au8CmpBuf, sizeof(au8TrimPatten)) == 0)
        {
            printf("RX Delay: %d = Pass\r\n", u8RdDelay);
            u8RdDelayRes[u8RdDelayIdx++] = u8RdDelay;
        }
    }

    if (u8RdDelayIdx >= 2)
    {
        u8RdDelayIdx = (u8RdDelayIdx / 2) /*- 1*/;
    }
    else
    {
        u8RdDelayIdx = 0;
    }

    printf("\r\nRX Delay = %d\r\n\r\n", u8RdDelayRes[u8RdDelayIdx]);
    SPIM_SET_RXCLKDLY_RDDLYSEL(pSPIMx, u8RdDelayRes[u8RdDelayIdx]);

    return u8RdDelay;
}

int32_t main(void)
{
    char        *ptr;                       /* str pointer                                */

    SYS_Init();                             /* Init System, clock and I/O pins.           */

    InitDebugUart();                        /* Init DeubgUART for printf */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+-----------------------------------------------+\n");
    printf("|   USB SPIM writer                             |\n");
    printf("+-----------------------------------------------+\n");

    SYS_UnlockReg();                             /* Unlock register lock protect               */

    SPIM_SET_CLOCK_DIVIDER(SPIM0, 1);            /* Set SPIM clock as HCLK divided by 4        */

    SPIM_DISABLE_CIPHER(SPIM0);

    SPIM_DISABLE_CACHE(SPIM0);

    if (SPIM_InitFlash(SPIM0, SPIM_OP_ENABLE) != 0)            /* Initialized SPI flash                      */
    {
        printf("SPIM flash initialize failed!\n");

        while (1);
    }

    SPIM_ReadJedecId(SPIM0, idBuf, sizeof(idBuf), 1, 0);   /* read SPIM flash ID                   */
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    usbh_core_init();                       /* initialize USB Host library                */
    usbh_umas_init();                       /* initialize USB mass storage driver         */
    usbh_pooling_hubs();                    /* monitor USB hub ports                      */

    f_chdrive(usbh_path);                   /* set default path                           */

    SPIM_DMADMM_InitPhase(SPIM0, &sWb03hRdCMD, SPIM_CTL0_OPMODE_PAGEREAD);
    SPIM_DMADMM_InitPhase(SPIM0, &sWb02hWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);
    SPIM_DMADMM_InitPhase(SPIM0, &sWb03hRdCMD, SPIM_CTL0_OPMODE_DIRECTMAP);

    /* Trim RX clock delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_TrimRXClkDlyNum(SPIM0, &sWb03hRdCMD);

    /*
     *  Erase flash page
     */
    printf("Erase SPI flash block 0x%x...", 0);
    SPIM_EraseBlock(SPIM0, 0, 0, OPCODE_BE_64K, 1, 1);
    printf("done.\n");

    for (;;)
    {
        usbh_pooling_hubs();

        printf(_T(">"));
        ptr = Line;

        get_line(ptr, sizeof(Line));

        switch (*ptr++)
        {

            case 'h':
            case '?':                           /* Show usage                                 */
                printf(
                    _T("l - List root directory files\n")
                    _T("w <file> <addr> - Write a file to SPIM flash. <addr> is an offset from SPIM flash base.\n")
                    _T("c <file> <addr> - Compare a file with SPIM flash. <addr> is an offset from SPIM flash base.\n")
                    _T("d <addr> <len> - Dump SPIM offset <addr> contents.\n")
                    _T("g <addr> - Branch to SPIM offset <addr> and execute.\n")
                );
                break;

            case 'l' :                          /* List root directory files                  */
                show_root_dir();
                break;

            case 'w' :                          /* w <file> <addr>                            */
                write_file_to_flash(ptr);       /* Write a file to SPIM flash.                */
                break;

            case 'c' :                          /* c <file> <addr>                            */
                compare_file_with_flash(ptr);   /* Compare a file with SPIM flash block.      */
                break;

            case 'd' :                          /* d <addr> <len>                             */
                dump_spim_flash(ptr);           /* Dump contents of a SPIM flash block.       */
                break;

            case 'g' :                          /* g <addr>                                   */
                go_to_flash(ptr);               /* Branch to SPIM flash offset <addr>         */
                break;
        }
    }
}
