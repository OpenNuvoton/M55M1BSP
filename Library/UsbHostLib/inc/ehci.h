/**************************************************************************//**
 * @file     ehci.h
 * @version  V1.00
 * @brief    USB EHCI host controller driver header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef _USBH_EHCI_H_
#define _USBH_EHCI_H_

/// @cond HIDDEN_SYMBOLS

struct utr_t;
struct udev_t;
struct qh_t;
struct iso_ep_t;
struct ep_info_t;


/*----------------------------------------------------------------------------------------*/
/*  Periodic Frame List Size (256, 512, or 1024)                                          */
/*----------------------------------------------------------------------------------------*/
#define FL_SIZE              1024U           /* frame list size can be 256, 512, or 1024   */
#define NUM_IQH              11U             /* depends on FL_SIZE, 256:9, 512:10, 1024:11 */

/*----------------------------------------------------------------------------------------*/
/*  Interrupt Threshold Control (1, 2, 4, 6, .. 64)                                       */
/*----------------------------------------------------------------------------------------*/
#define UCMDR_INT_THR_CTRL     (0x1UL << HSUSBH_UCMDR_ITC_Pos)     /* 1 micro-frames          */

/*----------------------------------------------------------------------------------------*/
/*  Queue Element Transfer Descriptor (qTD)                                               */
/*----------------------------------------------------------------------------------------*/
typedef struct qTD_t
{
    uint32_t      Next_qTD;                 /* Next qTD Pointer                           */
    uint32_t      Alt_Next_qTD;             /* Alternate Next qTD Pointer                 */
    uint32_t      Token;                    /* qTD Token                                  */
    uint32_t      Bptr[5];                  /* qTD Buffer Page Pointer List               */
    /*
     * The following members are used by USB Host libary.
     */
    struct utr_t  *utr;                     /* associated UTR                             */
    uint32_t      xfer_len;                 /* assigned transfer transfer length          */
    struct qh_t   *qh;                      /* The QH that this qTD belong to.            */
    struct qTD_t  *next;                    /* link for <qtd_list> of QH                  */
}  qTD_T;

#define QTD_LIST_END              0x1U      /* Indicate the terminate of qTD list.        */
#define QTD_PTR(x)                ((qTD_T *)((uint32_t)(x) & ~0x1FU))

/*
 *  Status: qTD Token[7:0]
 */
#define QTD_STS_PS_OUT            (0U << 0)    /* directs the HC to issue an OUT PID         */
#define QTD_STS_PS_PING           (1U << 0)    /* directs the HC to issue an PING PID        */
#define QTD_STS_SPLIT_STRAT       (0U << 1)    /* directs the HC to issue an Start split     */
#define QTD_STS_SPLIT_COMPLETE    (1U << 1)    /* directs the HC to issue an Complete split  */
#define QTD_STS_MISS_MF           (1U << 2)    /* miss a required complete-split transaction */
#define QTD_STS_XactErr           (1U << 3)    /* Transaction Error occurred                 */
#define QTD_STS_BABBLE            (1U << 4)    /* Babble Detected                            */
#define QTD_STS_DATA_BUFF_ERR     (1U << 5)    /* Data Buffer Error                          */
#define QTD_STS_HALT              (1U << 6)    /* Halted                                     */
#define QTD_STS_ACTIVE            (1U << 7)    /* Active                                     */

/*
 *  PID: qTD Token[9:8]
 */
#define QTD_PID_Msk              (0x3UL << 8)
#define QTD_PID_OUT               (0UL << 8)   /* generates token (E1H)                      */
#define QTD_PID_IN                (1UL << 8)   /* generates token (69H)                      */
#define QTD_PID_SETUP             (2UL << 8)   /* generates token (2DH)                      */

#define QTD_ERR_COUNTER           (3UL << 10)  /* Token[11:10]                               */
#define QTD_IOC                   (1UL << 15)  /* Token[15] - Interrupt On Complete          */
#define QTD_TODO_LEN_Pos          16U        /* Token[31:16] - Total Bytes to Transfer     */
#define QTD_TODO_LEN(x)           ((((uint32_t)(x)) >> QTD_TODO_LEN_Pos) & 0x7FFFU)
#define QTD_DT                    (1UL << 31) /* Token[31] - Data Toggle                    */

/*----------------------------------------------------------------------------------------*/
/*  Queue Head (QH)                                                                       */
/*----------------------------------------------------------------------------------------*/
typedef struct qh_t
{
    /* OHCI spec. Endpoint descriptor  */
    uint32_t    HLink;                      /* Queue Head Horizontal Link Pointer         */
    uint32_t    Chrst;                      /* Endpoint Characteristics: QH DWord 1       */
    uint32_t    Cap;                        /* Endpoint Capabilities: QH DWord 2          */
    uint32_t    Curr_qTD;                   /* Current qTD Pointer                        */
    /*
     * The followings are qTD Transfer Overlay
     */
    uint32_t    OL_Next_qTD;                /* Next qTD Pointer                           */
    uint32_t    OL_Alt_Next_qTD;            /* Alternate Next qTD Pointer                 */
    uint32_t    OL_Token;                   /* qTD Token                                  */
    uint32_t    OL_Bptr[5];                 /* qTD Buffer Page Pointer List               */
    /*
     * The following members are used by USB Host libary.
     */
    qTD_T       *dummy;                     /* point to the inactive dummy qTD            */
    qTD_T       *qtd_list;                  /* currently linked qTD transfers             */
    qTD_T       *done_list;                 /* currently linked qTD transfers             */
    struct qh_t *next;                      /* point to the next QH in remove list        */
}  QH_T;

/*  HLink[0] T field of "Queue Head Horizontal Link Pointer" */
#define QH_HLNK_END               0x1U

/*
 *  HLink[2:1] Typ field of "Queue Head Horizontal Link Pointer"
 */
#define QH_HLNK_ITD(x)            ((((uint32_t)(x)) & ~0x1FU) | 0x0U)
#define QH_HLNK_QH(x)             ((((uint32_t)(x)) & ~0x1FU) | 0x2U)
#define QH_HLNK_SITD(x)           ((((uint32_t)(x)) & ~0x1FU) | 0x4U)
#define QH_HLNK_FSTN(x)           ((((uint32_t)(x)) & ~0x1FU) | 0x6U)
#define QH_PTR(x)                 ((QH_T *)(((uint32_t)(x)) & ~0x1FU))

/*
 *  Bit fields of "Endpoint Characteristics"
 */
#define QH_NAK_RL                 (4UL << 28U)  /* Chrst[31:28] - NAK Count Reload            */
#define QH_CTRL_EP_FLAG           (1UL << 27U)  /* Chrst[27] - Control Endpoint Flag          */
#define QH_RCLM_LIST_HEAD         (1UL << 15U)  /* Chrst[15] - Head of Reclamation List Flag  */
#define QH_DTC                    (1UL << 14U)  /* Chrst[14] - Data Toggle Control            */
#define QH_EPS_FULL               (0UL << 12U)  /* Chrst[13:12] - Endpoint Speed (Full)       */
#define QH_EPS_LOW                (1UL << 12U)  /* Chrst[13:12] - Endpoint Speed (Low)        */
#define QH_EPS_HIGH               (2UL << 12U)  /* Chrst[13:12] - Endpoint Speed (High)       */
#define QH_I_NEXT                 (1UL << 7U)   /* Chrst[7] - Inactivate on Next Transaction  */

/*
 *  Bit fields of "Endpoint Capabilities"
 */
#define QH_MULT_Pos               30U       /* Cap[31:30] - High-Bandwidth Pipe Multiplier */
#define QH_HUB_PORT_Pos           23U       /* Cap[29:23] - Hub Port Number               */
#define QH_HUB_ADDR_Pos           16U       /* Cap[22:16] - Hub Addr                      */
#define QH_C_MASK_Msk             0xFF00U   /* Cap[15:8]  - uFrame C-mask                 */
#define QH_S_MASK_Msk             0x00FFU   /* Cap[7:0]   - uFrame S-mask                 */

/*----------------------------------------------------------------------------------------*/
/*  Isochronous (High-Speed) Transfer Descriptor (iTD)                                    */
/*----------------------------------------------------------------------------------------*/
typedef struct itd_t
{
    uint32_t      Next_Link;                /* Next Link Pointer                          */
    uint32_t      Transaction[8];           /* Transaction Status and Control             */
    uint32_t      Bptr[7];                  /* Buffer Page Pointer List                   */
    /*
     * The following members are used by USB Host libary.
     */
    struct iso_ep_t *iso_ep;                /* associated isochronous information block   */
    struct utr_t  *utr;                     /* associated UTR                             */
    uint32_t      buff_base;                /* buffer base address                        */
    uint8_t       fidx;                     /* iTD's first index to UTR iso frames        */
    uint8_t       trans_mask;               /* mask of activated transactions in iTD      */
    uint32_t      sched_frnidx;             /* scheduled frame index                      */
    struct itd_t  *next;                    /* used by software to maintain iTD list      */
}  iTD_T;

/*
 *  Next_Link[2:1] Typ field of "Next Schedule Element Pointer"  Typ field
 */
#define ITD_HLNK_ITD(x)           (((uint32_t)(x) & ~0x1FU) | 0x0U)
#define ITD_HLNK_QH(x)            (((uint32_t)(x) & ~0x1FU) | 0x2U)
#define ITD_HLNK_SITD(x)          (((uint32_t)(x) & ~0x1FU) | 0x4U)
#define ITD_HLNK_FSTN(x)          (((uint32_t)(x) & ~0x1FU) | 0x6U)
#define ITD_PTR(x)                ((iTD_T *)((uint32_t)(x) & ~0x1FU))

/*
 *  Transaction[8]
 */
#define ITD_STATUS(x)             ((((uint32_t)(x))>>28)&0xFU)
#define ITD_STATUS_ACTIVE         (0x80000000UL)      /* Active                           */
#define ITD_STATUS_BUFF_ERR       (0x40000000UL)      /* Data Buffer Error                */
#define ITD_STATUS_BABBLE         (0x20000000UL)      /* Babble Detected                  */
#define ITD_STATUS_XACT_ERR       (0x10000000UL)      /* Transcation Error                */

#define ITD_XLEN_Pos              16U
#define ITD_XFER_LEN(x)           ((((uint32_t)(x))>>16)&0xFFFU)
#define ITD_IOC                   (1UL<<15)
#define ITD_PG_Pos                12U
#define ITD_XFER_OFF_Msk          0xFFFU

/*
 *  Bptr[7]
 */
#define ITD_BUFF_PAGE_Pos         12U
/* Bptr[0] */
#define ITD_EP_NUM_Pos            8U
#define ITD_EP_NUM(itd)           (((itd)->Bptr[0]>>8)&0xFU)
#define ITD_DEV_ADDR_Pos          0U
#define ITD_DEV_ADDR(itd)         ((itd)->Bptr[0]&0x7FU)
/* Bptr[1] */
#define ITD_DIR_IN                (1UL<<11)
#define ITD_DIR_OUT               (0UL<<11)
#define ITD_MAX_PKTSZ_Pos         0U
#define ITD_MAX_PKTSZ(itd)        ((itd)->Bptr[1]&0x7FFU)

/*----------------------------------------------------------------------------------------*/
/*  Split Isochronous (Full-Speed) Transfer Descriptor (siTD)                             */
/*----------------------------------------------------------------------------------------*/
typedef struct sitd_t
{
    uint32_t      Next_Link;                /* Next Link Pointer                          */
    uint32_t      Chrst;                    /* Endpoint and Transaction Translator Characteristics */
    uint32_t      Sched;                    /* Micro-frame Schedule Control               */
    uint32_t      StsCtrl;                  /* siTD Transfer Status and Control           */
    uint32_t      Bptr[2];                  /* Buffer Page Pointer List                   */
    uint32_t      BackLink;                 /* siTD Back Link Pointer                     */
    /*
     * The following members are used by USB Host libary.
     */
    struct iso_ep_t *iso_ep;                /* associated isochronous information block   */
    struct utr_t  *utr;                     /* associated UTR                             */
    uint8_t       fidx;                     /* iTD's first index to UTR iso frames        */
    uint32_t      sched_frnidx;             /* scheduled frame index                      */
    struct sitd_t *next;                    /* used by software to maintain siTD list     */
}  siTD_T;

#define SITD_LIST_END              0x1U     /* Indicate the terminate of siTD list.       */

#define SITD_XFER_IO_Msk           (1UL<<31)
#define SITD_XFER_IN               (1UL<<31)
#define SITD_XFER_OUT              (0UL<<31)

#define SITD_PORT_NUM_Pos          24U
#define SITD_HUB_ADDR_Pos          16U
#define SITD_EP_NUM_Pos            8U
#define SITD_DEV_ADDR_Pos          0U

#define SITD_IOC                   (1UL<<31)
#define SITD_XFER_CNT_Pos          16U
#define SITD_XFER_CNT_Msk          (0x3FFUL<<SITD_XFER_CNT_Pos)

#define SITD_STATUS(x)             (((uint32_t)(x))&0xFCU)
#define SITD_STATUS_ACTIVE         0x80U
#define SITD_STATUS_ERR            0x40U
#define SITD_STATUS_BUFF_ERR       0x20U
#define SITD_BABBLE_DETECTED       0x10U
#define SITD_STATUS_XFER_ERR       0x08U
#define SITD_STATUS_MISSED_MF      0x04U
#define SITD_STATUS_ERROR_MASK     0x78U

/*
 *  Next_Link[2:1] Typ field of "Next Schedule Element Pointer"  Typ field
 */
#define SITD_HLNK_ITD(x)          (((uint32_t)(x) & ~0x1FU) | 0x0U)
#define SITD_HLNK_QH(x)           (((uint32_t)(x) & ~0x1FU) | 0x2U)
#define SITD_HLNK_SITD(x)         (((uint32_t)(x) & ~0x1FU) | 0x4U)
#define SITD_HLNK_FSTN(x)         (((uint32_t)(x) & ~0x1FU) | 0x6U)
#define SITD_PTR(x)               ((siTD_T *)((uint32_t)(x) & ~0x1FU))

#define HLINK_IS_TERMINATED(x)    ((((uint32_t)(x) & 0x1U) != 0U) ? 1 : 0)
#define HLINK_IS_SITD(x)          ((((uint32_t)(x) & 0x6U) == 0x4U) ? 1 : 0)

/*----------------------------------------------------------------------------------------*/
/*  Isochronous endpoint transfer information block. (Software only)                      */
/*----------------------------------------------------------------------------------------*/
typedef struct iso_ep_t
{
    struct ep_info_t  *ep;
    uint32_t      next_frame;               /* frame number of next scheduling            */
    iTD_T         *itd_list;                /* Reference to a list of installed iTDs      */
    iTD_T         *itd_done_list;           /* Reference to a list of completed iTDs      */
    siTD_T        *sitd_list;               /* Reference to a list of installed siTDs     */
    siTD_T        *sitd_done_list;          /* Reference to a list of completed siTDs     */
    struct iso_ep_t  *next;                 /* used by software to maintain ISO EP list   */
} ISO_EP_T;

extern void scan_isochronous_list(void);

extern uint32_t _PFList[FL_SIZE];           /* Periodic frame list                        */
extern ISO_EP_T  *iso_ep_list;              /* list of activated isochronous pipes        */


/// @endcond

#endif  /* _USBH_EHCI_H_ */
