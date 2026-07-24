/**************************************************************************//**
 * @file     ohci.h
 * @version  V1.00
 * @brief    USB OHCI host controller driver header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef _USBH_OHCI_H_
#define _USBH_OHCI_H_

/// @cond HIDDEN_SYMBOLS

struct utr_t;
struct udev_t;

/* OHCI CONTROL AND STATUS REGISTER MASKS */

/*
 * Host controller functional state.
 * for HCFS(HcControl[7:6])
 */
#define HCFS_RESET                (0UL << USBH_HcControl_HCFS_Pos)
#define HCFS_RESUME               (1UL << USBH_HcControl_HCFS_Pos)
#define HCFS_OPER                 (2UL << USBH_HcControl_HCFS_Pos)
#define HCFS_SUSPEND              (3UL << USBH_HcControl_HCFS_Pos)

/*----------------------------------------------------------------------------------------*/
/*   Endpoint descriptor                                                                  */
/*----------------------------------------------------------------------------------------*/
typedef struct ed_t
{
    /* OHCI spec. Endpoint descriptor  */
    uint32_t    Info;
    uint32_t    TailP;
    uint32_t    HeadP;
    uint32_t    NextED;
    /* The following members are used by USB Host libary.   */
    uint8_t     bInterval;
    uint16_t    next_sf;          /* for isochronous transfer, recording the next SF      */
    struct ed_t *next;            /* point to the next ED in remove list                  */
} ED_T;

#define ED_CTRL_FA_Pos            0U        /* Info[6:0]   - Function address             */
#define ED_CTRL_EN_Pos            7U        /* Info[10:7]  - Endpoint number              */
#define ED_CTRL_DIR_Pos           11U       /* Info[12:11] - Direction                    */
#define ED_CTRL_MPS_Pos           16U       /* Info[26:16] - Maximum packet size          */

#define ED_FUNC_ADDR_Msk          (0x7fU)
#define ED_EP_ADDR_Msk            ((uint32_t)0xfU<<7U)
#define ED_DIR_Msk                ((uint32_t)0x3U<<11U)
#define ED_SPEED_Msk              ((uint32_t)1U<<13U)
#define ED_MAX_PK_SIZE_Msk        ((uint32_t)0x7ffU<<16U)

#define ED_DIR_BY_TD              ((uint32_t)0U<<ED_CTRL_DIR_Pos)
#define ED_DIR_OUT                ((uint32_t)1U<<ED_CTRL_DIR_Pos)
#define ED_DIR_IN                 ((uint32_t)2U<<ED_CTRL_DIR_Pos)
#define ED_SPEED_FULL             ((uint32_t)0U<<13U)  /* Info[13] - 0: is full speed device         */
#define ED_SPEED_LOW              ((uint32_t)1U<<13U)  /* Info[13] - 1: is low speed device          */
#define ED_SKIP                   ((uint32_t)1U<<14U)  /* Info[14] - 1: HC skip this ED              */
#define ED_FORMAT_GENERAL         ((uint32_t)0U<<15U)  /* Info[15] - 0: is a general TD              */
#define ED_FORMAT_ISO             ((uint32_t)1U<<15U)  /* Info[15] - 1: is an isochronous TD         */
#define ED_HEADP_HALT             ((uint32_t)1U<<0U)   /* HeadP[0] - 1: Halt; 0: Not                 */

/*----------------------------------------------------------------------------------------*/
/*   Transfer descriptor                                                                  */
/*----------------------------------------------------------------------------------------*/
/* general transfer descriptor  */
typedef struct td_t
{
    uint32_t    Info;
    uint32_t    CBP;                        /* Current Buffer Pointer                     */
    uint32_t    NextTD;                     /* Next TD                                    */
    uint32_t    BE;                         /* Buffer End                                 */
    uint32_t    PSW[4];                     /* PSW 0~7                                    */
    /* The following members are used by USB Host libary.   */
    uint32_t    buff_start;                 /* Buffer Start                               */
    ED_T        *ed;                        /* The ED that this TD belong to.             */
    struct utr_t  *utr;                     /* associated UTR                             */
    struct td_t *next;                      /* point to next TD of the same UTR           */
} TD_T;

#define TD_ADDR_MASK              0xFFFFFFFCU

/* Completion codes */
enum OCHI_CC_CODE
{
    /* mapping of the OHCI CC status to error codes */
    CC_NOERROR,                             /* No  Error                                  */
    CC_CRC,                                 /* CRC Error                                  */
    CC_BITSTUFF,                            /* Bit Stuff                                  */
    CC_DATA_TOGGLE,                         /* Data Toggle                                */
    CC_STALL,                               /* Stall                                      */
    CC_NOTRESPONSE,                         /* DevNotResp                                 */
    CC_PID_CHECK,                           /* PIDCheck                                   */
    CC_UNEXPECTED_PID,                      /* UnExpPID                                   */
    CC_DATA_OVERRUN,                        /* DataOver                                   */
    CC_DATA_UNDERRUN,                       /* DataUnder                                  */
    CC_RESERVED1,                           /* reserved                                   */
    CC_RESERVED2,                           /* reserved                                   */
    CC_BUFFER_OVERRUN,                      /* BufferOver                                 */
    CC_BUFFER_UNDERRUN,                     /* BuffUnder                                  */
    CC_NOT_ACCESS                           /* Not Access                                 */
};

/* TD control field */
#define TD_CC                     0xF0000000U
#define TD_CC_GET(td)             (((td) >> 28) & 0x0FU)
#define TD_CC_SET(td, cc)         (td) = ((td) & 0x0FFFFFFFU) | (((uint32_t)(cc) & 0x0FU) << 28U)
#define TD_T_DATA0                0x02000000U
#define TD_T_DATA1                0x03000000U
#define TD_R                      0x00040000U
#define TD_DP                     0x00180000U
#define TD_DP_IN                  0x00100000U
#define TD_DP_OUT                 0x00080000U
#define MAXPSW                    8
/* steel TD reserved bits to keep driver data */
#define TD_TYPE_Msk               ((uint32_t)0x3U<<16U)
#define TD_TYPE_CTRL              ((uint32_t)0x0U<<16U)
#define TD_TYPE_BULK              ((uint32_t)0x1U<<16U)
#define TD_TYPE_INT               ((uint32_t)0x2U<<16U)
#define TD_TYPE_ISO               ((uint32_t)0x3U<<16U)
#define TD_CTRL_Msk               ((uint32_t)0x7U<<15U)
#define TD_CTRL_DATA              ((uint32_t)1U<<15U)

/*
 * The HCCA (Host Controller Communications Area) is a 256 byte
 * structure defined in the OHCI spec. that the host controller is
 * told the base address of.  It must be 256-byte aligned.
 */
typedef struct
{
    uint32_t   int_table[32];               /* Interrupt ED table                         */
    uint16_t   frame_no;                    /* current frame number                       */
    uint16_t   pad1;                        /* set to 0 on each frame_no change           */
    uint32_t   done_head;                   /* info returned for an interrupt             */
    uint8_t    reserved_for_hc[116];
} HCCA_T;

/// @endcond

#endif  /* _USBH_OHCI_H_ */
