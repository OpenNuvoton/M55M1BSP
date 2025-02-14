/**************************************************************************//**
 * @file     utcpdlib.h
 * @version  V3.00
 * @brief    Power Delivery library header file
 * @note
 *
 * Copyright (c) 2022 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UTCPDLIB_H__
#define __UTCPDLIB_H__

#include <string.h>
#include "NuMicro.h"
#include "utcpd_config.h"
#include "atomic.h"
#include "usb_pd.h"
#include "tcpci.h"
#include "usb_common.h"
#include "ec_timer.h"
#include "usb_sm.h"
#include "usb_pd_flags.h"
#include "nct38xx.h"
#include "usb_pe_sm.h"
#include "usb_emsg.h"
#include "usb_pd_timer.h"
#include "usb_tc_sm.h"
#include "usbc_ppc.h"
#include "usb_pd_tcpm.h"
#include "usb_prl_sm.h"
#include "usb_pd_dpm.h"
//New Add by cyhsu11
#include "usb_dp_alt_mode.h"
#include "task_event.h"
#include "usb_pd_policy.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup UTCPDLIB Power Delivery Library
  @{
*/

/** @addtogroup UTCPDLIB_EXPORTED_CONSTANTS Power Delivery Library Exported Constants
  @{
*/

#define CONFIG_USB_PID                           0x8260

typedef enum
{
    UTCPD_PD_ATTACHED = 0,          /* Port partner attached or disattached */
    UTCPD_PD_CONTRACT = 1,          /* PD contract established? */
    UTCPD_PD_SNK_VOLTAGE = 2,       /* Contract voltage */
} E_UTCPD_PD_EVENT;

/*@}*/ /* end of group UTCPDLIB_EXPORTED_CONSTANTS */



/** @addtogroup UTCPDLIB_EXPORTED_STRUCTS Power Delivery Library Exported Structs
  @{
*/
typedef void (*utcpd_pvFunPtr)(int port, E_UTCPD_PD_EVENT event, uint32_t op);   /* function pointer declaration */

/*@}*/ /* end of group UTCPDLIB_EXPORTED_STRUCTS */



/** @addtogroup SCLIB_EXPORTED_FUNCTIONS Smartcard Library Exported Functions
  @{
*/
void UTCPD_InstallCallback(int port, utcpd_pvFunPtr *pfn);
void UTCPD_NotifyEvent(int port, uint32_t event, uint32_t op);

enum pd_cc_states UTCPD_TC_get_cc_state(int port);
enum tcpc_cc_polarity UTCPD_TC_get_polarity(int port);
void UTCPD_PE_get_src_caps(int port, int32_t *pu32SrcArray, int32_t *pi32SrcCnt);
void UTCPD_PE_get_snk_caps(int port, int32_t *pu32SnkArray, int32_t *pi32SnkCnt);

extern void UTCPD_TimerBaseInc(void);
extern void EADC_ConfigPins(void);
extern void EADC_Init(void);
extern void pd_task_reinit(int port);
extern bool pd_task_loop(int port);
extern void vconn_polarity_active_low();
extern void UART_Commandshell(int port);
extern void cpu_dump(uint32_t start_addr, uint32_t end_addr);
extern void VBUS_Source_Level(int port, char i8Level);
extern void VBUS_Sink_Enable(int32_t port, bool bIsEnable);

/*@}*/ /* end of group UTCPDLIB_EXPORTED_FUNCTIONS */



/*@}*/ /* end of group UTCPDLIB */

/*@}*/ /* end of group LIBRARY */

#ifdef __cplusplus
}
#endif

#endif /* __UTCPDLIB_H__ */