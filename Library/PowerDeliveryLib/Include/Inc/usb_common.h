/* Copyright 2019 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
#ifndef __CROS_EC_USB_COMMON_H
#define __CROS_EC_USB_COMMON_H

#undef SW_UNDER_CHECK

/* To make SRC role as plug out (It is invalid and no code refer the definition)  */
/* #define PD_ROLE_DEFAULT(port)        PD_ROLE_SOURCE  */

/* Functions that are shared between old and new PD stacks */
#include "usb_pd.h"
#include "usb_pd_tcpm.h"
#include "ec_commands.h"
#include "usb_common.h"

#define DEBUG_DP(...)


#define PD_MAX_VOLTAGE_MV                        20000       //20000  /* It should be defined in board level */
#define PD_OPERATING_POWER_MW                    100000      //65000  /* It should be defined in board level */

#if 0
    /* SW Add {*/
    #define CONFIG_USB_PD_TRY_SRC                    1
    #define CONFIG_USB_PE_SM                         1
    #define CONFIG_USB_PRL_SM                        1
    #define CONFIG_VBOOT_EFS2                        0
    #define CONFIG_USB_PD_ALT_MODE_DFP               0
    #define CONFIG_USB_PD_ALT_MODE_UFP_DP            0
    #define CONFIG_USBC_OCP                          0
    #define CONFIG_USBC_SS_MUX                       0
    #define USB_PD_DEBUG_LABELS                      0
    #define CONFIG_CHARGE_MANAGER                    0
    #define CONFIG_USBC_PPC                          0
    #define CONFIG_LOW_POWER_IDLE                    0
    #define CONFIG_USB_PD_TCPC_ON_CHIP               0
    #define CONFIG_USBC_VCONN                        1
    #define CONFIG_USBC_VCONN_SWAP                   0
    #define CONFIG_USBC_PPC_VCONN                    0
    #define CONFIG_POWER_COMMON                      1    /* baseboard/kalista */
    #define CONFIG_CHARGE_MANAGER                    0
    #define CONFIG_BC12_DETECT_DATA_ROLE_TRIGGER     0
    #define CONFIG_USB_PD_REV30                      1

    #define CONFIG_BOARD_RESET_AFTER_POWER_ON        0
    #define CONFIG_VBOOT_EFS2                        0
    #define CONFIG_BATTERY                           0
    #define TEST_BUILD                               0

    #define CONFIG_USBC_RETIMER_FW_UPDATE            0
    #define CONFIG_USB_PD_REQUIRE_AP_MODE_ENTRY      1
    #define CONFIG_USB_PD_FRS                        1


    #define CONFIG_USB_PD_TCPM_TCPCI                 1
    #define CONFIG_USB_PD_LOGGING                    0

    #define CONFIG_USB_PD_DISCHARGE_TCPC                     1
    #define CONFIG_USB_PD_DUAL_ROLE                              1


    /* Following option will always not to be modified { */
    #define CONFIG_USB_PD_VBUS_DETECT_TCPC           1
    #define CONFIG_USB_PD_DECODE_SOP                 1
    #define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      1
    #define CONFIG_USB_PD_PPC                        1   /* Power path control */
    #define CONFIG_USB_PD_TCPC_LOW_POWER             0
    #define CONFIG_USB_PD_FRS_TCPC                   1
    #define CONFIG_USB_PD_TCPC_VCONN                 1   /* For VCONN Power. tcpm.h - tcpm_set_vconn() */
    /* } */

    /* usb_pd_alt_mode_dfp.c */
    #define CONFIG_CMD_MFALLOW                       0
    #define CONFIG_USB_PD_TCPMV1                     0
    #define CONFIG_USB_PD_TBT_COMPAT_MODE            0
    #define CONFIG_USBC_PPC_SBU                      0
    #define CONFIG_USB_PD_USB4                       0
    #define CONFIG_USB_PD_USB4_DRD                   0
    #define CONFIG_USB_PD_USB32_DRD                  0
    #define CONFIG_USB_PD_PCIE_TUNNELING                         0
    #define CONFIG_USB_PD_ALT_MODE_DFP                           0
    #define CONFIG_MKBP_EVENT                        0
    #define CONFIG_USB_PD_CUSTOM_PDO                 1
    #define CONFIG_IO_EXPANDER_NCT38XX               0
    #define CONFIG_USB_CTVPD                         0
    #define CONFIG_USB_VPD                           0
    #define CONFIG_USB_PD_EXTENDED_MESSAGES          1    /* Furture it will be enabled */
    #define CONFIG_USB_PD_HOST_CMD                   0    /* We didn't support "PD Host commands" */
    #define CONFIG_USB_PD_DPS                        0    /* We didn't support "dynamic PDO selection" */

    #define CONFIG_USB_PD_PORT_MAX_COUNT                     (1)  /* may 2 ~ 3 */
    #define CONFIG_USB_PID                           0x8260
    #define CONFIG_USB_PD_3A_PORTS                                   0    /* To turn off the TCPMv2 3.0 A current allocation from the DPM, set
    * CONFIG_USB_PD_3A_PORTS to 0. */
    #define CONFIG_USB_PD_PULLUP                     TYPEC_RP_1A5   /* Default pull-up value on the USB-C ports when they are used as source. */

    #define CONFIG_USB_DRP_ACC_TRYSRC                1
    #define CONFIG_USB_TYPEC_SM                      1
    #define CONFIG_USB_PRL_SM                        1
    #define CONFIG_USB_PE_SM                         1
    #define CONFIG_USB_PD_TCPC                       0


    #define CONFIG_TEST_USB_PE_SM                    0
    #define CONFIG_DUMP_REGISTER                     0

    /* tcpci.c */
    #define DEBUG_I2C_FAULT_LAST_WRITE_OP            0
    #define DEBUG_FORCED_DISCHARGE                   0
    #define DEBUG_AUTO_DISCHARGE_DISCONNECT          0
    #define DEBUG_GET_CC                             0      /* Turn off it as release */
    #define DEBUG_ROLE_CTRL_UPDATES                  0    /* Turn off it as release */
    #define CONFIG_CMD_TCPC_DUMP                     0



    #define CONFIG_MKBP_EVENT                        0

    /* usb_dual_role.c */
    #define CONFIG_USB_PD_PREFER_MV                  0
    #define CONFIG_USB_PD_ONLY_FIXED_PDOS            0
    #define PD_MAX_CURRENT_MA                        3000           /* defined in board level */
    #define PD_MAX_POWER_MW                          65000    /* defined in board level */
    #define PD_PREFER_LOW_VOLTAGE                    0        /* Can't found on config.h */
    #define PD_PREFER_HIGH_VOLTAGE                   0        /* Can't found on config.h */
    #define CONFIG_USB_PD_CHECK_MAX_REQUEST_ALLOWED  0


    #if (OPT_SNK_ONLY == 1)
        #undef CONFIG_USB_PD_TRY_SRC
        #define CONFIG_USB_PD_TRY_SRC                    1
        #undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
        #define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      0
        #define CONFIG_COMMAND_SHELL                     1

        #define CONFIG_SUPPORT_SNK                       1
        #define CONFIG_SUPPORT_SRC                       0
        #define CONFIG_SUPPORT_PR_SWAP                   0
        #define CONFIG_SUPPORT_DR_SWAP                   0
        #define CONFIG_VDM_IDENTITY_REQUEST              0
        #define CONFIG_VDM_SVIDS_REQUEST                 0
        #define CONFIG_VDM_MODES_REQUEST                 0
        #define CONFIG_VDM_REQUEST_DPM                   0
    #endif

    #if (OPT_SRC_ONLY==1)
        #undef CONFIG_USB_PD_TRY_SRC
        #define CONFIG_USB_PD_TRY_SRC                    0
        #undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
        #define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      0
        #define CONFIG_COMMAND_SHELL                     1

        #define CONFIG_SUPPORT_SNK                       0
        #define CONFIG_SUPPORT_SRC                       1
        #define CONFIG_SUPPORT_PR_SWAP                   0
        #define CONFIG_SUPPORT_DR_SWAP                   0
        #define CONFIG_VDM_IDENTITY_REQUEST              0
        #define CONFIG_VDM_SVIDS_REQUEST                 0
        #define CONFIG_VDM_MODES_REQUEST                 0
        #define CONFIG_VDM_REQUEST_DPM                   0
    #endif


    #if (OPT_DRP == 1)
        #undef CONFIG_USB_PD_TRY_SRC
        #define CONFIG_USB_PD_TRY_SRC                    1
        #undef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
        #define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE      1
        #define CONFIG_COMMAND_SHELL                     1

        #define CONFIG_SUPPORT_SNK                       1
        #define CONFIG_SUPPORT_SRC                       1
        #define CONFIG_SUPPORT_PR_SWAP                   1
        #define CONFIG_SUPPORT_DR_SWAP                   1
        #define CONFIG_VDM_IDENTITY_REQUEST              1
        #define CONFIG_VDM_SVIDS_REQUEST                 1
        #define CONFIG_VDM_MODES_REQUEST                 1
        #define CONFIG_VDM_REQUEST_DPM                   1
    #endif

#endif

#define HAS_TASK_CHARGER                         0   /* usb_common.c */
/* Copy from include/chipset.h { SW ADD */
/* Standard macros / definitions */
#define GENERIC_MAX(x, y) ((x) > (y) ? (x) : (y))
#define GENERIC_MIN(x, y) ((x) < (y) ? (x) : (y))
#ifndef MAX
#ifndef __ICCARM__
#define MAX(a, b)                   \
    ({                      \
        __typeof__(a) temp_a = (a);     \
        __typeof__(b) temp_b = (b);     \
        \
        GENERIC_MAX(temp_a, temp_b);        \
    })
#else
static inline int MAX(int a, int b)
{
    return ((a) > (b) ? (a) : (b));
}

#endif
#endif

#ifndef MIN
#ifndef __ICCARM__
#define MIN(a, b)                   \
    ({                      \
        __typeof__(a) temp_a = (a);     \
        __typeof__(b) temp_b = (b);     \
        \
        GENERIC_MIN(temp_a, temp_b);        \
    })
#else
static inline int MIN(int a, int b)
{
    return ((a) < (b) ? (a) : (b));
}
#endif
#endif

#ifndef NULL
    #define NULL ((void *)0)
#endif
/* Copy from include/chipset.h } */

/* ./include/compile_time_macros.h */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#if 1
    #define PD_POWER_SUPPLY_TURN_ON_DELAY       30000
    #define PD_POWER_SUPPLY_TURN_OFF_DELAY    30000
#else
    #define PD_POWER_SUPPLY_TURN_ON_DELAY       100000      //DEBUG_ONLY
    #define PD_POWER_SUPPLY_TURN_OFF_DELAY    100000        //DEBUG_ONLY
#endif
#define CONFIG_USBC_VCONN_SWAP_DELAY_US   5000


typedef uint8_t task_id_t;
/* SW Add }*/

enum pd_drp_next_states
{
    DRP_TC_DEFAULT,
    DRP_TC_UNATTACHED_SNK,
    DRP_TC_ATTACHED_WAIT_SNK,
    DRP_TC_UNATTACHED_SRC,
    DRP_TC_ATTACHED_WAIT_SRC,
    DRP_TC_DRP_AUTO_TOGGLE
};

/**
 * Returns the next state to transition to while in the drp auto toggle state.
 *
 * @param drp_sink_time timer for handling TOGGLE_OFF/FORCE_SINK mode when
 *          auto-toggle enabled. This is an in/out variable.
 * @param power_role current power role
 * @param drp_state dual role states
 * @param cc1 value of CC1 set by tcpm_get_cc
 * @param cc2 value of CC2 set by tcpm_get_cc
 * @param auto_toggle_supported indicates hardware auto toggle support.
 *          Hardware auto toggle support will perform the
 *          unattached to attached debouncing before notifying
 *          us of a connection.
 *
 */
#if 0
enum pd_drp_next_states drp_auto_toggle_next_state(uint64_t *drp_sink_time, \
                                                   enum pd_power_role power_role, enum pd_dual_role_states drp_state, \
                                                   enum tcpc_cc_voltage_status cc1, enum tcpc_cc_voltage_status cc2, \
                                                   bool auto_toggle_supported);
#else
uint8_t drp_auto_toggle_next_state(uint64_t *drp_sink_time, uint8_t power_role, uint8_t drp_state, uint8_t cc1, uint8_t cc2, bool auto_toggle_supported);
#endif

enum pd_pref_type
{
    /* prefer voltage larger than or equal to pd_pref_config.mv */
    PD_PREFER_BUCK,
    /* prefer voltage less than or equal to pd_pref_config.mv */
    PD_PREFER_BOOST,
};

struct pd_pref_config_t
{
    /* Preferred PD voltage in mV */
    int mv;
    /* above which percent the battery is in constant voltage stage */
    int cv;
    /* System PLT (minimum consuming) power in mW. */
    int plt_mw;
    /* Preferred PD voltage pick strategy */
    enum pd_pref_type type;
};

/*
 * This function converts an 8 character ascii string with hex digits, without
 * the 0x prefix, into a signed 32-bit number.
 *
 * @param str pointer to hex string to convert
 * @param val pointer to where the integer version is stored
 * @return EC_SUCCSSS on success else EC_ERROR_INVAL on failure
 */
int hex8tou32(char *str, uint32_t *val);

/*
 * Flash a USB PD device using the ChromeOS Vendor Defined Command.
 *
 * @param argc number arguments in argv. Must be greater than 3.
 * @param argv [1] is the usb port
 *             [2] unused
 *             [3] is the command {"erase", "rebooot", "signature",
 *                                 "info", "version", "write"}
 *             [4] if command was "write", then this will be the
 *                 start of the data that will be written.
 * @return EC_SUCCESS on success, else EC_ERROR_PARAM_COUNT or EC_ERROR_PARAM2
 *         on failure.
 */
int remote_flashing(int argc, char **argv);

/*
 * When AP requests to suspend PD traffic on the EC so it can do
 * firmware upgrade (retimer firmware, or TCPC chips firmware),
 * it calls this function to check if power is ready for performing
 * the upgrade.
 * @param port USB-C port number
 * @dreturn true  - power is ready
 *          false - power is not ready
 */
bool pd_firmware_upgrade_check_power_readiness(int port);

/* Returns the battery percentage [0-100] of the system. */
int usb_get_battery_soc(void);

#ifndef typec_current_t
    typedef uint32_t typec_current_t;
#endif

/*
 * Returns type C current limit (mA), potentially with the DTS flag, based upon
 * states of the CC lines on the partner side.
 *
 * @param polarity port polarity
 * @param cc1 value of CC1 set by tcpm_get_cc
 * @param cc2 value of CC2 set by tcpm_get_cc
 * @return current limit (mA) with DTS flag set if appropriate
 */
typec_current_t usb_get_typec_current_limit(enum tcpc_cc_polarity polarity,
                                            enum tcpc_cc_voltage_status cc1, enum tcpc_cc_voltage_status cc2);

/**
 * Returns the polarity of a Sink.
 *
 * @param cc1 value of CC1 set by tcpm_get_cc
 * @param cc2 value of CC2 set by tcpm_get_cc
 * @return polarity
 */
//enum tcpc_cc_polarity get_snk_polarity(enum tcpc_cc_voltage_status cc1, enum tcpc_cc_voltage_status cc2);
uint8_t get_snk_polarity(uint8_t cc1, uint8_t cc2);
/**
 * Returns the polarity of a Source.
 *
 * @param cc1 value of CC1 set by tcpm_get_cc
 * @param cc2 value of CC2 set by tcpm_get_cc
 * @return polarity
 */
//enum tcpc_cc_polarity get_src_polarity(enum tcpc_cc_voltage_status cc1,
//  enum tcpc_cc_voltage_status cc2);
uint8_t get_src_polarity(uint8_t cc1, uint8_t cc2);
/**
 * Find PDO index that offers the most amount of power and stays within
 * max_mv voltage.
 *
 * @param src_cap_cnt
 * @param src_caps
 * @param max_mv maximum voltage (or -1 if no limit)
 * @param pdo raw pdo corresponding to index, or index 0 on error (output)
 * @return index of PDO within source cap packet
 */
int pd_find_pdo_index(uint32_t src_cap_cnt, const uint32_t *const src_caps,
                      int max_mv, uint32_t *selected_pdo);

/**
 * Extract power information out of a Power Data Object (PDO)
 *
 * @param pdo raw pdo to extract
 * @param ma current of the PDO (output)
 * @param max_mv maximum voltage of the PDO (output)
 * @param min_mv minimum voltage of the PDO (output)
 */
void pd_extract_pdo_power(uint32_t pdo, uint32_t *ma, uint32_t *max_mv,
                          uint32_t *min_mv);

/**
 * Decide which PDO to choose from the source capabilities.
 *
 * @param vpd_vdo VPD VDO
 * @param rdo  requested Request Data Object.
 * @param ma  selected current limit (stored on success)
 * @param mv  selected supply voltage (stored on success)
 * @param port USB-C port number
 */
void pd_build_request(int32_t vpd_vdo, uint32_t *rdo, uint32_t *ma,
                      uint32_t *mv, int port);

/**
 * Notifies a task that is waiting on a system jump, that it's complete.
 */
void notify_sysjump_ready(void);

/**
 * Set USB MUX with current data role
 *
 * @param port USB-C port number
 */
void set_usb_mux_with_current_data_role(int port);

/**
 * Check if the mux should be set to enable USB3.1 mode based only on being in a
 * UFP data role. This is mode is required when attached to a port partner that
 * is type-c only, but still needs to enable USB3.1 mode.
 *
 * @param port USB-C port number
 * @return true if USB3 mode should be enabled, false otherwise
 */
__override_proto bool usb_ufp_check_usb3_enable(int port);

/**
 * Configure the USB MUX in safe mode.
 * Before entering into alternate mode, state of the USB-C MUX needs to be in
 * safe mode.
 * Ref: USB Type-C Cable and Connector Specification
 * Section E.2.2 Alternate Mode Electrical Requirements
 *
 * @param port The PD port number
 */
void usb_mux_set_safe_mode(int port);

/**
 * Configure the USB MUX in safe mode while exiting an alternate mode.
 * Although the TCSS (virtual mux) has a distinct safe mode state, it
 * needs to be in a disconnected state to properly exit an alternate
 * mode. Therefore, do not treat the virtual mux as a special case, as
 * usb_mux_set_safe_mode does.
 *
 * @param port The PD port number
 */
void usb_mux_set_safe_mode_exit(int port);

/**
 * Get the PD flags stored in BB Ram
 *
 * @param port USB-C port number
 * @param flags pointer where flags are written to
 * @return EC_SUCCESS on success
 */
int pd_get_saved_port_flags(int port, uint8_t *flags);

/**
 * Update the flag in BB Ram with the give value
 *
 * @param port USB-C port number
 * @param flag BB Ram flag to update
 * @param do_set value written to the BB Ram flag
 */
void pd_update_saved_port_flags(int port, uint8_t flag, uint8_t do_set);

/**
 * Build PD alert message
 *
 * @param msg pointer where message is stored
 * @param len pointer where length of message is stored in bytes
 * @param pr  current PD power role
 * @return EC_SUCCESS on success else EC_ERROR_INVAL
 */
int pd_build_alert_msg(uint32_t *msg, uint32_t *len, enum pd_power_role pr);

/**
 * During USB retimer firmware update, process operation
 * requested by AP
 *
 * @param port USB-C port number
 * @param op
 *       0 - USB_RETIMER_FW_UPDATE_QUERY_PORT
 *       1 - USB_RETIMER_FW_UPDATE_SUSPEND_PD
 *       2 - USB_RETIMER_FW_UPDATE_RESUME_PD
 *       3 - USB_RETIMER_FW_UPDATE_GET_MUX
 *       4 - USB_RETIMER_FW_UPDATE_SET_USB
 *       5 - USB_RETIMER_FW_UPDATE_SET_SAFE
 *       6 - USB_RETIMER_FW_UPDATE_SET_TBT
 *       7 - USB_RETIMER_FW_UPDATE_DISCONNECT
 */
void usb_retimer_fw_update_process_op(int port, int op);

/**
 * Get result of last USB retimer firmware update operation requested
 * by AP. Result is passed to AP via EC_CMD_ACPI_READ.
 *
 * @return Result of last operation. It's
 *         which port has retimer if last operation is
 *         USB_RETIMER_FW_UPDATE_QUERY_PORT;
 *         PD task is enabled or not if last operations are
 *         USB_RETIMER_FW_UPDATE_SUSPEND_PD or
 *         USB_RETIMER_FW_UPDATE_QUERY_PORT;
 *         current mux if last operations are
 *         USB_RETIMER_FW_UPDATE_GET_MUX, USB_RETIMER_FW_UPDATE_SET_USB,
 *         USB_RETIMER_FW_UPDATE_SET_SAFE, USB_RETIMER_FW_UPDATE_SET_TBT,
 *         or USB_RETIMER_FW_UPDATE_DISCONNECT.
 */
int usb_retimer_fw_update_get_result(void);

/**
 * Process deferred retimer firmware update operations.
 *
 * @param port USB-C port number
 */
void usb_retimer_fw_update_process_op_cb(int port);

/**
 * Dump SourceCap information.
 *
 * @param port USB-C port number
 */
void pd_srccaps_dump(int port);
#endif /* __CROS_EC_USB_COMMON_H */
