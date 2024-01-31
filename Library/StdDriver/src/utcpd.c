/**************************************************************************//**
 * @file     utcpd.c
 * @version  V1.00
 * @brief    M55M1 series UTCPD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"
#include "utcpdlib.h"

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UTCPD_Driver UTCPD Driver
  @{
*/

/** @addtogroup UTCPD_EXPORTED_FUNCTIONS UTCPD Exported Functions
  @{
*/


/**
  * @brief      Get UTCPD Alert Status
  *
  * @param[in]  port            Specify UTCPD port
  * @param[in]  i32AlertSts     Alert status point
  *             - \ref UTCPD_ALERT_CCSCHIS
  *             - \ref UTCPD_ALERT_PWRSCHIS
  *             - \ref UTCPD_ALERT_RXSOPIS
  *             - \ref UTCPD_ALERT_RXHRSTIS
  *             - \ref UTCPD_ALERT_TXFAILIS
  *             - \ref UTCPD_ALERT_TXDCUIS
  *             - \ref UTCPD_ALERT_TXOKIS
  *             - \ref UTCPD_ALERT_VBAMHIS
  *             - \ref UTCPD_ALERT_VBAMLIS
  *             - \ref UTCPD_ALERT_FUTIS
  *             - \ref UTCPD_ALERT_RXOFIS
  *             - \ref UTCPD_ALERT_SKDCDTIS
  *             - \ref UTCPD_ALERT_VNDIS
  * @return     0: Successful,  1: Fail
  *
  * @details    Get the  Alert Status flag
  *
  */
int32_t UTCPD_GetAlertStatus(int port, int *i32AlertSts)
{
    return tcpc_addr_read16(port, NULL, TCPC_REG_ALERT, i32AlertSts);
}

/**
  * @brief      Clear UTCPD Alert Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  AlertStClr     Alert status clear
  *             - \ref UTCPD_ALERT_CCSCHIS
  *             - \ref UTCPD_ALERT_PWRSCHIS
  *             - \ref UTCPD_ALERT_RXSOPIS
  *             - \ref UTCPD_ALERT_RXHRSTIS
  *             - \ref UTCPD_ALERT_TXFAILIS
  *             - \ref UTCPD_ALERT_TXDCUIS
  *             - \ref UTCPD_ALERT_TXOKIS
  *             - \ref UTCPD_ALERT_VBAMHIS
  *             - \ref UTCPD_ALERT_VBAMLIS
  *             - \ref UTCPD_ALERT_FUTIS
  *             - \ref UTCPD_ALERT_RXOFIS
  *             - \ref UTCPD_ALERT_SKDCDTIS
  *             - \ref UTCPD_ALERT_VNDIS
  * @return     0: Successful,  1: Fail
  *
  * @details    Write One Clear
  *
  */
int32_t UTCPD_ClearAlertStatus(int port, int AlertStClr)
{
    return tcpc_update16(port, TCPC_REG_ALERT, AlertStClr, MASK_SET);
}

/**
  * @brief      Enable UTCPD Alert Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Alert bit mask
  *             - \ref UTCPD_ALERTM_CCSCHIE
  *             - \ref UTCPD_ALERTM_PWRSCHIE
  *             - \ref UTCPD_ALERTM_RXSOPIE
  *             - \ref UTCPD_ALERTM_RXHRSTIE
  *             - \ref UTCPD_ALERTM_TXFAILIE
  *             - \ref UTCPD_ALERTM_TXDCUIE
  *             - \ref UTCPD_ALERTM_TXOKIE
  *             - \ref UTCPD_ALERTM_VBAMHIE
  *             - \ref UTCPD_ALERTM_VBAMLIE
  *             - \ref UTCPD_ALERTM_FUTIE
  *             - \ref UTCPD_ALERTM_RXOFIE
  *             - \ref UTCPD_ALERTM_SKDCDTIE
  *             - \ref UTCPD_ALERTM_VNDIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableAlertMask(int port, int mask_set)
{
    return tcpc_update16(port, TCPC_REG_ALERT_MASK, mask_set, MASK_SET);
}


/**
  * @brief      Disable UTCPD Alert Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     clear Alert bit mask
  *             - \ref UTCPD_ALERTM_CCSCHIE
  *             - \ref UTCPD_ALERTM_PWRSCHIE
  *             - \ref UTCPD_ALERTM_RXSOPIE
  *             - \ref UTCPD_ALERTM_RXHRSTIE
  *             - \ref UTCPD_ALERTM_TXFAILIE
  *             - \ref UTCPD_ALERTM_TXDCUIE
  *             - \ref UTCPD_ALERTM_TXOKIE
  *             - \ref UTCPD_ALERTM_VBAMHIE
  *             - \ref UTCPD_ALERTM_VBAMLIE
  *             - \ref UTCPD_ALERTM_FUTIE
  *             - \ref UTCPD_ALERTM_RXOFIE
  *             - \ref UTCPD_ALERTM_SKDCDTIE
  *             - \ref UTCPD_ALERTM_VNDIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableAlertMask(int port, int mask_clr)
{
    return tcpc_update16(port, TCPC_REG_ALERT_MASK, mask_clr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Power Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Power status bit mask
  *             - \ref UTCPD_PWRSM_SKVBIE
  *             - \ref UTCPD_PWRSM_VCPSIE
  *             - \ref UTCPD_PWRSM_VBPSIE
  *             - \ref UTCPD_PWRSM_VBDTDGIE
  *             - \ref UTCPD_PWRSM_SRVBIE
  *             - \ref UTCPD_PWRSM_SRHVIE
  *             - \ref UTCPD_PWRSM_DACONIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnablePowerStatusMask(int port, int mask_set)
{
    return tcpc_update16(port, TCPC_REG_POWER_STATUS_MASK, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Power Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     clear Power bit mask
  *             - \ref UTCPD_PWRSM_SKVBIE
  *             - \ref UTCPD_PWRSM_VCPSIE
  *             - \ref UTCPD_PWRSM_VBPSIE
  *             - \ref UTCPD_PWRSM_VBDTDGIE
  *             - \ref UTCPD_PWRSM_SRVBIE
  *             - \ref UTCPD_PWRSM_SRHVIE
  *             - \ref UTCPD_PWRSM_DACONIE
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
    *
  */
int32_t UTCPD_DisablePowerStatusMask(int port, int mask_clr)
{
    return tcpc_update16(port, TCPC_REG_POWER_STATUS_MASK, mask_clr, MASK_CLR);
}


/**
  * @brief      Enable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault Status bit mask
  *             - \ref UTCPD_FAULTSM_VCOCIE
  *             - \ref UTCPD_FAULTSM_VBOVIE
  *             - \ref UTCPD_FAULTSM_VBOCIE
  *             - \ref UTCPD_FAULTSM_FDGFALIE
  *             - \ref UTCPD_FAULTSM_ADGFALIE
  *             - \ref UTCPD_FAULTSM_FOFFVBIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableFaultMask(int port, int mask_set)
{
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     clear Fault Status bit mask
  *             - \ref UTCPD_FAULTSM_VCOCIE
  *             - \ref UTCPD_FAULTSM_VBOVIE
  *             - \ref UTCPD_FAULTSM_VBOCIE
  *             - \ref UTCPD_FAULTSM_FDGFALIE
  *             - \ref UTCPD_FAULTSM_ADGFALIE
  *             - \ref UTCPD_FAULTSM_FOFFVBIE
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableFaultMask(int port, int mask_clr)
{
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK, mask_clr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Power Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Power Control bit mask
  *             - \ref UTCPD_PWRCTL_VCEN
  *             - \ref UTCPD_PWRCTL_VCPWR
  *             - \ref UTCPD_PWRCTL_FDGEN
  *             - \ref UTCPD_PWRCTL_BDGEN
  *             - \ref UTCPD_PWRCTL_ADGDC
  *             - \ref UTCPD_PWRCTL_DSVBAM_DIS
  *             - \ref UTCPD_PWRCTL_VBMONI_DIS
  *
  * @return     None
  *
  * @details    Remember that VBUS Monitor and VBUS Alarm are "0" enabled. And both are "1" disabled
  *
  */
int32_t UTCPD_EnablePowerCtrl(int port, uint32_t mask_set)
{
    return tcpc_update16(port, TCPC_REG_POWER_CTRL, mask_set, MASK_SET);  /* VBUS Monitor and VBUS Alarm are 0 to enable */
}

/**
  * @brief      Disable UTCPD Power Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  maskclr      clear Power Control bit mask
  *             - \ref UTCPD_PWRCTL_VCEN
  *             - \ref UTCPD_PWRCTL_VCPWR
  *             - \ref UTCPD_PWRCTL_FDGEN
  *             - \ref UTCPD_PWRCTL_BDGEN
  *             - \ref UTCPD_PWRCTL_ADGDC
  *             - \ref UTCPD_PWRCTL_DSVBAM_DIS
  *             - \ref UTCPD_PWRCTL_VBMONI_DIS
  * @return     None
  *
  * @details    Remember that VBUS Monitor and VBUS Alarm are "0" enabled. And both are "1" disabled
  *
  */
int32_t UTCPD_DisablePowerCtrl(int port, uint32_t maskclr)
{
    return tcpc_update16(port, TCPC_REG_POWER_CTRL, maskclr, MASK_CLR);  /* VBUS Monitor and VBUS Alarm are 1 to disable */
}


/**
  * @brief      Enable UTCPD Fault Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault bit mask
  *             - \ref UTCPD_FAULTCTL_VCOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBOVDTDS
  *             - \ref UTCPD_FAULTCTL_VBOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBDGTMDS
  *             - \ref UTCPD_FAULTCTL_FOFFVBDS
  *
  * @return     None
  *
  * @details    When Time-out flag will be set, use this function to clear I2C Bus Time-out counter flag .
  *
  */
int32_t UTCPD_EnableFaultCtrl(int port, uint32_t mask_set)
{
    return tcpc_update16(port, TCPC_REG_FAULT_CTRL, mask_set, MASK_CLR);  /* 0 to enable */
}

/**
  * @brief      Disable UTCPD Fault Control
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  maskclr      clear Fault bit mask
  *             - \ref UTCPD_FAULTCTL_VCOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBOVDTDS
  *             - \ref UTCPD_FAULTCTL_VBOCDTDS
  *             - \ref UTCPD_FAULTCTL_VBDGTMDS
  *             - \ref UTCPD_FAULTCTL_FOFFVBDS
  * @return     None
  *
  * @details    None
  *
  */
int32_t UTCPD_DisableFaultCtrl(int port, uint32_t maskclr)
{
    return tcpc_update16(port, TCPC_REG_FAULT_CTRL, maskclr, MASK_SET);  /* 1 to disable */
}



/**
  * @brief      Clear UTCPD Power Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  PowerStClr   Power status clear
  *             - \ref UTCPD_PWRSTS_SKVB
  *             - \ref UTCPD_PWRSTS_VCPS
  *             - \ref UTCPD_PWRSTS_VBPS
  *             - \ref UTCPD_PWRSTS_VBPSDTEN
  *             - \ref UTCPD_PWRSTS_SRVB
  *             - \ref UTCPD_PWRSTS_SRHV
  *             - \ref UTCPD_PWRSTS_DACON
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_ClearPowerStatus(int port, int PowerStClr)
{
    return tcpc_update16(port, TCPC_REG_POWER_STATUS, PowerStClr, MASK_CLR);
}

/**
  * @brief      Enable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specified UTCPD port
  * @param[in]  mask_set     Fault status bit mask
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_EnableFaultStatusMask(int port, int mask_set)
{
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS, mask_set, MASK_SET);
}

/**
  * @brief      Disable UTCPD Fault Status Mask
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  mask_clr     clear Fault bit mask
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_DisableFaultStatusMask(int port, int mask_clr)
{
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS_MASK, mask_clr, MASK_CLR);
}

/**
  * @brief      Get UTCPD Fault Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pi32RegData  Fault status
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_GetFaultStatus(int port, int *pi32RegData)
{
    return tcpc_addr_read16(port, NULL, TCPC_REG_FAULT_STATUS, pi32RegData);
}

/**
  * @brief      Clear UTCPD Fault Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  FaultStClr   Fault status clear
  *
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_ClearFaultStatus(int port, int FaultStClr)
{
    return tcpc_update16(port, TCPC_REG_FAULT_STATUS, FaultStClr, MASK_CLR);
}

/**
  * @brief      Set Role Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32DrpToggle DRP toggling
  *             - \ref UTCPD_ROLECTL_DRP
  *             - \ref NULL
  * @param[in]  u32Rpvalue   Rp pull up value
  *             - \ref UTCPD_ROLECTL_RPVALUE_DEF
  *             - \ref UTCPD_ROLECTL_RPVALUE_1P5A
  *             - \ref UTCPD_ROLECTL_RPVALUE_3A
  * @param[in]  u32CC2       Force CC2 state
  *             - \ref UTCPD_ROLECTL_CC2_RA
  *             - \ref UTCPD_ROLECTL_CC2_RP
  *             - \ref UTCPD_ROLECTL_CC2_RD
  *             - \ref UTCPD_ROLECTL_CC2_OPEN
  * @param[in]  u32CC1       Force CC1 state
  *             - \ref UTCPD_ROLECTL_CC1_RA
  *             - \ref UTCPD_ROLECTL_CC1_RP
  *             - \ref UTCPD_ROLECTL_CC1_RD
  *             - \ref UTCPD_ROLECTL_CC1_OPEN
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetRoleCtrl(int port, uint32_t u32DrpToggle, uint32_t u32Rpvalue, uint32_t u32CC2, uint32_t u32CC1)
{
    uint8_t reg_set = u32DrpToggle | u32Rpvalue | u32CC2 | u32CC1;
    return tcpc_addr_write16(port, NULL, TCPC_REG_ROLE_CTRL, reg_set);
}

/**
  * @brief      Get Role Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32DrpToggle DRP toggling or not
  * @param[in]  pu32CC1       Force CC1 state
  *             - \ref UTCPD_ROLECTL_CC1_RA
  *             - \ref UTCPD_ROLECTL_CC1_RP
  *             - \ref UTCPD_ROLECTL_CC1_RD
  *             - \ref UTCPD_ROLECTL_CC1_OPEN
  * @param[in]  pu32CC2       Force CC2 state
  *             - \ref UTCPD_ROLECTL_CC2_RA
  *             - \ref UTCPD_ROLECTL_CC2_RP
  *             - \ref UTCPD_ROLECTL_CC2_RD
  *             - \ref UTCPD_ROLECTL_CC2_OPEN
  * @param[in]  pu32Rpvalue   Rp pull up value
  *             - \ref UTCPD_ROLECTL_RPVALUE_DEF
  *             - \ref UTCPD_ROLECTL_RPVALUE_1P5A
  *             - \ref UTCPD_ROLECTL_RPVALUE_3A
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetRoleCtrl(int port, uint32_t *pu32DrpToggle, uint32_t *pu32CC1, uint32_t *pu32CC2, uint32_t *pu32Rpvalue)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_ROLE_CTRL, &i32Reg);
    *pu32DrpToggle = i32Reg & UTCPD_ROLCTL_DRP_Msk;
    *pu32Rpvalue = i32Reg & UTCPD_ROLCTL_RPVALUE_Msk;
    *pu32CC2 = i32Reg & UTCPD_ROLCTL_CC2_Msk;
    *pu32CC1 = i32Reg & UTCPD_ROLCTL_CC1_Msk;
    return rv;
}

/**
  * @brief      Set Type-C Port Control
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32BistMode  Enable or Disable BIST Mode
  * @param[in]  u32Orient    Plug Oritentation
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetTypeCPortCtrl(int port, uint32_t u32BistMode, uint32_t u32Orient)
{
    uint8_t reg_set = u32BistMode | u32Orient;
    return tcpc_addr_write16(port, NULL, TCPC_REG_TCPC_CTRL, reg_set);
}


/**
  * @brief      Get Role Control
  *
  * @param[in]  port          Specify UTCPD port
  * @param[in]  pu32BistMode  Enable or Disable BIST Mode
  * @param[in]  pu32Orient    Plug Oritentation
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetTypeCPortCtrl(int port, uint32_t *pu32BistMode, uint32_t *pu32Orient)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_TCPC_CTRL, &i32Reg);
    *pu32BistMode = i32Reg & UTCPD_TCPCCTL_BISTEN;
    *pu32Orient = i32Reg & UTCPD_TCPCCTL_ORIENT;
    return rv;
}
/**
  * @brief      UTCPD Issue Command
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  cmd          Type-C Port Control
  *             - \ref UTCPD_CMD_DISABLE_VBUS_DETECT
  *             - \ref UTCPD_CMD_ENABLE_VBUS_DETECT
  *             - \ref UTCPD_CMD_DISABLE_SINK_VBUS
  *             - \ref UTCPD_CMD_SINK_VBUS
  *             - \ref UTCPD_CMD_SRC_VBUS_DEFAULT
  *             - \ref UTCPD_CMD_SRC_VBUS_NONDEFAULT
  *             - \ref UTCPD_CMD_LOOK4CONNECTION
  * @return     0: Successful,  1: Fail
  *
  * @details    The command can't be read back.
  *
  */
int32_t UTCPD_IsssueCmd(int port, uint32_t cmd)
{

    return tcpc_addr_write16(port, NULL, TCPC_REG_COMMAND, cmd);
}



/**
  * @brief      Get CC Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32Look4Con              Under Look for connection or done
  *             - \ref UTCPD_LK4CONN_DONE
  *             - \ref UTCPD_UNDER_LK4CONN
  * @param[in]  pu32ConRlt                Connection result
  *             - \ref UTCPD_CONN_RESULT_RP
  *             - \ref UTCPD_CONN_RESULT_RD
  * @param[in]  pu32CC2Sts    The is CC2_State.  CC2 connection state depends on connection result
  *              - \ref UTCPD_CCSTS_CC2STATE_SRC_OPEN
  *              - \ref UTCPD_CCSTS_CC2STATE_SRC_RA
  *              - \ref UTCPD_CCSTS_CC2STATE_SRC_RD
  *              - \ref UTCPD_CCSTS_CC2STATE_SNK_OPEN
  *              - \ref UTCPD_CCSTS_CC2STATE_SNK_DEF
  *              - \ref UTCPD_CCSTS_CC2STATE_SNK_1P5A
  *              - \ref UTCPD_CCSTS_CC2STATE_SNK_3A
  * @param[in]  pu32CC1Sts is CC1_State.   CC1 connection state depends on connection result
  *             - \ref UTCPD_CCSTS_CC1STATE_SRC_OPEN
  *             - \ref UTCPD_CCSTS_CC1STATE_SRC_RA
  *             - \ref UTCPD_CCSTS_CC1STATE_SRC_RD
  *             - \ref UTCPD_CCSTS_CC1STATE_SNK_OPEN
  *             - \ref UTCPD_CCSTS_CC1STATE_SNK_DEF
  *             - \ref UTCPD_CCSTS_CC1STATE_SNK_1P5A
  *             - \ref UTCPD_CCSTS_CC1STATE_SNK_3A
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetCCSts(int port, uint32_t *pu32Look4Con, uint32_t *pu32ConRlt, uint32_t *pu32CC2Sts, uint32_t *pu32CC1Sts)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_CC_STATUS, &i32Reg);
    *pu32Look4Con = (i32Reg & TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK);
    *pu32ConRlt = (i32Reg & TCPC_REG_CC_STATUS_CONNECT_RESULT_MASK);
    *pu32CC2Sts = (i32Reg & TCPC_REG_CC_STATUS_CC2_STATE_MASK);
    *pu32CC1Sts = (i32Reg & TCPC_REG_CC_STATUS_CC1_STATE_MASK);
    return rv;
}


/**
  * @brief      Get Power Status
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32VBUSDetEn         VBUS Present Detection Enabled or not
  * @param[in]  pu32VBUSPresent       VBUS Present or not
  * @param[in]  pu32VCONNPresent      VCONN Present or not
  * @param[in]  pu32SnkVBUS           Sinking VBUS or not
* @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetPwrSts(int port, uint32_t *pu32VBUSDetEn, uint32_t *pu32VBUSPresent, uint32_t *pu32VCONNPresent, uint32_t *pu32SnkVBUS)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_POWER_STATUS, &i32Reg);
    *pu32VBUSDetEn = i32Reg & UTCPD_PWRSTS_VBPSDTEN_Msk;
    *pu32VBUSPresent = i32Reg & UTCPD_PWRSTS_VBPS_Msk;
    *pu32VCONNPresent = i32Reg & UTCPD_PWRSTS_VCPS_Msk;
    *pu32SnkVBUS = i32Reg & UTCPD_PWRSTS_SKVB_Msk;
    return rv;
}

/**
  * @brief      Get Power Status Extention
  *
  * @param[in]  port                      Specify UTCPD port
  * @param[in]  pu32DbgAccessory          Debug Accessory Mode or not
  * @param[in]  pu32SrcNonDefVBUS         Source Non-Default VBUS or not
  * @param[in]  pu32SrcDefVBUS            Source Default VBUS or not
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetPwrStsExt(int port, uint32_t *pu32DbgAccessory, uint32_t *pu32SrcNonDefVBUS, uint32_t *pu32SrcDefVBUS)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_POWER_STATUS, &i32Reg);
    *pu32DbgAccessory = i32Reg & UTCPD_PWRSTS_DACON_Msk;
    *pu32SrcNonDefVBUS = i32Reg & UTCPD_PWRSTS_SRHV_Msk;
    *pu32SrcDefVBUS = i32Reg & UTCPD_PWRSTS_SRVB_Msk;
    return rv;
}

/**
  * @brief      Get Power Status
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  pu32VBUSOverCurr         VBUS Over Current Fault or not
  * @param[in]  pu32VBUSOverVolt         VBUS Over Voltage Fault or not
  * @param[in]  pu32VCONNOverCurr        VCONN Over Current  Fault or not
  * @param[in]  pu32I2CInfErr            I2C Interface Error(Not support)
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetFaultSts(int port, uint32_t *pu32VBUSOverCurr, uint32_t *pu32VBUSOverVolt, uint32_t *pu32VCONNOverCurr, uint32_t *pu32I2CInfErr)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_FAULT_STATUS, &i32Reg);
    *pu32VBUSOverCurr = i32Reg & UTCPD_FUTSTS_VBOCFUT_Msk;
    *pu32VBUSOverVolt = i32Reg & UTCPD_FUTSTS_VBOVFUT_Msk;
    *pu32VCONNOverCurr = i32Reg & UTCPD_FUTSTS_VCOCFUT_Msk;
    *pu32I2CInfErr = 0;  /* M55M1 didn't support I2C */
    return rv;
}


/**
  * @brief      Get Fault Status Extention
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  pu32ForceOffFat         Force Off VBUS Fault
  * @param[in]  pu32AutoDiscFat         Auto Discharge Fault
  * @param[in]  pu32ForceDiscFat        Foece Discharge Fault
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  */
int32_t UTCPD_GetFaultStsExt(int port, uint32_t *pu32ForceOffFat, uint32_t *pu32AutoDiscFat, uint32_t *pu32ForceDiscFat)
{
    int i32Reg;
    int32_t rv;

    rv = tcpc_addr_read32(port, NULL, TCPC_REG_FAULT_STATUS, &i32Reg);
    *pu32ForceOffFat = i32Reg & UTCPD_FUTSTS_FOFFVB_Msk;
    *pu32AutoDiscFat = i32Reg & UTCPD_FUTSTS_ADGFAL_Msk;
    *pu32ForceDiscFat = i32Reg & UTCPD_FUTSTS_FDGFAL_Msk;
    return rv;
}


/**
  * @brief      Set Message Header Information
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32DataRole  Data Role
  *             - \ref UTCPD_MHINFO_DROLE_UFP
  *             - \ref UTCPD_MHINFO_DROLE_DFP
  * @param[in]  u32Revision  PD Revision
  *             - \ref UTCPD_MHINFO_PDREV_10
  *             - \ref UTCPD_MHINFO_PDREV_20
  *             - \ref UTCPD_MHINFO_PDREV_30
  *             - \ref UTCPD_MHINFO_PDREV_31
  * @param[in]  u32PwrRole  Power Role
  *             - \ref UTCPD_MHINFO_PROLE_SNK
  *             - \ref UTCPD_MHINFO_PROLE_SRC
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetMsgHeaderInfo(int port, uint32_t u32DataRole, uint32_t u32Revision, uint32_t u32PwrRole)
{
    uint8_t reg_set = u32DataRole | u32Revision | u32PwrRole;
    return tcpc_addr_write16(port, NULL, TCPC_REG_MSG_HDR_INFO, reg_set);
}

/**
  * @brief      Set Message Header Information
  *
  * @param[in]  port         Specify UTCPD port
  * @param[in]  u32RegData   Combination from following set
  *             - \ref UTCPD_RDET_SOPEN
  *             - \ref UTCPD_RDET_SOPPEN
  *             - \ref UTCPD_RDET_SOPPPEN
  *             - \ref UTCPD_RDET_SDBGPEN
  *             - \ref UTCPD_RDET_SDBGPEN
  *             - \ref UTCPD_RDET_SDBGPPEN
  *             - \ref UTCPD_RDET_HRSTEN
  *             - \ref UTCPD_RDET_CABRSTEN
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetRecDetect(int port, uint32_t u32RegData)
{
    uint8_t reg_set = u32RegData;
    return tcpc_addr_write16(port, NULL, TCPC_REG_RX_DETECT, reg_set);
}

/**
  * @brief      Set VBUS Alarm High and Alarm Low Voltage
  *
  * @param[in]  port             Specify UTCPD port
  * @param[in]  u32AlarmH        VBUS Alarm High Threshold Voltage
  * @param[in]  u32AlarmL        VBUS Alarm Low Threshold Voltage
  * @return     0: Successful,  1: Fail
  *
  * @details    None
  *
  */
int32_t UTCPD_SetVBUSAlarm(int port, uint32_t u32AlarmH, uint32_t u32AlarmL)
{
    uint8_t u8TxBuf[4];
    u8TxBuf[0] = u32AlarmH;
    u8TxBuf[1] = u32AlarmH >> 8;
    u8TxBuf[2] = u32AlarmL;
    u8TxBuf[3] = u32AlarmL >> 8;
    return tcpc_write_block(port, TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG, u8TxBuf, 4);
}

/**
  * @brief      Set Sink Disconnection Voltage
  *
  * @param[in]  port             Specify UTCPD port
  * @param[in]  u32SnkDiscVolt   Sink Disconnection Threshold Voltae
  * @return     0: Successful,  1: Fail
  *
  * @details    Sink Will start the VBUS auto discharge if VBUS voltage less Sink Disconnection Threshold Level
  *             POWER_CONTROL[Auto Discharge] should set to 1.
  *
  */
int32_t UTCPD_SetSnkDisconnect(int port, uint32_t u32SnkDiscVolt)
{
    return tcpc_addr_write16(port, NULL, TCPC_REG_VBUS_SINK_DISCONNECT_THRESH, u32SnkDiscVolt);
}

/**
  * @brief      Set VBUS Stop Discharge Threshold Voltage
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32StopDischgVolt   VBUS Stop Discharge Threshold Voltage
  * @return     0: Successful,  1: Fail
  *
  * @details    SRC/SNK Will stop VBUS force discharge if VBUS voltage less VBUS Stop Discharge Threshold Level
  *             POWER_CONTROL[Force Discharge] should set to 1.
  *
  */
int32_t UTCPD_SetStopDischargeVolt(int port, uint32_t u32StopDischgVolt)
{
    return tcpc_addr_write16(port, NULL, TCPC_REG_VBUS_STOP_DISCHARGE_THRESH, u32StopDischgVolt);
}


/*
    The CC ststus interupt won't be issued if Enable_VCONN_SRC_CC with wrong VCONN source CC pin
*/
/**
  * @brief      Disable VCONN Power Source to CCx pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    None
  *
  */
void UTCPD_vconn_disable_src_cc(int port)
{
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_ENABLE_VCONN, MASK_CLR);
}
/**
  * @brief      Enable VCONN Power Source to CCx pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    None
  *
  */
void UTCPD_vconn_enable_src_cc(int port)
{
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_ENABLE_VCONN, MASK_SET);
}

/**
  * @brief      VCONN Power Source to CC2 pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    Enable VCONN Source CC2, Communication channel through CC1
  *
  */
void UTCPD_vconn_enable_from_cc2(int port)
{
    tcpc_update16(port, TCPC_REG_TCPC_CTRL, TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION, MASK_CLR);
}

/**
  * @brief      VCONN Power Source to CC1 pin
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    Enable VCONN Source CC1, Communication channel through CC2
  *
  */
void UTCPD_vconn_enable_from_cc1(int port)
{
    tcpc_update16(port, TCPC_REG_TCPC_CTRL, TCPC_REG_TCPC_CTRL_PLUG_ORIENTATION, MASK_SET);
}

/**
  * @brief      Set VCONN Active Polarity Low
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    It depends on the external circuit
  *
  */
void UTCPD_vconn_polarity_active_low(int port)
{
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VCEN, MASK_CLR);
}

/**
  * @brief      Set VCONN Active Polarity High
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details    It depends on the external circuit
  *
  */
void UTCPD_vconn_polarity_active_high(int port)
{
    /* Set VCONN Polarity Active Low due to CC1VCENS and CC2VCENS default high */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VCEN, MASK_SET);
}

/**
  * @brief      Disable VCONN OC Fault
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details
  *
  */
void UTCPD_vconn_disable_oc_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS, MASK_SET);
}

/**
  * @brief      Enable VCONN OC Fault
  *
  * @param[in]  port                Specify UTCPD port
  * @return     None
  *
  * @details
  *
  */
void UTCPD_vconn_enable_oc_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VCONN_OCP_FAULT_DIS, MASK_CLR);
}

/**
  * @brief      Specified VCONN Enable Multiplex Path
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  cc1vcensel          It should be 1 default
  * @param[in]  cc2vcensel          It should be 1 default
  *
  * @details
  *
  */
void UTCPD_vconn_mux_selection(int port, uint32_t cc1vcensel, uint32_t cc2vcensel)
{
    int data;
    tcpc_addr_read16(port, NULL, UTCPD_MUXSEL, &data);
    data = (data & ~(CC2VCENS | CC1VCENS)) | ((cc1vcensel << 24) | (cc2vcensel << 28));
    tcpc_addr_write32(port, NULL, UTCPD_MUXSEL, (unsigned int)data);
}

/**
  * @brief      Configure VCONN OC source detection
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Src              Specify VCONN OC Source Detection
  *             - \ref UTCPD_VCONN_OC_EINT0
  *             - \ref UTCPD_VCONN_OC_EINT1
  *             - \ref UTCPD_VCONN_OC_EINT2
  *             - \ref UTCPD_VCONN_OC_EINT3
  *             - \ref UTCPD_VCONN_OC_ADC_CMP1
  *             - \ref UTCPD_VCONN_OC_ACMP0
  *             - \ref UTCPD_VCONN_OC_ACMP1
  *             - \ref UTCPD_VCONN_OC_ACMP2
  * @details
  *
  */
void UTCPD_vconn_configure_oc_detection_soruce(int port, uint32_t u32Src)
{
    int data;
    tcpc_addr_read16(port, NULL, UTCPD_MUXSEL, &data);
    data = (data & ~VCOCS) | (u32Src << 4);
    tcpc_addr_write16(port, NULL, UTCPD_MUXSEL, data);
}

/* ============  VBUS SRCEN Polarity ==========*/
void UTCPD_vbus_srcen_polarity_active_low(int port)
{
    /* Set VBUS SRCEN Polarity active Low */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SRCEN, MASK_CLR);
}
void UTCPD_vbus_srcen_polarity_active_high(int port)
{
    /* Set VBUS SRCEN Polarity Active high */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SRCEN, MASK_SET);
}
/* ============  VBUS SNKEN Polarity ==========*/
void UTCPD_vbus_snken_polarity_active_low(int port)
{
    /* Set VBUS SNKEN Polarity active Low */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SNKEN, MASK_CLR);
}
void UTCPD_vbus_snken_polarity_active_high(int port)
{
    /* Set VBUS SNKEN Polarity Active high */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_SNKEN, MASK_SET);
}

void UTCPD_vbus_disable_oc_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS, MASK_SET);
}
void UTCPD_vbus_enable_oc_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OCP_FAULT_DIS, MASK_CLR);
}

void UTCPD_vbus_discharge_polarity_active_low(int port)
{
    /* Set VBUS discharge Polarity Active low */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VBDCHG, MASK_CLR);
}
void UTCPD_vbus_discharge_polarity_active_high(int port)
{
    /* Set VBUS discharge Polarity Active high */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_VBDCHG, MASK_SET);
}

/**
  * @brief      Configure VBUS OC source detection
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  u32Src              Specify VBUS OC Source Detection
  *             - \ref UTCPD_VBUS_OC_EINT0
  *             - \ref UTCPD_VBUS_OC_EINT1
  *             - \ref UTCPD_VBUS_OC_EINT2
  *             - \ref UTCPD_VBUS_OC_EINT3
  *             - \ref UTCPD_VBUS_OC_ADC_CMP0
  *             - \ref UTCPD_VBUS_OC_ACMP0
  *             - \ref UTCPD_VBUS_OC_ACMP1
  *             - \ref UTCPD_VBUS_OC_ACMP2
  * @details
  *
  */
void UTCPD_vbus_configure_oc_soruce(int port, uint32_t u32Src)
{
    int data;
    tcpc_addr_read16(port, NULL, UTCPD_MUXSEL, &data);
    data = (data & ~VBOCS) | (u32Src << 0);
    tcpc_addr_write16(port, NULL, UTCPD_MUXSEL, data);
}

/* VBUS ovp fault */
void UTCPD_vbus_disable_ov_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS, MASK_SET);
}
void UTCPD_vbus_enable_ov_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_OVP_FAULT_DIS, MASK_CLR);
}

/* VBUS force off fault */
void UTCPD_vbus_disable_forceoff_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS, MASK_SET);
}
void UTCPD_vbus_enable_forceoff_fault(int port)
{
    tcpc_update16(port, TCPC_REG_FAULT_CTRL, TCPC_REG_FAULT_CTRL_VBUS_FORCE_OFF_DIS, MASK_CLR);
}


uint32_t UTCPD_vbus_is_source(int port)
{
    int data;
    tcpc_addr_read16(port, NULL, TCPC_REG_POWER_STATUS, &data);

    if (data & TCPC_REG_POWER_STATUS_SOURCING_VBUS)
        return 1;
    else
        return 0;
}

uint32_t UTCPD_vbus_is_sink(int port)
{
    int data;
    tcpc_addr_read16(port, NULL, TCPC_REG_POWER_STATUS, &data);

    if (data & TCPC_REG_POWER_STATUS_SINKING_VBUS)
        return 1;
    else
        return 0;
}

uint32_t UTCPD_vbus_is_source_hv(int port)
{
    int data;
    tcpc_addr_read16(port, NULL, TCPC_REG_POWER_STATUS, &data);

    if (data & TCPC_REG_POWER_STATUS_SOURCING_HIGH_VBUS)
        return 1;
    else
        return 0;
}

//=================================== for VBUS and VCONN
void UTCPD_power_enable_monitor(int port)
{
    /* 0 enable monitor */
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS, MASK_CLR);
}

void UTCPD_power_disable_monitor(int port)
{
    /* 1 disable monitor */
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_VBUS_VOL_MONITOR_DIS, MASK_SET);
}

/* VBUS enable auto discharge */
void UTCPD_power_disable_auto_discharge(int port)
{
    /* Disable Auto Discharge = 0 */
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT, MASK_CLR);
}
void UTCPD_power_enable_auto_discharge(int port)
{
    /* Enable Auto Discharge = 0 */
    tcpc_update16(port, TCPC_REG_POWER_CTRL, TCPC_REG_POWER_CTRL_AUTO_DISCHARGE_DISCONNECT, MASK_SET);
}

void UTCPD_frs_tx_polarity_active_low(int port)
{
    /* Set FRS Polarity Active low */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_FRSTX, MASK_CLR);
}
void UTCPD_frs_tx_polarity_active_high(int port)
{
    /* Set FRS Polarity Active high */
    tcpc_update16(port, TCPC_REG_PINPL, TCPC_REG_PINPL_FRSTX, MASK_SET);
}

void UTCPD_frs_mux_selection(int port, uint32_t cc1frssel, uint32_t cc2frssel)
{
    int data;
    tcpc_addr_read16(port, NULL, UTCPD_MUXSEL, &data);
    data = (data & ~(CC2FRSS | CC1FRSS)) | ((cc1frssel << 25) | (cc2frssel << 29));
    tcpc_addr_write32(port, NULL, UTCPD_MUXSEL, data);
}

/**
  * @brief      Get VBUS VBUS/VCONN Voltage Information
  *
  * @param[in]  port                Specify UTCPD port
  * @param[in]  pu16VbusVol         VBUS Voltage Information. The voltage needs to be divided less than VREF of ADC
  * @param[in]  pu16VconnVol        VCONN Voltage Information. The voltage needs to be divided less than VREF of ADC
  * @details
  *
  */
int32_t UTCPD_GetVoltagInfo(int port, uint16_t *pu16VbusVol, uint16_t *pu16VconnVol)
{
    tcpc_addr_read16(port, NULL, TCPC_REG_VBUS_VOLTAGE, pu16VbusVol);
    *pu16VbusVol &= TCPC_REG_VBUS_VOLTAGE_VBVOL;
    tcpc_addr_read16(port, NULL, UTCPD_VCVOL, pu16VconnVol);
}


/**
  * @brief      Enable specify UTCPD Controller and set Clock Divider
  *
  * @param[in]  port         Specify UTCPD port
  * @return     Always 0
  *
  * @details    The function needs to enable the I2C Controller and set proper clock divider first.
  *
  */
uint32_t UTCPD_Open(int port)
{
    /* Enable Clock */
    /* Set Clock Divider */

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Reset UTCPD  */
    SYS_ResetModule(SYS_UTCPD0RST);

    /* Workaround solution for TESTCHIP */
    outp32((uint32_t)UTCPD0_BASE + 0x308, 0xA5);
    outp32((uint32_t)UTCPD0_BASE + 0x118, 0x82648423);

    /* Enable PHY */
    /* The register IOMODE(SYS_UTCPDCTL) in TESTCHIP needs to be set to 1 */
    SYS->UTCPDCTL = SYS->UTCPDCTL | (SYS_UTCPDCTL_POREN0_Msk | SYS_UTCPDCTL_IOMODE_Msk);
    tcpc_addr_write16(port, NULL, UTCPD_PHYCTL, 0x03);

    /* Workaround solution for TESTCHIP */
    outp32((uint32_t)UTCPD0_BASE + 0x308, 0);
    UTCPD_frs_mux_selection(port, 1, 1);
    UTCPD_vconn_mux_selection(port, 1, 1);

    return 0;
}


/** @} end of group UTCPD_EXPORTED_FUNCTIONS */

/** @} end of group UTCPD_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
