/**************************************************************************//**
 * @file     st1633i.h
 * @version  V1.00
 * @brief    st1633i touch controller driver header.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SI1633I_H__
#define __SI1633I_H__

#define ST_MAX_TOUCH             1
#include "stdint.h"
typedef struct
{
    //012H*n+0 (n=0, 1, ...,4)
    union
    {
        uint8_t m_u8XY0_H;
        struct
        {
            uint8_t u8Y0_H: 3;
            uint8_t : 1;
            uint8_t u8X0_H: 3;
            uint8_t u8Valid: 1;
        };
    };

    //012H*n+1 (n=0, 1, ...,4)
    uint8_t m_u8X0_L;

    //012H*n+2 (n=0, 1, ...,4)
    uint8_t m_u8Y0_L;

    //012H*n+3 (n=0, 1, ...,4)
    uint8_t m_u8Z;

} S_ST_TP;

#pragma pack(push)
#pragma pack(4)

//
// A structure to represent st1633 register map
// Ranges from 0x10~0x24, supports up to 5 touch points.
//
typedef struct
{
    union
    {
        uint8_t m_u8TouchInfo;
        struct
        {
            uint8_t u8Fingers: 4;
            uint8_t : 4;
        };
    };

    uint8_t m_u8Keys;

    S_ST_TP m_sTP[ST_MAX_TOUCH];

} S_ST_REGMAP;
#pragma pack(pop)

typedef struct
{
    uint16_t pos_x; //X  coordinate of touch
    uint16_t pos_y; //Y  coordinate of touch
    uint8_t   event_touched; // Touch event
    uint8_t   state; // State
} S_LVGL_TPINFO;

#ifdef  __cplusplus
extern  "C" {
#endif
void st1633i_low_level_init(void);
void st1633i_startup_sequence(void);
void st1633i_handler(void);
uint8_t st1663i_read_point(S_LVGL_TPINFO *pbuf, uint8_t read_num);
#ifdef  __cplusplus
}
#endif
#endif   /* __SI1633I_H__ */
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/