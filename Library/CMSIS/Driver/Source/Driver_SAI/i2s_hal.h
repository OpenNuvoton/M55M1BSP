#ifndef __SAI_HAL_H
#define __SAI_HAL_H

#include <stdint.h>

//------------------------------------------------------------------------------
#define SAI_OP_ENABLE                       (1)
#define SAI_OP_DISABLE                      (0)

// I2S flags
#define SAI_FLAG_INITIALIZED                (1U)
#define SAI_FLAG_POWERED                    (1U << 1)
#define SAI_FLAG_CONFIGURED                 (1U << 2)

#define RX_ZERO_SKIP_LIMIT                  (4)

#define ARM_SAI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 7)   // driver version

#define SAI_RES_NAME_INNER(n)               sai##n##_res
#define SAI_RES_NAME(n)                     SAI_RES_NAME_INNER(n)

#define SAI_DRIVER_NAME_INNER(n)            Driver_SAI##n
#define SAI_DRIVER_NAME(n)                  SAI_DRIVER_NAME_INNER(n)

#ifndef SAI_CAT2
    #define SAI_CAT2_INNER(a, b)            a##b
    #define SAI_CAT2(a, b)                  SAI_CAT2_INNER(a, b)
#endif

#ifndef SAI_CAT3
    #define SAI_CAT3_INNER(a, b, c)         a##b##c
    #define SAI_CAT3(a, b, c)               SAI_CAT3_INNER(a,b,c)
#endif

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                        \
    static  ARM_DRIVER_VERSION   I2S##n##_GetVersion      (void);                                               \
    static  ARM_SAI_CAPABILITIES I2S##n##_GetCapabilities (void);                                               \
    static  int32_t              I2S##n##_Initialize      (ARM_SAI_SignalEvent_t cb_event);                     \
    static  int32_t              I2S##n##_Uninitialize    (void);                                               \
    static  int32_t              I2S##n##_PowerControl    (ARM_POWER_STATE state);                              \
    static  int32_t              I2S##n##_Send            (const void *data, uint32_t num);                     \
    static  int32_t              I2S##n##_Receive         (void *data, uint32_t num);                           \
    static  uint32_t             I2S##n##_GetTxCount      (void);                                               \
    static  uint32_t             I2S##n##_GetRxCount      (void);                                               \
    static  int32_t              I2S##n##_Control         (uint32_t control, uint32_t arg1, uint32_t arg2);     \
    static  ARM_SAI_STATUS       I2S##n##_GetStatus       (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                                                 \
    static  ARM_DRIVER_VERSION   I2S##n##_GetVersion      (void)                                              { return SAIn_GetVersion(); }                             \
    static  ARM_SAI_CAPABILITIES I2S##n##_GetCapabilities (void)                                              { return SAIn_GetCapabilities(); }                        \
    static  int32_t              I2S##n##_Initialize      (ARM_SAI_SignalEvent_t cb_event)                    { return SAIn_Initialize  (n, cb_event); }                \
    static  int32_t              I2S##n##_Uninitialize    (void)                                              { return SAIn_Uninitialize(n); }                          \
    static  int32_t              I2S##n##_PowerControl    (ARM_POWER_STATE state)                             { return SAIn_PowerControl(n, state); }                   \
    static  int32_t              I2S##n##_Send            (const void *data, uint32_t num)                    { return SAIn_Send        (n, data, num); }               \
    static  int32_t              I2S##n##_Receive         (void *data, uint32_t num)                          { return SAIn_Receive     (n, data, num); }               \
    static  uint32_t             I2S##n##_GetTxCount      (void)                                              { return SAIn_GetTxDataCount(n); }                        \
    static  uint32_t             I2S##n##_GetRxCount      (void)                                              { return SAIn_GetRxDataCount(n); }                        \
    static  int32_t              I2S##n##_Control         (uint32_t control, uint32_t arg1, uint32_t arg2)    { return SAIn_Control     (n, control, arg1, arg2); }     \
    static  ARM_SAI_STATUS       I2S##n##_GetStatus       (void)                                              { return SAIn_GetStatus   (n); }


// Macro for defining driver structures (for instances)
#define I2S_DRIVER(n)                   \
    ARM_DRIVER_SAI SAI_DRIVER_NAME(n) = {    \
                                             I2S##n##_GetVersion,        \
                                             I2S##n##_GetCapabilities,   \
                                             I2S##n##_Initialize,        \
                                             I2S##n##_Uninitialize,      \
                                             I2S##n##_PowerControl,      \
                                             I2S##n##_Send,              \
                                             I2S##n##_Receive,           \
                                             I2S##n##_GetTxCount,        \
                                             I2S##n##_GetRxCount,        \
                                             I2S##n##_Control,           \
                                             I2S##n##_GetStatus          \
                                        };

//------------------------------------------------------------------------------
typedef struct
{
    int32_t i32RxChnId;         // PDMA RX channel ID
    int32_t i32TxChnId;         // PDMA TX channel ID
    uint32_t u32RxUsed;         // Use PDMA for RX transfer
    uint32_t u32TxUsed;         // Use PDMA for TX transfer
    uint32_t u32RxPort;         // PDMA RX port
    uint32_t u32TxPort;         // PDMA TX port
    uint32_t u32RxChn;          // PDMA RX channel
    uint32_t u32TxChn;          // PDMA TX channel
    uint32_t u32RxPerIpMode;    // PDMA RX peripheral transfer mode
    uint32_t u32TxPerIpMode;    // PDMA TX peripheral transfer mode
} I2S_PDMA_INFO;

// I2S Stream Information (Run-Time)
typedef struct
{
    uint32_t u32Num;                // Total number of data to be transmited/received
    uint8_t *pu8Buf;                // Pointer to data buffer
    uint32_t u32Cnt;                // Number of data transmited/receive
} I2S_STREAM_INFO;

typedef struct
{
    uint8_t u8TxBusy;               // Transmitter busy flag
    uint8_t u8RxBusy;               // Receiver busy flag
    uint8_t u8TxUnderflow;          // Transmit data underflow detected (cleared on start of next send operation)
    uint8_t u8RxOverflow;           // Receive data overflow detected (cleared on start of next receive operation)
    uint8_t u8FrameError;           // Sync Frame error detected (cleared on start of next send/receive operation)
} I2S_STATUS;

// I2S Information (Run-Time)
typedef struct
{
    ARM_SAI_SignalEvent_t cb_event; // Event callback
    I2S_STATUS sStatus;             // Status flags
    I2S_STREAM_INFO sTx;            // Transmit information
    I2S_STREAM_INFO sRx;            // Receive information
    uint8_t u8Flags;                // I2S driver flags
} I2S_INFO;

// I2S Reseurces definitions
typedef struct
{
    void *phi2s;                                        // Pointer to I2S peripheral
    I2S_INFO sInfo;                 // Run-Time information
    I2S_PDMA_INFO spdma;            // PDMA information
} I2S_RESOURCES;

#endif /* __SAI_HAL_H */
