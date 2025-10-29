#ifndef __SPI_HAL_H
#define __SPI_HAL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//------------------------------------------------------------------------------
/* Driver Capabilities */
/* Current driver status flag definition */
#define SPI_POWERED                         (1U << 1)       // SPI powered on
#define SPI_INITIALIZED                     (1U << 0)       // SPI initialized
#define SPI_CONFIGURED                      (1U << 2)       // SPI configured
#define SPI_DATA_LOST                       (1U << 3)       // SPI data lost occurred
#define SPI_MODE_FAULT                      (1U << 4)       // SPI mode fault occurred

#define SPI_OP_ENABLE                       (1)
#define SPI_OP_DISABLE                      (0)

#define SPI_ARRAY_SIZE(arr)                 (sizeof(arr) / sizeof((arr)[0]))

#define SPI_RES_NAME_INNER(n)               spi##n##_res
#define SPI_RES_NAME(n)                     SPI_RES_NAME_INNER(n)

#define SPI_DRIVER_NAME_INNER(n)            Driver_SPI##n
#define SPI_DRIVER_NAME(n)                  SPI_DRIVER_NAME_INNER(n)

#ifndef SPI_CONCAT2
    #define SPI_CAT2_INNER(a, b)            a##b
    #define SPI_CONCAT2(a, b)               SPI_CAT2_INNER(a, b)
#endif
#ifndef SPI_CONCAT3
    #define SPI_CAT3_INNER(a, b, c)         a##b##c
    #define SPI_CONCAT3(a, b, c)            SPI_CAT3_INNER(a, b, c)
#endif

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                        \
    static  ARM_DRIVER_VERSION   SPI##n##_GetVersion      (void);                                               \
    static  ARM_SPI_CAPABILITIES SPI##n##_GetCapabilities (void);                                               \
    static  int32_t              SPI##n##_Initialize      (ARM_SPI_SignalEvent_t cb_event);                     \
    static  int32_t              SPI##n##_Uninitialize    (void);                                               \
    static  int32_t              SPI##n##_PowerControl    (ARM_POWER_STATE state);                              \
    static  int32_t              SPI##n##_Send            (const void *data, uint32_t num);                     \
    static  int32_t              SPI##n##_Receive         (void *data, uint32_t num);                           \
    static  int32_t              SPI##n##_Transfer        (const void *data_out, void *data_in, uint32_t num);  \
    static  uint32_t             SPI##n##_GetDataCount    (void);                                               \
    static  int32_t              SPI##n##_Control         (uint32_t control, uint32_t arg);                     \
    static  ARM_SPI_STATUS       SPI##n##_GetStatus       (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                                                             \
    static  ARM_DRIVER_VERSION   SPI##n##_GetVersion      (void)                                              { return SPIn_GetVersion(); }                                         \
    static  ARM_SPI_CAPABILITIES SPI##n##_GetCapabilities (void)                                              { return SPIn_GetCapabilities(); }                                    \
    static  int32_t              SPI##n##_Initialize      (ARM_SPI_SignalEvent_t cb_event)                    { return SPIn_Initialize  (n, cb_event); }                \
    static  int32_t              SPI##n##_Uninitialize    (void)                                              { return SPIn_Uninitialize(n); }                          \
    static  int32_t              SPI##n##_PowerControl    (ARM_POWER_STATE state)                             { return SPIn_PowerControl(n, state); }                   \
    static  int32_t              SPI##n##_Send            (const void *data, uint32_t num)                    { return SPIn_Send        (n, data, num); }               \
    static  int32_t              SPI##n##_Receive         (void *data, uint32_t num)                          { return SPIn_Receive     (n, data, num); }               \
    static  int32_t              SPI##n##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return SPIn_Transfer    (n, data_out, data_in, num); }  \
    static  uint32_t             SPI##n##_GetDataCount    (void)                                              { return SPIn_GetDataCount(n); }                          \
    static  int32_t              SPI##n##_Control         (uint32_t control, uint32_t arg)                    { return SPIn_Control     (n, control, arg); }            \
    static  ARM_SPI_STATUS       SPI##n##_GetStatus       (void)                                              { return SPIn_GetStatus   (n); }


// Macro for defining driver structures (for instances)
#define SPI_DRIVER(n)                   \
    ARM_DRIVER_SPI SPI_DRIVER_NAME(n) = {   \
                                            SPI##n##_GetVersion,        \
                                            SPI##n##_GetCapabilities,   \
                                            SPI##n##_Initialize,        \
                                            SPI##n##_Uninitialize,      \
                                            SPI##n##_PowerControl,      \
                                            SPI##n##_Send,              \
                                            SPI##n##_Receive,           \
                                            SPI##n##_Transfer,          \
                                            SPI##n##_GetDataCount,      \
                                            SPI##n##_Control,           \
                                            SPI##n##_GetStatus          \
                                        };

//------------------------------------------------------------------------------
/* SPI status */
typedef struct
{
    uint8_t u8Busy;         // Transmitter/Receiver busy flag
    uint8_t u8DataLost;     // Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
    uint8_t u8ModeFault;    // Mode fault detected; optional (cleared on start of transfer operation)
} SPI_DRV_STATUS;

/* SPI Information (Run-time) */
typedef struct
{
    ARM_SPI_SignalEvent_t cb_event; // Event Callback
    SPI_DRV_STATUS sDrvStatus;      // Status flags
    uint32_t u32Mode;               // Current SPI mode
    uint8_t u8State;                // Current SPI state
} SPI_STATE_INFO;

/* SPI Transfer Information (Run-Time) */
typedef struct
{
    uint32_t u32Num;            // Total number of transfers
    uint8_t *pu8RxBuf;          // Pointer to in data buffer
    uint8_t *pu8TxBuf;          // Pointer to out data buffer
    uint32_t u32RxCnt;          // Number of data received
    uint32_t u32TxCnt;          // Number of data sent
    uint32_t u32DefVal;         // Default transfer value
    uint32_t u32TxPrepared;     // Prepared TX transfer value
    uint32_t u32RxPrepared;     // Prepared RX transfer value
} SPI_TRANSFER_INFO;

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
} SPI_PDMA_INFO;

typedef struct
{
    uint32_t u32XferMode;       // Full-Duplex, Half-Duplex, Simplex
    uint32_t u32BusSpeed;       // SPI Bus Speed
} SPI_CONFIG_INFO;

/* SPI Resources */
typedef struct
{
    void *phspi;                // SPI peripheral register interface
    SPI_CONFIG_INFO sConfig;    // SPI configuration information
    SPI_STATE_INFO sState;      // SPI Run-time information
    SPI_TRANSFER_INFO sXfer;    // SPI transfer information
    SPI_PDMA_INFO spdma;        // SPI PDMA information
} SPI_RESOURCES;

#endif /* __SPI_HAL_H */
