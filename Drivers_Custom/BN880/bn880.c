/* ============================================================================
* bn880.c
* ============================================================================
*
* @file    bn880.c
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   This library hold the APIs for the BN880 GPS sensor.
*
* @details
* This file holds the APIs to communicate and pull data form BN880 GPS sensor's
* M8030-KT GPS and HMC5883L magnetometer. Using UBX as communication for GPS.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "bn880.h"
#include <string.h>


/* =========================================================================
* Private Function Structs
* ========================================================================= */

typedef struct
{
    uint16_t        sync_bytes;
    uint8_t         class;
    uint8_t         id;
    uint16_t        length; // Little Endian
    uint8_t         *payload;
    uint16_t        ubx_checksum;
} PACKED BN880_UBX_Msg_t;

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880);
static void BN880_UBX_Convert_Payload(uint16_t *payload, uint8_t len);
static uint16_t BN880_UBX_Checksum(uint8_t *payload, uint8_t len);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BN880_Init(BN880_t *bn880)
{

    // Initialize GPS module of BN880
    FC_Status_t status = BN880_GPS_Init(bn880);
    if ( status != FC_OK ) return status;

    return FC_OK;
}

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880)
{
    /* This command mask the NMEA output and only allow UBX outputs. (0001)
     *  It also masks the input in a way that only NMEA and UBX inputs are allowed. (0003)
     *  Finally it changes the baud-rate from the default 9600 to 115200. */
    const char* cmd = "$PUBX,41,1,0003,0001,115200,0*1E\r\n";

    if ( HAL_UART_Transmit(bn880->config.uart, (uint8_t*)cmd, strlen(cmd), 1000) != HAL_OK ) return FC_UART_ERR;

    // Change baud rate to be able to get data from
    if ( HAL_UART_DeInit(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;
    bn880->config.uart->Init.BaudRate = 115200;
    if ( HAL_UART_Init(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;

    // Configure change of rate from 1Hz to 10Hz

    /* The parameters passed are the following to send the message to CFG-Rate
     * [0]: measRate in ms = 100ms = 10Hz
     * [1]: navRate = 1 ( 1 measurement per navigation solution)
     * [2]: timeRef = 1 (GPS time) */
    uint16_t payload_rate[3] = {100, 1, 1};

    // Converting payload into little endian acceptable for UBX
    BN880_UBX_Convert_Payload(payload_rate, 3);

    const BN880_UBX_Msg_t ubx_msg = {
        .sync_bytes = 0xB562,
        .class = 0x06,
        .id = 0x08,
        .length = __builtin_bswap16(6),
        .payload = (uint8_t*) payload_rate,
        .ubx_checksum = BN880_UBX_Checksum((uint8_t*)payload_rate, 6),
    };

    // Sending measure rate config message
    if ( HAL_UART_Transmit(bn880->config.uart, (uint8_t*) &ubx_msg, sizeof(BN880_UBX_Msg_t), 1000) != HAL_OK ) return FC_UART_ERR;

    

    return FC_OK;
}

static void BN880_UBX_Convert_Payload(uint16_t *payload, const uint8_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        payload[i] = __builtin_bswap16(payload[i]);
    }
}

static uint16_t BN880_UBX_Checksum(uint8_t *payload, const uint8_t len)
{
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 0; i < len; i++)
    {
        ck_a = ck_a + payload[i];
        ck_b = ck_b + ck_a;
    }

    return ck_a << 8 | ck_b;
}