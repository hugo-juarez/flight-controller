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
#include <FreeRTOS.h>
#include <task.h>

/* =========================================================================
* Private Function Structs
* ========================================================================= */

typedef struct
{
    uint8_t         msg_class;
    uint8_t         id;
    uint16_t        length; // Little Endian
    uint8_t         *payload;
} PACKED BN880_UBX_Msg_t;

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880);
static FC_Status_t BN880_UBX_SendMessage(const BN880_t *bn880, BN880_UBX_Msg_t *msg);
static uint16_t BN880_UBX_Checksum(const uint8_t *msg, uint8_t len);

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

    // Wait for module to switch
    vTaskDelay(pdMS_TO_TICKS(200));

    // Change baud rate to be able to get data from
    if ( HAL_UART_DeInit(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;
    bn880->config.uart->Init.BaudRate = 115200;
    if ( HAL_UART_Init(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;

    // Configure change of rate from 1Hz to 10Hz

    /* The parameters passed are the following to send the message to CFG-Rate
     * [0]: measRate in ms
     * [1]: navRate ( 1 measurement per navigation solution)
     * [2]: timeRef (GPS time) */
    const uint16_t payload_rate[3] = {
        bn880->settings.rate,
        bn880->settings.nav_rate,
        bn880->settings.time_format,
    };

    BN880_UBX_Msg_t ubx_msg = {
        .msg_class = BN880_UBX_CLASS_RATE,
        .id = BN880_UBX_ID_RATE,
        .length = BN880_UBX_LEN_RATE,
        .payload = (uint8_t*) payload_rate,
    };

    FC_Status_t status = BN880_UBX_SendMessage(bn880, &ubx_msg);
    if (status != FC_OK) return status;

    // Configure messages send through UBX

    /* The parameters passed are the following to send the message to UBX-CFG-MSG
     * [0]: msgClass (NAV class)
     * [1]: msgId (NAV-PVT message)
     * [3]: rate (Send NAV every 10Hz) */
    uint8_t payload_cfg_msg[3] = {
        BN880_UBX_CLASS_NAV_PVT,
        BN880_UBX_ID_NAV_PVT,
        bn880->settings.nav_rate
    };

    BN880_UBX_Msg_t ubx_cfg_msg = {
        .msg_class = BN880_UBX_CLASS_CFG,
        .id = BN880_UBX_ID_CFG,
        .length = BN880_UBX_LEN_CFG,
        .payload = payload_cfg_msg,
    };

    status = BN880_UBX_SendMessage(bn880, &ubx_cfg_msg);
    if (status != FC_OK) return status;

    return FC_OK;
}

static FC_Status_t BN880_UBX_SendMessage(const BN880_t *bn880, BN880_UBX_Msg_t *msg)
{

    // Sum of uint8_t bytes of UBX message plus the length
    const uint8_t msg_length = 8 + msg->length;

    // Load message
    uint8_t uart_msg[msg_length];

    uart_msg[0] = BN880_UBX_SYNC_1;
    uart_msg[1] = BN880_UBX_SYNC_2;
    uart_msg[2] = msg->msg_class;
    uart_msg[3] = msg->id;
    uart_msg[4] = msg->length;
    uart_msg[5] = msg->length >> 8;

    for (size_t i = 0; i < msg->length; i++)
    {
        uart_msg[6 + i] = msg->payload[i];
    }

    // Adding Checksum of all the bytes minus the checksum bytes and sync chars
    const uint16_t ubx_checksum = BN880_UBX_Checksum(&uart_msg[2], msg_length - 4);

    uart_msg[msg_length - 2] = ubx_checksum >> 8;
    uart_msg[msg_length - 1] = ubx_checksum;

    // Send message
    if ( HAL_UART_Transmit(bn880->config.uart, uart_msg, msg_length, 1000) != HAL_OK ) return FC_UART_ERR;

    return FC_OK;
}

static uint16_t BN880_UBX_Checksum(const uint8_t *msg, const uint8_t len)
{
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 0; i < len; i++)
    {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }

    return ck_a << 8 | ck_b;
}