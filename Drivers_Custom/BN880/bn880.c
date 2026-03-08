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
* Private Function Variables
* ========================================================================= */

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880);

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

    return FC_OK;
}