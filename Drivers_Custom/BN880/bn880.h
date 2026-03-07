/* ============================================================================
* bn880.h
* ============================================================================
*
* @file    bn880.h
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
#ifndef FLIGHT_CONTROLLER_BN880_H
#define FLIGHT_CONTROLLER_BN880_H

#include "fc_types.h"
#include "stm32f7xx_hal.h"

/* =========================================================================
* Macros
* ========================================================================= */

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    UART_HandleTypeDef      *uart;
    I2C_HandleTypeDef       *i2c;
} BN880_Config_t;

typedef struct
{
    BN880_Config_t      config;
} BN880_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BN880_Init(void);


#endif //FLIGHT_CONTROLLER_BN880_H