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
#define BN880_UBX_MAX_PAYLOAD       172
#define BN880_UBX_MAX_MSG_LEN       (172 + 8)
#define BN880_UBX_SYNC_1            0xB5
#define BN880_UBX_SYNC_2            0x62
#define BN880_UBX_CLASS_CFG         0x06
#define BN880_UBX_ID_CFG            0x01
#define BN880_UBX_LEN_CFG           3
#define BN880_UBX_CLASS_RATE        0x06
#define BN880_UBX_ID_RATE           0x08
#define BN880_UBX_LEN_RATE          6
#define BN880_UBX_CLASS_NAV_PVT     0x01
#define BN880_UBX_ID_NAV_PVT        0x07

/* =========================================================================
* Enums
* ========================================================================= */
typedef enum
{
    BN880_GPS_RATE_1HZ = 1000,
    BN880_GPS_RATE_10HZ = 100,
} BN880_GPS_Rate_t;

typedef enum
{
    BN880_GPS_TIME_UTC = 0,
    BN880_GPS_TIME_GPS = 1,
    BN880_GPS_TIME_GLONASS = 2,
    BN880_GPS_TIME_BEIDOU = 3,
    BN880_GPS_TIME_GALILEO = 4,
} BN880_GPS_Time_t;

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
    BN880_GPS_Rate_t    rate;
    uint16_t            nav_rate;
    BN880_GPS_Time_t    time_format;
} BN880_Settings_t;

typedef struct
{
    BN880_Config_t      config;
    BN880_Settings_t    settings;
} BN880_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BN880_Init(BN880_t *bn880);


#endif //FLIGHT_CONTROLLER_BN880_H