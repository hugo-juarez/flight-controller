/* ============================================================================
* fc_types.h
* ============================================================================
*
* @file    fc_types.h
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   Hold enums/structures/definitions used accross the project.
*
* @details
* Holds definition of enums/structures/defintions that are common to the
* project and it's libraries.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_FC_TYPES_H
#define FLIGHT_CONTROLLER_FC_TYPES_H

#include "stm32f7xx_hal.h"

/* =========================================================================
* Enums
* ========================================================================= */

typedef enum
{
    FC_OK = 0,
    FC_ERR,
    FC_NULL_PTR_ERR,
    FC_SPI_ERR,
    FC_UART_ERR,
    FC_DMA_ERR,
    FC_CONFIG_ERR,
    FC_ERR_TIMEOUT,
} FC_Status_t;

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    float ax, ay, az; /* Accelerometer data in m/s^2 */
    float gx, gy, gz; /* Gyro data in deg/s */
} FC_IMU_Data_t;

typedef struct
{
    SPI_HandleTypeDef       *hspi_imu;
    I2C_HandleTypeDef       *hi2c_mag;
    UART_HandleTypeDef      *huart_crsf;
    UART_HandleTypeDef      *huart_gps;
    UART_HandleTypeDef      *huart_print;
    FC_IMU_Data_t           accel_data;
} FC_Hw_t;

#endif //FLIGHT_CONTROLLER_FC_TYPES_H