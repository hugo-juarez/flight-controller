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
    FC_I2C_ERR,
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
    struct
    {
        float               x;
        float               y;
        float               z;
    } accel;
    struct
    {
        float               x;
        float               y;
        float               z;
    } gyro;
} FC_IMU_Data_t;

typedef struct
{
    uint16_t                ch_1:   11;
    uint16_t                ch_2:   11;
    uint16_t                ch_3:   11;
    uint16_t                ch_4:   11;
    uint16_t                ch_5:   11;
    uint16_t                ch_6:   11;
    uint16_t                ch_7:   11;
    uint16_t                ch_8:   11;
    uint16_t                ch_9:   11;
    uint16_t                ch_10:  11;
    uint16_t                ch_11:  11;
    uint16_t                ch_12:  11;
    uint16_t                ch_13:  11;
    uint16_t                ch_14:  11;
    uint16_t                ch_15:  11;
    uint16_t                ch_16:  11;
} __packed FC_RC_Ch_Data_t;

typedef struct
{
    SPI_HandleTypeDef       *hspi_imu;
    I2C_HandleTypeDef       *hi2c_mag;
    UART_HandleTypeDef      *huart_crsf;
    UART_HandleTypeDef      *huart_gps;
    UART_HandleTypeDef      *huart_print;
} FC_Hw_t;

typedef struct
{
    FC_IMU_Data_t           imu;
    FC_RC_Ch_Data_t         rc_ch;
} FC_State_t;

typedef struct
{
    FC_Hw_t                *hw;
    FC_State_t             *state;
} FC_App_Context_t;

#endif //FLIGHT_CONTROLLER_FC_TYPES_H