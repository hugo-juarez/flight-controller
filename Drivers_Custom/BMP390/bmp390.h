/* ============================================================================
* bmp390.h
* ============================================================================
*
* @file    bmp390.h
* @author  Hugo Juarez
* @date    2026-03-24
* @version 0.1.0
*
* @brief   BMP390 driver to get barometric pressure data.
*
* @details
* This driver should allow to grab data from BM390 sensor for altitude PID
* controller.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_BMP390_H
#define FLIGHT_CONTROLLER_BMP390_H

#include "stm32f7xx_hal.h"

/* =========================================================================
* Enums
* ========================================================================= */
typedef enum
{
    BMP390_REG_CHIP_ID = 0x00,
    BMP390_REG_REV_ID = 0X01,
    BMP390_REG_ERR_REG = 0X02,
    BMP390_REG_STATUS = 0X03,
    BMP390_REG_PRESSURE_DATA = 0X04,
    BMP390_REG_TEMP_DATA = 0X07,
    BMP390_REG_SENSOR_TIME = 0X0C,
    BMP390_REG_EVENT = 0X10,
    BMP390_REG_INT_STATUS = 0X11,
    BMP390_REG_INT_CTRL = 0X19,
    BMP390_REG_IF_CONF = 0X1A,
    BMP390_REG_PWR_CTRL = 0X1B,
    BMP390_REG_OSR = 0X1C,
    BMP390_REG_ODR = 0X1D,
    BMP390_REG_CONFIG = 0X1F,
    BMP390_REG_CMD = 0X7E,
} BMP390_Reg_t;

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    SPI_HandleTypeDef       *spi;
    uint16_t                csb_pin;
    GPIO_TypeDef            *csb_port;
} BMP390_Config_t;

typedef struct
{
    BMP390_Config_t         config;
} BMP390_t;

/* =========================================================================
* Public APIs
* ========================================================================= */


#endif //FLIGHT_CONTROLLER_BMP390_H