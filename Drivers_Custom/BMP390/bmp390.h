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