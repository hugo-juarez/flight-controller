/* ============================================================================
* bmi088.h
* ============================================================================
*
* @file    bmi088.h
* @author  Hugo Juarez
* @date    2026-02-27
* @version 0.1.0
*
* @brief   BMI088 IMU Sensor Firmware
*
* @details
* BMI088 is the IMU that would be used inside this flight controller and
* this library helps read the acceleration and gyroscope data from the
* sensor.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_BMI088_H
#define FLIGHT_CONTROLLER_BMI088_H

#include "stm32f7xx_hal.h"

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    SPI_HandleTypeDef       *spi;
    uint16_t                csb1_pin;
    GPIO_TypeDef            *csb1_port;
    uint16_t                csb2_pin;
    GPIO_TypeDef            *csb2_port;
} BMI088_Config_t;

typedef struct
{
    BMI088_Config_t         bmi088_config;
} BMI088_t;

/* =========================================================================
* Public APIs
* ========================================================================= */


#endif //FLIGHT_CONTROLLER_BMI088_H