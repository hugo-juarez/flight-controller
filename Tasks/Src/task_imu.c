/* ============================================================================
* task_imu.c
* ============================================================================
*
* @file    task_imu.c
* @author  Hugo Juarez
* @date    2026-02-28
* @version 0.1.0
*
* @brief   Task that handles IMU data read for flight controller.
*
* @details
* This task read IMU data from the BMI088 module neccessary from the flight
* controller through SPI.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "task_imu.h"
#include "fc_types.h"
#include "../../Drivers_Custom/BMI088/bmi088.h"

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_imu(void *params)
{
    FC_Hw_t *hw = (FC_Hw_t *)params;

    BMI088_Config_t config = {
        .spi = hw->hspi_imu,
    };
}