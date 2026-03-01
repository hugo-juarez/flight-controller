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
#include <FreeRTOS.h>
#include <task.h>
#include "BMI088/bmi088.h"
#include "fc_types.h"
#include "pin_map.h"

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_imu(void *params)
{
    FC_Hw_t *hw = (FC_Hw_t *)params;

    BMI088_Config_t config = {
        .spi = hw->hspi_imu,
        .csb1_pin = CSB1_Pin,
        .csb1_port = CSB1_GPIO_Port,
        .csb2_pin = CSB2_Pin,
        .csb2_port = CSB2_GPIO_Port,
    };

    BMI088_t bmi088 = {
        .bmi088_config = config,
    };

    FC_Status_t status = FC_ERR;

    status = BMI088_Init(&bmi088);
    configASSERT(status == FC_OK);

    while (1)
    {

    }
}