/* ============================================================================
* task_navigation.c
* ============================================================================
*
* @file    task_navigation.c
* @author  Hugo Juarez
* @date    2026-03-06
* @version 0.1.0
*
* @brief   LIbrary that contains GPS and navigation tasks.
*
* @details
* This file include GPS and navigation related FreeRTOS tasks.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "task_navigation.h"
#include "fc_types.h"
#include "BMI088/bmi088.h"
#include "BN880/bn880.h"

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_navigation(void *params)
{
    FC_Hw_t *hw = (FC_Hw_t *)params;

    const BN880_Config_t config = {
        .uart = hw->huart_gps,
    };

    const BN880_Settings_t settings = {
        .rate = BN880_GPS_RATE_10HZ,
        .nav_rate = 1,
        .time_format = BN880_GPS_TIME_GPS,
    };

    BN880_t bn880 = {
        .config = config,
        .settings = settings,
    };

    // 1s Delay before starting to grab useful data
    vTaskDelay(pdMS_TO_TICKS(1000));

    FC_Status_t status = BN880_Init(&bn880);
    configASSERT(status == FC_OK);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}