/* ============================================================================
* task_barometer.c
* ============================================================================
*
* @file    task_barometer.c
* @author  Hugo Juarez
* @date    2026-03-24
* @version 0.1.0
*
* @brief   Task that gets the barometer data to get altitude control.
*
* @details
* This task helps get altitude data from BMP390 sensor to get drone altitude
* control.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "task_barometer.h"
#include "BMP390/bmp390.h"
#include "fc_types.h"
#include "pin_map.h"

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_barometer(void *params)
{
    FC_App_Context_t *context = (FC_App_Context_t *)params;

    const BMP390_Config_t config = {
        .spi = context->hw->hspi_bar,
        .csb_pin = BAROMETER_CSB_Pin,
        .csb_port = BAROMETER_CSB_Port,
    };

    BMP390_t barometer = {
        .config = config,
    };

    while (1)
    {
        UNUSED(barometer);
    }

}