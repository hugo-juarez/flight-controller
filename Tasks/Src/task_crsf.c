/* ============================================================================
* task_crsf.c
* ============================================================================
*
* @file    task_crsf.c
* @author  Hugo Juarez
* @date    2026-03-05
* @version 0.1.0
*
* @brief   Task that gets the radio control signal and sets actions on it.
*
* @details
* This task helps get radio control data, parse it, send information and act on it.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "task_crsf.h"
#include <FreeRTOS.h>
#include <task.h>
#include "fc_types.h"

void task_crsf(void *params)
{
    FC_Hw_t *hw = (FC_Hw_t *)params;

    while (1)
    {
        
    }
}