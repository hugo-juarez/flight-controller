/* ============================================================================
* task_monitor.h
* ============================================================================
*
* @file    task_monitor.h
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   This holds the monitors taks
*
* @details
* Includes tasks that monitor state of flight controller
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_TASK_MONITOR_H
#define FLIGHT_CONTROLLER_TASK_MONITOR_H

#include "fc_types.h"

/* =========================================================================
* Functions
* ========================================================================= */

void task_monitor_led_run(void* params);


#endif //FLIGHT_CONTROLLER_TASK_MONITOR_H