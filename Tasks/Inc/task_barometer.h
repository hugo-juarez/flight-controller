/* ============================================================================
* task_barometer.h
* ============================================================================
*
* @file    task_barometer.h
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
#ifndef FLIGHT_CONTROLLER_TASK_BAROMETER_H
#define FLIGHT_CONTROLLER_TASK_BAROMETER_H

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_barometer(void *params);

#endif //FLIGHT_CONTROLLER_TASK_BAROMETER_H