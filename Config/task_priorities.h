/* ============================================================================
* task_priorities.h
 * ============================================================================
 *
 * @file    task_priorities.h
 * @author  Hugo Juarez
 * @date    2026-02-26
 * @version 0.1.0
 *
 * @brief   Holds project Tasks priorities.
 *
 * @details
 * Set tasks priorities and stack size. Look into FreeRTOSConfig for max and min
 * values.
 *
 * @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
 *             See LICENSE file in the root of the repository for details.
 * ========================================================================= */
#ifndef FLIGHT_CONTROLLER_TASK_PRIORITIES_H
#define FLIGHT_CONTROLLER_TASK_PRIORITIES_H

#define TASK_PRI_MONITOR_LED        (1)

#define STACK_MONITOR_LED           (120)

#endif //FLIGHT_CONTROLLER_TASK_PRIORITIES_H