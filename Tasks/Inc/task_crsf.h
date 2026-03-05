/* ============================================================================
* task_crsf.h
 * ============================================================================
 *
 * @file    task_crsf.h
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


#ifndef FLIGHT_CONTROLLER_TASK_CRSF_H
#define FLIGHT_CONTROLLER_TASK_CRSF_H

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_crsf(void *params);

#endif //FLIGHT_CONTROLLER_TASK_CRSF_H