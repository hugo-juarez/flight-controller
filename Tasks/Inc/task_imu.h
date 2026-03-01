/* ============================================================================
* task_imu.h
* ============================================================================
*
* @file    task_imu.h
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
#ifndef FLIGHT_CONTROLLER_TASK_IMU_H
#define FLIGHT_CONTROLLER_TASK_IMU_H

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_imu(void *params);

#endif //FLIGHT_CONTROLLER_TASK_IMU_H