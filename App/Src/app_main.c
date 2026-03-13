/* ============================================================================
* app_main.c
* ============================================================================
*
* @file    app_main.c
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   One line summary of what this file does.
*
* @details
* More detailed description of the module, its responsibilities,
* and how it fits into the overall firmware architecture.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */

/* =========================================================================
* Includes
* ========================================================================= */

#include "app_main.h"
#include <FreeRTOS.h>
#include <task.h>
#include "task_imu.h"
#include "task_crsf.h"
#include "task_navigation.h"
#include "task_monitor.h"
#include "task_priorities.h"
#include "BN880/bn880.h"

/* =========================================================================
 * Public APIs
 * ========================================================================= */
FC_Status_t app_init(FC_Hw_t *hw)
{
    BaseType_t status = pdFAIL;

    status = xTaskCreate(task_imu, "IMU Task", STACK_IMU, (void*)hw, TASK_PRI_IMU, NULL);
    if (status != pdPASS) return FC_ERR;

    status = xTaskCreate(task_navigation, "NAV Task", STACK_NAVIGATION, (void*)hw, TASK_PRI_NAVIGATION, NULL);
    if (status != pdPASS) return FC_ERR;

    status = xTaskCreate(task_monitor_led_run, "LED Monitor Task", STACK_MONITOR_LED, NULL, TASK_PRI_MONITOR_LED, NULL);
    if (status != pdPASS) return FC_ERR;

    return FC_OK;
}

FC_Status_t app_start(void)
{
    vTaskStartScheduler();

    // Would only return if vTaskStartScheduler crashes
    return FC_ERR;
}

/* =========================================================================
* Callback APIs
* ========================================================================= */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GPS_UART)
    {
        BN880_TxCmplt_Callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == GPS_UART)
    {
        BN880_RxCmplt_Callback(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GPS_UART)
    {
        BN880_Error_Callback(huart);
    }
}