/* ============================================================================
* task_monitor.c
* ============================================================================
*
* @file    task_monitor.c
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
#include "task_monitor.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "usbd_cdc_if.h"
#include "pin_map.h"


/* =========================================================================
* Public Functions
* ========================================================================= */
void task_monitor_led_run(void* params)
{
    TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_PIN);
        xTaskDelayUntil(&xLastWakeTime, xDelay );
    }
}

void task_monitor_usb(void* params)
{
    char buf[64];
    uint32_t count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        int len = snprintf(buf, sizeof(buf), "usb ok: %lu\r\n", (unsigned long)count++);

        while (CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len) == USBD_BUSY)
            vTaskDelay(pdMS_TO_TICKS(1));

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}