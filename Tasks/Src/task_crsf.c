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
#include <string.h>
#include "CRSF/crsf.h"
#include "fc_types.h"

void task_crsf(void *params)
{
    FC_App_Context_t *context = (FC_App_Context_t *)params;

    CRSF_t crsf = {
        .task_handle = xTaskGetCurrentTaskHandle(),
        .uart = context->hw->huart_crsf,
    };

    CRSF_Data_t data = {0};

    FC_Status_t status = CRSF_Init(&crsf);
    configASSERT(status == FC_OK);

    while (1)
    {
        uint32_t notify_val;

        xTaskNotifyWaitIndexed(CRSF_TASK_NOTIFY_INDEX, UINT32_MAX, UINT32_MAX, &notify_val, portMAX_DELAY);

        if ((notify_val & 0xFF) != FC_OK) continue;


        const uint16_t end_pos = (uint16_t) (notify_val >> 8 & 0xFFFF);
        if ( CRSF_Parse(&data, end_pos) != FC_OK) continue;

        // Now depending on the data we get is the action but right now the only one I am interested in is byte 16
        // which is the radio controller channel data

        if (data.type != CRSF_TYPE_RC_CH) continue;

        status = CRSF_Unpack_Channel(&data, &context->state->rc_ch);
        if (status != FC_OK) continue;

        status = CRSF_Print_Channel(context->hw->huart_print, &context->state->rc_ch);
        configASSERT(status == FC_OK);
    }
}