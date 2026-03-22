/* ============================================================================
* task_navigation.c
* ============================================================================
*
* @file    task_navigation.c
* @author  Hugo Juarez
* @date    2026-03-06
* @version 0.1.0
*
* @brief   LIbrary that contains GPS and navigation tasks.
*
* @details
* This file include GPS and navigation related FreeRTOS tasks.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "task_navigation.h"
#include "fc_types.h"
#include "BMI088/bmi088.h"
#include "BN880/bn880.h"

/* =========================================================================
* Public APIs
* ========================================================================= */
void task_navigation(void *params)
{
    FC_App_Context_t *context = (FC_App_Context_t *)params;

    uint32_t cb_status = 0;
    BN880_GPS_NAV_PVT_t nav_pvt = {0};
    BN880_Mag_Data_t mag_data = {0};

    const BN880_Config_t config = {
        .task_handle = xTaskGetCurrentTaskHandle(),
        .i2c = context->hw->hi2c_mag,
        .uart = context->hw->huart_gps,
    };

    BN880_t bn880 = {
        .config = config,
        .gps_period = BN880_GPS_PERIOD_10HZ,
        .gps_time_format = BN880_GPS_TIME_GPS,
        .mag_dor = BN880_MAG_DOR_15,
        .mag_meas_mode = BN880_MAG_MEAS_MODE_NORMAL,
        .mag_smp_avg = BN880_MAG_SMP_AVG_1,
        .mag_gain = BN880_MAG_GAIN_1_3,
        .mag_mode = BN880_MAG_MODE_CONTINUOUS,
    };

    // 1s Delay before starting to grab useful data
    vTaskDelay(pdMS_TO_TICKS(1000));

    FC_Status_t status = BN880_Init(&bn880);
     configASSERT(status == FC_OK);

    while (1)
    {
        // Wait for acknowledge bit
        if (xTaskNotifyWaitIndexed(BN880_TASK_GPS_RX_NOTIFY_INDEX, UINT32_MAX, UINT32_MAX, &cb_status, pdMS_TO_TICKS(100)) != pdFALSE)
        {
            if ((cb_status & 0xFF) == FC_OK)
            {
                const uint16_t end_pos = (uint16_t) (cb_status >> 8 & 0xFFFF);
                BN880_GPS_Parse(&nav_pvt, end_pos);
            }
        }

        BN880_Mag_Parse(&bn880, &mag_data);

    }
}