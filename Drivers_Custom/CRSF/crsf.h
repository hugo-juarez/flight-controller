/* ============================================================================
* crsf.h
* ============================================================================
*
* @file    crsf.h
* @author  Hugo Juarez
* @date    2026-03-20
* @version 0.1.0
*
* @brief   This is the CRSF protocol parsing and handler driver.
*
* @details
* This driver allows to parse the crsf messages received and sent through the ELRS
* module
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_CRSF_H
#define FLIGHT_CONTROLLER_CRSF_H

#include <FreeRTOS.h>
#include <task.h>
#include "stm32f7xx_hal.h"
#include "fc_types.h"

/* =========================================================================
* Macros
* ========================================================================= */
#define CRSF_TASK_NOTIFY_INDEX      0
#define CRSF_DMA_BUF_SIZE           256U

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    TaskHandle_t            task_handle;
    UART_HandleTypeDef      *uart;
} CRSF_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t CRSF_Init(CRSF_t *crsf);

/* =========================================================================
* Callback APIs
* ========================================================================= */
void CRSF_RxCmplt_Callback(uint16_t end_pos);
void CRSF_GPS_Error_Callback(void);

#endif //FLIGHT_CONTROLLER_CRSF_H