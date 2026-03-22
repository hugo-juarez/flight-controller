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
#define CRSF_MIN_FRAME_LENGTH       2U
#define CRSF_MAX_FRAME_LENGTH       62U
#define CRSF_SYNC_BYTE              0xC8
#define CRSF_TYPE_RC_CH             0x16
#define CRSF_RC_CH_MASK             0x07FF

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    uint8_t         frame_length;
    uint8_t         type;
    uint8_t         payload[CRSF_MAX_FRAME_LENGTH];
} CRSF_Data_t;

typedef struct
{
    TaskHandle_t            task_handle;
    UART_HandleTypeDef      *uart;
} CRSF_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t CRSF_Init(CRSF_t *crsf);
FC_Status_t CRSF_Parse(CRSF_Data_t *data, uint16_t end_pos);
FC_Status_t CRSF_Unpack_Channel(CRSF_Data_t *data, FC_RC_Ch_Data_t *rc_ch_data);
FC_Status_t CRSF_Print_Channel(UART_HandleTypeDef *print_uart, const FC_RC_Ch_Data_t *rc_ch_data);
/* =========================================================================
* Callback APIs
* ========================================================================= */
void CRSF_RxCplt_Callback(uint16_t end_pos);
void CRSF_Error_Callback(void);

#endif //FLIGHT_CONTROLLER_CRSF_H