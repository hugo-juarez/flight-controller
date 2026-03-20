/* ============================================================================
* crsf.c
* ============================================================================
*
* @file    crsf.c
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
#include "crsf.h"

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t crsf_dma[CRSF_DMA_BUF_SIZE];


/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t CRSF_Init(CRSF_t *crsf)
{
    if (crsf == NULL) return FC_NULL_PTR_ERR;

    // Enable DMA reading till IDLE interrupt
    if ( HAL_UARTEx_ReceiveToIdle_DMA(crsf->uart, crsf_dma, CRSF_DMA_BUF_SIZE) != HAL_OK)
    {
        return FC_UART_ERR;
    }

    // Disable DMA Half and Complete interrupts; only Idle interrupt will trigger Ex_Event Callback
    __HAL_DMA_DISABLE_IT(crsf->uart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(crsf->uart->hdmarx, DMA_IT_TC);

    return FC_OK;
}