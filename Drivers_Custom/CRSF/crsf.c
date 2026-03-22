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
#include <string.h>

/* =========================================================================
* Private Global Variables
* ========================================================================= */
static TaskHandle_t crsf_task_handle;
static uint16_t crsf_read_pos = 0;

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t crsf_dma[CRSF_DMA_BUF_SIZE];

static uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};


/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static uint8_t CRSF_Calc_CRC(const uint8_t *data, uint8_t len);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t CRSF_Init(CRSF_t *crsf)
{
    if (crsf == NULL) return FC_NULL_PTR_ERR;

    // Set global task handle
    crsf_task_handle = crsf->task_handle;

    //Set buffer empty
    memset(crsf_dma, 0, sizeof(crsf_dma));

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

FC_Status_t CRSF_Parse(CRSF_Data_t *data, uint16_t end_pos)
{
    if ( data == NULL ) return FC_NULL_PTR_ERR;

    // Calculate how many bytes are available without advancing the pointer
    const uint16_t available = (end_pos - crsf_read_pos + CRSF_DMA_BUF_SIZE) % CRSF_DMA_BUF_SIZE;

    // If data less or greater than min/max frame length including sync and frame length bytes then error in receiving
    if (available < (CRSF_MIN_FRAME_LENGTH + 2U) || available > (CRSF_MAX_FRAME_LENGTH + 2U) )
    {
        return FC_ERR;
    }

    // Pass data into a local buffer
    uint8_t buffer[CRSF_MAX_FRAME_LENGTH + 2U];
    uint16_t i = 0;

    while (crsf_read_pos != end_pos)
    {
        buffer[i++] = crsf_dma[crsf_read_pos];
        crsf_read_pos = (crsf_read_pos + 1) % CRSF_DMA_BUF_SIZE;
    }

    if (buffer[0] != CRSF_SYNC_BYTE)
    {
        return FC_ERR;
    }

    const uint8_t length = buffer[1];

    // Check if length matches available data or is less than the min or max
    if (length != available - 2U || length < CRSF_MIN_FRAME_LENGTH || length > CRSF_MAX_FRAME_LENGTH)
    {
        return FC_ERR;
    }

    // Get CRC value based on frame from type to before crc value
    const uint8_t crc = CRSF_Calc_CRC(&buffer[2], length - 1U);

    // If CRC values don't match return error
    if (crc != buffer[length + 1U])
    {
        return FC_ERR;
    }

    // Set values on data struct
    data->frame_length = length;
    data->type = buffer[2];
    // Copy everything except the type since i added it before and CRC
    memcpy(data->payload, &buffer[3], length - 2U);

    return FC_OK;
}

FC_Status_t CRSF_Unpack_Channel(CRSF_Data_t *data, FC_RC_Ch_Data_t *rc_ch_data)
{
    if (data == NULL || rc_ch_data == NULL) return FC_NULL_PTR_ERR;

    const uint8_t *payload = data->payload;

    rc_ch_data->ch_1  = (payload[0]  | payload[1]  << 8) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_2  = (payload[1]  >> 3 | payload[2]  << 5) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_3  = (payload[2]  >> 6 | payload[3]  << 2 | payload[4] << 10)& CRSF_RC_CH_MASK;
    rc_ch_data->ch_4  = (payload[4]  >> 1 | payload[5]  << 7) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_5  = (payload[5]  >> 4 | payload[6]  << 4) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_6  = (payload[6]  >> 7 | payload[7]  << 1 | payload[8] << 9) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_7  = (payload[8]  >> 2 | payload[9]  << 6) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_8  = (payload[9]  >> 5 | payload[10] << 3) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_9  = (payload[11] | payload[12] << 8) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_10 = (payload[12] >> 3 | payload[13] << 5) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_11 = (payload[13] >> 6 | payload[14] << 2 | payload[15]<<10) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_12 = (payload[15] >> 1 | payload[16] << 7) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_13 = (payload[16] >> 4 | payload[17] << 4) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_14 = (payload[17] >> 7 | payload[18] << 1 | payload[19]<<9)  & CRSF_RC_CH_MASK;
    rc_ch_data->ch_15 = (payload[19] >> 2 | payload[20] << 6) & CRSF_RC_CH_MASK;
    rc_ch_data->ch_16 = (payload[20] >> 5 | payload[21] << 3) & CRSF_RC_CH_MASK;

    return FC_OK;
}

/* =========================================================================
* Private Function
* ========================================================================= */
static uint8_t CRSF_Calc_CRC(const uint8_t *data, const uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ data[i]];
    return crc;
}

/* =========================================================================
* Callback APIs
* ========================================================================= */
void CRSF_RxCplt_Callback(uint16_t end_pos)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t value = (uint32_t) end_pos << 8 | FC_OK;
    xTaskNotifyIndexedFromISR(crsf_task_handle, CRSF_TASK_NOTIFY_INDEX, value, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void CRSF_Error_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyIndexedFromISR(crsf_task_handle, CRSF_TASK_NOTIFY_INDEX, FC_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}