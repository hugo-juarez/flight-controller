/* ============================================================================
* bn880.c
* ============================================================================
*
* @file    bn880.c
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   This library hold the APIs for the BN880 GPS sensor.
*
* @details
* This file holds the APIs to communicate and pull data form BN880 GPS sensor's
* M8030-KT GPS and HMC5883L magnetometer. Using UBX as communication for GPS.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "bn880.h"
#include <string.h>

/* =========================================================================
* Private Global Variables
* ========================================================================= */
static TaskHandle_t gps_task_handle;
static uint16_t gps_dma_rx_read_pos = 0;

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t gps_tx_dma[BN880_UBX_MAX_TX_MSG + 8U];

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t gps_rx_dma[BN880_UBX_MAX_RX_MSG];

/* =========================================================================
* Private Structs
* ========================================================================= */

typedef struct
{
    uint8_t         msg_class;
    uint8_t         id;
    uint16_t        length; // Little Endian
    uint8_t         *payload;
} BN880_UBX_Msg_t;

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880);
static FC_Status_t BN880_UBX_SendMessage(const BN880_t *bn880, const BN880_UBX_Msg_t *msg);
static uint16_t BN880_UBX_Checksum(const uint8_t *msg, uint8_t len);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BN880_Init(BN880_t *bn880)
{
    if (bn880 == NULL) return FC_ERR;

    // Assign task hanlder to private variable
    gps_task_handle = bn880->config.task_handle;

    // Initialize GPS module of BN880
    FC_Status_t status = BN880_GPS_Init(bn880);
    if ( status != FC_OK ) return status;

    return FC_OK;
}

/* =========================================================================
* Private Function
* ========================================================================= */
static FC_Status_t BN880_GPS_Init(BN880_t *bn880)
{
    /* This command mask the NMEA output and only allow UBX outputs. (0001)
     *  It also masks the input in a way that only NMEA and UBX inputs are allowed. (0003)
     *  Finally it changes the baud-rate from the default 9600 to 115200. */
    const char* cmd = "$PUBX,41,1,0003,0001,115200,0*1E\r\n";

    if ( HAL_UART_Transmit(bn880->config.uart, (uint8_t*)cmd, strlen(cmd), 1000) != HAL_OK ) return FC_UART_ERR;

    // Wait for module to switch
    vTaskDelay(pdMS_TO_TICKS(200));

    // Change baud rate to be able to get one specified above
    if ( HAL_UART_DeInit(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;
    bn880->config.uart->Init.BaudRate = 115200;
    if ( HAL_UART_Init(bn880->config.uart) != HAL_OK ) return FC_UART_ERR;

    // Let baud generator and RX line settle
    vTaskDelay(pdMS_TO_TICKS(2));

    // Initialized both DMA buffers
    memset(gps_rx_dma, 0, sizeof(gps_rx_dma));
    memset(gps_tx_dma, 0, sizeof(gps_tx_dma));

    // Initialized Circular DMA receive message
    HAL_UARTEx_ReceiveToIdle_DMA(bn880->config.uart, gps_rx_dma, BN880_UBX_MAX_RX_MSG);

    // Disable interrupt from DMA Half anc Cmplt only Idle interrupt will trigger Ex_Event Callback
    __HAL_DMA_DISABLE_IT(bn880->config.uart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(bn880->config.uart->hdmarx, DMA_IT_TC);

    // Configure change of rate from 1Hz to 10Hz

    /* The parameters passed are the following to send the message to CFG-Rate
     * [0]: measRate in ms
     * [1]: navRate ( 1 measurement per navigation solution)
     * [2]: timeRef (GPS time) */
    const uint16_t payload_rate[3] = {
        bn880->period,
        BN880_GPS_NAV_RATE,
        bn880->time_format,
    };

    const BN880_UBX_Msg_t ubx_msg = {
        .msg_class = BN880_UBX_CLASS_RATE,
        .id = BN880_UBX_ID_RATE,
        .length = BN880_UBX_LEN_RATE,
        .payload = (uint8_t*) payload_rate,
    };

    FC_Status_t status = BN880_UBX_SendMessage(bn880, &ubx_msg);
    if (status != FC_OK) return status;

    // Configure messages send through UBX

    /* The parameters passed are the following to send the message to UBX-CFG-MSG
     * [0]: msgClass (NAV class)
     * [1]: msgId (NAV-PVT message)
     * [2]: rate (Send NAV every 10Hz) */
    uint8_t payload_cfg_msg[3] = {
        BN880_UBX_CLASS_NAV_PVT,
        BN880_UBX_ID_NAV_PVT,
        BN880_GPS_NAV_RATE
    };

    const BN880_UBX_Msg_t ubx_cfg_msg = {
        .msg_class = BN880_UBX_CLASS_CFG,
        .id = BN880_UBX_ID_CFG,
        .length = BN880_UBX_LEN_CFG,
        .payload = payload_cfg_msg,
    };

    status = BN880_UBX_SendMessage(bn880, &ubx_cfg_msg);
    if (status != FC_OK) return status;

    return FC_OK;
}

static FC_Status_t BN880_UBX_SendMessage(const BN880_t *bn880, const BN880_UBX_Msg_t *msg)
{

    if (msg->length > BN880_UBX_MAX_TX_MSG) return FC_ERR;

    // DMA callback status
    uint32_t cb_status = 0;

    // Sum of uint16_t bytes of UBX message plus the length
    const uint16_t msg_length = 8 + msg->length;

    gps_tx_dma[0] = BN880_UBX_SYNC_1;
    gps_tx_dma[1] = BN880_UBX_SYNC_2;
    gps_tx_dma[2] = msg->msg_class;
    gps_tx_dma[3] = msg->id;
    gps_tx_dma[4] = msg->length;
    gps_tx_dma[5] = msg->length >> 8;

    for (size_t i = 0; i < msg->length; i++)
    {
        gps_tx_dma[6 + i] = msg->payload[i];
    }

    // Adding Checksum of all the bytes minus the checksum bytes and sync chars
    const uint16_t ubx_checksum = BN880_UBX_Checksum(&gps_tx_dma[2], msg_length - 4);

    gps_tx_dma[msg_length - 2] = ubx_checksum >> 8;
    gps_tx_dma[msg_length - 1] = ubx_checksum;

    // Send message
    if ( HAL_UART_Transmit_DMA(bn880->config.uart, gps_tx_dma, msg_length) != HAL_OK ) return FC_UART_ERR;

    // Wait for message to complete being sent
    if ( xTaskNotifyWaitIndexed(BN880_TASK_TX_NOTIFY_INDEX, UINT32_MAX, UINT32_MAX, &cb_status, pdMS_TO_TICKS(5)) == pdFALSE)
    {
        HAL_UART_AbortTransmit(bn880->config.uart);
        return FC_UART_ERR;
    }

    if (cb_status != FC_OK)
    {
        return FC_ERR;
    }

    // Wait for acknowledge bit
    if ( xTaskNotifyWaitIndexed(BN880_TASK_RX_NOTIFY_INDEX, UINT32_MAX, UINT32_MAX, &cb_status, pdMS_TO_TICKS(5)) == pdFALSE)
    {
        HAL_UART_AbortTransmit(bn880->config.uart);
        return FC_UART_ERR;
    }

    if ((cb_status & 0xFF) != FC_OK)
    {
        return FC_ERR;
    }

    // Check acknowledge message
    uint8_t ack_msg[BN880_UBX_ACK_MSG_LEN];
    uint16_t size = cb_status >> 8 & 0xFFFF;
    uint16_t i = 0;

    while (gps_dma_rx_read_pos != size)
    {
        if (i > BN880_UBX_ACK_MSG_LEN)
        {
            return FC_ERR;
        }

        ack_msg[i++] = gps_rx_dma[gps_dma_rx_read_pos];
        gps_dma_rx_read_pos = (gps_dma_rx_read_pos + 1) % BN880_UBX_MAX_RX_MSG;
    }

    if (ack_msg[0] != BN880_UBX_SYNC_1 || ack_msg[1] != BN880_UBX_SYNC_2)
    {
        return FC_ERR;
    }

    if (ack_msg[3] != 0x01)
    {
        return FC_ERR;
    }

    return FC_OK;
}

static uint16_t BN880_UBX_Checksum(const uint8_t *msg, const uint8_t len)
{
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (size_t i = 0; i < len; i++)
    {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }

    return (uint16_t)(ck_a) << 8 | ck_b;
}

/* =========================================================================
* Callback APIs
* ========================================================================= */
void BN880_TxCmplt_Callback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyIndexedFromISR(gps_task_handle, BN880_TASK_TX_NOTIFY_INDEX, FC_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void BN880_RxCmplt_Callback(uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t value = size << 8 | FC_OK;
    xTaskNotifyIndexedFromISR(gps_task_handle, BN880_TASK_RX_NOTIFY_INDEX, value, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void BN880_Error_Callback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t error = HAL_UART_GetError(huart);

    if (error & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE | HAL_UART_ERROR_NE  | HAL_UART_ERROR_PE | HAL_UART_ERROR_RTO))
    {
        // Rx Error
        xTaskNotifyIndexedFromISR(gps_task_handle, BN880_TASK_RX_NOTIFY_INDEX, FC_UART_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    } else if (error & HAL_UART_ERROR_DMA)
    {
        // Could be any DMA
        if (huart->gState == HAL_UART_STATE_ERROR)
        {
            xTaskNotifyIndexedFromISR(gps_task_handle, BN880_TASK_TX_NOTIFY_INDEX, FC_DMA_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        } else
        {
            xTaskNotifyIndexedFromISR(gps_task_handle, BN880_TASK_RX_NOTIFY_INDEX, FC_DMA_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}