/* ============================================================================
* bmp390.h
* ============================================================================
*
* @file    bmp390.h
* @author  Hugo Juarez
* @date    2026-03-24
* @version 0.1.0
*
* @brief   BMP390 driver to get barometric pressure data.
*
* @details
* This driver should allow to grab data from BM390 sensor for altitude PID
* controller.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "bmp390.h"
#include <FreeRTOS.h>
#include <task.h>
#include "task.h"

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
FC_Status_t BMP390_ReadRegister(BMP390_t *bmp390, BMP390_Reg_t reg, uint8_t *data);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BMP390_Init(BMP390_t *bmp390)
{
    // Ensure CSB is deselected before init
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_SET);

    // Delay for POR
    vTaskDelay(pdMS_TO_TICKS(3));

    // Dummy read to initialize the register to SPI
    uint8_t dummy;
    FC_Status_t status = BMP390_ReadRegister(bmp390, BMP390_REG_CHIP_ID, &dummy);
    if ( status != FC_OK ) return status;

    // Delay for SPI change
    vTaskDelay(pdMS_TO_TICKS(3));

    status = BMP390_WhoAmI(bmp390);
    if (status != FC_OK) return status;

    return FC_OK;
}


FC_Status_t BMP390_WhoAmI (BMP390_t *bmp390)
{
    if (bmp390 == NULL) return  FC_NULL_PTR_ERR;

    uint8_t chip_id;
    const FC_Status_t status = BMP390_ReadRegister(bmp390, BMP390_REG_CHIP_ID, &chip_id);
    if (status != FC_OK) return status;

    if (chip_id != BMP390_CHIP_ID) return  FC_ERR;

    return FC_OK;
}

/* =========================================================================
* Private Function
* ========================================================================= */
FC_Status_t BMP390_ReadRegister(BMP390_t *bmp390, BMP390_Reg_t reg, uint8_t *data)
{
    FC_Status_t status = FC_OK;

    // Pull down CSB pin to start communication
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_RESET);

    uint8_t dummy[3] = {(uint8_t) (reg | 0x80)};
    uint8_t buffer[3];

    if ( HAL_SPI_TransmitReceive(bmp390->config.spi, dummy, buffer, 3, HAL_MAX_DELAY) != HAL_OK ) status = FC_SPI_ERR;

    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        *data = buffer[2];
    }

    return status;
}