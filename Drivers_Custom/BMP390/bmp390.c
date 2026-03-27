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
#include <string.h>

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
FC_Status_t BMP390_GetCalibData(BMP390_t *bmp390);
FC_Status_t BMP390_WriteRegister(const BMP390_t *bmp390, BMP390_Reg_t reg, uint8_t data);
FC_Status_t BMP390_ReadRegister(const BMP390_t *bmp390, BMP390_Reg_t reg, uint8_t *data);
FC_Status_t BMP390_BurstRead(const BMP390_t *bmp390, BMP390_Reg_t reg, uint8_t *data, uint8_t len);

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

    // Get calibration data
    status = BMP390_GetCalibData(bmp390);
    if (status != FC_OK) return status;

    return FC_OK;
}


FC_Status_t BMP390_WhoAmI (const BMP390_t *bmp390)
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
FC_Status_t BMP390_GetCalibData(BMP390_t *bmp390)
{
    uint8_t calib_data[21];

    FC_Status_t status = BMP390_BurstRead(bmp390, BMP390_REG_CAL_TEMP, calib_data, 21);
    if (status != FC_OK) return status;

    // Setting values to calibration data
    memcpy(&bmp390->calib.t1, &calib_data[0], 2);
    memcpy(&bmp390->calib.t2, &calib_data[2], 2);
    bmp390->calib.t3 = (int8_t) calib_data[4];
    memcpy(&bmp390->calib.p1, &calib_data[5], 2);
    memcpy(&bmp390->calib.p2, &calib_data[7], 2);
    bmp390->calib.p3 = (int8_t) calib_data[9];
    bmp390->calib.p4 = (int8_t) calib_data[10];
    memcpy(&bmp390->calib.p5, &calib_data[11], 2);
    memcpy(&bmp390->calib.p6, &calib_data[13], 2);
    bmp390->calib.p7 = (int8_t) calib_data[15];
    bmp390->calib.p8 = (int8_t) calib_data[16];
    memcpy(&bmp390->calib.p9, &calib_data[17], 2);
    bmp390->calib.p10 = (int8_t) calib_data[19];
    bmp390->calib.p11 = (int8_t) calib_data[20];

    return FC_OK;
}

FC_Status_t BMP390_WriteRegister(const BMP390_t *bmp390, const BMP390_Reg_t reg, const uint8_t data)
{
    FC_Status_t status = FC_OK;

    // Pull down CSB pin to start communication
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_RESET);

    uint8_t buffer[2] = {(uint8_t) (reg & ~0x80), data};
    uint8_t dummy[2];

    if ( HAL_SPI_TransmitReceive(bmp390->config.spi, buffer, dummy, 2, HAL_MAX_DELAY) != HAL_OK ) status = FC_SPI_ERR;

    // End communication set CSB pin high
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_SET);

    return status;
}

FC_Status_t BMP390_ReadRegister(const BMP390_t *bmp390, const BMP390_Reg_t reg, uint8_t *data)
{
    FC_Status_t status = FC_OK;

    // Pull down CSB pin to start communication
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_RESET);

    uint8_t dummy[3] = {(uint8_t) (reg | 0x80)};
    uint8_t buffer[3];

    if ( HAL_SPI_TransmitReceive(bmp390->config.spi, dummy, buffer, 3, HAL_MAX_DELAY) != HAL_OK ) status = FC_SPI_ERR;

    // End communication set CSB pin high
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        *data = buffer[2];
    }

    return status;
}

FC_Status_t BMP390_BurstRead(const BMP390_t *bmp390, const BMP390_Reg_t reg, uint8_t *data, const uint8_t len)
{
    // Since reg takes 1st bit and the second bit is empty the message length is len + 2U
    const uint8_t msg_len = len + 2U;

    if (msg_len > BMP390_MSG_MAX_LEN) return FC_ERR;

    FC_Status_t status = FC_OK;

    // Pull down CSB pin to start communication
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_RESET);

    uint8_t dummy[BMP390_MSG_MAX_LEN] = {(uint8_t) (reg | 0x80)};
    uint8_t buffer[BMP390_MSG_MAX_LEN];

    if ( HAL_SPI_TransmitReceive(bmp390->config.spi, dummy, buffer, msg_len, HAL_MAX_DELAY) != HAL_OK ) status = FC_SPI_ERR;

    // End communication set CSB pin high
    HAL_GPIO_WritePin(bmp390->config.csb_port, bmp390->config.csb_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        memcpy(data, &buffer[2], len);
    }

    return status;
}