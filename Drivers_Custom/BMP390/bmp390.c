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
#include <math.h>

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

    // Set configuration values
    uint8_t iir = bmp390->iir;
    status = BMP390_WriteRegister(bmp390, BMP390_REG_CONFIG, iir);
    if (status != FC_OK) return status;

    uint8_t osr = bmp390->osr_pressure | bmp390->osr_temperature;
    status = BMP390_WriteRegister(bmp390, BMP390_REG_OSR, osr);
    if (status != FC_OK) return status;

    uint8_t odr = bmp390->odr;
    status = BMP390_WriteRegister(bmp390, BMP390_REG_ODR, odr);
    if (status != FC_OK) return status;

    uint8_t pwr = BMP390_PRESSURE_EN | BMP390_TEMPERATURE_EN | BMP390_MODE_NORMAL;
    status = BMP390_WriteRegister(bmp390, BMP390_REG_PWR_CTRL, pwr);
    if (status != FC_OK) return status;

    // Check configuration values are set
    dummy = 0;
    status = BMP390_ReadRegister(bmp390, BMP390_REG_CONFIG, &dummy);
    if (status != FC_OK) return status;
    if ( (dummy & 0x0E) != iir ) return FC_ERR;

    status = BMP390_ReadRegister(bmp390, BMP390_REG_OSR, &dummy);
    if (status != FC_OK) return status;
    if ( (dummy & 0x3F) != osr ) return FC_ERR;

    status = BMP390_ReadRegister(bmp390, BMP390_REG_ODR, &dummy);
    if (status != FC_OK) return status;
    if ( (dummy & 0x1F) != odr ) return FC_ERR;

    status = BMP390_ReadRegister(bmp390, BMP390_REG_PWR_CTRL, &dummy);
    if (status != FC_OK) return status;
    if ( (dummy & 0x3F) != pwr ) return FC_ERR;

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
    bmp390->calib.par_t1 = (float) ((uint16_t)( (calib_data[1] << 8) | calib_data[0] )) / powf(2, -8);
    bmp390->calib.par_t2 = (float) ((uint16_t)( (calib_data[3] << 8) | calib_data[2] )) / powf(2, 30);
    bmp390->calib.par_t3 = (float) ((int8_t) calib_data[4]) / powf(2, 48);
    bmp390->calib.par_p1 = (float) ((int16_t)( (calib_data[6] << 8) | calib_data[5] ) - 16384) / powf(2, 20);
    bmp390->calib.par_p2 = (float) ((int16_t)( (calib_data[8] << 8) | calib_data[7] ) - 16384) / powf(2, 29);
    bmp390->calib.par_p3 = (float) ((int8_t) calib_data[9]) / powf(2, 32);
    bmp390->calib.par_p4 = (float) ((int8_t) calib_data[10]) / powf(2, 37);
    bmp390->calib.par_p5 = (float) ((uint16_t)( (calib_data[12] << 8) | calib_data[11] )) / powf(2, -3);
    bmp390->calib.par_p6 = (float) ((uint16_t)( (calib_data[14] << 8) | calib_data[13] )) / powf(2, 6);
    bmp390->calib.par_p7 = (float) ((int8_t) calib_data[15]) / powf(2, 8);
    bmp390->calib.par_p8 = (float) ((int8_t) calib_data[16]) / powf(2, 15);
    bmp390->calib.par_p9 = (float) ((int16_t)( (calib_data[18] << 8) | calib_data[17] )) / powf(2, 48);
    bmp390->calib.par_p10 = (float) ((int8_t) calib_data[19]) / powf(2, 48);
    bmp390->calib.par_p11 = (float) ((int8_t) calib_data[20]) / powf(2, 65);

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