/* ============================================================================
* bmi088.c
* ============================================================================
*
* @file    bmi088.c
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   BMI088 IMU Sensor Firmware
*
* @details
* BMI088 is the IMU that would be used inside this flight controller and
* this library helps read the acceleration and gyroscope data from the
* sensor.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#include "bmi088.h"
#include "pin_map.h"

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BMI088_Accel_ReadRegister(const BMI088_t *bmi088, BMI088_AccelReg_t reg, uint8_t *data);
static FC_Status_t BMI088_Accel_WriteRegister(const BMI088_t *bmi088, BMI088_AccelReg_t reg, uint8_t data);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BMI088_Init(const BMI088_t *bmi088)
{
    if (bmi088 == NULL) return FC_NULL_PTR_ERR;

    // CSB Lines pull to high on start
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb2_port, bmi088->bmi088_config.csb2_pin, GPIO_PIN_SET);

    // 1ms after POR
    HAL_Delay(1);

    // Dummy read to initialize accelerometer on SPI mode by triggering first rising edge
    uint8_t dummy;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_CHIP_ID, &dummy) != FC_OK) return FC_SPI_ERR;

    // Wake up Accel
    uint8_t pwr_ctrl = 0x04;
    if ( BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_PWR_CTRL, pwr_ctrl) != FC_OK) return FC_SPI_ERR;

    // Wait for 450 microseconds
    HAL_Delay(1);

    if ( BMI088_WhoAmI(bmi088) != FC_OK) return FC_ERR;

    return FC_OK;
}

FC_Status_t BMI088_WhoAmI(const BMI088_t *bmi088)
{
    if (bmi088 == NULL) return FC_NULL_PTR_ERR;
    
    uint8_t chip_id;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_CHIP_ID, &chip_id) != FC_OK) return FC_SPI_ERR;

    if (chip_id != 0x1E) return FC_ERR;

    return FC_OK;
}

/* =========================================================================
* Private Function
* ========================================================================= */
static FC_Status_t BMI088_Accel_ReadRegister(const BMI088_t *bmi088, const BMI088_AccelReg_t reg, uint8_t *data)
{
    FC_Status_t status = FC_OK;
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_RESET);

    // Bit #7 marks 1 as reading from register and 0 as writting to register
    const uint8_t tx_buffer[3] = { (uint8_t) (reg | (1 << 7)) };
    uint8_t rx_buffer[3];
    if ( HAL_SPI_TransmitReceive(bmi088->bmi088_config.spi, tx_buffer, rx_buffer, 3, HAL_MAX_DELAY) != HAL_OK) status = FC_SPI_ERR;

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        *data = rx_buffer[2];
    }

    return status;
}

static FC_Status_t BMI088_Accel_WriteRegister(const BMI088_t *bmi088, const BMI088_AccelReg_t reg, const uint8_t data)
{
    FC_Status_t status = FC_OK;

    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_RESET);

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    const uint8_t tx_buffer[2] = { (uint8_t) (reg & ~(1 << 7)), data };
    if ( HAL_SPI_Transmit(bmi088->bmi088_config.spi, tx_buffer, 2, HAL_MAX_DELAY) != HAL_OK) status = FC_SPI_ERR;

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);

    return status;
}