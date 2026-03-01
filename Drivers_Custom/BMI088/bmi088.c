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
void BMI088_Accel_ReadRegister(const BMI088_t *bmi088, BMI088_AccelReg_t reg, uint8_t *data);
void BMI088_Accel_WriteRegister(const BMI088_t *bmi088, BMI088_AccelReg_t reg, uint8_t data);

/* =========================================================================
* Public APIs
* ========================================================================= */
void BMI088_Init(const BMI088_t *bmi088)
{
    // CSB Lines pull to high on start
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb2_port, bmi088->bmi088_config.csb2_pin, GPIO_PIN_SET);

    // Dummy read to initialize accelerometer on SPI mode by triggering first rising edge
    uint8_t dummy;
    BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_CHIP_ID, &dummy);

    // Wake up Accel
    uint8_t pwr_ctrl = 0x04;
    BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_PWR_CTRL, pwr_ctrl);
}

/* =========================================================================
* Private Function
* ========================================================================= */
void BMI088_Accel_ReadRegister(const BMI088_t *bmi088, const BMI088_AccelReg_t reg, uint8_t *data)
{
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_RESET);

    // Bit #0 marks 1 as reading from register and 0 as writting to register
    const uint8_t tx_buffer[3] = { (uint8_t) (reg | (1 << 7)) };
    uint8_t rx_buffer[3];
    HAL_SPI_TransmitReceive(bmi088->bmi088_config.spi, tx_buffer, rx_buffer, 3, HAL_MAX_DELAY);

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);

    *data = rx_buffer[2];
}

void BMI088_Accel_WriteRegister(const BMI088_t *bmi088, const BMI088_AccelReg_t reg, const uint8_t data)
{
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_RESET);

    // Bit #0 marks 1 as reading from register and 0 as writing to register
    const uint8_t tx_buffer[2] = { (uint8_t) (reg & ~(1 << 7)), data };
    HAL_SPI_Transmit(bmi088->bmi088_config.spi, tx_buffer, 2, HAL_MAX_DELAY);

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->bmi088_config.csb1_port, bmi088->bmi088_config.csb1_pin, GPIO_PIN_SET);

}