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
#include <string.h>
#include "pin_map.h"

/* =========================================================================
* Private Function Variables
* ========================================================================= */
__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t accel_tx_dma[8];

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t accel_rx_dma[8];

static SPI_HandleTypeDef* bmi088_spi;
static TaskHandle_t bmi088_task_handle;

static const float acc_range_conversion[] = {
    [BMI088_ACCEL_RANGE_3] = 3.0f / 32768.0f * 9.80665f,
    [BMI088_ACCEL_RANGE_6] = 6.0f / 32768.0f * 9.80665f,
    [BMI088_ACCEL_RANGE_12] = 12.0f / 32768.0f * 9.80665f,
    [BMI088_ACCEL_RANGE_24] = 24.0f / 32768.0f * 9.80665f,
};

/* =========================================================================
* Private Function Prototypes
* ========================================================================= */
static FC_Status_t BMI088_Accel_Init(const BMI088_t *bmi088);
static FC_Status_t BMI088_Accel_Config(const BMI088_t *bmi088);
static FC_Status_t BMI088_Accel_ReadRegister(const BMI088_t *bmi088, BMI088_Accel_Reg_t reg, uint8_t *data);
static FC_Status_t BMI088_Accel_WriteRegister(const BMI088_t *bmi088, BMI088_Accel_Reg_t reg, uint8_t data);
static FC_Status_t BMI088_Accel_BurstReadData(const BMI088_t *bmi088, uint8_t *data);

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BMI088_Init(const BMI088_t *bmi088)
{
    if (bmi088 == NULL) return FC_NULL_PTR_ERR;

    FC_Status_t status = FC_OK;

    // Set SPI connection
    bmi088_spi = bmi088->config.spi;

    // Set Task Handling bmi088 it should always be only one
    bmi088_task_handle = bmi088->config.task_handle;

    // Initialized buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // Initialized Accelerator
    status = BMI088_Accel_Init(bmi088);
    if (status != FC_OK) return status;

    // Check Chip initialized correctly
    status = BMI088_WhoAmI(bmi088);
    if (status != FC_OK) return status;

    // Configure Accel settings
    status = BMI088_Accel_Config(bmi088);
    if (status != FC_OK) return status;

    return FC_OK;
}

FC_Status_t BMI088_WhoAmI(const BMI088_t *bmi088)
{
    if (bmi088 == NULL) return FC_NULL_PTR_ERR;
    
    uint8_t chip_id;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CHIP_ID, &chip_id) != FC_OK) return FC_SPI_ERR;

    if (chip_id != 0x1E) return FC_ERR;

    return FC_OK;
}

FC_Status_t BMI088_Read_Accel(const BMI088_t *bmi088, FC_IMU_Data_t *imu_data)
{
    if (bmi088 == NULL || imu_data == NULL) return FC_NULL_PTR_ERR;

    FC_Status_t status = FC_OK;

    // Data should be able to hold accel LSB and MSB parts
    uint8_t data[6];
    status = BMI088_Accel_BurstReadData(bmi088, data);
    if ( status != FC_OK ) return status;

    // Join values MSB and LSB
    const int16_t ax_lsb = (int16_t) ((uint16_t) data[1] << 8 | data[0]);
    const int16_t ay_lsb = (int16_t) ((uint16_t) data[3] << 8 | data[2]);
    const int16_t az_lsb = (int16_t) ((uint16_t) data[5] << 8 | data[4]);

    // Checking acc_range is valid before assign it
    if (bmi088->settings.acc_range >= sizeof(acc_range_conversion)/sizeof(acc_range_conversion[0])) return FC_CONFIG_ERR;

    // Get range converted for mg
    const float acc_range = acc_range_conversion[bmi088->settings.acc_range];

    // Convert to m/s^2 and save to imu_data
    imu_data->ax = (float) ax_lsb * acc_range;
    imu_data->ay = (float) ay_lsb * acc_range;
    imu_data->az = (float) az_lsb * acc_range;

    return FC_OK;
}

/* =========================================================================
* Private Function
* ========================================================================= */
static FC_Status_t BMI088_Accel_Init(const BMI088_t *bmi088)
{
    // CSB Lines pull to high on start
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_SET);

    // 1ms after POR
    HAL_Delay(1);

    // Dummy read to initialize accelerometer on SPI mode by triggering first rising edge
    uint8_t dummy;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CHIP_ID, &dummy) != FC_OK) return FC_SPI_ERR;

    // Wake up Accel
    const uint8_t pwr_ctrl = 0x04;
    if ( BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_PWR_CTRL, pwr_ctrl) != FC_OK) return FC_SPI_ERR;

    // Wait for 450 microseconds
    HAL_Delay(1);

    return FC_OK;
}

static FC_Status_t BMI088_Accel_Config(const BMI088_t *bmi088)
{
    // Config Accel BWP and ODR
    const uint8_t acc_conf = bmi088->settings.acc_bwp | bmi088->settings.acc_odr;
    if ( BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_CONF, acc_conf) != FC_OK) return FC_SPI_ERR;

    // Config Accel Range
    const uint8_t acc_range = bmi088->settings.acc_range & 0x3;
    if ( BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_RANGE, acc_range) != FC_OK ) return FC_SPI_ERR;

    //Check accel config is correct
    uint8_t read_conf = 0;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CONF, &read_conf) != FC_OK ) return FC_SPI_ERR;
    if (read_conf != acc_conf) return FC_CONFIG_ERR;

    read_conf = 0;
    if ( BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_RANGE, &read_conf) != FC_OK ) return FC_SPI_ERR;
    if (read_conf != acc_range) return FC_CONFIG_ERR;

    return FC_OK;
}

static FC_Status_t BMI088_Accel_ReadRegister(const BMI088_t *bmi088, const BMI088_Accel_Reg_t reg, uint8_t *data)
{
    FC_Status_t status = FC_OK;
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writting to register
    accel_tx_dma[0] = (uint8_t) (reg | (1 << 7));

    if ( HAL_SPI_TransmitReceive_DMA(bmi088->config.spi, accel_tx_dma, accel_rx_dma, 3) != HAL_OK) status = FC_SPI_ERR;
    else if ( ulTaskNotifyTakeIndexed(1, pdTRUE, pdMS_TO_TICKS(2)) == 0) status = FC_ERR_TIMEOUT;

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        *data = accel_rx_dma[2];
    }

    return status;
}

static FC_Status_t BMI088_Accel_WriteRegister(const BMI088_t *bmi088, const BMI088_Accel_Reg_t reg, const uint8_t data)
{
    FC_Status_t status = FC_OK;

    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffer
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    accel_tx_dma[0] = (uint8_t) (reg & ~(1 << 7));
    accel_tx_dma[1] = data;

    if ( HAL_SPI_TransmitReceive_DMA(bmi088->config.spi, accel_tx_dma, accel_rx_dma, 2) != HAL_OK) status = FC_SPI_ERR;
    else if ( ulTaskNotifyTakeIndexed(1, pdTRUE, pdMS_TO_TICKS(2)) == 0) status = FC_ERR_TIMEOUT;

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);

    return status;
}

static FC_Status_t BMI088_Accel_BurstReadData(const BMI088_t *bmi088, uint8_t *data)
{
    FC_Status_t status = FC_OK;

    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // IMPORTANT: Reading 6 accel data register + 1 transmit + 1 dummy
    // On initializing tx_buffer array all other elements are zero-initialized
    accel_tx_dma[0] = (uint8_t) (BMI088_ACCEL_REG_X_LSB | 0x80);
    if ( HAL_SPI_TransmitReceive_DMA(bmi088->config.spi, accel_tx_dma, accel_rx_dma, 8) != HAL_OK) status = FC_SPI_ERR;
    else if ( ulTaskNotifyTakeIndexed(1, pdTRUE, pdMS_TO_TICKS(2)) == 0) status = FC_ERR_TIMEOUT;

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        memcpy(data, &accel_rx_dma[2], 6);
    }

    return status;
}

/* =========================================================================
* Callback Functions
* ========================================================================= */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == bmi088_spi->Instance)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        configASSERT(bmi088_task_handle != NULL);
        vTaskNotifyGiveIndexedFromISR(bmi088_task_handle, 1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == bmi088_spi->Instance)
    {
        HAL_SPI_Abort(hspi);
        //@todo: Notify / Manage SPI communication failure
    }
}