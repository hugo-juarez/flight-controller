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

#include <limits.h>
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

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t gyro_tx_dma[7];

__attribute__((section(".sram2")))
__attribute__((aligned(32)))
static uint8_t gyro_rx_dma[7];

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
static FC_Status_t BMI088_Gyro_Config(const BMI088_t *bmi088);
static FC_Status_t BMI088_Gyro_ReadRegister(const BMI088_t *bmi088, BMI088_Gyro_Reg_t reg, uint8_t *data);
static FC_Status_t BMI088_Gyro_WriteRegister(const BMI088_t *bmi088, BMI088_Gyro_Reg_t reg, uint8_t data);
static FC_Status_t BMI088_Gyro_BurstReadData(const BMI088_t *bmi088, uint8_t *data);
static FC_Status_t BMI088_SPI_DMA_Transfer(const BMI088_t *bmi088, const uint8_t *tx, uint8_t *rx, uint16_t len);

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

    // Gyro doesn't requires initialization sequence for SPI

    // Check Chip initialized correctly
    status = BMI088_WhoAmI(bmi088);
    if (status != FC_OK) return status;

    // Configure Accel settings
    status = BMI088_Accel_Config(bmi088);
    if (status != FC_OK) return status;

    // Configure Gyro settings
    status = BMI088_Gyro_Config(bmi088);
    if (status != FC_OK) return status;

    return FC_OK;
}

FC_Status_t BMI088_WhoAmI(const BMI088_t *bmi088)
{
    if (bmi088 == NULL) return FC_NULL_PTR_ERR;
    
    uint8_t chip_id;
    const FC_Status_t status = BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CHIP_ID, &chip_id);
    if (status != FC_OK ) return status;

    if (chip_id != BMI088_ACCEL_CHIP_ID) return FC_ERR;

    return FC_OK;
}

FC_Status_t BMI088_Read_Accel(const BMI088_t *bmi088, FC_IMU_Data_t *imu_data)
{
    if (bmi088 == NULL || imu_data == NULL) return FC_NULL_PTR_ERR;


    // Data should be able to hold accel LSB and MSB parts
    uint8_t data[6];
    const FC_Status_t status = BMI088_Accel_BurstReadData(bmi088, data);
    if ( status != FC_OK ) return status;

    // Join values MSB and LSB
    const int16_t ax_lsb = (int16_t) ((uint16_t) data[1] << 8 | data[0]);
    const int16_t ay_lsb = (int16_t) ((uint16_t) data[3] << 8 | data[2]);
    const int16_t az_lsb = (int16_t) ((uint16_t) data[5] << 8 | data[4]);

    // Checking acc_range is valid before assign it
    if (bmi088->settings.acc_range >= sizeof(acc_range_conversion)/sizeof(acc_range_conversion[0])) return FC_CONFIG_ERR;

    // Get range converted for m/s^2
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
    vTaskDelay(pdMS_TO_TICKS(1));

    // Dummy read to initialize accelerometer on SPI mode by triggering first rising edge
    uint8_t dummy;
    FC_Status_t status = BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CHIP_ID, &dummy);
    if (status != FC_OK) return status;

    // Wake up Accel
    status = BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_PWR_CTRL, BMI088_ACCEL_PWR_CTRL_EN);
    if (status != FC_OK) return status;

    // Wait for >=450 microseconds
    vTaskDelay(pdMS_TO_TICKS(1));

    return FC_OK;
}

static FC_Status_t BMI088_Accel_Config(const BMI088_t *bmi088)
{
    // Config Accel BWP and ODR
    const uint8_t acc_conf = bmi088->settings.acc_bwp | bmi088->settings.acc_odr;
    FC_Status_t status = BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_CONF, acc_conf);
    if ( status != FC_OK ) return status;

    // Config Accel Range
    const uint8_t acc_range = bmi088->settings.acc_range & 0x3;
    status = BMI088_Accel_WriteRegister(bmi088, BMI088_ACCEL_REG_RANGE, acc_range);
    if ( status != FC_OK ) return status;

    //Check accel config is correct
    uint8_t read_conf = 0;
    status = BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_CONF, &read_conf) ;
    if ( status != FC_OK ) return status;
    if (read_conf != acc_conf) return FC_CONFIG_ERR;

    read_conf = 0;
    status = BMI088_Accel_ReadRegister(bmi088, BMI088_ACCEL_REG_RANGE, &read_conf);
    if ( status != FC_OK ) return status;
    if (read_conf != acc_range) return FC_CONFIG_ERR;

    return FC_OK;
}

static FC_Status_t BMI088_Accel_ReadRegister(const BMI088_t *bmi088, const BMI088_Accel_Reg_t reg, uint8_t *data)
{
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    accel_tx_dma[0] = (uint8_t) (reg | 0x80);

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, accel_tx_dma, accel_rx_dma, 3);

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
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    accel_tx_dma[0] = (uint8_t) (reg & ~0x80);
    accel_tx_dma[1] = data;

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, accel_tx_dma, accel_rx_dma, 2);

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);

    return status;
}

static FC_Status_t BMI088_Accel_BurstReadData(const BMI088_t *bmi088, uint8_t *data)
{
    // Turn CSB1 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(accel_tx_dma, 0, sizeof(accel_tx_dma));
    memset(accel_rx_dma, 0, sizeof(accel_rx_dma));

    // Assigning transmit register, in burst read it gets auto incremented every read
    accel_tx_dma[0] = (uint8_t) (BMI088_ACCEL_REG_X_LSB | 0x80);

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, accel_tx_dma, accel_rx_dma, 8);

    // Turn CSB1 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb1_port, bmi088->config.csb1_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        memcpy(data, &accel_rx_dma[2], 6);
    }

    return status;
}

static FC_Status_t BMI088_Gyro_Config(const BMI088_t *bmi088)
{
    // Config Gyro Bandwidth
    const uint8_t bandwidth = bmi088->settings.gyro_bandwidth;
    FC_Status_t status = BMI088_Gyro_WriteRegister(bmi088, BMI088_GYRO_REG_BANDWIDTH, bandwidth);
    if (status != FC_OK) return status;

    // Config Gyro Range
    const uint8_t range = bmi088->settings.gyro_range;
    status = BMI088_Gyro_WriteRegister(bmi088, BMI088_GYRO_REG_RANGE, range);
    if (status != FC_OK) return status;

    // Confirm writes where successful
    uint8_t read_conf = 0;
    status = BMI088_Gyro_ReadRegister(bmi088, BMI088_GYRO_REG_BANDWIDTH, &read_conf);
    if (status != FC_OK) return status;
    if (read_conf != bandwidth) return FC_CONFIG_ERR;

    status = BMI088_Gyro_ReadRegister(bmi088, BMI088_GYRO_REG_RANGE, &read_conf);
    if (status != FC_OK) return status;
    if (read_conf != range) return FC_CONFIG_ERR;

    return FC_OK;
}

static FC_Status_t BMI088_Gyro_ReadRegister(const BMI088_t *bmi088, BMI088_Gyro_Reg_t reg, uint8_t *data)
{
    // Turn CSB2 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(gyro_tx_dma, 0, sizeof(gyro_tx_dma));
    memset(gyro_rx_dma, 0, sizeof(gyro_rx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    gyro_tx_dma[0] = (uint8_t) (reg | 0x80);

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, gyro_tx_dma, gyro_rx_dma, 2);

    // Turn CSB2 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        *data = gyro_rx_dma[1];
    }

    return status;
}

static FC_Status_t BMI088_Gyro_WriteRegister(const BMI088_t *bmi088, BMI088_Gyro_Reg_t reg, uint8_t data)
{
    // Turn CSB2 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(gyro_tx_dma, 0, sizeof(gyro_tx_dma));
    memset(gyro_rx_dma, 0, sizeof(gyro_rx_dma));

    // Bit #7 marks 1 as reading from register and 0 as writing to register
    gyro_tx_dma[0] = (uint8_t) (reg & ~0x80);
    gyro_tx_dma[1] = data;

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, gyro_tx_dma, gyro_rx_dma, 2);

    // Turn CSB2 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_SET);

    return status;
}
static FC_Status_t BMI088_Gyro_BurstReadData(const BMI088_t *bmi088, uint8_t *data)
{
    // Turn CSB2 to 0 to initialize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_RESET);

    // Clearing buffers
    memset(gyro_tx_dma, 0, sizeof(gyro_tx_dma));
    memset(gyro_rx_dma, 0, sizeof(gyro_rx_dma));

    // Assigning transmit register, in burst read it gets auto incremented every read
    gyro_tx_dma[0] = (uint8_t) (BMI088_GYRO_REG_X_LSB | 0x80);

    const FC_Status_t status = BMI088_SPI_DMA_Transfer(bmi088, gyro_tx_dma, gyro_rx_dma, 7);

    // Turn CSB2 to 1 to finalize communication
    HAL_GPIO_WritePin(bmi088->config.csb2_port, bmi088->config.csb2_pin, GPIO_PIN_SET);

    if (status == FC_OK)
    {
        memcpy(data, &gyro_rx_dma[1], 6);
    }

    return status;
}

static FC_Status_t BMI088_SPI_DMA_Transfer(const BMI088_t *bmi088, const uint8_t *tx, uint8_t *rx, const uint16_t len)
{
    // Variable to get callback status
    uint32_t cb_status = 0;

    const HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(bmi088->config.spi, tx, rx, len);

    if (hal_status == HAL_BUSY)
    {
        HAL_SPI_Abort(bmi088->config.spi);
    }

    if (hal_status != HAL_OK)
    {
        return FC_SPI_ERR;
    }


    if ( xTaskNotifyWaitIndexed(BMI088_TASK_NOTIFY_INDEX, ULONG_MAX, ULONG_MAX, &cb_status, pdMS_TO_TICKS(2)) == pdFALSE)
    {
        HAL_SPI_Abort(bmi088->config.spi);
        return FC_ERR_TIMEOUT;
    }

    if (cb_status != FC_OK)
    {
        return FC_SPI_ERR;
    }

    return FC_OK;
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
        xTaskNotifyIndexedFromISR(bmi088_task_handle, BMI088_TASK_NOTIFY_INDEX, FC_OK, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == bmi088_spi->Instance)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        configASSERT(bmi088_task_handle != NULL);
        xTaskNotifyIndexedFromISR(bmi088_task_handle, BMI088_TASK_NOTIFY_INDEX, FC_SPI_ERR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}