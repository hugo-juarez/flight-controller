/* ============================================================================
* bmi088.h
* ============================================================================
*
* @file    bmi088.h
* @author  Hugo Juarez
* @date    2026-02-27
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
#ifndef FLIGHT_CONTROLLER_BMI088_H
#define FLIGHT_CONTROLLER_BMI088_H

#include <FreeRTOS.h>
#include <task.h>
#include "stm32f7xx_hal.h"
#include "fc_types.h"

/* =========================================================================
* Macros
* ========================================================================= */
#define BMI088_ACCEL_CHIP_ID        0x1E
#define BMI088_ACCEL_PWR_CTRL_EN    0x04

/* =========================================================================
* Enums
* ========================================================================= */
typedef enum
{
    BMI088_ACCEL_REG_CHIP_ID = 0x00,
    BMI088_ACCEL_REG_ERR = 0x02,
    BMI088_ACCEL_REG_STATUS = 0x03,
    BMI088_ACCEL_REG_X_LSB = 0x12,
    BMI088_ACCEL_REG_Y_LSB = 0x14,
    BMI088_ACCEL_REG_Z_LSB = 0x16,
    BMI088_ACCEL_REG_CONF = 0x40,
    BMI088_ACCEL_REG_RANGE = 0x41,
    BMI088_ACCEL_REG_PWR_CTRL = 0x7D,
    BMI088_ACCEL_REG_SOFTRESET = 0x7E,
} BMI088_Accel_Reg_t;

typedef enum
{
    BMI088_ACCEL_BWP_OSR4 = 0x80,
    BMI088_ACCEL_BWP_OSR2 = 0x90,
    BMI088_ACCEL_BWP_NORMAL = 0xA0,
} BMI088_Accel_BWP_t;

typedef enum
{
    BMI088_ACCEL_ODR_12_5 = 0x05,
    BMI088_ACCEL_ODR_25 = 0x06,
    BMI088_ACCEL_ODR_50 = 0x07,
    BMI088_ACCEL_ODR_100 = 0x08,
    BMI088_ACCEL_ODR_200 = 0x09,
    BMI088_ACCEL_ODR_400 = 0x0A,
    BMI088_ACCEL_ODR_800 = 0x0B,
    BMI088_ACCEL_ODR_1600 = 0x0C,
} BMI088_Accel_ODR_t;

typedef enum
{
    BMI088_ACCEL_RANGE_3 = 0x00,
    BMI088_ACCEL_RANGE_6 = 0x01,
    BMI088_ACCEL_RANGE_12 = 0x02,
    BMI088_ACCEL_RANGE_24 = 0x03,
} BMI088_Accel_Range_t;
/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    TaskHandle_t            task_handle;
    SPI_HandleTypeDef       *spi;
    uint16_t                csb1_pin;
    GPIO_TypeDef            *csb1_port;
    uint16_t                csb2_pin;
    GPIO_TypeDef            *csb2_port;
} BMI088_Config_t;

typedef struct
{
    BMI088_Accel_BWP_t      acc_bwp;
    BMI088_Accel_ODR_t      acc_odr;
    BMI088_Accel_Range_t    acc_range;
} BMI088_Settings_t;

typedef struct
{
    BMI088_Config_t         config;
    BMI088_Settings_t       settings;
} BMI088_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BMI088_Init(const BMI088_t *bmi088);
FC_Status_t BMI088_WhoAmI(const BMI088_t *bmi088);
FC_Status_t BMI088_Read_Accel(const BMI088_t *bmi088, FC_IMU_Data_t *imu_data);

#endif //FLIGHT_CONTROLLER_BMI088_H