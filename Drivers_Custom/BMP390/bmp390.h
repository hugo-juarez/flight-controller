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
#ifndef FLIGHT_CONTROLLER_BMP390_H
#define FLIGHT_CONTROLLER_BMP390_H

#include "stm32f7xx_hal.h"
#include "fc_types.h"

/* =========================================================================
* Macros
* ========================================================================= */
#define BMP390_CHIP_ID          0x60
#define BMP390_PRESSURE_EN      0x01
#define BMP390_TEMPERATURE_EN   0x02
#define BMP390_MODE_NORMAL      0x30

/* =========================================================================
* Enums
* ========================================================================= */
typedef enum
{
    BMP390_REG_CHIP_ID = 0x00,
    BMP390_REG_REV_ID = 0X01,
    BMP390_REG_ERR_REG = 0X02,
    BMP390_REG_STATUS = 0X03,
    BMP390_REG_PRESSURE_DATA = 0X04,
    BMP390_REG_TEMP_DATA = 0X07,
    BMP390_REG_SENSOR_TIME = 0X0C,
    BMP390_REG_EVENT = 0X10,
    BMP390_REG_INT_STATUS = 0X11,
    BMP390_REG_INT_CTRL = 0X19,
    BMP390_REG_IF_CONF = 0X1A,
    BMP390_REG_PWR_CTRL = 0X1B,
    BMP390_REG_OSR = 0X1C,
    BMP390_REG_ODR = 0X1D,
    BMP390_REG_CONFIG = 0X1F,
    BMP390_REG_CMD = 0X7E,
} BMP390_Reg_t;

typedef enum
{
    BMP390_OSR_PRESSURE_1= 0x00,
    BMP390_OSR_PRESSURE_2 = 0x01,
    BMP390_OSR_PRESSURE_4 = 0x02,
    BMP390_OSR_PRESSURE_8 = 0x03,
    BMP390_OSR_PRESSURE_16 = 0x04,
    BMP390_OSR_PRESSURE_32 = 0x05,
} BMP390_OSR_Pressure_t;

typedef enum
{
    BMP390_OSR_TEMPERATURE_1 = 0x00,
    BMP390_OSR_TEMPERATURE_2 = 0x08,
    BMP390_OSR_TEMPERATURE_4 = 0x10,
    BMP390_OSR_TEMPERATURE_8 = 0x18,
    BMP390_OSR_TEMPERATURE_16 = 0x20,
    BMP390_OSR_TEMPERATURE_32 = 0x28,
} BMP390_OSR_Temperature_t;

typedef enum
{
    BMP390_ODR_200 = 0x00,
    BMP390_ODR_100 = 0x01,
    BMP390_ODR_50 = 0x02,
    BMP390_ODR_25 = 0x03,
    BMP390_ODR_12_5 = 0x04,
    BMP390_ODR_6_5 = 0x05,
    BMP390_ODR_3_1 = 0x06,
    BMP390_ODR_1_5 = 0x07,
    BMP390_ODR_0_78 = 0x08,
    BMP390_ODR_0_39 = 0x09,
    BMP390_ODR_0_2 = 0x0A,
    BMP390_ODR_0_1 = 0x0B,
    BMP390_ODR_0_05 = 0x0C,
    BMP390_ODR_0_02 = 0x0D,
    BMP390_ODR_0_01 = 0x0E,
    BMP390_ODR_0_006 = 0x0F,
    BMP390_ODR_0_003 = 0x10,
    BMP390_ODR_0_0015 = 0x11,
} BMP390_ODR_t;

typedef enum
{
    BMP390_IIR_0 = 0x00,
    BMP390_IIR_1 = 0x02,
    BMP390_IIR_3 = 0x04,
    BMP390_IIR_7 = 0x06,
    BMP390_IIR_15 = 0x08,
    BMP390_IIR_31 = 0x0A,
    BMP390_IIR_63 = 0x0C,
    BMP390_IIR_127 = 0x0E,
} BMP390_IIR_t;

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    SPI_HandleTypeDef           *spi;
    uint16_t                    csb_pin;
    GPIO_TypeDef                *csb_port;
} BMP390_Config_t;

typedef struct
{
    BMP390_Config_t             config;
    BMP390_OSR_Pressure_t       osr_pressure;
    BMP390_OSR_Temperature_t    osr_temperature;
    BMP390_ODR_t                odr;
    BMP390_IIR_t                iir;
} BMP390_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BMP390_Init (BMP390_t *bmp390);
FC_Status_t BMP390_WhoAmI (BMP390_t *bmp390);

#endif //FLIGHT_CONTROLLER_BMP390_H