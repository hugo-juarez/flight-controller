/* ============================================================================
* bn880.h
* ============================================================================
*
* @file    bn880.h
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   This library holds the APIs for the BN880 GPS sensor.
*
* @details
* This file holds the APIs to communicate and pull data from BN880 GPS sensor's
* M8030-KT GPS and HMC5883L magnetometer. Using UBX as communication for GPS.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */
#ifndef FLIGHT_CONTROLLER_BN880_H
#define FLIGHT_CONTROLLER_BN880_H

#include <FreeRTOS.h>
#include <task.h>
#include "stm32f7xx_hal.h"
#include "fc_types.h"

/* =========================================================================
* Macros
* ========================================================================= */
#define BN880_TASK_GPS_RX_NOTIFY_INDEX  0
#define BN880_TASK_GPS_TX_NOTIFY_INDEX  1
#define BN880_TASK_MAG_NOTIFY_INDEX     2
#define BN880_GPS_NAV_RATE              1U
#define BN880_MAG_ADDR                  0x3C
#define BN880_MAG_DMA_BUFFER            6U
#define BN880_UBX_MAX_RX_MSG            128U
#define BN880_UBX_MAX_TX_MSG            6U
#define BN880_UBX_ACK_MSG_LEN           10U
#define BN880_UBX_NAV_MSG_LEN           100U
#define BN880_UBX_SYNC_1                0xB5
#define BN880_UBX_SYNC_2                0x62
#define BN880_UBX_CLASS_CFG             0x06
#define BN880_UBX_ID_CFG                0x01
#define BN880_UBX_LEN_CFG               3U
#define BN880_UBX_ID_RATE               0x08
#define BN880_UBX_LEN_RATE              6U
#define BN880_UBX_CLASS_NAV_PVT         0x01
#define BN880_UBX_ID_NAV_PVT            0x07
#define BN880_UBX_CLASS_ACK             0x05
#define BN880_UBX_ID_ACK                0x01
#define BN880_UBX_LEN_ACK               2U

/* =========================================================================
* Enums
* ========================================================================= */
typedef enum
{
    BN880_UBX_VALID_DATE = 0x01,
    BN880_UBX_VALID_TIME = 0x02,
    BN880_UBX_VALID_FULLY_RESOLVED = 0x04,
    BN880_UBX_VALID_MAG = 0x08
} BN880_UBX_Validity_t;

typedef enum
{
    BN880_UBX_GNSS_FIX_TYPE_NA = 0,
    BN880_UBX_GNSS_FIX_TYPE_DR_ONLY = 1,
    BN880_UBX_GNSS_FIX_TYPE_2D = 2,
    BN880_UBX_GNSS_FIX_TYPE_3D = 3,
    BN880_UBX_GNSS_FIX_TYPE_GNSS_DR_COMBINED = 4,
    BN880_UBX_GNSS_FIX_TYPE_TIME_ONLY = 5,
} BN880_UBX_GNSS_Fix_Type_t;

typedef enum
{
    BN880_UBX_GNSS_FIX_FLAGS_GNSS_OK = 0x01,
    BN880_UBX_GNSS_FIX_FLAGS_DIFF_ON = 0x02,
    BN880_UBX_GNSS_FIX_FLAGS_PSM_STATE_ON = 0x04,
    BN880_UBX_GNSS_FIX_FLAGS_PSM_ACQUISITION = 0x08,
    BN880_UBX_GNSS_FIX_FLAGS_PSM_TRACKING = 0x0C,
    BN880_UBX_GNSS_FIX_FLAGS_PSM_POWER_OPTIMIZED_TRACKING = 0x10,
    BN880_UBX_GNSS_FIX_FLAGS_PSM_INACTIVE = 0x14,
    BN880_UBX_GNSS_FIX_FLAGS_HEADING_VALID = 0x20,
    BN880_UBX_GNSS_FIX_FLAGS_CARR_FLOAT = 0x40,
    BN880_UBX_GNSS_FIX_FLAGS_CARR_FIXED = 0x80,
} BN880_UBX_GNSS_Fix_Flags_t;

typedef enum
{
    BN880_UBX_ADD_FLAGS_CONFIRMED_AVAILABLE = 0x20,
    BN880_UBX_ADD_FLAGS_CONFIRMED_DATE = 0x40,
    BN880_UBX_ADD_FLAGS_CONFIRMED_TIME = 0x80,
} BN880_UBX_Add_Flags_t;

typedef enum
{
    BN880_GPS_PERIOD_1HZ = 1000,
    BN880_GPS_PERIOD_10HZ = 100,
} BN880_GPS_MeasRate_t;

typedef enum
{
    BN880_GPS_TIME_UTC = 0,
    BN880_GPS_TIME_GPS = 1,
    BN880_GPS_TIME_GLONASS = 2,
    BN880_GPS_TIME_BEIDOU = 3,
    BN880_GPS_TIME_GALILEO = 4,
} BN880_GPS_Time_t;

typedef enum
{
    BN880_MAG_REG_CFG_A = 0x00,
    BN880_MAG_REG_CFG_B = 0x01,
    BN880_MAG_REG_MODE = 0x02,
    BN880_MAG_REG_X_MSB = 0x03,
    BN880_MAG_REG_Z_MSB = 0x05,
    BN880_MAG_REG_Y_MSB = 0x07,
    BN880_MAG_REG_STATUS = 0x09,
    BN880_MAG_REG_ID_A = 0x0A,
    BN880_MAG_REG_ID_B = 0x0B,
    BN880_MAG_REG_ID_C = 0x0C,
} BN880_Mag_Reg_t;

typedef enum
{
    BN880_MAG_MEAS_MODE_NORMAL = 0x00,
    BN880_MAG_MEAS_MODE_POSITIVE_BIASED = 0x01,
    BN880_MAG_MEAS_MODE_NEGATIVE_BIASED = 0x02,
} BN880_Mag_Meas_Mode_t;

typedef enum
{
    BN880_MAG_DOR_0_75 = 0x00,
    BN880_MAG_DOR_1_5 = 0x04,
    BN880_MAG_DOR_3 = 0x08,
    BN880_MAG_DOR_7_5 = 0x0C,
    BN880_MAG_DOR_15 = 0x10,
    BN880_MAG_DOR_30 = 0x14,
    BN880_MAG_DOR_75 = 0x18,
} BN880_Mag_DOR_t;

typedef enum
{
    BN880_MAG_SMP_AVG_1 = 0x00,
    BN880_MAG_SMP_AVG_2 = 0x20,
    BN880_MAG_SMP_AVG_4 = 0x40,
    BN880_MAG_SMP_AVG_8 = 0x60,
} BN880_Mag_Smp_Avg_t;

typedef enum
{
    BN880_MAG_GAIN_0_88 = 0x00,
    BN880_MAG_GAIN_1_3 = 0x20,
    BN880_MAG_GAIN_1_9 = 0x40,
    BN880_MAG_GAIN_2_5 = 0x60,
    BN880_MAG_GAIN_4 = 0x80,
    BN880_MAG_GAIN_4_7 = 0xA0,
    BN880_MAG_GAIN_5_6 = 0xC0,
    BN880_MAG_GAIN_8_1 = 0xE0,
} BN880_Mag_Gain_t;

typedef enum
{
    BN880_MAG_MODE_CONTINUOUS = 0x00,
    BN880_MAG_MODE_SINGLE_MEAS = 0x01,
    BN880_MAG_MODE_IDLE = 0x02,
} BN880_Mag_Mode_t;

/* =========================================================================
* Structs
* ========================================================================= */
typedef struct
{
    uint32_t                    iTOW;
    uint16_t                    year;
    uint8_t                     month;
    uint8_t                     day;
    uint8_t                     hour;
    uint8_t                     min;
    uint8_t                     sec;
    BN880_UBX_Validity_t        valid;
    uint32_t                    time_acc;
    int32_t                     nano;
    BN880_UBX_GNSS_Fix_Type_t   fix_type;
    BN880_UBX_GNSS_Fix_Flags_t  flags1;
    BN880_UBX_Add_Flags_t       flags2;
    uint8_t                     num_sv;
    int32_t                     lon;
    int32_t                     lat;
    int32_t                     height;
    int32_t                     height_msl;
    uint32_t                    horizontal_acc;
    uint32_t                    vertical_acc;
    int32_t                     north_vel;
    int32_t                     east_vel;
    int32_t                     down_vel;
    int32_t                     ground_speed;
    int32_t                     heading_motion;
    uint32_t                    speed_acc;
    uint32_t                    heading_acc;
    uint16_t                    position_dop;
    int32_t                     heading_vel;
    int16_t                     mag_dec;
    uint16_t                    mag_acc;
} BN880_GPS_NAV_PVT_t;

typedef struct
{
    int16_t                     x;
    int16_t                     y;
    int16_t                     z;
} BN880_Mag_Data_t;

typedef struct
{
    TaskHandle_t                task_handle;
    UART_HandleTypeDef          *uart;
    I2C_HandleTypeDef           *i2c;
} BN880_Config_t;

typedef struct
{
    BN880_Config_t              config;
    BN880_GPS_MeasRate_t        gps_period;
    BN880_GPS_Time_t            gps_time_format;
    BN880_Mag_Smp_Avg_t         mag_smp_avg;
    BN880_Mag_DOR_t             mag_dor;
    BN880_Mag_Meas_Mode_t       mag_meas_mode;
    BN880_Mag_Gain_t            mag_gain;
    BN880_Mag_Mode_t            mag_mode;
} BN880_t;

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t BN880_Init(BN880_t *bn880);
FC_Status_t BN880_GPS_Parse(BN880_GPS_NAV_PVT_t *gps_nav_pvt, uint16_t end_pos);
FC_Status_t BN880_Mag_Parse(const BN880_t *bn880, BN880_Mag_Data_t *mag_data);

/* =========================================================================
* Callback APIs
* ========================================================================= */
void BN880_GPS_TxCplt_Callback(void);
void BN880_GPS_RxCplt_Callback(uint16_t end_pos);
void BN880_GPS_Error_Callback(UART_HandleTypeDef *huart);
void BN880_Mag_MasterTxCplt_Callback(void);
void BN880_Mag_MasterRxCplt_Callback(void);
void BN880_Mag_Error_Callback(void);

#endif //FLIGHT_CONTROLLER_BN880_H