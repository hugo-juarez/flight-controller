/* ============================================================================
* app_main.h
* ============================================================================
*
* @file    app_main.h
* @author  Hugo Juarez
* @date    2026-02-26
* @version 0.1.0
*
* @brief   One line summary of what this file does.
*
* @details
* More detailed description of the module, its responsibilities,
* and how it fits into the overall firmware architecture.
*
* @copyright Copyright (c) 2026 Hugo Juarez. Licensed under the MIT License.
*             See LICENSE file in the root of the repository for details.
* ========================================================================= */

#ifndef FLIGHT_CONTROLLER_APP_MAIN_H
#define FLIGHT_CONTROLLER_APP_MAIN_H

#include "fc_types.h"

/* =========================================================================
* Macros
* ========================================================================= */
#define CRSF_UART           USART1
#define GPS_UART            USART2
#define MAG_I2C             I2C2

/* =========================================================================
* Public APIs
* ========================================================================= */
FC_Status_t app_init(FC_Hw_t *hw);
FC_Status_t app_start(void);

#endif //FLIGHT_CONTROLLER_APP_MAIN_H