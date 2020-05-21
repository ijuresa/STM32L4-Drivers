/***********************************************************************************************//**
 * MIT License
 * 
 * Copyright (c) 2020 ivan.juresa
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **************************************************************************************************
 * @file   hal_gpio.h
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"
#include <stdint.h>

// DRV
#include "drv_error.h"

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! HAL GPIO configuration structure. Each used pin should have it
typedef struct HAL_GPIO_config_STRUCT {
    uint8_t port; //!< Port ID. ::DRV_GPIO_port_ENUM
    uint8_t pin; //!< Pin ID. 0 - 15
    uint8_t mode; //!< Mode ::DRV_GPIO_mode_E
    uint8_t outputType; //!< Output type ::DRV_GPIO_oType_E
    uint8_t outputSpeed; //!< Output speed ::DRV_GPIO_oSpeed_E
    uint8_t pudsel; //!< Pull up/down ::DRV_GPIO_pupd_E
    uint8_t alternateFunction; //!< Alternate function ID. 0 - 15
    uint8_t ahbApbClockId; //!< ::DRV_RCC_ahb_apb_E
} HAL_GPIO_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
void HAL_GPIO_init(HAL_GPIO_config_S *inPinConfig, DRV_ERROR_err_E *outErr);
uint8_t HAL_GPIO_read(HAL_GPIO_config_S *inPinConfig, DRV_ERROR_err_E *outErr);
void HAL_GPIO_write(HAL_GPIO_config_S *inPinConfig, uint8_t inData, DRV_ERROR_err_E *outErr);

#endif /* HAL_GPIO_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
