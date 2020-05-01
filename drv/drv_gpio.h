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
 * @file   drv_gpio.h
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef DRV_GPIO_H_
#define DRV_GPIO_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "drv_error.h"

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/
//! In data
typedef enum GPIO_inData_ENUM {
    GPIO_inData_OFF   = 0u,
    GPIO_inData_ON    = 1u,
    GPIO_inData_COUNT = 2u
} GPIO_inData_ENUM;

//! GPIO Ports
typedef enum GPIO_port_ENUM {
    GPIO_port_A     = 0u,
    GPIO_port_B     = 1u,
    GPIO_port_C     = 2u,
    GPIO_port_D     = 3u,
    GPIO_port_E     = 4u,
    GPIO_port_F     = 5u,
    GPIO_port_G     = 6u,
    GPIO_port_H     = 7u,
    GPIO_port_COUNT = 8u
} GPIO_port_ENUM;

//! GPIO port mode register
typedef enum GPIO_mode_ENUM {
    GPIO_mode_INPUT  = 0u,
    GPIO_mode_OUTPUT = 1u,
    GPIO_mode_AF     = 2u, //!< Alternate function
    GPIO_mode_ANALOG = 3u,
    GPIO_mode_COUNT  = 4u
} GPIO_mode_E;

//! GPIO port output type register
typedef enum GPIO_oType_ENUM {
    GPIO_oType_PUSH_PULL  = 0u, //!< Reset state
    GPIO_oType_OPEN_DRAIN = 1u,
    GPIO_oType_COUNT      = 2u
} GPIO_oType_E;

//! GPIO port output speed register
typedef enum GPIO_oSpeed_ENUM {
    GPIO_oSpeed_LOW       = 0u,
    GPIO_oSpeed_MEDIUM    = 1u,
    GPIO_oSpeed_HIGH      = 2u,
    GPIO_oSpeed_VERY_HIGH = 3u,
    GPIO_oSpeed_COUNT     = 4u
} GPIO_oSpeed_E;

//! GPIO port pull-up/pull-down register
typedef enum GPIO_pupd_ENUM {
    GPIO_pupd_NONE  = 0u,
    GPIO_pupd_UP    = 1u,
    GPIO_pupd_DOWN  = 2u,
    GPIO_pupd_COUNT = 3u
} GPIO_pupd_E;

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
void DRV_GPIO_init(uint8_t inPort, uint8_t inPin, uint8_t inMode, uint8_t inOutputType,
        uint8_t inOutputSpeed,uint8_t inPudsel, uint8_t inAlternateFunction, DRV_ERROR_err_E *outErr);
uint8_t DRV_GPIO_readPin(uint8_t inPort, uint8_t inPin, DRV_ERROR_err_E *outErr);
void DRV_GPIO_writePin(uint8_t inPort, uint8_t inPin, uint8_t inData, DRV_ERROR_err_E *outErr);

#endif /* DRV_GPIO_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
