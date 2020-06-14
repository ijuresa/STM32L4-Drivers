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
 * @file   hal_gpio.c
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef HAL_GPIO_C_
#define HAL_GPIO_C_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
// DRV
#include "drv_gpio.h"
#include "drv_rcc.h"

// HAL
#include "hal_gpio.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void HAL_GPIO_init(HAL_GPIO_config_S *inPinConfig, DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(inPinConfig == NULL_PTR) {
            *outErr = ERROR_err_NULL_PTR;
        } else {
            *outErr = ERROR_err_OK;

            // Enable clock
            DRV_RCC_peripheralEnable(inPinConfig->ahbApbClockId, TRUE, outErr);

            if(*outErr == ERROR_err_OK) {
                DRV_GPIO_init(inPinConfig->port,
                              inPinConfig->pin,
                              inPinConfig->mode,
                              inPinConfig->outputType,
                              inPinConfig->outputSpeed,
                              inPinConfig->pudsel,
                              inPinConfig->alternateFunction,
                              outErr);
            }
        }
    }
}

uint8_t HAL_GPIO_read(HAL_GPIO_config_S *inPinConfig, DRV_ERROR_err_E *outErr) {
    uint8_t outVal = 0u;

    if(outErr != NULL_PTR) {
        if(inPinConfig == NULL_PTR) {
            *outErr = ERROR_err_NULL_PTR;
        } else {
            *outErr = ERROR_err_OK;

            outVal = DRV_GPIO_readPin(inPinConfig->port, inPinConfig->pin, outErr);
        }
    }

    return outVal;
}

void HAL_GPIO_write(HAL_GPIO_config_S *inPinConfig, uint8_t inData, DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(inPinConfig == NULL_PTR) {
            *outErr = ERROR_err_NULL_PTR;
        } else {
            *outErr = ERROR_err_OK;

            DRV_GPIO_writePin(inPinConfig->port, inPinConfig->pin, inData, outErr);
        }
    }
}

/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/

#endif // HAL_GPIO_C_

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
