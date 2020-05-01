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
 * @file   drv_gpio.c
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef DRV_GPIO_C_
#define DRV_GPIO_C_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "stm32l475xx.h"
#include "typedefs.h"

// DRV
#include "drv_gpio.h"
#include "drv_rcc.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
#define DRV_GPIO_MAX_PIN_PER_PORT (16u)

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
static INLINE GPIO_TypeDef * GPIO_getPortBase(uint8_t inPort);

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void DRV_GPIO_init(uint8_t inPort, uint8_t inPin, uint8_t inMode, uint8_t inOutputType,
        uint8_t inOutputSpeed,uint8_t inPudsel, uint8_t inAlternateFunction, DRV_ERROR_err_E *outErr) {
    GPIO_TypeDef *port;

    if(outErr != NULL_PTR) {
        if((inPort >= (uint8_t)GPIO_port_COUNT)
                || (inPin >= DRV_GPIO_MAX_PIN_PER_PORT)
                || (inMode >= (uint8_t)GPIO_mode_COUNT)
                || (inOutputType >= (uint8_t)GPIO_oType_COUNT)
                || (inOutputSpeed >= (uint8_t)GPIO_oSpeed_COUNT)
                || (inPudsel >= (uint8_t)GPIO_pupd_COUNT)) {
            // TODO: Add Alternate Function. Ignored for now
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            // Enable clock
            // TODO: Refactor this so its not ugly like this
            DRV_RCC_peripheralEnable((inPort + (uint8_t)RCC_ahb_GPIO_A), TRUE, outErr);

            port = GPIO_getPortBase(inPort);

            // Set mode
            port->MODER |= (inMode << (inPin + inPin));

            // Set output type
            port->OTYPER |= (inOutputType << inPin);

            // Set output speed
            port->OSPEEDR |= (inOutputSpeed << (inPin + inPin));

            // Set pull up/down
            port->PUPDR |= (inPudsel << (inPin + inPin));
        }
    }
}

uint8_t DRV_GPIO_readPin(uint8_t inPort, uint8_t inPin, DRV_ERROR_err_E *outErr) {
    GPIO_TypeDef *port;
    uint8_t outData;

    if(outErr != NULL_PTR) {
        if((inPort >= (uint8_t)GPIO_port_COUNT) || (inPin >= DRV_GPIO_MAX_PIN_PER_PORT)) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            port = GPIO_getPortBase(inPort);

            // All modes will return I/O state except Analog which will return always 0
            outData = ((port->IDR >> inPin) & 0x01u);
        }
    }


    return outData;
}

void DRV_GPIO_writePin(uint8_t inPort, uint8_t inPin, uint8_t inData, DRV_ERROR_err_E *outErr) {
    GPIO_TypeDef *port;

    if(outErr != NULL_PTR) {
        if((inPort >= (uint8_t)GPIO_port_COUNT)
                || (inPin >= DRV_GPIO_MAX_PIN_PER_PORT)
                || (inData >= (uint8_t)GPIO_inData_COUNT)) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            port = GPIO_getPortBase(inPort);

            // Allow only writes to I/O pin configured in an Output Mode
            if((port->MODER >> (inPin + inPin) & 0x03u) == (uint8_t)GPIO_mode_OUTPUT) {
                port->ODR |= (inData << inPin);
            } else {
                *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
            }
        }
    }
}
/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/
static INLINE GPIO_TypeDef * GPIO_getPortBase(uint8_t inPort) {
    static uint32_t portBase[GPIO_port_COUNT] = {
        GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE,
        GPIOE_BASE, GPIOF_BASE, GPIOG_BASE, GPIOH_BASE
    };

    return (GPIO_TypeDef *)portBase[inPort];
}
#endif // DRV_GPIO_C_

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
