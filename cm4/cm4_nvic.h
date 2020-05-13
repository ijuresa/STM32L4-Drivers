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
 * @file   cm4_nvic.h
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef CM4_NVIC_H_
#define CM4_NVIC_H_

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
#define CM4_NVIC_MAX_INTERRUPTS (240u) //!< Maximum number of interrupts
#define CM4_NVIC_HIGHEST_PRIORITY (0u)
#define CM4_NVIC_LOWEST_PRIORITY (15u)

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

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
void CM4_NVIC_IRQ_enable(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
void CM4_NVIC_IRQ_disable(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
void CM4_NVIC_IRQ_setPending(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
void CM4_NVIC_IRQ_clearPending(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
bool_t CM4_NVIC_IRQ_isActive(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
void CM4_NVIC_IRQ_setPriority(uint32_t inIrqId, uint32_t inPriority, DRV_ERROR_err_E *outErr);
uint32_t CM4_NVIC_IRQ_getPriority(uint32_t inIrqId, DRV_ERROR_err_E *outErr);
void CM4_NVIC_IRQ_swTrigger(uint32_t inIrqId, DRV_ERROR_err_E *outErr);

#endif /* CM4_NVIC_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
