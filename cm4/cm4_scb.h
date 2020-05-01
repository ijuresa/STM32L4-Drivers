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
 * @file   cm4_scb.h
 * @author ivan.juresa
 * @brief  System Control Block for Cortex-M4 core peripherals.
 **************************************************************************************************/

#ifndef CM4_SCB_H_
#define CM4_SCB_H_

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
//! VTOR Memory Regions
typedef enum CM4_SCB_memoryRegion_ENUM {
    SCB_memoryRegion_FLASH = 0u,
    SCB_memoryRegion_SRAM  = 1u,
    SCB_memoryRegion_COUNT = 2u
} CM4_SCB_memoryRegion_E;

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! SCB configuration structure
typedef struct CM4_SCB_config_STRUCT {
    uint32_t address; //!< Vector table address location
    uint8_t memoryRegion; //!< ::CM4_SCB_memoryRegion_E
} CM4_SCB_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
/***************************************************************************************************
 * @brief   Function is used to initialize Cortex System Control Block (SCB).
 * @details Right now it will only set an address of vector interrupt table. Driver will be updated
 *          as project will progress.
 * *************************************************************************************************
 * @param   [in]      *inConfig - Input SCB configuration structure
 *                              - This parameter can be empty. In that case default settings will
 *                                be used.
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @exceptions        ERROR_err_ARGS_OUT_OF_RANGE: Memory Region is not valid.
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void CM4_SCB_init(CM4_SCB_config_S *inConfig, DRV_ERROR_err_E *outErr);

#endif /* CM4_SCB_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
