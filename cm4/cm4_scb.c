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
 * @file   cm4_scb.c
 * @author ivan.juresa
 * @brief  System Control Block for Cortex-M4 core peripherals
 **************************************************************************************************/

#ifndef CM4_SCB_C_
#define CM4_SCB_C_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "string.h"

// CM4
#include "cm4_scb.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
//! Registers
#define SCB_VTOR (0xE000ED08) //! Vector Table Offset Register

//! VTOR
#define SCB_VTOR_VECTOR_TABLE_POSITION (29u)

//! Flash start
#define FLASH_START (0x08000000u)

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/
static bool_t isInitialised; //! Flag indicating if SCB is initialized

static CM4_SCB_config_S lScbConfig = {
    .address = FLASH_START,
    .memoryRegion = (uint8_t)SCB_memoryRegion_FLASH
};

//! VTOR
static volatile uint32_t * const scbVtor = (volatile uint32_t *)SCB_VTOR;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void CM4_SCB_init(CM4_SCB_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(inConfig != NULL_PTR) {
            // Configuration is supplied. Used that one
            (void)memcpy((void *)&lScbConfig, inConfig, sizeof(CM4_SCB_config_S));
        }

        // Check arguments
        if(lScbConfig.memoryRegion >= (uint8_t)SCB_memoryRegion_COUNT) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            *scbVtor = (lScbConfig.address | (lScbConfig.memoryRegion << SCB_VTOR_VECTOR_TABLE_POSITION));

            isInitialised = TRUE;
        }
    }

}
/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/

#endif // CM4_SCB_C_

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
