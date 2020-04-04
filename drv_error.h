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
 * @file   drv_error.h
 * @author ivan.juresa
 * @brief  Header containing all errors for driver layer.
 **************************************************************************************************/

#ifndef DRV_ERROR_H_
#define DRV_ERROR_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/
typedef enum DRV_ERROR_err_ENUM {
    ERROR_err_OK                = 0u,
    ERROR_err_NULL_PTR          = 1u,
    ERROR_err_ARGS_OUT_OF_RANGE = 2u,
    ERROR_err_NOT_INITIALISED   = 3u,
    ERROR_err_COUNT             = 4u
} DRV_ERROR_err_E;

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

#endif /* DRV_ERROR_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
