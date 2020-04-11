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
 * @file   cm4_systick.h
 * @author ivan.juresa (www.scaluza.com)
 * @brief  SysTick Timer (STK) Header file
 **************************************************************************************************/

#ifndef CM4_SYSTICK_H_
#define CM4_SYSTICK_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "stm32l475xx.h"

// DRV
#include "drv_error.h"

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/
#define SYSTICK_RELOAD_VAL_MIN_VAL (0x00000001u) //!< Minimum reload value for SysTick
#define SYSTICK_RELOAD_VAL_MAX_VAL (0x00FFFFFFu) //!< Maximum reload value for SysTick

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/
//! Clock Source
typedef enum CM4_SYSTICK_clockSrc_ENUM {
    SYSTICK_clockSrc_AHB_8 = 0u, //!< AHB divided by 8
    SYSTICK_clockSrc_AHB   = 1u, //!< Directly AHB
    SYSTICK_clockSrc_COUNT = 2u
} CM4_SYSTICK_clockSrc_E;

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/
typedef void *SYSTICK_param;
typedef void (*CM4_SYSTICK_callback)(SYSTICK_param inParam);

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! SYSTICK configuration structure
typedef struct CM4_SYSTICK_config_STRUCT {
    uint8_t clockSrc; //!< Wanted clock source ::CM4_SYSTICK_clockSrc_E

    //! Example for: 1 second and 20ms:
    //!  - freqBelowSec = 20
    //!  - freqAboveSec = 1
    uint32_t freqBelowSec; //!< Requested frequency below one second. How many times you want it to
                           //!  trigger in one second. 1000 -> 1ms
    uint32_t freqAboveSec; //!< Requested frequency above one second. Leave 0 if you need only below
    uint32_t ahbClockFrequency; //!< Current AHB clock frequency in HZ.

    //! Exception part
    bool_t useException; //!< Flag indicating if exception is enabled
    CM4_SYSTICK_callback callbackFunct; //!< Pointer to called function when exception is triggered
    SYSTICK_param param; //!< Pointer to parameter passed with the exception
} CM4_SYSTICK_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
/***************************************************************************************************
 * @brief   Function is used to initialize Cortex System Timer (SysTick).
 * @details Timer will start after initialization completes.
 * *************************************************************************************************
 * @param   [in]      *inConfig - Input SYSTICK configuration structure
 *                              - This parameter can be empty. In that case default settings will
 *                                be used:
 *                                 - MSI as a clock source with 4MHz frequency
 *                                 - 1ms trigger
 *                                 - Without an interrupt -> You need to poll ::CM4_SYSTICK_isFinished
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void CM4_SYSTICK_init(const CM4_SYSTICK_config_S *inConfig, DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function will start SysTick.
 * @details It needs to be properly initialized beforehand.
 * *************************************************************************************************
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void CM4_SYSTICK_start(DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function will stop SysTick.
 * @details It needs to be properly initialized beforehand.
 * *************************************************************************************************
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void CM4_SYSTICK_stop(DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function will return current SysTick value.
 * @details It needs to be properly initialized beforehand.
 *          You can use this function to see current value but if you want to see if timer finished
 *          use ::CM4_SYSTICK_isFinished
 * *************************************************************************************************
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @return  Current System Cortex Timer value.
 **************************************************************************************************/
uint32_t CM4_SYSTICK_getCurrentVal(DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function will return flag indicating if timeout has finished.
 * @details It needs to be properly initialized beforehand.
 *          SysTick will set flag to TRUE after it counts to 0. If register is read flag will
 *          change the value to FALSE.
 *
 *          You can continuously call this function and in case return value is 1 it means timer
 *          has finished.
 * *************************************************************************************************
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @return  Flag indicating if timeout has counted to 0 (finished).
 **************************************************************************************************/
bool_t CM4_SYSTICK_hasFinished(DRV_ERROR_err_E *outErr);

#endif /* CM4_SYSTICK_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
