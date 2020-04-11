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
 * @file   cm4_systick.c
 * @author ivan.juresa (www.scaluza.com)
 * @brief  SYSTICK driver source file.
 **************************************************************************************************/

#ifndef CM4_SYSTICK_C_
#define CM4_SYSTICK_C_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"
#include "string.h"

// CM4
#include "cm4_systick.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
//! Registers
#define SYST_CSR   (0xE000E010u) //!< SysTick Control and Status Register
#define SYST_RVR   (0xE000E014u) //!< SysTick Reload Value Register
#define SYST_CVR   (0xE000E018u) //!< SysTick Current Value Register
#define SYST_CALIB (0xE000E01Cu) //!< SysTick Calibration Value Register

//! SYS_CSR
#define SYST_CSR_ENABLE        (1u << 0u)  //!< Enable counter
#define SYST_CSR_TICKINT_EN    (1u << 1u)  //!< Counting down will assert SysTick exception request
#define SYST_CSR_CLKSOURCE_AHB (1u << 2u)  //!< Use processor clock
#define SYST_CSR_COUNTFLAG     (1u << 16u) //!< Returns 1 if timer is finished

//! CLOCK_DIVIDER
#define SYSTICK_CLOCK_DIVIDER (8u) //!< In case ::SYSTICK_clockSrc_AHB_8 is chosen

//!
#define SYSTICK_MS_IN_SEC (1000u)

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/
//! Flag indicates if SysTick driver is initialized
static bool_t isInitialised = FALSE;

//! Default SysTick configuration
static CM4_SYSTICK_config_S lSysTickConfig = {
    .clockSrc = (uint8_t)SYSTICK_clockSrc_AHB,
    .freqBelowSec = 1000u, //! 1ms
    .freqAboveSec = 0u, //! 0 seconds
    .ahbClockFrequency = 4000000, //! 4MHz --> Default MSI Clock value, after reset
    .useException = FALSE,
    .callbackFunct = NULL_PTR,
    .param = NULL_PTR
};

// SysTick
static volatile uint32_t * const sysCsr = (volatile uint32_t *)SYST_CSR;
static volatile uint32_t * const sysRvr = (volatile uint32_t *)SYST_RVR;
static volatile uint32_t * const sysCvr = (volatile uint32_t *)SYST_CVR;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/
static uint32_t SYSTICK_calculateReloadVal(const CM4_SYSTICK_config_S *inConfig);

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void CM4_SYSTICK_init(const CM4_SYSTICK_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    uint32_t reloadVal;

    if(outErr != NULL_PTR) {
        if(inConfig != NULL_PTR) {
            // Configuration is supplied. Used that one
            (void)memcpy((void *)&lSysTickConfig, inConfig, sizeof(CM4_SYSTICK_config_S));
        }

        // Check arguments
        if(inConfig->clockSrc >= (uint8_t)SYSTICK_clockSrc_COUNT) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            // Let's calculate new Reload value
            reloadVal = SYSTICK_calculateReloadVal(&lSysTickConfig);
            if((reloadVal >= SYSTICK_RELOAD_VAL_MIN_VAL) && (reloadVal <= SYSTICK_RELOAD_VAL_MAX_VAL)) {

                // Program reload value
                *sysRvr = reloadVal;

                // Clear current value
                *sysCvr = 0u;

                // Check if interrupts are enabled
                if(lSysTickConfig.useException == TRUE) {
                    *sysCsr |= SYST_CSR_TICKINT_EN;
                }

                // Check clock source. AHB_8 is default --> 0
                if(lSysTickConfig.clockSrc == (uint8_t)SYSTICK_clockSrc_AHB) {
                    *sysCsr |= SYST_CSR_CLKSOURCE_AHB;
                }

                // Enable it
                *sysCsr |= SYST_CSR_ENABLE;

                isInitialised = TRUE;
            } else {
                *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
            }
        }
    }
}

void CM4_SYSTICK_start(DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(isInitialised != TRUE) {
            *outErr = ERROR_err_NOT_INITIALISED;
        } else {
            *outErr = ERROR_err_OK;
            *sysCsr |= SYST_CSR_ENABLE;
        }
    }
}

void CM4_SYSTICK_stop(DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(isInitialised != TRUE) {
            *outErr = ERROR_err_NOT_INITIALISED;
        } else {
            *outErr = ERROR_err_OK;
            *sysCsr &= ~SYST_CSR_ENABLE;
        }
    }
}

uint32_t CM4_SYSTICK_getCurrentVal(DRV_ERROR_err_E *outErr) {
    uint32_t outCurrVal = 0u;

    if(outErr != NULL_PTR) {
        if(isInitialised != TRUE) {
            *outErr = ERROR_err_NOT_INITIALISED;
        } else {
            *outErr = ERROR_err_OK;
            outCurrVal = *sysCvr;
        }
    }

    return outCurrVal;
}

bool_t CM4_SYSTICK_hasFinished(DRV_ERROR_err_E *outErr) {
    bool_t outHasFinished = FALSE;

    if(outErr != NULL_PTR) {
        if(isInitialised != TRUE) {
            *outErr = ERROR_err_NOT_INITIALISED;
        } else {
            *outErr = ERROR_err_OK;

            outHasFinished = ((*sysCsr & SYST_CSR_COUNTFLAG) >> 16u);
        }
    }

    return outHasFinished;
}

/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/
static uint32_t SYSTICK_calculateReloadVal(const CM4_SYSTICK_config_S *inConfig) {
    uint32_t outReloadVal = 0u;
    uint32_t clockSpeed = inConfig->ahbClockFrequency;

    // Check first clock source
    if(inConfig->clockSrc == (uint8_t)SYSTICK_clockSrc_AHB_8) {
        clockSpeed /= SYSTICK_CLOCK_DIVIDER;
    }

    // Calculate below second
    outReloadVal = ((clockSpeed / SYSTICK_MS_IN_SEC) * inConfig->freqBelowSec);

    // If needed, add above second
    if(inConfig->freqAboveSec > 0u) {
        outReloadVal += (inConfig->freqAboveSec * clockSpeed);
    }

    return (outReloadVal - 1u);
}

void SysTick_Handler(void) {
    if(lSysTickConfig.callbackFunct != NULL_PTR) {
        // Call it
        lSysTickConfig.callbackFunct(lSysTickConfig.param);
    }
}

#endif // CM4_SYSTICK_C_

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
