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
 * @file   cm4_fpu.c
 * @author ivan.juresa
 * @brief  Cortex-MF4 Floating Point Unit (FPU) driver.
 **************************************************************************************************/

#ifndef CM4_FPU_C_
#define CM4_FPU_C_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"
#include <stdint.h>

// CM4
#include "cm4_fpu.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
#define FPU_CPACR  (0xE000ED88u) //!< Coprocessor access control register
#define FPU_FPCCR  (0xE000EF34u) //!< Floating-point context control register
#define FPU_FPCAR  (0xE000EF38u) //!< Floating-point context address register
#define FPU_FPDSCR (0xE000EF3Cu) //!< Floating-point default status control register

//! FPU_CPACR
#define FPU_CPACR_CP10_POSITION (20u)
#define FPU_CPACR_CP11_POSITION (22u)

#define FPU_CPACR_CPN_DENIED     (0u) //!< Denied access. Generates NOCP Usage Fault
#define FPU_CPACR_CPN_PRIVILEGED (1u) //!< Privileged access. Any unprivileged access generates NOCP
#define FPU_CPACR_CPN_RESERVED   (2u) //!< The result of any access is Unpredictable
#define FPU_CPACR_CPN_FULL       (3u) //!< Full access

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/
static volatile uint32_t * const fpuCpacr = (volatile uint32_t *)FPU_CPACR;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void CM4_FPU_init() {
    // Enable CP10 and CP11
    *fpuCpacr = ((FPU_CPACR_CPN_FULL << FPU_CPACR_CP10_POSITION)
                    | (FPU_CPACR_CPN_FULL << FPU_CPACR_CP11_POSITION));
}

/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/

#endif // CM4_FPU_C_

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
