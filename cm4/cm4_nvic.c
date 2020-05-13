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
 * @file   cm4_nvic.c
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
// CM4
#include "cm4_nvic.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
//! NVIC Registers. They all have the same format just different addresses
typedef struct NVIC_reg_STRUCT {
    volatile uint32_t REG_0;
    volatile uint32_t REG_1;
    volatile uint32_t REG_2;
    volatile uint32_t REG_3;
    volatile uint32_t REG_4;
    volatile uint32_t REG_5;
    volatile uint32_t REG_6;
    volatile uint32_t REG_7;
} NVIC_reg_S;

#define CM4_PPB_BASE_ADDR (0xE000E000) //!< Base address of CM4 Private peripheral bus

//! NVIC register addresses
#define CM4_NVIC_ISER_ADDR (CM4_PPB_BASE_ADDR + 0x100)
#define CM4_NVIC_ICER_ADDR (CM4_PPB_BASE_ADDR + 0x180)
#define CM4_NVIC_ISPR_ADDR (CM4_PPB_BASE_ADDR + 0x200)
#define CM4_NVIC_ICPR_ADDR (CM4_PPB_BASE_ADDR + 0x280)
#define CM4_NVIC_IABR_ADDR (CM4_PPB_BASE_ADDR + 0x300)

//#define CM4_NVIC_IPR_ADDR  (CM4_PPB_BASE_ADDR + 0x400)
//#define CM4_NVIC_STIR_ADDR (CM4_PPB_BASE_ADDR + 0xE00)

#define CM4_NVIC_ISER (NVIC_reg_S *)CM4_NVIC_ISER_ADDR
#define CM4_NVIC_ICER (NVIC_reg_S *)CM4_NVIC_ICER_ADDR
#define CM4_NVIC_ISPR (NVIC_reg_S *)CM4_NVIC_ISPR_ADDR
#define CM4_NVIC_ICPR (NVIC_reg_S *)CM4_NVIC_ICPR_ADDR
#define CM4_NVIC_IABR (NVIC_reg_S *)CM4_NVIC_IABR_ADDR

//#define CM4_NVIC_IPR  (NVIC_reg_S *)CM4_NVIC_ISER_ADDR
//#define CM4_NVIC_STIR (NVIC_reg_S *)CM4_NVIC_ISER_ADDR


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
void CM4_NVIC_IRQ_enable(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

void CM4_NVIC_IRQ_disable(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

void CM4_NVIC_IRQ_setPending(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

void CM4_NVIC_IRQ_clearPending(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

bool_t CM4_NVIC_IRQ_isActive(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

void CM4_NVIC_IRQ_setPriority(uint32_t inIrqId, uint32_t inPriority, DRV_ERROR_err_E *outErr) {

}

uint32_t CM4_NVIC_IRQ_getPriority(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

void CM4_NVIC_IRQ_swTrigger(uint32_t inIrqId, DRV_ERROR_err_E *outErr) {

}

/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
