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
 * @file   drv_rcc.h
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef DRV_RCC_H_
#define DRV_RCC_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "stm32l475xx.h"

// DRV
#include "drv_error.h"

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/
//! AHB and APB peripheral reset and clock enable
typedef enum DRV_RCC_ahb_apb_ENUM {
    /*** AHB ***/
    // AHB1
    RCC_ahb_DMA_1  = 0u,
    RCC_ahb_DMA_2  = 1u,
    RCC_ahb_FLASH  = 2u,
    RCC_ahb_CRC    = 3u,
    RCC_ahb_TSC    = 4u,

    // AHB2
    RCC_ahb_GPIO_A = 5u,
    RCC_ahb_GPIO_B = 6u,
    RCC_ahb_GPIO_C = 7u,
    RCC_ahb_GPIO_D = 8u,
    RCC_ahb_GPIO_E = 9u,
    RCC_ahb_GPIO_F = 10u,
    RCC_ahb_GPIO_G = 11u,
    RCC_ahb_GPIO_H = 12u,
    RCC_ahb_OTG_FS = 13u,
    RCC_ahb_ADC    = 14u,
    RCC_ahb_RNG    = 15u,

    // AHB3
    RCC_ahb_FMC    = 16u,
    RCC_ahb_QSPI   = 17u,

    /*** APB ***/
    // APB1_1
    RCC_apb_TIM_2    = 18u,
    RCC_apb_TIM_3    = 19u,
    RCC_apb_TIM_4    = 20u,
    RCC_apb_TIM_5    = 21u,
    RCC_apb_TIM_6    = 22u,
    RCC_apb_TIM_7    = 23u,
    RCC_apb_WWDG     = 24u,
    RCC_apb_SPI_2    = 25u,
    RCC_apb_SPI_3    = 26u,
    RCC_apb_USART_2  = 27u,
    RCC_apb_USART_3  = 28u,
    RCC_apb_UART_4   = 29u,
    RCC_apb_UART_5   = 30u,
    RCC_apb_I2C_1    = 31u,
    RCC_apb_I2C_2    = 32u,
    RCC_apb_I2C_3    = 33u,
    RCC_apb_CAN_1    = 34u,
    RCC_apb_PWR      = 35u,
    RCC_apb_DAC_1    = 36u,
    RCC_apb_OPAMP    = 37u,
    RCC_apb_LPTIM_1  = 38u,

    // APB1_2
    RCC_apb_LPUART_1 = 39u,
    RCC_apb_SWPMI_1  = 40u,
    RCC_apb_LPTIM_2  = 41u,

    // APB2
    RCC_apb_SYSCFG   = 42u,
    RCC_apb_FW       = 43u,
    RCC_apb_SDMMC_1  = 44u,
    RCC_apb_TIM_1    = 45u,
    RCC_apb_SPI_1    = 46u,
    RCC_apb_TIM_8    = 47u,
    RCC_apb_USART_1  = 48u,
    RCC_apb_TIM_15   = 49u,
    RCC_apb_TIM_16   = 50u,
    RCC_apb_TIM_17   = 51u,
    RCC_apb_SAI_1    = 52u,
    RCC_apb_SAI_2    = 53u,
    RCC_apb_DFDM_1   = 54u,

    RCC_ahb_apb_COUNT = 55u
} DRV_RCC_ahb_apb_E;

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! RCC Configuration structure
typedef struct DRV_RCC_config_STRUCT {

} DRV_RCC_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
/***************************************************************************************************
 * @brief   Function is used to reset input AHB or APB peripheral to its default state.
 * @details Input peripheral will reset its status to a state as when MCU boots up.
 * *************************************************************************************************
 * @param   [in]      inPeripheral - Input AHB/APB peripheral ID ::DRV_RCC_ahb_apb_E
 * @param   [out]     outErr       - Output error enumerator
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void DRV_RCC_peripheralReset(uint8_t inPeripheral, DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function is used to enable or disable AHB or APB peripheral clock.
 * @details In case peripheral clock is not active, the peripheral registers read or write
 *          accesses are not supported.
 *          After the enable bit is set, there is a 2 clock cycles delay before the clock is
 *          active. It means that SW will also need to wait for a delay. (Not a problem here)
 * *************************************************************************************************
 * @param   [in]      inPeripheral - Input AHB/APB peripheral ID ::DRV_RCC_ahb_apb_E
 * @param   [in]      inVal        - Disable (FALSE) or Enable (TRUE) Clock
 * @param   [out]     outErr       - Output error enumerator
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void DRV_RCC_peripheralEnable(uint8_t inPeripheral, bool_t inVal, DRV_ERROR_err_E *outErr);

#endif /* DRV_RCC_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
