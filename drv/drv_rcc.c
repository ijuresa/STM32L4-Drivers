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
 * @file   drv_rcc.c
 * @author ivan.juresa
 * @brief  
 **************************************************************************************************/

#ifndef DRV_RCC_C_
#define DRV_RCC_C

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"

// DRV
#include "drv_rcc.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
#define SNA_VAL (0xFFu) //!< Signal not available value. Used when some feature is not supported

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/
// Reset bit values
static const uint32_t ahbApbResetVal[RCC_ahb_apb_COUNT] = {
    /*** AHB ***/
    // AHB1
    RCC_AHB1RSTR_DMA1RST,
    RCC_AHB1RSTR_DMA2RST,
    RCC_AHB1RSTR_FLASHRST,
    RCC_AHB1RSTR_CRCRST,
    RCC_AHB1RSTR_TSCRST,

    // AHB2
    RCC_AHB2RSTR_GPIOARST,
    RCC_AHB2RSTR_GPIOBRST,
    RCC_AHB2RSTR_GPIOCRST,
    RCC_AHB2RSTR_GPIODRST,
    RCC_AHB2RSTR_GPIOERST,
    RCC_AHB2RSTR_GPIOFRST,
    RCC_AHB2RSTR_GPIOGRST,
    RCC_AHB2RSTR_GPIOHRST,
    RCC_AHB2RSTR_OTGFSRST,
    RCC_AHB2RSTR_ADCRST,
    RCC_AHB2RSTR_RNGRST,

    // AHB3
    RCC_AHB3RSTR_FMCRST,
    RCC_AHB3RSTR_QSPIRST,

    /*** APB ***/
    // APB1_1
    RCC_APB1RSTR1_TIM2RST,
    RCC_APB1RSTR1_TIM3RST,
    RCC_APB1RSTR1_TIM4RST,
    RCC_APB1RSTR1_TIM5RST,
    RCC_APB1RSTR1_TIM6RST,
    RCC_APB1RSTR1_TIM7RST,
    SNA_VAL, //! Window Watchdog can't be reset
    RCC_APB1RSTR1_SPI2RST,
    RCC_APB1RSTR1_SPI3RST,
    RCC_APB1RSTR1_USART2RST,
    RCC_APB1RSTR1_USART3RST,
    RCC_APB1RSTR1_UART4RST,
    RCC_APB1RSTR1_UART5RST,
    RCC_APB1RSTR1_I2C1RST,
    RCC_APB1RSTR1_I2C2RST,
    RCC_APB1RSTR1_I2C3RST,
    RCC_APB1RSTR1_CAN1RST,
    RCC_APB1RSTR1_PWRRST,
    RCC_APB1RSTR1_DAC1RST,
    RCC_APB1RSTR1_OPAMPRST,
    RCC_APB1RSTR1_LPTIM1RST,

    // APB1_2
    RCC_APB1RSTR2_LPUART1RST,
    RCC_APB1RSTR2_SWPMI1RST,
    RCC_APB1RSTR2_LPTIM2RST,

    // APB2
    RCC_APB2RSTR_SYSCFGRST,
    SNA_VAL, //! Firewall can't be reset
    RCC_APB2RSTR_SDMMC1RST,
    RCC_APB2RSTR_TIM1RST,
    RCC_APB2RSTR_SPI1RST,
    RCC_APB2RSTR_TIM8RST,
    RCC_APB2RSTR_USART1RST,
    RCC_APB2RSTR_TIM15RST,
    RCC_APB2RSTR_TIM16RST,
    RCC_APB2RSTR_TIM17RST,
    RCC_APB2RSTR_SAI1RST,
    RCC_APB2RSTR_SAI2RST,
    RCC_APB2RSTR_DFSDM1RST
};

// Clock Enable bit values
static const uint32_t ahbApbEnVal[RCC_ahb_apb_COUNT] = {
    /*** AHB ***/
    // AHB1
    RCC_AHB1ENR_DMA1EN,
    RCC_AHB1ENR_DMA2EN,
    RCC_AHB1ENR_FLASHEN,
    RCC_AHB1ENR_CRCEN,
    RCC_AHB1ENR_TSCEN,

    // AHB2
    RCC_AHB2ENR_GPIOAEN,
    RCC_AHB2ENR_GPIOBEN,
    RCC_AHB2ENR_GPIOCEN,
    RCC_AHB2ENR_GPIODEN,
    RCC_AHB2ENR_GPIOEEN,
    RCC_AHB2ENR_GPIOFEN,
    RCC_AHB2ENR_GPIOGEN,
    RCC_AHB2ENR_GPIOHEN,
    RCC_AHB2ENR_OTGFSEN,
    RCC_AHB2ENR_ADCEN,
    RCC_AHB2ENR_RNGEN,

    // AHB3
    RCC_AHB3ENR_FMCEN,
    RCC_AHB3ENR_QSPIEN,

    /*** APB ***/
    // APB1_1
    RCC_APB1ENR1_TIM2EN,
    RCC_APB1ENR1_TIM3EN,
    RCC_APB1ENR1_TIM4EN,
    RCC_APB1ENR1_TIM5EN,
    RCC_APB1ENR1_TIM6EN,
    RCC_APB1ENR1_TIM7EN,
    RCC_APB1ENR1_WWDGEN,
    RCC_APB1ENR1_SPI2EN,
    RCC_APB1ENR1_SPI3EN,
    RCC_APB1ENR1_USART2EN,
    RCC_APB1ENR1_USART3EN,
    RCC_APB1ENR1_UART4EN,
    RCC_APB1ENR1_UART5EN,
    RCC_APB1ENR1_I2C1EN,
    RCC_APB1ENR1_I2C2EN,
    RCC_APB1ENR1_I2C3EN,
    RCC_APB1ENR1_CAN1EN,
    RCC_APB1ENR1_PWREN,
    RCC_APB1ENR1_DAC1EN,
    RCC_APB1ENR1_OPAMPEN,
    RCC_APB1ENR1_LPTIM1EN,

    // APB1_2
    RCC_APB1ENR2_LPUART1EN,
    RCC_APB1ENR2_SWPMI1EN,
    RCC_APB1ENR2_LPTIM2EN,

    // APB_2
    RCC_APB2ENR_SYSCFGEN,
    RCC_APB2ENR_FWEN,
    RCC_APB2ENR_SDMMC1EN,
    RCC_APB2ENR_TIM1EN,
    RCC_APB2ENR_SPI1EN,
    RCC_APB2ENR_TIM8EN,
    RCC_APB2ENR_USART1EN,
    RCC_APB2ENR_TIM15EN,
    RCC_APB2ENR_TIM16EN,
    RCC_APB2ENR_TIM17EN,
    RCC_APB2ENR_SAI1EN,
    RCC_APB2ENR_SAI2EN,
    RCC_APB2ENR_DFSDM1EN
};
/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/
static uint32_t *RCC_getRstReg(uint8_t inPeripheral);
static uint32_t *RCC_getEnReg(uint8_t inPeripheral);

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void DRV_RCC_peripheralReset(uint8_t inPeripheral, DRV_ERROR_err_E *outErr) {
    uint32_t *ahbApbAddr;

    if(outErr != NULL_PTR) {
        if(inPeripheral >= (uint8_t)RCC_ahb_apb_COUNT) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            ahbApbAddr = RCC_getRstReg(inPeripheral);

            // Address needs to be valid and feature supported
            if((ahbApbAddr != NULL_PTR) && (ahbApbResetVal[inPeripheral] != SNA_VAL)) {
                *ahbApbAddr |= ahbApbResetVal[inPeripheral];
            } else {
                *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
            }
        }
    }
}

void DRV_RCC_peripheralEnable(uint8_t inPeripheral, bool_t inVal, DRV_ERROR_err_E *outErr) {
    uint32_t *ahbApbAddr;

    if(outErr != NULL_PTR) {
        if(inPeripheral >= RCC_ahb_apb_COUNT) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            ahbApbAddr = RCC_getEnReg(inPeripheral);

            // Address needs to be valid
            if(ahbApbAddr != NULL_PTR) {
                if(inVal == TRUE) {
                    // Enable clock
                    *ahbApbAddr |= ahbApbEnVal[inPeripheral];
                } else {
                    // Disable clock
                    *ahbApbAddr &= (~ahbApbEnVal[inPeripheral]);
                }
            } else {
                *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
            }
        }
    }
}

/***************************************************************************************************
 *                      PRIVATE FUNCTIONS DEFINITION
 **************************************************************************************************/
static uint32_t *RCC_getRstReg(uint8_t inPeripheral) {
    uint32_t *outRegister;

    switch(inPeripheral) {
        // AHB1
        case (uint8_t)RCC_ahb_DMA_1 ... (uint8_t)RCC_ahb_TSC:
            outRegister = (uint32_t *)&RCC->AHB1RSTR;
            break;

        // AHB2
        case (uint8_t)RCC_ahb_GPIO_A ... (uint8_t)RCC_ahb_RNG:
            outRegister = (uint32_t *)&RCC->AHB2RSTR;
            break;

        // AHB3
        case (uint8_t)RCC_ahb_FMC ... (uint8_t)RCC_ahb_QSPI:
            outRegister = (uint32_t *)&RCC->AHB3RSTR;
            break;

        // APB1_1
        case (uint8_t)RCC_apb_TIM_2 ... (uint8_t)RCC_apb_LPTIM_1:
            outRegister = (uint32_t *)&RCC->APB1RSTR1;
            break;

        // APB1_2
        case (uint8_t)RCC_apb_LPUART_1 ... (uint8_t)RCC_apb_LPTIM_2:
            outRegister = (uint32_t *)&RCC->APB1RSTR2;
            break;

        // APB2
        case (uint8_t)RCC_apb_SYSCFG ...(uint8_t)RCC_apb_DFDM_1:
            outRegister = (uint32_t *)&RCC->APB2RSTR;
            break;

        default:
            // Should never get here as input value is already checked
            break;
    };

    return outRegister;
}

static uint32_t *RCC_getEnReg(uint8_t inPeripheral) {
    uint32_t *outRegister;

    switch(inPeripheral) {
        // AHB1
        case (uint8_t)RCC_ahb_DMA_1 ... (uint8_t)RCC_ahb_TSC:
            outRegister = (uint32_t *)&RCC->AHB1ENR;
            break;

        // AHB2
        case (uint8_t)RCC_ahb_GPIO_A ... (uint8_t)RCC_ahb_RNG:
            outRegister = (uint32_t *)&RCC->AHB2ENR;
            break;

        // AHB3
        case (uint8_t)RCC_ahb_FMC ... (uint8_t)RCC_ahb_QSPI:
            outRegister = (uint32_t *)&RCC->AHB3ENR;
            break;

        // APB1_1
        case (uint8_t)RCC_apb_TIM_2 ... (uint8_t)RCC_apb_LPTIM_1:
            outRegister = (uint32_t *)&RCC->APB1ENR1;
            break;

        // APB1_2
        case (uint8_t)RCC_apb_LPUART_1 ... (uint8_t)RCC_apb_LPTIM_2:
            outRegister = (uint32_t *)&RCC->APB1ENR2;
            break;

        // APB2
        case (uint8_t)RCC_apb_SYSCFG ...(uint8_t)RCC_apb_DFDM_1:
            outRegister = (uint32_t *)&RCC->APB2ENR;
            break;

        default:
            // Should never get here as input value is already checked
            break;
    };

    return outRegister;
}

#endif // DRV_RCC_C

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/