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
 * @author ivan.juresa (scaluza.com)
 * @brief  Reset and Clock Control header file.
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
#define DRV_RCC_HSI16_FREQUENCY_MHZ (16u) //!< Default frequency of HSI16 oscillator

#define DRV_RCC_SYSCLK_MAX_FREQ (80) //!< Maximum frequency on System Clock
#define DRV_RCC_PLL_48_MHZ_CLK_FRQ (48) //!< In case 48MHz is used

/* Boundaries limitations */

//! PLL_N
#define DRV_RCC_PLL_N_MIN (8u)
#define DRV_RCC_PLL_N_MAX (86)

//! VCO input frequency. Controlled with ::DRV_RCC_PLL_m_E
#define RCC_VCO_INPUT_MIN_FREQ (4u)  //!< 4MHz
#define RCC_VCO_INPUT_MAX_FREQ (16u) //!< 16MHz

//! VCO output frequency
#define RCC_VCO_OUTPUT_MIN_FREQ (64u)  //!< 64MHz
#define RCC_VCO_OUTPUT_MAX_FREQ (344u) //!< 344MHz

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

//! MSI frequency ranges
typedef enum DRV_RCC_MSI_freq_ENUM {
    RCC_MSI_freq_100kHz = 0u,
    RCC_MSI_freq_200kHz = 1u,
    RCC_MSI_freq_400kHz = 2u,
    RCC_MSI_freq_800kHz = 3u,
    RCC_MSI_freq_1MHz   = 4u,
    RCC_MSI_freq_2MHz   = 5u,
    RCC_MSI_freq_4MHz   = 6u,
    RCC_MSI_freq_8MHz   = 7u,
    RCC_MSI_freq_16MHz  = 8u,
    RCC_MSI_freq_24MHz  = 9u,
    RCC_MSI_freq_32MHz  = 10u,
    RCC_MSI_freq_48MHz  = 11u,
    RCC_MSI_freq_COUNT  = 12u
} DRV_RCC_MSI_freq_E;

//! System clock candidates
typedef enum DRV_RCC_sysClk_ENUM {
    RCC_sysClk_HSI_16 = 0u,
    RCC_sysClk_MSI    = 1u,
    RCC_sysClk_HSE    = 2u,
    RCC_sysClk_PLL    = 3u,
    RCC_sysClk_COUNT  = 4u
} DRV_RCC_sysClk_E;

//! APB division factors. HCLK is divided by it
typedef enum DRV_RCC_APB_prescaler_ENUM {
    RCC_APB_prescaler_1     = 0u, //!< Not divided
    RCC_APB_prescaler_2     = 1u,
    RCC_APB_prescaler_4     = 2u,
    RCC_APB_prescaler_8     = 3u,
    RCC_APB_prescaler_16    = 4u,
    RCC_APB_prescaler_COUNT = 5u
} DRV_RCC_APB_prescaler_E;

//! AHB division factors. SYSCLK is divided by it
typedef enum DRV_RCC_AHB_prescaler_ENUM {
    RCC_AHB_prescaler_1     = 0u, //!< Not divided
    RCC_AHB_prescaler_2     = 1u,
    RCC_AHB_prescaler_4     = 2u,
    RCC_AHB_prescaler_8     = 3u,
    RCC_AHB_prescaler_16    = 4u,
    RCC_AHB_prescaler_64    = 5u,
    RCC_AHB_prescaler_128   = 6u,
    RCC_AHB_prescaler_256   = 7u,
    RCC_AHB_prescaler_512   = 8u,
    RCC_AHB_prescaler_COUNT = 9u
} DRV_RCC_AHB_prescaler_E;

/** PLL Configuration **/
//! PPL input clock source
typedef enum DRV_RCC_PLL_inputClock_ENUM {
    RCC_PLL_inputClock_MSI    = 0u,
    RCC_PLL_inputClock_HSI_16 = 1u,
    RCC_PLL_inputClock_HSE    = 2u,
    RCC_PLL_inputClock_COUNT  = 3u
} DRV_RCC_PLL_inputClock_E;

//! PLL division factor for PLLCLK (System Clock) or (48MHz clock)
typedef enum DRV_RCC_PLL_r_q_ENUM {
    RCC_PLL_r_q_2     = 0u,
    RCC_PLL_r_q_4     = 1u,
    RCC_PLL_r_q_6     = 2u,
    RCC_PLL_r_q_8     = 3u,
    RCC_PLL_r_q_COUNT = 4u,
} DRV_RCC_PLL_r_q_E;

//! Main PLL division factor for PLLSAI3CLK (SAI1 and SAI2 clock)
typedef enum DRV_RCC_PLL_p_ENUM {
    RCC_PLL_p_7     = 0u,
    RCC_PLL_p_17    = 1u,
    RCC_PLL_p_COUNT = 2u
} DRV_RCC_PLL_p_E;

//! Division factor for PPL, SAI1 and SAI2 clocks. It will divide input clock before VCO calculation.
typedef enum DRV_RCC_PLL_m_ENUM {
    RCC_PLL_m_1     = 0u,
    RCC_PLL_m_2     = 1u,
    RCC_PLL_m_3     = 2u,
    RCC_PLL_m_4     = 3u,
    RCC_PLL_m_5     = 4u,
    RCC_PLL_m_6     = 5u,
    RCC_PLL_m_7     = 6u,
    RCC_PLL_m_8     = 7u,
    RCC_PLL_m_COUNT = 8u
} DRV_RCC_PLL_m_E;

//! PLL devices
typedef enum DRV_RCC_PLL_device_ENUM {
    RCC_PLL_device_MAIN  = 0u,
    RCC_PLL_device_SAI1  = 1u,
    RCC_PLL_device_SAI2  = 2u,
    RCC_PLL_device_COUNT = 3u
} DRV_RCC_PLL_device_E;

//! PLL Outputs
typedef enum DRV_RCC_PLL_output_ENUM {
    RCC_PLL_output_R     = 0u,
    RCC_PLL_output_Q     = 1u,
    RCC_PLL_output_P     = 2u,
    RCC_PLL_output_COUNT = 3u
} DRV_RCC_PLL_output_E;

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! PLL Configuration structure
typedef struct RCC_PLL_config_STRUCT {
    uint32_t pll_R; //!< Clock division, SysClk or ADCs ::RCC_PLL_r_q_COUNT.
    uint32_t pll_Q; //!< 48MHz clock division ::RCC_PLL_r_q_COUNT
    uint32_t pll_P; //!< PLL SAI1 and SAI2 output division factor ::DRV_RCC_PLL_p_E
    uint32_t pll_N; //!< Main VCO multiplication factor. (8 - 86 are valid values). VCO output
                       //!  frequency should be between 64 and 344MHz

    //! If not used clocks should be turned OFF to save power
    bool_t isClkUsed_R; //!< Flag indicating if PLL output clock with R division factor is used
    bool_t isClkUsed_Q; //!< Flag indicating if PLL output clock with Q division factor is used
    bool_t isClkUsed_P; //!< Flag indicating if PLL output clock with P division factor is used
} RCC_PLL_config_S;

//! PPL Configuration structure
typedef struct DRV_RCC_PLL_config_STRUCT {
    uint8_t inputClock; //!< PLL input clock ::DRV_RCC_PLL_inputClock_E
    uint32_t pllClk_M; //!< Division input clock factor, before the VCO ::DRV_RCC_PLL_m_E. VCO input
                       //!  frequency ranges must be between 4 and 16MHz
    RCC_PLL_config_S config[RCC_PLL_device_COUNT]; //!< Configuration structures for PLL devices
} DRV_RCC_PLL_config_S;

//! MSI Configuration structure
typedef struct RCC_MSI_config_STRUCT {
    uint8_t freq; //!< ::DRV_RCC_MSI_freq_E
    bool_t hwAutoCalibration; //!< Flag indicating if MSI will be auto calibrated using LSE external
                              //!  oscillator
} RCC_MSI_config_S;

//! RCC Configuration structure
typedef struct DRV_RCC_config_STRUCT {
    uint8_t systemClockSrc; //!< Chosen clock for System Clock ::DRV_RCC_sysClk_E
    uint8_t prescalerAhb; //!< ::DRV_RCC_AHB_prescaler_E
    uint8_t prescalerApb1; //!< ::DRV_RCC_APB_prescaler_E
    uint8_t prescalerApb2; //!< ::DRV_RCC_APB_prescaler_E
    uint8_t hseFreq; //!< Frequency of HSE oscillator. 4MHz - 48MHz
    RCC_MSI_config_S msiConfig; //!< MSI configuration structure
    DRV_RCC_PLL_config_S pllConfig; //!< PLL configuration structure
} DRV_RCC_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
/***************************************************************************************************
 * @brief   Clock initialization function.
 * @details Function allows input configuration to be NLL_PTR. In that case it will use default
 *          static configuration:
 *              - MSI as an input clock at 4 MHz
 *              - PLL used as a System Clock
 *              - The rest is OFF
 * *************************************************************************************************
 * @param   [in]      *inConfig - Pointer to DRV_RCC configuration structure ::DRV_RCC_config_S
 * @param   [out]     *outErr   - Output driver error enumerator
 * *************************************************************************************************
 * @exceptions        ERROR_err_ARGS_OUT_OF_RANGE: Input arguments are wrong
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void DRV_RCC_init(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 * @brief   Function is used to reset input AHB or APB peripheral to its default state.
 * @details Input peripheral will reset its status to the same state as when MCU boots up.
 * *************************************************************************************************
 * @param   [in]       inPeripheral - Input AHB/APB peripheral ID ::DRV_RCC_ahb_apb_E
 * @param   [out]     *outErr       - Output driver error enumerator
 * *************************************************************************************************
 * @exceptions        ERROR_err_ARGS_OUT_OF_RANGE: Input peripheral does not exist, out of boundaries
 *                                                 or reset for that peripheral is not supported.
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
 * @param   [in]       inPeripheral - Input AHB/APB peripheral ID ::DRV_RCC_ahb_apb_E
 * @param   [in]       inVal        - Disable (FALSE) or Enable (TRUE) Clock
 * @param   [out]     *outErr       - Output driver error enumerator
 * *************************************************************************************************
 * @exceptions        ERROR_err_ARGS_OUT_OF_RANGE: Input peripheral does not exist, out of boundaries
 *                                                 or reset for that peripheral is not supported.
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void DRV_RCC_peripheralEnable(uint8_t inPeripheral, bool_t inVal, DRV_ERROR_err_E *outErr);

#endif /* DRV_RCC_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
