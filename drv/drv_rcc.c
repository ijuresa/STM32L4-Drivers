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
 * @author ivan.juresa (scaluza.com)
 * @brief  Driver will initialize Clock for STM32L4 processor.
 **************************************************************************************************/

#ifndef DRV_RCC_C_
#define DRV_RCC_C

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"
#include "string.h"
#include "stm32l475xx.h"

// DRV
#include "drv_rcc.h"

/***************************************************************************************************
 *                      PRIVATE DEFINES
 **************************************************************************************************/
#define SNA_VAL (0xFFu) //!< Signal not available value. Used when some feature is not supported

#define FLASH_LATENCY_WS_16 (16u) //!< Wait 1 CPU Clock Cycle for frequencies below or 16MHz
#define FLASH_LATENCY_WS_32 (32u) //!< Wait 2 CPU Clock Cycle for frequencies below or 32MHz
#define FLASH_LATENCY_WS_48 (48u) //!< Wait 3 CPU Clock Cycle for frequencies below or 48MHz
#define FLASH_LATENCY_WS_64 (64u) //!< Wait 4 CPU Clock Cycle for frequencies below or 64MHz
#define FLASH_LATENCY_WS_80 (80u) //!< Wait 5 CPU Clock Cycle for frequencies below or 80MHz

#define RCC_KHZ_IN_MHZ (1000000u) //!< Used to convert KHz to MHz and vice versa

/***************************************************************************************************
 *                      PRIVATE DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE VARIABLES
 **************************************************************************************************/
//! Default clock configuration structure. It will be used in case none is supplied
static DRV_RCC_config_S DRV_RCC_localConfig = {
    .systemClockSrc = (uint8_t)RCC_sysClk_PLL,
    .prescalerAhb = (uint8_t)RCC_AHB_prescaler_1,
    .prescalerApb1 = (uint8_t)RCC_APB_prescaler_1,
    .prescalerApb2 = (uint8_t)RCC_APB_prescaler_1,
    .hseFreq = 0u,
    .msiConfig = {
            .freq = (uint8_t)RCC_MSI_freq_4MHz,
            .hwAutoCalibration = TRUE
    },
    .pllConfig = {
        .inputClock = (uint8_t)RCC_PLL_inputClock_MSI,
        .pllClk_M = (uint8_t)RCC_PLL_m_1,
        .config = {
            /** MAIN_PLL **/
            {
                .pll_R = (uint8_t)RCC_PLL_r_q_2,
                // Don't care about PLL_Q and PLL_P as they are not used
                .pll_N = 40u,
                .isClkUsed_R = TRUE,  //! It is used as a System Clock
                .isClkUsed_Q = FALSE, //! Don't need 48MHz clock
                .isClkUsed_P = FALSE  //! We are not using SAI1 or SAI2 clocks
            },
            /** SAI1 PLL **/
            {
                .isClkUsed_R = FALSE, //! We are not using ADC from PLLSAI1
                .isClkUsed_Q = FALSE, //! Use this clock for USB, RNG...
                .isClkUsed_P = FALSE  //! We are not using SAI1 or SAI2 clocks
            },
            /** SAI2 PLL**/
            {
                // Don't care as it is not used
                .isClkUsed_R = FALSE, //! We are not using ADC from PLLSAI2
                .isClkUsed_Q = FALSE, //! This is not even connected to anything
                .isClkUsed_P = FALSE  //! We are not using SAI1 or SAI2 clocks
            }
        }
    }
};

//! VCO Input Clock Frequency = (PLL_clock_input / PLL_M)
static fp32_t vcoPllInputClockFreq[RCC_PLL_device_COUNT] = { 0.0f, 0.0f, 0.0f };

//! VCO Output Clock Frequency = (VCO_input_clock_frequency * PLL_N)
static fp32_t vcoPllOutputClockFreq[RCC_PLL_device_COUNT] = { 0.0f, 0.0f, 0.0f };

//! SysClk frequency in case PLL is used
static fp32_t pllSysClk = 0u;

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

//! AHB divider setup values. Check divider possible values at ::DRV_RCC_AHB_prescaler_E
static const uint32_t ahbDividerVal[RCC_AHB_prescaler_COUNT] = {
    (0u),                                                                   //! 0xxx
    (RCC_CFGR_HPRE_3),                                                      //! 1000
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0),                                    //! 1001
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1),                                    //! 1010
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0),                  //! 1011
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2),                                    //! 1100
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_0),                  //! 1101
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1),                  //! 1110
    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0) //! 1111
};

//! APB_01 divider setup values. Check divider possible values at ::DRV_RCC_APB_prescaler_E
static const uint32_t apbDividerVal_01[RCC_APB_prescaler_COUNT] = {
    (0u),                                                    //! 0xx
    (RCC_CFGR_PPRE1_2),                                      //! 100
    (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0),                   //! 101
    (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1),                   //! 110
    (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE1_0) //! 111
};

//! APB_02 divider setup values. Check divider possible values at ::DRV_RCC_APB_prescaler_E
static const uint32_t apbDividerVal_02[RCC_APB_prescaler_COUNT] = {
    (0u),                                                    //! 0xx
    (RCC_CFGR_PPRE2_2),                                      //! 100
    (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_0),                   //! 101
    (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1),                   //! 110
    (RCC_CFGR_PPRE2_2 | RCC_CFGR_PPRE2_1 | RCC_CFGR_PPRE2_0) //! 111
};

//! MSI clock frequency setup values. Check possible values at ::DRV_RCC_MSI_freq_E
static const uint32_t msiClockFrequency[RCC_MSI_freq_COUNT] = {
    RCC_CR_MSIRANGE_0,
    RCC_CR_MSIRANGE_1,
    RCC_CR_MSIRANGE_2,
    RCC_CR_MSIRANGE_3,
    RCC_CR_MSIRANGE_4,
    RCC_CR_MSIRANGE_5,
    RCC_CR_MSIRANGE_6,
    RCC_CR_MSIRANGE_7,
    RCC_CR_MSIRANGE_8,
    RCC_CR_MSIRANGE_9,
    RCC_CR_MSIRANGE_10,
    RCC_CR_MSIRANGE_11
};

//! PLL_P_Q divider real numbers
static const uint8_t pllPQDivider[RCC_PLL_r_q_COUNT] = {
    2u, //! RCC_PLL_r_q_2
    4u, //! RCC_PLL_r_q_4
    6u, //! RCC_PLL_r_q_6
    8u  //! RCC_PLL_r_q_8
};

//! Flag indicating if any PLL output is used. General one so we don't need to go through FOR loop
static bool_t isPllUsed = FALSE;

//! Flags indicating if PLL outputs are used
static bool_t isPllDeviceUsed[RCC_PLL_device_COUNT] = {
    FALSE, //! PLL MAIN
    FALSE, //! PLL SAI1
    FALSE  //! PLL SAI2
};

/***************************************************************************************************
 *                      GLOBAL VARIABLES DEFINITION
 **************************************************************************************************/

/***************************************************************************************************
 *                      PRIVATE FUNCTION DECLARATION
 **************************************************************************************************/
static void RCC_configureHsi16(void);
static void RCC_configureMsi(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr);
static void RCC_configureHse(void);
static void RCC_configurePllClocks(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr);
static void RCC_configurePll_R(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr);
static void RCC_configurePll_Q(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr);
static void RCC_configurePll_P(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr);
static void RCC_calculatePllVco(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr);
static void RCC_configureLse(DRV_ERROR_err_E *outErr);
static void RCC_setAsSystemClock(uint8_t inSysClkSrc, DRV_ERROR_err_E *outErr);
static void RCC_configureFlashLatency(DRV_RCC_config_S *inConfig);
static uint32_t *RCC_getRstReg(uint8_t inPeripheral);
static uint32_t *RCC_getEnReg(uint8_t inPeripheral);
static uint32_t RCC_getMsiFrequency(uint8_t inMsiFreq, DRV_ERROR_err_E *outErr);

/***************************************************************************************************
 *                      PUBLIC FUNCTIONS DEFINITION
 **************************************************************************************************/
void DRV_RCC_init(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    if(outErr != NULL_PTR) {
        if(inConfig != NULL_PTR) {
            // User submitted custom RCC configuration. Use it
            (void)memcpy((void *)&DRV_RCC_localConfig, inConfig, sizeof(DRV_RCC_config_S));
        }

        // Do global checks
        if((DRV_RCC_localConfig.prescalerAhb >= (uint8_t)RCC_AHB_prescaler_COUNT)
                || (DRV_RCC_localConfig.prescalerApb1 >= (uint8_t)RCC_APB_prescaler_COUNT)
                || (DRV_RCC_localConfig.prescalerApb2 >= (uint8_t)RCC_APB_prescaler_COUNT)
                || (DRV_RCC_localConfig.systemClockSrc >= (uint8_t)RCC_sysClk_COUNT)) {
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
        } else {
            *outErr = ERROR_err_OK;

            if(DRV_RCC_localConfig.systemClockSrc == (uint8_t)RCC_sysClk_PLL) {
                // In case developer is trying to use PLL as a System Clock but forgot to enable it
                DRV_RCC_localConfig.pllConfig.config[RCC_PLL_device_MAIN].isClkUsed_R = TRUE;
            }

            // Before doing anything set flash latency to maximum value. With this we are avoiding
            // pre-calculating problem. Basically it means we don't need to divide clock frequency
            // calculations with initialization. Flash write delay will be written to correct
            // value right before new System Clock is set in ::RCC_configureFlashLatency
            FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | FLASH_ACR_LATENCY_4WS;

            // Do universal PLL VCO calculations, in case PLL is used
            RCC_calculatePllVco(&DRV_RCC_localConfig, outErr);

            if(*outErr == ERROR_err_OK) {
                // Configure PLL Devices
                RCC_configurePllClocks(&DRV_RCC_localConfig, outErr);
            }

            if(*outErr == ERROR_err_OK) {
                // Set requested clock
                switch(DRV_RCC_localConfig.systemClockSrc) {
                    case (uint8_t)RCC_sysClk_HSI_16:
                        RCC_configureHsi16();
                        break;

                    case (uint8_t)RCC_sysClk_MSI:
                        RCC_configureMsi(&DRV_RCC_localConfig, outErr);
                        break;

                    case (uint8_t)RCC_sysClk_HSE:
                        RCC_configureHse();
                        break;

                    case (uint8_t)RCC_sysClk_PLL:
                        // PLL is configured before switch case as there are multiple devices and
                        // clocks which are not dependent.
                        break;

                    default:
                        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                        break;
                }
            }

            if(*outErr == ERROR_err_OK) {
                RCC_configureFlashLatency(&DRV_RCC_localConfig);
                RCC_setAsSystemClock(DRV_RCC_localConfig.systemClockSrc, outErr);

                // Set AHB, APB1 and APB2
                RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_HPRE)) | ahbDividerVal[DRV_RCC_localConfig.prescalerAhb]);
                RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_PPRE1)) | apbDividerVal_01[DRV_RCC_localConfig.prescalerApb1]);
                RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_PPRE2)) | apbDividerVal_02[DRV_RCC_localConfig.prescalerApb2]);
            }
        }
    }
}

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
            if((ahbApbAddr != NULL_PTR) && (ahbApbEnVal[inPeripheral] != SNA_VAL)) {
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
static void RCC_configureHsi16(void) {
    // Enable HSI16 clock
    RCC->CR |= RCC_CR_HSION;

    // Wait for HSI to be ready
    while((RCC->CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY);
}

static void RCC_configureMsi(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    if(inConfig->msiConfig.freq >= (uint8_t)RCC_MSI_freq_COUNT) {
        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    } else {
        // Use MSIRANGE from RCC_CR register
        RCC->CR |= RCC_CR_MSIRGSEL;

        // Set MSI requested clock frequency
        RCC->CR = ((RCC->CR & (~RCC_CR_MSIRANGE)) | msiClockFrequency[inConfig->msiConfig.freq]);

        if(inConfig->msiConfig.hwAutoCalibration == TRUE) {
            // Configure LSE for hardware auto calibration
            RCC_configureLse(outErr);

            if(*outErr == ERROR_err_OK) {
                // Select PLL Mode
                RCC->CR |= RCC_CR_MSIPLLEN;
            }
        }
    }
}

static void RCC_configureHse(void) {
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;

    // Wait until its ready
    while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);
}

static void RCC_configurePllClocks(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    uint8_t device;
    uint8_t clock;

    // Disable MAIN, SAI1 and SAI2 PLL
    RCC->CR &= ((~RCC_CR_PLLON) & (~RCC_CR_PLLSAI1ON) & (~RCC_CR_PLLSAI2ON));

    // Wait for them to stop
    while(((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY)
            && ((RCC->CR & RCC_CR_PLLSAI1RDY) == RCC_CR_PLLSAI1RDY)
            && ((RCC->CR & RCC_CR_PLLSAI2RDY) == RCC_CR_PLLSAI2RDY));

    if(isPllUsed == TRUE) {
        // Write shared divider (M)
        RCC->PLLCFGR = ((RCC->PLLCFGR & (~RCC_PLLCFGR_PLLM_Msk))
                            | (inConfig->pllConfig.pllClk_M << RCC_PLLCFGR_PLLM_Pos));

        // Configure PLL Input Clock
        switch(inConfig->pllConfig.inputClock) {
            case (uint8_t)RCC_PLL_inputClock_MSI:
                RCC_configureMsi(inConfig, outErr);
                break;

            case (uint8_t)RCC_PLL_inputClock_HSI_16:
                RCC_configureHsi16();
                break;

            case (uint8_t)RCC_PLL_inputClock_HSE:
                RCC_configureHse();
                break;

            default:
                // Should never enter here
                break;
        }

        // Set PLL input clock. +1 as we are going from 0 but register values from 1
        RCC->PLLCFGR = ((RCC->PLLCFGR & (~RCC_PLLCFGR_PLLSRC)) | (inConfig->pllConfig.inputClock + 1u));

        // Configure clocks
        for(device = 0; device < (uint8_t)RCC_PLL_device_COUNT; device ++) {
            if(isPllDeviceUsed[device] == TRUE) {
                // Write device shared multiplier (N)
                switch(device) {
                    case (uint8_t)RCC_PLL_device_MAIN:
                        RCC->PLLCFGR |= (inConfig->pllConfig.config[device].pll_N << RCC_PLLCFGR_PLLN_Pos);
                        break;

                    case (uint8_t)RCC_PLL_device_SAI1:
                        RCC->PLLSAI1CFGR |= (inConfig->pllConfig.config[device].pll_N << RCC_PLLSAI1CFGR_PLLSAI1N_Pos);
                        break;

                    case (uint8_t)RCC_PLL_device_SAI2:
                        RCC->PLLSAI2CFGR |= (inConfig->pllConfig.config[device].pll_N << RCC_PLLSAI2CFGR_PLLSAI2N_Pos);
                        break;

                    default:
                        // Should never enter here as its protected by FOR loop
                        break;
                }

                // Configure particular clock dividers
                for(clock = 0; clock < (uint8_t)RCC_PLL_output_COUNT; clock ++) {
                    switch(clock) {
                        case (uint8_t)RCC_PLL_output_R:
                            if(inConfig->pllConfig.config[device].isClkUsed_R == TRUE) {
                                RCC_configurePll_R(inConfig, device, outErr);
                            }
                            break;

                        case (uint8_t)RCC_PLL_output_Q:
                            if(inConfig->pllConfig.config[device].isClkUsed_Q == TRUE) {
                                RCC_configurePll_Q(inConfig, device, outErr);
                            }
                            break;

                        case (uint8_t)RCC_PLL_output_P:
                            if(inConfig->pllConfig.config[device].isClkUsed_P == TRUE) {
                                RCC_configurePll_P(inConfig, device, outErr);
                            }
                            break;

                        default:
                            // Should never enter here as its protected by FOR loop
                           break;
                    }
                }
            }
        }

        // Clocks are configured. Now we can safely enable needed ones
        if(*outErr == ERROR_err_OK) {
            // Enable PLL
            RCC->CR |= RCC_CR_PLLON;

            // Wait for HW to indicate it is ready
            while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

            // Now for each PLL device enable or disable (in case it is not used) output clocks
            for(device = 0; device < (uint8_t)RCC_PLL_device_COUNT; device ++) {
                switch(device) {
                    case (uint8_t)RCC_PLL_device_MAIN:
                        if(inConfig->pllConfig.config[device].isClkUsed_R == TRUE) {
                            RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
                        } else {
                            RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLREN);
                        }

                        if(inConfig->pllConfig.config[device].isClkUsed_Q == TRUE) {
                            RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
                        } else {
                            RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLQEN);
                        }

                        if(inConfig->pllConfig.config[device].isClkUsed_P == TRUE) {
                            RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
                        } else {
                            RCC->PLLCFGR &= (~RCC_PLLCFGR_PLLPEN);
                        }
                        break;

                    case (uint8_t)RCC_PLL_device_SAI1:
                        if(inConfig->pllConfig.config[device].isClkUsed_R == TRUE) {
                            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
                        } else {
                            RCC->PLLSAI1CFGR &= (~RCC_PLLSAI1CFGR_PLLSAI1REN);
                        }

                        if(inConfig->pllConfig.config[device].isClkUsed_Q == TRUE) {
                            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;
                        } else {
                            RCC->PLLSAI1CFGR &= (~RCC_PLLSAI1CFGR_PLLSAI1QEN);
                        }

                        if(inConfig->pllConfig.config[device].isClkUsed_P == TRUE) {
                            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;
                        } else {
                            RCC->PLLSAI1CFGR &= (~RCC_PLLSAI1CFGR_PLLSAI1PEN);
                        }
                        break;

                    case (uint8_t)RCC_PLL_device_SAI2:
                        if(inConfig->pllConfig.config[device].isClkUsed_R == TRUE) {
                            RCC->PLLSAI2CFGR |= RCC_PLLSAI2CFGR_PLLSAI2REN;
                        } else {
                            RCC->PLLSAI2CFGR &= (~RCC_PLLSAI2CFGR_PLLSAI2REN);
                        }

                        // Q missing

                        if(inConfig->pllConfig.config[device].isClkUsed_P == TRUE) {
                            RCC->PLLSAI2CFGR |= RCC_PLLSAI2CFGR_PLLSAI2PEN;
                        } else {
                            RCC->PLLSAI2CFGR &= (~RCC_PLLSAI2CFGR_PLLSAI2PEN);
                        }
                        break;

                    default:
                        // Should never enter here as its protected by FOR loop
                        break;
                }
            }
        }
    }
}

static void RCC_configurePll_R(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr) {
    fp32_t outClock = (vcoPllOutputClockFreq[inDevice] / pllPQDivider[inConfig->pllConfig.config[inDevice].pll_R]);

    // Output clock can't be bigger than 80MHz
    if(outClock <= DRV_RCC_SYSCLK_MAX_FREQ) {
        switch(inDevice) {
            case (uint8_t)RCC_PLL_device_MAIN:
                // Write divider for SysTick. PLLR
                RCC->PLLCFGR = ((RCC->PLLCFGR & (~RCC_PLLCFGR_PLLR_Msk))
                                    | (inConfig->pllConfig.config[inDevice].pll_R << RCC_PLLCFGR_PLLR_Pos));
                pllSysClk = outClock;
                break;

            case (uint8_t)RCC_PLL_device_SAI1:
                // Write divider for ADC. PLLR
                RCC->PLLSAI1CFGR = ((RCC->PLLSAI1CFGR & (~RCC_PLLSAI1CFGR_PLLSAI1R_Msk))
                                    | (inConfig->pllConfig.config[inDevice].pll_R << RCC_PLLSAI1CFGR_PLLSAI1R_Pos));
                break;

            case (uint8_t)RCC_PLL_device_SAI2:
                // Write divider for ADC. PLLR
                RCC->PLLSAI2CFGR = ((RCC->PLLSAI2CFGR & (~RCC_PLLSAI2CFGR_PLLSAI2R_Msk))
                                    | (inConfig->pllConfig.config[inDevice].pll_R << RCC_PLLSAI2CFGR_PLLSAI2R_Pos));
                break;

            default:
                // Should never enter here as its protected by FOR loop
                break;
        }
    } else {
        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    }
}

static void RCC_configurePll_Q(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr) {
    fp32_t outClock = (vcoPllOutputClockFreq[inDevice] / pllPQDivider[inConfig->pllConfig.config[inDevice].pll_Q]);
    if(outClock <= DRV_RCC_PLL_48_MHZ_CLK_FRQ) {
        switch(inDevice) {
            case (uint8_t)RCC_PLL_device_MAIN:
                RCC->PLLCFGR |= (inConfig->pllConfig.config[inDevice].pll_Q << RCC_PLLCFGR_PLLQ_Pos);
                break;

            case (uint8_t)RCC_PLL_device_SAI1:
                RCC->PLLSAI1CFGR |= (inConfig->pllConfig.config[inDevice].pll_Q << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos);
                break;

            case (uint8_t)RCC_PLL_device_SAI2:
                // Its not connected to the anything
                break;

            default:
                // Should never enter here as its protected by FOR loop
                break;
        }
    } else {
        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    }
}

static void RCC_configurePll_P(DRV_RCC_config_S *inConfig, uint8_t inDevice, DRV_ERROR_err_E *outErr) {
    switch(inDevice) {
        case (uint8_t)RCC_PLL_device_MAIN:
            break;

        case (uint8_t)RCC_PLL_device_SAI1:
            break;

        case (uint8_t)RCC_PLL_device_SAI2:
            break;

        default:
            // Should never enter here as its protected by FOR loop
            break;
    }
}

static void RCC_calculatePllVco(DRV_RCC_config_S *inConfig, DRV_ERROR_err_E *outErr) {
    uint8_t inputRawFreq;
    fp32_t lVcoInputFreq = 0.0f;
    fp32_t lVcoOutputFreq = 0.0f;
    uint8_t i;

    if(inConfig->pllConfig.pllClk_M >= (uint8_t)RCC_PLL_m_COUNT) {
        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    } else {
        for(i = 0; i < (uint8_t)RCC_PLL_device_COUNT; i ++) {
            // Check if any output clock will be used
            if((inConfig->pllConfig.config[i].isClkUsed_R == TRUE)
                || (inConfig->pllConfig.config[i].isClkUsed_Q == TRUE)
                || (inConfig->pllConfig.config[i].isClkUsed_P == TRUE)) {
                // Some output on current PLL will be used
                if((inConfig->pllConfig.config[i].pll_N < DRV_RCC_PLL_N_MIN)
                        || (inConfig->pllConfig.config[i].pll_N > DRV_RCC_PLL_N_MAX)) {
                    *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                }
                isPllUsed = TRUE;
                isPllDeviceUsed[i] = TRUE;
            }

            // In case PLL device is used check what outputs are used
            if(isPllDeviceUsed[i] == TRUE) {
                // Check now independent outputs
                if(inConfig->pllConfig.config[i].isClkUsed_R == TRUE) {
                    if(inConfig->pllConfig.config[i].pll_R >= (uint8_t)RCC_PLL_r_q_COUNT) {
                        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                    }
                }

                if(inConfig->pllConfig.config[i].isClkUsed_Q == TRUE) {
                    if(inConfig->pllConfig.config[i].pll_Q >= (uint8_t)RCC_PLL_r_q_COUNT) {
                        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                    }
                }

                if(inConfig->pllConfig.config[i].isClkUsed_P == TRUE) {
                    if(inConfig->pllConfig.config[i].pll_P >= (uint8_t)RCC_PLL_p_COUNT) {
                        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                    }
                }
            }
        }
    }

    if((*outErr == ERROR_err_OK) && (isPllUsed == TRUE)) {
        // Figure out by which clock we are supplied and how fast it is
        switch(inConfig->pllConfig.inputClock) {
            case (uint8_t)RCC_PLL_inputClock_MSI:
                // VCO input frequency must be between 4MHz and 16MHz so if MSI is below 4MHz its unusable
                if(inConfig->msiConfig.freq < (uint8_t)RCC_MSI_freq_4MHz) {
                    *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                } else {
                    // Get frequency in KHz and convert it to MHz
                    inputRawFreq = (RCC_getMsiFrequency(inConfig->msiConfig.freq, outErr) / RCC_KHZ_IN_MHZ);
                }
                break;

            case (uint8_t)RCC_PLL_inputClock_HSI_16:
                // HSI has default frequency of 16MHz
                inputRawFreq = DRV_RCC_HSI16_FREQUENCY_MHZ;
                break;

            case (uint8_t)RCC_PLL_inputClock_HSE:
                inputRawFreq = inConfig->hseFreq;
                break;

            default:
                *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                break;
        }

        if(*outErr == ERROR_err_OK) {
            // Calculate VCO frequencies for each used PLL device
            for(i = 0; i < (uint8_t)RCC_PLL_device_COUNT; i ++) {
                if(isPllDeviceUsed[i] == TRUE) {
                    // Calculate VCO Input frequency. +1 as M has an enumerator value and going from 0
                    lVcoInputFreq = (inputRawFreq / (inConfig->pllConfig.pllClk_M + 1u));
                    if((lVcoInputFreq < RCC_VCO_INPUT_MIN_FREQ)
                            || (lVcoInputFreq > RCC_VCO_INPUT_MAX_FREQ)) {
                        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                    } else {
                        vcoPllInputClockFreq[i] = lVcoInputFreq;
                    }

                    if(*outErr == ERROR_err_OK) {
                        // Calculate VCO Output frequency
                        lVcoOutputFreq = (lVcoInputFreq * inConfig->pllConfig.config[i].pll_N);

                        // Check for VCO output frequency
                        if((lVcoOutputFreq < RCC_VCO_OUTPUT_MIN_FREQ)
                                || (lVcoOutputFreq > RCC_VCO_OUTPUT_MAX_FREQ)) {
                            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
                        } else {
                            vcoPllOutputClockFreq[i] = lVcoOutputFreq;
                        }
                    }
                }
            }
        }
    }
}

static void RCC_configureLse(DRV_ERROR_err_E *outErr) {
    // Enable Clock for Power Control (PWR) peripheral first
    DRV_RCC_peripheralEnable((uint8_t)RCC_apb_PWR, TRUE, outErr);

    if(*outErr == ERROR_err_OK) {
        // After reset Backup Domain registers are write protected. Enable writes
        PWR->CR1 |= PWR_CR1_DBP;

        // Enable LSE
        RCC->BDCR |= RCC_BDCR_LSEON;

        // Wait for HW to notify that LSE is stable
        while((RCC->BDCR & (RCC_BDCR_LSERDY)) != RCC_BDCR_LSERDY);

        // Set protection back
        PWR->CR1 &= ~PWR_CR1_DBP;

        // Disable Clock for peripheral
        DRV_RCC_peripheralEnable((uint8_t)RCC_apb_PWR, FALSE, outErr);
    }
}

static void RCC_setAsSystemClock(uint8_t inSysClkSrc, DRV_ERROR_err_E *outErr) {
    switch(inSysClkSrc) {
        case (uint8_t)RCC_sysClk_HSI_16:
            // Select HSI16 as a System Clock
            RCC->CFGR |= ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI);

            // Wait for HW to indicate that HSI is indeed used as a System Clock
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
            break;

        case (uint8_t)RCC_sysClk_MSI:
            // Switch MSI to be used as a System Clock
            RCC->CFGR |= (RCC->CFGR & (~RCC_CFGR_SW));

            // Wait for HW to indicate that MSI is indeed used as a System Clock
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
            break;

        case (uint8_t)RCC_sysClk_HSE:
            // Switch HSE to be used as a System Clock
            RCC->CFGR |= ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSE);

            // Wait for HW to indicate that HSE is indeed used as a System Clock
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
            break;

        case (uint8_t)RCC_sysClk_PLL:
            // Switch PLL to be used as a System Clock
            RCC->CFGR |= ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL);

            // Wait for HW to indicate that PLL is indeed used as a System Clock
            while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
            break;

        default:
            *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    }
}

/** Flash read latency (Section 3.3.3) **/
static void RCC_configureFlashLatency(DRV_RCC_config_S *inConfig) {
    // Take default one
    uint32_t flashLatency = FLASH_ACR_LATENCY_0WS;

    switch(inConfig->systemClockSrc) {
        case (uint8_t)RCC_sysClk_HSI_16:
            // Do nothing as HSI_16 is 16MHz and it should have 0WS (Wait Cycles)
            break;

        case (uint8_t)RCC_sysClk_MSI:
            // Find speed of MSI
            switch(inConfig->msiConfig.freq) {
                case (uint8_t)RCC_MSI_freq_100kHz ... (uint8_t)RCC_MSI_freq_16MHz:
                    // Same as HSI_16. Do nothing
                    break;

                case (uint8_t)RCC_MSI_freq_24MHz:
                case (uint8_t)RCC_MSI_freq_32MHz:
                    flashLatency = FLASH_ACR_LATENCY_1WS;
                    break;

                case (uint8_t)RCC_MSI_freq_48MHz:
                    flashLatency = FLASH_ACR_LATENCY_2WS;
                    break;

                default:
                    // Should never happen
                    break;
            }
            break;

        case (uint8_t)RCC_sysClk_HSE:
            if(inConfig->hseFreq <= FLASH_LATENCY_WS_16) {
                // Do nothing as this is default value
            } else if(inConfig->hseFreq <= FLASH_LATENCY_WS_32) {
                flashLatency = FLASH_ACR_LATENCY_1WS;
            } else {
                // HSE is not supposed to be above 48MHz
                flashLatency = FLASH_ACR_LATENCY_2WS;
            }
            break;

        case (uint8_t)RCC_sysClk_PLL:
            if(pllSysClk <= FLASH_LATENCY_WS_16) {
                // Do nothing as this is default value
            } else if(pllSysClk <= FLASH_LATENCY_WS_32) {
                flashLatency = FLASH_ACR_LATENCY_1WS;
            } else if(pllSysClk <= FLASH_LATENCY_WS_48) {
                flashLatency = FLASH_ACR_LATENCY_2WS;
            } else if(pllSysClk <= FLASH_LATENCY_WS_64) {
                flashLatency = FLASH_ACR_LATENCY_3WS;
            } else {
                flashLatency = FLASH_ACR_LATENCY_4WS;
            }
            break;

        default:
        // Should never get here
        break;
    }

    FLASH->ACR = (FLASH->ACR & (~FLASH_ACR_LATENCY)) | flashLatency;
}

static uint32_t *RCC_getRstReg(uint8_t inPeripheral) {
    uint32_t *outRegister = NULL_PTR;

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
    uint32_t *outRegister = NULL_PTR;

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

static uint32_t RCC_getMsiFrequency(uint8_t inMsiFreq, DRV_ERROR_err_E *outErr) {
    // Output frequency in KHz
    uint32_t outFreq;

    static const uint32_t msiFreq[RCC_MSI_freq_COUNT] = {
        100000u,   //! RCC_MSI_freq_100kHz
        200000u,   //! RCC_MSI_freq_200kHz
        400000u,   //! RCC_MSI_freq_400kHz
        800000u,   //! RCC_MSI_freq_800kHz
        1000000u,  //! RCC_MSI_freq_1MHz
        2000000u,  //! RCC_MSI_freq_2MHz
        4000000u,  //! RCC_MSI_freq_4MHz
        8000000u,  //! RCC_MSI_freq_8MHz
        16000000u, //! RCC_MSI_freq_16MHz
        24000000u, //! RCC_MSI_freq_24MHz
        32000000u, //! RCC_MSI_freq_32MHz
        48000000u  //! RCC_MSI_freq_48MHz
    };

    if(inMsiFreq < (uint8_t)RCC_MSI_freq_COUNT) {
        outFreq = msiFreq[inMsiFreq];
    } else {
        *outErr = ERROR_err_ARGS_OUT_OF_RANGE;
    }

    return outFreq;
}

#endif // DRV_RCC_C

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
