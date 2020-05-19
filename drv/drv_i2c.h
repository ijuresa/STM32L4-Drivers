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
 * @file   drv_i2c.h
 * @author ivan.juresa
 * @brief  Inter-integrated Circuit (I2C) driver.
 **************************************************************************************************/

#ifndef DRV_I2C_H_
#define DRV_I2C_H_

/***************************************************************************************************
 *                      INCLUDE FILES
 **************************************************************************************************/
#include "typedefs.h"

// DRV
#include "drv_error.h"
#include "drv_rcc.h"

/***************************************************************************************************
 *                      DEFINES
 **************************************************************************************************/
/********************************* DATA SETUP AND HOLD TIMES **************************************/
//! Data hold time is 0 (for MIN and MAX)

//! Maximum data valid time
#define I2C_TVD_DAT_STANDARD_MODE_MAX  (3.45f) //!< 3.45 us
#define I2C_TVD_DAT_FAST_MODE_MAX      (0.9f)  //!< 0.9 us
#define I2C_TVD_DAT_FAST_PLUS_MODE_MAX (0.45f) //!< 0.45 us

//! Minimum data setup time
#define I2C_TSU_DAT_STANDARD_MODE_MIN  (0.25f) //!< 0.25 us
#define I2C_TSU_DAT_FAST_MODE_MIN      (0.1f)  //!< 0.1 us
#define I2C_TSU_DAT_FAST_PLUS_MODE_MIN (0.05f) //!< 0.05 us

/*************************************** CLOCK TIMINGS ********************************************/
//! Maximum SCL I2C clock frequency depending on the mode
#define I2C_SCL_CLOCK_FREQ_STANDARD_MODE_MAX  (100u)  //!< 100 KHz
#define I2C_SCL_CLOCK_FREQ_FAST_MODE_MAX      (400u)  //!< 400 KHz
#define I2C_SCL_CLOCK_FREQ_FAST_PLUS_MODE_MAX (1000u) //!< 1000 KHz

//! Minimum hold time (repeated) START condition: t_HD:STA
#define I2C_THD_STA_STANDARD_MODE_MIN  (4.0f) //!< 4 us
#define I2C_THD_STA_FAST_MODE_MIN      (0.6f) //!< 0.6 us
#define I2C_THD_STA_FAST_PLUS_MODE_MIN (0.26) //!< 0.26 us

//! Minimum set-up time for a (repeated) START condition: t_SU:STA
#define I2C_TSU_STA_STANDARD_MODE_MIN  (4.7f)  //!< 4.7 us
#define I2C_TSU_STA_FAST_MODE_MIN      (0.6f)  //!< 0.6 us
#define I2C_TSU_STA_FAST_PLUS_MODE_MIN (0.26f) //!< 0.26us

//! Minimum set-up time for STOP condition: t_SU_STO
#define I2C_TSU_STO_STANDARD_MODE_MIN  (4.0f)  //!< 4 us
#define I2C_TSU_STO_FAST_MODE_MIN      (0.6f)  //!< 0.6 us
#define I2C_TSU_STO_FAST_PLUS_MODE_MIN (0.26f) //!< 0.26us

//! Minimum bus free time between a STOP and START condition: t_BUF
#define I2C_TBUF_STANDARD_MODE_MIN  (4.7f) //!< 4.7 us
#define I2C_TBUF_FAST_MODE_MIN      (1.3f) //!< 1.3 us
#define I2C_TBUF_FAST_PLUS_MODE_MIN (0.5f) //!< 0.5 us

//! Minimum low period of the SCL Clock
#define I2C_TLOW_STANDARD_MODE_MIN  (4.7f) //!< 4.7 us
#define I2C_TLOW_FAST_MODE_MIN      (1.3f) //!< 1.3 us
#define I2C_TLOW_FAST_PLUS_MODE_MIN (0.5f) //!< 0.5 us

//! Minimum period of the SCL Clock
#define I2C_THIGH_STANDARD_MODE_MIN  (4.0f)  //!< 4 us
#define I2C_THIGH_FAST_MODE_MIN      (0.6f)  //!< 0.6 us
#define I2C_THIGH_FAST_PLUS_MODE_MIN (0.26f) //!< 0.26 us

//! Maximum rise time of both SDA and SCL signals
#define I2C_TR_STANDARD_MODE_MAX  (1.0f)  //!< 1 us
#define I2C_TR_FAST_MODE_MAX      (0.3f)  //!< 0.3 us
#define I2C_TR_FAST_PLUS_MODE_MAX (0.12f) //!< 0.12 us

//! Maximum fall time of both SDA and SCL signals
#define I2C_TF_STANDARD_MODE_MAX  (0.3f)  //!< 0.3 us
#define I2C_TF_FAST_MODE_MAX      (0.3f)  //!< 0.3 us
#define I2C_TF_FAST_PLUS_MODE_MAX (0.12f) //!< 0.12 us

/***************************************************************************************************
 *                      ENUMERATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      UNIONS
 **************************************************************************************************/
//! Clock source. Make sure chosen clock is configured in RCC
typedef enum DRV_I2C_clkSrc_ENUM {
    I2C_clkSrc_APB1   = 0u,
    I2C_clkSrc_HSI16  = 1u,
    I2C_clkSrc_SYSCLK = 2u,
    I2C_clkSrc_COUNT  = 3u
} DRV_I2C_clkSrc_E;

//! Mode selection
typedef enum DRV_I2C_mode_ENUM {
    I2C_mode_STANDARD  = 0u,
    I2C_mode_FAST      = 1u,
    I2C_mode_FAST_PLUS = 2u,
    I2C_mode_COUNT     = 3u
} DRV_I2C_mode_E;

//! I2C interface
typedef enum DRV_I2C_interface_ENUM {
    I2C_interface_1     = 0u,
    I2C_interface_2     = 1u,
    I2C_interface_3     = 2u,
    I2C_interface_COUNT = 3u
} DRV_I2C_interface_E;

/***************************************************************************************************
 *                      DATA TYPES
 **************************************************************************************************/

/***************************************************************************************************
 *                      DATA STRUCTURES
 **************************************************************************************************/
//! I2C configuration structure
typedef struct DRV_I2C_config_STRUCT {
    uint8_t interface; //!< I2C interface ::DRV_I2C_interface_E
    uint8_t mode; //!< I2C mode ::DRV_I2C_mode_E
    uint16_t clkSrc; //!< I2C clock source ::DRV_I2C_clkSrc_E
} DRV_I2C_config_S;

/***************************************************************************************************
 *                      GLOBAL VARIABLES DECLARATIONS
 **************************************************************************************************/

/***************************************************************************************************
 *                      PUBLIC FUNCTION PROTOTYPES
 **************************************************************************************************/
/***************************************************************************************************
 * @brief   Function is used to initialize I2C peripheral.
 * @details It'll:
 *              - Enable APB clock for input I2C interface
 *              - Calculate timings
 * *************************************************************************************************
 * @param   [in]      *inConfig - Pointer to I2C configuration structure
 * @param   [out]     *outErr   - Pointer to DRV error enumerator
 * *************************************************************************************************
 * @exceptions        .
 * *************************************************************************************************
 * @return  Nothing
 **************************************************************************************************/
void DRV_I2C_init(DRV_I2C_config_S *inConfig, DRV_ERROR_err_E *outErr);
void DRV_I2C_deInit(DRV_I2C_config_S *inConfig, DRV_ERROR_err_E *outErr);
void DRV_I2C_softwareReset(uint8_t inInterface, DRV_ERROR_err_E *outErr);

#endif /* DRV_I2C_H_ */

/***************************************************************************************************
 *                      END OF FILE
 **************************************************************************************************/
