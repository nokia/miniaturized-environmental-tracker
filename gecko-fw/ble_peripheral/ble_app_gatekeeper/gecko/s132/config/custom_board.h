/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"
#define NRF_NUM_GPIO_PINS        32

#define OSC_XL1                  0
#define OSC_XL2                  1
#define CS_EXT                   2
#define ANA_DIG1                 3
#define ANA_DIG2                 4
#define ANA_DIG3                 5
#define SWITCH_CTRL              6
#define TWI_SDA                  7
#define TWI_SCL                  8
#define NFC1                     9
#define NFC2                     10
#define SPI_SCK                  11
#define IMU_INT1                 12
#define IMU_INT2                 13
#define SPI_MOSI                 14
#define SPI_MISO                 15
#define DIG3                     16
#define DIG2                     17
#define PRES_INT                 18
#define DIG4                     19
#define CS_MEM                   20
#define RESET                    21
#define COLOUR_INT               22
#define CS_SD                    23
#define MIC_PWR_CTRL             24
#define MIC_DOUT                 25
#define MIC_CLK                  26
#define BAT_MON_EN               27
#define BATTERY                  28
#define LED_G                    29
#define LED_B                    30
#define LED_R                    31

// LEDs definitions for CUSTOM_BOARD
#define LEDS_NUMBER    3

#define LED_START      29
#define LED_G          29
#define LED_B          30
#define LED_R          31
#define LED_STOP       31


#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

// BLE BSP uses first LED in this list to 
// indicate BLE status. Nice if this was Blue.
#define LEDS_LIST { LED_B, LED_G, LED_R }

#define LED_BLUE       0
#define LED_GREEN      1
#define LED_RED        2

#define BSP_LED_0      LED_BLUE
#define BSP_LED_1      LED_GREEN
#define BSP_LED_2      LED_RED

#define BUTTONS_NUMBER 0
#define BUTTONS_ACTIVE_STATE 1

#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           true

// SDC SPI Aliases
#  define SDC_SCK_PIN     SPI_SCK   ///< SDC serial clock (SCK) pin.
#  define SDC_MOSI_PIN    SPI_MOSI  ///< SDC serial data in (DI) pin.
#  define SDC_MISO_PIN    SPI_MISO  ///< SDC serial data out (DO) pin.
#  define SDC_CS_PIN      CS_SD     ///< SDC chip select (CS) pin.


#define BATT_VOLTAGE_DIVIDER_R1      1500000
#define BATT_VOLTAGE_DIVIDER_R2      180000

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}


// Board Components Selection
#define BOARD_HAS_BATMON     1
#define BOARD_HAS_LPS22HB    1
#define BOARD_HAS_BH1745NUC  1
#define BOARD_HAS_BME680     1
#define BOARD_HAS_BMI160     1
#define BOARD_HAS_PDM_MIC    1

#define BOARD_NAME "Gecko"

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_BOARD_H
