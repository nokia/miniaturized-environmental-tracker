/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
/** @file
 * @defgroup datalogger main.c
 * @{
 * @ingroup datalogger
 * @brief EMCD Datalogger main file.
 *
 * This file contains the source code for the EMCD Datalogger using FAT filesystem and SD card library.
 *
 */
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrfx_pdm.h"

//#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "I2C.h"            //everything on the I2C bus
#include "BMI160_if.h"      // Accelerometer
#include "BME680_if.h"      // Environmental (Pressure, Temperature, Humidity, Gas (VOC)
#include "BH1745NUC_if.h"   // Colour Sensor
#include "LPS22HB_if.h"     // LPS22HB Pressure / Temp
#include "ADC.h"            // ADC for Battery Level Measurement

// Anytime the format of the data is changed in a way that requires a change to the data 
// processing script that is NOT backward compatible with older data (i.e. not just adding 
// a new sensor) then increment this version code 

#define CONFIG_DATA_VERSION 1
#define REFORMAT_ON_START true 

#define REFORMAT_USES_DATALOG_MEM

#define MEMORY_SIZE_KB(n) (n) * 1024

#define DATALOG_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(21)
#define AUDIO_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(1)

// Swap buffers when 90% full
#define DATALOG_MEMORY_LIMIT DATALOG_MEMORY_SIZE_BYTES * 10 / 9 

// Replace NRFX_RTC_US_TO_TICKS. 
// If us > 0x1fff then the intermediate expression overflows and truncates giving smaller 
// delays than intended.  
//#define NBL_NRFX_RTC_US_TO_TICKS(us,freq) (us > 0x1fff) ? (us / 1000000) * freq : (((us) * (freq)) / 1000000U)

#define NBL_NRFX_RTC_US_TO_TICKS(us, freq) (int32_t) ((us / 1000000.0) * freq)


#define FILENAME_BASENAME "dlog"
#define FILENAME_SUFFIX "bin"
#define MAX_FILESIZE_4GB 0xffffffff /* Corresponds to 4GB-1 */
#define MAX_FILESIZE_2GB 0x80000000 /* Corresponds to 2GB   */
#define MAX_FILESIZE MAX_FILESIZE_2GB

#define SDC_SCK_PIN     SPI_SCK   ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    SPI_MOSI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    SPI_MISO  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      CS_SD     ///< SDC chip select (CS) pin.

//Time (in miliseconds) between LED Flashes
#define PULSE_LED_PERIOD_MS 10000    
//Duration (in miliseconds) of LED Flashes
#define PULSE_LED_ON_TIME_MS 50 

#define PULSE_LED_OFF_TIME_MS (PULSE_LED_PERIOD_MS - PULSE_LED_ON_TIME_MS)

#define IMU_SENSOR_SAMPLE_PERIOD_MS 50
#define ENV_SENSOR_SAMPLE_PERIOD_MS 1000
#define COLOUR_SENSOR_SAMPLE_PERIOD_MS 500
#if CAPTURE_AUDIO == 1
#define AUDIO_SAMPLE_PERIOD_MS 5000
#define AUDIO_SAMPLE_SIZE_MS 500
#endif // CAPTURE_AUDIO

/**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define COMPARE_COUNTERTIME  (30UL) 
#define RTC_CONFIG_PRESCALER 0

#ifndef BOARD_CUSTOM
// Define any leds as we don't know what colour they are.
// These are already defined for BOARD_CUSTOM.
#define LED_GREEN      0
#define LED_BLUE       1
#define LED_RED        2
#endif

///
/// EMCD Cambridge Data Logger
///
/// Data logger(s) run on timers, updating a RAM array with the aquired data.
/// 
/// SD Card Writes also run on a timer, and/or when the RAM array is getting full.
/// 
/// Each logger uses has different data lengths for entries used, so logger/reader 
/// has to be able to cope with that.
///
/// Each entry has MagicNumber, timestamp, logger_id, entry_size, log_data[entry_size].
/// Looks like size of packet_info_t is expanded to be word aligned so could put 
/// more information in here.
/// 
/// Memory is organised as uint8_t for easier file writing (and Radio Communication)
/// so logger is responsible for this conversion. 
/// 
/// NOTE: 
///    Not using malloc or nrf_malloc here as the extra code required to support this 
///    takes up so much RAM - better to use that memory by declaring an array here. 

typedef uint8_t datalog_memory_t ;
typedef uint16_t audio_memory_t ;

#define SOP_BELLLABS 0x42656c6c

typedef struct {
    uint32_t  sop;
    uint32_t  timestamp;  // Gives 36 hours at 32.768KHz. Need 64 bit for more.
    uint8_t   logger_id;
    uint8_t   length;
    uint16_t  custom;     // Makes the structure end 32-bit aligned.
} packet_info_t;


static datalog_memory_t datalog_memory_0 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
static datalog_memory_t datalog_memory_1 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)];
#if CAPTURE_AUDIO == 1  
static audio_memory_t   audiodata_memory [AUDIO_MEMORY_SIZE_BYTES/sizeof(audio_memory_t)];
#endif // CAPTURE_AUDIO

static datalog_memory_t *m_p_datalog_mem;
static uint16_t m_write_offset;

static FRESULT sdcard_write(datalog_memory_t *, datalog_memory_t *);
static int Open_SDCard(void);
static int List_SDCard(void);
void Swap_and_Save_Buffer(void);
//static FRESULT sdcard_write(datalog_memory_t *read_ptr, datalog_memory_t *read_ptr_done);

// 24 bits from RTC + 8 bits here makes 32 bits which @32768Hz gives 36 hours
static uint8_t m_timer_high;

/*
 * @brief LED Control
 */

#define led_state_off 0
#define led_state_on  1

static uint8_t  m_led_state = 0;
#define LED_OVERRIDE_BIT 7U
#define LED_OVERRIDE_MASK 1U << LED_OVERRIDE_BIT

void board_led_on(uint8_t led) {
  m_led_state |= led;
  if (0 == (m_led_state & LED_OVERRIDE_MASK))
    bsp_board_led_on(led);
}

void board_led_off(uint8_t led) {
  m_led_state &= ~led;
  bsp_board_led_off(led);
}

void board_led_override(void) {
  m_led_state |= LED_OVERRIDE_MASK;
  bsp_board_led_off(LED_RED);
  bsp_board_led_off(LED_GREEN);
  bsp_board_led_off(LED_BLUE);
}

void board_led_restore(void) {
  m_led_state &= ~LED_OVERRIDE_MASK;
  if (m_led_state & LED_RED) bsp_board_led_on(LED_RED);
  if (m_led_state & LED_GREEN) bsp_board_led_on(LED_GREEN);
  if (m_led_state & LED_BLUE) bsp_board_led_on(LED_BLUE);
}
 

void error_led_flash_loop(void) {

  bsp_board_leds_off();
  
  while (true) {
    bsp_board_led_invert(LED_RED);
    nrf_delay_ms(200);
  }
}


/// ===============================================
///
/// RTC, Timers and Handlers
///
/// ===============================================

/// LFCLK
///
/** 
 * @brief Dummy handler for LF clock init.
 *        SDK15.2+ requires handler for init (can no longer supply NULL)
 */
static void lfclk_handler(nrfx_clock_evt_type_t event_type)
{
    switch (event_type) {
    case NRFX_CLOCK_EVT_HFCLK_STARTED : SEGGER_RTT_printf(0,"HFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_LFCLK_STARTED : SEGGER_RTT_printf(0,"LFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_CTTO          : SEGGER_RTT_printf(0,"Calibration timeout.\n"); break;    
    case NRFX_CLOCK_EVT_CAL_DONE      : SEGGER_RTT_printf(0,"Calibration has been done.\n"); break;     
    default                           : SEGGER_RTT_printf(0,"???? (!!! Unknown Event !!!)\n"); 
    }  
    
    return;
}

/** 
 * @brief Function starting the internal LFCLK XTAL oscillator.
 */

static void lfclk_config(void)
{
    ret_code_t err_code = nrfx_clock_init(lfclk_handler);
    APP_ERROR_CHECK(err_code);
    if (false == nrfx_clock_lfclk_is_running())
      nrfx_clock_lfclk_start();
}

 /**
 * @brief RTC Timer Handle
 */
const nrfx_rtc_t rtc1               = NRF_DRV_RTC_INSTANCE(1); /**< Declaring an instance of nrfx_rtc for RTC1. */

#if LED_HEARTBEAT_ENABLED == 1
const nrfx_rtc_t rtc0               = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrfx_rtc for RTC1. */
#endif

void imu_datalogger_handler (void);
void env_datalogger_handler (void);
void lightsensor_datalogger_handler (void);
#if CAPTURE_AUDIO==1
void audio_datalogger_handler (nrfx_pdm_evt_t const * const p_evt);
void audio_datalogger_rtc_init_handler(uint8_t);
#endif // CAPTURE_AUDIO
void batt_level_handler (void);

#if LED_HEARTBEAT_ENABLED == 1
/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on COMPARE0 match.
 */
static void rtc0_handler(nrfx_rtc_int_type_t int_type)
{

    uint32_t        rtc_ticks;
    ret_code_t      err_code;
    uint32_t        next_interval_us = 0;
    
    switch (int_type) {
    case NRFX_RTC_INT_COMPARE0 : 
                                  // This is not a sensor, just a "heartbeat" so we don't mind
                                  // if the interval is not exact. It's OK therefore to do some
                                  // work before reading the current counter value.
                                  switch (m_led_state & LED_BLUE) {
                                    case (led_state_off)  : next_interval_us = 1000 * PULSE_LED_ON_TIME_MS;
                                                            board_led_on(LED_BLUE);
                                                            break;
                                    case (led_state_on)   : next_interval_us = 1000 * PULSE_LED_OFF_TIME_MS;
                                                            board_led_off(LED_BLUE);
                                                            break;
                                  }

                                  rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(next_interval_us, RTC_INPUT_FREQ);
                                  err_code    = nrfx_rtc_cc_set(&rtc0,0,(NRF_RTC0->COUNTER + rtc_ticks),true);
                                  APP_ERROR_CHECK(err_code);
                                  break;
    case NRFX_RTC_INT_COMPARE1 : 
    case NRFX_RTC_INT_COMPARE2 : 
    case NRFX_RTC_INT_COMPARE3 : 
    case NRFX_RTC_INT_OVERFLOW : 
    case NRFX_RTC_INT_TICK     :  
    default                    : SEGGER_RTT_printf(0,"!!! In RTC Handler due to Unknown or Unexpected Interrupt (%d) !!!)\n",
                                                    int_type
                                                    ); 
    }

    return;
}
#endif // LED_HEARTBEAT_ENABLED

/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on COMPAREn and OVERFLOW .
 */
static void rtc1_handler(nrfx_rtc_int_type_t int_type)
{

    uint32_t        rtc_ticks;
    ret_code_t      err_code;
#if CAPTURE_AUDIO == 1                                      
    static uint8_t  audio_state = 0;
    uint32_t        next_interval_us;
#endif // CAPTURE_AUDIO
  
    switch (int_type) {

    // Audio Handler
    // (a) Audio - Channel 0
    case NRFX_RTC_INT_COMPARE0 : 
#if CAPTURE_AUDIO == 1                                      
                                  switch (audio_state) {
                                    case (0)  :   next_interval_us = 1000 * (AUDIO_SAMPLE_SIZE_MS);
                                                  audio_state = 1;
                                                  break;
                                    case (1)   :  next_interval_us = 1000 * (AUDIO_SAMPLE_PERIOD_MS - AUDIO_SAMPLE_SIZE_MS);
                                                  audio_state = 0;
                                                  break;
                                  }
                                  rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(next_interval_us, RTC_INPUT_FREQ);
                                  err_code    = nrfx_rtc_cc_set(&rtc1,0,(NRF_RTC1->COUNTER + rtc_ticks),true);
                                  APP_ERROR_CHECK(err_code);  
                                  audio_datalogger_rtc_init_handler(audio_state);
#endif // CAPTURE_AUDIO
                                  break;

    // (b) IMU - Channel 1
    case NRFX_RTC_INT_COMPARE1 : 
                                  rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(IMU_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
                                  err_code    = nrfx_rtc_cc_set(&rtc1,1,(NRF_RTC1->COUNTER + rtc_ticks),true);
                                  APP_ERROR_CHECK(err_code);  
                                  imu_datalogger_handler();
                                  break;

    // (c) ENV - Channel 2
    case NRFX_RTC_INT_COMPARE2 : 
                                  rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(ENV_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
                                  err_code    = nrfx_rtc_cc_set(&rtc1,2,(NRF_RTC1->COUNTER + rtc_ticks),true);
                                  APP_ERROR_CHECK(err_code);                                   
                                  env_datalogger_handler();
                                  batt_level_handler();
                                  break;  


    // (c) Colour Sensor - Channel 3
    case NRFX_RTC_INT_COMPARE3 : 
                                  rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(COLOUR_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
                                  err_code    = nrfx_rtc_cc_set(&rtc1,3,(NRF_RTC1->COUNTER + rtc_ticks),true);
                                  APP_ERROR_CHECK(err_code);                                   
                                  lightsensor_datalogger_handler();
                                  break;
    case NRFX_RTC_INT_OVERFLOW : 
                                  m_timer_high++;
                                  break;  
    case NRFX_RTC_INT_TICK     :  
    default                    : SEGGER_RTT_printf(0,"!!! In RTC Handler due to Unknown or Unexpected Interrupt (%d) !!!)\n",
                                                    int_type
                                                    ); 
    }

//    SEGGER_RTT_printf(0, "RTC1  (exit):%d\n", NRF_RTC1->COUNTER);
    return;
}



/** @brief Function initialization and configuration of RTC driver instance.
 */

static void rtc_config(void)
{
    uint32_t err_code;
    uint32_t rtc_ticks;

    //Initialize RTC instance
    nrfx_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC_CONFIG_PRESCALER;
    err_code = nrfx_rtc_init(&rtc1, &config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    //Disable tick event & interrupt
    nrfx_rtc_tick_disable(&rtc1);

    // Set compare channels to trigger relevant interrupts
    // ===================================================

    uint32_t rtc_val = nrfx_rtc_counter_get(&rtc1);
    ASSERT(rtc_val==0); // Should be 0 as we have not previously started the RTC

#if LED_HEARTBEAT_ENABLED == 1

    // RTC 0
    // =====
    // Only used for LED Heartbeat so can be left powered off if this feature is not used.
    // Configure first triggering of LED Handler.
    
    err_code = nrfx_rtc_init(&rtc0, &config, rtc0_handler);
    APP_ERROR_CHECK(err_code);

    // (a) LED - Channel 0
    rtc_ticks   = NBL_NRFX_RTC_US_TO_TICKS(1000U * (PULSE_LED_PERIOD_MS - PULSE_LED_ON_TIME_MS), RTC_INPUT_FREQ);
    err_code = nrfx_rtc_cc_set(&rtc0,0,(rtc_val + rtc_ticks),true);
    APP_ERROR_CHECK(err_code);
#endif

    // RTC 1
    // =====

#if CAPTURE_AUDIO == 1  
    // Configure first triggering of Audio Handler.
    // (a) Audio - Channel 0
    rtc_ticks  = NBL_NRFX_RTC_US_TO_TICKS((AUDIO_SAMPLE_PERIOD_MS - AUDIO_SAMPLE_SIZE_MS) * 1000U, RTC_INPUT_FREQ);
    err_code = nrfx_rtc_cc_set(&rtc1,0,(rtc_val + rtc_ticks),true);
    APP_ERROR_CHECK(err_code);
#endif // CAPTURE_AUDIO

    // Configure first triggering of IMU Handler.
    // (b) IMU - Channel 1
    rtc_ticks  = NBL_NRFX_RTC_US_TO_TICKS(IMU_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
    err_code = nrfx_rtc_cc_set(&rtc1,1,(rtc_val + rtc_ticks),true);
    APP_ERROR_CHECK(err_code);

    // Configure first triggering of ENV Handler.
    // (c) ENV - Channel 2
    rtc_ticks  = NBL_NRFX_RTC_US_TO_TICKS(ENV_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
    err_code = nrfx_rtc_cc_set(&rtc1,2,(rtc_val + rtc_ticks),true);
    APP_ERROR_CHECK(err_code);

    // Configure first triggering of Colour Sensor Handler.
    // (c) Colour Sensor - Channel 3
    rtc_ticks  = NBL_NRFX_RTC_US_TO_TICKS(COLOUR_SENSOR_SAMPLE_PERIOD_MS * 1000U, RTC_INPUT_FREQ);
    err_code = nrfx_rtc_cc_set(&rtc1,3,(rtc_val + rtc_ticks),true);
    APP_ERROR_CHECK(err_code);


    // Enable Overflow
    nrfx_rtc_overflow_enable(&rtc1, true);

    //Power on RTC instances
#if LED_HEARTBEAT_ENABLED == 1
    nrfx_rtc_enable(&rtc0);
#endif
    nrfx_rtc_enable(&rtc1);
}

uint32_t get_timestamp(void) {
  // TODO: As it stands this wraps every 8.5 minutes. Can we use OVERFLOW event to increment the 
  //       TIMER/COUNTER? If not, put an interrupt on the event to increment a global.
  return NRF_RTC1->COUNTER | m_timer_high << 24;
}

/**
 * Env Sensor Timeout Handler.
 */ 

// Data is 4 32-bit words: - 
// [0] Pressure
// [1] Temperature
// [2] Humidity
// [3] Gas Resistance
// TODO: We don't check if new value ready for Gas Resistance.
#define LOGGER_ENV_DATA_LENGTH 4*4
#define LOGGER_ENV_ID  0x02

typedef struct {
    packet_info_t   packet_info;
    uint8_t         data[LOGGER_ENV_DATA_LENGTH];
} env_data_t;

void env_datalogger_handler (void) {
  env_data_t                env_data;
  static uint16_t           packet_count = 0;
  uint32_t                  timestamp = get_timestamp();
  uint16_t                  size = sizeof(env_data_t);

  SEGGER_RTT_printf(0, "ENV Handler");
  SEGGER_RTT_printf(0, "Timestamp %d\n", timestamp);
  SEGGER_RTT_printf(0, "Running for approx %d seconds\n", 
    timestamp * (RTC_CONFIG_PRESCALER+1)/RTC_INPUT_FREQ
    );


  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
  }

  env_data.packet_info.sop        = SOP_BELLLABS;
  env_data.packet_info.timestamp  = timestamp;
  env_data.packet_info.logger_id  = LOGGER_ENV_ID;
  env_data.packet_info.length     = LOGGER_ENV_DATA_LENGTH;
  env_data.packet_info.custom     = packet_count++;

#if USE_DUMMY_SENSORS == 1
  for (int i=0; i<LOGGER_ENV_DATA_LENGTH; i++) {
      env_data.data[i] = i;
  }
#else

  // BME680_Get_Data returns 4 32-bit words: - 
  //  data32[0] = BME680 pressure;
  //  data32[1] = BME680 temperature;
  //  data32[2] = BME680 humidity;
  //  data32[3] = BME680 gas_resistance;

  // Data is array of bytes but in BME680 is 32-bit words.
  // Change the pointer type so it is transferred correctly.
  BME680_Get_Data( (int32_t*) env_data.data );
  
  // LPS22HB_Get_Data returns 2 16-bit words: - 
  //  data16[0] = LPS22HB pressure;
  //  data16[1] = LPS22HB temperature;
  LPS22HB_Get_Data( (int16_t*) ((int32_t*) env_data.data + 4));

#endif

  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
//  SEGGER_RTT_printf(0, "Memory Copy destination 0x%08x to 0x%08x (env)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size);
  memcpy((m_p_datalog_mem+m_write_offset), &env_data,  size);
  m_write_offset += size;
}


/**
 * IMU Sensor Timeout Handler
 */ 

// Get data direct from sensor rather than use driver as driver converts into a structure that 
// wastes storage, and then needs to be converted into bytes for the SDCard writes. Better to 
// keep it in bytes as straight out of BMI160
// Format is gx0,gx1,gy0,gy1,gz0,gz1,ax0,ax1,ay0,ay1,az0,az1,s0,s1,s2 which is 15 bytes but 
// this then gets aligned to 32 bit boundary -> 16 bytes. 

#define LOGGER_IMU_DATA_LENGTH_BYTES 16
#define LOGGER_IMU_DATA_LENGTH_HALF_WORDS LOGGER_IMU_DATA_LENGTH_BYTES/2
#define LOGGER_IMU_ID  0x01

typedef struct {
    packet_info_t   packet_info;
    int16_t         data[LOGGER_IMU_DATA_LENGTH_HALF_WORDS];
} imu_data_t;

void imu_datalogger_handler (void) {
  imu_data_t                imu_data;
  static uint16_t           packet_count = 0;
  uint32_t                  timestamp = get_timestamp();
  uint16_t                  size = sizeof(imu_data_t);

  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      
      SEGGER_RTT_printf(0, "Timestamp %d\n", timestamp);
      SEGGER_RTT_printf(0, "Running for approx %d seconds\n", 
        timestamp * (RTC_CONFIG_PRESCALER+1)/RTC_INPUT_FREQ
        );
  }

  imu_data.packet_info.sop        = SOP_BELLLABS;
  imu_data.packet_info.timestamp  = timestamp;
  imu_data.packet_info.logger_id  = LOGGER_IMU_ID;
  imu_data.packet_info.length     = LOGGER_IMU_DATA_LENGTH_BYTES;
  imu_data.packet_info.custom     = packet_count++;

  // Data is array of bytes but in BMI160 is 16-bit half-words.
  // Change the pointer type so it is transferred correctly.
  BMI160_Get_Data( (int16_t*) imu_data.data );

  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  ASSERT((uint32_t) m_p_datalog_mem+m_write_offset > 0x2000000);
//  SEGGER_RTT_printf(0, "Memory Copy destination 0x%08x to 0x%08x (imu)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size);
//  SEGGER_RTT_printf(0, "m_p_datalog_mem = 0x%08x\nm_write_offset= 0x%08x\nsize=\n", m_p_datalog_mem, m_write_offset, size);
  memcpy((m_p_datalog_mem+m_write_offset), &imu_data,  size);
  m_write_offset += size;
}

/**
 * Light Sensor Timeout Handler
 */ 

#define LOGGER_LIGHTSENSOR_DATA_LENGTH_BYTES 8
#define LOGGER_LIGHTSENSOR_ID  0x03

typedef struct {
    packet_info_t   packet_info;
    uint8_t         data[LOGGER_LIGHTSENSOR_DATA_LENGTH_BYTES];
} lightsensor_data_t;

void lightsensor_datalogger_handler (void) {
  lightsensor_data_t        lightsensor_data;
  static uint16_t           packet_count = 0;
  uint32_t                  timestamp = get_timestamp();
  uint16_t                  size = sizeof(lightsensor_data_t);

  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      SEGGER_RTT_printf(0, "Timestamp %d\n", timestamp);
      SEGGER_RTT_printf(0, "Running for approx %d seconds\n", 
        timestamp * (RTC_CONFIG_PRESCALER+1)/RTC_INPUT_FREQ
        );
  }

  lightsensor_data.packet_info.sop        = SOP_BELLLABS;
  lightsensor_data.packet_info.timestamp  = timestamp;
  lightsensor_data.packet_info.logger_id  = LOGGER_LIGHTSENSOR_ID;
  lightsensor_data.packet_info.length     = LOGGER_LIGHTSENSOR_DATA_LENGTH_BYTES;
  lightsensor_data.packet_info.custom     = packet_count++;

  // Issue BitBucket #1
  // Ensure LEDs are OFF for measurement. 
  board_led_override();

  // Data is array of bytes driver returns 16-bit half-words.
  // Change the pointer type so it is transferred correctly.
  BH1745NUC_Get_Data(lightsensor_data.data );

  // Issue BitBucket #1
  // Restore LED state. 
  board_led_restore();


  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
//  SEGGER_RTT_printf(0, "Memory Copy destination 0x%08x to 0x%08x (lightsensor)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size);
  memcpy((m_p_datalog_mem+m_write_offset), &lightsensor_data,  size);
  m_write_offset += size;
}


#if CAPTURE_AUDIO == 1  

/**
 * Audio Timeout Handler
 */ 

// There are two Audio handlers. The first is triggered by the RTC timer and responsible 
// for starting and stopping the PDM module. 
// Starting the PDM will result in PDM Interrupts being generated to do any combination of
// the following: - 
//   - Request a new buffer for EasyDMA
//   - Release a buffer from EasyDMA
//   - Communicate an ERROR condition. 
//
// According to NRF5-SDK (v15.2) the correct sequence is: - 
//    - Initialise the PDM (nrfx_pdm_init(...)) - this is performed in pdm_init called from main.
//    - Start the PDM (nrfx_pdm_start()). This generates an interrupt to provide the location of 
//      the buffer for EasyDMA
//    - Service the Interrupt providing the first buffer
//    - Service the next Interrupt providing a new buffer
//    - Continue servicing interrupts providing new bufferes, releasing old buffers, checking errors. 
//    - Stop the PDM (nrfx_pdm_stop())

// Here the RTC triggered interrupt handler handles the PDM Start and Stop.

void audio_datalogger_rtc_init_handler(uint8_t turn_on) {
  ret_code_t ret_code;

  if (turn_on) {

    // Start the PDM
    SEGGER_RTT_printf(0,"Starting PDM at RTC1 time %d\n", (uint32_t) NRF_RTC1->COUNTER); 
    ret_code = nrfx_pdm_start();
 
    if (ret_code != 0) {
      __BKPT();
      SEGGER_RTT_printf(0,"PDM Already Started");
    }

  } else {
    SEGGER_RTT_printf(0,"Stopping PDM at RTC1 time %d\n", (uint32_t) NRF_RTC1->COUNTER); 
    ret_code = nrfx_pdm_stop();

    if (ret_code != 0) {
      __BKPT();
      SEGGER_RTT_printf(0,"PDM Already Stopping");
    }
  }


}

/**
 * Audio PDM Event Handler
 */ 
// This Interrupt handles the Audio PDM Events, i.e. 
//    - Providing new buffers for EasyDMA
//    - Processing the saved Audio Data and storing results.
//    - Check for Errors.

#define LOGGER_AUDIO_DATA_LENGTH 2
#define LOGGER_AUDIO_ID  0x04

typedef struct {
    packet_info_t       packet_info;
    audio_memory_t      data[LOGGER_AUDIO_DATA_LENGTH];
} audio_data_t;

void audio_datalogger_handler (nrfx_pdm_evt_t const * const p_evt) {
  ret_code_t        ret_code;
  audio_data_t      audio_data;
  static uint16_t   packet_count = 0;
  uint16_t          size = sizeof(audio_data_t);

  SEGGER_RTT_printf(0,"PDM Event Handler at RTC1 time %d\n", (uint32_t) NRF_RTC1->COUNTER);
  SEGGER_RTT_printf(0,"p_evt->buffer_requested = %d\n", p_evt->buffer_requested);
  SEGGER_RTT_printf(0,"p_evt->buffer_released = 0x%08x\n", p_evt->buffer_released);
  SEGGER_RTT_printf(0,"p_evt->error = %d\n", p_evt->error);  
  SEGGER_RTT_printf(0,"nrfx_pdm_enable_check returns %d\n", (int) nrfx_pdm_enable_check());

  if (p_evt->buffer_requested) {
    if (p_evt->buffer_released == NULL) {

      // This is the Initial Setup
      // Initialise Structure with the constant data.

      // Note that the PDM driver will call this function twice with NULL p_evt->buffer_released.
      // The first time is to get the PDM started and the second time is to obtain the location of the
      // NEXT buffer for data storage. So if continous capture is required, then at least two buffers 
      // are needed with the PDM ping-ponging between them. For this to work, the data needs to be able 
      // to be written to SDCard in the time that it takes to fill another buffer.
      // 
      //
      // For the DataLogger however, only a sample of the sound is taken at intervals, and the average 
      // and peak values calculated and stored in the data stream. Raw audio data is not stored and so 
      // a single buffer can be used. For this case, just provide one buffer and sampling will stop 
      // after that.
      // TODO: NOT SURE ABOUT PDM STOPPING !!!

      // The first call after nrfx_pdm_start occurs before the PDM is enabled.
      // So, only return the pointed when nrfx_pdm_enable_check returns 0;
      if (0 == nrfx_pdm_enable_check()) {
        // Point the PDM EasyDMA to the first buffer
        ret_code = nrfx_pdm_buffer_set(audiodata_memory,AUDIO_MEMORY_SIZE_BYTES/sizeof(uint16_t));
        if (ret_code != NRF_SUCCESS) {
          __BKPT();
        }
        APP_ERROR_CHECK(ret_code);
        SEGGER_RTT_printf(0, "PDM Event Handler Providing Buffer.\n");
        SEGGER_RTT_printf(0,"EasyDMA pointing to 0x%08x to 0x%08x\n", 
                            audiodata_memory, audiodata_memory+AUDIO_MEMORY_SIZE_BYTES);
      } else {
      SEGGER_RTT_printf(0, "PDM Event Handler Requesting second Buffer - ignored.\n");      }
    } else {
      SEGGER_RTT_printf(0, "PDM Event Handler - Unexpected Buffer Requesting with Release.\n");
      __BKPT();
    }
  } 
    
  if (p_evt->buffer_released != NULL) {
//    FIXME: Put back in after debug
//    ASSERT((audio_memory_t *) p_evt->buffer_released == (audio_memory_t *) audiodata_memory)

    audio_data.packet_info.sop        = SOP_BELLLABS;
    audio_data.packet_info.logger_id  = LOGGER_AUDIO_ID;
    audio_data.packet_info.length     = LOGGER_AUDIO_DATA_LENGTH*sizeof(audio_memory_t);
    audio_data.packet_info.custom     = packet_count++;
    audio_data.packet_info.timestamp  = get_timestamp();

    int64_t mean = 0;
    int16_t peak = 0;

    for (int i=0; i<AUDIO_MEMORY_SIZE_BYTES/sizeof(audio_memory_t); i++) {
      // Take the mean and peak of the absolute value (only interested in magnitude)
      // TODO: This assumes audio data always centred on 0?
      int16_t sample = abs(*(audiodata_memory+i));
      mean += sample;
      if (sample > peak) {
        peak = sample;
      }
    }

    mean = mean / (AUDIO_MEMORY_SIZE_BYTES/sizeof(audio_memory_t));

    audio_data.data[0] = mean;
    audio_data.data[1] = peak;

    SEGGER_RTT_printf(0, "PDM Event Handler audio mean:peak = 0x%04x:0x%04x\n", audio_data.data[0], audio_data.data[1]);

    if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
    }

    ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
    memcpy((m_p_datalog_mem+m_write_offset), &audio_data,  size);
    m_write_offset += size;

    // Clear the Audio Memory Buffer
    memset(audiodata_memory, 0, AUDIO_MEMORY_SIZE_BYTES);

    SEGGER_RTT_printf(0, "PDM Event Handler Data Released\n");

  }

  if (p_evt->error != NRF_SUCCESS) {
    SEGGER_RTT_printf(0,"PDM Event Handler\n");
    SEGGER_RTT_printf(0,"p_evt->buffer_requested = %d\n", p_evt->buffer_requested);
    SEGGER_RTT_printf(0,"p_evt->buffer_released = %d\n", p_evt->buffer_released);
    SEGGER_RTT_printf(0,"p_evt->error = %d\n", p_evt->error);
  }

}


/**
 * @brief Initialise the PDM for Audio Sampling.
 */
void pdm_init() {
  ret_code_t ret_code;
  nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(MIC_CLK, MIC_DOUT);

  // Clear the Audio Memory Buffer
  memset(audiodata_memory, 0, AUDIO_MEMORY_SIZE_BYTES);
    
  // Set the MIC Power Control pin to output
  nrf_gpio_cfg_output(MIC_PWR_CTRL);
  
  // Turn the mic on
  nrf_gpio_pin_set(MIC_PWR_CTRL);
  
  // Initialise the PDM
  ret_code = nrfx_pdm_init(&pdm_config, audio_datalogger_handler);
  if (ret_code != NRF_SUCCESS) {
    __BKPT();
  }
  nrf_pdm_int_enable(NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED | NRF_PDM_INT_END);
  APP_ERROR_CHECK(ret_code);
}

#endif // CAPTURE_AUDIO

/**
 * Battery Level Timeout Handler.
 */ 

#define LOGGER_BAT_LEVEL_DATA_LENGTH 2
#define LOGGER_BAT_LEVEL_ID  0x05

typedef struct {
    packet_info_t   packet_info;
    uint8_t         data[LOGGER_BAT_LEVEL_DATA_LENGTH];
} batt_lev_data_t;

//void batt_datalogger_handler (nrf_timer_event_t event_type, void* p_context) {
void batt_level_handler (void) {

  // Enable the Battery Measurement Circuitry
  nrf_gpio_pin_set(BAT_MON_EN);
//  nrf_delay_ms(10);

  //Battery ADC read
  nrfx_saadc_sample();

}

/* 
 * @brief Callback for when ADC Value us updated
 */
void batt_level_updated_callback (float vbatt) {
  batt_lev_data_t           batt_data;
  static uint16_t           packet_count = 0;
  uint32_t                  timestamp = get_timestamp();
  uint16_t                  size = sizeof(batt_lev_data_t);
  uint8_t                   battery_level_Percent = 0;
  static float              vbatt_history[BATTERY_AVERAGE_PERIOD];
  static float              filtered_vbatt;
  static                    uint8_t count = 0;



  // Figures taken from discharge curve measured overnight 27th November 2018
  // Entries correspond to           0%,  10%,  20%,  30%,  40%,  50%,  60%,  70%,  80%,  90%, 100%
  const float battery_lookup [] = {3.20, 3.80, 3.95, 3.99, 4.04, 4.10, 4.15, 4.20, 4.23, 4.24, 4.25};

  nrf_gpio_pin_clear(BAT_MON_EN);

#ifdef SW_FILTERING_SAADC

  // Note that voltage curve is noisy. See Nordic recommendation for battery 
  // measurement and how resistor values should be chosen. The resistors chosen 
  // for this Battery Monitor were copied from Thingy:52. 
  // To clean the trace up, average over a few samples.
  vbatt_history[count++%BATTERY_AVERAGE_PERIOD] = vbatt;

  // Don't report the first samples because the spread affects the results graph
  // too much. 
  if (count < BATTERY_AVERAGE_PERIOD) { return; };

  filtered_vbatt = 0;

  for (int i=0; i<BATTERY_AVERAGE_PERIOD; i++) {
    filtered_vbatt += vbatt_history[i];
  }

  filtered_vbatt /= BATTERY_AVERAGE_PERIOD;

#else 
  filtered_vbatt = vbatt;
#endif

  // Restrict voltage to within a reasonable range. The readings are 
  // not very accurate and so vary out-of-range a lot. 
  if (filtered_vbatt > 4.25) {
    filtered_vbatt = 4.25;
  } else if (filtered_vbatt < 3.0){
    filtered_vbatt = 3.0;
  }

  // Convert to percent from voltage - 4.25V is full
  // Use the lookup table  
  uint8_t decile = 1;
  while ((filtered_vbatt > battery_lookup[decile]) && (decile < 10))
    decile++;

  float vlo   = battery_lookup[decile-1];
  float vhi   = battery_lookup[decile];

  ASSERT((vhi-vlo) != 0);

  battery_level_Percent = (uint8_t) (10.0 * ((decile-1) + ((filtered_vbatt-vlo)/(vhi-vlo))));
 
  SEGGER_RTT_printf(0, "battery_percent: %d\n", battery_level_Percent);
  SEGGER_RTT_printf(0, "Battery Level (V): %d.%03d\n", (int) vbatt, (int) ((float) round(1000 * (vbatt - floor(vbatt)))));
  SEGGER_RTT_printf(0, "Filtered Level (V): %d.%03d\n", (int) filtered_vbatt, (int) ((float) round(1000 * (filtered_vbatt - floor(filtered_vbatt)))));

  
  // For debugging and discharge analysis record the actual voltage
  // To save this in uint8_t format: - 
  //      Battery Voltage should be in range c. 3V2 to 4V2. 
  //      Constrain 8-bit variable to voltage range 3v0 to 4v25 
  //        - Store in variable with range 0->255
  //        - resolution is 1/200V (0.005V) Offset is 3.0v
  float coded_vbatt = filtered_vbatt - 3.0;

  coded_vbatt *= 200;

  ASSERT(coded_vbatt < 256.0)

  uint8_t coded_vbatt_8 = (uint8_t) coded_vbatt;

  
  batt_data.data[0] = battery_level_Percent;
  batt_data.data[1] = coded_vbatt_8;

  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
  }

  batt_data.packet_info.sop        = SOP_BELLLABS;
  batt_data.packet_info.timestamp  = timestamp;
  batt_data.packet_info.logger_id  = LOGGER_BAT_LEVEL_ID;
  batt_data.packet_info.length     = LOGGER_BAT_LEVEL_DATA_LENGTH;
  batt_data.packet_info.custom     = packet_count++;

  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
  memcpy((m_p_datalog_mem+m_write_offset), &batt_data,  size);
  m_write_offset += size;
}


//
// Write some Configuration Information for backend processing.
//
//

#define LOGGER_CONFIG_DATA_LENGTH_BYTES 8
#define LOGGER_CONFIG_ID  0x00

typedef struct {
    packet_info_t   packet_info;
    int8_t          data[LOGGER_CONFIG_DATA_LENGTH_BYTES];
} config_data_t;

void config_datalogger_handler (void) {
  config_data_t             config_data;
  static uint16_t           packet_count = 0;
  uint32_t                  timestamp = get_timestamp();
  uint16_t                  size = sizeof(config_data_t);

  if (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES) {
      Swap_and_Save_Buffer();
      SEGGER_RTT_printf(0, "Timestamp %d\n", timestamp);
      SEGGER_RTT_printf(0, "Running for approx %d seconds\n", 
        timestamp * (RTC_CONFIG_PRESCALER+1)/RTC_INPUT_FREQ
        );
  }

  config_data.packet_info.sop        = SOP_BELLLABS;
  config_data.packet_info.timestamp  = timestamp;
  config_data.packet_info.logger_id  = LOGGER_CONFIG_ID;
  config_data.packet_info.length     = LOGGER_CONFIG_DATA_LENGTH_BYTES;
  config_data.packet_info.custom     = packet_count++;

  // Change the pointer type so it is transferred correctly.
  uint32_t *data32 = (uint32_t*) config_data.data;
  uint16_t *data16 = (uint16_t*) config_data.data;

  *data32 = NRF_RTC1->PRESCALER;
  *(data16+2) = RTC_INPUT_FREQ;
  *(data16+3) = CONFIG_DATA_VERSION;

  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES);
//  SEGGER_RTT_printf(0, "Memory Copy destination 0x%08x to 0x%08x (config)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size);
  memcpy((m_p_datalog_mem+m_write_offset), &config_data,  size);
  m_write_offset += size;
}

///
/// SDCard and FATFS
///
static FATFS m_fs;

/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);


/**
 * @brief Disk Access Wait Function.
 */
static void datalogger_wait_func(void)
{
//    nrf_delay_ms(1);
    __WFE();
}

/**
 * @brief Open the SD Card function.
 */

static int Open_SDCard(void)
{
//    static FIL file;
    static uint8_t reformat = REFORMAT_ON_START;

//    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), &datalogger_wait_func)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return NRF_ERROR_INTERNAL;
    }

    if (reformat) {
      // Re-use datalog_memory for the working buffer for formatting.
      FRESULT retval;
#ifdef REFORMAT_USES_DATALOG_MEM
      uint8_t *p_work = datalog_memory_0;
      uint16_t work_size = DATALOG_MEMORY_SIZE_BYTES;   
#else
      NRF_LOG_INFO("==============================================");
      NRF_LOG_INFO("!!! Reformat uses separate memory.   !!!");      
      NRF_LOG_INFO("    Disable when not debugging");
      NRF_LOG_INFO("    (%s:%d)", __FILE__, __LINE__);
      uint8_t reformat_work [2048];
      uint8_t *p_work = reformat_work;
      uint16_t work_size = sizeof(reformat_work);
#endif
      NRF_LOG_INFO("Re-formatting volume...");
      NRF_LOG_INFO("   Using work memory size of %d bytes...", work_size);
      retval=f_mkfs("",FM_ANY, 0, p_work, work_size);

      switch (retval) {
        case FR_MKFS_ABORTED : 
          NRF_LOG_INFO("Re-formatting Aborted!!!");
          NRF_LOG_INFO("                      - possibly work buffer too small (%d)...", work_size);
          NRF_LOG_FLUSH();
          break;
        case (FR_DISK_ERR)        :   NRF_LOG_INFO("Disk Error!!!"); break;
        case (FR_INT_ERR)         :   NRF_LOG_INFO("Internal Error!!!"); break;
        case (FR_DENIED)          :   NRF_LOG_INFO("Disk Access Denied!!!"); break;
        case (FR_INVALID_OBJECT)  :   NRF_LOG_INFO("File Object Invalid!!!"); break;
        case (FR_TIMEOUT)         :   NRF_LOG_INFO("Thread Control Timed-out!!!"); break;
        case (FR_OK)              :   NRF_LOG_INFO("Reformat Completed Successfully"); break;
        default                   :   NRF_LOG_INFO("Unknown Response (%d)", retval);

     }

      NRF_LOG_FLUSH();
      if (retval != FR_OK) {
        error_led_flash_loop();
        __BKPT();
      }

      reformat = false;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);



    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&m_fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}

/**
 * @brief List the SDCard (debugging only?).
 */
static int List_SDCard(void)
{
    static DIR dir;
    static FILINFO fno;
    FRESULT ff_result;

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&m_fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return NRF_ERROR_INTERNAL;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return NRF_ERROR_INTERNAL;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return NRF_ERROR_INTERNAL;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s\n",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s\n", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("");

    return 0;
}

void Swap_and_Save_Buffer(void) {

  static datalog_memory_t *read_ptr;
  static datalog_memory_t *read_ptr_done;
  static uint8_t m_mem_in_use = 0;

  // Disable IRQ before manipulating buffers and pointers.
  __disable_irq();

  // Move pointers
  if (m_mem_in_use) {
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_0;
    read_ptr            = (datalog_memory_t*) datalog_memory_1;
    m_mem_in_use        = 0;
  } else {
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_1;
    read_ptr            = (datalog_memory_t*) datalog_memory_0;
    m_mem_in_use        = 1;      
  }

  // Save the end of the buffer prior to resetting the write offset pointer
  read_ptr_done     = (datalog_memory_t*) (read_ptr + m_write_offset);\

  // Reset the Write Offset
  m_write_offset      = 0;

  // Re-enable IRQs
  __enable_irq();

  // Now write the buffer
  sdcard_write(read_ptr, read_ptr_done);

}


static FRESULT sdcard_write(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) {
    static FIL file;
    FRESULT ff_result;
    uint32_t bytes_written = 0; 

    board_led_on(LED_GREEN);
    board_led_off(LED_BLUE);

    static char     filename [20];
    static uint16_t fileindex = 0;

    sprintf(filename, "%s_%d.%s", FILENAME_BASENAME,fileindex,FILENAME_SUFFIX);

    NRF_LOG_INFO("Writing to file %s...", filename);
    NRF_LOG_FLUSH();
    ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);

    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: %s.", filename);
        NRF_LOG_FLUSH();
        return NRF_ERROR_INTERNAL;
    }
    uint32_t filesize = f_size(&file);

    NRF_LOG_INFO("File size is %d bytes...", filesize);
    NRF_LOG_FLUSH();

    if (filesize > MAX_FILESIZE) {
      f_close(&file);
      fileindex++;
      sprintf(filename, "%s_%d.%s", FILENAME_BASENAME,fileindex,FILENAME_SUFFIX);
      ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    }



    NRF_LOG_INFO("start_ptr = 0x%08x", (int) start_ptr);
    NRF_LOG_INFO("end_ptr = 0x%08x", (int) end_ptr);
    NRF_LOG_INFO("Total bytes to write = %d", (int) (end_ptr - start_ptr));
    NRF_LOG_FLUSH();

    uint8_t done = false;
    uint16_t total_bytes_written = 0;

#ifdef SDCARD_WRITE_PACKETS

    // Write the block one packet at a time.
    while (!done) {
      packet_info_t *p_header = (packet_info_t*) start_ptr;

      uint32_t bytes_to_write = p_header->length + sizeof(packet_info_t);

      // Audio packet longer than 255 bytes and so can't be represented by p_header->length.
      // Instead, custom field is used here.
      // TODO: TIDY UP USE OF CUSTOM FIELD. Re-organise to make length 16-bit for all.
      if ((p_header->logger_id) == LOGGER_AUDIO_ID) {
        ASSERT(p_header->length == 0);
        bytes_to_write = p_header->custom + sizeof(packet_info_t);
      }

      ff_result = f_write(&file, (const void*) start_ptr, bytes_to_write, (UINT *) &bytes_written);
      start_ptr += bytes_to_write;
      total_bytes_written += bytes_written;

      if (start_ptr >= end_ptr) {
        done = true;
      }
    }
#else
  // Write is as chunks of SD_WRITE_BLOCK_SIZE
#define SD_WRITE_BLOCK_SIZE 0
  while (!done) {

    uint16_t bytes_remaining = (int) (end_ptr - start_ptr);

    uint16_t bytes_per_write_cycle = SD_WRITE_BLOCK_SIZE;

    if (0 == bytes_per_write_cycle) {
      bytes_per_write_cycle = bytes_remaining;
    }  

    if (bytes_remaining < bytes_per_write_cycle) {
      bytes_per_write_cycle = bytes_remaining;
    }

    ff_result = f_write(&file, (const void*) start_ptr, bytes_per_write_cycle, (UINT *) &bytes_written);
    start_ptr += bytes_per_write_cycle;
    total_bytes_written += bytes_written;

    if (start_ptr >= end_ptr) {
      done = true;
    }
  }
#endif

    if (ff_result != FR_OK) {
        NRF_LOG_INFO("Write failed\r\n.");
        NRF_LOG_FLUSH();
    } else {
        NRF_LOG_INFO("%d bytes written.", total_bytes_written);
        NRF_LOG_FLUSH();
    }

    (void) f_close(&file);

    board_led_off(LED_GREEN);

    return ff_result;
  
}



///
/// MAIN
///

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code = 0;

    // Basic Checks
#if CAPTURE_AUDIO == 1
    ASSERT(AUDIO_SAMPLE_PERIOD_MS > AUDIO_SAMPLE_SIZE_MS);
#endif

    // Reset variable that counts RTC Wrap-Arounds.
    // TODO: Can we link the event to a TIMER/COUNTER task?
    m_timer_high = 0;

    bsp_board_init(BSP_INIT_LEDS);

    // Set the CS_MEM high to prevent any unwanted interactions
    nrf_gpio_cfg_output(CS_MEM);
    nrf_gpio_pin_set(CS_MEM);

    // Make sure the LEDs are off.
    board_led_off(LED_BLUE);
    board_led_off(LED_RED);
    board_led_off(LED_GREEN);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Starting EMCD Datalogger.");
    NRF_LOG_FLUSH();

#ifdef DEBUG
    // Use LEDs to indicate progress.
    board_led_on(LED_BLUE);
    board_led_on(LED_RED);
#endif

    //I2C bus 
    I2C_init();
    
    //BMI160 IMU
    BMI160_Turn_On();
    nrf_delay_ms(50);

    //BME680 Pressure Temperature Humidity and Gas Sensor 
    BME680_Turn_On();
    nrf_delay_ms(50);

    //BH1745NUC Colour Light Sensor
    BH1745NUC_Turn_On();
    nrf_delay_ms(50);

    // LPS22HB Pressure/Temp
    LPS22HB_Turn_On();

    //ADC subsystem 
    nrf_gpio_cfg_output(BAT_MON_EN);
    ADC_init_with_callback(&batt_level_updated_callback);

    // Point to first buffer for datalog memory and reset write offset
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_0;
    m_write_offset      = 0;

    NRF_LOG_INFO("Saving 2x %d Bytes of memory for data logging.", DATALOG_MEMORY_SIZE_BYTES);
    NRF_LOG_INFO("Will switch memory at %d Bytes.", DATALOG_MEMORY_LIMIT);
    NRF_LOG_FLUSH();

    // Open the SDCard.
    // Check that the card is there and readable
    err_code += Open_SDCard();
    if (err_code) {
      error_led_flash_loop();
    }

    err_code += List_SDCard();

    if (err_code) {
      error_led_flash_loop();
    }

    // Make sure the LEDs are off.
    board_led_off(LED_BLUE);
    board_led_off(LED_RED);
    board_led_off(LED_GREEN);

    // Write configuration information
    config_datalogger_handler();

#if CAPTURE_AUDIO == 1  
    // Configure PDM for Audio Capture
    pdm_init();
#endif // CAPTURE_AUDIO

    // RTC Timer
    lfclk_config();
    rtc_config();

    // Put the processor to some sort of sleep state.
    while (!err_code) {
        __WFE();
    }

    __BKPT();
}

/** @} */
