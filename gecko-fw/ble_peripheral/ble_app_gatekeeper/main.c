/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

/** @file 
 * 
 * @defgroup ble_sdk_uart_over_ble_main main.c 
 * @{ 
 * @ingroup  ble_sdk_app_nus_eval 
 * @brief    UART over BLE application main file. 
 * 
 * This file contains the source code for a sample application that uses the Nordic UART service. 
 * This application uses the @ref srvlib_conn_params module. 
 */ 
 
#include <stdint.h> 
#include <string.h> 
#include "sdk_config.h" 
 
#include "nordic_common.h" 
#include "nrf.h" 
#include "nrfx.h" 
#include "ble_hci.h" 
#include "ble_advdata.h" 
#include "ble_advertising.h" 
#include "ble_conn_params.h" 
#include "nrf_sdh.h" 
#include "nrf_sdh_soc.h" 
#include "nrf_sdh_ble.h" 
#include "nrf_ble_gatt.h" 
#include "nrf_ble_qwr.h" 
#include "nrf_drv_wdt.h"
#include "ff.h" 
#include "diskio_blkdev.h" 
#if USE_SDCARD == 1 
#include "nrf_block_dev_sdc.h" 
#endif 
#include "nrf_drv_rtc.h" 
#include "nrf_drv_clock.h" 
#include "app_timer.h" 
#include "ble_emcd_nus.h" 
#include "app_uart.h" 
#include "app_util_platform.h" 
#include "bsp_btn_ble.h" 
#include "nrf_pwr_mgmt.h" 
#include "nrfx_pdm.h" 
#include "nrf_drv_gpiote.h"
 
 
#if defined (UART_PRESENT) 
#include "nrf_uart.h" 
#endif 
#if defined (UARTE_PRESENT) 
#include "nrf_uarte.h" 
#endif 
 
#include "nrf_log.h" 
#include "nrf_log_ctrl.h" 
#include "nrf_log_default_backends.h" 


// OTA DFU SUPPORT
#if BLE_DFU_ENABLED == 1
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"
#endif

#include "peer_manager.h"
#include "peer_manager_handler.h"
 
#include "I2C.h"            //everything on the I2C bus 
#include "BMI160_if.h"      // Accelerometer 
#if USE_BSEC_LIBRARY == 1
#include "BSEC_if.h"        // Bosch Sensor Environmental Cluster driving BME680
#else
#include "BME680_if.h"      // Raw Environmental Sensor (Pressure, Temperature, Humidity, Gas Resistance 
#endif
#include "BH1745NUC_if.h"   // Colour Sensor 
#include "LPS22HB_if.h"     // LPS22HB Pressure / Temp 
#include "ADC.h"            // ADC for Battery Level Measurement 

// FIRMWARE VERSION information define at the beginning of sdk_config.h file
# define FIRMWARE_VERSION_ID FIRMWARE_MAJOR_VERSION * 1000000 + \
                             FIRMWARE_MINOR_VERSION *   10000 + \
                             FIRMWARE_PATCH_NUM *         100 + \
                             FIRMWARE_BUILD_NUM

// Anytime the format of the data is changed in a way that requires a change to the data  
// processing script that is NOT backward compatible with older data (i.e. not just adding  
// a new sensor) then increment this version code  
//#define CONFIG_DATA_VERSION             2 //Enable realtime set by ble master
//define  CONFIG_DATA_VERSION             3 //Increase Timestamp Bitwidth
#define CONFIG_DATA_VERSION             4  // Added Gas Quality (PPM VOC)
 
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */ 
 
#if USE_BLE_COMMS == 1
#ifdef BOARD_NAME
#define DEVICE_NAME BOARD_NAME
#else
#define DEVICE_NAME                     "Gecko"                                     /**< Name of device. Will be included in the advertising data. */ 
#endif
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */ 
    
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */ 
 
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */ 
 
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */ 
 
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */ 
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */ 
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */ 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */ 
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */ 
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */ 
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */ 
#endif 

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */ 

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */ 
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */ 
 
#define REFORMAT_ON_POR true  
 
#define REFORMAT_USES_DATALOG_MEM 
#define MEMORY_SIZE_KB(n) (n) * 1024 
 
#define DATALOG_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(4) 
// PDM, once it has a buffer, will continue until it has filled the buffer. 
// To limit this to 500ms, the buffer needs to hold only 500ms worth of mono
// audio data at 16.125kHz i.e. 8062 samples at 16 bit/sample -> 16124 

#define AUDIO_MEMORY_SIZE_BYTES MEMORY_SIZE_KB(32) 
#define AUDIO_MEMORY_NUM_BUFFERS  2
#define AUDIO_MEMORY_BUFFER_SIZE AUDIO_MEMORY_SIZE_BYTES/AUDIO_MEMORY_NUM_BUFFERS

typedef uint8_t datalog_memory_t; 
typedef int16_t audio_memory_t;
#define SOP_BELLLABS 0x42656c6c

// BLE CONTROL COMMANDS
#define SYS_CONFIG_INFO         0x00
#define SYS_LED_ENABLE          0x01
#define IMU_SAMPLE_INTERVAL     0x02
#define ENV_SAMPLE_INTERVAL     0x03
#define COLOUR_SENSOR_INTERVAL  0x04
#define SET_TIME_COMMAND        0xff
#define AUDIO_MODE              0x05

#if USE_BLE_COMMS == 1
#if BLE_DFU_ENABLED == 1

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
//        NRF_LOG_DEBUG("In app_shutdown_handler with event %d", (int) event);

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}


NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context){
  NRF_LOG_DEBUG("In buttonless_dfu_sdh_state_observer with state %d", state);
};

//NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {};
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
  .handler = buttonless_dfu_sdh_state_observer,
  .p_context = NULL,
  };


static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event){
  NRF_LOG_DEBUG("In ble_dfu_evt_handler with event %d", event);
  NRF_LOG_FLUSH();
  };
#endif
#endif

// Function Declarations

void led_heartbeat_handler (void * p_context);
void config_datalogger_handler (void); 
void imu_datalogger_handler (void * p_context); 
void env_datalogger_handler (void * p_context); 
void lightsensor_datalogger_handler (void * p_context); 
void batt_level_handler (void * p_context); 
void board_led_override(void);
void board_led_restore(void);
static void nus_data_handler(ble_nus_evt_t * p_evt);
 
#if USE_SDCARD == 1 
static FRESULT sdcard_write(datalog_memory_t *, datalog_memory_t *); 
static int Open_SDCard(void); 
static int List_SDCard(); 
#endif 
 
#if USE_BLE_COMMS == 1
static uint32_t ble_send_data(datalog_memory_t *, datalog_memory_t *); 
#endif

void Swap_and_Save_Buffer(void); 

// Replace NRFX_RTC_US_TO_TICKS.  
// If us > 0x1fff then the intermediate expression overflows and truncates giving smaller  
// delays than intended.   
//#define NBL_NRFX_RTC_US_TO_TICKS(us,freq) (us > 0x1fff) ? (us / 1000000) * freq : (((us) * (freq)) / 1000000U) 
 
#define NBL_NRFX_RTC_US_TO_TICKS(us, freq) (int32_t) ((us / 1000000.0) * freq) 
 
#define FILENAME_BASENAME "dlog_" 
#define FILENAME_SUFFIX "bin" 
#define MAX_FILESIZE_4GB 0xffffffff /* Corresponds to 4GB-1 */ 
#define MAX_FILESIZE_2GB 0x80000000 /* Corresponds to 2GB   */ 
#define MAX_FILESIZE MAX_FILESIZE_2GB 
 
#ifndef BOARD_CUSTOM 
#  define SDC_SCK_PIN     SPIM0_SCK_PIN   ///< SDC serial clock (SCK) pin. 
#  define SDC_MOSI_PIN    SPIM0_MOSI_PIN  ///< SDC serial data in (DI) pin. 
#  define SDC_MISO_PIN    SPIM0_MISO_PIN  ///< SDC serial data out (DO) pin. 
#  define SDC_CS_PIN      SPIM0_SS_PIN    ///< SDC chip select (CS) pin. 
#endif 
 
//Time (in miliseconds) between LED Flashes 
#define PULSE_LED_PERIOD_MS 10000     
//Duration (in miliseconds) of LED Flashes 
#define PULSE_LED_ON_TIME_MS 50  
#define PULSE_LED_OFF_TIME_MS (PULSE_LED_PERIOD_MS - PULSE_LED_ON_TIME_MS) 
 
#define DEFAULT_IMU_SENSOR_SAMPLE_PERIOD_MS 50  
#define DEFAULT_ENV_SENSOR_SAMPLE_PERIOD_MS 30000 
#define DEFAULT_COLOUR_SENSOR_SAMPLE_PERIOD_MS 300 
#define DEFAULT_BATTERY_LEVEL_SAMPLE_PERIOD_MS 30000
 
//#if BOARD_HAS_PDM_MIC == 1 
#define DEFAULT_AUDIO_SAMPLE_PERIOD_MS 1000 
#define DEFAULT_AUDIO_SAMPLE_SIZE_MS 500
//#endif // BOARD_HAS_PDM_MIC 

typedef struct packet_info_struct { 
    uint32_t  sop; 
    uint32_t  timestamp_hi; // If we represent this as 64-bit we have alignment issues with padding added
    uint32_t  timestamp_lo; // So send as two 32-bit numbers and put together in the app. 
    uint8_t   logger_id; 
    uint8_t   length; 
    uint16_t  custom;     // Makes the structure end 32-bit aligned.
} packet_info_t; 
 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
static datalog_memory_t datalog_memory_0 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)]; 
static datalog_memory_t datalog_memory_1 [DATALOG_MEMORY_SIZE_BYTES/sizeof(datalog_memory_t)]; 
 
static datalog_memory_t *m_p_datalog_mem; 
static uint16_t m_write_offset; 
#endif  //GATEKEEPER_BUFFER_SENSOR_DATA
 
#if BOARD_HAS_PDM_MIC == 1   
static audio_memory_t   audiodata_memory[AUDIO_MEMORY_NUM_BUFFERS] [AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t)]; 
#endif // BOARD_HAS_PDM_MIC 

uint16_t m_imu_sensor_sample_period_ms = DEFAULT_IMU_SENSOR_SAMPLE_PERIOD_MS;
uint16_t m_env_sensor_sample_period_ms = DEFAULT_ENV_SENSOR_SAMPLE_PERIOD_MS;
uint16_t m_colour_sensor_sample_period_ms = DEFAULT_COLOUR_SENSOR_SAMPLE_PERIOD_MS;
uint16_t m_audio_sample_period_ms = DEFAULT_AUDIO_SAMPLE_PERIOD_MS;
uint16_t m_audio_sample_size_ms = DEFAULT_AUDIO_SAMPLE_SIZE_MS;

// Variable to indicate if watchdog is running
uint8_t m_watchdog_running = 0;

// Variable to determine name of files written to SDCard
uint16_t m_fileindex = 0; 
 

// Variable to store reason for last reset
uint32_t m_last_reset_reason;

enum reset_type { PowerOnReset=0, 
                  ResetPin=1, 
                  WatchdogReset=2,
                  SoftwareReset=4,
                  CPULockupReset=8, 
                  GPIOWakeUp=0x10000, 
                  LPCompWakeUp=0x20000, 
                  DebugWakeUp=0x40000, 
                  NFCWakeUp=0x80000
                  };;
 
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
 
#if USE_BLE_COMMS == 1
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */ 
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */ 
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/ 
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */ 
 
uint8_t   m_ble_wait_for_tx = 0; 

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */ 
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */ 
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */ 
{ 
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}/*,
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BLOOD_PRESSURE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_ALERT_NOTIFICATION_SERVICE, BLE_UUID_TYPE_BLE}*/
}; 
#endif  //USE_BLE_COMMS

uint8_t   m_ble_connected = 0; 
uint16_t  m_rtc2_cc2_count = 0; 
uint8_t   m_adc_oversample; 
uint8_t   m_lps22hb_present = 1; 
 

/**@brief Function for assert macro callback. 
 * 
 * @details This function will be called in case of an assert in the SoftDevice. 
 * 
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert. 
 * @warning On assert from the SoftDevice, the system can only recover on reset. 
 * 
 * @param[in] line_num    Line number of the failing ASSERT call. 
 * @param[in] p_file_name File name of the failing ASSERT call. 
 */ 
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) 
{ 
    NRF_LOG_ERROR("ASSERTION ERROR %s:%d", p_file_name, line_num);
    app_error_handler(DEAD_BEEF, line_num, p_file_name); 
} 
 
/**@brief Function for SystemView Timestamp Retrieval. 
 * 
*/ 
 
#if DEBUG_WITH_SYSTEMVIEW == 1 
 
#include "SEGGER_SYSVIEW_Conf.h" 
#include "SEGGER_SYSVIEW.h" 
 
#endif //DEBUG_WITH_SYSTEMVIEW
 
 
/**@brief Function for initializing the timer module. 
 */ 
static void app_timers_init(void) 
{ 
    ret_code_t err_code = app_timer_init(); 
    APP_ERROR_CHECK(err_code); 
} 


 
#if USE_BLE_COMMS == 1
/**@brief Function for the GAP initialization. 
 * 
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance. 
 */ 
static void gap_params_init(void) 
{ 
    uint32_t                err_code; 
    ble_gap_conn_params_t   gap_conn_params; 
    ble_gap_conn_sec_mode_t sec_mode; 

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode); 

    ble_gap_addr_t ble_addr;

    err_code = sd_ble_gap_addr_get(&ble_addr);

#define MAIN_GAP_PARAMS_TXT_STRING_SIZE 32
#define MAIN_GAP_PARAMS_TMP_STRING_SIZE 4

    char tmp_string [MAIN_GAP_PARAMS_TMP_STRING_SIZE];
    char txt_string [MAIN_GAP_PARAMS_TXT_STRING_SIZE];
    
    sprintf(txt_string, "Device MAC ID:- ");
    for (int i=5; i>=0; i--) {
      sprintf(tmp_string, "%02x",ble_addr.addr[i]);
      if (i) {
        strlcat(tmp_string, ":", MAIN_GAP_PARAMS_TMP_STRING_SIZE);
      } else {
        strlcat(tmp_string,"\n", MAIN_GAP_PARAMS_TMP_STRING_SIZE);
      }
      strlcat(txt_string, tmp_string, MAIN_GAP_PARAMS_TXT_STRING_SIZE);
    }

    NRF_LOG_INFO("%s", txt_string);
    NRF_LOG_FLUSH();

    sprintf(txt_string, "%s_%02x%02x", DEVICE_NAME, ble_addr.addr[1], ble_addr.addr[0]);

    NRF_LOG_INFO("Device Name: -%s", txt_string);
    NRF_LOG_FLUSH();

    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *) txt_string, 
                                          strlen(txt_string));


    APP_ERROR_CHECK(err_code); 
 
    memset(&gap_conn_params, 0, sizeof(gap_conn_params)); 
 
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL; 
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL; 
    gap_conn_params.slave_latency     = SLAVE_LATENCY; 
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT; 
 
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params); 
    APP_ERROR_CHECK(err_code); 
} 

 
/**@brief Function for handling Queued Write Module errors. 
 * 
 * @details A pointer to this function will be passed to each service which may need to inform the 
 *          application about an error. 
 * 
 * @param[in]   nrf_error   Error code containing information about what went wrong. 
 */ 
static void nrf_qwr_error_handler(uint32_t nrf_error) 
{ 
    APP_ERROR_HANDLER(nrf_error); 
} 
 
 

/**@brief Function for initializing services that will be used by the application. 
 */ 
static void services_init(void) 
{ 
    uint32_t           err_code; 
    ble_nus_init_t     nus_init; 
    nrf_ble_qwr_init_t qwr_init = {0}; 
     
    // Initialize Queued Write Module. 
    qwr_init.error_handler = nrf_qwr_error_handler; 
 
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init); 
    APP_ERROR_CHECK(err_code); 

    // Initialize NUS. 
    memset(&nus_init, 0, sizeof(nus_init)); 
 
    nus_init.data_handler = nus_data_handler; 
 
    err_code = ble_nus_init(&m_nus, &nus_init); 
    APP_ERROR_CHECK(err_code); 

#if BLE_DFU_ENABLED == 1

    ble_dfu_buttonless_init_t dfus_init = {0};

    // Initialize the async SVCI interface to bootloader.
//    err_code = ble_dfu_buttonless_async_svci_init();
//    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    NRF_LOG_INFO("services_init: Enabling Buttonless DFU");
    NRF_LOG_FLUSH();
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code); 
        
    NRF_LOG_INFO("services_init: Buttonless DFU Enabled");
    NRF_LOG_FLUSH();
#endif 

} 
 
/**@brief Function for handling an event from the Connection Parameters Module. 
 * 
 * @details This function will be called for all events in the Connection Parameters Module 
 *          which are passed to the application. 
 * 
 * @note All this function does is to disconnect. This could have been done by simply setting 
 *       the disconnect_on_fail config parameter, but instead we use the event handler 
 *       mechanism to demonstrate its use. 
 * 
 * @param[in] p_evt  Event received from the Connection Parameters Module. 
 */ 
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) 
{ 
    uint32_t err_code; 
 
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) 
    { 
        m_ble_connected = 0; 
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); 
        APP_ERROR_CHECK(err_code); 
    }  
//    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)  
//    { 
//        m_ble_connected = 1; 
//    } 
} 
 
 
/**@brief Function for handling errors from the Connection Parameters module. 
 * 
 * @param[in] nrf_error  Error code containing information about what went wrong. 
 */ 
static void conn_params_error_handler(uint32_t nrf_error) 
{ 
    APP_ERROR_HANDLER(nrf_error); 
} 
 
 
/**@brief Function for initializing the Connection Parameters module. 
 */ 
static void conn_params_init(void) 
{ 
    uint32_t               err_code; 
    ble_conn_params_init_t cp_init; 
 
    memset(&cp_init, 0, sizeof(cp_init)); 
 
    cp_init.p_conn_params                  = NULL; 
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY; 
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY; 
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT; 
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; 
    cp_init.disconnect_on_fail             = false; 
    cp_init.evt_handler                    = on_conn_params_evt; 
    cp_init.error_handler                  = conn_params_error_handler; 
 
    err_code = ble_conn_params_init(&cp_init); 
    APP_ERROR_CHECK(err_code); 
} 
#endif
 
/**@brief Function for putting the chip into sleep mode. 
 * 
 * @note This function will not return. 
 */ 
static void sleep_mode_enter(void) 
{ 
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE); 
    APP_ERROR_CHECK(err_code); 
 
    NRF_LOG_DEBUG("Sleeping / Waiting for Event...\n"); 
 
    __WFE(); 
    // Prepare wakeup buttons. 
//    err_code = bsp_btn_ble_sleep_mode_prepare(); 
//    APP_ERROR_CHECK(err_code); 
// 
//    // Go to system-off mode (this function will not return; wakeup will cause a reset). 
//    err_code = sd_power_system_off(); 
//    APP_ERROR_CHECK(err_code); 
} 
 
 
#if USE_BLE_COMMS == 1
/**@brief Function for handling advertising events. 
 * 
 * @details This function will be called for advertising events which are passed to the application. 
 * 
 * @param[in] ble_adv_evt  Advertising event. 
 */ 
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) 
{ 
    uint32_t err_code; 
 
    switch (ble_adv_evt) 
    { 
        case BLE_ADV_EVT_FAST: 
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING); 
            APP_ERROR_CHECK(err_code); 
            break; 
        case BLE_ADV_EVT_IDLE: 
            sleep_mode_enter(); 
            break; 
        default: 
            break; 
    } 
} 
 
/**@brief Function for handling BLE events. 
 * 
 * @param[in]   p_ble_evt   Bluetooth stack event. 
 * @param[in]   p_context   Unused. 
 */ 
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) 
{ 
    uint32_t err_code; 
 
    switch (p_ble_evt->header.evt_id) 
    { 
        case BLE_GAP_EVT_CONNECTED: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_INFO("Connecting...");  
#endif
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED); 
            APP_ERROR_CHECK(err_code); 
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle; 
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle); 
            APP_ERROR_CHECK(err_code); 
            break; 
 
        case BLE_GAP_EVT_DISCONNECTED: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_INFO("Disconnected"); 
#endif
            m_ble_connected = 0; 
            // LED indication will be changed when advertising starts. 
            m_conn_handle = BLE_CONN_HANDLE_INVALID; 
            break; 
 
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: 
        { 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("PHY update request."); 
#endif
            ble_gap_phys_t const phys = 
            { 
                .rx_phys = BLE_GAP_PHY_AUTO, 
                .tx_phys = BLE_GAP_PHY_AUTO, 
            }; 
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys); 
            APP_ERROR_CHECK(err_code); 
        } break; 
 
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST: 
            // Pairing not supported 
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL); 
            APP_ERROR_CHECK(err_code); 
            break; 
 
        case BLE_GATTS_EVT_SYS_ATTR_MISSING: 
            // No system attributes have been stored. 
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0); 
            APP_ERROR_CHECK(err_code); 
            break; 
 
        case BLE_GATTC_EVT_TIMEOUT: 
            // Disconnect on GATT Client timeout event. 
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, 
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION); 
            APP_ERROR_CHECK(err_code); 
            break; 
 
        case BLE_GATTS_EVT_TIMEOUT: 
            // Disconnect on GATT Server timeout event. 
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, 
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION); 
            APP_ERROR_CHECK(err_code); 
            break; 
 
        case BLE_GATTS_EVT_HVN_TX_COMPLETE: 
            // Wake up CPU 
//            NRF_LOG_DEBUG("TX Complete Event\n"); 
            m_ble_wait_for_tx = 0; 
            __SEV(); 
            break; 
 
        case BLE_GATTC_EVT_WRITE_RSP: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GATTC Write Response Event\n"); 
#endif
            break; 
 
        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GATTC MTU Exchange Response Event\n"); 
#endif
            break; 
 
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP Data Length Update Event\n"); 
#endif
            break; 
 
        case BLE_GATTS_EVT_WRITE: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE Write Event\n"); 
#endif
            break; 
 
        case BLE_GAP_EVT_PHY_UPDATE: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP PHY Update Event\n"); 
#endif
            break; 
 
        case BLE_NUS_EVT_TX_RDY: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE GAP Connected Event\n"); 
#endif
            break; 
 
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\n"); 
#endif
            break; 
 
//        case BLE_GAP_EVT_PHY_UPDATE: 
//            NRF_LOG_DEBUG("\n"); 
//            break; 
//        case BLE_GAP_EVT_PHY_UPDATE: 
//            NRF_LOG_DEBUG("\n"); 
//            break; 
//        case BLE_GAP_EVT_PHY_UPDATE: 
//            NRF_LOG_DEBUG("\n"); 
//            break; 
//        case BLE_GAP_EVT_PHY_UPDATE: 
//            NRF_LOG_DEBUG("\n"); 
//            break; 
        default: 
            // No implementation needed. 
#if NBL_LOG_BLE_EVENTS == 1
            NRF_LOG_DEBUG("Event id 0x%x Received\n", p_ble_evt->header.evt_id); 
#endif
            break; 
    } 
} 
 
 
/**@brief Function for the SoftDevice initialization. 
 * 
 * @details This function initializes the SoftDevice and the BLE event interrupt. 
 */ 
static void ble_stack_init(void) 
{ 
    ret_code_t err_code; 
 

    err_code = nrf_sdh_enable_request(); 
    APP_ERROR_CHECK(err_code); 
 
    // Configure the BLE stack using the default settings. 
    // Fetch the start address of the application RAM. 
    uint32_t ram_start = 0; 
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start); 
    APP_ERROR_CHECK(err_code); 
    NRF_LOG_DEBUG("ram_start digi: %d",ram_start);
     NRF_LOG_DEBUG("ram_start hex: 0x%x",ram_start);
    // Enable BLE stack. 
    err_code = nrf_sdh_ble_enable(&ram_start); 
    APP_ERROR_CHECK(err_code); 
 
    // Register a handler for BLE events. 
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL); 
    NRF_LOG_DEBUG("Returning from ble_stack_init", err_code);
    NRF_LOG_FLUSH();
} 
 

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    NRF_LOG_DEBUG("Starting Peer Manager");
    NRF_LOG_FLUSH();

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("Peer Manager Initialized");
    NRF_LOG_FLUSH();
}


/**@brief Function for handling events from the GATT library. */ 
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) 
{ 
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) 
    { 
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH; 
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len); 
    } 
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x", 
                  p_gatt->att_mtu_desired_central, 
                  p_gatt->att_mtu_desired_periph); 
} 
 
 
/**@brief Function for initializing the GATT library. */ 
void gatt_init(void) 
{ 
    ret_code_t err_code; 
 
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler); 
    APP_ERROR_CHECK(err_code); 
 
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE); 
    APP_ERROR_CHECK(err_code); 
} 
 
 
/**@brief Function for handling events from the BSP module. 
 * 
 * @param[in]   event   Event generated by button press. 
 */ 
////void bsp_event_handler(bsp_event_t event) 
////{ 
////    uint32_t err_code; 
////    switch (event) 
////    { 
////        case BSP_EVENT_SLEEP: 
////            sleep_mode_enter(); 
////            break; 
//// 
////        case BSP_EVENT_DISCONNECT: 
////            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION); 
////            if (err_code != NRF_ERROR_INVALID_STATE) 
////            { 
////                APP_ERROR_CHECK(err_code); 
////            } 
////            break; 
//// 
////        case BSP_EVENT_WHITELIST_OFF: 
////            if (m_conn_handle == BLE_CONN_HANDLE_INVALID) 
////            { 
////                err_code = ble_advertising_restart_without_whitelist(&m_advertising); 
////                if (err_code != NRF_ERROR_INVALID_STATE) 
////                { 
////                    APP_ERROR_CHECK(err_code); 
////                } 
////            } 
////            break; 
//// 
////        default: 
////            break; 
////    } 
////} 
 
 
 
///**@brief   Function for handling app_uart events. 
// * 
// * @details This function will receive a single character from the app_uart module and append it to 
// *          a string. The string will be be sent over BLE when the last character received was a 
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length. 
// */ 
void uart_event_handle(app_uart_evt_t * p_event) {}; 
 
///**@snippet [Handling the data received over UART] */ 
//void uart_event_handle(app_uart_evt_t * p_event) 
//{ 
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN]; 
//    static uint8_t index = 0; 
//    uint32_t       err_code; 
// 
//    switch (p_event->evt_type) 
//    { 
//        case APP_UART_DATA_READY: 
//            UNUSED_VARIABLE(app_uart_get(&data_array[index])); 
//            index++; 
// 
//            if ((data_array[index - 1] == '\n') || 
//                (data_array[index - 1] == '\r') || 
//                (index >= m_ble_nus_max_data_len)) 
//            { 
//                if (index > 1) 
//                { 
//                    NRF_LOG_DEBUG("Ready to send data over BLE NUS"); 
//                    NRF_LOG_HEXDUMP_DEBUG(data_array, index); 
// 
//                    do 
//                    { 
//                        uint16_t length = (uint16_t)index; 
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle); 
//                        if ((err_code != NRF_ERROR_INVALID_STATE) && 
//                            (err_code != NRF_ERROR_RESOURCES) && 
//                            (err_code != NRF_ERROR_NOT_FOUND)) 
//                        { 
//                            APP_ERROR_CHECK(err_code); 
//                        } 
//                    } while (err_code == NRF_ERROR_RESOURCES); 
//                } 
// 
//                index = 0; 
//            } 
//            break; 
// 
//        case APP_UART_COMMUNICATION_ERROR: 
//            APP_ERROR_HANDLER(p_event->data.error_communication); 
//            break; 
// 
//        case APP_UART_FIFO_ERROR: 
//            APP_ERROR_HANDLER(p_event->data.error_code); 
//            break; 
// 
//        default: 
//            break; 
//    } 
//} 
///**@snippet [Handling the data received over UART] */ 
// 
 
/**@brief  Function for initializing the UART module. 
 */ 
/**@snippet [UART Initialization] */ 
//static void uart_init(void) 
//{ 
//    uint32_t                     err_code; 
//    app_uart_comm_params_t const comm_params = 
//    { 
//        .rx_pin_no    = RX_PIN_NUMBER, 
//        .tx_pin_no    = TX_PIN_NUMBER, 
//        .rts_pin_no   = RTS_PIN_NUMBER, 
//        .cts_pin_no   = CTS_PIN_NUMBER, 
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED, 
//        .use_parity   = false, 
//#if defined (UART_PRESENT) 
//        .baud_rate    = NRF_UART_BAUDRATE_115200 
//#else 
//        .baud_rate    = NRF_UARTE_BAUDRATE_115200 
//#endif 
//    }; 
// 
//    APP_UART_FIFO_INIT(&comm_params, 
//                       UART_RX_BUF_SIZE, 
//                       UART_TX_BUF_SIZE, 
//                       uart_event_handle, 
//                       APP_IRQ_PRIORITY_LOWEST, 
//                       err_code); 
//    APP_ERROR_CHECK(err_code); 
//} 
/**@snippet [UART Initialization] */ 
 
 
/**@brief Function for initializing the Advertising functionality. 
 */ 
static void advertising_init(void) 
{ 
    uint32_t               err_code; 
    ble_advertising_init_t init; 
 
    memset(&init, 0, sizeof(init)); 
 
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME; 
    init.advdata.include_appearance = false; 
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE; 
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]); 
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids; 
     NRF_LOG_DEBUG("m_adv_uuids: %d",m_adv_uuids);
    init.config.ble_adv_fast_enabled  = true; 
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL; 
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION; 
    init.evt_handler = on_adv_evt; 
 
    err_code = ble_advertising_init(&m_advertising, &init); 
    APP_ERROR_CHECK(err_code); 
 
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG); 
} 

 
/**@brief Function for starting advertising. 
 */ 
static void advertising_start(void) 
{ 
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST); 
    APP_ERROR_CHECK(err_code); 
} 
#endif 
 
/**@brief Function for initializing buttons and leds. 
 * 
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up. 
 */ 
static void buttons_leds_init(bool * p_erase_bonds) 
{ 
//  bsp_event_t startup_event; 
 
//  uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler); 
  uint32_t err_code = bsp_init(BSP_INIT_LEDS, NULL); 
  ;APP_ERROR_CHECK(err_code); 
 
//  err_code = bsp_btn_ble_init(NULL, &startup_event); 
//  APP_ERROR_CHECK(err_code); 
 
//  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA); 
  *p_erase_bonds = 0; 
} 
 

 
/**@brief Function for initializing the Watchdog module. 
 */ 
nrf_drv_wdt_channel_id m_wdt_channel_id;

void watchdog_event_handler(void)
{
    NRF_LOG_ERROR("!!! WDOG FIRED !!!");

    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void watchdog_refresh(void)
{

    if (m_watchdog_running) {
      nrf_drv_wdt_channel_feed(m_wdt_channel_id);
    }
}

static void watchdog_init(void) 
{ 
    uint32_t err_code;
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, watchdog_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();

    m_watchdog_running = 1;
} 
 
/**@brief Function for initializing the nrf log module. 
 */ 
static void log_init(void) 
{ 
    ret_code_t err_code = NRF_LOG_INIT(NULL); 
    APP_ERROR_CHECK(err_code); 
 
    NRF_LOG_DEFAULT_BACKENDS_INIT(); 
} 
 
 
/**@brief Function for initializing power management. 
 */ 
static void power_management_init(void) 
{ 
    ret_code_t err_code; 
    err_code = nrf_pwr_mgmt_init(); 
    APP_ERROR_CHECK(err_code); 
} 
 
 
/**@brief Function for handling the idle state (main loop). 
 * 
 * @details If there is no pending log operation, then sleep until next the next event occurs. 
 */ 
static void idle_state_handle(void) 
{ 
#if USE_POWER_MANAGEMENT==1 
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS()); 
    nrf_pwr_mgmt_run(); // Calls sd_app_evt_wait();
#endif 
} 
 
 
// 24 bits from RTC + 32 bits here makes 56 bits which @32768Hz gives 3,268 years
// NRF_RTC2->COUNTER bits 14->0 are sub-second times (each count is 1/32768 seconds)
// NRF_RTC2->COUNTER bits 23->15 are Unix Epoch Time (if synchronised) 
// TODO: INVESTIGATE >>>   For some reason if I declare the following: - 
// static uint64_t m_timestamp_offset; 
// Then the IMU interrupt is never detected!!!
// Have to declare it as 2x32 bit instead
typedef struct {
  uint32_t lo;
  uint32_t hi;
} ts64_t;

ts64_t m_timestamp_offset;
ts64_t m_old_timestamp_offset;

void set_ts64 (ts64_t *p_ts64, uint64_t ts) {
  p_ts64->lo = ts & 0xffffffff;
  p_ts64->hi = (ts >> 32) & 0xffffffff;
}

uint64_t ts64_to_uint64(ts64_t *p_ts64){
  uint64_t uint64 = ((uint64_t) p_ts64->hi << 32) + p_ts64->lo;
  return uint64;
}

void add_ts64 (ts64_t *p_ts64,  uint64_t ts) {
  uint64_t uint64 = ts64_to_uint64(p_ts64);
  uint64 += ts;
  set_ts64(p_ts64, uint64);
}

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

void error_led_flash_5s(void) { 
 
  bsp_board_leds_off(); 
   
  for (int16_t i =0;i<25; i++){ 
    bsp_board_led_invert(LED_RED); 
    nrf_delay_ms(200); 
  } 
} 
 
/// =============================================== 
/// 
/// RTC, Timers and Handlers 
/// 
/// =============================================== 

#if USE_BLE_COMMS == 0
// Do not need if BLE enabled as this is already set up (since it uses RTC1)
/// LFCLK
///
/** 
 * @brief Dummy handler for LF clock init.
 *        SDK15.2+ requires handler for init (can no longer supply NULL)
 */
static void lfclk_handler(nrfx_clock_evt_type_t event_type)
{
    switch (event_type) {
    case NRFX_CLOCK_EVT_HFCLK_STARTED : NRF_LOG_DEBUG("HFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_LFCLK_STARTED : NRF_LOG_DEBUG("LFCLK has been started.\n"); break;
    case NRFX_CLOCK_EVT_CTTO          : NRF_LOG_DEBUG("Calibration timeout.\n"); break;    
    case NRFX_CLOCK_EVT_CAL_DONE      : NRF_LOG_DEBUG("Calibration has been done.\n"); break;     
    default                           : NRF_LOG_DEBUG("???? (!!! Unknown Event !!!)\n"); 
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

#endif 
 /** 
 * @brief RTC Timer Handle 
 */ 
const nrfx_rtc_t m_rtc_2 = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrfx_rtc for RTC2. */ 
 
//#if BOARD_HAS_PDM_MIC==1 
//void audio_datalogger_pdm_ctrl(uint8_t); 
////void ble_send_audio_data();
//#endif // BOARD_HAS_PDM_MIC 
 

/** @brief: Function for handling the RTC2 interrupts. 
 * Triggered on COMPAREn and OVERFLOW . 
 */ 
static void rtc2_handler(nrfx_rtc_int_type_t int_type) 
{ 

    switch (int_type) { 
    case NRFX_RTC_INT_OVERFLOW :  
                                  add_ts64(&m_timestamp_offset, 1<<24); 
                                  break;    
    default                    :  NRF_LOG_DEBUG("!!! In rtc2_handler due to Unknown or Unexpected Interrupt (%d) !!!)\n", 
                                                    int_type 
                                                    ); 
                                  ASSERT(0);
    } 
 
    return; 
} 
 
 
 
/** @brief Function initialization and configuration of RTC driver instance. 
 */ 
 
static void rtc_config(void) 
{ 
    uint32_t err_code; 

    // RTC 0 (SoftDevice managed)
    // ==========================
 
    // RTC 1 (Used for app_timer) 
    // ========================== 
         
    // RTC 2 (Available) 
    // =================

    // Use RTC2 for Timestamp. 
    // All other timers use app_timer library.
 
    //Initialize RTC instance 
    NRF_LOG_DEBUG("Starting RTC2 for timestamp");

    nrfx_rtc_config_t rtc2_config = NRF_DRV_RTC_DEFAULT_CONFIG; 
    rtc2_config.prescaler         = RTC_CONFIG_PRESCALER; 
    err_code = nrfx_rtc_init(&m_rtc_2, &rtc2_config, rtc2_handler); 
    APP_ERROR_CHECK(err_code); 
 
    //Disable tick event & interrupt 
    nrfx_rtc_tick_disable(&m_rtc_2); 
       
    // Enable Overflow Event (to keep track of timestamp).
    nrfx_rtc_overflow_enable(&m_rtc_2, true);
 
    // Enable the timer
    nrfx_rtc_enable(&m_rtc_2); 
} 

// App Timer Handlers


/**@brief LED Heartbeat Timer Handle. 
 *  
 */ 
APP_TIMER_DEF(m_timer_led_heartbeat);     /**< Handler for single shot timer used to blink LED 1. */

void led_heartbeat_handler (void * p_context) {

  ret_code_t err_code;
 
  switch (m_led_state & LED_GREEN) { 
    case (led_state_off)  : err_code = app_timer_start(m_timer_led_heartbeat, APP_TIMER_TICKS(PULSE_LED_ON_TIME_MS), NULL);
                            APP_ERROR_CHECK(err_code);
                            board_led_on(LED_GREEN); 
                            break; 
    case (led_state_on)   : err_code = app_timer_start(m_timer_led_heartbeat, APP_TIMER_TICKS(PULSE_LED_OFF_TIME_MS), NULL);
                            APP_ERROR_CHECK(err_code);
                            board_led_off(LED_GREEN); 
                            break; 
  } 
 
}
                              
/** @brief Audio Sampling Timer Handle.

    Sets the start and stop times for Audio Sampling
*/ 
#if BOARD_HAS_PDM_MIC == 1 

APP_TIMER_DEF(m_timer_audio);             /**< Handler for repeated timer used to sample audio. */
static uint8_t    m_audio_data_send = 0;

//app_timer_audio_handle only clear the m_audio_data_send flag to indicate if audio data need to 
//be sent over ble or not. Audio data is continously recorded to buffers controlled by pdm.
void app_timer_audio_handle (void * p_context) {
 
 m_audio_data_send = 0;
//    uint32_t        next_interval_ms = 0;
//
//    switch (audio_state) { 
//    case (0)  : next_interval_ms = (m_audio_sample_size_ms); 
//                audio_state = 1; 
//                break; 
//    case (1)  : next_interval_ms = (m_audio_sample_period_ms - m_audio_sample_size_ms); 
//                audio_state = 0; 
//                break; 
//    } 
//
//    if (0 == m_audio_sample_period_ms) {
//      app_timer_stop(m_timer_audio);
//    } else {
//    app_timer_start(m_timer_audio, APP_TIMER_TICKS(next_interval_ms), NULL);
//    NRF_LOG_DEBUG("m_timer_audio start for %d ms.",next_interval_ms);
//    }
//
//    audio_datalogger_pdm_ctrl(audio_state);
    
//    if (audio_state == 0) {
//    ble_send_audio_data();
//    }

}
#endif // BOARD_HAS_PDM_MIC 

/** @brief: Function for controlling the IMU app_timer. 
*/ 

APP_TIMER_DEF(m_timer_imu);               /**< Handler for repeated timer used to sample IMU. */

void app_timer_imu_config () {
 
    ret_code_t      err_code; 

    if (0 == m_imu_sensor_sample_period_ms) {
      err_code = app_timer_stop(m_timer_imu);
      APP_ERROR_CHECK(err_code); 
    } else {
      err_code = app_timer_start(m_timer_imu, APP_TIMER_TICKS(m_imu_sensor_sample_period_ms), NULL); 
      APP_ERROR_CHECK(err_code); 
    }
}

/** @brief: Function for controlling the ENV app_timer. 
*/ 

APP_TIMER_DEF(m_timer_env);               /**< Handler for repeated timer used to sample ENV. */

void app_timer_env_config () {

    ret_code_t      err_code; 

    if (0 == m_env_sensor_sample_period_ms) {
      err_code = app_timer_stop(m_timer_env);
      APP_ERROR_CHECK(err_code); 
    } else {
      err_code = app_timer_start(m_timer_env, APP_TIMER_TICKS(m_env_sensor_sample_period_ms), NULL); 
      APP_ERROR_CHECK(err_code); 
    }
}


/** @brief: Function for controlling the Colour Sense app_timer. 
*/ 
APP_TIMER_DEF(m_timer_lightsensor);          /**< Handler for repeated timer used to sample Colour Sensor. */

void app_timer_lightsensor_config () {

    static uint8_t app_timer_running = 0;
    ret_code_t      err_code; 

    if ((0 == m_colour_sensor_sample_period_ms) && (1 == app_timer_running)) {
      err_code = app_timer_stop(m_timer_lightsensor);
      APP_ERROR_CHECK(err_code); 
      app_timer_running = 0;

    } else {
      if (app_timer_running) {
        err_code = app_timer_stop(m_timer_lightsensor);
        APP_ERROR_CHECK(err_code); 
      }
      err_code = app_timer_start(m_timer_lightsensor, APP_TIMER_TICKS(m_colour_sensor_sample_period_ms), NULL); 
      APP_ERROR_CHECK(err_code); 
      app_timer_running = 1;    
    }
}

APP_TIMER_DEF(m_timer_battery_level);          /**< Handler for repeated timer used to sample Colour Sensor. */

        
/** @brief Function initialization and configuration of RTC driver instance. 
 */ 
 
static void app_timer_config(void) 
{ 
    ret_code_t err_code; 
 
    // UI LEDS.
    // =======
      
#if LED_HEARTBEAT_ENABLED == 1 
        
    // Create the LED Heartbeat timer
    err_code = app_timer_create(&m_timer_led_heartbeat,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                led_heartbeat_handler);
    APP_ERROR_CHECK(err_code);

    // Start the LED Heartbeat timer.
    err_code = app_timer_start(m_timer_led_heartbeat, APP_TIMER_TICKS(PULSE_LED_OFF_TIME_MS), NULL);
    APP_ERROR_CHECK(err_code);
#endif 

    // Audio Timer
    // ===========

#if BOARD_HAS_PDM_MIC == 1  

    // Create the Audio timer
    err_code = app_timer_create(&m_timer_audio,
                                APP_TIMER_MODE_REPEATED,
                                app_timer_audio_handle);
    APP_ERROR_CHECK(err_code);

    // Start the Audio timer.
    err_code = app_timer_start(m_timer_audio, APP_TIMER_TICKS(m_audio_sample_period_ms), NULL);
    APP_ERROR_CHECK(err_code);
#endif // BOARD_HAS_PDM_MIC 

#if BOARD_HAS_BMI160 == 1

    // IMU Timer
    // =========


    // Create the IMU timer
    err_code = app_timer_create(&m_timer_imu,
                                APP_TIMER_MODE_REPEATED,
                                imu_datalogger_handler);
    APP_ERROR_CHECK(err_code);

    // Start the IMU timer.
    app_timer_imu_config();

#endif

#if BOARD_HAS_BME680 | BOARD_HAS_LPS22HB

    // ENV Timer
    // =========

    // Create the ENV timer
    err_code = app_timer_create(&m_timer_env,
                                APP_TIMER_MODE_REPEATED,
                                env_datalogger_handler);
    APP_ERROR_CHECK(err_code);

    // Start the ENV timer.
    app_timer_env_config();
#endif

#if BOARD_HAS_BH1745NUC == 1

    // Light Sensor Timer
    // ===================

    // Create the Light Sensor timer
    err_code = app_timer_create(&m_timer_lightsensor,
                                APP_TIMER_MODE_REPEATED,
                                lightsensor_datalogger_handler);
    APP_ERROR_CHECK(err_code);

    // Start the Light Sensor timer.
    app_timer_lightsensor_config();
#endif

#if BOARD_HAS_BATMON == 1
    // Battery Level Sample Timer
    // ==========================

    // Create the Battery Level Sample timer
    err_code = app_timer_create(&m_timer_battery_level,
                                APP_TIMER_MODE_REPEATED,
                                batt_level_handler);
    APP_ERROR_CHECK(err_code);

    // Start the Battery Level Sample timer.
    uint16_t adc_oversample = (int) pow(2.0, NRF_SAADC->OVERSAMPLE); 
    app_timer_start(m_timer_battery_level, APP_TIMER_TICKS(DEFAULT_BATTERY_LEVEL_SAMPLE_PERIOD_MS / adc_oversample), NULL);

#endif

} 
 
 
/**@brief Functions for getting and setting the timestamp.
 **/
/**@brief Functions for setting the timestamp.
 **/
void set_timestamp(uint32_t UnixEpochTime) { 

  uint64_t timestamp_offset = 0;
 
  timestamp_offset = ((uint64_t) UnixEpochTime) << 15;

  // Reset the Sensor RTC Counter
  NRF_RTC2->TASKS_CLEAR = 1;

  m_old_timestamp_offset = m_timestamp_offset;

  set_ts64(&m_timestamp_offset, timestamp_offset);

} 

ts64_t get_timestamp(void) { 
  ts64_t timestamp = m_timestamp_offset;
  add_ts64(&timestamp, (uint64_t) NRF_RTC2->COUNTER);
  return timestamp; 
} 

int64_t get_timestamp_us(void) { 

  ts64_t timestamp_t64 = get_timestamp();

  uint64_t timestamp = ts64_to_uint64(&timestamp_t64);

  // Timestamp is in units of 1/32768 second.
  // We need to conver this to us.
  timestamp *= 1000000/32768;
  
  return timestamp; 
} 

#if USE_BLE_COMMS == 1
/**@brief Function for handling the data from the Nordic UART Service. 
 * 
 * @details This function will process the data received from the Nordic UART BLE Service and send 
 *          it to the UART module. 
 * 
 * @param[in] p_evt       Nordic UART Service event. 
 */ 
/**@snippet [Handling the data received over BLE] */ 
static void nus_data_handler(ble_nus_evt_t * p_evt) 
{ 
 
  switch (p_evt->type) { 
    case BLE_NUS_EVT_RX_DATA: 
 
          NRF_LOG_DEBUG("Received data from BLE NUS: -");
          NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length); 
 
          for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) 
          { 

              NRF_LOG_INFO("%c", p_evt->params.rx_data.p_data[i]);

          } 

          uint8_t command = p_evt->params.rx_data.p_data[0];
          uint8_t arg1    = p_evt->params.rx_data.p_data[1];


          //    let ControlCommands:[String] = ["SYS_Conf", "LED_En", "IMU_I", "ENV_I", "COL_I", "Audio"]
          //    let ControlArguments_SYS_Conf:[String] = ["Resend"]
          //    let ControlArguments_LED_En:[String] = ["Disabled", "Enabled"]
          //    let ControlArguments_IMU_I:[String] = ["Disabled", "1 ms", "2 ms", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "500 ms", "1s", "2s", "5s"]

          //    let ControlArguments_ENV_I:[String] = ["Disabled", "1 sec", "2 sec", "5 sec", "10 sec", "1 min", "5 min"]
          //    let ControlArguments_COL_I:[String] = ["Disabled", "1 sec", "2 sec", "5 sec", "10 sec", "1 min", "5 min"]
          //    let ControlArguments_Audio:[String] = ["Peak/Ave", "Stream"]
          uint32_t imu_vals[] = {0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000};
          uint32_t env_vals[] = {0, 1000, 2000, 5000, 10000, 60000, 300000};
          uint16_t last_value;

          switch (command) {
          case (SYS_CONFIG_INFO):
            config_datalogger_handler();
            break;
          case (SYS_LED_ENABLE): // TODO: Not really compatible with ble led indication
            if (arg1 == 0) {
              board_led_override();
            } else {
              board_led_restore();
            }
            break;

          case (IMU_SAMPLE_INTERVAL): 
            last_value = m_imu_sensor_sample_period_ms;
            m_imu_sensor_sample_period_ms = imu_vals[arg1];
            if (last_value == 0) {
              // If this was disabled there will be no upcoming timing events for this.
              // Create a new event. 
              app_timer_imu_config();
            }
            NRF_LOG_DEBUG("Setting IMU Sensor Interval to %d ms", m_imu_sensor_sample_period_ms);
            break;

          case (ENV_SAMPLE_INTERVAL): 
            last_value = m_env_sensor_sample_period_ms;
            m_env_sensor_sample_period_ms = env_vals[arg1];
            if (last_value == 0) {
              // If this was disabled there will be no upcoming timing events for this.
              // Create a new event. 
              app_timer_env_config();
            }
            NRF_LOG_DEBUG("Setting ENV Sensor Interval to %d ms", m_env_sensor_sample_period_ms);
            break;

          case (COLOUR_SENSOR_INTERVAL): 
            last_value = m_colour_sensor_sample_period_ms;
            m_colour_sensor_sample_period_ms = env_vals[arg1];
            if (last_value == 0) {
              // If this was disabled there will be no upcoming timing events for this.
              // Create a new event. 
              app_timer_lightsensor_config();
            }
            NRF_LOG_DEBUG("Setting COL Sensor Interval to %d ms", m_colour_sensor_sample_period_ms);
            break;

          case (AUDIO_MODE):
//            m_audio_sample_period_ms
//            m_audio_sample_size_ms
            break;

          case (SET_TIME_COMMAND):
             {
                uint8_t t0          = p_evt->params.rx_data.p_data[1];
                uint8_t t1          = p_evt->params.rx_data.p_data[2];
                uint8_t t2          = p_evt->params.rx_data.p_data[3];
                uint8_t t3          = p_evt->params.rx_data.p_data[4];
                uint32_t timestamp  = ((uint64_t) t3 << 24) 
                                    + ((uint64_t) t2 << 16)
                                    + ((uint64_t) t1 <<  8)
                                    + ((uint64_t) t0 <<  0);
                set_timestamp(timestamp);

                // Send the config data packet again so post processing can record the 
                // updated timestamp.
                config_datalogger_handler(); 
             }
//            m_audio_sample_size_ms
            break;

          default: break;
          }
 
          break; 
 
    case BLE_NUS_EVT_TX_RDY: 
//      NRF_LOG_DEBUG("TX Ready for data "); 
      m_ble_wait_for_tx = 0; 
      break; 
 
 
    case BLE_NUS_EVT_COMM_STARTED: 
      NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STARTED"); 
      m_ble_connected = 1; 
      config_datalogger_handler(); 
 
      // Also send environment and battery data as it may be a while before this is scheduled. 
      env_datalogger_handler(NULL); 
      batt_level_handler(NULL); 
       
 
      break; 
    case BLE_NUS_EVT_COMM_STOPPED: 
      NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STOPPED"); 
      m_ble_connected = 0; 
      break; 
  } 
 
  NRF_LOG_FLUSH(); 
} 
/**@snippet [Handling the data received over BLE] */ 
#endif
 

/** 
 * BSEC Callback Handler. 
 */   
int32_t m_bsec_pressure; 
int32_t m_bsec_temperature; 
int32_t m_bsec_humidity; 
int32_t m_bsec_gas_resistance;
int32_t m_bsec_gas_iaq;

#if USE_BSEC_LIBRARY == 1
void bsec_callback(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent) {

#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("BSEC Callback\n");     
    NRF_LOG_DEBUG("=============\n"); 
    NRF_LOG_DEBUG("timestamp              %d", (timestamp >> 15) && 0xffffffff);
    NRF_LOG_DEBUG("iaq                    " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(iaq));
    NRF_LOG_DEBUG("iaq_accuracy           %d", iaq_accuracy);
    NRF_LOG_DEBUG("temperature            " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(temperature));
    NRF_LOG_DEBUG("humidity               " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(humidity));
    NRF_LOG_DEBUG("pressure               " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(pressure));
    NRF_LOG_DEBUG("raw_temperature        " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(raw_temperature));
    NRF_LOG_DEBUG("raw_humidity           " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(raw_humidity));
    NRF_LOG_DEBUG("gas                    " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(gas));
    NRF_LOG_DEBUG("static_iaq             " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(static_iaq));
    NRF_LOG_DEBUG("co2_equivalent         " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(co2_equivalent));
    NRF_LOG_DEBUG("breath_voc_equivalent  " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(breath_voc_equivalent));

    NRF_LOG_DEBUG("Status : %s", bsec_library_return2string(bsec_status));
#endif

    m_bsec_pressure = (int32_t) (pressure);
    m_bsec_temperature = (int32_t) (temperature *100.0);
    m_bsec_humidity = (int32_t) (humidity *1000.0);
    m_bsec_gas_resistance = (int32_t) (gas);
    m_bsec_gas_iaq = (int32_t) (iaq);
}
#endif
 
/** 
 * Env Sensor Timeout Handler. 
 */  
 
// Data is 4 32-bit words and 2 16-bit words: -  
// [0]  BME680 Pressure 
// [1]  BME680 Temperature 
// [2]  BME680 Humidity 
// [3]  BME680 Gas Resistance 
// [4a] LPS22HB Pressure 
// [4b] LPS22HB Temperature 
 
// TODO: We don't check if new value ready for Gas Resistance. 
#define LOGGER_ENV_DATA_LENGTH (4*5) + (2*2)
#define LOGGER_ENV_ID  0x02 

typedef struct { 
    packet_info_t   packet_info; 
    uint8_t         data[LOGGER_ENV_DATA_LENGTH]; 
} env_data_t; 

void env_datalogger_handler (void * p_context) { 
  env_data_t                env_data; 
  static uint16_t           packet_count = 0; 
  ts64_t                    timestamp = get_timestamp(); 
  uint16_t                  size = sizeof(env_data_t); 

 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
  } 
#endif 
 
  watchdog_refresh();

#if BOARD_HAS_BME680 == 0
  return;
#endif

 
    NRF_LOG_DEBUG("Env Handler"); 
    NRF_LOG_DEBUG("Env Timestamp %llu", ts64_to_uint64(&timestamp)); 
    NRF_LOG_DEBUG("Running for approx %d seconds",  
      NRF_RTC2->COUNTER * (RTC_CONFIG_PRESCALER+1)/RTC_INPUT_FREQ 
    ); 

  env_data.packet_info.sop            = SOP_BELLLABS; 
  env_data.packet_info.timestamp_lo   = timestamp.lo; 
  env_data.packet_info.timestamp_hi   = timestamp.hi; 
  env_data.packet_info.logger_id      = LOGGER_ENV_ID; 
  env_data.packet_info.length         = LOGGER_ENV_DATA_LENGTH; 
  env_data.packet_info.custom         = packet_count++; 
 
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
#if USE_BSEC_LIBRARY == 1
  *((int32_t*) env_data.data + 0) = m_bsec_pressure; 
  *((int32_t*) env_data.data + 1) = m_bsec_temperature; 
  *((int32_t*) env_data.data + 2) = m_bsec_humidity; 
  *((int32_t*) env_data.data + 3) = m_bsec_gas_resistance;
  *((int32_t*) env_data.data + 4) = m_bsec_gas_iaq;
#else
  BME680_Get_Data( (int32_t*) env_data.data ); 
  *((int32_t*) env_data.data + 4) = 0;
#endif
   
  // LPS22HB_Get_Data returns 2 16-bit words: -  
  //  data16[0] = LPS22HB pressure; 
  //  data16[1] = LPS22HB temperature; 
  if (m_lps22hb_present) { 
    LPS22HB_Get_Data( (uint16_t*) ((int32_t*) env_data.data + 5)); 
  } 
 
#endif //USE_DUMMY_SENSORS
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
  memcpy((m_p_datalog_mem+m_write_offset), &env_data,  size); 
  m_write_offset += size; 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1) { 
    ble_send_data((unsigned char*) &env_data, (unsigned char*) &env_data+size); 
  } 
#endif 
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
 
void imu_datalogger_handler (void * p_context) { 
  imu_data_t                imu_data; 
  static uint16_t           packet_count = 0; 
  ts64_t                    timestamp = get_timestamp(); 
  uint16_t                  size = sizeof(imu_data_t); 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
  } 
#endif 
 
  watchdog_refresh();

#if BOARD_HAS_BMI160 == 0
  return;
#endif

  imu_data.packet_info.sop            = SOP_BELLLABS; 
  imu_data.packet_info.timestamp_lo   = timestamp.lo; 
  imu_data.packet_info.timestamp_hi   = timestamp.hi;
  imu_data.packet_info.logger_id      = LOGGER_IMU_ID; 
  imu_data.packet_info.length         = LOGGER_IMU_DATA_LENGTH_BYTES; 
  imu_data.packet_info.custom         = packet_count++; 
 
#if USE_DUMMY_SENSORS == 0 
 
  BMI160_Get_Data(imu_data.data ); 
 
  // Clear the MSByte of the TimeStamp as this is only 24-bit into a 32-bit field 
  *((int8_t*) imu_data.data+15) = 0; 
#else 
  for (int i=0; i<LOGGER_IMU_DATA_LENGTH_HALF_WORDS; i++) { 
    imu_data.data[i] = (i<<8) + i; 
  } 
#endif 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
  ASSERT((uint32_t) m_p_datalog_mem+m_write_offset > 0x2000000); 
 
  memcpy((m_p_datalog_mem+m_write_offset), &imu_data,  size); 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1) { 
    ble_send_data((unsigned char*) &imu_data, (unsigned char*) &imu_data+size); 
  } 
#endif 
//#define DEBUG_IMU_PRINT_DATA 
#ifdef DEBUG_IMU_PRINT_DATA 
  NRF_LOG_DEBUG("gx=%5d gy=%5d gz=%5d ax=%5d ay=%5d az=%5d",  
    imu_data.data[0],  
    imu_data.data[1],  
    imu_data.data[2],  
    imu_data.data[3],  
    imu_data.data[4],  
    imu_data.data[5]) 
  NRF_LOG_DEBUG("ts=0x%08x", 
    (imu_data.data[7] << 16) + imu_data.data[6]); 
#endif 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  m_write_offset += size; 
#endif 
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
 
void lightsensor_datalogger_handler (void * p_context) { 
  lightsensor_data_t        lightsensor_data; 
  static uint16_t           packet_count = 0; 
  ts64_t                    timestamp = get_timestamp(); 
  uint16_t                  size = sizeof(lightsensor_data_t); 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
  } 
#endif 

#if (USE_BLE_COMMS==1) && (GECKO_IS_POWERED_SO_KEEP_READVERTISING==1)
    if (m_ble_connected == 0) {
      if (m_advertising.adv_evt == BLE_ADV_EVT_IDLE) {
        NRF_LOG_DEBUG("Starting Advertising");
        advertising_start();
      } //else {
//        NRF_LOG_DEBUG("Advertising already started...");
//      }
    }
#endif

  watchdog_refresh();
 
  lightsensor_data.packet_info.sop            = SOP_BELLLABS; 
  lightsensor_data.packet_info.timestamp_lo   = timestamp.lo; 
  lightsensor_data.packet_info.timestamp_hi   = timestamp.hi;
  lightsensor_data.packet_info.logger_id      = LOGGER_LIGHTSENSOR_ID; 
  lightsensor_data.packet_info.length         = LOGGER_LIGHTSENSOR_DATA_LENGTH_BYTES; 
  lightsensor_data.packet_info.custom         = packet_count++; 
 
  // Issue BitBucket #1 
  // Ensure LEDs are OFF for measurement.  
#if LIGHT_SENSOR_DISABLES_LED==1 
  board_led_override(); // 10-01-19 Physical Isolation of Colour Sensor. 
#endif 
 
 
#if USE_DUMMY_SENSORS == 0 
  // Data is array of bytes driver returns 16-bit half-words. 
  // Change the pointer type so it is transferred correctly. 
  BH1745NUC_Get_Data(lightsensor_data.data ); 
#endif 
 
  // Issue BitBucket #1 
  // Restore LED state.  
#if LIGHT_SENSOR_DISABLES_LED==1 
  board_led_restore(); 
#endif 

#if NBL_LOG_COLOUR==1
  uint16_t * p_data = (uint16_t*) lightsensor_data.data;
  NRF_LOG_DEBUG("RED    Value : %d", *(p_data+0));
  NRF_LOG_DEBUG("GREEN  Value : %d", *(p_data+1));
  NRF_LOG_DEBUG("BLUE   Value : %d", *(p_data+2));
  NRF_LOG_DEBUG("CLEAR  Value : %d", *(p_data+3));
  NRF_LOG_FLUSH();
#endif
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
//  NRF_LOG_DEBUG("Memory Copy destination 0x%08x to 0x%08x (lightsensor)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size); 
  memcpy((m_p_datalog_mem+m_write_offset), &lightsensor_data,  size); 
  m_write_offset += size; 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1) { 
    ble_send_data((unsigned char*) &lightsensor_data, (unsigned char*) &lightsensor_data+size); 
  } 
#endif 
} 
 
 
#if BOARD_HAS_PDM_MIC == 1   
 
/** 
 * Audio PDM Control Handler 
 */  
 
// There are two Audio handlers. The first is triggered by the timer and responsible  
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
//    - Continue servicing interrupts providing new buffers, releasing old buffers, checking errors.  
//    - Stop the PDM (nrfx_pdm_stop()) 
 
// Here the RTC triggered interrupt handler handles the PDM Start and Stop. 
 
//void audio_datalogger_pdm_ctrl(uint8_t turn_on) { 
//  ret_code_t ret_code; 
//  
////  NRF_LOG_DEBUG("Audio status turn_on/off (1/0): %d", turn_on);
//  
//  if (turn_on) { 
// 
//    // Start the PDM 
//#if NBL_LOG_PDM == 1
//    NRF_LOG_DEBUG("Starting PDM at RTC2 time ms = %d\n", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768);  
//    NRF_LOG_FLUSH();
//#endif
//    ret_code = nrfx_pdm_start(); 
//    
//    if (ret_code != 0) { 
//#if NBL_LOG_PDM == 1
//      NRF_LOG_DEBUG("PDM Already Started"); 
//      NRF_LOG_FLUSH();
//#endif
//      return;
//    } 
// 
//  } else { 
//#if NBL_LOG_PDM == 1
//    NRF_LOG_DEBUG("Stopping PDM at RTC2 time ms = %d\n", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768);  
//    NRF_LOG_FLUSH();
//#endif
//    ret_code = nrfx_pdm_stop(); 
//    if (ret_code != 0) { 
//      __BKPT(); 
//#if NBL_LOG_PDM == 1
//      NRF_LOG_DEBUG("PDM Already Stopping"); 
//      NRF_LOG_FLUSH();
//#endif
//    } 
//  } 
//
//} 
 
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
  static uint8_t    next_audio_buffer = 0;

#if NBL_LOG_PDM == 1
   static  uint16_t  event_count = 0;
   event_count++;
   NRF_LOG_DEBUG("audio_datalogger_handler event_count: %d",event_count);
  NRF_LOG_DEBUG("PDM Event Handler at ms = %d", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768); 
  NRF_LOG_DEBUG("p_evt->buffer_requested = %d", p_evt->buffer_requested); 
  NRF_LOG_DEBUG("p_evt->buffer_released = 0x%08x", p_evt->buffer_released); 
  NRF_LOG_DEBUG("p_evt->error = %d", p_evt->error);   
  NRF_LOG_DEBUG("nrfx_pdm_enable_check returns: %d\n", (int) nrfx_pdm_enable_check()); 
   NRF_LOG_FLUSH();
#endif

  if (p_evt->error) {
    __BKPT();
  }
  watchdog_refresh();
 
  if (p_evt->buffer_requested) { 

      // Note that the PDM driver will call this function twice with NULL p_evt->buffer_released. 
      // The first time is to get the PDM started with the first buffer and the second time is to obtain 
      // the location of the NEXT buffer for data storage. So if continous capture is required, then at 
      // least two buffers are needed with the PDM ping-ponging between them. For this to work, the data 
      // needs to be able to be processed / transmitted / stored in the time that it takes to fill another 
      // buffer.  
 
      // Point the PDM EasyDMA to the next buffer 
      ret_code = nrfx_pdm_buffer_set((int16_t*) audiodata_memory[next_audio_buffer],AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t));  
      APP_ERROR_CHECK(ret_code); 
#if NBL_LOG_PDM == 1
      NRF_LOG_DEBUG("PDM Event Handler Providing Next Buffer from 0x%08x to 0x%08x (%d elements)\n",  
                          audiodata_memory[next_audio_buffer], 
                          ((int) audiodata_memory[next_audio_buffer]) + AUDIO_MEMORY_BUFFER_SIZE,
                          AUDIO_MEMORY_BUFFER_SIZE/sizeof(audio_memory_t)
                   );
      NRF_LOG_FLUSH();
#endif  
      if (++next_audio_buffer == AUDIO_MEMORY_NUM_BUFFERS) {
        next_audio_buffer = 0;
      }
  }  
     
  if (p_evt->buffer_released != NULL) { 
//    comment out after debug 
//    ASSERT((audio_memory_t *) p_evt->buffer_released == (audio_memory_t *) audiodata_memory[0]) 

  if (m_audio_data_send == 0) {

//#if NBL_LOG_PDM == 1 
//    NRF_LOG_DEBUG("audio timestamp_lo in ms: %d", timestamp.lo*1000/32768);
    NRF_LOG_DEBUG("audio buffer released : = 0x%08x",(audio_memory_t *) p_evt->buffer_released);
    NRF_LOG_DEBUG("start processing time at %d ms.", (uint64_t) (NRF_RTC2->COUNTER)*1000/32768);
//    nrf_delay_ms(20);
    NRF_LOG_FLUSH();
//#endif


    audio_memory_t * p_buffer_released = (audio_memory_t *) p_evt->buffer_released;
    ts64_t timestamp                        = get_timestamp();
    audio_data.packet_info.sop              = SOP_BELLLABS; 
    audio_data.packet_info.logger_id        = LOGGER_AUDIO_ID; 
    audio_data.packet_info.length           = LOGGER_AUDIO_DATA_LENGTH*sizeof(audio_memory_t); 
    audio_data.packet_info.custom           = packet_count++; 
    audio_data.packet_info.timestamp_lo     = timestamp.lo; 
    audio_data.packet_info.timestamp_hi     = timestamp.hi;
 
 #if (USE_SDCARD + SAVE_FULL_AUDIO) == 2
 #error Not Implemented Yet!!!
 #else
    int64_t mean = 0; 
    int16_t peak = 0; 

    int16_t block_size = 32; 
    int32_t block_count = 0;      // used for calucation of mean, 

    int16_t pre_block[3] = {0};
    int16_t cur_block[3] = {0};
    int16_t pos_block[3] = {0};
         
    int64_t pre_block_mean = 0;
    int64_t pre_block_abs_mean = 0;
    int16_t pre_block_peak = 0; 

    int64_t cur_block_mean = 0;
    int64_t cur_block_abs_mean = 0;
    int16_t cur_block_peak = 0; 
          
    int64_t pos_block_mean = 0;
    int64_t pos_block_abs_mean = 0;
    int16_t pos_block_peak = 0;
    int16_t pos_block_mean_threshold = 5000;
//process previous blocks
   for (int j=0; j < block_size; j++) { 
      int16_t sample = *(p_buffer_released - block_size + j);
      if (sample == -32768) { sample ++;} //overflow at -32768 for abs
      int16_t abs_sample = abs(sample); 
      pre_block_mean += sample;
      pre_block_abs_mean += abs_sample;

      if (abs_sample > pre_block_peak) { 
          pre_block_peak = abs_sample;
          } 
      }
    pre_block_mean = pre_block_mean / block_size ; 
    pre_block_abs_mean = pre_block_abs_mean / block_size ; 

    if (abs(pre_block_mean) > pos_block_mean_threshold ) {
        pre_block[0] = 1;
        }

    pre_block[1] = pre_block_abs_mean;
    pre_block[2] = pre_block_peak;
//   NRF_LOG_DEBUG("pre_block info:mean:peak = %d:%d:%d",pre_block[0],pre_block[1],pre_block[2]);
//   nrf_delay_ms(20);

//process current blocks
//    NRF_LOG_DEBUG("start cur_block processing.");
//    NRF_LOG_DEBUG("cur_block address : = 0x%08x",p_buffer_released);
//    nrf_delay_ms(20);

    for (int j=0; j < block_size; j++) { 
      int16_t sample = *(p_buffer_released  + j);
      if (sample == -32768) { sample ++;} //overflow at -32768 for abs
      int16_t abs_sample = abs(sample); 
      cur_block_mean += sample;
      cur_block_abs_mean += abs_sample;

      if (abs_sample > cur_block_peak) { 
          cur_block_peak = abs_sample;
          } 
      }
    cur_block_mean = cur_block_mean / block_size ; 
    cur_block_abs_mean = cur_block_abs_mean / block_size ; 

    if (abs(cur_block_mean) > pos_block_mean_threshold ) {
        cur_block[0] = 1;
        }

    cur_block[1] = cur_block_abs_mean;
    cur_block[2] = cur_block_peak;
 
//     NRF_LOG_DEBUG("cur_block: %d, %d, %d",cur_block[0],cur_block[1],cur_block[2]);
//      nrf_delay_ms(10);

      for (int i=0; i<AUDIO_MEMORY_BUFFER_SIZE/(sizeof(audio_memory_t)*block_size); i++) { 
        
// process post blocks
           pos_block_mean = 0;
           pos_block_abs_mean = 0;
           pos_block_peak = 0;
          
            for (int j=0; j < block_size; j++) { 
                int16_t sample = *(p_buffer_released+(i+1)*block_size + j);
                if (sample == -32768) { sample ++;} //overflow at -32768 for abs
                int16_t abs_sample = abs(sample); 
                pos_block_mean += sample;
                pos_block_abs_mean += abs_sample;
            
                if (abs_sample > pos_block_peak) { 
                    pos_block_peak = abs_sample;
                    } 
                }
            pos_block_mean = pos_block_mean / block_size ; 
            pos_block_abs_mean = pos_block_abs_mean / block_size ; 
                pos_block[0] = 0;
            if (abs(pos_block_mean) > pos_block_mean_threshold ) {
                pos_block[0] = 1;
                }
            
              pos_block[1] = pos_block_abs_mean;
              pos_block[2] = pos_block_peak;          
//           NRF_LOG_DEBUG("pos_block: %d, %d, %d at %d ",pos_block[0],pos_block[1],pos_block[2],i);
//            nrf_delay_ms(10);

// check if current block data can be used or not?
        if ((pre_block[0] + cur_block[0] + pos_block[0]) == 0) {
            mean += cur_block[1];
            if (cur_block[2] > peak) { 
                peak = cur_block[2];
                }
            block_count++;
            }
 // reset pre_block and cur_block for next iteration     
          memcpy(pre_block, cur_block, 6);
          memcpy(cur_block, pos_block, 6);

        }
        NRF_LOG_DEBUG("total block_count used: %d",block_count);
        mean = mean / block_count;
    

    audio_data.data[0] = mean; 
    audio_data.data[1] = peak; 

//#if NBL_LOG_PDM == 1
   NRF_LOG_DEBUG("PDM Event Handler audio mean:peak = %d:%d", mean,peak);
   NRF_LOG_DEBUG("audio buffer processing finished at %d ms.\n",
       (uint64_t) (NRF_RTC2->COUNTER)*1000/32768,p_buffer_released); 
//   nrf_delay_ms(5);
//#endif

#if NBL_AUDIO_TEST_MODE == 1
 if(peak > 30000 && packet_count >1) {
        NRF_LOG_DEBUG("pdm stop ret_code: %d", ret_code = nrfx_pdm_stop()); 
        NRF_LOG_DEBUG("packet_count: %d",packet_count);
        nrf_delay_ms(10);
       __BKPT(); // Stop here to download an audio sample.
                 // Sample is at p_evt->buffer_released.
                 // Length is AUDIO_MEMORY_BUFFER_SIZE which is 
                 // AUDIO_MEMORY_SIZE / AUDIO_MEMORY_NUM_BUFFERS
       nrfx_pdm_start();
 }
#endif

#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
    } 
 
    ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
    memcpy((m_p_datalog_mem+m_write_offset), &audio_data,  size); 
    m_write_offset += size; 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
    if (m_ble_connected == 1) { 
      ble_send_data((unsigned char*) &audio_data, (unsigned char*) &audio_data+size); 
      NRF_LOG_DEBUG("ble audio data sent at ms = %d for buffer: 0x%08x\n",
       (uint64_t) (NRF_RTC2->COUNTER)*1000/32768,p_buffer_released); 
    } 
#endif 

    m_audio_data_send = 1;

    }  // end of m_audio_data_send == 0
  }    // end of p_evt->buffer_released != NULL
 
}      // end of audio_datalogger_handler

#endif // BOARD_HAS_PDM_MIC
 
/** 
 * @brief Initialise the PDM for Audio Sampling. 
 */ 
void pdm_init() { 
  ret_code_t ret_code; 
  nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(MIC_CLK, MIC_DOUT); 

  // Up the gain
  pdm_config.gain_l = NRF_PDM_GAIN_MAXIMUM; // 0x50
  pdm_config.gain_r = NRF_PDM_GAIN_MAXIMUM; // 0x50
//  
//  pdm_config.gain_l = NRF_PDM_GAIN_DEFAULT; // 0x28
//  pdm_config.gain_r = NRF_PDM_GAIN_DEFAULT; // 0x28
 
  // Clear the Audio Memory Buffers
  memset(audiodata_memory[0], 0, AUDIO_MEMORY_SIZE_BYTES); 
     
  // Set the MIC Power Control pin to output 
//  nrf_gpio_cfg_output(MIC_PWR_CTRL); 
    nrf_gpio_cfg(
      MIC_PWR_CTRL,
      NRF_GPIO_PIN_DIR_OUTPUT,
      NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL,
      NRF_GPIO_PIN_D0H1,
      NRF_GPIO_PIN_NOSENSE);
   
  // Turn the mic on 
  nrf_gpio_pin_set(MIC_PWR_CTRL); 
   
  // Initialise the PDM 
  ret_code = nrfx_pdm_init(&pdm_config, audio_datalogger_handler); 
  if (ret_code != NRF_SUCCESS) { 
    __BKPT(); 
  } 
//  nrf_pdm_int_enable(NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED | NRF_PDM_INT_END); 
  APP_ERROR_CHECK(ret_code); 

//  NRF_LOG_DEBUG("nrf_pdm_int_enable_check(start) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_STARTED ));
//  NRF_LOG_DEBUG("nrf_pdm_int_enable_check(stop) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_STOPPED ));
//  NRF_LOG_DEBUG("nrf_pdm_int_enable_check(end) returns: %d ",(int) nrf_pdm_int_enable_check(NRF_PDM_INT_END ));

}


// Start PDM Audio Sampling. 
void pdm_start() {
    ret_code_t ret_code;
#if NBL_LOG_PDM == 1
    NRF_LOG_DEBUG("Starting PDM at RTC2 time ms = %d\n", (uint32_t) (NRF_RTC2->COUNTER)*1000/32768);  
    NRF_LOG_FLUSH();
#endif

    ret_code = nrfx_pdm_start(); 
    APP_ERROR_CHECK(ret_code);
}

#endif // BOARD_HAS_PDM_MIC


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
void batt_level_handler (void * p_context) { 
 
   watchdog_refresh();

#if BOARD_HAS_BATMON == 0
  return;
#else

#if USE_DUMMY_SENSORS==0 
  // Enable the Battery Measurement Circuitry 
  nrf_gpio_pin_set(BAT_MON_EN); 
//  nrf_delay_ms(1); 
 
  //Battery ADC read 
  nrfx_saadc_sample(); 
#endif // USE_DUMMY_SENSORS

#endif // BOARD_HAS_BATMON
 

 
} 
 
/*  
 * @brief Callback for when ADC Value us updated 
 */ 
 
#if USE_DUMMY_SENSORS == 0 
#if BOARD_HAS_BATMON == 1
void batt_level_updated_callback (float vbatt) { 
  batt_lev_data_t           batt_data; 
  static uint16_t           packet_count = 0; 
  ts64_t                    timestamp = get_timestamp(); 
  uint16_t                  size = sizeof(batt_lev_data_t); 
  uint8_t                   battery_level_Percent = 0; 
  static float              filtered_vbatt; 
 
  // Figures taken from discharge curve measured overnight 27th November 2018 
  // Entries correspond to           0%,  10%,  20%,  30%,  40%,  50%,  60%,  70%,  80%,  90%, 100% 
//  const float battery_lookup [] = {3.20, 3.80, 3.95, 3.99, 4.04, 4.10, 4.15, 4.20, 4.23, 4.24, 4.25}; 

 // Entries correspond to          0%,  10%,  20%,  30%,  40%,  50%,  60%,  70%,  80%,  90%, 100% 
 const float battery_lookup [] = {3.30, 3.67, 3.73, 3.77, 3.81, 3.86, 3.89, 3.96, 4.04, 4.11, 4.20}; 

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
  if (filtered_vbatt > 4.2) { 
    filtered_vbatt = 4.2; 
  } else if (filtered_vbatt < 3.3){ 
    filtered_vbatt = 3.3; 
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
 
#if NBL_LOG_BATTERY == 1
  NRF_LOG_DEBUG("battery_percent: %d\n", battery_level_Percent); 
  NRF_LOG_DEBUG("Battery Level (V): %d.%03d\n", (int) vbatt, (int) ((float) round(1000 * (vbatt - floor(vbatt))))); 
  NRF_LOG_DEBUG("Filtered Level (V): %d.%03d\n", (int) filtered_vbatt, (int) ((float) round(1000 * (filtered_vbatt - floor(filtered_vbatt))))); 
  NRF_LOG_FLUSH(); 
#endif
   
  // For debugging and discharge analysis record the actual voltage 
  // To save this in uint8_t format: -  
  //      Battery Voltage should be in range c. 3V2 to 4V2.  
  //      Constrain 8-bit variable to voltage range 3v0 to 4v25  
  //        - Store in variable with range 0->255 
  //        - resolution is 1/200V (0.005V) Offset is 3.0v 
  float coded_vbatt = filtered_vbatt - 3.0; 
 
  coded_vbatt *= 200; 
 
  uint8_t coded_vbatt_8 = (uint8_t) coded_vbatt; 
  
  batt_data.data[0] = battery_level_Percent; 
  batt_data.data[1] = coded_vbatt_8; 
     
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
  } 
#endif 
 
  batt_data.packet_info.sop           = SOP_BELLLABS; 
  batt_data.packet_info.timestamp_lo  = timestamp.lo; 
  batt_data.packet_info.timestamp_hi  = timestamp.hi;
  batt_data.packet_info.logger_id     = LOGGER_BAT_LEVEL_ID; 
  batt_data.packet_info.length        = LOGGER_BAT_LEVEL_DATA_LENGTH; 
  batt_data.packet_info.custom        = packet_count++; 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
  memcpy((m_p_datalog_mem+m_write_offset), &batt_data,  size); 
  m_write_offset += size; 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1) { 
    ble_send_data((unsigned char*) &batt_data, (unsigned char*) &batt_data+size); 
  } 
#endif 
} 
 
#endif // BOARD_HAS_BATMON
#endif // USE_DUMMY_SENSORS
 
 
// 
// Write some Configuration Information for backend processing. 
// 
// 
 
#define LOGGER_CONFIG_DATA_LENGTH_BYTES 32 
#define LOGGER_CONFIG_ID  0x00 
 
typedef struct { 
    packet_info_t   packet_info; 
    int8_t          data[LOGGER_CONFIG_DATA_LENGTH_BYTES]; 
} config_data_t; 
 
void config_datalogger_handler (void) { 
  config_data_t             config_data; 
  static uint16_t           packet_count = 0; 
  ts64_t                    timestamp = get_timestamp(); 
  uint16_t                  size = sizeof(config_data_t); 
 
  watchdog_refresh();
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  if (m_ble_connected || (m_write_offset + size >= DATALOG_MEMORY_SIZE_BYTES)) { 
      Swap_and_Save_Buffer(); 
      NRF_LOG_DEBUG("Config Handler\n"); 
      NRF_LOG_FLUSH();
  } 
#endif 
 
  config_data.packet_info.sop           = SOP_BELLLABS; 
  config_data.packet_info.timestamp_lo  = timestamp.lo; 
  config_data.packet_info.timestamp_hi  = timestamp.hi;
  config_data.packet_info.logger_id     = LOGGER_CONFIG_ID; 
  config_data.packet_info.length        = LOGGER_CONFIG_DATA_LENGTH_BYTES; 
  config_data.packet_info.custom        = packet_count++; 
 
  // Change the pointer type so it is transferred correctly. 
  uint32_t *data32 = (uint32_t*) config_data.data; 
  uint16_t *data16 = (uint16_t*) config_data.data; 
 
  *data32 = NRF_RTC2->PRESCALER; 
  *(data16+2) = RTC_INPUT_FREQ; 
  *(data16+3) = CONFIG_DATA_VERSION;

  *(data32+2) = m_timestamp_offset.lo;
  *(data32+3) = m_timestamp_offset.hi;
  *(data32+4) = m_old_timestamp_offset.lo;
  *(data32+5) = m_old_timestamp_offset.hi;  
  *(data32+6) = FIRMWARE_VERSION_ID;  
  *(data32+7) = m_last_reset_reason;  

#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  ASSERT((m_write_offset + size) < DATALOG_MEMORY_SIZE_BYTES); 
//  NRF_LOG_DEBUG("Memory Copy destination 0x%08x to 0x%08x (config)\n", m_p_datalog_mem+m_write_offset, m_p_datalog_mem+m_write_offset + size); 
  memcpy((m_p_datalog_mem+m_write_offset), &config_data,  size); 
  m_write_offset += size; 
#endif 
 
#if BLE_STREAM_IMMEDIATE==1 
  if (m_ble_connected == 1) { 
    ble_send_data((unsigned char*) &config_data, (unsigned char*) &config_data+size); 
  } 
#endif 
} 
 

#if USE_SDCARD == 1 

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
    static uint8_t reformat = 0;
     
    // Only reformat if it was a Power On Reset and REFORMAT_ON_POR defined.
   if (REFORMAT_ON_POR && (m_last_reset_reason == PowerOnReset)) {
      reformat = 1;
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_DEBUG("PowerOnReset -> SDCard will be reformatted");
#endif
    } else {
      reformat = 0;
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_DEBUG("NOT PowerOnReset OR NOT REFORMAT_ON_POR -> SDCard will not be reformatted");
#endif
    }
 
//    uint32_t bytes_written; 
    FRESULT ff_result; 
    DSTATUS disk_state = STA_NOINIT; 
 
    // Initialize FATFS disk I/O interface by providing the block device. 
    static diskio_blkdev_t drives[] = 
    { 
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), &datalogger_wait_func) 
    }; 
 
    diskio_blockdev_register(drives, ARRAY_SIZE(drives)); 
 

#if NBL_LOG_OPEN_SDCARD == 1
    NRF_LOG_INFO("Initializing disk 0 (SDC)..."); 
    NRF_LOG_FLUSH();
#endif
 
    for (uint32_t retries = 3; retries && disk_state; --retries) 
    { 
        disk_state = disk_initialize(0); 
    } 
    if (disk_state) 
    { 
#if NBL_LOG_OPEN_SDCARD == 1
        NRF_LOG_INFO("Disk initialization failed."); 
        NRF_LOG_FLUSH(); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    // Only reformat if it was a Power On Reset.
    if (reformat) { 
      // Re-use datalog_memory for the working buffer for formatting. 
      FRESULT retval; 
#ifdef REFORMAT_USES_DATALOG_MEM 
      uint8_t *p_work = datalog_memory_0; 
      uint16_t work_size = DATALOG_MEMORY_SIZE_BYTES;    
#else 
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_INFO("=============================================="); 
      NRF_LOG_INFO("!!! Reformat uses separate memory.   !!!");       
      NRF_LOG_INFO("    Disable when not debugging"); 
      NRF_LOG_INFO("    (%s:%d)", __FILE__, __LINE__); 
#endif // NBL_LOG_OPEN_SDCARD
      uint8_t reformat_work [2048]; 
      uint8_t *p_work = reformat_work; 
      uint16_t work_size = sizeof(reformat_work); 
#endif // REFORMAT_USES_DATALOG_MEM

#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_INFO("Re-formatting volume..."); 
      NRF_LOG_INFO("   Using work memory size of %d bytes...", work_size); 
#endif // NBL_LOG_OPEN_SDCARD
      retval=f_mkfs("",FM_ANY, 0, p_work, work_size); 
 
#if NBL_LOG_OPEN_SDCARD == 1
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
#endif // NBL_LOG_OPEN_SDCARD
 
#if NBL_LOG_OPEN_SDCARD == 1
      NRF_LOG_FLUSH(); 
#endif // NBL_LOG_OPEN_SDCARD
      if (retval != FR_OK) { 
        error_led_flash_loop(); 
        __BKPT(); 
      } 
 
      reformat = false; 
    } 
 

#if NBL_LOG_OPEN_SDCARD == 1
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size; 
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb; 

    NRF_LOG_INFO("Capacity: %d MB", capacity); 
 
    NRF_LOG_INFO("Mounting volume..."); 
#endif
    ff_result = f_mount(&m_fs, "", 1); 
    if (ff_result) 
    { 
#if NBL_LOG_OPEN_SDCARD == 1
        NRF_LOG_INFO("Mount failed."); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    return NRF_SUCCESS; 
} 
 
/** 
 * @brief List the SDCard (debugging only?). 
 */ 
static int List_SDCard() 
{ 
    static DIR dir; 
    static FILINFO fno; 
    FRESULT ff_result; 
 
#if NBL_LOG_LIST_SDCARD == 1
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size; 
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb; 

    NRF_LOG_INFO("Capacity: %d MB", capacity); 
 
    NRF_LOG_INFO("Mounting volume..."); 
#endif

    ff_result = f_mount(&m_fs, "", 1); 
    if (ff_result) 
    { 
#if NBL_LOG_LIST_SDCARD == 1
        NRF_LOG_INFO("Mount failed."); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
#if NBL_LOG_LIST_SDCARD == 1
    NRF_LOG_INFO("\r\n Listing directory: /"); 
#endif
    ff_result = f_opendir(&dir, "/"); 
    if (ff_result) 
    { 
#if NBL_LOG_LIST_SDCARD == 1
        NRF_LOG_INFO("Directory listing failed!"); 
#endif
        return NRF_ERROR_INTERNAL; 
    } 
 
    uint16_t file_index = 0;
    do 
    { 
        ff_result = f_readdir(&dir, &fno); 
        if (ff_result != FR_OK) 
        { 
#if NBL_LOG_LIST_SDCARD == 1
            NRF_LOG_INFO("Directory read failed."); 
#endif
            return NRF_ERROR_INTERNAL; 
        } 
 
        if (fno.fname[0]) 
        { 
            if (fno.fattrib & AM_DIR) 
            { 
#if NBL_LOG_LIST_SDCARD == 1
                NRF_LOG_RAW_INFO("   <DIR>   %s\n",(uint32_t)fno.fname); 
#endif
            } 
            else 
            { 
#if NBL_LOG_LIST_SDCARD == 1
                NRF_LOG_RAW_INFO("%9lu  %s\n", fno.fsize, (uint32_t)fno.fname); 
#endif
            } 

            if (strncmp(fno.fname, FILENAME_BASENAME, strlen(FILENAME_BASENAME))) {
              uint8_t fileindex_position = strlen(FILENAME_BASENAME);
              char this_fileindex_char = fno.fname[fileindex_position];
              uint16_t this_fileindex = this_fileindex_char - '0';
              if (this_fileindex >= file_index) {
                file_index = this_fileindex+1;
                }
            }
        } 
    } 
    while (fno.fname[0]); 
#if NBL_LOG_LIST_SDCARD == 1
    NRF_LOG_RAW_INFO(""); 
#endif

    if (file_index > m_fileindex) {
      m_fileindex = file_index;
    }
 
#if NBL_LOG_LIST_SDCARD == 1 
    static char     filename [20];
    sprintf(filename, "%s%d.%s", FILENAME_BASENAME,m_fileindex,FILENAME_SUFFIX);   
    NRF_LOG_DEBUG("Setting initial file name to %s...", filename); 
    NRF_LOG_FLUSH(); 
#endif
    return 0; 
} 
#endif
 
void Swap_and_Save_Buffer(void) { 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
 
  static datalog_memory_t *read_ptr; 
  static datalog_memory_t *read_ptr_done; 
  static uint8_t mem_in_use = 0; 
 
  // Critical Region - prevent all non-SD interrupts. 
  sd_nvic_critical_region_enter(0); 
 
 
  // Move pointers 
  if (mem_in_use) { 
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_0; 
    read_ptr            = (datalog_memory_t*) datalog_memory_1; 
//    mem_in_use        = 0; 
  } else { 
    m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_1; 
    read_ptr            = (datalog_memory_t*) datalog_memory_0; 
//    mem_in_use        = 1;       
  } 
 
  // Save the end of the buffer prior to resetting the write offset pointer 
  read_ptr_done     = (datalog_memory_t*) (read_ptr + m_write_offset); 
 
  // Reset the Write Offset 
  m_write_offset      = 0; 
 
  mem_in_use ^= 1; 
 
  // End of Critical Region 
  sd_nvic_critical_region_exit(0); 
 
 
#if USE_SDCARD == 1 
  // Now write the buffer 
  sdcard_write(read_ptr, read_ptr_done); 
#endif 
 
#if BLE_STREAM_IMMEDIATE==0 
  if (m_ble_connected == 1) { 
#if USE_BLE_COMMS == 1
    ble_send_data(read_ptr, read_ptr_done); 
#else
    ASSERT(0);
#endif
  } else { 
#if USE_SDCARD == 0 
    NRF_LOG_INFO("Discarding Buffer as BLE not connected"); 
    NRF_LOG_FLUSH(); 
#endif
  } 
#endif 
#else 
  return; 
#endif 
 
} 
 
#if USE_BLE_COMMS == 1
static uint32_t ble_send_data(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) { 
 
  if (m_ble_connected == 0) {
    // Discard data and return as we are not connected.
    return 0;
  }

  static uint16_t packets_sent = 0; 
  ret_code_t err_code; 
 
#if NBL_LOG_BLE_SEND == 1 
  NRF_LOG_INFO("======================"); 
  NRF_LOG_INFO("   ble_send_data()"); 
  NRF_LOG_INFO("======================"); 
#endif 
 
//  board_led_on(LED_BLUE); 
//  board_led_off(LED_GREEN); 
//  board_led_off(LED_RED); 
 
#if NBL_LOG_BLE_SEND == 1 
  NRF_LOG_INFO("start_ptr = 0x%08x", (int) start_ptr); 
  NRF_LOG_INFO("end_ptr = 0x%08x", (int) end_ptr); 
  NRF_LOG_INFO("Total bytes to send = %d", (int) (end_ptr - start_ptr)); 
  NRF_LOG_INFO("Total packets to send (approx) = %d", (int) (end_ptr - start_ptr)/m_ble_nus_max_data_len); 
  NRF_LOG_FLUSH(); 
#endif 
 
  uint8_t done = false; 
  uint16_t total_bytes_sent = 0; 
 
  // Send as packets of m_ble_nus_max_data_len 
 
  while (!done) { 
 
    uint16_t bytes_remaining = (int) (end_ptr - start_ptr); 
 
    uint16_t packet_length = m_ble_nus_max_data_len; 
 
    if (bytes_remaining < packet_length) { 
      packet_length = bytes_remaining; 
    } 
 
    err_code = NRF_ERROR_IO_PENDING; // Anything non-zero. 
    uint16_t attempts_this_packet = 0; 
 
    while (err_code != NRF_SUCCESS) { 

      if (0 == m_ble_connected) {
         break;
      }

      err_code = ble_nus_data_send(&m_nus, start_ptr, &packet_length, m_conn_handle); 
      if (err_code == NRF_ERROR_INVALID_STATE) { 
//        NRF_LOG_ERROR("BLE : NRF_ERROR_INVALID_STATE.  (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent); 
//        NRF_LOG_FLUSH();       
      } else if (err_code == NRF_ERROR_BUSY) { 
        NRF_LOG_ERROR("BLE : NRF_ERROR_BUSY. (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent); 
        NRF_LOG_FLUSH();       
      } else if (err_code == NRF_ERROR_NOT_FOUND) { 
        NRF_LOG_ERROR("BLE : NRF_ERROR_NOT_FOUND. (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent); 
        NRF_LOG_FLUSH();  
      } else if (err_code == NRF_ERROR_RESOURCES) { 
#if NBL_LOG_BLE_SEND == 1 
        NRF_LOG_DEBUG("BLE : Not Enough Resource. Wait for TX Event... (Attempt %d; Packet %d)", attempts_this_packet++, packets_sent); 
        NRF_LOG_FLUSH(); 
#endif 
        m_ble_wait_for_tx = 1; 
        // Sample RTC to reset if we spend too long waiting for this to complete. 
        // Perhaps the client has gone out of range.
        uint32_t wait_for_tx_start_time = NRF_RTC2->COUNTER;
        uint32_t wait_for_tx_timeout_time = wait_for_tx_start_time + NBL_NRFX_RTC_US_TO_TICKS(5000000, RTC_INPUT_FREQ);

        uint32_t delay_loops = 0;
        while (m_ble_wait_for_tx) { 
            // Signal we are waiting for TX through the LED.
            // Blue should already be on...

            if (0 == m_ble_connected) {
              board_led_off(LED_GREEN);
              break;
            }

            if (!(delay_loops++%1000)) {
#if NBL_LOG_BLE_SEND == 1 
              NRF_LOG_INFO("Wait for TX");
#endif 
            }

            board_led_on(LED_GREEN);
            
//          __WFE(); 
            // TODO - Can we use a timer to do this? 
            nrf_delay_ms(1);
            if (NRF_RTC2->COUNTER > (wait_for_tx_timeout_time & 0x00ffffff)) {
              // TODO - Force a reset. It would be better if we know what sort of 
              // reset happened. 
              // Option 1 - keep in this loop. This (as it is called from the sensor handlers) will 
              // starve all the subsequent sensor handlers and thus starve the watchdog resulting in a 
              // WDOG reset. 
              // In future, do this more cleanly.
#if NBL_LOG_BLE_SEND == 1 
              NRF_LOG_DEBUG("Enter loop to starve WDOG");
#endif 
              board_led_on(LED_RED);
              nrf_delay_ms(1000);
              while(0) {};
            }
            board_led_off(LED_GREEN);
        } 

#if NBL_LOG_BLE_SEND == 1 
        if (delay_loops) {
          NRF_LOG_DEBUG("TX Done");
        }
        NRF_LOG_DEBUG("BLE : TX Event Received..."); 
        NRF_LOG_FLUSH(); 
#endif 
      } else if (err_code != NRF_SUCCESS) { 
        NRF_LOG_ERROR("Unexpected Return Code %x. Re-trying. (Attempt %d; Packet %d)", err_code, attempts_this_packet++, packets_sent); 
        NRF_LOG_FLUSH(); 
        __BKPT(); 
      } 
    } 
 
#if NBL_LOG_BLE_SEND == 1 
    NRF_LOG_DEBUG("Packet %d complete after %d retries", packets_sent++, attempts_this_packet); 
#endif 
    start_ptr += packet_length; 
    total_bytes_sent += packet_length; 
 
    if (start_ptr >= end_ptr) { 
      done = true; 
    } 
  } 
 
  packets_sent++; 
 
//  board_led_off(LED_BLUE); 
 
  return NRF_SUCCESS; 
} 
#endif


#if USE_SDCARD == 1 
static FRESULT sdcard_write(datalog_memory_t *start_ptr, datalog_memory_t *end_ptr) { 
    static FIL      file; 
    FRESULT         ff_result; 
    uint32_t        bytes_written = 0;  
    static char     filename [20]; 

    sprintf(filename, "%s%d.%s", FILENAME_BASENAME,m_fileindex,FILENAME_SUFFIX); 
 
#if NBL_LOG_SDCARD_WRITE == 1    
    NRF_LOG_DEBUG("Writing to file %s...", filename); 
    NRF_LOG_FLUSH(); 
#endif

    ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
 
    if (ff_result != FR_OK) 
    { 
        NRF_LOG_DEBUG("Unable to open or create file: %s.", filename); 
        NRF_LOG_FLUSH(); 
        return NRF_ERROR_INTERNAL; 
    } 
    uint32_t filesize = f_size(&file); 
 
#if NBL_LOG_SDCARD_WRITE == 1    
    NRF_LOG_DEBUG("File size is %d bytes...", filesize); 
    NRF_LOG_FLUSH(); 
#endif
 
    if (filesize > MAX_FILESIZE) { 
      f_close(&file); 
      m_fileindex++; 
      sprintf(filename, "%s_%d.%s", FILENAME_BASENAME,m_fileindex,FILENAME_SUFFIX); 
      ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND); 
    } 
 
#if NBL_LOG_SDCARD_WRITE == 1
    NRF_LOG_DEBUG("start_ptr = 0x%08x", (int) start_ptr); 
    NRF_LOG_DEBUG("end_ptr = 0x%08x", (int) end_ptr); 
    NRF_LOG_DEBUG("Total bytes to write = %d", (int) (end_ptr - start_ptr)); 
    NRF_LOG_FLUSH(); 
#endif
 
    uint8_t done = false; 
    uint16_t total_bytes_written = 0; 
 

// Write as chunks of SD_WRITE_BLOCK_SIZE 
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

#if NBL_LOG_SDCARD_WRITE == 1   
    if (ff_result != FR_OK) { 
        NRF_LOG_INFO("Write failed\r\n."); 
        NRF_LOG_FLUSH(); 
    } else { 
        NRF_LOG_INFO("%d bytes written.", total_bytes_written); 
        NRF_LOG_FLUSH(); 
    } 
#endif
 
    (void) f_close(&file); 
 
//    board_led_off(LED_GREEN); 
 
    return ff_result; 
   
} 
 
#endif 



void imu_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    NRF_LOG_DEBUG("IMU1 Interrupt Detected");

    watchdog_refresh();
 
#if USE_BLE_COMMS == 1
    if (m_ble_connected == 0) {
      if (m_advertising.adv_evt == BLE_ADV_EVT_IDLE) {
        NRF_LOG_DEBUG("Starting Advertising");
        advertising_start();
      }// else {
//        NRF_LOG_DEBUG("Advertising already started...");
//      }
    }
#endif

#if USE_SDCARD == 1
    // Write data to SDCard so we can switch off and/or remove card.
    NRF_LOG_DEBUG("Flushing Data to SDCard.");
    Swap_and_Save_Buffer();
#endif

    NRF_LOG_FLUSH();
}


void resetreas2string(uint32_t resetreas, char* reason_string) {
  char* strings [8] = {"PIN","WDOG","SREQ","LOCKUP","OFF","LPCOMP","DIF","NFC"};
  uint32_t compressed_reason;

  *reason_string = (char) '\0';


  if (resetreas == 0) {
    strcat(reason_string, "POR");
  } else {

    compressed_reason = resetreas & 0xf; // DCBA (bits 3:0);
    compressed_reason |= (resetreas & (0xf<<16)) >> 12; // HGFE (bits 19:16);

    for (int8_t i=0; i<8; i++) {
      if (compressed_reason & (1 << i)) {
        if (strlen(reason_string)) {
          strcat(reason_string, " | ");
        }
        strcat(reason_string, strings[i]);
      }
    }
  }

}



/**@brief Application main function. 
 */ 
int main(void) 
{ 
  bool erase_bonds; 
  ret_code_t err_code; 
 
#if DEBUG_WITH_SYSTEMVIEW == 1  
  SEGGER_SYSVIEW_Conf();            /* Configure and initialize SystemView  */ 
  SEGGER_SYSVIEW_Start();           /* Starts SystemView recording*/ 
  SEGGER_SYSVIEW_OnIdle();          /* Tells SystemView that System is currently in "Idle"*/ 
#endif 

  // Read and store the reset reason
  m_last_reset_reason = NRF_POWER->RESETREAS;

  // Reset the reset reason (could also write 0xf)
  NRF_POWER->RESETREAS = m_last_reset_reason;

  // Clear the timestamp.
  set_ts64(&m_timestamp_offset, 0);
  set_ts64(&m_old_timestamp_offset, 0);

  // Indicate that the watchdog is not yet running
  m_watchdog_running = 0;

#if USE_BLE_COMMS == 0
  // If BLE_COMMS not used then we need to start the LF Clock.
  lfclk_config(); 
#endif

  // Initialize. 
  log_init();

#if BLE_DFU_ENABLED==1 
  // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
  err_code = ble_dfu_buttonless_async_svci_init();
  APP_ERROR_CHECK(err_code);
#endif


  app_timers_init();
  buttons_leds_init(&erase_bonds); 
  power_management_init(); 

#if NBL_LOG_BOOT == 1
  char reason_string [55];
  resetreas2string(m_last_reset_reason, reason_string);
  NRF_LOG_DEBUG("");
  NRF_LOG_DEBUG("");
  NRF_LOG_DEBUG("");
  NRF_LOG_DEBUG("*****************************************");
  NRF_LOG_DEBUG("Booting.");
  NRF_LOG_DEBUG("--> Last reset was %s", reason_string);
  NRF_LOG_DEBUG("*****************************************");
  NRF_LOG_FLUSH();
#endif

#if BLE_DFU_ENABLED==1 
  NRF_LOG_DEBUG("Secure DFU Enabled!!!");
  NRF_LOG_FLUSH();
#else
  NRF_LOG_DEBUG("Secure NOT Enabled!!!");
  NRF_LOG_FLUSH();
#endif

#if USE_BLE_COMMS == 1
  NRF_LOG_DEBUG("ble_stack_init...");
  NRF_LOG_FLUSH();  
  ble_stack_init();  
  peer_manager_init();  
  gap_params_init(); 
  gatt_init(); 
  services_init(); 
  advertising_init(); 
  conn_params_init(); 
#endif

#if BLE_DFU_ENABLED==1 
  NRF_LOG_DEBUG("BLE Comms Setup Complete...");
  NRF_LOG_FLUSH();
#endif

  //I2C bus  
  I2C_init(); 

   
#if USE_SDCARD == 1 
  
  // Turn the SD Card on by setting SWITCH_CTRL low
  // Set the SWITCH_CTRL pin to output 
  nrf_gpio_cfg_output(SWITCH_CTRL); 
   
  // Power cycle the SD Card. SWITCH_CTRL high disables power.
  nrf_gpio_pin_set(SWITCH_CTRL);
  nrf_delay_ms(100);

  // Drive the SWITCH_CTRL low to enable power
  nrf_gpio_pin_clear(SWITCH_CTRL);


#ifdef DEBUG 
  // Use LEDs to indicate progress. 
  board_led_on(LED_BLUE); 
  board_led_on(LED_RED); 
#endif 
 
  // Open the SDCard. 
  // Check that the card is there and readable 
  err_code = Open_SDCard(); 
  if (err_code) { 
    //error_led_flash_loop(); 
    error_led_flash_5s();
  } 
 
  err_code = List_SDCard(); 
 
  if (err_code) { 
    //error_led_flash_loop(); 
       error_led_flash_5s();

  } 
#endif 
   

// RTC2 (timestamp) configuration 
rtc_config(); 


#if USE_DUMMY_SENSORS == 0 

  //BMI160 IMU 
#if BOARD_HAS_BMI160 == 1
  BMI160_Turn_On(); 

  // If using BLE, use INT1 interrupt to restart advertising
  // If using SDCard, use INT1 to force datapurge to SDCard.

  // Enable INT1 as interrupt using GPIOTE
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  err_code = nrf_drv_gpiote_in_init(IMU_INT1, &in_config, imu_int1_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(IMU_INT1, true);
  nrf_delay_ms(50); 
#endif


//BME680 Pressure Temperature Humidity and Gas Sensor  
#if BOARD_HAS_BME680 == 1

#if USE_BSEC_LIBRARY==1
      BSEC_Configure_and_Run(&bsec_callback);
#else
  BME680_Turn_On(); 
  nrf_delay_ms(50); 
#endif
#endif
 
  //BH1745NUC Colour Light Sensor 
#if BOARD_HAS_BH1745NUC == 1
  BH1745NUC_Turn_On(); 
  nrf_delay_ms(50); 
#endif

 
  // LPS22HB Pressure/Temp 
#if BOARD_HAS_LPS22HB == 1
  if ((NRF_UICR->CUSTOMER[0] & 0x2) == 0) { 
    m_lps22hb_present = 0; 
  } else { 
    LPS22HB_Turn_On(); 
  } 
#endif

#if BOARD_HAS_BATMON == 1
  //ADC subsystem for battery read
  nrf_gpio_cfg_output(BAT_MON_EN); 
  ADC_init_with_callback(&batt_level_updated_callback); 
#endif
 
#endif 
 
#if GATEKEEPER_BUFFER_SENSOR_DATA==1 
  // Point to first buffer for datalog memory and reset write offset 
  m_p_datalog_mem     = (datalog_memory_t*) datalog_memory_0; 
  m_write_offset      = 0; 
#endif 
 
  m_ble_connected = 0; 
 
  // Make sure the LEDs are off. 
  board_led_off(LED_BLUE); 
  board_led_off(LED_RED); 
  board_led_off(LED_GREEN); 
 
#if USE_SDCARD == 1 
  // Write configuration information 
  // (Although BLE not connected - it puts this as the first bit of information in the SDCard  
  // datalog. 
  config_datalogger_handler(); 
#endif 
 
#if BOARD_HAS_PDM_MIC == 1   
  // Configure PDM for Audio Capture 
  pdm_init(); 
#endif // BOARD_HAS_PDM_MIC 
 
  // app_timer configuration 
  app_timer_config();

#if BOARD_HAS_PDM_MIC == 1   
  // Start PDM for Audio Capture 
  pdm_start();
#endif // BOARD_HAS_PDM_MIC

  // Start the Watchdog Timer
  watchdog_init();
 
#if USE_BLE_COMMS == 1
  // Start execution. 
  NRF_LOG_INFO("Starting BLE..."); 
 
  advertising_start(); 
#endif
  // Enter main loop. 
  for (;;) 
  { 
      idle_state_handle(); // Calls sd_app_evt_wait(); 
  } 
} 
 
 
/** 
 * @} 
 */ 
