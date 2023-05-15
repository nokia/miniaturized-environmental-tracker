/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved. 
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_integration.c
 *
 * @brief
 * Private part of the example for using of BSEC library.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "bsec_integration.h"

/**********************************************************************************************************************/
/* local macro definitions */
/**********************************************************************************************************************/

#define NUM_USED_OUTPUTS 8

/**********************************************************************************************************************/
/* global variable declarations */
/**********************************************************************************************************************/

/* Global sensor APIs data structure */
static struct bme680_dev bme680_g;

/* Global temperature offset to be subtracted */
static float bme680_temperature_offset_g = 0.0f;

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *  
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme680_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;
    
    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    
    bsec_library_return_t status = BSEC_OK;
    
    /* note: Virtual sensors as desired to be added here */
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[7].sample_rate = sample_rate;
    
    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);
    
    return status;
}

/*!
 * @brief       Initialize the BME680 sensor and the BSEC library
 *
 * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
 * @param[in]   bus_write           pointer to the bus writing function
 * @param[in]   bus_read            pointer to the bus reading function
 * @param[in]   sleep               pointer to the system specific sleep function
 * @param[in]   state_load          pointer to the system-specific state load function
 * @param[in]   config_load         pointer to the system-specific config load function
 *
 * @return      zero if successful, negative otherwise
 */
return_values_init bsec_iot_init(float sample_rate, float temperature_offset, bme680_com_fptr_t bus_write, 
                    bme680_com_fptr_t bus_read, sleep_fct sleep, state_load_fct state_load, config_load_fct config_load)
{
    return_values_init ret = {BME680_OK, BSEC_OK};
    bsec_library_return_t bsec_status = BSEC_OK;
    
    uint8_t bsec_state[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    int bsec_state_len, bsec_config_len;
    
    /* Fixed I2C configuration */
    bme680_g.dev_id = BME680_I2C_ADDR_SECONDARY;
    bme680_g.intf = BME680_I2C_INTF;
    /* User configurable I2C configuration */
    bme680_g.write = bus_write;
    bme680_g.read = bus_read;
    bme680_g.delay_ms = sleep;
    
    /* Initialize BME680 API */
    ret.bme680_status = bme680_init(&bme680_g);
    if (ret.bme680_status != BME680_OK)
    {
        return ret;
    }
    
    /* Initialize BSEC library */
    ret.bsec_status = bsec_init();
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
    
    /* Load library config, if available */
    bsec_config_len = config_load(bsec_config, sizeof(bsec_config));
    if (bsec_config_len != 0)
    {       
        ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));     
        if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
    
    /* Load previous library state, if available */
    bsec_state_len = state_load(bsec_state, sizeof(bsec_state));
    if (bsec_state_len != 0)
    {       
        ret.bsec_status = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer));     
        if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
    
    /* Set temperature offset */
    bme680_temperature_offset_g = temperature_offset;
    
    /* Call to the function which sets the library with subscription information */
    ret.bsec_status = bme680_bsec_update_subscription(sample_rate);
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
    
    return ret;
}

/*!
 * @brief       Trigger the measurement based on sensor settings
 *
 * @param[in]   sensor_settings     settings of the BME680 sensor adopted by sensor control function
 * @param[in]   sleep               pointer to the system specific sleep function
 *
 * @return      none
 */
static void bme680_bsec_trigger_measurement(bsec_bme_settings_t *sensor_settings, uint16_t *sleep_duration)
{
    uint16_t meas_period;
    uint8_t set_required_settings;
    int8_t bme680_status = BME680_OK;
        
    /* Check if a forced-mode measurement should be triggered now */
    if (sensor_settings->trigger_measurement)
    {
        /* Set sensor configuration */

        bme680_g.tph_sett.os_hum  = sensor_settings->humidity_oversampling;
        bme680_g.tph_sett.os_pres = sensor_settings->pressure_oversampling;
        bme680_g.tph_sett.os_temp = sensor_settings->temperature_oversampling;
        bme680_g.gas_sett.run_gas = sensor_settings->run_gas;
        bme680_g.gas_sett.heatr_temp = sensor_settings->heater_temperature; /* degree Celsius */
        bme680_g.gas_sett.heatr_dur  = sensor_settings->heating_duration; /* milliseconds */
        
        /* Select the power mode */
        /* Must be set before writing the sensor configuration */
        bme680_g.power_mode = BME680_FORCED_MODE;
        /* Set the required sensor settings needed */
        set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;
        
        /* Set the desired sensor configuration */
        bme680_status = bme680_set_sensor_settings(set_required_settings, &bme680_g);
             
        /* Set power mode as forced mode and trigger forced mode measurement */
        bme680_status = bme680_set_sensor_mode(&bme680_g);
        
        /* Get the total measurement duration so as to sleep or wait till the measurement is complete */
        bme680_get_profile_dur(sleep_duration, &bme680_g);
    }
   
}

/*!
 * @brief       Read the data from registers and populate the inputs structure to be passed to do_steps function
 *
 * @param[in]   time_stamp_trigger      settings of the sensor returned from sensor control function
 * @param[in]   inputs                  input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs         number of inputs to be passed to do_steps
 * @param[in]   bsec_process_data       process data variable returned from sensor_control
 *
 * @return      none
 */
static void bme680_bsec_read_data(int64_t time_stamp_trigger, bsec_input_t *inputs, uint8_t *num_bsec_inputs,
    int32_t bsec_process_data)
{
    static struct bme680_field_data data;
    int8_t bme680_status = BME680_OK;
    
    /* We only have to read data if the previous call the bsec_sensor_control() actually asked for it */
    if (bsec_process_data)
    {
        bme680_status = bme680_get_sensor_data(&data, &bme680_g);

        if (data.status & BME680_NEW_DATA_MSK)
        {
            /* Pressure to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_PRESSURE)
            {
                /* Place presssure sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
                inputs[*num_bsec_inputs].signal = data.pressure;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Temperature to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_TEMPERATURE)
            {
                /* Place temperature sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
                #ifdef BME680_FLOAT_POINT_COMPENSATION
                    inputs[*num_bsec_inputs].signal = data.temperature;
                #else
                    inputs[*num_bsec_inputs].signal = data.temperature / 100.0f;
                #endif
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
                
                /* Also add optional heatsource input which will be subtracted from the temperature reading to 
                 * compensate for device-specific self-heating (supported in BSEC IAQ solution)*/
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
                inputs[*num_bsec_inputs].signal = bme680_temperature_offset_g;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Humidity to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_HUMIDITY)
            {
                /* Place humidity sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
                #ifdef BME680_FLOAT_POINT_COMPENSATION
                    inputs[*num_bsec_inputs].signal = data.humidity;
                #else
                    inputs[*num_bsec_inputs].signal = data.humidity / 1000.0f;
                #endif  
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Gas to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_GAS)
            {
                /* Check whether gas_valid flag is set */
                if(data.status & BME680_GASM_VALID_MSK)
                {
                    /* Place sample into input struct */
                    inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
                    inputs[*num_bsec_inputs].signal = data.gas_resistance;
                    inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                    (*num_bsec_inputs)++;
                }
            }
        }
    }
}

/*!
 * @brief       This function is written to process the sensor data for the requested virtual sensors
 *
 * @param[in]   bsec_inputs         input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs     number of inputs to be passed to do_steps
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 *
 * @return      none
 */
static void bme680_bsec_process_data(bsec_input_t *bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready)
{
    /* Output buffer set to the maximum virtual sensor outputs supported */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = 0;
    uint8_t index = 0;

    bsec_library_return_t bsec_status = BSEC_OK;
    
    int64_t timestamp = 0;
    float iaq = 0.0f;
    uint8_t iaq_accuracy = 0;
    float temp = 0.0f;
    float raw_temp = 0.0f;
    float raw_pressure = 0.0f;
    float humidity = 0.0f;
    float raw_humidity = 0.0f;
    float raw_gas = 0.0f;
    float static_iaq = 0.0f;
    uint8_t static_iaq_accuracy = 0;
    float co2_equivalent = 0.0f;
    uint8_t co2_accuracy = 0;
    float breath_voc_equivalent = 0.0f;
    uint8_t breath_voc_accuracy = 0;
    float comp_gas_value = 0.0f;
    uint8_t comp_gas_accuracy = 0;
    float gas_percentage = 0.0f;
    uint8_t gas_percentage_acccuracy = 0;
    
    /* Check if something should be processed by BSEC */
    if (num_bsec_inputs > 0)
    {
        /* Set number of outputs to the size of the allocated buffer */
        /* BSEC_NUMBER_OUTPUTS to be defined */
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        
        /* Perform processing of the data by BSEC 
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme680_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps(bsec_inputs, num_bsec_inputs, bsec_outputs, &num_bsec_outputs);
        
        /* Iterate through the outputs and extract the relevant ones. */
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                case BSEC_OUTPUT_IAQ:
                    iaq = bsec_outputs[index].signal;
                    iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_STATIC_IAQ:
                    static_iaq = bsec_outputs[index].signal;
                    static_iaq_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_CO2_EQUIVALENT:
                    co2_equivalent = bsec_outputs[index].signal;
                    co2_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                    breath_voc_equivalent = bsec_outputs[index].signal;
                    breath_voc_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    temp = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    raw_pressure = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    raw_gas = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_TEMPERATURE:
                    raw_temp = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    raw_humidity = bsec_outputs[index].signal;
                    break;
                case BSEC_OUTPUT_COMPENSATED_GAS:
                    comp_gas_value = bsec_outputs[index].signal;
                    comp_gas_accuracy = bsec_outputs[index].accuracy;
                    break;
                case BSEC_OUTPUT_GAS_PERCENTAGE:
                    gas_percentage = bsec_outputs[index].signal;
                    gas_percentage_acccuracy = bsec_outputs[index].accuracy;
                    break;
                default:
                    continue;
            }
            
            /* Assume that all the returned timestamps are the same */
            timestamp = bsec_outputs[index].time_stamp;
        }
        
        /* Pass the extracted outputs to the user provided output_ready() function. */
        output_ready(timestamp, iaq, iaq_accuracy, temp, humidity, raw_pressure, raw_temp, 
            raw_humidity, raw_gas, bsec_status, static_iaq, co2_equivalent, breath_voc_equivalent);
    }
}

/*!
 * @brief       Timer controlled version of endless loop (see original in examples)
 *
 */

// Define the timers needed to control this code.
//
/**< Handler for single shot timer used to control BSEC algorithm. */
APP_TIMER_DEF(m_timer_bsec_control);          
/**< Handler for single shot timer used to wait until the BME680 power modes is BME680_SLEEP_MODE. */
APP_TIMER_DEF(m_timer_bsec_wait_for_sleep);  
/**< Handler for single shot timer used to Read and Process the BMI680 data. */
APP_TIMER_DEF(m_timer_bsec_read_process);     
             


void bsec_iot_setup(void * p_context_void){

    bsec_context_t *p_context = p_context_void;
    ret_code_t err_code;
    uint16_t delay_to_start_ms = 10000;

#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("In bsec_iot_setup()");
    NRF_LOG_FLUSH();
#endif


    // Start the app_timer with the first entrant as handler
    app_timer_create(&m_timer_bsec_control,           APP_TIMER_MODE_SINGLE_SHOT, bsec_iot_sensor_control_handler);
    app_timer_create(&m_timer_bsec_wait_for_sleep,    APP_TIMER_MODE_SINGLE_SHOT, bsec_iot_wait_for_sleep_handler);
    app_timer_create(&m_timer_bsec_read_process,      APP_TIMER_MODE_SINGLE_SHOT, bsec_iot_read_process_handler);

#if NBL_LOG_BSEC == 1          
    NRF_LOG_DEBUG("Sleep for %d ms", delay_to_start_ms);
    NRF_LOG_FLUSH();
#endif 

    // Set the first control handler to be entered in 10 seconds.
    err_code = app_timer_start(m_timer_bsec_control,APP_TIMER_TICKS(delay_to_start_ms),p_context);
    APP_ERROR_CHECK(err_code);
};

void bsec_iot_sensor_control_handler(void * p_context_void){
  bsec_context_t *p_context = p_context_void;
  ret_code_t err_code;

  uint16_t meas_period;

#if NBL_LOG_BSEC == 1
  NRF_LOG_DEBUG("In bsec_iot_sensor_control_handler()");
  NRF_LOG_FLUSH();
#endif

  /* get the timestamp in nanoseconds before calling bsec_sensor_control() */ 
  p_context->time_stamp = p_context->get_timestamp_us() * 1000;

  /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
  bsec_sensor_control(p_context->time_stamp, &(p_context->sensor_settings));
  
  /* Trigger a measurement if necessary */
  bme680_bsec_trigger_measurement(&(p_context->sensor_settings), &meas_period);

  /* Delay till the measurement is ready. Timestamp resolution in ms */
  // TODO APP_TIMER_TICKS uses floating point arithmetic and is intended for constant arguments. 
  //      Replace with an integer calculation for performance. 
  if (meas_period) {
#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("Sleep for %d ms", meas_period);
    NRF_LOG_FLUSH();
#endif
    err_code = app_timer_start(m_timer_bsec_wait_for_sleep,APP_TIMER_TICKS(meas_period),p_context);
    APP_ERROR_CHECK(err_code);
  } else {
    // Not allowed to sleep! Go straight to next function
#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("Not allowed to sleep... calling bsec_iot_read_process_handler()");
    NRF_LOG_FLUSH();
#endif
    bsec_iot_read_process_handler(p_context_void);
  }

}

void bsec_iot_wait_for_sleep_handler(void * p_context_void){

  bsec_context_t *p_context = p_context_void;
  int8_t bme680_status;
  ret_code_t err_code;

  /* Call the API to get current operation mode of the sensor */
  bme680_status = bme680_get_sensor_mode(&bme680_g);  
  /* When the measurement is completed and data is ready for reading, the sensor must be in BME680_SLEEP_MODE.
  * Read operation mode to check whether measurement is completely done and wait until the sensor is no more
  * in BME680_FORCED_MODE. */
  if (bme680_g.power_mode == BME680_FORCED_MODE)
  {
    err_code = app_timer_start(m_timer_bsec_wait_for_sleep,APP_TIMER_TICKS(5),p_context);
    APP_ERROR_CHECK(err_code);
  } else {
    err_code = app_timer_start(m_timer_bsec_read_process,APP_TIMER_TICKS(5),p_context);
    APP_ERROR_CHECK(err_code);
  }
}

void bsec_iot_read_process_handler(void * p_context_void){
  bsec_context_t *p_context = p_context_void;
  ret_code_t err_code;

#if NBL_LOG_BSEC == 1
  NRF_LOG_DEBUG("In bsec_iot_read_process_handler()\n");
  NRF_LOG_FLUSH();
#endif

  /* Timestamp variables */
  int64_t time_stamp_interval_ms = 0;
 
  
  /* Number of inputs to BSEC */
  uint8_t num_bsec_inputs = 0;
  
  /* Save state variables */
  uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
  uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
  uint32_t bsec_state_len = 0;
  uint32_t n_samples = 0;
  
  bsec_library_return_t bsec_status = BSEC_OK;


  /* Read data from last measurement */
  num_bsec_inputs = 0;
  bme680_bsec_read_data(p_context->time_stamp, p_context->bsec_inputs, &num_bsec_inputs, p_context->sensor_settings.process_data);
  
  /* Time to invoke BSEC to perform the actual processing */
  bme680_bsec_process_data(p_context->bsec_inputs, num_bsec_inputs, p_context->output_ready);
  
  /* Increment sample counter */
  n_samples++;
  
  /* Retrieve and store state if the passed save_intvl */
  if (n_samples >= p_context->save_intvl)
  {
      bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
      if (bsec_status == BSEC_OK)
      {
          p_context->state_save(bsec_state, bsec_state_len);
      }
      n_samples = 0;
  }

#if NBL_LOG_BSEC == 1
  NRF_LOG_DEBUG("Compute how long we can sleep.\n");
  NRF_LOG_FLUSH();
#endif

  /* Compute how long we can sleep until we need to call bsec_sensor_control() next */
  /* Time_stamp is converted from microseconds to nanoseconds first and then the difference to milliseconds */
  time_stamp_interval_ms = (p_context->sensor_settings.next_call - (p_context->get_timestamp_us() * 1000)) / 1000000;
  if (time_stamp_interval_ms > 0) {
#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("Sleep for %d ms", time_stamp_interval_ms);
    NRF_LOG_FLUSH();
#endif

    // TODO APP_TIMER_TICKS uses floating point arithmetic and is intended for constant arguments. 
    //      Replace with an integer calculation for performance. 
    err_code = app_timer_start(m_timer_bsec_control,APP_TIMER_TICKS(time_stamp_interval_ms),p_context);
    APP_ERROR_CHECK(err_code);
  } else {
#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("Not allowed to sleep... calling bsec_iot_sensor_control_handler()");
    NRF_LOG_FLUSH();
#endif
  }

}



#if BSEC_LOOP == 1
void bsec_iot_loop(void * p_context_void){

    bsec_context_t *p_context = p_context_void;

#if NBL_LOG_BSEC == 1
    NRF_LOG_DEBUG("In bsec_iot_loop()");
    NRF_LOG_FLUSH();
#endif
   
    /* Timestamp variables */
    int64_t time_stamp = 0;
    int64_t time_stamp_interval_ms = 0;
    
    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    
    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;
    
    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
    
    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    
    bsec_library_return_t bsec_status = BSEC_OK;

    while (1)
    {

        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = p_context->get_timestamp_us() * 1000;

        /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
        bsec_sensor_control(time_stamp, &sensor_settings);
        
        /* Trigger a measurement if necessary */
        uint16_t sleep_duration;
        bme680_bsec_trigger_measurement(&sensor_settings, &sleep_duration);
        p_context->sleep(sleep_duration);
        
        /* Read data from last measurement */
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);
        
        /* Time to invoke BSEC to perform the actual processing */
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, p_context->output_ready);
        
        /* Increment sample counter */
        n_samples++;
        
        /* Retrieve and store state if the passed save_intvl */
        if (n_samples >= p_context->save_intvl)
        {
            bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
            if (bsec_status == BSEC_OK)
            {
                p_context->state_save(bsec_state, bsec_state_len);
            }
            n_samples = 0;
        }
        
        
        /* Compute how long we can sleep until we need to call bsec_sensor_control() next */
        /* Time_stamp is converted from microseconds to nanoseconds first and then the difference to milliseconds */
        time_stamp_interval_ms = (sensor_settings.next_call - ((bsec_context_t*) p_context)->get_timestamp_us() * 1000) / 1000000;
        if (time_stamp_interval_ms > 0)
        {
            ((bsec_context_t*) p_context)->sleep((uint32_t)time_stamp_interval_ms);
        }
    }
}
#endif

char* bsec_library_return2string (bsec_library_return_t return_value) {
  switch(return_value) {
  
  case (BSEC_OK ) : return "Function execution successful ";
  case (BSEC_E_DOSTEPS_INVALIDINPUT  ) : return "Input (physical) sensor id passed to bsec_do_steps() is not in the valid range or not valid for requested virtual sensor ";
  case (BSEC_E_DOSTEPS_VALUELIMITS ) : return "Value of input (physical) sensor signal passed to bsec_do_steps() is not in the valid range ";
  case (BSEC_E_DOSTEPS_DUPLICATEINPUT  ) : return "Duplicate input (physical) sensor ids passed as input to bsec_do_steps() ";
  case (BSEC_I_DOSTEPS_NOOUTPUTSRETURNABLE ) : return "No memory allocated to hold return values from bsec_do_steps(), i.e., n_outputs == 0 ";
  case (BSEC_W_DOSTEPS_EXCESSOUTPUTS ) : return "Not enough memory allocated to hold return values from bsec_do_steps(), i.e., n_outputs < maximum number of requested output (virtual) sensors ";
  case (BSEC_W_DOSTEPS_TSINTRADIFFOUTOFRANGE ) : return "Duplicate timestamps passed to bsec_do_steps() ";
  case (BSEC_E_SU_WRONGDATARATE ) : return "The sample_rate of the requested output (virtual) sensor passed to bsec_update_subscription() is zero ";
  case (BSEC_E_SU_SAMPLERATELIMITS ) : return "The sample_rate of the requested output (virtual) sensor passed to bsec_update_subscription() does not match with the sampling rate allowed for that sensor ";
  case (BSEC_E_SU_DUPLICATEGATE ) : return "Duplicate output (virtual) sensor ids requested through bsec_update_subscription() ";
  case (BSEC_E_SU_INVALIDSAMPLERATE ) : return "The sample_rate of the requested output (virtual) sensor passed to bsec_update_subscription() does not fall within the global minimum and maximum sampling rates  ";
  case (BSEC_E_SU_GATECOUNTEXCEEDSARRAY ) : return "Not enough memory allocated to hold returned input (physical) sensor data from bsec_update_subscription(), i.e., n_required_sensor_settings < #BSEC_MAX_PHYSICAL_SENSOR ";
  case (BSEC_E_SU_SAMPLINTVLINTEGERMULT ) : return "The sample_rate of the requested output (virtual) sensor passed to bsec_update_subscription() is not correct ";
  case (BSEC_E_SU_MULTGASSAMPLINTVL ) : return "The sample_rate of the requested output (virtual), which requires the gas sensor, is not equal to the sample_rate that the gas sensor is being operated ";
  case (BSEC_E_SU_HIGHHEATERONDURATION ) : return "The duration of one measurement is longer than the requested sampling interval ";
  case (BSEC_W_SU_UNKNOWNOUTPUTGATE ) : return "Output (virtual) sensor id passed to bsec_update_subscription() is not in the valid range; e.g., n_requested_virtual_sensors > actual number of output (virtual) sensors requested ";
  case (BSEC_W_SU_MODINNOULP ) : return "ULP plus can not be requested in non-ulp mode ";
  case (BSEC_I_SU_SUBSCRIBEDOUTPUTGATES ) : return "No output (virtual) sensor data were requested via bsec_update_subscription() ";
  case (BSEC_E_PARSE_SECTIONEXCEEDSWORKBUFFER ) : return "n_work_buffer_size passed to bsec_set_[configuration/state]() not sufficient ";
  case (BSEC_E_CONFIG_FAIL ) : return "Configuration failed ";
  case (BSEC_E_CONFIG_VERSIONMISMATCH ) : return "Version encoded in serialized_[settings/state] passed to bsec_set_[configuration/state]() does not match with current version ";
  case (BSEC_E_CONFIG_FEATUREMISMATCH ) : return "Enabled features encoded in serialized_[settings/state] passed to bsec_set_[configuration/state]() does not match with current library implementation ";
  case (BSEC_E_CONFIG_CRCMISMATCH ) : return "serialized_[settings/state] passed to bsec_set_[configuration/state]() is corrupted ";
  case (BSEC_E_CONFIG_EMPTY ) : return "n_serialized_[settings/state] passed to bsec_set_[configuration/state]() is to short to be valid ";
  case (BSEC_E_CONFIG_INSUFFICIENTWORKBUFFER) : return "Provided work_buffer is not large enough to hold the desired string ";
  case (BSEC_E_CONFIG_INVALIDSTRINGSIZE) : return "String size encoded in configuration/state strings passed to bsec_set_[configuration/state]() does not match with the actual string size n_serialized_[settings/state] passed to these functions ";
  case (BSEC_E_CONFIG_INSUFFICIENTBUFFER) : return "String buffer insufficient to hold serialized data from BSEC library ";
  case (BSEC_E_SET_INVALIDCHANNELIDENTIFIER) : return "Internal error code, size of work buffer in setConfig must be set to BSEC_MAX_WORKBUFFER_SIZE ";
  case (BSEC_E_SET_INVALIDLENGTH) : return "Internal error code ";
  case (BSEC_W_SC_CALL_TIMING_VIOLATION) : return "Difference between actual and defined sampling intervals of bsec_sensor_control() greater than allowed ";
  case (BSEC_W_SC_MODEXCEEDULPTIMELIMIT) : return "ULP plus is not allowed because an ULP measurement just took or will take place ";
  case (BSEC_W_SC_MODINSUFFICIENTWAITTIME) : return "ULP plus is not allowed because not sufficient time passed since last ULP plus ";
  default : return "Unknown Value for bsec library return type";
  }
}

/*! @}*/

