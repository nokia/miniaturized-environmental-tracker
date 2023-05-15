/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

#include "BME680_if.h"

struct bme680_dev gas_sensor;

void    BME680_Turn_On( void ) { 
    int8_t rslt = BME680_OK;

#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Turning on BME680");
#endif

    gas_sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
    gas_sensor.intf = BME680_I2C_INTF;
    gas_sensor.read = (bme680_com_fptr_t) I2C_Read;
;
    gas_sensor.write = (bme680_com_fptr_t) I2C_Write;
;
    gas_sensor.delay_ms = (bme680_delay_fptr_t) nrf_delay_ms;
    /* amb_temp can be set to 25 prior to configuring the gas sensor 
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor.amb_temp = 25;


    rslt = bme680_init(&gas_sensor);

    APP_ERROR_CHECK(rslt);
    
    BME680_Configure();
}

int8_t  BME680_Configure( void ) {
    int8_t rslt = BME680_OK;

    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);
    APP_ERROR_CHECK(rslt);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
    APP_ERROR_CHECK(rslt);

    return 0;
}


void    BME680_Get_Data(int32_t * dest) { 
                
    static uint8_t exec_count = 0;
    static uint32_t dead_cycles = 0;

    exec_count++;

    int8_t rslt = BME680_OK;
            
    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);

    //SEGGER_RTT_printf(0,"meas_period: %d\n", meas_period);

    struct bme680_field_data data;

    gas_sensor.delay_ms(meas_period); /* Delay till the measurement is ready */

    rslt = bme680_get_sensor_data(&data, &gas_sensor); 

    if (rslt != BME680_W_NO_NEW_DATA) {
      // Check if there was another error?
      APP_ERROR_CHECK(rslt);

      // Clear the dead cycles counter
      dead_cycles = 0;

      SEGGER_RTT_printf(0,"T: %d degC, P: %d hPa, H %d ", data.temperature / 100,
          data.pressure / 100, data.humidity / 1000 );
    
      /* Avoid using measurements from an unstable heating setup */
      if(data.status & BME680_GASM_VALID_MSK)
        SEGGER_RTT_printf(0,", G: %d ohms", data.gas_resistance);

      SEGGER_RTT_printf(0,"\r\n");

      /* Trigger the next measurement if you would like to read data out continuously */
      if (gas_sensor.power_mode == BME680_FORCED_MODE) {
          rslt = bme680_set_sensor_mode(&gas_sensor);
          APP_ERROR_CHECK(rslt);
      } else {
        dead_cycles++;
        if (0==(dead_cycles%100)) {
                SEGGER_RTT_printf(0,"%d cycles with no new data from BME680\n");

        }
      }

    if (data.status & BME680_NEW_DATA_MSK) { 
      dest[0] = data.pressure;
      dest[1] = data.temperature;
      dest[2] = data.humidity;
      dest[3] = data.gas_resistance;
    }
    return; 
   }
}


uint8_t BME680_SelfTest(void) { return 0; }

