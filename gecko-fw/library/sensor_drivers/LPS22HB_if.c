/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

#include "LPS22HB_if.h"

lps22hb_ctx_t dev_ctx;

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. Unused here (void)
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  return I2C_Write(LPS22HB_I2C_ADDR, reg, bufp, len);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. Unused here (void)
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  return I2C_Read(LPS22HB_I2C_ADDR, reg, bufp, len);
}


void    LPS22HB_Turn_On( void ) { 

#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Turning on LPS22HB");
#endif

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;
  uint8_t retval;

  /* Check device ID */
  lps22hb_device_id_get(&dev_ctx, &retval);
  if (retval != LPS22HB_ID){
    SEGGER_RTT_printf(0, "Cannot find LPS22HB Device at address 0x%08x\n", LPS22HB_I2C_ADDR);
  } 
  
  LPS22HB_Configure();
}

int8_t  LPS22HB_Configure( void ) {
  uint8_t rst;

  /*
   *  Restore default configuration
   */
  lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lps22hb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   *  Enable Block Data Update
   */
  lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set Output Data Rate to 0
   */
  lps22hb_data_rate_set(&dev_ctx, LPS22HB_POWER_DOWN);

  return 0;
}

// Bug in lps22hb_reg driver. The lps22hb_from_lsb_to_hpa takes 
// a 16 bit variable but the pressure is 32 (actually 24) bits. 

float_t lps22hb_from_lsb_to_hpa_fixed(int32_t lsb)
{

  //return ( (float_t)lsb / 4096.0f );
  return ( (float_t) ((uint32_t) lsb & 0x00ffffffU)/ 4096.0f );

}

void    LPS22HB_Get_Data(uint16_t * dest) { 

  uint8_t reg;
  static axis1bit32_t data_raw_pressure;
  static axis1bit16_t data_raw_temperature;
  static float temperature_degC;


  /*
   * Trigger one shot data acquisition
   */
  lps22hb_one_shoot_trigger_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Wait data ready
   */
  lps22hb_press_data_ready_get(&dev_ctx, &reg);

  while (reg==0) {
    lps22hb_press_data_ready_get(&dev_ctx, &reg);
  }

  if (reg)
  {
    memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
    lps22hb_pressure_raw_get(&dev_ctx, data_raw_pressure.u8bit);

    // Pressure is 24 bits but 12 bits are RHS of fixed point. We do not need 
    // that much detail so compress it here. 
    // 
    // Compression: - 
    //  Lowest ever recorded pressure was 870 hPa (Pacific Ocean 12 Oct 1979)
    //  Highest ever recorded pressure was 1083.8 hPa (Agate, Siberia, 31 Dec 1968)
    // 
    // So coded pressure only needs to store range approx 850 to 1100 = c.250
    // -> Use 8 bits for integer part of pressure giving 850 to 1105.
    // This leaves 8 bits for the fractional part giving a resolution of 1/256 hPa
    // 
    // Data fetched from LPS22HB has the following format: - 
    //  23                12 11           0
    //  -----------------------------------
    // |  Integer           |   Fractional |
    // |  12-bits           |    12-bits   |
    //  -----------------------------------
    // 
    // Transmitted format is as follows 
    //  15                8 7           0
    //  -----------------------------------
    // |  Integer - 850    |   Fractional |
    // |     8-bits        |   8-bits    |
    //  -----------------------------------
    // 
    // So the integer part (for transmission) is : - 
    uint8_t pressure16_int = ((uint32_t) data_raw_pressure.i32bit >> 12) - 850;

    // And the fractional part is simply: - 
    uint8_t pressure16_frct = (data_raw_pressure.i32bit >> 4) & 0xff;

    // Put this together
    *dest = (pressure16_int << 8) + pressure16_frct;

    // finally, correct for rounding (test bit 3): - 
    if (0 != (data_raw_pressure.i32bit & (1 << 3))) {
      (*dest)++;
    }

    // For local printing only.
    float  pressure_16_hPa = (float) 850.0 + (*dest / 256.0);
    uint32_t pressure_16_int = (int) pressure_16_hPa;
    // Save two decimal points. 
    uint8_t  pressure_16_2dp = (int) ((float) 100.0 * (pressure_16_hPa - (float) pressure_16_int));    
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0, "pressure [hPa]:%d.%02d\n", pressure_16_int, pressure_16_2dp );
#endif
  } else {

#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0, "ERROR - LPS22HB not ready for Pressure Reading");
    ASSERT(0);
 #endif
  }

  lps22hb_temp_data_ready_get(&dev_ctx, &reg);

  while (reg==0) {
    lps22hb_press_data_ready_get(&dev_ctx, &reg);
  }

  if (reg)
  {
    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    lps22hb_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);

    // Temperature is degC*100. Can send just as is.
    *(dest+1) = data_raw_temperature.i16bit;
    

    // For local printing only.
    temperature_degC = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);

    uint32_t temperature_degC_int = (int) temperature_degC;
    uint8_t  temperature_degC_2dp = (int) ((float) 100.0 * (temperature_degC - (float) temperature_degC_int));
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0, "Temperature [degC]:%d.%d\r\n", temperature_degC_int,temperature_degC_2dp);
#endif
  } else {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0, "WARNING - LPS22HB not ready for Temperature Reading");
    ASSERT(0);
#endif
  }
         
}


uint8_t LPS22HB_SelfTest(void) { return 0; }

