/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

/* 
 * File:   LPS22HB.h
 * Author: derrick
 *
 * Created on 26 July 2018, 17:34
 */

#ifndef LPS22HB_IF_H
#define LPS22HB_IF_H

// LPS22HB Driver defines I2C Address as 8-bit which is incorrect. 
// Fix that here. 
// SA0 is GND so use LPS22HB_I2C_ADD_L
#define LPS22HB_I2C_ADDR LPS22HB_I2C_ADD_L >> 1

#ifdef __cplusplus
extern "C" {
#endif

    #include "I2C.h"
    #include "app_error.h"
    #include "nrf_delay.h"
    #include "SEGGER_RTT.h"
    #include "LPS22HB/lps22hb_STdC/driver/lps22hb_reg.h"  
    #include "nrf_drv_twi.h"
    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
    
    int8_t  LPS22HB_Configure( void );
    void    LPS22HB_Turn_On( void );
    void    LPS22HB_Get_Data(uint16_t * dest);
    uint8_t LPS22HB_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* LPS22HB_IF_H */
