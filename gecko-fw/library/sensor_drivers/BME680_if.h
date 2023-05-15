/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

/* 
 * File:   BME680.h
 * Author: derrick
 *
 * Created on 26 July 2018, 17:34
 */

#ifndef BME680_IF_H
#define BME680_IF_H

#ifdef __cplusplus
extern "C" {
#endif

    #include "I2C.h"
    #include "app_error.h"
    #include "nrf_delay.h"
    #include "SEGGER_RTT.h"
    #include "BME680/bme680.h"  
    #include "nrf_drv_twi.h"
    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
    
    int8_t  BME680_Configure( void );
    void    BME680_Turn_On( void );
    void    BME680_Get_Data(int32_t * dest);
    uint8_t BME680_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* BME680_IF_H */
