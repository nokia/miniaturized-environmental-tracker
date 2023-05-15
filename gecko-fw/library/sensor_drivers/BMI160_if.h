/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */


/* 
 * File:   BMI160.h
 * Author: derrick
 *
 * Created on 26 July 2018, 17:34
 */

#ifndef BMI160_IF_H
#define BMI160_IF_H

#ifdef __cplusplus
extern "C" {
#endif

    #include "I2C.h"
    #include "app_error.h"
    #include "nrf_delay.h"
    #include "SEGGER_RTT.h"
    #include "BMI160/bmi160.h"  
    #include "nrf_drv_twi.h"
    
    int8_t  BMI160_Configure( void );
    void    BMI160_Turn_On( void );
    void    BMI160_Get_Data(int16_t * dest);
    uint8_t BMI160_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_IF_H */

