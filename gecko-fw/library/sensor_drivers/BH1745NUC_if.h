/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

/* 
 * File:   BME1745_if.h
 * Author: derrick
 *
 * Created on 27 Sept 2018
 */

#ifndef BH1745_IF_H
#define BH1745_IF_H

#ifdef __cplusplus
extern "C" {
#endif

    #include "I2C.h"
    #include "app_error.h"
    #include "nrf_delay.h"
    #include "SEGGER_RTT.h"
    #include "nrf_drv_twi.h"
    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
    
    void    BH1745NUC_Turn_On( void );
    void    BH1745NUC_Get_Data(uint8_t * dest);

#ifdef __cplusplus
}
#endif

#endif /* BH1745_IF_H */
