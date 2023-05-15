/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */


#ifndef MA_ADC_H__
#define MA_ADC_H__

#define BATTERY_AVERAGE_PERIOD 5

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*adc_callback_fn_t)(float vbatt);

    #include "nrfx_saadc.h"
    #include "boards.h"
//    void ADC_init(void);
    void ADC_init_with_callback(adc_callback_fn_t);
    void ADC_callback(nrfx_saadc_evt_t const * p_event);
    
#ifdef __cplusplus
}
#endif

#endif
