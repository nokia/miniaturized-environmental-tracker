/*
 * Colyright 2018-2022 Nokia
 *
 * All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
 * Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).
 */

#ifndef NBL_BSEC_IF_H
#define NBL_BSEC_IF_H
#include "sdk_config.h"
#include "BSEC/include/bsec_datatypes.h"
#include "BSEC/include/bsec_interface.h"
#include "BSEC/gecko_integration/bsec_integration.h"

#include "I2C.h"
#include "nrf_pwr_mgmt.h"
#include "nordic_common.h" 

// Functions defined in main.c
int64_t get_timestamp_us(void);

// Functions defined in BSEC_if.c
//int BSEC_Configure_and_Run(void (*callback)(t_bsec_data* p_data));

int BSEC_Configure_and_Run(bsec_callback_fn bsec_callback);

#endif