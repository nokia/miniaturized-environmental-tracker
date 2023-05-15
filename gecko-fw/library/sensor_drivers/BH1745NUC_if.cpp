// Colyright 2018-2022 Nokia
//
// All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
// Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

#include "BH1745NUC_if.h"
#include "BH1745NUC/BH1745NUC.h"


static BH1745NUC bh1745(BH1745NUC_DEVICE_ADDRESS_38);
;
void    BH1745NUC_Turn_On( void ) { 

#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Turning on BH1745NUC");
#endif
    bh1745.init();
}


void    BH1745NUC_Get_Data(uint8_t * dest) {  
    bh1745.get_val((uint16_t*) dest);
    return; 
}


