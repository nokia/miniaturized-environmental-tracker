/*****************************************************************************
  BH1745NUC.h

 Copyright (c) 2018 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#ifndef _BH1745NUC_H_
#define _BH1745NUC_H_

#include "stdint.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"

#define BH1745NUC_DEVICE_ADDRESS_38             (0x38)    // 7bit Addrss
#define BH1745NUC_DEVICE_ADDRESS_39             (0x39)    // 7bit Addrss
#define BH1745NUC_PART_ID_VAL                   (0x0B)  
#define BH1745NUC_MANUFACT_ID_VAL               (0xE0)

#define BH1745NUC_SYSTEM_CONTROL                (0x40)
#define BH1745NUC_MODE_CONTROL1                 (0x41)
#define BH1745NUC_MODE_CONTROL2                 (0x42)
#define BH1745NUC_MODE_CONTROL3                 (0x44)
#define BH1745NUC_RED_DATA_LSB                  (0x50)
#define BH1745NUC_MANUFACTURER_ID               (0x92)

#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_160MS (0)
#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_320MS (1)
#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_640MS (2)
#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_1280MS (3)
#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_2560MS (4)
#define BH1745NUC_MODE_CONTROL1_MEAS_MODE_5120MS (5)


#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X1     (0)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X2     (1)
#define BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16    (2)
#define BH1745NUC_MODE_CONTROL2_RGBC_EN         (1 << 4)
#define BH1745NUC_MODE_CONTROL2_VALID_MSK       (1 << 7)

#define BH1745NUC_MODE_CONTROL1_VAL             (BH1745NUC_MODE_CONTROL1_MEAS_MODE_160MS)
#define BH1745NUC_MODE_CONTROL2_VAL             (BH1745NUC_MODE_CONTROL2_ADC_GAIN_X16 | BH1745NUC_MODE_CONTROL2_RGBC_EN)
#define BH1745NUC_MODE_CONTROL3_VAL             (0)

#define GET_BYTE_RED_TO_CLEAR  (8)
#define WAIT_TMT2_MAX          (340)

class BH1745NUC
{
  public:
    BH1745NUC(int slave_address);
    ~BH1745NUC();
    uint8_t init(void) ;
    uint8_t get_rawval(unsigned char *data);
    uint8_t get_val(unsigned short *data);
    uint8_t write(unsigned char memory_address, unsigned char *data, unsigned char size);
    uint8_t read(unsigned char memory_address, unsigned char *data, int size);
  private:
      int _device_address;
};

#endif // _BH1745NUC_H_
