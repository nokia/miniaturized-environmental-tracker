/*****************************************************************************

  !!!!  NOTE  !!!! 

  This File, BH1745NUC.cpp, is based on the newer BH1749NUC Arduino driver. 

  Original header is below:

  *****************************************************************************

  BH1749NUC.cpp

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
//#include <avr/pgmspace.h>
#include "BH1745NUC.h"
#include "I2C.h"

BH1745NUC::BH1745NUC(int slave_address)
{
  _device_address = slave_address;
}

BH1745NUC::~BH1745NUC()
{
  _device_address = 0;
}

uint8_t BH1745NUC::init(void)
{
  uint8_t rc;
  unsigned char reg;

  rc = read(BH1745NUC_SYSTEM_CONTROL, &reg, sizeof(reg));
  if (rc != 0) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't access BH1745NUC\n");
#endif
    return (rc);
  }
  reg = reg & 0x3F;
#if NBL_LOG_BOARD == 1
  SEGGER_RTT_printf(0,"BH1745NUC Part ID Value = 0x%02x\n", reg);
#endif


  if (reg != BH1745NUC_PART_ID_VAL) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't find BH1745NUC\n");
#endif
    return (rc);
  }

  rc = read(BH1745NUC_MANUFACTURER_ID, &reg, sizeof(reg));
  if (rc != 0) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't access BH1745NUC\n");
#endif
    return (rc);
  }
#if NBL_LOG_BOARD == 1
  SEGGER_RTT_printf(0,"BH1745NUC MANUFACTURER ID Register Value = 0x%02x\n", reg);
#endif

  if (reg != BH1745NUC_MANUFACT_ID_VAL) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't find BH1745NUC\n");
#endif
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL1_VAL;
  rc = write(BH1745NUC_MODE_CONTROL1, &reg, sizeof(reg));
  if (rc != 0) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't write BH1745NUC MODE_CONTROL1 register\n");
#endif
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL2_VAL;
  rc = write(BH1745NUC_MODE_CONTROL2, &reg, sizeof(reg));
  if (rc != 0) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't write BH1745NUC MODE_CONTROL2 register\n");
#endif
    return (rc);
  }

  reg = BH1745NUC_MODE_CONTROL3_VAL;
  rc = write(BH1745NUC_MODE_CONTROL3, &reg, sizeof(reg));
  if (rc != 0) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Can't write BH1745NUC MODE_CONTROL3 register\n");
#endif
    return (rc);
  }

  nrf_delay_ms(WAIT_TMT2_MAX);

  return (rc);
}

uint8_t BH1745NUC::get_rawval(unsigned char *data)
{
  uint8_t rc;
  uint8_t control2;

  // TODO: Power down device between readings by clearing BH1745NUC_MODE_CONTROL2_RGBC_EN
  //       Power it back up 2ms before reading, thus would require changes to the timer.

  rc = read(BH1745NUC_MODE_CONTROL2, &control2, 1);
#if NBL_LOG_BOARD == 1
  if (rc != 0) {
    SEGGER_RTT_printf(0,"Can't get BH1745NUC_MODE_CONTROL2 value\n");
  }
#endif

  if (0 == (control2 & BH1745NUC_MODE_CONTROL2_VALID_MSK)) {
#if NBL_LOG_BOARD == 1
    SEGGER_RTT_printf(0,"Attempt to read RGB Values before ready\n");
#endif
//    ASSERT(0);
  }

  rc = read(BH1745NUC_RED_DATA_LSB, data, GET_BYTE_RED_TO_CLEAR);
#if NBL_LOG_BOARD == 1
  if (rc != 0) {
    SEGGER_RTT_printf(0,"Can't get BH1745NUC RGB and CLEAR values\n");
  }
#endif

  return (rc);
}

uint8_t BH1745NUC::get_val(unsigned short *data)
{
  uint8_t rc;
  unsigned char val[GET_BYTE_RED_TO_CLEAR];

  rc = get_rawval(val);
  if (rc != 0) {
    return (rc);
  }

  //val[6] and val[7] are RESERVED Register Value
  data[0] = ((unsigned short)val[1] << 8) | val[0]; // RED 
  data[1] = ((unsigned short)val[3] << 8) | val[2]; // GREEN
  data[2] = ((unsigned short)val[5] << 8) | val[4]; // BLUE
  data[3] = ((unsigned short)val[7] << 8) | val[6]; // CLEAR

  return (rc);
}

uint8_t BH1745NUC::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  uint8_t rc;
  rc=I2C_Write(_device_address, memory_address, data, size);
  return (rc);
}

uint8_t BH1745NUC::read(unsigned char memory_address, unsigned char *data, int size)
{
  uint8_t rc;
  rc = I2C_Read(_device_address, memory_address, data, size);
  APP_ERROR_CHECK(rc);

  return (0);
}
