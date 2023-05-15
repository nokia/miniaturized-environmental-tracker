# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

import sys,traceback
import numpy as np
import time
import config as gv
import datetime


def genericTx(dev_address,commandCode,argument):
    dev_name = dev_address[dev_address.find('dev_')+4:]
    
    # Construct Data Packet
    datapacket = bytearray()
    datapacket.append(commandCode)
    datapacket.append(argument)
    
    loop_count = 0
    while loop_count < 20:
        try:
            # Turn off LED
            gv.geckos[dev_name].Tx.WriteValue(bytes(datapacket),{"type":"request"})
            print("Parameter Updated")
            return 1
        except:
            loop_count += 1
            
    print("Write to Gecko Failed")
    return 0



def setTimebase(dev_address):
    dev_name = dev_address[dev_address.find('dev_')+4:]
    # Calculate new Timestamp for Gecko
    t = datetime.datetime.now()
    synct = np.int32((t-datetime.datetime(1970,1,1)).total_seconds())
    synct1 = (synct >> 0) & 0xff
    synct2 = (synct >> 8) & 0xff
    synct3 = (synct >> 16) & 0xff
    synct4 = (synct >> 24) & 0xff
    datapacket = bytearray()
    datapacket.append(0xff)
    datapacket.append(synct1)
    datapacket.append(synct2)
    datapacket.append(synct3)
    datapacket.append(synct4)
    datapacket.extend("\r".encode('ascii'))

    # Write New Timestamp
    gv.geckos[dev_name].Tx.WriteValue(bytes(datapacket),{"type":"request"})
                   
    print("Timebase syncronized")
    
def LEDoff(dev_address):
    dev_name = dev_address[dev_address.find('dev_')+4:]
    
    # Construct Data Packet
    datapacket = bytearray()
    datapacket.append(0x1)
    datapacket.append(0x0)
    
    loop_count = 0
    while loop_count < 20:
        try:
            # Turn off LED
            gv.geckos[dev_name].Tx.WriteValue(bytes(datapacket),{"type":"request"})
            print("LED Off")
            return 1
        except:
            loop_count += 1
            
    print("Write to Gecko Failed")
    return 0
    
def LEDon(dev_address):
    dev_name = dev_address[dev_address.find('dev_')+4:]
    
    # Construct Data Packet
    datapacket = bytearray()
    datapacket.append(0x1)
    datapacket.append(0x1)
    
    loop_count = 0
    while loop_count < 20:
        try:
            # Turn off LED
            gv.geckos[dev_name].Tx.WriteValue(bytes(datapacket),{"type":"request"})
            print("LED On")
            return 1
        except:
            loop_count += 1
            
    print("Write to Gecko Failed")
    return 0