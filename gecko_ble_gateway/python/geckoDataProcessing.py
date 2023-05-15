# 	Colyright 2018-2022 Nokia
#
# 	All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# 	Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

import sys
from struct import *
import numpy as np
import time


AAAID_BLE_XXXX = "D1:58:42:5B:DD:E2"
AAAID_BLE_3903 = "D1:A0:45:B7:39:03"
AAAID_BLE_9080 = "FC:1B:E9:E5:90:80"

SERVICE_UUID =  ['00002a00-0000-1000-8000-00805f9b34fb',
                   "00002a01-0000-1000-8000-00805f9b34fb",  # Don't know what this is
                   "00002a04-0000-1000-8000-00805f9b34fb",
                   '00002aa6-0000-1000-8000-00805f9b34fb',
                   '6e400003-b5a3-f393-e0a9-e50e24dcca9e',  # Nordic UART RX
                   '6e400002-b5a3-f393-e0a9-e50e24dcca9e']  # Nordic UART TX

PacketHeaderSize = 4 + 4 + 4 + 1 + 1 + 2

MagicString = b"Bell"[::-1]
MagicNumber = unpack('I', MagicString)

ConfigLoggerID = 0x00
IMULoggerID = 0x01
EnvLoggerID = 0x02
LightSensorLoggerID = 0x03
AudioDataID = 0x4
BatteryLevelID = 0x5

DataPackingVersion = 0
PrescalerValue = 0
RTCInputFrequency = 32768
Ticks_per_second = RTCInputFrequency / (PrescalerValue + 1)*3600
timestamp = 0

#plt.close('all')
#%% Data Processing Functions

def align32(size):
    offset = size % 4
    if offset:
        size += 4 - offset

    return size


def process_config_data(SensorData):
    global PrescalerValue
    global RTCInputFrequency
    global DataPackingVersion

    # print("Config Data Handler")
    PrescalerValue = (SensorData[3] << 24 | SensorData[2] << 16 << SensorData[1] << 8 | SensorData[0])
    RTCInputFrequency = (SensorData[5] << 8 | SensorData[4])
    DataPackingVersion = (SensorData[7] << 8 | SensorData[6]);

#    print("Updating PrescalarValue to", PrescalerValue)
#    print("Updating RTC Input Frequency to", RTCInputFrequency)
#    print("Data Format Version ", DataPackingVersion)
#    print("Now Streaming from New Gecko")
    return


def process_imu_data(SensorData):
#    gx = int(SensorData[1] << 8 | SensorData[0])
#    gy = int(SensorData[3] << 8 | SensorData[2])
#    gz = int(SensorData[5] << 8 | SensorData[4])
#    ax = int(SensorData[7] << 8 | SensorData[6])
#    ay = int(SensorData[9] << 8 | SensorData[8])
#    az = int(SensorData[11] << 8 | SensorData[10])
    
    gx = 125*3.14159/(2**16*180)*int.from_bytes(SensorData[0:2],byteorder='little',signed=True)
    gy = 125*3.14159/(2**16*180)*int.from_bytes(SensorData[2:4],byteorder='little',signed=True)
    gz = 125*3.14159/(2**16*180)*int.from_bytes(SensorData[4:6],byteorder='little',signed=True) 
    ax = 9.80665/2**14*int.from_bytes(SensorData[6:8],byteorder='little',signed=True) 
    ay = 9.80665/2**14*int.from_bytes(SensorData[8:10],byteorder='little',signed=True) 
    az = 9.80665/2**14*int.from_bytes(SensorData[10:12],byteorder='little',signed=True)
    imu_internal_ts = (SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12])
    
    return {'type':'imu_ts','gx':gx,'gy':gy,'gz':gz,'ax':ax,'ay':ay,'az':az,'imu_internal_ts':imu_internal_ts}


def process_env_data(SensorData):
    pressure = (SensorData[3] << 24 | SensorData[2] << 16 | SensorData[1] << 8 | SensorData[0])/100
    temperature = (SensorData[7] << 24 | SensorData[6] << 16 | SensorData[5] << 8 | SensorData[4])/100
    humidity = (SensorData[11] << 24 | SensorData[10] << 16 | SensorData[9] << 8 | SensorData[8])/1000
    gas_resistance = (SensorData[15] << 24 | SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12])

    return {'type':'env_ts','pressure':pressure,'temperature':temperature,'humidity':humidity,'gas_resistance':gas_resistance}


def process_lightsensor_data(SensorData):
    red = (SensorData[1] << 8 | SensorData[0])    # red
    green = (SensorData[3] << 8 | SensorData[2])  # green
    blue = (SensorData[5] << 8 | SensorData[4])   # blue
    clear = (SensorData[7] << 8 | SensorData[6])  # clear

    return {'type':'light_ts','red':red,'green':green,'blue':blue,'clear':clear}


def process_audio_data(SensorData):
    audio_mean = (SensorData[1] << 8 | SensorData[0])
    audio_peak = (SensorData[3] << 8 | SensorData[2])

    return {'type':'audio_ts','audio_mean':audio_mean,'audio_peak':audio_peak}


def process_battery_data(SensorData):
    battery_percent = (SensorData[0])
    if (DataPackingVersion>=1):
        battery_volt = 3.0 + SensorData[1]/200.0;
    else:
        battery_volt = (SensorData[1]+300.0)/100.0;
    
    battery_level = battery_volt

    return {'type':'battery_ts','battery_percent':battery_percent,'battery_level':battery_level}


def process(logger_id, SensorData):
    switcher = {
        ConfigLoggerID: process_config_data,
        IMULoggerID: process_imu_data,
        EnvLoggerID: process_env_data,
        LightSensorLoggerID: process_lightsensor_data,
        AudioDataID: process_audio_data,
        BatteryLevelID: process_battery_data
    }
    func = switcher.get(logger_id, lambda: "Invalid sensor")
    return func(SensorData)


def to_hex(x, pos):
    return '%x' % int(x)    