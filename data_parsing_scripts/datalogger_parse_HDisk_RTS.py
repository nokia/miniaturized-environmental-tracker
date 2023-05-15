# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

import sys
from struct import *
from pathlib import Path
import glob

from matplotlib import pyplot as plt
from matplotlib import ticker as ticker
import numpy as np

from datetime import datetime

stopat = 10
count = 1

# Sensor Packets
# Format is: -
#
#   typedef struct {
#       packet_info_t   packet_info;
#       uint8_t         data[LOGGER_YYZ_DATA_LENGTH];
#   } xyz_data_t;
#
#
# Where Packet header format is
#
#   #define SOP_BELLLABS 0x42656c6c4c616273
#   typedef struct {
#       uint32_t  sop;
#       uint32_t  timestamp_hi;
#       uint32_t  timestamp_lo;
#       uint8_t   logger_id;
#       uint8_t   length;
#       uint16_t  custom;
#   } packet_info_t;

#                  sop  ts   id len cus
PacketHeaderSize =  4 +  8 + 1 + 1 + 2

MagicString = b"Bell"[::-1]
MagicNumber = unpack('I', MagicString)

ConfigLoggerID: int = 0x00
IMULoggerID: int = 0x01
EnvLoggerID: int = 0x02
LightSensorLoggerID: int = 0x03
AudioDataID: int = 0x4
BatteryLevelID: int = 0x5
ID_OUT_OF_RANGE: int = 0x6

# Later versions of datalogger change the format
# e.g. coding of battery voltage is different.
# Select with version is used here.
DataPackingVersion: int = 0

PrescalerValue = 0
RTCInputFrequency = 32768
Ticks_per_second: float = RTCInputFrequency / (PrescalerValue + 1)
TimescaleOffset: int = 0
OldTimescaleOffset: int = 0

#Number of digis after second decimal point, 3: ms, 6: us
TimeSavingPrecision=3

# IMU Data Arrays
gx = []
gy = []
gz = []
ax = []
ay = []
az = []
imu_internal_ts: int = []
imu_ts: int = []

# ENV Data Arrays
pressure: int = []
temperature: int = []
humidity: int = []
gas_resistance: int = []
gas_iaq: int = []
env_ts: int = []
lps22_pressure: int = []
lps22_temperature: int = []

# LightSensor Data Arrays
red: int = []
green: int = []
blue: int = []
clear: int = []
lightsensor_ts: int = []

# Audio Sensor Data
audio_mean: int = []
audio_peak: int = []
audio_ts: int = []

# Battery Level
battery_percent: int = []
battery_level: float = []
battery_ts: int = []

def clear_data():
    # IMU Data Arrays
    global gx
    global gy
    global gz
    global ax
    global ay
    global az
    global imu_internal_ts
    global imu_ts

    # ENV Data Arrays
    global pressure
    global temperature
    global humidity
    global gas_resistance
    global gas_iaq
    global env_ts
    global lps22_pressure
    global lps22_temperature

    # LightSensor Data Arrays
    global red
    global green
    global blue
    global clear
    global lightsensor_ts

    # Audio Sensor Data
    global audio_mean
    global audio_peak
    global audio_ts

    # Battery Level
    global battery_percent
    global battery_level
    global battery_ts

    # IMU Data Arrays
    gx = []
    gy = []
    gz = []
    ax = []
    ay = []
    az = []
    imu_internal_ts = []
    imu_ts = []

    # ENV Data Arrays
    pressure  = []
    temperature = []
    humidity = []
    gas_resistance = []
    gas_iaq = []
    env_ts = []
    lps22_pressure = []
    lps22_temperature = []

    # LightSensor Data Arrays
    red = []
    green = []
    blue = []
    clear = []
    lightsensor_ts = []

    # Audio Sensor Data
    audio_mean = []
    audio_peak = []
    audio_ts = []

    # Battery Level
    battery_percent = []
    battery_level = []
    battery_ts = []


def align32(size):
    offset = size % 4
    if offset:
        size += 4 - offset

    return size

def find_sop(SensorData_PacketHeader) :

    sop = 0
    skipped = 0
    finished = 0

    while (sop != MagicNumber[0]):
        try:
            sop, timestamp_hi, timestamp_lo, logger_id, length, custom = unpack('IIIBBH', SensorData_PacketHeader)
        except:
            print("Incomplete header.")
            finished = 1
            return -1

        if (sop == MagicNumber[0]):
            break

        SensorData_PacketHeader.pop(0)
        SensorData_PacketHeader.append(bytearray(f.read(1))[0])
        skipped += 1

    return (finished, skipped, sop, timestamp_hi, timestamp_lo, logger_id, length, custom)


def process_config_data(SensorData):
    global PrescalerValue
    global RTCInputFrequency
    global DataPackingVersion
    global Ticks_per_second
    global TimeStampOffset
    global OldTimestampOffset

    # print("Config Data Handler")
    PrescalerValue =        (SensorData[3]  << 24   | SensorData[2]     << 16 | SensorData[1]  << 8 | SensorData[0])
    RTCInputFrequency =     (SensorData[5]  << 8    | SensorData[4])
    DataPackingVersion =    (SensorData[7]  << 8    | SensorData[6])
    TimestampOffsetLo =     (SensorData[11] << 24   | SensorData[10]    << 16 | SensorData[9]  << 8 | SensorData[8])
    TimestampOffsetHi =     (SensorData[15] << 24   | SensorData[14]    << 16 | SensorData[13] << 8 | SensorData[12])
    OldTimestampOffsetLo =  (SensorData[19] << 24   | SensorData[18]    << 16 | SensorData[17] << 8 | SensorData[16])
    OldTimestampOffsetHi =  (SensorData[23] << 24   | SensorData[22]    << 16 | SensorData[21] << 8 | SensorData[20])

    Ticks_per_second = RTCInputFrequency / (PrescalerValue + 1)

    old_timestamp_offset = OldTimestampOffsetHi << 32 | OldTimestampOffsetLo
    new_timestamp_offset = TimestampOffsetHi << 32 | TimestampOffsetLo

    print("CONFIG PACKET")
    print("====== ======")
    print("Updating PrescalarValue to", PrescalerValue)
    print("Updating RTC Input Frequency to", RTCInputFrequency)
    print("Data Format Version ", DataPackingVersion)
    print("Old TimeStamp Offset - " + repr(hex(old_timestamp_offset)))
    print("New TimeStamp Offset - " + repr(hex(new_timestamp_offset)))

    if (new_timestamp_offset != 0 and old_timestamp_offset == 0):
        # We have set the timestamp. Until code is available to backdate existing
        # data, simply disregard all data before this.
        print("TimeStamp Set - discarding all previous data")
        clear_data()

    print("-----------------------\n")

    return


def process_imu_data(SensorData):
#    print("IMU Data Handler")
    gx.append(SensorData[1] << 8 | SensorData[0])
    gy.append(SensorData[3] << 8 | SensorData[2])
    gz.append(SensorData[5] << 8 | SensorData[4])
    ax.append(SensorData[7] << 8 | SensorData[6])
    ay.append(SensorData[9] << 8 | SensorData[8])
    az.append(SensorData[11] << 8 | SensorData[10])

    imu_internal_ts.append(SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12])
    imu_ts.append(timestamp / Ticks_per_second)

    imufile.write(str(round(timestamp / Ticks_per_second,TimeSavingPrecision))+
    ","+str(np.int16(SensorData[1] << 8 | SensorData[0]))+\
    ","+str(np.int16(SensorData[3] << 8 | SensorData[2]))+\
    ","+str(np.int16(SensorData[5] << 8 | SensorData[4]))+\
    ","+str(np.int16(SensorData[7] << 8 | SensorData[6]))+\
    ","+str(np.int16(SensorData[9] << 8 | SensorData[8]))+\
    ","+str(np.int16(SensorData[11] << 8 | SensorData[10]))+\
    "\n")
    return


def process_env_data(SensorData):
    # print("Env Data Handler")
    pressure.append(SensorData[3] << 24 | SensorData[2] << 16 | SensorData[1] << 8 | SensorData[0])
    temperature.append(SensorData[7] << 24 | SensorData[6] << 16 | SensorData[5] << 8 | SensorData[4])
    humidity.append(SensorData[11] << 24 | SensorData[10] << 16 | SensorData[9] << 8 | SensorData[8])
    gas_resistance.append(SensorData[15] << 24 | SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12])
    gas_iaq.append(SensorData[19] << 24 | SensorData[18] << 16 | SensorData[17] << 8 | SensorData[16])
    lps22_pressure.append(SensorData[21] << 8 | SensorData[20])
    lps22_temperature.append(SensorData[23] << 8 | SensorData[22])
    env_ts.append(timestamp / Ticks_per_second)

    envfile.write(str(round(timestamp / Ticks_per_second,TimeSavingPrecision))+\
    ","+str(int(SensorData[3] << 24 | SensorData[2] << 16 | SensorData[1] << 8 | SensorData[0]))+\
    ","+str(int(SensorData[7] << 24 | SensorData[6] << 16 | SensorData[5] << 8 | SensorData[4]))+\
    ","+str(int(SensorData[11] << 24 | SensorData[10] << 16 | SensorData[9] << 8 | SensorData[8]))+\
    ","+str(int(SensorData[15] << 24 | SensorData[14] << 16 | SensorData[13] << 8 | SensorData[12]))+
    ","+str(int(SensorData[19] << 24 | SensorData[18] << 16 | SensorData[17] << 8 | SensorData[16]))+    "\n")
    return



def process_lightsensor_data(SensorData):
    # print("Light Sensor Data Handler")
    red.append(SensorData[1] << 8 | SensorData[0])
    green.append(SensorData[3] << 8 | SensorData[2])
    blue.append(SensorData[5] << 8 | SensorData[4])
    clear.append(SensorData[7] << 8 | SensorData[6])
    lightsensor_ts.append(timestamp / Ticks_per_second)
    # print("red      : ", repr(hex(SensorData[1] << 8 | SensorData[0])))
    # print("green    : ", repr(hex(SensorData[3] << 8 | SensorData[2])))
    # print("blue     : ", repr(hex(SensorData[5] << 8 | SensorData[4])))
    # print("clear    : ", repr(hex(SensorData[7] << 8 | SensorData[6])))

    lightfile.write(str(round(timestamp / Ticks_per_second,TimeSavingPrecision))+\
    ","+str(np.int16(SensorData[1] << 8 | SensorData[0]))+\
    ","+str(np.int16(SensorData[3] << 8 | SensorData[2]))+\
    ","+str(np.int16(SensorData[5] << 8 | SensorData[4]))+\
    ","+str(np.int16(SensorData[7] << 8 | SensorData[6]))+\
    "\n" )
    return


def process_audio_data(SensorData):
    # print("Audio Data Handler")
    audio_mean.append(SensorData[1] << 8 | SensorData[0])
    audio_peak.append(SensorData[3] << 8 | SensorData[2])
    audio_ts.append(timestamp / Ticks_per_second)
    # print("mean     : ", repr(hex(SensorData[1] << 8 | SensorData[0])))
    # print("peak     : ", repr(hex(SensorData[3] << 8 | SensorData[2])))

    audiofile.write(str(round(timestamp / Ticks_per_second,TimeSavingPrecision))+\
    ","+str(np.int16(SensorData[1] << 8 | SensorData[0]))+\
    ","+str(np.int16(SensorData[3] << 8 | SensorData[2]))+\
    "\n")
    return


def process_battery_data(SensorData):
    # print("Audio Data Handler")
    battery_percent.append(SensorData[0])
    if (DataPackingVersion>=1):
        battery_volt = 3.0 + SensorData[1]/200.0;
    else:
        battery_volt = (SensorData[1]+300.0)/100.0;
    battery_level.append(battery_volt)
    battery_ts.append(timestamp / Ticks_per_second)
    # print("mean     : ", repr(hex(SensorData[1] << 8 | SensorData[0])))
    # print("peak     : ", repr(hex(SensorData[3] << 8 | SensorData[2])))

    batteryfile.write(str(round(timestamp / Ticks_per_second,TimeSavingPrecision))+\
    ","+str(np.int8(SensorData[0]))+\
    ','+str(battery_volt)+\
    "\n")
    return


def process(logger_id, SensorData):
    if logger_id >= ID_OUT_OF_RANGE:
        print("Data Packet Header makes no sense. Invalid Logger ID (" + repr(logger_id) + ")... Skipping Packet..")
        return -1
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


if (sys.platform == "darwin"):
    sourcedir = "/Volumes/NO NAME/"
    filetimestamp=''

# sourcedir = 'good_data/'
# #sourcedir = "/Users/hli/work/Nokia-VTT/Datalogging/logger4/"
# filetimestamp='20200305_'
#sourcedir = "./"

max_index = 0

filebasename = 'DLOG'
filesuffix = '.BIN'

# Find out  how many filebasename files there are.
filesearchstring = sourcedir + filetimestamp + filebasename + '*' + filesuffix
print("Checking existence of " + filesearchstring)
filenames = glob.glob(filesearchstring)
print("Found " + str(len(filenames)) + " Data Files")
print(*filenames, sep='\n')
if (0 == len(filenames)) :
    print("Exiting")
    quit()

textfilenamebase=sourcedir + filetimestamp

imufile = open(textfilenamebase+"imu.csv", "w")
imufile.write("Time (s),gx,gy,gz,ax,ay,az\n")

envfile = open(textfilenamebase+"env.csv", "w")
envfile.write("Time (s),Pres (hPa),Temp (degC),Humidity (%),GasRes (ohm),GasIAQ\n")

lightfile = open(textfilenamebase+"light.csv", "w")
lightfile.write("Time (s), Red, Green, Blue, clear\n")

audiofile = open(textfilenamebase+"audio.csv", "w")
audiofile.write("Time (s), audio_peak, audio_mean\n")

batteryfile = open(textfilenamebase+"battary.csv", "w")
batteryfile.write("Time (s),Battery percentage (%), Votlage (V)\n")

total_bytes_skipped = 0

for file in filenames:
    with open(file, "rb") as f:
        print("Processing File ", file)

        while True:

            SensorData_PacketHeader = bytearray(f.read(PacketHeaderSize))

            bytes_read = len(SensorData_PacketHeader)

            if (bytes_read < PacketHeaderSize) :
                print("\n\n===============================================")
                print("End of Data file reached with " + repr(bytes_read) + " bytes left over")
                print("Total bytes skipped in data file is " + repr(total_bytes_skipped))
                print("===============================================")

                break

            (finished, skipped, sop, timestamp_hi, timestamp_lo, logger_id, length, custom) = find_sop(SensorData_PacketHeader)

            if (skipped) :
                print("WARNING: Skipped " + repr(skipped) + "bytes to find SOP")

            total_bytes_skipped += skipped


            # while (sop != MagicNumber[0]):
            #     try:
            #         sop, timestamp_hi, timestamp_lo, logger_id, length, custom  = unpack('IIIBBH', SensorData_PacketHeader)
            #     except:
            #         print("Incomplete header for final entry.")
            #         finished = 1
            #         break
            #     if (sop != MagicNumber[0]):
            #         SensorData_PacketHeader.pop(0)
            #         SensorData_PacketHeader.append(bytearray(f.read(1))[0])
            #         count+=1
            #         print("WARNING - Extra data in previous packet (Skipping Byte... " + repr(count) + ")")

            if finished:
                break

            timestamp = ((timestamp_hi * (2**32)) + timestamp_lo)
            if sop == 0 and timestamp == 0 and logger_id == 0 and length == 0 and custom == 0:
                print("End of Data...")
                break
            else:
                checkstring = 'StartOfPacket: ' + repr(hex(sop)) + ' MagicNumber: ' + repr(hex(MagicNumber[0]))
                # print(checkstring)
                # print('timestamp_hi         :' + repr(timestamp_hi))
                # print('timestamp_lo         :' + repr(timestamp_lo))
                # print('timestamp (s)        :' + repr(float(timestamp/Ticks_per_second)))
                # print('logger_id            :' + repr(hex(logger_id)))
                # print('length               :' + repr(length))
                # print('custom               :' + repr(hex(custom)))
                # print('PacketDataHeader     :' + repr(SensorData_PacketHeader))

                align32_length = align32(length)
                # print('aligned length       :' + repr(align32_length))

                # if (logger_id == AudioDataID):
                #     print("Audio Data")

                try:
                    SensorData = f.read(align32_length)
                except:
                    print("Not enough data in file for last entry.")
                    break
                # print('SensorData     :' + repr(SensorData))
                # print('Payload:')
                # for i in range(length):
                #     print("     ", i, " : ", repr(hex(SensorData[i])))
                #assert sop == MagicNumber[0]

                success = process(logger_id, SensorData)
                if (0 == success):
                    break

        f.close()

gxarray = np.array(gx, np.int16)
gyarray = np.array(gy, np.int16)
gzarray = np.array(gz, np.int16)
axarray = np.array(ax, np.int16)
ayarray = np.array(ay, np.int16)
azarray = np.array(az, np.int16)

temperaturearray = np.array(temperature, np.int32) / 100.0
pressurearray = np.array(pressure, np.int32) / 100
humidityarray = np.array(humidity, np.int32) / 1000.0
gasresarray = np.array(gas_resistance, np.int32)
gasiaqarray = np.array(gas_iaq, np.int32)


redarray = np.array(red, np.uint16)
greenarray = np.array(green, np.uint16)
bluearray = np.array(blue, np.uint16)
cleararray = np.array(clear, np.uint16)

audiomeanarray = np.array(audio_mean)
audiopeakarray = np.array(audio_peak)

batterypercentarray = np.array(battery_percent)
batteryrawarray = np.array(battery_level)

imufile.close()
envfile.close()
lightfile.close()
audiofile.close()
batteryfile.close()

# audio_ts_array = np.array(audio_ts) / (60*60)

plt.figure(1)
plt.subplot(521)
plt.plot(imu_ts, gxarray, 'r', imu_ts, gyarray, 'b', imu_ts, gzarray, 'g')
plt.legend(['gx', 'gy', 'gz'])
plt.xlabel('Time (s)')
plt.ylabel('Gyroscope')

plt.subplot(522)
plt.plot(imu_ts, axarray, 'r', imu_ts, ayarray, 'b', imu_ts, azarray, 'g')
plt.legend(['ax', 'ay', 'az'])
plt.xlabel('Time (s)')
plt.ylabel('Accelerometer')

plt.subplot(523)
plt.plot(env_ts, temperaturearray)
plt.xlabel('Time (s)')
plt.ylabel('Temperature (℃)')

plt.subplot(524)
plt.plot(env_ts, pressurearray)
plt.xlabel('Time (s)')
plt.ylabel('Pressure (hPa)')

plt.subplot(525)
plt.plot(env_ts, humidityarray)
plt.xlabel('Time (s)')
plt.ylabel('Humidity (%)')

plt.subplot(526)
plt.plot(env_ts, gasiaqarray)
plt.xlabel('Time (s)')
plt.ylabel('Gas IAQ')

plt.subplot(527)
plt.plot(lightsensor_ts, redarray, 'r', lightsensor_ts, greenarray, 'g', lightsensor_ts, bluearray, 'b', lightsensor_ts,
         cleararray, 'm')
plt.legend(['red', 'green', 'blue', 'clear'])
plt.xlabel('Time (s)')
plt.ylabel('Light Sensor')

plt.subplot(528)
plt.plot(audio_ts, audiomeanarray, 'b', audio_ts, audiopeakarray, 'r')
plt.legend(['mean', 'peak'])
plt.xlabel('Time (s)')
plt.ylabel('Audio')

plt.subplot(529)
plt.plot(battery_ts, batterypercentarray)
plt.xlabel('Time (s)')
plt.ylabel('Battery (%)')

plt.subplot(5, 2, 10)
plt.plot(battery_ts, batteryrawarray)
plt.xlabel('Time (s)')
plt.ylabel('Battery Voltage (V)')


plt.show()
