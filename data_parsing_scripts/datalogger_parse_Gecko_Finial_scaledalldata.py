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


if (sys.platform == "darwin"):
    sourcedir = "/Volumes/NO NAME/"
elif(sys.platform == "win32"):
    sourcedir = "D:/"
else:
    sourcedir = "/dev/sda1/"


# sourcedir = ''

filetimestamp=''
fileprefix = ''

# sourcedir = 'good_data/'
# #sourcedir = "/Users/hli/work/Nokia-VTT/Datalogging/logger4/"
# filetimestamp='20200305_'
#sourcedir = "./"

max_index = 0

filebasename = 'DLOG'
filesuffix = '.BIN'


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
    ","+str(int(SensorData[3] << 24 | SensorData[2] << 16 | SensorData[1] << 8 | SensorData[0])/100)+\
    ","+str(int(SensorData[7] << 24 | SensorData[6] << 16 | SensorData[5] << 8 | SensorData[4])/100)+\
    ","+str(int(SensorData[11] << 24 | SensorData[10] << 16 | SensorData[9] << 8 | SensorData[8])/1000)+\
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
imufile.write("Time(s),gx(°/s),gy(°/s),gz(°/s),ax(g),ay(g),az(g)\n")

envfile = open(textfilenamebase+"env.csv", "w")
envfile.write("Time(s),Pres(hPa),Temp(°C),Humidity(%),GasRes(ohm),GasIAQ\n")

lightfile = open(textfilenamebase+"light.csv", "w")
lightfile.write("Time (s), Red, Green, Blue, clear\n")

audiofile = open(textfilenamebase+"audio.csv", "w")
audiofile.write("Time (s), audio_mean, audio_peak\n")

batteryfile = open(textfilenamebase+"battary.csv", "w")
batteryfile.write("Time(s),Battery percentage(%), Votlage(V)\n")

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

gx_scaled=gxarray*(2000/32767)
gy_scaled=gyarray*(2000/32767)
gz_scaled=gzarray*(2000/32767)
ax_scaled = (axarray/16384)
ay_scaled = (ayarray/16384)
az_scaled = (azarray/16384)

temperaturearray = np.array(temperature, np.int32) / 100.0
pressurearray = np.array(pressure, np.int32) / 100
humidityarray = np.array(humidity, np.int32) / 1000.0
gasresarray = np.array(gas_resistance, np.int32)
gasiaqarray = np.array(gas_iaq, np.int32)


redarray = np.array(red, np.uint32)
greenarray = np.array(green, np.uint32)
bluearray = np.array(blue, np.uint32)
cleararray = np.array(clear, np.uint32)

audiomeanarray = np.array(audio_mean)
audiopeakarray = np.array(audio_peak)

batterypercentarray = np.array(battery_percent)
batteryrawarray = np.array(battery_level)

imufile.close()
envfile.close()
lightfile.close()
audiofile.close()
batteryfile.close()

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

# illuminane calculation
Gain = 16
Itime = 160
lxarray = [0]*len(cleararray)
cctarray = [0]*len(cleararray)

for idx, G in enumerate(greenarray, start=0):
    R = redarray[idx]
    B = bluearray[idx]
    C = cleararray[idx]

#   Calculation of illuminace
    if (G < 1 ) :
        lx_tmp = 0
    elif (C/G) < 0.160:
        lx_tmp = 0.202 * R + 0.766 * G
    else:
        lx_tmp = 0.159 *R + 0.646 * G
    lx_tmp = max(lx_tmp, 0)
    lx = lx_tmp/Gain/Itime * 160  # unit of itime is ms
    lxarray[idx] = lx

#     calculation of color temperature
    total = R+G+B
    if ((G<1) or (total < 1)):
        CCT = 0
        # break
        # print(total)
    # B_ratio = B/(R+G+B)
    # R_ratio = R/(R+G+B)
    else: 
        B_ratio = B/total
        R_ratio = R/total
        if (C/G) < 0.160:
            B_eff = min(B_ratio * 3.13, 1)
            CCT = (1 - B_eff) * 12746 * np.exp(-2.911 * R_ratio) + B_eff * 1637 * np.exp(4.865 * B_ratio)
        else:
            B_eff = min(B_ratio * 10.67, 1)
            CCT = (1 - B_eff) * 16234 * np.exp(-2.781 * R_ratio) + B_eff * 1882 * np.exp(4.448 * B_ratio)
    #     if (CCT > 10000):
    #         CCT = 10000
    cctarray[idx] = CCT

light_ave_range = 1
lxarray_a  =   moving_average(lxarray, light_ave_range)
cctarray_a =   moving_average(cctarray, light_ave_range)

#     Calculation of sould level
Amarray = [0]*len(audiomeanarray)
Aparray = [0]*len(audiomeanarray)
for index, M in enumerate(audiomeanarray, start=0):
    P = audiopeakarray[index]
    Am = 20 * np.log10(M/32767) - 20 -3.2 +120
    Ap = 20 * np.log10(P/32767) - 20 -3.2 +120
    Amarray[index] = Am
    Aparray[index] = Ap

audio_ave_range = 1
try:
    Amarray_a =   moving_average(Amarray, audio_ave_range)
    Aparray_a =   moving_average(Aparray, audio_ave_range)
except:
    print("Audio data is empty.")
#    break
plt.figure(1,figsize = (15,10))
plt.subplot(421)
plt.plot(imu_ts, gx_scaled, 'r', imu_ts, gy_scaled, 'b', imu_ts, gz_scaled, 'g')
plt.legend(['gx', 'gy', 'gz'],loc='upper right')
# plt.xlabel('Time (s)')
# plt.ylabel('Gyroscope')
plt.ylabel('Angular Vel.[dps]')

plt.subplot(422)
plt.plot(imu_ts, ax_scaled, 'r', imu_ts, ay_scaled, 'b', imu_ts, az_scaled, 'g')
plt.legend(['ax', 'ay', 'az'],loc='upper right')
# plt.xlabel('Time (s)')
# plt.ylabel('Accelerometer')
plt.ylabel('Acceleration [g]')

plt.subplot(423)
plt.plot(env_ts, temperaturearray,'b')
# plt.xlabel('Time (s)')
plt.ylabel('Temperature [℃]')

plt.subplot(424)
plt.plot(env_ts, pressurearray,'b')
# plt.xlabel('Time (s)')
plt.ylabel('Pressure [hPa]')
# plt.ylim([1000,1010])

plt.subplot(425)
plt.plot(env_ts, humidityarray,'b')
# plt.xlabel('Time (s)')
plt.ylabel('Humidity [%]')

ax0=plt.subplot(426)
ax1 = ax0.twinx()

ax0.plot(env_ts, gasresarray/1000, 'b')
# ax0.set_xlabel('Time (s)')
ax0.set_ylabel('Gas Sensor R [kΩ]',color='b')
ax0.tick_params(axis='y', colors='b')
ax1.spines['left'].set_color('b')

ax1.plot(env_ts, gasiaqarray,'r')
ax1.set_ylabel('Gas IAQ',color='r')
ax1.tick_params(axis='y', colors='red')
ax1.spines['right'].set_color('red')

ax2 = plt.subplot(428)
# ax2.plot(lightsensor_ts, redarray, 'r', lightsensor_ts, greenarray, 'g', lightsensor_ts, bluearray, 'b', lightsensor_ts,cleararray, 'm')
# plt.legend(['red', 'green', 'blue', 'clear'],loc='upper right')
# plt.xlabel('Time (s)')
# plt.ylabel('Data Count [a.u.]')

ax3 = ax2.twinx()
# ax2.plot(lightsensor_ts,lxarray,'b')
ax2.plot(lightsensor_ts[light_ave_range-1:],lxarray_a,'b')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('light intensity [lx]',color ='b')
ax2.tick_params(axis='y', colors='b')
# ax2.set_ylim([00,1000])

ax3.spines['left'].set_color('b')

ax3.plot(lightsensor_ts,cctarray, 'r')
# ax3.set_ylabel('Color Temperature [K]',color='r')
ax3.plot(lightsensor_ts[light_ave_range-1:],cctarray_a, 'r')
ax3.tick_params(axis='y', colors='red')
ax3.spines['right'].set_color('red')
# ax3.set_ylim([4000,10000])

plt.subplot(427)
plt.plot(audio_ts[audio_ave_range-1:],Amarray_a, 'r',audio_ts[audio_ave_range-1:],Aparray_a, 'b')
# plt.plot(audio_ts, Amarray, 'r', audio_ts, Aparray, 'b')
plt.legend(['mean', 'peak'],loc='upper right')
plt.xlabel('Time (s)')
plt.ylabel('Sound level [dB$_{SPL}$]')

plt.show()
