# Introduction 

 Miniaturized Environmental Tracker is a device developed by Data and Device Group of Nokia Bell Labs at Cambridge site. The device is called Gecko (originally: gatekeeper) and both names are widely used in this repository. 

## Devices details

Gecko is a portable environmental sensing device. It contains temperature, air pressure, air quality, light, sound and IMU sensors. The device also has a solar panel for energy harvesting. We have manufactured hardware devices in two sizes, with different dimensions and battery sizes. 
1) Gecko, the normal size device, has a size of 55x25x15 mm with 500 mAh LiPo battery and  weight of about 24 grams. 
2) Geckino, smaller size version, has a size of  40x25x15 mm with 150 mAh LiPo battery and weight of about 16 grams. 

### MCU and Sensors
* MCU is ARM-based Cortex M4-F 64MHz processor nRF52832 from Nordic Semiconductor.   
* Pressure and temperature sensor: LPS22HB from STMicroelectronics.  
* Gas Quality Sensor (Volatile Organic Compounds): BME680 from Bosch Sensortec. It can measure temperature, pressure and humidity as well.  
* Colour/Light Sensor: BH1745NUC from Rohm Semiconductor.  
* Digital microphone: MP34DT05 from STMicroelectronics.  
* Motion Sensor (6-axis, Accelerometer and Gyroscope): BMI160 from Bosch Sensortec.  

## Data parsing
Gecko's data can be saved on SD card or real time streamed over BLE according to application scenarios. All sensor's data saved in SD cards is in binary format with filename DLOG\_\*.BIN. The BIN file data can be procesed by python scripts for human and machine reading. 

## Firmware and programming instructions

The firmware for Gecko was developed in Segger Embedded Studio with Nordic Semiconductor nRF5 SDK 15.3.0 with softdevice version s132. 
Segger Embedded Studio can be downloaded: https://www.segger.com/products/development-tools/embedded-studio/

nRF5 SDK 15.3.0 can be downloaded: https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs

JLink device hardare, its drive (https://www.segger.com/downloads/jlink/) and nRF Command Line Tools are also required for compiling and uploading the firmware to device. 

## How to start 
Download and Install Segger Embedded Studio and get a license for Nordic MCU as needed. Version of SDK used is:   
* nRF5 SDK 15.3.0    

Download above nRF5 SDK 15.3.0 and unzip the whole package of nRF5_SDK_15.3.0_59ac345 in a working directory and copy gecko-fw folder in this repository into it. The Segger project file for gecko-fw can be loaded from:  
`gecko-fw/ble_peripheral/ble_app_gatekeeper/gecko/s132/ses/ble_app_gecko_s132.emProject`

### Modifications to nRF5 SDK 15.3.0 files
The following code modifications may be required for compiling in current version of Segger Embedded Studio 6.34: 
1. 	 components/libraries/uart/regart.c 


	Line 101 change: 
	`#if defined(__SES_VERSION) && (__SES_VERSION >= 34000)` to 
	`#if defined(__SES_VERSION) && (__SES_VERSION >= 34000) && !defined(__SEGGER_RTL_VERSION)`
2.   external/fatfs/src/ffconf.g

	Line 101 change to the following to enable long filename support:
	`#define	_USE_LFN	2`
3.   components/libraries/bsp/bsp_config.h


	Line 89 and 90: 
	`#define ADVERTISING_LED_ON_INTERVAL            50 //200`
	`#define ADVERTISING_LED_OFF_INTERVAL           9950 //1800`
	This is to make the LED advertising less frequently. 
4.   You may also need to change the following file: 	external/segger_rtt/SEGGER_RTT_Syscalls_SES.c


  Add `#if !defined(__SEGGER_RTL_VERSION)` at line 72 and `#endif` at around line 262.
   
If there are more errors during compiling, you may want to switch Segger Embedded Studio to early version published during 2019, for instance V5.40.
### Other compiler
The project can also be complied with armgcc. the Makefile is located at: gecko-fw/ble_peripheral/ble_app_gatekeeper/gecko/s132/armgcc 

### Hardware programming interface
First time programming of Gecko need to go through SWD interface embedded with micro-USB plug. SWD pin configuration can be found in the hardware schematic files. A customized interface connection need to be made to programmer (JLink or others). 

### Other related instructions 
 Other useful documents and related instructions:

 * gecko-fw/README.md
 * gecko-fw/ble_peripheral/ble_app_gatekeeper/DFU_README.md  
 
 ## Other files in this repository
 
 * gecko-hw  
 Gecko PCB hardware design files for production.
 
 * gecko_ble_gateway  
 Raspberry Pi gateway used to collect Gecko data over BLE in real time, parsing data and forward  data in json format to remote server. 
 
 * gecko-case  
 Case designs for both versions of the device: Gecko and Geckino.

 * data_parsing_scripts  
 For processing binary files in \*.BIN format saved on SD card to human readable format. Similar script is also used in gecko_ble_gateway files.  