
# Gecko Sensor Electronics Firmware

This project develops firmware for Gecko environmental sensor. 

This database (gecko-fw folder) should be cloned into the top level of the nRF5 SDK. The version used for development is nRF5\_SDK\_15.3.0\_59ac345. This can be obtained from the Nordic Website ([https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs]())

In this case, place gecko-fw folder under nRF5_SDK_15.3.0_59ac345.

All code has been developed using SES and aslo tested with armgcc. 

### To Make a DFU firmware image
1) Build the firmware in Segger Studio.  
2) go to /ble_peripheral/ble_app_gatekeeper.  
3) On a PC in command line run: "sh generateSoftDeviceAndBootloaderHex.sh".  
4) Go to Segger Studio and Upload Image to device.   

### To Make a .zip file
1) Build the firmware in Segger Studio  
2) go to /ble_peripheral/ble_app_gatekeeper  
3) On a PC in command line run: "sh make_zip_file.sh"  
4) Transfer .zip to phone and use nRF Connect 

### Use of Makefile
Makefile under the folder /ble_peripheral/ble_app_gatekeeper is fully working now. One thing need to be done is to add Segger Embedded Studio bin folder to path of the shell you are using. One Mac the following can be added to the end of .bash_prifile (or .zshrc if you are ing zsh), with your version of Segger Embedded Studio. 

```bash
# Segger Embedded Studio  
export SEGGER_VERSION=4.40  
export PATH="$PATH:/Applications/SEGGER Embedded Studio for ARM ${SEGGER_VERSION}/bin"

```

#### Then make has the follow targes to use:  

     all                                    :  Prepare hex and zip files, not including program and upload2cloud  
     micro-ecc                              :  Obtain and make the micro-ecc library. Only needs to be done once.  
     bootloader                             :  Compile secure bootloader image
     bootloader_Settings.hex                :  Generate bootloader settings for current application.  
     bootloaderWithSettings.hex             :  Combine bootloader with bootloader settings.  
     merged_SD_Bootloader.hex               :  Combine bootloader with bootloader settings and SoftDevice.  
     merged_SD_Bootloader_Application.hex   :  Combine bootloader with bootloader settings, SoftDevice, and Application.  
     app_dfu_package.zip                    :  Generate the package for DFU update
     program                                :  Program the Initial Image to nrf52 device. 
     upload2cloud                           :  Upload the initial Image to iClould on Mac computer.  
     clean                                  :  Clean volatile stuff.  
     clean-all                              :  Clean Everything including micro-ecc