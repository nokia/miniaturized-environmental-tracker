# Gecko Device Firmware Update (DFU) Instruction


## Step 1 - Keys

Private key has not been uploaded for security reason. Keys are paired files in dfu folder of the workspace: `gecko_private.key` and `gecko_public_key.c`. new pair of keys need to be generated with the following commands: 

`nrfutil keys generate private.key`

`nrfutil keys display --key pk --format code private.key --out_file dfu_public_key.c`

## Step 2 - Install Micro-ECC

Go to the SDK Root directory and enter external/micro-ecc. Run build_all.sh or build_all.exe. On UNIX based systems (including OS-X) it may be necessary to run dos2unix on build_all.sh first.

## Step 3 - Build Bootloader

Bootloader is in dfu/secure\_bootloader folder. To build the bootloader go to that directory and run make. Use the argument DEBUG=1 for the debug build.

## Step 4 - Compile the Application
Compile the Application in Segger Embedded Studio using 

`gecko/s132/ses/ble\_app\_gecko\_s132.emProject`

The application hex file should be located under 

`gecko/s132/ses/Output/Debug/Exe/ble\_app\_gecko\_s132.hex`


## Step 5 - Modify Bootloader and combine images.
The bootloader then needs to be modified with the details of the Application executable. For this, the generateSoftDeviceAndBootloaderHex.sh can be used. This generates the correct bootloader settings for the Application, and merges this resultant hex with the bootloader hex, and then with the softdevice hex producing a single hex file that can be loaded by Segger Embedded Studio. It also creates the combined Application, Bootloader and Softdevice for programming with nrfjprog. 

SES calls a script from the <project>/s132/ses directory which calls the generateSoftDeviceAndBootloaderHex.sh after Linking and before Loading.

## Makefile Option
Ultimately these steps should be wrapped up in a makefile but currently this is not fully working.

## Application DFU Package
To generate the package to transmit over the DFU protocol run 

`make app_dfu_package.zip`