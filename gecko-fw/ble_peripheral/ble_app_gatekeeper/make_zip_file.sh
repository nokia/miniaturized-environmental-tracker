#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


CONFIG='Debug'
APPLICATION_HEX=gecko/s132/ses/Output/${CONFIG}/Exe/ble_app_gecko_s132.hex

nrfutil pkg generate --hw-version 52 --application-version 1 --application ${APPLICATION_HEX} --sd-req 0xb7 --key-file ../../dfu/gecko_private.key gecko_app_dfu_package.zip
echo "DFU zip package created: gecko_app_dfu_package.zip created"

# cp -a gecko_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
cp -a gecko_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
echo "copied zip package to iCould folder"


