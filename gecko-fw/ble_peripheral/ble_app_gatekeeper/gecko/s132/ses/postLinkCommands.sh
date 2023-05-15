#!/bin/bash -l
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


source ~/.bash_profile
cd ../../..
./generateSoftDeviceAndBootloaderHex.sh

# the following command makes sure Segger will also generate zip file for uploading with DFU:
./make_zip_file.sh