#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


./generateBootloaderSettingsHex.sh

nrfjprog --program merged_SD_Bootloader_Application.hex --chiperase --verify  
nrfjprog --reset

