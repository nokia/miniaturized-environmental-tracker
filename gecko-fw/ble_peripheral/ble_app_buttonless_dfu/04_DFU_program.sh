#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


# Merge the Bootloader and the Softdevice

mergehex -m ../../../components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex ../../dfu/secure_bootloader/gecko_ble/armgcc/_build/nrf52832_xxaa_s132.hex -o merged_SD_Bootloader.hex

#Program the merged file

nrfjprog --program merged_SD_Bootloader.hex --chiperase

# Reboot the device so new program is run
nrfjprog --reset


