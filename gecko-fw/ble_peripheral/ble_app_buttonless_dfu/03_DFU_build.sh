#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


# sd-req for SoftDevice 6.1.1 is 0xb7.

pushd gecko/s132/armgcc
make
popd

nrfutil pkg generate --hw-version 52 --application-version 1 --application gecko/s132/armgcc/_build/nrf52832_xxaa.hex --sd-req 0xb7 --key-file ../../dfu/gecko_private.key app_dfu_package.zip

# Option - build the program with Segger, in which case the following command would be used...

# nrfutil pkg generate --hw-version 52 --application-version 1 --application gecko/s132/ses/Output/Debug/Exe/ble_app_buttonless_dfu_gecko_s132.hex --sd-req 0xb7 --key-file ../../dfu/gecko_private.key app_dfu_package.zip


