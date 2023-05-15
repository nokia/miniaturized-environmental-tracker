#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).


# active conda becuase nfrutil is installed under conda base environment
# comment these two lines if no conda installed(e.g. bash can find nrfutil)
# source ~/.bash_profile
# conda activate base
echo "$PATH"
nrfutil settings generate \
	--application gecko/s132/ses/Output/Debug/Exe/ble_app_gecko_s132.hex \
	--family NRF52 \
	--application-version 1 \
	--bootloader-version 1 \
	--softdevice ../../../components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex \
	--bl-settings-version 2 \
	bootloader_Settings.hex

mergehex -m ../../dfu/secure_bootloader/gecko_ble_debug/armgcc/_build/nrf52832_xxaa_s132.hex bootloader_Settings.hex -o bootloaderWithSettings.hex
mergehex -m ../../../components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex bootloaderWithSettings.hex -o merged_SD_Bootloader.hex
mergehex -m merged_SD_Bootloader.hex gecko/s132/ses/Output/Debug/Exe/ble_app_gecko_s132.hex -o merged_SD_Bootloader_Application.hex

echo "Softdevice Bootloader Application hex file created"
