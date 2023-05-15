
# ========================================================================
#
# Makefile for the whole DFU build process.
#
# ========================================================================

DEBUG                   = 1
VERBOSE                 = 1
EMBUILD 		= emBuild
BUILD   		= Debug
SOFTDEVICE		= ../../../components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex
FAMILY			= NRF52
APPLICATION_VERSION	= 1
BOOTLOADER_VERSION	= 1
BL_SETTINGS_VERSION	= 1
BOOTLOADER_DIR		= ../../dfu/secure_bootloader
BOOTLOADER_BASENAME	= secure_bootloader_ble_s132_gecko

ifeq (${DEBUG},1)
	CONFIG='Debug'
	BOOTLOADER_SUFFIX     = _debug
else
	CONFIG='Release'
	BOOTLOADER_SUFFIX     =
endif

ifeq (${VERBOSE},1)
	EMBUILD_VERBOSE=-verbose
else
	EMBUILD_VERBOSE=
endif


BOOTLOADER_HEX           = ${BOOTLOADER_DIR}/hex/${BOOTLOADER_BASENAME}${BOOTLOADER_SUFFIX}.hex
APPLICATION_HEX          = gecko/s132/ses/Output/${CONFIG}/Exe/ble_app_gecko_s132.hex


all: micro-ecc bootloader-gcc app_dfu_package.zip merged_SD_Bootloader_Application.hex

clean-all: micro-ecc-clean bootloader-clean clean

clean:	app-clean hex-clean

# ========================================================================
#
# Micro-ECC Library
#
# This library is not included in the SDK and needs to be obtained from git.
# There is however a script included in the SDK that handles this - however it
# fails under OsX (and possibly Linux?) due to dos line endings. This is fixed
# by running dos2unix (and for traceability this is saved as a new file).
#
# ========================================================================

ifeq (${FAMILY},NRF52)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF52832)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF52840)
MICRO_ECC_LIBFILE = nrf52hf_armgcc/armgcc/micro_ecc_lib_nrf52.a
else ifeq (${FAMILY},NRF51)
MICRO_ECC_LIBFILE = nrf51_armgcc/armgcc/micro_ecc_lib_nrf51.a
else
	$(error Need to define family of MCU for Micro-ECC (e.g. NRF52))
endif

MICRO_ECC_SDK_DIR     = ../../../external/micro-ecc
MICRO_ECC_GIT_DIR     = micro-ecc

UNIX_BUILD_SCRIPT = build_all.unix.sh
ORIG_BUILD_SCRIPT = build_all.sh


${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT} : ${MICRO_ECC_SDK_DIR}/${ORIG_BUILD_SCRIPT}
	dos2unix -n ${MICRO_ECC_SDK_DIR}/${ORIG_BUILD_SCRIPT} ${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT}

${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR} : ${MICRO_ECC_SDK_DIR}/${UNIX_BUILD_SCRIPT}
	cd ${MICRO_ECC_SDK_DIR} && source ${UNIX_BUILD_SCRIPT}


.PHONY:micro-ecc
micro-ecc: ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}: ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR}

.PHONY:micro-ecc-clean
micro-ecc-clean:
	@rm -rf ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_GIT_DIR}


# ========================================================================
#
# BootLoader
#
# This can be found in ../../dfu/secure_bootloader.
#
# Compiler options are emBuild (Segger's compiler using the ses project file)
# or gcc (uses the armgcc Makefile. Since everything else for the firmware
# uses the Segger compiler it makes sense to be consistent for this so a
# Makefile has been written to handle the compilation using Segger. However,
# this appears to generate a larger image has the attempt to mergehex fails
# due to conflicts. For this reason two targets have been written to select
# between armgcc and ses.
#
# There are debug and non-debug versions of the bootloader available. These
# are in separate directories: -
#
# 	gecko_ble
# 	gecko_ble_debug
#
# To compile and use the DEBUG version, set the DEBUG=1 flag in the call
# to this makefile.
#
# ========================================================================


.PHONY:bootloader
.PHONY:bootloader-gcc
.PHONY:bootloader-ses

bootloader: bootloader-gcc

bootloader-ses : ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
	cd ${BOOTLOADER_DIR} && make DEBUG=${DEBUG} MICRO_ECC_SDK_DIR=${MICRO_ECC_SDK_DIR} MICRO_ECC_GIT_DIR=${MICRO_ECC_GIT_DIR} MICRO_ECC_LIBFILE=${MICRO_ECC_LIBFILE}

bootloader-gcc : ${MICRO_ECC_SDK_DIR}/${MICRO_ECC_LIBFILE}
	cd ${BOOTLOADER_DIR}/gecko_ble${BOOTLOADER_SUFFIX}/armgcc && make
	mkdir -p ${BOOTLOADER_DIR}/hex
	cp ${BOOTLOADER_DIR}/gecko_ble${BOOTLOADER_SUFFIX}/armgcc/_build/nrf52832_xxaa_s132.hex ${BOOTLOADER_HEX}

bootloader-clean :
	@cd ${BOOTLOADER_DIR} && make DEBUG=${DEBUG} clean \
		MICRO_ECC_SDK_DIR=${MICRO_ECC_SDK_DIR} \
		MICRO_ECC_GIT_DIR=${MICRO_ECC_GIT_DIR} \
		MICRO_ECC_LIBFILE=${MICRO_ECC_LIBFILE}


# ========================================================================
#
# Gecko Firmware Application
#
#
# app_dfu_package.zip
#
# This is the Application compiled, signed, and package up ready for an
# Over-the-Air update.
#
# Note that the Firmware ID of the SoftDevice has to be defined.
# This can be found in the RELEASE NOTES for the appropriate SoftDevice.
#
# For SoftDevice 6.1.1 the Firmware ID is 0x00B7
#
# ========================================================================


${APPLICATION_HEX}:
	cd gecko/s132/ses && emBuild -rebuild -config ${CONFIG} -D BLE_DFU_ENABLED=1 ble_app_gecko_s132.emProject ${EMBUILD_VERBOSE}


app_dfu_package.zip: ${APPLICATION_HEX}
	nrfutil pkg generate --hw-version 52 --application-version 1 --application ${APPLICATION_HEX} --sd-req 0xb7 --key-file ../../dfu/gecko_private.key app_dfu_package.zip

app-clean:
	@rm -f app_dfu_package.zip
	@rm -rf gecko/s132/ses/Output


# ========================================================================
#
# Initial Image
#
# SoftDevice + Bootloader + Bootloader Settings + Gecko FW Image
#
#
# This is the hex file combining everything for the initial DFU capable
# image to be programmed via nrfjprog
#
# ========================================================================


bootloader_Settings.hex : ${APPLICATION_HEX} ${SOFTDEVICE}
	nrfutil settings generate \
		--application ${APPLICATION_HEX} \
		--family ${FAMILY} \
		--application-version ${APPLICATION_VERSION} \
		--bootloader-version ${BOOTLOADER_VERSION} \
		--softdevice ${SOFTDEVICE} \
		--bl-settings-version ${BL_SETTINGS_VERSION} \
		bootloader_Settings.hex

bootloaderWithSettings.hex: bootloader_Settings.hex ${BOOTLOADER_HEX}
	mergehex -m ${BOOTLOADER_HEX} bootloader_Settings.hex -o bootloaderWithSettings.hex

#bootloaderWithSettings.hex: bootloader_Settings.hex
#	mergehex -m ../../dfu/secure_bootloader/gecko_ble_debug/armgcc/_build/nrf52832_xxaa_s132.hex bootloader_Settings.hex -o bootloaderWithSettings.hex


merged_SD_Bootloader.hex: bootloaderWithSettings.hex
	mergehex -m ${SOFTDEVICE} bootloaderWithSettings.hex -o merged_SD_Bootloader.hex


merged_SD_Bootloader_Application.hex: merged_SD_Bootloader.hex
	mergehex -m merged_SD_Bootloader.hex ${APPLICATION_HEX} -o merged_SD_Bootloader_Application.hex

hex-clean:
	@rm -f *.hex

# ========================================================================
#
# Program the Initial Image
#
# ========================================================================

.PHONY: program
program: merged_SD_Bootloader_Application.hex
	nrfjprog --program merged_SD_Bootloader_Application.hex --chiperase
	nrfjprog --reset


# ========================================================================
#
# Copy the gecko_app_dfu_package to Cloud for access from other devices.
#
# TODO: Support for other cloud storage e.g. OneDrive.
# ========================================================================

OS := $(shell uname)

ifeq ($(OS), Darwin)
.PHONY:cloud
upload2cloud: gecko_app_dfu_package.zip
	cp -a gecko_app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs
else
upload2cloud:
	@echo Not Copying to Cloud
endif

# ========================================================================
# HELP
# ========================================================================

.PHONEY: help
help:
	@echo 'make targets: -'
	@echo '     all                                          :  Prepare hex and zip files, not including program and upload2cloud '
	@echo '     micro-ecc                                    :  Obtain and make the micro-ecc library. Only needs to be done once.'
	@echo '     bootloader                                   :  Compile secure bootloader image'
	@echo '     bootloader_Settings.hex                      :  Generate bootloader settings for current application'
	@echo '     bootloaderWithSettings.hex                   :  Combine bootloader with bootloader settings.'
	@echo '     merged_SD_Bootloader.hex                     :  Combine bootloader with bootloader settings and SoftDevice'
	@echo '     merged_SD_Bootloader_Application.hex         :  Combine bootloader with bootloader settings, SoftDevice, and Application'
	@echo '     gecko_app_dfu_package.zip                          :  Generate the package for DFU update'
	@echo '     program                                      :  Program the Initial Image to nrf52 device'
	@echo '     upload2cloud                                 :  Upload the initial Image to iClould on Mac computer'
	@echo '     clean                                        :  Clean volatile stuff'
	@echo '     clean-all                                    :  Clean Everything including micro-ecc'
