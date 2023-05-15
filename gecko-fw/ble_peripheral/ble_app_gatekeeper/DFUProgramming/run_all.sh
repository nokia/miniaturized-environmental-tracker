# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).

sh ./01_uECC_build.sh
sh ./02_Bootloader_build.sh	
sh ./03_DFU_build.sh		
sh ./04_DFU_program.sh

cp -a app_dfu_package.zip ~/Library/Mobile\ Documents/com~apple~CloudDocs

