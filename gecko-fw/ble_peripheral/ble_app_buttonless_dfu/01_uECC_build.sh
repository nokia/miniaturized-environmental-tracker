#!/bin/bash
# Colyright 2018-2022 Nokia
#
# All software in this repository are licensed under the [BSD 3 Cluse License (BSD-3-Clause)](LICENSES\BSD-3-Clause.txt).
# Hardware design is licensed with the [Nokia HW License](LICENSES\nokia-hw-license.txt).



# Micro_ECC is part of the SDK but requires source code from https://github.com/kmackay/micro-ecc.git.
# The script in the SDK directory takes care of all of this.

pushd ../../../external/micro-ecc/
dos2unix build_all.sh
sh build_all.sh
popd

