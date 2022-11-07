#!/bin/sh

export TOP_DIR=$(pwd)
export GNU_INSTALL_ROOT=/usr/bin/
export GNU_VERSION=10.3.1
export GNU_PREFIX=arm-none-eabi

cd app
make -j8
cd $TOP_DIR

cd ble-firmware/external/micro-ecc/
make -C nrf52hf_armgcc/armgcc
cd $TOP_DIR

cd dfu
make -j8
cd $TOP_DIR

mkdir artifacts
mv dfu/_build/nrf52832_xxaa_s132.bin artifacts/dfu.bin
mv dfu/_build/nrf52832_xxaa_s132.hex artifacts/dfu.hex
mv app/_build/nrf52832_xxaa.bin artifacts/app.bin
mv app/_build/nrf52832_xxaa.hex artifacts/app.hex
rm artifacts.zip
zip -rj artifacts.zip artifacts/*