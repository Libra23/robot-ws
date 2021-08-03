#!/bin/bash

# enable esp
echo "----------esp-idf----------"
source ~/esp/esp-idf/export.sh

#cmake
echo "----------cmake----------"
cmake -B build_esp32 -G Ninja -D PYTHON=python3 -DIDF_TARGET=esp32

#build
echo "----------build----------"
ninja -C build_esp32

#flash
echo "----------flash----------"
cd build_esp32
# ESPPORT=/dev/tty.usbserial-3D528D75A9 ninja flash
# ESPPORT=/dev/tty.usbserial-9D52F70E93 ninja flash
cd ..