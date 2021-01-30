#!/bin/bash

# enable esp
echo "----------esp-idf----------"
source ~/esp/esp-idf/export.sh

#cmake
echo "----------cmake----------"
cmake -B build_esp32 -G Ninja

#build
echo "----------build----------"
# cmake --build build_esp32
ninja -C build_esp32

#flash
echo "----------flash----------"
cd build_esp32
ESPPORT=/dev/tty.usbserial-3D528D75A9 ninja flash
cd ..