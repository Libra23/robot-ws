#!/bin/bash

for opt in "$@"
do
    case "$opt" in
        '-c')
            arg_c=1
            ;;
        '-f')
            arg_f=1
            ;;
    esac
done

if [[ ${arg_c} -eq 1 ]]; then
    echo "----------clean----------"
    rm -rf build_esp32
    mkdir build_esp32
fi

# enable esp
echo "----------esp-idf----------"
source ~/esp/esp-idf/export.sh

#cmake
echo "----------cmake----------"
cmake -B build_esp32 -G Ninja -D PYTHON=python3 -DIDF_TARGET=esp32

#build
echo "----------build----------"
ninja -C build_esp32

if [[ ${arg_f} -eq 1 ]]; then
    echo "----------flash----------"
    cd build_esp32
    # ESPPORT=/dev/tty.usbserial-3D528D75A9 ninja flash
    ESPPORT=/dev/tty.usbserial-9D52F70E93 ninja flash
    cd ..
fi