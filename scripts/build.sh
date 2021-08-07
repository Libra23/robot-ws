#!/bin/bash

for opt in "$@"
do
    case "$opt" in
        '-c')
            arg_c=1
            ;;
    esac
done

if [[ ${arg_c} -eq 1 ]]; then
    echo "----------clean----------"
    rm -rf build
    mkdir build
fi

#cmake
echo "----------cmake----------"
cmake -B build -G Ninja

#build
echo "----------build----------"
ninja -C build