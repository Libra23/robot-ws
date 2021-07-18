#!/bin/bash

#cmake
echo "----------cmake----------"
cmake -B build

#build
echo "----------build----------"
cmake --build build

for opt in "$@"
do
    case $opt in
        '-c')
            arg_f = 1
            ;;
        '-l')
            arg_l = 1
            ;;
    esac
done
