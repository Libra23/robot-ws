#!/bin/bash

#cmake
echo "----------cmake----------"
cmake -B build

#build
echo "----------build----------"
cmake --build build

#reset
cd ..