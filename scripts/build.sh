#!/bin/bash

#cmake
echo "----------cmake----------"
cd build
cmake ..

#build
echo "----------build----------"
cmake --build .

#reset
cd ..