#!/bin/bash

#cmake
echo "----------cmake----------"
cd build
cmake ..

#build
echo "----------build----------"
make

#reset
cd ..