#!/bin/bash

cd build/test
ctest --output-on-failure
cd ../..