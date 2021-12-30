#!/bin/bash

# read parameter
. ./scripts/config.sh

echo "----------monitor----------"
screen ${TARGET_PORT} 115200