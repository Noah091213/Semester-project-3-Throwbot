#!/bin/bash

# 1. Define Paths
MATLAB_BIN="/home/noahv/Programs/MatLab/bin/glnxa64"

# 2. FORCE load the MATLAB ICU libraries into memory first
export LD_PRELOAD="${MATLAB_BIN}/libicuuc.so.74:${MATLAB_BIN}/libicui18n.so.74:${MATLAB_BIN}/libicudata.so.74"

# 3. Run the executable
./Throwbot
