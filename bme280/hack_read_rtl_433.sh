#!/bin/bash
../rtl_433/src/rtl_433 -A 2>&1 | stdbuf -o0 grep '\[00].*72} 02' | stdbuf -o0 sed 's/\[00].*72} 02//g' | ./calculate.py 1.calib
