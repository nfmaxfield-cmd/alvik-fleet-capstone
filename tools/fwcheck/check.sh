#!/bin/sh
# Firmware syntax check without the Arduino toolchain: compiles firmware/home_lab (or the folder given) with g++
# -fsyntax-only against the small stand-in headers in this folder (Arduino.h, Arduino_Alvik.h, WiFi ...). It
# catches typos and type errors, not behaviour (that is tools/sim/tests). Prints nothing when it is clean.
#   sh tools/fwcheck/check.sh [firmware folder]
H=$(dirname "$(readlink -f "$0")")
F=${1:-$H/../../firmware/home_lab}
D=$(mktemp -d)
cp "$F"/*.h "$D"/ ; cp "$F"/secrets.example.h "$D"/secrets.h
cp "$F"/home_lab.ino "$D"/main.cpp
cd "$D" && g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -I "$H" -include Arduino.h -include WiFi.h main.cpp 2>&1 | head -60
rc=$?
rm -rf "$D"
exit $rc
