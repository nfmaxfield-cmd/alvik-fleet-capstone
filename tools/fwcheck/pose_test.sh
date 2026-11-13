#!/bin/sh
# Host test of the own pose's camera-fix filter (firmware/home_lab/pose.h, namespace pose; 2026-11-12e21) with a stand-in
# P(), clock, gyro and wheels: the age limit, the gate, the history, refusal reasons, the linear sd growth, the
# consistency escape, and the old behaviour behind the params. Not the Arduino toolchain.
#   sh tools/fwcheck/pose_test.sh
# Changelog: 2026-11-12e21 first version (e21).
H=$(dirname "$(readlink -f "$0")")
D=$(mktemp -d)
awk '/^namespace pose \{/{on=1} on{print} /^\}  \/\/ namespace pose/{on=0}' "$H/../../firmware/home_lab/pose.h" > "$D/pose_only.h"
cp "$H/pose_test.cpp" "$D/t.cpp"
g++ -std=gnu++17 -o "$D/t" "$D/t.cpp" -I "$D" && "$D/t"
rc=$?
rm -rf "$D"
exit $rc
