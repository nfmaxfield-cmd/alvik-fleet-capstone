#!/bin/sh
# Host test of the four-sided virtual border (firmware/home_lab/border.h, 2026-10-27a) with a stand-in P() and pose:
# 11 cases (kinds, the partial stretch, the follower's towards / along / away rule). Not the Arduino toolchain.
H=$(dirname "$(readlink -f "$0")")
D=$(mktemp -d)
grep -v '#include "pose.h"\|#pragma once' "$H/../../firmware/home_lab/border.h" > "$D/border_only.h"
cp "$H/border_test.cpp" "$D/t.cpp"
g++ -std=gnu++17 -o "$D/t" "$D/t.cpp" -I "$D" && "$D/t"
rc=$?
rm -rf "$D"
exit $rc
