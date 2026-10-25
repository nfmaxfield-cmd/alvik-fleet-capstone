"""DOE row "camera drift watch" (e02_camera CameraShiftWatch PERIOD_S 5 s / WARN_PX 1 px): how big the measured
camera shifts were in the sessions that logged them (state camera_shift: t, dx px, dy px, alignment score), what
that is on the board, how fast it changed (what a longer period would have missed), and how it compares with the
board-line touches' own spread (state board_touches).

Usage: python3 camera_shift_check.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

CM_PER_PX = (0.399, 0.455)   # the roam1 calibration's scale across the board (night analyses, B3)

for name in ("stage1", "stage2"):
    st = C.NC.state(name)
    cs = st.get("camera_shift") or []
    if not cs:
        continue
    t = np.array([c[0] for c in cs]); dx = np.array([c[1] for c in cs]); dy = np.array([c[2] for c in cs])
    mag = np.hypot(dx, dy)
    print("%s: %d measurements over %.0f min; shift max %.2f px (%.2f-%.2f cm on the board), at the end %.2f px; "
          "alignment score min %.3f; over WARN_PX 1: %d" % (
              name, len(cs), (t[-1] - t[0]) / 60, mag.max(), mag.max() * CM_PER_PX[0], mag.max() * CM_PER_PX[1],
              mag[-1], min(c[3] for c in cs), int(np.sum(mag > 1.0))))
    for period in (5, 15, 30, 60, 120):
        step = max(1, int(round(period / np.median(np.diff(t)))))
        ch = [math.hypot(dx[i + step] - dx[i], dy[i + step] - dy[i]) for i in range(0, len(t) - step)]
        print("   change within %3d s: median %.3f px, max %.3f px" % (period, np.median(ch), max(ch)))
    bt = st.get("board_touches") or []
    for b in ("x", "y"):
        v = [x[3] for x in bt if x[2] == b]
        if len(v) >= 3:
            print("   %s-board touches: n %d, spread (sd) %.2f cm, range %.2f cm" % (b, len(v), np.std(v, ddof=1),
                                                                                max(v) - min(v)))
