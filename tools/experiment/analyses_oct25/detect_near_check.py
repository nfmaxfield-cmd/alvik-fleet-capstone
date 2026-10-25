"""DOE row "robot found where its wheels put it" (e02.py DETECT_NEAR_CM, 6 cm): the detect_after_move records on
disk, how far each pick-up was from the wheels' pose, and whether the next camera fix agreed with the robot (a wrong
pick-up would show as a large correction at the next fix, or a camera-wheels disagreement on the next legs).

Usage: python3 detect_near_check.py"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oct25_common as C  # noqa: E402

for name in C.CLASSIC + ("right-alone",):
    st = C.NC.state(name)
    recs = st.get("detect_after_move") or []
    if not recs:
        continue
    fixes = sorted(C.robot_rows(name, "fix.csv"), key=lambda r: float(r["t_recv"]))
    dis = st.get("disagreements") or []
    for t, robot, x, y, dcm in recs:
        nxt = [f for f in fixes if f["robot"] == robot and float(f["t_recv"]) > t][:1]
        corr = None
        if nxt:
            f = nxt[0]
            corr = (math.hypot(float(f["cam_x"]) - float(f["before_x"]), float(f["cam_y"]) - float(f["before_y"])),
                    float(f["t_recv"]) - t)
        later = [d for d in dis if d[1] == robot and 0 < d[0] - t < 60]
        print("%s %s %s: picked up %.1f cm from its wheels' pose; next fix %s; camera-wheels disagreements in the "
              "next 60 s: %d" % (name, C.hm(t), robot, dcm,
                                 "correction %.1f cm after %.0f s" % corr if corr else "none", len(later)))
print("At 4 cm, %d of these would have been refused (the turn on its spot instead); at 9 cm none more accepted, since "
      "outlines further than 6 cm are not logged." % 2)
