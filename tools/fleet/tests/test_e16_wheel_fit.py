"""Build e16 (11 Nov 2026): the school's 17:02 wheel fit (AL-EB54, 0.726 +- 0.20, refused) against its own legs.
The numbers below are the 19 phase-A drives of 20261110-170255-calib/legs.csv (uninterrupted, done, 15 cm or more):
(the robot's own length, the camera's length). One of them, "roll 45 back", has its own length 0: its "started"
report never came back, so its end pose stood in for its start. No robot, no Pi."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "live")):
    if p not in sys.path:
        sys.path.insert(0, p)

import calib_pool as CP   # noqa: E402
import calib_run as CR    # noqa: E402

LEGS_1702 = [(19.89, 20.12), (19.89, 20.20), (44.91, 44.38), (0.00, 44.52), (69.90, 68.61), (19.89, 19.99),
             (19.87, 20.08), (44.80, 44.39), (31.21, 30.91), (56.02, 55.07), (56.04, 55.10), (41.36, 41.14),
             (42.75, 42.54), (68.32, 67.31), (53.94, 53.32), (53.90, 53.29), (35.00, 35.01), (49.52, 48.58),
             (81.03, 79.27)]
DEFAULTS = dict(wheel_scale=1.0, gyro_scale=1.0, gyro_drift_dps=0.0)


def test_the_1702_legs_fit_about_099_once_the_unmeasured_leg_is_out():
    f = CR.fit_values([], [], LEGS_1702, DEFAULTS)
    assert f["rolls_unmeasured"] == 1 and f["n_rolls"] == 18
    assert f["verdict"]["wheel_scale"] == "ok"
    assert 0.96 <= f["wheel_scale"] <= 0.98 and 1.96 * f["se_wheel_scale"] < 0.008
    # what the robot really covers on a 50 cm drive: about 0.99 of it (the day's 126 drives: 0.990)
    assert abs((f["wheel_scale"] * 50 + f["coast_cm"]) / 50 - 0.988) < 0.006


def test_before_the_fix_the_same_legs_gave_the_schools_073():
    b = [r[0] for r in LEGS_1702]
    c = [r[1] for r in LEGS_1702]
    import numpy as np
    X = np.stack([np.array(b), np.ones(len(b))], 1)
    beta = np.linalg.lstsq(X, np.array(c), rcond=None)[0]
    assert abs(beta[0] - 0.726) < 0.002 and abs(beta[1] - 13.5) < 0.1      # the 17:02 fit.json, reproduced


def test_a_lost_started_report_marks_the_leg():
    import threading

    class Link:
        lines = ["D,LEG,-,1,go,45.00,done,7000,44.93,0.04,0.13,front=143.8"]     # no "started" record before it
        cond = threading.Condition()

        def mark(self):
            return 0

        def send(self, ip, cmd):
            pass

        def wait_for(self, pred, k, t):
            for ln in self.lines[k:]:
                m = pred("R", ln)
                if m:
                    return m
            return None
    rb = CR.Robot.__new__(CR.Robot)
    rb.link, rb.ip, rb.name, rb.leg_id = Link(), "1.2.3.4", "R", 0
    rb._index_after = lambda r, k: k + 1
    out = rb.leg("go", 45.0, timeout=1.0)
    assert out["result"] == "done" and out.get("start_missing") is True and out["start"] == out["end"]


def test_the_pool_never_takes_an_unmeasured_leg():
    vals = lambda t: DEFAULTS          # noqa: E731
    bat = lambda t: 80.0               # noqa: E731
    rows = [dict(phase="A", kind="go", result="done", value=45.0, bel_len=0.0, cam_len=44.52, cam0=(0, 0, 0),
                 cam1=(44.5, 0, 0), t0=1.0, bel_ok=False),
            dict(phase="A", kind="go", result="done", value=45.0, bel_len=0.0, cam_len=44.52, cam0=(0, 0, 0),
                 cam1=(44.5, 0, 0), t0=1.0),                       # an old row (no bel_ok): the length guard
            dict(phase="A", kind="go", result="done", value=45.0, bel_len=44.9, cam_len=44.4, cam0=(0, 0, 0),
                 cam1=(44.4, 0, 0), t0=1.0, bel_ok=True)]
    got = CP.raw_rows(rows, vals, bat, "s", lambda r: (0, 0, 0))
    assert len(got) == 1 and abs(got[0]["counted"] - 44.9) < 1e-9
