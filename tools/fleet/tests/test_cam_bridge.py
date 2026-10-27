"""Camera bridge, line check and the run_plan camera guard (Phase 0, 2026-10-27).

    python3 -m pytest tools/fleet/tests/test_cam_bridge.py -q             # fast checks (about 10 s)
    FLEET_DRYRUN=1 python3 -m pytest tools/fleet/tests/test_cam_bridge.py -q -k dry   # + the end-to-end dry run (~80 s)
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition"),
          TOOLS):
    if p not in sys.path:
        sys.path.append(p)

import cam_bridge as CB     # noqa: E402
import line_check as LC     # noqa: E402


def test_seg_dist_and_stats():
    assert LC.seg_dist((5, 3), (0, 0), (10, 0)) == pytest.approx(3.0)
    assert LC.seg_dist((-4, 3), (0, 0), (10, 0)) == pytest.approx(5.0)
    s = LC.stats([0.1] * 95 + [5.0] * 5, 3.0)
    assert s["within_gate_pct"] == 95.0 and s["max_cm"] == 5.0


def test_line_check_on_files(tmp_path):
    plan = {"robots": {"Left": {"steps": [
        {"type": "drive", "from": [0, 0, 0], "to": [50, 0, 0]},
        {"type": "turn", "to": [50, 0, 90]},
        {"type": "drive", "from": [50, 0, 90], "to": [50, 50, 90]}]}}}
    (tmp_path / "plan_given.json").write_text(json.dumps(plan))
    (tmp_path / "coordinator.jsonl").write_text(
        json.dumps({"t": 100.0, "event": "step_sent"}) + "\n" +
        json.dumps({"t": 200.0, "event": "step_report", "state": "done"}) + "\n")
    br = tmp_path / "bridge"
    br.mkdir()
    rows = ["t,robot,x,y,status,sent_th,why"]
    for k in range(50):
        rows.append("%.2f,Left,%.2f,%.2f,seen,," % (101 + k, k, 1.0))          # 1 cm beside the first line
    rows.append("150.5,Left,25,40,unsure,,merged")                              # not a sure spot: left out
    (br / "track.csv").write_text("\n".join(rows) + "\n")
    res = LC.check(str(tmp_path), str(br))
    r = res["robots"]["Left"]
    assert r["camera"]["n"] == 50 and r["camera"]["p95_cm"] == pytest.approx(1.0)
    assert r["pass"] and res["pass"]
    assert r["sure_share_pct"] < 100


def test_prediction_follows_own_pose():
    """Who is who comes from the robot's own pose: the prediction is the last sure spot moved by the own pose."""
    import e02_camera as CAM
    import table_cam as TC
    tc = TC.TableCamera((0, 0, 122, 122), seed=3, px_per_cm=2.34)
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "cb_test_cam_%d.json" % os.getpid())
    tc.true_model(path)
    cam = CAM.load_camera(path)
    T = CB.make_tracker_class()
    trk = T(cam, board_center=(61, 61), board_rect=(0, 0, 122, 122))
    trk.add("Left", 30, 30, heading=90)
    tr = trk.tracks["Left"]
    tr.bt = 10.0
    trk.own_pose("Left", 31, 29, 90, 9.9)
    trk.own_pose("Left", 31, 39, 90, 11.0)
    x, y = trk._predict(tr, 11.0)
    assert (x, y) == pytest.approx((30.0 + 0.0, 30.0 + 10 * 1.0 / 1.1), abs=0.05)
    os.remove(path)


def test_outline_heading_near_the_middle():
    """The outline's square near the picture's middle (away from it, perspective makes it unreliable: see
    cam_bridge --start-heading). A median of 8 pictures, as the bridge would take."""
    import cv2
    import e02_camera as CAM
    import table_cam as TC
    tc = TC.TableCamera((0, 0, 122, 122), seed=4, px_per_cm=2.34)
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "cb_test_cam_h_%d.json" % os.getpid())
    tc.true_model(path)
    cam = CAM.load_camera(path)
    T = CB.make_tracker_class()
    trk = T(cam, board_center=(61, 61), board_rect=(0, 0, 122, 122))
    trk.set_background([tc.render([]) for _ in range(3)])
    hs = []
    for _ in range(8):
        img = tc.render([(61.0, 61.0, math.radians(15.0))])
        g = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
        h = CB.outline_heading(trk, g, 61.0, 61.0, 20.0)
        if h is not None:
            hs.append(h)
    assert len(hs) >= 6
    assert abs(CB.wrap180(float(np.median(hs)) - 15.0)) < 4.0
    os.remove(path)


def test_cam_guard_stops_on_disagreement():
    import run_plan as RP

    class Src:
        live = True

    class A:
        cam_guard_cm = 6.0

    c = RP.Coordinator.__new__(RP.Coordinator)
    c.args, c.source, c.abort = A(), Src(), None
    b = RP.Bot("Left")
    b.pose, b.pose_t = (30.0, 30.0, 90.0), RP.now()
    b.last_cam, b.cam_t = [30.0, 40.0, None], RP.now()
    c.bots = {"Left": b}
    said = []
    c.log = lambda *a, **k: None
    c.say = said.append
    c.halt_all = lambda: said.append("HALT")
    c.cam_guard()
    assert c.abort is None                      # first sighting of the gap: wait CAM_GUARD_S
    b.guard_since -= RP.Coordinator.CAM_GUARD_S + 0.1
    b.pose_t = b.cam_t = RP.now()
    c.cam_guard()
    assert c.abort and "disagree" in c.abort and "HALT" in said
    # agreeing again clears it
    c2 = RP.Coordinator.__new__(RP.Coordinator)
    c2.args, c2.source, c2.abort, c2.bots = A(), Src(), None, {"Left": b}
    b.last_cam = [30.5, 30.0, None]
    b.guard_since = None
    c2.log = lambda *a, **k: None
    c2.cam_guard()
    assert b.guard_since is None and c2.abort is None


@pytest.mark.skipif(os.environ.get("FLEET_DRYRUN") != "1", reason="end-to-end dry run: set FLEET_DRYRUN=1")
def test_dry_run_cross_through_the_bridge(tmp_path):
    """Simulated camera pictures -> bridge -> run_plan -> simulated robots: both cross 4 times, within 3 cm."""
    mp = os.path.join(FLEET, "examples", "phase0_cross.json")
    cal = str(tmp_path / "run" / "sim" / "table_camera.json")
    br = subprocess.Popen([sys.executable, os.path.join(FLEET, "cam_bridge.py"), "--map", mp, "--calib", cal,
                           "--stream", "127.0.0.1:8765", "--out", str(tmp_path / "bridge"), "--max-s", "200",
                           "--wait-calib-s", "30"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        time.sleep(1.0)
        rc = subprocess.call([sys.executable, os.path.join(FLEET, "run_plan.py"), "--map", mp, "--dry-run", "--yes",
                              "--session", str(tmp_path / "run"), "--poses-to", "127.0.0.1:5011",
                              "--sim-arg=--fix-hz=0", "--sim-arg=--serve-live=8765",
                              "--sim-arg=--live-calib-out=" + cal], timeout=240,
                             stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    finally:
        br.terminate()
        br.wait(timeout=20)
    assert rc == 0
    res = LC.check(str(tmp_path / "run"), str(tmp_path / "bridge"), str(tmp_path / "run" / "sim" / "truth.csv"))
    assert res["pass"]
    for r in res["robots"].values():
        assert r["camera_vs_truth"]["p95_cm"] < 2.0
    s = json.load(open(tmp_path / "run" / "run_plan_summary.json"))
    assert not s["simulator"]["collision"]
