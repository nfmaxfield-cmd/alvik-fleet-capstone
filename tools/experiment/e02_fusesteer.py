"""Steering from the combined camera + wheels + gyro estimate (2026-10-25 evening, MIRLT; Nick approved 19:03).
OFF unless the runner is started with --fusion-steer: it has not run on the real board yet.

Today the combined estimate (e02_live.LiveLab, the fusion EKF) only runs on the live lab page, display only. On the
real sessions (docs/build-reports/e02-oct25-real-fusion.md) it was 15-30 % closer than wheels + gyro after a camera
gap (2.2 against 2.7 cm after 5 s, 5.1 against 7.3 after 40 s), and closer than the camera itself near the boards,
where the camera's position jumps 5-8 cm (3.6 against 5.1 cm at find ends). On routes with the camera seeing the
robot, the camera alone was as good or better (2 s gaps: 2.4 cm fused against 1.7). So the combined estimate is used
where it was better, and the camera everywhere else:

  camera fresh (a position at most CAM_FRESH_S old, from the robot's outline or its lock):
     the camera, unless the robot is within NEAR_BOARD_CM of a blocked board AND the combined estimate disagrees by
     more than DISAGREE_CM: then the combined estimate (the camera's board-side jump);
  camera stale or missing (a gap):
     the combined estimate, if it is fresh (FUSE_FRESH_S), its heading is known and it is sure (SD_MAX_CM);
  else nothing (as today: the caller finds the robot again).

choose() is a pure function (tests: tests/test_e02_tracker_process.py); fused_now() reads the lab.
"""
import math
import time

# Chosen numbers (DOE notes: docs/doe-backlog.md rows "Fusion steering").
CAM_FRESH_S = 0.5        # chosen: a camera position this old is still "the camera sees it" (the runner's pictures are
                         # 0.16 s old at the median, 0.30 at the 95th percentile on 15:45). DOE: replay with the camera
                         # withheld for 0.3-1.0 s, steering error of each rule
FUSE_FRESH_S = 0.3       # chosen: the combined estimate must have been updated this recently (RAW rows come ~10 a
                         # second while a robot drives). No DOE: freshness guard
SD_MAX_CM = 3.0          # chosen: the combined estimate's 1-sigma position spread must be below this. DOE: the real
                         # fusion replay's gaps, error against reported spread (is the spread honest?)
NEAR_BOARD_CM = 15.0     # the real fusion report's own figure: the camera jumps within about 15 cm of a board
DISAGREE_CM = 3.0        # chosen: below this the two agree and the camera is kept. DOE: same replay, 2/3/5 cm


def fused_now(lab, robot, now=None):
    """The lab's latest combined estimate for `robot`: dict(t, x, y, sd_cm, heading_known), or None."""
    if lab is None:
        return None
    now = time.time() if now is None else now
    try:
        with lab.lock:
            if lab.f is None or lab.robot != robot or not lab.track:
                return None
            t, x, y, sxx, syy, _ = lab.track[-1]
            return dict(t=float(t), x=float(x), y=float(y), sd_cm=math.sqrt(max(0.0, 0.5 * (sxx + syy))),
                        heading_known=bool(lab.hdg_known))
    except Exception:
        return None


def choose(cam, fused, near_board_cm=None, now=None):
    """The position to steer by. cam: the camera's (t, x, y) or None; fused: fused_now()'s dict or None;
    near_board_cm: the robot's distance to the nearest blocked board (None: unknown). Returns (t, x, y, why) or None;
    why is 'camera', 'fused (camera gap)' or 'fused (camera near a board)'."""
    now = time.time() if now is None else now
    f_ok = (fused is not None and now - fused["t"] <= FUSE_FRESH_S and fused["heading_known"]
            and fused["sd_cm"] <= SD_MAX_CM)
    cam_ok = cam is not None and now - cam[0] <= CAM_FRESH_S
    if cam_ok:
        if f_ok and near_board_cm is not None and near_board_cm <= NEAR_BOARD_CM and \
                math.hypot(fused["x"] - cam[1], fused["y"] - cam[2]) > DISAGREE_CM:
            return (fused["t"], fused["x"], fused["y"], "fused (camera near a board)")
        return (cam[0], cam[1], cam[2], "camera")
    if f_ok:
        return (fused["t"], fused["x"], fused["y"], "fused (camera gap)")
    return None
