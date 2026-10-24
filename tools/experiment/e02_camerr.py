"""The camera's own error, place by place (2026-10-24 review: around the board's middle the camera put every robot
1.4-3.6 cm further along x than the robot believed, all three robots alike; KC's camera fixes pulled the robots
towards that error).

Samples are places where the true position is known some other way than the camera:
  - 'mark': a robot put by hand on a tape mark measured from the boards' faces (the strongest: weight MARK_W);
  - 'find': where a robot stands right after a find, its pose set by touching both boards (weight FIND_W).
Each sample is (x, y, dx, dy): the camera's position minus the true one, cm, in the board frame.

correction(x, y) -> (dx, dy, sd): a Gaussian-kernel average of the samples near (x, y) (BANDWIDTH_CM), and how
sure that is (sd, cm). Far from every sample there is no correction and sd is NO_DATA_SD_CM. The camera position
corrected = camera - (dx, dy).

All the numbers here are chosen, not measured (DOE notes beside them).
Pure functions and a small class; unit-tested in tests/test_e02_camerr.py."""
import json
import math

BANDWIDTH_CM = 25.0     # chosen: about the spacing of the tape marks; a DOE (leave-one-mark-out error at 15/25/40)
MARK_W = 4.0            # chosen: a hand-placed robot on a measured mark (about 0.5 cm) against a find (about 1 cm)
FIND_W = 1.0
NOISE_SD_CM = 0.7       # chosen: one still camera position's scatter (the KC replay's still pictures: 0.3-1 cm)
NO_DATA_SD_CM = 3.0     # chosen: the camera's error where nothing has been checked (session 1: far points 1.4-3.6)
MIN_WEIGHT = 0.5        # below this much kernel weight a place counts as unchecked

# Where to put the tape marks: (name, cm from the y-board's face, cm from the x-board's face). Along the diagonal
# the routes run on (start, L far point, square far point, straight far point) and two off it, so the map covers
# the board. Session 1's routes: start about 28.6 / 28.5 cm from the faces; far points L 58.9 / 58.4, square
# 79.0 / 78.3, straight 100.1 / 98.3 (e02.csv far_goal against the board lines).
MARKS = [("A", 30.0, 30.0), ("B", 60.0, 60.0), ("C", 80.0, 80.0), ("D", 100.0, 100.0), ("E", 30.0, 90.0),
         ("F", 90.0, 30.0), ("G", 50.0, 50.0)]


def mark_xy(board, from_y_board_cm, from_x_board_cm):
    """A mark's board-frame position from its distances to the boards' faces. The y-board's face is the line
    x = board.yboard_cm, the x-board's y = board.xboard_cm; the board's inside is towards its middle."""
    ys, xs = getattr(board, "yboard_side", None), getattr(board, "xboard_side", None)
    sx = (1.0 if ys else -1.0) if ys is not None else (1.0 if board.yboard_cm > 61.0 else -1.0)
    sy = (1.0 if xs else -1.0) if xs is not None else (1.0 if board.xboard_cm > 61.0 else -1.0)
    return board.yboard_cm - sx * from_y_board_cm, board.xboard_cm - sy * from_x_board_cm


class CameraErrorMap:
    def __init__(self, samples=None):
        self.samples = list(samples or [])      # dicts: x, y, dx, dy, w, source, t, robot

    def add(self, x, y, cam_x, cam_y, source, robot="", t=None):
        w = MARK_W if source == "mark" else FIND_W
        s = dict(x=round(x, 2), y=round(y, 2), dx=round(cam_x - x, 2), dy=round(cam_y - y, 2), w=w, source=source,
                 robot=robot, t=t)
        self.samples.append(s)
        return s

    def correction(self, x, y):
        """(dx, dy, sd) of the camera at (x, y)."""
        if not self.samples:
            return 0.0, 0.0, NO_DATA_SD_CM
        tot = sx = sy = 0.0
        for s in self.samples:
            k = s["w"] * math.exp(-0.5 * ((s["x"] - x) ** 2 + (s["y"] - y) ** 2) / BANDWIDTH_CM ** 2)
            tot += k
            sx += k * s["dx"]
            sy += k * s["dy"]
        if tot < MIN_WEIGHT:
            return 0.0, 0.0, NO_DATA_SD_CM
        dx, dy = sx / tot, sy / tot
        # spread of the samples about the average, and how much they weigh: the less data, the less sure
        var = 0.0
        for s in self.samples:
            k = s["w"] * math.exp(-0.5 * ((s["x"] - x) ** 2 + (s["y"] - y) ** 2) / BANDWIDTH_CM ** 2)
            var += k * ((s["dx"] - dx) ** 2 + (s["dy"] - dy) ** 2) / 2.0
        var /= tot
        sd = math.sqrt(NOISE_SD_CM ** 2 + var + (NO_DATA_SD_CM ** 2) * MIN_WEIGHT / (tot + MIN_WEIGHT))
        return dx, dy, min(sd, NO_DATA_SD_CM)

    def corrected(self, x, y):
        dx, dy, sd = self.correction(x, y)
        return x - dx, y - dy, sd

    def to_list(self):
        return list(self.samples)

    def summary(self):
        """Plain numbers for the session log: samples by source, and the error's size at the marks."""
        out = {}
        for s in self.samples:
            out.setdefault(s["source"], []).append(math.hypot(s["dx"], s["dy"]))
        return {k: dict(n=len(v), median_cm=round(sorted(v)[len(v) // 2], 2), max_cm=round(max(v), 2))
                for k, v in out.items()}


def blend_weight(robot_sd_cm, camera_sd_cm):
    """How far towards the camera a fix moves the robot's own position (0 = ignore the camera, 1 = take it):
    each side weighted by how sure it is (the variance-weighted average, as a Kalman filter does for one step)."""
    vr, vc = robot_sd_cm ** 2, camera_sd_cm ** 2
    return vr / (vr + vc) if vr + vc > 0 else 0.5


def load(path):
    with open(path) as f:
        return CameraErrorMap(json.load(f))
