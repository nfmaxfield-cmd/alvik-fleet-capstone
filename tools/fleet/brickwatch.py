"""Brick drop (school demo, docs/build-plans/mission-control.md item 5, 2026-10-27): the overhead camera's "what
changed" watch during a run. Something new standing still on the table (a brick Nick drops) becomes a planner
obstacle (run_plan.py's {"type": "obstacle"} message), so the robot that would meet it stops, the others drive on, and
routes and the mower's lanes go round it.

How (the brick demo's floor memory, tools/experiment/e02_bricks.py and e02_camera.MotionTracker):
  - the floor memory is the table as it looked at the start, with the robots' spots marked unknown (filled in as
    they leave: learn_free);
  - every check (CHECK_HZ), the picture is compared with it (e02_bricks.change_mask), the robots masked out where the
    camera puts them (ROBOT_MASK_SCALE robot sizes round each);
  - a changed patch that reaches the picture's edge or the table's outline, or runs out past the table, is a hand
    and arm (or a person leaning in): IGNORED, and nothing new is confirmed while it is there;
  - a changed patch on the table that stays put (centre within STILL_CM) for CONFIRM_CHECKS checks in a row, with no
    hand in the picture, is confirmed: its floor cells (e02_bricks.occupancy, projected at z = 0: conservative)
    become an obstacle polygon (their outline's convex hull, grown by GROW_CM);
  - a confirmed object is not reported again; anything later that overlaps it is part of it.
"""
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "experiment"), os.path.join(TOOLS, "recognition"), TOOLS):
    if p not in sys.path:
        sys.path.append(p)          # after the caller's own modules (the simulator has its own config.py, camera.py)

import e02_bricks as BR    # noqa: E402

CHECK_HZ = 4.0           # chosen: 4 checks a second (the Pi's tracker runs 13-25 pictures a second; a check costs about
                         # one tracker picture). With CONFIRM_CHECKS 3 a brick is confirmed ~0.75 s after the hand
                         # leaves. DOE note: 2 / 4 / 8 Hz x 2 / 3 / 5 checks on recorded drops (time to confirm against
                         # false objects per hour); docs/doe-backlog.md
CONFIRM_CHECKS = 3       # chosen: see CHECK_HZ
STILL_CM = 3.0           # chosen: a patch's centre may move this much between checks and still be "the same, standing
                         # still" (the camera's ~1 cm noise on a patch's centre, plus a brick settling). No DOE: a hand
                         # moves far more
ROBOT_MASK_SCALE = 1.6   # chosen: robot sizes masked round each robot's camera spot (1.5 in the brick demo, + room for
                         # the camera's 0.1 s lag at 10 cm/s). DOE note: with the drop dry runs, false objects at robots'
                         # trailing edges at 1.3 / 1.6 / 2.0
GROW_CM = 1.0            # the polygon round the cells: half a 1 cm cell plus 0.5 cm (geometry, no DOE)
RIM_PX = 3               # a patch within this many pixels of the picture's edge comes from outside it (a hand)
MIN_AREA_CM2 = BR.MIN_BRICK_CM2


class BrickWatch:
    def __init__(self, cam, rect, send=None, log=None):
        """cam: a camera model (project((x, y, z)), px_per_cm(x, y, z)); rect: the table (x0, y0, x1, y1) floor cm;
        send(dict): where obstacle messages go (run_plan's UDP source); log(str)."""
        import e02_camera as CAM
        self.cam, self.rect = cam, tuple(float(v) for v in rect)
        self.tracker = CAM.MotionTracker(cam, board_rect=self.rect)
        self.floor = None
        self.send, self.log = send or (lambda m: None), log or (lambda s: None)
        self.cands = []           # [dict(c=(x, y), n, area, cells)]
        self.confirmed = []       # [dict(id, polygon, t)]
        self.hand_seen = 0
        self.checks = 0
        self.last_t = -1e9
        self.shape = None
        self.board_px = None

    # -- the floor memory --------------------------------------------------------
    def start(self, frames, robots):
        """Learn the table from a few pictures (median) with the robots standing at robots [(x, y)]."""
        self.tracker.set_background(frames, robots=robots)
        self.floor = BR.floor_snapshot(self.tracker)
        self.shape = frames[0].shape[:2]
        self.board_px = BR.board_mask(self.cam, self.shape, self.rect)
        inner = cv2.erode(self.board_px, np.ones((2 * RIM_PX + 1, 2 * RIM_PX + 1), np.uint8))
        self.rim_px = (self.board_px > 0) & (inner == 0)

    def learn_free(self, frame, robots, changed):
        """Floor that was unknown (a robot stood there at the start) and is now clear of robots and unchanged: learn it."""
        unk = self.floor["known"] == 0
        if not unk.any():
            return
        ign = BR.robot_mask(self.cam, frame.shape, robots, scale=ROBOT_MASK_SCALE + 0.4)
        g = cv2.GaussianBlur(frame, (3, 3), 0).astype(np.float32)
        ok = unk & (ign == 0) & (changed == 0)
        self.floor["bg"][ok] = g[ok] - self.floor.get("offset", 0.0)
        self.floor["known"][ok] = 1

    # -- each picture ------------------------------------------------------------
    def check(self, frame, robots, t=None):
        """One picture (grey) and the robots' camera spots [(x, y)]: returns the obstacles confirmed by this check."""
        t = time.time() if t is None else t
        if self.floor is None or t - self.last_t < 1.0 / CHECK_HZ - 1e-6:
            return []
        self.last_t = t
        self.checks += 1
        ign = BR.robot_mask(self.cam, frame.shape, robots, scale=ROBOT_MASK_SCALE)
        mask = BR.change_mask([frame], self.floor, ignore=ign)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        H, W = mask.shape
        hand = False
        keep = np.zeros_like(mask)
        for k in range(1, n):
            x, y, w, h = (int(stats[k, j]) for j in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH,
                                                     cv2.CC_STAT_HEIGHT))
            blob = lab == k
            at_edge = x <= RIM_PX or y <= RIM_PX or x + w >= W - RIM_PX or y + h >= H - RIM_PX
            off_table = bool((blob & (self.board_px == 0)).sum() > 0.1 * blob.sum())
            on_rim = bool((blob & self.rim_px).any())
            if at_edge or off_table or on_rim:
                hand = True          # a hand and arm reaching in (or a person leaning over): never an obstacle
                continue
            keep[blob] = 255
        self.learn_free(frame, robots, mask)
        if hand:
            self.hand_seen += 1
            self.cands = []          # nothing is confirmed while a hand is in the picture (it may still be moving it)
            return []
        grid = BR.Grid(self.rect, 1.0)
        occ = BR.occupancy(self.cam, keep, grid, min_area_cm2=MIN_AREA_CM2)
        nb, lab2, st2, _ = cv2.connectedComponentsWithStats(occ.astype(np.uint8), connectivity=8)
        X, Y = grid.centres()
        seen = []
        for k in range(1, nb):
            cells = lab2 == k
            area = float(st2[k, cv2.CC_STAT_AREA])
            if area < MIN_AREA_CM2:
                continue
            c = (float(X[cells].mean()), float(Y[cells].mean()))
            if any(self._inside(c, o["polygon"]) for o in self.confirmed):
                continue
            seen.append(dict(c=c, area=area, cells=cells))
        new_c, out = [], []
        for s in seen:
            prev = next((p for p in self.cands if math.dist(p["c"], s["c"]) <= STILL_CM), None)
            s["n"] = (prev["n"] + 1) if prev else 1
            if s["n"] >= CONFIRM_CHECKS:
                out.append(self._confirm(s, X, Y, t))
            else:
                new_c.append(s)
        self.cands = new_c
        return out

    def _confirm(self, s, X, Y, t):
        pts = np.column_stack([X[s["cells"]], Y[s["cells"]]]).astype(np.float32)
        hull = cv2.convexHull(pts).reshape(-1, 2)
        c = hull.mean(axis=0)
        grown = [(float(p[0] + (p[0] - c[0]) / max(1e-6, np.linalg.norm(p - c)) * GROW_CM),
                  float(p[1] + (p[1] - c[1]) / max(1e-6, np.linalg.norm(p - c)) * GROW_CM)) for p in hull]
        ob = {"type": "obstacle", "id": "camera-%d" % (len(self.confirmed) + 1), "shape": "polygon",
              "points": [[round(x, 2), round(y, 2)] for x, y in grown], "source": "camera",
              "area_cm2": round(s["area"], 1), "t": t}
        self.confirmed.append({"id": ob["id"], "polygon": ob["points"], "t": t})
        self.log("brick watch: new object %s at (%.1f, %.1f), %.0f cm2, after %d still checks"
                 % (ob["id"], s["c"][0], s["c"][1], s["area"], s["n"]))
        self.send(ob)
        return ob

    @staticmethod
    def _inside(p, poly):
        return cv2.pointPolygonTest(np.array(poly, np.float32), (float(p[0]), float(p[1])), False) >= 0
