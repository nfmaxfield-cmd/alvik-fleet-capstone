#!/usr/bin/env python3
"""The fleet simulator (tools/sim/fleet_sim.py) with the school's conditions added, for the Monday sweep
(tools/fleet/sweeps/monday_sweep.py, 2026-10-27). Takes every fleet_sim.py argument, plus --sweep-cond FILE (JSON):

  drop_pct        lost packets, % each way (commands to a robot and its records back), drawn per packet, per robot,
                  from the run's seed (fleet_sim's own hook, Radio._random_drop)
  cam_stall_s     the whole camera stops for this long (no fixes for anyone), every cam_stall_every_s, first at
                  cam_stall_first_s after the plan starts (the brick watch's pictures stall with it)
  stall_watch     "skip" (default): the brick watch makes no check while the camera is stalled; "stale": it checks
                  the repeated picture against the robots' newest spots
  cam_age_s       picture age: the fixes arrive this late (fleet_sim --fix-delay) and so does the brick watch's report
  cam_tilt        {"tilt_deg", "height_cm", "led_cm", "corrected", "height_err_pct", "nadir_err_cm"}: the camera on a
                  stand at the audience side (y_min), tilted towards the table's centre. The robot's top lights are
                  led_cm up, so the camera sees them pushed away from the point under the lens by led/(H - led) of
                  their distance from it. corrected = the floor calibration knows the camera's height and spot, but
                  height_err_pct / nadir_err_cm off; else no height correction (tools/live/floormap.py's "no camera
                  height" case). Noise grows with range: sigma x (range / 262 cm)^2 / cos(view angle). The brick
                  watch's picture is tilted too (table_cam tilt), its own calibration perfect.
  cam_px_per_cm   the brick watch's picture scale (table_cam's default 3.4 px/cm, the home camera at 262 cm, sees
                  188 x 141 cm: a 122 cm deep table runs off the picture); lower = the camera higher or wider
  robot_out       {"name", "t", "mode"}: at t s after the plan starts the robot is out. "dead": stops where it is and
                  is never heard again (battery or WiFi gone); "lifted": picked up off the table (the firmware's
                  safe_lift ends its run; the camera no longer sees it)

The wheel scale needs no code here: fleet_sim --config with robot.radius_scale_err_pct_by_robot.
"""
import json
import math
import os
import random
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(os.path.dirname(HERE))
SIM = os.path.join(TOOLS, "sim")
for p in (SIM, os.path.join(TOOLS, "fleet")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fleet_sim as FS   # noqa: E402

REF_RANGE_CM = 262.0     # the home camera's height, where fleet_sim's 0.3 cm fix noise comes from (README: 262 cm)


def tilt_model(cond, floor):
    """(bias(x, y) -> (dx, dy), sigma_scale(x, y)) for a camera on the y_min side tilted towards the table centre."""
    ct = cond["cam_tilt"]
    a = math.radians(ct["tilt_deg"])
    Hc = float(ct["height_cm"])
    h = float(ct.get("led_cm", 4.0))
    cx = (floor["x_min"] + floor["x_max"]) / 2.0
    cy = (floor["y_min"] + floor["y_max"]) / 2.0
    nx, ny = cx, cy - Hc * math.tan(a)                  # the floor point under the lens

    def push(x, y, Hc_, nx_, ny_):
        k = h / (Hc_ - h)
        return (x - nx_) * k, (y - ny_) * k

    def bias(x, y):
        bx, by = push(x, y, Hc, nx, ny)
        if ct.get("corrected"):
            ex, ey = push(x, y, Hc * (1 + ct.get("height_err_pct", 10.0) / 100.0), nx, ny - ct.get("nadir_err_cm", 10.0))
            return bx - ex, by - ey
        return bx, by

    def sigma_scale(x, y):
        d = math.hypot(x - nx, y - ny)
        rng = math.hypot(d, Hc)
        return (rng / REF_RANGE_CM) ** 2 / (Hc / rng)
    return bias, sigma_scale


def install(sim, cond, seed):
    a = sim.args
    f = sim.m["floor"]
    # ---- lost packets
    pct = float(cond.get("drop_pct", 0.0) or 0.0)
    if pct > 0:
        sim.drop_pct = lambda direction: pct
        for k, rb in enumerate(sim.robots.values()):
            rb.radio.drop_rng = random.Random(seed * 1000 + 17 * k + 3)
    # ---- camera stalls (everyone at once)
    st = float(cond.get("cam_stall_s", 0.0) or 0.0)
    every = float(cond.get("cam_stall_every_s", 20.0))
    first = float(cond.get("cam_stall_first_s", 7.0))
    orig_gap = sim.cam_gap

    def stalled(now):
        if st <= 0 or sim.t_plan0 is None:
            return False
        t = now - sim.t_plan0 - first
        return t >= 0 and (t % every) < st
    sim.cam_gap = lambda name, now: orig_gap(name, now) or stalled(now)
    sim.cam_stalled = stalled
    # ---- picture age (fixes: fleet_sim --fix-delay; the brick watch's report: here)
    age = float(cond.get("cam_age_s", 0.0) or 0.0)
    tilt = cond.get("cam_tilt")
    bias = sig = None
    if tilt:
        bias, sig = tilt_model(cond, f)
    trng = random.Random(seed * 7919 + 5)
    orig_send = sim.send_json

    def send_json(msg):
        if msg.get("type") == "fix" and bias is not None:
            # fleet_sim made the fix from the true pose + its own 0.3 cm noise: add the tilted camera's push and the
            # extra noise of a longer, slanted view
            x, y = msg["x"], msg["y"]
            bx, by = bias(x, y)
            s = max(0.0, sig(x, y) - 1.0) * a.fix_sigma
            msg = dict(msg, x=x + bx + trng.gauss(0, s), y=y + by + trng.gauss(0, s))
        if msg.get("type") == "obstacle" and age > 0:
            threading.Timer(age, orig_send, args=(msg,)).start()
            return
        orig_send(msg)
    sim.send_json = send_json
    pxcm = cond.get("cam_px_per_cm")
    if tilt or st > 0 or pxcm:
        # the brick watch's camera: tilted like the robots' camera; its pictures stall with it
        import table_cam as TC
        TCcls = TC.TableCamera
        tdeg = float(tilt["tilt_deg"]) if tilt else None

        class TiltedCam(TCcls):
            def __init__(self, rect, seed=5, **kw):
                if tdeg is not None:
                    kw["tilt"] = tdeg
                if pxcm:
                    kw["px_per_cm"] = float(pxcm)
                super().__init__(rect, seed=seed, **kw)
                self._last = None

            def render(self, robots=(), boxes=(), hand=None):
                if self._last is not None and sim.cam_stalled(time.time()):
                    return self._last                     # a stalled camera repeats its last picture
                self._last = super().render(robots, boxes, hand)
                return self._last
        TC.TableCamera = TiltedCam
        if tilt:
            # aim the tilted camera at the table: table_cam puts the lens straight above center_floor_cm and tilts
            # about it, which at 35-50 deg looks past the table (first tilt runs: the watch never saw the table).
            # One tilt axis, the lens at the cond's height, moved along y until the table's centre is mid-picture
            import numpy as np
            OrigG = TC.SC.CameraGeometry

            class AimedGeometry(OrigG):
                def __init__(self, c):
                    c.tilt_deg = [-float(tilt["tilt_deg"]), 0.0]     # sign: the lens on the y_min (audience) side, as the fixes' model
                    c.height_cm = float(tilt["height_cm"])
                    tx, ty = c.center_floor_cm
                    best = None
                    for dy in np.arange(-500.0, 500.0, 1.0):
                        c.center_floor_cm = [tx, ty + dy]
                        g = OrigG(c)
                        dz = g.Hc
                        pz = g.R[2, 0] * 0.0 + g.R[2, 1] * (-(ty - g.Y0)) + g.R[2, 2] * dz
                        if pz <= 0:
                            continue
                        u, v = g.project(tx, ty)
                        err = abs(v - g.cy) + abs(u - g.cx)
                        if best is None or err < best[0]:
                            best = (err, dy)
                    c.center_floor_cm = [tx, ty + best[1]]
                    super().__init__(c)
            TC.SC.CameraGeometry = AimedGeometry
        if st > 0 and cond.get("stall_watch", "skip") == "skip":
            # no new picture, no check: the watch and the tracker share the camera's pictures, so a stalled camera
            # gives the watch nothing new (the "stale" mode instead checks the repeated picture against the robots'
            # newest spots: robots' old images left unmasked)
            import brickwatch as BW
            orig_check = BW.BrickWatch.check

            def check(self, frame, robots, t=None):
                if sim.cam_stalled(time.time()):
                    return []
                return orig_check(self, frame, robots, t)
            BW.BrickWatch.check = check
    # ---- one robot out
    out = cond.get("robot_out")
    if out:
        rb = sim.robots[out["name"]]
        mode = out.get("mode", "dead")
        state = {"done": False}
        orig_adv = rb.advance
        orig_phys = rb._physics

        def advance(dt_ms, busy=False):
            if not state["done"] and sim.t_plan0 is not None and time.time() - sim.t_plan0 >= float(out["t"]):
                state["done"] = True
                sim.event(dict(kind="robot_out", robot=rb.name, mode=mode, t=round(time.time() - sim.t_plan0, 2)))
                if mode == "lifted":
                    rb.start_lift(-200.0, -200.0, rb.th, 1e9, "taken off the table")
                else:
                    now = time.time()
                    sim.drops.append((rb.name, now, now + 1e6))
            orig_adv(dt_ms, busy)
        rb.advance = advance
        if mode == "dead":
            rb._physics = lambda h, still: orig_phys(h, True if state["done"] else still)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    cond = {}
    if "--sweep-cond" in argv:
        k = argv.index("--sweep-cond")
        with open(argv[k + 1]) as f:
            cond = json.load(f)
        del argv[k:k + 2]
    else:
        for k, v in enumerate(argv):
            if v.startswith("--sweep-cond="):
                with open(v.split("=", 1)[1]) as f:
                    cond = json.load(f)
                del argv[k]
                break
    if cond.get("cam_age_s"):
        argv += ["--fix-delay", str(cond["cam_age_s"])]
    seed = 1
    if "--seed" in argv:
        seed = int(argv[argv.index("--seed") + 1])
    orig_init = FS.FleetSim.__init__

    def init(self, args):
        orig_init(self, args)
        install(self, cond, seed)
    FS.FleetSim.__init__ = init
    return FS.main(argv)


if __name__ == "__main__":
    sys.exit(main())
