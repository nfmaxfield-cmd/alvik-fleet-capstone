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
            rb.radio.drop_rng = random.Random((seed + 7919 * int(cond.get("retry", 0))) * 1000 + 17 * k + 3)
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
    # ---- the live cameras (--serve-live, the camera bridge's pictures; Monday 2-robot sweep, 2026-10-28)
    ls = float(cond.get("live_stall_s", 0.0) or 0.0)
    side_only = bool(cond.get("side_only"))
    if ls > 0 or side_only:
        import table_cam as TC
        l_every = float(cond.get("live_stall_every_s", 20.0))
        l_first = float(cond.get("live_stall_first_s", 7.0))
        l_mode = cond.get("live_stall_mode", "stall")

        def stall_left(now):
            """seconds left of the current stall (0: not stalled)"""
            if ls <= 0 or sim.t_plan0 is None:
                return 0.0
            t = now - sim.t_plan0 - l_first
            if t < 0:
                return 0.0
            ph = t % l_every
            return ls - ph if ph < ls else 0.0
        o_tr, o_sr = TC.TableCamera.render, TC.SideCamera.render

        def t_render(self, robots=(), boxes=(), hand=None):
            left = stall_left(time.time())
            if left > 0:
                if l_mode == "stall":
                    time.sleep(left)             # no picture from either camera until it ends (one loop renders both)
                elif getattr(self, "_stale", None) is not None:
                    return self._stale           # "stale": the last picture again, with a new time stamp
            img = o_tr(self, () if side_only else robots, boxes, hand)   # side_only: the overhead sees no robot
            self._stale = img
            return img

        def s_render(self, robots=()):
            if l_mode == "stale" and stall_left(time.time()) > 0 and getattr(self, "_stale", None) is not None:
                return self._stale
            img = o_sr(self, robots)
            self._stale = img
            return img
        TC.TableCamera.render = t_render
        TC.SideCamera.render = s_render
    # ---- a robot restarts mid-run (power dip, a sleep check-in): it stops, is silent while it boots, then answers
    # with a new boot ID, uncalibrated, every setting back to its factory value (as LiveRobot's `simctl restart`)
    rs = cond.get("restart")
    if rs:
        rbr = sim.robots[rs["name"]]
        rst = {"done": False}
        orig_adv_r = rbr.advance

        def advance_r(dt_ms, busy=False):
            if not rst["done"] and sim.t_plan0 is not None and time.time() - sim.t_plan0 >= float(rs["t"]):
                rst["done"] = True
                now = time.time()
                sim.drops.append((rbr.name, now, now + float(rs.get("boot_s", 3.0))))
                rbr.abort_reason = "restarted"
                rbr.boot_id = (rbr.boot_id * 7 + 0x1357) % 0xF000 + 0x1000
                rbr.fw_yaw_sign = 0
                import config as SCFG
                for k, v in SCFG.firmware_params():
                    rbr.params[k] = v
                sim.event(dict(kind="robot_restart", robot=rbr.name, t=round(now - sim.t_plan0, 2),
                               boot_s=rs.get("boot_s", 3.0)))
            orig_adv_r(dt_ms, busy)
        rbr.advance = advance_r
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
    # ---- the floor in the front zones, and false stops (false-stops build, 2026-10-29)
    floor_and_phantoms(sim, cond, seed)
    # ---- a robot held still for a while, and a robot nudged off its own pose (mirroring build, 2026-10-29)
    holds_and_nudges(sim, cond)


def holds_and_nudges(sim, cond):
    """hold   [{"name": "Left", "t": 50, "s": 3}]: the first step the robot starts at or after t s (after the plan's first
              step) stands s seconds before it runs: a robot paused mid-mowing; t 0: a robot that starts s late. Logged
              as a sim event (hold).
    nudge  [{"name": "Left", "t": 10, "cm": 2.5}]: at t s its TRUE pose moves cm back along its heading, its own pose
              not: the camera's fixes then correct it by cm (a position reset), and it drives the extra cm. Logged
              (nudge)."""
    holds = cond.get("hold") or []
    for h in holds:
        rb = sim.robots[h["name"]]
        st = {"done": False}
        orig = rb.run_step

        def run_step(s_, rb=rb, h=h, st=st, orig=orig):
            if not st["done"] and sim.t_plan0 is not None and time.time() - sim.t_plan0 >= float(h["t"]):
                st["done"] = True
                sim.event(dict(kind="hold", robot=rb.name, t=round(time.time() - sim.t_plan0, 2), s=float(h["s"]),
                               step=s_.get("id")))
                end = time.time() + float(h["s"])
                rb.set_wheels_speed(0.0, 0.0)
                while time.time() < end:
                    # should_abort() runs the firmware's tick (FOLLOW records, the radio): a held robot keeps
                    # talking, it only does not move (without it the robot went silent and was taken as dead)
                    if rb.should_abort() and rb.abort_reason:
                        break
                    rb.loop_delay(10)
            return orig(s_)
        rb.run_step = run_step
    for nd in cond.get("nudge") or []:
        rb = sim.robots[nd["name"]]
        stn = {"done": False}
        orig_adv = rb.advance

        def advance(dt_ms, busy=False, rb=rb, nd=nd, stn=stn, orig_adv=orig_adv):
            if not stn["done"] and sim.t_plan0 is not None and time.time() - sim.t_plan0 >= float(nd["t"]):
                stn["done"] = True
                cm = float(nd.get("cm", 2.5))
                rb.x -= cm * math.cos(rb.th)
                rb.y -= cm * math.sin(rb.th)
                sim.event(dict(kind="nudge", robot=rb.name, t=round(time.time() - sim.t_plan0, 2), cm=cm,
                               true=[round(rb.x, 2), round(rb.y, 2)]))
            orig_adv(dt_ms, busy)
        rb.advance = advance


def floor_and_phantoms(sim, cond, seed):
    """floor   {"Left": {"zone": 3, "cm": 11.4, "sd": 0.3}}: that zone reads the plywood floor at about cm whenever
               nothing nearer is in view (the 29 Oct logs: Left's centre-right zone, 11-12 cm standing). The floor
               check reads it at start-up.
    phantom    {"name": "Left" | "all", "readings": 4, "cm": 9.0, "zone": 3, "every": 1}: after every `every`-th turn,
               the next drive's first `readings` new readings in that zone are this low (nothing is there: a false
               stop with 3 in a row needs readings >= 3); with "long_every": k, "long_readings": 5, every k-th turn's is that
               long instead. Each one is logged as a sim event (phantom)."""
    fl = cond.get("floor") or {}
    ph = cond.get("phantom")
    for name, rb in sim.robots.items():
        f1 = fl.get(name)
        p1 = ph if ph and ph.get("name") in (name, "all") else None
        if not f1 and not p1:
            continue
        rng = random.Random(seed * 131 + sum(map(ord, name)))
        st = {"frame": None, "vals": None, "prev": "idle", "left": 0, "armed": None, "turns": 0}

        def tof(rb=rb, orig=rb.tof, f1=f1, p1=p1, st=st, rng=rng):
            v = orig()
            fr = getattr(rb, "_tof_ms", None)
            if fr is not None and fr == st["frame"] and st["vals"] is not None:
                return st["vals"]
            st["frame"] = fr
            v = list(v)
            if f1:
                k = int(f1.get("zone", 3))
                v[k] = min(v[k], max(0.5, rng.gauss(float(f1["cm"]), float(f1.get("sd", 0.3)))))
            if p1:
                s = getattr(rb, "f_state", "idle") or "idle"
                cur = getattr(rb, "f_cur", 0)
                if s == "drive" and st["prev"] == "turn" and st["armed"] != cur:
                    st["armed"] = cur
                    st["turns"] += 1
                    if (st["turns"] - 1) % max(1, int(p1.get("every", 1))) == 0:
                        st["left"] = int(p1.get("readings", 4))
                    le = int(p1.get("long_every", 0) or 0)
                    if le and st["turns"] % le == 0:
                        st["left"] = int(p1.get("long_readings", 5))    # now and then a long one: a stop even at 3
                    if st["left"] and sim.t_plan0 is not None:
                        sim.event(dict(kind="phantom", robot=rb.name, step=cur, readings=st["left"],
                                       t=round(time.time() - sim.t_plan0, 2)))
                if s != "idle":
                    st["prev"] = s
                if st["left"] > 0:
                    k = int(p1.get("zone", 3))
                    v[k] = min(v[k], float(p1.get("cm", 9.0)))
                    st["left"] -= 1
            st["vals"] = v
            return v
        rb.tof = tof


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
