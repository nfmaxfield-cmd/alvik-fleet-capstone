"""evalfits.py OUT.jsonl RUN_FOLDER [...] [--variants a,b,c] : explore's finish replayed offline on each dry run's
recorded stops (the robot checks, the joint fit, the per-robot drop, the two-robot rule, the contacts, the board's
sides: as e02 explore_finish does it), under fit variants, scored against the simulator's truth. A variant changes
only the calculation, not the driving, so this is the cheap first test of a fit change (the driving changes need
dry runs). Variants: base (as before this build), repair (legs the camera saw much shorter than the wheels counted),
tilt (prior: the camera looks roughly straight down), sprior (each robot's wheel scale near 1), seen (left-out robots' contacts placed by the
fitted camera); joined with '+'."""
import json
import os
import sys
import time

import numpy as np

import exsum
from exsum import EX
import e02_map as MAP

SIZE = (640, 480)
SIZE_CHECK = False          # variant "size": explore's board refused when a side is 6 % off the plywood (e02 explore_finish)
KEEP_UNREFINED = False      # variant "keep": as e02 explore_finish since the after batch (r04), the sides as met when the
                            # learned reach breaks them


def set_variant(v):
    global KEEP_UNREFINED, SIZE_CHECK
    parts = set(v.split("+"))
    KEEP_UNREFINED = "keep" in parts
    SIZE_CHECK = "size" in parts
    EX.REPAIR_LEGS = "repair" in parts
    EX.TILT_PRIOR_SD_DEG, EX.TILT_PRIOR_W = (10.0, 10.0) if "tilt" in parts else (None, 3.0)
    EX.S_PRIOR_SD_LOG, EX.S_PRIOR_W = (0.04, 3.0) if "sprior" in parts else (None, 3.0)
    EX.F_PRIOR_PX = None


def finish(obs, min_robots=2, seen=False):
    robots = {n: v for n, v in obs.items() if len(v) >= 5}
    checks = {n: EX.robot_check(v) for n, v in robots.items()}
    left = [n for n in robots if not checks[n]["ok"]]
    use = {n: v for n, v in robots.items() if checks[n]["ok"]}
    if not use:
        return dict(ok=False, why="no robot's path fits"), None, use
    sol = EX.fit(use, SIZE)
    worse = [n for n, v in sol["per_robot_rms_px"].items() if not (v <= EX.FIT_ROBOT_MAX_PX)]
    if worse and len(worse) < len(use):
        left += worse
        use = {n: v for n, v in use.items() if n not in worse}
        sol = EX.fit(use, SIZE)
    bad = EX.fit_check(sol)
    if not bad and len(use) < min_robots and len(obs) >= min_robots:
        bad = "one robot"
    out = dict(rms=round(sol["rms_px"], 2), used=list(use), repaired=sum(len(v) for v in (sol.get("repairs") or {}).values()))
    if bad:
        out.update(ok=False, why=bad[:100])
        return out, sol, use
    use2 = EX.apply_repairs(use, sol) if hasattr(EX, "apply_repairs") else use
    pts = EX.contacts(use2, sol, others={n: obs[n] for n in left if n in obs})
    if seen and left:
        paths = {n: [(q.get("t"), p[0], p[1]) for p, q in zip(EX.to_world(EX.rebuild(v, sol["s"][n], sol["g"][n]),
                                                                        sol["frames"][n]), v)] for n, v in use2.items()}
        add = EX.contacts_seen({n: obs[n] for n in left if n in obs}, sol, paths)
        out["seen_contacts"] = len(add)
        pts = pts + add
    rect = EX.rectangle_from_rays(pts)
    if rect:
        rect0 = rect
        rect, ah = EX.refine_edges(rect, MAP.BOTTOM_AHEAD_CM)
        if KEEP_UNREFINED and ah is not None and EX.rect_check(rect) and not EX.rect_check(rect0):
            rect = rect0
    rb = EX.rect_check(rect) if rect else "no rectangle"
    bf = EX.board_frame(rect) if rect and not rb else None
    out["contacts"] = len(pts)
    if bf is None:
        out.update(ok=False, why=("sides: " + rb)[:100])
        return out, sol, use
    if SIZE_CHECK:
        off = max(abs(bf["W"] / 122.15 - 1.0), abs(bf["H"] / 121.9 - 1.0)) * 100.0
        if off > 6.0:
            out.update(ok=False, why="size %.1f x %.1f: %.0f %% off the plywood" % (bf["W"], bf["H"], off),
                       W_seen=round(bf["W"], 2), H_seen=round(bf["H"], 2))
            return out, sol, use
    out.update(ok=True, why="", W=round(bf["W"], 2), H=round(bf["H"], 2))
    return out, sol, use


def main():
    a = sys.argv[1:]
    vs = ["base", "repair", "repair+tilt", "repair+tilt+sprior"]
    if "--variants" in a:
        i = a.index("--variants")
        vs = a[i + 1].split(",")
        a = a[:i] + a[i + 2:]
    out, runs = a[0], a[1:]
    for d in runs:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        info = json.load(open(os.path.join(d, "sim", "sim_info.json")))
        W, H = info["board"]["yboard_cm"], info["board"]["xboard_cm"]
        obs = (st.get("setup") or {}).get("explore_obs") or {}
        for v in vs:
            set_variant(v)
            t0 = time.time()
            try:
                r, sol, use = finish(json.loads(json.dumps(obs)), seen="seen" in v.split("+"))
            except Exception as e:
                r, sol = dict(ok=False, why="error %s" % e), None
            if sol is not None:
                r["cam_scale"] = round(exsum.cam_scale(sol, d)[0], 4)
            if r.get("W"):
                r["W_err_pct"] = round(100 * (r["W"] / W - 1), 1)
                r["H_err_pct"] = round(100 * (r["H"] / H - 1), 1)
            r.update(run=os.path.basename(d.rstrip("/")), variant=v, sec=round(time.time() - t0))
            print(json.dumps(r), flush=True)
            with open(out, "a") as f:
                f.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    main()
