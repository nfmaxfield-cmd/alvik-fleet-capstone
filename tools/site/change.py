#!/usr/bin/env python3
"""Change check: during normal runs, does the front distance sensor keep
disagreeing with the locked site map about one landmark? Then that landmark may
have moved (bumped, carried away, a new board in front of it).

    python tools/site/change.py --map site/map.json --session logs/<session> [--poses poses.csv]

Live: tools/fleet/run_plan.py --site-map site/map.json runs the same check on
every RAW row while a plan runs.

How. Each zone reading, at the robot's pose (its own pose from the FOLLOW records,
or --poses: t,robot,x_cm,y_cm,heading_deg[,sd_cm], e.g. the fusion's output or
the camera), is tied to the face the locked map says it hits (the fusion's own
rules: landmarks.predict_zone), with the robot's sensor model from the lock
(zone directions, range bias and scale). Readings far from any mapped face
(more than 15 cm off: a hand, a robot, a brick) are not counted against a
landmark. Per landmark the last `window` readings (default 60) within `span_s`
(default 120 s) are kept. A landmark is flagged when

  - at least `min_n` readings (30) over at least `min_s` seconds (5 s), and
  - their median residual (reading minus map) is over max(`min_cm`, k x its
    standard error) with min_cm = 0.5 cm and k = 4; the standard error comes
    from the readings' own scatter and counts them as 1 in 5 independent
    (successive frames and zones share errors).

A robot's own pose error would move every landmark it sees, so readings are not
used while its pose is unsure (sd over 0.5 cm), and the readings must span at
least 5 s. The landmark's own locked uncertainty is added to the standard error.
Each flag is printed once per landmark per session and written to
<session>/site_changes.jsonl.
"""
import argparse
import collections
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion"), os.path.join(TOOLS, "fleet")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import engine as ENG  # noqa: E402
import landmarks as LM  # noqa: E402
import sitemap as SM  # noqa: E402

D2R = math.pi / 180.0
DEFAULTS = dict(window=60, span_s=120.0, min_n=30, min_s=5.0, min_cm=0.5, k=4.0, n_per_independent=5.0,
                max_pose_sd_cm=0.5, gross_cm=15.0)


class ChangeCheck:
    """Feed it RAW rows with the robot's pose; it says when a landmark seems to have moved."""

    def __init__(self, map_path, opts=None, log_path=None, say=print):
        self.o = dict(DEFAULTS, **(opts or {}))
        m, site = SM.read_lock(map_path)
        if site is None:
            raise ValueError(f"{map_path}: not a locked site map (no site block, or the map was edited after locking)")
        self.site = site
        import fleetmap as FM
        mm = FM.normalize(m)
        self.landmarks = SM.landmarks_from_map(mm)
        self.faces, self.face_lm = [], []
        for l in self.landmarks:
            for f in l.faces:
                self.faces.append(f)
                self.face_lm.append(l.name)
        self.sigma = {n: float(v.get("sigma_cm", 0.3)) for n, v in site.get("landmarks", {}).items()}
        self.robots = site.get("robots") or {}
        w = float((site.get("sensor") or {}).get("zone_width_deg", 0.0))
        self.cfg = ENG.config(tof_zone_width_deg=w)
        self.hist = collections.defaultdict(collections.deque)      # landmark -> (t, resid, sd, robot)
        self.flagged = {}
        self.log_path = log_path
        self.say = say
        self.prev = {}

    def feed(self, robot, t, pose, zones, pose_sd=0.0):
        """pose (x, y, heading_deg); zones the five readings (cm, None if none). Returns new flags (list)."""
        if pose is None or pose_sd > self.o["max_pose_sd_cm"]:
            return []
        z5 = tuple(zones)
        if self.prev.get(robot) == z5:            # a RAW row repeats the last sensor frame
            return []
        self.prev[robot] = z5
        r = self.robots.get(robot) or {}
        off = r.get("zone_offset_deg") or [0.0] * 5
        bias, scale = float(r.get("range_bias_cm", 0.0)), float(r.get("range_scale", 1.0))
        s = np.array([pose[0], pose[1], pose[2] * D2R])
        P = np.diag([max(pose_sd, 0.1) ** 2] * 2 + [(1.0 * D2R) ** 2])
        out = []
        for zi, zz in enumerate(zones):
            if zz is None or not (self.cfg["tof_min_cm"] <= zz <= self.cfg["tof_max_cm"]):
                continue
            ang = (self.cfg["tof_zone_deg"][zi] + off[zi]) * D2R
            why, info = LM.predict_zone(s, P, ang, self.faces, self.cfg)
            if why:
                continue
            h = info["h"] * scale + bias
            res = zz - h
            if abs(res) > self.o["gross_cm"]:
                continue
            name = self.face_lm[info["face"]]
            sd = LM.zone_sigma(h, self.cfg, info["tan_inc"])
            q = self.hist[name]
            q.append((t, res, sd, robot))
            while q and (len(q) > self.o["window"] or t - q[0][0] > self.o["span_s"]):
                q.popleft()
            f = self.evaluate(name, t)
            if f:
                out.append(f)
        return out

    def evaluate(self, name, t):
        q = self.hist[name]
        o = self.o
        if len(q) < o["min_n"] or name in self.flagged:
            return None
        ts = [v[0] for v in q]
        if ts[-1] - ts[0] < o["min_s"]:
            return None
        res = np.array([v[1] for v in q])
        med = float(np.median(res))
        n_eff = max(1.0, len(q) / o["n_per_independent"])
        spread = 1.4826 * float(np.median(np.abs(res - med)))          # the readings' own scatter (robust)
        se = 1.2533 * max(spread, 0.1) / math.sqrt(n_eff)
        thr = max(o["min_cm"], o["k"] * math.hypot(se, self.sigma.get(name, 0.0)))
        if abs(med) <= thr:
            return None
        robots = sorted({v[3] for v in q})
        flag = dict(t=t, landmark=name, median_resid_cm=round(med, 2), threshold_cm=round(thr, 2), readings=len(q),
                    seconds=round(ts[-1] - ts[0], 1), robots=robots,
                    message=(f"landmark {name} may have moved: the front sensors read it {abs(med):.1f} cm "
                             f"{'further' if med > 0 else 'closer'} than the locked map says ({len(q)} readings, "
                             f"{', '.join(robots)}, over {ts[-1] - ts[0]:.0f} s). Consider a site setup refresh for "
                             f"that area: python tools/site/setup.py --site <folder> --refresh {name}"))
        self.flagged[name] = flag
        if self.log_path:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(flag) + "\n")
        if self.say:
            self.say(flag["message"])
        return flag


def poses_from_follow(session):
    """Each robot's own pose from the session's follow.csv: {robot: [(t, x, y, th_deg, sd)]} on the laptop
    clock (t_recv)."""
    out = collections.defaultdict(list)
    import glob
    for p in sorted(glob.glob(os.path.join(session, "follow*.csv"))):
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                if (r.get("state") or "") == "nopose":
                    continue
                try:
                    out[r["robot"]].append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]), float(r["th_deg"]),
                                            float(r.get("pos_sd_cm") or 0.0)))
                except (KeyError, ValueError):
                    continue
    return out


def poses_from_csv(path):
    out = collections.defaultdict(list)
    with open(path, newline="", encoding="utf-8-sig") as f:
        for r in csv.DictReader(f):
            out[r["robot"]].append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]), float(r["heading_deg"]),
                                    float(r.get("sd_cm") or 0.0)))
    return out


def pose_at(track, t, max_gap=0.3):
    if not track:
        return None, None
    ts = [v[0] for v in track]
    i = int(np.searchsorted(ts, t))
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(track) and abs(track[j][0] - t) <= max_gap:
            if best is None or abs(track[j][0] - t) < abs(best[0] - t):
                best = track[j]
    if best is None:
        return None, None
    return (best[1], best[2], best[3]), best[4]


def check_session(map_path, session, poses=None, opts=None, say=print):
    """Run the check over a recorded session. Returns the flags."""
    import runs as RN
    cc = ChangeCheck(map_path, opts, log_path=os.path.join(session, "site_changes.jsonl"), say=say)
    tracks = poses_from_csv(poses) if poses else poses_from_follow(session)
    for tr in tracks.values():
        tr.sort()
    flags = []
    runs, _ = RN.load_runs(session)
    rows = sorted(((r["t"], run.robot, r) for run in runs for r in run.raw), key=lambda v: v[0])
    for t, robot, r in rows:
        pose, sd = pose_at(tracks.get(robot), t)
        z = [ENG.zone_reading(r.get(c)) for c in LM.ZONE_COLS]
        flags += cc.feed(robot, t, pose, z, sd or 0.0)
    return flags


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="the locked site map.json")
    ap.add_argument("--session", required=True, help="console / run_plan session folder")
    ap.add_argument("--poses", help="t,robot,x_cm,y_cm,heading_deg[,sd_cm] (default: the robots' own FOLLOW poses)")
    args = ap.parse_args(argv)
    flags = check_session(args.map, args.session, args.poses)
    if not flags:
        print("No landmark disagreed with the locked map.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
