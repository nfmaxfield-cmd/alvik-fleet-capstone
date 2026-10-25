"""Fix 1 on the same data, old rule against new: a dry run's explore legs replayed through the camera border with the
old distance (abs: 'closer to the line') and the new one (signed: + on the board, - past the line).

Inputs: the dry run's track.csv (the tracker's positions on the provisional camera while exploring), its robots' leg
records (robot-logs-1/leg.csv: when each leg started and ended), and the border's camera: the simulator's true
camera shifted by --dry-guard-shift (the run's own args). Each leg is checked as the watchdog does: every 0.1 s, the
last 0.6 s of positions, home's 8 cm border (BORDER_CM away). A leg counts as 'tripped' if any check trips.

    python3 border_replay.py [--shift=DX/DY] LOG_FOLDER [...]   -> one table row per run and leg kind
--shift: replay with that border offset instead of the run's own (-14/-14 leaves the simulator's line-sensor edge
stops 4-5 cm past the line, as every 15:10 edge stop was)."""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [EXP, os.path.join(os.path.dirname(EXP), "recognition")]
import e02 as E  # noqa: E402
import e02_camera as CAM  # noqa: E402
import e02_explore as EX  # noqa: E402
import e02_plan as PL  # noqa: E402
import e02_steer as ST  # noqa: E402


def check(rule, bd, smp, bcm):
    a, b = smp[0], smp[-1]
    dt_ = max(1e-3, b[0] - a[0])
    speed = math.hypot(b[1] - a[1], b[2] - a[2]) / dt_
    for ax, val in bd.open_edges():
        p0, p1 = (a[1], b[1]) if ax == "x" else (a[2], b[2])
        if rule == "old":
            d, towards = abs(p1 - val), (abs(p0 - val) - abs(p1 - val)) / dt_
        else:
            d = ST.inward_cm(bd, ax, val, p1)
            towards = (ST.inward_cm(bd, ax, val, p0) - d) / dt_
        if d < bcm and E.BORDER_TOWARDS_CMS < towards < E.BORDER_JUMP_CMS and towards >= E.BORDER_TOWARDS_FRAC * speed:
            return True
    return False


def explore_end(d):
    """When explore's fit replaced the provisional camera (the log's 'the board:' or 'EXPLORE FAILED' line)."""
    import datetime as dt
    import re
    day = dt.datetime.fromtimestamp(os.path.getmtime(os.path.join(d, "e02_log.txt"))).date()
    for line in open(os.path.join(d, "e02_log.txt"), errors="replace"):
        m = re.match(r"\[(\d\d):(\d\d):(\d\d)\]", line)
        if m and ("the board:" in line or "EXPLORE FAILED" in line):
            return dt.datetime(day.year, day.month, day.day, *(int(g) for g in m.groups())).timestamp()
    return float("inf")


SHIFT = None


def main():
    global SHIFT
    if sys.argv[1].startswith("--shift="):
        SHIFT = sys.argv.pop(1).split("=", 1)[1]     # replay with another border offset than the run had
    print("| Run | Legs | Kind | Tripped, old rule | Tripped, new rule | Of those, past the line at the start |")
    print("|---|---|---|---|---|---|")
    for d in sys.argv[1:]:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        args = (st.get("setup") or {}).get("args") or {}
        dx, dy = (float(v) for v in (SHIFT or args.get("dry_guard_shift") or "0/0").split("/"))
        home = bool(args.get("home"))
        bcm = E.HOME_BORDER_CM if home else E.BORDER_CM
        gcam = CAM.load_camera(os.path.join(d, "sim", "camera_model.json"))
        size = tuple(gcam.image_size) if hasattr(gcam, "image_size") else (640, 480)
        pcam = E.EX_provisional_camera(size[0], size[1])
        bd = PL.Board(xboard_side=1, yboard_side=1)
        t_cut = explore_end(d)
        trk = {}
        for r in csv.DictReader(open(os.path.join(d, "track.csv"))):
            if r["order"] != "explore" or float(r["t"]) > t_cut:
                continue          # after explore's fit the tracker's positions are in the board's frame
            u, v = pcam.project((float(r["x_cm"]), float(r["y_cm"]), EX.ROBOT_TOP_CM))
            gx, gy = gcam.to_floor(float(u), float(v), EX.ROBOT_TOP_CM)
            trk.setdefault(r["robot"], []).append((float(r["t"]), gx + dx, gy + dy))
        legs, start = [], {}
        for r in csv.DictReader(open(os.path.join(d, "robot-logs-1", "leg.csv"))):
            key = (r["robot"], r["id"])
            if r["result"] == "started":
                start[key] = float(r["t_recv"])
            elif r["result"] in ("done", "failed") and key in start and r["kind"] == "go":
                legs.append((r["robot"], start.pop(key), float(r["t_recv"]), float(r["value"])))
        t_end = max((t for v in trk.values() for t, _, _ in v), default=0)
        res = {}
        for robot, t0, t1, val in legs:
            if t0 > t_end:
                continue
            kind = "back-off (go -)" if val < 0 else ("ray / probe crawl" if val in (105.0, 12.0) else "return / home")
            smp_all = [q for q in trk.get(robot, []) if t0 - 0.6 <= q[0] <= t1]
            if len(smp_all) < 3:
                continue
            first = [q for q in smp_all if q[0] >= t0][:1]
            past0 = bool(first) and min(ST.inward_cm(bd, ax, v, first[0][1] if ax == "x" else first[0][2])
                                        for ax, v in bd.open_edges()) < 0
            trip = {"old": False, "new": False}
            t = t0 + 0.3
            while t <= t1:
                w = [q for q in smp_all if t - 0.6 <= q[0] <= t and q[0] >= t0]
                if len(w) >= 3:
                    for rule in trip:
                        trip[rule] = trip[rule] or check(rule, bd, w, bcm)
                t += 0.1
            r_ = res.setdefault(kind, [0, 0, 0, 0])
            r_[0] += 1
            r_[1] += trip["old"]
            r_[2] += trip["new"]
            r_[3] += past0 and trip["old"]
        for kind, (n, o, nw, p) in sorted(res.items()):
            print("| %s | %d | %s | %d | %d | %d |" % (os.path.basename(d), n, kind, o, nw, p))


if __name__ == "__main__":
    main()
