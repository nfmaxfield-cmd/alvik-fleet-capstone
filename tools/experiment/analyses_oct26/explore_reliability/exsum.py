"""exsum.py RUN_FOLDER [...] [--jsonl OUT] : one explore-only dry run scored against the simulator's truth
(docs/build-reports/e02-oct26-explore-reliability.md). Per run: pass or fail and why, the camera fit (px), the
camera's scale against the true camera (explore's cm per true cm, from a grid of floor points seen by both), the
board's size against the true board, the sides the robots really touched (simulator truth: each contact stop's true
place), the sides explore could use, the robots whose paths were used, holds, skipped rays and the time taken."""
import csv
import json
import math
import os
import re
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, EXP)
import e02_explore as EX  # noqa: E402

SIDE_NEAR_CM = 15.0       # a contact whose true reach point is this near a true side touched it (analysis only)
AHEAD_CM = 8.0            # the reach point: this far ahead of the robot's true centre (front sensor / bottom zone)


def true_cam(d):
    c = json.load(open(os.path.join(d, "sim", "camera_model.json")))
    return np.array(c["K"], float), np.array(c["rvec"], float), np.array(c["tvec"], float)


def cam_scale(sol, d):
    """Explore's camera against the true one: true floor points (board frame, robot-top height) to pixels by the true
    camera, then to explore's world frame by explore's camera; the best similarity (turn, shift, scale, a mirror
    allowed) from explore's world to the truth. Returns (scale = explore cm per true cm, median cm left after it,
    95 % cm, the similarity)."""
    K, rv, tv = true_cam(d)
    P = np.array([(x, y, EX.ROBOT_TOP_CM) for x in np.arange(5, 120, 10.0) for y in np.arange(5, 120, 10.0)])
    uv, _ = cv2.projectPoints(P, rv, tv, K, None)
    uv = uv.reshape(-1, 2)
    A = np.array([EX.world_of(sol, u, v) for u, v in uv])
    B = P[:, :2]
    ok = np.all(np.isfinite(A), 1)
    A, B = A[ok], B[ok]
    best = None
    for mir in (1.0, -1.0):
        Am = A * np.array([1.0, mir])
        ma, mb = Am.mean(0), B.mean(0)
        U, S, Vt = np.linalg.svd((B - mb).T @ (Am - ma))
        D = np.eye(2)
        if np.linalg.det(U @ Vt) < 0:
            D[1, 1] = -1
        R = U @ D @ Vt
        s = float(np.trace(np.diag(S) @ D) / ((Am - ma) ** 2).sum())   # true = s R explore + t
        res = np.linalg.norm((s * (R @ (Am - ma).T)).T + mb - B, axis=1)
        if best is None or np.median(res) < best[1]:
            best = (s, float(np.median(res)), float(np.percentile(res, 95)), (mir, R, s, ma, mb))
    s, med, p95, tr = best
    return 1.0 / s, med, p95, tr


def truth_reader(d):
    tr = {}
    p = os.path.join(d, "sim", "truth.csv")
    import gzip
    with (open(p) if os.path.exists(p) else gzip.open(p + ".gz", "rt")) as f:
        for r in csv.DictReader(f):
            tr.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"]),
                                                  float(r["heading_deg"])))
    out = {}
    for n, v in tr.items():
        a = np.array(v)
        out[n] = a
    return out


def at(tr, n, t):
    a = tr[n]
    i = int(np.clip(np.searchsorted(a[:, 0], t), 0, len(a) - 1))
    return a[i, 1], a[i, 2], a[i, 3]


def touched_sides(st, tr, W, H):
    """{side: [kinds]} the robots really touched: each contact stop's true reach point near a true side."""
    out = {}
    for n, stops in (st["setup"].get("explore_obs") or {}).items():
        if n not in tr:
            continue
        for q in stops:
            k = q.get("contact")
            if not k or q.get("t") is None:
                continue
            x, y, h = at(tr, n, q["t"])
            cx, cy = x + AHEAD_CM * math.cos(math.radians(h)), y + AHEAD_CM * math.sin(math.radians(h))
            dd = dict(x0=abs(cx), y0=abs(cy), x1=abs(cx - W), y1=abs(cy - H))
            s = min(dd, key=dd.get)
            if dd[s] <= SIDE_NEAR_CM:
                out.setdefault(s, []).append(k)
    return out


def leg_errors(st, tr, lim_cm=5.0):
    """Straight legs whose own distance differs from the true one by more than lim_cm (wheels turning while the body
    does not follow: a hang at an edge, pushing a board, or a missed record): {robot: [(stop index, own, true, kind)]}."""
    out = {}
    for n, stops in (st["setup"].get("explore_obs") or {}).items():
        if n not in tr:
            continue
        for i in range(1, len(stops)):
            a, b = stops[i - 1], stops[i]
            if a.get("t") is None or b.get("t") is None or abs(b.get("dth", 0.0) or 0.0) > 20:
                continue
            xa, ya, ha = at(tr, n, a["t"])
            xb, yb, hb = at(tr, n, b["t"])
            hm = math.radians(ha + ((hb - ha + 180) % 360 - 180) / 2)
            dt = (xb - xa) * math.cos(hm) + (yb - ya) * math.sin(hm)
            if abs(dt - (b.get("d") or 0.0)) > lim_cm:
                out.setdefault(n, []).append((i, round(b.get("d") or 0.0, 1), round(dt, 1), b.get("contact", "")))
    return out


def minutes(d):
    t0 = t1 = None
    for ln in open(os.path.join(d, "e02_log.txt"), errors="replace"):
        m = re.match(r"\[(\d\d):(\d\d):(\d\d)\]", ln)
        if not m:
            continue
        s = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))
        if "=== Explore" in ln and t0 is None:
            t0 = s
        if ("EXPLORE FAILED" in ln or "explore done in" in ln) and t0 is not None:
            t1 = s
    return None if t0 is None or t1 is None else round(((t1 - t0) % 86400) / 60.0, 1)


def summarise(d):
    st = json.load(open(os.path.join(d, "e02_state.json")))
    info = json.load(open(os.path.join(d, "sim", "sim_info.json")))
    a = (st.get("setup") or {}).get("args") or {}
    b = info["board"]
    W, H = b["yboard_cm"], b["xboard_cm"]           # board frame: the y-board's face at x = W, the x-board's at y = H
    tr = truth_reader(d)
    su = st.get("setup") or {}
    ex = su.get("explore") or {}
    tries = su.get("explore_tries") or []
    last = tries[-1] if tries else {}
    passed = bool(ex.get("W"))
    row = dict(run=os.path.basename(d), seed=a.get("seed"), fps=a.get("dry_camera_fps") or 15,
               drop=a.get("dry_drop_pct") or 0, washed=a.get("dry_low_contrast") or "", start=a.get("dry_start"),
               passed=int(passed), why=(su.get("explore_failed") or {}).get("why", "")[:160],
               tries=len(tries), fit_px=round(last["sol"]["rms_px"], 2) if last.get("sol") else None,
               used=len(last.get("used") or []), used_names="/".join(last.get("used") or []),
               left_out="/".join(last.get("left_out") or []), minutes=minutes(d))
    if last.get("sol"):
        sc, med, p95, _ = cam_scale(last["sol"], d)
        _ = (sc, med, p95, _)
        row.update(cam_scale=round(sc, 4), cam_res_med_cm=round(med, 2), cam_res_p95_cm=round(p95, 2))
        s_true = {n: info["robots"][n]["w_true"] for n in last["sol"]["s"]}
        mir, Rm, sm, ma, mb = _[3]
        offs = {"board": [], "edge": []}
        for c in last.get("contacts") or []:
            q = sm * (Rm @ (np.array([c[0], mir * c[1]]) - ma)) + mb      # true board frame
            dd = dict(x0=-q[0], y0=-q[1], x1=q[0] - W, y1=q[1] - H)          # signed: + = outside the board
            sd_ = min(dd, key=lambda k: abs(dd[k]))
            if abs(dd[sd_]) <= 20.0 and c[2] in offs:
                offs[c[2]].append(dd[sd_])
        for k, v in offs.items():
            row["contact_out_%s_cm" % k] = round(float(np.median(v)), 1) if v else None
            row["contact_n_%s" % k] = len(v)
        row["wheel_fit"] = " ".join("%s %.3f/%.3f" % (n, last["sol"]["s"][n], s_true[n]) for n in last["sol"]["s"])
    if passed:
        row.update(W=ex["W"], H=ex["H"], W_err_pct=round(100 * (ex["W"] / W - 1), 1), H_err_pct=round(100 * (ex["H"] / H - 1), 1))
    sides = touched_sides(st, tr, W, H)
    row["sides_touched"] = len(sides)
    row["sides_touched_2"] = sum(1 for v in sides.values() if len(v) >= 2)
    row["touch_detail"] = " ".join("%s:%d" % (k, len(v)) for k, v in sorted(sides.items()))
    rect = last.get("rect") or {}
    row["sides_used"] = sum(1 for v in rect.values() if v[2] >= 2) if rect else 0
    row["rect_bad"] = (last.get("rect_bad") or "")[:120]
    row["contacts"] = len(last.get("contacts") or [])
    le = leg_errors(st, tr)
    row["legs_off"] = sum(len(v) for v in le.values())
    row["legs_off_detail"] = " ".join("%s:%s" % (n, ",".join("%s/%s%s" % (x[1], x[2], x[3][:1]) for x in v)) for n, v in le.items())
    row["holds"] = len(st.get("holds") or [])
    row["skipped_rays"] = len(su.get("explore_skipped_rays") or [])
    row["gap_rays"] = len(su.get("explore_gap_rays") or [])
    row["gap_rays_driven"] = sum(1 for g in su.get("explore_gap_rays") or [] if g[3] is not None)
    try:
        fps = [float(r["fps_handled"]) for r in csv.DictReader(open(os.path.join(d, "perf.csv")))
               if r.get("phase") in ("exploring", "explore", "finding")]
        row["fps_seen"] = round(float(np.median(fps)), 1) if fps else None
    except (OSError, KeyError, ValueError):
        row["fps_seen"] = None
    return row


def main():
    args = sys.argv[1:]
    out = None
    if "--jsonl" in args:
        i = args.index("--jsonl")
        out = args[i + 1]
        args = args[:i] + args[i + 2:]
    for d in args:
        try:
            r = summarise(d)
        except Exception as e:
            print("%s: %s" % (d, e))
            continue
        print(json.dumps(r))
        if out:
            with open(out, "a") as f:
                f.write(json.dumps(r) + "\n")


if __name__ == "__main__":
    main()
