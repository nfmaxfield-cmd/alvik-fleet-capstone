#!/usr/bin/env python3
"""Tag-free replay (12 Nov 2026, build tag-free): a run's saved pictures through tag-free mode, offline.

For every labelled picture (autolabel.py --pictures output: <labels>/<rec>/labels/labels.csv + frames, and the
camera model it found, floor_calibration.json), the robots' OWN poses at that moment (the run's follow.csv) go into a
fresh tracking layer with every robot unaccounted for (as a lost / unsure robot would be); the detector runs on demand
(tagfree.DetectorSource, overhead first, the side camera only where the overhead found nothing near the robot asked
for); the layer names what it sees (its gate around the own pose, then elimination). Each robot is then scored
against the camera track's label (the truth proxy autolabel used):
  named right   a fix within RIGHT_CM of its own label;   named wrong   within RIGHT_CM of ANOTHER robot's label;
  not named     no fix.
Heading (overhead, robots whose label has a camera-checked heading): the detector's heading, the robot's own heading,
and the fused heading (tagfree.HeadingFuser over the robot's pictures in time order) against the label.

ASSUMPTION (stated in the report): one saved picture is fed as STEADY_N pictures 0.5 s apart (the event pictures are
minutes apart; these robots were mostly standing, "unsure near an edge"), so the layer's steady-sighting rule can act.

    python3 tools/analysis/tagfree_replay.py --run <run folder> --labels <labels folder> --model detector-v1.onnx \
        --only <rec>_overhead_t7,<rec>_overhead_t8 --out replay.json

    python3 tools/analysis/tagfree_replay.py --trust --run <run folder> --labels <labels> --model detector-v1.onnx \
        --only <rec>_overhead_t1,...,<rec>_overhead_t8 --out trust.json

--trust (e20): every picture through EVERY camera's detector (no camera is named in the rules), each detection matched
to a robot by its OWN pose (the layer's widest gate), its heading checked against the robot's own heading where the run
itself had confirmed that heading (the label's heading_known: its travel direction agreed with its own heading); the
camera's learned heading trust (camera_trust.TrustBook) is then compared with what the labels say each camera's heading
error is. The labels are used ONLY for that comparison, never for the learning.

Changelog
  2026-11-12a  first version
  2026-11-12e20  --trust: learned camera trust from the pictures (camera_trust)
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for p in (os.path.join(TOOLS, "fleet"), os.path.join(TOOLS, "detector")):
    if p not in sys.path:
        sys.path.insert(0, p)

VERSION = "tagfree-replay-2026-11-12e20"   # e20: --trust. Before: "tagfree-replay-2026-11-12a"
RIGHT_CM = 6.0           # CHOSEN: a fix this near a label is that robot (a robot's half width + the label's noise)
STEP_S = 0.5


def read_own(run):
    own = {}
    with open(os.path.join(run, "follow.csv")) as f:
        for r in csv.DictReader(f):
            if r.get("state") == "nopose":
                continue
            try:
                own.setdefault(r["robot"], []).append((float(r["t_recv"]), float(r["x_cm"]), float(r["y_cm"]),
                                                       float(r["th_deg"])))
            except (KeyError, ValueError):
                continue
    for v in own.values():
        v.sort()
    return own


def own_at(own, name, t):
    v = own.get(name) or []
    if not v:
        return None
    i = bisect.bisect_left(v, (t,))
    cand = [v[j] for j in (i - 1, i) if 0 <= j < len(v)]
    q = min(cand, key=lambda q: abs(q[0] - t))
    return q if abs(q[0] - t) < 1.0 else None


def read_labels(rec_dir):
    rows = {}
    with open(os.path.join(rec_dir, "labels", "labels.csv")) as f:
        for r in csv.DictReader(f):
            rows.setdefault(int(r["frame"]), []).append(r)
    frames = sorted(os.listdir(os.path.join(rec_dir, "labels", "frames")))
    return rows, frames


def trust_replay(a):
    """(e20) The learned camera trust on the real pictures (see the module doc)."""
    import cv2
    import infer as INF
    import tagfree as TF
    import tracking_layer as TL
    import camera_trust as CT
    model = INF.load(a.model, threads=1)
    own = read_own(a.run)
    names = sorted(own)
    board = (0.0, 0.0, 122.24, 121.92)
    book = CT.TrustBook("replay-home-2026-11-11", board=board, seed=1)
    rows = []
    gate = TL.GATE_MAX_CM
    for rec in a.only.split(","):
        recs = {}
        for cam in ("overhead", "side"):
            d = os.path.join(a.labels, rec.replace("_overhead_", "_%s_" % cam))
            if not os.path.isdir(d):
                continue
            H = json.load(open(os.path.join(d, "floor_calibration.json")))["H_floor_to_image"]
            rl, fr = read_labels(d)
            recs[cam] = (d, TF.homography_to_floor(H), rl, fr)
        for cam, (d, to_f, rl, fr) in sorted(recs.items()):
            for k in sorted(rl):
                labs = rl[k]
                t = float(labs[0]["t_wall"])
                img = cv2.imread(os.path.join(d, "labels", "frames", fr[k]))
                if img is None:
                    continue
                dets = [q for q in model(img) if q.get("cls", "alvik") == "alvik"]
                for lab in labs:
                    n = lab["obj"]
                    o = own_at(own, n, t)
                    if o is None:
                        continue
                    best = None
                    for q in dets:
                        x, y = to_f(float(q["x"]), float(q["y"]))
                        e = math.hypot(x - o[1], y - o[2])
                        if e <= gate and (best is None or e < best[0]):
                            best = (e, q, x, y)
                    if best is None or best[1].get("heading_deg") is None:
                        continue
                    _, q, x, y = best
                    th_det = TF.floor_heading(to_f, float(q["x"]), float(q["y"]), float(q["heading_deg"]))
                    sure = lab.get("heading_known") == "1"
                    err_own = abs(TF.wrap180(th_det - o[3]))
                    tight = CT.HEAD_K * math.hypot(TF.SIGMA_DET0_DEG, 2.0)       # its own heading as sure as it is
                    if sure:
                        book.observe(cam, "ml", "heading", x, y, err_own <= tight, 1.0, None, t,
                                     "its own heading (confirmed by its travel)")
                    th_true = None
                    if lab.get("heading_known") == "1":
                        th_true = TF.floor_heading(to_f, float(lab["x_px"]), float(lab["y_px"]),
                                                   float(lab["heading_deg"]))
                    rows.append(dict(rec=rec, cam=cam, t=round(t, 2), robot=n, sure=int(sure),
                                     err_own=round(err_own, 1),
                                     err_true=None if th_true is None else round(abs(TF.wrap180(th_det - th_true)), 1),
                                     state=book.state(cam, "ml", "heading", area=None)))
    res = dict(version=VERSION, camera_trust=CT.VERSION, model=os.path.basename(a.model), recordings=a.only.split(","),
               words=book.plain_words(), trust=book.summary(), rows=rows)
    per = {}
    for cam in sorted({r["cam"] for r in rows}):
        v = sorted(r["err_true"] for r in rows if r["cam"] == cam and r["err_true"] is not None)
        s = book.score(cam, "ml", "heading", area=None)
        rng = __import__("random").Random(1)
        picks = [book.choose(sorted({r["cam"] for r in rows}), "ml", "heading", 60, 60, rng=rng) for _ in range(1000)]
        per[cam] = dict(state=s["state"], agree=s["agree"], checks=s["checks"], lower=s["lower"], upper=s["upper"],
                        label_err_median=v[len(v) // 2] if v else None,
                        label_err_p95=v[int(0.95 * (len(v) - 1))] if v else None, n_label=len(v),
                        heading_calls_share=round(picks.count(cam) / 1000.0, 3))
    res["per_camera"] = per
    print(json.dumps(dict(per_camera=per, words=res["words"]), indent=1))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1, default=str)
    return res


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if "--trust" in argv:
        ap = argparse.ArgumentParser()
        ap.add_argument("--trust", action="store_true")
        ap.add_argument("--run", required=True)
        ap.add_argument("--labels", required=True)
        ap.add_argument("--model", required=True)
        ap.add_argument("--only", required=True)
        ap.add_argument("--out", default="")
        return trust_replay(ap.parse_args(argv))
    import cv2
    import infer as INF
    import tagfree as TF
    import tracking_layer as TL
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--labels", required=True, help="autolabel's output folder (one sub-folder a recording)")
    ap.add_argument("--model", required=True)
    ap.add_argument("--only", required=True, help="overhead recordings to replay, comma separated (the side camera's "
                                                  "recording of the same block is found by name)")
    ap.add_argument("--out", default="")
    ap.add_argument("--no-side", action="store_true")
    ap.add_argument("--own-bias", action="append", default=[], help="NAME=DEG: break the guess: that robot's own "
                    "heading this far off (as Center's 71 deg at 08:48); does the fuser catch it?")
    a = ap.parse_args(argv)
    model = INF.load(a.model, threads=1)
    bias = {kv.split("=")[0]: float(kv.split("=")[1]) for kv in a.own_bias}
    own = read_own(a.run)
    names = sorted(own)
    board = (0.0, 0.0, 122.24, 121.92)
    res = dict(own_bias=bias if a.own_bias else None, version=VERSION, tagfree=TF.VERSION, model=os.path.basename(a.model), pictures=0,
               robots=dict(right=0, wrong=0, unnamed=0, n=0), by_how={}, by_cam={}, heading=[], wrong_cases=[],
               assumption="one picture fed as %d pictures %.1f s apart" % (TL.STEADY_N + 1, STEP_S))
    fuser = TF.HeadingFuser()
    for rec in a.only.split(","):
        od = os.path.join(a.labels, rec)
        sd = os.path.join(a.labels, rec.replace("_overhead_", "_side_"))
        Ho = json.load(open(os.path.join(od, "floor_calibration.json")))["H_floor_to_image"]
        to_o = TF.homography_to_floor(Ho)
        rows_o, frames_o = read_labels(od)
        side = None
        if not a.no_side and os.path.isdir(sd):
            Hs = json.load(open(os.path.join(sd, "floor_calibration.json")))["H_floor_to_image"]
            rows_s, frames_s = read_labels(sd)
            ts_s = {float(v[0]["t_wall"]): k for k, v in rows_s.items()}
            side = (TF.homography_to_floor(Hs), rows_s, frames_s, ts_s)
        for k in sorted(rows_o):
            labs = rows_o[k]
            t = float(labs[0]["t_wall"])
            img_o = cv2.imread(os.path.join(od, "labels", "frames", frames_o[k]))
            img_s = None
            if side is not None:
                near = min(side[3], key=lambda q: abs(q - t)) if side[3] else None
                if near is not None and abs(near - t) < 2.0:
                    img_s = cv2.imread(os.path.join(sd, "labels", "frames", side[2][side[3][near]]))
            truth = {r["obj"]: to_o(float(r["x_px"]), float(r["y_px"])) for r in labs}
            res["pictures"] += 1
            clock = [t]

            def pic(img):
                return lambda: (clock[0], img) if img is not None else None
            cams = [TF.Camera("overhead", pic(img_o), to_o, True, TF.SIGMA_OVERHEAD_CM)]
            if img_s is not None:
                cams.append(TF.Camera("side", pic(img_s), side[0], False, TF.SIGMA_SIDE_CM))
            src = TF.DetectorSource(model, cams, sync=True, clock=lambda: clock[0])
            src.budget = TF.Budget(duty=1.0, budget_ms=1e9)           # offline: no load limit
            L = TL.TrackingLayer(names, ("motion",), board=board, camera_watch=False)
            st = TF.attach(L, src, mode="1")
            got, how = {}, {}
            for j in range(TL.STEADY_N + 1):
                tj = t + j * STEP_S
                clock[0] = tj
                for n in names:
                    o = own_at(own, n, t)
                    if o is not None:
                        L.own(n, tj, o[1], o[2], o[3], True)
                    L.set_unaccounted(n, "replay: one picture")
                for f in L.feed(tj, []):
                    got[f["robot"]] = f
                    if f.get("how"):
                        how[f["robot"]] = f["how"]
            for n in names:
                if n not in truth:
                    continue
                res["robots"]["n"] += 1
                f = got.get(n)
                if f is None:
                    res["robots"]["unnamed"] += 1
                    seen = any(math.hypot(x - truth[n][0], y - truth[n][1]) < RIGHT_CM for x, y in (
                        to_o(float(d["x"]), float(d["y"])) for d in (src.cache.get("overhead") or (None, []))[1]))
                    res["robots"]["unnamed_seen" if seen else "unnamed_not_detected"] = \
                        res["robots"].get("unnamed_seen" if seen else "unnamed_not_detected", 0) + 1
                    continue
                dn = math.hypot(f["x"] - truth[n][0], f["y"] - truth[n][1])
                other = [m for m in truth if m != n and math.hypot(f["x"] - truth[m][0], f["y"] - truth[m][1]) < RIGHT_CM]
                kind = "right" if dn < RIGHT_CM else ("wrong" if other else "unnamed")
                res["robots"][kind] += 1
                hw = (how.get(n) or "gate").split(":")[0]
                res["by_how"].setdefault(hw, dict(right=0, wrong=0, unnamed=0))[kind] += 1
                if kind == "wrong":
                    o = own_at(own, n, t)
                    res["wrong_cases"].append(dict(t=t, robot=n, took=other[0], how=hw, own=None if o is None else
                                                   [round(o[1], 1), round(o[2], 1)], fix=[round(f["x"], 1),
                                                                                          round(f["y"], 1)]))
            # heading: the overhead detections matched to labels with a checked heading
            for r in labs:
                if r["heading_known"] != "1":
                    continue
                n = r["obj"]
                tx, ty = truth[n]
                ld = [d for d in (src.cache.get("overhead") or (None, []))[1]]
                if not ld:
                    continue
                best = None
                for d in ld:
                    x, y = to_o(float(d["x"]), float(d["y"]))
                    e = math.hypot(x - tx, y - ty)
                    if e < RIGHT_CM and (best is None or e < best[0]):
                        best = (e, d)
                if best is None or best[1].get("heading_deg") is None:
                    continue
                d = best[1]
                th_true = TF.floor_heading(to_o, float(r["x_px"]), float(r["y_px"]), float(r["heading_deg"]))
                th_det = TF.floor_heading(to_o, float(d["x"]), float(d["y"]), float(d["heading_deg"]))
                o = own_at(own, n, t)
                th_own = None if o is None else TF.wrap180(o[3] + bias.get(n, 0.0))
                fh, fs, verdict = fuser.update(n, t, th_own, th_det)
                res["heading"].append(dict(t=round(t, 2), robot=n, det=round(abs(TF.wrap180(th_det - th_true)), 1),
                                           own=None if th_own is None else round(abs(TF.wrap180(th_own - th_true)), 1),
                                           fused=None if fh is None else round(abs(TF.wrap180(fh - th_true)), 1),
                                           used=fuser.advice(n) == "hand", check=fuser.advice(n) == "check",
                                           verdict=verdict))
            for c, v in src.counts.items():
                res["by_cam"][c] = res["by_cam"].get(c, 0) + v
            del st
    h = res["heading"]

    def med(v):
        v = sorted(x for x in v if x is not None)
        return (round(v[len(v) // 2], 1), round(v[int(0.95 * (len(v) - 1))], 1), len(v)) if v else None
    res["heading_summary"] = dict(detector=med([q["det"] for q in h]), own=med([q["own"] for q in h]),
                                  fused=med([q["fused"] for q in h]),
                                  handed_on=med([(q["fused"] if q["used"] else q["own"]) for q in h]),
                                  fuser=fuser.summary())
    print(json.dumps({k: v for k, v in res.items() if k not in ("heading",)}, indent=1, default=str))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
