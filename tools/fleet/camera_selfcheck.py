#!/usr/bin/env python3
"""The camera self-check at the start of a command-line run (build e12, Nick 10 Nov: "camera self-check at start and
re-measure without Mission Control").

A run started from the terminal (./run-fleet-pi.sh visit / freeroam / calib) used to trust the camera model saved by
the last Measure. Observe, then trust: before the robots move, the reference tags on the table (ids 20-25) are read in
a few pictures and compared with where the saved model puts them (tag_bridge.ref_expected: the setup's places in
site_fit.json through the camera model). The median distance per camera, in pixels:

  under BUMP_PX (ref_tags, 3 px, the bump watch's own limit)  -> "checks out": the run goes on with the model;
  over it                                                       -> REFIT: the same Measure as Mission Control's button
                                                                   (tag_job.TagJob.measure, with Mission Control's own
                                                                   saved clicks / typed values for this site), the old
                                                                   model kept in tagcal-before-<time>/, what changed
                                                                   said and logged (camera_selfcheck.json);
  no reference tag read / no model / no site_fit.json            -> said plainly; the run goes on with the model it has
                                                                   (the bump watch inside the bridge keeps checking).

Switch: MIRLT_CAMERA_SELFCHECK=0 (or --no-camera-check on the runners): the model trusted as before. "check" only
(MIRLT_CAMERA_SELFCHECK=check): measured and said, never refitted.

    python3 tools/fleet/camera_selfcheck.py [--site school-conference] [--out <folder>] [--check-only]
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for _p in (HERE, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "experiment")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np   # noqa: E402

VERSION = "2026-11-10a"
PICTURE_S = 2.0          # chosen: pictures read for the check (30-50 at 15-25 a second; the tags stand still). No DOE
REFIT_PX = None          # None: ref_tags.BUMP_PX (3 px), the bump watch's limit. Not chosen here


def mode():
    v = os.environ.get("MIRLT_CAMERA_SELFCHECK", "1").strip().lower()
    return "off" if v in ("0", "off", "no") else ("check" if v == "check" else "refit")


def compare(expected, seen):
    """{camera: {"n", "median_px", "max_px", "ids"}} for the reference tags seen in both (corner distances, px)."""
    out = {}
    for c, exp in (expected or {}).items():
        got = (seen or {}).get(c) or {}
        d = {}
        for i, e in exp.items():
            if str(i) in got:
                d[str(i)] = float(np.median(np.linalg.norm(np.array(got[str(i)], float) - np.array(e, float), axis=1)))
        out[c] = {"n": len(d), "median_px": round(float(np.median(list(d.values()))), 2) if d else None,
                  "max_px": round(max(d.values()), 2) if d else None, "ids": {k: round(v, 2) for k, v in d.items()}}
    return out


def verdict(cmp_, limit):
    """("ok" | "refit" | "unknown", words)."""
    seen = {c: v for c, v in cmp_.items() if v["n"]}
    if not seen:
        return "unknown", "no reference tag was read where the model expects one"
    bad = {c: v for c, v in seen.items() if v["median_px"] > limit}
    words = "; ".join("%s: %d reference tags, %.1f px from the model (median, worst %.1f)" % (
        c, v["n"], v["median_px"], v["max_px"]) for c, v in sorted(seen.items()))
    return ("refit" if bad else "ok"), words


class _Session:
    """What tag_job.TagJob needs from Mission Control's session, for a refit from the terminal: Mission Control's own
    folder for this site (its saved clicks / typed values) and its map, saved there as the button does."""

    def __init__(self, folder):
        self.dir = folder
        self.map, self.program, self.last_production = None, None, None
        self.map_path = os.path.join(folder, "map.json")

    def save_map(self):
        if self.map is not None:
            import fleetmap as FM
            FM.save(self.map, self.map_path)


def run(site, out=None, check_only=False, streams=None, seconds=PICTURE_S, log=print, grab=None, refit=None):
    """The self-check. Returns its record (also written to <out>/camera_selfcheck.json when out is given)."""
    import ref_tags as RT
    import tag_bridge as TB
    import tag_setup as TSU
    import e02_camera as CAM
    m = "check" if check_only else mode()
    rec = {"version": VERSION, "t": time.time(), "mode": m, "site": (site or {}).get("name")}
    if m == "off":
        rec.update(result="off", words="switched off (MIRLT_CAMERA_SELFCHECK=0): the saved camera model is trusted")
        return _done(rec, out, log)
    cal_dir = os.path.join((site or {}).get("conf_dir") or os.path.expanduser("~/.config/mirlt"), "tagcal")
    rc = (site or {}).get("ref_tags") or {}
    if not rc or rc.get("use") is False:
        rec.update(result="unknown", words="this site has no reference tags: nothing to check the camera against")
        return _done(rec, out, log)
    models = {}
    for c in ("overhead", "side"):
        p = os.path.join(cal_dir, c + ".json")
        if os.path.exists(p):
            models[c] = CAM.load_camera(p)
    fitp = os.path.join(cal_dir, "site_fit.json")
    if not models or not os.path.exists(fitp):
        rec.update(result="unknown", words="no camera model / site_fit.json in %s yet: run Measure once (Mission "
                                           "Control, Setup)" % cal_dir)
        return _done(rec, out, log)
    cfg = dict(RT.default_config(), **rc)
    exp = TB.ref_expected(models, fitp, cfg)
    streams = streams or {c: u for c, u in {"overhead": "127.0.0.1:8000", "side": "127.0.0.1:8001"}.items()
                          if c in models}
    pics = (grab or TSU.grab_pictures)(streams, seconds)
    seen = RT.read_refs(pics, cfg)
    cmp_ = compare(exp, seen)
    limit = REFIT_PX if REFIT_PX is not None else RT.BUMP_PX
    v, words = verdict(cmp_, limit)
    rec.update(cameras=cmp_, limit_px=limit, pictures={c: len(p) for c, p in pics.items()})
    if v == "ok":
        rec.update(result="ok", words="CAMERA CHECKS OUT against the reference tags (%s; limit %.0f px)" % (words, limit))
        return _done(rec, out, log)
    if v == "unknown":
        rec.update(result="unknown", words="CAMERA NOT CHECKED: %s; the saved model is used (the bridge's bump watch "
                                           "keeps checking)" % words)
        return _done(rec, out, log)
    if m == "check":
        rec.update(result="disagrees", words="THE CAMERA DISAGREES with its saved model (%s; limit %.0f px). Not "
                                             "refitted (check only): Measure again in Mission Control" % (words, limit))
        return _done(rec, out, log)
    log("THE CAMERA DISAGREES with its saved model (%s; limit %.0f px): measuring again (as Mission Control's "
        "Measure) ..." % (words, limit))
    before = _model_summary(cal_dir)
    try:
        res = (refit or _refit)(site)
    except Exception as e:  # noqa: BLE001  (the run goes on with the old model; said)
        rec.update(result="refit failed", words="THE CAMERA DISAGREES (%s) and the refit failed: %s. The OLD model is "
                                                "used; Measure again in Mission Control" % (words, e))
        return _done(rec, out, log)
    after = _model_summary(cal_dir)
    rec.update(result="refitted", before=before, after=after, refit={k: res.get(k) for k in (
        "size_cm", "rms_px", "saved_to", "words") if isinstance(res, dict)})
    ch = _changes(before, after)
    rec["words"] = "CAMERA REFITTED: it disagreed with its saved model (%s). %s The old model is kept beside it " \
                   "(tagcal-before-<time>)." % (words, ch)
    return _done(rec, out, log)


def _refit(site):
    import tag_job as TJ
    mc = (site or {}).get("mc_dir") or os.path.join(REPO, "logs", "mission-control-%s" % (site or {}).get("name"))
    os.makedirs(mc, exist_ok=True)
    return TJ.TagJob(_Session(mc), site).measure("robots")


def _model_summary(cal_dir):
    out = {}
    try:
        with open(os.path.join(cal_dir, "site_fit.json")) as f:
            fit = json.load(f)
        out["size_cm"] = fit.get("size_cm")
        out["rms_px"] = fit.get("rms_px")
    except (OSError, ValueError):
        pass
    for c in ("overhead", "side"):
        p = os.path.join(cal_dir, c + ".json")
        try:
            with open(p) as f:
                d = json.load(f)
            out[c] = {k: d.get(k) for k in ("f", "fx", "height_cm", "z_cm", "C", "center") if k in d}
        except (OSError, ValueError):
            pass
    return out


def _changes(before, after):
    ws = []
    if before.get("size_cm") != after.get("size_cm"):
        ws.append("table %s -> %s cm" % (before.get("size_cm"), after.get("size_cm")))
    if before.get("rms_px") != after.get("rms_px"):
        ws.append("fit error %s -> %s px" % (before.get("rms_px"), after.get("rms_px")))
    for c in ("overhead", "side"):
        if before.get(c) != after.get(c) and (before.get(c) or after.get(c)):
            ws.append("the %s camera's model changed" % c)
    return ("What changed: " + "; ".join(ws) + ".") if ws else "Nothing in the model's summary changed."


def _done(rec, out, log):
    if log:
        log(rec.get("words", ""))
    if out:
        try:
            os.makedirs(out, exist_ok=True)
            with open(os.path.join(out, "camera_selfcheck.json"), "w") as f:
                json.dump(rec, f, indent=1, default=str)
        except OSError:
            pass
    return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--check-only", action="store_true")
    a = ap.parse_args(argv)
    if a.site:
        os.environ["MIRLT_SITE"] = a.site
    import sites as SI
    rec = run(SI.load(a.site or None), out=a.out or None, check_only=a.check_only)
    return 0 if rec.get("result") in ("ok", "refitted", "off", "unknown") else 1


if __name__ == "__main__":
    sys.exit(main())
