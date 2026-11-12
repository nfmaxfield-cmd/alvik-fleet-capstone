#!/usr/bin/env python3
"""(e19, the Fable audit H2, 12 Nov) Finds what simulator ("dry") runs wrote into the REAL robots' settings, and repairs
it without deleting anything.

Before e19 a dry free roam wrote into the same settings folder as the real robots (~/.config/mirlt):
  - robots/<name>.json: each run's wheel / gyro fit, pooled; the next REAL run sends the pooled value to the robot's
    flash (corr save wheel_scale / gyro_scale). A dry run's fit is the simulator's, not the robot's;
  - workspace/<site>.json: the proven workspace (where robots have driven, where an edge dropped);
  - heard_robots.json: the emergency stop's list of robot addresses (a simulated robot is 127.0.0.x: a dry run
    replaced Left's real address with a loopback one).
From e19 a dry run has its own settings folder (~/.config/mirlt-dry, freeroam.dry_conf_dir), loopback addresses are
never remembered, and every pooled row and workspace save records its source (dry / real, the run's folder).

How each entry is judged (observe, then trust: evidence, not guesses):
  - a row with a recorded source: that source;
  - an untagged row (written before e19): matched against the run folders' discovery.json fits (logs/*-dryrun* =
    dry; other free-roam folders = real), then against the dry-run folders' time windows (run.json started..finished);
    nothing matches: "unknown" (said, left in, never guessed);
  - the workspace: its recorded saves; untagged: dry when its last change falls inside a dry run's window, or when a
    dry-run folder holds a workspace.json (a dry run saved it at least once: its felt cells are mixed in).

    python3 tools/fleet/dry_check.py check  [--conf-dir DIR] [--logs DIR]     says what it finds; changes nothing
    python3 tools/fleet/dry_check.py repair [--conf-dir DIR] [--logs DIR]     backs up, then quarantines what is dry

Repair: every file it changes is copied first into <conf_dir>/backup-dry-check-<date-time>/. Dry rows move from
"runs" into "runs_quarantined_dry" in the same file (kept, not deleted) and the pool is worked out again from the real
rows; a dry-touched workspace is moved into the backup folder (the next real run starts it again from the camera and its
own drives); loopback addresses move into "quarantined_dry" in heard_robots.json. Run automatically, with a clear
message, at the start of every real free roam (freeroam.py).
"""
import argparse
import datetime as dt
import glob
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

VERSION = "dry-check-2026-11-12e19"   # e19 (audit H2): first version
MATCH_TOL = 1e-4                      # a stored fit equals a run's fit (both are rounded to 4 places): not chosen
WINDOW_PAD_S = 120.0                  # a dry run's window is padded by this at each end (its end saves come after
                                      # "finished" is written). Chosen; no DOE (a bookkeeping margin)


def _load(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _stamp(s):
    """'2026-11-12 13:24:35' / ISO -> unix seconds (local time), or None."""
    if not s:
        return None
    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%dT%H:%M:%S"):
        try:
            return dt.datetime.strptime(str(s)[:19], fmt).timestamp()
        except ValueError:
            continue
    return None


def run_folders(logs):
    """Every run folder under logs with what it tells: dry or real, its time window, its discovery fits, whether it
    saved a workspace."""
    out = []
    for d in sorted(glob.glob(os.path.join(logs, "*"))):
        if not os.path.isdir(d):
            continue
        rj = _load(os.path.join(d, "run.json")) or {}
        disc = _load(os.path.join(d, "discovery.json")) or {}
        name = os.path.basename(d)
        dry = bool(rj.get("dry")) or "dryrun" in name or "-dry" in name
        t0, t1 = _stamp(rj.get("started")), _stamp(rj.get("finished"))
        if t0 is None:
            try:
                t0 = os.path.getmtime(os.path.join(d, "run.json"))
            except OSError:
                t0 = None
        if t1 is None and t0 is not None:
            try:
                t1 = max(os.path.getmtime(os.path.join(d, f)) for f in os.listdir(d))
            except (OSError, ValueError):
                t1 = t0
        out.append(dict(dir=d, name=name, dry=dry, t0=t0, t1=t1, fits=disc.get("fits") or {},
                        ws=os.path.exists(os.path.join(d, "workspace.json"))))
    return out


def _same_fit(row, fit):
    keys = [k for k in ("wheel", "gyro", "wheel_se", "gyro_se") if row.get(k) is not None or fit.get(k) is not None]
    if not keys or not any(row.get(k) is not None for k in ("wheel", "gyro")):
        return False
    for k in keys:
        a, b = row.get(k), fit.get(k)
        if a is None or b is None or abs(float(a) - float(b)) > MATCH_TOL:
            return False
    return True


def judge_row(robot, row, runs):
    """('dry' | 'real' | 'unknown', why) for one pooled row."""
    src = row.get("source")
    if src in ("dry", "real"):
        return src, "recorded: %s run %s" % (src, row.get("run", "?"))
    hits = [r for r in runs if _same_fit(row, r["fits"].get(robot) or {})]
    if hits:
        kinds = {("dry" if r["dry"] else "real") for r in hits}
        k = "dry" if kinds == {"dry"} else "real" if kinds == {"real"} else "unknown"
        return k, "its values are the fit in %s" % ", ".join(r["name"] for r in hits[:3])
    t = _stamp(row.get("t"))
    if t is not None:
        win = [r for r in runs if r["t0"] is not None and r["t0"] - WINDOW_PAD_S <= t <= (r["t1"] or r["t0"]) +
               WINDOW_PAD_S]
        kinds = {("dry" if r["dry"] else "real") for r in win}
        if kinds == {"dry"}:
            return "dry", "written at %s, during the dry run %s" % (row.get("t"), win[0]["name"])
        if kinds == {"real"}:
            return "real", "written at %s, during the real run %s" % (row.get("t"), win[0]["name"])
    return "unknown", "no source recorded and no run folder matches (written before e19)"


def check(conf_dir, logs):
    """Everything found, as a dict; changes nothing."""
    conf_dir = os.path.expanduser(conf_dir)
    runs = run_folders(logs)
    rep = dict(version=VERSION, conf_dir=conf_dir, logs=logs, n_runs=len(runs), n_dry_runs=sum(r["dry"] for r in runs),
               robots={}, workspace={}, heard={}, contaminated=False)
    for p in sorted(glob.glob(os.path.join(conf_dir, "robots", "*.json"))):
        d = _load(p) or {}
        robot = d.get("robot") or os.path.splitext(os.path.basename(p))[0]
        rows = []
        for i, row in enumerate(d.get("runs") or []):
            k, why = judge_row(robot, row, runs)
            rows.append(dict(i=i, kind=k, why=why, t=row.get("t"), wheel=row.get("wheel"), gyro=row.get("gyro")))
        nd = sum(r["kind"] == "dry" for r in rows)
        rep["robots"][robot] = dict(path=p, rows=rows, dry=nd, unknown=sum(r["kind"] == "unknown" for r in rows),
                                    pooled_now=d.get("pooled"),
                                    pooled_clean=pool_rows([r for r, j in zip(d.get("runs") or [], rows)
                                                            if j["kind"] != "dry"]))
        rep["contaminated"] |= nd > 0
    for p in sorted(glob.glob(os.path.join(conf_dir, "workspace", "*.json"))):
        d = _load(p) or {}
        saves = d.get("saved_by") or []
        why = None
        if saves:
            dry_s = [s for s in saves if s.get("dry")]
            if dry_s:
                why = "a dry run saved it (%s)" % ", ".join(str(s.get("run")) for s in dry_s[:3])
        else:
            try:
                mt = os.path.getmtime(p)
            except OSError:
                mt = None
            last = [r for r in runs if mt is not None and r["t0"] is not None and
                    r["t0"] - WINDOW_PAD_S <= mt <= (r["t1"] or r["t0"]) + WINDOW_PAD_S]
            wrote = [r for r in runs if r["dry"] and r["ws"]]
            if last and all(r["dry"] for r in last):
                why = "its last change (%s) was during the dry run %s" % (
                    dt.datetime.fromtimestamp(mt).strftime("%Y-%m-%d %H:%M:%S"), last[0]["name"])
            elif wrote:
                why = "dry runs saved the site workspace (%s): their felt cells are mixed in" % ", ".join(
                    r["name"] for r in wrote[:3])
        rep["workspace"][os.path.basename(p)] = dict(path=p, dry=why is not None, why=why or "no dry run found in it")
        rep["contaminated"] |= why is not None
    hp = os.path.join(conf_dir, "heard_robots.json")
    hd = _load(hp)
    if isinstance(hd, dict):
        lo = {n: v for n, v in hd.items() if n != "quarantined_dry" and isinstance(v, dict) and
              str(v.get("ip", "")).startswith("127.")}
        rep["heard"] = dict(path=hp, loopback=sorted(lo))
        rep["contaminated"] |= bool(lo)
    return rep


def pool_rows(rows):
    """start_discovery.pool_fit's pooling over these rows (inverse-variance means), without writing."""
    import start_discovery as D
    pooled = {}
    for k in ("wheel", "gyro"):
        xs = [(r[k], r.get(k + "_se") or 0.03) for r in rows if r.get(k) and abs(r[k] - 1.0) <= D.CAL_MAX_DEV]
        if not xs:
            continue
        w = [1.0 / max(1e-6, se * se) for _, se in xs]
        m = sum(x * wi for (x, _), wi in zip(xs, w)) / sum(w)
        se = math.sqrt(1.0 / sum(w))
        pooled[k] = dict(value=round(m, 4), se=round(se, 4), runs=len(xs), sure=se < D.CAL_SURE)
    return pooled


def words(rep):
    """The check in plain lines."""
    out = ["Dry-run check of %s (%d run folders in %s, %d of them dry):" % (rep["conf_dir"], rep["n_runs"],
                                                                         rep["logs"], rep["n_dry_runs"])]
    for robot, r in sorted(rep["robots"].items()):
        pn, pc = r["pooled_now"] or {}, r["pooled_clean"] or {}
        out.append("  %s: %d pooled rows, %d from dry runs, %d unknown%s" % (
            robot, len(r["rows"]), r["dry"], r["unknown"],
            "; wheel pool %s -> %s without them" % ((pn.get("wheel") or {}).get("value"),
                                                    (pc.get("wheel") or {}).get("value")) if r["dry"] else ""))
        for q in r["rows"]:
            if q["kind"] != "real":
                out.append("     row %d (%s): %s, %s" % (q["i"], q["t"], q["kind"].upper(), q["why"]))
    for n, w in sorted(rep["workspace"].items()):
        out.append("  workspace %s: %s" % (n, ("DRY: " + w["why"]) if w["dry"] else w["why"]))
    if rep["heard"].get("loopback"):
        out.append("  heard_robots.json: simulated (127.x) addresses for %s: the emergency stop would miss the real "
                   "robots" % ", ".join(rep["heard"]["loopback"]))
    out.append("RESULT: %s" % ("DRY-RUN DATA FOUND in the real settings (repair: python3 tools/fleet/dry_check.py "
                               "repair)" if rep["contaminated"] else "clean: nothing from dry runs found"))
    return out


def repair(conf_dir, logs, stamp=None):
    """Backs up every file it changes, then quarantines what is dry (never deletes). Returns (report, lines)."""
    rep = check(conf_dir, logs)
    lines = []
    if not rep["contaminated"]:
        return rep, ["Nothing to repair."]
    bdir = os.path.join(rep["conf_dir"], "backup-dry-check-%s" % (stamp or dt.datetime.now().strftime("%Y%m%d-%H%M%S")))
    os.makedirs(bdir, exist_ok=True)

    def backup(p):
        rel = os.path.relpath(p, rep["conf_dir"])
        dst = os.path.join(bdir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(p, dst)
        return dst
    for robot, r in rep["robots"].items():
        if not r["dry"]:
            continue
        backup(r["path"])
        d = _load(r["path"]) or {}
        rows = d.get("runs") or []
        dry_i = {q["i"] for q in r["rows"] if q["kind"] == "dry"}
        d.setdefault("runs_quarantined_dry", []).extend(
            dict(row, quarantined=dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")) for i, row in enumerate(rows)
            if i in dry_i)
        d["runs"] = [row for i, row in enumerate(rows) if i not in dry_i]
        d["pooled_before_repair"] = d.get("pooled")
        d["pooled"] = pool_rows(d["runs"])
        tmp = r["path"] + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f, indent=1)
        os.replace(tmp, r["path"])
        lines.append("%s: %d dry rows moved to runs_quarantined_dry; pool now %s" % (robot, len(dry_i), d["pooled"]))
    for n, w in rep["workspace"].items():
        if w["dry"]:
            dst = backup(w["path"])
            os.replace(w["path"], w["path"] + ".dry-quarantined")
            lines.append("workspace %s: moved aside (copy in %s; the next real run starts it again from the camera)" % (
                n, dst))
    if rep["heard"].get("loopback"):
        p = rep["heard"]["path"]
        backup(p)
        d = _load(p) or {}
        q = d.setdefault("quarantined_dry", {})
        for n in rep["heard"]["loopback"]:
            q[n] = d.pop(n)
        with open(p + ".tmp", "w") as f:
            json.dump(d, f, indent=1)
        os.replace(p + ".tmp", p)
        lines.append("heard_robots.json: simulated addresses set aside for %s" % ", ".join(rep["heard"]["loopback"]))
    lines.append("Backup of every changed file: %s" % bdir)
    return rep, lines


def at_real_start(conf_dir, logs=None, say=print):
    """(e19) Called at the start of every real free roam: check, and repair when dry data is found, said loudly. Never
    stops the run (degrade, not stop)."""
    logs = logs or os.path.join(REPO, "logs")
    try:
        rep = check(conf_dir, logs)
        if not rep["contaminated"]:
            say("SETTINGS CHECK: no dry-run data in %s" % rep["conf_dir"])
            return rep
        say("!" * 100)
        for ln in words(rep):
            say("!! " + ln)
        _, lines = repair(conf_dir, logs)
        for ln in lines:
            say("!! REPAIRED: " + ln)
        say("!! The robots' own flash may still hold a value saved from the mixed pool: this run's start re-saves the "
            "clean pool when it is sure (said below as 'calibration saved').")
        say("!" * 100)
        return rep
    except Exception as e:  # noqa: BLE001
        say("(settings check not done: %s)" % e)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("check", "repair"))
    ap.add_argument("--conf-dir", default=os.environ.get("MIRLT_CONF_DIR") or "~/.config/mirlt")
    ap.add_argument("--logs", default=os.path.join(REPO, "logs"))
    ap.add_argument("--json", action="store_true", help="the full check as JSON")
    a = ap.parse_args(argv)
    if a.what == "check":
        rep = check(a.conf_dir, a.logs)
        print(json.dumps(rep, indent=1, default=str) if a.json else "\n".join(words(rep)))
        return 1 if rep["contaminated"] else 0
    rep, lines = repair(a.conf_dir, a.logs)
    print("\n".join(words(rep)))
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
