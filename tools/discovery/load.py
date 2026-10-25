"""Load one discovery case (one robot in one session) with every RAW column's name hidden.

A case is what a real "From scratch: discover" run would have: anonymous streams (values as the robot sent them,
plus the laptop arrival time of each row), the motions the system commanded (leg.csv: go / turn, value, start and
end times), the camera track (track.csv) and, where explore or the setup measured them, the board lines.

Column names stop here. `load_case` returns a Case whose streams are called stream_01, stream_02, ... in a shuffled
order (seeded by the case id, so a rerun gives the same names) and a separate `key` {stream id: real column name}
that only `disc_score.py` may read. Nothing in features / relations / classify takes the key.

Offline only: reads session folders on disk (default root /tmp/claude-0, override with DISCOVERY_ROOT)."""
import csv
import glob
import json
import os
import random
from dataclasses import dataclass, field

import numpy as np

ROOT = os.environ.get("DISCOVERY_ROOT", "/tmp/claude-0")
CONSOLE_COLS = ("robot", "t_recv")   # added by the laptop console, not sent by the robot


@dataclass
class Leg:
    kind: str        # go / turn
    value: float     # cm or deg, sign as commanded (turn + = left)
    t0: float        # laptop time the robot said it started
    t1: float        # laptop time it said done / failed
    ok: bool


@dataclass
class Case:
    case_id: str                 # "<session>/<robot>"
    session: str
    robot: str
    t: np.ndarray                # laptop arrival time per row (s)
    streams: dict                # stream id -> np.ndarray of str (as sent)
    legs: list = field(default_factory=list)
    cam: np.ndarray = None       # (n, 3): t, x_cm, y_cm  (camera track, this robot)
    board: dict = None           # {"x": x line cm, "y": y line cm} or None
    fw: str = ""


def find_sessions(root=ROOT):
    """session id -> folder. A session copied to several places on disk is used once (the copy with the most RAW
    bytes: some copies lack a robot-logs folder; the two copies of e02-20261024-193526 differ by 450 bytes)."""
    best = {}
    for f in glob.glob(os.path.join(root, "**", "e02-2026*", "robot-logs-*", "raw.csv"), recursive=True):
        if "scratchpad" in f or "/-home-claude/" in f:
            continue
        sdir = os.path.dirname(os.path.dirname(f))
        sid = os.path.basename(sdir)
        if sid.endswith("dryrun") or "dryrun" in sdir:
            continue
        size = sum(os.path.getsize(p) for p in glob.glob(os.path.join(sdir, "robot-logs-*", "raw.csv")))
        if sid not in best or size > best[sid][0]:
            best[sid] = (size, sdir)
    return {k: v[1] for k, v in sorted(best.items())}


def _read(sdir, fname):
    out = []
    for p in sorted(glob.glob(os.path.join(sdir, "robot-logs-*", fname))):
        with open(p, newline="") as f:
            rd = csv.reader(f)
            head = None
            for row in rd:
                if not row:
                    continue
                if row[0] == "robot":          # header (a second header mid-file starts a new block)
                    head = row
                    continue
                if head is None or len(row) != len(head):
                    continue
                out.append(dict(zip(head, row)))
    return out


def _f(v):
    try:
        x = float(v)
        return x if np.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def robots_in(sdir):
    seen = {}
    for p in sorted(glob.glob(os.path.join(sdir, "robot-logs-*", "raw.csv"))):
        with open(p, newline="") as f:
            for row in csv.reader(f):
                if row and row[0] != "robot":
                    seen[row[0]] = seen.get(row[0], 0) + 1
    return seen


def load_case(sdir, robot, seed_extra=""):
    """Returns (Case, key). key = {stream id: real column name}: for scoring only."""
    sid = os.path.basename(sdir)
    raw = [r for r in _read(sdir, "raw.csv") if r.get("robot") == robot]
    raw.sort(key=lambda r: float(r["t_recv"]))
    cols = [c for c in (raw[0].keys() if raw else []) if c not in CONSOLE_COLS]
    rng = random.Random("%s/%s%s" % (sid, robot, seed_extra))
    order = cols[:]
    rng.shuffle(order)
    key, streams = {}, {}
    for i, c in enumerate(order):
        s = "stream_%02d" % (i + 1)
        key[s] = c
        streams[s] = np.array([r.get(c, "") for r in raw], dtype=object)
    t = np.array([float(r["t_recv"]) for r in raw])

    legs, open_ = [], {}
    for r in _read(sdir, "leg.csv"):
        if r.get("robot") != robot or r.get("kind") not in ("go", "turn"):
            continue
        tr, res = _f(r.get("t_recv")), r.get("result")
        if tr is None:
            continue
        k = (r.get("id"), r.get("kind"))
        if res == "started":
            open_[k] = (tr, _f(r.get("value")))
        elif res in ("done", "failed") and k in open_:
            t0, val = open_.pop(k)
            if val is not None and tr > t0:
                legs.append(Leg(r["kind"], val, t0, tr, res == "done"))
    legs.sort(key=lambda g: g.t0)

    cam = []
    tp = os.path.join(sdir, "track.csv")
    if os.path.exists(tp):
        with open(tp, newline="") as f:
            for r in csv.DictReader(f):
                if r.get("robot") != robot:
                    continue
                q = (_f(r.get("t")), _f(r.get("x_cm")), _f(r.get("y_cm")))
                if None not in q:
                    cam.append(q)
    cam = np.array(sorted(cam)) if cam else np.zeros((0, 3))

    board = None
    sp = os.path.join(sdir, "e02_state.json")
    if os.path.exists(sp):
        try:
            with open(sp) as f:
                b = json.load(f).get("board") or {}
            if b.get("xboard_cm") and b.get("yboard_cm"):
                board = {"x": float(b["xboard_cm"]), "y": float(b["yboard_cm"]),
                         "xs": int(b.get("xboard_side", 1)), "ys": int(b.get("yboard_side", 1))}
        except (OSError, ValueError):
            board = None

    fw = ""
    info = [r for r in _read(sdir, "runinfo.csv") if r.get("robot") == robot]
    if info:
        fw = info[-1].get("fw_version", "")
    return Case("%s/%s" % (sid, robot), sid, robot, t, streams, legs, cam, board, fw), key


def all_cases(root=ROOT, min_rows=200):
    """Every (session, robot) on disk with at least min_rows RAW rows, as (session id, folder, robot, rows)."""
    out = []
    for sid, sdir in find_sessions(root).items():
        for robot, n in sorted(robots_in(sdir).items()):
            if n >= min_rows:
                out.append((sid, sdir, robot, n))
    return out
