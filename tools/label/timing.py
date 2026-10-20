"""Time-saving record: how long labelling takes, frame by frame, and how much each model saves.

Every frame a person finishes adds one line to an append-only file (the recognition studio
keeps one per recording in <project>/stats/<recording>.jsonl):

    {"v": 1, "frame": 120, "shown": "...", "saved": "...", "active_s": 7.4, "edits": 2,
     "outcome": "corrected", "how": "user", "guesses": "motion", "model": "none", "objects": 3}

outcome: accepted (as guessed, nothing changed) / corrected / unsure (holds an object marked
unsure) / auto (accepted without looking) / same (accepted for having the same picture as a
frame a person accepted) / undone (taken back: the frame no longer counts).
active_s: seconds the person spent on the frame, counting pauses longer than IDLE_S as IDLE_S,
at most CAP_S per frame. edits: changes made on the frame before it was finished.

Frames finished before this record existed are estimated from review.json (the time between one
saved frame and the one before it, capped at CAP_S) and marked "estimated".
"""
import csv
import datetime as dt
import io
import json
import os
import statistics

CAP_S = 120.0
IDLE_S = 20.0
PERSON = ("accepted", "corrected", "unsure")
OUTCOMES = PERSON + ("auto", "same", "undone")
CSV_COLUMNS = ["recording", "frame", "shown", "saved", "active_s", "edits", "outcome", "how", "guesses", "model",
               "objects", "estimated"]


def now_iso():
    return dt.datetime.now().isoformat(timespec="seconds")


def _parse(t):
    try:
        return dt.datetime.fromisoformat(str(t))
    except (TypeError, ValueError):
        return None


def append(path, rec):
    """Add one line (never rewrites the file)."""
    if not path:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    line = json.dumps(dict(v=1, **rec), separators=(",", ":"))
    with open(path, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def read(path):
    """Every line of a record file (lines that can't be read are skipped)."""
    out = []
    if not path or not os.path.exists(path):
        return out
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if isinstance(r, dict) and "frame" in r:
                out.append(r)
    return out


def clean_timing(t):
    """What the page sends with a finished frame -> (shown, active_s, edits), checked."""
    t = t or {}
    try:
        active = float(t.get("active_s") or 0.0)
    except (TypeError, ValueError):
        active = 0.0
    try:
        edits = int(t.get("edits") or 0)
    except (TypeError, ValueError):
        edits = 0
    shown = t.get("shown")
    return (str(shown)[:32] if shown else None), round(min(CAP_S, max(0.0, active)), 1), max(0, min(edits, 10000))


def outcome_of(objects, changed):
    if any(o.get("status") == "unsure" for o in objects):
        return "unsure"
    return "corrected" if changed else "accepted"


# --------------------------------------------------------------------------------------------------
# Estimates for frames finished before the record existed
# --------------------------------------------------------------------------------------------------

def estimate_from_review(review, unsure_frames=(), recorded=(), first_model_time=None):
    """Records for finished frames of a review.json that have no line in the record.
    Time per frame: the gap since the previous saved frame (any frame); a gap longer than CAP_S is
    a break, not work, so that frame gets the median of the other gaps (as does the first one).
    Model: "none" if saved before the first model existed, else "unknown". Every record says
    estimated=True. Frames changed only by Investigate's "change all" (how change_all) were not
    checked one by one: they count like auto-accepted frames (no time)."""
    frames = review.get("frames") or {}
    recorded = set(int(f) for f in recorded)
    unsure = set(int(f) for f in unsure_frames)
    first_model = _parse(first_model_time) if first_model_time else None
    person = []
    for fs, st in frames.items():
        t = _parse(st.get("updated"))
        if st.get("state") == "reviewed" and st.get("how") != "change_all" and t is not None:
            person.append((t, int(fs), st))
    person.sort()
    gaps = {}
    for k, (t, f, st) in enumerate(person):
        if k:
            g = max(0.0, (t - person[k - 1][0]).total_seconds())
            gaps[f] = g if g <= CAP_S else None          # None: a break
    work = [g for g in gaps.values() if g is not None]
    fill = statistics.median(work) if work else None
    out = []
    for t, f, st in person:
        if f in recorded:
            continue
        g = gaps.get(f)
        if g is None:
            g = fill
        model = "none" if (first_model is None or t < first_model) else "unknown"
        out.append(dict(frame=f, shown=None, saved=st.get("updated"), active_s=None if g is None else round(g, 1),
                        edits=None, outcome="unsure" if f in unsure else ("corrected" if st.get("changed") else "accepted"),
                        how=st.get("how") or "user", guesses=None, model=model, objects=None, estimated=True))
    for fs, st in frames.items():
        f = int(fs)
        bulk = st.get("state") == "reviewed" and st.get("how") == "change_all"
        if (st.get("state") == "auto" or bulk) and f not in recorded:
            t = _parse(st.get("updated"))
            model = "none" if (first_model is None or (t is not None and t < first_model)) else "unknown"
            out.append(dict(frame=f, shown=None, saved=st.get("updated"), active_s=0.0, edits=0,
                            outcome="same" if st.get("how") == "same" else "auto", how=st.get("how") or "auto",
                            guesses=None, model=model, objects=None, estimated=True))
    return out


def latest_per_frame(records):
    """frame -> (latest record, seconds spent on that frame over all its records)."""
    out = {}
    for r in records:
        f = int(r["frame"])
        spent = out.get(f, (None, 0.0))[1] + float(r.get("active_s") or 0.0)
        out[f] = (r, spent)
    return out


# --------------------------------------------------------------------------------------------------
# Summary
# --------------------------------------------------------------------------------------------------

def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 1) if xs else None


def _median(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 1) if xs else None


def _model_order(m):
    if m == "none":
        return (0, 0)
    if m.startswith("v") and m[1:].isdigit():
        return (1, int(m[1:]))
    return (2, 0)


def summarize(frames, setup_seconds=None, install_seconds=None):
    """frames: list of per-frame dicts (CSV_COLUMNS; active_s = all the time spent on it).
    Returns totals and one row per guess source: model version and kind of guesses (motion,
    model, either "+camera"), measured frames and estimated frames in separate rows, so models are
    compared only on measured frames. Frames accepted without a person looking (auto, same
    picture) are counted, never credited as time saved; estimate_if_checked_s says what checking
    them would have cost at the measured no-model pace (an estimate: they are the easier frames)."""
    groups = {}
    for r in frames:
        if r["outcome"] == "undone":
            continue
        key = (r.get("model") or "unknown", r.get("guesses") or "", bool(r.get("estimated")))
        groups.setdefault(key, []).append(r)
    rows = []
    for key in sorted(groups, key=lambda k: (k[2], _model_order(k[0]), k[0], k[1])):
        m, guesses, est = key
        rs = groups[key]
        person = [r for r in rs if r["outcome"] in PERSON]
        timed = [r for r in person if r.get("active_s") is not None]
        exact = [r for r in person if not r.get("estimated")]
        rows.append(dict(model=m, guesses=guesses, measured=not est, frames=len(person),
                         seconds_per_frame=_mean(r["active_s"] for r in timed),
                         median_seconds=_median(r["active_s"] for r in timed),
                         accepted_unchanged=round(sum(1 for r in person if r["outcome"] == "accepted") / len(person), 3)
                         if person else None,
                         edits_per_frame=_mean(float(r["edits"]) for r in exact if r.get("edits") is not None),
                         unsure=sum(1 for r in person if r["outcome"] == "unsure"),
                         auto=sum(1 for r in rs if r["outcome"] == "auto"),
                         same=sum(1 for r in rs if r["outcome"] == "same"),
                         estimated=sum(1 for r in rs if r.get("estimated")),
                         seconds=round(sum(float(r["active_s"] or 0) for r in timed), 1)))
    person_all = [r for r in frames if r["outcome"] in PERSON]
    timed_all = [r for r in person_all if r.get("active_s") is not None]
    measured = [r for r in timed_all if not r.get("estimated")]
    estimated = [r for r in timed_all if r.get("estimated")]
    bases = [r for r in rows if r["measured"] and r["model"] == "none" and r["seconds_per_frame"] is not None
             and r["frames"] >= 5]
    base = max(bases, key=lambda r: r["frames"]) if bases else None
    auto = sum(1 for r in frames if r["outcome"] == "auto")
    same = sum(1 for r in frames if r["outcome"] == "same")
    saved_models = 0.0
    if base:
        for r in rows:
            if r["measured"] and r["model"] not in ("none", "unknown") and r["seconds_per_frame"] is not None:
                saved_models += (base["seconds_per_frame"] - r["seconds_per_frame"]) * r["frames"]
    totals = dict(frames_by_person=len(person_all), seconds_by_person=round(sum(float(r["active_s"] or 0) for r in timed_all), 1),
                  measured_frames=len(measured), seconds_per_frame=_mean(r["active_s"] for r in measured),
                  estimated_frames=len(estimated), estimated_seconds_per_frame=_mean(r["active_s"] for r in estimated),
                  auto=auto, same=same,
                  estimated=sum(1 for r in frames if r.get("estimated")),
                  baseline_seconds_per_frame=base["seconds_per_frame"] if base else None, baseline_measured=bool(base),
                  saved_by_accepting_s=None,
                  estimate_if_checked_s=round((auto + same) * base["seconds_per_frame"], 0) if base and (auto + same) else None,
                  saved_by_models_s=round(saved_models, 0) if base else None,
                  setup_seconds=setup_seconds, install_seconds=install_seconds)
    return dict(totals=totals, by_model=rows)


def frames_csv(frames):
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(CSV_COLUMNS)
    for r in frames:
        w.writerow(["" if r.get(c) is None else (int(r[c]) if c == "estimated" else r[c]) for c in CSV_COLUMNS])
    return buf.getvalue()


SUMMARY_COLUMNS = ["model", "guesses", "measured", "frames", "seconds_per_frame", "median_seconds", "accepted_unchanged", "edits_per_frame",
                   "unsure", "auto", "same", "estimated", "seconds"]


def summary_csv(summary):
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(SUMMARY_COLUMNS)
    for r in summary["by_model"]:
        w.writerow(["" if r.get(c) is None else r[c] for c in SUMMARY_COLUMNS])
    w.writerow([])
    w.writerow(["total", "value"])
    for k, v in summary["totals"].items():
        w.writerow([k, "" if v is None else v])
    return buf.getvalue()
