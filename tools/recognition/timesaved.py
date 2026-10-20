"""Time-saving stats for a whole project (Maxfield IRL Tracking).

Reads the per-recording record the labelling page writes (<project>/stats/<recording>.jsonl,
see tools/label/timing.py), adds estimates for frames finished before the record existed
(from review.json, marked "estimated"), and sums it all up by model version: seconds per frame,
share accepted unchanged, edits per frame, frames accepted without looking and for having the
same picture, plus setup and install time.

    python tools/recognition/timesaved.py <project folder>      writes stats/time-saved-frames.csv
                                                                 and stats/time-saved-summary.csv
"""
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LABEL = os.path.join(os.path.dirname(HERE), "label")
for p in (HERE, LABEL):
    if p not in sys.path:
        sys.path.insert(0, p)

import timing  # noqa: E402


def stats_dir(proj):
    return os.path.join(proj.folder, "stats")


def stats_path(proj, rid):
    return os.path.join(stats_dir(proj), "%s.jsonl" % rid)


def record_install(proj, seconds):
    """start.py measured how long the first install took: kept in stats/install.json (once)."""
    p = os.path.join(stats_dir(proj), "install.json")
    if os.path.exists(p):
        return
    os.makedirs(stats_dir(proj), exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(dict(install_seconds=round(float(seconds), 1), recorded=timing.now_iso()), f, indent=1)


def install_seconds(proj):
    p = os.path.join(stats_dir(proj), "install.json")
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f).get("install_seconds")
    except (OSError, ValueError):
        return None


def _unsure_frames(label_dir):
    p = os.path.join(label_dir, "labels.csv")
    out = set()
    if os.path.exists(p):
        with open(p, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                if (r.get("status") or "") == "unsure":
                    out.add(int(r["frame"]))
    return out


def _first_model_time(proj):
    ts = [m.get("created") for m in proj.models() if m.get("created")]
    return min(ts) if ts else None


def project_frames(proj):
    """One dict per finished frame of every recording (timing.CSV_COLUMNS)."""
    out = []
    first_model = _first_model_time(proj)
    for r in proj.recordings():
        rid = r["id"]
        recs = timing.read(stats_path(proj, rid))
        latest = timing.latest_per_frame(recs)
        for f, (last, spent) in sorted(latest.items()):
            out.append(dict(recording=rid, frame=f, shown=last.get("shown"), saved=last.get("saved"),
                            active_s=round(spent, 1) if last.get("outcome") in timing.PERSON else float(last.get("active_s") or 0.0),
                            edits=last.get("edits"), outcome=last.get("outcome"), how=last.get("how"),
                            guesses=last.get("guesses"), model=last.get("model") or "unknown",
                            objects=last.get("objects"), estimated=False))
        rp = os.path.join(proj.labels_dir(rid), "review.json")
        if os.path.exists(rp):
            try:
                with open(rp, encoding="utf-8") as fh:
                    review = json.load(fh)
            except ValueError:
                review = {}
            for e in timing.estimate_from_review(review, _unsure_frames(proj.labels_dir(rid)), latest.keys(), first_model):
                out.append(dict(e, recording=rid))
    return out


def summary(proj):
    st = proj.data.get("settings", {})
    return timing.summarize(project_frames(proj), setup_seconds=st.get("setup_seconds"),
                            install_seconds=install_seconds(proj))


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import project as projmod
    proj = projmod.Project(sys.argv[1])
    frames = project_frames(proj)
    os.makedirs(stats_dir(proj), exist_ok=True)
    a = os.path.join(stats_dir(proj), "time-saved-frames.csv")
    b = os.path.join(stats_dir(proj), "time-saved-summary.csv")
    with open(a, "w", newline="", encoding="utf-8") as f:
        f.write(timing.frames_csv(frames))
    with open(b, "w", newline="", encoding="utf-8") as f:
        f.write(timing.summary_csv(summary(proj)))
    print("Wrote %s\nWrote %s" % (a, b))


if __name__ == "__main__":
    main()
