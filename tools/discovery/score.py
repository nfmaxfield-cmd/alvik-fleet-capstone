"""Score discovery results against the hidden key. The only module that reads truth.py.

Rows in: one dict per (case, stream) with at least: split, kind, column (real name, from the key), truth, guess,
confidence, decision, derived (bool)."""
import numpy as np

from . import classify as C

CONF_BINS = (0.0, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 1.0001)


def accuracy(rows):
    n = len(rows)
    ok = sum(r["truth"] == r["guess"] for r in rows)
    return ok, n


def confusion(rows, types=None):
    types = list(types or C.TYPES)
    idx = {t: i for i, t in enumerate(types)}
    M = np.zeros((len(types), len(types)), dtype=int)
    for r in rows:
        M[idx[r["truth"]], idx[r["guess"]]] += 1
    return types, M


def calibration(rows, bins=CONF_BINS):
    """Per confidence band: guesses (excluding 'unknown' guesses), right, share right."""
    out = []
    g = [r for r in rows if r["guess"] != "unknown"]
    for lo, hi in zip(bins[:-1], bins[1:]):
        b = [r for r in g if lo <= r["confidence"] < hi]
        ok = sum(r["truth"] == r["guess"] for r in b)
        out.append({"band": "%.2f-%.2f" % (lo, min(hi, 1.0)), "n": len(b), "right": ok,
                    "share_right": ok / len(b) if b else None,
                    "mean_conf": float(np.mean([r["confidence"] for r in b])) if b else None})
    return out


def sweep(rows, accepts=(0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95), asks=(0.4, 0.5, 0.6, 0.7)):
    """For each (accept, ask) pair: labels accepted without a question, how many of those are wrong, questions
    asked (ask band), streams left unknown. A question is assumed answered correctly (the person knows what
    was plugged in), so wrong labels come only from accepted guesses."""
    out = []
    for a in accepts:
        for q in asks:
            if q > a:
                continue
            acc = [r for r in rows if r["guess"] != "unknown" and r["confidence"] >= a]
            ask = [r for r in rows if r["guess"] != "unknown" and q <= r["confidence"] < a]
            unk = [r for r in rows if r["guess"] == "unknown" or r["confidence"] < q]
            wrong = sum(r["truth"] != r["guess"] for r in acc)
            # a question shows the top 3; it fails only when the truth is not among them
            ask_miss = sum(r["truth"] not in [t for t, v in r.get("top3", [])] for r in ask)
            out.append({"accept": a, "ask": q, "accepted": len(acc), "accepted_wrong": wrong,
                        "wrong_rate": wrong / len(acc) if acc else None, "questions": len(ask),
                        "question_truth_not_offered": ask_miss, "unknown": len(unk),
                        "unknown_truly_identifiable": sum(r["truth"] not in ("unknown",) for r in unk)})
    return out


def derived_scores(rows):
    """Of the streams that really are computed (pose, drive speed, tilt, the IMU's fused yaw), how many were flagged
    derived; of the rest, how many were wrongly flagged (the centre zone, which is the mean of its neighbours,
    counts here as a sensor, so its correct flag shows as a 'sensor flagged')."""
    comp = [r for r in rows if r["truth"] in ("derived", "gyro_angle")]
    other = [r for r in rows if r["truth"] not in ("derived", "gyro_angle", "constant")]
    return {"computed": len(comp), "flagged": sum(r["derived"] for r in comp),
            "sensors": len(other), "sensors_flagged": sum(r["derived"] for r in other)}


def per_column(rows):
    """Real column -> (right, cases) across the rows."""
    out = {}
    for r in rows:
        k = r["column"]
        a, n = out.get(k, (0, 0))
        out[k] = (a + (r["truth"] == r["guess"]), n + 1)
    return dict(sorted(out.items()))
