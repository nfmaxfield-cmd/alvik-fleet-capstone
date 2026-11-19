"""How old is a picture when the tag bridge has finished reading it? Read-only on Jay-Sun's run folders.
bridge/detect_times.csv per picture: t (the picture's stamp), ms (the detector's time on it), t_done (when the bridge
had finished it). age = t_done - t. The robots' firmware refuses a fix older than fix_max_age_ms (1500 ms in the e21
firmware, 1000 ms before). python3 -I picture_age.py <jayruns> [out.json]"""
import csv, glob, json, os, sys
root = sys.argv[1]
out = {}
def q(v, p):
    v = sorted(v)
    return round(v[min(len(v) - 1, int(p * len(v)))], 3)
for run in sorted(glob.glob(os.path.join(root, "*"))):
    f = os.path.join(run, "bridge", "detect_times.csv")
    if not os.path.exists(f):
        continue
    by = {}
    for r in csv.DictReader(open(f)):
        try:
            t, td, ms = float(r["t"]), float(r["t_done"]), float(r["ms"])
        except (ValueError, KeyError, TypeError):
            continue
        by.setdefault(r["cam"], []).append((t, td - t, ms))
    rec = {}
    for cam, v in by.items():
        v = v[20:]                      # the start-up searches left out
        if len(v) < 30:
            continue
        age = [a for _, a, _ in v]
        gaps = [b[0] - a[0] for a, b in zip(v, v[1:])]
        rec[cam] = {"pictures": len(v), "age_median_s": q(age, 0.5), "age_p90_s": q(age, 0.9), "age_max_s": q(age, 1.0),
                    "over_1.0_s_pct": round(100.0 * sum(a > 1.0 for a in age) / len(age), 1),
                    "over_1.5_s_pct": round(100.0 * sum(a > 1.5 for a in age) / len(age), 1),
                    "detect_ms_median": q([m for _, _, m in v], 0.5), "detect_ms_p90": q([m for _, _, m in v], 0.9),
                    "gap_between_pictures_median_s": q(gaps, 0.5), "per_s": round(len(v) / max(1e-6, v[-1][0] - v[0][0]), 2)}
    if rec:
        out[os.path.basename(run)] = rec
for k, v in out.items():
    for cam, r in v.items():
        print(k[9:15], cam.ljust(8), r)
if len(sys.argv) > 2:
    json.dump(out, open(sys.argv[2], "w"), indent=1)
