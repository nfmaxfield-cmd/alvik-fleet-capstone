"""One dry run of the 2026-10-25 runner-fix build, summarised: holds, the scene's events, explore's legs checked and
finished, the camera's pictures a second during explore's fits (from the 'e02 heavy' marks, or the fit's log lines
with --fits-in-runner), the stall watch, and the speed guard.  python3 runsum.py LOG_FOLDER [...]"""
import csv
import json
import os
import re
import statistics as S
import sys


def fit_windows(d, st):
    """[(t0, t1)] of explore's fits: from heavy_calls, else from the log's 'fitting the camera' to 'camera fit:'."""
    hc = [(c[0], c[0] + c[1]) for c in st.get("heavy_calls") or [] if c[2] == "fit"]
    if hc:
        return hc
    out, t0 = [], None
    import datetime as dt
    day = None
    for line in open(os.path.join(d, "e02_log.txt"), errors="replace"):
        m = re.match(r"\[(\d\d):(\d\d):(\d\d)\]", line)
        if not m:
            continue
        if day is None:
            day = dt.datetime.fromtimestamp(st["created"] if isinstance(st.get("created"), (int, float)) else
                                            os.path.getmtime(os.path.join(d, "e02_log.txt"))).date()
        t = dt.datetime(day.year, day.month, day.day, int(m.group(1)), int(m.group(2)), int(m.group(3))).timestamp()
        if "fitting the camera and the board" in line or "rays aimed at the sides" in line and t0 is None:
            t0 = t
        if ("camera fit:" in line or "LEFT OUT of the camera" in line) and t0 is not None:
            out.append((t0, t))
            t0 = None
    return out


def main():
    for d in sys.argv[1:]:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        perf = list(csv.DictReader(open(os.path.join(d, "perf.csv"))))
        fps = [(float(r["t"]), float(r["fps_handled"]), float(r["age_mean_s"]), r["phase"]) for r in perf]
        win = fit_windows(d, st)
        ex = [f for t, f, a, ph in fps if ph in ("exploring", "finding")]
        during = [f for t, f, a, ph in fps if any(a0 <= t <= a1 + 1.0 for a0, a1 in win)]
        rep = dict(folder=os.path.basename(d), holds=[[h[1], h[2], h[3][:60], h[5]] for h in st.get("holds") or []],
                   scene=st.get("dry_scene"), resumed=(st.get("setup") or {}).get("explore_resumed"),
                   rays_not_driven=(st.get("setup") or {}).get("explore_rays_not_driven"),
                   legs_checked=(st.get("setup") or {}).get("explore_legs_checked"),
                   skipped_rays=len((st.get("setup") or {}).get("explore_skipped_rays") or []),
                   gap_rays=(st.get("setup") or {}).get("explore_gap_rays"),
                   explore=(st.get("setup") or {}).get("explore"), failed=(st.get("setup") or {}).get("explore_failed"),
                   watchdog_stops=[w[1:3] for w in st.get("watchdog_stops") or []],
                   heavy=st.get("heavy_calls"), fit_windows_s=[round(b - a, 1) for a, b in win],
                   fps_explore_median=S.median(ex) if ex else None,
                   fps_during_fits_median=S.median(during) if during else None,
                   fps_during_fits_min=min(during) if during else None,
                   slow_spans=st.get("slow_spans"), stall=st.get("stall_summary"),
                   speed_guard_stops=st.get("speed_guard_stops"), speed_guard_waits=st.get("speed_guard_waits"),
                   baseline=st.get("speed_baseline"), perf=st.get("perf_summary"))
        print(json.dumps(rep, indent=1, default=str))


if __name__ == "__main__":
    main()
