"""One dry run of the 2026-10-25 15:45-fix build, summarised (docs/build-reports/e02-oct25-1545-fixes.md): turn faults
and how soon after the simulator broke the turns, alerts (the live page's sounds), heading crawls, cut turns, moves
away from an edge before a turn, check drives, slow turns, back-outs, skipped held fixes, blocked routes, holds,
watchdog stops, runs scored, explore's result, each robot's closest true approach to an open edge (simulator truth),
and the camera's pictures a second.   python3 runsum.py LOG_FOLDER [...]"""
import csv
import json
import os
import statistics as S
import sys


def closest_edge(d):
    """{robot: (cm, t)}: each robot's true centre's closest approach to the open edges x = 0 and y = 0 (the
    simulator's board with the boards at the far sides, --board-sides 1,1)."""
    out = {}
    p = os.path.join(d, "sim", "truth.csv")
    if not os.path.exists(p):
        return out
    for r in csv.DictReader(open(p)):
        v = min(float(r["x_cm"]), float(r["y_cm"]))
        if r["robot"] not in out or v < out[r["robot"]][0]:
            out[r["robot"]] = (round(v, 1), round(float(r["t"]), 1))
    return out


def main():
    for d in sys.argv[1:]:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        tf = os.path.join(d, "sim", "turn_fault.json")
        broke = json.load(open(tf)) if os.path.exists(tf) else None
        faults = st.get("turn_faults") or []
        perf = list(csv.DictReader(open(os.path.join(d, "perf.csv")))) if os.path.exists(os.path.join(d, "perf.csv")) \
            else []
        fps = [float(r["fps_handled"]) for r in perf]
        runs = [r for r in st.get("schedule") or [] if r.get("status") == "done"]
        rep = dict(
            folder=os.path.basename(d),
            turns_broken_at=broke and round(broke["t"], 1),
            turn_faults=faults,
            caught_after_s=(round(faults[0][0] - broke["t"], 1) if (broke and faults) else None),
            alerts=[(a["kind"], a["robot"], a["words"][:70]) for a in st.get("alerts") or []],
            left_out=st.get("left_out"),
            heading_crawls=st.get("heading_crawls"),
            turns_cut=st.get("turns_cut"),
            edge_aways=st.get("edge_aways"), edge_turns_along=st.get("edge_turns_along"),
            heading_checks=len(st.get("heading_checks") or []), slow_turns=len(st.get("slow_turns") or []),
            edge_back_outs=st.get("edge_back_outs"), edge_touches=st.get("edge_touches"),
            edge_touches_skipped=st.get("edge_touches_skipped"),
            fix_skipped_held=st.get("fix_skipped_held"), routes_blocked=st.get("routes_blocked"),
            holds=[[h[1], h[2], h[3][:70], h[5]] for h in st.get("holds") or []],
            help_calls=st.get("help_calls"),
            watchdog_stops=[w[1:3] for w in st.get("watchdog_stops") or []],
            runs_scored=len(runs), runs_by_robot={n: len([r for r in runs if r["robot"] == n]) for n in st["robots"]},
            explore=(st.get("setup") or {}).get("explore"), explore_failed=(st.get("setup") or {}).get("explore_failed"),
            closest_true_edge_cm=closest_edge(d),
            fps_median=S.median(fps) if fps else None, fps_p10=sorted(fps)[len(fps) // 10] if fps else None)
        print(json.dumps(rep, indent=1, default=str))


if __name__ == "__main__":
    main()
