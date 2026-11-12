#!/usr/bin/env python3
"""The replays (replay_all.py) and the as-run measures (asrun.py) to the report's tables. Logs only.

    python3 summarise.py REPLAY_OUT ASRUN.json OUT_DIR [--tag w1]

Writes OUT_DIR/runs_<tag>.csv (one row per run and mode) and OUT_DIR/robots_<tag>.csv (one row per run, robot and
mode), and prints a Markdown table.

Changelog
  2026-11-12 v1  first version (replay-all-nov12)
"""
import argparse
import csv
import glob
import json
import os

VERSION = "summarise-2026-11-12v1"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("replays")
    ap.add_argument("asrun")
    ap.add_argument("out")
    ap.add_argument("--tag", default="w1")
    a = ap.parse_args()
    asrun = json.load(open(a.asrun))["runs"] if os.path.exists(a.asrun) else {}
    runs_rows, rob_rows = [], []
    for f in sorted(glob.glob(os.path.join(a.replays, "*-off-%s.json" % a.tag))):
        run = os.path.basename(f).split("-off-")[0]
        for mode in ("off", "on"):
            j = json.load(open(f.replace("-off-", "-%s-" % mode)))
            R = j["robots"]
            tk = j["taken"]
            sizes = sorted(t["d_cm"] for t in tk if t.get("d_cm") is not None)
            wrong_elim = sum(1 for t in tk if t["bridge_name"] != t["robot"])
            ga = [r["gap_all"] for r in R.values() if r["gap_all"]["n"]]
            row = dict(run=run, mode=mode, robots=len(R), bridge_fixes=sum(r["bridge_fixes"] for r in R.values()),
                       named=sum(r["named"] for r in R.values()),
                       longest_unnamed_s=max(r["unnamed_longest_s"] for r in R.values()),
                       longest_robot=max(R, key=lambda n: R[n]["unnamed_longest_s"]),
                       steady_spells=sum(r["steady_spells"] for r in R.values()),
                       steady_spells_s=round(sum(r["steady_spells_s"] for r in R.values()), 1),
                       steady_longest_s=max(r["steady_longest_s"] for r in R.values()),
                       wrong_name_fixes=sum(r["wrong_name"] for r in R.values()),
                       swap_episodes=sum(r["swap_episodes"] for r in R.values()),
                       corrections=len(tk), corr_wrong_name=wrong_elim,
                       corr_revoked=j["counts"].get("elim_revoked", 0),
                       corr_confirmed=j["counts"].get("elim_confirmed", 0),
                       corr_median_cm=sizes[len(sizes) // 2] if sizes else None,
                       corr_max_cm=max(sizes) if sizes else None,
                       own_resets=j["counts"].get("own_resets", 0),
                       gap_p50=round(max(g["p50"] for g in ga), 1) if ga else None,
                       gap_p99=round(max(g["p99"] for g in ga), 1) if ga else None,
                       gap_max=round(max(g["max"] for g in ga), 1) if ga else None,
                       gap_over25=sum(g["over25"] for g in ga))
            h = (asrun.get(run) or {}).get("heading")
            rj = (asrun.get(run) or {}).get("rejoin") or {}
            rej = rj.get("rejoined", [])
            row.update(reseeds_measured=h["n"] if h else None,
                       heading_jumps_over15=(h["big_off"] if mode == "off" else h["big_on_est"]) if h else None,
                       needs_a_hand_lines=rj.get("needs_a_hand_lines"),
                       rejoined=len(rej), false_rejoined=sum(1 for x in rej if x["verdict"].startswith("not moved")),
                       false_rejoined_e17_corrected=sum(1 for x in rej if x["verdict"].startswith("not moved") and
                                                        x["e17_corrected_at"] is not None))
            runs_rows.append(row)
            for n, r in R.items():
                rob_rows.append(dict(run=run, mode=mode, robot=n, bridge_fixes=r["bridge_fixes"], named=r["named"],
                                     unacc_s=r["unacc_s"], unnamed_longest_s=r["unnamed_longest_s"],
                                     steady_spells=r["steady_spells"], steady_spells_s=r["steady_spells_s"],
                                     steady_longest_s=r["steady_longest_s"], wrong_name=r["wrong_name"],
                                     swap_episodes=r["swap_episodes"],
                                     corrections=sum(1 for t in tk if t["robot"] == n),
                                     gap_p50=r["gap_all"]["p50"] and round(r["gap_all"]["p50"], 2),
                                     gap_p90=r["gap_all"]["p90"] and round(r["gap_all"]["p90"], 2),
                                     gap_p99=r["gap_all"]["p99"] and round(r["gap_all"]["p99"], 2),
                                     gap_max=r["gap_all"]["max"] and round(r["gap_all"]["max"], 2),
                                     gap_over25=r["gap_all"]["over25"],
                                     named_gap_p99=r["gap_named"]["p99"] and round(r["gap_named"]["p99"], 2),
                                     named_gap_max=r["gap_named"]["max"] and round(r["gap_named"]["max"], 2)))
    os.makedirs(a.out, exist_ok=True)
    for name, rows in (("runs", runs_rows), ("robots", rob_rows)):
        with open(os.path.join(a.out, "%s_%s.csv" % (name, a.tag)), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    keys = ["longest_unnamed_s", "longest_robot", "steady_spells", "steady_spells_s", "wrong_name_fixes",
            "swap_episodes", "corrections", "corr_revoked", "corr_median_cm", "corr_max_cm", "heading_jumps_over15",
            "false_rejoined"]
    print("| run | mode | " + " | ".join(keys) + " |")
    for r in runs_rows:
        print("| %s | %s | " % (r["run"][:15], r["mode"]) + " | ".join(str(r[k]) for k in keys) + " |")


if __name__ == "__main__":
    main()
