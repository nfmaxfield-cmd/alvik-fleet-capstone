"""runsum.py LOG_FOLDER [...] : one dry run of the 2026-10-26 runner-fix build, summarised
(docs/build-reports/e02-oct26-runner-fixes.md): the link (commands, first-try answers, resends, never answered,
settings resent / given up, the gauge's level changes), holds by kind and answer, automatic finds, rescues, hung and
stuck stops, moves out from an edge before a turn, turns refused at an edge, parking "close enough", runs scored,
explore's result, each robot's closest true approach to the open edges (simulator truth) and whether any robot hung."""
import csv
import json
import os
import sys
from collections import Counter


def closest_edge(d):
    out = {}
    p = os.path.join(d, "sim", "truth.csv")
    if not os.path.exists(p):
        return out
    for r in csv.DictReader(open(p)):
        try:
            v = min(float(r["x_cm"]), float(r["y_cm"]))
        except (KeyError, ValueError):
            continue
        if r["robot"] not in out or v < out[r["robot"]]:
            out[r["robot"]] = round(v, 1)
    return out


def main():
    for d in sys.argv[1:]:
        st = json.load(open(os.path.join(d, "e02_state.json")))
        args = (st.get("setup") or {}).get("args") or {}
        print("=" * 100)
        print(os.path.basename(d), "drop %s%%" % args.get("dry_drop_pct"), "start", args.get("dry_start"),
              "low-contrast", args.get("dry_low_contrast") or "-", "scene", args.get("dry_scene") or "-")
        ev = Counter()
        lp = os.path.join(d, "link.csv")
        if os.path.exists(lp):
            for r in csv.DictReader(open(lp)):
                ev[r["event"]] += 1
        sent = ev["ack"] + ev["ack_after_resend"] + ev["never_answered"]
        print("  link: %d numbered commands answered/given up; first try %d, after resend %d (%.1f %%), never answered "
              "%d; resends %d; settings read back (get) %d, resent %d, given up %d" % (
                  sent, ev["ack"], ev["ack_after_resend"], 100.0 * ev["ack_after_resend"] / max(1, sent),
                  ev["never_answered"], ev["resend"], ev["get"], ev["set_resend"], ev["set_failed"]))
        print("  link events:", st.get("link_events") or "-", "| moves waited:", len(st.get("link_waits") or []))
        lk = st.get("link") or {}
        print("  link gauge at the end: %s (share %s over 5 min)" % (lk.get("level"), lk.get("share")))
        hk = Counter((h[2], h[5]) for h in st.get("holds") or [])
        print("  holds (kind, answer):", dict(hk) or "-")
        for h in st.get("holds") or []:
            print("     ", h[1], h[2], "|", (h[3] or "")[:110], "|", h[5])
        print("  automatic finds:", [(a[1], a[3], a[5]) for a in st.get("auto_finds") or []] or "-")
        print("  rescues:", [(r[1], r[2], r[4], r[5]) for r in st.get("rescues") or []] or "-")
        print("  hung / stuck stops:", [(h[1], h[2], h[3][:70]) for h in st.get("hung_stops") or []] or "-")
        print("  straight out before a turn:", len(st.get("edge_clear_moves") or []), [(m[1], m[3]) for m in
                                                                                     st.get("edge_clear_moves") or []][:8])
        print("  turns refused at an edge:", st.get("edge_turns_refused") or "-")
        print("  identify without a turn (at an edge):", len(st.get("identify_no_turn") or []))
        print("  parking close enough:", st.get("park_close_enough") or "-")
        print("  explore jumps accepted:", st.get("explore_jumps_accepted") or "-")
        print("  settings unconfirmed / failed:", st.get("settings_unconfirmed") or "-", st.get("settings_failed") or "-")
        ex = (st.get("setup") or {}).get("explore") or {}
        if ex or (st.get("setup") or {}).get("explore_failed"):
            print("  explore: %s; %s" % ("board %s x %s cm, fit %s px, %s min" % (ex.get("W"), ex.get("H"),
                                                                                 ex.get("rms_px"), ex.get("minutes"))
                                         if ex.get("W") else "no board", (st["setup"].get("explore_failed") or {})
                                         .get("why", "")))
        sides = (st.get("setup") or {}).get("board_sides_from_explore")
        print("  board sides from explore (not asked):", sides or "-",
              "| words kept:", (st.get("setup") or {}).get("explore_board_words") or "-")
        sch = [r for r in (st.get("schedule") or []) if r.get("status") in ("done", "scored")]
        print("  runs scored:", len(sch), "| session end:", (st.get("session_end") or {}).get("why", "-"))
        print("  closest true approach to an open edge (cm, - = past it):", closest_edge(d))
        sp = os.path.join(d, "sim", "sim_stdout.txt")
        if os.path.exists(sp):
            txt = open(sp).read()
            print("  simulator errors:", txt.count("Traceback"))


if __name__ == "__main__":
    main()
