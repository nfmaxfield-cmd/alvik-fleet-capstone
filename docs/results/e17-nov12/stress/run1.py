"""One fast-simulator run for the e17 comparison: python3 run1.py SEED HOURS DELAY OUTJSON (env: the switches)."""
import json, os, sys, time
sys.path.insert(0, sys.argv[5])
import freeroam_fastsim as F
seed, hours, delay, out = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
t0 = time.time()
s = F.Sim(seed, hours, dead_robot=False, pi_freeze=True, edge_physics=True, camera="detections", sensors=True,
          cam_delay=delay if delay > 0 else None, hdg_fault={"deg": 25, "near_cm": 14, "every_s": 60})
r = s.run()
c = s.ctrl.counts
keep = {k: r.get(k) for k in ("contacts", "contact_s", "min_gap_cm", "off_board", "edge_min_cm", "moving_pct",
                              "watchdog_stops", "longest_still_s", "lost_episodes", "trophy_breaks")}
keep.update(falls=s.stats.get("falls"), hangs=s.stats.get("hangs"), edge_trips=s.stats.get("edge_trips"))
keep.update(longest_still=s.longest_still(), swaps=s.src.stats["swaps"], wrong=s.src.stats["wrong"],
            tracked=s.src.summary()["tracked_pct"],
            ctrl={k: c.get(k) for k in ("camera_corrections", "corrected_rejoins", "hand_rejoined", "need_hand",
                                        "creeps", "late_slowed", "late_waited", "hd_confirm_crawls", "hd_confirmed",
                                        "hd_confirm_set", "reseeds", "reseeds_deferred", "rejoined", "freed_itself",
                                        "corrections_revoked")},
            layer={k: s.src.layer.counts.get(k) for k in ("eliminated", "elim_wiggles", "elim_revoked",
                                                          "elim_confirmed", "own_resets")},
            secs=round(time.time() - t0, 1))
json.dump(keep, open(out, "w"), default=str)
print(out, keep["secs"])
