"""Group hard-simulator stress results by profile + faults: python3 stress_table.py DIR [DIR ...]"""
import glob
import json
import sys
from collections import OrderedDict

groups = OrderedDict()
for d in sys.argv[1:]:
    for f in sorted(glob.glob(d + "/*.json")):
        r = json.load(open(f))
        key = (d.rstrip("/").split("/")[-1], r["profile"] + ("+" + "+".join(r["faults"]) if r["faults"] and
                                                               r["profile"] != "harsh" else ""))
        groups.setdefault(key, []).append(r)
hdr = ("group", "runs", "h", "touch runs", "contacts", "min gap med", "falls", "off-board runs", "swaps", "hands",
       "stands>5s/h", "longest stand", "breach %", "blind err p95", "wd/h")
print(" | ".join(hdr))
out = []
for (d, k), rs in groups.items():
    H = sum(r["broke"].get("hours", r["hours"]) if False else r["hours"] for r in rs)
    B = [r["broke"] for r in rs]
    mg = sorted(b["min_gap_cm"] for b in B)
    bs = sum(b["blind_samples"] for b in B)
    row = ("%s/%s" % (d, k), len(rs), round(H, 1), sum(1 for b in B if b["contacts"]), sum(b["contacts"] for b in B),
           round(mg[len(mg) // 2], 1), sum(b["falls"] for b in B), sum(1 for b in B if b["off_board"]),
           sum(b["swaps"] for b in B), sum(b["needs_hand"] for b in B),
           round(sum(b["stands_over"] for b in B) / H, 1), max(b["longest_stand_s"] for b in B),
           round(100.0 * sum(b["blind_breaches"] for b in B) / max(1, bs), 1),
           max((b["blind_err_p95"] or 0) for b in B), round(sum(b["watchdog_stops"] for b in B) / H, 0))
    out.append(row)
    print(" | ".join(str(x) for x in row))
json.dump(out, open("/dev/stdout", "w")) if "--json" in sys.argv else None
