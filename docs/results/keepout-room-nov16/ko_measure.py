import csv, json, sys
site = json.load(open(sys.argv[1]))
kos = []
for k in site["keepouts"]:
    if k.get("use") is False: continue
    cx, cy = [v * 2.54 for v in k["centre_in"]]; sx, sy = [v * 2.54 for v in k["size_in"]]
    kos.append((k["name"], cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2))
def dist(x, y, k):
    _, x0, x1, y0, y1 = k; dx = max(x0 - x, 0, x - x1); dy = max(y0 - y, 0, y - y1); return (dx * dx + dy * dy) ** .5
for tag in sys.argv[2:]:
    st = json.load(open(tag + "/status.json")); f = st["folder"]
    summ = json.load(open(f + "/mission_summary.json"))
    best = {}
    for r in csv.DictReader(open(f + "/run/camera_track.csv")):
        x, y = float(r["x_cm"]), float(r["y_cm"])
        for k in kos:
            v = dist(x, y, k)
            if (r["robot"], k[0]) not in best or v < best[(r["robot"], k[0])]: best[(r["robot"], k[0])] = v
    stops = list(csv.DictReader(open(f + "/run/stops.csv")))
    print(tag, st.get("words"), "result", summ.get("result"), "dur", summ.get("duration_s"), "stops", len(stops),
          "replans", summ.get("replans"), "out", [o["robot"] + ": " + o["why"][:60] for o in summ.get("robots_out", [])])
    for k, v in sorted(best.items()):
        if v < 20: print("   %-6s %-20s centre %.1f cm from the plate (body edge about %.1f cm; corner %.1f cm)" % (k[0], k[1], v, v - 4.75, v - 6.72))
