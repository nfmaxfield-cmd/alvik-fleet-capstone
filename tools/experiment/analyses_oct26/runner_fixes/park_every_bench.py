"""park_every_bench.py CLIP_FOLDER : the tracker's own time per picture on recorded overhead video with the parked-robot
watch off, on every picture, and on every 5th (2026-10-26, coordinator item from the 09:55 session analysis: the watch
cost 25-28 ms a picture on the Pi). Every picture is pushed (no skipping), one thread, this machine. Two robots are
parked (the watch searches for them in every watched picture whether or not they are there: its cost is the outline
search) and, in the "following" case, a third is followed. CLIP_FOLDER: clip.bin + clip_index.csv (a stretch of a real
recording; the build used 79 s of the 2026-10-25 21:14 overhead video from about 21:41, fetched by byte range)."""
import csv, os, statistics as S, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
TOOLS = os.path.dirname(EXP)
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live")):
    sys.path.insert(0, p)
import cv2, numpy as np
import e02_alltrack as AT, e02_camera as CAM, e02_plan as PL

clip = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(clip, "clip_index.csv"))))
blob = open(os.path.join(clip, "clip.bin"), "rb")
imgs = []
for r in rows:
    blob.seek(int(r["off"]))
    imgs.append(cv2.imdecode(np.frombuffer(blob.read(int(r["size"])), np.uint8), 0))
cam = CAM.load_camera(os.path.join(EXP, "data", "overhead_floorcal_roam1.json"))
board = PL.Board()
walls = [("y", board.xboard_cm + board.uy * PL.ROBOT_HALF_CM), ("x", board.yboard_cm + board.ux * PL.ROBOT_HALF_CM)]


def run(watch, every, follow):
    AT.AllTracker.WATCH_PARKED = watch
    AT.AllTracker.PARK_EVERY_N = every
    tr = AT.AllTracker(cam, (61.0, 61.0))
    tr.set_area((board.x0, board.y0, board.x1, board.y1), walls=walls)
    tr.set_background(imgs[:5])
    for n, (x, y) in (("P1", (35.0, 85.0)), ("P2", (85.0, 35.0))):
        tr.robot_at(x, y)
        tr.park(n, x, y)
    if follow:
        tr.follow("F", 60.0, 60.0, t=0.0)
    ms = []
    for k, im in enumerate(imgs[5:]):
        t0 = time.perf_counter()
        tr.push(im, 1000.0 + k / 25.0)
        ms.append(1000.0 * (time.perf_counter() - t0))
    ms = ms[25:]
    return S.median(ms), S.mean(ms)


print("%d pictures (%.0f s at 25/s)" % (len(imgs), len(imgs) / 25.0))
out = []
for follow in (False, True):
    for watch, every in ((False, 1), (True, 1), (True, 5)):
        med, mean = run(watch, every, follow)
        lab = "off" if not watch else "every picture" if every == 1 else "every %dth" % every
        out.append((("following" if follow else "idle"), lab, round(med, 1), round(mean, 1), round(1000.0 / mean, 1)))
        print("%-9s parked watch %-14s median %5.1f ms, mean %5.1f ms a picture (at most %5.1f pictures a second)" %
              out[-1])
with open(os.path.join(HERE, "park_every_bench.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["case", "parked_watch", "ms_median", "ms_mean", "max_pictures_per_s"])
    w.writerows(out)
