"""Part 2a's replay measurement: the camera loop's pictures a second while explore's camera fit runs, in the runner's
own process (as on 2026-10-25 15:10) and in the helper process (e02_offload).

The camera loop: the runner's real tracker (e02_camera.MotionTracker, the stored roam1 calibration) on two real
overhead pictures from 15:10 (15:14:19.9 and 15:14:21.5, Right moving between them), offered at 25 a second, newest
only (as e02_camera.Feed). The fit: EX.fit on 15:10's own 151 explore stops (e02_state.json), the same call the Pi
made. Per mode: 5 s alone, the fit, 5 s alone; pictures handled each second and the longest gap between two.

    python3 fitbench.py STATE_JSON FRAME_DIR [repeats]     -> prints a table and writes fitbench.json beside it
    FB_INTERVAL_S=0 python3 fitbench.py ...                 -> pictures offered as fast as the loop takes them
"""
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [EXP, os.path.join(os.path.dirname(EXP), "recognition")]
import cv2  # noqa: E402
import e02_camera as CAM  # noqa: E402
import e02_explore as EX  # noqa: E402
import e02_offload as OFF  # noqa: E402


INTERVAL_S = float(os.environ.get("FB_INTERVAL_S", "0.04"))   # 0.04: 25 a second offered; 0.0: as fast as it can
                                                             # (the camera loop saturated, as the Pi's was: 93 % busy)


def camera_loop(frames, stop, handled):
    tr = CAM.MotionTracker(CAM.load_camera(os.path.join(EXP, "data", "overhead_floorcal_roam1.json")), (61.0, 61.0))
    for g in frames[:3]:
        tr.push(g, time.time())
    tr.follow("Right", 85.0, 31.0)
    pending = {"f": None}
    cond = threading.Condition()

    def producer():
        k = 0
        while not stop.is_set():
            with cond:
                pending["f"] = frames[k % len(frames)]
                cond.notify_all()
            k += 1
            time.sleep(INTERVAL_S)
    threading.Thread(target=producer, daemon=True).start()
    while not stop.is_set():
        with cond:
            while pending["f"] is None and not stop.is_set():
                cond.wait(0.2)
            f, pending["f"] = pending["f"], None
        if f is None:
            continue
        tr.push(f, time.time())
        handled.append(time.time())


def one(mode, robots, frames, worker):
    stop = threading.Event()
    handled = []
    th = threading.Thread(target=camera_loop, args=(frames, stop, handled), daemon=True)
    th.start()
    time.sleep(6.0)
    t_fit0 = time.time()
    if mode == "runner":
        EX.fit(robots, (640, 480))
    else:
        worker.call(EX.fit, robots, (640, 480))
    t_fit1 = time.time()
    time.sleep(5.0)
    stop.set()
    th.join(2.0)
    h = [t for t in handled if t > t_fit0 - 5.0]

    def rate(a, b):
        n = sum(1 for t in h if a <= t < b)
        return n / max(1e-6, b - a)

    def gap(a, b):
        w = [t for t in h if a <= t <= b]
        return max([q - p for p, q in zip(w, w[1:])] or [0.0])
    secs = [round(sum(1 for t in h if t_fit0 + k <= t < t_fit0 + k + 1), 1) for k in range(int(t_fit1 - t_fit0))]
    return dict(mode=mode, fit_s=round(t_fit1 - t_fit0, 1), fps_before=round(rate(t_fit0 - 5.0, t_fit0), 1),
                fps_during=round(rate(t_fit0, t_fit1), 1), fps_after=round(rate(t_fit1, t_fit1 + 4.5), 1),
                worst_second=min(secs) if secs else None, gap_max_ms_during=round(1000 * gap(t_fit0, t_fit1)),
                gap_max_ms_before=round(1000 * gap(t_fit0 - 5.0, t_fit0)))


def main():
    state, fdir = sys.argv[1], sys.argv[2]
    reps = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    obs = json.load(open(state))["setup"]["explore_obs"]
    robots = {n: v for n, v in obs.items() if len(v) >= 5}
    frames = [cv2.cvtColor(cv2.imread(os.path.join(fdir, f)), cv2.COLOR_BGR2GRAY)
              for f in ("f_151419.9.png", "f_151421.5.png")]
    w = OFF.Worker()
    w.start()
    out = []
    for r in range(reps):
        for mode in ("runner", "helper"):
            res = one(mode, robots, frames, w)
            res["rep"] = r + 1
            out.append(res)
            print(json.dumps(res), flush=True)
    w.stop()
    name = "fitbench.json" if INTERVAL_S > 0 else "fitbench_saturated.json"
    json.dump(dict(rows=out, cpus=os.cpu_count(), interval_s=INTERVAL_S, note=__doc__.split("\n")[0]),
              open(os.path.join(HERE, name), "w"), indent=1)


if __name__ == "__main__":
    main()
