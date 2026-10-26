"""How the runner (e02.py) uses the tracker process, frame saving, the CPU log and fusion steering (2026-10-25
evening, MIRLT). Kept here so the runner's own changes are a few calls.

Switches (e02.py's command line; run-e02-pi.sh passes the Pi's):
  (default)            the tracker in its own process (e02_trackproc) reading the recorder's /live stream; on the Pi
                       (--on-pi, 4 cores) pinned to cores TRACKER_CORES. If it can't start, the runner says so and
                       tracks in its own process as before.
  --tracker-in-runner  the old way: the tracker as a thread of the runner (the fallback; also what --dry-slow-camera
                       needs, since it slows that thread on purpose).
  --production         no video: the recorder is not started (run-e02-pi.sh), the tracker process opens the camera
                       itself, and frame saving is on (it implies --no-recording).
  --save-frames / --no-save-frames   frame saving beside the video in an experiment session (default: on, so the
                       production path is tried in experiment sessions too; Nick can switch it off).
  --fusion-steer       steer by the combined camera + wheels + gyro estimate where it was better (e02_fusesteer).
                       OFF by default: not yet tried on the real board.
  --cpu-log-s S        cpu.csv every S seconds (default e02_cpulog.EVERY_S; 0: off).
"""
import csv
import os
import time

import e02_camera as CAM
import e02_cpulog as CL
import e02_framesaver as FS
import e02_fusesteer as FU
import e02_trackproc as TP


def add_args(ap):
    ap.add_argument("--tracker-in-runner", action="store_true",
                    help="track in the runner's own process (the old way; the fallback)")
    ap.add_argument("--tracker-cores", default=None,
                    help="cores for the tracker process, e.g. 2,3 (default: 2,3 on a 4-core Pi with --on-pi, else none)")
    ap.add_argument("--production", action="store_true",
                    help="no video: the tracker process opens the camera itself and saves event clips, stills and "
                         "thumbnails (frames/); start with ./run-e02-pi.sh --production")
    ap.add_argument("--camera-device", type=int, default=0, help="--production: the camera's number (/dev/videoN)")
    ap.add_argument("--save-frames", dest="save_frames", action="store_true", default=None,
                    help="frame saving beside the video (default: on)")
    ap.add_argument("--no-save-frames", dest="save_frames", action="store_false")
    ap.add_argument("--fusion-steer", action="store_true",
                    help="steer by the combined camera + wheels + gyro estimate where it was better (OFF by default: "
                         "not yet tried on the real board)")
    ap.add_argument("--cpu-log-s", type=float, default=CL.EVERY_S, help="cpu.csv every this many seconds (0: off)")


def fix_args(a):
    """--production implies --no-recording."""
    if getattr(a, "production", False):
        a.no_recording = True
    if getattr(a, "save_frames", None) is None:
        a.save_frames = True
    return a


def tracker_cores(a):
    s = getattr(a, "tracker_cores", None)
    if s:
        return [int(c) for c in str(s).split(",") if c.strip()]
    if getattr(a, "on_pi", False) and (os.cpu_count() or 1) >= 4:
        return list(TP.TRACKER_CORES)
    return None


def source_of(a, stream_url):
    if getattr(a, "production", False):
        return "device:%d" % int(getattr(a, "camera_device", 0) or 0)
    return "stream:" + stream_url


def make_feed(run, a, hist, stream_url):
    """The runner's camera feed: the tracker process (default), else e02_camera.Feed. Never raises for the process:
    on any failure the runner says why and uses the old feed (a production session can't: no recorder to read)."""
    if not getattr(a, "tracker_in_runner", False) and not getattr(a, "dry_slow_camera", ""):
        try:
            f = TP.ProcFeed(source_of(a, stream_url), hist, log=lambda m: run.say("  " + m, quiet=True),
                            session_dir=run.dir, cores=tracker_cores(a), save_frames=bool(a.save_frames))
            f.on_park_event = lambda *ev: on_park_event(run, *ev)
            f.on_perf_extra = lambda d: on_perf_extra(run, d)
            f.start()
            info = f.proc.info
            run.say("Camera tracking in its own process (pid %s, cores %s)%s." % (
                info.get("pid"), ",".join(str(c) for c in info.get("cores") or []) or "any",
                "; frame saving on (frames/)" if a.save_frames else ""))
            run.st.setdefault("setup", {})["tracker_process"] = dict(pid=info.get("pid"), cores=info.get("cores"),
                                                                     source=f.source, save_frames=bool(a.save_frames))
            return f
        except Exception as e:
            if getattr(a, "production", False):
                raise SystemExit("The tracker process could not open the camera (%s). A production session has no "
                                 "recorder to fall back on. Stopped." % e)
            run.say("!!! The tracker process did not start (%s): tracking in the runner as before." % e)
            run.st.setdefault("setup", {})["tracker_process"] = dict(failed=str(e))
    return CAM.Feed(stream_url, None, hist, log=lambda m: run.say("  (camera: %s)" % m))


def is_proc(feed):
    return isinstance(feed, TP.ProcFeed)


def make_tracker(run, centre, use_lock=True):
    """A new tracker, in the process when the feed is the process's (else MotionTracker, as before)."""
    if is_proc(run.feed):
        return run.feed.make_tracker(run.cam, centre, use_lock=use_lock)
    import e02_alltrack as AT
    tr = AT.AllTracker(run.cam, centre)          # the same tracker, in the runner (faster matching, parked watch)
    tr.use_lock = use_lock
    tr.on_park_event = lambda *ev: on_park_event(run, *ev)
    run.feed.tracker = tr
    return tr


def restart_feed(run):
    """camera_alive(): a dead tracker process is started again with the same set-up."""
    f = run.feed
    run.say("  (the tracker process stopped: %s; starting it again)" % (f.proc.ended if f.proc else "?"))
    f.restart()
    run.st.setdefault("tracker_restarts", []).append(round(time.time(), 1))
    return f


def start_cpu_log(run, a):
    every = float(getattr(a, "cpu_log_s", CL.EVERY_S) or 0)
    if every <= 0:
        return None
    procs = {"runner": os.getpid(), "tracker": getattr(run.feed, "pid", None) if is_proc(run.feed) else None}
    w = CL.CpuLog(run.dir, procs=procs, find={"recorder": "camrec.py"}, every_s=every,
                  log=lambda m: run.say(m, quiet=True))
    w.start()
    return w


# --- events worth a clip ----------------------------------------------------------------------------------------
def on_mark(run, text):
    """Every runner mark: a frame-saving event when it is one of Nick's kinds."""
    kind = FS.kind_of_mark(text)
    if kind is None or not is_proc(getattr(run, "feed", None)):
        return
    robot = ""
    for n in (run.st.get("robots") or []):
        if n in text:
            robot = n
            break
    run.feed.event(kind, text, time.time(), robot)


def on_note(run, row):
    if is_proc(getattr(run, "feed", None)):
        run.feed.event("note", "Note %s %s" % (row.get("note", ""), row.get("text", "")), time.time(), "")


# --- parked robots --------------------------------------------------------------------------------------------
PARKED_COLS = ["t", "clock", "robot", "event", "x0", "y0", "x", "y", "cm", "unseen_s"]


def on_park_event(run, t, robot, kind, info):
    """A parked robot moved (bumped, pushed, a hand), went unseen, or was seen again: parked.csv, the state, a line,
    and a clip when it moved. A move puts its stored position where the camera now sees it."""
    try:
        path = os.path.join(run.dir, "parked.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=PARKED_COLS, extrasaction="ignore")
            if new:
                w.writeheader()
            w.writerow(dict(info, t="%.2f" % t, clock=time.strftime("%H:%M:%S", time.localtime(t)), robot=robot,
                            event=kind))
        run.st.setdefault("parked_events", []).append([round(t, 1), robot, kind, info])
        if kind == "moved":
            q = (run.st.get("positions") or {}).get(robot)
            if q:
                run.st["positions"][robot] = [round(info["x"], 2), round(info["y"], 2)] + list(q[2:3] or [None])
            run.say("  (camera: %s, standing parked, was moved %.1f cm (a bump, a push or a hand); its stored place "
                    "now follows it)" % (robot, info.get("cm", 0.0)))
            if is_proc(getattr(run, "feed", None)):
                run.feed.event("parked robot moved", "%s moved %.1f cm while parked" % (robot, info.get("cm", 0.0)),
                               t, robot)
        elif kind == "unseen":
            run.say("  (camera: %s, parked, has not been seen at its place for %.0f s)" % (robot,
                                                                                           info.get("unseen_s", 0)),
                    quiet=True)
    except Exception as e:
        run.say("  (parked-robot event not recorded: %s)" % e, quiet=True)


def on_perf_extra(run, d):
    run.__dict__.setdefault("tracker_proc_perf", []).append(d)
    del run.tracker_proc_perf[:-600]


# --- fusion steering --------------------------------------------------------------------------------------------
def steer_pos(run, robot, cam_p, fresh_s):
    """pos() with --fusion-steer: the camera's (t, x, y) or the combined estimate's, as e02_fusesteer.choose decides;
    the camera's own rule (fresh_s) when neither is chosen. Logs fused choices in fusion_steer.csv."""
    now = time.time()
    near = None
    try:
        b = run.board
        if b is not None and cam_p is not None:
            near = min(abs(cam_p[2] - b.xboard_cm), abs(cam_p[1] - b.yboard_cm))
    except Exception:
        near = None
    fused = FU.fused_now(getattr(run, "lab", None), robot, now)
    c = FU.choose(cam_p, fused, near, now)
    if c is None:
        return cam_p if (cam_p is not None and now - cam_p[0] <= fresh_s) else None
    if c[3] != "camera":
        try:
            path = os.path.join(run.dir, "fusion_steer.csv")
            new = not os.path.exists(path)
            with open(path, "a") as f:
                if new:
                    f.write("t,robot,why,x,y,cam_x,cam_y,cam_age_s,sd_cm\n")
                f.write("%.3f,%s,%s,%.2f,%.2f,%s,%s,%s,%.2f\n" % (
                    now, robot, c[3], c[1], c[2], "" if cam_p is None else "%.2f" % cam_p[1],
                    "" if cam_p is None else "%.2f" % cam_p[2], "" if cam_p is None else "%.2f" % (now - cam_p[0]),
                    fused["sd_cm"]))
        except Exception:
            pass
    return (c[0], c[1], c[2])


def summary(run):
    """Lines for the session's end: the tracker process, frame saving, the CPU log."""
    out = []
    f = getattr(run, "feed", None)
    if is_proc(f):
        s = f.frames_summary()
        out.append("Tracker process: %d positions handled, %d dropped as older than a command%s." % (
            f.n_trk, f.dropped_stale, "" if not s else "; frames saved: %d events, %d clips, %d stills, %d "
                                                          "thumbnails, %.0f MB (frames/)" % (
                s["events"], s["clips"], s["stills"], s["thumbs"], s["mb"])))
        run.st.setdefault("setup", {}).setdefault("tracker_process", {})["frames"] = s
    w = getattr(run, "cpulog", None)
    if w is not None:
        s = w.summary()
        out.append("Pi load: cpu.csv %d rows; hottest %s C; throttling: %s." % (
            s["rows"], s["max_temp_c"], s["throttled_words"] or "none"))
        run.st["cpu_summary"] = s
    return out


# --- the recorder checks, in a production session (no recorder) --------------------------------------------------
def check_overhead(run, a, seconds=2.0):
    """setup()'s overhead camera check: the recorder's stream (CAM.check_stream), or, in a production session, the
    tracker process's own pictures. Returns (ok, words, (w, h) or None, fps)."""
    if not getattr(a, "production", False):
        return CAM.check_stream(a.pi, a.overhead_port)
    f = run.feed
    n0, t0 = f.frames, time.time()
    time.sleep(seconds)
    n = f.frames - n0
    img = f.last_img
    if not n or img is None:
        return False, "no pictures from the camera (/dev/video%d)" % int(getattr(a, "camera_device", 0) or 0), None, 0.0
    fps = n / (time.time() - t0)
    return True, "%d pictures in %.1f s from the camera itself (production: no recorder)" % (n, seconds), \
        (img.shape[1], img.shape[0]), fps


def recorder_used(a):
    return not getattr(a, "production", False)
