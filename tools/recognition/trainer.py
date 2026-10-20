"""Training a project's model in the background, and scoring every model on the same frames.

How the frames are split (the part that keeps the scores honest):
- Each recording's labelled frames are cut into blocks of `block_s` seconds (30 s by
  default). A block goes wholly to train, val (to pick the best epoch) or test (to
  score). Neighbouring frames are near-copies, so a split by single frames would score
  the model on frames it has practically seen.
- Frames within `guard_s` (1 s) of a block edge are left out of training and scoring,
  so the frames either side of an edge are not near-copies in different parts.
- Which part a block goes to depends only on its recording session and its time
  (a hash), not on how many frames exist yet. Adding labels never moves an old test
  block into training, so every model version is scored on frames it never saw.
  Two cameras filming the same moment share a session, so the same moment is test
  for both cameras or for neither.
- "No model" (the starting point) is scored on the same test frames from the motion
  detector's raw guesses, so the learning curve starts where labelling starts.

Results: models/<vN>/meta.json, models/curve.json.
"""
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
DETECTOR = os.path.join(TOOLS, "detector")
LABEL = os.path.join(TOOLS, "label")
for p in (HERE, DETECTOR, LABEL):
    if p not in sys.path:
        sys.path.insert(0, p)

import project as projmod  # noqa: E402

TEST_SHARE, VAL_SHARE = 0.2, 0.15


def _part(key):
    h = int(hashlib.sha1(key.encode()).hexdigest()[:8], 16) / 0xFFFFFFFF
    return "test" if h < TEST_SHARE else ("val" if h < TEST_SHARE + VAL_SHARE else "train"), h


def _times(rec_folder):
    """t_mono per frame from frames.csv (None if missing)."""
    p = os.path.join(rec_folder, "frames.csv")
    if not os.path.exists(p):
        return None
    out = []
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            try:
                out.append(float(r["t_mono"]))
            except (KeyError, ValueError):
                out.append(None)
    return out


def _link(src, dst):
    if os.path.exists(dst):
        return
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def build_dataset(proj, out_dir, block_s=30.0, guard_s=1.0, fps_default=25.0):
    """Write one pseudo-recording folder per block (labels.csv + frames/) under out_dir.
    Returns (assign: block name -> part, info: block name -> dict)."""
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir)
    assign, info = {}, {}
    for r in proj.recordings():
        ld = proj.labels_dir(r["id"])
        lp = os.path.join(ld, "labels.csv")
        if not os.path.exists(lp):
            continue
        with open(lp, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            cols = reader.fieldnames
            rows = list(reader)
        times = _times(r["path"]) or []
        t0 = next((t for t in times if t is not None), 0.0)
        by_block = {}
        for row in rows:
            fr = int(row["frame"])
            t = times[fr] if fr < len(times) and times[fr] is not None else fr / fps_default
            # absolute time (shared by cameras of one session) when frames.csv has it
            tb = t if times else t
            b = int(tb // block_s)
            edge = min(tb - b * block_s, (b + 1) * block_s - tb)
            if edge < guard_s:
                continue
            by_block.setdefault(b, []).append(row)
        sess = r.get("session") or r["id"]
        for b, brow in sorted(by_block.items()):
            name = "%s__b%05d" % (r["id"], b - int(t0 // block_s))
            part, h = _part("%s|%d" % (sess, b))
            d = os.path.join(out_dir, name)
            os.makedirs(os.path.join(d, "frames"))
            with open(os.path.join(d, "labels.csv"), "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=cols)
                w.writeheader()
                for row in brow:
                    w.writerow(row)
                    img = os.path.join(ld, "frames", "%06d.jpg" % int(row["frame"]))
                    if os.path.exists(img):
                        _link(img, os.path.join(d, "frames", os.path.basename(img)))
            assign[name] = part
            info[name] = dict(recording=r["id"], camera=r.get("camera", ""), block=b, hash=h, rows=len(brow))
    # every part needs something: if the hash left one empty, take the blocks whose hash
    # is nearest that part's range (deterministic, and stable once blocks exist)
    names = sorted(assign, key=lambda n: info[n]["hash"])
    if len(names) >= 3:
        for need in ("test", "val"):
            if need not in assign.values():
                cand = [n for n in names if assign[n] == "train"]
                if len(cand) > 1:
                    pick = cand[0] if need == "test" else cand[-1]
                    assign[pick] = need
    return assign, info


def count_frames(label_root, classes, assign):
    import detcore
    samples = detcore.read_many([label_root], classes, verbose=False) if os.listdir(label_root) else []
    out = dict(train=0, val=0, test=0)
    for s in samples:
        p = assign.get(s["recording"])
        if p:
            out[p] += 1
    return out, samples


def motion_predictions(proj, info, assign, path):
    """Predictions CSV of the raw motion guesses on every test block (the 'no model' row)."""
    import app as labelapp
    by_rec = {}
    for name, i in info.items():
        if assign.get(name) == "test":
            by_rec.setdefault(i["recording"], []).append(name)
    n = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["recording", "frame", "cls", "x_px", "y_px", "heading_deg", "score"])
        for rid, blocks in by_rec.items():
            det_path = os.path.join(proj.labels_dir(rid), "cache", "motion_detections.csv")
            if not os.path.exists(det_path):
                continue
            frames_of = {}
            for b in blocks:
                with open(os.path.join(os.path.dirname(path), "data", b, "labels.csv"), newline="") as lf:
                    for row in csv.DictReader(lf):
                        frames_of[int(row["frame"])] = b
            # what the labelling tool shows with no model and no confirmations: motion
            # guesses followed by the tracker (no anchors, so nothing a person did leaks in)
            import tracker
            rec = labelapp.Recording(proj.rec(rid)["path"])
            dets = labelapp.load_detections(det_path, rec.n)
            robot_px = None
            info_p = os.path.join(proj.labels_dir(rid), "cache", "motion_info.json")
            if os.path.exists(info_p):
                with open(info_p) as jf:
                    robot_px = json.load(jf).get("robot_px")
            guesses, _ = tracker.TrackRun(dets, rec.times, robot_px or 30.0, (rec.width, rec.height),
                                          det_source="motion").run({}, set())
            for fr, b in frames_of.items():
                for d in guesses.get(fr, []):
                    w.writerow([b, fr, d["cls"], "%.2f" % d["x"], "%.2f" % d["y"],
                                "" if d.get("heading_deg") is None else "%.1f" % d["heading_deg"], "%.3f" % d["score"]])
                    n += 1
    return n


class Trainer:
    """One training job at a time, in a background thread; the studio polls .job."""

    def __init__(self, proj, log=print):
        self.proj = proj
        self.log = log
        self.job = None
        self.lock = threading.Lock()
        self.proc = None

    def busy(self):
        return bool(self.job and self.job["status"] == "running")

    def start(self, minutes=20.0, python=sys.executable):
        with self.lock:
            if self.busy():
                raise ValueError("a model is already training")
            mid = self.proj.new_model_id()
            self.job = dict(kind="train", model=mid, status="running", started=time.time(),
                            message="Training model %s: preparing the frames ..." % mid)
        t = threading.Thread(target=self._run, args=(mid, float(minutes), python), daemon=True)
        t.start()
        return mid

    def _say(self, msg):
        self.job["message"] = msg
        self.log(msg)

    def _run(self, mid, minutes, python):
        proj = self.proj
        mdir = proj.model_dir(mid)
        os.makedirs(mdir, exist_ok=True)
        logf = open(os.path.join(mdir, "train.log"), "a", encoding="utf-8")
        entry = dict(id=mid, created=projmod.now_iso(), status="training", classes=proj.class_names(),
                     heading_classes=proj.heading_classes(), n_train_frames=0, metrics={}, minutes=minutes)
        proj.data["models"].append(entry)
        proj.save()
        try:
            data = os.path.join(mdir, "data")
            assign, info = build_dataset(proj, data, float(proj.data["settings"].get("block_s", 30)))
            counts, _ = count_frames(data, proj.class_names(), assign)
            entry["split_counts"] = counts
            if counts["train"] < 10:
                raise RuntimeError("only %d confirmed frames to train on (need at least 10; confirm more frames first)"
                                   % counts["train"])
            if counts["test"] == 0:
                raise RuntimeError("no confirmed frames in a test block yet: confirm frames from more of the "
                                   "recording (the test part is about a fifth of the time blocks)")
            split = os.path.join(mdir, "split.json")
            with open(split, "w") as f:
                json.dump({"by_recording": assign}, f, indent=1, sort_keys=True)
            run = os.path.join(mdir, "run")
            sizes = []
            for c in proj.classes:
                size = c.get("size_px") or self._measured_size(c["name"])
                if size:
                    sizes.append("%s=%.1f" % (c["name"], size))
            cmd = [python, os.path.join(DETECTOR, "train.py"), "--labels", data, "--out", run,
                   "--classes", ",".join(proj.class_names()), "--heading-classes", ",".join(proj.heading_classes()),
                   "--split", split, "--max-minutes", "%.1f" % minutes]
            if sizes:
                cmd += ["--obj-size", ",".join(sizes)]
            prev = self._previous_same_classes()
            if prev:
                cmd += ["--init-from", prev]
            self._say("Training model %s on %d frames (%d to pick the best round, %d held back for scoring); "
                      "at most %.0f min ..." % (mid, counts["train"], counts["val"], counts["test"], minutes))
            self._call(cmd, logf, minutes * 60 + 600, progress=True, mid=mid)
            best = os.path.join(run, "best.pt")
            if not os.path.exists(best):
                raise RuntimeError("training finished without a best.pt (see %s)" % os.path.join(mdir, "train.log"))
            self._say("Model %s: exporting ..." % mid)
            onnx = os.path.join(mdir, "model.onnx")
            self._call([python, os.path.join(DETECTOR, "export.py"), best, "--out", onnx], logf, 600)
            entry.update(status="ready", n_train_frames=counts["train"], onnx=os.path.relpath(onnx, proj.folder),
                         init_from=os.path.relpath(prev, proj.folder) if prev else None)
            proj.save()
            self._say("Model %s: scoring every model on the held-back frames ..." % mid)
            self.score_all(data, assign, info, logf, python)
            m = proj.model(mid)
            sc = m.get("metrics", {})
            self.job.update(status="done", finished=time.time(),
                            message="Model %s is ready: finds %s, right %s on frames it never saw. Pick it in "
                                    "\"Guesses from\"." % (mid, _pct(sc.get("recall")), _pct(sc.get("precision"))))
        except Exception as e:     # keep the studio running; show the problem
            entry["status"] = "failed"
            entry["error"] = str(e)
            proj.save()
            self.job.update(status="failed", finished=time.time(), message="Training %s failed: %s" % (mid, e))
            logf.write("FAILED: %s\n" % e)
        finally:
            logf.close()

    def _measured_size(self, cls):
        """Robot size measured by the labelling tool (for the first class with a front)."""
        if cls != (self.proj.heading_classes() or [None])[0]:
            return None
        vals = []
        for r in self.proj.recordings():
            rp = os.path.join(self.proj.labels_dir(r["id"]), "review.json")
            if os.path.exists(rp):
                try:
                    with open(rp) as f:
                        vals.append(float(json.load(f).get("robot_px")))
                except (ValueError, TypeError):
                    pass
        vals = [v for v in vals if v and v > 0]
        return sorted(vals)[len(vals) // 2] if vals else None

    def _previous_same_classes(self):
        for m in reversed(self.proj.models()):
            if m.get("status") == "ready" and m.get("classes") == self.proj.class_names():
                p = os.path.join(self.proj.model_dir(m["id"]), "run", "best.pt")
                if os.path.exists(p):
                    return p
        return None

    def _call(self, cmd, logf, timeout, progress=False, mid=None):
        logf.write("\n$ %s\n" % " ".join(cmd))
        logf.flush()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     cwd=DETECTOR)
        t0 = time.time()
        for line in self.proc.stdout:
            logf.write(line)
            logf.flush()
            s = line.strip()
            if progress and s.lower().startswith("epoch"):
                self.job["message"] = "Training model %s: %s (%.0f min so far)" % (mid, s[:90], (time.time() - t0) / 60)
            if time.time() - t0 > timeout:
                self.proc.kill()
                raise RuntimeError("took too long; stopped")
        rc = self.proc.wait()
        self.proc = None
        if rc != 0:
            raise RuntimeError("%s exited with an error (see train.log)" % os.path.basename(cmd[1]))

    def score_all(self, data, assign, info, logf, python=sys.executable):
        """Score 'no model' and every ready model on the current test blocks; write models/curve.json."""
        proj = self.proj
        mdir = os.path.dirname(data)
        split = os.path.join(mdir, "split.json")
        motion_csv = os.path.join(mdir, "motion_test_preds.csv")
        n_motion = motion_predictions(proj, info, assign, motion_csv)
        cmd = [python, os.path.join(DETECTOR, "evaluate.py"), "--labels", data, "--split", split, "--part", "test",
               "--classes", ",".join(proj.class_names()), "--out", os.path.join(mdir, "eval.csv")]
        if n_motion:
            cmd += ["--model", "csv:" + motion_csv]
        for m in proj.models():
            if m.get("status") == "ready" and m.get("classes") == proj.class_names() and m.get("onnx"):
                p = os.path.join(proj.folder, m["onnx"])
                if os.path.exists(p):
                    cmd += ["--model", "infer:" + p]
        # every model file is called model.onnx: score them one at a time so their rows can be told apart
        rows = []
        base = cmd[:cmd.index("--out") + 2]
        specs = [cmd[i + 1] for i, c in enumerate(cmd) if c == "--model"]
        ids = []
        for spec in specs:
            kind, _, path = spec.partition(":")
            mid = "none" if kind == "csv" else next(m["id"] for m in proj.models()
                                                    if m.get("onnx") and os.path.join(proj.folder, m["onnx"]) == path)
            out = os.path.join(mdir, "eval_%s.csv" % mid)
            c2 = list(base)
            c2[c2.index("--out") + 1] = out
            c2 += ["--model", spec]
            if kind == "csv":
                c2 += ["--score-thresh", "0"]     # the labelling page shows every guess, whatever its score
            self._call(c2, logf, 1800)
            with open(out, newline="") as f:
                got = list(csv.DictReader(f))
            pick = [r for r in got if r["cls"] == "all"] or [r for r in got if _f(r.get("n_labels"))]
            if pick:
                pick[0]["model"] = mid
                rows.append(pick[0])
            ids.append(mid)
        _, samples = count_frames(data, proj.class_names(), assign)
        test_frames = sum(1 for s in samples if assign.get(s["recording"]) == "test" and not s.get("auto"))
        pts = []
        for r in rows:
            mid = r["model"]
            n = 0 if mid == "none" else proj.model(mid).get("n_train_frames", 0)
            pt = dict(model=mid, label="No model (motion)" if mid == "none" else "Model " + mid, n_train_frames=n,
                      precision=_f(r.get("precision")), recall=_f(r.get("recall")), f1=_f(r.get("f1")),
                      centre_err_px=_f(r.get("centre_err_px_mean")), heading_err_deg=_f(r.get("heading_err_deg_median")),
                      ms_per_frame=_f(r.get("ms_per_frame")), n_labels=_f(r.get("n_labels")),
                      n_objects=_f(r.get("n_objects")))
            pts.append(pt)
            if mid != "none":
                proj.model(mid)["metrics"] = {k: pt[k] for k in ("precision", "recall", "f1", "centre_err_px",
                                                                 "heading_err_deg", "ms_per_frame")}
        pts.sort(key=lambda p: (p["n_train_frames"], p["model"]))
        curve = dict(updated=projmod.now_iso(), points=pts,
                     test=dict(frames=test_frames, blocks=sum(1 for v in assign.values() if v == "test"),
                               note="%d time blocks of %.0f s held back; never trained on"
                                    % (sum(1 for v in assign.values() if v == "test"),
                                       float(proj.data["settings"].get("block_s", 30)))))
        projmod.write_json(os.path.join(proj.folder, "models", "curve.json"), curve)
        proj.save()
        return curve


def _f(v):
    try:
        x = float(v)
        return None if x != x else x
    except (TypeError, ValueError):
        return None


def _pct(v):
    return "-" if v is None else "%d%%" % round(100 * v)
