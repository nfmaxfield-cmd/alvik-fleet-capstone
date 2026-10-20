"""Training a project's model in the background, and scoring every model on the same frames.

How the frames are split (the part that keeps the scores honest):
- Each recording's labelled frames are cut into blocks of `block_s` seconds (30 s when
  there is enough footage; shorter for short recordings, see choose_block_s()). A block goes wholly to train, val (to pick the best epoch) or test (to
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
import math
import os
import re
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
BLOCK_CHOICES = (30, 20, 15, 10, 6, 4, 3)   # block lengths (s) tried, longest first
MIN_BLOCKS = 12          # the footage must hold at least this many blocks, so every part gets some
EPOCHS = 60              # training rounds at most (the time limit may stop it sooner)
MIN_EPOCHS = 15          # never stop early before this many rounds
NOTHING_FOUND = ("Not enough labels or training time yet: label more frames (spread over the recording) "
                 "or train longer.")


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


def footage(proj):
    """Seconds of footage per recording session (cameras of one session film the same time)."""
    out = {}
    for r in proj.recordings():
        key = r.get("session") or r["id"]
        times = [t for t in (_times(r["path"]) or []) if t is not None]
        d = (max(times) - min(times)) if len(times) > 1 else 0.0
        if d <= 0:          # recording not on this computer: judge from the labelled frame numbers
            lp = os.path.join(proj.labels_dir(r["id"]), "labels.csv")
            if os.path.exists(lp):
                with open(lp, newline="", encoding="utf-8") as f:
                    frames = [int(x["frame"]) for x in csv.DictReader(f) if x.get("frame", "").isdigit()]
                d = max(frames) / 25.0 if frames else 0.0
        out[key] = max(out.get(key, 0.0), d)
    return out


def n_blocks(durations, block_s):
    return sum(max(1, int(math.ceil(d / block_s))) for d in durations.values() if d > 0)


def choose_block_s(proj):
    """The block length for the split, from the recordings' lengths: the longest choice (at most
    the project's block_s setting, 30 s) that cuts the footage into at least MIN_BLOCKS blocks,
    so training, picking and scoring each get frames. A project that already trained with a
    length that still fits keeps it, so no frames move between parts as footage is added."""
    longest = float(proj.data.get("settings", {}).get("block_s") or 30)
    durs = footage(proj)
    prev = next((m.get("block_s") for m in reversed(proj.models())
                 if m.get("block_s") and m.get("status") == "ready"), None)
    if prev and prev <= longest and n_blocks(durs, prev) >= MIN_BLOCKS:
        return float(prev)
    choices = [b for b in BLOCK_CHOICES if b <= longest] or [longest]
    for b in choices:
        if n_blocks(durs, b) >= MIN_BLOCKS:
            return float(b)
    return float(choices[-1])


def guard_for(block_s):
    """Frames this close to a block edge are left out (1 s; less for short blocks)."""
    return min(1.0, block_s / 6.0)


def mark_interrupted(proj):
    """Models left "training" by a studio that was closed or crashed mid-training: they are
    not training any more. Returns their ids."""
    stuck = []
    with proj.lock:
        for m in proj.models():
            if m.get("status") == "training":
                m.update(status="failed", interrupted=True,
                         error="Stopped (interrupted): the studio was closed while this model was training. "
                               "Press Train new model to train again.")
                stuck.append(m["id"])
        if stuck:
            proj.save()
    return stuck


def split_message(counts, blocks, block_s):
    """Why there is not enough to train on yet, in plain words."""
    return ("Not enough confirmed frames in each part yet. To keep the scores honest, the recordings are cut into "
            "%d-second pieces and each piece is used for one job only. Now: %d frames for learning (in %d pieces), "
            "%d for picking the best training round (%d), %d held back for scoring (%d). Training needs at least "
            "10 for learning and at least 1 in each of the other two. Confirm frames spread over more of the "
            "recording (frames from different minutes, not neighbouring frames), then train again."
            % (round(block_s), counts["train"], blocks.get("train", 0), counts["val"], blocks.get("val", 0),
               counts["test"], blocks.get("test", 0)))


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
        # a frame holding an object marked unsure is left out of training, picking and scoring
        # until the person resolves it
        unsure = {row["frame"] for row in rows if (row.get("status") or "").strip() == "unsure"}
        rows = [row for row in rows if row["frame"] not in unsure]
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
        if "train" not in assign.values():
            # every labelled block landed in test or val: give training the one from the larger part
            # whose hash is nearest the training range
            parts = {p: [n for n in names if assign[n] == p] for p in ("test", "val")}
            big = max(parts, key=lambda p: (len(parts[p]), p))
            if len(parts[big]) > 1:
                assign[parts[big][-1]] = "train"
        for need in ("val", "test"):          # still a part with nothing: from the largest part
            if need not in assign.values():
                parts = {p: [n for n in names if assign[n] == p] for p in ("train", "val", "test") if p != need}
                big = max(parts, key=lambda p: (len(parts[p]), p))
                if len(parts[big]) > 1:
                    assign[parts[big][-1]] = need
    return assign, info


def count_frames(label_root, classes, assign, blocks=None):
    """Frames per part; with blocks={}, also fills in how many blocks of each part hold frames."""
    import detcore
    samples = detcore.read_many([label_root], classes, verbose=False) if os.listdir(label_root) else []
    out = dict(train=0, val=0, test=0)
    used = {}
    for s in samples:
        p = assign.get(s["recording"])
        if p:
            out[p] += 1
            used.setdefault(p, set()).add(s["recording"])
    if blocks is not None:
        blocks.update({p: len(v) for p, v in used.items()})
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
            import workarea
            try:
                rec = labelapp.Recording(proj.rec(rid)["path"])
            except SystemExit:          # the recording is not on this computer: no row for it
                continue
            dets = labelapp.load_detections(det_path, rec.n)
            robot_px = None
            info_p = os.path.join(proj.labels_dir(rid), "cache", "motion_info.json")
            if os.path.exists(info_p):
                with open(info_p) as jf:
                    robot_px = json.load(jf).get("robot_px")
            # the motion finder's own names (an object-sized blob, a big blob reaching in, ...) become
            # this project's types, as on the labelling page; so does its work area
            dets = labelapp.map_classes(dets, proj.motion_class_map(), proj.class_names())
            area = workarea.Area.from_settings(proj.camera_settings(proj.camera_key(proj.rec(rid))),
                                               default_margin=float(robot_px or 30.0))
            dets, _ = workarea.filter_guesses(dets, area)
            params = tracker.TrackerParams()
            params.loose_classes = tuple(proj.loose_classes())
            params.front_classes = tuple(proj.heading_classes())
            guesses, _ = tracker.TrackRun(dets, rec.times, robot_px or 30.0, (rec.width, rec.height), params=params,
                                          det_source="motion").run({}, set())
            for fr, b in frames_of.items():
                for d in guesses.get(fr, []):
                    w.writerow([b, fr, d["cls"], "%.2f" % d["x"], "%.2f" % d["y"],
                                "" if d.get("heading_deg") is None else "%.1f" % d["heading_deg"], "%.3f" % d["score"]])
                    n += 1
    return n


def finds_objects(m):
    """False for a model that found nothing when it was scored (marked with a warning, or an
    older model whose scores say it found no objects): a new model must not start from it."""
    if m.get("warning"):
        return False
    rec = (m.get("metrics") or {}).get("recall")
    return not (rec is not None and rec <= 0) and not (m.get("val_f1") is not None and m["val_f1"] <= 0)


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
        """The training thread. Whatever happens, the job ends as done or failed (never stuck
        "running") and the model entry as ready or failed."""
        proj = self.proj
        entry = None
        logf = None
        try:
            mdir = proj.model_dir(mid)
            os.makedirs(mdir, exist_ok=True)
            logf = open(os.path.join(mdir, "train.log"), "a", encoding="utf-8")
            entry = dict(id=mid, created=projmod.now_iso(), status="training", classes=proj.class_names(),
                         heading_classes=proj.heading_classes(), n_train_frames=0, metrics={}, minutes=minutes)
            with proj.lock:
                proj.data["models"].append(entry)
                proj.save()
            self._train(mid, minutes, python, entry, mdir, logf)
        except BaseException as e:      # keep the studio running; show the problem in plain words
            plain = isinstance(e, (RuntimeError, ValueError)) and not isinstance(e, NotImplementedError)
            msg = str(e) if plain else ("an unexpected problem (%s: %s). The details are in models/%s/train.log. "
                                        "Your labels are safe; press Train new model to try again."
                                        % (type(e).__name__, e, mid))
            if not plain:
                import traceback
                if logf:
                    traceback.print_exc(file=logf)
            try:
                if entry is not None:
                    with proj.lock:
                        entry["status"] = "failed"
                        entry["error"] = msg
                        proj.save()
            except Exception as e2:         # even saving failed: still end the job
                self.log("Could not save the failed model entry: %s" % e2)
            self.job.update(status="failed", finished=time.time(), message="Training %s stopped: %s" % (mid, msg))
            self.log(self.job["message"])
            if logf:
                try:
                    logf.write("FAILED: %s\n" % msg)
                except Exception:
                    pass
        finally:
            if logf:
                logf.close()

    def _train(self, mid, minutes, python, entry, mdir, logf):
        proj = self.proj
        data = os.path.join(mdir, "data")
        block_s = choose_block_s(proj)
        entry["block_s"] = block_s
        assign, info = build_dataset(proj, data, block_s, guard_s=guard_for(block_s))
        blocks = {}
        counts, _ = count_frames(data, proj.class_names(), assign, blocks)
        entry["split_counts"] = counts
        if counts["train"] < 10 or counts["val"] == 0 or counts["test"] == 0:
            raise RuntimeError(split_message(counts, blocks, block_s))
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
               "--split", split, "--max-minutes", "%.1f" % minutes, "--epochs", str(EPOCHS),
               "--min-epochs", str(MIN_EPOCHS)]
        if sizes:
            cmd += ["--obj-size", ",".join(sizes)]
        prev = self._previous_same_classes()
        if prev:
            cmd += ["--init-from", prev]
        self._say("Training model %s on %d frames (%d to pick the best round, %d held back for scoring); "
                  "at most %.0f min ..." % (mid, counts["train"], counts["val"], counts["test"], minutes))
        self._call(cmd, logf, minutes * 60 + 600, progress=dict(mid=mid, epochs=EPOCHS, minutes=minutes))
        best = os.path.join(run, "best.pt")
        if not os.path.exists(best):
            raise RuntimeError("training finished without a saved model (see %s)" % os.path.join(mdir, "train.log"))
        val_f1 = best_val_f1(os.path.join(run, "train_log.csv"))
        self._say("Model %s: exporting ..." % mid)
        onnx = os.path.join(mdir, "model.onnx")
        self._call([python, os.path.join(DETECTOR, "export.py"), best, "--out", onnx], logf, 600)
        with proj.lock:
            entry.update(status="ready", n_train_frames=counts["train"], onnx=os.path.relpath(onnx, proj.folder),
                         init_from=os.path.relpath(prev, proj.folder) if prev else None, val_f1=val_f1)
            proj.save()
        self._say("Model %s: scoring every model on the held-back frames ..." % mid)
        self.score_all(data, assign, info, logf, python, block_s=block_s)
        m = proj.model(mid)
        sc = m.get("metrics", {})
        nothing = (val_f1 is not None and val_f1 <= 0) or not sc.get("recall")
        if nothing:
            with proj.lock:
                m["warning"] = NOTHING_FOUND
                proj.save()
            msg = ("Model %s is saved, but it does not find the objects yet. %s It is marked with a warning, and "
                   "the next model will not start from it." % (mid, NOTHING_FOUND))
        else:
            msg = ("Model %s is ready: it finds %s of the objects on frames it never saw, and %s of what it marks "
                   "is right. Pick it in \"Guesses from\"." % (mid, _pct(sc.get("recall")), _pct(sc.get("precision"))))
        self.job.update(status="done", finished=time.time(), message=msg, warning=NOTHING_FOUND if nothing else None)
        self.log(msg)

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
            if m.get("status") == "ready" and m.get("classes") == self.proj.class_names() and finds_objects(m):
                p = os.path.join(self.proj.model_dir(m["id"]), "run", "best.pt")
                if os.path.exists(p):
                    return p
        return None

    def _call(self, cmd, logf, timeout, progress=None):
        """Run a step, writing its output to train.log. progress: dict(mid, epochs, minutes) for
        training, turned into a plain status line."""
        logf.write("\n$ %s\n" % " ".join(cmd))
        logf.flush()
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     cwd=DETECTOR)
        t0 = time.time()
        watch = TrainProgress(progress["epochs"], progress["minutes"], t0) if progress else None
        for line in self.proc.stdout:
            logf.write(line)
            logf.flush()
            if watch and watch.feed(line):
                self.job["message"] = "Training model %s: %s" % (progress["mid"], watch.text(time.time()))
            if time.time() - t0 > timeout:
                self.proc.kill()
                raise RuntimeError("took too long; stopped")
        rc = self.proc.wait()
        self.proc = None
        if rc != 0:
            raise RuntimeError("%s exited with an error (see train.log)" % os.path.basename(cmd[1]))

    def score_all(self, data, assign, info, logf, python=sys.executable, block_s=None):
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
                with proj.lock:
                    proj.model(mid)["metrics"] = {k: pt[k] for k in ("precision", "recall", "f1", "centre_err_px",
                                                                 "heading_err_deg", "ms_per_frame")}
        pts.sort(key=lambda p: (p["n_train_frames"], p["model"]))
        if block_s is None:
            block_s = float(proj.data["settings"].get("block_s", 30))
        curve = dict(updated=projmod.now_iso(), points=pts,
                     test=dict(frames=test_frames, blocks=sum(1 for v in assign.values() if v == "test"),
                               block_s=block_s,
                               note="%d time blocks of %.0f s held back; never trained on"
                                    % (sum(1 for v in assign.values() if v == "test"), block_s)))
        projmod.write_json(os.path.join(proj.folder, "models", "curve.json"), curve)
        proj.save()
        return curve


EPOCH_RE = re.compile(r"^epoch\s+(\d+)\s+([\d.]+)s.*?\bR\s+([\d.na]+)\s+F1\s+([\d.na]+)(.*)$")


class TrainProgress:
    """train.py's per-round lines -> a plain status: round x of about y, time left, and how many
    of the objects the best round so far finds (on the frames used to pick the best round)."""

    def __init__(self, epochs, minutes, t0):
        self.epochs, self.limit_s, self.t0 = int(epochs), float(minutes) * 60.0, t0
        self.epoch, self.secs, self.best_recall = 0, [], None

    def feed(self, line):
        m = EPOCH_RE.match(line.strip())
        if not m:
            return False
        self.epoch = int(m.group(1))
        self.secs.append(float(m.group(2)))
        if m.group(5).rstrip().endswith("*"):
            r = _f(m.group(3))
            self.best_recall = r if r is not None else 0.0
        return True

    def text(self, now):
        spent = now - self.t0
        per = sum(self.secs[-5:]) / len(self.secs[-5:]) if self.secs else 0.0
        about = self.epochs
        if per > 0 and self.limit_s:
            about = min(self.epochs, max(self.epoch, int(self.epoch + max(0.0, self.limit_s - spent) / per)))
        left = min(max(0, about - self.epoch) * per, max(0.0, self.limit_s - spent) if self.limit_s else 1e9)
        out = "round %d of about %d" % (self.epoch, about)
        if per > 0:
            out += ", " + (_left(left) if left > 0 else "finishing")
        if self.best_recall:
            out += ". Best round so far finds %s of the objects" % _pct(self.best_recall)
        else:
            out += ". It finds nothing yet (normal for the first rounds)"
        return out + "."


def _left(s):
    if s < 60:
        return "under a minute left"
    return "about %d min left" % int(round(s / 60.0))


def best_val_f1(log_csv):
    """The best F1 on the frames used to pick the best round, from train.py's train_log.csv."""
    try:
        with open(log_csv, newline="") as f:
            vals = [_f(r.get("val_f1")) for r in csv.DictReader(f)]
    except OSError:
        return None
    vals = [v for v in vals if v is not None]
    return max(vals) if vals else None


def _f(v):
    try:
        x = float(v)
        return None if x != x else x
    except (TypeError, ValueError):
        return None


def _pct(v):
    return "-" if v is None else "%d%%" % round(100 * v)
