"""Run the exported detector (.onnx) with ONNX Runtime. Needs only numpy,
onnxruntime and opencv, so it runs on the Raspberry Pi without PyTorch.

As a module (the detector interface, docs/data-formats.md):

    import infer
    model = infer.load("best.onnx")              # optional: threads=4, score_thresh=0.3
    dets = infer.predict(model, frame_bgr)        # [dict(cls, x, y, heading_deg, score), ...]

From the command line, on a recording, a video file, an image or the camera:

    python3 infer.py best.onnx --video ~/recordings/20261020-141500_E1/video.avi --csv preds.csv
    python3 infer.py best.onnx --camera 0 --seconds 20          # live: prints frame rate
    python3 infer.py best.onnx --image frame.jpg --draw out.jpg

x, y are camera-frame pixels; heading_deg follows the labels convention
(0 = image right, counter-clockwise on screen positive), None for classes
without a front (hand, unknown).
"""
import argparse
import csv
import json
import os
import sys
import time

import cv2
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402

META_KEY = "alvik_detector"


class OnnxDetector:
    """Holds the ONNX Runtime session and the settings read from the file."""

    def __init__(self, path, threads=None, score_thresh=None, input_size=None, max_objects=50):
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        if threads:
            so.intra_op_num_threads = int(threads)
            so.inter_op_num_threads = 1
        self.session = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])
        meta = self.session.get_modelmeta().custom_metadata_map
        if META_KEY not in meta:
            raise ValueError(f"{path} has no detector settings in it; export it with export.py")
        self.cfg = json.loads(meta[META_KEY])
        self.classes = self.cfg["classes"]
        self.stride = self.cfg["stride"]
        self.in_ch = self.cfg["in_ch"]
        self.in_w, self.in_h = input_size or (self.cfg["in_w"], self.cfg["in_h"])
        self.score_thresh = score_thresh if score_thresh is not None else self.cfg.get("score_thresh", 0.3)
        self.max_objects = max_objects
        self.input_name = self.session.get_inputs()[0].name
        self.path = path

    def raw(self, frame_bgr):
        """Network output maps for one frame, and the frame->input scale."""
        x, scale = detcore.preprocess(frame_bgr, self.in_w, self.in_h, self.in_ch)
        heat, off, hd = self.session.run(None, {self.input_name: x})
        return heat[0], off[0], hd[0], scale

    def __call__(self, frame_bgr):
        heat, off, hd, scale = self.raw(frame_bgr)
        return detcore.decode(heat, off, hd, self.cfg, scale, self.score_thresh, self.max_objects)


def load(path, threads=None, score_thresh=None, input_size=None):
    """Open a detector exported by export.py. threads: CPU threads for ONNX
    Runtime (default: all cores). input_size: (w, h) to run at another size
    than trained (multiples of 16)."""
    return OnnxDetector(path, threads, score_thresh, input_size)


def predict(model, frame_bgr):
    """Detections for one BGR frame: list of dict(cls, x, y, heading_deg, score)."""
    return model(frame_bgr)


def draw(frame, dets):
    """Draw detections on a copy of the frame (for checking by eye)."""
    out = frame.copy()
    colours = {"alvik": (0, 200, 0), "hand": (0, 140, 255), "unknown": (200, 200, 0), "machine": (255, 0, 200)}
    for d in dets:
        c = colours.get(d["cls"], (0, 0, 255))
        p = (int(round(d["x"])), int(round(d["y"])))
        cv2.circle(out, p, 12, c, 1, cv2.LINE_AA)
        if d["heading_deg"] is not None:
            dx, dy = detcore.heading_to_vec(d["heading_deg"])
            cv2.line(out, p, (int(p[0] + 18 * dx), int(p[1] + 18 * dy)), c, 2, cv2.LINE_AA)
        cv2.putText(out, f"{d['cls']} {d['score']:.2f}", (p[0] + 12, p[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model", help=".onnx from export.py")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--video", help="video file, e.g. a recording's video.avi")
    src.add_argument("--camera", help="camera number, e.g. 0")
    src.add_argument("--image", help="one image file")
    ap.add_argument("--csv", help="write predictions here: recording,frame,cls,x_px,y_px,heading_deg,score")
    ap.add_argument("--draw", help="image: save a drawn copy here; video/camera: save a drawn video (.avi)")
    ap.add_argument("--threads", type=int)
    ap.add_argument("--score-thresh", type=float)
    ap.add_argument("--seconds", type=float, default=0, help="camera: stop after this long")
    ap.add_argument("--every", type=int, default=1, help="only run on every n-th frame")
    a = ap.parse_args()

    model = load(a.model, a.threads, a.score_thresh)
    if a.image:
        img = cv2.imread(a.image)
        t = time.perf_counter()
        dets = predict(model, img)
        print(f"{len(dets)} detections in {1000 * (time.perf_counter() - t):.1f} ms")
        for d in dets:
            print(d)
        if a.draw:
            cv2.imwrite(a.draw, draw(img, dets))
        return

    if a.camera is not None:
        cap = cv2.VideoCapture(int(a.camera), cv2.CAP_V4L2) if os.name != "nt" else cv2.VideoCapture(int(a.camera))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        rec = "camera"
    else:
        cap = cv2.VideoCapture(a.video)
        rec = os.path.basename(os.path.dirname(os.path.abspath(a.video)))
    writer = None
    out_rows = []
    n, t_det, t0, f = 0, 0.0, time.time(), -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        f += 1
        if f % a.every:
            continue
        t = time.perf_counter()
        dets = predict(model, frame)
        t_det += time.perf_counter() - t
        n += 1
        for d in dets:
            out_rows.append([rec, f, d["cls"], f"{d['x']:.2f}", f"{d['y']:.2f}",
                             "" if d["heading_deg"] is None else f"{d['heading_deg']:.1f}", f"{d['score']:.3f}"])
        if a.draw:
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(a.draw, cv2.VideoWriter_fourcc(*"MJPG"), 30, (w, h))
            writer.write(draw(frame, dets))
        if n % 100 == 0:
            print(f"frame {f}: {len(dets)} objects, detector {1000 * t_det / n:.1f} ms/frame on average")
        if a.seconds and time.time() - t0 > a.seconds:
            break
    cap.release()
    if writer is not None:
        writer.release()
    if n:
        print(f"{n} frames, detector {1000 * t_det / n:.1f} ms per frame ({n / t_det:.1f} fps of detector time), "
              f"wall {n / (time.time() - t0):.1f} fps")
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["recording", "frame", "cls", "x_px", "y_px", "heading_deg", "score"])
            w.writerows(out_rows)
        print(f"Wrote {len(out_rows)} predictions to {a.csv}")


if __name__ == "__main__":
    main()
