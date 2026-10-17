"""How fast is the detector? Times it frame by frame and prints a table.
Made to run on the Raspberry Pi too (needs only numpy, onnxruntime, opencv
for .onnx models).

    python3 benchmark.py --model best.onnx                           # default sizes and thread counts
    python3 benchmark.py --model best.onnx --sizes 320x240,640x480 --threads 1,2,4
    python3 benchmark.py --model best.onnx best.int8.onnx --image frame.jpg --csv bench.csv
    python3 benchmark.py --arch 0.5x1,1.0x1,1.0x2,1.5x2                  # untrained designs (needs PyTorch)
    python3 benchmark.py --detector baseline_tv --weights runs/base/best.pt   # any detector module

For every combination it runs a few warm-up frames (not timed: the first
runs are slow while memory is set up), then times --iters frames and reports:

- model_ms: the network alone (ONNX Runtime run), mean
- mean_ms, p50_ms, p95_ms: the whole predict() call (shrink the frame,
  network, find peaks), which is what the live system pays per frame.
  p50 is the typical frame, p95 is a slow frame (95% are faster).
- fps: frames per second at the mean (1000 / mean_ms)

Use a real camera frame (--image) when you can: peak finding takes longer
when there are objects in the picture. Close other programs while timing;
on the Pi, also check it is not hot and throttling (vcgencmd get_throttled).
"""
import argparse
import importlib
import importlib.util
import os
import platform
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402


def parse_sizes(s):
    out = []
    for part in s.split(","):
        w, h = part.lower().split("x")
        out.append((int(w), int(h)))
    return out


def timeit(fn, warmup, iters):
    for _ in range(warmup):
        fn()
    t = np.empty(iters)
    for i in range(iters):
        t0 = time.perf_counter()
        fn()
        t[i] = time.perf_counter() - t0
    return t * 1000.0


def test_frame(path):
    if path:
        img = cv2.imread(path)
        if img is None:
            sys.exit(f"Cannot read {path}")
        return img
    # a plain grey floor with noise and a few dark squares, if no frame is given
    rng = np.random.default_rng(0)
    img = np.clip(rng.normal(150, 8, (480, 640, 3)), 0, 255).astype(np.uint8)
    for x, y in [(150, 120), (400, 300), (500, 100)]:
        cv2.rectangle(img, (x - 14, y - 14), (x + 14, y + 14), (40, 40, 40), -1)
    return img


def bench_onnx(path, name, sizes, threads, frame, warmup, iters):
    import infer
    rows = []
    for th in threads:
        for (w, h) in sizes:
            model = infer.load(path, threads=th, input_size=(w, h))
            x, _ = detcore.preprocess(frame, w, h, model.in_ch)
            feed = {model.input_name: x}
            t_model = timeit(lambda: model.session.run(None, feed), warmup, iters)
            t_full = timeit(lambda: infer.predict(model, frame), warmup // 2 + 1, iters)
            rows.append(row(name, f"{w}x{h}", th, t_model, t_full))
            print(f"  {name} {w}x{h} threads={th}: {np.mean(t_full):.2f} ms", flush=True)
    return rows


def bench_module(module, weights, name, threads, frame, warmup, iters):
    rows = []
    for th in threads:
        try:
            import torch
            torch.set_num_threads(th)
        except ImportError:
            pass
        model = module.load(weights)
        t_full = timeit(lambda: module.predict(model, frame), warmup, iters)
        rows.append(row(name, "own", th, None, t_full))
        print(f"  {name} threads={th}: {np.mean(t_full):.2f} ms", flush=True)
    return rows


def row(name, size, th, t_model, t_full):
    return dict(model=name, input=size, threads=th,
                model_ms=float(np.mean(t_model)) if t_model is not None else float("nan"),
                mean_ms=float(np.mean(t_full)), p50_ms=float(np.percentile(t_full, 50)),
                p95_ms=float(np.percentile(t_full, 95)), fps=1000.0 / float(np.mean(t_full)))


def export_arch(spec, sizes, in_ch, tmpdir):
    """Build an untrained CentreNet of width x depth and export it, to time designs before training."""
    import torch
    from export import export
    from model import CentreNet, count_params, default_config
    w, d = spec.split("x")
    w0, h0 = sizes[0]
    cfg = default_config(width=float(w), depth=int(d), in_ch=in_ch, in_w=w0, in_h=h0)
    net = CentreNet(cfg).eval()
    pt = os.path.join(tmpdir, f"arch_{spec}.pt")
    torch.save(dict(model=net.state_dict(), config=cfg), pt)
    onnx_path = pt.replace(".pt", ".onnx")
    export(pt, onnx_path)
    return onnx_path, f"w{w}d{d} ({count_params(net) / 1000:.0f}k)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", nargs="*", default=[], help=".onnx files")
    ap.add_argument("--arch", help="comma list of WIDTHxDEPTH to build untrained and time, e.g. 0.5x1,1.0x1")
    ap.add_argument("--in-ch", type=int, default=3, help="for --arch: 3 colour or 1 grey")
    ap.add_argument("--detector", help="any module with load/predict, e.g. baseline_tv or predict_torch")
    ap.add_argument("--weights", help="weights for --detector")
    ap.add_argument("--sizes", default="320x240,480x352,640x480", help="input sizes for .onnx models")
    ap.add_argument("--threads", default="1,2,4", help="comma list of CPU thread counts")
    ap.add_argument("--image", help="a real camera frame to time on (recommended)")
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--csv", help="also write the table here")
    a = ap.parse_args()

    sizes = parse_sizes(a.sizes)
    threads = [int(t) for t in a.threads.split(",")]
    frame = test_frame(a.image)
    import onnxruntime
    print(f"Machine: {platform.machine()} {platform.processor() or ''}, {os.cpu_count()} cores, "
          f"Python {platform.python_version()}, onnxruntime {onnxruntime.__version__}, OpenCV {cv2.__version__}")
    print(f"Frame {frame.shape[1]}x{frame.shape[0]}, warm-up {a.warmup}, timed {a.iters} frames each\n")

    rows = []
    for m in a.model:
        rows += bench_onnx(m, os.path.basename(m), sizes, threads, frame, a.warmup, a.iters)
    if a.arch:
        with tempfile.TemporaryDirectory() as tmp:
            for spec in a.arch.split(","):
                path, name = export_arch(spec, sizes, a.in_ch, tmp)
                rows += bench_onnx(path, name, sizes, threads, frame, a.warmup, a.iters)
    if a.detector:
        if a.detector.endswith(".py"):
            spec = importlib.util.spec_from_file_location("det_mod", a.detector)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
        else:
            mod = importlib.import_module(a.detector)
        rows += bench_module(mod, a.weights, a.detector, threads, frame, a.warmup, a.iters)
    if not rows:
        sys.exit("Nothing to time: give --model, --arch or --detector.")

    print()
    print(detcore.format_table(rows, ["model", "input", "threads", "model_ms", "mean_ms", "p50_ms", "p95_ms", "fps"],
                               "{:.1f}"))
    if a.csv:
        detcore.write_csv(rows, a.csv)
        print(f"\nWrote {a.csv}")


if __name__ == "__main__":
    main()
