"""Export trained weights (best.pt from train.py) to ONNX, for the Pi.

    python export.py runs/v1/best.pt                       # -> runs/v1/best.onnx
    python export.py runs/v1/best.pt --out alvik.onnx
    python export.py runs/v1/best.pt --int8 D:/recordings   # also an 8-bit version, calibrated on real frames

The ONNX file holds everything infer.py needs (class names, input size,
colour or grey, stride, threshold) in its metadata, so the Pi only needs
the one file plus numpy, onnxruntime and opencv.

The width and height of the input are left free ("dynamic") in the file, so
the same file can be run at other sizes (benchmark.py --sizes); the default
size is the one it was trained at.

--int8: also writes <name>.int8.onnx, with weights and activations stored as
8-bit integers ("static quantization", calibrated on frames from the given
labels folders). It can be faster on the Pi's ARM cores; check with
benchmark.py and evaluate.py whether it is faster AND still accurate.
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detcore  # noqa: E402
from model import load_checkpoint  # noqa: E402

META_KEY = "alvik_detector"


def export(weights, out, opset=17):
    model = load_checkpoint(weights)
    cfg = model.cfg
    dummy = torch.zeros(1, cfg["in_ch"], cfg["in_h"], cfg["in_w"])
    kwargs = dict(input_names=["image"], output_names=["heat", "offset", "heading"], opset_version=opset,
                  dynamic_axes={"image": {2: "height", 3: "width"}, "heat": {2: "h4", 3: "w4"},
                                "offset": {2: "h4", 3: "w4"}, "heading": {2: "h4", 3: "w4"}},
                  do_constant_folding=True)
    try:
        # The classic exporter: needs only the onnx package.
        torch.onnx.export(model, dummy, out, dynamo=False, **kwargs)
    except TypeError:           # older PyTorch without the dynamo argument
        torch.onnx.export(model, dummy, out, **kwargs)

    import onnx
    m = onnx.load(out)
    meta = dict(cfg)
    meta["format"] = 1
    entry = m.metadata_props.add()
    entry.key, entry.value = META_KEY, json.dumps(meta)
    onnx.checker.check_model(m)
    onnx.save(m, out)
    return model


def check(model, path):
    """Run the same random input through PyTorch and ONNX Runtime; they should agree."""
    import onnxruntime as ort
    cfg = model.cfg
    x = np.random.default_rng(0).normal(0, 1, (1, cfg["in_ch"], cfg["in_h"], cfg["in_w"])).astype(np.float32)
    with torch.no_grad():
        ref = [t.numpy() for t in model(torch.from_numpy(x))]
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    got = sess.run(None, {"image": x})
    return max(float(np.abs(a - b).max()) for a, b in zip(ref, got))


class _Calib:
    """Feeds real frames to the int8 calibration."""

    def __init__(self, samples, cfg, n):
        import cv2
        self.items = []
        idx = np.linspace(0, len(samples) - 1, min(n, len(samples))).astype(int)
        for i in idx:
            img = cv2.imread(samples[i]["image"])
            x, _ = detcore.preprocess(img, cfg["in_w"], cfg["in_h"], cfg["in_ch"])
            self.items.append({"image": x})
        self.it = iter(self.items)

    def get_next(self):
        return next(self.it, None)

    def rewind(self):
        self.it = iter(self.items)


def quantize(fp32_path, out_path, labels, cfg, n_frames=100):
    from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_static
    from onnxruntime.quantization.shape_inference import quant_pre_process

    class Reader(_Calib, CalibrationDataReader):
        pass

    samples = detcore.read_many(labels, cfg["classes"], verbose=False)
    pre = out_path + ".pre.onnx"
    quant_pre_process(fp32_path, pre, skip_symbolic_shape=True)   # our input size is left free
    try:
        quantize_static(pre, out_path, Reader(samples, cfg, n_frames), quant_format=QuantFormat.QDQ,
                        per_channel=True, activation_type=QuantType.QUInt8, weight_type=QuantType.QInt8)
    finally:
        os.remove(pre)
    # keep the metadata
    import onnx
    src, dst = onnx.load(fp32_path), onnx.load(out_path)
    del dst.metadata_props[:]
    dst.metadata_props.extend(src.metadata_props)
    onnx.save(dst, out_path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("weights", help="best.pt from train.py")
    ap.add_argument("--out", help="output .onnx (default: next to the weights)")
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--int8", nargs="+", metavar="LABELS", help="also make an 8-bit model, calibrated on these labels folders")
    ap.add_argument("--calib-frames", type=int, default=100)
    a = ap.parse_args()
    out = a.out or os.path.splitext(a.weights)[0] + ".onnx"
    model = export(a.weights, out, a.opset)
    diff = check(model, out)
    print(f"Wrote {out} ({os.path.getsize(out) / 1024:.0f} KB). PyTorch vs ONNX Runtime max difference: {diff:.2e}")
    if a.int8:
        q = os.path.splitext(out)[0] + ".int8.onnx"
        quantize(out, q, a.int8, model.cfg, a.calib_frames)
        print(f"Wrote {q} ({os.path.getsize(q) / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
