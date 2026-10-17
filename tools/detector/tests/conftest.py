"""Make the detector modules importable in tests, and shared helpers."""
import csv
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

HEADER = ["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known", "source", "status", "score"]


def write_labels(folder, rows, images=None):
    """Write a labels folder. rows: lists in HEADER order. images: {frame: BGR array}."""
    import cv2
    import numpy as np
    os.makedirs(os.path.join(folder, "frames"), exist_ok=True)
    with open(os.path.join(folder, "labels.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        w.writerows(rows)
    frames = {int(r[0]) for r in rows}
    for fr in frames:
        img = (images or {}).get(fr)
        if img is None:
            img = np.full((480, 640, 3), 150, np.uint8)
        cv2.imwrite(os.path.join(folder, "frames", f"{fr:06d}.jpg"), img)
    return folder


@pytest.fixture
def tiny_synth(tmp_path_factory):
    """A very small synthetic dataset (4 recordings x 10 frames)."""
    import numpy as np
    import make_synthetic
    out = tmp_path_factory.mktemp("synth")
    rng = np.random.default_rng(5)
    for i in range(4):
        make_synthetic.make_recording(os.path.join(out, f"rec{i}", "labels"), 10, rng, 1.79e9)
    return str(out)
