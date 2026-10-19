"""The research-only YOLO comparison (baseline_yolo.py): it must stay out of the
delivered program, and it must answer the same detector interface as the others."""
import json
import os
import re
import subprocess
import sys

import numpy as np
import pytest

import baseline_yolo
import detcore

DET_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(DET_DIR))
IMPORT_RE = re.compile(r"^\s*(import|from)\s+(ultralytics|baseline_yolo)\b|import_module\(\s*['\"](ultralytics|baseline_yolo)",
                       re.M)


def _py_files():
    for root, dirs, files in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in (".git", "__pycache__", "runs", ".venv", "venv")]
        for f in files:
            if f.endswith(".py"):
                yield os.path.join(root, f)


def test_ultralytics_stays_out_of_the_program():
    """AGPL-3.0: only baseline_yolo.py may import ultralytics, and only tests may import baseline_yolo."""
    me = os.path.abspath(baseline_yolo.__file__)
    tests_dir = os.path.join(DET_DIR, "tests")
    bad = []
    for path in _py_files():
        with open(path, encoding="utf-8", errors="replace") as f:
            for m in IMPORT_RE.finditer(f.read()):
                what = m.group(2) or m.group(3)
                if what == "ultralytics" and os.path.abspath(path) != me:
                    bad.append(f"{os.path.relpath(path, REPO)} imports ultralytics")
                if what == "baseline_yolo" and not os.path.abspath(path).startswith(tests_dir):
                    bad.append(f"{os.path.relpath(path, REPO)} imports baseline_yolo")
    assert not bad, bad
    req = open(os.path.join(REPO, "requirements.txt")).read().lower()
    assert "ultralytics" not in req
    research = open(os.path.join(REPO, "requirements-research.txt")).read().lower()
    assert "ultralytics" in research


def test_label_lines_boxes_and_front_keypoint():
    classes = detcore.DEFAULT_CLASSES
    objs = [dict(cls="alvik", x=319.5, y=239.5, heading=90.0), dict(cls="hand", x=99.5, y=49.5, heading=None)]
    det = baseline_yolo.label_lines(objs, classes, {"alvik": 32, "hand": 64}, 640, 480)
    assert det[0].split() == ["0", "0.500000", "0.500000", "0.050000", "0.066667"]
    assert det[1].split()[0] == "1" and len(det[1].split()) == 5
    pose = baseline_yolo.label_lines(objs, classes, {"alvik": 32, "hand": 64}, 640, 480, "pose", {"alvik"}, 0.5)
    kx, ky, v = pose[0].split()[5:]
    # heading 90 deg = up on screen: keypoint 16 px above the centre
    assert float(kx) == pytest.approx(0.5) and float(ky) == pytest.approx((240 - 16) / 480) and v == "2"
    assert pose[1].split()[5:] == ["0", "0", "0"]          # hands have no front


@pytest.mark.parametrize("task", ["detect", "pose"])
def test_train_and_interface(tiny_synth, tmp_path, task):
    """Random start (no download), one tiny epoch, then load()/predict() and evaluate.py's KIND:PATH."""
    pytest.importorskip("ultralytics")
    out = str(tmp_path / "yolo")
    subprocess.run([sys.executable, os.path.join(DET_DIR, "baseline_yolo.py"), "train", "--labels", tiny_synth,
                    "--out", out, "--weights", "none", "--task", task, "--epochs", "1", "--batch", "4"],
                   check=True, cwd=str(tmp_path))
    with open(os.path.join(out, baseline_yolo.CONFIG_NAME)) as f:
        assert json.load(f)["task"] == task
    det = baseline_yolo.load(os.path.join(out, "best.pt"), score_thresh=0.0, threads=1)
    import torch
    assert torch.get_num_threads() == 1
    dets = baseline_yolo.predict(det, np.full((480, 640, 3), 150, np.uint8))
    assert isinstance(dets, list)
    for d in dets:
        assert set(d) == {"cls", "x", "y", "heading_deg", "score"}
        assert d["cls"] in detcore.DEFAULT_CLASSES and 0.0 <= d["score"] <= 1.0
        if task == "detect" or d["cls"] not in ("alvik", "machine"):
            assert d["heading_deg"] is None
    res = str(tmp_path / "eval.csv")
    subprocess.run([sys.executable, os.path.join(DET_DIR, "evaluate.py"), "--labels", tiny_synth, "--split",
                    os.path.join(out, "split.json"), "--model", f"baseline_yolo:{os.path.join(out, 'best.pt')}",
                    "--out", res], check=True, cwd=DET_DIR)
    assert os.path.isfile(res)
