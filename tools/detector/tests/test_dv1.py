"""detector-v1 (12 Nov 2026): labels from a picture-only run (autolabel --pictures), roof changes (roofmask.py,
--roof-p) and the PyTorch-free trainer (train_jax.py: same network, same ONNX interface)."""
import csv
import json
import math
import os

import numpy as np
import pytest

import detcore
import roofmask

cv2 = pytest.importorskip("cv2")


# ---------------------------------------------------------------------------------------------- roofmask
@pytest.mark.parametrize("mode", roofmask.MODES)
def test_roof_change_stays_on_the_roof(mode):
    rng = np.random.default_rng(0)
    img = np.full((120, 160, 3), 200, np.uint8)
    img[45:75, 65:95] = 40                                 # a dark 30 px robot at (80, 60)
    img[50:56, 70:90] = 255                                # a bright mark on its roof
    before = img.copy()
    roofmask.apply(img, 80.0, 60.0, 0.0, 30.0, mode, rng, frac=0.75)
    changed = np.argwhere(np.any(img != before, axis=2))
    assert len(changed), mode
    assert changed[:, 0].min() >= 60 - 12 and changed[:, 0].max() <= 60 + 12      # inside the 22.5 px square
    assert changed[:, 1].min() >= 80 - 12 and changed[:, 1].max() <= 80 + 12
    if mode in ("blur", "flat"):
        assert img[52, 80, 0] < 250                        # the roof's bright mark is gone


def test_roof_square_turns_with_the_heading():
    q0 = roofmask._square(0, 0, 0.0, 10)
    q45 = roofmask._square(0, 0, 45.0, 10)
    # turning 45 deg puts a corner on the axis: the half-diagonal 7.07 px straight up (picture y negative)
    assert abs(min(q45[:, 1]) + 5 * math.sqrt(2)) < 1e-4
    assert abs(min(q0[:, 1]) + 5) < 1e-4


def test_roofmask_copy_keeps_labels(tmp_path):
    from conftest import write_labels
    img = np.full((480, 640, 3), 180, np.uint8)
    img[290:310, 190:210] = 30
    d = write_labels(str(tmp_path / "src" / "recA" / "labels"),
                     [[0, 0, "Left", "alvik", 200, 300, 90, 1, "t", "confirmed", "1"]], {0: img})
    out = roofmask.copy_recordings([d], str(tmp_path / "out"), "blur", size_px=20)
    with open(os.path.join(out[0], "labels", "labels.csv")) as f, open(os.path.join(d, "labels.csv")) as g:
        assert f.read() == g.read()
    assert os.path.exists(os.path.join(out[0], "labels", "frames", "000000.jpg"))


# ---------------------------------------------------------------------------------------------- autolabel --pictures
def _sim_picture_run(folder, n=40, seed=3):
    """A run folder as the 11 Nov home run looks: pictures.csv + pictures/*_overhead.jpg, bridge/track.csv,
    follow.csv, map.json; the board 100 x 100 cm seen through a known homography; three dark robots with a bright front."""
    rng = np.random.default_rng(seed)
    H = np.array([[2.4, 0.15, 150.0], [-0.1, 2.3, 40.0], [0.0, 0.0003, 1.0]])
    os.makedirs(os.path.join(folder, "pictures"))
    os.makedirs(os.path.join(folder, "bridge"))
    bg = np.full((480, 640), 70, np.uint8)
    corners = np.float32([[0, 0], [100, 0], [100, 100], [0, 100]])
    quad = cv2.perspectiveTransform(corners[None], H)[0]
    cv2.fillConvexPoly(bg, np.round(quad).astype(np.int32), 190)
    bg = cv2.GaussianBlur(bg, (3, 3), 0)
    robots = {"A": [20.0, 30.0, 0.0], "B": [70.0, 60.0, 90.0], "C": [40.0, 80.0, -120.0]}
    track, follow, pics = [], [], []
    t0 = 1.79e9
    for k in range(n):
        t = t0 + 10.0 * k
        img = bg.copy()
        for name, p in robots.items():
            if name != "C" or k < n // 2:          # C stands still for the second half
                p[2] += float(rng.uniform(-40, 40))
                step = 6.0
                nx, ny = p[0] + step * math.cos(math.radians(p[2])), p[1] + step * math.sin(math.radians(p[2]))
                if 12 < nx < 88 and 12 < ny < 88:
                    p[0], p[1] = nx, ny
            # drawn as a 9.5 cm square with a bright spot at the front
            h = math.radians(p[2])
            sq = [(p[0] + 4.75 * (math.cos(h) * a - math.sin(h) * b), p[1] + 4.75 * (math.sin(h) * a + math.cos(h) * b))
                  for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            px = cv2.perspectiveTransform(np.float32(sq)[None], H)[0]
            cv2.fillConvexPoly(img, np.round(px).astype(np.int32), 35)
            fx = cv2.perspectiveTransform(np.float32([[(p[0] + 3.2 * math.cos(h), p[1] + 3.2 * math.sin(h))]]), H)[0, 0]
            cv2.circle(img, (int(fx[0]), int(fx[1])), 3, 250, -1)
            for dt in (-1.6, -1.1, -0.55, 0.0, 0.55, 1.1, 1.6):     # the track: before and after, the robot's motion
                back = 0.0 if (name == "C" and k >= n // 2) else dt * 4.0       # driving 4 cm/s
                track.append([t + dt, name, p[0] + back * math.cos(h), p[1] + back * math.sin(h), "seen"])
            follow.append([name, t, p[0] + 0.3, p[1] - 0.2, p[2] + 2.0, "drive"])
        img = np.clip(img.astype(np.float32) + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)  # sensor noise
        fn = "%06d_A_test_overhead.jpg" % k
        cv2.imwrite(os.path.join(folder, "pictures", fn), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        pics.append([t, "A", "test", fn])
    with open(os.path.join(folder, "pictures.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "why", "files"])
        w.writerows(pics)
    with open(os.path.join(folder, "bridge", "track.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x", "y", "status", "sent_th", "why", "src"])
        for r in sorted(track):
            w.writerow(["%.3f" % r[0], r[1], "%.2f" % r[2], "%.2f" % r[3], r[4], "", "", "both"])
    with open(os.path.join(folder, "follow.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["robot", "t_recv", "x_cm", "y_cm", "th_deg", "state"])
        for r in follow:
            w.writerow([r[0], "%.3f" % r[1], r[2], r[3], r[4], r[5]])
    with open(os.path.join(folder, "map.json"), "w") as f:
        json.dump({"floor": {"x_min": 0, "y_min": 0, "x_max": 100, "y_max": 100}}, f)
    return H


def test_picture_run_labels(tmp_path):
    import autolabel
    run = str(tmp_path / "20261111-000000-sim")
    H = _sim_picture_run(run)
    res = autolabel.label_pictures_run(run, str(tmp_path / "labels"), blocks=2, log=lambda *a: None)
    assert sorted(res) == ["20261111-000000-sim_overhead_t1", "20261111-000000-sim_overhead_t2"]
    assert sum(r["labelled"] for r in res.values()) >= 36           # nearly every picture kept
    herrs = []
    for rec, st in res.items():
        assert detcore.find_homography_for(os.path.join(st["folder"], "labels")) is not None   # camera model written
        for smp in detcore.read_labels(os.path.join(st["folder"], "labels"), verbose=False):
            herrs += [o["heading"] for o in smp["objects"] if o["heading"] is not None]
    # the fitted model against the true one: a floor point lands within 3 px
    st = next(iter(res.values()))
    with open(os.path.join(st["folder"], "floor_calibration.json")) as f:
        Hf = np.array(json.load(f)["H_floor_to_image"])
    for x, y in ((20, 20), (50, 50), (80, 70)):
        a, b = Hf @ [x, y, 1.0], H @ [x, y, 1.0]
        assert math.hypot(*(a[:2] / a[2] - b[:2] / b[2])) < 3.0
    assert len(herrs) > 50                                             # most robots get a checked heading


def test_picture_run_drops_a_label_with_no_robot(tmp_path):
    """A camera position where no robot is drawn (a stale track) must drop that picture."""
    import autolabel
    run = str(tmp_path / "20261111-000000-sim")
    _sim_picture_run(run, n=30)
    rows = list(csv.reader(open(os.path.join(run, "bridge", "track.csv"))))
    t_bad = float(list(csv.DictReader(open(os.path.join(run, "pictures.csv"))))[10]["t"])
    others = [(float(r[2]), float(r[3])) for r in rows[1:] if r[1] != "B" and abs(float(r[0]) - t_bad) < 0.1]
    spot = max(((x, y) for x in range(15, 90, 5) for y in range(15, 90, 5)),
               key=lambda q: min(math.hypot(q[0] - a, q[1] - b) for a, b in others))   # empty board, far from A and C
    out = [rows[0]]
    for r in rows[1:]:
        if r[1] == "B" and abs(float(r[0]) - t_bad) < 2.0:
            r = [r[0], r[1], "%.2f" % spot[0], "%.2f" % spot[1]] + r[4:]  # B "seen" on empty board
        out.append(r)
    with open(os.path.join(run, "bridge", "track.csv"), "w", newline="") as f:
        csv.writer(f).writerows(out)
    msgs = []
    res = autolabel.label_pictures_run(run, str(tmp_path / "labels"), blocks=1, log=msgs.append)
    assert sum(r["dropped"] for r in res.values()) >= 1
    assert any("no robot under its label" in m for m in msgs)


# ---------------------------------------------------------------------------------------------- train_jax
def test_jax_net_matches_torch_layout():
    """train_jax builds model.CentreNet's exact layers (same state_dict names and shapes)."""
    torch = pytest.importorskip("torch")
    import model
    import train_jax as TJ
    cfg = TJ.default_config(classes=["alvik"], in_ch=1, width=0.5)
    assert cfg == model.default_config(classes=["alvik"], in_ch=1, width=0.5)
    sd = model.CentreNet(cfg).state_dict()
    p, st = TJ.init_params(cfg)
    want = {k: tuple(v.shape) for k, v in sd.items() if "num_batches" not in k}
    got = {k: tuple(v.shape) for k, v in list(p.items()) + list(st.items())}
    assert want == got
    del torch


def test_jax_onnx_matches_jax(tmp_path):
    jax = pytest.importorskip("jax")
    ort = pytest.importorskip("onnxruntime")
    import train_jax as TJ
    cfg = TJ.default_config(classes=["alvik", "hand"], in_ch=1, width=0.5, in_w=64, in_h=48)
    p, st = TJ.init_params(cfg, seed=1)
    rng = np.random.default_rng(2)
    st = {k: (np.abs(v) + rng.uniform(0.1, 0.5, v.shape)).astype(np.float32) if "var" in k else
          rng.normal(0, 0.1, v.shape).astype(np.float32) for k, v in st.items()}     # non-trivial BN to fold
    out = TJ.write_onnx(p, st, cfg, str(tmp_path / "m.onnx"))
    x = rng.normal(0, 1, (1, 1, 48, 64)).astype(np.float32)
    (h, o, d), _ = TJ.make_forward(cfg)(p, st, jax.numpy.asarray(x), False)
    d = np.array(d)
    d[:, 0] = 1 / (1 + np.exp(-d[:, 0]))
    got = ort.InferenceSession(out, providers=["CPUExecutionProvider"]).run(None, {"image": x})
    for a, b in zip([1 / (1 + np.exp(-np.asarray(h))), np.asarray(o), d], got):
        assert a.shape == b.shape and np.abs(a - b).max() < 1e-4
    import infer
    det = infer.load(out)                                    # the Pi's loader reads it (metadata, names)
    assert det.classes == ["alvik", "hand"]
    assert isinstance(infer.predict(det, np.full((480, 640, 3), 128, np.uint8)), list)
