#!/usr/bin/env python3
"""Train the same detector (CentreNet, model.py) WITHOUT PyTorch, with JAX, and write the ONNX file directly.

Why this exists (detector-v1, 12 Nov 2026): the machines the agents work on cannot install PyTorch (its CPU download
site is refused; the PyPI build carries GPU libraries too big for the disk), while JAX installs from PyPI. train.py
stays the main trainer; this file is a stand-in that gives the SAME network, the SAME training data and augmentation
(dataset.py), the SAME losses, optimiser and schedule, and the SAME .onnx interface (infer.py, evaluate.py,
benchmark.py and the Pi need nothing new). Differences from train.py: no copy-paste, no --init-from, no --select
macro, no GPU; one process prepares the batches (no loader workers).

    python3 train_jax.py --labels <labels root> --out <dir> --classes alvik --heading-classes alvik \
        --val <rec>[,<rec>] --test <rec>[,<rec>] --max-minutes 40 [--roof-p 0.5] [--in-ch 1]
    python3 train_jax.py --onnx <dir>/best.npz [--out x.onnx]     # (again) write the .onnx from saved weights
    python3 train_jax.py --to-pt <dir>/best.npz                   # on a machine WITH PyTorch: a best.pt for train.py
                                                                  # (--init-from), predict_torch.py, export.py

Writes to --out: best.npz / last.npz (weights under PyTorch's state_dict names + the config), best.onnx (float),
train_log.csv (the same columns as train.py), split.json, config.json.

Changelog
  2026-11-12 dv1  first version (the first real training: the 11 Nov home run, docs/results/detector-v1)
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import detcore  # noqa: E402

VERSION = "train_jax-2026-11-12dv1"
STRIDE = 4
BN_EPS, BN_MOM = 1e-5, 0.1          # PyTorch's BatchNorm2d defaults


# ---------------------------------------------------------------------------------------------- the network's layout
def default_config(**over):
    """model.default_config without importing PyTorch (kept identical; a test compares them where torch exists)."""
    cfg = dict(classes=list(detcore.DEFAULT_CLASSES), obj_size_px=dict(detcore.DEFAULT_OBJ_SIZE_PX),
               heading_classes=sorted(detcore.DEFAULT_HEADING_CLASSES), in_ch=3, in_w=320, in_h=240, width=1.0,
               depth=1, expand=2, stride=STRIDE, heading_mode="front", front_frac=0.35, frame_w=640, frame_h=480)
    cfg.update({k: v for k, v in over.items() if v is not None})
    return cfg


def _ch(c, width):
    return max(8, int(round(c * width / 8)) * 8)


def layout(cfg):
    """The layers in model.CentreNet's order, each (name, kind, args), with PyTorch's state_dict names.
    kinds: cbr (conv+BN, relu flag), conv (plain conv with bias). Blocks are listed as nested programs."""
    w, d, e = cfg["width"], cfg["depth"], cfg["expand"]
    c1, c2, c3, c4 = _ch(16, w), _ch(24, w), _ch(48, w), _ch(96, w)
    neck = _ch(32, w)
    n_cls = len(cfg["classes"])
    front = cfg.get("heading_mode", "front") == "front"

    def cbr(name, cin, cout, k=3, s=1, groups=1, relu=True):
        return dict(name=name, cin=cin, cout=cout, k=k, s=s, groups=groups, relu=relu)

    def ir(name, c):
        m = c * e
        return [cbr(name + ".body.0", c, m, 1), cbr(name + ".body.1", m, m, 3, groups=m),
                cbr(name + ".body.2", m, c, 1, relu=False)]

    def down(name, cin, cout):
        return [cbr(name + ".0", cin, cin, 3, 2, groups=cin), cbr(name + ".1", cin, cout, 1)]

    def sep(name, cin, cout):
        return [cbr(name + ".0", cin, cin, 3, groups=cin), cbr(name + ".1", cin, cout, 1)]

    L = dict(stem=[cbr("stem", cfg["in_ch"], c1, 3, 2)],
             s4=[("seq", [cbr("s4.0", c1, c2, 3, 2)])] + [("ir", ir("s4.%d" % (i + 1), c2)) for i in range(d)],
             s8=[("seq", down("s8.0", c2, c3))] + [("ir", ir("s8.%d" % (i + 1), c3)) for i in range(d)],
             s16=[("seq", down("s16.0", c3, c4))] + [("ir", ir("s16.%d" % (i + 1), c4)) for i in range(d)],
             lat16=[cbr("lat16", c4, neck, 1)], lat8=[cbr("lat8", c3, neck, 1)], lat4=[cbr("lat4", c2, neck, 1)],
             fuse8=sep("fuse8", neck, neck), fuse4=sep("fuse4", neck, neck), head=sep("head", neck, neck),
             head_hd=sep("head_hd.0", neck, neck) + sep("head_hd.1", neck, neck),
             out=dict(name="out", cin=neck, cout=n_cls + 2), out_hd=dict(name="out_hd", cin=neck, cout=3 if front else 2))
    return L


def all_cbr(L):
    out = []
    for k in ("stem", "s4", "s8", "s16", "lat16", "lat8", "lat4", "fuse8", "fuse4", "head", "head_hd"):
        for item in L[k]:
            if isinstance(item, tuple):
                out += item[1]
            else:
                out.append(item)
    return out


def init_params(cfg, seed=0):
    """Weights as model.CentreNet._init makes them (kaiming normal, fan_out, relu; BN 1/0; heat bias -4.6)."""
    rng = np.random.default_rng(seed)
    L = layout(cfg)
    p, st = {}, {}
    for c in all_cbr(L):
        k, g = c["k"], c["groups"]
        std = math.sqrt(2.0 / (c["cout"] * k * k))
        p[c["name"] + ".0.weight"] = rng.normal(0, std, (c["cout"], c["cin"] // g, k, k)).astype(np.float32)
        p[c["name"] + ".1.weight"] = np.ones(c["cout"], np.float32)
        p[c["name"] + ".1.bias"] = np.zeros(c["cout"], np.float32)
        st[c["name"] + ".1.running_mean"] = np.zeros(c["cout"], np.float32)
        st[c["name"] + ".1.running_var"] = np.ones(c["cout"], np.float32)
    n_cls = len(cfg["classes"])
    for o in (L["out"], L["out_hd"]):
        p[o["name"] + ".weight"] = rng.normal(0, 0.01, (o["cout"], o["cin"], 1, 1)).astype(np.float32)
        p[o["name"] + ".bias"] = np.zeros(o["cout"], np.float32)
    p["out.bias"][:n_cls] = -math.log((1 - 0.01) / 0.01)
    if cfg.get("heading_mode", "front") == "front":
        p["out_hd.bias"][0] = -math.log((1 - 0.01) / 0.01)
    return p, st


# ---------------------------------------------------------------------------------------------- forward (JAX)
def make_forward(cfg):
    import jax
    import jax.numpy as jnp
    L = layout(cfg)
    n_cls = len(cfg["classes"])

    def conv(x, w, s, groups, pad):
        return jax.lax.conv_general_dilated(x, w, (s, s), [(pad, pad), (pad, pad)], feature_group_count=groups,
                                            dimension_numbers=("NCHW", "OIHW", "NCHW"))

    def cbr(c, x, p, st, train, new_st):
        y = conv(x, p[c["name"] + ".0.weight"], c["s"], c["groups"], c["k"] // 2)
        g, b = p[c["name"] + ".1.weight"], p[c["name"] + ".1.bias"]
        if train:
            mu = y.mean(axis=(0, 2, 3))
            var = y.var(axis=(0, 2, 3))
            n = y.shape[0] * y.shape[2] * y.shape[3]
            new_st[c["name"] + ".1.running_mean"] = (1 - BN_MOM) * st[c["name"] + ".1.running_mean"] + BN_MOM * mu
            new_st[c["name"] + ".1.running_var"] = ((1 - BN_MOM) * st[c["name"] + ".1.running_var"] +
                                                    BN_MOM * var * n / max(1, n - 1))
        else:
            mu, var = st[c["name"] + ".1.running_mean"], st[c["name"] + ".1.running_var"]
        y = (y - mu[None, :, None, None]) * (g / jnp.sqrt(var + BN_EPS))[None, :, None, None] + b[None, :, None, None]
        return jax.nn.relu(y) if c["relu"] else y

    def run(items, x, p, st, train, new_st):
        for it in items:
            if isinstance(it, tuple):
                kind, cs = it
                if kind == "seq":
                    for c in cs:
                        x = cbr(c, x, p, st, train, new_st)
                else:                                   # inverted residual
                    y = x
                    for c in cs:
                        y = cbr(c, y, p, st, train, new_st)
                    x = jax.nn.relu(x + y)
            else:
                x = cbr(it, x, p, st, train, new_st)
        return x

    def up2(x):
        return jnp.repeat(jnp.repeat(x, 2, axis=2), 2, axis=3)

    def forward(p, st, x, train):
        """(heat logits, offset, heading raw), new BN state."""
        new_st = dict(st)
        f4 = run(L["s4"], run(L["stem"], x, p, st, train, new_st), p, st, train, new_st)
        f8 = run(L["s8"], f4, p, st, train, new_st)
        f16 = run(L["s16"], f8, p, st, train, new_st)
        q = run(L["lat16"], f16, p, st, train, new_st)
        q = run(L["fuse8"], up2(q) + run(L["lat8"], f8, p, st, train, new_st), p, st, train, new_st)
        q = run(L["fuse4"], up2(q) + run(L["lat4"], f4, p, st, train, new_st), p, st, train, new_st)
        o = conv(run(L["head"], q, p, st, train, new_st), p["out.weight"], 1, 1, 0) + p["out.bias"][None, :, None, None]
        hd = conv(run(L["head_hd"], q, p, st, train, new_st), p["out_hd.weight"], 1, 1, 0) + \
            p["out_hd.bias"][None, :, None, None]
        return (o[:, :n_cls], o[:, n_cls:], hd), new_st
    return forward


# ---------------------------------------------------------------------------------------------- losses (train.py's)
def make_loss(w_hd=1.0):
    import jax
    import jax.numpy as jnp

    def focal(logits, target, mask=None, neg_w=None):
        p = jax.nn.sigmoid(logits)
        pos = (target == 1).astype(jnp.float32)
        neg = 1.0 - pos
        if mask is not None:
            pos, neg = pos * mask, neg * mask
        if neg_w is not None:
            neg = neg * neg_w
        pos_loss = (jax.nn.log_sigmoid(logits) * (1 - p) ** 2 * pos).sum()
        neg_loss = (jax.nn.log_sigmoid(-logits) * p ** 2 * (1 - target) ** 4 * neg).sum()
        return -(pos_loss + neg_loss) / jnp.maximum(pos.sum(), 1.0)

    def l1(pred, target, weight):
        w = weight[:, None]
        return (jnp.abs(pred - target) * w).sum() / jnp.maximum(w.sum() * pred.shape[1], 1.0)

    def loss(outs, b):
        heat, off, hd = outs
        lh = focal(heat, b["heat"], neg_w=b["negw"])
        lo = l1(off, b["off"], b["off_w"])
        lhd = focal(hd[:, :1], b["fheat"], b["fmask"], neg_w=b["negw"]) + l1(hd[:, 1:], b["foff"], b["foff_w"])
        return lh + lo + w_hd * lhd, (lh, lo, lhd)
    return loss


# ---------------------------------------------------------------------------------------------- ONNX writer
def write_onnx(params, state, cfg, out, opset=17):
    """The float .onnx, the same graph and interface as export.py's (BatchNorm folded into the convolutions)."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    L = layout(cfg)
    n_cls = len(cfg["classes"])
    nodes, inits = [], []
    cnt = [0]

    def nm(base):
        cnt[0] += 1
        return "%s_%d" % (base, cnt[0])

    def const(name, arr):
        inits.append(numpy_helper.from_array(np.asarray(arr), name))
        return name

    def conv(x, w, b, s, groups, pad):
        y = nm("conv")
        nodes.append(helper.make_node("Conv", [x, const(y + "_w", w.astype(np.float32)), const(y + "_b", b.astype(
            np.float32))], [y], strides=[s, s], pads=[pad] * 4, group=groups, kernel_shape=list(w.shape[2:])))
        return y

    def cbr(c, x):
        g = params[c["name"] + ".1.weight"] / np.sqrt(state[c["name"] + ".1.running_var"] + BN_EPS)
        w = params[c["name"] + ".0.weight"] * g[:, None, None, None]
        b = params[c["name"] + ".1.bias"] - state[c["name"] + ".1.running_mean"] * g
        y = conv(x, w, b, c["s"], c["groups"], c["k"] // 2)
        if c["relu"]:
            r = nm("relu")
            nodes.append(helper.make_node("Relu", [y], [r]))
            return r
        return y

    def run(items, x):
        for it in items:
            if isinstance(it, tuple):
                kind, cs = it
                if kind == "seq":
                    for c in cs:
                        x = cbr(c, x)
                else:
                    y = x
                    for c in cs:
                        y = cbr(c, y)
                    a, r = nm("add"), nm("relu")
                    nodes.append(helper.make_node("Add", [x, y], [a]))
                    nodes.append(helper.make_node("Relu", [a], [r]))
                    x = r
            else:
                x = cbr(it, x)
        return x

    scales = const("up_scales", np.array([1, 1, 2, 2], np.float32))
    const("empty_roi", np.array([], np.float32))

    def up2(x):
        y = nm("up")
        nodes.append(helper.make_node("Resize", [x, "empty_roi", scales], [y], mode="nearest",
                                      coordinate_transformation_mode="asymmetric", nearest_mode="floor"))
        return y

    def add(a, b):
        y = nm("add")
        nodes.append(helper.make_node("Add", [a, b], [y]))
        return y

    f4 = run(L["s4"], run(L["stem"], "image"))
    f8 = run(L["s8"], f4)
    f16 = run(L["s16"], f8)
    q = run(L["lat16"], f16)
    q = run(L["fuse8"], add(up2(q), run(L["lat8"], f8)))
    q = run(L["fuse4"], add(up2(q), run(L["lat4"], f4)))
    o = conv(run(L["head"], q), params["out.weight"], params["out.bias"], 1, 1, 0)
    hd = conv(run(L["head_hd"], q), params["out_hd.weight"], params["out_hd.bias"], 1, 1, 0)
    nh = L["out_hd"]["cout"]
    nodes.append(helper.make_node("Split", [o, const("split_o", np.array([n_cls, 2], np.int64))], ["heat_l", "offset"],
                                  axis=1))
    nodes.append(helper.make_node("Sigmoid", ["heat_l"], ["heat"]))
    if cfg.get("heading_mode", "front") == "front":
        nodes.append(helper.make_node("Split", [hd, const("split_h", np.array([1, nh - 1], np.int64))],
                                      ["hd0", "hd1"], axis=1))
        nodes.append(helper.make_node("Sigmoid", ["hd0"], ["hd0s"]))
        nodes.append(helper.make_node("Concat", ["hd0s", "hd1"], ["heading"], axis=1))
    else:
        nodes.append(helper.make_node("Identity", [hd], ["heading"]))
    inp = helper.make_tensor_value_info("image", TensorProto.FLOAT, [1, cfg["in_ch"], "height", "width"])
    outs = [helper.make_tensor_value_info("heat", TensorProto.FLOAT, [1, n_cls, "h4", "w4"]),
            helper.make_tensor_value_info("offset", TensorProto.FLOAT, [1, 2, "h4", "w4"]),
            helper.make_tensor_value_info("heading", TensorProto.FLOAT, [1, nh, "h4", "w4"])]
    g = helper.make_graph(nodes, "CentreNet", [inp], outs, inits)
    m = helper.make_model(g, opset_imports=[helper.make_opsetid("", opset)], producer_name=VERSION)
    m.ir_version = 8
    meta = dict(cfg)
    meta["format"] = 1
    meta["trained_with"] = VERSION
    e = m.metadata_props.add()
    e.key, e.value = "alvik_detector", json.dumps(meta)
    onnx.checker.check_model(m)
    onnx.save(m, out)
    return out


def save_npz(path, params, state, cfg, extra=None):
    arrs = {k: np.asarray(v) for k, v in params.items()}
    arrs.update({k: np.asarray(v) for k, v in state.items()})
    arrs["__config__"] = np.frombuffer(json.dumps(dict(cfg, **(extra or {}))).encode(), np.uint8)
    np.savez(path, **arrs)


def load_npz(path):
    z = np.load(path)
    cfg = json.loads(bytes(z["__config__"]).decode())
    params = {k: z[k] for k in z.files if k != "__config__" and "running_" not in k}
    state = {k: z[k] for k in z.files if "running_" in k}
    return params, state, cfg


def to_pt(npz, out=None):
    """A best.pt for train.py / predict_torch.py / export.py (needs PyTorch)."""
    import torch
    params, state, cfg = load_npz(npz)
    sd = {k: torch.from_numpy(np.array(v)) for k, v in list(params.items()) + list(state.items())}
    for k in list(state):
        sd[k.replace("running_mean", "num_batches_tracked")] = torch.tensor(0)
    model_cfg = {k: v for k, v in cfg.items() if k in default_config()} | {"score_thresh": cfg.get("score_thresh", 0.3)}
    out = out or os.path.splitext(npz)[0] + ".pt"
    torch.save(dict(model=sd, config=model_cfg, epoch=cfg.get("epoch"), val=cfg.get("val")), out)
    return out


# ---------------------------------------------------------------------------------------------- training
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--labels", nargs="+")
    ap.add_argument("--out")
    ap.add_argument("--onnx", help="write the .onnx from a saved .npz and stop")
    ap.add_argument("--to-pt", help="write a PyTorch best.pt from a saved .npz and stop (needs PyTorch)")
    ap.add_argument("--classes")
    ap.add_argument("--obj-size")
    ap.add_argument("--heading-classes")
    ap.add_argument("--size", default="320x240")
    ap.add_argument("--in-ch", type=int, default=3, choices=[1, 3])
    ap.add_argument("--width", type=float, default=1.0)
    ap.add_argument("--depth", type=int, default=1)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--steps-per-epoch", type=int, default=0)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--lr", type=float, default=3e-3)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--heading-weight", type=float, default=1.0)
    ap.add_argument("--patience", type=int, default=12)
    ap.add_argument("--min-epochs", type=int, default=0)
    ap.add_argument("--max-minutes", type=float, default=0)
    ap.add_argument("--val")
    ap.add_argument("--test")
    ap.add_argument("--split")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-augment", action="store_true")
    ap.add_argument("--hue-p", type=float, default=0.5)
    ap.add_argument("--roof-p", type=float, default=0.0)
    ap.add_argument("--radius", default="8")
    ap.add_argument("--score-thresh", type=float, default=0.3)
    a = ap.parse_args(argv)
    if a.onnx:
        p, s, cfg = load_npz(a.onnx)
        out = a.out or os.path.splitext(a.onnx)[0] + ".onnx"
        print("Wrote", write_onnx(p, s, cfg, out))
        return 0
    if a.to_pt:
        print("Wrote", to_pt(a.to_pt, a.out))
        return 0
    if not (a.labels and a.out):
        ap.error("--labels and --out are needed to train")

    import jax
    import jax.numpy as jnp
    import optax
    from dataset import AugmentConfig, DetDataset, collate

    random.seed(a.seed)
    np.random.seed(a.seed)
    os.makedirs(a.out, exist_ok=True)
    in_w, in_h = (int(v) for v in a.size.lower().split("x"))
    cfg = default_config(classes=a.classes.split(",") if a.classes else None, in_ch=a.in_ch, in_w=in_w, in_h=in_h,
                         width=a.width, depth=a.depth)
    cfg["score_thresh"] = a.score_thresh
    if a.obj_size:
        for kv in a.obj_size.split(","):
            k, v = kv.split("=")
            cfg["obj_size_px"][k.strip()] = float(v)
    if a.heading_classes is not None:
        cfg["heading_classes"] = [c for c in a.heading_classes.split(",") if c]
    for c in cfg["classes"]:
        cfg["obj_size_px"].setdefault(c, 30.0)
    cfg["obj_size_px"] = {c: cfg["obj_size_px"][c] for c in cfg["classes"]}

    samples = detcore.read_many(a.labels, cfg["classes"], sizes=cfg["obj_size_px"])
    if a.split:
        assign = detcore.load_split(a.split)
    else:
        assign = detcore.make_split(samples, 0.15, 0.15, a.seed, a.val.split(",") if a.val else None,
                                    a.test.split(",") if a.test else None)
    detcore.save_split(assign, os.path.join(a.out, "split.json"))
    tr, va = detcore.select(samples, assign, "train"), detcore.select(samples, assign, "val")
    te = detcore.select(samples, assign, "test")
    for name, part in (("train", tr), ("val", va), ("test", te)):
        print("%-5s: %4d frames, %5d labels, %d recordings" % (name, len(part), sum(len(s["objects"]) for s in part),
                                                              len({s["recording"] for s in part})))
    aug = None if a.no_augment else AugmentConfig(hue_p=a.hue_p, roof_p=a.roof_p)
    ds_tr = DetDataset(tr, cfg, augment=aug)
    ds_va = DetDataset(va, cfg, augment=None, use_negatives=False)
    steps = a.steps_per_epoch or max(1, len(ds_tr) // a.batch)
    total = a.epochs * steps

    params, state = init_params(cfg, a.seed)
    n_par = sum(v.size for v in params.values())
    print("CentreNet width=%s depth=%s in=%dx%dx%d classes=%d: %d parameters" % (
        cfg["width"], cfg["depth"], in_w, in_h, cfg["in_ch"], len(cfg["classes"]), n_par))
    with open(os.path.join(a.out, "config.json"), "w") as f:
        json.dump(dict(model=cfg, args=vars(a), params=int(n_par), trainer=VERSION), f, indent=2)
    sched = optax.cosine_onecycle_schedule(total, a.lr, pct_start=0.1, div_factor=25.0, final_div_factor=1e4)
    opt = optax.chain(optax.clip_by_global_norm(10.0), optax.adamw(sched, b1=0.9, b2=0.999, eps=1e-8,
                                                                    weight_decay=a.weight_decay))
    params = {k: jnp.asarray(v) for k, v in params.items()}
    state = {k: jnp.asarray(v) for k, v in state.items()}
    opt_state = opt.init(params)
    fwd = make_forward(cfg)
    lossf = make_loss(a.heading_weight)
    keys = ("heat", "off", "off_w", "fheat", "fmask", "foff", "foff_w", "negw")

    @jax.jit
    def step(params, state, opt_state, x, b):
        def f(p):
            outs, new_st = fwd(p, state, x, True)
            l, parts = lossf(outs, b)
            return l, (parts, new_st)
        (l, (parts, new_st)), g = jax.value_and_grad(f, has_aux=True)(params)
        upd, opt_state = opt.update(g, opt_state, params)
        return optax.apply_updates(params, upd), new_st, opt_state, l, parts

    @jax.jit
    def evaluate(params, state, x, b):
        outs, _ = fwd(params, state, x, False)
        l, parts = lossf(outs, b)
        return outs, l, parts

    radius = detcore.parse_radius(a.radius)

    def batch_of(ds, idx):
        bt = collate([ds[i] for i in idx])
        return jnp.asarray(bt["image"]), {k: jnp.asarray(bt[k]) for k in keys}, bt

    def validate():
        scorer = detcore.Scorer(cfg["classes"], radius)
        tot, parts, n = 0.0, np.zeros(3), 0
        for k in range(0, len(ds_va), a.batch):
            idx = list(range(k, min(len(ds_va), k + a.batch)))
            x, b, bt = batch_of(ds_va, idx)
            (heat, off, hd), l, pp = evaluate(params, state, x, b)
            tot += float(l) * len(idx)
            parts += np.array([float(v) for v in pp]) * len(idx)
            n += len(idx)
            heat, off, hd = 1 / (1 + np.exp(-np.asarray(heat))), np.asarray(off), np.array(hd)
            hd[:, 0] = 1 / (1 + np.exp(-hd[:, 0]))
            for j, i in enumerate(bt["index"]):
                dets = detcore.decode(heat[j], off[j], hd[j], cfg, bt["scale"][j], a.score_thresh)
                scorer.add(dets, ds_va.samples[i]["objects"])
        rows = scorer.rows()
        return tot / max(1, n), parts / max(1, n), (dict(rows[-1]) if rows else {})

    cols = ["epoch", "seconds", "lr", "train_loss", "train_heat", "train_off", "train_heading", "val_loss", "val_heat",
            "val_off", "val_heading", "val_precision", "val_recall", "val_f1", "val_centre_err_px",
            "val_heading_err_deg", "best"]
    logf = open(os.path.join(a.out, "train_log.csv"), "w", newline="")
    log = csv.DictWriter(logf, fieldnames=cols)
    log.writeheader()
    best, best_epoch, t_start, it = (-1, -1.0), -1, time.time(), 0
    for epoch in range(1, a.epochs + 1):
        te0 = time.time()
        tl, tp, n = 0.0, np.zeros(3), 0
        order = [random.randrange(len(ds_tr)) for _ in range(steps * a.batch)] if a.steps_per_epoch else \
            random.sample(range(len(ds_tr)), steps * a.batch)
        for s_ in range(steps):
            x, b, _ = batch_of(ds_tr, order[s_ * a.batch:(s_ + 1) * a.batch])
            params, state, opt_state, l, pp = step(params, state, opt_state, x, b)
            it += 1
            tl += float(l) * a.batch
            tp += np.array([float(v) for v in pp]) * a.batch
            n += a.batch
        vl, vp, vrow = validate()
        f1 = vrow.get("f1", 0.0)
        err = vrow.get("centre_err_px_mean", float("nan"))
        herr = vrow.get("heading_err_deg_mean", float("nan"))
        score = f1 - 0.01 * (err if err == err else 10.0) - 0.001 * (herr if herr == herr else 180.0)
        key = (1, score) if f1 > 0 else (0, -vl)
        improved = key > best
        npp = {k: np.asarray(v) for k, v in params.items()}
        nst = {k: np.asarray(v) for k, v in state.items()}
        extra = dict(epoch=epoch, val={k: (float(v) if isinstance(v, (int, float, np.floating)) else v)
                                      for k, v in vrow.items() if k != "cls"})
        if improved:
            best, best_epoch = key, epoch
            save_npz(os.path.join(a.out, "best.npz"), npp, nst, cfg, extra)
        save_npz(os.path.join(a.out, "last.npz"), npp, nst, cfg, extra)
        row = dict(epoch=epoch, seconds=round(time.time() - te0, 1), lr="%.2e" % float(sched(it)),
                   train_loss=round(tl / n, 4), train_heat=round(tp[0] / n, 4), train_off=round(tp[1] / n, 4),
                   train_heading=round(tp[2] / n, 4), val_loss=round(vl, 4), val_heat=round(vp[0], 4),
                   val_off=round(vp[1], 4), val_heading=round(vp[2], 4),
                   val_precision=round(vrow.get("precision", 0), 4), val_recall=round(vrow.get("recall", 0), 4),
                   val_f1=round(f1, 4), val_centre_err_px=round(err, 3), val_heading_err_deg=round(herr, 2),
                   best=int(improved))
        log.writerow(row)
        logf.flush()
        print("epoch %3d %6.1fs loss %.3f val %.3f P %.3f R %.3f F1 %.3f err %.2fpx hd %.1fdeg%s" % (
            epoch, row["seconds"], row["train_loss"], vl, row["val_precision"], row["val_recall"], f1, err, herr,
            "  *" if improved else ""), flush=True)
        if best[0] == 1 and epoch >= a.min_epochs and epoch - best_epoch >= a.patience:
            print("No improvement for %d epochs: stopping early." % a.patience)
            break
        if a.max_minutes and (time.time() - t_start) / 60 > a.max_minutes:
            print("Time limit of %s min reached." % a.max_minutes)
            break
    logf.close()
    p, s, c = load_npz(os.path.join(a.out, "best.npz"))
    write_onnx(p, s, c, os.path.join(a.out, "best.onnx"))
    # the ONNX file against JAX on one random input (they must agree)
    import onnxruntime as ort
    xr = np.random.default_rng(0).normal(0, 1, (1, cfg["in_ch"], in_h, in_w)).astype(np.float32)
    (h, o, d), _ = fwd({k: jnp.asarray(v) for k, v in p.items()}, {k: jnp.asarray(v) for k, v in s.items()},
                       jnp.asarray(xr), False)
    d = np.array(d)
    d[:, 0] = 1 / (1 + np.exp(-d[:, 0]))
    ref = [1 / (1 + np.exp(-np.asarray(h))), np.asarray(o), d]
    got = ort.InferenceSession(os.path.join(a.out, "best.onnx"), providers=["CPUExecutionProvider"]).run(
        None, {"image": xr})
    diff = max(float(np.abs(r - g_).max()) for r, g_ in zip(ref, got))
    print("Best epoch %d (val score %.4f). Wrote best.npz and best.onnx (JAX vs ONNX Runtime max difference %.1e), "
          "total %.1f min" % (best_epoch, best[1], diff, (time.time() - t_start) / 60))
    return 0


if __name__ == "__main__":
    sys.exit(main())
