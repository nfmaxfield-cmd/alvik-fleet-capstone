"""Make a FAKE labels dataset, in the exact labels format, for testing the
detector pipeline before real labelled frames exist.

    python make_synthetic.py --out ~/synthetic --recordings 8 --frames 120

Each "recording" is a folder <out>/<name>/labels/ with labels.csv and
frames/<frame:06d>.jpg (see docs/data-formats.md). Inside a recording the
floor, tape grid and lighting stay the same and the robots drive smoothly,
like a real video; between recordings they change, so a split by recording
is a fair test.

What is drawn (640x480, overhead):
- floor: random grey/beige tone, fine texture, a lighting gradient, tape lines
- 1-3 robots (cls alvik): dark rounded squares about 30 px across, darker
  wheels on the left and right sides, two small light LEDs at the front
  (so the heading is visible). Heading is labelled.
- sometimes a hand (cls hand): a skin-coloured palm with fingers coming in
  from the edge of the picture. No heading.
- sometimes a round cup-like object (cls unknown). No heading.
- about 10% empty frames (one cls none row)
- a few rows left "pending" or "rejected" so the label filtering is exercised

The drawing is a guess at what the real scene looks like. It is only for
checking that the code runs end to end and learns something; numbers
measured on it say nothing about accuracy on the real camera.
"""
import argparse
import csv
import math
import os

import cv2
import numpy as np

W, H = 640, 480


def floor_texture(rng):
    base = rng.uniform(110, 200)
    tint = rng.uniform(-12, 12, size=3)
    img = np.ones((H, W, 3), np.float32) * (base + tint)
    noise = rng.normal(0, 1, (H // 4, W // 4)).astype(np.float32)
    noise = cv2.resize(cv2.GaussianBlur(noise, (0, 0), 2), (W, H)) * rng.uniform(4, 12)
    img += noise[:, :, None]
    img += rng.normal(0, 3, (H, W, 1)).astype(np.float32)
    # lighting: brighter in one direction
    gx, gy = np.meshgrid(np.linspace(-1, 1, W), np.linspace(-1, 1, H))
    ang = rng.uniform(0, 2 * math.pi)
    img += (np.cos(ang) * gx + np.sin(ang) * gy)[:, :, None] * rng.uniform(5, 25)
    # tape grid lines
    tapes = [(40, 40, 40), (230, 230, 230), (60, 160, 220), (50, 50, 200)]
    tape = tuple(float(c) for c in tapes[int(rng.integers(0, len(tapes)))])
    spacing = rng.integers(70, 110)
    off_x, off_y = rng.integers(0, spacing, 2)
    for x in range(off_x, W, spacing):
        cv2.line(img, (int(x), 0), (int(x), H), tape, int(rng.integers(3, 6)))
    for y in range(off_y, H, spacing):
        cv2.line(img, (0, int(y)), (W, int(y)), tape, int(rng.integers(3, 6)))
    return np.clip(img, 0, 255)


def rot(pts, cx, cy, h_deg):
    """Rotate points given in robot frame (x forward, y left) to image px."""
    r = math.radians(h_deg)
    fx, fy = math.cos(r), -math.sin(r)        # forward in image coordinates
    lx, ly = -math.sin(r), -math.cos(r)       # left in image coordinates
    return np.array([[cx + px * fx + py * lx, cy + px * fy + py * ly] for px, py in pts], np.float32)


def rounded_rect(half_len, half_wid, rad, n=6):
    pts = []
    corners = [(half_len - rad, half_wid - rad, 0), (-half_len + rad, half_wid - rad, 90),
               (-half_len + rad, -half_wid + rad, 180), (half_len - rad, -half_wid + rad, 270)]
    for cx, cy, a0 in corners:
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + rad * math.cos(a), cy + rad * math.sin(a)))
    return pts


def draw_robot(img, cx, cy, h, size, shade):
    s = size / 2
    # soft shadow
    sh = rot(rounded_rect(s * 1.05, s * 1.05, s * 0.35), cx + 2, cy + 2, h)
    over = img.copy()
    cv2.fillPoly(over, [sh.astype(np.int32)], (0, 0, 0), cv2.LINE_AA)
    img[:] = img * 0.75 + over * 0.25
    # wheels (left and right sides, slightly darker than the body)
    for side in (1, -1):
        wpts = [(s * 0.55, side * s * 0.75), (-s * 0.55, side * s * 0.75),
                (-s * 0.55, side * s * 1.02), (s * 0.55, side * s * 1.02)]
        cv2.fillPoly(img, [rot(wpts, cx, cy, h).astype(np.int32)], (float(shade * 0.5),) * 3, cv2.LINE_AA)
    body = rot(rounded_rect(s, s * 0.8, s * 0.3), cx, cy, h)
    cv2.fillPoly(img, [body.astype(np.int32)], (float(shade), float(shade), float(shade * 1.05)), cv2.LINE_AA)
    # a lighter top plate detail at the rear half, two LEDs at the front
    plate = rot([(-s * 0.1, s * 0.4), (-s * 0.7, s * 0.4), (-s * 0.7, -s * 0.4), (-s * 0.1, -s * 0.4)], cx, cy, h)
    cv2.fillPoly(img, [plate.astype(np.int32)], (float(shade + 15),) * 3, cv2.LINE_AA)
    led_col = (200, 240, 255)
    for side in (1, -1):
        p = rot([(s * 0.72, side * s * 0.38)], cx, cy, h)[0]
        cv2.circle(img, (int(round(p[0])), int(round(p[1]))), max(2, int(size * 0.07)), led_col, -1, cv2.LINE_AA)


def draw_hand(img, cx, cy, ang, size, rng):
    skin = np.array([rng.uniform(90, 140), rng.uniform(120, 170), rng.uniform(170, 225)])
    r = math.radians(ang)
    # palm
    cv2.ellipse(img, (int(cx), int(cy)), (int(size * 0.35), int(size * 0.28)), -ang, 0, 360, skin.tolist(), -1, cv2.LINE_AA)
    # arm going out of the picture (backwards)
    back = (int(cx - math.cos(r) * size * 3), int(cy + math.sin(r) * size * 3))
    cv2.line(img, (int(cx), int(cy)), back, (skin * 0.95).tolist(), int(size * 0.45), cv2.LINE_AA)
    # fingers forward
    for k in range(4):
        a = r + math.radians(-30 + 20 * k)
        tip = (int(cx + math.cos(a) * size * 0.65), int(cy - math.sin(a) * size * 0.65))
        cv2.line(img, (int(cx), int(cy)), tip, (skin * 1.02).tolist(), max(4, int(size * 0.1)), cv2.LINE_AA)


def draw_cup(img, cx, cy, size, col):
    cv2.circle(img, (int(cx), int(cy)), int(size / 2), col, -1, cv2.LINE_AA)
    cv2.circle(img, (int(cx), int(cy)), int(size / 3), [c * 0.7 for c in col], -1, cv2.LINE_AA)


def make_recording(folder, n_frames, rng, start_time):
    os.makedirs(os.path.join(folder, "frames"), exist_ok=True)
    floor = floor_texture(rng)
    n_robots = int(rng.integers(1, 4))
    shade = rng.uniform(25, 60)
    robots = []
    for i in range(n_robots):
        robots.append(dict(x=rng.uniform(60, W - 60), y=rng.uniform(60, H - 60), h=rng.uniform(0, 360),
                           v=rng.uniform(1, 4), w=rng.uniform(-4, 4), size=rng.uniform(27, 33), id=i))
    hand_t = int(rng.integers(0, n_frames)) if rng.random() < 0.8 else -999
    cup = dict(x=rng.uniform(60, W - 60), y=rng.uniform(60, H - 60), size=rng.uniform(25, 40),
               col=rng.uniform(30, 230, 3).tolist()) if rng.random() < 0.5 else None
    rows = []
    gain = rng.uniform(0.8, 1.2)
    for f in range(n_frames):
        img = floor.copy()
        objs = []
        empty = rng.random() < 0.1
        if not empty:
            if cup is not None:
                draw_cup(img, cup["x"], cup["y"], cup["size"], cup["col"])
                objs.append(dict(cls="unknown", x=cup["x"], y=cup["y"], h=None, obj=90))
            for r in robots:
                # drive: turn a little, move forward, bounce off the edges
                r["h"] = (r["h"] + r["w"] + rng.normal(0, 2)) % 360
                dx, dy = math.cos(math.radians(r["h"])), -math.sin(math.radians(r["h"]))
                r["x"] += dx * r["v"]
                r["y"] += dy * r["v"]
                if not (30 < r["x"] < W - 30 and 30 < r["y"] < H - 30):
                    r["h"] = (r["h"] + 180) % 360
                    r["x"] = float(np.clip(r["x"], 31, W - 31))
                    r["y"] = float(np.clip(r["y"], 31, H - 31))
                draw_robot(img, r["x"], r["y"], r["h"], r["size"], shade)
                objs.append(dict(cls="alvik", x=r["x"], y=r["y"], h=r["h"], obj=r["id"]))
            if abs(f - hand_t) < 15:
                # a hand comes in from the right-hand side and leaves again
                k = 1 - abs(f - hand_t) / 15
                hx, hy = W + 20 - k * 160, H * 0.5 + 60 * math.sin(hand_t)
                draw_hand(img, hx, hy, 180 + rng.normal(0, 5), 70, rng)
                if hx < W - 10:
                    objs.append(dict(cls="hand", x=hx, y=hy, h=None, obj=99))
        # camera: exposure, slight blur, noise, JPEG
        img = img * gain
        if rng.random() < 0.3:
            img = cv2.GaussianBlur(img, (0, 0), rng.uniform(0.5, 1.2))
        img = np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(folder, "frames", f"{f:06d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        t = start_time + f / 30.0
        if not objs:
            rows.append([f, f"{t:.4f}", "", "none", "", "", "", "0", "user", "confirmed", ""])
        for o in objs:
            status = "confirmed"
            u = rng.random()
            if u < 0.02:
                status = "pending"     # exercise the "skip half-reviewed frames" rule
            elif u < 0.12:
                status = "corrected"
            hd = "" if o["h"] is None else f"{o['h']:.1f}"
            rows.append([f, f"{t:.4f}", o["obj"], o["cls"], f"{o['x']:.2f}", f"{o['y']:.2f}", hd,
                         "0" if o["h"] is None else "1", "user", status, ""])
        if rng.random() < 0.03:   # a wrong automatic guess the user rejected
            rows.append([f, f"{t:.4f}", 77, "alvik", f"{rng.uniform(0, W):.1f}", f"{rng.uniform(0, H):.1f}",
                         "", "0", "motion", "rejected", "0.41"])
    with open(os.path.join(folder, "labels.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t_wall", "obj", "cls", "x_px", "y_px", "heading_deg", "heading_known",
                    "source", "status", "score"])
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--recordings", type=int, default=8)
    ap.add_argument("--frames", type=int, default=120, help="frames per recording")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    for i in range(a.recordings):
        name = f"20261001-{100000 + i * 1000}_synth{i}"
        make_recording(os.path.join(a.out, name, "labels"), a.frames, rng, 1.79e9 + i * 3600)
    print(f"Wrote {a.recordings} synthetic recordings x {a.frames} frames to {a.out}")


if __name__ == "__main__":
    main()
