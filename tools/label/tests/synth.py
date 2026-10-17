#!/usr/bin/env python3
"""Make synthetic camera recordings with known truth, for testing the
labelling tools without a robot.

The output folder looks exactly like one written by tools/pi/camrec.py
(video.avi as Motion-JPEG, frames.csv, events.csv, meta.json) plus truth.csv:

    frame,t,obj,cls,x,y,heading_deg,phase

phase says what the object was doing: drive, arc, turn, stop, reverse, park,
hand. Headings follow docs/data-formats.md (0 = image right, counter-clockwise
on screen positive).

Scenarios:
  main   3 robots (~30 px): one drives a square loop with turns on the spot,
         one stops for 6 s, later reverses and passes the loop robot 5 px
         apart; one is parked for the first 3 s then drives curves; a hand
         reaches in briefly; lighting flickers.
  crowd  3 robots: two start side by side 6 px apart and then split and
         pass each other; one is parked for the first 25 s (more than half
         the video, so the background swallows it: motion alone cannot see it
         until it moves).

    python tools/label/tests/synth.py <out folder> [--scenario main] [--seconds 40]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import cv2

W, H, FPS = 640, 480, 30.0
ROBOT = 30.0           # robot side, px
DARK = (42, 40, 38)    # robot body colour (BGR)


# ----------------------------------------------------------------------------
# Motion scripts: a robot follows a list of (kind, seconds, value...) steps
# ----------------------------------------------------------------------------

def run_script(x, y, h, script, seconds):
    """Pose at every frame. Returns list of (x, y, heading, phase)."""
    out = []
    steps = list(script)
    i, left = 0, steps[0][1] if steps else 1e9
    dt = 1.0 / FPS
    for f in range(int(round(seconds * FPS))):
        kind = steps[i][0] if i < len(steps) else "stop"
        val = steps[i][2:] if i < len(steps) else ()
        out.append((x, y, h % 360.0, kind))
        if kind in ("drive", "reverse"):
            v = val[0] * (-1 if kind == "reverse" else 1)
            x += v * dt * math.cos(math.radians(h))
            y -= v * dt * math.sin(math.radians(h))
        elif kind == "turn":
            h += val[0] * dt
        elif kind == "arc":
            x += val[0] * dt * math.cos(math.radians(h))
            y -= val[0] * dt * math.sin(math.radians(h))
            h += val[1] * dt
        left -= dt
        if i < len(steps) and left <= 1e-9:
            i += 1
            left += steps[i][1] if i < len(steps) else 1e9
    return out


def scenario(name, seconds):
    """Robots as (start x, y, heading, script)."""
    if name == "main":
        loop = []
        for _ in range(4):
            loop += [("drive", 5.0, 60.0), ("turn", 1.0, 90.0), ("drive", 3.5, 60.0), ("turn", 1.0, 90.0)]
        robots = [
            (120.0, 390.0, 0.0, loop),
            (560.0, 430.0, 90.0, [("drive", 4.0, 50.0), ("stop", 6.0), ("drive", 2.0, 50.0),
                                  ("reverse", 2.5, 40.0), ("stop", 1.0), ("turn", 1.0, 90.0),
                                  ("drive", 4.0, 50.0), ("turn", 1.0, 90.0), ("drive", 2.5, 50.0),
                                  ("turn", 1.0, 90.0), ("drive", 4.0, 50.0), ("stop", 60.0)]),
            (300.0, 280.0, 45.0, [("park", 3.0), ("arc", 6.0, 45.0, 60.0), ("drive", 1.5, 50.0),
                                  ("turn", 1.5, 120.0), ("drive", 2.5, 45.0), ("stop", 2.0),
                                  ("arc", 3.0, 40.0, -60.0), ("drive", 3.0, 40.0), ("turn", 1.0, -90.0),
                                  ("drive", 2.0, 40.0), ("stop", 60.0)]),
        ]
        hand = (20.0, 21.6, 330.0)   # starts, ends, x where it reaches in
    elif name == "crowd":
        robots = [
            (80.0, 200.0, 0.0, [("drive", 6.0, 55.0), ("turn", 1.0, 90.0), ("drive", 2.0, 50.0),
                                ("turn", 1.0, 90.0), ("drive", 6.0, 50.0), ("turn", 1.0, 90.0),
                                ("drive", 4.0, 50.0), ("turn", 1.0, 90.0), ("drive", 10.0, 40.0), ("stop", 60.0)]),
            (80.0, 236.0, 0.0, [("drive", 6.0, 55.0), ("turn", 1.0, -90.0), ("drive", 2.0, 50.0),
                                ("turn", 1.0, -90.0), ("drive", 5.0, 50.0), ("turn", 1.0, -90.0),
                                ("drive", 3.0, 50.0), ("turn", 1.0, -90.0), ("drive", 6.0, 45.0), ("stop", 60.0)]),
            (450.0, 380.0, 180.0, [("park", 25.0), ("drive", 4.0, 50.0), ("turn", 1.0, -90.0),
                                   ("drive", 6.0, 40.0), ("stop", 60.0)]),
        ]
        hand = None
    else:
        raise ValueError("unknown scenario " + name)
    return robots, hand


# ----------------------------------------------------------------------------
# Drawing
# ----------------------------------------------------------------------------

def make_floor(rng):
    """A plain but textured floor: smooth blotches, fine grain, a few stains and
    two strips of tape."""
    base = np.zeros((H, W, 3), np.float32)
    base[:] = (168, 174, 178)
    for sigma, amp in ((40, 10.0), (8, 6.0), (1.2, 4.0)):
        n = rng.normal(0, 1, (H, W)).astype(np.float32)
        n = cv2.GaussianBlur(n, (0, 0), sigma)
        n *= amp / max(1e-6, n.std())
        base += n[:, :, None]
    for _ in range(4):   # stains
        c = (int(rng.uniform(40, W - 40)), int(rng.uniform(40, H - 40)))
        m = np.zeros((H, W), np.float32)
        cv2.ellipse(m, c, (int(rng.uniform(10, 30)), int(rng.uniform(6, 20))), rng.uniform(0, 180), 0, 360, 1.0, -1)
        m = cv2.GaussianBlur(m, (0, 0), 3)
        base -= 18 * m[:, :, None]
    cv2.line(base, (0, 300), (W, 330), (150, 120, 70), 6)    # tape strips
    cv2.line(base, (470, 0), (450, H), (150, 120, 70), 6)
    return base


def rounded_square(cx, cy, h, side, r=6.0, n=6):
    """Outline points of a rounded square centred at (cx, cy) facing heading h."""
    s = side / 2.0 - r
    pts = []
    for (ux, vy, a0) in ((s, s, 0), (-s, s, 90), (-s, -s, 180), (s, -s, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90.0 * k / n)
            pts.append((ux + r * math.cos(a), vy + r * math.sin(a)))
    return [to_screen(cx, cy, h, u, v) for (u, v) in pts]


def to_screen(cx, cy, h, u, v):
    """Robot-local (u forward, v left) to image pixels."""
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    return (cx + u * c - v * s, cy - u * s - v * c)


def fill(img, pts, color):
    arr = np.array([[int(round(px * 16)), int(round(py * 16))] for (px, py) in pts], np.int32)
    cv2.fillPoly(img, [arr], color, lineType=cv2.LINE_AA, shift=4)


def draw_robot(img, x, y, h):
    fill(img, rounded_square(x, y, h, ROBOT), DARK)
    # Light bar at the front, so a person can see which way it faces.
    s = ROBOT / 2.0
    bar = [to_screen(x, y, h, s - 8, -s + 6), to_screen(x, y, h, s - 3, -s + 6),
           to_screen(x, y, h, s - 3, s - 6), to_screen(x, y, h, s - 8, s - 6)]
    fill(img, bar, (215, 215, 210))
    cv2.circle(img, (int(round(x)), int(round(y))), 3, (90, 90, 95), -1, cv2.LINE_AA)


def hand_shape(t, hand):
    """Arm + palm reaching up from the bottom edge. Returns (polygon of arm,
    palm centre, palm axes) or None when out of view."""
    t0, t1, hx = hand
    if not (t0 <= t <= t1):
        return None
    span = t1 - t0
    k = (t - t0) / span
    reach = min(1.0, k / 0.3) if k < 0.5 else min(1.0, (1 - k) / 0.3)   # in, hold, out
    tip_y = H + 40 - reach * 190.0
    palm = (hx + 25 * k, tip_y + 30)
    arm = [(palm[0] - 26, palm[1]), (palm[0] + 26, palm[1]), (palm[0] + 40, H + 10), (palm[0] - 10, H + 10)]
    return arm, palm, (34, 42)


# ----------------------------------------------------------------------------
# Writing a recording
# ----------------------------------------------------------------------------

def make_recording(out, name="main", seconds=40.0, seed=1, quality=75):
    """Render a scenario to folder `out`. Returns the path."""
    rng = np.random.default_rng(seed)
    os.makedirs(out, exist_ok=True)
    floor = make_floor(rng)
    robots, hand = scenario(name, seconds)
    poses = [run_script(x, y, h, s, seconds) for (x, y, h, s) in robots]
    n = len(poses[0])
    writer = cv2.VideoWriter(os.path.join(out, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)
    t_start = 1760000000.0
    truth_rows, frame_rows = [], []
    t_wall = t_start
    for f in range(n):
        t = f / FPS
        # Soft shadows first (darken the floor), then the robots on top.
        shadow = np.zeros((H, W), np.float32)
        for k, pose in enumerate(poses):
            x, y, h, phase = pose[f]
            fill(shadow, rounded_square(x + 2.0, y + 2.5, h, ROBOT + 2), 1.0)
        shadow = cv2.GaussianBlur(shadow, (0, 0), 2.0)
        img = floor * (1.0 - 0.15 * shadow[:, :, None])
        for k, pose in enumerate(poses):
            x, y, h, phase = pose[f]
            draw_robot(img, x, y, h)
            truth_rows.append([f, "%.4f" % t, k + 1, "alvik", "%.2f" % x, "%.2f" % y, "%.2f" % h, phase])
        hs = hand_shape(t, hand) if hand else None
        if hs is not None:
            arm, palm, axes = hs
            fill(img, arm, (120, 150, 196))
            cv2.ellipse(img, (int(palm[0]), int(palm[1])), axes, 10, 0, 360, (125, 156, 205), -1, cv2.LINE_AA)
            if palm[1] < H - 10:
                truth_rows.append([f, "%.4f" % t, 99, "hand", "%.2f" % palm[0], "%.2f" % palm[1], "", "hand"])
        # Lighting: slow wave, occasional one-frame dip, small jitter.
        gain = 1.0 + 0.05 * math.sin(2 * math.pi * 0.5 * t) + rng.normal(0, 0.01)
        if f % 97 == 50:
            gain -= 0.10
        img = img * np.float32(gain) + 2.5 * rng.standard_normal(img.shape, dtype=np.float32)
        frame = np.clip(img, 0, 255).astype(np.uint8)
        writer.write(frame)
        t_wall += 1.0 / FPS + rng.normal(0, 0.002)
        frame_rows.append([f, "%.4f" % t_wall, "%.4f" % (t_wall - t_start)])
    writer.release()
    with open(os.path.join(out, "frames.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t_wall", "t_mono"])
        w.writerows(frame_rows)
    with open(os.path.join(out, "events.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_wall", "frame", "text"])
        w.writerow([frame_rows[30][1], 30, "synthetic %s start" % name])
    with open(os.path.join(out, "meta.json"), "w") as fh:
        json.dump(dict(device="synthetic", width=W, height=H, fps_requested=30, fps_reported=FPS,
                       host="synth", start="2025-10-09T08:53:20", label="synth_" + name,
                       note="synthetic recording with truth.csv"), fh, indent=2)
    with open(os.path.join(out, "truth.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t", "obj", "cls", "x", "y", "heading_deg", "phase"])
        w.writerows(truth_rows)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--scenario", default="main", choices=["main", "crowd"])
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()
    make_recording(args.out, args.scenario, args.seconds, args.seed)
    print("wrote", args.out)


if __name__ == "__main__":
    sys.exit(main())
