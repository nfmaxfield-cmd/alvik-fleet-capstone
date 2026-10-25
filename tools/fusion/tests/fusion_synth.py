"""Small synthetic runs for the fusion tests (imported by test_fusion_identity.py)."""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION = os.path.dirname(HERE)
for p in (FUSION, os.path.dirname(FUSION)):
    if p not in sys.path:
        sys.path.insert(0, p)

import runs as rn  # noqa: E402
import raw_units  # noqa: E402

D2R = math.pi / 180


def straight_run(run_id, robot, x0, y0, h_deg, v=10.0, t0=1000.0, still_s=1.0, drive_s=6.0, hz=100.0,
                 wheel_r=1.7, gyro_bias_dps=0.0, turn_dps=0.0):
    """A robot standing still `still_s`, then driving at v cm/s (turning turn_dps)
    for drive_s, then still for 1 s. Returns (Run, truth [(t, x, y, th_rad)])."""
    run = rn.Run(run_id, robot)
    run.tape_start = (x0, y0, h_deg)
    x, y, th = x0, y0, h_deg * D2R
    truth = []
    n = int((still_s + drive_s + 1.0) * hz)
    rpm_per_cms = 60 / (2 * math.pi * wheel_r)
    for i in range(n):
        t = t0 + i / hz
        moving = still_s <= i / hz < still_s + drive_s
        vv = v if moving else 0.0
        w = turn_dps if moving else 0.0
        half = w * D2R * 4.5            # wheel speed difference for a 9 cm track
        # logged as a robot logs them (tools/raw_units.py: v_cms in mm/s, speeds 1.08 high)
        sp = raw_units.robot_columns((vv - half) * rpm_per_cms, (vv + half) * rpm_per_cms, wheel_r, 9.0)
        run.raw.append(dict(t=t, gz=w + gyro_bias_dps, lifted=0, **sp))
        truth.append((t, x, y, th))
        x += vv * math.cos(th) / hz
        y += vv * math.sin(th) / hz
        th += w * D2R / hz
    return run, truth


def sightings(truth, fps=30.0, noise=0.3, off=None, frame0=0, seed=1, every=1, **extra):
    """Camera sightings of a truth path at fps: the centre, or the light at `off`
    (forward, left). Frame numbers follow the time, so robots seen at the same
    moment share frames."""
    rng = random.Random(seed)
    out = []
    t0, t1 = truth[0][0], truth[-1][0]
    k = 0
    while True:
        t = t0 + k / fps
        if t > t1:
            break
        j = min(range(len(truth)), key=lambda i: abs(truth[i][0] - t)) if len(truth) < 50 else int(
            round((t - t0) / (truth[1][0] - truth[0][0])))
        j = min(j, len(truth) - 1)
        _, x, y, th = truth[j]
        if off is not None:
            x += math.cos(th) * off[0] - math.sin(th) * off[1]
            y += math.sin(th) * off[0] + math.cos(th) * off[1]
        if k % every == 0:
            out.append(dict(t=round(t, 4), frame=frame0 + k, x=x + rng.gauss(0, noise), y=y + rng.gauss(0, noise),
                            score=1.0, heading=None, **extra))
        k += 1
    return out


def blink_rows(truth, robot, p=1.0, names=("A", "B"), track=1, off=(2.75, 0.0), **kw):
    probs = {n: (p if n == robot else (1 - p) / (len(names) - 1 + 1)) for n in names}
    p_none = 1.0 - sum(probs.values())
    return sightings(truth, off=off, track=track, robot=robot if p >= 0.6 else "", probs=probs, p_none=max(p_none, 0.0),
                     n_spots=2, **kw)
