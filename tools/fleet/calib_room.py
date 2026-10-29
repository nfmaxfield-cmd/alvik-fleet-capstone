#!/usr/bin/env python3
"""Calibration room (2026-10-29, Nick: "I want the robots to be able to calibrate with less space"; he chose "point the
roll at open space + skip").

What the calibration does (firmware/home_lab/home_lab.ino, calibSteps): it spins on the spot (720 deg each way), then
rolls forward up to calib_roll_cm (15) and back the same way. The spin is the only part that needs room all round; the
roll needs room in ONE direction, and its own front-sensor look cannot see a plywood edge. So:

  room     a robot may calibrate where it stands when its spin circle fits: ROOM_EDGE_CM from every edge and
           ROOM_ROBOT_CM from every other robot's middle (was 30 cm and 35 cm: the whole roll had to fit every way).
  caps     before each calibration the Pi works out, from the camera's position of the robot, the board's rectangle and
           the other robots' positions, how far the robot may roll in each of 8 directions round it, and sends them
           (`set calib_dir_limits <deg> <cm> ...`, firmware look.h: each caps the roll for headings within 22.5 deg of
           <deg>, relative to where the robot faces, left positive; the next calibration only). The robot then rolls
           towards open space; where no direction has calib_roll_min_cm (5) it skips the roll ("limited", which the
           firmware and session_checks already treat as calibrated). With the robot's heading not known (the first
           calibration after a power-on: it cannot crawl before its compass check), every direction gets the roll
           that is safe whichever way it faces.
  keep     a robot calibrated today on its current power-on (same boot ID) keeps that calibration: the firmware still
           has its compass check, and the stored corrections are sent on every connect. A full calibration runs when it
           restarted (new boot ID), when it has no calibration today, when forced, or when the quick check after a kept
           calibration (the first turn check) fails. The record is engine_checks.DayState (~/.config/mirlt/day/
           <date>.json: calibrated_boot, calibrated_t), shared with run_plan.
"""
import math
import os
import time

# ---------------------------------------------------------------------------------------------- the numbers
BODY_HALF_CM = 4.75     # the body is 9.5 x 9.5 cm (tools/sim/config.py length_cm / width_cm), axle under its middle
SWEEP_R_CM = round(math.hypot(BODY_HALF_CM, BODY_HALF_CM), 2)   # 6.72: the corners' circle when it spins on the spot
CAM_ERR_CM = 2.0        # chosen: the camera's spot of a standing robot against its turning centre (still spots wobble
                        # 0.5-1.5 cm, ladder.still_spot; near an edge the spot can ride towards the nose). DOE note: yes,
                        # from the calibration runs themselves: the camera's spin-circle centre (tools/blink/calib.py)
                        # against the still spot before the spin, per robot, near and away from the edges
MARGIN_CM = 3.0         # chosen: kept free beyond that (as the firmware's calib_margin_cm). No DOE: a safety room
ROOM_EDGE_CM = float(math.ceil(SWEEP_R_CM + CAM_ERR_CM + MARGIN_CM))                 # 6.72 + 2 + 3 = 11.7 -> 12 cm
ROOM_ROBOT_CM = float(math.ceil(2 * SWEEP_R_CM + 2 * CAM_ERR_CM + MARGIN_CM))        # 13.4 + 4 + 3 = 20.4 -> 21 cm
#   (both robots may be spinning at once, as run_plan calibrates them together; each camera spot can be 2 cm off)
ROLL_CM = 15.0          # the firmware's calib_roll_cm (params.h)
ROLL_MIN_CM = 5.0       # the firmware's calib_roll_min_cm: a shorter roll is skipped ("limited")
CAP_DEGS = (0, 45, 90, 135, 180, -135, -90, -45)    # 8 caps (the firmware's most), each covering +-22.5 deg: all round
CAP_HALF_DEG = 22.5     # look.h CAP_HALF
STEP_DEG = 3.0          # directions tried inside a cap's window (look.h's own bins are 3 deg)
HEADING_UNC_DEG = 15.0  # chosen: a heading the camera measured (a crawl, a drive's line: ~1-7 deg spread) plus the
                        # robot's turn back after its compass wiggle ("within a few degrees"). DOE note: yes, the same
                        # calibration runs: the roll heading the camera saw against the one the caps assumed
HEADING_UNC_LOOSE_DEG = 45.0   # a heading from a weaker source (h_how naming "lights" or "facing", or a plan's start
                               # heading nobody checked). No DOE until that source's spread is measured


# ---------------------------------------------------------------------------------------------- geometry
def edge_dist(rect, x, y):
    return min(x - rect[0], rect[2] - x, y - rect[1], rect[3] - y)


def spin_room(p, rect, others):
    """(True, "") when the spin circle fits where p stands; else (False, plain words). others: {name: (x, y)}."""
    ed = edge_dist(rect, *p)
    if ed < ROOM_EDGE_CM:
        return False, "its middle is %.0f cm from the nearest edge (the spin needs %.0f)" % (ed, ROOM_EDGE_CM)
    close = sorted((math.dist(p, q), m) for m, q in others.items() if q is not None and math.dist(p, q) < ROOM_ROBOT_CM)
    if close:
        return False, "%s is %.0f cm away, middle to middle (the spin needs %.0f)" % (close[0][1], close[0][0],
                                                                                        ROOM_ROBOT_CM)
    return True, ""


def _swept_corners(p, th, L):
    """The swept body's 4 corners for a roll of L cm along world heading th (deg): back corners where it stands, front
    corners L cm on. The swept area is the rectangle they span."""
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    out = []
    for f in (-BODY_HALF_CM, L + BODY_HALF_CM):
        for lat in (-BODY_HALF_CM, BODY_HALF_CM):
            out.append((p[0] + f * c - lat * s, p[1] + f * s + lat * c))
    return out


def _dist_to_swept(q, p, th, L):
    """Distance from point q to the swept rectangle (0 inside)."""
    c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
    dx, dy = q[0] - p[0], q[1] - p[1]
    f, lat = dx * c + dy * s, -dx * s + dy * c
    df = max(-BODY_HALF_CM - f, 0.0, f - (L + BODY_HALF_CM))
    dl = max(abs(lat) - BODY_HALF_CM, 0.0)
    return math.hypot(df, dl)


def roll_fits(p, th, L, rect, fixed=(), moving=()):
    """Does a roll of L cm along world heading th fit? fixed: other robots' middles that stay put (a circle of
    SWEEP_R_CM, any heading); moving: robots calibrating at the same time (each keeps to its own side of the line half
    way between the two)."""
    inset = MARGIN_CM + CAM_ERR_CM
    for x, y in _swept_corners(p, th, L):
        if x < rect[0] + inset or x > rect[2] - inset or y < rect[1] + inset or y > rect[3] - inset:
            return False
    need = SWEEP_R_CM + MARGIN_CM + 2 * CAM_ERR_CM
    for q in fixed:
        if _dist_to_swept(q, p, th, L) < need:
            return False
    for q in moving:
        d = math.dist(p, q)
        if d < 1e-6:
            return False
        vx, vy = (q[0] - p[0]) / d, (q[1] - p[1]) / d
        lim = d / 2.0 - (MARGIN_CM / 2.0 + CAM_ERR_CM)
        if any((x - p[0]) * vx + (y - p[1]) * vy > lim for x, y in _swept_corners(p, th, L)):
            return False
    return True


def max_roll(p, th, rect, fixed=(), moving=(), step=0.1):
    """The longest roll (cm, 0..ROLL_CM) along world heading th that fits (0 when not even a short one does)."""
    if roll_fits(p, th, ROLL_CM, rect, fixed, moving):
        return ROLL_CM
    lo, hi = 0.0, ROLL_CM
    if not roll_fits(p, th, 0.0, rect, fixed, moving):
        return 0.0
    while hi - lo > step:           # the rolls that fit form one interval from 0 (a convex board, convex keep-outs)
        mid = (lo + hi) / 2.0
        if roll_fits(p, th, mid, rect, fixed, moving):
            lo = mid
        else:
            hi = mid
    return round(lo, 1)


def caps(p, h, rect, fixed=(), moving=(), unc=HEADING_UNC_DEG):
    """[(relative deg, cm)] for `set calib_dir_limits`: per cap direction, the shortest max_roll over every world
    heading the robot could roll at inside that cap's window, widened by the heading's uncertainty unc. h: the robot's
    world heading (deg, camera frame: a left turn adds) or None (unknown: every cap is the roll safe all round)."""
    if h is None:
        unc = 180.0
    allround = None
    out = []
    for c in CAP_DEGS:
        half = min(180.0, CAP_HALF_DEG + unc)
        n = max(1, int(math.ceil(2 * half / STEP_DEG)))
        if half >= 180.0:
            if allround is None:
                allround = min(max_roll(p, k * STEP_DEG, rect, fixed, moving) for k in range(int(360 / STEP_DEG)))
            out.append((c, allround))
            continue
        worst = ROLL_CM
        for k in range(n + 1):
            a = (h or 0.0) + c - half + k * (2 * half / n)
            worst = min(worst, max_roll(p, a, rect, fixed, moving))
        out.append((c, worst))
    return out


def caps_cmd(cp):
    """The firmware command for caps cp."""
    return "set calib_dir_limits " + " ".join("%d %.1f" % (d, max(0.0, cm)) for d, cm in cp)


def best_world_heading(p, h, rect, fixed=(), moving=(), unc=HEADING_UNC_DEG):
    """Where the robot will roll, roughly (for the screen): the cap with the longest roll (ties: nearest straight
    ahead) and its length; (None, 0) when no cap reaches ROLL_MIN_CM."""
    cp = caps(p, h, rect, fixed, moving, unc)
    best = max(cp, key=lambda dc: (dc[1], -abs(dc[0])))
    if best[1] < ROLL_MIN_CM:
        return None, 0.0
    return best[0], best[1]


def compass_words(world_deg):
    """'towards +x', 'towards -y +x' ...: plain words for a world direction on the board."""
    a = (world_deg + 360.0) % 360.0
    names = ["+x", "+x +y", "+y", "-x +y", "-x", "-x -y", "-y", "+x -y"]
    return "towards " + names[int((a + 22.5) // 45) % 8]


def caps_words(cp, h, unc):
    """One line for the screen: what the caps allow."""
    best = max(cp, key=lambda dc: (dc[1], -abs(dc[0])))
    if best[1] < ROLL_MIN_CM:
        return ("no direction has room for a %.0f cm roll: it will skip the roll (calibrated, 'limited': the spin "
                "still measures its gyro)" % ROLL_MIN_CM)
    full = [d for d, cm in cp if cm >= ROLL_CM - 0.05]
    if h is None:
        return ("its heading is not known yet, so every direction gets the roll that is safe whichever way it faces: "
                "%.1f cm" % best[1])
    where = compass_words(h + best[0])
    return ("rolls allowed: %s (deg from where it faces : cm); the longest, %.1f cm, %s on the board%s" % (
        ", ".join("%+d:%.0f" % (d, cm) for d, cm in cp), best[1], where,
        "" if full else "; no direction has the full %.0f cm" % ROLL_CM))


# ---------------------------------------------------------------------------------------------- keep or calibrate
def day_state(dry=False, path=None):
    """engine_checks.DayState: the real record (~/.config/mirlt/day/<date>.json), or for dry runs a separate one
    (~/.config/mirlt/day-dry/<date>.json) so the simulator's boot IDs never overwrite the real robots' record."""
    import engine_checks as EC
    if not path and dry:
        path = os.path.join(EC.conf_dir(), "day-dry", EC.today() + ".json")
    return EC.DayState(path)


def decide(name, boot, day, force=False):
    """("full" | "keep", plain words) for one robot about to calibrate."""
    r = day.robot(name)
    prev, t = r.get("calibrated_boot"), r.get("calibrated_t")
    when = time.strftime("%H:%M", time.localtime(t)) if t else "?"
    if force:
        return "full", "a full calibration was asked for"
    if not boot:
        return "full", "it did not say its boot ID, so a restart cannot be ruled out: full calibration"
    if not prev:
        return "full", "not calibrated yet today: full calibration (about 65 s)"
    if prev != boot:
        return "full", ("restarted since its calibration at %s (boot %s, now %s): it lost its compass check, full "
                        "calibration (about 65 s)" % (when, prev, boot))
    return "keep", ("calibrated today at %s on this power-on (boot %s): kept, no 65 s calibration. Its stored "
                    "corrections were sent on connect; its first turn check is the quick check (a FAIL there "
                    "calibrates it fully)" % (when, boot))
