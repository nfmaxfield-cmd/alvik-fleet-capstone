#!/usr/bin/env python3
"""A person shows the run where a held robot is: one click on it in any camera's picture (build e13; Nick 12:56, CLAUDE.md
"never give up").

When a robot holds because the sum of its possible error could reach an edge (freeroam_core.blind_stop), the run asks
(request(): a file in the questions folder, safety_rules.questions_dir(); the terminal prints Mission Control's address).
Mission Control shows a card with every camera's live picture, the robot's name and its last known spot drawn on each
(e11's webcam clicks, reused); the person clicks the robot (optionally a second click on its nose). click() projects the
click onto the table at the roof tag's height through that camera's model and answers with:

    x, y        the robot's middle
    sigma_cm    the click's own accuracy, DERIVED at that spot: one pixel's worth of table there (the camera model's cm
                per px, the click's quantisation) combined with the person's aim, taken as the spread of a click that
                lands anywhere on the robot's roof (a uniform spread over the roof's side: side / sqrt(12) each way);
                once CLICKS_TO_LEARN clicks with a camera residual are logged, the aim is their own 95th percentile
                (measured, learn_aim())
    heading     (Nick 13:01: the Alvik is a cube, no nose to click) from TWO clicks: after the first (position, error sum
                reset) the robot drives straight forward a short way, as far as its fresh error sum + body stay inside
                the edge rule (the run derives it: freeroam_core.heading_drive_cm), and stops; the person clicks it again
                on a picture taken after the drive; the line between the clicks is its heading, +- atan(sigma x sqrt 2
                / drive length) (1 cm clicks over 10 cm: about 8 deg; over 25 cm: about 3 deg). When the motion detector
                or lock-on saw the drive, its spot is offered as the second click for the person to confirm (suggest)

The run (freeroam.py) reads the answer and sets the robot's position: its error sum restarts at sigma_cm, so it moves
again with a new running total. If no camera shows the robot, the person puts it on a known point and clicks that point
on the map (point()). Every click is logged (clicks.csv beside the requests: who, when, camera, pixel, table spot,
sigma, and later the residual against the next camera fix, residual()) so the click accuracy can be measured.
"""
import csv
import json
import math
import os
import time

VERSION = "place-click-2026-11-10e13"
CLICKS_TO_LEARN = 10       # clicks with a residual before the logged aim replaces the derived one. Not a tuning number:
                           # enough for a 95th percentile to mean something (the DOE row asks for 30+)
ROOF_CM = 9.5              # the roof's side (the body's), what a person aims at. Not chosen
STILL_CM = 2.0             # a residual measures the click only if the robot drove under this since (its own wheels):
                           # the camera's ~1 cm and the stop overrun (1.4-2.4 cm, measured) are of this size. Not tuned


def qdir():
    import safety_rules as SR
    d = SR.questions_dir()
    os.makedirs(d, exist_ok=True)
    return d


def _path(robot):
    return os.path.join(qdir(), "place-%s.json" % robot)


def request(robot, last_xy, run="", why="", stage="position", first=None, drive_cm=None, sigma_cm=None):
    """The run asks for a click on this robot (last_xy: its last known spot, cm). stage "position" (the first click) or
    "heading" (the second, after its straight drive from first over drive_cm). Returns the request."""
    rq = {"robot": robot, "last": [round(float(v), 1) for v in last_xy] if last_xy else None, "run": run, "why": why,
          "t": time.time(), "answer": None, "stage": stage, "suggest": None,
          "first": [round(float(v), 2) for v in first] if first else None,
          "drive_cm": None if drive_cm is None else round(float(drive_cm), 1),
          "sigma_cm": None if sigma_cm is None else round(float(sigma_cm), 2)}
    with open(_path(robot), "w") as f:
        json.dump(rq, f)
    return rq


def open_requests():
    out = []
    for fn in sorted(os.listdir(qdir())):
        if fn.startswith("place-") and fn.endswith(".json"):
            try:
                with open(os.path.join(qdir(), fn)) as f:
                    d = json.load(f)
                if d.get("answer") is None:
                    out.append(d)
            except (OSError, ValueError):
                continue
    return out


def take_answer(robot):
    """The run: the answer once given (then the request is closed), else None."""
    try:
        with open(_path(robot)) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    a = d.get("answer")
    if a is None:
        return None
    try:
        os.replace(_path(robot), _path(robot) + ".done")
    except OSError:
        pass
    return a


def cancel(robot):
    try:
        os.replace(_path(robot), _path(robot) + ".cancelled")
    except OSError:
        pass


def _log(row):
    p = os.path.join(qdir(), "clicks.csv")
    new = not os.path.exists(p)
    with open(p, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["t", "who", "robot", "how", "cam", "u", "v", "nose_u", "nose_v", "x_cm", "y_cm", "heading_deg",
                        "sigma_cm", "px_cm", "aim_cm", "residual_cm", "residual_t", "driven_cm"])
        w.writerow(row)


def learn_aim():
    """The person's aim (cm) from the logged clicks' residuals (their 95th percentile, less the pixel's share), once
    CLICKS_TO_LEARN have one; else None."""
    p = os.path.join(qdir(), "clicks.csv")
    if not os.path.exists(p):
        return None
    res = []
    with open(p, newline="") as f:
        for r in csv.DictReader(f):
            try:
                # only clicks the robot had not driven away from (its own wheels: under STILL_CM) measure the aim; a
                # residual after driving blind holds its dead reckoning's error too
                if r.get("residual_cm") and r.get("driven_cm") not in (None, "") and float(r["driven_cm"]) <= STILL_CM:
                    res.append(float(r["residual_cm"]))
            except ValueError:
                continue
    if len(res) < CLICKS_TO_LEARN:
        return None
    res.sort()
    return res[min(len(res) - 1, int(0.95 * len(res)))]


def sigma_for(model, x, y, z):
    """(sigma_cm, px_cm, aim_cm) for a click at table spot (x, y): one pixel's worth there, and the aim (logged, else a
    click anywhere on the roof: side / sqrt(12) each way, both axes: x sqrt(2)); combined as independent errors."""
    ppc = float(model.px_per_cm(x, y, z))
    px_cm = 1.0 / max(ppc, 1e-6)
    aim = learn_aim()
    if aim is None:
        aim = ROOF_CM / math.sqrt(12.0) * math.sqrt(2.0)
    return math.hypot(px_cm, aim), px_cm, aim


def heading_pm(sigma_cm, drive_cm):
    """The heading's +- (deg) from two clicks drive_cm apart, each +- sigma_cm: atan(sigma x sqrt 2 / drive)."""
    return math.degrees(math.atan(float(sigma_cm) * math.sqrt(2.0) / max(1e-6, float(drive_cm))))


def _stage(robot):
    try:
        with open(_path(robot)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _with_heading(robot, ans):
    """A second click: the heading from the first click to this one, and its +-."""
    rq = _stage(robot)
    if rq.get("stage") != "heading" or not rq.get("first"):
        return ans
    fx, fy = rq["first"]
    d = math.hypot(ans["x"] - fx, ans["y"] - fy)
    sig = max(float(ans["sigma_cm"]), float(rq.get("sigma_cm") or 0.0))
    if d < 1.0:
        ans.update(heading=None, heading_pm=None, stage="heading", drive_seen_cm=round(d, 2))
        return ans
    ans.update(heading=round(math.degrees(math.atan2(ans["y"] - fy, ans["x"] - fx)), 1),
               heading_pm=round(heading_pm(sig, d), 1), stage="heading", drive_seen_cm=round(d, 2))
    return ans


def click(robot, cam, model, u, v, z, who=""):
    """A click on robot in camera cam's picture (u, v px): projected to the table at height z, the answer written for
    the run (a second click after the heading drive also gives the heading), logged. Returns the answer."""
    x, y = model.to_floor(float(u), float(v), float(z))
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("that click is not on the table in camera %s's picture" % cam)
    sig, px_cm, aim = sigma_for(model, x, y, z)
    ans = {"x": round(x, 2), "y": round(y, 2), "heading": None, "sigma_cm": round(sig, 2), "how": "click", "cam": cam,
           "t": time.time(), "who": who, "stage": _stage(robot).get("stage", "position")}
    ans = _with_heading(robot, ans)
    _answer(robot, ans)
    _log([round(ans["t"], 3), who, robot, "click" if ans["stage"] == "position" else "click 2 (heading)", cam,
          round(float(u), 1), round(float(v), 1), "", "", ans["x"], ans["y"],
          "" if ans.get("heading") is None else ans["heading"], ans["sigma_cm"], round(px_cm, 3), round(aim, 2),
          "", "", ""])
    return ans


def set_suggest(robot, x, y, src, sigma_cm):
    """The run: the motion detector / lock-on saw the heading drive's end: offered as the second click."""
    rq = _stage(robot)
    if not rq or rq.get("answer") is not None:
        return
    rq["suggest"] = {"x": round(float(x), 2), "y": round(float(y), 2), "src": src, "sigma_cm": round(float(sigma_cm), 2)}
    with open(_path(robot), "w") as f:
        json.dump(rq, f)


def accept_suggest(robot, who=""):
    """The person confirmed the offered spot as the second click."""
    rq = _stage(robot)
    sg = rq.get("suggest")
    if not sg:
        raise ValueError("nothing was offered for %s" % robot)
    ans = {"x": sg["x"], "y": sg["y"], "heading": None, "sigma_cm": sg["sigma_cm"], "how": "confirmed " + sg["src"],
           "cam": "", "t": time.time(), "who": who, "stage": rq.get("stage", "position")}
    ans = _with_heading(robot, ans)
    _answer(robot, ans)
    _log([round(ans["t"], 3), who, robot, "confirmed %s" % sg["src"], "", "", "", "", "", ans["x"], ans["y"],
          "" if ans.get("heading") is None else ans["heading"], ans["sigma_cm"], "", "", "", "", ""])
    return ans


def point(robot, label, xy, who="", sigma_cm=None):
    """No camera shows it: the person put it on a known point (label at xy) and clicked that point on the map. Its
    accuracy: how well a robot is set on a sticker by hand, the roof's spread as for a click (no pixel term)."""
    sig = sigma_cm if sigma_cm is not None else (learn_aim() or ROOF_CM / math.sqrt(12.0) * math.sqrt(2.0))
    ans = {"x": round(float(xy[0]), 2), "y": round(float(xy[1]), 2), "heading": None, "sigma_cm": round(sig, 2),
           "how": "point %s" % label, "cam": "", "t": time.time(), "who": who}
    _answer(robot, ans)
    _log([round(ans["t"], 3), who, robot, "point %s" % label, "", "", "", "", "", ans["x"], ans["y"], "",
          ans["sigma_cm"], "", round(sig, 2), "", "", ""])
    return ans


def _answer(robot, ans):
    try:
        with open(_path(robot)) as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {"robot": robot}
    d["answer"] = ans
    with open(_path(robot), "w") as f:
        json.dump(d, f)


def residual(robot, t_click, x_click, y_click, x_cam, y_cam, t_cam, driven_cm=None, res_cm=None):
    """The next camera fix after a click: the distance between them (res_cm: given by the run, its own driving since the
    click taken out), logged with how far it drove meanwhile (the click accuracy's measure: learn_aim)."""
    r = math.hypot(x_cam - x_click, y_cam - y_click) if res_cm is None else float(res_cm)
    _log([round(time.time(), 3), "", robot, "residual", "", "", "", "", "", round(x_cam, 2), round(y_cam, 2), "", "",
          "", "", round(r, 2), round(t_cam - t_click, 2), "" if driven_cm is None else round(driven_cm, 2)])
    return r
