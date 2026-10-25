"""Units of the RAW record's speed columns, in one place (docs/data-formats.md, "RAW speed columns").

The robots log what the Arduino_Alvik library returns, and two of those numbers are not what their names say
(docs/build-reports/e02-oct25-right-front-and-rpm.md):

- `v_cms` is in **mm/s**, not cm/s. Arduino_Alvik 1.1.1 `get_drive_speed` passes `'MM_S'` (a character
  constant, quotes included) as the source unit, so the library's mm/s value goes through unconverted.
  The column keeps its name so old logs keep their meaning.
- `wl_rpm`, `wr_rpm` (and `v_cms`, `w_dps`, which the motor board works out from the same numbers) read
  **1.08x** the true speed: the motor board divides encoder counts per update by an assumed 20 ms, but its
  loop runs about 21.6 ms. Measured 1.080 / 1.085 / 1.081 (Left / Center / Right, 212 runs, spread 0.004),
  so it is a measured number, not a chosen one (no DOE). The wheel angles (`wl_deg`, `wr_deg`) and the
  library pose (`x_cm`, `y_cm`, `th_deg`) count encoder steps with no time in them and are right.

Readers turn a column into a physical speed with the functions below; writers of synthetic RAW rows (the
simulator, test fixtures) use `robot_columns` so they log exactly what a real robot would.

Simulator logs written before this module (2026-10-25) kept `v_cms` in cm/s and the speeds without the 1.08.
`legacy_sim_folder` finds them (a simulator marker with no `raw_units` key) and `to_robot_units` rescales
their rows to the robots' units, so every reader can assume robot units.
"""
import json
import math
import os

V_COL_PER_CMS = 10.0          # v_cms column units per cm/s: the column holds mm/s
SPEED_REPORT_FACTOR = 1.08    # rpm, v_cms, w_dps read this much above the true speed (measured, see above)
RAW_UNITS = "robot-2026-10-25"   # the marker the simulator writes: its RAW speeds are in the robots' units
SPEED_COLS = ("wl_rpm", "wr_rpm", "v_cms", "w_dps")


def _num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return float("nan")
    return v


def speed_cms(v_col):
    """True forward speed, cm/s, from the RAW `v_cms` column (mm/s, 1.08 high)."""
    return _num(v_col) / (V_COL_PER_CMS * SPEED_REPORT_FACTOR)


def wheel_rpm(rpm_col):
    """True wheel rpm (output shaft) from a RAW `wl_rpm` / `wr_rpm` value (1.08 high)."""
    return _num(rpm_col) / SPEED_REPORT_FACTOR


def turn_dps(w_col):
    """True turn rate from the wheels, deg/s, from the RAW `w_dps` column (1.08 high; the wheel track
    and wheel scrub in turns are not corrected here)."""
    return _num(w_col) / SPEED_REPORT_FACTOR


def robot_columns(true_rpm_l, true_rpm_r, wheel_radius_cm=1.7, wheel_track_cm=8.9):
    """What a robot logs for these true wheel speeds (rpm): dict wl_rpm, wr_rpm, v_cms, w_dps.
    The library's own conversion with nominal sizes; v_cms in mm/s; all 1.08 high."""
    f = SPEED_REPORT_FACTOR
    l, r = true_rpm_l * f, true_rpm_r * f
    vl = l * 2 * math.pi * wheel_radius_cm / 60
    vr = r * 2 * math.pi * wheel_radius_cm / 60
    return dict(wl_rpm=l, wr_rpm=r, v_cms=(vl + vr) / 2 * V_COL_PER_CMS,
                w_dps=math.degrees((vr - vl) / wheel_track_cm))


def _json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def legacy_sim_folder(folder):
    """True if `folder` (a console log folder holding raw.csv) was written by the simulator before its RAW
    speeds were put in the robots' units. Looks at the folder's session.json (simulate.py writes
    simulated: true), and at sim_info.json beside it, one level up, and in ../sim (e02_sim.py)."""
    if not folder:
        return False
    folder = os.path.abspath(folder)
    up = os.path.dirname(folder)
    marks = []
    s = _json(os.path.join(folder, "session.json"))
    if isinstance(s, dict) and s.get("simulated"):
        marks.append(s)
    for p in (os.path.join(folder, "sim_info.json"), os.path.join(up, "sim_info.json"),
              os.path.join(up, "sim", "sim_info.json")):
        d = _json(p)
        if isinstance(d, dict):
            marks.append(d)
    return bool(marks) and not any(m.get("raw_units") for m in marks)


def to_robot_units(rows):
    """Rescale legacy simulator RAW rows (dicts, values as text or numbers) in place to the robots' units:
    v_cms cm/s -> mm/s, and every speed column x 1.08. Returns rows."""
    f = SPEED_REPORT_FACTOR
    for r in rows:
        for c in SPEED_COLS:
            v = _num(r.get(c))
            if math.isnan(v):
                continue
            v *= f * (V_COL_PER_CMS if c == "v_cms" else 1.0)
            r[c] = "%.3f" % v if isinstance(r.get(c), str) else v
    return rows
