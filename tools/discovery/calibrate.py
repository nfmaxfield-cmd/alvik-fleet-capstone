"""Per-stream calibration where a reference exists: the factor that turns the stream into physical units, and an
offset where one can be measured. References, in order of independence from the robot:
  camera heading change between drives (turn rates and angles), camera distance driven (wheel angles, positions),
  camera speed (speeds), camera distance to the boards ahead (range zones), gravity at rest (accelerometer),
  commanded turns (sign and rough size only: the robot ends its turns on its own gyro, so the commanded angle is
  not independent of the gyro).
Units are snapped to the nearest common factor when within SNAP (design doc: 1, 57.3, 9.81, 1000); the remainder
is the stream's calibration scale."""
import numpy as np

SNAP = 0.15        # chosen (design doc section 2): a ratio within 15% of a unit factor is snapped to it
UNIT_FACTORS = {"same units": 1.0, "rad -> deg": 180 / np.pi, "deg -> rad": np.pi / 180, "x10": 10.0,
                "x0.1": 0.1, "x100": 100.0, "x0.01": 0.01, "m/s2 per g": 9.80665, "1/9.81": 1 / 9.80665,
                "x1000": 1000.0, "x0.001": 0.001}
WHEEL_DEG_PER_CM_NOMINAL = 360 / (2 * np.pi * 1.7)   # data-formats.md: wheel radius about 1.7 cm, to be measured


def snap(ratio):
    """(unit name, unit factor, remaining scale) for a measured ratio (physical per stream unit)."""
    if ratio is None or not np.isfinite(ratio) or ratio == 0:
        return None, None, None
    best = None
    for name, u in UNIT_FACTORS.items():
        rem = abs(ratio) / u
        if abs(rem - 1) <= SNAP and (best is None or abs(rem - 1) < abs(best[2] - 1)):
            best = (name, u * np.sign(ratio), rem)
    if best is None:
        return "no common unit", None, abs(ratio)
    return best


def calibrate(feats, results):
    """{stream: calibration dict} for streams whose guessed type has a reference in this case."""
    out = {}
    for s, r in results.items():
        f = feats[s]
        t = r["type"]
        c = None
        if t in ("gyro_rate",) and np.isfinite(f.get("cam_turn_int_k", np.nan)) and \
                (f.get("cam_turn_int_excess") or 0) > 0.15:
            k = f["cam_turn_int_k"]          # camera deg per (stream unit x s)
            c = {"reference": "camera heading change between %d pairs of drives" % f.get("cam_turn_n", 0),
                 "physical_per_unit": k, "physical_unit": "deg/s", "quality": f.get("cam_turn_int_R")}
        elif t in ("gyro_angle",) or (t == "derived" and np.isfinite(f.get("cam_turn_ang_k", np.nan)) and
                                      (f.get("cam_turn_ang_excess") or 0) > 0.15):
            if np.isfinite(f.get("cam_turn_ang_k", np.nan)) and (f.get("cam_turn_ang_excess") or 0) > 0.15:
                c = {"reference": "camera heading change between %d pairs of drives" % f.get("cam_turn_n", 0),
                     "physical_per_unit": f["cam_turn_ang_k"], "physical_unit": "deg",
                     "quality": f.get("cam_turn_ang_R")}
        if c is None and t in ("wheel_angle", "derived") and np.isfinite(f.get("ep_per_cm", np.nan)) and \
                f.get("accumulates"):
            c = {"reference": "camera distance over %d drive stretches" % f.get("ep_n", 0),
                 "physical_per_unit": 1.0 / f["ep_per_cm"], "physical_unit": "cm",
                 "quality": 1 - min(1.0, f.get("ep_per_cm_spread", 1.0))}
        if c is None and t in ("wheel_speed",) and np.isfinite(f.get("ep_per_cms", np.nan)):
            c = {"reference": "camera speed over drive stretches",
                 "physical_per_unit": 1.0 / f["ep_per_cms"], "physical_unit": "cm/s",
                 "quality": 1 - min(1.0, f.get("ep_per_cms_spread", 1.0))}
        if c is None and t == "range" and np.isfinite(f.get("board_slope", np.nan)) and \
                abs(f.get("board_corr") or 0) > 0.5:
            c = {"reference": "camera distance to the boards ahead at %d drive ends" % f.get("board_n", 0),
                 "physical_per_unit": 1.0 / f["board_slope"], "physical_unit": "cm",
                 "offset_units": f.get("board_offset"), "quality": abs(f.get("board_corr"))}
        if c is None and t == "accel" and abs(f.get("level_over_noise") or 0) > 300:
            c = {"reference": "gravity at rest (this axis reads a steady %.4g)" % f.get("rest_level"),
                 "physical_per_unit": 1.0 / abs(f["rest_level"]), "physical_unit": "g",
                 "quality": 1.0}
        if c is None:
            continue
        name, u, rem = snap(c["physical_per_unit"])
        c["unit_guess"], c["unit_factor"], c["scale_after_unit"] = name, u, rem
        out[s] = c
    return out
