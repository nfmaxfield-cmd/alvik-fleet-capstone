"""All the numbers the simulator uses, in one place.

Every value here is an ASSUMPTION unless it is copied from the firmware.
Replace them with measurements as you make them (see README.md). Values come
in three kinds:

  - nominal values the robot itself believes (wheel radius, wheel track):
    the robot's own odometry uses these;
  - "+/- range" values: each simulated robot draws its own true value
    uniformly inside the range (so every robot is a little different);
  - noise levels (standard deviations).

Override any of them with a JSON file:  simulate.py --config my.json
(write the full default file to edit with:  simulate.py --write-config my.json)
"""
import json
import os
import re
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import List, Optional

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIRMWARE = os.path.join(REPO, "firmware", "home_lab")


@dataclass
class RobotModel:
    # --- geometry (TO BE MEASURED on the real robots) ---
    wheel_radius_cm: float = 1.7          # nominal; the robot's odometry uses this
    wheel_track_cm: float = 9.0           # distance between the wheels' contact points; nominal
    length_cm: float = 9.5                # front face to rear face (tape marks are at +/- length/2)
    width_cm: float = 9.5
    height_cm: float = 4.0                # top of the shell above the floor (the camera sees the top)
    axle_to_center_cm: float = 0.0        # how far the body centre sits AHEAD of the wheel axle
    # --- how each real robot differs from nominal (drawn per robot, uniform +/-) ---
    radius_scale_err_pct: float = 1.0     # both wheels bigger/smaller than nominal together
    radius_mismatch_pct: float = 0.5      # left vs right wheel radius difference
    track_err_pct: float = 3.0            # real wheel track vs nominal
    # --- motion (library defaults are not documented; guesses) ---
    move_speed_cms: float = 10.0          # library move() cruise speed
    move_accel_cms2: float = 20.0         # library move() ramp up/down
    motor_tau_s: float = 0.05             # wheel speed follows a set_wheels_speed() command with this lag
    brake_tau_s: float = 0.03             # ... and stops with this lag after brake()
    wheel_track_noise_deg: float = 0.5    # how far each wheel wanders from the move() profile (sd, wheel degrees)
    wheel_track_noise_tau_s: float = 0.2
    move_slip_mean_pct: float = 0.2       # each wheel covers this much less ground than it turned, per move
    move_slip_sd_pct: float = 0.05
    accel_slip_pct_per_g: float = 2.0     # extra slip while speeding up / slowing down
    turn_slip_mean_pct: float = 3.0       # wheels scrub during in-place turns: rotation is this much smaller
    turn_slip_sd_pct: float = 1.0
    encoder_res_deg: float = 0.36         # wheel angle resolution (1000 counts per wheel turn)
    wheel_speed_noise_rpm: float = 0.4
    loop_overhead_ms: List[float] = field(default_factory=lambda: [1.5, 3.5])  # sensor reads per control loop
    library_block_ms: float = 200.0       # move()/rotate() wait inside the library before starting
    target_reached_ms: float = 100.0      # the successful is_target_reached() call waits this long
    move_done_settle_ms: float = 30.0     # library reports "reached" this long after the wheels stop
    # --- battery ---
    battery_start_pct: List[float] = field(default_factory=lambda: [90.0, 100.0])
    battery_idle_pct_per_h: float = 3.0
    battery_moving_pct_per_h: float = 20.0


@dataclass
class ImuModel:
    yaw_sign: int = 1                     # +1: raw yaw grows on a left turn (firmware measures this; unknown)
    gyro_bias_dps: float = 0.05           # left-over gyro bias after the board's own calibration (sd per boot)
    gyro_bias_walk_dps_per_rt_s: float = 0.002   # bias slowly wanders (random walk)
    gyro_noise_dps: float = 0.07          # white noise on each reported gyro sample (sd)
    yaw_arw_deg_per_rt_s: float = 0.01    # noise that gets integrated into the fused yaw
    gyro_scale_err_pct: float = 0.5       # gyro scale factor error, drawn per robot (uniform +/-)
    accel_noise_g: float = 0.004
    accel_vibration_g: float = 0.01       # extra noise while the wheels turn
    accel_bias_g: float = 0.01            # mounting tilt / offset, drawn per robot (sd)
    tilt_noise_deg: float = 0.1           # roll/pitch noise


@dataclass
class TofModel:
    zone_angles_deg: List[float] = field(default_factory=lambda: [22.0, 11.0, 0.0, -11.0, -22.0])  # L, CL, C, CR, R
    sensor_forward_cm: float = 4.5        # sensor sits this far ahead of the axle
    noise_cm: float = 0.5
    noise_pct: float = 1.0
    max_range_cm: float = 350.0           # reading when nothing is in range
    bottom_cm: float = 9.0                # "bottom" zones see the floor about this far ahead
    top_extra_pct: float = 3.0            # "top" zones look slightly up: a bit further than the centre zone
    # rectangular room walls the ToF rays hit, floor coordinates (cm)
    arena_x: List[float] = field(default_factory=lambda: [-80.0, 220.0])
    arena_y: List[float] = field(default_factory=lambda: [-120.0, 140.0])


@dataclass
class LineModel:
    sensor_forward_cm: float = 3.5        # line sensors ahead of the axle (= grid_center_cm in params.h)
    spacing_cm: float = 1.2               # between neighbouring line sensors
    floor_reading: float = 150.0
    floor_noise: float = 15.0
    tape_reading: float = 850.0
    tape_noise: float = 30.0
    footprint_cm: float = 0.6             # each sensor averages over this width


@dataclass
class NetworkModel:
    raw_loss: float = 0.005               # chance each RAW row is lost over WiFi
    clock_loss: float = 0.005             # chance a clock answer is lost
    burst_per_min: float = 0.0            # WiFi drop-outs (every packet lost for burst_ms); 0 = none
    burst_ms: List[float] = field(default_factory=lambda: [50.0, 250.0])
    oneway_base_ms: float = 1.5           # fixed part of the robot -> laptop delay
    oneway_exp_ms: float = 3.0            # plus a random (exponential) part with this mean
    spike_prob: float = 0.01              # chance a packet is held up by the access point ...
    spike_ms: List[float] = field(default_factory=lambda: [20.0, 120.0])   # ... for this long
    clock_slow_prob: float = 0.03         # chance a clock question is answered slowly
    clock_slow_ms: List[float] = field(default_factory=lambda: [60.0, 250.0])
    clock_drift_ppm: float = 30.0         # robot crystal error, drawn per robot (uniform +/-)
    boot_before_s: List[float] = field(default_factory=lambda: [60.0, 900.0])  # robots switched on this long before the session
    wifi_dbm: List[float] = field(default_factory=lambda: [-62.0, -48.0])


@dataclass
class CameraModel:
    width: int = 640
    height: int = 480
    fps: float = 30.0                     # recording rate (lower it to generate faster)
    floor_width_cm: float = 160.0         # floor width seen across the image -> sets the pixel scale
    height_cm: float = 115.0              # camera height above the floor
    center_floor_cm: List[float] = field(default_factory=lambda: [60.0, 10.0])  # floor point under the image centre
    yaw_deg: float = 1.0                  # camera turned slightly against the tape axes
    tilt_deg: List[float] = field(default_factory=lambda: [0.8, -0.5])  # not perfectly level (about image x, y)
    k1: float = 0.0                       # barrel distortion (e.g. -0.05); 0 = none
    exposure_ms: float = 12.0
    pi_clock_offset_s: float = 0.040      # Pi clock minus laptop clock
    capture_delay_frames: float = 1.0     # frame reaches the Pi about one frame after it was taken
    capture_jitter_ms: float = 2.0
    frame_drop_prob: float = 0.002        # a frame now and then never arrives
    clock_ppm: float = 10.0               # camera frame clock vs real time
    noise_sd: float = 3.0                 # sensor noise, grey levels
    vignette: float = 0.30                # corners this much darker
    light_gradient: float = 0.12          # uneven lighting across the image
    hands_per_min: float = 0.6            # random hands reaching into view
    boxes_per_10min: float = 1.0          # a box put down in view for a while
    hand_on_carry: bool = True            # draw a hand when the robot is picked up and placed


@dataclass
class DetectorModel:
    sigma_px: float = 1.5                 # pixel noise of a detection
    miss_prob: float = 0.03
    miss_prob_occluded: float = 0.6       # when a hand covers the robot
    false_pos_per_frame: float = 0.003    # random clutter detections
    hand_false_pos_prob: float = 0.3      # per frame a hand is in view
    box_false_pos_prob: float = 0.05      # per frame a box is in view
    blind_region_px: Optional[List[float]] = None   # [x0, y0, x1, y1]: nothing is detected inside


@dataclass
class KidnapModel:
    run_order: Optional[int] = None       # sheet order number of the run with a kidnap (None = no kidnap)
    at_s: float = 3.0                     # seconds after the experiment's motion starts
    lifted_s: float = 2.0
    move_cm: List[float] = field(default_factory=lambda: [20.0, 40.0])
    turn_deg: List[float] = field(default_factory=lambda: [30.0, 120.0])   # new heading differs by this (random sign)
    # "pause": wheels stop while lifted and the route carries on afterwards (sim adds `set safe_lift 0`).
    # "abort": firmware default safe_lift 1: the run stops with "robot lifted".
    mode: str = "pause"


@dataclass
class SessionModel:
    start_iso: str = "2026-10-18T10:15:00-04:00"
    start_pose: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # x_cm, y_cm, heading_deg
    placement_err_cm: float = 0.5         # uniform +/- on x and y
    placement_err_deg: float = 1.0        # uniform +/- on heading
    tape_noise_cm: float = 0.15           # sd of each tape-measured coordinate
    tape_round_cm: float = 0.1
    after_run_s: float = 6.0              # end marks measured
    carry_s: float = 3.0                  # picked up and put back on the start
    before_run_s: float = 8.0             # start marks measured
    cmd_spacing_s: float = 1.5            # between typed console commands
    # calibration run (run_sheet.py sends `calibrate` once per robot per power-on, before its first row,
    # with the robot in view but off the start mark): where it is put down, and the operator's pauses
    calib_area_cm: List[float] = field(default_factory=lambda: [20.0, 50.0, 20.0, 45.0])   # x0, x1, y0, y1
    calib_wait_s: float = 2.0             # put down -> Enter pressed -> `calibrate` sent
    after_calib_s: float = 2.0            # RUNEND seen -> robot picked up to go to the start mark
    parked_pose: List[float] = field(default_factory=lambda: [-160.0, -60.0, 0.0])   # robots not running wait here, out of view
    video_lead_s: float = 5.0
    video_tail_s: float = 5.0
    battery_every: int = 3                # battery_pct written every Nth sheet row
    motor_fw: str = "1.1.0"
    tape_lines: List[List[float]] = field(default_factory=lambda: [[-10.0, 0.0, 140.0, 0.0], [0.0, -40.0, 0.0, 60.0]])
    tape_width_cm: float = 1.9


@dataclass
class SimConfig:
    robot: RobotModel = field(default_factory=RobotModel)
    imu: ImuModel = field(default_factory=ImuModel)
    tof: TofModel = field(default_factory=TofModel)
    line: LineModel = field(default_factory=LineModel)
    net: NetworkModel = field(default_factory=NetworkModel)
    camera: CameraModel = field(default_factory=CameraModel)
    detector: DetectorModel = field(default_factory=DetectorModel)
    kidnap: KidnapModel = field(default_factory=KidnapModel)
    session: SessionModel = field(default_factory=SessionModel)


def _merge(obj, data, path=""):
    names = {f.name for f in fields(obj)}
    for k, v in data.items():
        if k.startswith("_"):
            continue                      # "_comment" keys are allowed
        if k not in names:
            raise ValueError(f"unknown config setting {path}{k}")
        cur = getattr(obj, k)
        if is_dataclass(cur):
            _merge(cur, v, f"{path}{k}.")
        else:
            setattr(obj, k, v)


def load_config(path=None):
    cfg = SimConfig()
    if path:
        with open(path) as f:
            _merge(cfg, json.load(f))
    return cfg


def dump_config(cfg, path):
    with open(path, "w") as f:
        json.dump(asdict(cfg), f, indent=2)


# ------------------------------------------------------------ firmware facts
def firmware_params():
    """[(name, value)] in the order params.h lists them (PARAM records follow this order)."""
    with open(os.path.join(FIRMWARE, "params.h")) as f:
        text = f.read()
    out = [(m.group(1), float(m.group(2)))
           for m in re.finditer(r'\{\s*"(\w+)"\s*,\s*([-+0-9.eE]+)\s*,\s*"', text)]
    if not out:
        raise RuntimeError("could not read the parameter table from firmware/home_lab/params.h")
    return out


def firmware_version():
    with open(os.path.join(FIRMWARE, "home_lab.ino")) as f:
        m = re.search(r'#define\s+FW_VERSION\s+"([^"]+)"', f.read())
    return m.group(1) if m else "unknown"


def firmware_raw_columns():
    """The RAW header columns exactly as rawlog.h sends them (the C string pieces joined)."""
    with open(os.path.join(FIRMWARE, "rawlog.h")) as f:
        text = f.read()
    m = re.search(r'void header\(\)\s*\{\s*net::sendLine\((.*?)\);', text, re.S)
    if not m:
        raise RuntimeError("could not find the RAW header in firmware/home_lab/rawlog.h")
    joined = "".join(re.findall(r'"([^"]*)"', m.group(1)))
    return joined.split(",")[2:]          # drop "H", "RAW"
