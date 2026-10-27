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
    # exact per-robot values instead of the random draws above (robot name -> percent), for tests of
    # wheel-size estimation, e.g. {"Left": -5, "Center": 5}: Left's wheels 5% smaller than nominal (its
    # odometry reads about 5% long). Robots not listed keep their random draw.
    radius_scale_err_pct_by_robot: dict = field(default_factory=dict)
    track_err_pct_by_robot: dict = field(default_factory=dict)
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
    zone_width_deg: float = 11.0          # each zone reports the nearest thing in its slice (3 rays across it)
    sensor_forward_cm: float = 4.5        # sensor sits this far ahead of the axle
    noise_cm: float = 0.5
    noise_pct: float = 1.0
    max_range_cm: float = 350.0           # reading when nothing is in range
    min_range_cm: float = 2.0             # below this the reading is untrustworthy (here: any value under it)
    hz: float = 15.0                      # new readings per second (held in between); 0 = fresh every call
    # how the real sensor differs from the nominal zone directions (deg, left positive), per robot name:
    # {"Left": [dL, dCL, dC, dCR, dR]} (sensor turned on its mount + each zone's own offset); and a range
    # error per robot: {"Left": [bias_cm, scale_pct]}. Empty = perfect, as before.
    zone_angle_err_deg_by_robot: dict = field(default_factory=dict)
    range_err_by_robot: dict = field(default_factory=dict)
    bottom_cm: float = 9.0                # "bottom" zones see the floor about this far ahead (no board set)
    # bottom zone geometry, used when session.board_cm is set (GUESSES; E0c rows edge_n15/10/5 and the
    # sensor height measured by hand confirm or replace them): the sensor sits sensor_height_cm above the
    # plywood; the zone looks bottom_deg below horizontal, spanning bottom_vspan_deg up/down and
    # bottom_hspan_deg across; it reports the NEAREST thing any part of it sees (5 x 3 rays), so it reads
    # the cement only once its whole floor patch is past an open edge. 2.6 cm / 16.8 deg gives 9.0 cm on
    # plywood at the zone centre (about 8.3 cm nearest), its floor patch about 7-9 cm ahead of the sensor.
    sensor_height_cm: float = 2.6
    bottom_deg: float = 16.8
    bottom_vspan_deg: float = 5.6
    bottom_hspan_deg: float = 45.0
    bottom_noise_cm: float = 0.3
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
    # past an open plywood edge (session.board_cm / open_edges): the sensor looks down at the cement
    # one plywood thickness further away. GUESSES until the E0c edge rows: less light comes back from
    # further away, so the reading is assumed to go UP (like dark tape), near the top of the range.
    past_edge_reading: float = 1000.0
    past_edge_noise: float = 40.0


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
    calib_commands: List[str] = field(default_factory=list)   # console commands sent just before `calibrate`
    #                                       (e.g. "set calib_dir_limits 0 2": the laptop's roll caps, firmware look.h)
    parked_pose: List[float] = field(default_factory=lambda: [-160.0, -60.0, 0.0])   # robots not running wait here, out of view
    video_lead_s: float = 5.0
    video_tail_s: float = 5.0
    battery_every: int = 3                # battery_pct written every Nth sheet row
    motor_fw: str = "1.1.0"
    tape_lines: List[List[float]] = field(default_factory=lambda: [[-10.0, 0.0, 140.0, 0.0], [0.0, -40.0, 0.0, 60.0]])
    tape_width_cm: float = 1.9
    # the plywood board under the robots: [x0, y0, x1, y1] floor cm (None = endless floor, as before), which
    # of its sides have no border board ("x0" = the x = x0 side, "x1", "y0", "y1"), and the step down to the
    # cement past an open side. Only the line sensors and the bottom zone use it (nothing falls off).
    board_cm: Optional[List[float]] = None
    open_edges: List[str] = field(default_factory=list)
    # partly open sides (school workspace, 2026-10-27): {"y0": [[lo, hi], ...]}: only these stretches along the side
    # are open (a side in open_edges with no entry here is open all along)
    open_spans: dict = field(default_factory=dict)
    board_thick_cm: float = 1.9
    # floor seams (E02): lines [x0, y0, x1, y1] where two plywood sheets meet. Crossing one with the axle jolts
    # the accelerometer (seam_jolt_g, random sign, ~15 ms) and slows the wheels to seam_dip_frac for seam_dip_ms.
    # GUESSES (no real seam has been measured): a false-bump test, not a model of the real floor.
    seams: List[List[float]] = field(default_factory=list)
    seam_jolt_g: float = 0.4
    seam_dip_ms: float = 50.0
    seam_dip_frac: float = 0.5


def _dock_params():
    """hardware/dock/dock_params.py (the printed dock and bumper), or None if it is missing."""
    import importlib.util
    path = os.path.join(REPO, "hardware", "dock", "dock_params.py")
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("dock_params", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_DP = _dock_params()


def _dp(name, default):
    return round(getattr(_DP, name) / 10.0, 4) if _DP is not None and hasattr(_DP, name) else default


@dataclass
class DockModel:
    """Home docks, the nose bumper and its two switches (firmware dock.h, switches.h). The geometry defaults
    come from hardware/dock/dock_params.py (mm there, cm here), so the simulated dock is the printed one.
    With no docks and contact_boards False nothing here changes the simulation."""
    # each dock: [x_cm, y_cm, heading_deg]: the middle of its back wall's inner face, and the way a docked robot
    # faces (into the wall)
    docks: List[List[float]] = field(default_factory=list)
    contact_boards: bool = False          # boards (world_polys) are solid for the robot's body too (touch probe)
    # the TRUE bumper and switches (the firmware's bump_ahead_cm / bump_half_cm / sw_gap_cm are its belief)
    bump_ahead_cm: float = _dp("BUMP_AHEAD_MM", 6.10)       # wheel axle to the bumper's front face
    # the back attachment (firmware sw_back 1, 2026-10-25b): wheel axle to its switch face, behind the axle. PLACEHOLDER
    # (as params.h e02_back_face_cm): the nose bracket's depth mirrored, until hardware/dock has the back attachment
    back_face_cm: float = 6.10
    bump_half_cm: float = _dp("SWITCH_LATERAL_MM", 4.27)    # each lever's contact, left/right of the centre line
    sw_gap_cm: float = _dp("SW_GAP_MM", 0.02)               # face-to-wall gap when a lever clicks
    sw_hyst_cm: float = 0.02              # it lets go only this much further out (switch hysteresis)
    sw_bounce_ms: float = 1.5             # the raw reading chatters this long after each change
    chamfer_cm: float = _dp("CORNER_CHAMFER_MM", 0.3)
    # the nose bracket is wider than the robot (an arm down each side to the side inserts): its width, and how far
    # back from its face the arms (and their screw heads) reach; behind that the outline narrows to the robot
    bumper_w_cm: float = _dp("BRACKET_W_MM", 10.22)
    bumper_len_cm: float = _dp("BRACKET_LEN_MM", 3.8)
    # the dock (its pocket fits the bracket)
    pocket_w_cm: float = _dp("POCKET_W_MM", 10.32)
    pocket_depth_cm: float = _dp("POCKET_DEPTH_MM", 2.5)
    funnel_len_cm: float = _dp("FUNNEL_LEN_MM", 4.5)
    funnel_flare_cm: float = _dp("FUNNEL_FLARE_MM", 2.8)
    wall_t_cm: float = _dp("WALL_T_MM", 0.32)
    back_depth_cm: float = _dp("BACK_BLOCK_DEPTH_MM", 4.8)
    stripe_x0_cm: float = _dp("STRIPE_X0_MM", -5.2)         # floor stripe, dock X (far edge, near edge)
    stripe_x1_cm: float = _dp("STRIPE_X1_MM", -4.4)
    stripe_reading: float = 850.0         # GUESS: black PLA reads like black tape
    stripe_noise: float = 30.0
    dock_tof: bool = True                 # the front distance sensor sees the dock's walls
    # how the body moves when something pushes it (position-based contact): rolling forward/back is easy (1),
    # sliding sideways on the tyres is hard, turning in between (per cm^2 of lever arm)
    lat_mobility: float = 0.12
    turn_mobility: float = 0.05
    # GUESSES: what makes one docking differ from the next besides the pocket's 1 mm play: dust and print
    # ripples on the back wall / bumper face, the wall flexing. Drawn again at the start of every creep.
    seat_sd_cm: float = 0.005             # back wall's effective position along the axis
    seat_sd_deg: float = 0.1              # ... and its effective tilt
    # E02 bump detection: a wheel held back by a solid keeps only this share of its commanded speed (the motor
    # stalls, the tyre slips the rest). 1.0 = the wheels keep turning at full speed and only slip (the model the
    # dock tests were built on); E02 tests use less. GUESS: the real Alvik pushing a board has not been measured.
    wall_stall_frac: float = 1.0
    # an occasional MISSED bump (dry runs, e02_sim.py --miss-bump): per contact with a board, the chance that the
    # wheels keep slipping round (keeping miss_slip_frac of their speed, so no stall is seen) and the jolt is lost,
    # so the creep runs on to its limit and the find fails safely; and the chance that only the jolt is lost
    # (detected by the stall alone, later). 0 = never (the tests).
    miss_bump_prob: float = 0.0
    miss_jolt_prob: float = 0.0
    miss_slip_frac: float = 0.75


@dataclass
class LightsModel:
    # light pattern on each robot's top (tools/lights/patterns.py, firmware lights.h). "off" = not drawn and
    # the firmware's lights_mode stays 0 (as before). "l3": three LEDs in an L on a black holder; "strip":
    # an 8-LED Modulino Pixels row. The pattern is lit while a run is going (like the blink code).
    mode: str = "off"
    names: dict = field(default_factory=dict)   # robot -> code ("S1L", "S1L/0011"; strip "1".."5"); {} = default table
    family: dict = field(default_factory=dict)  # patterns.L3Family / StripFamily overrides (arm_cm, side_cm, pitch_cm...)
    brightness: float = 450.0             # peak grey level a lit LED adds (above 255: the core saturates; ASSUMED)
    blur_px: float = 0.7                  # optics + motion blur, px (sd), on top of the LED's own size
    led_d_cm: float = 0.5                 # LED lens diameter (5 mm LEDs; the strip's LC8822-2020: 0.2)
    holder_grey: float = 35.0             # black printed holder under the l3 LEDs / the strip's board, dark (0 = not drawn)


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
    dock: DockModel = field(default_factory=DockModel)
    lights: LightsModel = field(default_factory=LightsModel)


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
