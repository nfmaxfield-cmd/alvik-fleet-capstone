// Tunable settings for every experiment.
// Change them here (then re-upload), or live from the laptop console with
//   set <name> <value>      e.g.  set lf_base_rpm 40
// Live changes last until the robot is switched off.
#pragma once

struct Param {
  const char* name;
  float value;
  const char* help;
};

// Starting values are reasonable guesses, not measurements. The sensor
// explorer and your first runs should replace the threshold guesses.
Param PARAMS[] = {
  // --- turn test ---
  {"turn_n",          12,   "turns per run"},
  {"turn_deg",        90,   "size of each turn, degrees"},
  {"turn_alt",         0,   "0 = always turn left, 1 = alternate left/right"},
  {"turn_method",      1,   "0 = library rotate(), 1 = IMU-feedback turn"},
  {"turn_max_rpm",    40,   "IMU turn: top wheel speed"},
  {"turn_min_rpm",    10,   "IMU turn: slowest wheel speed near the target"},
  {"turn_kp",        1.0,   "IMU turn: wheel rpm per degree of heading error"},
  {"turn_tol_deg",   1.5,   "IMU turn: done when within this many degrees..."},
  {"turn_hold_ms",    80,   "...for this long"},
  {"turn_timeout_ms",5000,  "give up on a turn after this long"},
  {"settle_ms",      400,   "wait after a turn before reading the final heading"},
  {"turn_gap_ms",    300,   "pause between turns"},

  // --- line following (lap test and grid test) ---
  {"lf_base_rpm",     30,   "cruise wheel speed (motor max is 70)"},
  {"lf_kp",           30,   "steering rpm per unit of line error"},
  {"lf_kd",          0.5,   "steering rpm per unit/s of error change"},
  {"lf_max_corr",     25,   "cap on steering correction, rpm"},
  {"lf_line_thr",    300,   "a line sensor above this is on tape (check with sensor_explorer)"},
  {"lf_lost_ms",     500,   "stop if no sensor sees tape for this long"},
  {"lf_trace",         0,   "1 = also log every control step (50/s) for tuning"},

  // --- lap marker (a colored sticker on the tape) ---
  {"lap_n",            3,   "laps to time after the first marker crossing"},
  {"lap_min_ms",    3000,   "ignore the marker for this long after a lap"},
  {"mk_h_lo",        340,   "marker hue window start, degrees (red wraps past 0)"},
  {"mk_h_hi",         20,   "marker hue window end, degrees"},
  {"mk_s_min",      0.40,   "marker minimum saturation"},
  {"mk_v_min",      0.15,   "marker minimum brightness (keep well above black tape's reading, or tape noise looks red)"},
  {"mk_samples",       3,   "consecutive matching readings to count a marker"},

  // --- grid test ---
  {"grid_mode",        0,   "0 = stop at every node (lab style), 1 = drive through straight nodes"},
  {"grid_stop_ms",   300,   "stop-mode pause at each node (the lab navigator settles 0.3 s)"},
  {"grid_center_cm", 3.5,   "distance from line sensors to wheel axle: how far to roll before turning"},
  {"grid_ignore_cm",   5,   "after a node, ignore crossings for this distance"},
  {"grid_cross_samples", 2, "consecutive all-dark readings to count a crossing"},

  // --- straight-line test (E2) ---
  {"straight_cm",     60,   "straight test: distance to drive, cm"},
  {"straight_rpm",     0,   "straight test: 0 = library move(), otherwise both wheels at this rpm"},

  // --- demo routines (open floor, no tape) ---
  {"demo_route",       0,   "0 = by robot name, 1 = L route (Left's), 2 = circle (Center's), 3 = square"},
  {"square_cm",       50,   "square route: side length, cm"},
  {"square_laps",      1,   "square route: laps (left turns, ends where it started)"},
  {"demo_a_cm",       30,   "first straight leg, cm"},
  {"demo_b_cm",       30,   "Left: leg after the right turn, cm"},
  {"demo_r_cm",       20,   "Center: circle radius, cm"},
  {"demo_speed_cms",   8,   "Center: speed around the circle, cm/s (max about 12)"},
  {"demo_circle_lead_deg", 3, "Center: stop the circle this many degrees early (it coasts)"},

  // --- own pose (survey and follow): wheel distance + gyro heading, camera fixes blended in ---
  {"drive_kct",        3,   "straight drives: degrees of steering per cm off the line"},
  {"drive_max_corr",   8,   "straight drives: most steering away from the line's heading, degrees"},
  {"drive_kh",         4,   "straight drives: turn rate (deg/s) per degree of heading error"},
  {"drive_accel",     20,   "straight drives: speed-up and slow-down, cm/s per s"},
  {"pose_q_cm",     0.01,   "own pose: position variance added per cm driven (cm^2/cm)"},
  {"pose_q_deg",   0.003,   "own pose: heading variance added per degree turned (deg^2/deg)"},
  {"pose_q_dps",   0.002,   "own pose: heading variance added per second (deg^2/s, gyro drift)"},
  {"fix_max_age_ms", 1000,  "follow: ignore a camera fix older than this"},
  {"fix_gate",         4,   "follow: ignore a fix further than this many standard deviations (and 3 cm) from the own pose"},

  // --- survey drive (camera self-calibration) ---
  {"survey_size_cm",  80,   "survey: legs 10,10,20,20,... cm; stops before a leg longer than this"},
  {"survey_clear_cm", 10,   "survey: end a leg early if something is closer than this ahead, then drive back"},
  {"survey_cms",      10,   "survey: driving speed, cm/s"},

  // --- step follower (laptop sends steps from plan.json) ---
  {"tof_recess_cm",  0.25,  "front distance sensor sits this far behind the front face (to be measured)"},
  {"tof_ahead_cm",   4.5,   "front distance sensor sits this far ahead of the wheel axle (turning centre); 9.5 cm body / 2 - tof_recess_cm"},
  {"tof_min_cm",       2,   "front distance sensor: readings below this are not trusted"},
  {"tof_ms",          66,   "approach: read the front distance sensor this often (it updates about 15 times a second)"},
  {"tof_lag_ms",      33,   "approach: a reading is this old on average when read (half a frame; to be measured)"},
  {"front_margin_cm",  2,   "follow: stop if the front reading is this much below what the map expects"},
  {"front_confirm_ms", 100, "survey/follow: the front reading must stay too close this long (2 sensor frames) to stop"},
  {"follow_look_cm",  12,   "follow: straight drives stop for anything in the robot's path closer than this (all 5 zones)"},
  {"path_half_cm",  5.25,   "survey/follow: half the width of the robot's path checked ahead (half the body + 0.5 cm)"},
  {"front_end_cm",     2,   "follow drives: check the path only as far as the step still goes, plus this"},
  {"fuse_from_cm",    10,   "approach: average readings below this (reading + wheel distance = board position)"},
  {"follow_log_hz",   10,   "follow: FOLLOW records per second"},
  {"follow_quiet_ms", 1500, "follow: say so when nothing has come from the laptop for this long"},

  // --- sensor stream ---
  {"stream_hz",       20,   "sensor stream rate"},

  // --- data set logging (every experiment except stream) ---
  {"raw_hz",          50,   "full sensor log rate during runs, per second (0 = off)"},
  {"sync_flash",       1,   "1 = flash both lights white at the start and end of each run, for lining up video"},
  {"sync_ms",        300,   "how long each sync flash stays on, ms (at least 3 camera frames)"},
  {"settle_start_ms", 500,  "pause before the start flash so the robot is still"},
  {"settle_end_ms",  1000,  "stand still this long at the end of a run, before the end flash (camera, gyro and end-pose checks see it at rest)"},
  {"start_nudge_cm",   2,   "turn/demo/straight/survey runs: roll this far forward and back after the start flash, so the camera sees which way the robot faces (0 = off)"},
  {"blink_codes",      1,   "1 = blink the robot's own light pattern during runs, so the camera can find and name it"},
  {"blink_code",       0,   "0 = pattern from the name (Left 1, Center 2, Right 3, others 4 pulses), or 1..4"},
  // --- light pattern on top for the camera (lights.h; tools/lights). OFF unless lights_mode is set ---
  {"lights_mode",      0,   "top light pattern: 0 = off, 1 = three LEDs in an L on GPIO pins (holder), 2 = Modulino Pixels strip (Qwiic)"},
  {"lights_seq",       0,   "L pattern: side LED cycle (160 ms slots) 0 = steady, 1 = 0111, 2 = 0011, 3 = 0101, 4 = 0001; differ for robots with the same holder"},
  {"lights_pin_c",    19,   "L pattern: Nano pin of the corner LED (19 = A2; pin numbers as for sw_left_pin; -1 = not wired)"},
  {"lights_pin_f",    20,   "L pattern: Nano pin of the front LED (20 = A3)"},
  {"lights_pin_s",    24,   "L pattern: Nano pin of the side LED (24 = A7)"},
  {"lights_strip_code", 0,  "strip: code 1..5 (0 = from the name: Left 1, Center 2, Right 3, any other name 4)"},
  {"lights_strip_bri", 10,  "strip: brightness 0..100 (low: a saturated LED blooms over its dark neighbours)"},
  {"lights_idle",      0,   "1 = pattern on from power-on, also between runs; 0 = only during runs (like the blink code)"},
  {"calib_blink_ms", 3000,  "calibration run: how long to stand still blinking its code before moving"},
  {"calib_roll_cm",   15,   "calibration run: roll this far forward and back (camera: heading and wheel distance scale), in the direction with most room it saw while spinning; shorter if there is less room"},
  {"calib_roll_min_cm", 5,  "calibration run: shortest roll worth making; with less room than this (+ safe_front_cm + calib_margin_cm) every way it does not roll"},
  {"calib_margin_cm",  3,   "calibration run: keep this much more than safe_front_cm between the end of the roll and the nearest thing ahead"},
  {"calib_spin_r_cm",  8,   "calibration run: no spin if something is closer than this to its turning centre (corners sweep ~6.7 cm)"},
  {"calib_spin_deg", 720,   "calibration run: spin in place this far left, then back right (gyro scale, light offset, front sensor sweep: where to roll); 0 = skip"},
  {"calib_spin_rpm",  15,   "calibration run: wheel speed while spinning (15 = about 34 deg/s, 11 s a turn)"},
  {"calib_still_ms", 1000,  "calibration run: stand still this long after each roll and before/after each spin (gyro bias, camera pictures)"},

  // --- per-robot corrections (corr.h): saved on the robot, survive power-off; change with corr, not set ---
  // Out of the box (corr reset): 1, 0, 1. Every change is logged as a CORR record.
  {"gyro_scale",     1.0,   "turns: real turning / what the gyro counts (1.075 = the gyro misses 7.5% of every turn; same meaning as calib.py's gyro_scale)"},
  {"gyro_drift_dps", 0.0,   "turns: how fast the gyro's heading creeps while standing still, deg/s, left + (run gyro measures it)"},
  {"wheel_scale",    1.0,   "driving: real cm per wheel-counted cm (0.976 = it drove 97.6 of 100 cm; same meaning as calib.py's wheel_scale)"},
  {"gyro_check_s",    30,   "gyro run: stand still this long, measuring how fast the heading creeps"},

  // --- safety ---
  {"safe_front_cm",    7,   "stop if something is closer than this in front (0 = off)"},
  {"safe_lift",        1,   "1 = stop if the robot reports being lifted (0 = off, if it false-trips)"},
  // --- plywood edge guard (edge.h): OFF until tools/edge_check.py has the E0c edge rows ---
  {"safe_edge",        0,   "stop before an open plywood edge while driving forward: 0 = off, 1 = line sensors, 2 = distance sensor's bottom zone, 3 = both (backing up is blind)"},
  {"edge_dir",         0,   "line cue: +1 = readings go up past an edge, -1 = down, 0 = either way (from tools/edge_check.py)"},
  {"edge_thr",       400,   "line cue: a sensor this far from its learned plywood level (in edge_dir) is past the edge (placeholder)"},
  {"edge_max",         0,   "line cue: ...but no further than this (0 = no limit; set when tape moves the reading further than the edge)"},
  {"edge_n",           3,   "line cue: this many checks in a row (5-10 ms apart) before stopping"},
  {"edge_learn_ms",  300,   "line cue: learn each sensor's plywood level for this long at the start of a run, standing still"},
  {"edge_min_rpm",     2,   "edge guard: active only while the wheels average more than this forward (rpm)"},
  {"edge_bot_cm",     13,   "bottom-zone cue: the floor ahead has dropped away when the bottom zone reads more than this (placeholder)"},
  {"edge_bot_ms",    100,   "bottom-zone cue: ...for this long (2 sensor frames)"},

  // --- nose bumper switches, home dock, touch probe (switches.h, dock.h): OFF until the bumper is fitted ---
  // The bracket numbers (bump_*, bumper_cm) come from hardware/dock/dock_params.py (Arduino's CAD model of the
  // Alvik + the printed bracket); keep them equal to it. sw_gap_cm is still ASSUMED until E00e.
  {"sw_on",            0,   "nose bumper switches fitted and wired: 0 = no (dock/touch refuse to run, RAW swL/swR = -1), 1 = yes"},
  {"sw_left_pin",     17,   "left nose switch: Nano ESP32 pin, IDE numbering 'By Arduino pin' (17 = A0; D5 = 5, A1..A7 = 18..24); switch between the pin and GND, pressed = LOW"},
  {"sw_right_pin",    18,   "right nose switch: Nano ESP32 pin (18 = A1)"},
  {"sw_debounce_ms",   4,   "a switch must read the same this long to count (lever switches chatter 1-5 ms)"},
  {"bumper_cm",        0,   "nose bracket fitted: it sticks out this far in front of the shell, cm (1.34 = the printed one; 0 = none): the calibration's spin and roll keep this much more room"},
  {"bump_ahead_cm", 6.10,   "wheel axle (turning centre) to the bracket's front face, cm (dock_params.py BUMP_AHEAD_MM / 10)"},
  {"bump_half_cm",  4.27,   "each switch's lever touches this far left/right of the centre line, cm (dock_params.py SWITCH_LATERAL_MM / 10)"},
  {"sw_gap_cm",     0.02,   "the bracket face is still this far from what it touches when a lever clicks, cm (ASSUMED lever travel: E00e)"},
  {"dock_rpm",         6,   "dock/touch: creeping speed, wheel rpm (6 = about 1.1 cm/s)"},
  {"dock_press_rpm",   2,   "dock: the clicked side's wheel keeps pushing this gently while the other side pivots in"},
  {"dock_pivot_rpm",   5,   "dock: the other wheel's speed while it pivots"},
  {"dock_seat_ms",   250,   "dock: after both click, both wheels push at dock_press_rpm this long so the face sits flush"},
  {"dock_max_cm",     40,   "dock/touch: give up if no switch clicks within this much wheel travel (wheels slip while the funnel turns the robot)"},
  {"dock_pivot_ms", 6000,   "dock: give up if the second switch has not clicked this long after the first"},
  {"dock_still_ms", 3000,   "dock: stand still, docked, this long (gyro drift at a known heading)"},
  {"dock_drift_max_dps", 0.5, "dock: a heading drift above this while docked is not used (something moved it)"},
  {"dock_back_cm",     8,   "dock run: back out this far at the end (0 = stay docked, e.g. to tape-measure it)"},
  {"dock_n",           1,   "dock run: dockings per run (each after backing out dock_back_cm; 0 back-out = 1 docking)"},
  {"stripe_thr",     300,   "dock stripe: a line sensor this far from its level at the start of the creep is on the stripe"},
  {"stripe_dir",       1,   "dock stripe: +1 = readings go up on it (dark), -1 = down, 0 = either way"},
  {"dock_twist_deg", 0.5,   "dock: a heading change this big before the first click means the funnel has started turning the robot (twist_at_cm)"},
  {"stripe_on",        0,   "dock: 1 = learn and log the floor stripe (dock version 1 only; version 2 has none)"},
  {"posts_n",          1,   "posts run: measurements per run (back off posts_back_cm after each)"},
  {"posts_back_cm",    6,   "posts run: back off this far after each measurement"},
  {"posts_max_cm",    12,   "posts run: give up after this much creep with no click (miss both: start 2-8 cm from the posts)"},
  {"posts_pivot_max_deg", 8, "posts: a lever that has not clicked after the robot has pivoted this far has missed its post"},
  {"touch_back_cm",    2,   "touch run: back off this far after each click"},
  {"touch_n",          1,   "touch run: touches per run (back off touch_back_cm, wait, creep again)"},
};

const int N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// Route for the grid test: one letter per tape crossing.
// S = straight, L = left, R = right, E = end (stop on that crossing).
// Change live with:  route SLSRE
String gRoute = "SSLSE";

Param* findParam(const String& name) {
  for (int i = 0; i < N_PARAMS; i++) {
    if (name.equalsIgnoreCase(PARAMS[i].name)) return &PARAMS[i];
  }
  return nullptr;
}

// P("lf_kp") returns the current value. A typo returns 0 and logs a warning.
float P(const char* name);
