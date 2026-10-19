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
  {"lights_pin_c",    -1,   "L pattern: GPIO of the corner LED (-1 = not wired)"},
  {"lights_pin_f",    -1,   "L pattern: GPIO of the front LED (-1 = not wired)"},
  {"lights_pin_s",    -1,   "L pattern: GPIO of the side LED (-1 = not wired)"},
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
