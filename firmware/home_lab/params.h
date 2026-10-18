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
  {"start_nudge_cm",   2,   "turn/demo/straight/survey runs: roll this far forward and back after the start flash, so the camera sees which way the robot faces (0 = off)"},
  {"blink_codes",      1,   "1 = blink the robot's own light pattern during runs, so the camera can find and name it"},
  {"blink_code",       0,   "0 = pattern from the name (Left 1, Center 2, Right 3, others 4 pulses), or 1..4"},
  {"calib_blink_ms", 3000,  "calibration run: how long to stand still blinking its code before moving"},

  // --- safety ---
  {"safe_front_cm",    7,   "stop if something is closer than this in front (0 = off)"},
  {"safe_lift",        1,   "1 = stop if the robot reports being lifted (0 = off, if it false-trips)"},
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
