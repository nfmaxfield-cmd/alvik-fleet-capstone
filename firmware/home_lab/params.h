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

  // --- demo routines (open floor, no tape) ---
  {"demo_a_cm",       30,   "first straight leg, cm"},
  {"demo_b_cm",       30,   "Left: leg after the right turn, cm"},
  {"demo_r_cm",       20,   "Center: circle radius, cm"},
  {"demo_speed_cms",   8,   "Center: speed around the circle, cm/s (max about 12)"},
  {"demo_circle_lead_deg", 3, "Center: stop the circle this many degrees early (it coasts)"},

  // --- sensor stream ---
  {"stream_hz",       20,   "sensor stream rate"},

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
