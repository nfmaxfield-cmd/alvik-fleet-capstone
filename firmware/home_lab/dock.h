// Home dock and touch probe (firmware 2026-10-19a+), with the nose bumper's two
// lever switches (switches.h). Hardware: hardware/dock/README.md. Records:
// docs/data-formats.md (DOCK, TOUCH). Test: docs/experiments/E00e-dock.
//
// HOME DOCK: a printed funnel fixed to the plywood narrows to a pocket 1 mm
// wider than the nose bracket, closed by a flat back wall. creep() drives both wheels
// slowly forward (dock_rpm, front stop off); the funnel centres the robot
// sideways. When one switch clicks, that side's wheel keeps pushing gently
// (dock_press_rpm) and the other wheel pivots the robot in (dock_pivot_rpm)
// until the other switch clicks too: the bumper face then lies along the wall.
// Both wheels push gently for dock_seat_ms more (the face seats flush), then
// brake. The robot's pose there is the dock's pose minus the bumper geometry
// (bump_ahead_cm), to about 0.5 mm and 0.3 deg: a near-exact fix. It then stands
// still dock_still_ms: the gyro heading's drift over that time, at a heading
// known not to change, is the gyro's left-over bias (follow mode takes it off
// its own heading from then on: pose.h driftDps).
// The control needs no state: neither switch -> both wheels creep; only the
// left -> left wheel presses, right wheel pivots (turns the robot left, bringing
// the right corner forward); only the right -> the mirror image; both -> done.
// So a switch that lets go while pivoting just makes it creep again.
//
// FLOOR STRIPE: a dark printed stripe across the dock mouth. The line sensors
// cross it on the way in. Each sensor's level is learned standing still at the
// start of the creep (the edge guard's way: edge::median, edge::moved with
// stripe_thr / stripe_dir), and the travel at which each sensor first reads the
// stripe is logged: a second, independent reading of where the robot is along
// the dock's axis (the laptop knows where the stripe is). The edge guard's
// line cue is off during the dock creep (it would take the stripe for an edge);
// its bottom-zone cue stays on.
//
// TOUCH PROBE: creep straight ahead until EITHER switch clicks, brake. The
// travel at the first raw closing (switches.h) plus where the robot started
// says where the face it touched is: the bumper face is sw_gap_cm short of it
// at the lever's position (bump_ahead_cm ahead of the axle, bump_half_cm to the
// side). Wheel distance over a few cm is good to about 0.1 mm.
//
// POST STATION (`run posts`, firmware 2026-10-20a, dock version 2): two narrow posts one switch spacing
// apart, on the dock's outside. Creep straight (no funnel); at the first click that side presses and the
// other pivots in, as in the dock, until the other lever clicks on its post. The travel at the first click
// gives the distance to the station; the gyro's turn from the first click to the second (a few degrees: the
// gyro's scale error barely matters there) gives the angle the robot came in at. A lever that misses its
// post (the robot is off sideways by more than about the lever's half length) never clicks: the pivot passes
// posts_pivot_max_deg, and the result names the side that missed.
//
// FUNNEL TWIST (dock mode, 2026-10-20a): both wheels creep at the same speed until the first click, so any
// heading change before it is the funnel pushing the robot straight (funnel_deg); the pivot to both clicks
// (pivot_deg) finishes the job. Their sum, the whole turn from the start to seated, is how crooked the robot
// came in, with the opposite sign; the travel where the turning began (twist_at_cm) says how far into the
// funnel it first touched a wall. So the parking dock reports the robot's approach error although it erases
// it (simulation: within about 1 deg of the true approach angle at 8 deg).
//
// Both refuse to start unless sw_on is 1 (failed: "nose switches not
// configured"), and refuse if a switch is already pressed (stuck or wiring).
#pragma once

#include "pose.h"

extern String gRunId;

namespace dock {

const float NONE = -1.0f;

struct Result {
  bool ok = false;
  char why[48] = "";
  unsigned long msStart = 0, msFirst = 0, msBoth = 0;  // creep start, first click, both clicked (dock) / the click (touch)
  unsigned long msPress[2] = {0, 0};                    // each switch's first raw closing (0 = never)
  float travel = 0;                                     // wheel distance from the creep's start at the end, cm
  float travelAt[2] = {NONE, NONE};                     // ... when each switch first closed (raw), cm
  int first = -1;                                       // 0 left, 1 right
  float pivotDeg = 0;                                   // heading change from the first click to both (dock, posts)
  float funnelDeg = 0;                                  // heading change from the start to the first click (dock: the funnel's twist)
  float twistAt = NONE;                                 // travel when that change first passed dock_twist_deg, cm
  float stripe[3] = {NONE, NONE, NONE};                 // travel when each line sensor (L, C, R) first read the stripe
  float x = 0, y = 0, th = 0;                           // own pose when it ended (follow pose, else library odometry)
  float x0 = 0, y0 = 0, th0 = 0;                        // ... at the creep's start
  float front0 = 0;                                     // front sensor (centre zone) at the start, cm
};

// Pose to log: the follow pose if it is tracked, else the library's odometry (start frame).
void ownPose(float& x, float& y, float& th) {
  if (pose::on && pose::known) { x = pose::x; y = pose::y; th = pose::th(); return; }
  alvik.get_pose(x, y, th);
  th = gOdoSign * th;
}

bool ready(Result& r) {
  if (!sw::on) {
    snprintf(r.why, sizeof(r.why), "nose switches not configured (sw_on 0)");
    return false;
  }
  sw::tick();
  if (sw::raw[0] || sw::raw[1]) {
    snprintf(r.why, sizeof(r.why), "%s switch pressed before the start", sw::raw[0] ? "left" : "right");
    return false;
  }
  return true;
}

// Learn the line sensors' floor level, standing still (~150 ms).
bool learnStripe(float level[3]) {
  const int N = 24;
  int buf[3][N];
  int n = 0;
  unsigned long t0 = millis();
  while (n < N && millis() - t0 < 300) {
    if (shouldAbort()) return false;
    alvik.get_line_sensors(buf[0][n], buf[1][n], buf[2][n]);
    n++;
    delay(5);
  }
  for (int k = 0; k < 3; k++) level[k] = n ? edge::median(buf[k], n) : 0;
  return n > 0;
}

// Creep in. dockMode: until both switches (with the pivot and the seat push); otherwise (touch) until either.
// Returns false on a stop / failure (r.why, gFail or gAbortReason say why).
// posts: the post station (pivot to both clicks like the dock, but no stripe, no seat push, and a pivot limit).
bool creep(bool dockMode, Result& r, float maxCm = -1, bool posts = false) {
  if (!ready(r)) { gFail = r.why; return false; }
  float saved = gFrontLimitCm;
  gFrontLimitCm = 0;                       // it is meant to touch what is ahead
  float level[3] = {0, 0, 0};
  bool stripe = dockMode && !posts && P("stripe_on") > 0.5f;
  if (stripe && !learnStripe(level)) { gFrontLimitCm = saved; return false; }
  edge::lineOff = dockMode && !posts;    // a version 1 dock's stripe is not an edge (logged or not)
  float lx0, ly0, lth;
  alvik.get_pose(lx0, ly0, lth);
  ownPose(r.x0, r.y0, r.th0);
  {
    float d[5];
    alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
    r.front0 = d[2];
  }
  float creepRpm = P("dock_rpm"), press = P("dock_press_rpm"), pivot = P("dock_pivot_rpm");
  if (maxCm <= 0) maxCm = P("dock_max_cm");
  float sthr = P("stripe_thr"), sdir = P("stripe_dir");
  unsigned long pivotMs = (unsigned long)P("dock_pivot_ms");
  unsigned long limit = 5000 + (unsigned long)(maxCm / fmaxf(0.2f, creepRpm * 0.178f) * 2000.0f);   // rpm -> cm/s
  r.msStart = millis();
  float hFirst = 0, hStart = heading(), twistDeg = P("dock_twist_deg"), pivotMax = P("posts_pivot_max_deg");
  bool ok = false;
  int stripeN[3] = {0, 0, 0};
  while (true) {
    if (shouldAbort()) break;
    unsigned long now = millis();
    float x, y, t_;
    alvik.get_pose(x, y, t_);
    float travel = sqrtf((x - lx0) * (x - lx0) + (y - ly0) * (y - ly0));
    r.travel = travel;
    for (int k = 0; k < 2; k++)
      if (sw::firstEdge[k] && r.travelAt[k] == NONE) { r.travelAt[k] = travel; r.msPress[k] = sw::firstEdge[k]; }
    if (r.first < 0 && r.twistAt == NONE && fabsf(wrap180(heading() - hStart)) > twistDeg) r.twistAt = travel;
    if (stripe) {
      int L, C, R;
      alvik.get_line_sensors(L, C, R);
      int v[3] = {L, C, R};
      for (int k = 0; k < 3; k++) {
        if (r.stripe[k] != NONE) continue;
        if (edge::moved(v[k], level[k], sdir, sthr, 0)) { if (++stripeN[k] >= 2) r.stripe[k] = travel; }
        else stripeN[k] = 0;
      }
    }
    bool L = sw::left(), R = sw::right();
    if ((L || R) && r.first < 0) {
      r.first = L ? 0 : 1;
      r.msFirst = now;
      hFirst = heading();
      r.funnelDeg = wrap180(hFirst - hStart);
      if (!dockMode) { alvik.brake(); r.msBoth = sw::pressMs[r.first]; ok = true; break; }
    }
    if (dockMode && L && R) { alvik.brake(); r.msBoth = now; ok = true; break; }
    if (travel > maxCm) { snprintf(r.why, sizeof(r.why), "no click within %.0f cm", maxCm); break; }
    if (r.first >= 0 && now - r.msFirst > pivotMs) {
      snprintf(r.why, sizeof(r.why), "%s switch never clicked", r.first == 0 ? "right" : "left");
      break;
    }
    if (posts && r.first >= 0 && fabsf(wrap180(heading() - hFirst)) > pivotMax) {
      snprintf(r.why, sizeof(r.why), "miss %s", r.first == 0 ? "right" : "left");   // that lever passed its post
      break;
    }
    if (now - r.msStart > limit) { snprintf(r.why, sizeof(r.why), "creep timed out (stalled?)"); break; }
    if (!L && !R) alvik.set_wheels_speed(creepRpm, creepRpm);
    else if (L) alvik.set_wheels_speed(press, pivot);    // left corner on the wall: turn left about it
    else alvik.set_wheels_speed(pivot, press);           // right corner on the wall: turn right about it
    delay(5);
  }
  alvik.brake();
  if (r.first >= 0 && dockMode) r.pivotDeg = wrap180(heading() - hFirst);
  if (ok && dockMode && !posts) {
    // seat: both wheels push gently so the face lies flush, then let go
    unsigned long t0 = millis();
    alvik.set_wheels_speed(press, press);
    while (millis() - t0 < (unsigned long)P("dock_seat_ms")) {
      if (shouldAbort()) { ok = false; break; }
      delay(5);
    }
    alvik.brake();
    if (ok && !waitMs(200)) ok = false;
    if (ok && !(sw::left() && sw::right())) {
      snprintf(r.why, sizeof(r.why), "lost contact after seating (L %d R %d)", sw::left(), sw::right());
      ok = false;
    }
  }
  edge::lineOff = false;
  gFrontLimitCm = saved;
  ownPose(r.x, r.y, r.th);
  if (!ok && !gAbortReason.length() && !gStepStop.length() && r.why[0]) gFail = r.why;
  r.ok = ok;
  return ok;
}

// Stand still dock_still_ms at the docked heading: the gyro heading's drift over that time (deg/s), and the
// mean raw gyro z reading (motor board units) for the laptop. Returns false on a stop.
bool holdStill(float& drift, float& gzMean, unsigned long& stillMs) {
  unsigned long ms = (unsigned long)P("dock_still_ms");
  drift = 0; gzMean = 0; stillMs = 0;
  if (ms == 0) return true;
  float h0 = heading(), sum = 0;
  int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (shouldAbort()) return false;
    float ax, ay, az, gx, gy, gz;
    alvik.get_imu(ax, ay, az, gx, gy, gz);
    sum += gz;
    n++;
    delay(10);
  }
  stillMs = millis() - t0;
  drift = gYawSign != 0 ? wrap180(heading() - h0) / (stillMs / 1000.0f) : 0;
  gzMean = n ? sum / n : 0;
  return true;
}

void dockHeader() {
  net::sendLine("H,DOCK,run,ms_start,ms_first,ms_left,ms_right,ms_both,first,result,travel_cm,travel_left_cm,"
                "travel_right_cm,pivot_deg,stripe_l_cm,stripe_c_cm,stripe_r_cm,frame,x0_cm,y0_cm,th0_deg,x_cm,y_cm,"
                "th_deg,still_ms,drift_dps,gz_mean,drift_used,n,funnel_deg,twist_at_cm");
}

void dockRecord(const Result& r, unsigned long stillMs, float drift, float gz, int used, int n) {
  net::logf("D,DOCK,%s,%lu,%lu,%lu,%lu,%lu,%s,%s,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%lu,%.4f,%.4f,%d,%d,%.2f,%.3f",
            gRunId.c_str(), r.msStart, r.msFirst, r.msPress[0], r.msPress[1], r.msBoth,
            r.first == 0 ? "L" : r.first == 1 ? "R" : "-", r.ok ? "docked" : r.why, r.travel, r.travelAt[0],
            r.travelAt[1], r.pivotDeg, r.stripe[0], r.stripe[1], r.stripe[2], (pose::on && pose::known) ? "floor" : "odo",
            r.x0, r.y0, r.th0, r.x, r.y, r.th, stillMs, drift, gz, used, n, r.funnelDeg, r.twistAt);
}

void touchHeader() {
  net::sendLine("H,TOUCH,run,ms_start,ms_click,side,result,travel_cm,travel_left_cm,travel_right_cm,frame,x0_cm,"
                "y0_cm,th0_deg,x_cm,y_cm,th_deg,front0_cm,n");
}

void touchRecord(const Result& r, int n) {
  const char* side = r.first < 0 ? "-" : (sw::left() && sw::right()) ? "LR" : r.first == 0 ? "L" : "R";
  net::logf("D,TOUCH,%s,%lu,%lu,%s,%s,%.3f,%.3f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d", gRunId.c_str(),
            r.msStart, r.msBoth, side, r.ok ? "touched" : r.why, r.first >= 0 ? r.travelAt[r.first] : r.travel,
            r.travelAt[0], r.travelAt[1], (pose::on && pose::known) ? "floor" : "odo", r.x0, r.y0, r.th0, r.x, r.y,
            r.th, r.front0, n);
}

void postsHeader() {
  net::sendLine("H,POSTS,run,ms_start,ms_first,ms_left,ms_right,first,result,travel_first_cm,travel_left_cm,"
                "travel_right_cm,pivot_deg,approach_deg,frame,x0_cm,y0_cm,th0_deg,x_cm,y_cm,th_deg,front0_cm,n");
}

// result: "both" (each lever clicked on its post), "miss left"/"miss right" (that lever passed beside its post),
// or why it stopped. pivot_deg: the gyro's turn from the first click to the second = the angle it came in at
// (sign: + = it turned left to bring the right lever on). approach_deg: heading change before the first click.
void postsRecord(const Result& r, int n) {
  net::logf("D,POSTS,%s,%lu,%lu,%lu,%lu,%s,%s,%.3f,%.3f,%.3f,%.2f,%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d",
            gRunId.c_str(), r.msStart, r.msFirst, r.msPress[0], r.msPress[1],
            r.first == 0 ? "L" : r.first == 1 ? "R" : "-", r.ok ? "both" : r.why,
            r.first >= 0 ? r.travelAt[r.first] : r.travel, r.travelAt[0], r.travelAt[1], r.pivotDeg, r.funnelDeg,
            (pose::on && pose::known) ? "floor" : "odo", r.x0, r.y0, r.th0, r.x, r.y, r.th, r.front0, n);
}

// Dock, measure the drift, and (follow mode) reset the own pose from the dock's pose. wallX/wallY: the middle
// of the back wall's inner face, h: the way a docked robot faces (floor frame); NAN = not known (dock run).
bool dockOnce(int n, float wallX, float wallY, float h, Result& r, float maxCm = -1) {
  bool ok = creep(true, r, maxCm);
  float drift = 0, gz = 0;
  unsigned long stillMs = 0;
  int used = 0;
  if (ok) {
    if (pose::on && !isNaN(wallX) && !isNaN(wallY) && !isNaN(h)) {
      float a = P("bump_ahead_cm");
      pose::set(wallX - a * cosf(h * pose::DEG2RAD), wallY - a * sinf(h * pose::DEG2RAD), h, 0.05f, 0.3f);
    }
    ok = holdStill(drift, gz, stillMs);
    if (ok && gYawSign != 0 && fabsf(drift) <= P("dock_drift_max_dps")) {
      used = 1;
      if (pose::on) pose::driftDps = drift;    // what is left after gyro_drift_dps (corr.h): heading() already takes that off
      if (pose::on && !isNaN(h)) pose::set(pose::x, pose::y, h, 0.05f, 0.3f);   // still docked: heading = h exactly
    }
  }
  dockRecord(r, stillMs, drift, gz, used, n);
  return ok;
}

}  // namespace dock

// ===================================================================
// DOCK run (`run dock`): put the robot down 5-25 cm in front of the dock
// mouth, roughly facing in. dock_n dockings, each followed by backing out
// dock_back_cm (0: one docking, stays docked so it can be tape-measured).
// ===================================================================
void runDock() {
  dock::dockHeader();
  int n = (int)fmaxf(1.0f, P("dock_n"));
  float back = P("dock_back_cm");
  if (back <= 0) n = 1;
  gFrontLimitCm = 0;                        // the whole run faces the dock's wall (runExperiment restores it)
  for (int i = 1; i <= n; i++) {
    rawlog::step = i;
    dock::Result r;
    if (!dock::dockOnce(i, NAN, NAN, NAN, r)) return;
    if (back > 0 && !(libraryMove(-back) && waitMs(500))) return;
  }
  net::logf("M,%s dock: %d docking%s done", net::name, n, n == 1 ? "" : "s");
}

// ===================================================================
// TOUCH run (`run touch`): put the robot square to a board a few cm away.
// touch_n touches, backing off touch_back_cm after each.
// ===================================================================
void runTouch() {
  dock::touchHeader();
  int n = (int)fmaxf(1.0f, P("touch_n"));
  float back = P("touch_back_cm");
  gFrontLimitCm = 0;                        // the whole run faces the board it touches (runExperiment restores it)
  for (int i = 1; i <= n; i++) {
    rawlog::step = i;
    dock::Result r;
    bool ok = dock::creep(false, r);
    dock::touchRecord(r, i);
    if (!ok) return;
    if (!waitMs(300)) return;
    if (back > 0 && !(libraryMove(-back) && waitMs(500))) return;
  }
}

// ===================================================================
// POSTS run (`run posts`): put the robot 2-8 cm in front of a post
// station on dock version 2, roughly facing it (posts_max_cm of creep:
// if neither lever clicks by then, both posts met the bracket's face). posts_n measurements,
// backing off posts_back_cm after each. A miss is a result, not a failure:
// the run backs off and carries on.
// ===================================================================
void runPosts() {
  dock::postsHeader();
  int n = (int)fmaxf(1.0f, P("posts_n"));
  float back = P("posts_back_cm");
  gFrontLimitCm = 0;                        // the whole run faces the station (runExperiment restores it)
  for (int i = 1; i <= n; i++) {
    rawlog::step = i;
    dock::Result r;
    bool ok = dock::creep(true, r, P("posts_max_cm"), true);
    // neither lever clicked within posts_max_cm: both posts met the bracket's face beside the levers (off
    // sideways by more than the window both ways at once is impossible, so: off by more than a lever's length)
    if (!ok && r.first < 0 && strncmp(r.why, "no click", 8) == 0) snprintf(r.why, sizeof(r.why), "miss both");
    dock::postsRecord(r, i);
    bool miss = !ok && strncmp(r.why, "miss", 4) == 0;
    if (!ok && !miss) return;
    if (miss) { gFail = ""; }
    if (!waitMs(300)) return;
    if (back > 0 && !(libraryMove(-back) && waitMs(500))) return;
  }
  net::logf("M,%s posts: %d measurement%s done", net::name, n, n == 1 ? "" : "s");
}
