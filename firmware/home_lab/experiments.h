// The experiments. Each one logs H (header) and D (data) lines that the
// laptop console saves to CSV, one file per record type.
#pragma once

#include "motion.h"
#include "pose.h"

int gRun = 0;          // increases every time any experiment starts
String gRunId = "";    // boot id + run number, e.g. "3F2A-4", unique across reboots

// ===================================================================
// STREAM: send every sensor reading, for exploring and checking WiFi.
// ===================================================================
void runStream() {
  net::sendLine("H,STREAM,run,ms,L,C,R,h,s,v,label,dL,dCL,dC,dCR,dR,top,bottom,roll,pitch,yaw,battery");
  unsigned long period = (unsigned long)(1000.0f / fmaxf(1.0f, P("stream_hz")));
  unsigned long next = millis();
  while (!shouldAbort()) {
    if (millis() < next) { delay(1); continue; }
    next += period;
    int l, c, r;
    alvik.get_line_sensors(l, c, r);
    float h, s, v;
    alvik.get_color(h, s, v, HSV);
    String label = alvik.get_color_label(h, s, v);
    float dL, dCL, dC, dCR, dR;
    alvik.get_distance(dL, dCL, dC, dCR, dR);
    float roll, pitch, yaw;
    alvik.get_orientation(roll, pitch, yaw);
    net::logf("D,STREAM,%s,%lu,%d,%d,%d,%.1f,%.3f,%.3f,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d",
              gRunId.c_str(), millis(), l, c, r, h, s, v, label.c_str(), dL, dCL, dC, dCR, dR,
              alvik.get_distance_top(), alvik.get_distance_bottom(),
              roll, pitch, yaw, alvik.get_battery_charge());
  }
}

// ===================================================================
// TURN: repeat in-place turns, log accuracy and time for each.
//   err_rel: how far this turn missed its own size (turn quality)
//   err_abs: how far the heading is from where it should be overall (drift)
// The IMU method aims at the absolute heading each time, so it corrects
// earlier misses; the library method turns relative to wherever it is.
// ===================================================================
void runTurn() {
  if (!calibrateYawSign()) return;
  net::sendLine("H,TURN,run,trial,method,ok,cmd_deg,dyaw_deg,err_rel_deg,err_abs_deg,odo_dyaw_deg,t_done_ms,lib_overhead_ms,t_total_ms,max_rpm,min_rpm,kp,tol_deg,battery");
  int n = (int)P("turn_n"), method = (int)P("turn_method");
  float size = P("turn_deg");
  bool alt = P("turn_alt") > 0;
  float start = heading(), goal = start;
  float sumAbsErr = 0, sumMs = 0;
  int done = 0;
  for (int i = 1; i <= n; i++) {
    float cmd = (alt && i % 2 == 0) ? -size : size;
    goal = wrap180(goal + cmd);
    float before = heading(), odoBefore = odoHeading();
    unsigned long t0 = millis();
    TurnResult tr = method == 0 ? libraryTurn(cmd) : imuTurnTo(goal);
    if (!tr.ok && gAbortReason.length()) break;
    if (!waitMs((unsigned long)P("settle_ms"))) break;
    unsigned long total = millis() - t0;
    float after = heading();
    float dyaw = wrap180(after - before);
    float errRel = wrap180(dyaw - cmd);
    float errAbs = wrap180(after - goal);
    // Wheel-odometry heading change: a second opinion that doesn't come
    // from the IMU, so an IMU scale error can't hide.
    float odoDyaw = wrap180(odoHeading() - odoBefore);
    net::logf("D,TURN,%s,%d,%s,%d,%.1f,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%.0f,%.0f,%.2f,%.2f,%d",
              gRunId.c_str(), i, method == 0 ? "library" : "imu", tr.ok ? 1 : 0, cmd, dyaw, errRel, errAbs,
              odoDyaw, tr.ms, tr.overhead_ms, total, P("turn_max_rpm"), P("turn_min_rpm"), P("turn_kp"),
              P("turn_tol_deg"), alvik.get_battery_charge());
    if (tr.ok) {
      sumAbsErr += fabsf(errRel);
      sumMs += tr.ms;
      done++;
    }
    if (!waitMs((unsigned long)P("turn_gap_ms"))) break;
  }
  if (done) net::logf("M,turn run %s: %d good turns, mean |error| %.2f deg, mean time %.0f ms",
                      gRunId.c_str(), done, sumAbsErr / done, sumMs / done);
  net::sendLine("M,check by eye too: with turn_n a multiple of 4 the robot should end facing where it started");
}

// ===================================================================
// LAP: follow a taped loop, time each lap between marker-sticker crossings.
// Put the robot on the tape anywhere; timing starts at the first marker.
// ===================================================================
bool markerSeen() {
  float h, s, v;
  alvik.get_color(h, s, v, HSV);
  float lo = P("mk_h_lo"), hi = P("mk_h_hi");
  bool hueOk = lo <= hi ? (h >= lo && h <= hi) : (h >= lo || h <= hi);  // window may wrap past 0
  return hueOk && s >= P("mk_s_min") && v >= P("mk_v_min");
}

void runLap() {
  net::sendLine("H,LAP,run,lap,lap_ms,mean_abs_err,max_abs_err,max_corr,steps,base_rpm,kp,kd,battery");
  bool trace = P("lf_trace") > 0;
  if (trace) net::sendLine("H,TRACE,run,ms,L,C,R,err,corr,on_line,marker");
  LineState s;
  lineBegin(s);
  int lapsWanted = (int)P("lap_n"), lap = 0, streak = 0;
  unsigned long lapStart = 0, lastMarker = 0, lastTrace = 0;
  float sumAbs = 0, maxAbs = 0, maxCorr = 0;
  int steps = 0;
  bool timing = false;
  while (!shouldAbort()) {
    if (!lineStep(s, P("lf_base_rpm"))) {
      net::logf("M,lap run %s: lost the tape (lap %d)", gRunId.c_str(), lap + 1);
      gFail = "lost the tape";
      return;
    }
    bool mk = markerSeen();
    streak = mk ? streak + 1 : 0;
    unsigned long now = millis();
    if (streak == (int)P("mk_samples") && (lastMarker == 0 || now - lastMarker > (unsigned long)P("lap_min_ms"))) {
      lastMarker = now;
      if (timing) {
        lap++;
        net::logf("D,LAP,%s,%d,%lu,%.3f,%.3f,%.1f,%d,%.0f,%.2f,%.2f,%d",
                  gRunId.c_str(), lap, now - lapStart, steps ? sumAbs / steps : 0, maxAbs, maxCorr, steps,
                  P("lf_base_rpm"), P("lf_kp"), P("lf_kd"), alvik.get_battery_charge());
        if (lap >= lapsWanted) { alvik.brake(); break; }
      } else {
        timing = true;
        net::logf("M,lap run %s: marker found, timing starts", gRunId.c_str());
      }
      lapStart = now;
      sumAbs = maxAbs = maxCorr = 0;
      steps = 0;
    }
    if (timing) {
      sumAbs += fabs(s.err);
      maxAbs = fmaxf(maxAbs, fabsf(s.err));
      maxCorr = fmaxf(maxCorr, fabsf(s.corr));
      steps++;
    }
    if (trace && now - lastTrace >= 20) {
      lastTrace = now;
      net::logf("D,TRACE,%s,%lu,%d,%d,%d,%.3f,%.2f,%d,%d", gRunId.c_str(), now, s.L, s.C, s.R, s.err, s.corr, s.onLine, mk);
    }
    delay(10);
  }
  alvik.brake();
}

// ===================================================================
// GRID: drive a route across taped crossings, two ways.
//   grid_mode 0: stop at every crossing (like the lab navigator)
//   grid_mode 1: roll straight through crossings that need no turn
// Same route, both modes, several runs each -> the time difference is the
// value of drive-through.
// Start the robot on the tape, facing along it, before the first crossing.
// ===================================================================
bool crossingNow(const LineState& s) {
  float thr = P("lf_line_thr");
  return s.L > thr && s.C > thr && s.R > thr;
}

void runGrid() {
  if (!calibrateYawSign()) return;
  net::sendLine("H,GRID_NODE,run,mode,node,action,t_arrive_ms,t_leave_ms,stopped_ms");
  net::sendLine("H,GRID_RUN,run,mode,route,result,total_ms,nodes,turns,base_rpm,stop_ms,battery");
  int mode = (int)P("grid_mode");
  float base = P("lf_base_rpm");
  String route = gRoute;
  route.toUpperCase();
  float gridHeading = heading();
  LineState s;
  lineBegin(s);
  float nx = 0, ny = 0;
  poseXY(nx, ny);
  bool first = true;
  int node = 0, turns = 0, streak = 0;
  unsigned long t0 = millis();
  String result = "done";

  while (true) {
    if (shouldAbort()) { result = String("aborted:") + gAbortReason; break; }
    if (!lineStep(s, base)) { result = "lost_tape"; gFail = "lost the tape"; break; }
    bool armed = first || distFrom(nx, ny) > P("grid_ignore_cm");
    streak = (armed && crossingNow(s)) ? streak + 1 : 0;
    if (streak < (int)P("grid_cross_samples")) { delay(10); continue; }

    // ---- at a crossing ----
    streak = 0;
    first = false;
    char a = node < (int)route.length() ? route.charAt(node) : 'E';
    node++;
    unsigned long arrive = millis() - t0, stopped = 0;
    bool turn = (a == 'L' || a == 'R');
    bool mustStop = turn || a == 'E' || mode == 0;

    if (mustStop) {
      // Roll until the wheel axle sits on the crossing, then stop.
      if (!rollStraight(P("grid_center_cm"), base)) { result = String("aborted:") + gAbortReason; break; }
      alvik.brake();
      unsigned long ts = millis();
      if (a == 'E') {
        net::logf("D,GRID_NODE,%s,%d,%d,%c,%lu,%lu,0", gRunId.c_str(), mode, node, a, arrive, millis() - t0);
        break;
      }
      if (mode == 0 && !waitMs((unsigned long)P("grid_stop_ms"))) { result = String("aborted:") + gAbortReason; break; }
      if (turn) {
        gridHeading = wrap180(gridHeading + (a == 'L' ? 90.0f : -90.0f));
        TurnResult tr = imuTurnTo(gridHeading);
        if (!tr.ok) {
          result = "turn_failed";
          gFail = "turn failed";
          if (gAbortReason.length()) result = String("aborted:") + gAbortReason;
          break;
        }
        turns++;
      }
      stopped = millis() - ts;
      lineBegin(s);   // fresh steering state on the new leg
    }
    // A drive-through straight node just keeps following the line.
    poseXY(nx, ny);
    net::logf("D,GRID_NODE,%s,%d,%d,%c,%lu,%lu,%lu", gRunId.c_str(), mode, node, a, arrive, millis() - t0, stopped);
  }
  alvik.brake();
  net::logf("D,GRID_RUN,%s,%d,%s,%s,%lu,%d,%d,%.0f,%.0f,%d",
            gRunId.c_str(), mode, route.c_str(), result.c_str(), millis() - t0, node, turns,
            base, P("grid_stop_ms"), alvik.get_battery_charge());
}

// ===================================================================
// DEMO: open-floor routines, no tape. Which routine runs depends on the
// robot's saved name (set once from the console: `name Left`), unless
// demo_route picks one: 1 = the L route, 2 = the circle, 3 = a square
// (square_cm sides, square_laps laps, left turns), on any robot.
//   Left:   forward A, turn right, forward B, then retrace in reverse:
//           back B, turn back left, back A. Ends where it started,
//           facing the same way.
//   Center: forward A, one full circle of radius R, back A.
//   Right:  no routine yet.
// Odometry is zeroed at the start, so the final pose is the return error.
// ===================================================================
void demoLog(const char* step) {
  rawlog::step++;   // RAW records carry this count, so samples can be matched to route steps
  float x, y, th;
  alvik.get_pose(x, y, th);
  y *= gOdoSign; th *= gOdoSign;   // left-positive, like every other heading we log (RAW stays raw)
  net::logf("D,DEMO_STEP,%s,%s,%s,%.1f,%.1f,%.1f,%lu", gRunId.c_str(), net::name, step, x, y, th, millis());
}

// One full circle (left), stopping when the IMU says 360 deg have passed.
bool driveCircle(float radiusCm, float speedCms) {
  float wDeg = speedCms / radiusCm * 57.2958f;
  float prev = heading(), turned = 0;
  unsigned long t0 = millis();
  unsigned long limit = (unsigned long)(360.0f / wDeg * 1000.0f * 2.0f) + 3000;
  alvik.drive(speedCms, wDeg);
  while (fabsf(turned) < 360.0f - P("demo_circle_lead_deg")) {
    if (shouldAbort()) return false;
    if (millis() - t0 > limit) { alvik.brake(); gAbortReason = "circle timed out"; return false; }
    float h = heading();
    turned += wrap180(h - prev);
    prev = h;
    delay(5);
  }
  alvik.brake();
  return true;
}

void runDemo() {
  String who = String(net::name);
  int route = (int)P("demo_route");
  bool left = route == 1 || (route == 0 && who.equalsIgnoreCase("Left"));
  bool center = route == 2 || (route == 0 && who.equalsIgnoreCase("Center"));
  bool square = route == 3;
  if (!left && !center && !square) {
    net::logf("M,%s has no demo routine of its own. Pick one with: set demo_route 1 (L), 2 (circle) or 3 (square)", net::name);
    gFail = "no demo routine";
    return;
  }
  if (!calibrateYawSign()) return;
  net::sendLine("H,DEMO_STEP,run,robot,step,x_cm,y_cm,theta_deg,ms");
  net::sendLine("H,DEMO_END,run,robot,route,result,return_err_cm,heading_err_deg,total_ms,a_cm,b_cm,r_cm,battery");
  alvik.reset_pose(0, 0, 0);
  float start = heading();
  float a = P("demo_a_cm"), b = P("demo_b_cm"), r = P("demo_r_cm");
  unsigned long t0 = millis();
  bool ok = true;
  demoLog("start");

  if (square) {
    float side = P("square_cm");
    int laps = (int)P("square_laps");
    if (laps < 1) laps = 1;
    a = side; b = laps; r = 0;   // logged in DEMO_END's a_cm / b_cm columns: side and laps
    for (int k = 1; ok && k <= 4 * laps; k++) {
      ok = ok && libraryMove(side);                    if (ok) demoLog("side");
      ok = ok && imuTurnTo(wrap180(start + 90.0f * k)).ok; if (ok) demoLog("turn_left");
    }
  } else if (left) {
    ok = ok && libraryMove(a);                         if (ok) demoLog("forward_a");
    ok = ok && imuTurnTo(wrap180(start - 90)).ok;      if (ok) demoLog("turn_right");
    ok = ok && libraryMove(b);                         if (ok) demoLog("forward_b");
    ok = ok && waitMs(500);
    ok = ok && libraryMove(-b);                        if (ok) demoLog("back_b");
    ok = ok && imuTurnTo(start).ok;                    if (ok) demoLog("turn_back_left");
    ok = ok && libraryMove(-a);                        if (ok) demoLog("back_a");
  } else {
    ok = ok && libraryMove(a);                         if (ok) demoLog("forward_a");
    ok = ok && driveCircle(r, P("demo_speed_cms"));    if (ok) demoLog("circle");
    ok = ok && imuTurnTo(start).ok;                    if (ok) demoLog("square_up");
    ok = ok && libraryMove(-a);                        if (ok) demoLog("back_a");
  }
  alvik.brake();

  float x, y, th;
  alvik.get_pose(x, y, th);
  float err = sqrtf(x * x + y * y);
  float headErr = wrap180(heading() - start);
  String result = ok ? String("done")
                : gAbortReason.length() ? String("aborted:") + gAbortReason : String("failed:") + gFail;
  net::logf("D,DEMO_END,%s,%s,%s,%s,%.1f,%.1f,%lu,%.0f,%.0f,%.0f,%d", gRunId.c_str(), net::name,
            square ? "square" : (left ? "L" : "circle"), result.c_str(),
            err, headErr, millis() - t0, a, b, r, alvik.get_battery_charge());
  net::logf("M,%s demo %s: back within %.1f cm of start (wheel odometry), heading off by %.1f deg. Measure with a ruler too.",
            net::name, ok ? "finished" : "stopped", err, headErr);
}

// ===================================================================
// STRAIGHT (experiment E2): drive straight_cm with the library's move()
// and log where odometry and the IMU think the robot ended up. The true
// end point is measured on the floor with the tape rig; this record is
// the comparison. x is along the commanded line, y to the left.
// ===================================================================
void runStraight() {
  if (!calibrateYawSign()) return;
  net::sendLine("H,STRAIGHT,run,robot,result,cmd_cm,rpm,odo_x_cm,odo_y_cm,odo_theta_deg,imu_dyaw_deg,total_ms,battery");
  float d = P("straight_cm"), rpm = P("straight_rpm");
  alvik.reset_pose(0, 0, 0);
  float start = heading();
  unsigned long t0 = millis();
  bool ok = rpm > 0 ? rollStraight(d, rpm) : libraryMove(d);   // rpm 0 = library's own speed control
  alvik.brake();
  rawlog::step = 1;   // RAW records after this are the robot settling
  ok = ok && waitMs((unsigned long)P("settle_ms"));
  float x, y, th;
  alvik.get_pose(x, y, th);
  y *= gOdoSign; th *= gOdoSign;   // left-positive, like every other heading we log (RAW stays raw)
  float dyaw = wrap180(heading() - start);
  String result = ok ? String("done")
                : gAbortReason.length() ? String("aborted:") + gAbortReason : String("failed:") + gFail;
  net::logf("D,STRAIGHT,%s,%s,%s,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%lu,%d", gRunId.c_str(), net::name, result.c_str(),
            d, rpm, x, y, th, dyaw, millis() - t0, alvik.get_battery_charge());
  net::logf("M,run %s: odometry says %.1f cm along, %.1f cm sideways. Now mark and measure the real end point.",
            gRunId.c_str(), x, y);
}

// ===================================================================
// SURVEY: a square spiral outward from where the robot was put down, for the
// camera's self-calibration (tools/calibrate_floor.py --survey pairs the
// robot's own wheel + gyro path with the camera track). Legs of 10, 10, 20,
// 20, 30, 30 ... cm with a 90 deg left turn before each new leg, until the
// next leg would be longer than survey_size_cm. A leg ends early if the front
// sensor sees something closer than survey_clear_cm (logged); the spiral then
// stops. Finally it turns to face the start, drives straight back to it, and
// turns to its starting heading, all on its own pose (pose.h).
// Straight legs use driveLine() (gyro-held heading, steering back onto the
// line) rather than the library's move(): move() holds the wheel positions,
// so a left/right wheel size difference curves every leg; the gyro hold
// keeps the legs straight, and the same code is what the step follower uses.
// Turns are IMU turns (imuTurnTo), the most accurate in the turn test.
// RAW step = leg number (1, 2, ...); legs + 1 = the drive back.
// ===================================================================
void surveyLeg(int leg, float cmd, float driven, const char* how) {
  float dL, dCL, dC, dCR, dR;
  alvik.get_distance(dL, dCL, dC, dCR, dR);
  net::logf("D,SURVEY_LEG,%s,%d,%.0f,%.1f,%s,%.2f,%.2f,%.2f,%.1f,%lu", gRunId.c_str(), leg, cmd, driven, how,
            pose::x, pose::y, pose::th(), dC, millis());
}

void runSurvey() {
  if (!calibrateYawSign()) return;
  net::sendLine("H,SURVEY_LEG,run,leg,cmd_cm,driven_cm,end,x_cm,y_cm,th_deg,front_cm,ms");
  net::sendLine("H,SURVEY_END,run,result,legs,path_cm,return_cm,x_cm,y_cm,th_deg,total_ms,size_cm,battery");
  float size = P("survey_size_cm"), v = P("survey_cms");
  pose::start(true, 0, 0, 0);       // start frame: where it was put down, facing +x
  gTickHook = pose::tick;
  gFrontSoft = true;
  gFrontLimitCm = fmaxf(P("survey_clear_cm"), P("safe_front_cm"));
  gStepStop = "";
  unsigned long t0 = millis();
  float cx = 0, cy = 0, path = 0;
  int legs = 0;
  bool ok = true, blocked = false;
  for (int k = 1;; k++) {
    float L = 10.0f * ((k + 1) / 2);
    if (L > size) break;
    float h = 90.0f * (k - 1);
    rawlog::step = k;
    gFrontLimitCm = P("safe_front_cm");   // turning in place: only the usual centre stop (it may face what ended the last leg)
    if (k > 1 && !turnToFloor(wrap180(h))) {
      if (gStepStop.length()) { blocked = true; surveyLeg(k, L, 0, "obstacle"); }
      else ok = false;
      break;
    }
    float hx = cosf(h * pose::DEG2RAD), hy = sinf(h * pose::DEG2RAD);
    float d0 = pose::dist;
    gFrontLimitCm = fmaxf(P("survey_clear_cm"), P("safe_front_cm"));
    gFrontPath = true;     // legs: anything in the robot's path, all five zones
    int r = driveLine(cx, cy, cx + L * hx, cy + L * hy, wrap180(h), v, false);
    gFrontPath = false;
    float driven = pose::dist - d0;
    path += fabsf(driven);
    legs = k;
    if (r == DRIVE_STEPSTOP) { blocked = true; surveyLeg(k, L, driven, "obstacle"); break; }
    if (r != DRIVE_OK) { ok = false; surveyLeg(k, L, driven, r == DRIVE_ABORT ? "stopped" : "timeout"); break; }
    surveyLeg(k, L, driven, "full");
    cx += L * hx;
    cy += L * hy;
  }
  gStepStop = "";
  float back = 0;
  if (ok && !gAbortReason.length()) {
    // back to the start: face it, drive the distance, face the starting heading
    rawlog::step = legs + 1;
    float d = sqrtf(pose::x * pose::x + pose::y * pose::y);
    if (d > 1.0f) {
      float hb = atan2f(-pose::y, -pose::x) / pose::DEG2RAD;
      float sx = pose::x, sy = pose::y;
      gFrontLimitCm = P("safe_front_cm");
      ok = turnToFloor(hb);
      float d0 = pose::dist;
      gFrontLimitCm = fmaxf(P("survey_clear_cm"), P("safe_front_cm"));
      gFrontPath = true;
      if (ok) ok = driveLine(sx, sy, 0, 0, hb, v, false) == DRIVE_OK;
      gFrontPath = false;
      back = pose::dist - d0;
      path += fabsf(back);
      if (!ok && gStepStop.length()) gFail = "blocked on the way back";
    }
    gStepStop = "";
    gFrontLimitCm = P("safe_front_cm");
    if (ok) ok = turnToFloor(0);
  }
  alvik.brake();
  gTickHook = nullptr;
  gFrontSoft = false;
  gFrontLimitCm = -1;
  gStepStop = "";
  pose::stop();
  String result = ok ? String(blocked ? "done:blocked" : "done")
                : gAbortReason.length() ? String("aborted:") + gAbortReason : String("failed:") + gFail;
  net::logf("D,SURVEY_END,%s,%s,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%lu,%.0f,%d", gRunId.c_str(), result.c_str(), legs, path,
            back, pose::x, pose::y, pose::th(), millis() - t0, size, alvik.get_battery_charge());
  net::logf("M,%s survey %s: %d legs, %.0f cm driven, back within %.1f cm of the start (own pose)", net::name,
            result.c_str(), legs, path, sqrtf(pose::x * pose::x + pose::y * pose::y));
}
