// The four experiments. Each one logs H (header) and D (data) lines that the
// laptop console saves to CSV, one file per record type.
#pragma once

#include "motion.h"

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
    if (!lineStep(s, base)) { result = "lost_tape"; break; }
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
