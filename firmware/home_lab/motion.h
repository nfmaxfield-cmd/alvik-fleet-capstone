// Shared robot helpers: heading, turns, line following, safety.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"
#include "rawlog.h"
#include "edge.h"

extern Arduino_Alvik alvik;

// Defined in home_lab.ino. Returns true if the command means "stop now".
bool handleCommand(const String& cmd, bool running);

// ---------- abort / safety ----------

String gAbortReason = "";
String gFail = "";   // why a run failed without being stopped (a timeout, a failed check); shown in RUNEND

// Survey and follow only (both off otherwise, so the other experiments behave as before):
// gTickHook runs on every shouldAbort() call (the own-pose tracker rides along like the RAW log);
// gFrontLimitCm >= 0 replaces safe_front_cm for the current step (0 = no front check);
// gFrontSoft: a front trip ends only the current step (gStepStop = "obstacle"), not the run;
// gStepStop: why the current step must stop (obstacle, halted); the run goes on.
void (*gTickHook)() = nullptr;
float gFrontLimitCm = -1;
bool gFrontSoft = false;
bool gEdgeSoft = false;   // follow: an edge-guard trip ends only the current step (gStepStop = "edge"), not the
                          // run: the robot holds and the laptop maps the edge and replans (run_plan.py)
String gStepStop = "";
bool gFrontPath = false;  // straight drives: check all five zones inside the robot's path, not just the centre
float gFrontEndBase = -1; // follow drives (>= 0): gFrontLimitCm = min(this, distance left in the step + front_end_cm)
float gFrontTripCm = 0;   // the reading that tripped it
unsigned long gFrontLowSince = 0;   // soft mode: when the reading first went below the limit (0 = it isn't)

// Nearest thing ahead INSIDE the robot's own path: each of the five front
// zones' reading taken at the zone's direction (ZONE_DEG, an assumption to be
// checked: the 60 deg view split in five), kept if it falls within
// path_half_cm of the robot's centre line; returns its distance straight
// ahead of the sensor (0 = nothing in the path). Something the robot's side
// would clip is only in view while it is still several cm ahead (at 4.5 cm to
// the side it leaves the 30 deg half-view below about 8 cm), so the limit for
// this check must look further ahead than the centre-only safe_front_cm.
float frontInPath() {
  static const float ZONE_DEG[5] = {22.0f, 11.0f, 0.0f, -11.0f, -22.0f};
  float d[5];
  alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
  float half = P("path_half_cm"), best = 0;
  for (int k = 0; k < 5; k++) {
    if (d[k] <= 0) continue;
    float a = ZONE_DEG[k] * 0.017453293f;
    float lat = d[k] * sinf(a), fwd = d[k] * cosf(a);
    if (fabsf(lat) <= half && (best == 0 || fwd < best)) best = fwd;
  }
  return best;
}

// Call often inside every loop. True means stop the experiment (or, with
// gStepStop set, just the current step).
bool shouldAbort() {
  rawlog::tick();   // full sensor log rides along with every motion loop
  if (gTickHook) gTickHook();
  if (gAbortReason.length()) return true;
  if (alvik.get_touch_cancel()) { gAbortReason = "cancel button"; }
  else if (P("safe_lift") > 0 && alvik.get_lifted()) { gAbortReason = "robot lifted"; }
  // open plywood edge (edge.h; safe_edge 0 = off): forward driving only, backing up is blind;
  // a hard stop in every experiment, follow steps and approaches included (gFrontLimitCm doesn't apply);
  // not a failure: RUNEND says "limited:plywood edge" (runExperiment), the calibration rolls back (runCalib)
  else if (edge::check()) { if (gEdgeSoft) gStepStop = "edge"; else gAbortReason = "plywood edge ahead"; }
  else if (!gStepStop.length()) {
    float lim = gFrontLimitCm >= 0 ? gFrontLimitCm : P("safe_front_cm");
    if (lim > 0) {
      float dL, dCL, dC, dCR, dR;
      alvik.get_distance(dL, dCL, dC, dCR, dR);
      if (gFrontPath) dC = frontInPath();
      if (dC > 0 && dC < lim) {
        if (!gFrontSoft) gAbortReason = "obstacle ahead";
        else {
          // soft mode (survey, follow): one noisy frame is not enough; it must stay too close
          if (!gFrontLowSince) gFrontLowSince = millis() | 1;
          if (millis() - gFrontLowSince >= (unsigned long)P("front_confirm_ms")) { gStepStop = "obstacle"; gFrontTripCm = dC; }
        }
      } else {
        gFrontLowSince = 0;
      }
    }
  }
  String cmd;
  if (net::poll(cmd) && handleCommand(cmd, true)) gAbortReason = "stop command";
  if (gAbortReason.length() || gStepStop.length()) {
    alvik.brake();
    return true;
  }
  return false;
}

// Wait without going deaf to stop commands. Returns false if aborted.
bool waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (shouldAbort()) return false;
    delay(5);
  }
  return true;
}

// ---------- heading ----------

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// +1 or -1 so that heading() grows when the robot turns left (CCW from above).
// Measured at the start of each experiment instead of assumed, because the
// IMU's sign convention isn't documented and older code disagrees about it.
int gYawSign = 0;

float rawYaw() {
  float roll, pitch, yaw;
  alvik.get_orientation(roll, pitch, yaw);
  return yaw;
}

// Corrected heading (corr.h): the library's yaw, unwrapped, times gyro_scale, minus gyro_drift_dps x time.
//   gyro_scale: real turning / what the gyro counts (1.075 = the gyro misses 7.5% of every turn)
//   gyro_drift_dps: how fast the gyro's heading creeps while the robot stands still (deg/s, left +)
// Both are 1 and 0 out of the box (`corr reset`). yawTrack() runs on every heading() call and every pass
// of loop(), so the unwrapping never misses half a turn (a robot turned by hand while idle included).
float gYawRawAcc = 0;          // library yaw since power-on, unwrapped, raw sign (deg)
float gYawDriftAcc = 0;        // gyro_drift_dps integrated since power-on (deg, left +)
float gYawPrevRaw = 0;
unsigned long gYawPrevMs = 0;
bool gYawInit = false;

void yawTrack() {
  float r = rawYaw();
  unsigned long now = millis();
  if (!gYawInit) { gYawPrevRaw = r; gYawPrevMs = now; gYawInit = true; return; }
  gYawRawAcc += wrap180(r - gYawPrevRaw);
  gYawDriftAcc += P("gyro_drift_dps") * (now - gYawPrevMs) / 1000.0f;
  gYawPrevRaw = r;
  gYawPrevMs = now;
}

// The gyro's own heading, left-positive, with only the sign applied (no scale or drift correction):
// for measuring the corrections themselves (runGyro).
float rawHeadingAcc() { yawTrack(); return gYawSign * gYawRawAcc; }

float heading() {
  yawTrack();
  return wrap180(gYawSign * P("gyro_scale") * gYawRawAcc - gYawDriftAcc);
}

// Heading from wheel odometry (get_pose theta), same left-positive convention.
int gOdoSign = 1;
float rawOdo() {
  float x, y, th;
  alvik.get_pose(x, y, th);
  return th;
}
float odoHeading() { return wrap180(gOdoSign * rawOdo()); }

// Wait while logging (and blinking), with no safety checks: for short timed moves.
void tickWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { rawlog::tick(); delay(5); }
}

// The sign check moves the robot (a timed wiggle whose undo is not exact),
// so it runs once per power-on. For tape-measured experiments, send
// `calibrate` first, THEN place the robot on its start mark.
bool calibrateYawSign(bool force = false) {
  if (gYawSign != 0 && !force) return true;
  net::sendLine("M,checking compass direction: small wiggle left and back");
  float y0 = rawYaw(), o0 = rawOdo();
  alvik.set_wheels_speed(-20, 20);  // left wheel back, right forward = turn left
  tickWait(300);
  alvik.brake();
  tickWait(300);
  float dy = wrap180(rawYaw() - y0);
  float dOdo = wrap180(rawOdo() - o0);
  if (fabsf(dOdo) >= 2.0f) gOdoSign = dOdo > 0 ? 1 : -1;
  if (fabs(dy) < 2.0f) {
    net::logf("M,yaw check failed: heading moved only %.1f deg", dy);
    gFail = "yaw check failed";
    return false;
  }
  gYawSign = dy > 0 ? 1 : -1;
  net::logf("M,yaw check ok: left turn moved raw yaw %.1f deg, sign %d", dy, gYawSign);
  // Undo the nudge so the robot faces where you put it.
  alvik.set_wheels_speed(20, -20);
  tickWait(300);
  alvik.brake();
  tickWait(300);
  return true;
}

// Spin in place by `deg` (positive = left), counted by the gyro (heading()),
// the wheels at `rpm` opposite ways, then brake. Needs the compass sign
// (calibrateYawSign() first). Where it stops is not important (the camera and
// the log both measure what really happened), so it simply brakes about a
// degree early for the coast. The front distance check is off while it spins:
// it does not move forward, and a board inside safe_front_cm would otherwise
// stop it; so the RAW log carries the sensor's whole sweep across nearby boards.
// Non-blocking motor calls only, so the blink code keeps going.
bool spinBy(float deg, float rpm) {
  if (deg == 0) return true;
  if (gYawSign == 0) { gFail = "spin without a compass check"; return false; }
  int dir = deg > 0 ? 1 : -1;
  float target = fabsf(deg) - 1.0f;
  float dps = fmaxf(fabsf(rpm), 1.0f) * 2.27f;   // nominal: rpm x 6 deg/s x wheel radius / half track (1.7 / 4.5 cm)
  unsigned long limit = 3000 + (unsigned long)(fabsf(deg) / dps * 2000.0f);   // allows half speed
  float saved = gFrontLimitCm;
  gFrontLimitCm = 0;
  float turned = 0, prev = heading();
  unsigned long t0 = millis();
  bool ok = true;
  alvik.set_wheels_speed(-dir * fabsf(rpm), dir * fabsf(rpm));
  while (turned < target) {
    if (shouldAbort()) { ok = false; break; }
    if (millis() - t0 > limit) { gFail = "spin timed out"; ok = false; break; }
    float h = heading();
    turned += dir * wrap180(h - prev);
    prev = h;
    delay(5);
  }
  alvik.brake();
  gFrontLimitCm = saved;
  return ok;
}

// ---------- turns ----------

struct TurnResult {
  bool ok;
  unsigned long ms;            // time until the method said it was done
  unsigned long overhead_ms;   // library only: time inside rotate() before the turn command is sent
};

// Turn to an absolute heading using IMU feedback (left = positive).
// Speed shrinks as the error shrinks; done after turn_hold_ms inside tolerance.
TurnResult imuTurnTo(float target) {
  unsigned long t0 = millis(), inTolSince = 0;
  float kp = P("turn_kp"), vmax = P("turn_max_rpm"), vmin = P("turn_min_rpm");
  float tol = P("turn_tol_deg");
  while (true) {
    if (shouldAbort()) return {false, millis() - t0, 0};
    if (millis() - t0 > (unsigned long)P("turn_timeout_ms")) {
      alvik.brake();
      net::logf("M,turn timed out");
      gFail = "turn timed out";
      return {false, millis() - t0, 0};
    }
    float err = wrap180(target - heading());
    if (fabs(err) <= tol) {
      alvik.brake();
      if (!inTolSince) inTolSince = millis();
      if (millis() - inTolSince >= (unsigned long)P("turn_hold_ms")) {
        return {true, millis() - t0, 0};
      }
    } else {
      inTolSince = 0;
      float v = constrain(kp * fabs(err), vmin, vmax);
      if (err > 0) alvik.set_wheels_speed(-v, v);   // turn left
      else         alvik.set_wheels_speed(v, -v);   // turn right
    }
    delay(5);
  }
}

// Turn by `delta` degrees with the library's own rotate() (left = positive).
// Non-blocking on purpose: the blocking version computes its wait from
// round(angle/100) seconds, which goes negative for right turns of 50 deg or
// more and (by our reading of the library) never returns.
TurnResult libraryTurn(float delta) {
  unsigned long t0 = millis();
  rawlog::tick();   // rotate() blocks ~200 ms inside the library: log right before it
  blink::hold();
  alvik.rotate(delta, DEG, false);   // the library waits 200 ms in here first
  unsigned long overhead = millis() - t0;
  while (!alvik.is_target_reached()) {
    if (shouldAbort()) return {false, millis() - t0, 0};
    if (millis() - t0 > (unsigned long)P("turn_timeout_ms")) {
      alvik.brake();
      net::logf("M,library turn timed out");
      gFail = "turn timed out";
      return {false, millis() - t0, 0};
    }
    delay(10);
  }
  // Note: the successful is_target_reached() call itself adds a 100 ms delay.
  return {true, millis() - t0, overhead};
}

// Drive straight `cm` (negative = backward) with the library's closed-loop
// move(). Non-blocking for the same reason as libraryTurn: the blocking
// version never returns for moves of about -50 cm or more.
// cm is real floor distance: the library is asked for cm / wheel_scale (corr.h).
bool libraryMove(float cm) {
  unsigned long t0 = millis();
  unsigned long limit = 3000 + (unsigned long)(fabsf(cm) * 400);   // generous: ~2.5 cm/s minimum
  rawlog::tick();   // move() blocks ~200 ms inside the library: log right before it
  blink::hold();
  alvik.move(cm / P("wheel_scale"), CM, false);   // wheel_scale (corr.h): real cm per wheel-counted cm
  while (!alvik.is_target_reached()) {
    if (shouldAbort()) return false;
    if (millis() - t0 > limit) {
      alvik.brake();
      gAbortReason = "move timed out";
      return false;
    }
    delay(10);
  }
  return true;
}

// ---------- line following ----------

struct LineState {
  float err = 0, prevErr = 0, corr = 0;
  bool onLine = false;
  unsigned long lastSeen = 0, lastStep = 0;
  int L = 0, C = 0, R = 0;
};

// Same centroid as the library's line_follower example:
// 0 = centered, positive = tape is to the robot's left.
float lineError(int l, int c, int r) {
  float sum = l + c + r;
  if (sum <= 0) return 0;
  return -((l + 2.0f * c + 3.0f * r) / sum) + 2.0f;
}

void lineBegin(LineState& s) {
  s = LineState();
  s.lastSeen = s.lastStep = millis();
  // Seed the error from the first reading so the first derivative isn't a spike.
  alvik.get_line_sensors(s.L, s.C, s.R);
  s.err = s.prevErr = lineError(s.L, s.C, s.R);
}

// One control step. Returns false if the tape has been lost too long.
bool lineStep(LineState& s, float base) {
  unsigned long now = millis();
  float dt = (now - s.lastStep) / 1000.0f;
  s.lastStep = now;
  alvik.get_line_sensors(s.L, s.C, s.R);
  float thr = P("lf_line_thr");
  s.onLine = (s.L > thr || s.C > thr || s.R > thr);
  if (s.onLine) {
    s.lastSeen = now;
    s.err = lineError(s.L, s.C, s.R);
  } else if (now - s.lastSeen > (unsigned long)P("lf_lost_ms")) {
    alvik.brake();
    return false;
  }
  // Off tape briefly (e.g. over a sticker): keep the last error.
  float d = dt > 0 ? (s.err - s.prevErr) / dt : 0;
  s.prevErr = s.err;
  float m = P("lf_max_corr");
  s.corr = constrain(P("lf_kp") * s.err + P("lf_kd") * d, -m, m);
  float left = constrain(base - s.corr, -70.0f, 70.0f);
  float right = constrain(base + s.corr, -70.0f, 70.0f);
  alvik.set_wheels_speed(left, right);
  return true;
}

// ---------- odometry ----------

void poseXY(float& x, float& y) {
  float th;
  alvik.get_pose(x, y, th);
}

float distFrom(float x0, float y0) {
  float x, y;
  poseXY(x, y);
  return sqrtf((x - x0) * (x - x0) + (y - y0) * (y - y0));
}

// Drive straight (equal wheel speeds) for `cm`. Used to put the wheel axle
// over a crossing, where all three line sensors see tape and can't steer.
bool rollStraight(float cm, float rpm) {
  float x0, y0;
  poseXY(x0, y0);
  cm /= P("wheel_scale");   // the library's odometry counts real cm / wheel_scale (corr.h)
  unsigned long t0 = millis();
  alvik.set_wheels_speed(rpm, rpm);
  while (distFrom(x0, y0) < cm) {
    if (shouldAbort()) return false;
    if (millis() - t0 > 3000 + (unsigned long)(cm * 600)) {   // ~1.7 cm/s minimum
      alvik.brake();
      gAbortReason = "wheels did not cover the roll distance (stalled?)";
      return false;
    }
    delay(5);
  }
  return true;
}
