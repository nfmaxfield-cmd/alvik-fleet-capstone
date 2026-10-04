// Shared robot helpers: heading, turns, line following, safety.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

extern Arduino_Alvik alvik;

// Defined in home_lab.ino. Returns true if the command means "stop now".
bool handleCommand(const String& cmd, bool running);

// ---------- abort / safety ----------

String gAbortReason = "";

// Call often inside every loop. True means stop the experiment.
bool shouldAbort() {
  if (gAbortReason.length()) return true;
  if (alvik.get_touch_cancel()) { gAbortReason = "cancel button"; }
  else if (P("safe_lift") > 0 && alvik.get_lifted()) { gAbortReason = "robot lifted"; }
  else {
    float lim = P("safe_front_cm");
    if (lim > 0) {
      float dL, dCL, dC, dCR, dR;
      alvik.get_distance(dL, dCL, dC, dCR, dR);
      if (dC > 0 && dC < lim) gAbortReason = "obstacle ahead";
    }
  }
  String cmd;
  if (net::poll(cmd) && handleCommand(cmd, true)) gAbortReason = "stop command";
  if (gAbortReason.length()) {
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

float heading() { return wrap180(gYawSign * rawYaw()); }

// Heading from wheel odometry (get_pose theta), same left-positive convention.
int gOdoSign = 1;
float rawOdo() {
  float x, y, th;
  alvik.get_pose(x, y, th);
  return th;
}
float odoHeading() { return wrap180(gOdoSign * rawOdo()); }

bool calibrateYawSign() {
  net::sendLine("M,checking compass direction: small wiggle left and back");
  float y0 = rawYaw(), o0 = rawOdo();
  alvik.set_wheels_speed(-20, 20);  // left wheel back, right forward = turn left
  delay(300);
  alvik.brake();
  delay(300);
  float dy = wrap180(rawYaw() - y0);
  float dOdo = wrap180(rawOdo() - o0);
  if (fabsf(dOdo) >= 2.0f) gOdoSign = dOdo > 0 ? 1 : -1;
  if (fabs(dy) < 2.0f) {
    net::logf("M,yaw check failed: heading moved only %.1f deg", dy);
    return false;
  }
  gYawSign = dy > 0 ? 1 : -1;
  net::logf("M,yaw check ok: left turn moved raw yaw %.1f deg, sign %d", dy, gYawSign);
  // Undo the nudge so the robot faces where you put it.
  alvik.set_wheels_speed(20, -20);
  delay(300);
  alvik.brake();
  delay(300);
  return true;
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
  alvik.rotate(delta, DEG, false);   // the library waits 200 ms in here first
  unsigned long overhead = millis() - t0;
  while (!alvik.is_target_reached()) {
    if (shouldAbort()) return {false, millis() - t0, 0};
    if (millis() - t0 > (unsigned long)P("turn_timeout_ms")) {
      alvik.brake();
      net::logf("M,library turn timed out");
      return {false, millis() - t0, 0};
    }
    delay(10);
  }
  // Note: the successful is_target_reached() call itself adds a 100 ms delay.
  return {true, millis() - t0, overhead};
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
  unsigned long t0 = millis();
  alvik.set_wheels_speed(rpm, rpm);
  while (distFrom(x0, y0) < cm) {
    if (shouldAbort()) return false;
    if (millis() - t0 > 5000) {
      alvik.brake();
      gAbortReason = "wheels did not cover the roll distance (stalled?)";
      return false;
    }
    delay(5);
  }
  return true;
}
