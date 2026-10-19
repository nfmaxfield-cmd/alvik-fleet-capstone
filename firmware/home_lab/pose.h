// The robot's own pose, and straight drives that steer on it.
// Used by the survey drive and the step follower.
//
// Pose: x, y (cm) and heading (deg, left-positive). Survey: in its own start
// frame (0, 0, 0 where it was put down). Follow: in the floor frame of
// tools/fleet/map.json, set by the laptop (fpose / first camera fix).
//   - distance comes from the wheels: the library's odometry step projected on
//     the library's own heading (so it doesn't depend on which way the wheel
//     encoders or the library's y axis count, only on forward being +x);
//   - heading comes from the gyro: heading() + thOff, where thOff ties the
//     IMU's zero to the floor.
// Between camera fixes, errors grow; each pose keeps a variance var (cm^2 per
// axis) and varTh (deg^2) that grow with distance driven, degrees turned and
// time (pose_q_cm, pose_q_deg, pose_q_dps).
//
// Camera fixes (follow): the laptop sends "fix t_ms x y th sigma [sigma_th]"
// where t_ms is the ROBOT's millis() at the moment the camera saw that pose.
// Latency compensation: the robot keeps its pose every 40 ms for the last
// ~1.3 s, looks up where it thought it was at t_ms, and blends the difference
// in with a one-step Kalman (complementary) gain k = var / (var + sigma^2):
// the whole history and the current pose move by k * difference (heading:
// rotated about the pose at t_ms by k_th * heading difference). A fix older
// than fix_max_age_ms (1 s) or from the future (> 0.2 s ahead) is ignored, as
// is one further than fix_gate standard deviations (and 3 cm) from the own
// pose (a wrong detection). The first fix (pose not known yet) sets the pose.
#pragma once

#include "motion.h"

// NAN marks "not given" (a fix without a heading, a step without a front distance).
inline bool isNaN(float v) { return v != v; }

namespace pose {

const float DEG2RAD = 0.017453293f;
const int NH = 32;                 // pose history: 32 x 40 ms = 1.28 s
const unsigned long H_MS = 40;

struct Hist { unsigned long ms; float x, y, th; };

bool on = false;
bool known = false;
float x = 0, y = 0;
float thOff = 0;                   // floor heading = heading() + thOff
float var = 0, varTh = 0;          // variances: cm^2 (per axis), deg^2
float dist = 0;                    // signed wheel distance since start(), cm (forward +)
int fixesUsed = 0, fixesRejected = 0;
const char* lastReject = "";

Hist hist[NH];
int hHead = 0, hCount = 0;         // hist[hHead - 1] is the newest
unsigned long hLast = 0, lastMs = 0;
float lastLx = 0, lastLy = 0, lastTh = 0;
float qCm = 0.01f, qDeg = 0.003f, qDps = 0.002f;
float driftDps = 0;                // gyro heading drift measured while docked (dock.h); taken off th() as time passes

float th() { return wrap180(heading() + thOff); }

void libPose(float& lx, float& ly, float& lth) { alvik.get_pose(lx, ly, lth); }

void push(unsigned long now) {
  hist[hHead] = {now, x, y, th()};
  hHead = (hHead + 1) % NH;
  if (hCount < NH) hCount++;
  hLast = now;
}

// Start tracking. With known = false the first fix (or set()) gives the pose.
void start(bool isKnown, float x0 = 0, float y0 = 0, float th0 = 0) {
  float lth;
  libPose(lastLx, lastLy, lth);
  x = x0; y = y0;
  thOff = wrap180(th0 - heading());
  lastTh = th();
  var = 0; varTh = 0; dist = 0;
  driftDps = 0;
  known = isKnown;
  fixesUsed = fixesRejected = 0;
  hCount = hHead = 0;
  lastMs = millis();
  qCm = P("pose_q_cm"); qDeg = P("pose_q_deg"); qDps = P("pose_q_dps");
  push(lastMs);
  on = true;
}

void stop() { on = false; }

// Set the pose outright (laptop "fpose", or the first fix).
void set(float nx, float ny, float nth, float sig, float sigTh) {
  x = nx; y = ny;
  thOff = wrap180(nth - heading());
  lastTh = th();
  var = sig * sig; varTh = sigTh * sigTh;
  known = true;
  hCount = hHead = 0;
  push(millis());
}

// Called on every shouldAbort() (every 5-10 ms in every motion loop).
void tick() {
  if (!on) return;
  float lx, ly, lth;
  libPose(lx, ly, lth);
  float dx = lx - lastLx, dy = ly - lastLy;
  lastLx = lx; lastLy = ly;
  float ds = dx * cosf(lth * DEG2RAD) + dy * sinf(lth * DEG2RAD);   // along the library's own heading
  unsigned long now = millis();
  if (driftDps != 0) thOff = wrap180(thOff - driftDps * (now - lastMs) / 1000.0f);   // docked drift measurement
  float h = th();
  float dh = wrap180(h - lastTh);
  float mid = (lastTh + 0.5f * dh) * DEG2RAD;
  x += ds * cosf(mid);
  y += ds * sinf(mid);
  dist += ds;
  var += qCm * fabsf(ds);
  varTh += qDeg * fabsf(dh) + qDps * (now - lastMs) / 1000.0f;
  lastTh = h;
  lastMs = now;
  if (now - hLast >= H_MS) push(now);
}

// Own pose at robot time t (ms): from the history, or the current pose if t is newer.
bool at(unsigned long t, float& hx, float& hy, float& hth) {
  if (!hCount) return false;
  int newest = (hHead + NH - 1) % NH;
  if ((long)(t - hist[newest].ms) >= 0) {
    // between the newest saved pose and now
    unsigned long now = millis();
    float span = (float)(now - hist[newest].ms);
    float f = span > 0 ? (float)(t - hist[newest].ms) / span : 1.0f;
    if (f > 1) f = 1;
    hx = hist[newest].x + f * (x - hist[newest].x);
    hy = hist[newest].y + f * (y - hist[newest].y);
    hth = hist[newest].th + f * wrap180(th() - hist[newest].th);
    return true;
  }
  for (int k = 1; k < hCount; k++) {
    const Hist& b = hist[(hHead + NH - k) % NH];        // newer
    const Hist& a = hist[(hHead + NH - k - 1) % NH];    // older
    if ((long)(t - a.ms) >= 0) {
      float f = (float)(t - a.ms) / (float)(b.ms - a.ms);
      hx = a.x + f * (b.x - a.x);
      hy = a.y + f * (b.y - a.y);
      hth = a.th + f * wrap180(b.th - a.th);
      return true;
    }
  }
  return false;   // older than the history
}

// A camera fix. th = NAN: position only. Returns false (and lastReject) if ignored.
bool fix(unsigned long t, float fx, float fy, float fth, float sig, float sigTh) {
  unsigned long now = millis();
  if ((long)(now - t) > (long)P("fix_max_age_ms")) { lastReject = "old"; fixesRejected++; return false; }
  if ((long)(t - now) > 200) { lastReject = "future"; fixesRejected++; return false; }
  if (sig <= 0) sig = 0.1f;
  if (sigTh <= 0) sigTh = 1.0f;
  if (!known) {
    if (isNaN(fth)) { lastReject = "first fix needs a heading"; fixesRejected++; return false; }
    set(fx, fy, fth, sig, sigTh);   // the robot has not moved on its own yet: take it as it is
    fixesUsed++;
    return true;
  }
  float hx, hy, hth;
  if (!at(t, hx, hy, hth)) { lastReject = "older than the pose history"; fixesRejected++; return false; }
  float ex = fx - hx, ey = fy - hy;
  float S = var + sig * sig;
  float gate = P("fix_gate");
  if (ex * ex + ey * ey > gate * gate * S && ex * ex + ey * ey > 9.0f) {
    lastReject = "too far from the own pose"; fixesRejected++; return false;
  }
  float k = var / S;
  float dx = k * ex, dy = k * ey;
  var = (1 - k) * var;
  float dth = 0;
  if (!isNaN(fth)) {
    float eth = wrap180(fth - hth);
    float Sth = varTh + sigTh * sigTh;
    if (!(eth * eth > gate * gate * Sth && fabsf(eth) > 5.0f)) {
      float kt = varTh / Sth;
      dth = kt * eth;
      varTh = (1 - kt) * varTh;
    }
  }
  // Move the current pose and the whole history: rotate about the pose at t by dth, then shift.
  float c = cosf(dth * DEG2RAD), s = sinf(dth * DEG2RAD);
  float rx = x - hx, ry = y - hy;
  x = hx + c * rx - s * ry + dx;
  y = hy + s * rx + c * ry + dy;
  for (int k2 = 0; k2 < hCount; k2++) {
    Hist& e = hist[(hHead + NH - 1 - k2) % NH];
    float qx = e.x - hx, qy = e.y - hy;
    e.x = hx + c * qx - s * qy + dx;
    e.y = hy + s * qx + c * qy + dy;
    e.th = wrap180(e.th + dth);
  }
  thOff = wrap180(thOff + dth);
  lastTh = th();
  fixesUsed++;
  return true;
}

float sd() { return sqrtf(var); }
float sdTh() { return sqrtf(varTh); }

}  // namespace pose

// ---------- straight drives on the own pose ----------

enum DriveEnd { DRIVE_OK = 0, DRIVE_ABORT, DRIVE_STEPSTOP, DRIVE_TIMEOUT };

// Wheel speeds for a forward speed v (cm/s, negative = backwards) and turn rate w (deg/s).
void setDrive(float v, float w) {
  const float r = 1.7f, halfTrack = 4.45f;           // library: 34 mm wheels, 89 mm track
  float dv = w * pose::DEG2RAD * halfTrack;
  float k = 60.0f / (2.0f * 3.14159265f * r);          // cm/s -> rpm
  alvik.set_wheels_speed(constrain((v - dv) * k, -70.0f, 70.0f), constrain((v + dv) * k, -70.0f, 70.0f));
}

// Drive straight along the line (x0, y0) -> (x1, y1) facing heading h (deg),
// backwards if rev (heading stays h). How far is left is measured along the
// line's own direction, never along "here -> end" (that points anywhere when
// the end is a fraction of a cm away). Small heading corrections bring the
// robot back onto the line (drive_kct deg per cm, at most drive_max_corr),
// stopped in the last 3 cm. Speed ramps up and down (drive_accel), slowest 2 cm/s.
// Front check: the caller sets gFrontLimitCm / gFrontSoft.
int driveLine(float x0, float y0, float x1, float y1, float h, float vmax, bool rev) {
  float ux = cosf(h * pose::DEG2RAD), uy = sinf(h * pose::DEG2RAD);
  float dir = rev ? -1.0f : 1.0f;
  float mx = dir * ux, my = dir * uy;                 // direction of motion
  float L = (x1 - x0) * mx + (y1 - y0) * my;
  float kct = P("drive_kct"), maxc = P("drive_max_corr"), kh = P("drive_kh"), acc = P("drive_accel");
  const float vmin = 2.0f, lead_s = 0.03f;
  unsigned long t0 = millis(), last = t0;
  unsigned long limit = 3000 + (unsigned long)(fabsf(L) / fmaxf(vmax, 1.0f) * 2500.0f);
  float v = 0;
  while (true) {
    if (shouldAbort()) return gAbortReason.length() ? DRIVE_ABORT : DRIVE_STEPSTOP;
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;
    float rem = (x1 - pose::x) * mx + (y1 - pose::y) * my;
    if (rem <= v * lead_s + 0.05f) break;
    // follow: only what is in the part of the path this step still drives can be hit
    // (beyond its end the next step checks; another robot may legitimately wait there)
    if (gFrontEndBase >= 0) gFrontLimitCm = fminf(gFrontEndBase, fmaxf(rem, 0.0f) + P("front_end_cm"));
    if (now - t0 > limit) { alvik.brake(); gFail = "drive timed out"; return DRIVE_TIMEOUT; }
    float cross = mx * (pose::y - y1) - my * (pose::x - x1);   // + = left of the line (facing the motion)
    float corr = rem > 3.0f ? constrain(-kct * cross, -maxc, maxc) : 0.0f;
    float herr = wrap180(h + corr - pose::th());
    float vt = fminf(vmax, sqrtf(2.0f * acc * rem) + vmin);
    v = fminf(vt, v + acc * dt + 0.5f);
    float w = constrain(kh * herr, -60.0f, 60.0f);
    setDrive(dir * v, w);
    delay(5);
  }
  alvik.brake();
  return DRIVE_OK;
}

// Turn in place to floor heading h (deg) with the IMU turn (imuTurnTo).
bool turnToFloor(float h) {
  return imuTurnTo(wrap180(h - pose::thOff)).ok;
}
