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
//     IMU's zero to the floor. Both use the robot's stored corrections
//     (corr.h: wheel_scale, gyro_scale, gyro_drift_dps).
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
//
// 2026-11-12e21 (the 12 Nov error and loss audit, Nick's decision 6; code and tests only, flashed in a staged rollout):
//   - the variance grows from the measured record: its sd grows pose_sd_per_cm per cm (linear, as the real error does;
//     pose_sd_per_cm 0 = the old pose_q_cm variance growth, about 6x too sure after 100 cm);
//   - fix_max_age_ms 1500 from the measured delay (11 Nov: p90 1.14 s, p95 1.39 s; 17 % of fixes were refused as old
//     at 1000), and the Pi sends its own measured value;
//   - a longer history: 96 x 40 ms = 3.84 s (pose_hist_ms limits it; 1280 = the old 32 entries);
//   - every refused fix counted by its reason (old, future, history, far, no heading) and reported in FOLLOW;
//   - the consistency escape: fix_escape_n far fixes in a row whose offsets agree with each other mean its own pose is
//     wrong, so it takes the camera's offset (fix_escape_n 0 = the old ratchet).
#pragma once

#include "motion.h"

// NAN marks "not given" (a fix without a heading, a step without a front distance).
inline bool isNaN(float v) { return v != v; }

namespace pose {

const float DEG2RAD = 0.017453293f;
const int NH = 96;                 // pose history: 96 x 40 ms = 3.84 s (2026-11-12e21; was 32 = 1.28 s; pose_hist_ms)
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
// (2026-11-12e21) refusals by reason, and the escape
int rejOld = 0, rejFuture = 0, rejHist = 0, rejFar = 0, rejFirst = 0, escapes = 0;
const int NE = 16;
float escX[NE], escY[NE];
int escN = 0;
float sdLin = 0;                   // the linear sd (pose_sd_per_cm > 0)

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
  rejOld = rejFuture = rejHist = rejFar = rejFirst = escapes = 0;
  escN = 0;
  sdLin = 0;
  hCount = hHead = 0;
  lastMs = millis();
  qCm = P("pose_q_cm"); qDeg = P("pose_q_deg"); qDps = P("pose_q_dps");
  push(lastMs);
  on = true;
}

void stop() { on = false; }

// Tracking on again WITHOUT forgetting the pose (E02, e02.h: the pose a `run find` set carries into the next
// run; the robot stood still in between, and heading() kept counting turns while idle). Fresh history,
// variances and counters; x, y, thOff and known are kept.
void resume() {
  float lth;
  libPose(lastLx, lastLy, lth);
  lastTh = th();
  dist = 0;
  driftDps = 0;
  fixesUsed = fixesRejected = 0;
  hCount = hHead = 0;
  lastMs = millis();
  qCm = P("pose_q_cm"); qDeg = P("pose_q_deg"); qDps = P("pose_q_dps");
  push(lastMs);
  on = true;
}

// Set the pose outright (laptop "fpose", or the first fix).
void set(float nx, float ny, float nth, float sig, float sigTh) {
  x = nx; y = ny;
  thOff = wrap180(nth - heading());
  lastTh = th();
  var = sig * sig; varTh = sigTh * sigTh;
  sdLin = sig;
  escN = 0;
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
  float ds = (dx * cosf(lth * DEG2RAD) + dy * sinf(lth * DEG2RAD))   // along the library's own heading,
             * P("wheel_scale");                                      // in real cm (corr.h)
  unsigned long now = millis();
  if (driftDps != 0) thOff = wrap180(thOff - driftDps * (now - lastMs) / 1000.0f);   // docked drift measurement
  float h = th();
  float dh = wrap180(h - lastTh);
  float mid = (lastTh + 0.5f * dh) * DEG2RAD;
  x += ds * cosf(mid);
  y += ds * sinf(mid);
  dist += ds;
  if (P("pose_sd_per_cm") > 0) {                 // (2026-11-12e21) the sd grows linearly, as the measured error does
    float sdNow = sqrtf(var);
    sdNow += P("pose_sd_per_cm") * fabsf(ds);
    var = sdNow * sdNow;
  } else {
    var += qCm * fabsf(ds);
  }
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
  unsigned long now0 = millis();
  long histMs = (long)P("pose_hist_ms");
  for (int k = 1; k < hCount; k++) {
    const Hist& b = hist[(hHead + NH - k) % NH];        // newer
    const Hist& a = hist[(hHead + NH - k - 1) % NH];    // older
    if (histMs > 0 && (long)(now0 - a.ms) > histMs) break;   // (2026-11-12e21) only pose_hist_ms of it
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
  if ((long)(now - t) > (long)P("fix_max_age_ms")) { lastReject = "old"; fixesRejected++; rejOld++; return false; }
  if ((long)(t - now) > 200) { lastReject = "future"; fixesRejected++; rejFuture++; return false; }
  if (sig <= 0) sig = 0.1f;
  if (sigTh <= 0) sigTh = 1.0f;
  if (!known) {
    if (isNaN(fth)) { lastReject = "first fix needs a heading"; fixesRejected++; rejFirst++; return false; }
    set(fx, fy, fth, sig, sigTh);   // the robot has not moved on its own yet: take it as it is
    fixesUsed++;
    return true;
  }
  float hx, hy, hth;
  if (!at(t, hx, hy, hth)) { lastReject = "older than the pose history"; fixesRejected++; rejHist++; return false; }
  float ex = fx - hx, ey = fy - hy;
  float S = var + sig * sig;
  float gate = P("fix_gate");
  if (ex * ex + ey * ey > gate * gate * S && ex * ex + ey * ey > 9.0f) {
    // (2026-11-12e21) the consistency escape: far fixes in a row that agree with each other (the camera's offset steady
    // within fix_gate of its sigma) mean the own pose is wrong, not the camera: take the camera's offset
    int needN = (int)P("fix_escape_n");
    if (needN > 0) {
      if (needN > NE) needN = NE;
      if (escN >= needN) {
        for (int k = 1; k < needN; k++) { escX[k - 1] = escX[k]; escY[k - 1] = escY[k]; }
        escN = needN - 1;
      }
      escX[escN] = ex; escY[escN] = ey; escN++;
      if (escN >= needN) {
        float mx = 0, my = 0;
        for (int k = 0; k < escN; k++) { mx += escX[k]; my += escY[k]; }
        mx /= escN; my /= escN;
        bool agree = true;
        for (int k = 0; k < escN; k++) {
          float dx = escX[k] - mx, dy = escY[k] - my;
          if (dx * dx + dy * dy > gate * gate * sig * sig) { agree = false; break; }
        }
        if (agree) {
          x += mx; y += my;
          for (int k2 = 0; k2 < hCount; k2++) {
            Hist& e = hist[(hHead + NH - 1 - k2) % NH];
            e.x += mx; e.y += my;
          }
          var = sig * sig + 0.25f * (mx * mx + my * my);
          escN = 0;
          escapes++;
          fixesUsed++;
          lastReject = "";
          return true;
        }
      }
    }
    lastReject = "too far from the own pose"; fixesRejected++; rejFar++; return false;
  }
  escN = 0;
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
// (2026-11-05b, home free roam) the follow commands `fspd` and `fsteer` change a drive WHILE it runs:
// gSpeedScale multiplies the step's speed (0 = stand on the line, holding the step; up to 1.5), gSteer points the
// rest of a forward drive at a new end (the line from where the robot is now to (gSteerX, gSteerY)). Both are reset
// to "no change" at the start of every follow step; nothing else sets them, so every other drive is unchanged.
float gSpeedScale = 1.0f;
bool gSteer = false;
float gSteerX = 0, gSteerY = 0;

int driveLine(float x0, float y0, float x1, float y1, float h, float vmax, bool rev) {
  float ux = cosf(h * pose::DEG2RAD), uy = sinf(h * pose::DEG2RAD);
  float dir = rev ? -1.0f : 1.0f;
  float mx = dir * ux, my = dir * uy;                 // direction of motion
  float L = (x1 - x0) * mx + (y1 - y0) * my;
  float slackMs = 0;                                  // (2026-11-05b) time not counted against the limit while slowed
  float kct = P("drive_kct"), maxc = P("drive_max_corr"), kh = P("drive_kh"), acc = P("drive_accel");
  const float vmin = 2.0f, lead_s = 0.03f;
  unsigned long t0 = millis(), last = t0;
  unsigned long limit = 3000 + (unsigned long)(fabsf(L) / fmaxf(vmax, 1.0f) * 2500.0f);
  float v = 0;
  struct BackFlag { BackFlag(bool r) { gTiltBackOK = r; } ~BackFlag() { gTiltBackOK = false; } } backFlag(rev);  // 11-05d
  while (true) {
    if (shouldAbort()) return gAbortReason.length() ? DRIVE_ABORT : DRIVE_STEPSTOP;
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;
    if (gSteer && !rev) {                             // (2026-11-05b) fsteer: the rest of the drive to a new end
      gSteer = false;
      x1 = gSteerX; y1 = gSteerY;
      h = atan2f(y1 - pose::y, x1 - pose::x) / pose::DEG2RAD;
      mx = cosf(h * pose::DEG2RAD); my = sinf(h * pose::DEG2RAD);
      L = (x1 - pose::x) * mx + (y1 - pose::y) * my;
      t0 = now; slackMs = 0;
      limit = 3000 + (unsigned long)(fabsf(L) / fmaxf(vmax, 1.0f) * 2500.0f);
    }
    float sc = constrain(gSpeedScale, 0.0f, 1.5f);
    if (sc < 1.0f) slackMs += (1.0f - sc) * dt * 1000.0f;
    float rem = (x1 - pose::x) * mx + (y1 - pose::y) * my;
    if (rem <= v * lead_s + 0.05f) break;
    // follow: only what is in the part of the path this step still drives can be hit
    // (beyond its end the next step checks; another robot may legitimately wait there)
    if (gFrontEndBase >= 0) gFrontLimitCm = fminf(gFrontEndBase, fmaxf(rem, 0.0f) + P("front_end_cm"));
    if (now - t0 > limit + (unsigned long)slackMs) { alvik.brake(); gFail = "drive timed out"; return DRIVE_TIMEOUT; }
    float cross = mx * (pose::y - y1) - my * (pose::x - x1);   // + = left of the line (facing the motion)
    float corr = rem > 3.0f ? constrain(-kct * cross, -maxc, maxc) : 0.0f;
    float herr = wrap180(h + corr - pose::th());
    float vt = fminf(vmax * sc, sqrtf(2.0f * acc * rem) + vmin * fminf(sc, 1.0f));
    v = fminf(vt, v + acc * dt + 0.5f);
    float w = constrain(kh * herr, -60.0f, 60.0f);
    setDrive(dir * v, w);
    delay(5);
  }
  alvik.brake();
  return DRIVE_OK;
}

// Drive round a circle on the own pose (follow step C, firmware 2026-10-28a; Monday's circle, Nick 2026-10-27: "a true
// curve"): centre (cx, cy), radius r cm, from angle a0 round the centre through sweep deg (+ = anticlockwise), at
// vmax cm/s along the circle. Steering: the turn rate v / r fed forward (both wheels run, the outer faster), plus
// drive_kh x the heading error to the circle's tangent, which leans arc_kr deg per cm of radial error back towards
// the circle (at most 20 deg). Progress is the robot's own angle round the centre, unwrapped from a0, so a sweep of
// more than 360 deg works (the laptop sends 30 deg pieces). cont: the next step carries on round the same circle, so
// it neither slows down at the end nor brakes. Front check: the caller's (a turn's: the centre zone).
int driveArc(float cx, float cy, float r, float a0, float sweep, float vmax, bool cont) {
  float sg = sweep >= 0 ? 1.0f : -1.0f;
  float end = a0 + sweep;
  float phi = atan2f(pose::y - cy, pose::x - cx) / pose::DEG2RAD;
  float prog = a0 + wrap180(phi - a0), prev = phi;
  float kr = P("arc_kr"), kh = P("drive_kh"), acc = P("drive_accel");
  unsigned long t0 = millis(), last = t0;
  unsigned long limit = 3000 + (unsigned long)(fabsf(sweep) * pose::DEG2RAD * r / fmaxf(vmax, 1.0f) * 2500.0f);
  float v = cont ? vmax : 0;
  while (true) {
    if (shouldAbort()) return gAbortReason.length() ? DRIVE_ABORT : DRIVE_STEPSTOP;
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;
    phi = atan2f(pose::y - cy, pose::x - cx) / pose::DEG2RAD;
    prog += wrap180(phi - prev);
    prev = phi;
    float rem = sg * (end - prog) * pose::DEG2RAD * r;          // cm still to go round the circle
    if (rem <= v * 0.03f + 0.05f) break;
    if (now - t0 > limit) { alvik.brake(); gFail = "arc timed out"; return DRIVE_TIMEOUT; }
    float e = sqrtf((pose::x - cx) * (pose::x - cx) + (pose::y - cy) * (pose::y - cy)) - r;   // + = outside
    float h = phi + sg * 90.0f + sg * constrain(kr * e, -20.0f, 20.0f);
    float herr = wrap180(h - pose::th());
    float vt = cont ? vmax : fminf(vmax, sqrtf(2.0f * acc * fmaxf(rem, 0.0f)) + 2.0f);
    v = fminf(vt, v + acc * dt + 0.5f);
    float w = sg * (v / r) / pose::DEG2RAD + constrain(kh * herr, -60.0f, 60.0f);
    setDrive(v, w);
    delay(5);
  }
  if (!cont) alvik.brake();
  return DRIVE_OK;
}

// ---------- (2026-11-05c) continuous curves: follow step R, farc, coast, dodge ----------
// Nick, 5 Nov 16:06: "eliminate the stop and turn ... curving turns where velocity never drops to 0". A piece is a
// path of constant curvature k (1/cm, + = left / anticlockwise, 0 = straight) from (x0, y0) facing h0, len cm long,
// driven on the own pose: the turn rate v x k fed forward, plus drive_kh x the heading error to the path's tangent,
// which leans back towards the path (arc_kr deg per cm on a curve, arc_kct on a straight; at most 20 deg). The speed
// is never ramped down at the end: the next R piece carries it on (follow.h), or the robot coasts. The calibration's
// arc_k_scale / arc_d_scale multiply the curvature and length asked (1 = as asked).
// gCurveV / gCurveK: the speed and curvature the robot is driving now (carried into the next piece and the coast).
// gArcSet (farc): the running piece bends to curvature gArcSetK from where the robot is, for gArcSetLen cm (NaN: what
// was left). Front check (gCurveLook >= 0): anything in the path closer than gCurveLook for obs_n new readings (or
// front_confirm_ms) ends the piece WITHOUT a stop: DRIVE_DODGE, gDodgeK the tight curve away from the nearer side.
float gCurveV = 0, gCurveK = 0;
bool gArcSet = false;
float gArcSetK = 0, gArcSetLen = NAN;
float gCurveLook = -1;
float gDodgeK = 0, gDodgeCm = 0;
const int DRIVE_DODGE = 4;

// The robot's front: the nearest thing in its path (frontInPath) and which side is more open (+1 left, -1 right).
int openSide() {
  float d[5];
  alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
  for (int k = 0; k < 5; k++) if (d[k] <= 0) d[k] = 999.0f;
  float l = fminf(d[0], d[1]), r = fminf(d[3], d[4]);   // zones 22, 11 deg = left; -11, -22 = right
  return l >= r ? 1 : -1;
}

int driveCurve(float x0, float y0, float h0, float k, float len, float vmax) {
  k *= P("arc_k_scale");
  len *= P("arc_d_scale");
  float kr = P("arc_kr"), kct = P("arc_kct"), kh = P("drive_kh"), acc = P("drive_accel");
  float s = 0, prev = 0;                                 // progress along the piece, cm
  bool line = fabsf(k) < 1e-4f;
  float R = line ? 0 : 1.0f / fabsf(k), sg = k >= 0 ? 1.0f : -1.0f;
  float cx = 0, cy = 0, phi0 = 0;
  auto setup = [&]() {
    line = fabsf(k) < 1e-4f;
    R = line ? 0 : 1.0f / fabsf(k);
    sg = k >= 0 ? 1.0f : -1.0f;
    if (!line) {
      float hr = h0 * pose::DEG2RAD;
      cx = x0 - sinf(hr) * sg * R;                       // the centre: R to the left (k > 0) or right
      cy = y0 + cosf(hr) * sg * R;
      phi0 = atan2f(y0 - cy, x0 - cx) / pose::DEG2RAD;
      prev = phi0;
    }
    s = 0;
  };
  setup();
  unsigned long t0 = millis(), last = t0;
  float slackMs = 0;
  unsigned long limit = 3000 + (unsigned long)(len / fmaxf(vmax, 1.0f) * 2500.0f);
  float v = gCurveV;
  int lowN = 0;
  float prevF = -1;
  while (true) {
    if (shouldAbort()) { gCurveV = 0; return gAbortReason.length() ? DRIVE_ABORT : DRIVE_STEPSTOP; }
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;
    if (gArcSet) {                                       // farc: bend from here, no stop
      gArcSet = false;
      float left = len - s;
      x0 = pose::x; y0 = pose::y; h0 = pose::th();
      k = gArcSetK * P("arc_k_scale");
      len = isNaN(gArcSetLen) ? left : gArcSetLen * P("arc_d_scale");
      setup();
      t0 = now; slackMs = 0;
      limit = 3000 + (unsigned long)(len / fmaxf(vmax, 1.0f) * 2500.0f);
    }
    float href;                                          // the path's heading where the robot is, deg
    float lean;                                          // steering back towards the path, deg
    if (line) {
      float ux = cosf(h0 * pose::DEG2RAD), uy = sinf(h0 * pose::DEG2RAD);
      s = (pose::x - x0) * ux + (pose::y - y0) * uy;
      float cross = ux * (pose::y - y0) - uy * (pose::x - x0);   // + = left of the path
      href = h0;
      lean = constrain(-kct * cross, -20.0f, 20.0f);
    } else {
      float phi = atan2f(pose::y - cy, pose::x - cx) / pose::DEG2RAD;
      s += sg * wrap180(phi - prev) * pose::DEG2RAD * R;
      prev = phi;
      float e = sqrtf((pose::x - cx) * (pose::x - cx) + (pose::y - cy) * (pose::y - cy)) - R;   // + = outside
      href = phi + sg * 90.0f;
      lean = sg * constrain(kr * e, -20.0f, 20.0f);
    }
    float rem = len - s;
    float sc = constrain(gSpeedScale, 0.0f, 1.5f);
    if (sc < 1.0f) slackMs += (1.0f - sc) * dt * 1000.0f;
    if (rem <= v * 0.03f + 0.05f) break;
    if (now - t0 > limit + (unsigned long)slackMs) { alvik.brake(); gCurveV = 0; gFail = "curve timed out"; return DRIVE_TIMEOUT; }
    // the front check: a dodge instead of a stop
    if (gCurveLook > 0) {
      float d = frontInPath();
      bool fresh = d != prevF;
      prevF = d;
      if (d > 0 && d < gCurveLook) {
        int needN = (int)P("obs_n");
        if (fresh) lowN++;
        if (lowN >= (needN > 0 ? needN : 2)) {
          gFrontTripCm = d;
          if (P("arc_dodge_r") > 0 && sc > 0) {          // held at speed 0 (fspd 0): a stop, never a dodge
            gDodgeK = openSide() / P("arc_dodge_r");
            gDodgeCm = d;
            gCurveK = gDodgeK;
            gCurveV = fminf(fmaxf(v, P("arc_vmin")), P("arc_dodge_cms"));
            return DRIVE_DODGE;
          }
          gStepStop = "obstacle";
          alvik.brake();
          gCurveV = 0;
          return DRIVE_STEPSTOP;
        }
      } else if (fresh) {
        lowN = 0;
      }
    }
    float vt = vmax * sc;
    v = v < vt ? fminf(vt, v + acc * dt + 0.3f) : fmaxf(vt, v - acc * dt - 0.3f);
    float herr = wrap180(href + lean - pose::th());
    float w = (line ? 0.0f : sg * (v / R) / pose::DEG2RAD) + constrain(kh * herr, -60.0f, 60.0f);
    setDrive(v, w);
    gCurveV = v;
    gCurveK = line ? 0.0f : sg / R;
    delay(5);
  }
  return DRIVE_OK;                                       // no brake: the next piece or the coast carries on
}

// Turn in place to floor heading h (deg) with the IMU turn (imuTurnTo).
bool turnToFloor(float h) {
  return imuTurnTo(wrap180(h - pose::thOff)).ok;
}
