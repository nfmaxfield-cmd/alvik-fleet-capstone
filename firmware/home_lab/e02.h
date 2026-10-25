// E02 accuracy experiment (firmware 2026-10-22b; reverse find 2026-10-25b). Plan: docs/build-plans/accuracy-experiment-E02.md.
// Records and commands: docs/data-formats.md "E02". Simulator copy: tools/sim/fw_e02.py.
//
// REVERSE FIND (firmware 2026-10-25b, e02_find_reverse 1; Nick, 2026-10-25: one attachment, switches at the BACK).
// Each board is found by BACKING into it, so the front distance sensor and the front line sensors (the edge guard)
// stay clear and look where the robot is going between boards. Per board: face it (a board further than
// e02_find_turn_cm is first approached nose first, front sensor and edge guard on, to that distance; from the
// staging point it is nearer); look (below); turn round to face AWAY from it; creep backwards; bump = either back switch clicking (cue switch-L / switch-R / switch-LR: the
// strongest cue; the press is timed from the switch's first raw closing), or the stall / jolt cues as before (the
// front touch cue and the "something far ahead" check do not apply backwards); square by pushing backwards until
// both switches are pressed or the heading stops changing (BUMP squared = 2: both pressed, 1: heading still).
// The touching face is the back face, e02_back_face_cm behind the axle, and the robot's heading against a board is
// the board's `toward` + 180: the reset, toGo() and the turn_k re-trace use both. Leaving a board is a FORWARD
// drive of e02_find_back_cm (edge guard and front stop on). Backing is blind, so: the creep backs at most
// e02_find_back_margin_cm (+ e02_find_tol_pct of the travel since the pose was set) past where the pose says the
// board is, and never so far that the axle comes within e02_find_keep_cm of the board's line, then stops with
// "no board behind" (never further than e02_find_max_cm); the virtual border stays on. Before it turns its back,
// it LOOKS: facing the board, standing still, the front sensor must see it no further than the map puts it +
// e02_find_see_cm (+ slack), else the find fails without backing ("no board seen"): a removed board over an open
// edge cannot draw it backwards off the plywood (its rear caster is ~4 cm behind the axle).
// It needs the switches (sw_on 1, sw_back 1) and a pose (a camera fix or a find): without either it finds nose
// first, as before, and says so (FIND reverse 0). RAW steps as below; each board's step 1 / 5 is (turn + drive near
// it +) the turn to face away.
//
// BUMP DETECTION (no switches; nose first. Backing in with the back switches: REVERSE FIND above). The robot
// creeps into a board at bump_creep_cms, holding its gyro heading.
// Two cues:
//   - stall: the wheels' measured speed (get_wheels_speed) falls below bump_stall_frac of what was commanded;
//   - jolt: the accelerometer's driving-axis reading (ax) jumps more than bump_jolt_g from its running level.
// A bump = a stall that lasts bump_stall_ms and began within bump_jolt_win_ms after a jolt, OR a stall that
// lasts bump_stall_only_ms with no jolt, OR (2026-10-22a) TOUCH: the front distance sensor's centre zones read at
// most bump_touch_cm for bump_touch_ms (the robot's face is on the board). Why the touch cue: on the real board
// (2026-10-22, Left's find) the tyres slipped on the plywood at full speed for 9 s after contact, so the wheels
// never "stalled", and the creep's contact jolt was 0.146 g, just under bump_jolt_g; the front sensor read 12 cm
// falling to 0.2-0.8 cm and stayed there. A jolt alone (a floor seam, vibration) is only counted (BUMP seams);
// a short slow-down (a seam's lip) never lasts long enough. Real numbers behind the defaults
// (roam1 logs, 2026-10-19/20): creep vibration on ax has sd ~0.05 g and 1 sample in 100 beyond ~0.13 g, so a
// jolt alone can never be trusted; ax is the driving axis (it tracks speeding up / slowing down; its sign is
// not used here, only the size). The moment of contact = the jolt (when there was one), else the stall's
// start; the believed pose there comes from the pose history. A stall while the front sensor sees something
// clearly further away (bump_far_cm .. 100 cm) is not a board: the find fails instead of trusting it.
//
// PUSH-TO-SQUARE. After the bump both wheels push at push_rpm (gentle) for push_min_ms .. push_ms; the board
// turns the robot until its whole front face lies on it. Done when the heading changes less than
// push_still_deg in 250 ms. Then the heading is the board's (facing it), exactly.
//
// FIND (`run find`). 0 (pose known: send a camera fix first) drive to the staging point e02_find_stage_cm from
// both faces, so neither approach is long. 1 face the x board, drive near it (the front sensor stops this drive
// early if the board is nearer than the pose says), creep, bump, check the bump is not much sooner than the
// pose says the board is (e02_find_tol_cm + e02_find_tol_pct of the distance dead-reckoned since the pose was
// last set: something in the way; a later bump can only be the board), square; the pose's distance from the board
// and its heading are now known exactly. 2 back off e02_find_back_cm, turn to face the y board, creep, bump,
// square. The two board headings are known, so the gyro's turn between the two squarings gives its scale for
// that stretch (turn_k = board-to-board angle / gyro turn); the path between the bumps is re-traced with it,
// so a gyro that under-reads turns does not push the position sideways. The pose is reset: distance from
// the y board and heading from bump 2, distance from the x board from bump 1 plus the re-traced path. 3 back
// off again. Pose unknown before (first find after power-on, no camera fix): the robot takes it that it faces
// the x board, and creeps up to e02_find_max_cm. RAW step: 0 staging, 1 turn + drive near the x board, 2 creep, 3 square,
// 4 back off, 5 turn + drive near the y board, 6 creep, 7 square, 8 back off.
//
// ROUTES (`run e02`). Waypoints in the frame of the start pose (x ahead, y left), each reached by turning to
// face it and driving straight at it on the own pose (driveLine: gyro heading, wheel distance, corrections
// from corr.h). 1 straight: (L, 0) far, back to (0, 0). 2 L: (a, 0), (a, t*a) far, back (a, 0), (0, 0).
// 3 square: (s, 0), (s, t*s) far, (0, t*s), (0, 0). t = +1 (left) / -1 (right): towards the plywood's middle
// (away from the x board) unless e02_l_turn / e02_square_turn says. Then it turns to the start heading. It
// stands still e02_pause_ms at the far point. Mode 1 (camera-steered): it drives only what the laptop sends
// (go / turn), and the laptop says where the start, far point and end are (mark start / far / end, or
// e02 start / far / end). go / turn also work with no run going (idle legs: parking, finding a robot in the
// picture, bringing a lost one back), one at a time, from loop(), with the same checks.
//
// CAMERA FIX (`fix x y th t_wall`; th "-" = position only). Used only when the wheels have been still for
// e02_fix_still_ms (so the camera picture shows where it is now, whatever the WiFi delay). During an e02 run
// the blend, plainly: the new pose = old + e02_fix_k x (camera - old), heading the same way (the shorter way
// round): e02_fix_k (0.5 = halfway) for KC (mode 0); in a camera-steered run (mode 1, K) a fix replaces the
// pose. While idle (no run) a fix replaces the pose too: it says where the robot is now (the laptop sends one
// before every find). A robot with no pose takes the fix as it is.
//
// SAFETY. The edge guard (edge.h) stays on in every mode (forward driving; backing up is blind: the backward
// moves here retrace the path just driven forward, or are at most e02_back_max_cm; the reverse find's creep is
// capped at e02_find_back_margin_cm past where the board should be, "no board behind"). Find, e02 runs and go legs
// refuse to start while safe_edge is 0 (2026-10-21b: the guard was found off for a whole session's plan).
// Blind backing: go legs backwards add up (backSince); more than e02_back_max_cm in all since the last camera fix
// or find is refused, so the laptop must look (camera) and send a fix between pieces. A stop command always
// works (every loop calls shouldAbort()). The wheels never travel more than transit + route + e02_margin_cm in
// a run (find: 2 x e02_find_max_cm + 2 x back off + 20 cm + the staging drive + the drives towards the boards).
#pragma once

#include "pose.h"
#include "switches.h"

extern String gRunId;

namespace e02 {

const float NONE = -1.0f;
const float UNSET = -999.0f;

// VIRTUAL BORDER (firmware 2026-10-25a; Nick, 2026-10-24: the robots' own check, under the laptop's camera border).
// Once the robot knows where it is (pose::known: a find or a camera fix), every motion is checked against the open
// edges the laptop sent (e02_open_x_cm: the open edge along x = that, e02_open_y_cm: along y = that; the inside is
// away from the boards, by e02_xboard_side / e02_yboard_side). Within border_cm of one and moving towards it (its
// distance down by BORDER_TOWARDS_CM since it was nearest... the reference resets whenever it moves away) the motion
// stops, a BORDER record goes to the laptop, and the border latches: no go leg towards that edge (by its heading)
// until it is BORDER_UNLATCH_CM back outside the border, or the laptop sends `border clear`. Turns and moves away are always allowed. A crawl
// that is looking for the edge on purpose (explore, the edge find) has border_seek 1 set by the laptop for its
// leg: forwards then passes (the edge guard stops it). border_cm 0 = off (the default: the laptop turns it on).
const float BORDER_TOWARDS_CM = 0.5f;   // chosen: how much nearer counts as moving towards (not wheel jitter)
const float BORDER_MOSTLY = 0.5f;       // chosen: ... and nearer by at least half the distance driven (driving along
                                        // an edge is allowed: the laptop's camera border does the same)
const float BORDER_UNLATCH_CM = 3.0f;   // chosen: back this far outside the border, the latch lets go by itself
bool borderLatched = false;
float borderRef = 1e9f;                 // its distance when it was last moving away or still
float borderRefDist = 0;                // pose::dist then

float borderDist(float px, float py) {
  if (!pose::known || P("border_cm") <= 0) return UNSET;
  float ox = P("e02_open_x_cm"), oy = P("e02_open_y_cm");
  float d = 1e9f;
  if (ox > UNSET + 1) d = fminf(d, (px - ox) * (P("e02_yboard_side") > 0 ? 1.0f : -1.0f));
  if (oy > UNSET + 1) d = fminf(d, (py - oy) * (P("e02_xboard_side") > 0 ? 1.0f : -1.0f));
  return d >= 1e8f ? UNSET : d;
}

void borderRecord(float d) {
  net::sendLine("H,BORDER,run,ms,x_cm,y_cm,th_deg,dist_cm,border_cm");
  net::logf("D,BORDER,%s,%lu,%.2f,%.2f,%.2f,%.2f,%.1f", gRunId.length() ? gRunId.c_str() : "-", millis(), pose::x,
            pose::y, pose::th(), d, P("border_cm"));
}

void borderTick() {
  float d = borderDist(pose::x, pose::y);
  if (d == UNSET) { borderRef = 1e9f; return; }
  if (borderLatched && d >= P("border_cm") + BORDER_UNLATCH_CM) borderLatched = false;   // well inside again
  if (borderRef >= 1e8f) { borderRef = d; borderRefDist = pose::dist; return; }        // the first reading
  if (d > borderRef || fabsf(pose::dist - borderRefDist) > 30.0f) {   // moving away, a fix, or a long way since
    borderRef = d;
    borderRefDist = pose::dist;
  }
  if (d >= P("border_cm") || borderRef - d < BORDER_TOWARDS_CM) return;
  if (borderRef - d < BORDER_MOSTLY * fabsf(pose::dist - borderRefDist)) return;   // mostly along it, not at it
  float sl, sr;
  alvik.get_wheels_speed(sl, sr);
  bool forward = sl + sr > 0;
  if (P("border_seek") > 0 && forward) return;          // looking for the edge on purpose: the edge guard's job
  if (gAbortReason.length()) return;
  gAbortReason = "virtual border";
  alvik.brake();
  if (!borderLatched) borderRecord(d);
  borderLatched = true;
  borderRef = d;
  borderRefDist = pose::dist;
}

// would a go leg of arg cm (along its heading) bring it nearer an open edge while it is inside the border?
bool borderTowards(float arg) {
  float d0 = borderDist(pose::x, pose::y);
  if (d0 == UNSET || d0 >= P("border_cm")) return false;
  float h = pose::th() * (float)M_PI / 180.0f;
  float d1 = borderDist(pose::x + arg * cosf(h), pose::y + arg * sinf(h));
  return d1 != UNSET && d1 < d0 - 0.1f;
}

bool active = false;          // a find or e02 run is going (fix / go / turn / mark are for it)
bool finding = false;         // the find: no fixes now (the bumps are the fix)
bool mode1 = false;           // e02 run, camera-steered
float travelCap = 0;          // > 0: stop the run once the wheels have travelled this far, cm
float travelAbs = 0;          // wheel travel (forward and back) since the run started, cm
float lastDist = 0;
unsigned long lastMoveMs = 0; // the wheels last turned (measured), robot ms
float sinceSet = 0;           // wheel travel since the pose was last set (a camera fix, a find), cm
float backSince = 0;          // backwards go legs since the last camera fix or find, cm (blind: capped)
int fixesUsed = 0, fixesRejected = 0;

// path between the two bumps of a find: wheel steps and the gyro's heading change since square 1
const int NPATH = 200;
float pathDs[NPATH], pathRel[NPATH];
int nPath = 0;
bool pathOn = false;
float pathG0 = 0, segDs = 0, segRelW = 0;
// the gyro's turn since square 1, UNWRAPPED (2026-10-25b): the reverse find turns round between the boards, so the
// heading can wind past +-180 deg from square 1 (e.g. +90 then +180 = +270); a wrapped angle then broke turn_k and
// the re-trace (simulation: 5 cm off). Added up every tick, turning in place included.
float pathTurn = 0, pathLastH = 0;

void pathPush() {
  if (segDs == 0) return;
  float rel = segRelW / fabsf(segDs);
  if (nPath < NPATH) { pathDs[nPath] = segDs; pathRel[nPath] = rel; nPath++; }
  else { pathRel[NPATH - 1] = (pathRel[NPATH - 1] * fabsf(pathDs[NPATH - 1]) + rel * fabsf(segDs)) /
                              (fabsf(pathDs[NPATH - 1]) + fabsf(segDs)); pathDs[NPATH - 1] += segDs; }
  segDs = segRelW = 0;
}

void tickHook() {
  pose::tick();
  float ds = pose::dist - lastDist;
  lastDist = pose::dist;
  travelAbs += fabsf(ds);
  sinceSet += fabsf(ds);
  if (pathOn) {
    float hNow = heading();
    pathTurn += wrap180(hNow - pathLastH);
    pathLastH = hNow;
  }
  if (pathOn && ds != 0) {
    segDs += ds;
    segRelW += pathTurn * fabsf(ds);
    if (fabsf(segDs) >= 0.5f) pathPush();
  }
  float sl, sr;
  alvik.get_wheels_speed(sl, sr);
  if (fabsf(sl) + fabsf(sr) > 4.0f || fabsf(ds) > 0.05f) lastMoveMs = millis();   // turning or rolling (not noise)
  if (travelCap > 0 && travelAbs > travelCap && !gAbortReason.length())
    gAbortReason = "drove further than the route allows";
  borderTick();
}

// Tracking on for a run: keep the pose if one is known (a find, a fix), else start one.
void begin() {
  if (pose::known) pose::resume();
  else pose::start(false);
  lastDist = 0;
  travelAbs = 0;
  travelCap = 0;
  lastMoveMs = 0;               // runs start from standing still
  fixesUsed = fixesRejected = 0;
  pathOn = false;
  borderRef = 1e9f;             // the border's reference starts again with every leg or run (a fix may have moved it)
  gTickHook = tickHook;
  active = true;
}

void end() {
  alvik.brake();
  pathOn = false;
  active = finding = mode1 = false;
  travelCap = 0;
  gTickHook = nullptr;
  gFrontSoft = false;
  gEdgeSoft = false;
  gFrontLimitCm = -1;
  gFrontPath = false;
  gFrontEndBase = -1;
  gStepStop = "";
  pose::stop();             // the pose itself is kept for the next run (pose::resume)
}

float faceCm() {
  float f = P("e02_face_cm");
  if (f > 0) return f;
  return P("bumper_cm") > 0 ? P("bump_ahead_cm") : P("tof_ahead_cm") + P("tof_recess_cm");
}

// Reverse find (2026-10-25b): this find backs into the boards (set at the start of runFind).
bool findRev = false;

// The front sensor's view straight ahead, standing still (reverse find's look): the nearest of the three middle
// zones, median of 5 frames ~70 ms apart, after letting the sensor settle 150 ms. -1 on a stop.
float lookAhead() {
  if (!waitMs(150)) return -1;
  float v[5];
  for (int i = 0; i < 5; i++) {
    float d[5];
    alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
    v[i] = fminf(d[1], fminf(d[2], d[3]));
    if (i < 4 && !waitMs(70)) return -1;
  }
  for (int i = 1; i < 5; i++)                     // sort (5 values)
    for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}

// The face that touches the board in this find: the front face (faceCm) or the back attachment's (reverse).
float touchFace() { return findRev ? P("e02_back_face_cm") : faceCm(); }

// A board's face as a line: n = unit vector from the robot towards the board, c = n . (any point on the face).
// A robot square against it has its axle at n . a = c - touchFace(), heading = faceHeading() (toward, or
// toward + 180 when it backs into it).
struct Board { float nx, ny, c, toward; const char* name; };

Board board(bool xb) {
  Board b;
  float side = P(xb ? "e02_xboard_side" : "e02_yboard_side") > 0 ? 1.0f : -1.0f;   // 0 or -1 = low end, 1 = high end
  float pos = P(xb ? "e02_xboard_cm" : "e02_yboard_cm");
  float a = P(xb ? "e02_xboard_deg" : "e02_yboard_deg") * pose::DEG2RAD;
  if (xb) { b.nx = -sinf(a) * side; b.ny = cosf(a) * side; b.c = b.ny * pos; }
  else    { b.nx = cosf(a) * side;  b.ny = sinf(a) * side; b.c = b.nx * pos; }
  b.toward = atan2f(b.ny, b.nx) / pose::DEG2RAD;
  b.name = xb ? "x" : "y";
  return b;
}

// The robot's heading while square against the board in this find.
float faceHeading(const Board& b) { return findRev ? wrap180(b.toward + 180.0f) : b.toward; }

// How far the axle still has to go along n to sit square against the board (own pose).
float toGo(const Board& b) { return b.c - touchFace() - (b.nx * pose::x + b.ny * pose::y); }

// The edge guard must be on before anything here drives (find, e02, go legs). Empty = fine.
const char* edgeOff() {
  return P("safe_edge") > 0 ? "" : "edge guard off (safe_edge 0): set safe_edge 1 (or 2 / 3) first";
}

// ------------------------------------------------------------------ records
void bumpHeader() {
  net::sendLine("H,BUMP,run,ms,n,board,result,cue,ms_start,ms_contact,ms_detect,delay_ms,jolt_g,stall_frac,x_cm,y_cm,"
                "th_deg,travel_cm,overshoot_cm,front_cm,jolts_ignored,square_deg,push_ms,squared");
}

struct Bump {
  int n = 0;
  const char* board = "x";
  bool ok = false;
  char why[72] = "";
  const char* cue = "-";            // jolt+stall, stall
  unsigned long msStart = 0, msContact = 0, msDetect = 0;
  float jolt = 0, stallFrac = 0;    // biggest jolt around the contact (g); measured / commanded wheel speed at detection
  float x = 0, y = 0, th = 0;       // believed pose at the contact (before any reset)
  float travel = 0;                 // creep travel to the contact, cm (wheels)
  float overshoot = 0;              // wheel travel from the contact until the wheels stood still, cm
  float front = 0;                  // front sensor centre zone at detection, cm
  int seams = 0;                    // jolts with no stall after them (seams, vibration): ignored
  float squareDeg = 0;              // heading change while pushing square (the approach angle, opposite sign)
  unsigned long pushMs = 0;
  bool squared = false;
  bool bothSw = false;              // reverse: both back switches pressed at the end of the push (squared = 2)
};

void bumpRecord(const Bump& b) {
  net::logf("D,BUMP,%s,%lu,%d,%s,%s,%s,%lu,%lu,%lu,%ld,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%lu,%d",
            gRunId.c_str(), b.msDetect ? b.msDetect : millis(), b.n, b.board, b.ok ? "bump" : b.why, b.cue, b.msStart, b.msContact, b.msDetect,
            b.msDetect ? (long)(b.msDetect - b.msContact) : -1L, b.jolt, b.stallFrac, b.x, b.y, b.th, b.travel,
            b.overshoot, b.front, b.seams, b.squareDeg, b.pushMs, b.squared ? (b.bothSw ? 2 : 1) : 0);
}

// ------------------------------------------------------------------ bump and square
// Creep along floor heading h until a bump (true), or a stop / failure (false: b.why, gAbortReason).
// Reverse find (findRev): backwards, heading kept, the back switches are the first cue.
bool creep(float h, float maxCm, Bump& b) {
  const float sgn = findRev ? -1.0f : 1.0f;    // direction of motion along the heading
  if (findRev) {
    sw::tick();
    if (!sw::on || sw::raw[0] || sw::raw[1]) {
      snprintf(b.why, sizeof(b.why), "%s", !sw::on ? "back switches not configured (sw_on 0)"
               : sw::raw[0] ? "left back switch pressed before the creep (stuck or wiring)"
               : "right back switch pressed before the creep (stuck or wiring)");
      b.ok = false;
      return false;
    }
  }
  bool swSeen = false;                          // a back switch's first raw closing seen (reverse)
  float distSw = 0;
  float v = P("bump_creep_cms"), kh = P("drive_kh");
  float joltThr = P("bump_jolt_g"), frac = P("bump_stall_frac"), farCm = P("bump_far_cm");
  unsigned long stallMs = (unsigned long)P("bump_stall_ms"), stallOnly = (unsigned long)P("bump_stall_only_ms");
  unsigned long win = (unsigned long)P("bump_jolt_win_ms"), spin = (unsigned long)P("bump_spinup_ms");
  float touchCm = P("bump_touch_cm");
  unsigned long touchMs = (unsigned long)P("bump_touch_ms");
  unsigned long touchSince = 0;
  float distTouch = 0;
  const float cmdRpm = v * 60.0f / (2.0f * 3.14159265f * 1.7f);   // setDrive(): cm/s -> wheel rpm
  float saved = gFrontLimitCm;
  gFrontLimitCm = 0;                       // it is meant to touch what is ahead
  float d0 = pose::dist;
  float base = 0, peak = 0, lastJoltG = 0, distJolt = 0, distStall = 0;
  bool haveBase = false;
  unsigned long t0 = millis(), lastJolt = 0, stallSince = 0;
  unsigned long limit = 4000 + (unsigned long)(maxCm / fmaxf(v, 0.5f) * 1500.0f);
  b.msStart = t0;
  bool ok = false;
  while (true) {
    if (shouldAbort()) break;
    unsigned long now = millis();
    float travel = sgn * (pose::dist - d0);
    float ax, ay, az, gx, gy, gz;
    alvik.get_imu(ax, ay, az, gx, gy, gz);
    if (!haveBase) { base = ax; haveBase = true; }
    float dev = fabsf(ax - base);
    bool armed = now - t0 >= spin;
    if (!armed || dev < joltThr) base += (ax - base) * 0.05f;   // running level (~20 calls); a jolt does not move it
    else if (armed) {
      if (!lastJolt || now - lastJolt > win) { lastJoltG = 0; distJolt = travel; if (!stallSince) b.seams++; }
      lastJolt = now | 1;
      lastJoltG = fmaxf(lastJoltG, dev);
    }
    if (stallSince) peak = fmaxf(peak, dev);
    float sl, sr;
    alvik.get_wheels_speed(sl, sr);
    float meas = sgn * 0.5f * (sl + sr);          // speed along the direction of motion
    bool stalled = armed && meas < frac * cmdRpm;
    if (stalled && !stallSince) { stallSince = now | 1; distStall = travel; peak = dev; }
    if (!stalled) stallSince = 0;
    // reverse: a back switch (switches.h, debounced; the contact is its first raw closing). Not gated by `armed`:
    // a click is a click, even while the wheels speed up.
    if (findRev) {
      bool edge = sw::firstEdge[0] || sw::firstEdge[1];
      if (edge && !swSeen) { swSeen = true; distSw = travel; }
      if (!edge) swSeen = false;                   // a chatter that never became a press
      bool L = sw::left(), R = sw::right();
      if (L || R) {
        alvik.brake();
        b.msDetect = now;
        b.cue = L && R ? "switch-LR" : L ? "switch-L" : "switch-R";
        unsigned long pm = 0;
        for (int k = 0; k < 2; k++)
          if (sw::pressMs[k] && (!pm || (long)(sw::pressMs[k] - pm) < 0)) pm = sw::pressMs[k];
        b.msContact = pm ? pm : now;
        b.travel = swSeen ? distSw : travel;
        bool jolted = lastJolt && (long)(lastJolt + win - b.msContact) >= 0 && (long)(b.msContact + win - lastJolt) >= 0;
        b.jolt = jolted ? lastJoltG : 0;
        if (jolted && b.seams > 0) b.seams--;      // that jolt was this contact, not a seam
        b.stallFrac = meas / cmdRpm;               // (b.front keeps the look's reading: oneBoard)
        if (!pose::at(b.msContact, b.x, b.y, b.th)) { b.x = pose::x; b.y = pose::y; b.th = pose::th(); }
        ok = true;
        break;
      }
    }
    // touch: the front sensor's three middle zones; the nearest of them at most touchCm (face on the board).
    // Nose first only (backwards the front sensor looks away from the board).
    if (touchCm > 0 && !findRev) {
      float dd[5];
      alvik.get_distance(dd[0], dd[1], dd[2], dd[3], dd[4]);
      float near = fminf(dd[1], fminf(dd[2], dd[3]));
      if (near <= touchCm) {
        if (!touchSince) { touchSince = now | 1; distTouch = travel; }
        if (now - touchSince >= touchMs) {
          alvik.brake();
          b.msDetect = now;
          b.cue = "touch";
          bool jolted = lastJolt && (long)(lastJolt + win - touchSince) >= 0 && (long)(touchSince + win - lastJolt) >= 0;
          b.msContact = jolted && (long)(lastJolt - touchSince) < 0 ? lastJolt : touchSince;
          b.travel = jolted && (long)(lastJolt - touchSince) < 0 ? distJolt : distTouch;
          b.jolt = jolted ? lastJoltG : 0;
          b.stallFrac = meas / cmdRpm;
          if (jolted && b.seams > 0) b.seams--;
          b.front = near;
          if (!pose::at(b.msContact, b.x, b.y, b.th)) { b.x = pose::x; b.y = pose::y; b.th = pose::th(); }
          ok = true;
          break;
        }
      } else {
        touchSince = 0;
      }
    }
    if (stallSince) {
      unsigned long s = now - stallSince;
      bool jolted = lastJolt && (long)(lastJolt + win - stallSince) >= 0;   // a jolt at most win before the stall
      if ((jolted && s >= stallMs) || s >= stallOnly) {
        alvik.brake();
        b.msDetect = now;
        b.cue = jolted ? "jolt+stall" : "stall";
        b.msContact = jolted && (long)(lastJolt - stallSince) < 0 ? lastJolt : stallSince;
        b.travel = jolted && (long)(lastJolt - stallSince) < 0 ? distJolt : distStall;
        b.jolt = jolted ? fmaxf(peak, lastJoltG) : peak;
        b.stallFrac = meas / cmdRpm;
        if (jolted && b.seams > 0) b.seams--;              // that jolt was the bump, not a seam
        if (!findRev) {                            // backwards b.front keeps the look's reading (oneBoard)
          float d[5];
          alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
          b.front = d[2];
        }
        if (!pose::at(b.msContact, b.x, b.y, b.th)) { b.x = pose::x; b.y = pose::y; b.th = pose::th(); }
        if (!findRev && b.front > farCm && b.front < 100.0f) {   // backwards the front sensor looks away: no check
          snprintf(b.why, sizeof(b.why), "stalled with something %.0f cm ahead: not at a board", b.front);
          break;
        }
        ok = true;
        break;
      }
    }
    if (travel > maxCm) {
      if (findRev) snprintf(b.why, sizeof(b.why), "no board behind within %.0f cm", maxCm);
      else snprintf(b.why, sizeof(b.why), "no bump within %.0f cm", maxCm);
      break;
    }
    if (now - t0 > limit) { snprintf(b.why, sizeof(b.why), "creep timed out"); break; }
    float herr = wrap180(h - pose::th());
    setDrive(sgn * v, constrain(kh * herr, -20.0f, 20.0f));   // turn rate keeps its sign backwards (setDrive)
    delay(5);
  }
  alvik.brake();
  // overshoot: wheel travel from the contact until the wheels stand still (at most 400 ms)
  unsigned long ts = millis();
  while (millis() - ts < 400) {
    if (shouldAbort()) { ok = false; break; }
    float sl, sr;
    alvik.get_wheels_speed(sl, sr);
    if (millis() - ts > 60 && fabsf(sl) < 0.5f && fabsf(sr) < 0.5f) break;
    delay(5);
  }
  if (b.msDetect) b.overshoot = sgn * (pose::dist - d0) - b.travel;
  gFrontLimitCm = saved;
  b.ok = ok;
  return ok;
}

// Push gently so the whole touching face lies on the board (reverse: push backwards; done once both back
// switches are pressed, or the heading stops changing). False on a stop, or if it never settled.
bool square(Bump& b) {
  float rpm = P("push_rpm"), still = P("push_still_deg");
  if (findRev) rpm = -rpm;
  unsigned long maxMs = (unsigned long)P("push_ms"), minMs = (unsigned long)P("push_min_ms");
  float saved = gFrontLimitCm;
  gFrontLimitCm = 0;
  float h0 = heading(), hRef = h0;
  unsigned long t0 = millis(), tRef = t0;
  bool settled = false, ok = true;
  alvik.set_wheels_speed(rpm, rpm);
  while (true) {
    if (shouldAbort()) { ok = false; break; }
    unsigned long now = millis();
    if (now - tRef >= 250) {
      float h = heading();
      if (now - t0 >= minMs && findRev && sw::left() && sw::right()) { settled = true; break; }   // flush: both levers
      if (now - t0 >= minMs && fabsf(wrap180(h - hRef)) <= still) { settled = true; break; }
      hRef = h;
      tRef = now;
    }
    if (now - t0 >= maxMs) break;
    delay(5);
  }
  alvik.brake();
  b.pushMs = millis() - t0;
  if (ok && !waitMs(250)) ok = false;       // let go of the push before reading the heading
  gFrontLimitCm = saved;
  b.squareDeg = wrap180(heading() - h0);
  b.squared = settled;
  b.bothSw = findRev && sw::left() && sw::right();
  if (ok && !settled) {
    snprintf(b.why, sizeof(b.why), "still turning after %lu ms of pushing: not square", b.pushMs);
    b.ok = false;
    gFail = b.why;
    return false;
  }
  return ok;
}

// Drive straight along floor heading h for cm (negative = backwards, heading kept), on the own pose.
bool straight(float h, float cm, float cms) {
  float ux = cosf(h * pose::DEG2RAD), uy = sinf(h * pose::DEG2RAD);
  return driveLine(pose::x, pose::y, pose::x + cm * ux, pose::y + cm * uy, h, cms, cm < 0) == DRIVE_OK;
}

// Turn to face (wx, wy) and drive straight to it on the own pose. Front stop while driving forward: the usual
// safe_front_cm, but only for what is within the rest of the leg + front_end_cm (as follow drives), so a leg
// that ends facing a board (the return to a start spot near the y board) is not stopped by it; none while
// turning in place (it may face the board it just backed away from). pause: legPause() after the turn and the
// drive (routes).
bool legPause();
bool driveTo(float wx, float wy, bool pause) {
  float dx = wx - pose::x, dy = wy - pose::y;
  if (sqrtf(dx * dx + dy * dy) < 1.0f) return true;
  float hb = atan2f(dy, dx) / pose::DEG2RAD;
  gFrontLimitCm = 0;
  if (!turnToFloor(hb) || (pause && !legPause())) return false;
  dx = wx - pose::x; dy = wy - pose::y;               // a camera fix during the pause may have moved the pose
  float hb2 = atan2f(dy, dx) / pose::DEG2RAD;
  if (fabsf(wrap180(hb2 - hb)) > 2.0f && !turnToFloor(hb2)) return false;
  gFrontEndBase = P("safe_front_cm");
  gFrontLimitCm = gFrontEndBase;
  int r = driveLine(pose::x, pose::y, wx, wy, hb2, P("e02_cms"), false);
  gFrontEndBase = -1;
  gFrontLimitCm = 0;
  return r == DRIVE_OK && (!pause || legPause());
}

// ------------------------------------------------------------------ find
struct FindRec {
  const char* result = "failed";
  char why[96] = "";
  int knownBefore = 0;
  float bx = 0, by = 0, bth = 0;          // believed pose before the find
  float b2x = 0, b2y = 0, b2th = 0;       // believed pose squared on the y board, just before the reset
  float rx = 0, ry = 0, rth = 0;          // the reset pose (squared on the y board)
  float ax = 0, ay = 0, ath = 0;          // pose at the end (after backing off)
  float corr1 = 0, corr1Deg = 0;          // what bump 1 changed: distance from the x board, heading
  float corr2 = 0, corr2Deg = 0;          // bump 2: distance from the y board, heading
  float turnK = 1;                        // board-to-board angle / gyro turn between the squarings
  float shift = 0;                        // how far re-tracing the path with turn_k moved the position, cm
};

// face_cm: the face that touched (the back face when reverse); reverse: 1 = it backed into the boards (2026-10-25b)
void findRecord(const FindRec& f, unsigned long ms) {
  net::sendLine("H,FIND,run,result,known_before,before_x,before_y,before_th,bump2_x,bump2_y,bump2_th,reset_x,"
                "reset_y,reset_th,after_x,after_y,after_th,corr1_cm,corr1_deg,corr2_cm,corr2_deg,turn_k,retrace_cm,"
                "face_cm,ms_total,battery,reverse");
  String res = String(f.result);
  if (f.why[0]) res += String(": ") + String(f.why);
  net::logf("D,FIND,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,"
            "%.2f,%.2f,%lu,%d,%d", gRunId.c_str(), res.c_str(), f.knownBefore, f.bx, f.by, f.bth, f.b2x, f.b2y, f.b2th,
            f.rx, f.ry, f.rth, f.ax, f.ay, f.ath, f.corr1, f.corr1Deg, f.corr2, f.corr2Deg, f.turnK, f.shift,
            touchFace(), ms, alvik.get_battery_charge(), findRev ? 1 : 0);
}


// One board: turn to face it, drive near it (pose known), creep, bump, check the bump is where the board should
// be (pose known), square. Sends the BUMP record. Returns false with f.why set on any failure.
// Reverse (findRev, pose always known then): a board further than e02_find_turn_cm is first approached nose first
// to that distance; then it turns its back to the board and creeps backwards, at most e02_find_back_margin_cm
// (+ slack, axle never within BACK_KEEP_CM of the line) past where the pose says the board is.
bool oneBoard(const Board& bd, int n, bool known, FindRec& f, Bump& b) {
  b.n = n;
  b.board = bd.name;
  rawlog::step = n == 1 ? 1 : 5;
  gFrontLimitCm = 0;                         // the board is ahead (it may be inside safe_front_cm)
  float maxCm = P("e02_find_max_cm");
  // how far the own pose may be off by now: dead reckoning since the pose was last set drifts
  float slack = P("e02_find_tol_pct") / 100.0f * sinceSet;
  if (findRev) {
    // face it first: the front sensor must see the board before the robot turns its back to it
    if (!turnToFloor(bd.toward)) {
      snprintf(f.why, sizeof(f.why), "turn to the %s board %s", bd.name, gAbortReason.length() ? "stopped" : "failed");
      return false;
    }
    float along = bd.c - (bd.nx * pose::x + bd.ny * pose::y);       // axle to the face, by the own pose
    float pre = fminf(along - P("e02_find_turn_cm"), P("e02_transit_max_cm"));
    if (pre > 2.0f) {
      travelCap += pre;
      gFrontLimitCm = 0.5f * P("e02_find_slow_cm");   // soft: a board nearer than the pose says ends it early
      gFrontSoft = true;
      bool ok = straight(bd.toward, pre, P("e02_cms"));
      gFrontSoft = false;
      gFrontLimitCm = 0;
      gFrontLowSince = 0;
      if (!ok && gStepStop == "obstacle") { ok = true; gStepStop = ""; }
      if (!ok) {
        if (gAbortReason == "plywood edge ahead")
          snprintf(f.why, sizeof(f.why), "edge guard tripped before the %s board (an open plywood edge, not a board)", bd.name);
        else snprintf(f.why, sizeof(f.why), "drive towards the %s board stopped (%s)", bd.name,
                      gAbortReason.length() ? gAbortReason.c_str() : gFail.c_str());
        return false;
      }
    }
    // LOOK: standing still, facing the board, the front sensor must see it no further than the map puts it (+
    // e02_find_see_cm + slack). A removed board over an open edge (the remove-a-board test) reads the room beyond:
    // the find fails here instead of backing blind towards the edge. BUMP front_cm = this reading (reverse).
    along = bd.c - (bd.nx * pose::x + bd.ny * pose::y);
    float expectSee = along - (P("tof_ahead_cm") + P("tof_recess_cm"));   // sensor to the face
    float seen = lookAhead();
    b.front = seen;
    if (seen < 0) {
      snprintf(f.why, sizeof(f.why), "stopped (%s)", gAbortReason.length() ? gAbortReason.c_str() : "look");
      return false;
    }
    if (seen > expectSee + P("e02_find_see_cm") + slack) {
      snprintf(b.why, sizeof(b.why), "no board seen (front %.0f cm; the board should be %.0f)", seen, expectSee);
      bumpRecord(b);
      snprintf(f.why, sizeof(f.why), "%s board: %s (not backing in)", bd.name, b.why);
      return false;
    }
    if (!turnToFloor(faceHeading(bd))) {         // its back to the board
      snprintf(f.why, sizeof(f.why), "turn away from the %s board %s", bd.name, gAbortReason.length() ? "stopped" : "failed");
      return false;
    }
    // how far past the board's line it may back (second layer, after the look): margin + slack, but never so far
    // that the axle comes nearer than e02_find_keep_cm to that line.
    // Where the boards stand (Nick, 2026-10-25): on the cement, pressed against the side of the plywood, 6-8 inches
    // (15-20 cm) tall, so a board's line IS the plywood's edge. With the board there, the back face (6.1 cm behind
    // the axle) touches it while the rear caster (4.1 cm behind the axle) is still 2 cm inside the edge. With the
    // board gone AND the look fooled, the default cap (keep 2 cm) would let the caster reach 2.1 cm past the edge:
    // kept anyway, because a cap that keeps the caster on (keep 4.6) stopped 8 of 30 simulated finds WITH the board
    // there ("no board behind", each a hold for Nick) against 2 of 30 (camera fix 0.8 cm / 2 deg; build report
    // e02-oct25-backing-unc.md). The look is what protects the caster; DOE: doe-backlog.md "Backing cap".
    float expect = toGo(bd);                       // back face to the board, by the own pose
    float over = fminf(P("e02_find_back_margin_cm") + slack, P("e02_back_face_cm") - P("e02_find_keep_cm"));
    maxCm = fminf(P("e02_find_max_cm"), fmaxf(0.0f, expect + fmaxf(0.0f, over)));
  } else if (!turnToFloor(bd.toward)) {
    snprintf(f.why, sizeof(f.why), "turn to the %s board %s", bd.name, gAbortReason.length() ? "stopped" : "failed");
    return false;
  }
  if (known && !findRev) {
    float expect = toGo(bd);
    float margin = P("e02_find_margin_cm") + slack;
    maxCm = fminf(maxCm, fmaxf(0.0f, expect) + margin);
    float pre = fminf(expect - P("e02_find_slow_cm"), P("e02_transit_max_cm"));
    if (pre > 2.0f) {
      travelCap += pre;
      // the front sensor ends it early if the board is nearer than the pose says (soft: then it creeps from there)
      gFrontLimitCm = 0.5f * P("e02_find_slow_cm");
      gFrontSoft = true;
      bool ok = straight(bd.toward, pre, P("e02_cms"));
      gFrontSoft = false;
      gFrontLimitCm = 0;
      gFrontLowSince = 0;
      if (!ok && gStepStop == "obstacle") { ok = true; gStepStop = ""; }
      if (!ok) {
        snprintf(f.why, sizeof(f.why), "drive towards the %s board stopped (%s)", bd.name,
                 gAbortReason.length() ? gAbortReason.c_str() : gFail.c_str());
        return false;
      }
      maxCm = fminf(P("e02_find_max_cm"), P("e02_find_slow_cm") + margin);   // the creep's own cap
    }
  }
  rawlog::step = n == 1 ? 2 : 6;
  bool ok = creep(faceHeading(bd), fmaxf(maxCm, 1.0f), b);
  if (ok && known) {
    // where the bump was against where the own pose said the board would be
    // sooner: something in the way (another robot?); later can only be the board (nothing stands beyond it)
    float off = bd.c - touchFace() - (bd.nx * b.x + bd.ny * b.y);
    if (off > P("e02_find_tol_cm") + slack) {
      snprintf(b.why, sizeof(b.why), "bump %.1f cm sooner than the %s board should be", off, bd.name);
      b.ok = ok = false;
    }
  }
  if (!ok) {
    bumpRecord(b);
    if (gAbortReason == "plywood edge ahead")
      snprintf(f.why, sizeof(f.why), "edge guard tripped before the %s board (an open plywood edge, not a board)", bd.name);
    else if (gAbortReason.length()) snprintf(f.why, sizeof(f.why), "stopped (%s)", gAbortReason.c_str());
    else snprintf(f.why, sizeof(f.why), "%s board: %s", bd.name, b.why);
    return false;
  }
  rawlog::step = n == 1 ? 3 : 7;
  bool sq = square(b);
  bumpRecord(b);
  if (!sq) {
    if (gAbortReason.length()) snprintf(f.why, sizeof(f.why), "stopped (%s)", gAbortReason.c_str());
    else snprintf(f.why, sizeof(f.why), "%s board: %s", bd.name, b.why);
    return false;
  }
  return true;
}

void runFind() {
  unsigned long t0 = millis();
  bumpHeader();
  FindRec f;
  bool known = pose::known;
  f.knownBefore = known ? 1 : 0;
  Board bx = board(true), by = board(false);
  // reverse (2026-10-25b): only with the back switches and a pose; otherwise nose first, and it says why
  findRev = false;
  if (P("e02_find_reverse") > 0) {
    sw::configure();
    const char* no = !(sw::on && P("sw_back") > 0) ? "the back switches are not set up (sw_on 1, sw_back 1)"
                   : !known ? "no pose (backing blind with no pose is not safe: send a camera fix first)" : "";
    if (no[0]) net::logf("M,%s find: not backing in: %s; nose first instead", net::name, no);
    else findRev = true;
  }
  float face = touchFace(), back = P("e02_find_back_cm");
  if (edgeOff()[0]) {
    snprintf(f.why, sizeof(f.why), "%s", edgeOff());
    gFail = f.why;
    findRecord(f, 0);
    net::logf("M,%s find refused: %s", net::name, f.why);
    return;
  }
  begin();
  finding = true;
  if (!known) {
    pose::start(true, 0, 0, bx.toward);      // no pose: take it that it faces the x board (FIND known_before 0)
    net::logf("M,%s find: position unknown (no find or camera fix since power-on): assuming it faces the x board",
              net::name);
  }
  f.bx = pose::x; f.by = pose::y; f.bth = pose::th();
  travelCap = 2 * P("e02_find_max_cm") + 2 * back + 20;
  bool ok = false, reset1 = false;
  do {
    // pose known: first to the staging point e02_find_stage_cm from both faces, near the corner, so neither
    // approach is long (a long drive along the x board with a gyro a few degrees off would rub it)
    float stage = P("e02_find_stage_cm");
    float det = bx.nx * by.ny - bx.ny * by.nx;
    if (known && stage > 0 && fabsf(det) > 0.5f) {
      float c1 = bx.c - stage, c2 = by.c - stage;
      float sx = (c1 * by.ny - bx.ny * c2) / det, sy = (bx.nx * c2 - c1 * by.nx) / det;
      float d = sqrtf((sx - pose::x) * (sx - pose::x) + (sy - pose::y) * (sy - pose::y));
      travelCap += d;                          // at most the board's diagonal: always worth it
      rawlog::step = 0;
      if (!driveTo(sx, sy, false)) { snprintf(f.why, sizeof(f.why), "drive to the staging point stopped"); break; }
    }
    Bump b1;
    if (!oneBoard(bx, 1, known, f, b1)) break;
    // bump 1: the distance from the x board and the heading are now exact
    float off1 = toGo(bx);
    float h1 = faceHeading(bx);                // its heading square on the x board (toward, or + 180 reverse)
    f.corr1 = off1;
    f.corr1Deg = wrap180(h1 - pose::th());
    pose::set(pose::x + off1 * bx.nx, pose::y + off1 * bx.ny, h1, 0.05f, 0.3f);
    reset1 = true;
    // the distance from the x board and the heading are exact now; the position along it is not: keep the slack
    float a1x = pose::x, a1y = pose::y;
    pathG0 = heading();
    pathLastH = pathG0;
    pathTurn = 0;
    nPath = 0;
    segDs = segRelW = 0;
    pathOn = true;
    rawlog::step = 4;
    if (findRev) {
      // off the board nose first (edge guard on; front stop at safe_front_cm: another robot there ends the find)
      gFrontLimitCm = P("safe_front_cm");
      bool good = straight(h1, back, 3.0f);
      gFrontLimitCm = 0;
      if (!good) { snprintf(f.why, sizeof(f.why), "driving off the x board stopped (%s)", gAbortReason.length() ? gAbortReason.c_str() : "failed"); break; }
    } else {
      gFrontLimitCm = 0;
      if (!straight(bx.toward, -back, 3.0f)) { snprintf(f.why, sizeof(f.why), "backing off the x board stopped"); break; }
    }
    Bump b2;
    if (!oneBoard(by, 2, known, f, b2)) break;
    pathPush();
    pathOn = false;
    // the gyro's turn between the two squarings against the boards' own angle: its scale on this stretch
    float rel2 = pathTurn;                                  // unwrapped
    float trueTurn = wrap180(by.toward - bx.toward);
    // the body's true turn is the boards' angle + a whole number of turns: the one nearest the gyro's
    while (trueTurn - rel2 > 180.0f) trueTurn -= 360.0f;
    while (rel2 - trueTurn > 180.0f) trueTurn += 360.0f;
    float k = 1;
    if (fabsf(rel2) > 30.0f && fabsf(trueTurn) > 30.0f) {
      k = trueTurn / rel2;
      if (k < 0.8f || k > 1.25f) {
        snprintf(f.why, sizeof(f.why), "the gyro turned %.1f deg between the boards but they are %.1f deg apart "
                 "(board sides or angles set wrong?)", rel2, trueTurn);
        break;
      }
    }
    f.turnK = k;
    // re-trace the path from square 1 with that scale
    // (each step's heading = the heading at square 1 + k x the gyro's turn since; ds is signed along the heading)
    float px = a1x, py = a1y;
    for (int i = 0; i < nPath; i++) {
      float th = (h1 + k * pathRel[i]) * pose::DEG2RAD;
      px += pathDs[i] * cosf(th);
      py += pathDs[i] * sinf(th);
    }
    float h2 = faceHeading(by);
    f.b2x = pose::x; f.b2y = pose::y; f.b2th = pose::th();
    f.shift = sqrtf((px - pose::x) * (px - pose::x) + (py - pose::y) * (py - pose::y));
    f.corr2 = by.c - face - (by.nx * pose::x + by.ny * pose::y);
    f.corr2Deg = wrap180(h2 - pose::th());
    float off2 = by.c - face - (by.nx * px + by.ny * py);
    pose::set(px + off2 * by.nx, py + off2 * by.ny, h2, 0.05f, 0.3f);
    sinceSet = 0;
    backSince = 0;
    f.rx = pose::x; f.ry = pose::y; f.rth = pose::th();
    rawlog::step = 8;
    if (findRev) {
      gFrontLimitCm = P("safe_front_cm");
      bool good = straight(h2, back, 3.0f);
      gFrontLimitCm = 0;
      if (!good) { snprintf(f.why, sizeof(f.why), "driving off the y board stopped (%s)", gAbortReason.length() ? gAbortReason.c_str() : "failed"); break; }
    } else {
      gFrontLimitCm = 0;
      if (!straight(by.toward, -back, 3.0f)) { snprintf(f.why, sizeof(f.why), "backing off the y board stopped"); break; }
    }
    ok = true;
  } while (false);
  pathOn = false;
  f.ax = pose::x; f.ay = pose::y; f.ath = pose::th();
  if (ok) f.result = "ok";
  else {
    if (!known) pose::known = false;    // it never learnt both coordinates
    if (!gAbortReason.length() && !gFail.length()) gFail = f.why;
    (void)reset1;
  }
  findRecord(f, millis() - t0);
  if (ok) net::logf("M,%s find: at %.1f, %.1f facing %.1f deg (corrections %.1f cm / %.1f deg and %.1f cm / %.1f deg; "
                    "turn_k %.3f)", net::name, pose::x, pose::y, pose::th(), f.corr1, f.corr1Deg, f.corr2, f.corr2Deg,
                    f.turnK);
  else net::logf("M,%s find failed: %s", net::name, f.why);
  end();
  findRev = false;
}

// ------------------------------------------------------------------ routes
struct Pt { float x, y; };
struct Mark { bool set = false; float x = 0, y = 0, th = 0; unsigned long ms = 0; };
Mark mStart, mFar, mEnd;

// Which way the L / square turns: param +1 left, -1 right, 0 = towards the plywood's middle, i.e. the side
// away from the x board as seen from the start heading.
float turnSign(const char* param) {
  float v = P(param);
  if (v > 0) return 1.0f;
  if (v < 0) return -1.0f;
  Board bx = board(true);
  float h = mStart.th * pose::DEG2RAD;
  return (-sinf(h) * bx.nx + cosf(h) * bx.ny) > 0 ? -1.0f : 1.0f;   // left points at the x board: turn right
}

// Waypoints in the start frame (x ahead, y left); far = index of the far point. Returns the count.
int waypoints(int route, Pt* w, int& far) {
  if (route == 1) {
    float L = P("e02_straight_cm");
    w[0] = {L, 0}; w[1] = {0, 0}; far = 0; return 2;
  }
  if (route == 2) {
    float a = P("e02_l_cm"), t = turnSign("e02_l_turn");
    w[0] = {a, 0}; w[1] = {a, t * a}; w[2] = {a, 0}; w[3] = {0, 0}; far = 1; return 4;
  }
  float s = P("e02_square_cm"), t = turnSign("e02_square_turn");
  w[0] = {s, 0}; w[1] = {s, t * s}; w[2] = {0, t * s}; w[3] = {0, 0}; far = 1; return 4;
}

float routeLen(int route) {
  if (route == 1) return 2 * P("e02_straight_cm");
  if (route == 2) return 4 * P("e02_l_cm");
  return 4 * P("e02_square_cm");
}

void mark(Mark& m) { m.set = true; m.x = pose::x; m.y = pose::y; m.th = pose::th(); m.ms = millis(); }

void e02Header() {
  net::sendLine("H,E02,run,route,mode,result,fix_k,start_x,start_y,start_th,far_goal_x,far_goal_y,far_x,far_y,far_th,"
                "end_x,end_y,end_th,ms_start,ms_far,ms_end,path_cm,legs,fixes_used,fixes_rejected,gyro_scale,"
                "gyro_drift_dps,wheel_scale,battery");
}

void e02Record(int route, int mode, const String& result, int legs) {
  Pt w[4];
  int far = 0;
  waypoints(route >= 1 && route <= 3 ? route : 1, w, far);
  float c = cosf(mStart.th * pose::DEG2RAD), s = sinf(mStart.th * pose::DEG2RAD);
  float gx = mStart.x + c * w[far].x - s * w[far].y, gy = mStart.y + s * w[far].x + c * w[far].y;
  net::logf("D,E02,%s,%d,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%.1f,%d,%d,"
            "%d,%.4f,%.4f,%.4f,%d", gRunId.c_str(), route, mode, result.c_str(), P("e02_fix_k"), mStart.x, mStart.y,
            mStart.th, gx, gy, mFar.x, mFar.y, mFar.th, mEnd.x, mEnd.y, mEnd.th, mStart.ms, mFar.ms, mEnd.ms,
            travelAbs, legs, fixesUsed, fixesRejected, P("gyro_scale"), P("gyro_drift_dps"), P("wheel_scale"),
            alvik.get_battery_charge());
}

// Stand still e02_leg_pause_ms (camera fixes arrive and are used while it stands).
bool legPause() {
  unsigned long ms = (unsigned long)P("e02_leg_pause_ms");
  return ms == 0 || waitMs(ms);
}

// A route leg: driveTo() with the KC pauses.
bool legTo(float wx, float wy) { return driveTo(wx, wy, true); }

// At the far point and the end: if the pose (camera fixes during the pause included: KC) says it is more than
// e02_redo_cm off, drive there once more. Without fixes the pose is on the spot and nothing happens.
bool redo(float wx, float wy) {
  float r = P("e02_redo_cm"), dx = wx - pose::x, dy = wy - pose::y;
  if (r <= 0 || sqrtf(dx * dx + dy * dy) <= fmaxf(r, 1.0f)) return true;
  return legTo(wx, wy);
}

bool turnTo(float h) {
  gFrontLimitCm = 0;
  return turnToFloor(h) && legPause();
}

// The condition's corrections for the route itself (2026-10-21b): the laptop runs the find, the transit to the start
// spot and the turn to the start heading on its best corrections, and asks for the condition's (U: 1, 0, 1) from the
// route's start on, so an uncalibrated start turn does not aim the whole route off (what the camera measures starts
// at the start mark). The heading does not jump when gyro_scale changes (motion.h).
void routeCorrections() {
  static const char* names[3][2] = {{"e02_route_gyro_scale", "gyro_scale"}, {"e02_route_gyro_drift_dps", "gyro_drift_dps"},
                                    {"e02_route_wheel_scale", "wheel_scale"}};
  for (int k = 0; k < 3; k++) {
    float v = P(names[k][0]);
    if (v == UNSET) continue;
    Param* p = findParam(names[k][1]);
    if (p) p->value = v;
  }
}

// ------------------------------------------------------------------ camera-steered legs (mode 1)
struct Leg { char kind; float arg; long id; };   // g go cm, t turn deg, m mark (0 start, 1 far, 2 end)
const int NQ = 4;
Leg q[NQ];
int qHead = 0, qLen = 0;
long lastId = 0;
bool endMarked = false;

// Idle legs: `go` / `turn` with no run going (the laptop parks a robot, finds it in the picture, brings a lost
// one back). One at a time, run from loop() (idleTick): the edge guard learns the plywood first, then the leg
// runs with the same checks as in mode 1 (soft front stop and edge guard, length limits, stop), and a LEG
// record (run "-"). The pose is kept if it is known.
bool idleLeg = false;           // an idle leg is running
bool idlePending = false;
Leg idleQ = {'g', 0, 0};

void legHeader() { net::sendLine("H,LEG,run,id,kind,value,result,ms,x_cm,y_cm,th_deg,info"); }

const char* kindName(char k) { return k == 'g' ? "go" : k == 't' ? "turn" : "mark"; }

// columns as legHeader(): value = cm / deg / mark number, result = queued, started, done, failed, refused, dup
void legRecord(long id, char kind, float value, const char* result, const char* info) {
  net::logf("D,LEG,%s,%ld,%s,%.2f,%s,%lu,%.2f,%.2f,%.2f,%s", active && !idleLeg ? gRunId.c_str() : "-", id, kindName(kind), value,
            result, millis(), pose::x, pose::y, pose::th(), info);
}

// A finished leg's info gets the front sensor's middle zone (cm) standing still: " front=<cm>" (2026-10-22b). The
// laptop compares it with where the camera and the map put the board or robot ahead: a free check of the position
// whenever the robot stops facing something (Nick, 2026-10-22).
void addFront(char* info, size_t n) {
  float d[5];
  alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
  size_t k = strlen(info);
  if (k + 14 < n) snprintf(info + k, n - k, "%sfront=%.1f", k ? " " : "", d[2]);
}

// go <cm> [id] | turn <deg> [id] | mark start|far|end [id]  (from handleCommand; `e02 start|far|end` = mark)
void legCommand(const String& verb, const String& rest, bool running) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s", rest.c_str());
  char* tok[4];
  int n = 0;
  for (char* t = strtok(buf, " "); t && n < 4; t = strtok(nullptr, " ")) tok[n++] = t;
  char kind = verb == "go" ? 'g' : verb == "turn" ? 't' : 'm';
  float arg = 0;
  long id = n > 1 ? atol(tok[1]) : 0;
  bool idle = !running && kind != 'm';
  if (!idle && (!running || !active || !mode1 || idleLeg)) {
    legHeader();
    legRecord(id, kind, arg, "refused", idleLeg ? "a leg is running" : kind == 'm' ? "marks are for a camera-steered e02 run"
                                                 : "busy: not in a camera-steered e02 run");
    return;
  }
  if (idle) legHeader();
  bool good = n >= 1;
  if (good && kind == 'm') {
    if (!strcasecmp(tok[0], "start")) arg = 0;
    else if (!strcasecmp(tok[0], "far")) arg = 1;
    else if (!strcasecmp(tok[0], "end")) arg = 2;
    else good = false;
  } else if (good) {
    char* e = nullptr;
    arg = strtof(tok[0], &e);
    good = e != tok[0];
  }
  if (!good) { legRecord(id, kind, arg, "refused", "could not read it (go <cm> | turn <deg> | mark start|far|end)"); return; }
  if (kind == 'g' && (arg > P("e02_go_max_cm") || arg < -P("e02_back_max_cm"))) {
    legRecord(id, kind, arg, "refused", "too long (e02_go_max_cm / e02_back_max_cm)");
    return;
  }
  if (kind == 't' && fabsf(arg) > 180.0f) { legRecord(id, kind, arg, "refused", "turns are -180 to 180 deg"); return; }
  if (kind == 'g' && edgeOff()[0]) { legRecord(id, kind, arg, "refused", edgeOff()); return; }
  if (kind == 'g' && borderLatched && borderTowards(arg) && !(P("border_seek") > 0 && arg > 0)) {
    legRecord(id, kind, arg, "refused", "virtual border: towards the open edge (the laptop clears it)");
    return;
  }
  if (kind == 'g' && arg < 0 && backSince - arg > P("e02_back_max_cm") + 0.05f) {
    legRecord(id, kind, arg, "refused", "backing blind: more than e02_back_max_cm since the last camera fix");
    return;
  }
  if (id > 0 && id <= lastId) { legRecord(id, kind, arg, "dup", ""); return; }
  if (idle) {
    if (idlePending) { legRecord(id, kind, arg, "refused", "a leg is waiting"); return; }
    if (id > 0) lastId = id;
    idleQ = {kind, arg, id};
    idlePending = true;
    legRecord(id, kind, arg, "queued", "");
    return;
  }
  if (qLen >= NQ) { legRecord(id, kind, arg, "refused", "queue full"); return; }
  if (id > 0) lastId = id;
  q[(qHead + qLen) % NQ] = {kind, arg, id};
  qLen++;
  legRecord(id, kind, arg, "queued", "");
}

bool runLeg(const Leg& L, char* info, size_t n) {
  info[0] = 0;
  if (L.kind == 'g') {
    // forward: front stop only for what lies within the rest of the leg + front_end_cm (at most safe_front_cm),
    // so a leg may approach a board it knows of (ending a few cm short of it) and is still stopped by anything
    // nearer than the leg's end; backwards: no front check (blind; capped by backSince)
    float fe = P("front_end_cm");
    gFrontEndBase = L.arg > 0 ? P("safe_front_cm") : -1;
    gFrontLimitCm = L.arg > 0 ? fminf(P("safe_front_cm"), L.arg + fe) : 0;
    float d0 = pose::dist;
    bool ok = straight(pose::th(), L.arg, P("e02_cms"));
    if (L.arg < 0) backSince += fabsf(pose::dist - d0);
    gFrontEndBase = -1;
    gFrontLimitCm = 0;
    return ok;
  }
  if (L.kind == 't') {
    gFrontLimitCm = 0;
    return turnToFloor(wrap180(pose::th() + L.arg));
  }
  int m = (int)L.arg;
  if (m == 0) mark(mStart);
  else if (m == 1) { mark(mFar); if (!waitMs((unsigned long)P("e02_pause_ms"))) return false; }
  else { mark(mEnd); endMarked = true; }
  snprintf(info, n, "%s", m == 0 ? "start" : m == 1 ? "far" : "end");
  return true;
}

// Called from loop(): run a waiting idle leg.
void idleTick() {
  if (!idlePending) return;
  idlePending = false;
  Leg L = idleQ;
  gAbortReason = "";
  gFail = "";
  gStepStop = "";
  begin();
  idleLeg = true;
  gFrontSoft = true;
  gEdgeSoft = true;
  travelCap = fabsf(L.arg) + 10.0f;
  edge::begin();
  legRecord(L.id, L.kind, L.arg, "started", "");
  gFrontLimitCm = 0;     // standing still: nothing to stop (a board close in front is fine); runLeg sets the leg's own
  bool good = waitMs((unsigned long)P("edge_learn_ms") + 50);   // still: the edge guard learns the plywood
  char info[48] = "";
  if (good) good = runLeg(L, info, sizeof(info));
  alvik.brake();
  if (gAbortReason.length()) legRecord(L.id, L.kind, L.arg, "failed", gAbortReason.c_str());
  else if (good) { addFront(info, sizeof(info)); legRecord(L.id, L.kind, L.arg, "done", info); }
  else {
    char why[96];
    if (gStepStop == "edge") snprintf(why, sizeof(why), "plywood edge (%s)", edge::lastCue);
    else snprintf(why, sizeof(why), "%s", gStepStop.length() ? gStepStop.c_str() : gFail.length() ? gFail.c_str() : "failed");
    legRecord(L.id, L.kind, L.arg, "failed", why);
  }
  edge::stop();
  idleLeg = false;
  end();
  gAbortReason = "";
  gFail = "";
}

// ------------------------------------------------------------------ camera fix
// fix <x> <y> <th|-> <t_wall>   (t_wall: the laptop's clock at the camera picture, logged as given)
void fixCommand(const String& rest, bool running) {
  char buf[128];
  snprintf(buf, sizeof(buf), "%s", rest.c_str());
  char* tok[5];
  int n = 0;
  for (char* t = strtok(buf, " "); t && n < 5; t = strtok(nullptr, " ")) tok[n++] = t;
  net::sendLine("H,FIX,run,ms,t_wall,cam_x,cam_y,cam_th,k,result,before_x,before_y,before_th,after_x,after_y,after_th");
  if (n < 4) { net::sendLine("M,usage: fix <x> <y> <heading|-> <t_wall>"); return; }
  float x = strtof(tok[0], nullptr), y = strtof(tok[1], nullptr);
  float th = strcmp(tok[2], "-") ? strtof(tok[2], nullptr) : NAN;
  // idle (or during an idle leg), K (mode 1): replace; KC: blend
  float k = !running || mode1 || idleLeg ? 1.0f : constrain(P("e02_fix_k"), 0.0f, 1.0f);
  float bx = pose::x, by = pose::y, bth = pose::known ? pose::th() : NAN;
  const char* result;
  if (running && !active) result = "rejected:another run is going";
  else if (finding) result = "rejected:finding (the boards set the pose)";
  else if (running && millis() - lastMoveMs < (unsigned long)P("e02_fix_still_ms")) result = "rejected:moving";
  else if (!pose::known) {
    if (isNaN(th)) result = "rejected:the first fix needs a heading";
    else { pose::set(x, y, th, 0.5f, 1.0f); result = "set"; }
  } else {
    float nth = isNaN(th) ? pose::th() : wrap180(pose::th() + k * wrap180(th - pose::th()));
    pose::set(pose::x + k * (x - pose::x), pose::y + k * (y - pose::y), nth, 0.5f, 1.0f);
    result = k >= 1.0f ? "replaced" : "blended";
  }
  bool used = strncmp(result, "rejected", 8) != 0;
  if (used) fixesUsed++; else fixesRejected++;
  if (used && (k >= 1.0f || !strcmp(result, "set"))) { sinceSet = 0; backSince = 0; }   // a replacing fix: the camera's pose
  net::logf("D,FIX,%s,%lu,%s,%.2f,%.2f,%.2f,%.2f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", running ? gRunId.c_str() : "-",
            millis(), tok[3], x, y, th, k, result, bx, by, bth, pose::x, pose::y, pose::known ? pose::th() : NAN);
}

// ------------------------------------------------------------------ the run
void runE02() {
  e02Header();
  int route = (int)P("e02_route");
  int mode = P("e02_mode") >= 0.5f ? 1 : 0;
  mStart = Mark(); mFar = Mark(); mEnd = Mark();
  if (route < 1 || route > 3) {
    gFail = "e02_route must be 1, 2 or 3";
    e02Record(route, mode, String("failed:") + gFail, 0);
    return;
  }
  if (edgeOff()[0]) {
    gFail = edgeOff();
    e02Record(route, mode, String("failed:") + gFail, 0);
    return;
  }
  if (!pose::known) {
    gFail = "position unknown: run find first (or send a camera fix)";
    e02Record(route, mode, String("failed:") + gFail, 0);
    return;
  }
  begin();
  mode1 = mode == 1;
  gFrontLimitCm = 0;
  int legs = 0;
  bool ok = false;
  if (!mode1) {
    do {
      float sx = P("e02_start_x"), sy = P("e02_start_y");
      travelCap = P("e02_transit_max_cm") + 10.0f;
      if (sx != UNSET && sy != UNSET) {
        float d = sqrtf((sx - pose::x) * (sx - pose::x) + (sy - pose::y) * (sy - pose::y));
        if (d > P("e02_transit_max_cm")) {
          char why[96];
          snprintf(why, sizeof(why), "start spot %.0f cm away (more than e02_transit_max_cm)", d);
          gFail = why;
          break;
        }
        rawlog::step = 0;
        if (!legTo(sx, sy)) break;
      }
      float hs = P("e02_route_deg");
      if (hs == UNSET) hs = wrap180(board(false).toward + 180.0f);   // away from the y board
      if (!turnTo(hs)) break;
      routeCorrections();                     // the condition's corrections from here (transit: the find's)
      travelCap = travelAbs + routeLen(route) + P("e02_margin_cm");
      mark(mStart);
      Pt w[4];
      int far = 0, nw = waypoints(route, w, far);
      float c = cosf(mStart.th * pose::DEG2RAD), s = sinf(mStart.th * pose::DEG2RAD);
      bool good = true;
      for (int i = 0; i < nw && good; i++) {
        rawlog::step = i + 1;
        float wx = mStart.x + c * w[i].x - s * w[i].y, wy = mStart.y + s * w[i].x + c * w[i].y;
        good = legTo(wx, wy);
        if (good && (i == far || i == nw - 1)) good = redo(wx, wy);
        legs++;
        if (good && i == far) { mark(mFar); good = waitMs((unsigned long)P("e02_pause_ms")); }
      }
      if (!good) break;
      rawlog::step = nw + 1;
      if (!turnTo(mStart.th)) break;
      mark(mEnd);
      ok = true;
    } while (false);
  } else {
    // camera-steered: only what the laptop sends
    legHeader();
    qHead = qLen = 0;
    lastId = 0;
    endMarked = false;
    gFrontSoft = true;                       // an obstacle or an open edge stops the leg, not the run
    gEdgeSoft = true;
    travelCap = routeLen(route) + P("e02_transit_max_cm") + P("e02_margin_cm");
    mark(mStart);                             // until the laptop sends mark start
    net::logf("M,%s e02: camera-steered, waiting for go / turn / mark", net::name);
    while (true) {
      if (shouldAbort()) {
        if (gAbortReason.length()) break;
        gStepStop = "";
      }
      if (millis() - net::lastRx > (unsigned long)P("e02_quiet_ms") && !qLen) {
        char why[64];
        snprintf(why, sizeof(why), "nothing from the laptop for %.0f s", P("e02_quiet_ms") / 1000.0f);
        gFail = why;
        break;
      }
      if (!qLen) { delay(5); continue; }
      Leg L = q[qHead];
      qHead = (qHead + 1) % NQ;
      qLen--;
      legs++;
      rawlog::step = legs;
      gStepStop = "";
      gFail = "";
      legRecord(L.id, L.kind, L.arg, "started", "");
      char info[48] = "";
      bool good = runLeg(L, info, sizeof(info));
      alvik.brake();
      if (gAbortReason.length()) { legRecord(L.id, L.kind, L.arg, "failed", gAbortReason.c_str()); break; }
      if (good) { addFront(info, sizeof(info)); legRecord(L.id, L.kind, L.arg, "done", info); }
      else {
        char why[96];
        if (gStepStop == "edge") snprintf(why, sizeof(why), "plywood edge (%s)", edge::lastCue);
        else snprintf(why, sizeof(why), "%s", gStepStop.length() ? gStepStop.c_str() : gFail.length() ? gFail.c_str() : "failed");
        legRecord(L.id, L.kind, L.arg, "failed", why);
        gStepStop = "";
        gFail = "";
      }
      if (endMarked) { ok = true; break; }
    }
    if (!mEnd.set) mark(mEnd);
  }
  if (!ok && !mEnd.set) mark(mEnd);
  String result = ok ? String("done")
                : gAbortReason == "plywood edge ahead" ? String("limited:plywood edge")
                : gAbortReason.length() ? String("stopped:") + gAbortReason
                : String("failed:") + (gFail.length() ? gFail : String("?"));
  e02Record(route, mode, result, legs);
  float ex = mEnd.x - mStart.x, ey = mEnd.y - mStart.y;
  net::logf("M,%s e02 route %d %s: by its own reckoning it ended %.1f cm from its start, heading %.1f deg off",
            net::name, route, ok ? "done" : result.c_str(), sqrtf(ex * ex + ey * ey), wrap180(mEnd.th - mStart.th));
  end();
}

}  // namespace e02

void runFind() { e02::runFind(); }
void runE02() { e02::runE02(); }
