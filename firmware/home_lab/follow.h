// Step follower ("run follow"): the laptop (tools/fleet/run_plan.py) sends
// the steps of plan.json as they are released; the robot buffers up to 8 and
// runs them in order on its own pose (pose.h), corrected by camera fixes.
// Protocol: docs/data-formats.md, "Step follower". In short (laptop -> robot,
// one command per UDP packet, numbers in cm / degrees, floor frame):
//
//   fs <id> T <h> <fmin>                                     turn in place to heading h
//   fs <id> D <x0> <y0> <x1> <y1> <h> <cms> <rev> <fmin>     straight drive (rev 1 = backwards)
//   fs <id> A <x0> <y0> <x1> <y1> <h> <cms> <handover> <gap> <max_travel> <fmin>   approach a board
//   fs <id> W <ms>                                           wait (a dwell; 0 = just keep the order)
//   fs <id> C <cx> <cy> <r> <a0> <sweep> <cms> <fmin>        round the circle centre (cx, cy), radius r, from angle a0
//                                                            through sweep deg (+ anticlockwise) (2026-10-28a)
//   fs <id> Z <ms>                                           dance where it stands for ms (0 = until stopped;
//                                                            dance.h, 2026-10-28a)
//   fs <id> K <wall_x> <wall_y> <h> <max_travel> <back>      home dock (dock.h): creep in facing h until both
//                                                            nose switches click; own pose reset from the dock
//                                                            (wall_x, wall_y: middle of the back wall's inner face);
//                                                            stand still dock_still_ms; back out <back> cm (0 = stay)
//   fs <id> P <h> <max_travel> <back>                        touch probe (dock.h): creep facing h until a nose
//                                                            switch clicks (TOUCH record), back off <back> cm
//   fspd <scale>                                             (2026-11-05b) the running drive's speed x scale (0-1.5;
//                                                            0 = stand on the line, the step kept); answered FSPD
//   fsteer <x1> <y1>                                         (2026-11-05b) the running forward drive goes on to a new
//                                                            end from where it is (turns while moving); answered FSPD
//   fs <id> R <x0> <y0> <h0> <k> <len> <cms> <fmin>         (2026-11-05c) a piece of constant curvature k (1/cm, + = left,
//                                                            0 = straight) from (x0, y0) facing h0, len cm at cms; R
//                                                            pieces in a row run on without slowing; nothing queued at
//                                                            the end: it coasts round the same curve arc_coast_ms, then
//                                                            brakes. Front check: a dodge (tight curve away), not a stop
//   farc <k> [<len>]                                         (2026-11-05c) the running piece (or coast) bends to curvature
//                                                            k from where it is (len cm; default what was left); queued
//                                                            pieces dropped; answered FSPD
//   fix <t_ms> <x> <y> <th|-> <sigma_cm> [<sigma_deg>]       camera pose at robot time t_ms
//   fpose <x> <y> <th> [<sigma_cm> <sigma_deg>]              set the pose outright
//   hold [<id>]   finish the current step (or up to step id), drop the rest
//   halt          stop at once, drop every step, stay in follow mode
//   stop          stop at once and end the run (as for every experiment)
//
// fmin: smallest front-face distance the map expects ahead during the step
// ("-" = nothing within 100 cm). The plywood edge guard (safe_edge) stops only
// the step too (failed "plywood edge at <x> <y> <th> <line|bottom>": the robot holds, the
// laptop maps that edge and replans), not the run.
// The front check stops the step (failed:obstacle)
// below min(safe_front_cm, fmin + tof_recess_cm - front_margin_cm), so planned
// passes near boards don't trip the 7 cm stop. Approaches do their own check.
//
// Robot -> laptop: STEP records (queued, dup, full, bad, started, done, failed,
// dropped; held / halted acknowledge hold / halt) and a FOLLOW record
// follow_log_hz times a second. Step ids must go up: a step id at or below the
// highest seen is answered "dup" (so a resent step is acked again but never
// run twice); so the laptop sends a step only after the one before it was
// acked. If the laptop goes quiet the robot finishes what it has buffered,
// then stands still.
#pragma once

#include "pose.h"
#include "dock.h"
#include "border.h"
#include "dance.h"

extern String gRunId;

namespace follow {

struct Step {
  uint16_t id;
  char type;                 // T, D, A, W, K, P, C (arc: x0 y0 = centre, x1 = radius, y1 = a0, h = sweep), Z (dance),
                             // R (2026-11-05c curve piece: x0 y0 = start, h = heading there, x1 = curvature, y1 = length)
  uint8_t rev;
  bool cont;                 // C: the next queued step goes on round the same circle (no slowing, no brake)
  float x0, y0, x1, y1, h, speed;
  float fmin;                // < 0: none
  float handover, gap, maxTravel;
  uint32_t waitMs;
};

const int NQ = 8;
Step q[NQ];
int qHead = 0, qLen = 0;
bool active = false;
uint16_t maxId = 0, curId = 0, lastDone = 0, lastFailed = 0;
const char* state = "idle";
float travel = 0;
unsigned long nextLog = 0, period = 100;
bool quietSaid = false;
char info[64] = "";
bool dockHeaderSent = false, touchHeaderSent = false;
// (2026-11-05c) continuous curves: after an R piece the wheels keep turning (curving) until the next R piece starts,
// the coast runs out (arc_coast_ms), something is too close ahead (arc_dodge_stop_cm), a halt, or the watchdog
bool curving = false, dodging = false, dodged = false;
unsigned long coastT0 = 0;

float readFront() {
  float dL, dCL, dC, dCR, dR;
  alvik.get_distance(dL, dCL, dC, dCR, dR);
  return dC;
}

void report(uint16_t id, const char* st, const char* why) {
  net::logf("D,STEP,%s,%u,%s,%lu,%.2f,%.2f,%.2f,%.1f,%s", gRunId.c_str(), (unsigned)id, st, millis(),
            pose::x, pose::y, pose::th(), readFront(), why);
}

void log() {
  unsigned long now = millis();
  if ((long)(now - nextLog) < 0) return;
  nextLog = now + period;
  // (2026-11-12e21) five columns appended: refused fixes by reason (old, far, history, future) and escapes
  net::logf("D,FOLLOW,%s,%lu,%u,%s,%d,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.1f,%.2f,%lu,%d,%d,%d,%d,%d", gRunId.c_str(),
            now, (unsigned)curId, pose::known ? state : "nopose", qLen, (unsigned)lastDone, (unsigned)lastFailed,
            pose::x, pose::y, pose::th(), pose::sd(), pose::sdTh(), pose::fixesUsed, pose::fixesRejected,
            readFront(), travel, now - net::lastRx, pose::rejOld, pose::rejFar, pose::rejHist, pose::rejFuture,
            pose::escapes);
}

void tickHook() {
  pose::tick();
  // (2026-11-05b) heartbeat watchdog (follow_wd_ms, 0 = off): the laptop / Pi or the WiFi gone while a step runs:
  // the step brakes and fails ("watchdog"), the queue is dropped (the run loop's failure path), the robot stands
  if (curId && P("follow_wd_ms") > 0 && millis() - net::lastRx > (unsigned long)P("follow_wd_ms") &&
      !gStepStop.length() && !gAbortReason.length()) {
    gStepStop = "watchdog";
    alvik.brake();
  }
  // the four-sided virtual border (border.h, 2026-10-27a): only the step stops; the laptop re-plans
  if (border::followTick(pose::dist, pose::known) && curId && !gStepStop.length() && !gAbortReason.length()) {
    gStepStop = "border";
    alvik.brake();
  }
  log();
}

void dropQueued(uint16_t keepUpTo, const char* why) {
  // keep queued steps with id <= keepUpTo (0 = drop all)
  int kept = 0;
  Step tmp[NQ];
  for (int k = 0; k < qLen; k++) {
    Step& s = q[(qHead + k) % NQ];
    if (keepUpTo && s.id <= keepUpTo) tmp[kept++] = s;
    else report(s.id, "dropped", why);
  }
  for (int k = 0; k < kept; k++) q[k] = tmp[k];
  qHead = 0;
  qLen = kept;
}

float num(const char* t) {
  if (!t || !strcmp(t, "-")) return NAN;
  char* end = nullptr;
  float v = strtof(t, &end);
  return end == t ? NAN : v;
}

// A follow command (from handleCommand while the follow run is going).
void command(const String& verb, const String& rest) {
  char buf[160];
  snprintf(buf, sizeof(buf), "%s", rest.c_str());
  char* tok[16];
  int n = 0;
  for (char* t = strtok(buf, " "); t && n < 16; t = strtok(nullptr, " ")) tok[n++] = t;

  if (verb == "fix") {
    if (n < 5) return;
    float sigTh = n > 5 ? num(tok[5]) : 1.0f;
    pose::fix(strtoul(tok[0], nullptr, 10), num(tok[1]), num(tok[2]), num(tok[3]), num(tok[4]),
              isNaN(sigTh) ? 1.0f : sigTh);
    return;
  }
  if (verb == "fpose") {
    if (n < 3 || isNaN(num(tok[0])) || isNaN(num(tok[1])) || isNaN(num(tok[2]))) {
      net::sendLine("M,usage: fpose x y heading [sigma_cm sigma_deg]");
      return;
    }
    pose::set(num(tok[0]), num(tok[1]), num(tok[2]), n > 3 ? num(tok[3]) : 0.5f, n > 4 ? num(tok[4]) : 2.0f);
    net::logf("M,%s: pose set to %.1f %.1f %.1f", net::name, pose::x, pose::y, pose::th());
    return;
  }
  if (verb == "hold") {
    uint16_t keep = n > 0 ? (uint16_t)atoi(tok[0]) : 0;
    dropQueued(keep, "hold");
    report(keep, "held", "");        // the ack: the laptop resends hold until it sees this
    return;
  }
  if (verb == "fspd" || verb == "fsteer") {       // (2026-11-05b) change the running drive
    if (verb == "fspd") {
      float sc = n > 0 ? num(tok[0]) : NAN;
      if (isNaN(sc) || sc < 0 || sc > 1.5f) { net::sendLine("M,usage: fspd <scale 0-1.5>"); return; }
      gSpeedScale = sc;
      if (sc <= 0 && curving && !curId) { curving = false; alvik.brake(); gCurveV = 0; }   // (2026-11-05c) coast
    } else {
      if (n < 2 || isNaN(num(tok[0])) || isNaN(num(tok[1]))) { net::sendLine("M,usage: fsteer <x> <y>"); return; }
      gSteerX = num(tok[0]); gSteerY = num(tok[1]); gSteer = true;
    }
    net::logf("D,FSPD,%s,%lu,%u,%s,%.2f,%.2f,%.2f", gRunId.c_str(), millis(), (unsigned)curId, verb.c_str(),
              gSpeedScale, gSteer ? gSteerX : 0.0f, gSteer ? gSteerY : 0.0f);
    return;
  }
  if (verb == "farc") {                           // (2026-11-05c) bend the running curve piece or the coast
    float k = n > 0 ? num(tok[0]) : NAN, len = n > 1 ? num(tok[1]) : NAN;
    if (isNaN(k) || fabsf(k) > 0.5f || (!isNaN(len) && len <= 0)) { net::sendLine("M,usage: farc <k 1/cm> [<len cm>]"); return; }
    if (curId && state[0] == 'c') {               // an R piece runs ("curve")
      gArcSetK = k; gArcSetLen = len; gArcSet = true;
      dropQueued(0, "farc");                      // the queued pieces started at the old end
    } else if (curving) {
      gCurveK = k;
      coastT0 = millis();
    } else {
      net::sendLine("M,farc: no curve running");
      return;
    }
    net::logf("D,FSPD,%s,%lu,%u,farc,%.2f,%.4f,%.2f", gRunId.c_str(), millis(), (unsigned)curId, gSpeedScale, k,
              isNaN(len) ? -1.0f : len);
    return;
  }
  if (verb == "halt") {
    dropQueued(0, "halt");
    if (curving && !curId) { curving = false; alvik.brake(); gCurveV = 0; }   // (2026-11-05c) a coast stops too
    if (curId) gStepStop = "halted";
    report(0, "halted", "");         // the ack
    return;
  }
  // fs <id> <type> ...
  if (n < 2) { net::sendLine("M,usage: fs <id> <T|D|A|W> ..."); return; }
  uint16_t id = (uint16_t)atoi(tok[0]);
  if (id == 0) { report(0, "bad", "id must be 1 or more"); return; }
  if (id <= maxId) { report(id, "dup", ""); return; }
  if (qLen >= NQ) { report(id, "full", ""); return; }
  Step s;
  memset(&s, 0, sizeof(s));
  s.id = id;
  s.type = tok[1][0];
  s.fmin = -1;
  bool ok = true;
  if (s.type == 'T' && n >= 3) {
    s.h = num(tok[2]);
    ok = !isNaN(s.h);
    if (n >= 4 && !isNaN(num(tok[3]))) s.fmin = num(tok[3]);
  } else if ((s.type == 'D' && n >= 9) || (s.type == 'A' && n >= 11)) {
    s.x0 = num(tok[2]); s.y0 = num(tok[3]); s.x1 = num(tok[4]); s.y1 = num(tok[5]);
    s.h = num(tok[6]); s.speed = num(tok[7]);
    ok = !(isNaN(s.x0) || isNaN(s.y0) || isNaN(s.x1) || isNaN(s.y1) || isNaN(s.h) || isNaN(s.speed) || s.speed <= 0);
    if (s.type == 'D') {
      s.rev = atoi(tok[8]) ? 1 : 0;
      if (n >= 10 && !isNaN(num(tok[9]))) s.fmin = num(tok[9]);
    } else {
      s.handover = num(tok[8]); s.gap = num(tok[9]); s.maxTravel = num(tok[10]);
      ok = ok && !(isNaN(s.handover) || isNaN(s.gap) || isNaN(s.maxTravel));
      if (n >= 12 && !isNaN(num(tok[11]))) s.fmin = num(tok[11]);
    }
  } else if (s.type == 'C' && n >= 8) {
    s.x0 = num(tok[2]); s.y0 = num(tok[3]); s.x1 = num(tok[4]); s.y1 = num(tok[5]); s.h = num(tok[6]);
    s.speed = num(tok[7]);
    ok = !(isNaN(s.x0) || isNaN(s.y0) || isNaN(s.x1) || isNaN(s.y1) || isNaN(s.h) || isNaN(s.speed) || s.speed <= 0 ||
           s.x1 <= 0);
    if (n >= 9 && !isNaN(num(tok[8]))) s.fmin = num(tok[8]);
  } else if (s.type == 'R' && n >= 8) {          // (2026-11-05c) R x0 y0 h0 k len cms [fmin]
    s.x0 = num(tok[2]); s.y0 = num(tok[3]); s.h = num(tok[4]); s.x1 = num(tok[5]); s.y1 = num(tok[6]);
    s.speed = num(tok[7]);
    ok = !(isNaN(s.x0) || isNaN(s.y0) || isNaN(s.h) || isNaN(s.x1) || isNaN(s.y1) || isNaN(s.speed) ||
           s.speed <= 0 || s.y1 <= 0 || fabsf(s.x1) > 0.5f);
    if (n >= 9 && !isNaN(num(tok[8]))) s.fmin = num(tok[8]);
  } else if (s.type == 'Z' && n >= 3) {
    s.waitMs = strtoul(tok[2], nullptr, 10);
  } else if (s.type == 'W' && n >= 3) {
    s.waitMs = strtoul(tok[2], nullptr, 10);
  } else if (s.type == 'K' && n >= 7) {
    s.x1 = num(tok[2]); s.y1 = num(tok[3]); s.h = num(tok[4]); s.maxTravel = num(tok[5]); s.handover = num(tok[6]);
    ok = !(isNaN(s.x1) || isNaN(s.y1) || isNaN(s.h) || isNaN(s.maxTravel) || isNaN(s.handover));
  } else if (s.type == 'P' && n >= 5) {
    s.h = num(tok[2]); s.maxTravel = num(tok[3]); s.handover = num(tok[4]);
    ok = !(isNaN(s.h) || isNaN(s.maxTravel) || isNaN(s.handover));
  } else {
    ok = false;
  }
  if (!ok) { report(id, "bad", "could not read the step"); return; }
  maxId = id;
  q[(qHead + qLen) % NQ] = s;
  qLen++;
  report(id, "queued", "");
}

// Front check limit for a turn (centre zone, safe_front_cm) or a forward drive
// (anything in the robot's path, follow_look_cm), never above what the map
// expects ahead (fmin, front face) minus front_margin_cm.
float frontLimit(float fmin, bool drive) {
  float lim = drive ? fmaxf(P("follow_look_cm"), P("safe_front_cm")) : P("safe_front_cm");
  if (fmin >= 0) lim = fminf(lim, fmin + P("tof_recess_cm") - P("front_margin_cm"));
  return lim > 0 ? lim : 0;
}

// Slow final approach to a board: average (reading + wheel distance) while the
// sensor is in range, hand over to the wheels at the handover reading, stop with
// the front face `gap` from the board. Never travels more than max_travel.
bool approach(const Step& s) {
  float ux = cosf(s.h * pose::DEG2RAD), uy = sinf(s.h * pose::DEG2RAD);
  float L = (s.x1 - s.x0) * ux + (s.y1 - s.y0) * uy;
  float remA = (s.x1 - pose::x) * ux + (s.y1 - pose::y) * uy;    // map: travel left to the dock
  // the travel cap, counted from the step's start point (so "here" comes from the own pose: the
  // cap is only as good as the pose, about +/-0.3 cm with camera fixes; max_travel leaves 0.5 cm)
  float cap = s.maxTravel - (L - remA);
  float recess = P("tof_recess_cm"), tmin = P("tof_min_cm"), margin = P("front_margin_cm");
  float fuse = P("fuse_from_cm"), acc = P("drive_accel"), kh = P("drive_kh"), kct = P("drive_kct");
  float target = s.gap + recess;                                  // reading when docked
  float expect0 = remA + s.gap + recess;                          // map: reading now
  unsigned long tofMs = (unsigned long)P("tof_ms");
  float lag = P("tof_lag_ms") / 1000.0f;                          // a reading belongs to slightly earlier travel
  float d0 = pose::dist, sum = 0, wheelTo = 0, v = 0;
  int n = 0, low = 0;
  bool wheels = false;
  unsigned long t0 = millis(), last = t0, nextRead = t0;
  unsigned long limit = 5000 + (unsigned long)(fmaxf(remA, 0.0f) / fmaxf(s.speed, 0.5f) * 3000.0f);
  snprintf(info, sizeof(info), "no handover");
  while (true) {
    if (shouldAbort()) return false;
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;
    travel = pose::dist - d0;
    if (travel >= cap - 0.02f) { snprintf(info, sizeof(info), "travel cap %.1f cm reached; %d readings", cap, n); break; }
    if (now - t0 > limit) { alvik.brake(); gFail = "approach timed out"; return false; }
    if (!wheels && (long)(now - nextRead) >= 0) {
      nextRead = now + tofMs;
      float d = readFront();
      if (d > tmin) {
        float at = travel - v * lag;                  // travel when the reading was taken
        if (d < expect0 - at - margin) {              // something in front of the board (two readings in a row)
          if (++low >= 2) {
            gStepStop = "obstacle";
            gFrontTripCm = d;
            alvik.brake();
            return false;
          }
        } else {
          low = 0;
          if (d <= fuse) { sum += d + at; n++; }
          if (d <= s.handover && n >= 3) {
            float est = sum / n;                      // where the board is, in travel cm (sensor reading at travel 0)
            wheelTo = est - target;
            wheels = true;
            snprintf(info, sizeof(info), "%d readings; wheels from %.2f cm", n, d);
          }
        }
      }
      if (!wheels && n < 3 && travel >= remA - 0.5f) { // no readings near the board: finish on the map
        wheelTo = remA;
        wheels = true;
        snprintf(info, sizeof(info), "wheels only: no distance readings");
      }
    }
    float vt = s.speed;
    if (wheels) {
      wheelTo = fminf(wheelTo, cap);
      float rem = wheelTo - travel;
      if (rem <= v * 0.03f + 0.02f) break;
      vt = fminf(vt, sqrtf(2.0f * acc * rem) + 0.5f);
    }
    v = fminf(vt, v + acc * dt + 0.3f);
    // hold the heading; small sideways corrections only while more than 3 cm from the dock
    float along = (s.x1 - pose::x) * ux + (s.y1 - pose::y) * uy;
    float cross = ux * (pose::y - s.y1) - uy * (pose::x - s.x1);
    float corr = along > 3.0f ? constrain(-kct * cross, -2.0f, 2.0f) : 0.0f;
    float herr = wrap180(s.h + corr - pose::th());
    setDrive(v, constrain(kh * herr, -20.0f, 20.0f));
    delay(5);
  }
  alvik.brake();
  travel = pose::dist - d0;
  return true;
}

bool runStep(const Step& s) {
  info[0] = 0;
  switch (s.type) {
    case 'T':
      gFrontLimitCm = frontLimit(s.fmin, false);
      return turnToFloor(s.h);
    case 'D': {
      // well off the step's heading (e.g. after a halt): square up first
      gFrontLimitCm = s.rev ? 0 : frontLimit(s.fmin, false);
      if (fabsf(wrap180(s.h - pose::th())) > 10.0f && !turnToFloor(s.h)) return false;
      gFrontLimitCm = s.rev ? 0 : frontLimit(s.fmin, true);
      gFrontEndBase = s.rev ? -1 : gFrontLimitCm;
      gFrontPath = !s.rev;
      float d0 = pose::dist;
      int r = driveLine(s.x0, s.y0, s.x1, s.y1, s.h, s.speed, s.rev);
      gFrontPath = false;
      gFrontEndBase = -1;
      travel = pose::dist - d0;
      return r == DRIVE_OK;
    }
    case 'A':
      gFrontLimitCm = 0;
      return approach(s);
    case 'W': {
      gFrontLimitCm = 0;
      return waitMs(s.waitMs);
    }
    case 'C': {
      // round a circle (pose.h driveArc); well off the circle's heading at the start: square up first
      gFrontLimitCm = frontLimit(s.fmin, false);
      float tang = s.y1 + (s.h >= 0 ? 90.0f : -90.0f);
      if (fabsf(wrap180(tang - pose::th())) > 20.0f && !turnToFloor(tang)) return false;
      float d0 = pose::dist;
      int r = driveArc(s.x0, s.y0, s.x1, s.y1, s.h, s.speed, s.cont);
      travel = pose::dist - d0;
      return r == DRIVE_OK;
    }
    case 'R': {
      // (2026-11-05c) a curve piece; from a stand far off its heading: square up first (it stood anyway)
      if (!curving && fabsf(wrap180(s.h - pose::th())) > 30.0f) {
        gFrontLimitCm = frontLimit(s.fmin, false);
        if (!turnToFloor(s.h)) return false;
      }
      if (!curving) gCurveV = 0;
      gFrontLimitCm = 0;                       // the piece does its own front check: a dodge, not a stop
      gCurveLook = frontLimit(s.fmin, true);
      float d0 = pose::dist;
      int r = driveCurve(s.x0, s.y0, s.h, s.x1, s.y1, s.speed);
      gCurveLook = -1;
      travel = pose::dist - d0;
      if (r == DRIVE_DODGE) { dodged = true; return false; }
      return r == DRIVE_OK;
    }
    case 'Z':
      gFrontLimitCm = 0;
      if (!dance::run(s.waitMs)) return false;
      snprintf(info, sizeof(info), "danced %lu ms", (unsigned long)s.waitMs);
      return true;
    case 'K':
    case 'P': {
      // dock / touch: it is meant to touch what is ahead; square up first if it is off the heading
      gFrontLimitCm = 0;
      if (fabsf(wrap180(s.h - pose::th())) > 3.0f && !turnToFloor(s.h)) return false;
      dock::Result r;
      bool ok;
      if (s.type == 'K') {
        if (!dockHeaderSent) { dock::dockHeader(); dockHeaderSent = true; }
        ok = dock::dockOnce(s.id, s.x1, s.y1, s.h, r, s.maxTravel);
      } else {
        if (!touchHeaderSent) { dock::touchHeader(); touchHeaderSent = true; }
        ok = dock::creep(false, r, s.maxTravel);
        dock::touchRecord(r, s.id);
      }
      travel = r.travel;
      if (!ok) return false;
      if (s.type == 'K') snprintf(info, sizeof(info), "docked; pivot %.1f deg; stripe %.2f cm", r.pivotDeg, r.stripe[1]);
      else snprintf(info, sizeof(info), "touch %s %.3f cm", r.first == 0 ? "L" : "R", r.travelAt[r.first]);
      if (s.handover > 0) {
        if (!waitMs(300)) return false;
        // away from what it touched: backwards with the nose switches, forwards with the back ones (sw_back 1)
        float ux = cosf(s.h * pose::DEG2RAD), uy = sinf(s.h * pose::DEG2RAD), sg = dock::dir();
        int d = driveLine(pose::x, pose::y, pose::x - sg * s.handover * ux, pose::y - sg * s.handover * uy, s.h, 3.0f,
                          sg > 0);
        if (d != DRIVE_OK) return false;
      }
      return true;
    }
  }
  gFail = "unknown step type";
  return false;
}

const char* stateName(char t) {
  return t == 'T' ? "turn" : t == 'D' ? "drive" : t == 'A' ? "approach" : t == 'W' ? "wait" : t == 'K' ? "dock"
       : t == 'P' ? "touch" : t == 'C' ? "arc" : t == 'Z' ? "dance" : t == 'R' ? "curve" : "?";
}

void run() {
  if (!calibrateYawSign()) return;
  net::sendLine("H,STEP,run,id,state,ms,x_cm,y_cm,th_deg,front_cm,info");
  net::sendLine("H,FOLLOW,run,ms,step_id,state,queued,last_done,last_failed,x_cm,y_cm,th_deg,pos_sd_cm,th_sd_deg,"
                "fixes_used,fixes_rejected,front_cm,travel_cm,quiet_ms,rej_old,rej_far,rej_hist,rej_future,escapes");
  net::sendLine("H,FSPD,run,ms,step_id,verb,scale,x,y");      // (2026-11-05b) fspd / fsteer answers
  qHead = qLen = 0;
  maxId = curId = lastDone = lastFailed = 0;
  dockHeaderSent = touchHeaderSent = false;
  state = "idle";
  travel = 0;
  quietSaid = false;
  period = (unsigned long)(1000.0f / fmaxf(1.0f, P("follow_log_hz")));
  nextLog = millis();
  pose::start(false);
  active = true;
  gTickHook = tickHook;
  gFrontSoft = true;
  gEdgeSoft = true;
  gFrontLimitCm = 0;
  gStepStop = "";
  net::logf("M,%s follow: ready for steps (needs fpose or a camera fix first)", net::name);
  while (true) {
    if (shouldAbort()) {
      if (gAbortReason.length()) break;
      gStepStop = "";          // a halt while nothing was running
      if (curving) { curving = dodging = false; gCurveV = 0; }   // (2026-11-05c) the edge guard (braked) ends a coast
    }
    unsigned long quiet = millis() - net::lastRx;
    if (quiet > (unsigned long)P("follow_quiet_ms")) {
      if (!quietSaid && qLen == 0 && !curId) {
        net::logf("M,%s follow: nothing from the laptop for %lu ms; standing still", net::name, quiet);
        quietSaid = true;
      }
    } else {
      quietSaid = false;
    }
    // (2026-11-05c) coasting after an R piece: on round the same curve until the next R piece (no stop between)
    if (curving && !(qLen && q[qHead].type == 'R' && pose::known)) {
      bool wd = P("follow_wd_ms") > 0 && quiet > (unsigned long)P("follow_wd_ms");
      float d = frontInPath();
      const char* why = qLen ? nullptr : wd ? "watchdog" : !pose::known ? "no pose" :
                        millis() - coastT0 > (unsigned long)P("arc_coast_ms") ? "nothing queued" :
                        (d > 0 && d < P("arc_dodge_stop_cm")) ? "something close ahead" : nullptr;
      if (qLen || why) {
        alvik.brake();
        curving = dodging = false;
        gCurveV = 0;
        if (why) net::logf("M,%s follow: coast ended (%s): stopped", net::name, why);
      } else {
        state = dodging ? "dodge" : "coast";
        float v = fmaxf(P("arc_vmin"), gCurveV);      // fspd 0 brakes a coast (the fspd command)
        setDrive(v, gCurveK * v / pose::DEG2RAD);
        delay(5);
        continue;
      }
    }
    if (!qLen || !pose::known) { state = "idle"; delay(5); continue; }
    // (2026-11-05b) watchdog: no new step starts while the laptop has been silent past follow_wd_ms
    if (P("follow_wd_ms") > 0 && quiet > (unsigned long)P("follow_wd_ms")) {
      dropQueued(0, "watchdog");
      state = "idle"; delay(5); continue;
    }
    Step s = q[qHead];
    qHead = (qHead + 1) % NQ;
    qLen--;
    s.cont = false;
    if (s.type == 'C' && qLen > 0) {     // the next step goes on round the same circle, the same way: no stop between
      const Step& nx = q[qHead];
      s.cont = nx.type == 'C' && fabsf(nx.x0 - s.x0) < 0.05f && fabsf(nx.y0 - s.y0) < 0.05f &&
               fabsf(nx.x1 - s.x1) < 0.05f && (nx.h >= 0) == (s.h >= 0);
    }
    curId = s.id;
    rawlog::step = s.id;       // RAW step = the step id while it runs (0 between steps)
    state = stateName(s.type);
    travel = 0;
    gStepStop = "";
    gFail = "";
    if (!(s.type == 'R' && curving)) gSpeedScale = 1.0f;   // (2026-11-05b) every step starts at its own speed, on its
                               // own line; (2026-11-05c) except an R piece run on from the last: the scale stays
    dodged = false;
    gSteer = false;
    gFrontLowN = 0;            // obs_n counts readings within this step
    border::reset();           // the border's reference starts again with every step (a re-plan may drive it back in)
    report(s.id, "started", "");
    bool ok;
    if (P("tilt_stop_deg") > 0 && tilt::now() > P("tilt_stop_deg") && !(s.type == 'D' && s.rev)) {
      // (2026-11-05d) tilted (hung at an edge): no forward drive and no turn starts; only a straight backward drive
      gStepStop = String("tilted ") + String(tilt::last, 1) + " deg: only a backward drive is allowed";
      ok = false;
    } else {
      ok = runStep(s);
    }
    // (2026-11-05c) an R piece that ended (or dodged) leaves the wheels turning: the next R piece or the coast
    bool rolling = s.type == 'R' && (ok || dodged) && !gAbortReason.length();
    if (!(ok && s.cont) && !rolling) alvik.brake();
    if (!rolling) { curving = dodging = false; gCurveV = 0; }
    else { curving = true; dodging = dodged; coastT0 = millis(); }
    gFrontLimitCm = 0;
    gFrontPath = false;
    gFrontEndBase = -1;
    if (gAbortReason.length()) {
      lastFailed = s.id;
      report(s.id, "failed", gAbortReason.c_str());
      curId = 0;
      break;
    }
    if (ok) {
      lastDone = s.id;
      report(s.id, "done", info);
    } else {
      char why[96];
      if (gStepStop == "obstacle") {
        // all five zones, so the laptop can draw the obstacle's outline (zone directions: motion.h)
        float d[5];
        alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
        snprintf(why, sizeof(why), "obstacle %.1f cm ahead; zones %.1f %.1f %.1f %.1f %.1f", gFrontTripCm,
                 d[0], d[1], d[2], d[3], d[4]);
      }
      else if (dodged)
        // (2026-11-05c) no stop: it curves away (the coast) until the laptop sends the next piece
        snprintf(why, sizeof(why), "dodged obstacle %.1f cm ahead; curving %s at %.1f cm/s", gDodgeCm,
                 gDodgeK > 0 ? "left" : "right", gCurveV);
      else if (gStepStop == "edge")
        // where it stopped (own pose, floor cm / deg): the laptop maps the open edge just ahead of the line
        // sensors as a no-go strip and replans (run_plan.py); the robot holds
        snprintf(why, sizeof(why), "plywood edge at %.1f %.1f %.1f %s", pose::x, pose::y, pose::th(), edge::lastCue);
      else if (gStepStop == "border")
        // where it stopped and how far inside the open side it is (border.h): the laptop re-plans from there
        snprintf(why, sizeof(why), "virtual border at %.1f %.1f %.1f %.1f", pose::x, pose::y, pose::th(), border::lastD);
      else snprintf(why, sizeof(why), "%s", gStepStop.length() ? gStepStop.c_str() : gFail.length() ? gFail.c_str() : "failed");
      lastFailed = s.id;
      report(s.id, "failed", why);
      dropQueued(0, "an earlier step failed");   // the next steps assumed this one finished
      gStepStop = "";
      gFail = "";
    }
    curId = 0;
    rawlog::step = 0;
    state = "idle";
  }
  dropQueued(0, "run ended");
  curving = dodging = false;
  gCurveV = 0;
  gSpeedScale = 1.0f;
  gSteer = false;
  active = false;
  gTickHook = nullptr;
  gFrontSoft = false;
  gEdgeSoft = false;
  gFrontLimitCm = -1;
  gStepStop = "";
  pose::stop();
}

}  // namespace follow
