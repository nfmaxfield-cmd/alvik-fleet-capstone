// Plywood edge guard (firmware 2026-10-18b+): stop before the robot drives off
// an open edge of the board (one with no border board). OFF by default
// (safe_edge 0) until the E0c edge rows give real numbers: run
// tools/edge_check.py on them and it prints the settings to use.
//
// Two cues, each on its own or both (safe_edge 1 = line sensors, 2 = distance
// sensor's bottom zone, 3 = both):
//
//  1. Line sensors (lineL/C/R, about 3.5 cm ahead of the wheel axle). The
//     plywood level of each sensor is LEARNED at the start of every run, while
//     the robot stands still on its start spot (median over edge_learn_ms), so
//     differences between robots and in room light cancel. While driving
//     forward, a sensor "sees the edge" when its reading has moved from that
//     level by at least edge_thr in direction edge_dir (+1 up, -1 down, 0 either
//     way; the sign past an edge is not known yet) and by no more than edge_max
//     (0 = no upper limit; for when tape moves the reading even further than the
//     edge does). edge_n checks in a row (shouldAbort() calls, 5-10 ms apart)
//     with any one sensor past the edge trip the guard. A sensor already reading
//     tape (above lf_line_thr) while learning takes the other sensors' level; if
//     all three do, the line cue is off for that run (EDGE record "nobase").
//  2. Bottom zone of the distance sensor (dbot_cm), which looks forward and down
//     at the floor several cm ahead of the nose. Past an open edge that floor
//     patch is the cement, one plywood thickness lower, so the reading gets
//     longer. An ABSOLUTE limit (edge_bot_cm) is used, not a learned level: the
//     reading depends only on how the sensor is mounted, and a start spot facing
//     a nearby board would teach a wrong (short) level. A board ahead only makes
//     the reading shorter, so it can't trip this cue. The reading must stay
//     above edge_bot_cm for edge_bot_ms (the sensor gives ~15 frames a second).
//
// Only while driving FORWARD (mean measured wheel speed above edge_min_rpm).
// Backing up is blind: both cues look ahead of the axle. Turning in place is
// not guarded either: the line sensors sit closer to the turning centre
// (~3.7 cm) than the wheels (~4.5 cm), so in a spin a wheel always reaches the
// edge first. Driving at a slant is only partly covered: with the line cue
// alone, the outer wheel reaches an edge before the outer line sensor whenever
// the robot meets the edge at less than about 45 degrees.
//
// After a trip the guard stays quiet until the wheels are still (the stop is
// under way), so a caller that carries on after it (the calibration run rolls
// back the way it came: home_lab.ino) is not stopped twice.
//
// A trip is not a failure: the run ends RUNEND "limited:plywood edge" (the tools
// leave it out of accuracy statistics and count it apart; docs/data-formats.md
// "Run outcomes"); the calibration's forward roll rolls back and ends done.
//
// EDGE records (docs/data-formats.md): "base" when the plywood level is learned,
// "nobase" when it could not be, "trip" when the guard stops the robot.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

extern Arduino_Alvik alvik;
extern String gRunId;

namespace edge {

const int NL = 40;                 // learning samples kept per sensor
int buf[3][NL];
int nLearn = 0;
bool learning = false;             // collecting the plywood level
bool ready = false;                // line cue armed for this run
float base[3] = {0, 0, 0};
unsigned long t0 = 0;
int lineCount = 0;                 // consecutive checks with a line sensor past the edge
unsigned long botSince = 0;        // when dbot first went above edge_bot_cm (0 = it isn't)
const char* lastCue = "";          // the cue of the last trip: "line" or "bottom" (follow.h reports it)
bool braking = false;              // tripped: quiet until the wheels are still (the stop is under way;
                                   // a caller that carries on, the calibration's roll, isn't stopped twice)

void record(const char* ev, int l, int c, int r, float dbot, const char* info) {
  net::sendLine("H,EDGE,run,ms,event,lineL,lineC,lineR,baseL,baseC,baseR,dbot_cm,info");
  net::logf("D,EDGE,%s,%lu,%s,%d,%d,%d,%.0f,%.0f,%.0f,%.1f,%s", gRunId.c_str(), millis(), ev, l, c, r,
            base[0], base[1], base[2], dbot, info);
}

// At the start of every run (runExperiment), before the settle pause.
void begin() {
  learning = P("safe_edge") > 0;
  ready = false;
  nLearn = 0;
  lineCount = 0;
  botSince = 0;
  braking = false;
  base[0] = base[1] = base[2] = 0;
  t0 = millis();
}

void stop() { learning = ready = false; }

float median(int* a, int n) {
  int t[NL];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) {             // insertion sort: n <= 40
    int v = t[i], j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return n % 2 ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

void finishLearning(int l, int c, int r) {
  learning = false;
  if (nLearn < 5) { record("nobase", l, c, r, alvik.get_distance_bottom(), "too few still readings"); return; }
  float thr = P("lf_line_thr");
  bool tape[3];
  float sum = 0;
  int ok = 0;
  for (int k = 0; k < 3; k++) {
    base[k] = median(buf[k], nLearn);
    tape[k] = base[k] > thr;                // started with this sensor on (dark) tape
    if (!tape[k]) { sum += base[k]; ok++; }
  }
  if (!ok) { record("nobase", l, c, r, alvik.get_distance_bottom(), "all three sensors on tape"); return; }
  for (int k = 0; k < 3; k++) if (tape[k]) base[k] = sum / ok;
  ready = true;
  record("base", l, c, r, alvik.get_distance_bottom(), ok < 3 ? "a sensor on tape took the others' level" : "");
}

// Has this line sensor's reading moved past the edge level?
bool pastEdge(int v, int k) {
  float d = v - base[k], dir = P("edge_dir");
  float s = dir > 0 ? d : dir < 0 ? -d : fabsf(d);
  float mx = P("edge_max");
  return s >= P("edge_thr") && (mx <= 0 || s <= mx);
}

// Called by shouldAbort() on every call. True = stop: the nose is at an open edge.
bool check() {
  int mode = (int)P("safe_edge");
  if (mode <= 0) return false;
  float sl, sr;
  alvik.get_wheels_speed(sl, sr);
  float minRpm = P("edge_min_rpm");
  bool still = fabsf(sl) < minRpm && fabsf(sr) < minRpm;
  bool forward = 0.5f * (sl + sr) > minRpm;
  int L, C, R;
  alvik.get_line_sensors(L, C, R);
  if (learning) {
    if (still && nLearn < NL) { buf[0][nLearn] = L; buf[1][nLearn] = C; buf[2][nLearn] = R; nLearn++; }
    if (nLearn >= NL || (millis() - t0 >= (unsigned long)P("edge_learn_ms") && nLearn >= 5) || forward)
      finishLearning(L, C, R);
  }
  if (braking) {
    if (still) braking = false;
    else return false;
  }
  if (!forward) { lineCount = 0; botSince = 0; return false; }
  if ((mode & 1) && ready) {
    if (pastEdge(L, 0) || pastEdge(C, 1) || pastEdge(R, 2)) lineCount++;
    else lineCount = 0;
    if (lineCount >= (int)fmaxf(1.0f, P("edge_n"))) {
      record("trip", L, C, R, alvik.get_distance_bottom(), "line sensors");
      lastCue = "line";
      braking = true;
      lineCount = 0;
      return true;
    }
  }
  float lim = P("edge_bot_cm");
  if ((mode & 2) && lim > 0) {
    float d = alvik.get_distance_bottom();
    if (d > lim) {
      if (!botSince) botSince = millis() | 1;
      if (millis() - botSince >= (unsigned long)P("edge_bot_ms")) {
        record("trip", L, C, R, d, "bottom zone");
        lastCue = "bottom";
        braking = true;
        botSince = 0;
        return true;
      }
    } else {
      botSince = 0;
    }
  }
  return false;
}

}  // namespace edge
