// Four-sided virtual border (firmware 2026-10-27a; school demo, docs/build-plans/mission-control.md).
//
// The table is a rectangle (bd_x0 <= x <= bd_x1, bd_y0 <= y <= bd_y1, floor cm, the camera's frame) and each side has
// a kind (bd_k_x0, bd_k_x1, bd_k_y0, bd_k_y1): 0 = barrier (a wall: nothing to fall off, no border), 1 = open edge (a
// drop: the border holds the robot's centre border_cm inside it), 2 = partly both (the border holds only where the
// robot is alongside the open stretch bd_lo_* .. bd_hi_*, measured along the side). The laptop sends all of it from
// the workspace map (tools/fleet/workspace.py firmware_params) with bd_on 1. bd_on 0 (the default) = the older
// two-edge border (e02_open_x_cm / e02_open_y_cm, e02.h), unchanged.
//
// Used by e02.h's borderDist (e02 runs, when bd_on is 1) and by the step follower (follow.h): there a border stop
// ends only the step (failed "virtual border at <x> <y> <th> <dist>"), like the edge guard, and the laptop re-plans.
#pragma once

#include "pose.h"

namespace border {

const float UNSET = -999.0f;

// Inward distance (cm) of the point (px, py) from the nearest side that is open there; UNSET when no side is open
// at that point or the four-sided border is off.
float dist4(float px, float py) {
  if (P("bd_on") <= 0) return UNSET;
  float d = 1e9f;
  struct S { const char* k; const char* lo; const char* hi; float in; float along; };
  S sides[4] = {
    {"bd_k_x0", "bd_lo_x0", "bd_hi_x0", px - P("bd_x0"), py},
    {"bd_k_x1", "bd_lo_x1", "bd_hi_x1", P("bd_x1") - px, py},
    {"bd_k_y0", "bd_lo_y0", "bd_hi_y0", py - P("bd_y0"), px},
    {"bd_k_y1", "bd_lo_y1", "bd_hi_y1", P("bd_y1") - py, px},
  };
  for (int i = 0; i < 4; i++) {
    int kind = (int)P(sides[i].k);
    if (kind == 1) d = fminf(d, sides[i].in);
    else if (kind == 2) {
      // alongside the open stretch (with the robot's half width either side of it: a corner can still go over)
      float half = 5.0f;   // chosen: the Alvik's half width plus a little (4.75 cm); DOE: not needed (geometry)
      if (sides[i].along >= P(sides[i].lo) - half && sides[i].along <= P(sides[i].hi) + half)
        d = fminf(d, sides[i].in);
    }
  }
  return d >= 1e8f ? UNSET : d;
}

// The step follower's border (follow.h tickHook): within border_cm of an open part and moving towards it by more
// than wheel jitter -> the step stops (gStepStop = "border"). Moving away or along the side is always allowed, so the
// next step of a re-plan can drive the robot back in.
const float TOWARDS_CM = 0.5f;      // as e02.h BORDER_TOWARDS_CM (chosen there)
const float MOSTLY = 0.5f;          // as e02.h BORDER_MOSTLY
float ref = 1e9f, refDist = 0;
float lastD = UNSET;

void reset() { ref = 1e9f; refDist = 0; }

// Returns true when the step must stop now.
bool followTick(float poseDist, bool known) {
  if (!known || P("border_cm") <= 0 || P("bd_on") <= 0) { ref = 1e9f; return false; }
  float d = dist4(pose::x, pose::y);
  lastD = d;
  if (d == UNSET) { ref = 1e9f; return false; }
  if (ref >= 1e8f) { ref = d; refDist = poseDist; return false; }
  if (d > ref || fabsf(poseDist - refDist) > 30.0f) { ref = d; refDist = poseDist; }
  if (d >= P("border_cm") || ref - d < TOWARDS_CM) return false;
  if (ref - d < MOSTLY * fabsf(poseDist - refDist)) return false;
  ref = d;
  refDist = poseDist;
  return true;
}

}  // namespace border
