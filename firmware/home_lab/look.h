// Calibration run: look around first, then roll where there is room.
//
// While the robot spins in place (the calibration spins), every front sensor
// reading is turned into a point around the robot's turning centre and kept
// per 3-degree direction (the nearest point seen that way). Then, for every
// heading, the room to roll is the distance to the nearest point inside the
// robot's own lane (path_half_cm either side of its centre line, the body's
// width), measured ahead of the sensor like any front reading. The robot
// turns to the heading with room for the full roll (the middle of the widest
// open stretch, so it rolls away from boards rather than along them) and
// rolls min(calib_roll_cm, room - safe_front_cm - calib_margin_cm). The
// sensor cannot see a plywood edge (no board to reflect from): the laptop
// may send caps per heading (`set calib_dir_limits ...`, below), and with
// safe_edge on the edge guard (edge.h) stops the forward roll at one.
//
// Headings here are RELATIVE to the way the robot faced when the look started
// (after the compass wiggle, which returns it to within a few degrees of where
// it was put down), left positive.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"
#include "motion.h"

extern Arduino_Alvik alvik;

namespace look {

const int NB = 120;               // directions, 3 degrees apart; bin 0 = straight ahead
const float BIN = 3.0f;
const float ZONE_HALF = 5.5f;     // each of the five front zones covers about 11 degrees
const float SEEN_HALF = 24.0f;    // a heading counts only if every direction this close to it was seen
const float NOTHING = 999.0f;     // room when nothing at all was seen in the lane
static const float ZONE_DEG[5] = {22.0f, 11.0f, 0.0f, -11.0f, -22.0f};

float minR[NB];                   // nearest point seen that way, cm from the turning centre (0 = none)
bool seen[NB];
float h0 = 0;                     // heading() the look is relative to
bool active = false;              // gTickHook: sample on every shouldAbort() call
float nearNow = 0;                // nearest point (cm from the turning centre) in the frames since resetNear()

// Laptop caps (`set calib_dir_limits <deg> <cm> ...`): each caps the roll length to cm for
// headings within CAP_HALF of deg; the smaller cap wins where they overlap; headings no cap
// covers are not capped. Used by the next calibration run only, then cleared.
const int MAXCAP = 8;
const float CAP_HALF = 22.5f;
float capDeg[MAXCAP], capCm[MAXCAP];
int nCap = 0;

int binOf(float rel) {
  float a = fmodf(rel, 360.0f);
  if (a < 0) a += 360.0f;
  return ((int)floorf(a / BIN + 0.5f)) % NB;
}

void reset() {
  for (int b = 0; b < NB; b++) { minR[b] = 0; seen[b] = false; }
  h0 = heading();
  nearNow = 0;
}

void resetNear() { nearNow = 0; }

// One reading d (cm ahead of the sensor, along zone direction z deg) with the robot at
// relative heading rel.
void addPoint(float rel, float z, float d) {
  for (float x = -ZONE_HALF; x <= ZONE_HALF + 0.01f; x += BIN) seen[binOf(rel + z + x)] = true;
  seen[binOf(rel + z + ZONE_HALF)] = true;
  if (d <= 0 || d < P("tof_min_cm")) return;   // no reading, or too close to trust: nothing counted that way
  float s = P("tof_ahead_cm");
  float zr = z * 0.017453293f;
  float x = s + d * cosf(zr), y = d * sinf(zr);
  float r = sqrtf(x * x + y * y);
  float ang = rel + atan2f(y, x) * 57.29578f;
  int b = binOf(ang);
  if (minR[b] == 0 || r < minR[b]) minR[b] = r;
  if (nearNow == 0 || r < nearNow) nearNow = r;
}

// The current front frame (five zones and the top zone) into the map.
void sample() {
  float d[5];
  alvik.get_distance(d[0], d[1], d[2], d[3], d[4]);
  float rel = wrap180(heading() - h0);
  for (int k = 0; k < 5; k++) addPoint(rel, ZONE_DEG[k], d[k]);
  addPoint(rel, 0.0f, alvik.get_distance_top());
}

void tickHook() { if (active) sample(); }

// Room to roll at relative heading h: cm ahead of the sensor to the nearest point inside
// the robot's lane (NOTHING if none). A point beside the body gives a negative room.
float roomAt(float h) {
  float half = P("path_half_cm"), s = P("tof_ahead_cm"), best = NOTHING;
  for (int b = 0; b < NB; b++) {
    if (minR[b] <= 0) continue;
    float a = (b * BIN - h) * 0.017453293f;
    float fwd = minR[b] * cosf(a), lat = minR[b] * sinf(a);
    if (fwd > 0 && fabsf(lat) <= half && fwd - s < best) best = fwd - s;
  }
  return best;
}

bool seenAround(float h) {
  for (float x = -SEEN_HALF; x <= SEEN_HALF + 0.01f; x += BIN) if (!seen[binOf(h + x)]) return false;
  return true;
}

float capAt(float h) {      // the laptop's cap for heading h, cm (-1 = none)
  float c = -1;
  for (int k = 0; k < nCap; k++)
    if (fabsf(wrap180(h - capDeg[k])) <= CAP_HALF && (c < 0 || capCm[k] < c)) c = capCm[k];
  return c;
}

// The roll that fits at heading h (cm; may be below calib_roll_min_cm or negative).
float rollAt(float h) {
  if (!seenAround(h)) return -1;
  float r = fminf(P("calib_roll_cm"), roomAt(h) - P("safe_front_cm") - P("calib_margin_cm"));
  float c = capAt(h);
  if (c >= 0) r = fminf(r, c);
  return r;
}

struct Choice { float heading, room, cap, roll; bool full; };

// Pick the roll heading: among headings with room for the full roll, the middle of the
// widest stretch of them (straight ahead if every heading has room); otherwise the heading
// with the longest roll that fits (the nearest to straight ahead on a tie).
Choice choose() {
  float want = P("calib_roll_cm");
  static bool full[NB];
  int nFull = 0;
  for (int b = 0; b < NB; b++) { full[b] = rollAt(b * BIN) >= want - 0.01f; if (full[b]) nFull++; }
  float h = 0;
  bool isFull = nFull > 0;
  if (nFull > 0 && nFull < NB) {
    int bestLen = 0; float bestMid = 0;
    for (int b = 0; b < NB; b++) {
      if (!full[b] || full[(b + NB - 1) % NB]) continue;     // b starts a stretch
      int len = 0;
      while (len < NB && full[(b + len) % NB]) len++;
      float mid = wrap180((b + (len - 1) / 2.0f) * BIN);
      if (len > bestLen || (len == bestLen && fabsf(mid) < fabsf(bestMid))) { bestLen = len; bestMid = mid; }
    }
    h = bestMid;
  } else if (nFull == 0) {
    float best = -1e9;
    for (int b = 0; b < NB; b++) {
      float hb = wrap180(b * BIN), r = rollAt(hb);
      if (r > best + 0.01f || (fabsf(r - best) <= 0.01f && fabsf(hb) < fabsf(h))) { best = r; h = hb; }
    }
  }
  Choice c;
  c.heading = h;
  c.room = roomAt(h);
  c.cap = capAt(h);
  c.roll = rollAt(h);
  c.full = isFull;
  return c;
}

// Clear ranges at 0, 45, ..., 315 deg for the log: "r0;r45;...", -1 = not seen.
String roomSummary() {
  String s = "";
  for (int k = 0; k < 8; k++) {
    float h = k * 45.0f;
    float r = seenAround(h) ? roomAt(h) : -1;
    if (k) s += ";";
    s += String(r, 1);
  }
  return s;
}

// `set calib_dir_limits <deg> <cm> [<deg> <cm> ...]` or `set calib_dir_limits off`.
void setLimits(String rest) {
  rest.trim();
  nCap = 0;
  if (!rest.length() || rest.equalsIgnoreCase("off")) {
    net::sendLine("M,calib_dir_limits off");
    return;
  }
  float v[2 * MAXCAP];
  int n = 0;
  const char* p = rest.c_str();
  while (*p && n < 2 * MAXCAP) {
    char* end = nullptr;
    float x = strtof(p, &end);
    if (end == p) { net::logf("M,calib_dir_limits: '%s' is not a number (none set)", p); return; }
    v[n++] = x;
    p = end;
    while (*p == ' ' || *p == ',' || *p == ':') p++;
  }
  if (*p || n % 2) { net::sendLine("M,usage: set calib_dir_limits <deg> <cm> [<deg> <cm> ...] (at most 8 pairs) or off"); return; }
  for (int k = 0; k < n / 2; k++) { capDeg[k] = wrap180(v[2 * k]); capCm[k] = v[2 * k + 1]; }
  nCap = n / 2;
  String s = "";
  for (int k = 0; k < nCap; k++) s += String(" ") + String(capDeg[k], 0) + ":" + String(capCm[k], 1);
  net::logf("M,calib_dir_limits%s (deg:cm, relative to where the robot faces; next calibration only)", s.c_str());
}

void printLimits() {
  if (!nCap) { net::sendLine("M,calib_dir_limits off"); return; }
  String s = "";
  for (int k = 0; k < nCap; k++) s += String(" ") + String(capDeg[k], 0) + ":" + String(capCm[k], 1);
  net::logf("M,calib_dir_limits%s", s.c_str());
}

}  // namespace look
