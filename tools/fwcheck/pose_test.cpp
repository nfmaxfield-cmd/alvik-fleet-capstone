// Host test of pose.h's fix filter (2026-11-12e21). Stand-ins: P(), millis(), the gyro heading and the library pose.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
std::map<std::string, float> PV;
float P(const char* n) { return PV[n]; }
unsigned long NOW = 100000;
unsigned long millis() { return NOW; }
float HEAD = 0, LX = 0, LY = 0;
inline bool isNaN(float v) { return v != v; }
float wrap180(float a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }
float heading() { return HEAD; }
struct Alvik { void get_pose(float& x, float& y, float& th) { x = LX; y = LY; th = HEAD; } } alvik;
#include "pose_only.h"
int fails = 0;
void chk(const char* what, bool ok) { printf("%s %s\n", ok ? "ok  " : "FAIL", what); if (!ok) fails++; }
void defaults_new() {
  PV.clear();
  PV["wheel_scale"] = 1; PV["pose_q_cm"] = 0.01f; PV["pose_q_deg"] = 0.003f; PV["pose_q_dps"] = 0.002f;
  PV["fix_gate"] = 4; PV["fix_max_age_ms"] = 1500; PV["pose_sd_per_cm"] = 0.095f; PV["pose_hist_ms"] = 3840;
  PV["fix_escape_n"] = 5;
}
void defaults_old() {
  defaults_new();
  PV["fix_max_age_ms"] = 1000; PV["pose_sd_per_cm"] = 0; PV["pose_hist_ms"] = 1280; PV["fix_escape_n"] = 0;
}
// drive straight along +x by d cm in steps, 40 ms apart
void drive(float d, float step = 1.0f) {
  for (float s = 0; s < d - 1e-4f; s += step) { LX += step; NOW += 40; pose::tick(); }
}
void start_at(float x, float y) {
  NOW = 100000; HEAD = 0; LX = LY = 0;
  pose::start(true, x, y, 0);
  pose::set(x, y, 0, 1.0f, 2.0f);
}
int main() {
  // 1. the linear sd growth (new) against the old variance growth
  defaults_new(); start_at(50, 50); drive(100);
  float sdNew = pose::sd();
  defaults_old(); start_at(50, 50); drive(100);
  float sdOld = pose::sd();
  printf("     sd after 100 cm: new %.2f cm, old %.2f cm\n", sdNew, sdOld);
  chk("new: sd grows 0.095 cm per cm (1 + 9.5 = 10.5 cm after 100 cm)", fabsf(sdNew - 10.5f) < 0.3f);
  chk("old: sd grows as sqrt(0.01 per cm) (about 1.4 cm after 100 cm)", sdOld < 1.6f && sdOld > 1.2f);
  // 2. the age limit: 1.2 s old is refused by the old firmware (1000 ms) and taken by the new (1500 ms)
  defaults_old(); start_at(50, 50); drive(30);
  unsigned long t = NOW - 1200;
  chk("old: a fix 1.2 s old is refused as old", !pose::fix(t, 60, 50, NAN, 1, 3) && pose::rejOld == 1);
  defaults_new(); start_at(50, 50); drive(30);
  t = NOW - 1200;
  {
    float ax = 0, ay = 0, ath = 0;
    pose::at(t, ax, ay, ath);
    chk("new: a fix 1.2 s old is used", pose::fix(t, ax + 0.5f, ay, NAN, 1, 3));
  }
  // 3. the history: 2 s back is beyond the old 1.28 s history (with the age limit raised) but inside the new 3.84 s
  defaults_old(); PV["fix_max_age_ms"] = 3000; start_at(50, 50); drive(80, 1.0f);
  t = NOW - 2000;
  chk("old history (1280 ms): a fix 2 s old is refused as older than the history",
      !pose::fix(t, 60, 50, NAN, 1, 3) && pose::rejHist == 1);
  defaults_new(); PV["fix_max_age_ms"] = 3000; start_at(50, 50); drive(80, 1.0f);
  t = NOW - 2000;
  float hx = 0, hy = 0, hth = 0;
  pose::at(t, hx, hy, hth);
  chk("new history (3840 ms): a fix 2 s old is used against its pose then", pose::fix(t, hx + 0.5f, hy, NAN, 1, 3));
  // 4. the gate and the ratchet: 10 cm off just after a fix (sd 1): refused as far; the old one refuses for ever
  defaults_old(); start_at(50, 50);
  int used = 0;
  for (int k = 0; k < 20; k++) { NOW += 600; used += pose::fix(NOW - 300, 60, 50, NAN, 1, 3) ? 1 : 0; }
  chk("old: 20 fixes steady 10 cm off, all refused (the ratchet)", used == 0 && pose::rejFar == 20);
  // 5. the escape: the new firmware takes them after fix_escape_n agreeing ones
  defaults_new(); start_at(50, 50);
  used = 0;
  int at_n = -1;
  for (int k = 0; k < 20; k++) {
    NOW += 600;
    bool u = pose::fix(NOW - 300, 60.0f + 0.3f * ((k % 3) - 1), 50.0f + 0.3f * ((k % 2) ? 1 : -1), NAN, 1, 3);
    if (u && at_n < 0) at_n = k + 1;
    used += u ? 1 : 0;
  }
  printf("     escape at fix %d, own pose now %.1f %.1f\n", at_n, pose::x, pose::y);
  chk("new: the escape at the 5th agreeing far fix", at_n == 5 && pose::escapes == 1);
  chk("new: its own pose moved to the camera's (within 1 cm)", fabsf(pose::x - 60) < 1.0f && fabsf(pose::y - 50) < 1.0f);
  chk("new: the fixes after the escape are used", used >= 15);
  // 6. far fixes that disagree with each other (a flicker between two spots) never escape
  defaults_new(); start_at(50, 50);
  for (int k = 0; k < 20; k++) { NOW += 600; pose::fix(NOW - 300, (k % 2) ? 60 : 50, (k % 2) ? 50 : 62, NAN, 1, 3); }
  chk("new: far fixes that jump between two spots never escape", pose::escapes == 0 && pose::rejFar == 20);
  // 7. a near fix resets the escape count
  defaults_new(); start_at(50, 50);
  for (int k = 0; k < 4; k++) { NOW += 600; pose::fix(NOW - 300, 60, 50, NAN, 1, 3); }
  NOW += 600; pose::fix(NOW - 300, 50.5f, 50, NAN, 1, 3);
  for (int k = 0; k < 4; k++) { NOW += 600; pose::fix(NOW - 300, 60, 50, NAN, 1, 3); }
  chk("new: an agreeing fix between resets the count (4 + 4 far: no escape)", pose::escapes == 0);
  // 8. future fixes and the first fix's heading are counted by reason
  defaults_new(); NOW = 100000; HEAD = 0; LX = LY = 0; pose::start(false);
  chk("first fix without a heading: refused, counted", !pose::fix(NOW, 1, 1, NAN, 1, 3) && pose::rejFirst == 1);
  chk("first fix with a heading sets the pose", pose::fix(NOW, 1, 1, 0, 1, 3) && pose::known);
  chk("a fix from the future: refused, counted", !pose::fix(NOW + 500, 1, 1, NAN, 1, 3) && pose::rejFuture == 1);
  printf("%d failed\n", fails);
  return fails;
}
