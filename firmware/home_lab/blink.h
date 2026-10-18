// Blink codes: while a run is going, each robot blinks both top lights in its
// own pattern, so the overhead camera can find it on any background (floors
// don't blink) and tell which robot it is without markers.
//
// Rate code: one 100 ms flash, repeating every (n + 2) x 100 ms:
//   Left 0.3 s, Center 0.4 s, Right 0.5 s, any other name 0.6 s
//   (n = 1, 2, 3, 4; or set blink_code 1..4 to choose).
// Every robot flashes several times a second, so the camera gets a fix
// often, and identity comes from the rhythm, read in about a second.
// The lights can only change when the code runs, and some library calls
// block for up to ~200 ms. The lights go dark before the longest blocking
// calls (hold()), but a flash can still be stretched by up to ~115 ms by the
// shorter ones: it stays under ~220 ms, well short of the 300 ms sync flash.
// At 30 camera frames a second a flash is 3 frames.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

extern Arduino_Alvik alvik;

namespace blink {

bool on = false;
int code = 0;
bool lit = false;
unsigned long t0 = 0;
const unsigned long SLOT_MS = 100;

int codeFor(const char* name) {
  int c = (int)P("blink_code");
  if (c >= 1 && c <= 4) return c;
  if (!strcmp(name, "Left")) return 1;
  if (!strcmp(name, "Center")) return 2;
  if (!strcmp(name, "Right")) return 3;
  return 4;
}

void set(bool v) {
  if (v == lit) return;
  lit = v;
  float k = v ? 1 : 0;
  alvik.left_led.set_color(k, k, k);
  alvik.right_led.set_color(k, k, k);
}

// True if slot s (counted from blink start) is lit for code n.
bool slotLit(unsigned long s, int n) {
  return (s % (unsigned long)(n + 2)) == 0;
}

void start() {
  on = P("blink_codes") > 0;
  if (!on) return;
  code = codeFor(net::name);
  t0 = millis();
  lit = true;    // force the first set() to send
  set(false);
}

void tick() {
  if (!on) return;
  unsigned long slot = (millis() - t0) / SLOT_MS;
  if (slot < 3) { set(false); return; }   // 300 ms dark first, so the start flash stands alone
  slot -= 3;
  set(slotLit(slot, code));
}

// Before a call that blocks for a while: go dark, so the pulse can't stretch.
void hold() {
  if (on) set(false);
}

void stop() {
  if (!on) return;
  set(false);
  on = false;
}

}  // namespace blink
