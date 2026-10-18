// Blink codes: while a run is going, each robot blinks both top lights in its
// own pattern, so the overhead camera can find it on any background (floors
// don't blink) and tell which robot it is without markers.
//
// Pattern, in 100 ms slots, repeating every 1.5 s (15 slots):
//   n pulses of 200 ms on + 100 ms off, then dark for the rest of the cycle.
//   n = 1 for Left, 2 for Center, 3 for Right, 4 for any other name
//   (or set blink_code 1..4 to choose).
// At 30 camera frames per second each pulse is 6 frames and each gap at
// least 3, so the pattern reads reliably. Pulses never last as long as the
// 300 ms sync flash, and the lights go dark before any call that blocks
// (hold()), so a delayed update can only shorten a pulse, never stretch it
// into something that looks like a sync flash.
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
const int CYCLE_SLOTS = 15;

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

// True if slot s of the cycle is lit for code n.
bool slotLit(int s, int n) {
  if (s >= 3 * n) return false;
  return (s % 3) != 2;   // on, on, off
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
  set(slotLit((int)(slot % CYCLE_SLOTS), code));
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
