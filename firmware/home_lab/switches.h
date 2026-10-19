// Nose bumper switches (firmware 2026-10-19a+): two lever microswitches on the
// printed nose bracket, one at each front corner (hardware/dock/README.md). They
// tell the home dock run (dock.h) when the robot sits flush against the dock's
// back wall, and make the robot a touch probe (`run touch`, follow step P).
//
// OFF by default (sw_on 0): nothing is read, the pins are left alone, RAW
// swL/swR log -1, and the dock / touch runs refuse to start. Fit the bumper,
// wire it, then `set sw_on 1` (and `set sw_left_pin` / `sw_right_pin` if not
// A0 / A1).
//
// Wiring: each switch's C (common) pin to GND, its NO (normally open) pin to
// the Nano ESP32 pin: A0 left, A1 right (2026-10-19b; D5 / D6 before), the
// holes in the Nano's bottom-row breakout. GND from a Grove connector (a
// Grove-to-jumper cable). The pin is an INPUT_PULLUP: it reads HIGH while the
// lever is free and LOW while it is pressed. Pin numbers are the IDE's
// default numbering ("By Arduino pin"): D0..D13 = 0..13, A0..A7 = 17..24.
// The Alvik itself uses D2, D3, D4 (motor board), A4 / A5 (I2C) and A6.
//
// Debounce: a lever switch chatters for 1-5 ms when it closes. A change of the
// raw reading counts only once it has read the same for sw_debounce_ms; the
// press is timed from the FIRST raw closing (pressMs), so the debounce delay
// does not shift the timing. tick() rides on rawlog::tick() (every 5-10 ms in
// every motion loop), so at the 1.1 cm/s creep speed one call is about 0.1 mm.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"

// Every pin number in params.h (switches, light pattern) is an Arduino pin number. With Tools > Pin Numbering >
// "By GPIO number (legacy)" A0 would be GPIO 1 and 17 a different pin: stop the build instead.
static_assert(A0 == 17 && A7 == 24, "Set Tools > Pin Numbering > 'By Arduino pin (default)': params.h pin numbers are Arduino pins (A0 = 17)");

namespace sw {

bool on = false;
int pin[2] = {-1, -1};               // left, right
bool raw[2] = {false, false};        // pressed, as read (may bounce)
bool st[2] = {false, false};         // pressed, debounced
unsigned long since[2] = {0, 0};     // when the raw reading last changed
unsigned long firstEdge[2] = {0, 0}; // first raw closing since the switch was last (debounced) free; 0 = none
unsigned long pressMs[2] = {0, 0};   // the debounced press's first raw closing; 0 = not pressed
unsigned long presses[2] = {0, 0};   // debounced presses since configure()

bool readPin(int k) { return digitalRead(pin[k]) == LOW; }   // INPUT_PULLUP: pressed pulls the pin to GND

// Called at power-on and at the start of every run, so `set sw_on 1` and pin changes take effect.
void configure() {
  bool want = P("sw_on") > 0;
  int l = (int)P("sw_left_pin"), r = (int)P("sw_right_pin");
  if (!want || l < 0 || r < 0 || l == r) { on = false; return; }
  if (!on || l != pin[0] || r != pin[1]) {
    pin[0] = l;
    pin[1] = r;
    pinMode(l, INPUT_PULLUP);
    pinMode(r, INPUT_PULLUP);
    delay(2);                        // let the pull-ups charge the wires
  }
  on = true;
  unsigned long now = millis();
  for (int k = 0; k < 2; k++) {
    raw[k] = st[k] = readPin(k);
    since[k] = now;
    firstEdge[k] = pressMs[k] = st[k] ? now : 0;
    presses[k] = 0;
  }
}

void tick() {
  if (!on) return;
  unsigned long now = millis();
  unsigned long deb = (unsigned long)fmaxf(0.0f, P("sw_debounce_ms"));
  for (int k = 0; k < 2; k++) {
    bool v = readPin(k);
    if (v != raw[k]) {
      raw[k] = v;
      since[k] = now;
      if (v && !st[k] && !firstEdge[k]) firstEdge[k] = now | 1;
    }
    if (raw[k] != st[k] && now - since[k] >= deb) {
      st[k] = raw[k];
      if (st[k]) { pressMs[k] = firstEdge[k] ? firstEdge[k] : now; presses[k]++; }
      else { pressMs[k] = 0; firstEdge[k] = 0; }
    }
    // a closing that never became a press (free again for the whole debounce time): forget it
    if (!raw[k] && !st[k] && now - since[k] >= deb) firstEdge[k] = 0;
  }
}

bool left() { return on && st[0]; }
bool right() { return on && st[1]; }

// RAW column value: 1 pressed, 0 free, -1 switches not configured
int col(int k) { return on ? (st[k] ? 1 : 0) : -1; }

}  // namespace sw
