// Light pattern on the robot's top, for the overhead camera (OFF by default).
//
// A second way, next to the blink code, for the camera to name a robot: a
// pattern of bright LEDs it can read in ONE picture (the blink code needs
// about a second of rhythm). The camera tools: tools/lights (design note in
// tools/blink/README.md, "Light patterns").
//
// lights_mode 1: three LEDs in an L on the printed holder (hardware/dock),
//   wired to three Nano pins (lights_pin_c corner A2, _f front A3, _s side A7;
//   Arduino pin numbers 19, 20, 24, as switches.h explains). Corner
//   and front are lit steadily; the side LED follows lights_seq, a 4-slot
//   cycle of 160 ms slots: 0 = steady (1111), 1 = 0111, 2 = 0011, 3 = 0101,
//   4 = 0001. Which holder variant (side arm length, left or right) a robot
//   wears is the hardware's part of its code; give robots with the SAME
//   holder different lights_seq.
//   A GPIO pin gives 3.3 V and a few mA: for bright LEDs drive each through
//   a transistor from 5 V with a series resistor (see tools/blink/README.md).
// lights_mode 2: an Arduino Modulino Pixels board (8 RGB LEDs in a row, on
//   the Qwiic connector) along the robot's forward axis, LED 0 at the front,
//   showing a static code (lights_strip_code 1..5, 0 = from the name).
//   Needs the Arduino_Modulino library; without it this mode only reports
//   that it is missing. White (all three colours) because the camera is
//   black-and-white; keep lights_strip_bri low so a lit LED does not bloom
//   over its dark neighbours in the picture.
//
// When: during runs, from blink::start() to blink::stop() (lights_idle 1:
// also while idle, from power-on). The side-LED cycle only advances when
// tick() runs, as the blink code does.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

#if __has_include(<Modulino.h>)
#include <Modulino.h>
#define LIGHTS_HAVE_MODULINO 1
#else
#define LIGHTS_HAVE_MODULINO 0
#endif

namespace lights {

const char* SEQS[] = {"1111", "0111", "0011", "0101", "0001"};
const int N_SEQS = 5;
// strip codes: LED 0 (front) and 1 = start, 2 and 6 dark, 7 = stop, 3..5 = the code
const char* STRIP_CODES[] = {"11000001", "11010001", "11001001", "11000101", "11010101"};
const unsigned long SLOT_MS = 160;

bool on = false;
int mode = 0;
int seq = 0;
int pins[3] = {-1, -1, -1};
int sideLit = -1;      // last state written to the side LED (-1 = unknown)
unsigned long t0 = 0;
bool stripReady = false;

#if LIGHTS_HAVE_MODULINO
ModulinoPixels pixels;
#endif

int stripCodeFor(const char* name) {
  int c = (int)P("lights_strip_code");
  if (c >= 1 && c <= 5) return c;
  if (!strcmp(name, "Left")) return 1;
  if (!strcmp(name, "Center")) return 2;
  if (!strcmp(name, "Right")) return 3;
  return 4;
}

void pin(int p, bool v) {
  if (p >= 0) digitalWrite(p, v ? HIGH : LOW);
}

void strip(bool show) {
#if LIGHTS_HAVE_MODULINO
  if (!stripReady) {
    Modulino.begin();
    pixels.begin();
    stripReady = true;
  }
  const char* bits = STRIP_CODES[stripCodeFor(net::name) - 1];
  int bri = (int)constrain(P("lights_strip_bri"), 0, 100);
  for (int i = 0; i < 8; i++) {
    bool lit = show && bits[i] == '1';
    pixels.set(i, ModulinoColor(255, 255, 255), lit ? bri : 0);
  }
  pixels.show();
#else
  (void)show;
#endif
}

// Starts showing the pattern (no-op if lights_mode is 0 or the hardware isn't set up).
void start() {
  if (on) return;
  mode = (int)P("lights_mode");
  if (mode == 1) {
    pins[0] = (int)P("lights_pin_c");
    pins[1] = (int)P("lights_pin_f");
    pins[2] = (int)P("lights_pin_s");
    if (pins[0] < 0 || pins[1] < 0 || pins[2] < 0) {
      net::sendLine("M,lights: lights_mode 1 needs lights_pin_c, lights_pin_f and lights_pin_s (Arduino pin numbers: A2 = 19)");
      return;
    }
    seq = (int)P("lights_seq");
    if (seq < 0 || seq >= N_SEQS) seq = 0;
    for (int k = 0; k < 3; k++) pinMode(pins[k], OUTPUT);
    pin(pins[0], true);
    pin(pins[1], true);
    pin(pins[2], SEQS[seq][0] == '1');
    sideLit = SEQS[seq][0] == '1';
  } else if (mode == 2) {
    if (!LIGHTS_HAVE_MODULINO) {
      net::sendLine("M,lights: lights_mode 2 needs the Arduino_Modulino library (Library Manager: Modulino), then upload again");
      return;
    }
    strip(true);
  } else {
    return;
  }
  t0 = millis();
  on = true;
}

void tick() {
  if (!on || mode != 1 || seq == 0) return;
  unsigned long slot = ((millis() - t0) / SLOT_MS) % 4;
  int v = SEQS[seq][slot] == '1';
  if (v != sideLit) {
    pin(pins[2], v);
    sideLit = v;
  }
}

void stop(bool force = false) {
  if (!on) return;
  if (!force && P("lights_idle") > 0) return;   // stays on between runs
  if (mode == 1) for (int k = 0; k < 3; k++) pin(pins[k], false);
  else if (mode == 2) strip(false);
  on = false;
}

// At power-on: lights_idle 1 shows the pattern from the start.
void begin() {
  if (P("lights_idle") > 0) start();
}

}  // namespace lights
