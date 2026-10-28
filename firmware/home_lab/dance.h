// Dance (firmware 2026-10-28a; Monday's end, Nick 2026-10-27 18:26: "a small built-in dance (spins, wiggles, lights)
// until switched off"; the Arduino factory demo is gone with this firmware). Where it stands: a spin each way, wiggles,
// both lights stepping through colours. Timed wheel moves only (no gyro needed), so `run dance` works even before the
// compass check. Every wait goes through waitMs(): stop, halt, the cancel button, a lift and the edge guard all end it
// at once. In a follow run it is step Z (`fs <id> Z <ms>`, 0 = until stopped); on its own: `run dance` (until stop).
#pragma once

#include "motion.h"

namespace dance {

// one move: left wheel rpm, right wheel rpm, ms; colours: which of r g b are lit
struct Move { int8_t l, r; uint16_t ms; uint8_t col; };
// spin left about a turn, wiggle, spin back right, wiggle, a short stand (colour index into COLS)
const Move MOVES[] = {
  {-1, 1, 2300, 0}, {0, 0, 250, 1}, {1, -1, 300, 2}, {-1, 1, 300, 3}, {1, -1, 300, 4}, {-1, 1, 300, 5},
  {0, 0, 250, 6}, {1, -1, 2300, 1}, {0, 0, 250, 2}, {-1, 1, 300, 3}, {1, -1, 300, 4}, {0, 0, 600, 0}};
const uint8_t COLS[][3] = {{1, 0, 1}, {0, 1, 1}, {1, 1, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
const int N_MOVES = sizeof(MOVES) / sizeof(MOVES[0]);

void lights(int c, bool swap) {
  const uint8_t* a = COLS[c % 7];
  const uint8_t* b = COLS[(c + 3) % 7];
  if (swap) { const uint8_t* t = a; a = b; b = t; }
  alvik.left_led.set_color(a[0], a[1], a[2]);
  alvik.right_led.set_color(b[0], b[1], b[2]);
}

// Dance for ms (0 = until something stops it). Returns false when stopped early (as waitMs).
bool run(unsigned long ms) {
  float rpm = fmaxf(5.0f, fminf(60.0f, P("dance_rpm")));
  float saved = gFrontLimitCm;
  gFrontLimitCm = 0;                 // it turns where it stands: nothing ahead matters
  bool blinking = blink::on;
  blink::on = false;                 // its own colours (the blink code would overwrite them); back after
  unsigned long t0 = millis();
  bool ok = true;
  for (int k = 0; ok && (ms == 0 || millis() - t0 < ms); k++) {
    const Move& m = MOVES[k % N_MOVES];
    lights(m.col + k / N_MOVES, k & 1);
    alvik.set_wheels_speed(m.l * rpm, m.r * rpm);
    ok = waitMs(m.ms);
  }
  alvik.brake();
  gFrontLimitCm = saved;
  blink::on = blinking;
  if (blinking) { blink::lit = true; blink::set(false); }
  return ok;
}

}  // namespace dance
