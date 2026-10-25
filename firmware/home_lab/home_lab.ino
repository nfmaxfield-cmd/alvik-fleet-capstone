/*
  home_lab — WiFi-logged experiments for one Alvik at home.

  Experiments:  stream (sensor log), turn (turn accuracy/time),
                lap (line-following lap timer), grid (stop vs drive-through),
                demo (open-floor routes), straight (straight-line accuracy),
                survey (square spiral for the camera's self-calibration),
                follow (runs the steps tools/fleet/run_plan.py sends; laptop only),
                dock (home dock with the nose switches), touch (touch probe; dock.h),
                find, e02 (E02 accuracy experiment: find itself against two boards by bumping,
                drive a route and back; e02.h)

  Setup:
    1. Copy secrets.example.h to secrets.h in this folder and add your WiFi.
    2. Upload. On your laptop run:  python tools/alvik_console.py
    3. In the console type:  help

  Without the laptop, use the buttons:
    LEFT / RIGHT  choose an experiment (left LED color shows which)
                  blue = stream, green = turn, yellow = lap, purple = grid,
                  white = demo, cyan = straight, red = survey
    OK            start it        CANCEL  stop it
  Results are logged over WiFi either way (and to Serial if USB is plugged in).
*/

#include "Arduino_Alvik.h"
#if __has_include("esp_sleep.h")
#include "esp_sleep.h"     // deep sleep (the `sleep` command)
#endif
#include "params.h"
#include "net.h"
#include "motion.h"
#include "corr.h"
#include "experiments.h"
#include "dock.h"
#include "follow.h"
#include "look.h"
#include "e02.h"
#include "ota.h"

Arduino_Alvik alvik;

// Bump this whenever the firmware changes, so every logged run says which code it ran.
#define FW_VERSION "2026-10-25b"
// 2026-10-25b: switches at the BACK (one attachment, Nick 2026-10-25): sw_back (dock / touch / posts backwards),
// e02_find_reverse (the find backs into the boards; e02.h REVERSE FIND), e02_back_face_cm; FIND gets a reverse column.

uint16_t gBootId = 0;   // random each power-on; starts every run id

// SLEEP (firmware 2026-10-25a; Nick, 2026-10-24: robots left switched on drain about 14% an hour idle).
// `sleep <minutes>`: motors off, lights off, the WiFi chip in deep sleep for that long. Then it wakes (a fresh
// start), joins the WiFi and checks in ("M,<name>: awake for a check-in ..."); if nothing sends `wake` within
// sleep_awake_s it sleeps again for the same time. While asleep it can't hear anything. `sleep 0`: until the
// reset button or the power switch. The minutes survive the deep sleep in the chip's RTC memory.
RTC_DATA_ATTR uint32_t gSleepMin = 0;
unsigned long gCheckinUntil = 0;   // millis() when this check-in ends (0: not a check-in)

void goToSleep(uint32_t minutes) {
  alvik.brake();
  alvik.left_led.set_color(0, 0, 0);
  alvik.right_led.set_color(0, 0, 0);
  alvik.set_illuminator(false);
  gSleepMin = minutes;
  if (minutes > 0) net::logf("M,%s: sleeping %lu min (it checks in for %d s each time it wakes; `wake` then keeps "
                             "it awake)", net::name, (unsigned long)minutes, (int)P("sleep_awake_s"));
  else net::logf("M,%s: sleeping until its reset button or power switch", net::name);
  delay(300);                      // the message goes out first
  if (minutes > 0) esp_sleep_enable_timer_wakeup((uint64_t)minutes * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

// The buttons cycle through the first N_EXP. "follow" (index 7) needs the
// laptop, so it is started with `run follow` only; "calib" (index 8) is the
// calibration run, started with the calibrate command only; "dock" (9) and
// "touch" (10) and "posts" (11) need the nose switches (sw_on 1) and are started with `run`;
// "gyro" (12) stands still measuring the gyro's drift (corr.h), started with `run gyro`;
// "find" (13) and "e02" (14) are the E02 accuracy experiment (e02.h), started with `run find` / `run e02`.
const char* EXP_NAMES[] = {"stream", "turn", "lap", "grid", "demo", "straight", "survey", "follow", "calib", "dock",
                           "touch", "posts", "gyro", "find", "e02"};
const int N_EXP = 7;
const int N_NAMES = 15;
const int EXP_SURVEY = 6;
const int EXP_FOLLOW = 7;
const int EXP_CALIB = 8;
const int EXP_DOCK = 9;
const int EXP_TOUCH = 10;
const int EXP_POSTS = 11;
const int EXP_GYRO = 12;
const int EXP_FIND = 13;
const int EXP_E02 = 14;
int gSelected = 1;
int gPending = -1;   // experiment requested by the laptop, started from loop()

float P(const char* name) {
  Param* p = findParam(name);
  if (!p) {
    net::logf("M,unknown parameter %s", name);
    return 0;
  }
  return p->value;
}

unsigned long gLedUntil = 0;   // `led`: when the laptop's colour ends (0 = none)

void showSelection(bool running) {
  bool r = 0, g = 0, b = 0;
  switch (gSelected) {
    case 0: b = 1; break;              // stream: blue
    case 1: g = 1; break;              // turn: green
    case 2: r = 1; g = 1; break;       // lap: yellow
    case 3: r = 1; b = 1; break;       // grid: purple
    case 4: r = 1; g = 1; b = 1; break; // demo: white
    case 5: g = 1; b = 1; break;       // straight: cyan
    case 6: r = 1; break;              // survey: red
  }
  alvik.left_led.set_color(r, g, b);
  if (running) alvik.right_led.set_color(r, g, b);
  else         alvik.right_led.set_color(0, 0, 0);
}

void printHelp() {
  net::sendLine("M,commands: run stream|turn|lap|grid|demo|straight|survey|follow|dock|touch|posts|gyro|find|e02  stop  corr [reset|tape|save <name> <value>]  calibrate  clock  name <Left|Center|Right>  set <name> <value>  get <name>  params  route <SLRE...>  status  identify  lights  help");
  net::sendLine("M,during run follow: fs <id> <T|D|A|W> ...  fix <t_ms> <x> <y> <th> <sigma>  fpose <x> <y> <th>  hold [id]  halt  (tools/fleet/run_plan.py sends these)");
  net::sendLine("M,E02: fix <x> <y> <heading|-> <t_wall> (camera fix: idle or K replaces, KC blends e02_fix_k)  go <cm> [id]  turn <deg> [id] (idle, or in run e02 with e02_mode 1)  mark|e02 start|far|end [id]");
  net::sendLine("M,sleep <min> (deep sleep, checks in every <min>; 0 = until reset)  wake  border [clear] (the virtual border)");
  net::sendLine("M,before calibrate: set calib_dir_limits <deg> <cm> ... caps its roll per direction (deg from where it faces, left +; next calibration only; off = none)");
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    net::logf("M,%-18s %8.2f  %s", PARAMS[i].name, PARAMS[i].value, PARAMS[i].help);
    delay(3);   // a burst of back-to-back UDP sends can drop packets
  }
  net::logf("M,route %s  (grid test: S straight, L left, R right, E end)", gRoute.c_str());
}

int expIndex(const String& name) {   // `run <name>`: every experiment but calib (the calibrate command)
  for (int i = 0; i < N_NAMES; i++) if (i != EXP_CALIB && name.equalsIgnoreCase(EXP_NAMES[i])) return i;
  return -1;
}

// Handles one console command. Returns true if it means "stop now".
// While an experiment runs, settings can still change (live tuning), but
// a new "run" is refused.
bool handleCommand(const String& raw, bool running) {
  String cmd = raw;
  cmd.trim();
  int sp = cmd.indexOf(' ');
  String verb = sp < 0 ? cmd : cmd.substring(0, sp);
  String rest = sp < 0 ? "" : cmd.substring(sp + 1);
  rest.trim();
  verb.toLowerCase();

  if (verb == "stop") {
    if (!running) net::sendLine("M,nothing running");
    return true;
  }
  if (verb == "help") { printHelp(); return false; }
  // E02 (e02.h): a camera fix (any time but during a follow run), camera-steered legs
  if (verb == "fix" && !(running && follow::active)) { e02::fixCommand(rest, running); return false; }
  if (verb == "go" || verb == "turn" || verb == "mark") { e02::legCommand(verb, rest, running); return false; }
  if (verb == "e02") { e02::legCommand("mark", rest, running); return false; }   // e02 start|far|end = mark
  if (verb == "fs" || verb == "fix" || verb == "fpose" || verb == "hold" || verb == "halt") {
    if (running && follow::active) follow::command(verb, rest);
    else if (verb == "fs") net::sendLine("M,not following: send run follow first");
    return false;
  }
  if (verb == "name") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    if (rest.length() > 10) { net::sendLine("M,names can be at most 10 characters"); return false; }
    for (unsigned i = 0; i < rest.length(); i++) {
      char c = rest.charAt(i);
      if (!isalnum((unsigned char)c) && c != '-' && c != '_') { net::sendLine("M,names use letters, digits, - and _ only"); return false; }
    }
    String old = String(net::name);
    net::saveName(rest);   // empty name = back to the MAC-based one
    net::logf("M,%s is now called %s (kept after power-off)", old.c_str(), net::name);
    return false;
  }
  if (verb == "led") {
    // led <r> <g> <b> [ms]: both LEDs this colour (0/1 each) for ms (default 2000), then back to normal. Does
    // not block (2026-10-22a): the laptop finds a robot by a steady light at any camera frame rate, and the
    // dance flashes on its beat. Ignored during a run (its own lights are part of the data).
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    int r = 1, g = 1, bl = 1;
    long ms = 2000;
    sscanf(raw.c_str(), "%*s %d %d %d %ld", &r, &g, &bl, &ms);
    alvik.left_led.set_color(r ? 1 : 0, g ? 1 : 0, bl ? 1 : 0);
    alvik.right_led.set_color(r ? 1 : 0, g ? 1 : 0, bl ? 1 : 0);
    gLedUntil = millis() + (unsigned long)constrain(ms, 50L, 20000L);
    net::logf("M,%s: led %d %d %d for %ld ms", net::name, r, g, bl, ms);
    return false;
  }
  if (verb == "identify") {
    // Blocks for 3 s: never during a run (the wheels would keep going unchecked,
    // and the white flashes would look like sync flashes in the video).
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    // Flash both LEDs white so you can tell which physical robot this is.
    for (int i = 0; i < 6; i++) {
      alvik.left_led.set_color(1, 1, 1); alvik.right_led.set_color(1, 1, 1); delay(250);
      alvik.left_led.set_color(0, 0, 0); alvik.right_led.set_color(0, 0, 0); delay(250);
    }
    showSelection(running);
    net::logf("M,%s: that was me", net::name);
    return false;
  }
  if (verb == "lights") {
    // Show the light pattern for 10 s standing still (lights_mode must be set), e.g. to check
    // it on the camera (docs/experiments/ACCURACY-CHECKS.md). Blocks: never during a run.
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    if ((int)P("lights_mode") == 0) { net::sendLine("M,lights: set lights_mode 1 (L on GPIO pins) or 2 (Modulino Pixels) first"); return false; }
    bool was = lights::on;
    lights::start();
    if (!lights::on) return false;
    net::logf("M,%s: showing the light pattern for 10 s", net::name);
    unsigned long t = millis();
    while (millis() - t < 10000) { lights::tick(); delay(10); }
    if (!was) lights::stop(true);
    net::sendLine("M,lights: done");
    return false;
  }
  if (verb == "params") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }   // blocks ~135 ms
    printParams();
    return false;
  }
  if (verb == "sleep") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    long m = rest.length() ? rest.toInt() : 10;
    goToSleep(m > 0 ? (uint32_t)m : 0);
    return false;
  }
  if (verb == "wake") {
    gSleepMin = 0;
    gCheckinUntil = 0;
    net::logf("M,%s: awake (it stays awake now)", net::name);
    return false;
  }
  if (verb == "border") {
    // `border clear`: the laptop's answer after a virtual border stop (e02.h); `border`: where it stands
    if (rest == "clear") { e02::borderLatched = false; e02::borderRef = 1e9f; }
    float d = e02::borderDist(pose::x, pose::y);
    net::logf("M,%s: border %s, %s", net::name, e02::borderLatched ? "latched" : "clear",
              d == e02::UNSET ? "not known" : String(d, 1).c_str());
    return false;
  }
  if (verb == "clock") {
    // Clock check from the laptop: answer at once with our own clock, so the
    // laptop can work out the offset and the round-trip time.
    unsigned long now = millis();
    net::sendLine("H,CLOCK,token,robot_ms,boot");
    net::logf("D,CLOCK,%s,%lu,%04X", rest.c_str(), now, gBootId);   // boot id: the clock restarts if the robot reboots
    return false;
  }
  if (verb == "calibrate") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    gPending = EXP_CALIB;   // a logged run, so the camera sees it too
    return false;
  }
  if (verb == "ping" || verb == "status") {
    net::logf("M,status %s (firmware %s): %s, selected %s, battery %d%%, WiFi %d dBm", net::name, FW_VERSION,
              running ? "running" : "idle", EXP_NAMES[gSelected],
              alvik.get_battery_charge(), (int)WiFi.RSSI());
    return false;
  }
  if (verb == "get") {
    if (rest.equalsIgnoreCase("calib_dir_limits")) { look::printLimits(); return false; }
    Param* p = findParam(rest);
    if (p) net::logf("M,%s = %.3f  (%s)", p->name, p->value, p->help);
    else   net::logf("M,no parameter called %s (type params)", rest.c_str());
    return false;
  }
  if (verb == "set") {
    int sp2 = rest.indexOf(' ');
    // caps on the calibration roll per heading, from the laptop (look.h); not a single number
    if ((sp2 < 0 ? rest : rest.substring(0, sp2)).equalsIgnoreCase("calib_dir_limits")) {
      look::setLimits(sp2 < 0 ? String("") : rest.substring(sp2 + 1));
      return false;
    }
    if (sp2 < 0) { net::sendLine("M,usage: set <name> <value>"); return false; }
    String name = rest.substring(0, sp2);
    String val = rest.substring(sp2 + 1);
    val.trim();
    Param* p = findParam(name);
    if (!p) { net::logf("M,no parameter called %s (type params)", name.c_str()); return false; }
    if (corr::isCorr(name)) net::sendLine("M,(until power-off only: corr save keeps it)");
    char* end = nullptr;
    float v = strtof(val.c_str(), &end);
    if (end == val.c_str()) { net::logf("M,'%s' is not a number", val.c_str()); return false; }
    p->value = v;
    net::logf("M,%s = %.3f", p->name, v);
    return false;
  }
  if (verb == "corr") { corr::command(rest, running); return false; }
  if (verb == "route") {
    rest.toUpperCase();
    for (unsigned i = 0; i < rest.length(); i++) {
      char c = rest.charAt(i);
      if (c != 'S' && c != 'L' && c != 'R' && c != 'E') {
        net::sendLine("M,route letters must be S, L, R or E");
        return false;
      }
    }
    gRoute = rest;
    net::logf("M,route = %s", gRoute.c_str());
    return false;
  }
  if (verb == "run") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }
    int i = expIndex(rest);
    if (i < 0) { net::sendLine("M,run what? stream, turn, lap, grid, demo, straight, survey, follow, dock, touch, posts, gyro, find or e02"); return false; }
    gSelected = i;
    gPending = i;
    return false;
  }
  net::logf("M,unknown command '%s' (type help)", cmd.c_str());
  return false;
}


// Wait while still logging, but WITHOUT the safety checks or command
// polling in waitMs(): used around the sync flashes, which must run to full
// length even when the run was stopped by a still-pressed cancel button, a
// lift or an obstacle. Commands sent meanwhile wait in the WiFi buffer.
void logWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { rawlog::tick(); delay(5); }
}

// Dark, then both lights white for sync_ms, then dark again: the camera sees
// the change, and the robot logs its own clock at that moment, which lines up
// video time with robot time. The lights stay dark ~150 ms either side so the
// flash stands out whatever color they showed before.
void syncFlash(const char* tag) {
  if (P("sync_flash") <= 0) return;
  alvik.left_led.set_color(0, 0, 0);
  alvik.right_led.set_color(0, 0, 0);
  logWait(150);
  // ms_on: just before the calls that switch the lights on; ms_on_after: just
  // after them (each set_color() goes over the serial link to the motor board,
  // so the two differ by how long switching takes: the light-on delay)
  unsigned long on = millis();
  alvik.left_led.set_color(1, 1, 1);
  alvik.right_led.set_color(1, 1, 1);
  unsigned long onAfter = millis();
  logWait((unsigned long)P("sync_ms"));
  alvik.left_led.set_color(0, 0, 0);
  alvik.right_led.set_color(0, 0, 0);
  unsigned long off = millis();
  logWait(150);
  net::sendLine("H,SYNC,run,tag,ms_on,ms_off,ms_on_after");
  net::logf("D,SYNC,%s,%s,%lu,%lu,%lu", gRunId.c_str(), tag, on, off, onAfter);
}

// One record describing the run before it starts: what ran, on what, in
// what state. Every setting follows as its own PARAM record.
void logRunInfo(int i) {
  uint8_t a, b, c;
  alvik.get_fw_version(a, b, c);
  // yaw_sign / odo_sign: what the compass check found (0 = not checked). RAW
  // yaw_deg and th_deg are raw library values; multiply by these for left-positive.
  net::sendLine("H,RUNINFO,run,experiment,fw_version,motor_fw,ms_start,battery,charging,wifi_dbm,route,yaw_sign,odo_sign");
  net::logf("D,RUNINFO,%s,%s,%s,%d.%d.%d,%lu,%d,%d,%d,%s,%d,%d", gRunId.c_str(), EXP_NAMES[i], FW_VERSION,
            a, b, c, millis(), alvik.get_battery_charge(), alvik.is_battery_charging() ? 1 : 0, (int)WiFi.RSSI(),
            gRoute.c_str(), gYawSign, gOdoSign);
  net::sendLine("H,PARAM,run,name,value");
  for (int k = 0; k < N_PARAMS; k++) {
    net::logf("D,PARAM,%s,%s,%.4f", gRunId.c_str(), PARAMS[k].name, PARAMS[k].value);
    delay(2);   // a burst of back-to-back UDP sends can drop packets
  }
}

// Experiments that turn need the compass direction checked once per power-on.
// The check wiggles the robot, so it must never happen inside a measured run
// (after the tape start marks were taken).
bool needsYawSign(int i) { return i == 1 || (i >= 3 && i <= 7) || i == EXP_GYRO || i == EXP_FIND || i == EXP_E02; }

// Calibration run, once per power-on and whenever robots may have been mixed
// up. Logged like any run (RUNINFO, PARAM, RAW at raw_hz, flashes), so the
// camera recording holds it too; tools/blink/calib.py reads it from the video
// and the log. It LOOKS before it rolls (look.h): the spins come first, the
// front sensor's sweep during them shows where there is room, and the roll
// goes that way. RAW step (what the camera / log measure in it):
//   1  still, blinking its code, calib_blink_ms        who is who; front view
//                                                      (anything inside its spin circle?)
//   2  compass wiggle (calibrateYawSign(true))         gyro sign
//   3  still calib_still_ms                            gyro bias; picture before the spin
//   4  spin left calib_spin_deg at calib_spin_rpm      gyro scale, light offset from the
//                                                      turning centre, front sensor sweep (look)
//   5  still calib_still_ms                            picture between the spins; gyro bias
//   6  spin back right calib_spin_deg                  the same, the other way
//   7  still calib_still_ms                            picture after the spins; gyro bias
//   8  turn to the roll heading (IMU turn; none if     heading before the roll (picture);
//      it already faces it), still calib_still_ms      front view re-checked
//   9  roll forward (up to calib_roll_cm), still       heading, wheel distance scale
//  10  roll back the same, still                       the same, the other way
// 4-7 are skipped when calib_spin_deg is 0 or something is within calib_spin_r_cm
// of its turning centre at steps 1-3 (it then only knows the view straight
// ahead); 9-10 are skipped when no heading has room for calib_roll_min_cm. One
// CALIB_LOOK record (docs/data-formats.md), sent after the rolls, says what it
// saw, chose and did; its spin / roll / reason columns say what was skipped or
// cut short and why. A run that skips the spin or the roll for lack of room,
// or whose forward roll the edge guard stops, is NOT a failure (the robot
// refused something it had no room for or that was unsafe): RUNEND says done,
// and the tools (session_checks.calib_outcome) count it as a third outcome,
// "limited" (reason "no room" and/or "plywood edge"). The front stop is off
// except during the rolls (standing and spinning in place it may face a
// board). The blink code runs throughout (moves go dark before the library's
// blocking call, as elsewhere). Defaults: about 62-65 s from RUNINFO to RUNEND
// with the roll (the turn in step 8 takes 0-3 s; the 1 s settle_end_ms at the
// end included); it needs a 25 cm circle to spin in and, in some direction,
// 15 + 7 + 3 cm clear ahead of its front. With safe_edge on (edge.h), the
// forward roll (step 9) stops at an open plywood edge, which the front sensor
// cannot see: it then rolls back the distance it covered (step 10, retracing
// its own path) and the run still ends done, limited (plywood edge); the roll
// counts (roll "edge", roll_cm = the distance covered) if that distance is at
// least calib_roll_min_cm, else it is skipped. The spins, the turn and the
// roll back are not guarded; for the roll back that is by design: it retraces
// the path just driven forward on the board, ending where the roll started. Firmware 2026-10-18b
// rolled first, straight ahead (1 blink, 2 roll, 3 back, 4 wiggle, 5 still,
// 6 spin, 7 still, 8 spin back, 9 still); before that: 1 still, 2 roll
// max(3, start_nudge_cm) with 300 ms pauses, 3 back, 4 wiggle, 5 300 ms.
// What the calibration skipped or cut short (CALIB_LOOK spin / roll / reason), for runCalib().
const char* gCalibSpin = "done";   // done, skipped (something inside its spin circle), off (calib_spin_deg 0)
const char* gCalibRoll = "full";   // full, short, edge (stopped at a plywood edge, kept), skipped
bool gCalibNoRoom = false;         // spin or roll skipped for lack of room          } the "limited"
bool gCalibEdge = false;           // the edge guard stopped the forward roll         } outcome

const char* calibReason() {
  return gCalibNoRoom && gCalibEdge ? "no room; plywood edge" : gCalibNoRoom ? "no room" : gCalibEdge ? "plywood edge" : "";
}

void calibSteps() {
  float spin = P("calib_spin_deg");
  gCalibSpin = spin != 0 ? "done" : "off";
  gCalibRoll = "skipped";
  gCalibNoRoom = gCalibEdge = false;
  unsigned long still = (unsigned long)fmaxf(300.0f, P("calib_still_ms"));
  net::logf("M,%s calibration: blinking code %d, a small wiggle%s, then a roll of up to %.0f cm forward and back "
            "where it finds room", net::name, blink::codeFor(net::name), spin != 0 ? ", a slow spin each way" : "",
            fmaxf(3.0f, P("calib_roll_cm")));
  rawlog::step = 1;
  look::reset();
  look::active = true;
  if (!waitMs((unsigned long)P("calib_blink_ms"))) return;
  float nearStart = look::nearNow;
  look::active = false;
  rawlog::step = 2;
  if (!calibrateYawSign(true)) return;
  rawlog::step = 3;
  look::reset();             // headings from here on are relative to this one
  look::active = true;
  if (!waitMs(still)) return;
  float nearest = look::nearNow;
  if (nearStart > 0 && (nearest <= 0 || nearStart < nearest)) nearest = nearStart;
  bool spun = false;
  float spinR = P("calib_spin_r_cm") + P("bumper_cm");   // a nose bumper's corners sweep further out
  if (spin != 0 && nearest > 0 && nearest < spinR) {
    gCalibSpin = "skipped";
    gCalibNoRoom = true;
    net::logf("M,%s: something is %.1f cm from its middle, inside the %.0f cm circle it needs to spin: no spin "
              "(no gyro scale or light offset this time); it can only look straight ahead",
              net::name, nearest, 2 * spinR);
  } else if (spin != 0) {
    rawlog::step = 4;
    if (!spinBy(fabsf(spin), P("calib_spin_rpm"))) return;
    rawlog::step = 5;
    if (!waitMs(still)) return;
    rawlog::step = 6;
    if (!spinBy(-fabsf(spin), P("calib_spin_rpm"))) return;
    rawlog::step = 7;
    if (!waitMs(still)) return;
    spun = true;
  }
  look::active = false;
  look::Choice c = look::choose();
  float rollMin = fmaxf(1.0f, P("calib_roll_min_cm"));
  bool roll = c.roll >= rollMin;
  float target = wrap180(look::h0 + c.heading), turn = 0;
  float live = 0, L = roll ? c.roll : 0;
  const char* result = !roll ? "no_room" : c.full ? "roll" : "short";
  // turn, look again, roll: every way out of it (an abort included) ends with the one CALIB_LOOK record
  auto turnAndRoll = [&]() {
    rawlog::step = 8;
    if (roll) {
      turn = wrap180(target - heading());
      if (fabsf(turn) > P("turn_tol_deg") && !imuTurnTo(target).ok) return;
    }
    // still: the picture before the roll, and the live front view along the lane once more
    unsigned long t0 = millis();
    while (millis() - t0 < still) {
      if (shouldAbort()) return;
      float f = frontInPath();
      if (f > 0 && (live == 0 || f < live)) live = f;
      delay(5);
    }
    if (roll && live > 0) {
      float fit = live - P("safe_front_cm") - P("calib_margin_cm");
      if (fit < L) {
        L = fit;
        result = L >= rollMin ? "short" : "no_room_after_turn";
        if (L < rollMin) L = 0;
      }
    }
    gCalibRoll = L <= 0 ? "skipped" : !strcmp(result, "roll") ? "full" : "short";
    if (L <= 0) {
      gCalibNoRoom = true;
      net::logf("M,%s: no room to roll (best direction %.0f deg from where it faced: %.0f cm clear ahead%s, a %.0f cm "
                "roll needs %.0f): no heading or wheel scale this time. Give it 13 in (33 cm) of clear floor in some "
                "direction and calibrate again for those.", net::name, c.heading, c.room,
                c.cap >= 0 ? ", capped by the laptop" : "", rollMin, rollMin + P("safe_front_cm") + P("calib_margin_cm"));
      return;
    }
    if (fabsf(c.heading) > 2) net::logf("M,%s: rolling %.1f cm towards %.0f deg (left positive) from where it faced",
                                         net::name, L, c.heading);
    gFrontLimitCm = -1;        // rolling: the usual front stop (safe_front_cm)
    rawlog::step = 9;
    float x0, y0;
    poseXY(x0, y0);
    if (!libraryMove(L)) {
      if (strcmp(gAbortReason.c_str(), "plywood edge ahead")) return;
      // The edge guard refused to roll on (the look-around can't see an open edge): not a failure.
      // Settle, keep the roll if it went far enough, and roll back the way it came (retracing is safe).
      gAbortReason = "";
      gCalibEdge = true;
      float planned = L;
      gCalibRoll = "skipped";  // until it knows how far it went
      result = "edge";
      L = 0;
      if (!waitMs(still)) return;
      float went = distFrom(x0, y0);
      bool keep = went >= rollMin;
      net::logf("M,%s: stopped at a plywood edge after %.1f cm of a %.1f cm roll; %s", net::name, went, planned,
                keep ? "keeping that roll, rolling back" : "too short to measure, rolling back");
      gCalibRoll = keep ? "edge" : "skipped";
      L = keep ? went : 0;
      rawlog::step = 10;
      if (went >= 0.5f && (!libraryMove(-went) || !waitMs(still))) return;
      gFrontLimitCm = 0;
      return;
    }
    if (!waitMs(still)) return;
    rawlog::step = 10;
    if (!libraryMove(-L) || !waitMs(still)) return;
    gFrontLimitCm = 0;
  };
  turnAndRoll();
  String caps = "";
  for (int k = 0; k < look::nCap; k++) caps += String(k ? ";" : "") + String(look::capDeg[k], 0) + ":" + String(look::capCm[k], 1);
  net::sendLine("H,CALIB_LOOK,run,spun,near_cm,heading_deg,turn_deg,room_cm,cap_cm,live_cm,roll_cm,result,room8,caps,"
                "spin,roll,reason");
  net::logf("D,CALIB_LOOK,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%s,%s,%s,%s,%s", gRunId.c_str(), spun ? 1 : 0,
            nearest, c.heading, turn, c.room, c.cap, live, L, result, look::roomSummary().c_str(), caps.c_str(),
            gCalibSpin, gCalibRoll, calibReason());
}

void runCalib() {
  void (*savedHook)() = gTickHook;
  gFrontLimitCm = 0;
  gTickHook = look::tickHook;
  calibSteps();
  look::active = false;
  look::nCap = 0;            // the laptop's caps were for this placement only
  gTickHook = savedHook;
  gFrontLimitCm = -1;        // back to the usual front stop for the next run
  if (gAbortReason.length() || gFail.length()) return;
  if (gCalibNoRoom || gCalibEdge)   // not a failure: it did what the space allowed (RUNEND done; "limited")
    net::logf("M,calibrated, limited (%s):%s%s the rest is kept. Now place the robot on its start mark", calibReason(),
              !strcmp(gCalibSpin, "skipped") ? " no spin (something inside its spin circle);" : "",
              gCalibEdge ? (!strcmp(gCalibRoll, "edge") ? " roll cut short at a plywood edge;"
                                                       : " no roll (stopped at a plywood edge too soon);")
                         : !strcmp(gCalibRoll, "skipped") ? " no roll (no room in any direction);" : "");
  else
    net::sendLine("M,calibrated: now place the robot on its start mark");
}

// Start nudge: a short roll forward and back right after the start flash, so
// the camera sees which way the robot faces. RAW step -1 while it happens.
// Not in follow runs: the laptop gives the pose, and the robot stays on its spot.
bool startNudge(int i) {
  float n = P("start_nudge_cm");
  if (n <= 0 || !(i == 1 || i == 4 || i == 5 || i == EXP_SURVEY)) return true;
  rawlog::step = -1;
  bool ok = libraryMove(n) && waitMs(200) && libraryMove(-n) && waitMs(300);
  rawlog::step = 0;
  return ok;
}

void runExperiment(int i, bool fromButton) {
  if (needsYawSign(i) && gYawSign == 0) {
    if (!fromButton) {
      net::logf("M,not calibrated since power-on: lift the robot off its mark, send calibrate, put it back, "
                "then start again (%s not started)", EXP_NAMES[i]);
      return;
    }
    // Started from the buttons (no laptop): check now, before anything is logged.
    gFail = "";
    if (!calibrateYawSign()) { showSelection(false); return; }
    net::sendLine("M,compass checked: the robot moved slightly, place it again if it is tape-measured");
    delay(1500);
  }
  gRun++;
  char id[16];
  snprintf(id, sizeof(id), "%04X-%d", gBootId, gRun);
  gRunId = id;
  gAbortReason = "";
  gFail = "";
  if (i != EXP_CALIB && i != EXP_FOLLOW) gSelected = i;   // calibration and follow keep showing the chosen experiment
  showSelection(true);
  net::logf("M,run %s: %s starting", gRunId.c_str(), EXP_NAMES[i]);
  bool logged = i != 0;   // the stream experiment is its own log
  unsigned long t0 = millis();
  // follow: the robot stands on its start spot (which may face a board closer than
  // safe_front_cm) until steps arrive; each step sets its own front check (follow.h).
  // calib: it may be put down facing a board; only its rolls check ahead (runCalib).
  // dock / touch: put down facing the dock or board, which may be closer than safe_front_cm.
  // find / e02: it may stand facing a board it just backed away from; their drives set their own front check.
  if (i == EXP_FOLLOW || i == EXP_CALIB || i == EXP_DOCK || i == EXP_TOUCH || i == EXP_POSTS || i == EXP_FIND ||
      i == EXP_E02) gFrontLimitCm = 0;
  sw::configure();   // nose switches (switches.h): picks up `set sw_on 1` and pin changes
  if (!logged) edge::begin();
  if (logged) {
    logRunInfo(i);
    edge::begin();   // after the ~0.3 s of PARAM records, so it learns the plywood during the settle pause
    rawlog::start();
    waitMs((unsigned long)P("settle_start_ms"));
    syncFlash("start");
    showSelection(true);
    blink::start();   // after the start flash, so the two can't be confused
    lights::start();  // light pattern (lights_mode; off by default)
    if (!gAbortReason.length()) startNudge(i);
  }
  // A stop during the settle pause skips the experiment itself.
  if (!gAbortReason.length()) switch (i) {
    case 0: runStream(); break;
    case 1: runTurn(); break;
    case 2: runLap(); break;
    case 3: runGrid(); break;
    case 4: runDemo(); break;
    case 5: runStraight(); break;
    case EXP_SURVEY: runSurvey(); break;
    case EXP_FOLLOW: follow::run(); break;
    case EXP_CALIB: runCalib(); break;
    case EXP_DOCK: runDock(); break;
    case EXP_TOUCH: runTouch(); break;
    case EXP_POSTS: runPosts(); break;
    case EXP_GYRO: runGyro(); break;
    case EXP_FIND: runFind(); break;
    case EXP_E02: runE02(); break;
  }
  alvik.brake();
  if (i == EXP_DOCK || i == EXP_TOUCH || i == EXP_POSTS || i == EXP_FIND || i == EXP_E02) gFrontLimitCm = -1;   // they ran with the front stop off
  bool edgeStop = !strcmp(gAbortReason.c_str(), "plywood edge ahead");
  if (logged) tickWait((unsigned long)P("settle_end_ms"));   // at rest, still logging and blinking
  edge::stop();
  blink::stop();
  lights::stop();
  if (logged) {
    syncFlash("end");   // runs to full length even after a stop (no safety checks inside)
    rawlog::stop();
    // The edge guard refusing to drive off the work area is not a failure: "limited:plywood edge"
    // (the tools leave such runs out of accuracy statistics and count them separately).
    String result = edgeStop             ? String("limited:plywood edge")
                  : gAbortReason.length() ? String("stopped:") + gAbortReason
                  : gFail.length()        ? String("failed:") + gFail : String("done");
    net::sendLine("H,RUNEND,run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent");
    net::logf("D,RUNEND,%s,%s,%lu,%lu,%d,%d,%lu,%lu", gRunId.c_str(),
              result.c_str(),
              millis(), millis() - t0, alvik.get_battery_charge(), (int)WiFi.RSSI(), rawlog::rows,
              (unsigned long)net::seq);
  }
  if (edgeStop) net::logf("M,run %s: limited: the edge guard stopped it before an open plywood edge (not a failure; "
                          "left out of accuracy statistics)", gRunId.c_str());
  else if (gAbortReason.length()) net::logf("M,run %s: stopped (%s)", gRunId.c_str(), gAbortReason.c_str());
  else                       net::logf("M,run %s: %s finished", gRunId.c_str(), EXP_NAMES[i]);
  gAbortReason = "";
  // Wait for the cancel button to be released so it doesn't stop the next run.
  while (alvik.get_touch_cancel()) delay(20);
  showSelection(false);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) { delay(10); }

  if (Serial) Serial.println("Starting... if nothing happens, switch the robot's power switch ON");
  alvik.begin();
  gBootId = (uint16_t)(esp_random() & 0xFFFF);
  alvik.left_led.set_color(1, 1, 0);
  alvik.right_led.set_color(1, 1, 0);

  sw::configure();   // nose switches: nothing while sw_on is 0 (the default)
  net::begin();
  if (net::up) ota::begin();
  if (!alvik.check_firmware_compatibility()) {
    // begin() skips part of its setup on a mismatch, including the color
    // sensor's light, so turn that on ourselves and warn loudly.
    alvik.set_illuminator(true);
    net::sendLine("M,WARNING: robot firmware doesn't match the Alvik library; run the library's firmware updater example. Color readings may be off.");
  }
  net::logf("M,home_lab %s ready, battery %d%%", FW_VERSION, alvik.get_battery_charge());
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && gSleepMin > 0) {
    // woken by its sleep timer: a check-in (the laptop may send `wake`; else back to sleep)
    gCheckinUntil = millis() + (unsigned long)(P("sleep_awake_s") * 1000);
    net::logf("M,%s: awake for a check-in (sleeping %lu min at a time; send `wake` within %d s to keep it awake), "
              "battery %d%%", net::name, (unsigned long)gSleepMin, (int)P("sleep_awake_s"), alvik.get_battery_charge());
  }
  corr::load();       // this robot's saved corrections (gyro_scale, gyro_drift_dps, wheel_scale)
  printHelp();
  showSelection(false);
  lights::begin();    // lights_idle 1: pattern on from power-on
}

// Called by the WiFi updater (ota.h) at the start and end of an update.
void otaStarting() {
  alvik.brake();
  alvik.left_led.set_color(0, 0, 1);
  alvik.right_led.set_color(0, 0, 1);   // both blue while updating
}
void otaFinished(bool ok) {
  alvik.left_led.set_color(ok ? 0 : 1, ok ? 1 : 0, 0);   // green = done (restarting), red = failed
  alvik.right_led.set_color(ok ? 0 : 1, ok ? 1 : 0, 0);
}

void loop() {
  if (gCheckinUntil && (long)(millis() - gCheckinUntil) >= 0) { gCheckinUntil = 0; goToSleep(gSleepMin); }
  if (gLedUntil && (long)(millis() - gLedUntil) >= 0) { gLedUntil = 0; showSelection(false); }   // `led` ends
  yawTrack();         // keeps the corrected heading's turn count while idle (motion.h)
  net::keepTrying();  // joins WiFi later if it failed at startup
  if (net::up && !ota::tried) ota::begin();
  ota::handle();      // WiFi firmware updates, only while idle
  net::heartbeat();
  String cmd;
  if (net::poll(cmd)) handleCommand(cmd, false);

  e02::idleTick();    // an idle go / turn from the laptop (e02.h)

  if (gPending >= 0) {
    int i = gPending;
    gPending = -1;
    runExperiment(i, false);
    return;
  }

  // Buttons: LEFT/RIGHT choose, OK starts.
  static unsigned long lastPress = 0;
  if (millis() - lastPress > 300) {
    if (alvik.get_touch_left())  { gSelected = (gSelected + N_EXP - 1) % N_EXP; lastPress = millis(); showSelection(false); }
    if (alvik.get_touch_right()) { gSelected = (gSelected + 1) % N_EXP;         lastPress = millis(); showSelection(false); }
    if (alvik.get_touch_ok())    {
      lastPress = millis();
      while (alvik.get_touch_ok()) delay(20);   // start after your finger is off
      delay(500);
      runExperiment(gSelected, true);
    }
  }
  delay(10);
}
