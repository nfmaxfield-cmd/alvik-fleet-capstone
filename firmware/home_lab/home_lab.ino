/*
  home_lab — WiFi-logged experiments for one Alvik at home.

  Experiments:  stream (sensor log), turn (turn accuracy/time),
                lap (line-following lap timer), grid (stop vs drive-through),
                demo (open-floor routes), straight (straight-line accuracy),
                survey (square spiral for the camera's self-calibration),
                follow (runs the steps tools/fleet/run_plan.py sends; laptop only)

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
#include "params.h"
#include "net.h"
#include "motion.h"
#include "experiments.h"
#include "follow.h"
#include "ota.h"

Arduino_Alvik alvik;

// Bump this whenever the firmware changes, so every logged run says which code it ran.
#define FW_VERSION "2026-10-18b"

uint16_t gBootId = 0;   // random each power-on; starts every run id

// The buttons cycle through the first N_EXP. "follow" (index 7) needs the
// laptop, so it is started with `run follow` only; "calib" (index 8) is the
// calibration run, started with the calibrate command only.
const char* EXP_NAMES[] = {"stream", "turn", "lap", "grid", "demo", "straight", "survey", "follow", "calib"};
const int N_EXP = 7;
const int N_RUNNABLE = 8;   // `run <name>` accepts these
const int EXP_SURVEY = 6;
const int EXP_FOLLOW = 7;
const int EXP_CALIB = 8;
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
  net::sendLine("M,commands: run stream|turn|lap|grid|demo|straight|survey|follow  stop  calibrate  clock  name <Left|Center|Right>  set <name> <value>  get <name>  params  route <SLRE...>  status  identify  help");
  net::sendLine("M,during run follow: fs <id> <T|D|A|W> ...  fix <t_ms> <x> <y> <th> <sigma>  fpose <x> <y> <th>  hold [id]  halt  (tools/fleet/run_plan.py sends these)");
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    net::logf("M,%-18s %8.2f  %s", PARAMS[i].name, PARAMS[i].value, PARAMS[i].help);
    delay(3);   // a burst of back-to-back UDP sends can drop packets
  }
  net::logf("M,route %s  (grid test: S straight, L left, R right, E end)", gRoute.c_str());
}

int expIndex(const String& name) {
  for (int i = 0; i < N_RUNNABLE; i++) if (name.equalsIgnoreCase(EXP_NAMES[i])) return i;
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
  if (verb == "params") {
    if (running) { net::sendLine("M,busy: send stop first"); return false; }   // blocks ~135 ms
    printParams();
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
    Param* p = findParam(rest);
    if (p) net::logf("M,%s = %.3f  (%s)", p->name, p->value, p->help);
    else   net::logf("M,no parameter called %s (type params)", rest.c_str());
    return false;
  }
  if (verb == "set") {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) { net::sendLine("M,usage: set <name> <value>"); return false; }
    String name = rest.substring(0, sp2);
    String val = rest.substring(sp2 + 1);
    val.trim();
    Param* p = findParam(name);
    if (!p) { net::logf("M,no parameter called %s (type params)", name.c_str()); return false; }
    char* end = nullptr;
    float v = strtof(val.c_str(), &end);
    if (end == val.c_str()) { net::logf("M,'%s' is not a number", val.c_str()); return false; }
    p->value = v;
    net::logf("M,%s = %.3f", p->name, v);
    return false;
  }
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
    if (i < 0) { net::sendLine("M,run what? stream, turn, lap, grid, demo, straight, survey or follow"); return false; }
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
bool needsYawSign(int i) { return i == 1 || (i >= 3 && i <= 7); }

// Calibration run, once per power-on and whenever robots may have been mixed
// up. Logged like any run (RUNINFO, PARAM, RAW at raw_hz, flashes), so the
// camera recording holds it too; tools/blink/calib.py reads it from the video
// and the log. RAW step (what the camera / log measure in it):
//   1  still, blinking its code, calib_blink_ms       who is who
//   2  roll forward calib_roll_cm, still calib_still_ms   heading, wheel distance scale
//   3  roll back, still calib_still_ms                 the same, the other way
//   4  compass wiggle (calibrateYawSign(true))         gyro sign
//   5  still calib_still_ms                            gyro bias; picture before the spin
//   6  spin left calib_spin_deg at calib_spin_rpm      gyro scale, light offset from the
//                                                      turning centre, front sensor sweep
//   7  still calib_still_ms                            picture after the spin; gyro bias
//   8  spin back right calib_spin_deg                  the same, the other way
//   9  still calib_still_ms                            picture after; gyro bias
// calib_spin_deg 0 skips 6-8 (step 9 is then 300 ms). The blink code runs
// throughout (moves go dark before the library's blocking call, as elsewhere).
// Defaults: about 38 s from RUNINFO to RUNEND; clear floor: a 32 cm lane (10 cm
// robot + 15 cm roll + 7 cm safe_front_cm stop distance) and a 25 cm circle
// for the spin. With safe_edge on, the roll stops at an open plywood edge
// ("stopped:plywood edge ahead"; the spin is not guarded). Firmware before 2026-10-18b: 1 still, 2 roll
// max(3, start_nudge_cm) with 300 ms pauses, 3 back, 4 wiggle, 5 300 ms.
void runCalib() {
  float n = fmaxf(3.0f, P("calib_roll_cm"));
  float spin = P("calib_spin_deg");
  unsigned long still = (unsigned long)fmaxf(300.0f, P("calib_still_ms"));
  net::logf("M,%s calibration: blinking code %d, rolling %.0f cm forward and back, a small wiggle%s",
            net::name, blink::codeFor(net::name), n, spin != 0 ? ", then a slow spin each way" : "");
  rawlog::step = 1;
  if (!waitMs((unsigned long)P("calib_blink_ms"))) return;
  rawlog::step = 2;
  if (!libraryMove(n) || !waitMs(still)) return;
  rawlog::step = 3;
  if (!libraryMove(-n) || !waitMs(still)) return;
  rawlog::step = 4;
  if (!calibrateYawSign(true)) return;
  if (spin != 0) {
    rawlog::step = 5;
    if (!waitMs(still)) return;
    rawlog::step = 6;
    if (!spinBy(fabsf(spin), P("calib_spin_rpm"))) return;
    rawlog::step = 7;
    if (!waitMs(still)) return;
    rawlog::step = 8;
    if (!spinBy(-fabsf(spin), P("calib_spin_rpm"))) return;
  }
  rawlog::step = 9;
  if (!waitMs(spin != 0 ? still : 300)) return;
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
  // safe_front_cm) until steps arrive; each step sets its own front check (follow.h)
  if (i == EXP_FOLLOW) gFrontLimitCm = 0;
  edge::begin();   // the edge guard learns the plywood under the line sensors during the settle pause
  if (logged) {
    logRunInfo(i);
    rawlog::start();
    waitMs((unsigned long)P("settle_start_ms"));
    syncFlash("start");
    showSelection(true);
    blink::start();   // after the start flash, so the two can't be confused
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
  }
  alvik.brake();
  edge::stop();
  blink::stop();
  if (logged) {
    syncFlash("end");   // runs to full length even after a stop (no safety checks inside)
    rawlog::stop();
    String result = gAbortReason.length() ? String("stopped:") + gAbortReason
                  : gFail.length()        ? String("failed:") + gFail : String("done");
    net::sendLine("H,RUNEND,run,result,ms_end,duration_ms,battery,wifi_dbm,raw_rows_sent,packets_sent");
    net::logf("D,RUNEND,%s,%s,%lu,%lu,%d,%d,%lu,%lu", gRunId.c_str(),
              result.c_str(),
              millis(), millis() - t0, alvik.get_battery_charge(), (int)WiFi.RSSI(), rawlog::rows,
              (unsigned long)net::seq);
  }
  if (gAbortReason.length()) net::logf("M,run %s: stopped (%s)", gRunId.c_str(), gAbortReason.c_str());
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

  net::begin();
  if (net::up) ota::begin();
  if (!alvik.check_firmware_compatibility()) {
    // begin() skips part of its setup on a mismatch, including the color
    // sensor's light, so turn that on ourselves and warn loudly.
    alvik.set_illuminator(true);
    net::sendLine("M,WARNING: robot firmware doesn't match the Alvik library; run the library's firmware updater example. Color readings may be off.");
  }
  net::logf("M,home_lab %s ready, battery %d%%", FW_VERSION, alvik.get_battery_charge());
  printHelp();
  showSelection(false);
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
  net::keepTrying();  // joins WiFi later if it failed at startup
  if (net::up && !ota::tried) ota::begin();
  ota::handle();      // WiFi firmware updates, only while idle
  net::heartbeat();
  String cmd;
  if (net::poll(cmd)) handleCommand(cmd, false);

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
