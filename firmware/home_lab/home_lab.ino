/*
  home_lab — WiFi-logged experiments for one Alvik at home.

  Experiments:  stream (sensor log), turn (turn accuracy/time),
                lap (line-following lap timer), grid (stop vs drive-through)

  Setup:
    1. Copy secrets.example.h to secrets.h in this folder and add your WiFi.
    2. Upload. On your laptop run:  python tools/alvik_console.py
    3. In the console type:  help

  Without the laptop, use the buttons:
    LEFT / RIGHT  choose an experiment (left LED color shows which)
                  blue = stream, green = turn, yellow = lap, purple = grid
    OK            start it        CANCEL  stop it
  Results are logged over WiFi either way (and to Serial if USB is plugged in).
*/

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"
#include "motion.h"
#include "experiments.h"

Arduino_Alvik alvik;

const char* EXP_NAMES[] = {"stream", "turn", "lap", "grid", "demo"};
const int N_EXP = 5;
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
  }
  alvik.left_led.set_color(r, g, b);
  if (running) alvik.right_led.set_color(r, g, b);
  else         alvik.right_led.set_color(0, 0, 0);
}

void printHelp() {
  net::sendLine("M,commands: run stream|turn|lap|grid|demo  stop  name <Left|Center|Right>  set <name> <value>  get <name>  params  route <SLRE...>  status  identify  help");
}

void printParams() {
  for (int i = 0; i < N_PARAMS; i++) {
    net::logf("M,%-18s %8.2f  %s", PARAMS[i].name, PARAMS[i].value, PARAMS[i].help);
    delay(3);   // a burst of back-to-back UDP sends can drop packets
  }
  net::logf("M,route %s  (grid test: S straight, L left, R right, E end)", gRoute.c_str());
}

int expIndex(const String& name) {
  for (int i = 0; i < N_EXP; i++) if (name.equalsIgnoreCase(EXP_NAMES[i])) return i;
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
    // Flash both LEDs white so you can tell which physical robot this is.
    for (int i = 0; i < 6; i++) {
      alvik.left_led.set_color(1, 1, 1); alvik.right_led.set_color(1, 1, 1); delay(250);
      alvik.left_led.set_color(0, 0, 0); alvik.right_led.set_color(0, 0, 0); delay(250);
    }
    showSelection(running);
    net::logf("M,%s: that was me", net::name);
    return false;
  }
  if (verb == "params") { printParams(); return false; }
  if (verb == "ping" || verb == "status") {
    net::logf("M,status %s: %s, selected %s, battery %d%%, WiFi %d dBm", net::name,
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
    if (i < 0) { net::sendLine("M,run what? stream, turn, lap, grid or demo"); return false; }
    gSelected = i;
    gPending = i;
    return false;
  }
  net::logf("M,unknown command '%s' (type help)", cmd.c_str());
  return false;
}

uint16_t gBootId = 0;

void runExperiment(int i) {
  gRun++;
  char id[16];
  snprintf(id, sizeof(id), "%04X-%d", gBootId, gRun);
  gRunId = id;
  gAbortReason = "";
  gSelected = i;
  showSelection(true);
  net::logf("M,run %s: %s starting", gRunId.c_str(), EXP_NAMES[i]);
  switch (i) {
    case 0: runStream(); break;
    case 1: runTurn(); break;
    case 2: runLap(); break;
    case 3: runGrid(); break;
    case 4: runDemo(); break;
  }
  alvik.brake();
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
  if (!alvik.check_firmware_compatibility()) {
    // begin() skips part of its setup on a mismatch, including the color
    // sensor's light, so turn that on ourselves and warn loudly.
    alvik.set_illuminator(true);
    net::sendLine("M,WARNING: robot firmware doesn't match the Alvik library; run the library's firmware updater example. Color readings may be off.");
  }
  net::logf("M,home_lab ready, battery %d%%", alvik.get_battery_charge());
  printHelp();
  showSelection(false);
}

void loop() {
  net::heartbeat();
  String cmd;
  if (net::poll(cmd)) handleCommand(cmd, false);

  if (gPending >= 0) {
    int i = gPending;
    gPending = -1;
    runExperiment(i);
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
      runExperiment(gSelected);
    }
  }
  delay(10);
}
