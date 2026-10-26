// WiFi logging and commands over UDP.
//
// Robot -> laptop (port 5005): one text line per packet, "name|seq|line".
//   name is this robot's id, made from the last 4 hex digits of its WiFi MAC
//   (e.g. AL-3F2A), so the same sketch runs on every robot unchanged.
//   seq lets the laptop count dropped packets per robot. Lines are:
//     H,TYPE,col,col,...   column names for a record type
//     D,TYPE,val,val,...   one data row
//     M,text               a message to show on screen
// Laptop -> robot (port 5006): a plain command such as "run turn", or a NUMBERED one (firmware 2026-10-26a):
//   "#<n> <command>"   n: the laptop's command number (1..2^31, never 0). Every numbered command is answered at
//   once, before it runs, with "D,ACK,<n>,<new|dup>,<ms>" (header H,ACK,n,result,ms). A number seen among the last
//   ACK_KEEP is answered "dup" and NOT run again: the laptop resends a command whose answer did not arrive within
//   about 1 s (2026-10-26 runner fixes: the 2025-10-25 21:14 link lost 1 command in 10, settings included), so a
//   resend of one that did arrive is harmless whatever the command (go, turn, set, fix, run, calibrate, stop ...).
//   Plain (unnumbered) commands work as before (the console, clock pings).
//
// Until the laptop sends something, the robot broadcasts to the whole
// network. After the first command it replies straight to that laptop.
#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>
#include <Preferences.h>
#include "secrets.h"

#ifndef LOG_PORT
#define LOG_PORT 5005
#endif
#ifndef CMD_PORT
#define CMD_PORT 5006
#endif

namespace net {

WiFiUDP udp;
IPAddress dest(255, 255, 255, 255);
IPAddress laptop;
bool haveLaptop = false;
bool up = false;
uint32_t seq = 0;
unsigned long lastRx = 0;      // millis() when the last command arrived (follow: is the laptop still there?)
unsigned long lastBegin = 0;   // millis() of the last WiFi.begin(): keepTrying() waits 20 s after it
char name[12] = "AL-????";

// Robot id: a saved name (e.g. "Left", set once with the console's
// `name` command and kept in flash across reboots), otherwise one made
// from the MAC address. Valid once WiFi.mode() has been called.
void makeName() {
  Preferences prefs;
  prefs.begin("homelab", true);
  String saved = prefs.getString("name", "");
  prefs.end();
  if (saved.length()) {
    snprintf(name, sizeof(name), "%s", saved.c_str());
    return;
  }
  String mac = WiFi.macAddress();   // "AA:BB:CC:DD:EE:FF"
  if (mac.length() >= 17) {
    snprintf(name, sizeof(name), "AL-%c%c%c%c", mac.charAt(12), mac.charAt(13), mac.charAt(15), mac.charAt(16));
  }
}

// Save a new name ("" clears it back to the MAC-based one). Max 10 chars.
void saveName(const String& n) {
  Preferences prefs;
  prefs.begin("homelab", false);
  prefs.putString("name", n);
  prefs.end();
  makeName();
}

void sendLine(const char* line) {
  if (Serial) Serial.println(line);
  if (!up) return;
  udp.beginPacket(haveLaptop ? laptop : dest, LOG_PORT);
  udp.printf("%s|%lu|%s", name, (unsigned long)seq++, line);
  udp.endPacket();
}

__attribute__((format(printf, 1, 2)))
void logf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  sendLine(buf);
}

void finishUp();

// Join WiFi. Returns false (and the lab still runs over USB) if it can't.
bool begin(unsigned long timeout_ms = 15000) {
  WiFi.mode(WIFI_STA);
  makeName();
  if (Serial) Serial.printf("This robot is %s (MAC %s)\n", name, WiFi.macAddress().c_str());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastBegin = millis();   // so keepTrying()'s first retry doesn't cut off a slow join right away
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (Serial) Serial.println("M,WiFi did not connect yet; logging over USB, and still trying in the background");
    return false;
  }
  finishUp();
  return true;
}

// Called once WiFi is connected, at startup or later.
void finishUp() {
#ifdef LOG_HOST
  dest.fromString(LOG_HOST);
#endif
  WiFi.setSleep(false);   // power-save mode delays incoming commands (e.g. stop) by 100-300 ms
  udp.begin(CMD_PORT);
  up = true;
  logf("M,%s joined WiFi as %s (signal %d dBm)", name, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
}

// If WiFi didn't connect at startup, keep trying (call often while idle):
// a slow router or a weak signal at power-on no longer leaves the robot
// offline until it is restarted.
void keepTrying() {
  static unsigned long lastCheck = 0;
  if (up || millis() - lastCheck < 1000) return;
  lastCheck = millis();
  if (WiFi.status() == WL_CONNECTED) { finishUp(); return; }
  if (millis() - lastBegin > 20000) {
    lastBegin = millis();
    if (Serial) Serial.println("M,still trying to join WiFi...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// Until a laptop has talked to us, announce ourselves every 2 s so a console
// started after the robot still finds it.
void heartbeat() {
  static unsigned long last = 0;
  if (!up || haveLaptop || millis() - last < 2000) return;
  last = millis();
  logf("M,hello from %s at %s (type help in the console)", name, WiFi.localIP().toString().c_str());
}

// Numbered commands (see the top): the last ACK_KEEP numbers seen. A resend comes within seconds; the laptop sends
// at most ~20 commands a second (camera fixes 3 a second per robot, legs, settings), so 64 covers 3 s or more.
// Chosen, no DOE: a buffer size well above any resend window (the runner resends for at most ~8 s; a number older
// than that is never resent). The laptop starts its numbers from its clock, so a restarted laptop never reuses one.
const int ACK_KEEP = 64;
uint32_t ackSeen[ACK_KEEP];
int ackPos = 0;
uint32_t ackCount = 0;

bool ackKnown(uint32_t n) {
  for (int i = 0; i < ACK_KEEP; i++) if (ackSeen[i] == n) return true;
  return false;
}

// Returns true and fills `out` when a command has arrived (a numbered one: answered, and a repeat dropped).
bool poll(String& out) {
  if (!up) return false;
  int n = udp.parsePacket();
  if (n <= 0) return false;
  laptop = udp.remoteIP();
  haveLaptop = true;
  lastRx = millis();
  char buf[192];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len > 0 ? len : 0] = '\0';
  char* cmd = buf;
  while (*cmd == ' ') cmd++;
  if (*cmd == '#') {
    char* e = nullptr;
    uint32_t num = strtoul(cmd + 1, &e, 10);
    if (e == cmd + 1 || num == 0) return false;            // "#" with no number: not a command
    bool dup = ackKnown(num);
    if (ackCount % 50 == 0) sendLine("H,ACK,n,result,ms");  // the header now and then (the laptop also knows it)
    ackCount++;
    char a[48];
    snprintf(a, sizeof(a), "D,ACK,%lu,%s,%lu", (unsigned long)num, dup ? "dup" : "new", millis());
    sendLine(a);
    if (dup) return false;                                   // already had it: answered, not run again
    ackSeen[ackPos] = num;
    ackPos = (ackPos + 1) % ACK_KEEP;
    cmd = e;
  }
  out = String(cmd);
  out.trim();
  return out.length() > 0;
}

}  // namespace net
