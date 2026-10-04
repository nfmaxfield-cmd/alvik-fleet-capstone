// WiFi logging and commands over UDP.
//
// Robot -> laptop (port 5005): one text line per packet, "seq|line".
//   seq lets the laptop count dropped packets. Lines are:
//     H,TYPE,col,col,...   column names for a record type
//     D,TYPE,val,val,...   one data row
//     M,text               a message to show on screen
// Laptop -> robot (port 5006): a plain command such as "run turn".
//
// Until the laptop sends something, the robot broadcasts to the whole
// network. After the first command it replies straight to that laptop.
#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>
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

void sendLine(const char* line) {
  if (Serial) Serial.println(line);
  if (!up) return;
  udp.beginPacket(haveLaptop ? laptop : dest, LOG_PORT);
  udp.printf("%lu|%s", (unsigned long)seq++, line);
  udp.endPacket();
}

void logf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  sendLine(buf);
}

// Join WiFi. Returns false (and the lab still runs over USB) if it can't.
bool begin(unsigned long timeout_ms = 15000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    if (Serial) Serial.println("M,WiFi did not connect; logging over USB only");
    return false;
  }
#ifdef LOG_HOST
  dest.fromString(LOG_HOST);
#endif
  WiFi.setSleep(false);   // power-save mode delays incoming commands (e.g. stop) by 100-300 ms
  udp.begin(CMD_PORT);
  up = true;
  logf("M,joined WiFi as %s (signal %d dBm)", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  return true;
}

// Until a laptop has talked to us, announce ourselves every 2 s so a console
// started after the robot still finds it.
void heartbeat() {
  static unsigned long last = 0;
  if (!up || haveLaptop || millis() - last < 2000) return;
  last = millis();
  logf("M,hello from %s (type help in the console)", WiFi.localIP().toString().c_str());
}

// Returns true and fills `out` when a command has arrived.
bool poll(String& out) {
  if (!up) return false;
  int n = udp.parsePacket();
  if (n <= 0) return false;
  laptop = udp.remoteIP();
  haveLaptop = true;
  char buf[160];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len > 0 ? len : 0] = '\0';
  out = String(buf);
  out.trim();
  return out.length() > 0;
}

}  // namespace net
