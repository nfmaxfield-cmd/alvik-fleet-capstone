// Firmware updates over WiFi (no USB cable).
//
// Off unless secrets.h defines OTA_PASSWORD, so nobody else on the network
// can reflash a robot. Once on, the Arduino IDE lists each robot as a
// network port (e.g. "Left at 192.168.1.250") under Tools > Port; pick it,
// click Upload, and type the password when asked. Updates are accepted only
// while the robot is idle: running experiments never call otaHandle().
#pragma once

#include <ArduinoOTA.h>
#include "secrets.h"
#include "net.h"

void otaStarting();          // defined in home_lab.ino: stop the motors, show it on the lights
void otaFinished(bool ok);

namespace ota {

bool on = false;
bool tried = false;

// Call once WiFi is up (at startup or later).
void begin() {
  tried = true;
#ifdef OTA_PASSWORD
  if (!net::up) return;
  ArduinoOTA.setHostname(net::name);     // shows as "<name>" in the IDE's network ports
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { otaStarting(); net::logf("M,%s: firmware update starting", net::name); });
  ArduinoOTA.onEnd([]() { otaFinished(true); });
  ArduinoOTA.onError([](ota_error_t e) {
    net::logf("M,%s: firmware update failed (error %d)", net::name, (int)e);
    otaFinished(false);
  });
  ArduinoOTA.begin();
  on = true;
  net::logf("M,%s: WiFi firmware updates on", net::name);
#else
  net::logf("M,%s: WiFi firmware updates off (add OTA_PASSWORD to secrets.h to turn them on)", net::name);
#endif
}

// Call often while idle.
void handle() {
  if (on) ArduinoOTA.handle();
}

}  // namespace ota
