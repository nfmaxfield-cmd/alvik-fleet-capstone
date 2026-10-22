#pragma once
#include <functional>
typedef int ota_error_t;
struct ArduinoOTAClass { void setHostname(const char*){} void setPassword(const char*){} void onStart(std::function<void()>){} void onEnd(std::function<void()>){} void onError(std::function<void(ota_error_t)>){} void begin(){} void handle(){} };
static ArduinoOTAClass ArduinoOTA;
