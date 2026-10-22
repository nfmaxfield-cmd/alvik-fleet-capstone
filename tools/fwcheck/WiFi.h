#pragma once
#include "Arduino.h"
struct IPAddress { IPAddress(){} IPAddress(int,int,int,int){} bool fromString(const char*){return true;} String toString() const {return "";} };
#define WIFI_STA 1
#define WL_CONNECTED 3
struct WiFiT { void mode(int){} void begin(const char*,const char*){} void setSleep(bool){} bool disconnect(){return true;} String macAddress(){return "";} int status(){return 3;} IPAddress localIP(){return {};} long RSSI(){return 0;} };
extern WiFiT WiFi;
