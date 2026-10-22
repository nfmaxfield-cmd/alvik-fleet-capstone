#pragma once
#include "WiFi.h"
struct WiFiUDP { int begin(uint16_t){return 1;} int beginPacket(IPAddress,uint16_t){return 1;} int endPacket(){return 1;}
 int printf(const char*,...) __attribute__((format(printf,2,3))) {return 0;} int parsePacket(){return 0;} IPAddress remoteIP(){return {};} int read(char*,size_t){return 0;} };
