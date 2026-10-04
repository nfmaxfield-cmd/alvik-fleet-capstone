// Copy this file to secrets.h (same folder) and fill in your home WiFi.
// secrets.h is git-ignored, so your password never reaches the public repo.
// The Alvik's ESP32 only joins 2.4 GHz networks.
#pragma once

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

// Optional. By default the robot broadcasts its logs to every computer on
// your network, so you don't need your laptop's IP. If your router blocks
// broadcast (logs never show up), put the laptop's IP here instead:
// #define LOG_HOST "192.168.1.50"
