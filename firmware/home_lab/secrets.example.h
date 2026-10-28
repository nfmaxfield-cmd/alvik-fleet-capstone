// Copy this file to secrets.h (same folder) and fill in your home WiFi.
// secrets.h is git-ignored, so your password never reaches the public repo.
// The Alvik's ESP32 only joins 2.4 GHz networks.
#pragma once

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

// Second network (firmware 2026-10-27a): the school's. The robot tries the first one, then this one, and keeps
// alternating until one answers, so one flash works at home and at school. The school's network has "ISE" IN its name; the full
// name is still to confirm (Nick, 2026-10-28 09:25: "ISE is in the name"). Put the exact name below. Type its password in place of the placeholder below, in YOUR secrets.h only (never in this
// file: it is in the public repo). Delete or comment out both lines to use only the first network.
#define WIFI_SSID2     "type-the-exact-school-network-name-here"
#define WIFI_PASSWORD2 "type-the-school-password-here"

// Optional. By default the robot broadcasts its logs to every computer on
// your network, so you don't need your laptop's IP. If your router blocks
// broadcast (logs never show up), put the laptop's IP here instead:
// #define LOG_HOST "192.168.1.50"

// Optional. Turns on firmware updates over WiFi (see ota.h): pick a password
// only you know. Without this line, updates need the USB cable as before.
// #define OTA_PASSWORD "choose-a-password"
