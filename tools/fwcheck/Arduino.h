#pragma once
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352
#define CHANGE 0x03
#define FALLING 0x02
#define RISING 0x01
#define SERIAL 0x0
#define DISPLAY 0x1
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <cctype>
#include <strings.h>
unsigned long millis(); void delay(unsigned long); uint32_t esp_random();
#define RTC_DATA_ATTR
enum { ESP_SLEEP_WAKEUP_UNDEFINED = 0, ESP_SLEEP_WAKEUP_TIMER = 4 };
int esp_sleep_get_wakeup_cause(); int esp_sleep_enable_timer_wakeup(uint64_t us); void esp_deep_sleep_start();
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
class String { std::string s; public:
 String(const char* c=""):s(c){} String(const std::string& x):s(x){}
 String(float f,int d){char b[32];snprintf(b,32,"%.*f",d,f);s=b;} String(int i){s=std::to_string(i);}
 String& operator+=(const char*c){s+=c;return *this;} String& operator+=(const String&o){s+=o.s;return *this;}
 unsigned length() const {return s.size();} void trim(){}
 void toUpperCase(){ for(auto&c:s)c=toupper(c);} void toLowerCase(){ for(auto&c:s)c=tolower(c);}
 int indexOf(char c) const {auto p=s.find(c);return p==std::string::npos?-1:(int)p;}
 String substring(unsigned a) const {return String(s.substr(a));} String substring(unsigned a,unsigned b) const {return String(s.substr(a,b-a));}
 char charAt(unsigned i) const {return s[i];} bool equalsIgnoreCase(const String&o) const {return strcasecmp(s.c_str(),o.s.c_str())==0;} bool operator==(const String&o) const {return s==o.s;} bool operator!=(const char*o) const {return s!=o;} int indexOf(const char*c) const {auto p=s.find(c);return p==std::string::npos?-1:(int)p;} bool startsWith(const char*c) const {return s.rfind(c,0)==0;} float toFloat() const {return strtof(s.c_str(),nullptr);} int toInt() const {return atoi(s.c_str());}
 const char* c_str() const {return s.c_str();} bool operator==(const char*o) const {return s==o;}
 friend String operator+(const String&a,const String&b){return String(a.s+b.s);} };
struct SerialT { operator bool() const {return true;} void begin(long){} void println(const char*){} void println(const String&){}
 int printf(const char*,...) __attribute__((format(printf,2,3))) {return 0;} };
extern SerialT Serial;
inline int digitalRead(int) { return 1; }
#ifndef OUTPUT
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define HIGH 0x1
#define LOW 0x0
#endif
inline void digitalWrite(int, int) {}
inline void pinMode(int, int) {}
// Nano ESP32 pins_arduino.h, "By Arduino pin (default)" numbering (espressif/arduino-esp32 variants/arduino_nano_nora)
#include <cstdint>
static constexpr uint8_t D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6, D13 = 13;
static constexpr uint8_t A0 = 17, A1 = 18, A2 = 19, A3 = 20, A4 = 21, A5 = 22, A6 = 23, A7 = 24;
