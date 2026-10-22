#pragma once
#include "Arduino.h"
#define DEG 0
#define RPM 0
#define CM 0
#define CM_S 0
#define DEG_S 1
#define RGB 0
#define HSV 1
#define RUN_ON_CORE_0 0
#define WHITE_ID 3
struct Led { void set_color(const bool,const bool,const bool){} };
class Arduino_Alvik { public: Led left_led, right_led;
  int begin(const bool verbose = true, const uint8_t core = RUN_ON_CORE_0) {return 0;}
  void stop() {}
  bool is_on() {return 0;}
  void idle() {}
  uint8_t get_ack() {return 0;}
  void get_wheels_speed(float & left, float & righ, const uint8_t unit = RPM) {}
  void set_wheels_speed(const float left, const float right, const uint8_t unit = RPM) {}
  void get_wheels_position(float & left, float & right, const uint8_t unit = DEG) {}
  void set_wheels_position(const float left, const float right, const uint8_t unit = DEG, const bool blocking = true) {}
  void get_drive_speed(float & linear, float & angular, const uint8_t linear_unit = CM_S, const uint8_t angular_unit = DEG_S) {}
  void drive(const float linear, const float angular, const uint8_t linear_unit = CM_S, const uint8_t angular_unit = DEG_S) {}
  void get_pose(float & x, float & y, float & theta, const uint8_t distance_unit = CM, const uint8_t angle_unit = DEG) {}
  void reset_pose(const float x = 0.0, const float y = 0.0, const float theta = 0.0, const uint8_t distance_unit = CM, const uint8_t angle_unit = DEG) {}
  bool is_target_reached() {return 0;}
  void rotate(const float angle, const uint8_t unit = DEG, const bool blocking = true) {}
  void move(const float distance, const uint8_t unit = CM, const bool blocking = true) {}
  void brake() {}
  void get_line_sensors(int & left, int & center, int & right) {}
  void get_color_raw(int & red, int & green, int & blue) {}
  void rgb2norm(const int16_t r, const int16_t g, const int16_t b, float & r_norm, float & g_norm, float & b_norm) {}
  void norm2hsv(const float r, const float g, const float b, float & h, float & s, float & v) {}
  void get_color(float & value0, float & value1, float & value2, const uint8_t format = RGB) {}
  uint8_t get_color_id(const float h, const float s, const float v) {return 0;}
  uint8_t get_color_id() {return 0;}
  String get_color_label(const float h, const float s, const float v) {return String();}
  String get_color_label() {return String();}
  void color_calibration(const uint8_t background = WHITE_ID) {}
  void get_orientation(float & roll, float & pitch, float & yaw) {}
  void get_accelerations(float & x, float & y, float & z) {}
  void get_gyros(float & x, float & y, float & z) {}
  void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz) {}
  bool get_shake() {return 0;}
  bool get_lifted() {return 0;}
  String get_tilt() {return String();}
  void get_distance(float & left, float & center_left, float & center, float & center_right, float & right, const uint8_t unit = CM) {}
  float get_distance_top(const uint8_t unit = CM) {return 0;}
  float get_distance_bottom(const uint8_t unit = CM) {return 0;}
  bool get_touch_any() {return 0;}
  bool get_touch_ok() {return 0;}
  bool get_touch_cancel() {return 0;}
  bool get_touch_center() {return 0;}
  bool get_touch_up() {return 0;}
  bool get_touch_left() {return 0;}
  bool get_touch_down() {return 0;}
  bool get_touch_right() {return 0;}
  void set_builtin_led(const bool value) {}
  void set_illuminator(const bool value) {}
  void set_servo_positions(const uint8_t a_position, const uint8_t b_position) {}
  void get_servo_positions(int & a_position, int & b_position) {}
  void set_behaviour(const uint8_t behaviour) {}
  void get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower, const String version="fw") {}
  void get_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower) {}
  void get_lib_version(uint8_t & upper, uint8_t & middle, uint8_t & lower) {}
  void get_required_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower) {}
  bool check_firmware_compatibility() {return 0;}
  int get_battery_charge() {return 0;}
  bool is_battery_charging() {return 0;}
};