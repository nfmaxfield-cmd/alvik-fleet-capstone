// Full sensor log for the fusion data set.
//
// While an experiment runs, every sensor reading goes to the laptop as a RAW
// record raw_hz times a second (default 50). Each record carries the robot's
// own clock (ms), so the laptop's clock-sync records and the light flashes
// can line it up with the camera video afterwards. Logging rides on
// shouldAbort(), which every motion loop already calls every 5-10 ms.
// swL, swR (firmware 2026-10-19a+, just before raw_n): the nose bumper's left
// and right switch, debounced: 1 pressed, 0 free, -1 switches not configured
// (sw_on 0; switches.h). raw_n stays the LAST column; every tool reads RAW by
// column name, so older files without swL/swR still load.
// raw_n (firmware 2026-10-18b+, last column): this row's number in the run,
// 0, 1, 2, ... with no gaps on the robot's side, so on the laptop a jump in
// raw_n is a lost row and a gap in ms with consecutive raw_n is a real pause
// (e.g. the ~200 ms the library blocks in move()).
// IMU columns (ax..gz) are in the units the motor board reports; check with a
// still, flat robot: az should read about 1 if accelerations are in g.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"
#include "blink.h"
#include "switches.h"
#include "lights.h"

extern Arduino_Alvik alvik;
extern String gRunId;

namespace rawlog {

bool on = false;
unsigned long next = 0, period = 20;
unsigned long rows = 0;     // RAW records sent this run (the laptop compares with what arrived)
int step = 0;               // route steps completed so far; matches the DEMO_STEP records

void header() {
  net::sendLine("H,RAW,run,ms,step,wl_deg,wr_deg,wl_rpm,wr_rpm,v_cms,w_dps,x_cm,y_cm,th_deg,"
                "ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,"
                "dL_cm,dCL_cm,dC_cm,dCR_cm,dR_cm,dtop_cm,dbot_cm,lineL,lineC,lineR,battery,charging,lifted,swL,swR,raw_n");
}

void start() {
  float hz = P("raw_hz");
  rows = 0;
  step = 0;
  on = hz > 0;
  if (!on) return;
  period = (unsigned long)(1000.0f / fminf(hz, 100.0f));
  header();
  next = millis();
}

void stop() { on = false; }

// Call as often as you like; sends at most one record per period.
void tick() {
  blink::tick();   // blink codes ride on the same frequent calls
  sw::tick();      // so do the nose switches (switches.h; nothing while sw_on is 0)
  lights::tick();  // and so does the light pattern's side-LED cycle
  if (!on) return;
  unsigned long now = millis();
  if ((long)(now - next) < 0) return;
  next += period;
  if ((long)(now - next) > (long)period) next = now + period;   // fell behind: don't burst to catch up

  float wl, wr, sl, sr, v, w, x, y, th, ax, ay, az, gx, gy, gz, roll, pitch, yaw;
  float dL, dCL, dC, dCR, dR;
  int lL, lC, lR;
  alvik.get_wheels_position(wl, wr);
  alvik.get_wheels_speed(sl, sr);
  alvik.get_drive_speed(v, w);
  alvik.get_pose(x, y, th);
  alvik.get_imu(ax, ay, az, gx, gy, gz);
  alvik.get_orientation(roll, pitch, yaw);
  alvik.get_distance(dL, dCL, dC, dCR, dR);
  alvik.get_line_sensors(lL, lC, lR);
  net::logf("D,RAW,%s,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
            "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%lu",
            gRunId.c_str(), now, step, wl, wr, sl, sr, v, w, x, y, th,
            ax, ay, az, gx, gy, gz, roll, pitch, yaw,
            dL, dCL, dC, dCR, dR, alvik.get_distance_top(), alvik.get_distance_bottom(), lL, lC, lR,
            alvik.get_battery_charge(), alvik.is_battery_charging() ? 1 : 0,
            alvik.get_lifted() ? 1 : 0,    // picked up: position must be re-found ("kidnapped robot")
            sw::col(0), sw::col(1),        // nose switches: 1 pressed, 0 free, -1 not configured
            rows);                         // raw_n: rows sent before this one
  rows++;
}

}  // namespace rawlog
