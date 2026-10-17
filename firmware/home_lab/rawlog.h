// Full sensor log for the fusion data set.
//
// While an experiment runs, every sensor reading goes to the laptop as a RAW
// record raw_hz times a second (default 50). Each record carries the robot's
// own clock (ms), so the laptop's clock-sync records and the light flashes
// can line it up with the camera video afterwards. Logging rides on
// shouldAbort(), which every motion loop already calls every 5-10 ms.
// IMU columns (ax..gz) are in the units the motor board reports; check with a
// still, flat robot: az should read about 1 if accelerations are in g.
#pragma once

#include "Arduino_Alvik.h"
#include "params.h"
#include "net.h"

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
                "dL_cm,dCL_cm,dC_cm,dCR_cm,dR_cm,dtop_cm,dbot_cm,lineL,lineC,lineR,battery,charging");
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
            "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d",
            gRunId.c_str(), now, step, wl, wr, sl, sr, v, w, x, y, th,
            ax, ay, az, gx, gy, gz, roll, pitch, yaw,
            dL, dCL, dC, dCR, dR, alvik.get_distance_top(), alvik.get_distance_bottom(), lL, lC, lR,
            alvik.get_battery_charge(), alvik.is_battery_charging() ? 1 : 0);
  rows++;
}

}  // namespace rawlog
