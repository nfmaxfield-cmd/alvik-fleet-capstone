/*
  sensor_explorer — print everything the Alvik senses, 5 times a second.

  Use it at your desk with the USB cable plugged in:
    1. Upload, then open Serial Monitor at 115200 baud.
    2. Slide the robot over the floor, black tape, a tape crossing and each
       colored sticker. Write down what the line sensors and color read.
    3. Those numbers set the thresholds in home_lab/params.h
       (lf_line_thr, the marker color window).

  Nothing moves: the wheels are never commanded.
  Columns:
    L C R        line sensors (higher = darker; black tape reads high)
    H S V label  color sensor under the nose (H in degrees 0-360)
    dist         front distance zones left..right, then top and bottom, in cm
    roll pitch yaw  IMU orientation in degrees
    batt         battery percent
*/

#include "Arduino_Alvik.h"

Arduino_Alvik alvik;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }

  Serial.println("Starting... if nothing happens, switch the robot's power switch ON");
  alvik.begin();

  uint8_t a, b, c;
  alvik.get_fw_version(a, b, c);
  Serial.printf("Alvik firmware %u.%u.%u", a, b, c);
  alvik.get_required_fw_version(a, b, c);
  Serial.printf("  (library wants %u.%u.%u)%s\n", a, b, c,
                alvik.check_firmware_compatibility() ? "" : "  <-- MISMATCH: run the library's firmware updater example");
  // On a mismatch begin() skips turning on the color sensor's light.
  alvik.set_illuminator(true);

  alvik.left_led.set_color(0, 0, 1);
  alvik.right_led.set_color(0, 0, 1);
  Serial.println("   L    C    R |    H     S     V  label        | dL   dCL  dC   dCR  dR   top  bot  | roll  pitch   yaw  | batt");
}

void loop() {
  int l, c, r;
  alvik.get_line_sensors(l, c, r);

  float h, s, v;
  alvik.get_color(h, s, v, HSV);
  String label = alvik.get_color_label(h, s, v);

  float dL, dCL, dC, dCR, dR;
  alvik.get_distance(dL, dCL, dC, dCR, dR);
  float top = alvik.get_distance_top();
  float bot = alvik.get_distance_bottom();

  float roll, pitch, yaw;
  alvik.get_orientation(roll, pitch, yaw);

  Serial.printf("%4d %4d %4d | %5.1f %5.2f %5.2f  %-12s | %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f | %5.1f %6.1f %6.1f | %3d%%\n",
                l, c, r, h, s, v, label.c_str(),
                dL, dCL, dC, dCR, dR, top, bot,
                roll, pitch, yaw, alvik.get_battery_charge());

  // Left LED mirrors "is the center sensor on dark tape?" using a rough guess
  // of 300; your own readings decide the real threshold.
  alvik.left_led.set_color(0, c > 300, c <= 300);
  delay(200);
}
