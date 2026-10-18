# Blink codes: who is who, and which way they face (tools/blink)

Two programs for the overhead recording:

- **blink.py** finds the robots' blinking top lights in the video, follows
  each one, and reads its blink code to say which robot it is.
- **calib.py** uses that, the robots' own logs and the video to work out,
  for every calibration run and every start nudge: which robot it was,
  which way it faced, and where its lights sit on it; from a calibration
  run of firmware 2026-10-18b or later also how far its wheels really carry
  it (wheel distance scale), how its gyro's degrees compare with real ones
  (gyro scale), and the light's offset from its turning centre.

Needs Python 3.9+ with `numpy` and `opencv-python` (`scipy` is used if
installed). No internet connection needed.

## The blink code (firmware 2026-10-18a and later)

While a logged run is going, both top LEDs blink white: after 300 ms dark,
one 100 ms flash every (n + 2) x 100 ms. Left flashes every 0.3 s, Center
every 0.4 s, Right every 0.5 s, any other name every 0.6 s. A flash can be
stretched to about 215 ms or cut short or skipped (the lights go dark before
long blocking calls), and now and then its start is delayed by up to about
115 ms. The 300 ms sync flashes are not part of the code.

Identity is meant as one clue among several: the fusion also matches wheel
motion to camera tracks. So blink.py reports, for every flashing track, a
probability for each robot name (and for "not a robot code"), and only adds
a name when the code is clear.

## How to run

```
python tools/blink/blink.py recordings/20261018-101455_E1 --session logs/20261018-101500
python tools/blink/calib.py --session logs/20261018-101500 --recording recordings/20261018-101455_E1
python tools/blink/calib.py --session ... --recording ... --detections recordings/.../detections.csv
```

blink.py takes about 5 ms per frame (a 10-minute video: about 1.5 minutes).
calib.py runs blink.py itself if `blink_detections.csv` is missing. Without
`--detections` (robot centres from the detector, or the simulator's
`camera_detections.csv`), calib.py still finds identity and heading, but not
the LED offset.

Which code is which robot: the firmware picks a robot's code from its PARAM
`blink_code` (1..4), or, when that is 0 (the default), from its name, exactly
and case-sensitively: Left 1, Center 2, Right 3, any other name 4 (`left` or
`Bob` blink code 4). `blink.py --session <console log folder>` reads that from
the runs' PARAM records and warns when two robots blink the same code or a
name isn't exactly Left / Center / Right; calib.py does the same from its
`--session`. Without `--session`, blink.py assumes the default mapping.
tools/check_session.py also warns about a non-zero `blink_code` or a shared
code.

Options worth knowing: `--session` (blink.py), `--names 1=Left,2=Center,3=Right,4=other` (both
programs; overrides the session), `--interpolate` (blink.py: also rows between flashes), `--min-name-p` (blink.py),
`--vmax-cms` (blink.py: fastest robot speed, default 40), `--video-offset`
(calib.py: skip the sync-flash search).

## How blink.py tells a robot from everything else

Brightness alone is not enough: paper, tape and lamps are bright too. What
only a robot does is switch on and off in its rhythm. So:

1. **Spots**: small spots brighter than everything in a ring 4-6 px around them.
2. **Flashes**: a spot that was clearly darker just before *and* just after,
   and was lit at most 250 ms. A 250-420 ms flash is a sync flash and is left
   out; anything lit longer, or that never went dark, is not a robot light.
   The two LEDs of a robot flash together; they are merged into one light.
3. **Tracks**: flashes are chained into one track per robot, by position (a
   robot moves at most `--vmax-cms` and can't jump sideways), by whether the
   light shows two LEDs, and once a track's rate is clear, by its beat (a new
   flash comes a whole number of periods after the last one).
4. **Code**: for each code, how well do the track's flash starts sit on a
   beat of that code's period (each within frame-timing noise, now and then
   late; missed flashes don't matter), compared with flashes at random
   moments? The long-run rate (total time over the number of periods) must
   also match, so a lamp flickering at 0.25 s is not taken for 0.3 s. This
   gives a probability for each name and for "none". `period_s` is the
   median of the flash-to-flash intervals that are one period long.

A track is **named** only if its best code has probability at least 0.6
(`--min-name-p`), it has at least 5 flash-to-flash intervals (Left: 1.5 s,
other: 3 s), at least 75% of its flashes start right on its beat, and its spot
isn't shared with other flashing (a flickering lamp can be split into several
"tracks" that each keep the flashes that happen to fit). Every track with 3
or more flashes is reported, named or not.

## blink_detections.csv

One row per video frame in which a tracked light was lit (with
`--interpolate`, also rows between its flashes).

| Column | Meaning |
| --- | --- |
| frame | video frame number (row in frames.csv) |
| t_wall | that frame's time from frames.csv (Pi clock, Unix seconds) |
| robot | robot name from the blink code (`--names` mapping); empty if the code isn't clear (see the p_ columns) |
| x_px, y_px | centre of the light: the midpoint of the two LEDs, image pixels (origin top-left, pixel centres on whole numbers). This is a point on the robot's top, about 4 cm above the floor, not the robot's centre |
| code | the code (1-4) if the track is named, else 0 |
| score | the probability of that name; for unnamed tracks the best code's probability |
| track | track number; the same robot seen in separate stretches gets separate tracks |
| lit | 1 = measured in this frame; 0 = filled in between flashes (`--interpolate` only, straight line between the flash before and after, gaps up to 1 s) |
| n_spots | 2 = both LEDs seen; 1 = one LED seen, position moved to the pair midpoint using the nearest flash where both were seen (or left as is if there was none); 0 = interpolated row |
| period_s | the track's flash period: median of its one-period flash-to-flash intervals, s |
| shared | other flashing seen on the track's spot while it was tracked, per flash of the track (a robot's own spot: about 0) |
| p_Left, p_Center, p_Right, p_other | probability that the track is that robot (column names follow `--names`) |
| p_none | probability that it is no robot code at all (a lamp, a reflection) |

All columns from `code` on are per track (the same on every row of a track).

## calibration_results.csv

One row per forward roll: the forward roll of each `calib` run (RAW step 9
from firmware 2026-10-18c, which spins first and rolls up to `calib_roll_cm`
where it found room, the length in its CALIB_LOOK record; RAW step 2 before:
15 cm in 2026-10-18b, 3 cm earlier), and RAW step -1 of each
turn/demo/straight run (the start nudge, `start_nudge_cm`). A calib row of
firmware 2026-10-18b or later also covers the roll back and the two spins
(columns after `led_left_cm`; blank for older runs and for nudges). A
2026-10-18c run that found no room to roll still gets a row: identity from
its blink code only, the spins' gyro scale and the light's circle radius (no
heading, so no offset direction); what it could not measure is blank with
quality `skipped: no room`, and its `outcome` is `limited (no room)`, not a
failure (a run that skipped the spin likewise). `outcome` is `done`,
`limited (no room)` or `failed` for every 2026-10-18c+ calib row, and the
summary counts the three. The calib run's steps (by firmware) are listed in
docs/data-formats.md ("Calibration outcomes" for the three states) and in
calib.py's STEPS_*.

| Column | Meaning |
| --- | --- |
| run, robot_log, experiment | the run, the robot whose log it is, the experiment |
| event | `calib` (calibration roll) or `nudge` (start nudge) |
| t_video | when the wheels started turning, on the video clock |
| robot_blink, blink_code, blink_score | the name, code and probability blink.py gave the light that moved at that moment (empty if not named) |
| blink_p_log | blink.py's probability that that light is the robot whose log shows the roll |
| identity | `agree` / `DISAGREE`: does the light that moved carry the code of the robot whose log shows the roll? `code not readable` = it was tracked but not named (see blink_p_log). `motion only` = no blink track there at that moment (e.g. it moved before blinking started: then the start flash located it). `(other robots moving too)` = another robot's log shows its wheels turning at the same moment, so timing alone could not tell them apart |
| others_moving | those other robots |
| lights_moved | how many lights near that moment moved by about the commanded distance |
| onset_diff_ms | when the picture around the light started to change minus when the log says the wheels started (video clock); used to pick the right light when several moved |
| heading_img_deg | direction of the forward roll in the image: 0 = image right, counter-clockwise on screen positive (as in labels.csv) |
| heading_floor_deg | which way the robot faced, standing still before the roll, on the floor: 0 = +x, counter-clockwise (left) positive. Older calib runs and nudges: the forward roll's direction. Calib runs 2026-10-18b+: the forward and back rolls' estimates combined (circular mean), each corrected for any curve in the roll (see heading_gyro_corr_deg) |
| heading_method | `video shift`: measured by lining up the picture around the robot just before the roll with the picture during the pause at its far end |
| moved_px, moved_cm | how far it moved (compare with commanded_cm: a big difference means something else moved) |
| commanded_cm | the roll the firmware commanded |
| motion_quality | how clearly the shift stood out (peak height in noise widths; 8 is the minimum accepted) |
| det_heading_floor_deg | heading from the robot-centre detections instead (cross-check; blank without --detections) |
| light_x_px, light_y_px | the light (LED midpoint) while the robot stood still |
| centre_x_px, centre_y_px | the robot centre from --detections while still |
| led_fwd_px, led_left_px | the light's offset from the robot centre in the robot's own frame, image pixels (forward along the heading, left = 90 deg counter-clockwise on screen) |
| led_fwd_cm, led_left_cm | the same in cm on the floor (both points corrected for being on the robot's top, 4 cm up, if the calibration file gives the camera height) |
| heading_chord_deg | the forward roll's direction on the floor, uncorrected (calib runs 2026-10-18b+) |
| heading_gyro_corr_deg | correction added to the chord: minus half the heading change the gyro saw during the roll (drift taken out). A roll that curves (left and right wheels slightly different) goes in the robot's average heading over the roll, not its starting one |
| heading_back_deg | the starting heading worked out from the roll back instead (its direction + 180, less half its own turn and the forward roll's turn) |
| heading_spread_deg | forward estimate minus back estimate |
| heading_quality | `ok` (spread within 1 deg), `check: ...`; `single roll` for short nudges |
| moved_back_cm, back_quality | the roll back: distance on the floor, and its picture-match peak (as motion_quality) |
| wheel_cm, wheel_back_cm | the same rolls by the robot's own wheel odometry (RAW x_cm, y_cm) |
| wheel_scale | camera distance / wheel distance, both rolls together: multiply wheel odometry by it. Depends on the floor calibration's scale being right |
| wheel_scale_spread, wheel_scale_quality | forward ratio minus back ratio; `ok` if they agree within 1% and the scale is within 0.9-1.1 |
| gyro_bias_dps | the gyro's drift while standing still (left positive), from the stills (blink still, after each roll, around the spins) |
| spin1_gyro_deg, spin2_gyro_deg | each spin by the gyro (RAW yaw_deg, signed left positive from the log itself, drift taken out), still to still |
| spin1_cam_deg, spin2_cam_deg | each spin by the camera: the robot's picture in the still before and after aligned by rotation and shift (OpenCV ECC over the robot's outline plus 1 cm, found by comparing with the picture of the empty spot during the forward roll's pause; a 4.6 cm disc if there is none); whole turns taken from the gyro |
| spin_match | the weaker of the two alignments' correlation (0-1) |
| gyro_scale | camera degrees / gyro degrees over both spins (multiply gyro turns by it); the two spins turn opposite ways, so leftover drift cancels |
| gyro_scale_spread | spin 1 ratio minus spin 2 ratio |
| gyro_scale_trace | cross-check: slope of the light's angle around the turning centre against the gyro's angle, during the spins (less precise in the simulator: 0.14% rms against 0.09%) |
| gyro_scale_quality | `ok` if the spins agree within 0.5%, the scale is within 0.95-1.05, spin_match is at least 0.8 and the trace agrees within 0.5% |
| spin_centre_x_px, spin_centre_y_px | the turning centre in the image (circle through the light's flashes, at the lights' height) |
| led_points, led_radius_cm, led_fit_rms_cm | light flashes seen during the spins, the circle's radius on the floor and the points' rms distance from it (a circle is fitted to each spin separately) |
| led_angle_sd_deg | spread of where on the circle the light sat relative to the heading (heading from the roll plus the gyro, scaled by gyro_scale) |
| spin_led_fwd_cm, spin_led_left_cm | the light's offset from the TURNING CENTRE (the wheel-axle midpoint, which the robot's pose refers to), forward and left. tools/fusion uses these when led_offset_quality is ok, else led_fwd_cm / led_left_cm |
| led_offset_quality | `ok` if the fit rms is at most 0.3 cm, at least 12 flashes, radius 0.5-8 cm |
| look_result, look_heading_deg, look_room_cm | calib runs 2026-10-18c+: what the look-around chose (CALIB_LOOK result, heading_deg relative to where it faced, room_cm); `commanded_cm` is the roll it made |
| note | anything worth checking |

The quality limits (1 deg, 1%, 0.5%, 0.8, 0.3 cm, 12 flashes) are chosen
from the simulator's spread, not derived; check them against the first real
calibration runs.

The LED offset is what lets a later single flash give a robot's heading:
the robot centre is the light minus this offset turned to the heading.

## Accuracy (measured 2026-10-18, firmware 2026-10-18a rate code)

- Synthetic videos (`tests/synthworld.py`: cluttered floor with a drifting
  shadow, 2-3 robots up to 30 cm/s, two crossing robots, 3% dropped and 1%
  late frames, stretched, cut and delayed flashes, flickering lamps
  including one at 4 Hz, a moving white object, a static white spot, lone
  300 ms flashes, an idle robot with a steady white LED), 6 random seeds x 3
  scenes: 100% of blinking robot-seconds named correctly in the cluttered
  scenes, 96.7% in the crossing scene (one second before the first track had
  enough flashes), no robot named wrongly, no distractor named; light centre
  within 0.21 px of the drawn LED midpoint (95%).
- Simulator, `simulate.py --video --seed 7 --max-runs 6` (Left only, 1 calib
  run + 6 runs): 260/260 robot-seconds named correctly; light centre error
  median 0.14 px, 95% 0.24 px; calib + 6 start nudges: identity agreed 7/7,
  heading error mean 0.62 deg, max 0.95 deg; LED offset 2.73 cm forward
  (model 2.75), -0.02 cm left.
- Simulator, full E1 sheet with `--robots Left,Center,Right` (37 min):
  1589/1590 robot-seconds (99.9%) named correctly, no wrong names, 2 named
  rows off any robot; identity agreed on 39/39 rolls; heading error mean
  0.45 deg, max 1.33 deg; LED offset 2.73 / 0.00 cm.

## Accuracy of the calibration run (measured 2026-10-18, firmware 2026-10-18b)

Simulator, 17 calibration runs (seeds 11-15 with Left, Center and Right,
plus seeds 7 and 21), each against the simulator's truth:

| | 15 cm roll + spins (new) | 3 cm roll (old run, old calib.py; 15 runs) |
| --- | --- | --- |
| heading error, rms / max | 0.08 / 0.17 deg | 0.38 / 0.67 deg |
| wheel distance scale error, rms / max | 0.14 / 0.39 % (true scales 0.990-1.008) | not measured |
| gyro scale error, rms / max | 0.05 / 0.11 % (true 0.9955-1.0049) | not measured |
| light offset from turning centre, error | forward +0.01 cm (max 0.03), left max 0.02 cm | (2.75 cm assumed) |

Without the gyro's curve correction (forward chord alone) the heading
error was 0.19 deg rms: the correction and the second (back) roll each help.
All quality columns were `ok` on all 17. The simulator's robot is plain
(top plate, LEDs, sensor window); a real robot top has more detail, which
should make the spin pictures align better, but its lighting, shadows and
reflections are not simulated.

Firmware 2026-10-18c (looks around first, spins before the roll), same
simulator, Center only, against 2026-10-18b on the same seeds (21-30):

| | 2026-10-18b (roll first) | 18c, open floor (rolls straight ahead) | 18c, turned ~180 deg before the roll (laptop cap ahead) |
| --- | --- | --- | --- |
| heading error, rms / max | 0.09 / 0.21 deg | 0.09 / 0.16 deg | 0.12 / 0.24 deg |
| wheel scale error, rms / max | 0.41 / 0.81 % | 0.27 / 0.55 % | 0.31 / 0.54 % |
| gyro scale error, rms / max | 0.025 / 0.043 % | 0.020 / 0.037 % | 0.019 / 0.034 % |
| light offset error, forward / left max | 0.02 / 0.02 cm | 0.02 / 0.03 cm | 0.02 / 0.03 cm |

All quality columns `ok` on all 30. Boxed in with no room to roll (one run,
tests/test_calib_spin.py): gyro scale within 0.4 % (disc mask), light radius
within 0.1 cm; no heading, wheel scale or offset direction (quality
`skipped: no room`, outcome `limited (no room)`).

## Limits

- A robot is named after about 6 flashes (Left 1.5 s, other 3 s) of a
  steady rhythm; before that its rows carry probabilities but no name. A robot
  that drives out of view and back comes back as a new track.
- Two robots with the same code passing within a few cm can swap tracks.
- A lamp that flashes like a robot (100 ms flashes exactly every 0.3-0.6 s)
  would be taken for one.
- Lights must look white and small: a robot under a bright spotlight, or seen
  so small that its LEDs are under 2 px wide, may be missed. The ring sizes
  (4-6 px) assume about 4 px per cm.
- The decoder assumes the robot's and Pi's clocks run at the same rate (true
  to about 0.01%); very long tracks (many minutes) could drift off the beat.
- calib.py needs the video clock lined up with the robot clocks (from the
  sync flashes, as tools/fusion does) to within about 100 ms, a still moment
  before the roll and the 200-300 ms pause at its far end in the video, and
  no other object moving within ~14 cm of the robot then.
- Heading comes from the roll's direction, so it assumes the robot rolls
  straight ahead (no sideways slip). The curve correction assumes the roll
  turns evenly along its length (half the gyro's turn at the middle).
- Calibration run (2026-10-18b+): the spin analysis needs the forward roll's
  pause picture as the empty floor under the robot (roll at least 12 cm; a
  shorter roll from 2026-10-18c, or none, falls back to a disc-shaped mask:
  less precise gyro scale),
  blinking lights during the spins (blink_codes 1) for the light offset, and
  nothing else moving within ~8 cm of the robot. wheel_scale is only as good
  as the floor calibration's scale (a 0.5% scale error there is a 0.5% error
  here).

## Checks

```
python -m pytest tools/blink/tests -q            # about 3-6 minutes
python tools/blink/tests/synthworld.py out --scenario clutter   # look at a test video
python tools/sim/simulate.py --out sim_out --video --seed 7 --max-runs 6
python tools/blink/eval_sim.py sim_out           # decode accuracy, position and heading errors vs the simulator's truth
```

| File | What |
| --- | --- |
| `blink.py` | finds, follows and decodes the blinking lights |
| `calib.py` | identity, heading and LED offset from calib rolls and start nudges |
| `eval_sim.py` | scores both against a simulator run |
| `tests/synthworld.py` | synthetic recordings (and console logs) with known truth |
| `tests/score.py` | compares blink rows with a truth file |
| `tests/test_blink.py`, `tests/test_calib.py` | the tests (test_calib.py: synthetic, pre-2026-10-18b 3 cm calibration runs) |
| `tests/test_calib_spin.py` | a simulated calibration run (current firmware: look first, spins, then the roll; simulator + video, about 2 minutes): heading, wheel scale, gyro scale, light offset vs truth; plus a run boxed in by boards (no roll: spin results only) |
