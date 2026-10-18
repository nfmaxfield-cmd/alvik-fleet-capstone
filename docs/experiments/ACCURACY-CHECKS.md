# Accuracy checks

Every result in this project rests on numbers that were chosen, assumed or
measured only once. This page lists each one that could make a result wrong,
and the test that checks it. A sweep of the repo on Oct 18 found 86 such
items; they are grouped here into checks.

Four kinds of check, by when they happen:

| Kind | When | Extra time | Tool |
| --- | --- | --- | --- |
| **A. Every session, automatic** | after each session | none | `tools/check_session.py` (being extended) |
| **B. From the data already recorded** | after T0, E0b and T1 | none | `tools/accuracy_checks.py` (being built), `heading_check.py`, `nudge_check.py`, `calibrate_floor.py` |
| **C. Measured once by hand** | E0c session, about 75 min | 75 min | `E00c-rig-measurements/` sheet |
| **D. In a later test** | T4 to T7 | part of those tests | their records |

Status: **built** = the tool exists and is tested; **building** = in
progress Oct 18; **planned** = written here, not built yet.

## A. Every session, automatic

| # | What could be off | Check | Pass rule | Status |
| --- | --- | --- | --- | --- |
| A1 | The video clock drifts against the laptop clock (one constant offset is assumed) | Offset at every sync flash vs time; slope and scatter | Drift adds under 5 ms over the session; scatter under 10 ms | building |
| A2 | The camera was bumped or sagged during the session (one calibration is assumed for the whole session) | Compare the still floor at the start, middle and end of the video | Moved under 0.5 px (0.15 cm), turned under 0.1° | building |
| A3 | Lighting changed (camera settings are locked at the start) | Brightness of the still floor over time; camera settings at start vs end | Under 10% change | building |
| A4 | Frames late, dropped or delivered with uneven delay | Frame times vs the camera's own times | No drops; delivery jitter under 5 ms | partly built |
| A5 | A blink flash stretched long enough to look like the 300 ms sync flash | Longest blink flash in the video | Under 220 ms | building |
| A6 | Flash timing uses an assumed 12 ms exposure and some tools assume 30 fps | Read the real exposure and frame rate from the recording | Values read, not assumed | building |
| A7 | WiFi losses come in bursts (the simulator assumes random single losses) | Lost sensor rows: longest run, gaps | No gap over 300 ms outside the library's moves | partly built |
| A8 | Robot clock fit (each robot's clock vs the laptop's) | Fit residuals and drift per robot and power-on | Residuals under 10 ms | built |
| A9 | The floor isn't level, or the gyro isn't vertical | Accelerometer tilt while still | Under 2° | building |
| A10 | Rest noise is above the "standing still" threshold the fusion uses | Wheel and gyro noise while still | Below 0.2 rpm, 0.1 cm/s, 0.5°/s, or the threshold is changed to fit | building |
| A11 | Gyro bias changes as the robot warms up | Bias in each still period vs time since power-on | Under 0.2°/s change | building |
| A12 | The "lifted" flag trips when the robot isn't lifted | Lifted readings during runs | None | building |
| A13 | Front distance sensor frame rate and age differ from the assumed 15 per second, no delay | Changes per second in each zone | Measured value used | building |
| A14 | Calibration run results per power-on: light position, heading, wheel and gyro scale | Next firmware's longer calibration run, read by `blink/calib.py` | Spread across power-ons under 0.3 cm and 0.5% | building (next firmware) |

## B. From data already recorded (T0, E0b, T1)

| # | What could be off | Check | Pass rule | Status |
| --- | --- | --- | --- | --- |
| B1 | Camera floor calibration | Leave-one-out error at the T0 spots and board corners | Under 0.5 cm | built |
| B2 | The plywood is warped, or the lens bends more than the one-term model allows | Map of calibration errors across the floor: random scatter or a pattern | No pattern bigger than 0.3 cm | planned (with B1) |
| B3 | Camera height 262 cm (tape, once) | Height the calibration fit implies vs the tape | Within 2 cm | built (fit reports it) |
| B4 | The start roll doesn't return the robot exactly to its marks | E0b: marks before and after the roll | Under 0.2 cm and 0.3° | built |
| B5 | Tape heading precision (T0's pass 2 was skipped) | E0b: heading change between the two sets of marks; camera vs tape heading | Tape heading SD under 0.5° | built |
| B6 | Start-roll heading noise is assumed 0.8° (a simulator value; 1 px is about 8° on a 2 cm roll) | `heading_check.py` spread, per roll size | Measured value replaces the assumed one | built; fusion use building |
| B7 | Robots aren't symmetric, or the 9.5 cm mark spacing differs per robot | Mark spacing per robot over all T1 runs | Each robot within 0.2 cm of 9.5 | building |
| B8 | Wheel size: 1.7 cm radius assumed for all robots | Straight runs: tape distance vs wheel distance, per robot | Known to 0.5%; fusion learns it per robot | building |
| B9 | Wheel turn size (distance between wheels) | L and square runs: wheel heading vs tape | Known to 1% | building |
| B10 | Gyro scale assumed exact | Square runs: 1080° of turning, gyro vs tape end heading | Within 0.5% | building |
| B11 | Gyro bias and noise use simulator values | Still periods in every T1 run | Measured values replace them | building |
| B12 | Wheel and gyro readings in the same row aren't from the same moment | Lag between wheel and gyro at each start of motion | Under 20 ms, or corrected | building |
| B13 | Camera position noise assumed 0.8 cm, light 0.5 cm, camera heading 8° | Camera vs tape at every still moment (T0 spots, each run's start and end) | Measured values written to `measured_noise.json` for the fusion | building |
| B14 | The robot moved between the end of a run and its end marks | Video after each run's end flash | No movement over 0.2 cm | building |
| B15 | Floor wear, warm-up or time of day changes results over the session | End error vs run order, and vs battery | No significant trend | building |
| B16 | A robot is named wrongly by its blink code or its motion | Blink name vs the robot that actually ran, every run | Under 1 error in 100, with interval | building |
| B17 | Robots differ (Center has newer motor firmware) | End error by robot | Reported per robot; robot kept as a block in every test | building |
| B18 | Light position on the robot assumed 2.75 cm ahead of centre | Calibration runs: light vs centre, per robot | Measured per robot (spread under 0.3 cm) | built (`blink/calib.py`) |
| B19 | Fusion settings (noise, gates, thresholds) are simulator values chosen while looking at results | Tune on half of T1, score on the other half | Results reported only on the scoring half | building (split rule) |
| B20 | End error is skewed (never negative), so t-intervals can mislead | Bootstrap intervals beside the t-intervals | Both reported; conclusions hold under both | building |
| B21 | E1b compared unpaired, though it is paired by route | Paired comparison with Right's T1 runs | Paired test used | building |
| B22 | The simulator doesn't match the real camera (115 cm, 30 fps, colour) | A simulator preset for the real camera (262 cm, 25 fps, grey); re-run the simulation checks with it | Simulation conclusions still hold | building |
| B23 | Camera latency, motion blur, rolling shutter | Camera fix error vs robot speed and vs image row (T2) | No trend over 0.3 cm | planned (T2) |
| B24 | Heading from motion assumes no side slip | Heading from motion vs tape at the end of straight runs | Within its stated noise | planned (T2) |
| B25 | Filter noise settings are inconsistent with real errors | Normalised errors (NIS) on the scoring half | Within the 95% band | planned (T3) |
| B26 | Front distance sensor model (directions, width, noise, delay) and board positions | Front sensor calibrated against the camera, board positions refined, then checked on held-out runs | Distance sensor never worsens the camera-on result | building |
| B27 | The site map (board, block and open-edge positions) is wrong: clicks off, a board moved, the map's uncertainty understated | Site setup ([E0d](E00d-site-setup/)): robots survey every landmark, joint fit; tape audit of the least-sure points + 1 random before locking; afterwards every landmark point tape-measured | Audit: each point within max(0.3 cm, 3 sigma); E0d: 90% of points within 0.5 cm, normalised errors consistent with the stated sigma | built (simulation) |
| B28 | A landmark moves after the map was locked | Change check on every run (`tools/site/change.py`, `run_plan.py --site-map`): a landmark whose front-sensor readings keep disagreeing is flagged | No flag on unmoved landmarks over T1; a board moved 1 cm on purpose is flagged | built (simulation) |

## C. Measured once by hand (E0c, about 75 minutes)

Procedure and sheet: [E00c-rig-measurements](E00c-rig-measurements/).

| # | What could be off | Measurement | Feeds |
| --- | --- | --- | --- |
| C1 | Robot length and width; face centres; whether the body is symmetric | Each robot, tape or calipers | Mark spacing (B7), robot centre |
| C2 | Robot centre (mark midpoint) assumed to be the wheel axle | Front face to axle, each robot | `axle_to_center_cm` in fusion and calibration |
| C3 | Robot top and light height assumed 4 cm | Floor to top surface and to the lights | Camera calibration (robot points), light positions |
| C4 | Wheel diameter and distance between wheels | Each robot | Priors for B8 and B9 |
| C5 | Board corners square, factory edge straight | Both diagonals; string along each edge | Floor coordinates |
| C6 | Taped axes at exactly 90° | 3-4-5 check: 30 cm along x, 40 cm along y, diagonal should be 50 cm | Floor coordinates |
| C7 | Point straight below the camera (estimated by the fit, no plumb line) and camera height | Plumb line from the lens; height re-measured with a second person | Camera calibration check (B3) |
| C8 | Plywood flat | Straightedge across the board in 4 directions; largest gap | Floor flatness assumption (B2) |
| C9 | Front distance sensor: noise and bias vs distance | Each robot square to a board at 10, 20, 40, 60, 80, 100 cm, 10 s each | Distance sensor noise (B26) |
| C10 | Front distance sensor zone directions and width | The next firmware's calibration spin sweeps the sensor across the boards | Zone model (B26) |
| C11 | A board 1 cm off its mapped position | Replay T1 or E0c with the map shifted 1 cm (no extra runs) | Map error sensitivity |
| C12 | Light switch-on delay (the flash time is logged just before the light is told to switch on) | Next firmware logs the time before and after the switch-on call | Camera timing |
| C14 | Gyro warm-up | Left still for 10 min from power-on, logging | Bias drift (A11) |
| C15 | Room temperature | Thermometer reading at each session start | Warm-up and drift notes |
| C17 | The robot can't tell where the plywood ends where no board borders it | Line sensors under the nose: on plywood, on tape, and 1 cm past an open edge (E0c edge rows) | Edge guard threshold for the next firmware |
| C16 | Lifted flag | 60 s untouched (must not trip); lifted once (must stop the run) | A12 |

## D. In a later test

| # | What could be off | Where |
| --- | --- | --- |
| D1 | Wheels spinning when lifted; robot picked up and moved | T4 (failures, kidnap runs) |
| D2 | WiFi loss and delay with three robots | T5 |
| D3 | Other floors, lighting, hands, look-alike objects | T6 |
| D4 | Identity and name hand-off with several robots close together; overconfident identity probabilities | T7 (reliability of the probabilities) |
| D5 | Identity uses later video (not live) | T7, re-run on video cut at each moment |
| D6 | Speed effects (blur, slip) | T8 |
| D7 | Robots parked out of view during other robots' runs | T7 video check |

## Order of work

1. Today: E0b (B4, B5), then E1b and T1. Session checks (A) run on each.
2. This week: E0c by hand (C), then `accuracy_checks.py` on T0, E0b and T1 (B).
3. Each later test runs A and the B checks that apply, and fixes or reports
   any FAIL before its results are used.
