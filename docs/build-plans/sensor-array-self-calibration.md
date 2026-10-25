# Self-calibrating robots and sensors it has never seen: design assessment (2026-10-25, revised 11:05)

**Premise.** Nick, 2026-10-25: "The from scratch setting should work in a new system with different sensor types.
We need to apply the logic you describe for front sensor applying to new unknown sensors. Do an assessment of
everything the system would need in user input for an offline system to self-calibrate an array of sensors,
identify and label anything and issue instructions. The UI for this will likely precede the live lab."
His answers (11:05) set the scope: three settings named **Fresh**, **From scratch** and **New system from
scratch**; the dock's printed dimensions are the one known length; truly unknown robots are in scope (the program
guesses what each sensor does *and* where the drive commands are and how they work, with UI confirmation and
cautious testing allowed); sensors of any kind (lidar, sonar, homing beacons), with or without a camera; a shifted
sensor is simply re-learned and noted; all setup screens in one place; and the long-term goal is to find the
robots, find every relevant sensor and control the robots accurately, all automatically, with minimal user input.

This is a design assessment only. No code was changed, and nothing below is built until Nick approves it. The
recommendations and judgements are Claude's and are marked **[Claude]**; facts about the existing code cite files.

**Data read.** `tools/experiment/e02_front.py`, `e02_trust.py`, `e02_fit.py`, `e02_explore.py`, the runner's setup,
identify and question code (`e02.py`, `e02_view.py`), `tools/fusion/tof_calib.py`, `firmware/home_lab/rawlog.h`
and `home_lab.ino` (the RAW record, the `help` command list), `docs/data-formats.md`,
`docs/build-reports/e02-oct25-heading.md`, `e02-oct25-reverse-find.md`, and the build plans `generality-audit.md`,
`publication-plan.md`, `live-lab-outline.md`, `generic-platform-outline.md`.

---

## Conclusion, in short

1. **One engine, three starting points [Claude].** New system from scratch = find the devices, learn the drive
   commands, learn what each sensor is, then run From scratch. From scratch = measure everything with only generic
   starting guesses. Fresh = From scratch, with old measurements as the starting guess and the yardstick for change.
2. **The setting belongs to each sensor, not the session [Claude].** A known robot with one new sensor is Fresh
   for its wheels and New system from scratch for the new sensor. The engine keeps a state per stream.
3. **Provenance is the first rule.** Every number carries whether it was *measured*, *derived*, *guessed* or
   *copied*, and from what. A guessed or copied number is never used as if measured: never written into a robot,
   never used to judge another sensor. The heading problem (section 3) was exactly this rule broken: the runner
   wrote unmeasured headings into the robots, and 100 of 112 large heading errors traced to it.
4. **The front-sensor logic generalises to any stream:** predict what it should read from an independent
   reference, take the difference, learn offset, scale and noise, compare with old measurements, turn it into a
   trust factor. Today's code learns offset but not scale, and its trust code keeps old evidence too long after a
   swap (section 4).
5. **On a computer MIRLT was just installed on, finding the devices is realistic; reading an arbitrary closed
   protocol is not.** Cameras, USB and serial devices, network devices (mDNS, open ports, broadcasts), Bluetooth
   devices and ROS 2 graphs can all be listed automatically, and ROS 2 topics describe themselves (LaserScan, Imu,
   Range, Twist). A closed binary protocol cannot be reverse-engineered safely; it needs a small adapter or a
   robot-side agent (section 5).
6. **Drive commands are learned by cautious system identification:** prove the stop works first, then tiny
   commands, watching the camera or the robot's own sensors, growing only while the response is consistent; out
   come sign, scale, deadband, latency and turning geometry (section 6).
7. **Sensor discovery works with or without a camera.** Without one, the references are board touches, gravity,
   commanded motion, other robots, beacons, the dock, and agreement between independent sensors (three ways of
   measuring a turn that agree confirm all three) (section 7).
8. **Truly user-only input shrinks to two things:** places robots must not go, and consent to start moving. Which
   robots are ours (the ones that respond to our commands), drops (edge sensors, approached cautiously) and the
   dock (a known printed shape) are all inferred (section 8).
9. **The cheapest proof is free data [Claude]:** the Alvik's 37 RAW columns with names hidden, and the heading
   logs, are labelled test sets. Logs-only replays can score discovery and the error checks before any board time.
10. **Biggest risk: no From scratch session has finished on real hardware yet.** The one real explore (2026-10-24
    stage 3) stopped during identify.

---

## 1. The three settings

| | **Fresh** | **From scratch** | **New system from scratch** |
|---|---|---|---|
| In Nick's words | New measurements compared against old | No old measurements; sensor types known | No old measurements; sensor types unknown |
| Knows at the start | Robot and sensor list, types, mounts, units, drivers; every past session's measurements | Robot and sensor list, types, mounts, units, drivers (the robot profile); generic starting guesses | That MIRLT is installed, and the dock's printed dimensions. Not which devices exist, how to drive them, or what their sensors are |
| Measures | Every value again: offset, scale, noise, latency against independent references; compares with old | The same, plus the camera (explore) and work area if there is a camera, and every robot's wheel and gyro scale | First the devices, the drive commands and each stream's type; then everything From scratch measures |
| When a value has shifted | Uses the new value at once, adapts the fusion to it, notes it in the results (Nick, 11:05). No question | (nothing old to compare) | (nothing old to compare) |
| Asks the user | Consent to start | Consent; no-go areas; low-confidence confirmations only | Consent; no-go areas; low-confidence confirmations of sensor types and drive channels |
| Calibration run, 3 robots (estimate, Appendix C) | 8-12 min before trials; keeps checking through the session | 30-40 min | 75-90 min |

Nick has said accuracy and autonomy matter more than run time, so New system from scratch repeats its test motions
at the end as a repeatability check (about 10 minutes, included above).

---

## 2. Provenance: every number knows where it came from

**Rule [Claude, from Nick's 11:05 answer].** Every value the system holds (a calibration, a heading, a sensor's
offset, a mount position, a type label) carries a tag:

| Tag | Meaning | May it be written into a robot? | May it judge another sensor? |
|---|---|---|---|
| **measured** | From this session's data against a named reference, with its n and sd | Yes | Yes, within its sd |
| **derived** | Computed from measured values (a heading from a path the camera saw) | Yes, if its inputs are measured | Yes, but never counted as separate evidence from its inputs |
| **guessed** | A default, a placeholder, a discovery guess not yet confirmed, a commanded value assumed to have happened | **No** | **No** |
| **copied** | From an earlier session, another robot or a profile | Only as a starting guess, labelled as such | Only as the yardstick for "changed" |

Each tag also records the reference, the time and the session. Every log row, fix and report line shows the tag.
Values are promoted (guessed to measured) only by a measurement, never by time passing or by being used.

---

## 3. Errors: identify, diagnose, adapt to reality

### The worked example: the heading problem (`docs/build-reports/e02-oct25-heading.md`)

- **Symptom.** 112 of 663 stretches between camera fixes (17%) ended with a heading disagreement of 30 degrees or
  more, needing a 22 cm (median) position correction.
- **Identify.** The disagreement was the runner's heading against the robot's. Neither was a measurement, so the
  check compared two unmeasured things. The independent reference was the direction the camera saw the robot
  drive on a go leg of 8 cm or more: 803 legs measured.
- **Diagnose.** Five hypotheses, each tested against the logs: a quarter-turn artefact (partly), a fix writing a
  wrong heading (**yes: 100 of 112**), gyro scale, drift or wrap (no: the gyro's own error grew a median 1.4-3.6
  degrees), wheel slip (no), one robot or phase (mostly parking). Attribution came from *which* reference
  disagreed *when*: the error was already there at the first measured leg after a fix, so the fix wrote it in; it
  did not grow on the way.
- **The cause was provenance.** A placeholder heading of plus or minus 90 degrees (68%), a run's own end heading
  echoed back (12%) and a stale heading kept across a refind (10%) were all *guessed* values sent as if
  *measured*. Of 140 ordinary fixes that found the robot right, 1 made it wrong.
- **Adapt.** Send a heading only when it was measured; measure one with a short commanded drive as soon as
  possible; never write a guessed one (the report's fixes 1-3, waiting on Nick).

### The general method, for every setting

| Step | What the system does | Fresh | From scratch | New system from scratch |
|---|---|---|---|---|
| **Identify** | Residuals against **independent** references (never a value checked against itself or its own source); consistency across sensors (do gyro, wheels and camera agree on a turn?) and across robots (does one stand out?); provenance checks (is any value in a comparison guessed or copied?) | Also against old measurements: a shift beyond max(minimum, 3 standard errors) | Against the references only | Also: does each stream still behave like its label? Does each drive channel still move as learned? |
| **Diagnose** | List hypotheses, test each against the logs; attribute by which references disagree (only the camera disagrees with all robots = camera; one robot disagrees with camera and siblings = that robot; the error appears at a reset = what the reset wrote) and by when (at a reset, growing with distance, growing with turning, only in one region) | History says whether this is new | Siblings and references only | Plus: is the label itself wrong? |
| **Adapt** (cheapest first) | 1. Re-estimate from this session's data. 2. Quarantine the source (trust factor up, or out of the fusion), keep logging it. 3. Re-test with a commanded motion designed to separate the hypotheses. 4. Ask the user, last, only for intent | Use the new value, note it (no question) | Same | Same; a stream whose behaviour no longer fits its label goes back to "unknown" |

<sub>Residual: a reading minus what it should have read. Standard error: how far the median of n readings is likely
to be from the true value, about 1.25 x sd / sqrt(n).</sub>

---

## 4. A general sensor model, and how each part is learned

Every stream (camera, wheels, IMU, ranging, lidar, sonar, beacons, switches, line and cliff sensors, anything)
gets the same record, each entry tagged with its provenance. **References, strongest first:** a board or dock
touch (1-2 cm); the dock's printed dimensions (the absolute length); the camera after explore (0.8 cm per axis at
a find, `e02_trust.HEALTHY`), if there is one; fixed beacons once their positions are learned; gravity (1 g at
rest, no turning at rest); agreement between independent sensors; the other robots (what all share is the common
reference's, what one has on top is its own); the motion the system commanded (sign and timing known, size only
roughly, and only *guessed* until measured).

| What the system needs | How it is estimated | Example references |
|---|---|---|
| Identity and grouping | Which device, which channel; streams that update at the same instants and move together are one sensor (5 ToF zones; 3 gyro axes; 360 lidar beams) | Timestamps |
| Type | Sections 5-7 | All |
| Units | The ratio to its reference snapped to the nearest unit factor (1, 10, 100, 1000, 57.3, 9.81) when within 15%; the rest is a calibration scale | Camera, gravity, dock, touches |
| Rate | Count only rows where the value changes (RAW repeats the front sensor's last frame: about 15 a second inside 50 rows) | Timestamps |
| Noise | Robust spread at rest and about its prediction in motion, minus the reference's own share (`front_sd`: sqrt(sd² − reference sd²)) | Still periods, siblings |
| Bias and scale | Robust regression of reading on prediction: intercept and slope. **`e02_front` learns only the offset**; a sensor reading 5% long hides in the median. `tof_calib.py` already fits a range scale offline | Camera + map, touches, gravity |
| Latency | The time shift that best lines the stream up with its reference at the sharp start and stop of a commanded motion | Commands, camera (`capture_delay.py`), encoders |
| Position and direction on the robot | Range or lidar: where the walls must be for the readings to fit during a spin (`tof_calib` zone directions). IMU: gravity gives down, the spin gives the yaw axis and sign. Line or cliff: where the robot was when the reading stepped at a known edge. Switch: which side clicks first on an angled approach, and how far from the axle the wall is at the click. In ROS 2 the TF tree may give it, as *copied* until checked | Camera, gravity, touches, map |
| Valid range and field of view | The widest readings that still agree with the prediction | Camera + map, touches |
| Failure signs | Stuck (no change while the reference changes); dropouts or sentinels (−1); clipping; noise above its own baseline (k ≥ 2, `DEGRADED_K`); offset or scale shift; disagreement only in one region or speed | All, continuously |

<sub>Robust spread: MAD x 1.48, the median distance from the median, scaled to equal the standard deviation for
normal data; a few outliers barely move it. Robust regression: a straight-line fit that gives little weight to
readings far from the line.</sub>

**The front-sensor logic, written for any stream.** (1) Predict what it should read from an independent
reference. (2) Residual = reading − prediction, kept only where the prediction is trustworthy (`front_check`: square
on, measured heading, in range). (3) Until this session has N residuals (8 for the front sensor), start from old
measurements (Fresh) or a generic guess (the others), tagged *copied* or *guessed*; from N on, this session's own,
tagged *measured*. (4) Fresh only: a shift beyond max(minimum, 3 standard errors) is noted and the new value used.
(5) Trust factor k = learned noise / baseline noise; flag at 2, clear below 1.5; with two or more robots flag only
one that stands out (`STANDS_OUT`); never blame a sensor while its reference is unknown or flagged.

**Three gaps when applied to sensors it does not know [Claude]:**
- **No healthy value exists for a new sensor.** `e02_trust.HEALTHY` holds Alvik numbers (audit E6). The baseline
  must be the session's own first stretch (tagged provisional) or the same type on sibling robots. With one robot
  and a new sensor, the system can say "agrees with the reference within X", not "worse than normal", until a
  second session.
- **Trust keeps old evidence too long.** `SensorTrust.new_session` halves old evidence but keeps it: after a swap,
  20 old items at half weight outweigh 5 new ones. The front rule (this session's own after N) catches a swap at
  once and matches Nick's "just use the new value". Use the front rule for every stream.
- **Add the scale term**, so a sensor that reads long in proportion is caught.

---

## 5. Discovery on a computer MIRLT was just installed on

What a fresh install can realistically find, with no configuration:

| Kind of device | How it is found | What it tells the system | Realistic? |
|---|---|---|---|
| USB and built-in cameras | The operating system's camera list (Video4Linux, DirectShow / Media Foundation, AVFoundation) | A picture stream, resolution, frame rate | **Yes** |
| Network cameras | ONVIF WS-Discovery, mDNS, RTSP on port 554, MJPEG over HTTP (the Pi recorder's `/live`) | A picture stream; often a name | **Yes** (a password may be needed: a user-only input) |
| USB serial devices | The USB vendor and product IDs (an Arduino, an ESP32, a USB-serial chip); then listening at common speeds for readable text | That a microcontroller is there; whether it talks text | **Yes** to list; listening is safe; *sending* is not, until a protocol is known |
| Network robots | mDNS / DNS-SD service names; listening for broadcasts (the Alvik's UDP hello on ports 5005/5006); a scan of the local subnet's common ports | An address and often a name | **Yes** (a scan may be blocked on campus or enterprise networks) |
| Bluetooth LE devices | Advertisements and their GATT services (standard ones such as battery are self-describing) | A device, its standard services | **Partly**: custom services are opaque |
| ROS 2 robots | DDS discovery lists nodes and topics **with their message types**: `sensor_msgs/LaserScan`, `Imu`, `Range`, `BatteryState`, `nav_msgs/Odometry`, and `geometry_msgs/Twist` for drive commands (`cmd_vel`); the TF tree gives mounts; REP 103 fixes the units | Sensor types, units, frames and the drive channel, nearly all at once | **Yes, the best case.** Types are *copied* until a test motion confirms them |
| MAVLink robots and drones | The heartbeat message, a self-describing message set | Type of vehicle, sensors, command set | **Yes** (rare in this lab) |

**Levels of how much can be learned, and what to do at each [Claude]:**

| Level | Example | Sensors | Drive commands |
|---|---|---|---|
| 1. Self-describing | ROS 2, MAVLink | Types listed; confirmed by test motions | Listed (`cmd_vel`); scale, sign, latency learned (section 6) |
| 2. Known protocol | The Alvik's UDP lines (`help` lists every command; `H,RAW,...` names every column); Firmata | From an adapter in MIRLT's adapter library | From the adapter |
| 3. Readable text, unknown meaning | A robot printing comma-separated numbers or `key=value` lines | Column discovery (section 7): realistic | **A help or command list must be readable.** Then candidate commands are tried cautiously (section 6). With no list, guessing commands is not safe |
| 4. Closed binary protocol | A vendor robot with an undocumented byte protocol | Not realistic | **Not realistic**: reverse-engineering it means sending bytes of unknown meaning to a machine that moves |

**At level 4, two ways forward:** a small **driver adapter** (one file implementing the audit's `RobotDriver`:
go, turn, stop, status, streams), written once per robot kind and shared; or a **robot-side agent**, a small
program or firmware flashed onto the robot (or onto a microcontroller wired to it) that speaks MIRLT's protocol:
it announces itself, lists its streams by name, and accepts velocity and stop commands. The Alvik firmware is
already close to such an agent. **[Claude]** Make the agent's protocol the documented MIRLT standard, so any lab
can bring a robot to level 2 in an afternoon.

---

## 6. Learning the drive commands: cautious system identification

**Order of work [Claude].**

1. **Find the stop first.** Before any motion, the stop must be known and shown to work: a robot already moving
   (or a motion that is the smallest possible) is stopped, and the stop is confirmed by the camera or the robot's
   encoders. No stop, no motion. At levels 3-4 with no known stop, the user is asked to hold the robot or keep a
   hand on its power switch (cautious testing, Nick 11:05).
2. **Tiny commands, one at a time.** The smallest command the channel accepts, for a fraction of a second, then
   stop. Watch every reference: camera, encoders, IMU, lidar scan shift, beacons.
3. **Grow only while it is consistent.** Each step doubles the size only if the last one moved the way predicted;
   any surprise (moved the other way, did not stop, moved too far) stops the sequence and lowers the size.
4. **Learn the model.** From the pairs of command and measured motion:

| What | How | Why it matters |
|---|---|---|
| Which channel moves which way | Forward, backward, left, right, each alone; which references respond | A robot whose "forward" is backwards |
| Sign | The direction of the measured motion | Same |
| Scale | Measured speed / commanded value (a straight-line fit) | Commanded 10 = 4.2 cm/s |
| Deadband | The smallest command that moves it at all | Small corrections that do nothing |
| Saturation | The command beyond which it moves no faster | Planning within what it can do |
| Latency | Command to first motion; stop to standstill | Steering and stopping distance |
| Turning geometry | Does it turn on the spot? Smallest turning circle? Left/right asymmetry | Path planning (audit R1, a car-like robot) |
| Stopping distance | Distance from stop command to standstill at each speed | Safe margins from walls and drops |

5. **Confirm, if confidence is low.** The UI shows the learned model with a short clip ("I sent 'forward 0.05';
   the robot moved 1.9 cm the way its lidar faces"), for a one-click confirm.

**Without a camera** the same steps use the robot's own encoders and IMU once they are identified (a turn command
shows on the gyro and as opposite wheel counts), a lidar or range scan's shift, or another robot's sensors
watching it. **Every learned value is tagged *measured* with its reference; a commanded motion is never assumed
to have happened.**

<sub>System identification: learning a model of how a machine responds to its inputs by trying inputs and
measuring the responses.</sub>

---

## 7. Sensor discovery (New system from scratch)

**Input:** every stream the device sends. Names (`gz`, `dL_cm`, ROS topic types) are a starting hint, tagged
*copied*; the label comes from behaviour.

**Stage A, passive (robot still, 30 s, twice).** Sort every stream: constant (a setting, or a "not configured"
flag), counter (+1 a row: `raw_n`), clock (`ms`), on/off, slow drift (battery), noisy continuous. Measure rest
level, noise and true update rate.

**Stage B, test motions** (after the drive model in section 6), chosen so that each type has a pattern no other has:

| Test motion | Gyro | Accel. | Wheel counts | Range / sonar ahead | Lidar | Beacon range | Line / cliff | Switch | Charging |
|---|---|---|---|---|---|---|---|---|---|
| 1. Still | about 0 | one axis reads gravity | flat | steady | steady scan | steady | steady | 0 | as is |
| 2. Spin left, then right (360 deg each) | one axis follows the turn; sign flips | small | equal and opposite | sweeps once per turn | whole scan rotates (scan matching gives the angle) | steady (bearing, if reported, rotates) | steady | 0 | as is |
| 3. Drive 20 cm forward and back | about 0 | pulse at start and stop along one axis | same sign | changes by the distance if facing a wall | scan shifts | changes with the distance to the beacon | steady | 0 | as is |
| 4. Back slowly into a wall | 0 | jolt at contact | stall while commanded | grows (front) as it backs away | wall approaches behind | small change | steady | 0 to 1 at contact | as is |
| 5. Approach an edge slowly (stopping on the first step of any stream) | 0 | 0 | steady | bottom zone jumps | (blind to drops) | small change | steps | 0 | as is |
| 6. Dock | 0 | jolt | stall | dock walls close | dock walls close | at the dock: known | steady | 1 | 0 to 1 |

**Scoring.** For each stream and candidate type, a match score from the pattern: correlation with the reference
in the motion where that type should respond, the ratio after integrating (a gyro integrated over a 720 deg spin
gives 720 x a unit factor), sign agreement between the left and right spins, step timing at contact. Confidence =
the best type's share of all scores and its lead over the second.

- **0.90 or more, clear lead:** labelled, used, listed for an optional glance.
- **0.60-0.90:** used only once confirmed; the user picks from the top two or three, each with its evidence chart.
- **Below 0.60:** "unknown", logged, kept out of the fusion.

These bars are chosen. **A design of experiments is the sensible way to set them:** replay the Alvik's logs with
names hidden at 0.5 / 0.6 / 0.7 / 0.8 / 0.9 and score wrong labels against questions asked.

<sub>Correlation: from −1 to 1, how closely two signals rise and fall together.</sub>

**With no camera.** The references are the board and dock touches (position at contact, from the dock's printed
dimensions), gravity, commanded motion (measured by whichever sensor is confirmed first), other robots (one
robot's range sensor or lidar watching another that is told to move), beacons once their places are learned, and
**agreement between independent sensors**: if the gyro, the wheel difference and the lidar's scan rotation agree
on a spin, all three are confirmed together; if one disagrees, it is the suspect. The dock is the absolute length:
the robot's position when docked is known from the dock's printed geometry, and two touches at known points of the
dock fix scale.

**Three traps [Claude].**
- **Derived streams** (`x_cm`, `y_cm`, `th_deg`, `v_cms`, `w_dps`, `roll_deg`, `pitch_deg`, `yaw_deg` on the Alvik;
  `Odometry` in ROS 2) pass every test a real sensor would. A stream the others predict almost exactly is tagged
  *derived*, never counted as separate evidence.
- **Sensors the motions do not exercise** (light, temperature, a microphone) stay unknown unless the user names
  them.
- **Beacons and lidar need the room to stay put** during discovery; a person walking past looks like a moving
  wall. Stage C catches it.

**Stage C, repeat** at the end of the run. A label whose score changes between passes goes back to a question.

---

## 8. User input: what is inferred, what is truly the user's

| Input | Inferred how | Truly user-only? | UI | Fallback if nobody answers |
|---|---|---|---|---|
| Which robots are ours | The ones that respond to our commands: a commanded wiggle seen by the camera or by the device's own sensors | No | Shown for a glance | Every responding device is ours; a device that does not respond is ignored |
| Is an edge a drop? | Edge and cliff sensors, during a slow approach that stops on the first step of any stream; with no edge sensor, robots stay inside the area already driven | No | Shown as the found border | Unseen edges are treated as drops |
| Walls safe to touch | Gentle contact at stall-limited force; a wall that moves when touched is not a reference | No (fragile things go under no-go areas) | none | No-go areas cover it |
| The dock and the absolute length | The dock's printed shape found by the camera, or by a robot's charging flag on a search; its printed dimensions give scale | No | Shown for a glance | Search slowly; scale tagged *guessed* until docked |
| Robot names | Auto (R1, R2 ... with light colours if any) | Preference only | Optional rename | Auto names |
| Sensor types and drive channels below 0.90 | Section 7 | Confirmation only | Card per item | Kept out of the fusion until confirmed |
| A shifted sensor | Re-learned (section 3) | No (Nick 11:05) | Noted in results | (no question) |
| Network or camera passwords | Cannot be inferred | **Yes** | Password box | That device is skipped |
| **Places robots must not go** | Intent | **Yes** | Paint areas on the camera frame, or draw on the map | Only the found border |
| **Consent to start moving** | Safety and Nick's rule (he approves calibration runs) | **Yes** | One Start button with the plan and time estimate | Fresh may start on a schedule; the others wait |
| Holding a robot during a level 3-4 drive test with no known stop | Safety | **Yes**, only then | Instruction with a countdown | That robot is not driven |
| Tape marks for grading | Truth for the paper, not for calibration | Yes, grading only | Place a robot when asked | Results say "camera-scored only" |

---

## 9. Instructions the system issues

Rules [Claude]: one action per instruction; name the object by what the user sees (its light colour, the ring on
screen); say what "done" looks like; **check it with a sensor or the camera instead of trusting a button press**;
when the check fails, say what was seen in one sentence and repeat the whole instruction.

| Instruction (example) | How it is checked | If the check fails |
|---|---|---|
| "Put the robot with blue lights anywhere inside the green outline, wheels on the floor, then step back." | Camera sees a new robot-sized object inside; accelerometer level, not lifted; no hand in the frame (MIRLT's "hand" class) | "I can see a hand near the bottom edge. Step back from the board so the robots can move." |
| "Drag the ring onto the robot that is turning now." | The spot agrees with the turn-and-look change within a robot's width | "That spot did not change when the robot turned. The robot turning now has blue lights; drag the ring onto it." |
| "Plug the dock's power cable into the wall socket." | The dock answers, or a docked robot's charging flag reads 1 | "The dock is not answering yet. Check the cable is in the dock and the socket is switched on." |
| "Keep a hand near the robot on the left of the picture: I will send it the smallest move I can." | The robot's response is seen; the stop is confirmed | "It did not stop when I sent stop. Switch it off with its power switch." |
| "Type the password for the camera named 'garage-cam'." | The stream opens | "That password was refused by garage-cam. Type it again." |
| "Stand clear of the board: the run starts in 10 seconds." | No moving object other than robots for 10 s | Countdown restarts; after three tries it asks whether to start anyway |

---

## 10. The setup screens (one place, tabs; laptop)

The same dark console look as the live view (`e02_view.py`: background `#0E1116`, panels `#151A22`, amber `#E0A33A`
for actions, IBM Plex Sans and Mono). One page with tabs; each tab shows a status dot (done, working, needs you).
Runs offline and reuses the live view's question channel (`e02_view.LiveView.ask`, `test_e02_webask.py`).

| Tab | What it shows | User action |
|---|---|---|
| 1. Setting | Fresh / From scratch / New system from scratch, pre-selected from what it found (old measurements exist and known robots answered = Fresh), with what each will measure and its time estimate | Pick; Start (consent) |
| 2. Devices | Everything found: cameras (thumbnails, pictures a second), robots and other devices, with their level (1-4) and adapter | Passwords if asked |
| 3. Drive | Per robot: the learned drive model (sign, scale, deadband, latency, turning), each with its provenance and a clip | Confirm low-confidence ones |
| 4. Sensors | A card per sensor group: type, confidence, provenance, a small chart of the stream against its reference in the motion that decided it | Confirm / Change / Ignore below 0.90 |
| 5. Work area | Camera frame or map: found border, edges, dock, robots (the mark-the-robot page, being built separately, sits here) | Paint no-go areas; drag a ring if identify was unsure |
| 6. Run | Live picture with overlays; stage timeline; the question queue; a large stop button | Nothing unless asked |
| 7. Errors and changes | Every flagged residual, its diagnosis, which references disagreed, and what was done (re-estimated, quarantined, re-tested); every shifted value with old and new | Read |
| 8. Results | Per robot and sensor: offset, scale, noise, latency, mount, trust, each with provenance | "Open live lab" |

---

## 11. What exists, what must be built (ordered by value)

| Rank | Piece | Exists | To build | Effort (Claude hours, a guess) |
|---|---|---|---|---|
| 1 | Provenance tags on every value, fixes, logs and reports | The heading report's fixes 1-3 (designed, not applied) | Tag type and rules; apply the heading fixes | 6-8 h |
| 2 | Blind discovery replay on existing logs (names hidden), plus the heading logs as an error-check test set | All the logs | Features, scores, confusion table | 8-12 h, logs only |
| 3 | General stream model (predict, residual, offset + scale, noise less reference, front rule, trust) | `e02_front.py`, `e02_trust.py`, `tof_calib.py` | One class per stream; scale term; provisional baselines | 10-14 h |
| 4 | Error engine (identify, diagnose, adapt) | Trust flags, `heading_diag.py`, camera drift watch | Hypothesis checks per stream, attribution by reference, the adapt ladder | 10-14 h |
| 5 | `RobotDriver` interface and adapter library (Alvik first) | Audit R1 (not built) | Interface, Alvik adapter, ROS 2 adapter | 14-22 h (audit's figure) |
| 6 | Device discovery (cameras, serial, mDNS, broadcasts, BLE list, ROS 2 graph) | Alvik hello listener, Pi camera streams | Scanners and the Devices tab data | 8-12 h |
| 7 | Cautious drive identification | `identify`, firmware calib run | Stop-first sequence, growing steps, model fit | 8-12 h |
| 8 | Test-motion battery as a runner stage | Calib run, backing find with switches (2026-10-25b), lights | Edge approach, docking motion, repeat pass | 6-8 h |
| 9 | Mount, latency and units estimators | `tof_calib` zone directions, capture delay tool, compass wiggle | IMU axis map, edge offsets, switch side, unit snapping, lag | 8-12 h |
| 10 | Setup screens (tabs 1-8) | Live view look and question channel; the mark page (separately) | The tabs | 16-24 h |
| 11 | Robot-side agent protocol (documented standard) | The Alvik firmware's protocol | A written spec and a reference agent | 4-6 h spec; agent per board |

Total about 98-144 h. Ranks 1-3 (24-34 h) prove the idea on real data before any screen is built.

### Risks

| Risk | Effect | What reduces it |
|---|---|---|
| Explore has never finished on real hardware | From scratch with a camera cannot be shown | Finish one real explore with marks before the screens |
| Sending commands to an unknown device | A robot moves unexpectedly or will not stop | Stop-first rule; tiny steps; a hand on the power switch at levels 3-4 |
| Circular references (camera taught by robots, robots by the camera) | A shared error looks like the reference's (`shared_unexplained` says it cannot tell) | Touches at several walls; the dock as an absolute length |
| A new robot falls off before its edge sensors are known | Damage | Unseen edges are drops; crawl until a cliff stream is confirmed |
| The classifier learns the Alvik, not sensors | Fails on the first other robot | Disguised-log replays; a ROS 2 simulator robot (TurtleBot3 in Gazebo) as a second test |
| Only one robot | No sibling comparison in the first session | Say so; provisional baselines |
| Campus networks block discovery | Robots not found | Manual address entry as a fallback (a user input, but rare) |

---

## 12. What's next

1. Nick decides on the heading fixes 1-3 (the report's), since provenance rank 1 includes them.
2. Build ranks 2 and 3 (logs-only discovery replay and the general stream model): no board time.
3. Finish one real explore with marks (the board-time prerequisite for From scratch).
4. Then the error engine, the driver interface and the screens.

---

## New tests that may now be needed

1. **Blind discovery on Alvik logs:** names hidden; score correct types, questions asked and wrong labels (sets the
   0.60 / 0.90 bars).
2. **Disguised-sensor replay:** columns rescaled (x10, x57.3), sign-flipped, delayed 100 ms, axes swapped; units,
   signs, latency and axis map must all be recovered.
3. **Derived-stream test:** `th_deg`, `x_cm`, `yaw_deg` must be tagged derived.
4. **Provenance test:** no value tagged guessed or copied is ever written into a robot or used to judge another
   sensor (unit tests on every fix and trust path; the heading report's tests 1-3).
5. **Error-engine replay on the heading logs:** given only the logs, the engine must attribute the large heading
   errors to the fixes, not the gyro.
6. **Swap test per stream:** degrade one stream in replay (offset, scale, noise x2, stuck); readings until flagged
   under the front rule against the current decaying trust.
7. **Front scale term:** do the 115 front residuals grow with distance?
8. **Drive identification in the simulator:** a robot with flipped sign, 3x scale, a deadband and 200 ms latency
   must be learned without leaving a safe radius.
9. **No-camera discovery in the simulator:** IMU, wheels, a range sensor and the dock only; labels and scale.
10. **ROS 2 robot in a simulator** (TurtleBot3 in Gazebo): devices, drive and sensor types found end to end.
11. **One full real explore with marks**, then From scratch end to end.
12. **Instruction checks:** each instruction tried wrongly on purpose (hand in frame, wrong ring) must be caught.

## Possible inaccuracies

- **Run times** (section 1, Appendix C) are worked out from constants, not timed; explore has never finished on
  real hardware, and the device discovery and drive identification times are guesses.
- **Effort estimates are guesses**, made the same way as the audit's.
- **Chosen numbers:** the 0.60 / 0.90 bars, the 15% unit-snap window, 30 s still periods, 360 deg spins, 20 cm
  drives, the doubling step in drive identification. A design of experiments (blind replay at several values) is
  the sensible way to set the bars and the snap window; the simulator drive tests can set the step size.
- **The test-motion table is predicted, not measured**: the accelerometer's contact jolt is weak on a light robot,
  and the Alvik's IMU units are still "to be confirmed on hardware" (`data-formats.md`).
- **The discovery table** (mDNS, ONVIF, ROS 2, BLE, MAVLink) is from general knowledge of those standards, not from
  tests on this lab's network or this Pi. Campus networks often block broadcasts.
- **The 0.8 cm camera and 1-2 cm touch figures** come from `e02_trust.py` and the publication plan; the touch was
  never measured directly.
- **The dock as the absolute length** assumes its printed dimensions are accurate after printing and that the robot
  seats the same way each time; neither is measured yet.
- **The decaying-trust finding** is reasoned from `new_session`'s code, not seen in a session; test 6 checks it.
- **The heading figures** (112 of 663, 100 of 112) are the heading report's, which takes the camera's direction of
  travel as truth (about ±8 degrees sd on an 8 cm leg).

---

## Appendix A. Discovery features per stream

| Feature | From | Separates |
|---|---|---|
| Rest mean and spread | Stage A | Gravity axis (about 1 g or 9.81) from other IMU axes; constant flags |
| Distinct-value rate | Stage A | Real update rate |
| +1 a row / rises with time | Stage A | Row counter; clock; battery (slow, falling) |
| Correlation with the measured turn rate | Motion 2 | Gyro yaw axis and sign |
| Integral over the spin / measured angle | Motion 2 | Gyro units and scale |
| Periodic at the spin's period | Motion 2 | Range and sonar sensors; their phase gives direction |
| Whole-profile rotation | Motion 2 | Lidar; scan matching gives the angle |
| Opposite during a spin, same driving | Motions 2-3 | Wheel encoders, which side |
| Pulse at start and stop | Motion 3 | Accelerometer along the drive |
| Change / measured distance | Motions 3-4 | Range, sonar, beacon scale and units; wheel scale |
| 0 to 1 at the stop | Motion 4 | Switch; which one by approach angle |
| Step at the edge | Motion 5 | Line and cliff sensors |
| 0 to 1 when docked | Motion 6 | Charging |
| Predicted almost exactly by other streams | All | Derived |

## Appendix B. The front-sensor numbers this generalises

From `e02_front.py`: 115 readings (2026-10-23/24), median −1.1 cm, spread 5.5 cm; per robot −3.4 / −0.9 / +1.2 cm;
this session's own after 8 readings; tolerance 3 sd, at least 3 cm; a shift is reported beyond max(3 cm, 3
standard errors). From `e02_trust.py`: healthy sds camera 0.8 cm, wheels 0.6 cm, gyro 1.0 deg, front 5.0 cm; flag
at k 2, clear below 1.5; evidence weight halves each session.

## Appendix C. How the run times were estimated

- **Fresh:** connect and health 1 min; per robot a 10 s still check, a 62-65 s calib run and one find (1-2 min);
  3 robots one at a time: 8-12 min.
- **From scratch:** explore per robot 8 rays of up to 105 cm at 4 cm/s out and 10 cm/s back (about 40 s a ray,
  5-6 min) plus its square (about 1 min), 3 robots about 20 min; calib runs 3 min; finds at two walls per robot
  6-10 min; marks and checks 3-5 min: 30-40 min.
- **New system from scratch:** device discovery 2-3 min; drive identification 3-4 min per robot (10-12 min);
  stage A 1 min per robot; the six motions 4-5 min per robot (12-15 min); then From scratch 30-40 min; the repeat
  pass about 10 min; confirmations 2-5 min: 70-85 min. Rounded up to 75-90 min for retries.
