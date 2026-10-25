# Self-calibrating a sensor array it has never seen: design assessment (2026-10-25)

**Premise.** Nick, 2026-10-25: "The from scratch setting should work in a new system with different sensor types.
We need to apply the logic you describe for front sensor applying to new unknown sensors. Do an assessment of
everything the system would need in user input for an offline system to self-calibrate an array of sensors,
identify and label anything and issue instructions. The UI for this will likely precede the live lab."
He named three settings (a known system with history; from scratch on a known system; a new system from scratch)
and wants every user input made at the laptop (marking the robot in camera frames), not with a tape on the floor.

This is a design assessment only. No code was changed, and nothing below is built until Nick approves it. Every
conclusion in this document is Claude's (marked **[Claude]** where it is a recommendation or a judgement); the
facts about the existing code are cited by file.

**Data read.** `tools/experiment/e02_front.py`, `e02_trust.py`, `e02_fit.py`, `e02_explore.py`, the runner's setup,
identify and question code (`e02.py`, `e02_view.py`), `tools/fusion/tof_calib.py`, `firmware/home_lab/rawlog.h`
(the RAW record), `docs/data-formats.md`, `docs/build-plans/generality-audit.md`, `publication-plan.md`,
`live-lab-outline.md`, `generic-platform-outline.md`, the 2026-10-25 reverse-find build report, and the MIRLT
recognition and labelling READMEs.

---

## Conclusion, in short

1. **The three settings are one engine with three starting points [Claude].** Setting 3 (discover) adds a
   "what is this stream?" stage in front of setting 2; setting 2 (known kit) measures everything fresh with only
   generic starting guesses; setting 1 is setting 2 plus a comparison with saved history. Build one pipeline, not
   three.
2. **The setting belongs to each sensor, not to the whole session [Claude].** A known Alvik with one swapped or
   newly added sensor is setting 1 for its wheels and gyro and setting 3 for the new sensor. The engine keeps a
   state per stream: *unknown*, *known type, no history*, *known with history*, *changed*.
3. **Recommended name for setting 1: "Fresh vs history"** (candidates in section 1). Settings 2 and 3 become
   **"From scratch: known kit"** and **"From scratch: discover"**, so "from scratch" always means "no old data".
4. **The front-sensor logic generalises to any stream** as one recipe: predict what the stream should read from
   a reference the system already trusts, take the difference, learn offset and scale from it, take the
   reference's own noise off, compare with history, and turn the result into a trust factor (section 2).
5. **Discovery (setting 3) is feasible for any sensor that responds to motion, contact, gravity or the camera**,
   using six commanded test motions whose responses do not overlap (section 3). A sensor that responds to none
   of them (a temperature or light sensor) stays "unknown" and is kept out of the fusion.
6. **Only two things truly need a person [Claude]:** one known length (absolute scale: a spec-sheet number typed
   at the laptop, not a floor measurement), and intent (which robots are ours, where they may not go, whether
   an edge is a drop, whether a sensor was swapped on purpose). Everything else is a confirmation the system can
   do without, with a stated fallback (section 4).
7. **Setting 3 still needs a known way to drive the robot.** A system cannot safely discover how to move a robot
   it has never seen. "New system" here means unknown sensors and cameras on a robot that speaks a known driver
   (the audit's `RobotDriver`, R1). This is an assumption; if Nick wants truly unknown robots, the user would
   supply a driver file, and that is a separate design.
8. **The cheapest proof is free data [Claude]:** the Alvik's 37 RAW columns, with their names hidden, are a
   labelled test set for discovery. A logs-only replay can score the classifier before any board time.
9. **Biggest risk: no from-scratch session has finished on real hardware yet.** The one real explore (2026-10-24
   stage 3) stopped during identify. Setting 2 and 3 both stand on explore.

---

## 1. The three settings

### Names for setting 1 (fresh raw data compared against old data)

| Candidate | Says "fresh" | Says "against old data" | Short enough for a button | Note |
|---|---|---|---|---|
| **Fresh vs history** | yes | yes | yes | Reads as exactly what it does. **Recommended [Claude]** |
| Fresh check | yes | implied ("check" against what?) | yes | Shortest; loses the "old data" half |
| Compare to history | no | yes | yes | Does not say the data are new each time |
| Re-baseline | no | partly | yes | An engineering term, but suggests old data are thrown away, which they are not |
| Recheck against record | yes (re-) | yes | long | Clear, clunky on a phone |

"Fresh vs history" keeps the principle already in `e02_front.py`: this session's own readings are used as soon as
there are enough; history is only the starting guess and the yardstick for "something changed", never a
substitute for measuring.

### What each setting knows, measures and asks

| | **Fresh vs history** (setting 1) | **From scratch: known kit** (setting 2) | **From scratch: discover** (setting 3) |
|---|---|---|---|
| Knows at the start | Robot and sensor list, each sensor's type, mount and units; every past session's calibrations, noise and trust | Robot and sensor list, types, mounts, units (the robot profile); generic starting guesses only | A driver to move each robot; that a camera is somewhere; nothing about the sensors |
| Does not use | Nothing is skipped: every value is measured again | Any saved calibration, noise, trust or map | Column names (used only as a hint), any profile |
| Measures | Each sensor's offset, scale, noise, latency against the camera and touches, fresh; compares with history and flags shifts | The same, plus the camera (explore), the work area's edges and every robot's wheel and gyro scale | First: which streams exist, how fast, at rest and in motion, and what each one is; then everything setting 2 measures |
| Asks the user | Approve the run; "was this swapped on purpose?" only when a shift is found | Approve; confirm or drag each robot's ring; drops at the edges; no-go areas | All of setting 2, plus one known length, and a confirmation for each sensor guess below the confidence bar |
| Calibration run (3 robots, estimate) | 8-12 min before trials; the comparison keeps going through the session | 30-40 min | 60-70 min, plus 2-5 min of the user's answers |

Run times are estimates from the code's constants (Appendix C), not timed sessions. Nick has said accuracy and
autonomy matter more than run time, so setting 3 repeats its test motions at the end of the run as a
repeatability check (section 3), which costs about 10 extra minutes.

---

## 2. A general sensor model, and how each part is learned

Every sensor stream gets the same record. Each entry is estimated from data against a **reference the system
already trusts**, in this order of strength: a board touch (switch click, 1-2 cm), the camera after explore
(0.8 cm per axis at a find, `e02_trust.HEALTHY`), physics (gravity reads 1 g at rest; a still robot is not
turning), the other robots (what all share is the camera's, what one has on top is its own), and the motion the
system commanded (sign and timing are known, size only roughly).

| What the system needs | How it is estimated | Reference | Test motion |
|---|---|---|---|
| Identity | Which robot sent it, which column; streams that update at the same instants and move together are grouped into one sensor (the front sensor's 5 zones, the IMU's 3 axes) | Timestamps | Any |
| Type | Section 3 (discovery) | All | All six |
| Units | The ratio between the stream and its reference is snapped to the nearest unit factor (1, 10, 100, 1000, 57.3, 9.81) when within 15%; the remainder is a calibration scale | Camera, gravity | Still, spin, drive |
| Rate | Count only rows where the value changes: RAW repeats the front sensor's last frame (about 15 a second inside 50 rows a second) | Timestamps | Still |
| Noise | Robust spread at rest and about its prediction in motion, minus the reference's own share (`front_sd`: sqrt(sd² − camera sd²)) | Camera, siblings | Still, drive |
| Bias (offset) | Median of reading minus prediction (`e02_front`: −1.1 cm across 115 readings) | Camera + map, gravity, still | Still, approach |
| Scale | Slope of reading against prediction (robust regression). **Not in `e02_front` today**: a median offset hides a sensor that reads 5% long, which shows as error growing with distance. `tof_calib.py` already fits a range scale offline | Camera, gravity | Spin (gyro), drive (wheels), approach (range) |
| Latency | The time shift that best lines the stream up with its reference at the sharp start and stop of a commanded spin or drive | Camera (capture delay measured by `tools/live/capture_delay.py`), commands | Spin, drive |
| Position and direction on the robot | Range: where the boards must be for the readings to fit while spinning (`tof_calib` zone directions). IMU: gravity at rest gives "down", the spin gives the yaw axis and its sign. Line or cliff: where the camera puts the robot when the reading steps at a known edge. Switch: which side clicks first on an angled approach, and how far from the axle the board line is at the click | Camera, gravity, map | Still, spin, approach, edge |
| Valid range and field of view | Largest and smallest readings that still agree with the prediction; zone directions from the spin | Camera + map | Spin, approach |
| Failure signs | Stuck (no change while the reference changes); dropouts (missing rows, a sentinel such as −1); clipping at a limit; noise above its own baseline (trust factor k = learned sd ÷ baseline sd ≥ 2, `DEGRADED_K`); offset shift beyond max(3 cm, 3 standard errors) (`e02_front`); disagreement only in one region or speed | All | All, continuously |

<sub>Robust spread: MAD × 1.48 (the median distance from the median, scaled so it equals the standard deviation for
normal data); few outliers barely move it. Standard error: how far the median of n readings is likely to be from
the true value, about 1.25 × sd ÷ √n.</sub>

### The worked example, generalised

The front sensor today (`e02_front.py`, `e02_trust.py`) does five things. Written for any stream:

1. **Predict.** From the reference (camera position + map), what should this stream read now? For the front
   sensor: the distance to the board ahead. For a gyro: the camera's turn rate. For a switch: 1 when the camera
   and map put the robot's back face on a board line.
2. **Residual.** Reading minus prediction, kept only where the prediction is trustworthy (square on, measured
   heading, in range: `front_check`).
3. **Learn offset, scale and noise.** Until this session has N residuals (8 for the front sensor), use history
   (setting 1) or a generic starting guess (settings 2 and 3); from N on, this session's own. Take the
   reference's share off the noise.
4. **Compare with history** (setting 1 only). A shift beyond max(minimum, 3 standard errors) is reported in plain
   words, and this session's value is used.
5. **Trust.** k = learned noise ÷ baseline noise; flag at 2, clear below 1.5. With two or more robots, flag only a
   robot that stands out from its siblings (`STANDS_OUT`); never blame a sensor when the reference itself is
   unknown or flagged (`update_flags`).

Three gaps when this is applied to sensors it does not know **[Claude]**:

- **No "healthy" value exists for a new sensor.** `e02_trust.HEALTHY` holds Alvik numbers (audit E6). In settings
  2 and 3 the baseline must be the session's own first stretch (marked provisional) or the same sensor type on the
  other robots. With one robot and a new sensor, the system can say "agrees with the camera within X" but not
  "worse than normal" until a second session exists.
- **Trust mixes history in; the front sensor does not.** `SensorTrust.new_session` halves old evidence's weight
  but keeps it, so in the first session after a swap, 20 old items at half weight outweigh 5 new ones. The front
  rule (this session's own after N) catches a swap at once. The general model should use the front rule for
  every stream, and keep history only as the prior and the yardstick.
- **Offset only is not enough.** Add the scale term (step 3) so a sensor that reads long in proportion is caught.

---

## 3. Sensor discovery (From scratch: discover)

**Input:** every raw stream the robot sends (for an Alvik, the 37 RAW columns in `rawlog.h`), each with a name
that may or may not mean anything. Names give a starting hint ("gz", "_cm"), shown to the user, but the guess comes
from behaviour.

**Stage A, passive (robot standing still, 30 s, twice).** Sort every column into: constant (a setting or a
"not configured" flag), counter (+1 a row: `raw_n`), clock (rises with time: `ms`), on/off, slow drift (battery),
noisy continuous. Measure rest level, rest noise and true update rate.

**Stage B, six test motions.** Chosen so each sensor type has a pattern no other type has:

| Test motion | Gyro | Accelerometer | Wheel counts | Range ahead | Line / cliff | Switch | Lift flag | Charging |
|---|---|---|---|---|---|---|---|---|
| 1. Stand still | about 0 (its bias) | one axis reads gravity | flat | steady | steady | 0 | 0 | as is |
| 2. Spin left, then right (360 deg each) | one axis follows the camera's turn, sign flips | small | equal and opposite | sweeps once per turn | steady on a uniform floor | 0 | 0 | as is |
| 3. Drive 20 cm forward and back | about 0 | a pulse along one axis at each start and stop | same sign, matches camera distance | changes by the distance driven if facing something | steady | 0 | 0 | as is |
| 4. Back slowly into a board the camera sees | 0 | a jolt at contact | stop while commanded (stall) | grows as it backs away from what is ahead | steady | steps 0 to 1 at the moment the camera stops it | 0 | as is |
| 5. Drive slowly toward an open edge (only if the user says edges are drops, at crawl speed, stopping on the first step change) | 0 | 0 | steady | bottom zone jumps | steps | 0 | 0 | as is |
| 6. Lights on and off; lift (asks the user); dock (if one exists) | lift: noisy | lift: off-gravity | lift: wheels may spin | lift: bottom zone jumps | lift: jumps | 0 | lift: 1 | dock: 0 to 1 |

**Scoring.** For each stream and each candidate type, a match score from the pattern above: correlation with the
reference in the motion where that type should respond, the ratio after integrating (a gyro integrated over a
camera-measured 720 deg spin gives 720 × a unit factor), sign consistency between left and right spins, and step
timing at a contact. Confidence = the best type's share of all the scores, and its lead over the second.

- **0.90 or more, and a clear lead:** labelled "likely X", listed for one-tap confirmation, used provisionally.
- **0.60-0.90:** the user picks from the top two or three, each shown with its evidence chart.
- **Below 0.60:** "unknown"; kept in the logs, kept out of the fusion. The user may name it, but it is used only
  once a reference exists to check it against.

These bars are chosen, not measured. **A design of experiments is the sensible way to set them:** replay the
Alvik's labelled logs with names hidden at bars of 0.5 / 0.6 / 0.7 / 0.8 / 0.9 and score wrong-label rate against
the number of questions asked.

<sub>Correlation: a number from −1 to 1 saying how closely two signals rise and fall together; near ±1 means one
tracks the other.</sub>

**Two traps the discovery must handle [Claude].**

- **Derived streams.** `x_cm`, `y_cm`, `th_deg`, `v_cms`, `w_dps`, `roll_deg`, `pitch_deg`, `yaw_deg` are worked
  out on the robot from the wheels and the IMU. They pass every test a real sensor would. A stream the others
  predict almost exactly (for example `th_deg` against the integrated gyro and wheel difference) is labelled
  "derived" and never counted as separate evidence, or the fusion would trust the same information twice.
- **Inputs that the motions do not exercise.** A light or temperature sensor, or a microphone, will not respond
  to any of the six motions. It stays unknown unless the user names it; that is the honest answer.

**Stage C, repeat.** The six motions run again at the end of the calibration. A label whose score changes between
the two passes is reopened as a question. This costs about 10 minutes and buys the accuracy Nick prefers to speed.

---

## 4. Every point of user input, minimised and moved to the laptop

| # | What is asked | Why it can't be inferred | UI element | When | Fallback if nobody is there | Settings |
|---|---|---|---|---|---|---|
| 1 | Approve the calibration run | Nick approves calibration runs before trials (his rule); motion near people needs consent | One "Start" button with the plan and time estimate | Start | Fresh vs history may start on a schedule; the others wait | 1, 2, 3 |
| 2 | Which robots are ours | A hello on the network could be any device (a neighbour's, a spare) | List of robots that answered, with checkboxes | Connect | Saved list (setting 1); otherwise wait | 1, 2, 3 |
| 3 | Is any edge of the work area a drop? | Finding out by driving off it breaks the robot | Yes / No / Not sure | Work area | "Not sure" = treat every edge as a drop: crawl speed and stop on the first edge-sensor step | 2, 3 |
| 4 | Places the robots must not go | Intent (a laptop on the floor, a fragile model) | Paint areas on the camera frame | Work area (optional) | Explore's own border with a margin; front-sensor obstacle stop | 2, 3 |
| 5 | May robots touch the walls and boards? | Fragility and safety of what is there | Yes / No | Work area | No: the camera and edges only, no touch references (fewer and weaker references, flagged in the report) | 2, 3 |
| 6 | Mark each robot (drag the ring onto it) | When the camera model is unknown, the turn-and-look guess (`identify`) can pick a shadow or a second change | The mark-the-robot page: guess shown as a ring, drag to fix, tap to confirm | After connect | Identify by turn, then by lights (both exist); retry; a robot still unfound sits out | 2, 3 (1 only if identify fails) |
| 7 | One known length: the robot's width or wheel diameter from its spec sheet | Absolute scale. Wheels, gyro and camera can all agree with each other at the wrong size; explore assumes the robots' wheels average to correct and checks with the robot's known size (6% apart from the clicks on 2026-10-23) | Number box with units, plus a picture of which length | Before the run | Use the robot's own reported cm (if its firmware reports cm) and flag "scale not confirmed" in every result | 3 (2 reads it from the profile) |
| 8 | Confirm sensor guesses below 0.90 | Two types can look alike in a short test; the person knows what was plugged in | One card per sensor group: guess, confidence, evidence chart; Confirm / Change / Ignore | During or after the run, as a queue | Below 0.90: kept out of the fusion until answered; logs kept | 3 |
| 9 | Lift a robot for 3 seconds | Only a hand can do it; it is the only way to find a lift sensor | Instruction with a countdown | End of the motion battery (optional) | Skipped; a lift flag stays unknown | 3 |
| 10 | Was this sensor swapped on purpose? | Intent: a planned swap starts a new history; an unplanned one is a fault to report | Yes (start new history) / No (report as a fault) | When a shift is found | This session's value is used either way (front rule); both histories kept; the question waits in the report | 1 |
| 11 | Robot names | Human preference | Text box beside each robot's light colour | Connect (optional) | Auto names (R1, R2, ...) | 1, 2, 3 |
| 12 | Where the dock is (if any) | A dock is an unknown object to the camera in setting 3 | Drag a box onto it in the frame | Work area (optional) | No charging this session | 2, 3 |
| 13 | Tape marks for grading | Truth for the paper, not needed to calibrate | Place a robot on each mark when asked | Separate grading stage | Skipped; results say "camera-scored only" | Grading only |

**What is not asked, though one might expect it [Claude]:** camera choice (the camera where robots move when
commanded is the one that sees the work area); camera height or lens (explore fits them); board size and edges
(explore); sensor positions on the robot (section 2); time sync (clock records and light flashes, existing).

---

## 5. Instructions the system issues

Rules for wording **[Claude]**: one action per instruction; name the object by what the user sees (its light
colour, the ring on screen), never by an internal name; say what "done" looks like; and **check it with a sensor
or the camera instead of trusting a button press**. When the check fails, say what was seen, in one sentence, and
repeat the full instruction (Nick's rule: never "see step 4").

| Instruction (example wording) | How it is checked | If the check fails |
|---|---|---|
| "Put the robot with blue lights anywhere inside the green outline, wheels on the floor, then step back." | The camera sees a new robot-sized object inside the outline; accelerometer level and not lifted; no hand in the frame (MIRLT's "hand" class) | "I can see a hand near the bottom edge. Step back from the board so the robots can move." |
| "Drag the ring onto the robot that is turning now." | The dragged spot agrees with the turn-and-look change within a robot's width | "That spot did not change when the robot turned. The robot turning now has blue lights; drag the ring onto it." |
| "Plug the dock's power cable into the wall socket." | The dock answers, or a docked robot's charging flag reads 1 | "The dock is not answering yet. Check the cable is in the dock and the socket is switched on." |
| "Lift the robot with blue lights about a hand's height and hold it for 3 seconds." | Lift flag or accelerometer off gravity, for at least 2 s | "I did not feel a lift. Lift the robot with blue lights straight up about a hand's height and hold it there for 3 seconds." |
| "Type the robot's width from its spec sheet, in cm or inches." | Within 0.5-3x the camera's own estimate | "That is far from what the camera sees (about 10 cm). Check the number and the units." |
| "Stand clear of the board: the run starts in 10 seconds." | No moving object other than robots for 10 s | Countdown restarts; after three tries it asks whether to start anyway |

---

## 6. The setup wizard (precedes the live lab)

The same dark console look as the live view (`e02_view.py` tokens: background `#0E1116`, panels `#151A22`, amber
`#E0A33A` for actions, IBM Plex Sans and Mono). One column on a phone, two on a laptop. Questions arrive as a
queue that can be answered from either; a phone at the board is where the person placing robots will be. It runs
offline on the laptop, and reuses the live view's question channel (`e02_view.LiveView.ask`, tested in
`test_e02_webask.py`).

| Screen | What it shows | User action |
|---|---|---|
| 0. Choose a setting | Three cards: Fresh vs history / From scratch: known kit / From scratch: discover. The system pre-selects one (history exists and known robots answered = Fresh vs history) and marks it recommended. Each card states what it will measure and the time estimate | Pick one |
| 1. Connect | Cameras found (thumbnails and pictures a second); robots that answered (light colour, battery, rows a second, losses) | Tick our robots; name them (optional) |
| 2. Work area | Live camera frame | Drops? (Yes/No/Not sure); touch allowed?; paint no-go areas and the dock (optional) |
| 3. Mark the robots | The mark-the-robot page (being built separately): one robot turns and lights up, its ring is shown | Drag and confirm, one robot at a time |
| 4. One known length (discover only) | Picture of the robot with the length to enter | Type it |
| 5. Calibration run | Live frame with overlays; a stage timeline (still, spin, drive, explore, finds, repeat); one card per sensor filling in as it is learned; the question queue badge | Nothing unless a question arrives; a large stop button always |
| 6. Review sensors | A card per sensor group: type, confidence, a small chart of the stream against its reference during the motion that decided it; low-confidence cards open, others folded | Confirm / Change / Ignore |
| 7. Results | Per robot and sensor: offset, scale, noise, latency, position on the robot, trust; in Fresh vs history, what changed since last time | Answer "swapped on purpose?" if asked; "Open live lab" |

---

## 7. What exists, what must be built (ordered by value)

| Rank | Piece | Exists | To build | Effort (Claude hours, a guess) |
|---|---|---|---|---|
| 1 | Blind discovery replay on existing logs (Alvik RAW with names hidden): stage A and B features, scores, confusion table | All the logs; nothing of the classifier | Feature extraction, scoring, report | 8-12 h, logs only, no board time |
| 2 | General stream model (predict, residual, offset + scale, noise less reference, history compare, trust) with a state per stream | `e02_front.py`, `e02_trust.py`, `tof_calib.py` | Merge into one class per stream; front rule for history; scale term; provisional baselines | 10-14 h |
| 3 | Test-motion battery as a runner stage | Firmware calib run (still, spin both ways, roll), `identify`, backing find with switches (2026-10-25b), lights | Approach-to-edge crawl; lift prompt; repeat pass; records per motion | 6-8 h |
| 4 | Setup wizard (screens 0-7) | Live view look, question channel, the mark page (separately) | The screens and the question queue | 16-24 h |
| 5 | Mount, latency and units estimators | Zone directions in `tof_calib`, capture delay tool, IMU sign check (compass wiggle) | IMU axis map, line-sensor offset at an edge, switch side and face distance, unit snapping, lag by cross-correlation | 8-12 h |
| 6 | Robot profile with where each value came from (confirmed / history / discovered / default) and `site.json` | Audit rank 1 (not built) | As the audit, plus the provenance tag | 8-10 h (audit's figure) + 2 h |
| 7 | Derived-stream detector | Nothing | Predict each stream from the others; flag near-perfect fits | 3-4 h |
| 8 | `RobotDriver` interface (needed before any non-Alvik robot) | Audit R1 (not built) | As the audit | 6-10 h |

Total about 67-96 h. Ranks 1 and 2 alone (18-26 h) would show that the idea works on real data before any UI.

### Risks

| Risk | Effect | What reduces it |
|---|---|---|
| Explore has never finished on real hardware | Settings 2 and 3 cannot be shown | Finish one real explore with marks before building the wizard |
| Absolute scale | Everything right in shape, wrong in size (6% seen on 2026-10-23) | Input 7; tape marks for grading |
| Circular references (camera taught by robots, robots by camera) | A shared error all robots have looks like the camera's (`shared_unexplained` already says it cannot tell) | Touch references at several walls; the known length |
| A new robot falls off before its edge sensors are known | Damage | Input 3 defaults to "drop"; crawl speed until a cliff stream is confirmed |
| The classifier learns the Alvik, not sensors | Fails on the first other robot | Test on a second robot or the simulator with renamed, rescaled and re-mounted columns |
| One robot only | No sibling comparison, so no "this one is worse" in the first session | Say so in the results; provisional baselines |
| The person is not there | Questions stall the run | Every question has a fallback (section 4) and the run continues on it |

---

## 8. What's next

1. Nick answers the questions below (names, scope of "new system", the known length).
2. Build rank 1 (blind discovery replay on existing logs): logs-only, so it can run without board time.
3. Finish one real explore with marks (the board-time prerequisite for settings 2 and 3).
4. Build rank 2, then the wizard (rank 4) once the mark page is in.

---

## New tests that may now be needed

1. **Blind discovery on Alvik logs:** names hidden, every column labelled by behaviour; score correct types,
   questions asked and wrong labels (sets the 0.60 / 0.90 bars).
2. **Disguised-sensor replay:** the same logs with columns rescaled (×10, ×57.3), sign-flipped, delayed 100 ms and
   with axes swapped; units, signs, latency and axis map must all be recovered.
3. **Derived-stream test:** `th_deg`, `x_cm`, `yaw_deg` must be labelled derived, not independent.
4. **Swap test per stream:** degrade one stream in replay (offset, scale, noise ×2, stuck) and measure how many
   readings until it is flagged, under the front rule and under the current decaying-trust rule.
5. **Front scale term:** do the 115 front residuals grow with distance? If so, the scale term changes the offset.
6. **One full real explore with marks**, then From scratch: known kit end to end.
7. **Repeatability of discovery:** the two test-motion passes in one session must give the same labels.
8. **Wizard dry run with no person present:** every question must fall back and the run must finish.
9. **Instruction checks:** each instruction in section 5 tried wrongly on purpose (hand in frame, wrong ring, no
   lift) must be caught.

## Possible inaccuracies

- **Run times** (section 1, Appendix C) are worked out from constants (crawl speed 4 cm/s, 8 rays of up to 105 cm,
  62 s calib runs), not timed. Real explore has never finished, so its time is the least certain.
- **Effort estimates are guesses**, made the same way as the audit's (not checked against a time tracker).
- **Confidence bars 0.60 / 0.90, the 15% unit-snap window, 30 s still periods, 360 deg spins and 20 cm drives are
  chosen.** A design of experiments (the blind replay at several values) is the sensible way to set the bars and
  the snap window; the motion sizes could be set the same way against label accuracy.
- **The six-motion table is predicted, not measured.** In particular the accelerometer's contact jolt is weak on
  a light robot (the find uses stall and switches first), and the Alvik's IMU units are still "to be confirmed on
  hardware" (`data-formats.md`).
- **The 0.8 cm camera and 1-2 cm touch figures** come from `e02_trust.py` and the publication plan; the touch
  figure was never measured directly (the trust docstring says about a millimetre, the analysis says 1-2 cm).
- **The decaying-trust finding** (old evidence outweighs new after a swap) is reasoned from `new_session`'s code,
  not seen in a session; test 4 checks it.
- **"No sensor needs a tape measure"** assumes the robot's spec-sheet size is right and the floor is flat (audit
  A6).
- **The derived-stream list** is from the RAW header and the firmware's library calls; the Alvik library's own
  fusion of `yaw_deg` was not read.
- **Setting 3's scope** (a known driver) is an assumption pending Nick's answer.

---

## Appendix A. Discovery features per stream (detail)

| Feature | Computed from | Separates |
|---|---|---|
| Rest mean and spread | Stage A | Gravity axis (mean about 1 g or 9.81) from other IMU axes; constant flags |
| Distinct-value rate | Stage A | Real update rate (front sensor about 15 a second inside 50 rows) |
| Monotonic step +1 | Stage A | Row counter (`raw_n`) |
| Monotonic with time | Stage A | Clock (`ms`); battery (slow, falling) |
| Correlation with the camera's turn rate | Motion 2 | Gyro yaw axis; its sign from left vs right |
| Integral over the spin ÷ the camera's angle | Motion 2 | Gyro units (1 = deg/s, 1/57.3 = rad/s) and scale |
| Periodic at the spin's period | Motion 2 | Range sensors; their phase gives their direction |
| Opposite signs left and right during a spin, same sign driving | Motions 2-3 | Wheel encoders, which side each is |
| Pulse at start and stop of a drive | Motion 3 | Accelerometer along the drive; its sign |
| Change ÷ camera distance | Motion 3-4 | Range scale and units; wheel scale |
| 0 to 1 step at the camera's stop | Motion 4 | Switch; which one by approach angle |
| Step at the edge | Motion 5 | Line and cliff sensors; bottom range zone |
| Responds only to the lift | Motion 6 | Lift flag |
| Predicted almost exactly by other streams | All | Derived stream |

## Appendix B. The front-sensor numbers this generalises

From `e02_front.py`: 115 readings (2026-10-23/24), median −1.1 cm, spread 5.5 cm; per robot −3.4 / −0.9 / +1.2 cm;
this session's own after 8 readings; tolerance 3 sd, at least 3 cm; a shift is reported beyond max(3 cm, 3
standard errors); 200 readings kept per robot. From `e02_trust.py`: healthy sds camera 0.8 cm, wheels 0.6 cm,
gyro 1.0 deg, front 5.0 cm; flag at k 2, clear below 1.5; evidence weight halves each session.

## Appendix C. How the run times were estimated

- **Fresh vs history:** connect and health 1 min; per robot a 10 s still check, a 62-65 s calib run and one find
  (about 1-2 min); 3 robots one at a time: 8-12 min. The front sensor's 8 readings arrive during the first trials.
- **From scratch: known kit:** connect and marks 3-5 min (user); explore per robot 8 rays of up to 105 cm at 4 cm/s
  out and 10 cm/s back (about 40 s a ray, 5-6 min) plus its square dance (about 1 min), 3 robots one at a time
  about 20 min; calib runs 3 min; finds at two walls per robot 6-10 min: 30-40 min with the user's 3-5 min.
- **From scratch: discover:** stage A 1 min per robot; the six motions about 4-5 min per robot (15 min for 3);
  then the known-kit run 30-40 min; the repeat pass about 10 min: 60-70 min, plus 2-5 min of the user's review.
