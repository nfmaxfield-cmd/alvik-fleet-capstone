# e24: the 23 Nov school runs and the usability audit (24 Nov)

Built on `39df4cf`, simulator, logs and page tests only. Nothing was run on robots, Pis or cameras. **No firmware
changed: nothing to flash.** RFID stays off for Jay-Sun's runs and is not an input anywhere in this build.

## Premise

Two reports asked for this build. The analysis of Jay-Sun's 16 school runs of 23 Nov
(`docs/results/school-2026-11-23/report.md`) found that e23's fixes held and listed eight new faults (N1 to N8). The
usability audit of Mission Control (24 Nov) listed 12 findings and 12 bugs (B1 to B12). Nick decided D1 to D6 on 24 Nov
and asked for the rest to be built with defaults, listed here as questions (Q-a to Q-f). The users are Jay-Sun (two
robots at the school table, a Visit points list per robot) and Nick (one typed command at a time).

## Data

### 1. What changed

Every changed behaviour keeps its old way behind a switch. Safety switches are in `tools/fleet/safety_group.py` only
(115 now, 16 new, all decided automatically; `docs/how-to/safety-group.md`). "Test" names are in
`tools/fleet/tests/test_e24_school_usability.py` unless a file is given.

**Nick's decisions**

| Item | What the run does now | Old way (switch) | Evidence |
|---|---|---|---|
| D1. A driving robot the camera loses | It slows, and turns back only when its doubt at that speed could reach an edge or keep-out; otherwise it keeps going slowly | `MIRLT_VEER_RULE=ahead` | test `a_driving_robot_with_little_room_slows_instead_of_turning_back`; acceptance, section 2 |
| D2. A standing robot the camera loses | It stays where it is, anywhere on the table (before: only in its dwell at a point) | `MIRLT_BLIND_STANDING=act` | test `a_standing_robot_anywhere_stays_put...`; the real run 5's chain started with turn-backs of standing robots (Appendix B) |
| D3. Missed points | A point missed, not reached or refused is tried once more at the end of the list, by the other robot if its own is held. No point leaves every list without a line (N4) | `MIRLT_POINT_RETRY=off` | test `a_point_that_leaves_every_list_gets_a_line_and_is_tried_once_more...` |
| D4. How the run ended | "Ended: 34 of 40 points reached" with one line per missed point (why, what to do next) in the terminal, `mission_summary.json` and Mission Control's end card. "Done" only when every point was reached | (wording only) | test `done_is_said_only_when_every_point_was_reached`; end card screenshots |
| D5. Jay-Sun's Pi changes | Camera 2 is on at the school site. His camera switch is in both pages. `docs/jay-sun-changes.md` is in the repo. A Pi's own choices (camera on or off, a capture rate set by hand) live in `~/.config/mirlt/sites/<site>/site_local.json`, not in the repo's file. The capture rate is chosen from what the Pi's bridge managed to read in its last 5 runs; the school starts at 10 | `./run-fleet-pi.sh local set capture.fps 15` fixes it by hand | tests `a_pis_own_choices_live_beside_its_camera_names...`, `the_capture_rate_follows_what_the_bridge_can_read`; replayed on 23 Nov: two-robot runs give 10, one-robot runs 15 |
| D6. Usability | The map-first page is the first page (`/`); the old page stays at `/classic`. See "The page" below | old page selectable in Settings | screenshots, section 4 |

**Defaults built, to confirm (questions Q-a to Q-f)**

| Item | What the run does now | Old way (switch) | Evidence |
|---|---|---|---|
| Q-a. Heading check after a turn | After every turn the camera's heading is compared with the robot's own. It waits for a fresh picture only when the next leg could reach an edge or keep-out if the heading were 30 degrees off; a turn found wrong is driven again. A gyro step its own turn rate does not explain is logged as a yaw jump | `MIRLT_TURN_HEADING=off` | 5 tests; acceptance "gyro jump" rows |
| Q-b. Battery under the floor | The robot finishes the point it is on, goes home, and its points go to the other robot | `MIRLT_BATTERY_LOW=finish` | test; acceptance "battery" rows |
| Q-c. A robot drops out, a list per robot | Its points wait for it. They go to the other robot only at the end of that robot's own list (D3), or when the robot was never found | `MIRLT_HELD_POINTS=reassign` | 2 tests |
| Q-d. Time limit of a Visit points run | 4 times the plan's length; past that, the run ends only once no new point has been reached for 4 times the plan's time per point (before: 900 s fixed) | `MIRLT_VISIT_TIME_LIMIT=fixed` | Appendix A |
| Q-e. "Is Jay-Sun here?" | Asked once per Mission Control session, in one start panel with the other two questions | Settings: `start_popups` | screenshot `e24-feature-start-panel-mock-*` |
| Q-f. Practice runs | Save no camera frames (about 130 MB a minute before) | Settings: `practice_frames` | read in the run's command |

**Faults from the 23 Nov report**

| Fault | Fix | Evidence |
|---|---|---|
| N1. Gyro jump at a turn put a robot on a plate | Q-a above. The simulator can now inject a jump at a turn | acceptance "gyro jump" rows |
| N2 and the slow re-plans | Every re-plan rebuilt the table's grid from nothing. It is now derived from the remembered one (`MIRLT_PLAN_FAST=0`: as before). Same plans, step for step | section 3 |
| N3. Naming gave a tag to the wrong robot | The naming waits for late pictures after each robot's wiggle and gives a tag only where motion was found (`MIRLT_NAMING_SETTLE=0`) | test `naming_with_pictures_two_seconds_late...`: old way names neither robot, new way both |
| N4. A point lost with no line | D3's ledger | test |
| N5. "disagree by 0.0 cm" | A heading disagreement is said in degrees | read in the code path; no separate test (Sweep 1) |
| N6. Second Ctrl+C lost the exports | The wrap-up ignores further Ctrl+C until the exports are written, says so, and gives up only at the third (`MIRLT_WRAPUP_GUARD=0`) | stop case, section 2 |
| N7. A pause step far from its point | A stop due at a point the robot is no longer at gets its drive back before it (`MIRLT_DWELL_PLACE=where`) | test `a_stop_due_at_a_point_the_robot_is_no_longer_at_gets_its_drive_back` |
| A tag's turn read wrong by the start crawl (found by this build's acceptance runs) | An answer between two quarter turns is crawled once more and averaged (`MIRLT_CRAWL_AGAIN=0`) | test `a_crawl_answer_between_two_quarter_turns...`; Appendix D |
| N8. "nothing from the laptop for 1.5 s" 192 times | Traced: it is the firmware's idle notice, not a stop. It appears whenever a robot waits for a slow re-plan or is held. No change needed beyond N2 | Appendix B |
| The thrash that ended run 5 | Chain found in the logs: standing robots turned back, the camera guard held them, their points were handed over, refused, and the 10-rejoin cap ended the run. D2, Q-c, no cap on rejoins (`MIRLT_REJOIN_CAP=ten`), a rejoined robot with no route waits instead of being held again (`MIRLT_REJOIN_STRANDED=hold`) | test `a_held_robot_is_never_given_up_on...`; Appendix B |
| AL-EB54 drove on general corrections | Software, not the robot: every run start sent the general corrections over the robot's saved calibration. A calibrated robot now gets its own (`MIRLT_CALIBRATED_CORR=prior`) | test `a_calibrated_robot_is_sent_its_own_corrections...` |
| Card filling up | Finished runs move to the USB drive by themselves at the start and end of a real run when the card is low; Mission Control's nested run folders are counted (`MIRLT_MOVE_RUNS=0`) | test `mission_controls_finished_runs_move_to_the_usb_drive...` |

**The page and the Pi commands (the audit)**

| Finding | Change | Evidence |
|---|---|---|
| 1, B8. Messages off-screen; a lost server looks healthy | Messages appear on screen at any size. After two missed answers a red band says Mission Control is not answering and the numbers are old; STOP then says it did not get through | `e24-feature-not-answering-*` |
| 2, B2. Run again lost the lists | The server remembers the last order; the button reads "Run again: Alpha P01, P02 · Bravo P12, P14"; lists survive a reload and "New run" | reload case, section 2 |
| 3, B9. The end did not say how it went | D4's end card; a robot not found is on it | `e24-feature-end-card-*` |
| 4. Dead cameras and held robots looked normal | Red alert cards on top (no pictures, every robot held, needs a hand), each robot's card says why it is held | `e24-feature-alerts-*` |
| 5, B11. Lists and picture on different pages | The map-first page has the camera picture with Jay-Sun's switch (falls back to overhead when a camera has no model), a typed order, presets per robot, rings on unreadable points | `e24-feature-plan-view-*` |
| 6. Phone | One-row bar, lists in a pull-up sheet, alerts and robot cards above the map, 24 px tap ring | `*-phone.png` |
| 7. Updating the Pi | `./run-fleet-pi.sh update`: one command, backs up everything that is only on the Pi, says PASS or FAIL | rehearsed 3 ways, Appendix C |
| 8. Raw program output in the bar | A plain phase ("Naming each robot by its wiggle", "Driving: 12 of 40 points"), "last heard" from the run's own log | `e24-feature-run-phase-*` |
| 9, B6, B7. Help and command names | `./run-fleet-pi.sh` with nothing or a typo prints 12 lines and "did you mean"; `mc-stop` waits until the port is free; `stop` asks which stop is meant | `tools/pi/tests/test_update_pi.py` |
| 10. Safety drawer | Switches set by hand first, "All back to automatic", one plain sentence per switch, a question before an old way is saved | `e24-feature-safety-*` |
| 11. Three pop-ups at Start | One in-page start panel, ticks remembered for the session | `e24-feature-start-panel-*` |
| 12, B3. Smaller | The practice simulator lives as long as its run; restart asks first; practice frames off (Q-f) | - |
| B1. Stop took 52 s from a shell that ignores Ctrl+C | Every run program takes Ctrl+C again whatever its parent did (`MIRLT_SIGINT_RESTORE=0`) | stop case, section 2 |
| B4. A dropped robot's point went missing | Q-c and D3 | tests |
| B5 | N5 | - |
| B10. `/classic` page error on typing an order | Fixed | Playwright run: no page errors |
| B12. "7.8 GB free of 0 GB" | The total is read from the same place as the free space | to check on the Pi (Sweep 2) |

### 2. Acceptance in the simulator

Before = `39df4cf`, after = this build, on the new stand-in for the school Pi (`tools/sim/standins/school23.json`):
camera losses drawn from the 956 measured on 23 Nov (6.3 a minute per robot in camera 1's half, 2 a minute in the
other) and planning 15.1 times slower than this machine (the Pi's measured re-plans against the same re-plans timed
here). Two robots, 10 cm/s, a list per robot. A run's outcome depends on timing, so the same seed does not repeat
exactly.

<<ACCEPT>>

<sub>A range is the lowest and highest value over the seeds run. With two or three runs it says what happened in those
runs; it is not a prediction for the next run. "Points reached" counts points a robot truly stopped at (checked
against the simulator's true positions).</sub>

Two cases are not driven lists:

| Case | Before | After |
|---|---|---|
| Stop during start-up, Mission Control started from a shell that ignores Ctrl+C | Stop took 50.3 and 50.6 s; the practice robots drove 36 to 128 cm after STOP | 2.5 s; 0 cm (`acceptance/stop_case.jsonl`) |
| Lists Alpha P01, P02 and Bravo P12, P14; run; reload the page; Run again | Button "Run again"; the second run got no order and shared the table's points out | Button "Run again: Alpha P01, P02 · Bravo P12, P14"; the second run drove the same lists (`acceptance/run_again_case.jsonl`) |

### 3. Planning speed, and how many points to plan ahead

The same 17 re-plans saved from simulator runs were timed on the planner before and after
(`planbench/compare.md`): the plans are the same step for step in 17 of 17; the total went from 22.0 s to 13.8 s; the
re-plans of a few points are 3 to 6 times quicker, the whole-list ones 0.4 to 1.1 times the old time.

<sub>Median ratio 0.54: half of the re-plans took less than 0.54 of their old time. Timed on this machine, one run
each, so differences of a few percent between single re-plans are noise.</sub>

Why it matters (my inference): on 23 Nov every re-plan on the Pi cost about 5 s before any point was planned, which is
over the 2 s budget, so the "plan a few points ahead" rule fell back to the whole list every time. With the grid
remembered, the fixed part measured here is 0.03 to 0.06 s for small re-plans (about 0.5 to 0.9 s at the Pi's factor).

<<DOE>>

### 4. Tests

<<SUITES>>

## Conclusion

<<CONCLUSION>>

## What's next

1. Update the school Pi (steps below), then one 4 + 4 run and one 20 + 20 run with Jay-Sun's usual lists. The first
   thing to read afterwards: the end card, the log's `CAPTURE RATE:` line, and `turn_heading` in the run's summary (how many turns were checked and corrected).
2. Nick's answers to the questions below (Q-a to Q-f change behaviour Jay-Sun will see).
3. After that school session: the usual logs-only analysis, with the heat tracker, and a check of the stand-in against
   what the Pi did with the quicker planner.

## School Pi: update steps (Nick)

This first update is longer than the next ones, because the update command is not on the Pi yet. After it, an update
is steps 1 to 3 and then `./run-fleet-pi.sh update`.

**1.** On the laptop, click the **Tailscale** icon near the clock and switch the account to **joincapstonedb**.

**2.** Open **PowerShell** on the laptop. Every box below is typed in that PowerShell window: type the one line, press
Enter, and wait for the line to finish before the next box.

```
ssh nick@100.72.183.28
```

Type the Pi's password if it asks. You should see a line ending in `$`. If it says `timed out` or `could not resolve`,
Tailscale is on the wrong account: do step 1 again.

**3.** In the same PowerShell window:

```
cd ~/alvik-fleet-capstone
```

You should see the `$` line again, now with `alvik-fleet-capstone` in it.

**4.** In the same PowerShell window:

```
git fetch origin
```

You should see a few lines ending with `main -> origin/main`, or nothing at all. Nothing on the Pi has changed yet. If
it says `fatal` or `Could not resolve host`, the Pi has no internet: stop here and send me that screen.

**5.** In the same PowerShell window:

```
git show origin/main:tools/pi/update_pi.py > ~/update_pi.py
```

You should see only the `$` line again. If it says `fatal: path ... does not exist`, step 4 did not finish: do step 4
again.

**6.** In the same PowerShell window:

```
python3 ~/update_pi.py
```

It takes about a minute and says each step in plain words: closing Mission Control, getting GitHub's newest code,
backing up everything that is only on this Pi (Jay-Sun's three changes and the edited site file), putting the code on
GitHub's newest, checking the result. The last block should start with `PASS: this Pi is on GitHub's newest code` and
then list where each backup is. Nothing is deleted.

- If it says `a run is going`: wait for the run to end (or press STOP in Mission Control), then type step 6 again.
- If it says `FAIL`: it has already put the Pi back exactly as it was. Do not type anything else; send me that screen.

**7.** In the same PowerShell window:

```
./run-fleet-pi.sh local
```

Among the lines you should see `Camera 2 (side): ON. Capture rate: 10 a second` (the reason in brackets after it may differ). If camera 2 says
`off`, send me that screen before a run.

**8.** In the same PowerShell window:

```
python3 tools/fleet/safety_group.py show | tail -1
```

You should see `Safety group: every switch on its automatic default (115 switches; tools/fleet/safety_group.py show)`.

**9.** In the same PowerShell window:

```
./run-fleet-pi.sh mc
```

You should see a line with `:8095` in it. If step 6 said it started Mission Control again, this says it is already
open: that is fine.

**10.** In the laptop's browser (still on the **joincapstonedb** Tailscale account), open `http://100.72.183.28:8095/`.
You should see Mission Control with the table's map first (the old page is the button "Open the old page" in Settings). Choose
**Visit points** and you should see a box "Type the order" under the map.

Firmware: nothing changed, nothing to flash.

## Questions for Nick

**Substantive**

- **Q-a. Heading check after every turn.** Built as: wait for a fresh camera heading only when the next leg could
  reach an edge or keep-out with a 30 degree error, otherwise check while driving. Keep? (The 30 degrees is chosen:
  the largest jump measured, 25.8, plus 15 %. DOE: no, it should come from more recorded jumps.)
- **Q-b. Battery under the floor.** Built as: finish this point, go home, hand the rest over. With one robot alone
  there is nobody to hand over to, so its points end as missed with "charge it". Is that right, or should a lone
  robot finish its list?
- **Q-c. A list per robot when a robot drops out.** Built as: its points wait for it, and go to the other robot only
  after that robot has finished its own list. Jay-Sun's data depends on which robot visits which point: should the
  hand-over at the end happen at all for his runs?
- **Q-d. Time limit.** Built as 4 times the plan's length, and even then the run goes on while points are still being reached. Or no limit at all for Visit points?
- **New 1. The 18 Nov plate contact (run 14) was not a heading fault.** The raw rows show the robot's own heading
  within 0.5 degrees of the camera's right up to the plate; the 20 to 27 degree difference appears after the contact.
  The 23 Nov report offered a yaw jump as a candidate cause; the rows do not support it. The cause of that contact is
  open again. Do you want it traced before the next school session?
- **New 2. The capture rate now follows the Pi's load.** It picks 15 when the bridge read at least half of the
  pictures at 15 in its last 5 runs, else 10. On 23 Nov's logs that gives 10 for two robots. Jay-Sun set 10 by hand;
  this keeps his 10 and would go back to 15 only if the Pi keeps up. Is an automatic rate acceptable during his
  experiments, or should the school stay fixed at 10 until he says otherwise? (The "half" is chosen. DOE: yes.)

**Other**

- **Q-e.** "Is Jay-Sun here?" once per Mission Control session, in the start panel: enough?
- **Q-f.** Practice runs save no frames: any case where you want them by default?
- **New 3.** The update keeps a Pi's own choice only for camera on/off and the capture rate. Any other edit to a
  site file on a Pi is backed up and replaced by GitHub's. Are there other per-Pi choices to keep?
- **New 4.** The phone layout was built from the audit's numbers and checked at 390 x 844 only. Which phone do you
  and Jay-Sun use at the table?
- **New 5.** The stand-in shows almost no difference between before and after on the plain 4 + 4 list. Do you
  want the stand-in made harsher than the 23 Nov measurements, or left as measured?

## Appendix A: chosen numbers

Each is marked `[chosen]` in the code.

| Number | Value | Where | Why this value | DOE? |
|---|---|---|---|---|
| Heading error treated as possible after a turn | 30 degrees | `run_plan.py` YAW_JUMP_POSSIBLE_DEG | the largest jump measured (25.8) plus 15 % | No: re-measure when more school runs exist |
| Two camera headings count as settled when within | 4.2 degrees | `run_plan.py` fresh_headings | twice the measured spread of the difference between two pictures' headings | No (measured) |
| A picture counts as after the turn from | 0.45 s after it ended | `run_plan.py` | the school's measured picture lag | No (follows the lag record) |
| Time limit of a Visit points run | 4 x the plan | `run_plan.py` VISIT_LIMIT_X | the eight runs that finished by themselves on 23 Nov took 1.2 to 3.3 x their plan | No: a limit, not a tuning value |
| Wrap-up gives up at the | 3rd Ctrl+C | `mission_run.py` WRAP_GIVE_UP_N | one by mistake, one more in impatience | No |
| Capture rate must be at most | 2.0 x what the bridge reads | `tools/pi/capture_rate.py` NEED_X | pictures nobody reads only load the Pi | **Yes**: 10 / 15 / 20 a second against time unseen, on the Pi |
| Capture rate looks at the last | 5 runs | `capture_rate.py` KEEP_N | one bad run should not flip it | No |
| Calibration values taken as sane | wheel and gyro scale 0.8 to 1.2, drift within 1 degree a second | `run_plan.py` CAL_SANE | a saved value outside this is taken as a broken file, and the general values are used | No |
| A tag belongs to a wiggle within | 14 cm of the motion found | `motion_find.py` TAG_STROKE_NEAR_CM | 1.5 robot bodies | No |
| Phone tap ring | 24 px | page | the audit's suggestion | No |
| A warning stays in the bar for | 15 s | `start_job.py` WARN_SHOW_S | long enough to read twice | No |
| mc-stop waits for the port | 40 s | launcher | a run's own stop takes up to 30 s | No |
| Stand-in: planning slower than this machine | 15.1 x | `standins/school23.json` | fitted to the Pi's 23 Nov re-plans | No (measured); re-fit after the next session |

## Appendix B: what the logs showed

- **Run 5 of 23 Nov (the thrash).** Standing robots were turned back when the camera lost them; the camera guard
  refused three times and held them; their points were handed to the other robot, which refused them ("no room to
  turn"); after 10 rejoins the cap ended the run with "every robot is out".
- **N8.** The line "nothing from the laptop for 1.5 s; standing still" is printed by the firmware when a robot with
  nothing to do hears no step for `follow_quiet_ms`. It is not a stop and no step was lost. Its count follows the time
  robots spent waiting.
- **18 Nov run 14** (`20261118-161142`): no yaw jump in the raw rows before the plate. One 17.2 degree jump was found
  the same day in run `183130` (Bravo).
- **AL-EB54.** Every start and re-check sent wheel 1.075, gyro 0.979, drift 0 (the general values) over the robot's
  saved calibration.

## Appendix C: how to reproduce

- Acceptance: `python3 docs/results/e24-school-usability/run_case.py <repo> <out> "<name>:<seed>:pts=4:standin=school23"`
  (its header lists every case option); table: `acceptance_table.py <out> docs/results/e24-school-usability/acceptance`.
- Stop case: `stop_case.py`; reload case: `run_again_case.js` (Mission Control in practice mode).
- Planning: `planbench.py`; results in `planbench/`.
- Update rehearsal: `update_rehearsal.sh <scratch folder> [fps15|sideoff|other]`; the three outputs are in
  `update-rehearsal/`. Each rebuilds a clone like the school Pi on 23 Nov (Jay-Sun's three commits, an edited site
  file, loose backup files) and checks that nothing was lost.
- Screenshots: `docs/screenshots/2026-11-24/e24-*` (33 pictures; "mock" in a name means the real page fed staged
  answers, because the state needs real robots or a fault).

## Appendix D: Claude's review of this build

<<REVIEW>>

## Sweep 1: new tests that may now be needed

<<SWEEP1>>

## Sweep 2: possible inaccuracies

<<SWEEP2>>
