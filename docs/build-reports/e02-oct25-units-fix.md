# RAW speed units fix: v_cms is mm/s, speeds read 1.08 high (2026-10-25)

Logs and simulator only, no board time. Follows [Right's front sensor and wl_rpm](e02-oct25-right-front-and-rpm.md). Nick approved at 13:03 ("Go ahead with the mm/s simulator and log fix"). No firmware behaviour changed. Only a comment was added in `rawlog.h`.

## Premise

The robots' RAW column `v_cms` holds **mm/s** because of an Arduino_Alvik library bug. `wl_rpm`, `wr_rpm` read **1.08×** the true speed because the motor board assumes a 20 ms update but runs about 21.6 ms. The laptop tools read `v_cms` as cm/s, and the simulator wrote it in cm/s, so simulated and real logs disagreed by 10.8×.

One more thing turned up while checking. The motor board works out `v_cms` and `w_dps` from the same inflated rpm (`kinematics->inverse(getRPM())`), so **both also read 1.08 high**. On the 23rd morning session, `v_cms` ÷ the wheel-angle speed is 10.7-10.8 on every run, and `w_dps` ÷ the wheel-angle turn rate is 1.09-1.12. The library pose (`x_cm`, `y_cm`, `th_deg`) multiplies that velocity by the same assumed 20 ms, so the error cancels and the pose is right.

## Changes

| Where | What |
|---|---|
| `tools/raw_units.py` (new) | One place for the units. `speed_cms(v_cms)` = v / 10 / 1.08, `wheel_rpm(rpm)` = rpm / 1.08, `turn_dps(w_dps)` = w / 1.08. `robot_columns(...)` gives what a robot logs for given true wheel speeds. `legacy_sim_folder` / `to_robot_units` handle old simulator logs (below). |
| `tools/sim/robot.py` | RAW `wl_rpm`, `wr_rpm`, `v_cms`, `w_dps` are now logged as a real robot logs them: rpm ×1.08, `v_cms` in mm/s ×1.08, `w_dps` ×1.08. The random draws are unchanged, so a seed gives the same physical session (truth.csv is identical for seed 7). |
| `tools/sim/simulate.py`, `network.py`, `tools/experiment/e02_sim.py` | Output is marked `"raw_units": "robot-2026-10-25"` in `sim_info.json` (and in `session.json` for simulate.py). |
| `tools/check_session.py` `rows(folder, "raw")`, `e02_replay_view.load_raw` | A simulator folder **without** that mark is rescaled on reading to the robots' units, so old simulated sessions and dry runs still read right. Real logs are never touched. |
| `tools/fusion/engine.py` | Wheel speed (`v`), wheel turn rate (`w`), the per-wheel speeds `vl` / `vr` used for identity by motion, the "possible area" top speed, and the speed-mode still test all go through `raw_units`. |
| `tools/fusion/ekf.py` | Docstring: `wheel_v` takes cm/s. |
| `tools/experiment/e02_live.py` | The Live lab's wheel speed and wheel turn rate are converted. |
| `tools/session_checks.py` | The rest_noise check compares true rpm, cm/s and deg/s with the fusion's still thresholds. |
| `tools/site/observe.py` | Stillness (`STILL_V` 0.3 cm/s, `STILL_W` 2 deg/s) and the drive-straight test (> 3 / > 2 cm/s) use true speeds. |
| `tools/accuracy_checks.py` | The still-test cross-check passes true cm/s. |
| `firmware/home_lab/rawlog.h` | Comment only: the column units. The column names and values are unchanged. |
| `docs/data-formats.md` | "RAW speed columns: their names are not their units" (table, how to read them, the legacy rule). |
| Tests | `tools/tests/test_raw_units.py` (new, 4 tests). `test_sim.py::test_raw_speed_columns_in_robot_units` (new). Fixtures that write RAW speeds now write robot units: `tools/fusion/tests/fusion_synth.py`, `tools/tests/test_session_checks.py`. |

**Choices made [Claude]:**
- **Speed source:** speed is taken as column ÷ 1.08, not from the wheel-angle rate. Every reader works row by row, and the angles update in bursts (one step = 0.36°), so a rate would need its own smoothing window. That window would be a new chosen number. Dividing by 1.08 keeps the 50 ms delay of the board's average.
- **1.08:** this is the measured fleet value (1.080-1.085). It needs no DOE.
- **`w_dps` included:** it comes from the same inflated rpm.
- **Simulator rpm:** only the logged columns carry the 1.08. The simulated wheels still turn at the commanded speed. The real ones turn 7.5% slower, because their PID holds the inflated reading on the command. The simulated firmware's own `get_wheels_speed` reads the commanded level, as the real one does at steady speed. So the bump-stall ratio means the same in both. (Question S1.)

**Left unchanged on purpose:**
- Tools that use rpm only as an "is it moving" test with coarse thresholds: `battery_rates.py`, `floorcal_runs.py` (moving test), the oct24 night analyses, `fit_settings_score.py`.
- Sign checks (`wheel_fwd_signs`, `wheel_turn_signs`) and the gyro unit check (`gyro_factors`). These use only signs and the deg/s-or-rad/s choice.
- The discovery lane's synthetic logs (`tools/discovery/tests/synth.py`). Another agent is working in `tools/discovery`.
- Past analysis scripts (`analyses_oct25/heading_diag.py`).

## Effect on past results

**The offline fusion results do not move.** Every published fusion number (tools/fusion/README.md, the E00d site-setup README) came from simulated sessions. There the simulator wrote cm/s and the fusion read cm/s, so they were consistent. Rerun on simulator seed 7, 3 robots, 35 scored runs, wheel-based setups (no camera; `tools/fusion/replay.py`):

| Run | wheels only | wheels + gyro | wheel distance + gyro heading |
|---|---|---|---|
| A. old simulator, old fusion (as published) | 8.74 / 4.72 / 8.90 | 2.71 / 1.75 / 2.89 | 1.12 / 1.02 / 1.50 |
| B. new simulator, new fusion | 8.74 / 4.72 / 8.90 | 2.71 / 1.75 / 2.89 | 1.12 / 1.02 / 1.50 |
| C. old simulator logs read by the new fusion (legacy rescale) | same as A | same as A | same as A |
| D. **old fusion on robot-unit logs** (what it would have done on real data) | **525.8** / 407.6 / 27.4 | **341.0** / 423.6 / 2.49 | **325.9** / 423.3 / 1.50 |

<sub>Each cell: end error cm / path RMSE cm / heading RMSE deg, mean over the 35 runs.</sub>

B matches A to the last digit, apart from one confidence bound (−10.02 against −10.03 cm). C shows the legacy rescale works. D is the size of the bug if real logs had gone through the fusion's wheel speed: about 3-5 m of end error instead of about 1 cm. **No real-session fusion result was published, so none moves.**

**The Live lab (real sessions, display only).** The lab drives the live view's trust and "the wheels and the camera parted ways" messages. It never steers the runner. Replaying two real sessions through it (camera = `track.csv`):

| Session | Filter restarts ("parted ways"), old → new | Camera jump from prediction, median / 95th pct cm, old → new |
|---|---|---|
| 23rd morning (085107) | 10,597 → 525 | 1.33 / 10.5 → 0.47 / 4.2 |
| 24th 161612 | 3,987 → 301 | 1.02 / 11.7 → 0.46 / 3.9 |

<sub>A restart is the filter throwing away its estimate because the camera disagreed by more than about 7.7 sigma (squared distance 60).</sub>

So the live view's "parted ways" messages and low camera trust on real sessions came mostly from this bug, not from wheel slip. Nothing the runner decided depended on them.

**Other readers on real data:**
- Before the fix, `session_checks` rest_noise and `site/observe.py` stillness compared mm/s with cm/s thresholds, making them about 10× stricter than intended. No real-session result of either has been reported. The site setup has run only in the simulator so far.
- Bump stall detection, the edge guard and the runner are firmware-side or don't read these columns. They are unchanged.

## Conclusion

1. Every laptop reader of the RAW speed columns now goes through `tools/raw_units.py`. The simulator logs what the robots log, so simulated and real logs agree.
2. Published fusion results stand: they were simulated, and they are unchanged to the digit after the fix.
3. The real-data reader that mattered was the Live lab display. Its restarts drop about 13-20× on real sessions.

## What's next

1. Tests: `tools/experiment/tests tools/sim/tests tools/tests` 561 pass, 17 skipped. Fusion: 75 pass. Site, blink, recognition, dock: 307 pass. 1 fails only in the combined run: `test_site.py::test_score_against_tape`. It passes alone. The cause is a module-name clash: `tools/discovery/score.py` and `tools/site/score.py` are both imported as `score`. This change did not cause it.
2. Optional: rerun the fusion with the camera on simulator seeds 7 and 11 to confirm the camera setups are unchanged too. The wheel setups already match exactly, and the camera path does not read these columns.

---

## New tests that may now be needed

1. **The fusion on a real session** with wheels + gyro (+ camera), now that it reads the robots' speed right. This would be the first real-data fusion number.
2. **Live lab restarts on real sessions after the fix** (525 and 301 above): are the remaining ones real slips (pushes against boards), or a too-tight restart threshold (60, chosen)? A sensible DOE row: threshold 30 / 60 / 120 on logs.
3. **rest_noise and observe stillness on a real session** with the corrected units: what share of still rows passes now?
4. **Simulator with slowed motors** (true speed = command / 1.08), if Nick wants dry-run timings to match the robots (question S1). Rerun one dry run and compare durations.
5. **`w_dps` against the gyro per robot:** 1.12-1.14 before the fix, so about 1.04-1.06 after. The rest is the wheel track and scrub. The fusion's turn-scale state should learn it. Check that it does on a real session.

## Possible inaccuracies

- **1.08 is one number for the fleet.** Measured 1.080-1.085 per robot. A different motor-board firmware or loop load would change it (new test 3 in the earlier report).
- **The legacy detection** relies on a simulator marker (`simulated` in session.json, or a `sim_info.json` beside or at `../sim/`) that has no `raw_units`. A simulated folder copied without those files would be read as a real one, 10.8× off.
- **Dry runs going on right now** were started before this change. Their logs are cm/s with no mark, so they are read as legacy (correct). A dry run whose simulator started before the commit, but whose `sim_info.json` is rewritten after, would be wrong. That can't happen with the current code paths.
- **`turn_dps` corrects only the 1.08.** `w_dps` also uses the library's nominal wheel track.
- **The Live lab comparison** feeds `track.csv` (the runner's camera positions) at arrival times, as the replay viewer does. It is not the exact live feed.
- **"Moving" thresholds left in raw rpm** (battery, floorcal, night analyses) are now 8% looser than their numbers say. They are all coarse (0.5-4 rpm).
