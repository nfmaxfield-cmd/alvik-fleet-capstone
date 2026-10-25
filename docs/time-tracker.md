# Time tracker: estimates against actual time

Nick (standing preference): before any task expected to take over 10 minutes, give a time estimate; keep this file
so estimates get more accurate. Every chat working on this repo adds a row when a task ends. "Actual" is wall-clock
time from start to pushed, including tests and fixes; Nick's own time at the board is separate.

| Date | Task | Estimate | Actual | Ratio (actual / estimate) | Notes |
|---|---|---|---|---|---|
| 2026-10-24 | Virtual border, lost-robot stop, hold for Nick, battery by movement (runner) | 1.5-2 h | 1.3 h | 0.75 | plus 0.6 h of simulator iterations finding 5 false alarms |
| 2026-10-24 | Sensor trust + software bad-sensor test + today's fixes (runner) | 6.5-7.5 h (4-5 h trust, 1.5 h fixes, 1 h tests) | 1.5 h | 0.2 | estimate far too high; the parts reused existing fits, error map and logs |
| 2026-10-24 | Independent code review + fixing its findings | (not estimated) | 1.0 h | - | 24 findings, the serious ones fixed |
| 2026-10-24 | Firmware: robot's own border + sleep/check-in, simulator model, tests | 2-3 h | 0.8 h | 0.3 | compiled only with the stand-in headers (tools/fwcheck), not the real toolchain |
| 2026-10-25 | Recorder fps + Right's session diagnosis, ghost recheck fix | (not estimated) | 0.6 h | - | cause found from timing (upload during the session) |
| 2026-10-25 | Bad camera shrinks the area; robots' own border takes over; front holds off | (not estimated) | 0.5 h | - | simulator check queued in the grid (camera noise 2 and 4 again) |
| 2026-10-25 | Dataset packager + tests | (not estimated) | 0.3 h | - | |
| 2026-10-25 | Live-view mockup (2 artboards), checklist update, paper outline page | (not estimated) | 0.8 h | - | |
| 2026-10-25 | Agents: publication plan + generality audit; charging plan + CAD; side camera | (not estimated) | 0.3-0.4 h each, run in parallel | - | agents' wall-clock: 19 min, 23 min, about 1.5 h |
| 2026-10-25 | Firmware 2026-10-25b: backing-in find + dock backwards (back switches), simulator, runner flag, docs | (not estimated: assigned with a full spec) | 1.0 h | - | its own simulator runs found 3 issues (wrapped turn; the cap vs the rear caster, hence the look; a comma in a CSV message); another agent's WIP commit overwrote one edit mid-build |
| 2026-10-25 | Mark-the-robot page (MIRLT): ranged footage download, tracker guesses that restart at corrections, page, tests, how-to | (not estimated: assigned by an agent with a full spec) | 0.8 h | - | real-footage checks found 3 bugs (empty-board picture never used; stale guesses after a correction; a page race when opening a stretch) |
| 2026-10-25 | Blind sensor discovery replay (MIRLT, logs only): tools/discovery package, synthetic robot, 34 tests, replay of 52 cases, report, DOE rows | (not estimated: assigned by an agent; design doc guessed 8-12 h) | 1.3 h | 0.13 of the design doc guess | most time went into rule iterations on development cases; the synthetic robot found 5 generality bugs; full replays take about 4 min |
| 2026-10-25 | Real session 15:10 diagnosis (MIRLT, logs + video frames): 7 holds, collision, explore failure, calibration check, report, register and DOE rows | (not estimated: assigned by an agent) | 0.45 h | - | offline refit reproduced the Pi exactly; the video settled the collision and the outline question |
| 2026-10-25 | First real-session fusion (wheels+gyro vs +camera, leave-camera-out gaps) and Live lab restart DOE (MIRLT, logs only): 2 scripts, report, 3 charts | (not estimated: assigned by an agent) | 2.0 h | - | about 1.2 h of it waiting on replays (2 cores shared); the lab DOE alone took 50 min per pass on the longest session |

**What this says so far.** Estimates for code built on existing parts ran 3-5x too high. Simulator testing of
behaviour that interacts with the real board (holds, borders) took longer than the code itself. Next estimates:
code on existing parts about a third of the gut figure; add a separate line for simulator iterations. The
overnight 2026-10-25 items were not estimated beforehand (one batch approved together), so they add actuals only.
