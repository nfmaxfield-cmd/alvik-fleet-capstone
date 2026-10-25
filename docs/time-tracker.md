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

**What this says so far.** Estimates for code built on existing parts ran 3-5x too high. Simulator testing of
behaviour that interacts with the real board (holds, borders) took longer than the code itself. Next estimates:
code on existing parts about a third of the gut figure; add a separate line for simulator iterations.
