# 2026-10-23: why the second session went worse than the first, in numbers

**Premise.** Session 1 (08:51-11:05, 44 runs done, 0 help calls after the restart) went well. Session 2 (11:46-12:25,
no runs finished, 2 help calls) was run while a 6 GB copy of session 1's video came off the Pi over the same WiFi,
with the Pi camera in preview-only mode, at about 50% battery. Nick: "quantify what made this session worse."
Data: both sessions' perf.csv, track.csv, robot leg logs and state (logs release e02-session-oct23).

## Data
| | Session 1 (morning) | Session 2 (copy running) |
|---|---|---|
| Camera lost the robot or disagreed with its wheels, per runner drive | **5%** (32 of 597) | **28%** (15 of 54) |
| ... drives starting within ~20 cm of the top (y-) board | 5% (6 of 115) | 36% (10 of 28) |
| ... drives elsewhere | 5% (26 of 482) | 19% (5 of 26) |
| Share of drives that started by the top board | 19% | 52% |
| Picture age when its position was ready: median | 0.12 s | 0.19 s |
| ... 95th percentile of each second's worst | **0.29 s** | **2.33 s** |
| Seconds with a picture over 1 s late | 12 of 7390 (0.2%) | 607 of 2350 (**26%**) |
| Tracker re-locks per minute | about 1.8 | about 5.8 |
| Battery at the losses | 69-96% | 50-53% |

Late pictures by 10 minutes in session 2: 11:40 15%, 11:50 37%, 12:00 40%, 12:10 23%, 12:20 0% of seconds.

## Conclusion
1. **The WiFi copy is the main measured cause.** Pictures arrived seconds late in a quarter of all seconds (vs
   0.2% in the morning); the late share follows the copy's busiest period. Late pictures make a driving robot
   "lost" (the tracker expects it where it was).
2. **The top board is a second, separate cause.** In session 2, drives by the top board failed twice as often as
   elsewhere (36% vs 19%), and half of all drives started there (the robots kept being put back near it). In the
   morning that area was no worse than elsewhere (5% both), so the board area hurts mainly when pictures are late.
3. **Battery cannot be judged from this.** All of session 2 was at 50-53% *and* under the copy; the morning had no
   drives at 50%. A battery session without a copy running is needed.

## What's next
- Never copy video during a session (the session guide says so from now on); or copy over a cable.
- The fixes pushed during session 2 address the symptoms: a robot found again where its wheels put it after a lost
  drive (no turn on its spot), the "off the board" check using the measured board line, and the camera check
  trusting the runner's own fresh pictures.
- A clean battery session (same plan, no copy) to separate battery from WiFi.

## Sweep 1: new tests that may now be needed
- A deliberate WiFi-load test (a copy running, then not, alternating every 10 minutes) to put a number on how much
  picture delay causes how many losses: this is also the latency test's real-world version.

## Sweep 2: possible inaccuracies
- "Drives" are the runner's own go legs (parking, finds' drive-backs, heading drives), not route runs.
- "By the top board" uses the tracker's last position before the drive, x > 105 cm (about 20 cm from the board's
  line): a chosen cut.
- The copy's start and end times are from Nick's screen (50:47 long, finished about 12:29), not logged.
- The 24 s stall in session 1 (10:23:48, during a find: no pictures at all for 24 s) is separate: its cause is not
  in these logs.
