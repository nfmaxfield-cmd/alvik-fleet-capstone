# Open questions for Nick

Kept up to date by Claude (Nick, 2026-10-27 07:45: "Keep track of unanswered questions and I'll return to them later").
Answered ones move to the bottom with the answer and date.

## Open

### School demo (Monday)
18. Should the inward rescue also apply to the other back-out ("try again on its own" step)? Same weakness.
25. Ladder L4: with robots parked where they were left, the square is usually 42 cm instead of 62. A fair test for
    Monday?
31. 3 more seeds for the conditions that held, before calling them safe (about 2 h of machine time)?
32. Does the moved side camera see the whole board? A robot outside its picture can't be helped while the overhead
    is washed out (fix: move it further back, or a third camera).
33. Should a robot seen only by its lights be allowed to move? (Now: kept out at L0, the safe choice.)
34. The taped-marks test (T1) to settle which camera carries the ~5 cm constant offset?
35. Robots' white lights staying on while the ladder watches: OK?
36. 15:26-15:27: how did you wake the robots (the wake script on the Pi?), and could it still have been running when
    the ladder started at 15:27? (For 20 s the planner heard no robot at all.)
37. Should the ladder redo L0 and L1 automatically whenever a robot has restarted (about 4 min)?
38. Light search: use the side camera alone, or both cameras taking the stronger signal?
39. Leave turn-around points out of the 3 cm gate? (Right would pass L3; it changes what Phase 0 measures.)
40. Did the room light change between 15:04 and 15:20? (The side picture brightened from 39 to 58 with its exposure locked.)
41. Monday's Chicken is not played together: each robot runs its half when it gets there (61 x 122: Left swerves at
    82 s, Right at 103 s; Left is dancing by then). Fix: make both wait for each other at the ready spots (as "meet"
    does) before the run-in. Build it? (It changes the mission the Monday sweeps are simulating.)

### Robots and sessions
8. What stands just beyond the home board's x = 0 edge at about y = 30-60? (Your 07:45 "yes" to this one was read as
   "yes, mark that side as partly blocked"; say if you meant something else.)
9. Run 4 of last night (Left, wrong camera identity): leave it out of the results, or use the video estimate (about
   33 cm, by eye)?
10. What cooling does the Pi have (you said yes to a fan or heatsink: which, and since when)? The heat tracker needs
    it in words.

### Analysis follow-ups
11. Should a robot's old sensor flag clear after N sessions with no new evidence (N = 1, 2 or 3 tested by replay), or
    show its date on the live page?
12. Is a speed-sweep test (4 / 7 / 10 / 13 cm/s) worth board time? It separates camera timing from wheel slip.
13. Should creep steps change? 12 of 23 stalled (faster creep, or end on the front sensor instead of the wheel count).

## Answered
- 2026-10-28: Q4 (Mission Control as a tab): yes. Built as the tabs on the start page, http://100.122.222.75:8091
  (Live lab | Mission Control | Start/Stop); see docs/build-reports/e02-oct28-one-system.md, "One address".
- 2026-10-27 18:26:
  - Warm-up results are kept and used for that day's runs (Nick: "they're allowed to retain that data").
  - Q14 pass mark: 95 of 100 camera readings within 3 cm; also analyse whether misses come as one-off frames or runs of
    several in a row.
  - Q17: limit the own-pose window after a barrier find to about 10 s.
  - Q16: Nick thought he flashed firmware; the 15:31 check showed 2026-10-25c on all three, so it did not take.
- 2026-10-27 16:43:
  - School table: a little more than 2 ft x 4 ft, not more than 2.5 ft x 4.5 ft (about 61-76 x 122-137 cm); exact size
    on Thursday.
  - School WiFi: SSID "ISE"; Nick enters the password himself into secrets.h when flashing (not stored here).
  - School cameras: set up like the home lab, one overhead and one at an angle; the system should calibrate itself.
  - A robot that dies (battery) mid-run is treated as a brick (an obstacle where it stopped) and the others carry on.
  - The 8 cm safety net: yes; narrow it this week if possible.
  - Monday: no brick drop. A short choreographed run, now with 2 robots: lawnmower from each side, meet in the middle,
    circle each other, back off, then Chicken.
  - Board-test time: unknown this week.
  - Monday robots: 2 (Q15). Q2 and Q3 (the 3-robot routine's timing and Chicken during mowing) no longer apply.
  - Firmware flashed last night: no; all three still run 2026-10-25c (the 15:31 check) (Q16).
  - Who's who: should take under 2 minutes (Q19).
  - DOE notes also go into docs/doe-backlog.md (Q20).
  - Overhead camera height about the same as before (Q21).
  - Exposure: keep automatic (Q22).
  - 0,0 corner: not marked; use the checkered rectangle (Q23).
  - Which robot was low-contrast at 10:07: unknown (Q24).
  - Ladder default: run straight on (Q26).
  - Robots should not die (battery), but a dying robot is a brick for the others (Q28).
  - Re-plan fallback (drop play, finish, park): yes, but avoid needing it (Q29).
- 2026-10-27 07:45: build fixes 1-4 from last night (yes; who's who may stop all robots to confirm); Nick saw Center
  drive off (yes, E1's 00:27:28 sidestep); Center and Left were still at the x = 122 edge (yes: bench `turn 90` check
  for Center first); the Pi has cooling (yes; which kind still open, question 10).
- 2026-10-27 07:08: standing still to re-plan is acceptable ("much rather ... than collide or go over the edge");
  the "plan only the next 30-60 s" re-plan work is dropped for now.
- 2026-10-27 00:08: re-plans send the first safe plan at once; Chicken 15 / 8 cm; nothing locked in the operator
  view; scope all of it, engine first.
- 2026-10-26 23:15: no music on Monday.
