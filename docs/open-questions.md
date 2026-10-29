# Open questions for Nick

Kept up to date by Claude (Nick, 2026-10-27 07:45: "Keep track of unanswered questions and I'll return to them later").
Answered ones move to the bottom with the answer and date.

## Open

### School demo (Monday)
18. Should the inward rescue also apply to the other back-out ("try again on its own" step)? Same weakness.
25. Ladder L4: with robots parked where they were left, the square is usually 42 cm instead of 62. A fair test for
    Monday?
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
42. Should the cameras find the dock's position themselves? Options: keep as built (bump check only, dock position by
    tape); glue a 4 x 4 piece of 20-25 mm squares on the plywood just behind the dock (Claude's pick: no dock
    reprint); or deepen the dock's back to about 9.5 cm (dock grows 16 -> 22 cm).
43. Card and paper thickness for the dock pocket (assumed 0.45 mm card, 0.10 mm paper): measure 10 sheets of each
    with a ruler?
44. A robot that dies or restarts is now passed at 3.2-6.9 cm (bodies) in 9 of 24 simulated runs: it is a fixed obstacle
    with 2.5 cm round it, and the 8 cm net covers live robots only. Keep it a brick (as now), or keep it 8 cm away like
    a live robot (fewer routes on small tables)? (monday-2robot-sweep.md)
45. Safety net: go to 6.5 cm for Monday after a 10 min board check (5 Chickens, 2 circles), and should Chicken's "never
    under" move with it or stay at 8? (safety-gap-doe.md; 5 cm needs the board DOE)
46. Left's distance sensor: its centre-right zone reads under 12 cm in about a third of its readings (others 0-5 %) and
    caused all three of Left's phantom stops. Look at its window? Until then, ignore that zone on Left? (ladder report)
    29 Oct: it reads the plywood floor ahead (every robot's does, at 11-18 cm), Left's nearest; superseded by 59.
47. Lane 1's lane-end turn comes 2.3-3.0 cm from the x = 0 edge in a few runs (median 4.6): add a short straight run-in
    before lane 1, or move lane 1 1-2 cm in?

48. A robot whose turn check is HELD every round (one camera's evidence, or a misread heading) never drives. Hold
    forever (as now), or set it aside after 3 holds in a row? The dry run shows why it matters: a robot whose turns
    are really 12 % off (36 deg over 4 turns) is held every round, never parked. (e02-oct28-overhead-contrast)
49. Thursday's daytime DOE loop: normal mode (it tests the new holds and re-checks), or keep-going (nothing set aside)?
    The plan uses normal mode.
50. Let L0 nudge each robot 3 cm forward and back before measuring, so its start floor is seen? It removes the last
    0.4-3.6 cm of first-crawl error, but L0 then moves robots.
51. Turn check crawl 15 cm instead of 10 (headings misread by up to 14 deg on 10 cm)? Needs 5 cm more room per robot.
52. Should L0 STOP when the lighting check says both cameras are washed out, or only warn (as now)?
53. The find step's frames are now kept (about 25 s a loop on the USB drive): OK?
54. A robot near an edge (or another robot) is now driven back to open space by the DOE loop on its own, slowly
    (5 cm/s), on a planned path the cameras watch, in normal and keep-going mode alike. OK in normal mode too, or
    keep-going only? (e02-oct28-keep-going-run)
55. ANSWERED 2026-10-29 (Nick 07:3x: build a facing check): every Monday / Mission Control start and every DOE find
    step measures which way each robot faces from its own two lights (they sit across the robot, left and right);
    over 15 deg off its start heading nothing moves, in plain words (docs/build-reports/e02-oct29-facing-check.md).
    Needs firmware 2026-10-29a flashed to tell front from back every time.
56. After a calibration that run_plan itself runs (a robot that restarted), the camera bridge has no start heading
    and the run fails to start; the second start works. Build a proper fix after Monday (the bridge vouches for a robot
    back on its start spot), or now?
57. Center's WiFi read -71 to -72 dBm all evening (Left and Right -62 to -64), and it lost one answer. Worth moving
    the router, or checking Center, since it is the spare?
58. Night sessions: is there a light setting that gives the overhead 18+ grey levels (the ceiling light only, the lamp
    off)? Under the table lamp tonight it was 16.

59. Monday, Left's phantom stops (about 1 a drive in today's 3 h loop, all on its centre-right zone reading the floor
    at about 11 cm; 5 "no plan" ends): set Left's stop limit to 10 cm (`set follow_look_cm 10`, Left only; today's
    replay: 0 phantom stops; real obstacles seen 2 cm later), ignore its centre-right zone (firmware change), or leave
    it? Claude's pick: 10 cm on Left after a 10 min board check with a brick. (e02-oct29-daytime-sessions)
60. The camera bridge refuses to start a robot that only the side camera sees (its spot is ~3 cm off, the "moved" rule
    is 3 cm): 5 start-ups failed today on the overhead's blind strip (x = 30-45 cm, daylight). Allow more for side-only
    sightings (e.g. 5 cm), after a logs replay, before Monday?
61. The DOE loop parks a robot for good when it misses one status check; Center answered again 2 minutes later and
    stayed out for 2 hours. Re-check it at the next loop instead?
62. Was the morning sun on the Pi at 07:43 (that loop ran 5 C hotter than the 09:22 one, 19 min at 80 C+)?
63. Right has not driven since 28 Oct night (kept out today: 27 cm from the edge). Give it the next session's drives,
    since it is Monday's robot?
64. The overhead camera loses robots on a strip at x = 30-45 cm of the home board in daylight (sun patch or glare?):
    blinds closed for the next daytime session, or does it matter only at home (Monday is at school)?
65. Facing check: a robot 15-45 deg off its start heading stops the start now. Give it one small turn on the spot
    before the run instead (it moves, but saves walking to the table)? (e02-oct29-facing-check)
66. Center's lights read 24 deg off its 10 cm crawl in loops 0-2 on 28 Oct (both cameras agreed with each other). A
    still photo of Center from above with a ruler along its side would say which is right. (e02-oct29-facing-check)
67. The facing check takes 6-8 s for two robots (planned 4-5). OK before Monday's run? (e02-oct29-facing-check)

68. Distance-sensor stops: switch every robot to 3 readings in a row before Monday (plus Left at 10 cm)? Replay of
    29 Oct: phantom stops 69 % of Left's drives now, 0 % with both; costs about 2 cm of reach for a real brick.
    (e02-oct29-rules-audit)
69. Where did the side camera's 13-24 grey levels come from on 29 Oct? The five DOE releases only have 21-45 at L0.
    (e02-oct29-rules-audit)
70. May a distance-sensor obstacle the camera did not confirm expire on its own (about 15 s), and be dropped rather
    than end the mission with "no plan"? The stop itself always stays. All 5 "no plan" ends on 29 Oct followed a
    phantom stop. (e02-oct29-rules-audit)
71. Should the camera bridge count a robot as moved off its start only when it has been told to move, not by a 3 cm
    camera distance (5 start-ups refused on 29 Oct with the side camera alone)? (e02-oct29-rules-audit)
72. Monday: should a robot that goes silent for 4 s rejoin when it is heard again, instead of staying out for the
    whole run? (e02-oct29-rules-audit)
73. Lower your DOE-loop room rules from 25 cm (edge) / 30 cm (robots) to 20 / 25 cm, now that robots are driven to
    open space and calibrate in 12 / 21 cm? (e02-oct29-rules-audit)
74. Is Center in Monday's run? Its centre-left zone gives phantom stops in 41 % of its drives too. (e02-oct29-rules-audit)
75. Should the lighting check sample where the robots will drive (the x = 30-45 strip), not only where they stand?
    (e02-oct29-rules-audit)

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
- 2026-10-28 18:0x: Q22 (exposure) updated: automatic, EXCEPT the overhead steps its exposure down while the board is
  washed out (clips), says so on screen and logs it, then returns to automatic; on by default (camrec WashoutGuard,
  docs/build-reports/e02-oct28-overhead-contrast.md). Neither night's board clipped so far, so it has not acted yet.
- 2026-10-28 (Claude, overnight): Q31 done in the simulator for the 2-robot mission: 3 seeds of every condition on
  two tables (four for the main ones), docs/results/monday-2robot-sweep.md.
- 2026-10-28: Q41 (Chicken not played together): fixed. Both robots wait for each other at their ready spots, then run
  Chicken together (swerve 15 cm, never under 8 cm); see docs/build-reports/e02-oct28-one-system.md, "Chicken
  played together". Monday sweep runs made before that commit had the old, one-at-a-time Chicken.
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
  - School WiFi: the name CONTAINS "ISE" (Nick, 2026-10-28 09:25; Claude had taken it as the whole name; exact name still to confirm); Nick enters the password himself into secrets.h when flashing (not stored here).
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
  - Exposure: keep automatic (Q22; updated 2026-10-28 18:0x, see above).
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
