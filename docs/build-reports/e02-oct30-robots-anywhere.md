# Robots put down anywhere: found by their lights, driven to their start (30 Oct)

**Status: stage 1 of 2 pushed** (robots anywhere, refused fixes, calibration caps). Stage 2 (no stops, no sit-outs:
Nick 10:08) follows in this report.

## Premise

Nick's rule: the user only clicks corners 1-4; the robots correct themselves. The real warm-up at 09:52 (firmware
2026-10-28a, board measured 125.8 x 125.8 cm) stopped with "No camera fix for Left: no camera finds it on the board",
although the facing check had just found Left by its lights. Right's camera fixes: 76 of 77 ignored by the robot. Both
calibrations said "rolls allowed ... 15 cm" and then "no roll (no room in any direction)".

## Data

Simulator (mission_run.py --dry, both simulated cameras through the real camera bridge, robots on firmware 28a
lights, not calibrated since power-on), Monday's program:

| Board | Placed | Old code | New code |
|---|---|---|---|
| 126 x 126 | Left 35 cm off, backwards; Right 38 cm off, sideways | start-up failed, "No camera fix for Right"; Left's fixes 82 of 83 ignored | done 143 s; both driven 36 cm to their spots; re-check 0.4 cm / 1 deg; 0 fixes ignored |
| 61 x 122 | Left 35 cm off, facing -x; Right 28 cm off, backwards | (same failure) | done 99 s; re-check 0.2-0.3 cm / 1 deg; 0 fixes ignored |

<sub>Re-check: the facing check after the correction, distance and heading from the start pose. Data: each run's
terminal output and run/summary.json.</sub>

## Conclusion

1. **The bridge looked only round each robot's start spot.** It took its floor memory with the robots where they
   really stood (so a robot off its spot became part of the "floor") and searched near the map's start spots. Now,
   after every facing check, run_plan sends the bridge each robot's position and heading from its own lights; the
   bridge moves its track there, forgets the floor under the robot, and re-learns the empty start spot. This uses
   the lights because they are the one measurement that already finds a robot anywhere on the board and knows which
   robot it is (each lights up on command), so who is who never depends on where it stands.
2. **Calibration turns the robots.** The facing check now runs again after a calibration (the sim showed 24-25 deg
   turns), and the bridge is told again before straightening.
3. **76 of 77 fixes ignored:** a robot whose follow run has no pose yet refuses a fix without a heading, and the
   start (straightening) deliberately strips the heading until it gives the robot its pose. Those fixes are now held
   back until the robot has a pose (counted as "held" in the summary) instead of being sent and refused.
4. **The 15 cm roll vs "no room":** the laptop's caps are only the most a robot may roll; the robot rolls only where
   its own front sensor also sees 15 cm clear. The caps were also worked out from the plan's start spot and heading,
   not where the robot stood (Left was facing 149 deg away). Caps now come from the lights' position and heading,
   the screen says the robot's own sensor must also see room, and a skipped roll prints what the sensor saw. Most
   likely cause on the board (not provable from here): the sensor reads the floor (Left's floor reads 10.6-15 cm),
   so every direction looks blocked. A skipped roll is not a failure: the robot is calibrated, "limited".

## What's next

- Nick: confirm the floor cause from the 09:52 run on the Pi: in the Pi's terminal, `find ~ -name calib_look.csv -mmin -600 -exec tail -n 2 {} \;`
  (the room8 column: eight numbers all under 15 = the floor).
- Stage 2 (below, when pushed): no stops and no sit-outs.
