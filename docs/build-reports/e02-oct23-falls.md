# Two robots off the board during explore (2026-10-23, 22:25 session): first look at the logs

Session logs/e02-20261023-222510 (Nick's zip). Footage: Pi recording 20261023-22*_E02 (being copied). Private note
to Nick until there is a tested fix (summary rule).

## What the logs show (robot-logs-1/leg.csv, edge.csv)
- **Right went over at 22:35:55** (laptop 1792809355.9): a 105 cm ray, then a turn of 4.7 deg and another ray; 5 cm
  in, the **line sensors** tripped (right sensor 214, the others 58 and 75: one sensor over, the edge met at a
  slant) and 3.8 s later all three read off the plywood (630 / 652 / 628, "no forward drive"). The distance sensor's
  bottom zone (the cue that sees the drop ~9 cm ahead) **did not trip** on this approach; it stayed 7-8 cm.
  Its back-off (9.5 cm) ran, but after it one sensor stayed over the tape (lineR ~420) for the rest of the session:
  perched with a wheel over.
- **Center:** three full 105 cm "rays" in a row (22:31:05-22:31:59) with no edge trip and the front sensor at
  10-16 cm after every one, even after turning round: wheels turning without getting anywhere (pushing against
  something), then a straight drive that turned 10 deg on its own and turns that timed out (22:32:45 on): a wheel
  hanging. Its last reading on the board (22:33:31) had the left line sensor rising (58-82). Found with two sensors
  over the tape at 22:40:06. While it stood there, Left drove its second round of rays nearby and was stopped by
  "obstacles" six times.
- **Left** stayed on: every edge approach tripped the bottom zone first.

## Likely causes (to confirm on the footage)
1. **A slanting approach to an open edge leaves only the line sensors**, which trip when a sensor is already over
   the edge; at ray speed the robot stops a few cm later, past the edge (Right). The bottom zone looks straight
   ahead, so at a slant it can still see plywood.
2. **Robots exploring among each other:** explore rays do not check the other robots' places (mapping's
   `ray_blocked` does), so rays end against robots, a robot can be pushed, and two robots near one edge leave
   little room (Center).
3. Possibly the blind back-offs (raised allowance this build) near a corner. To check on the footage.

## Fix options (for Nick in the morning; nothing changed yet)
- Rays slow down where the edge could be: full speed for the first part, then a creep (the firmware's find uses a
  creep against the boards for the same reason).
- Rays checked against the other robots' places (provisional camera positions are enough for that).
- Explore one robot at a time with the others parked well away, or give each robot its own sector.
- Edge guard tuning from tonight's real readings (first real edge stops at ray speed: 7 bottom-zone, 1 line).

## The footage (release e02-session-oct23-night, part aa)
Both robots went over the **right-hand open edge** of the picture. Frame 14944 (22:32:54) shows one past the
plywood's end, lower right; frame 24500 (22:39:15) shows robots along that edge. The 22:45 snapshot shows Center off
the top-right corner and Right off the right edge a third of the way up. Center had been put down about 10 cm from
that edge.

## Fixes built 2026-10-24 (Nick: all four)
1. Room check first: 12 cm crawling drives four ways before anything else; a robot that meets an open edge moves in
   12 cm the opposite way first.
2. Rays, the square and the way home crawl at 4 cm/s (`EXPLORE_CRAWL_CMS`); returns over ground just driven at
   10 cm/s. DOE: docs/experiments/DOE-explore-crawl.md.
3. A ray that would pass within 25 cm of another robot is skipped (its own frame laid on the provisional floor by
   its stops so far).
4. The robot with the most room explores first.
`e02_cms` and `e02_back_max_cm` are put back after explore and at every session start (SAFETY_SETTINGS).
