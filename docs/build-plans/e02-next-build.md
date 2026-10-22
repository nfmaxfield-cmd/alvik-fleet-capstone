# E02 runner: notes for the next build (to brainstorm with Nick first)

- (2026-10-22 08:10, Nick) **The starting outline (floor calibration) is better but not perfect yet.** On the
  morning's check picture the green/yellow outline sits a little outside the plywood on the left, and the
  corners don't all land on the board's corners. It comes from roam1's corner clicks (overhead_floorcal_roam1.json);
  ideas to brainstorm: re-fit from today's picture; let the robots' bumps and the touch map refine it (the find check
  and the mapping measure the boards directly); a 4-corner re-click in the studio; check whether the camera was
  nudged.
- (2026-10-22 08:10) **Parking dead-locked with robots spread one per corner, 30 cm from the walls.** Each robot's
  park spot or the point it backs in from was within 24 cm of another robot, so "every spot is taken"; Left was sent
  to the middle (not checked for room), the watchdog stopped it near Center, then Center found "no clear way round".
  Fix ideas: plan all three parkings together (an order plus temporary holding spots that are checked free), use the
  open middle only when it is free, a looser "taken" test for the backing-in point, and a test with the robots in
  the three corners Nick used (simulator --dry-start).
- The Pi recorder's disk warning overestimates 4x (says 2.0 GB per 10 min; real is about 0.4 GB): fix the estimate.
