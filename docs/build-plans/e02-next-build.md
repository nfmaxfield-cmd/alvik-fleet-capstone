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
- (2026-10-22 08:20) **The live picture reaches the laptop at only about 2 frames a second** (setup: "5 pictures in
  2.4 s"; last night the same). Everything was tested on the 25 fps recording. At 2 fps the flash finder can't work
  (the lights blink at 2 Hz: the samples alias) and tracking is far coarser. Find where the rate drops (Pi preview
  encoding, WiFi, the laptop's reader), measure it during the session, and either raise it or make the finder use a
  slower blink (needs the non-blocking light command Nick approved) / do the flash finding on the Pi.
- (2026-10-22 08:19) **Left lost twice in the bottom-left corner** (the left board's shadow): found again by the turn
  at (40.5, 102.7), lost again backing into its park spot (17, 108). Move that park spot out of the shadow, or make the
  tracker cope with low contrast there (per-area thresholds from the floor memory's noise).
- (2026-10-22 09:10) Firmware 2026-10-22a adds the TOUCH bump cue (front sensor at most 1.5 cm for 150 ms) because
  the real tyres slip at ~96% speed against a board (no stall). In the simulator with that slip, push-to-square
  sometimes can't turn the robot square (1 find in 6 came out 7 deg off, judged "squared" because the heading
  stopped changing). Watch the real finds' square_deg / reset heading; if it happens, check squareness from the
  camera's back-off heading and redo the find, or push harder / longer.
- Park spots: moving Left's spot out of the left board's shadow broke the route clearance checks (routes pass
  near); needs a joint re-plan of park spots and routes, not a nudge.
