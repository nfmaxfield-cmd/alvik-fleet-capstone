# Standing rules for work in this repo

- **Screenshots (Nick, 2026-10-26):** every build saves screenshots of the features it adds and the problems it
  finds or fixes (live page and live lab via Playwright on dry/simulator runs, charts, terminal output where it
  helps), so Nick can show them to people. Save to `docs/screenshots/<YYYY-MM-DD>/` with descriptive file names and
  add one line per picture to `docs/screenshots/index.md`: what it shows, **feature** or **problem**, and the build
  or report it belongs to. Append to the index; never overwrite another build's lines.
- **Heat tracker (Nick, 2026-10-26):** every real session's analysis runs `python3 tools/analysis/heat_tracker.py
  <session folder> [--cooling=<what cooling the Pi had>]`, which adds the session's row and chart to
  `docs/results/heat-tracker.md`, plus a line under "Readings" saying what it shows.
- **Keep features, switch them off (Nick, 2026-11-02):** when a new method replaces an existing feature (e.g. corner
  tags replacing corner clicking), never delete the old feature. Keep it working and put it behind a setting or
  switch (per site where it makes sense), off while it isn't needed, so it can be turned back on.
- **Discover, don't pre-map (Nick, 2026-11-05):** for every Alvik adjustment (calibration, odometry corrections,
  surface features such as table seams, bumps, slippery patches), build systems that DISCOVER the issue from the
  robots' own data and the camera and then account for it — the goal is dropping a new robot on an unknown surface
  and driving on it accurately. Mapping a feature by hand (e.g. typing a seam's position) is allowed only as a
  stand-in for initial data-gathering runs, never as the finished method.
- **Observe, then trust (Nick, 2026-11-05):** every belief the software holds about the physical setup (how sensors
  and tags are mounted, names, addresses, positions, heights, clocks, surfaces, camera coverage, robot state) is a
  starting guess, not a fact. (1) Check it against live evidence at the start and throughout (e.g. the first drive's
  direction checks the tag's heading). (2) When evidence disagrees, the evidence wins: adapt, log what was learned,
  carry on; stop only when safety can't be kept. (3) Fuse every available signal (tag pose, motion in the picture,
  wheel odometry, gyro, accelerometer, front and line sensors); a disagreement between sources is information, not a
  reason to drop one. RFID: see "Jay-Sun's RFID data" below. (4) Degrade gracefully: a lost tag, a missing
  camera or an unreadable area narrows what a run does instead of ending it. (5) The simulator deliberately breaks the
  guesses (tags turned, wrong camera names, a missing camera, clock skew, surface marks, robots restarting, dark
  patches) so tests catch rigidity before the real table does. "Discover, don't pre-map" above is this rule's surface
  case. Switches kept under "Keep features, switch them off" are decided automatically by default and live out of a
  new user's way.
- **Jay-Sun's RFID data (Nick, 2026-11-10; Jay-Sun agreed):** Nick's project and Jay-Sun's thesis are two separate
  projects sharing hardware and most of the software; each may use the other's data, as long as neither interferes with
  the other's experiments. RFID data may be an input to Nick's own runs and analyses, but only behind a switch that is
  OFF for every run made for Jay-Sun's experiments (tours, experiment routes), so his runs never use his own RFID data.
  Raw RFID data never goes in this public repo (keep it private); Jay-Sun keeps full access to all raw data. Whether RFID
  improves accuracy is an open question to test against camera truth, not an assumption.
- **Derived, risk-based thresholds (Nick, 2026-11-10):** thresholds and timeouts (e.g. how long without camera pictures
  before warning, how long before acting on a robot) are derived from the data and vary with the actual risk (e.g. how
  likely the robot has drifted near an edge given its speed, heading and uncertainty), never fixed arbitrary times;
  where a fixed number is unavoidable, it is marked "chosen" with a DOE note. Robots never "give up" on a robot: one
  held at an edge waits for help and rejoins when moved; it is never dropped from the run by a timer.
  **What "never give up" means (Nick, 2026-11-10 12:51):** a robot that has lost the camera (or any source) keeps moving
  on what it has left (its own odometry and gyro, motion, lock-on) for as long as the sum of its possible error could
  NOT carry any part of its body off an edge: its position uncertainty bubble, grown with every cm and degree driven
  blind, plus its body and stopping distance, stays inside the edge rule along the whole planned path. It stops only
  when that sum could reach an edge, then holds safely and asks for a hand, and carries on when a source returns. No
  fixed blind-distance caps (e.g. "stop after 8 cm") and no timers.
  When it holds, it asks the person (Mission Control, and the terminal) to click the robot on a picture from any camera
  that shows it (Nick, 12:56). The click sets its position (two clicks, centre then nose, also set its heading),
  resets its error sum to about the click's own accuracy, and it starts moving again with a new running total. A robot
  no camera shows: the person puts it on a known point and clicks that point on the map instead.

