# Open items and Nick's answers (2026-10-25, 14:54)

Nick: "For now I just want to get the robots running ... we can circle back to the rest later on."

## Nick's answers to fold in
- IISE 2027 abstract: **submit** (plan it; deadline about 8 November).
- Unknown-robots build (104-152 h): make a smart plan: use the cloud credit if possible, or about 1/4 a week, with simulation work running during all laptop down time when it doesn't use tokens.
- $250 cloud credit: the meter has never moved in weeks; look up how it's used (support.claude.com).
- Learned trust between sessions: carry as history (Fresh) — agreed; Nick asks **when discrepancies are addressed**: answer and design that.
- Printed nose piece fits perfectly everywhere: does that confirm the Alvik dimensions? (answer: largely yes for the shell width; note what it doesn't confirm).
- Delete the latency-test release: **yes**.
- Weekly summary: due **Saturday**; keep notes on what it should contain (sensor trust / degraded tests, DOE section, adapting to a slower computer, simulation register, robot differences, conclusions, sweeps, appendix).
- Prints: plan to get prints done ASAP; have questions ready about the attachment design.
- Board-in/board-out picture test: OK.
- Switch placement experiment: **switches on the back of one Alvik and the front of another**; test the effectiveness of each.
- Mid-run environment-change experiments (lifting a board out etc.): **postponed to next week**.
- Discovery: "which approach next" — Nick asks what I suggest.

## Still to ask Nick later
- Discovery: acceptable wrong labels per 100 (I assumed at most 1); hand tilt/lift vs hands-free wedge.
- Mark page: tape vs firmware switch for the lights-off test; the ±2% pass mark; marks to one release vs each session's.
- Dock: cover measurements (steel rule), caster weight (kitchen scale), 38% lift acceptable?, cross-piece soldering, spring sizes, tape type.
- Runner: plywood-edge hand test; whether a missing board stays missing for the session.
- Simulator wheels 7.5% slower than commanded (to match real): do after the trust fix's runs.
- Flash firmware 2026-10-25b on all robots (needed for the robot's own border, sleep, back switches).
- GitHub token: laptop setup (new token or regenerate + Pi).

## Added after 15:11 (discovery threshold result)
- Rule in code: accept at 0.90, motion labels held until 60 s of wheel motion (held-out: 1 wrong in 1,374; 11.5 questions/robot).
- Ask Nick: is 1 wrong per 100 accepted the right limit? Calibration run as a runner stage (square drive, one-wheel bump, edge approach) hands-free, or with a 5 s hand nose-tilt + lift? Edge guard or printed black strip for the edge approach? Print the bump strip (STL)?

## Added 15:3x (first real-session fusion, docs/build-reports/e02-oct25-real-fusion.md)
- Fused wheels+gyro+camera beats wheels+gyro alone by 15-30% over camera gaps (2.2 vs 2.7 cm at 5 s, 3.8 vs 4.6 at 20 s, 5.1 vs 7.3 at 40 s) and halves the robot's own pose error.
- Biggest finding: the robot steers by the IMU's yaw (7% short until calibrated); the raw gyro rate with drift measured at standstill gets uncalibrated routes to 2.0 cm vs 15.9 cm.
- Ask Nick: heading from the raw gyro rate in firmware (simulator first)? K routes on factory corrections? Live lab fix now (display only: one filter per robot started at the camera, threshold 120)? Longer routes for 40 s+ drift data?
- Also: Note button on the live page (yes/no); Left-Center collision at ~15:21 and Right's edge approach at 15:15 to diagnose.
