# Notes for next week's summary (started 2026-10-24)

Gathered as the week goes; the summary is written from these. Failures go in only with their plan.

## Themes Nick wants carried through
- **The system's strength is that it calibrates itself** (Nick, 2026-10-24: "I always want to emphasize" it).
  Every section should tie back to it: the robots learn their own wheel and gyro corrections, explore calibrates
  the camera from the robots' own driving, the boards' lines are moved to where the robots feel them.
- **Autonomy.** Each of these adds to the system running without a person:
  - runs on the Pi with nobody home (2026-10-24: a four-stage plan, started remotely, logs uploaded per stage);
  - the virtual border: a robot inside a line 15 cm from the open edges, moving towards it, stops everything and
    the session waits for Nick's answer from anywhere (built 2026-10-24);
  - a robot the camera loses while driving also stops everything and waits (built 2026-10-24);
  - the self-charging dock (planned: the Alvik's USB-C port through a short extension to our own contact, in one
    nose attachment with the switches and lights);
  - battery logged in every session (driving against idle, rested readings) to plan run length and charging.

## Planned experiment: degraded sensors (after the robots stay on the board reliably)
Nick, 2026-10-24: runs in which a sensor's input is deliberately degraded (or swapped for an ineffective one) to
see whether the program notices and adapts. Goes with the next build: the system learns how much to trust each
sensor for itself (docs/build-plans/e02-oct22-build.md, "Next build").

## Results this week (with their plans)
- 2026-10-24 Pi plan, stages 1-2: 30 runs, none failed; the Pi kept up (14-16 pictures a second, picture age
  0.12-0.13 s at the median). C (calibrated) ended 3.4 cm off on average against U's 21.9.
- K/KC: the two runs the camera kept hold of were the session's best (2.1 and 2.8 cm at the far point); three
  lost the robot at the start. Plan: the lost-robot stop, and the analysis no longer scores a far point the camera
  did not see.
- Learned corrections refused all of stage 2 (the camera noisier than the 1.5 cm check). Plan: the check from the
  camera's own measured noise (sensor-trust build); 2.5 cm for the Right-only session meanwhile.
- Explore drove Left and Center over an open edge (a pose kept from the session before read as a 112 cm drive).
  Fixed the same evening; the virtual border and the lost-robot stop now guard explore too.
