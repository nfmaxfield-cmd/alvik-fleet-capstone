# Decision: rebuild the test plan around self-calibration (Oct 19, 2026)

Chosen by Nick from the "Test Plan Decisions" panel.

1. **Old test list:** B. Self-calibration is the core; the questions of T2-T7 (sensor
   errors, merging gain, failures, delay, difficult scenes, several robots) are answered
   inside the calibration sessions; T8 and T9 optional.
2. **Out-of-the-box baseline:** A. Camera-measured and automated: a baseline session with
   corrections off (camera measures end error and its causes: wheel size, turn error per
   corner, drift), then paired corrections-on/off runs in later sessions. Oct 19's tape
   runs check the camera.
3. **What the robots touch:** A. Dock version 2 with a pair of posts on each of its three
   closed sides; two docks near the middle, diagonally offset, facing opposite ways
   (6 test sites, all four directions); machine models around the edges.
   Note (Nick): cinderblocks at a corner can be added if helpful, but likely redundant.
4. **Confirmation safeguards:** C. Ask only on disagreement (camera vs touch, moved
   fixture, 90-degree pair angles disagree, implausible correction, unexplained post
   misses), plus one tape spot check per setup.
   Note (Nick): everything except the docking stations should be movable, with the robots
   calibrating and aligning themselves with minimal user participation.
5. **Landmark identification:** C. Clicked at setup; robot-triggered (distance-sensor
   signature -> camera frame saved -> highlighted in the labelling queue -> user names it)
   for anything new or moved.
6. **Machine shapes:** C. Rely on the machines' overall shapes for now (no signature plates).

Implications:
- The docks are the fixed frame of reference. Everything else, including the camera, can
  move: the robots re-find the camera's calibration from the docks (their posts and check
  tiles are at known places), and re-find the machines by touch, distance sensor and camera.
- The tape spot check per setup can shrink to the first setup of the docks; after that the
  docks are the referee (proposed, to confirm with Nick).
- Firmware needs: per-robot wheel and turn corrections saved on the robot; the touch routine
  reporting both switch distances; the look-around floor filter.
