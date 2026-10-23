# Made hard on purpose: the choices, what each bought, and why it matters at scale

For the Saturday summary and the final capstone report (Nick, 2026-10-23). The framing: versatility and
adaptability (see the end), not difficulty for its own sake. The first three are Nick's; the rest are
easier routes we could have taken and didn't. A running list: add to it as choices come up.

| # | The easy route we didn't take | What we did instead | What it bought | At commercial scale |
|---|---|---|---|---|
| 1 | A good camera: high resolution, fixed lighting, straight down | One 640x480 webcam at 25 fps, at a slant, under basement light; a robot is about 22 pixels across; the picture size kept low on purpose (real-world conditions) | Tracking that works when the robot is a few grey pixels with a glare spot | Works with the cameras a site already has (security cameras), not new ones |
| 2 | Measuring and mapping the board by hand | The robots map it by feel: front sensor at the boards, edge sensors at the open edges | A map from the robots' own sensors, checked against the camera | No site survey before a deployment, and the map stays current as things move |
| 3 | Measured calibration numbers typed in (gyro scale, wheel scale, drift) | Self-calibration from the uncalibrated runs, refit as runs come in | The calibrated runs end about ten times closer than the uncalibrated (today's session, before statistics) | A new or repaired robot calibrates itself in its first runs; no technician time |
| 4 | Markers on the robots (AprilTag or ArUco printed squares): the standard way, and nearly trivial to track | No markers: the robot is found by its motion and followed by its own look, learned when it is handed over | A tracker that follows whatever is there, lights or attachments, with nothing told beforehand | Any robot or vehicle, any make, no retrofitting; markers can still be added later as one more witness |
| 5 | Robots told apart by colour or labels | Three identical robots, told apart by their own blink codes and by which one turns when told to | Identity that comes from the robot, not its paint | Fleets of identical units; a robot swapped in needs no new label |
| 6 | A trained recognition model (labelled pictures, a neural network) from the start | No model and no labelled data; the system's own tracking will label its own footage for training later | No hand labelling | Each site trains its own model from its own footage, at no labelling cost |
| 7 | Fixed start spots | Robots put down anywhere; each finds itself (a short turn the camera sees, then pushing into the boards) | Recovery from anywhere, which is also what a lost robot needs | A robot moved by a person or after a fault rejoins on its own |
| 8 | Rails or walls on all four sides | Two open edges, a real drop | Cliff sensing that had to work (line sensors and the distance sensor, both measured from logs) | Docks, loading edges and stairs are real; the robots sense them, not just the map |
| 9 | Several cameras, or a motion-capture system as truth | One camera; truth from that camera and the robots' own touches of the boards | A system honest about its own uncertainty (it knows when it is unsure) | Blind spots are normal on a site; the robots carry on through them on their own sensors |
| 10 | A lab-grade camera calibration (a checkerboard) | The floor calibration from the board's four corners only | The checks catch a calibration that is off (the felt boards against the picture) | Setup from what is already in view |
| 11 | Processing on a fast computer or a GPU, over a wired network | An ordinary laptop over home WiFi, the camera streaming from a Raspberry Pi | It had to keep up: newest picture only, and every picture's age measured (0.12 s typical today) | Cheap hardware; the delay is measured, so our approach can show it doesn't depend on it (the latency test) |
| 12 | A person on hand to rescue a stuck robot | A recovery ladder: retry, another robot searches, it finds itself against the boards, it drives out blind, and only then a person; backups so a session doesn't stop | 30 runs this morning with no help calls after the restart | Fewer operators per robot |

## The argument (Nick, 2026-10-23)
It is easy to make anything work under sterile lab conditions with the best equipment. This system is built to be
deployed anywhere, to use whatever equipment is there, and to stay accurate in poor conditions: versatility and
adaptability are the point, not a cost. Where a site does have better equipment or aids (a sharper camera,
markers, a survey), the system takes them as extra evidence; it never depends on them.
