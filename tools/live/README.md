# The live link (tools/live)

run_plan.py can take camera positions while the robots drive (`--fixes udp`).
This folder makes them from the real overhead camera: it finds each robot by
its blinking top lights, names it by the blink rhythm, and sends its floor
position and heading, as the frames arrive. How to run it:
tools/fleet/README.md, "Running with the live camera".

| File | What |
| --- | --- |
| `track_live.py` | the program: `--stream <pi>:8000` (live) or `--replay <recording>` (test), sends to run_plan.py's port, `--dry-run` prints |
| `livecore.py` | the tracker, one frame at a time (built on tools/blink/blink.py's stages) |
| `sources.py` | the Pi's `/live` stream, recording replay in real time, Pi -> laptop clock |
| `floormap.py` | pixels -> floor cm with floor_calibration.json (lens, homography, light height) |
| `obstacles.py` | hook for camera obstacle reports (not built yet: why, and how to add one) |
| `capture_delay.py` | measures the camera's delay from one run (sync flashes) for `--capture-delay-ms` |
| `eval_live.py` | scores a replay of a simulated recording against the simulator's truth |
| `tests/` | `python -m pytest tools/live/tests -q` (about 20 s) |

## Where it runs: on the laptop, reading the Pi's stream

camrec.py on the Pi already decodes every frame to record it. It now also
serves every frame at `/live` (grey JPEG with the frame number and Pi time),
only while someone is connected. track_live.py on the laptop reads that.

| | Pi does the tracking | **Laptop does the tracking (chosen)** |
| --- | --- | --- |
| Extra work on the Pi per frame | tracker: 2.9 ms median, 7.8 ms p95 on this test machine, so roughly 10-40 ms on a Pi (assumed 3-5x slower) = 25-100% of a core at 25 fps, next to camrec's own decode + record | grey JPEG: 0.8 ms here, roughly 3-4 ms on a Pi = about 10% of a core |
| Added delay | tracker time on the Pi (10-40 ms) + UDP | JPEG + WiFi (22 KB a frame, about 0.55 MB/s) + 0.8 ms decode + 3 ms tracker |
| Pi -> laptop clock | needs NTP or a time exchange | measured from the stream itself (below) |
| Risk | frames dropped from the recording if the Pi falls behind | WiFi hiccups delay fixes (the robot rewinds to the picture's time anyway) |

The two delays come out about the same (both estimates; the laptop design
was measured at 46 ms median from picture to fix in replay, see below); the laptop design keeps the Pi's
recording safe, needs no clock set-up, and is easy to change. The Pi numbers
are estimates (no Pi here): measure with `top` while it runs.

Clock: laptop arrival time - Pi t_wall = clock offset + network trip; the
smallest value over the last 30 s is the offset (+ the shortest trip, a few
ms). A fix's time = t_wall + offset - capture delay (one frame unless
measured with capture_delay.py).

## How it finds and names the robots

1. Every frame: blink.py's spot finder and on-segments (bright spots that
   switch on, then off), run frame by frame (`blink.SegmentFinder`).
2. A flash = the two LEDs lighting together; flashes are chained into a track
   per light (blink.py's `assign_batch`). A track that could have taken
   another robot's flash (two robots close, flashing at once) gets no fixes
   for its next 3 flashes. Two tracks within 5 cm are one robot (merged).
3. Name: blink.py's rhythm decoder over the track's last 12 flashes, needing
   p >= 0.9, 5+ intervals, 75% on the beat, no other flashing on its spot,
   and **no other track anywhere else carrying (or likely to carry, p >= 0.5,
   or carried in the last 3 s) the same name**.
4. Heading: the line between the two LEDs is square to the heading. Front or
   back: the robot's body is behind its lights, so the picture just behind
   them differs from the floor beside the robot more than the picture just
   ahead (plus, weakly, the direction it drives). This evidence must add up
   to 2 before any fix is sent; between flashes the heading follows the
   nearest of the two candidates (turns up to 100 deg/s).
5. Position: the light midpoint (corrected for the lights being 4 cm up)
   minus the light's offset from the turning centre (2.75 cm forward, or each
   robot's from calibration_results.csv).
6. Sent: `fast` from the frame itself when a named robot's two LEDs light up
   where and when expected (on its beat, near the predicted spots, right
   spacing, dark the frame before, no other robot's light expected there);
   `late` for the rest of each flash once it has ended and passed every check
   (about 0.2 s later; the robot still applies it at the picture's time).

## Measured (simulated; not yet on hardware)

Simulated E1 sessions with the real Oct 18 camera (`tools/sim/configs/real_oct18.json`:
262 cm up, 3.45 px/cm, 25 fps), replayed through track_live.py; errors
against the simulator's truth (the turning centre):

| session | robot | fixes | wrong name | position error rms / max, cm | heading error p95 / max, deg | fixes a second while blinking | blinking seconds with a fix |
| --- | --- | --- | --- | --- | --- | --- | --- |
| seed 11, full E1 sheet, 43 min | Left | 2649 | 0 | 0.03 / 0.10 | 0.65 / 1.2 | 4.4 | 70% |
| (robots take turns) | Center | 2006 | 0 | 0.06 / 1.78 | 0.64 / 1.5 | 3.4 | 67% |
| | Right | 1549 | 0 | 0.06 / 0.15 | 0.75 / 1.4 | 2.6 | 64% |
| seed 7, 11 min | Left | 2179 | 0 | 0.05 / 1.37 | 0.61 / 2.9 | 4.6 | 71% |

Fixes come only while the lights are on: Left flashes every 0.3 s, Center
0.4 s, Right 0.5 s, each flash on for 2-3 frames. Blinking seconds without a
fix: the lights dark through blocking calls, the 2-3 s it takes to name a robot
at the start of each run (once per run_plan run), and flashes where only one
LED was seen (about 18%).

Delay, real time (3-robot recording replayed at its own speed into a listener
that uses run_plan.py's own UDP receiver, same computer): picture taken -> fix
received, `fast` median 46 ms, p95 50 ms; `late` median 207 ms, p95 250 ms;
of which 40 ms is the assumed camera delay (one frame). 89% of fixes were
`fast`. Processing 2.9-3.8 ms per frame (median), 7-9 ms p95.

capture_delay.py on the simulated session: 44 ms (simulator: 40 ms + up to
a few ms jitter).

Synthetic scenes (tools/blink/tests/synthworld.py: cluttered floor, lamps
flickering at robot-like rates, two robots crossing and passing 11 cm apart,
spins at 180 deg/s), 4 seeds each of clutter and crossing: 0 wrong names,
0 fixes more than 3 cm off; no fixes at all in the distractor scene; two
robots blinking the same code: no fixes.

## Not tested on hardware

Everything above is simulated. On the real set-up check: the Pi's CPU with
`/live` connected (`top`), the WiFi frame rate (live_summary.json
frames_missing), the capture delay (capture_delay.py), whether the real
robot's top looks different enough from the floor for the front/back vote
(live_summary.json `unresolved`; a robot that never gets fixes although named
is this), and the light offset (calibration_results.csv).

## Next

- Obstacle reports (obstacles.py says what is needed).
- The detector (tools/detector) once a model is trained: it gives heading and
  position between flashes (blink alone gives fixes only while the lights are
  on: 2-5 a second).
- A closed-loop test: fleet_sim.py's robots drawn into live video, so
  run_plan.py drives on these fixes (today the replay feeds a listener that
  uses run_plan.py's own receiving code).

## Tests that may now be needed

- On the real set-up: one run_plan.py run with `--fixes udp` and the camera
  recording, then capture_delay.py; compare live fixes with the tape marks.
- A real robot's top under the real lights: does the front/back vote settle
  (fixes start within about 3 s of the blinking)?
- Two real robots passing close: count `ambiguous` and `contested` in
  live_summary.json and check no fix jumped between them.

## Possible inaccuracies in what this relies on

- Pi CPU and WiFi numbers are estimates (a Pi assumed 3-5x slower than the
  test machine; WiFi not measured).
- Capture delay assumed one frame (40 ms) until measured.
- Light height 4 cm, light offset 2.75 cm forward (simulator's model) unless
  calibration_results.csv is given.
- The front/back vote assumes the robot's top looks different from the floor;
  the simulator's robot is plain grey, the real one is not simulated.
- Thresholds chosen, not derived: name p 0.9, front/back evidence 2, fastest
  turn 100 deg/s, 3 flashes held after an ambiguous one, 5 cm merge radius,
  fix sigma 0.5 cm / 1.5 deg.
- All accuracy numbers are simulated; the simulator has no reflections or
  real lighting.
