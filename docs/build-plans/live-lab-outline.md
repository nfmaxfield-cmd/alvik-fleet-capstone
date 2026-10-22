# Outline: the live lab interface (draft for Nick, 2026-10-21; not started)

Nick: a UI for running the robots, primarily for Professor Sodhi; can start from scratch (no visual
model, no driving corrections); watch the system calibrate itself in real time; a live view of the
corrections and the Kalman filter's weights; linked to Maxfield IRL Tracking to show how a run trains a
visual model; a core part of the final presentation, streamed from the basement to a screen at the
university. Must be smooth, polished and easy to understand.

## What exists to build on
- E02 runner (tools/experiment): one-robot-at-a-time runs, find against the boards, four conditions,
  self-calibration from camera-measured runs, safety watchdog, video marks. Becomes the engine.
- Fusion (tools/fusion): an offline Kalman filter per robot (wheels, gyro, camera, blink, heading
  fixes) and a factor graph. Needs a live version that streams its state.
- Live tracker (tools/live) and the motion tracker (tools/experiment/e02_camera.py).
- Maxfield IRL Tracking (tools/recognition, tools/label): recordings, robot logs as outside data,
  draft labels, training, the "use it" tool with live roll call.
- Pi recorder (tools/pi/camrec.py): both cameras, preview streams, video marks.

## Proposed shape
1. Engine (laptop): a long-running service around the E02 runner and a live fusion filter per robot;
   publishes everything (camera fixes, robot state, filter state, corrections, run events) as a live
   stream; all control goes through it, with the safety rules unchanged.
2. Screen (browser, the same dark look): one big live picture with overlays (each robot's true
   position from the camera, its own belief, the filter's estimate with its uncertainty drawn as an
   ellipse, planned route, far point and end); side panels:
   - Calibration: gyro scale, drift, wheel scale per robot, each converging over the runs, with a band
     showing how sure the system is; "before" and "after" numbers.
   - Kalman filter: for each update, how much it trusted wheels / gyro / camera (the gains as plain
     bars), the surprise (innovation) and the uncertainty shrinking or growing; a plain caption that
     says what just happened ("camera disagreed by 3 cm: trusted it 80%").
   - Results: end error per condition building up run by run (dots + average + range).
   - Run timeline: what's happening now, what's next, time left.
3. Modes: From scratch (factory settings, no model) / Use saved calibration / Compare (the four E02
   conditions) / Free roam; Presentation mode (bigger text, captions, a guided sequence); Replay mode
   (plays a recorded session through the same screen: the fallback if the live link fails on the day).
4. Maxfield IRL Tracking link: each session becomes a project automatically (recording + robot logs
   as outside data + the filter's positions as draft labels); a panel shows "this run produced N
   labelled frames for free"; one click opens it in the studio; optionally trains a first model live.
5. Remote viewing: the screen reachable from the university over the internet, securely; who can
   only watch and who can control is decided below.
6. Safety: a big stop button for whoever controls; watchdog and edge guard as now; viewers can't drive.

## Build stages (rough)
A live fusion filter + engine service; B the screen (live view, panels); C modes incl. replay;
D the studio link; E remote access + security; F presentation polish, rehearsals, failure drills.
Each with a simulated end-to-end run, then a real one.
