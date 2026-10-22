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

## Nick's answers (2026-10-21)
1 Both: some presentations only watch, some have someone controlling. 2 Runs on Nick's laptop for now.
3 Control access is part of the design; Sodhi says Nick may want to monetise this software: keep a
running list (docs/monetization-ideas.md). 4 Controlled from Nick's laptop, remotely; nobody at home.
5 Nothing scheduled; plan for ~10 minutes. 6 Probably a TV. 7 Headline: it calibrates itself, and
accurate labelling and tracking need minimal user interaction. 8 A script for Nick; captions additive to
his words or able to stand alone. 9 "Weights" = how much the filter trusts each sensor; show the
calibration corrections too. 10 Same as his manuals: conclusions up front with a small note of what
backs them; supporting data in appendices (an engineering view one click away). 11 A person designates
the work area and the robots; then the program calibrates everything itself, camera included.
12 Two presentations: a live one that trains live; a recorded demo with the script and captions.
13 Yes: no models and no calibrations at the start (the name "from scratch" isn't required).
14 Internet looked fine during setup. 15 Scripted: whatever makes sense; live training demo: a condensed
version of tonight's E02 session. 16 Docks and lights included if still in use by then. 17 Sometime in
December. 18 Three robots now, a fleet next semester: build for any number. 19 "Live lab" mode of
Maxfield IRL Tracking.

## More answers (2026-10-21, 21:14)
- Absolute scale: solved with the Alvik anchors (docks); other known references (robot size) confirm
  and strengthen the estimate.
- Live demo: yes, show the visual model improving live too; think of live displays that captivate the
  audience; a second screen can run alongside the first.
- Recorded-demo script: not yet; keep running notes and questions (docs/presentation-notes.md).
- Look: it should feel like a commercial product.

Second-screen ideas (Claude, to discuss): the model's guesses drawn over the live picture getting
tighter round by round; a "labels made for free" counter ticking up with a small mosaic of the frames;
each sensor's trust as a live meter that shifts when the camera and wheels disagree; uncertainty
ellipses shrinking as calibration settles; the learning curve climbing as each training round finishes;
a per-robot "how sure am I" card.
