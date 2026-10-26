# Screenshots: features and problems

Nick (2026-10-26): screenshots are saved during every build so features and problems can be shown with pictures.
One line per picture: what it shows (feature or problem), and where it comes from. Builds append below.

## 2026-10-25 (Nick's own, real sessions)
- `2026-10-25/1613-right-past-border.png`: **problem.** Right parked inside the bottom border band at 10 pictures a second. Cause: Right's turns had failed since 15:58 (its compass check learned the wrong direction); fixed in firmware 2026-10-25c and the 15:45 fixes. Report: build-reports/e02-oct25-session-1545.md.
- `2026-10-25/2127-false-lost-hold-center.png`: **problem.** A "lost robot" hold on Center while the camera clearly sees it. Cause: a back-out command lost on the WiFi (about 1 in 10 commands lost that evening). Report: build-reports/e02-oct25-session-2114.md.
- `2026-10-25/2132-false-lost-hold-right.png`: **problem.** The same false hold on Right, with the hold panel and its four answers. Same cause.
- `2026-10-25/2253-left-over-corner-edge.png`: **problem.** Left left out and hanging over the bottom-right corner ("0 cm from the nearest open edge"). Cause: the camera border blocked its back-out, then explore turned it on the spot at the edge. Report: build-reports/e02-oct25-session-2114.md.

## 2026-10-26
- `2026-10-26/0956-session-start-tracker-process.png`: **feature.** A session starting on the Pi: the session timeouts listed, camera tracking in its own process on cores 2 and 3, all three robots on firmware 2026-10-25c, every compass check done.
- `2026-10-26/1000-left-washed-out-lost-hold.png`: **problem.** Left on the board but washed out (white body on bright plywood); the camera's circle sits on empty plywood about 10 cm away, giving a "lost robot" hold. Nick: the blinking lights should have found it. Fix going into the 2026-10-26 runner build: try the lights before asking. Also shows the new tracking at 21 of 25 pictures a second.
- `2026-10-26/1002-refind-jump-hold.png`: **problem.** Right after the washed-out hold, the camera found Left correctly (circle on the robot), but the runner held again: "it only turned and the camera's spot jumped 40 px". The jump was the camera correcting itself. Fix going into the 2026-10-26 runner build: a jump confirmed by the robot's lights or its expected position counts as a re-find, not a hold.
- `2026-10-26/1007-border-hold-center-right-edge.png`: **problem.** Explore: Center at the right open edge after an edge creep (red dot = where its own edge sensor stopped it); the camera border holds it as "0 cm from an open edge, moving towards it", most likely while it backs out, the same pattern as Left at 21:42 on Oct 25. Also shows the explore rays (coloured lines) and both edge stops. Fix in the 2026-10-26 runner build: a back-out after the robot's own edge stop is always allowed.
- `2026-10-26/1044-explore-outline-question.png`: **feature.** Explore's own outline of the board, shown inside the runner's question on the live page: green = the blocked boards (left, top), yellow = the open edges (right, bottom), all four on the real edges. First time explore found the board on the real board; the new tracking at 26 of 25 pictures a second.

### Start page (build: e02-oct26-launcher; dry runs against the simulator, stand-in cameras with the Pi's names)
- `2026-10-26/launcher-01-pin-wrong-phone.png`: **feature.** The PIN screen on a phone after a wrong PIN ("Wrong PIN."). Report: build-reports/e02-oct26-launcher.md.
- `2026-10-26/launcher-02-idle-desktop.png`: **feature.** The start page on a computer with nothing running: session, hours, home/away, setup, video, side camera, robots; cameras by name with their role; free space.
- `2026-10-26/launcher-02-idle-phone.png`: **feature.** The same on a phone.
- `2026-10-26/launcher-02b-batteries-no-answer-phone.png`: **feature.** "Ask the robots now" with no robot reachable: says which addresses did not answer.
- `2026-10-26/launcher-03-missing-overhead-desktop.png`: **feature (an error shown).** Overhead camera unplugged: red "can't start", what the Pi does see, Start greyed.
- `2026-10-26/launcher-03-missing-overhead-phone.png`: **feature (an error shown).** The same on a phone.
- `2026-10-26/launcher-04-confirm-start-desktop.png`: **feature.** The in-page confirmation listing every choice before a start; More options open.
- `2026-10-26/launcher-05-refused-second-start-phone.png`: **feature.** A second start from another phone, refused: "A session is starting already."; Start locked.
- `2026-10-26/launcher-06-starting-desktop.png`: **feature.** Starting: what was chosen and the seconds so far.
- `2026-10-26/launcher-07-running-desktop.png`: **feature.** Running: the runner's current question, the live page button, Stop, batteries from the session.
- `2026-10-26/launcher-07-running-phone.png`: **feature.** The same on a phone.
- `2026-10-26/launcher-08-live-page-on-8090-desktop.png`: **feature.** During the session port 8090 is the live page again, with the new "start page / Stop" link at the top right.
- `2026-10-26/launcher-09-confirm-stop-desktop.png`: **feature.** The in-page Stop confirmation.
- `2026-10-26/launcher-09-confirm-stop-phone.png`: **feature.** The same on a phone.
- `2026-10-26/launcher-10-stopping-desktop.png`: **feature.** Stopping: every robot being stopped and saved; the upload follows.
- `2026-10-26/launcher-11-uploading-desktop.png`: **feature.** Uploading: progress bar, MB sent, minutes to go, current file; Start locked.
- `2026-10-26/launcher-11-uploading-phone.png`: **feature.** The same on a phone.
- `2026-10-26/launcher-12-done-idle-desktop.png`: **feature.** Back to idle: last session, how it ended, the upload link.
- `2026-10-26/launcher-12-done-idle-phone.png`: **feature.** The same on a phone.
