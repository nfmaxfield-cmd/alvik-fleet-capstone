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
