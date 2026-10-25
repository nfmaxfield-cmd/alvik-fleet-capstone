# Right's session (2026-10-24 19:35) and the recorder's 21 pictures a second

Written overnight 2026-10-25 from the uploaded logs and the recorder's frame list (release
e02-session-20261024-193526). Conclusions marked *(Claude)* are mine, not the group's.

## Premise
Right ran alone at 19:35. Within 4 minutes the session held twice (virtual border at setup, then "lost robot"), Nick
answered `s` at 20:43, the tracker summary said "none 93%", and the recorder warned "only 21 fps of 25".

## Data
**Recorder.** 396,223 pictures over 265.6 min, 24.86 a second; the camera's own clock shows 0.54% missed.
Per minute it was 25.0 except **19:57–20:15**, when it fell to 21.4–24.2. The same session's runner handled only
11–14 pictures a second in that window (65–85 ms a picture), against 19–21 after it.
The second upload (latency-test-20261024: 5.6 GB read from the same USB drive, split and sent) ran
**19:38:56–20:15:47**. Both the recorder and the runner came back to normal the minute it ended.

The afternoon recording (15:36–17:39, no upload running) was a flat 25.0 for all 124 minutes, with no gaps.

**The "lost" hold at 19:39:33.** Right drove a 10.8 cm leg. For 3 s after it stopped, the camera reported
(32.4, 113.0) cm, 9 cm past the robot; then it reported (23.5, 105.2) cm, where it stayed for the hour. The leg's
start was (14.9, 97.9), so the robot had moved 11.3 cm, as its wheels said (11 cm). The hold's "camera saw it move
24 cm" was the ghost, read during those 3 s.

**"None 93%".** For 64 of the session's 68 minutes the session was holding. During the hold only the detector
ran and the lock-on tracker was off, so most pictures counted as "none" even though Right stood in plain view; the
detector saw it at the same spot (within 0.3 cm) 1,502 times. *(Claude)* The 93% describes the hold, not a camera
that couldn't see Right.

## Conclusions *(Claude)*
1. The 21 pictures a second came from running an upload during the session (CPU and USB-drive contention). The
   camera and its settings were fine. The normal after-session upload doesn't overlap a session, so nothing to change
   in the recorder; **don't start uploads during a session**. A mid-session upload for latency data would need its
   own planned test, and this one already shows the cost: the runner's picture rate roughly halved.
2. The lost hold was a false alarm: a ghost held for 3 s right after the leg. **Fix (pushed):** before calling a
   robot lost, the runner waits RECHECK_S (3.5 s), looks again, and carries on if the camera now agrees with the
   wheels. That adds 3.5 s only when they disagree. DOE note: 3.5 s comes from one event (a 3 s ghost); count
   rescues against holds over the next sessions.
3. The setup border hold at 19:35:51 was right to stop: Right was placed 13 cm from an open edge.

## What's next
- Tracker summary: count hold time separately, so "none" isn't inflated by a hold.
- Look at the ghost in the video (part aa covers 19:35–about 20:15) to see what the camera locked onto. Possibly
  Right's own lights or a reflection.

## Sweeps
- **New tests that may be needed:** a simulator case with a 3 s ghost after a leg (the recheck should rescue it); a
  planned mid-session upload test, if latency under load is wanted for the paper.
- **Possible inaccuracies:** the upload/frame-rate link is a timing match (end to the minute, start only partly:
  19:39–19:57 stayed at 25), not a controlled test; the first part of an upload may be lighter (reading a cached file).
  The ghost's cause is unknown until the video is checked.
