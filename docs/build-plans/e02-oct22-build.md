# E02 build after the 2026-10-22 morning session

Session: 09:51-10:43, release `e02-session-oct22` (video split in two: `cat video.part-aa video.part-ab > video.avi`).
1 run finished (Right U straight, far point "100 cm off": the camera did not follow it, so not a real result).
Nick away from 11:05; he asked: work until the usage runs out, as much in parallel as possible, then make the
logic more generic for other equipment and sensors.

## What went wrong (from the logs, the track and the footage)
1. The tracker lost moving robots for seconds and jumped: at 10:40:03, during Left's 167-deg turn, it took Right
   (7.6 cm from Right's last spot, just outside the 7.6 cm "another robot" zone) for Left; then "the camera saw it
   move 0 cm, its wheels say 10 cm" (it was watching Right). The runner gets all 25 pictures a second (track.csv).
2. Its position flipped 1.5-6 cm between two pieces of one robot, frame after frame.
3. A robot standing still with the tracker's position a few cm off was learned into the floor bit by bit; its
   outline then broke up (footage 10:40:12: the floor memory holds a ghost of Left).
4. "Something in front: turning away" looped up to 13 times: the turn legs between reset the count.
5. Every park spot is 17-20 cm from an open edge, under the 20 cm "move it in first" limit; by the x-board the
   move-in point was 1.4 cm from the board's line.
6. Center fell off the open edge (10:23): found 3.5 cm off the board, then driven blind; its edge guard had learned
   with all three line sensors over the edge ("all three on tape"), which switched the line cue off.
7. Pressing Enter with no hands nearly always worked: the help call was only a retry.
8. Right never named in the naming check (it was not parked: its parking after help did not finish).
9. One find check landed 45 cm off on the x-board (two robots' touches 93 cm apart): not explained yet.

## Done in this build
- [x] 4: obstacle legs counted on their own; back off 8 cm, then a quarter turn towards the board's middle
- [x] 5: move-in only within 14 cm of an open edge, to a point 25+ cm from both board lines
- [x] 7: three tries on its own (fresh floor, turn, heading, park) before Nick is called
- [x] 6: runner: a robot found at or over an open edge is never driven (Nick); firmware 2026-10-22b: all three
      line sensors off the plywood at the start = no forward drive that run (simulator mirrored)
- [x] 1-3: tracker: search circle grows for at most 0.6 s (was 1 s) at 30 cm/s (was 40); another robot's zone
      1.1 robot widths (was 0.8); pieces within 0.8 robot widths merged; a held robot's floor never learned within
      1.35 robot widths; each leg tells the tracker where the robot can be (turn: 7 cm; drive d: d + 8 cm)
- [ ] Left by the x-board (footage 10:40:18-10:41) is still unseen most of the time: its outline joins the board's
      edge and fails the shape check. Next: a shape check that allows for a board beside it.
- [ ] 9 (the bad find check), 8
- [ ] Nick's ideas: front-sensor board check while driving; robot-to-robot check; finder robot sweep; the lost
      robot does a find to rescue itself; second camera (after the overhead tracker is solid); colour sensor (low)
- [ ] Generic: the same logic for other equipment and sensors

## Before the next session
Nick uploads firmware 2026-10-22b to all three robots (same steps as this morning: Tools > Port, each robot).
