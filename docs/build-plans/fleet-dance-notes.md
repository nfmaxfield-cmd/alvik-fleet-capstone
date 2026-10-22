# Notes for a fleet dance (8+ robots): what the 3-robot dance taught (2026-10-22)

Not built yet (Nick, 2026-10-22: build the 3-robot dance now, the fleet one next semester). The 3-robot dance is
`tools/experiment/e02_dance.py` (composer, checker, music, execution, page) and `tools/experiment/e02_multitrack.py`
(every robot followed at once). This file is what will matter when there are 8 or more.

## Conclusions up front
1. **Space is the hard limit, not software.** With the current conservative gap (12 cm between body circles,
   default until precision is proven), each robot needs a disc about 26 cm across to itself while passing others.
   The 122 x 122 cm board has a usable box of about 87 x 87 cm (open-edge and board margins taken off). Three robots
   fit with room to move; 8 would cover ~55% of the usable area before anyone moves. A fleet dance needs either a
   proven gap near 4-5 cm (then ~18 cm discs), a bigger floor, or both. Rough capacity for a dance that still
   moves: usable area / (3 x disc area) => 87 x 87 board: ~4 robots at a 12 cm gap, ~7 at a 5 cm gap.
2. **Timing is per robot, so it scales; the radio is the first shared thing to watch.**
3. **The greedy "go when it is safe" planner will not scale as is**: it plans robot by robot; with 8+ it gets slow
   and boxes robots in. Formation-based choreography (everyone on rings / grids, moving in lockstep) does scale.

## Tracking (e02_multitrack.py)
- Identity by prediction (commanded legs + LEG records), assignment by least total distance (scipy's
  `linear_sum_assignment`; a greedy fallback is there without scipy). Cost per frame grows with robots x outlines:
  fine at 8-20.
- Merged outlines: two robots closer than ~2 cm in the picture are one outline: both are flagged unsure and keep
  their predictions. With a fleet, close passes happen many times a minute: the supervisor stops the dance when a
  robot stays unsure 1.5 s. At small gaps (< 5 cm) expect more merges; splitting a merged outline using the two
  predictions (a two-centre fit inside the outline) is the next step.
- Contested outlines (equally near two predictions) are flagged too: with many robots in lockstep formations
  (everyone moving the same way at the same speed), predictions can be equally good for neighbours if a whole
  formation is shifted by a common error. Mitigation: stagger starts by a quarter beat inside formations; or give
  each robot a distinct light pattern (firmware: a non-blocking `led` command, see below) as a second check.
- Floor memory: robots cover more of the floor; the "don't learn here" discs grow for unsure robots. With 8+ the
  share of floor seen per frame drops: the floor memory should be taken before the robots enter (they are put on
  one by one, or the floor picture is taken with the robots parked along one edge).
- Camera: 2.34 px/cm overhead at ~15 fps; a robot is ~22 px across. A bigger floor needs a second camera or a
  wider lens; multi-camera tracking needs one shared floor frame (the studio's calibration per camera) and a
  hand-over between cameras (a robot seen by both is matched by prediction, as now).
- CPU: one tracker per frame on a laptop CPU was well under one core for 3 robots. Profile at 8 (the outline
  extraction is per frame, not per robot, so it should hold).

## Planning (e02_dance.py)
- The checker samples every robot's commanded motion every 0.05 s and checks every pair: cost ~ pairs x samples
  (28 pairs at 8 robots, 120 at 16): still well under a second per check; the robust check (40 random timing /
  speed draws) multiplies that by 40: about 10-30 s at 16 robots. Fine offline; use a spatial grid if needed.
- The "safe" search places a move at the first half beat where it keeps the gap to every robot already planned.
  It is greedy: order matters and later robots can be boxed in by earlier ones standing still. For a fleet: plan
  formations (rings, lines, grids) where every robot does the same move at the same beat; check them as a whole;
  use the safe search only for small solo moments.
- The dance ring: every figure starts and ends on a ring round the middle, so figures chain without long
  transitions. For a fleet: concentric rings, or a grid of slots, with every robot's slot known at every bar.
- Assigning robots to start spots: least total driving (permutations up to 7 robots, Hungarian method above).
- Gap rule: gap = K_GAP (2) x e95, the proven 95th-percentile error (upper 95% confidence bound from >= 59 measured
  dance moves). The same rule holds for any number of robots, but with N robots there are ~N^2/2 pairs and many
  more close passes per dance: the chance of SOME contact grows with the number of close passes. If a fleet dance
  has ~10x the close passes, consider K_GAP 2.5.

## Timing
- Every leg is sent early by the radio's one-way delay (measured from each leg's `queued` answer) plus the
  firmware's 0.35 s stand-still before an idle leg (the edge guard learning the floor). Each robot is independent:
  scales with N.
- `identify` blocks a robot for 3 s and only flashes at 2 Hz: at 120 BPM that is one flash per beat. Anything else
  (other tempos, colours, flashing while moving) needs the firmware change below.
- The music is played by the browser against the laptop's clock (the page measures its offset from the laptop by
  round trips, keeps the best, and re-seeks the audio if it drifts over 50 ms). For a remote screen (the
  university TV), the same works over the internet; the audio's own output delay is subtracted
  (AudioContext.outputLatency).

## Radio (UDP, one laptop)
- Per robot per move: one `fix`, one leg, and its LEG answers (queued, started, done). At one move per ~2 s per
  robot, 16 robots is ~25 packets a second out and ~40 in: small. The risk is bursts: synchronized figures send N
  legs in the same few ms. Packet loss is handled per leg (resend once after 0.45 s with the same id; the firmware
  answers `dup`), but a resend arrives 0.45 s late and that robot is late for its beat. For a fleet: stagger sends
  by a few ms, and measure loss per robot (the e02 setup's `status` answers already show WiFi dBm).
- `stop` to all robots is sent three times; with N robots that is 3N packets in ~0.3 s: fine.
- The 2.4 GHz WiFi in the basement may not carry 16 robots reliably; a dedicated access point is likely needed.

## Space and safety
- Open edges: kept at the gap (as if the edge were another robot); blocked boards: 3 cm, and no leg may end
  where the front sensor would stop it (the checker models the firmware's "rest of the leg + 2 cm" rule).
- The supervisor stops ALL robots when any pair is predicted (now and 0.6 s ahead) closer than the hard minimum
  (= e95), when a robot is about to come within 9 cm of an open edge, or when the camera has been unsure of a robot
  for 1.5 s. With a fleet, one unsure robot stops everyone: a fleet supervisor should stop only the robots near
  the unsure one (within the distance they can cover in 2 s) and let the rest finish the bar.
- Parking afterwards is one robot at a time (the E02 runner): ~1 min per robot. With 8+ robots, park in
  formation instead (every robot to its slot at once, the same checker).

## Firmware wishes (not made)
- `led <r> <g> <b> [ms]` (or `pulse <ms>`): non-blocking, works while a leg runs; per-robot colours; flashes on any
  tempo. Also gives the tracker a second identity check (each robot's own colour or pattern).
- An idle leg's 0.35 s stand-still (edge_learn_ms 300 + 50) could be skipped when the robot stood still on the same
  plywood within the last few seconds: tighter timing and ~10 s more dance per minute.
- Optional: a time-stamped leg (`go <cm> <id> at <robot ms>`) so a leg starts on the robot's clock instead of on
  arrival: removes radio jitter from the timing entirely.
