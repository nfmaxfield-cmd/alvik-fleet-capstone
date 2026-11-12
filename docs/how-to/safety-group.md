# The Safety group: every safety switch in one place (e18, Nick 12 Nov)

Every safety behaviour the fleet has (never still, the rejoin evidence rule, the creep, the edge behaviour, the
watchdog-related rules, the blind-drive allowance, the out-and-back identity drive, ...) is one switch in ONE table:
`tools/fleet/safety_group.py`. Each is **decided automatically by default**: nothing has to be set for a normal run,
and a new user never needs to look here. The old behaviour stays behind each switch (CLAUDE.md "keep features, switch
them off").

## Seeing what is in force

On the Pi, in the terminal (after `ssh st1maxfini@100.122.222.75` and `cd ~/alvik-fleet-capstone`):

```
python3 tools/fleet/safety_group.py show
```

The last line says either "every switch on its automatic default" or which ones are set by hand. Every run says the
same line at its start and writes the whole group into its `run.json` (free roam) or `run_plan_summary.json`
(missions).

## Changing one (only to compare with the old way)

Two ways; the first one set wins:

1. **For one run:** put it in front of the command, for example
   `MIRLT_NEVER_STILL=0 ./run-fleet-pi.sh freeroam 1`.
2. **For every run on this Pi:** Mission Control, the Run screen, **More**, then **Safety**. Each switch shows
   `auto`. Type the old value (the "Old way" column below) and press **Save safety settings**. It is kept in the Pi's
   `~/.config/mirlt/safety.json` and used by free roam, missions and calibration until you set it back to `auto`.

## The switches

| Switch | Automatic default | Old way | Used in | What it does | Since |
|---|---|---|---|---|---|
| `MIRLT_NEVER_STILL` | 1 | 0 | free roam, missions, calibration | Held, unsure, lost or needs-a-hand robots keep making small slow moves toward safety and being found (step and speed scaled by their doubt); 0: the e16 hold-and-wait everywhere | e17 (free roam), e18 (missions, calibration) |
| `MIRLT_HELD_NEVER_END` | 1 | 0 | missions | Every robot held never ends a mission by itself; 0: the 90 s wait and the keep-going cap | e16 |
| `MIRLT_EDGE_PARK` | 0 | 1 | free roam | 1: a robot held at an edge for 60 s is parked for the rest of the run (e12); 0: it stays in and rejoins | e13 |
| `MIRLT_PARK_GIVEUP` | 0 | 1 | calibration | 1: parking after a calibration gives up on a lost tag (e13); 0: never gives up | e14 |
| `MIRLT_CALIB_GIVEUP` | 0 | 1 | calibration | 1: a calibration whose roof tag is lost ends after 30 s (e12); 0: held, never gives up | e13 |
| `MIRLT_ASK_UNATTENDED` | 0 | 1 | free roam | 1: keeps asking a person when nobody answers; 0: decides by itself after 30 s | e16 |
| `MIRLT_ELIMINATE` | 1 | 0 | tracking layer | A steady sighting nobody else can claim names the one unaccounted robot (on probation) | e17 |
| `MIRLT_OUTBACK_ID` | 1 | 0 | tracking layer, free roam | Two or more robots unaccounted and the wiggle cannot tell them apart: one at a time drives a short slow straight out-and-back (2-4 cm, from the camera's measured noise) so the camera sees which sighting moved | e18 |
| `MIRLT_ELIM_PAIR_GUARD` | 1 | 0 | tracking layer | The elimination counts robots in a close pass as unaccounted too, and never gives a spot that lies on another robot's own pose (within the 6 cm fix gate) to an unaccounted robot (5 Nov 14:55: Right corrected onto Left's spot 56 cm away) | e18 (replay of all home runs) |
| `MIRLT_FAR_NAMED_CORRECT` | 1 | 0 | free roam | A standing robot the layer keeps naming whose own pose is beyond its firmware's fix gate (6 cm) for a second, the offset steady and no other robot on that spot: its pose is set from the camera (a correction, as after an elimination; 11 Nov 08:26: Center held 86 s at 31 cm, then a false REJOINED) | e18 (replay) |
| `MIRLT_RESEED_HEADING_KEEP` | 1 | 0 | free roam | A re-seed to a standing robot keeps its own heading unless the camera's heading is better evidence (a roof tag's 1-2 deg against its own doubt, grown 1.6 deg per 90 deg turned since its heading was last set; the replay: standing re-seeds jumped the heading 16-64 deg, worse in 4 of 6 checked) | e18 (replay) |
| `MIRLT_TRUST_FLEET` | 1 | 0 | free roam, missions | The blind-drive trust model is fitted to the whole fleet's pooled samples, updated live each run; a robot gets its own record only once its own samples differ (Mann-Whitney p < 0.01); curve samples count only in their own run (the replay: drift 0.06 cm/cm for all three robots, stable between runs; curve response moves by session). 0: each robot's own record, every run's curves | e18 (replay) |
| `MIRLT_CURVE_SCALE_SAVE` | 0 | 1 | free roam | Curve scales (arc_k_scale, arc_d_scale) measured in a run are logged, NOT saved on the robot, until Nick decides (the replay: the curve response moves by session for all robots together; Left's 1.0736 came from one session). 1: saved by the agreement rule (e17) | e18 (replay; Nick to decide) |
| `MIRLT_IDENTITY` | wiggle | predict | tracking layer | Close passes resolved by a wiggle (predict: by the prediction alone, before e09) | e09 |
| `MIRLT_GATE_RECORD` | 1 | 0 | tracking layer | Each robot's gate grows with its own measured drift record | e17 |
| `MIRLT_OWN_RESET_RULE` | velocity | speed | tracking layer | An own-pose re-set is found by its own velocity at any picture rate | e17 |
| `MIRLT_CAMERA_CORRECT` | 1 | 0 | free roam | A sighting named by elimination or a wiggle / out-and-back is a CORRECTION (pose set at once) | e17 |
| `MIRLT_RESEED_TURN` | 1 | 0 | free roam | A re-seed is projected to now and never lands mid-turn | e17 |
| `MIRLT_REJOIN_EVIDENCE` | 1 | 0 | free roam | 'Moved in' (REJOINED) only on evidence | e16 |
| `MIRLT_CAMERA_WATCH` | 1 | 0 | tracking layer | The camera's turn directions checked against the robots' own | e10 |
| `MIRLT_POSE_RULE` | widen | wait2s | free roam, missions | Own pose vs camera disagreement widens the doubt at once | e08 |
| `MIRLT_EDGE_RULE` | derived | fixed | free roam, missions, calibration | One derived edge clearance (body + doubt + stop + camera bias) on the whole swept path | e08 |
| `MIRLT_CAM_EDGE_BIAS` | on | off | free roam, missions | The lights camera's measured 5 cm edge bias added | e08 |
| `MIRLT_EDGE_BACKOUT` | 1 | 0 | free roam, missions | An edge stop reverses along the robot's own recorded track | e16 |
| `MIRLT_START_EDGE` | 1 | 0 | free roam | A robot put down at or over an edge is checked before it starts | e10 |
| `MIRLT_START_EDGE_MODE` | recover | wait | free roam | ... and frees itself first (wait: always waits for a person) | e10 |
| `MIRLT_MARK_UNC` | 1 | 0 | free roam | A line-sensor trip near an edge (with its doubt) is an edge, not a mark | e10 |
| `MIRLT_QUEUE_WATCH` | 1 | 0 | free roam | Queued curve pieces re-checked from where the robot really is | e10 |
| `MIRLT_PINNED_OBSTACLE` | 1 | 0 | free roam | A robot held at an edge is a fixed obstacle with its doubt | e16 |
| `MIRLT_CONTACT_WATCH` | 1 | 0 | free roam | Pushes and collisions noticed (camera and accelerometer) | e16 |
| `MIRLT_TILT_RULE` | logs | old | free roam, missions | Tilt stop 4 deg (from the 5-6 Nov logs); old: 5 deg | e08 |
| `MIRLT_KEEPOUT_RULE` | fade | permanent | free roam, missions | Sensor-only keep-outs fade after 15 s | e08 |
| `MIRLT_BLIND_RULE` | edge | cap | free roam, missions, calibration | A robot with no camera drives on while its whole possible error cannot reach an edge (cap: the 8 / 40 cm caps) | e13 |
| `MIRLT_BLIND_ALLOW` | 1 | 0 | free roam, missions | Each robot's own measured odometry record (cm per cm, deg per turn and per curve) predicts its error on a blind leg; a blind excursion is allowed only where that prediction keeps the margin from edges and robots with a way back; checked against the error found when it is seen again. 0: the fixed 29 Oct numbers (2 cm + 0.07 cm per cm) and no planned blind legs | e18 |
| `MIRLT_BLIND_BUBBLE` | fixed | record | free roam, missions | An UNPLANNED loss of the camera grows the robot's error bubble by the fixed 29 Oct numbers (as e13-e17), its own record's prediction logged beside it and checked when it is seen again; on a blind leg its record allowed, the record's own prediction. record: every loss by its own record (the e18 stress re-run: robots with 1 s late fixes and heading faults then stood up to 20 min) | e18 |
| `MIRLT_NOPIC_RULE` | risk | off | free roam, missions | No pictures: each robot's risk-based limit | e13 |
| `MIRLT_FIX_GAP_RULE` | derived | fixed | free roam, missions | How old a fix may be, from each robot's measured fix gap | e14 |
| `MIRLT_STAND_TRUST` | 1 | 0 | free roam | A standing robot keeps its fix (not lost while it stands) | e13 |
| `MIRLT_STAND_UNC` | 1 | 0 | free roam | ... and its doubt does not grow while it stands | e16 |
| `MIRLT_LATE_FIX` | 1 | 0 | free roam | Late fixes slow a drive to what its measured drift allows | e17 |
| `MIRLT_STRAY_RETURN` | 1 | 0 | free roam | A robot outside the readable area comes back first | e13 |
| `MIRLT_FRAME_WATCH` | 1 | 0 | free roam, missions, calibration | The bridge says when the camera stops sending pictures (watchdog) | e13 |
| `MIRLT_HEADING_CONFIRM` | 1 | 0 | free roam | After a heading re-set, a short crawl the camera measures | e17 |
| `MIRLT_HEADING_CLICKS` | 1 | 0 | free roam | After a click, a straight drive and a second click give the heading | e13 |
| `MIRLT_DOUBT_NEIGHBOUR` | 1 | 0 | free roam | A robot whose heading is in doubt beside another: the neighbour camera-confirmed -> wait for room (or creep straight away when even its worst heading cannot close the gap); not confirmed -> creep cautiously (half steps, slowest) away from where the neighbour was last believed to be. 0: e17 (creeps keep the reflex gap on the heading it has) | e18 |
| `MIRLT_LOST_PRIORITY` | 1 | 0 | free roam, missions | A lost robot (or one with no readable point) goes first to the nearest readable point nobody else uses; the others yield (free roam: it outranks them in the reflex and standing robots in its way make room; missions: right of way in the plan, a robot standing in its recovery path steps aside) | e18 |
| `MIRLT_EMPTY_TO_READABLE` | 1 | 0 | missions | A robot with no readable points goes to the nearest one nobody has | e17 |
| `MIRLT_ARC_SAVE_AT_END` | 1 | 0 | free roam | The curves' calibration saved at any run end (agreement rule) | e17 |
| `MIRLT_SAVE_CAL_AT_END` | 1 | 0 | free roam, missions | Every calibration correction a run measured (curves, straight drives, turns: each by the agreement rule; each robot's odometry record) saved at every ending: the time cap, Stop / Ctrl+C, an error (a checkpoint every minute covers a crash or power cut). 0: only the curves at a normal end (e17) | e18 |

## Numbers that are not switches

The safety numbers each switch uses (creep step 2-8 cm, the out-and-back's 2-4 cm, the trust model's prior) are listed
with "chosen / derived / measured" and a DOE note in the code next to them and in the build reports' Sweep 2.
