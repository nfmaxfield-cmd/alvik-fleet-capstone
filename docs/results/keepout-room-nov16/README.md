# Room round the outlet plates (16 Nov 2026)

## Premise

At school on 16 Nov, Jay-Sun's Visit points runs drove into the outlet plates on the table. The site file has his
measured plates (zone 1 plate, middle outlet box, zone 4 plate), and Visit points planned round them, but with only the
robot's body and the planner's 1.5 cm safety margin. The table's edges get a margin from measured data; the plates did
not. A real robot is typically 1-2 cm from where it believes it is (Jay-Sun's 9 Nov tours: median 1.1 cm, worst 7.5 cm),
so a leg that grazes a plate often clips it. Nick asked for the fix for Visit points (free roam later).

## Data (simulator only: Mission Control `--dry --sim-room school`, two robots, `vp_shots.py`, `ko_measure.py`)

The same lists in both runs: Alpha P03, P10, P02, P08 (P03 -> P10 passes the zone 1 plate); Bravo P20, P21, P12, P11
(P20 -> P21 passes the middle box). Closest approach from the camera track (`closest_approach.txt`; the body is 9.5 cm
square: its side 4.75 cm and its corner 6.72 cm from its middle):

| Run | Closest robot middle to a plate | Its corner from the plate |
| --- | --- | --- |
| Before (MIRLT_KEEPOUT_ROOM=body) | 7.2 cm (Alpha, zone 1 plate) | about 0.5 cm |
| After (the new room, 4.1 cm more) | 9.0 cm (Bravo, zone 1 plate) | about 2.3 cm |

Both runs finished ("done"). The simulator restarts a robot on purpose (Alpha at 64 s); its points went to Bravo.

## What changed

- `production.py`: `keepout_room_cm` = the edge rule's own terms less the body the planner already counts: the
  measured track error (1.7 cm) + the stopping distance (2.4 cm, the largest measured) + the camera's edge bias (roof
  tags: 0) = 4.1 cm at school. The paths plan round each keep-out grown by that much (`widen_polygon`); the points stay
  at their stickers, and every school sticker keeps room to turn (test).
- `safety_group.py`: the Safety switch `MIRLT_KEEPOUT_ROOM` (auto: the new room; `body`: as before).
- `mission_control_table.html`: the measured plates drawn on the map in grey.
- Free roam is not changed (it still does not know about the plates; Nick: later).

## What's next

- The school Pi updated, then a Visit points run with legs past each plate.
- Free roam given the plates (Nick, later).

## Sweep 1: tests that may now be needed

- The real table: legs past each plate at 10 cm/s, with the closest approach read from the camera track.
- A site whose keep-outs sit close together, where the wider plates might leave no way between them.

## Sweep 2: possible inaccuracies

- One simulated run each way; the closest approach is from the camera track (about 1 cm noise), not the truth.
- The 4.1 cm rests on the track error measured at home (2,176 drives) and the stopping distances measured at home;
  the school table's own track error has not been measured. A designed experiment is not needed: the room follows those
  measurements, and the school's own track error can replace the home figure once it is measured.
- The plates' positions are Jay-Sun's tape measurements (3 Nov), typed into the site file.
