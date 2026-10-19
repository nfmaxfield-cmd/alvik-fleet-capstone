# Simulated cable-plant equipment (3D printed)

Four stylised machines from a cable plant, for the plywood board: landmarks the robots drive to, park beside and
measure with their front distance sensor, and objects the overhead camera learns to tell apart from robots.

| File | Machine | Size (mm) | Notes |
| --- | --- | --- | --- |
| `stl/extruder.stl` | extrusion line: barrel housing, feed hopper, control cabinet, die head | 162 x 40 x 75 | |
| `stl/strander.stl` | cabling machine: eight-sided cage on a base, end bearing blocks | 150 x 56 x 56 | |
| `stl/takeup_reel.stl` | take-up reel full of cable, on a base, with a drive box | 100 x 64 x 68 | |
| `stl/test_bay.stl` | L-shaped inspection station | 140 x 140 x 60 | the L shows its orientation from above |

`python hardware/factory/build.py` rebuilds them and checks each one:
- fits a 200 x 200 mm bed with room for a brim;
- prints as exported with no supports (every slope 45 degrees or steeper);
- the robot's front distance sensor (29.5 mm up) meets a wall at least 45 mm tall from every side
  (no low stretch wider than 20 mm);
- its outline from above is not close to a robot's 95 x 95 mm square.

Print: PLA, any colour except white (the robots' tops are white); a dark or strong colour reads best on the
plywood. 0.2 mm layers, 2-3 walls, 5-10% infill (lightning infill if the slicer has it): they only need to stand
still. Fix each to the board with a loop of tape or a dab of poster putty underneath, so a robot bumping it
doesn't move a landmark.
