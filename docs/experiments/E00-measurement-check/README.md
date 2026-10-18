# E0: Measurement check (tape rig)

Status: planned    Week: 5    Run dates:

## Question
Is the tape rig precise enough to be the ground truth?

## Hypothesis
Repeated measurements of the same robot placement agree within 0.5 cm SD in
position and 0.5 deg in heading (a tenth of the 5 cm target).

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | placement | 10 spots (P01-P10) across the rig |
| Dependent | x, y of the midpoint of the front and rear dots; heading | cm, deg |
| Held constant | measurer, rig, robot | Nick, the taped L, Left |

## The rig
1. Tape an L of two straight axes with a carpenter's square: x axis 200 cm
   along the direction of travel, y axis 100 cm to the left. Mark every 10 cm.
2. Put a thin strip of tape on the robot's front and rear faces, on its
   centerline.
3. To mark the robot's position: hold a small square flat on the floor, its
   upright edge against the front face at the tape strip, and pencil a dot on
   the floor. Same at the rear.
4. Measure each dot's x and y from the axes to 0.1 cm.
5. Position = midpoint of the two dots. Heading = direction from rear to front.

## Procedure
0. Film it: start `python3 camrec.py --label T0` on the Pi before the first
   placement and leave it running to the end (exposure and focus lock by
   default). Keep hands and the measuring square out of view for a second
   or two after each placement.
1. Place the robot by hand at each spot in the order on the sheet, mark and
   measure, and fill in the row.
2. Pass 2 repeats every spot in a new order. Lift the robot between readings
   and don't look at pass-1 numbers.
3. Camera: tape-measure the lens height above the floor (cm), and hang a plumb
   line from the lens: measure the floor x, y of the point under it.
4. Floor calibration, on the laptop with the recording copied over:
   `python tools/calibrate_floor.py <recording folder> --height-cm <h> --nadir <x>,<y> --sheet docs/experiments/E00-measurement-check/data/measurements.csv`
   In the page: Floor points: click the origin and the axes' 10 cm marks
   (Enter accepts the suggested next mark). Robot points: for each placement,
   find a frame where the robot sits still, click the centre of its top. Fix or
   untick red points, then Save calibration. It writes floor_calibration.json,
   floor_calibration_check.jpg (grid should sit on the tape) and, because the
   placements helped make it, one held-out calibration per placement row:
   score the camera on a row only with its file from floor_calibration_loro.csv.

## Trials
20 readings: 10 spots x 2 passes. Each pair's difference holds two
measurement errors, so one reading's SD = SD(differences) / sqrt(2).
Sheet: data/measurements.csv (seed 5). Fill the end columns (xf_cm..yr_cm); the start columns stay empty.

## Results
`python tools/tape_results.py docs/experiments/E00-measurement-check`

## Conclusion

## Deviations
