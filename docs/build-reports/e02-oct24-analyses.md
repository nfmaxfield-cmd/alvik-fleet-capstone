# Four checks on the 2026-10-23 data (2026-10-24, no robots)

Nick asked for these after the summary review. The methods are written here so they can be repeated; the
scripts are in tools/experiment/analyses_oct24/.

## 1. Did the camera move? (the top board's line walked 1 cm outward over the day)

**Method.** The morning recording's picture at 08:50 is the reference. Every other picture (morning recording at
09:20 / 10:00 / 10:30 / 10:55; the afternoon, evening and night sessions' snapshots on the laptop) is aligned to it
with OpenCV's ECC (a turn and a shift), using only what is outside the plywood (the walls, the shelf, the level),
so robots and overlays do not count.

| When | Picture shift (x, y px) | Turn (deg) |
|---|---|---|
| 09:20 | -0.6, +0.1 | 0.007 |
| 10:00 | -0.4, -0.4 | 0.008 |
| 10:30 | -0.3, -0.6 | 0.005 |
| 10:55 | -0.2, -0.8 | 0.005 |
| 11:50-12:25 (session 2) | 0.0, -1.0 to -1.2 | 0.01 |
| 18:33-18:37 | +0.2, -0.3 to -0.6 | 0.005 |
| 19:14 | -0.1, -1.3 | 0.017 |
| 19:58 | +0.2, -2.0 | 0.035 |
| 22:25-22:45 | +0.4 to +0.6, -1.8 to -1.9 | 0.04 |

**Result.** The board did not move; the camera's picture did, by up to 2 pixels (about 1 cm on the board near the
top board), creeping while sessions ran (19:14 to 19:58: -1.3 to -2.0 px) and partly coming back between them
(18:33: -0.5 px). That matches the 1 cm walk of the top board's line (123.8 cm in the morning, 124.9 in the
evening). Likely a slow sag or warming of the camera's mount; the program's line measurement was right about the
picture it was given.

**Plan.** A camera-drift watch: the runner keeps the session's first picture and aligns each new one to it the
same way (the background outside the plywood), corrects positions by the shift, and says so when it grows past a
limit (1 px chosen; DOE). Cheap (a few ms a second) and needs nothing from Nick.

## 2. How well the camera sees a robot, place by place (morning footage)

**Method.** One picture every 4 s of the morning recording (1,455 robot sightings). At each robot's tracked
position (projected with session 1's hand-clicked calibration): the robot's contrast against the floor around it,
its detail (the spread of brightness over the robot), the floor's brightness, and what the tracker was using then
(track.csv: both / lock-on only / detector only). Cells of 20 cm.

**Result.** The detail the camera sees on the robot predicts whether the lock-on tracker holds it: across the 25
cells, r = 0.73 between the robot's detail and the lock-on's share; the floor's brightness r = -0.67. Near the top
board (cells 60-100 cm along x, 100-120 cm along y) the floor is brightest (190-197), the robot shows little detail
(spread 12-21 against 45-60 near the open edges) and the lock-on holds 8-36% of the time (90%+ near the open
edges). The glare does not wash pixels out to white (under 0.2% of robot pixels at 250+); it flattens the robot's
look against a bright floor.

**Plan.** The camera-quality map built live from the calibration runs (robots drive the whole board while the
camera watches): the robot's detail and the floor's brightness per cell. Where it is poor the camera's
uncertainty is raised (KC weights it less; e02_camerr's sd), the tracker leans on the robot's own motion reports,
and parking spots avoid those cells.

## 3. Center's finds that were 13-24 cm off

**Method.** finds.csv (footage analysis) with the robots' own find.csv and bump.csv, and the camera fixes (fix.csv)
before each find.

**Result.** Not the pushes. The big errors come from Center's heading being 5-20 deg wrong before the find: the
heading correction at the first board and the position correction at the second agree almost perfectly
(r = -0.95 for Center; Left -0.43, Right -0.75). Center parks about 100 cm from the first board its find drives to (the top board), so 10 deg of
heading error becomes about 17 cm by the time it gets there; Left parks about 15 cm from it, Right about 45. Early in the
session the heading error came from a camera heading fix sent 2 s before the find (Center's -19.8 deg correction
followed a +19.4 deg camera heading change); later from heading creep during parking drives. The same finds met the
first board at a slant (squaring 3-7 deg, pushing 1.0-1.3 s instead of 0.5 s), and one failed ("still turning
after 1511 ms of pushing"). The find itself corrected all of them.

**Plan.** No camera heading fix just before a find unless the gyro agrees with it; park Center nearer the boards
(or give the find a heading check by the front sensor on its way); log the heading correction per find as a
quality measure.

## 4. Picture delay by phase

**Method.** perf.csv (one row a second) by the runner's phase, three sessions.

| Session | Phase | Seconds | Picture age, median | Worst a second, 95th pct | Seconds over 1 s late | Laptop work per picture | Pictures handled a second |
|---|---|---|---|---|---|---|---|
| Morning | route | 2,744 | 0.12 s | 0.30 s | 0.3% | 41 ms | 21 |
| Morning | parking | 2,353 | 0.13 | 0.29 | 0.0% | 51 | 18 |
| Morning | find | 1,890 | 0.12 | 0.28 | 0.2% | 42 | 21 |
| Afternoon (WiFi copy) | parking | 1,976 | 0.20 | 2.37 | 27% | 19 | 23 |
| Afternoon | finding a lost robot | 35 | 1.13 | 1.61 | 63% | 11 | 23 |
| Evening | route | 870 | 0.14 | 0.43 | 0.0% | 46 | 18 |
| Evening | parking | 1,004 | 0.15 | 0.54 | 2.1% | 46 | 18 |

**Result.** No phase causes delay: in the morning and the evening every phase had the same picture age. The
afternoon's late pictures came from the WiFi copy in every phase (the "finding" phase's 63% is 35 seconds that fell
in the copy's worst stretch). What is steady is the laptop's own work: 41-51 ms per picture with three robots
tracked, against a picture every 40 ms, so it handles 18-21 of the camera's 25 pictures a second.

**Plan.** Nothing to fix for delay beyond no copies during sessions. The Pi-run measures its own work per picture
(perf.csv the same), which says whether the Pi keeps up.
