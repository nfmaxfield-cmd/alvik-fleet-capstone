# E02 accuracy session: step by step

About 2 hours of runs plus about 20 minutes of setup and mapping, which you watch. Then you can leave.
Windows on the laptop: **GitHub Desktop**, and **PowerShell** with two tabs: **PI** (connected to the
Raspberry Pi) and **LAPTOP** (in the project folder). One command per box: type it (or paste it), press Enter.

What changed since the first try (2026-10-22 build, docs/build-plans/e02-fix-build.md): you no longer put robots
on particular spots. Put them anywhere on the open board; the runner finds each one by a small turn on its spot,
parks them itself (going round the others), and checks the camera against the robots' own wheels all the time.
New: the robots feel out the board first (mapping), which also measures the edge guard for the first time.

## Before you start (at the board)
1. Charge all three robots fully. Firmware: 2026-10-22b, already on all three (2026-10-23: no new upload).
2. Boards on the **top** and **left** edges of the overhead picture, cement blocks behind them. The other two
   edges stay open. Nothing else on the plywood.
3. Robots switched on, anywhere on the open part of the board, at least 30 cm (1 ft) from each other, the boards
   and the edges.

## 1. Get the new code
**GitHub Desktop:** Fetch origin, then Pull origin.

## 2. Start the recording (PI tab)
If the PI tab's line already starts with `st1maxfini@st1maxfini`, you are on the Pi: skip the ssh line.
```
ssh st1maxfini@st1maxfini.local
```
```
cd ~/alvik-fleet-capstone && git pull
```
Free the space of last night's two recordings (they are on GitHub, release e02-session-oct22-evening):
```
ls ~/recordings
```
You should see folders starting `20261022-19`. Then:
```
rm -r ~/recordings/20261022-19*
```
```
df -h ~
```
**Avail** should be about 6 GB (1.5 hours of video needs about 4.5 GB).
```
python3 tools/pi/camrec.py --device 0 --names overhead --fps 25 --label E02 --no-lock
```
Leave this tab alone until the end.

## 3. Start the runner (LAPTOP tab)
```
python tools\experiment\e02.py --pi st1maxfini.local --hours 1.5 --no-side --map --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253
```
Answers: **y** if the green outline sits on the board's edges. If it doesn't, type **n**: the runner then walks you
through making a new one in the studio (More -> Floor calibration -> rectangle) and waits. Board questions: type
**left**, then **top**, as before (just pressing Enter takes the word the question offers, which should be the same).
Then it works on its own. To see what it thinks, open **http://localhost:8090** in a browser on the laptop
(the camera picture with each robot's name, the one it is following in green, and its last lines). You watch:
1. **Finding** (about 20 s): each robot turns a little on its spot and back; it says where each one is.
2. **Parking** (about 2 minutes): each robot drives a short way (to see which way it faces), then to its
   corner, going round the others.
3. **Mapping** (about 6 minutes): one robot drives out from the middle of the board in 16 directions, stopping
   at each board (its front sensor) and each open edge (its edge sensors); it never aims at the corner where the two
   open edges meet. **Both edge sensors are on for the first time** (the line sensors, and the distance sensor's
   bottom zone as a backup). Watch the open edges: if it looks like it will drive off, pick it up and press **Ctrl + C**.
   (It may only fall an inch, but tell me if it does.) It then says where it felt each side.
4. The find check, the 2-minute naming check, then the runs.

**New since 2026-10-23: the tracker.** The camera finds a robot by its motion, then locks on to whatever that robot
looks like (keep its lights steady while driving: the runner does this) and follows it; the motion detector checks
it every 2 s and at every stop. In the live view, the robot it follows should stay ringed in green through its
turns, also beside the boards, where it lost them last night.

## 4. Watch the first 3 runs, then leave
If the first 3 runs go without a call for help, the rest should too. If something odd happens, take a
screenshot of the LAPTOP tab and send it. If a robot needs you, the runner says exactly what to do: put it
anywhere open on the board and press Enter; it finds it and parks it itself.

## 5. The end
The last lines say how the tracker did ("Tracker: position from the lock-on ...%"): screenshot them.
**Start copying the footage straight away** (Nick, 2026-10-21): it takes 10+ minutes, so it runs while the
next steps are discussed. Claude gives the copy and upload steps first, before anything else.
1. PI tab: **Ctrl + C** stops the recording.
2. Copy the recording and the laptop's `logs\e02-...` folder to a GitHub release (steps as for e02-setup-oct21);
   if the video is over 2 GB, cut it first with `ffmpeg -ss <seconds> -i video.avi -c copy part.avi`.

## After the session: the statistics
Where the files must be: the runner's session folder `logs\e02-<date-time>` (made by the runner, untouched), and
inside it a folder named **recording** holding the Pi's whole recording folder's contents (from
`~/recordings/<date-time>_E02`: `cam0\video.avi`, `cam0\frames.csv`, `cam0\events.csv`, `session.json`).

LAPTOP tab (or Claude, on the downloaded copy), with your session folder's name in place of `e02-20261022-093000`:
```
python tools\experiment\e02_analysis.py --session logs\e02-20261022-093000
```
You should see one line per run (`run 7ABD-4 Left U square end 3.21 cm ...`), then `wrote ...\analysis\report.md`.
It takes about 1 minute per 10 runs. If the recording is somewhere else, add `--recording <that folder>`.

It writes `logs\e02-<date-time>\analysis\`: **report.md** (premise, data, conclusion, what's next, the two sweeps),
**charts\** (each condition's spread), **summary.csv** (mean, SD, 95% ranges per condition, route and robot),
comparisons.csv (the tests), runs.csv (every run's measurements and notes), bumps.csv, tracks\ (the robot's
position in every frame of each run). Run it again any time; it overwrites the analysis folder.
