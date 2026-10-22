# E02 accuracy session: step by step

About 2 hours of runs plus about 20 minutes of setup and mapping, which you watch. Then you can leave.
Windows on the laptop: **GitHub Desktop**, and **PowerShell** with two tabs: **PI** (connected to the
Raspberry Pi) and **LAPTOP** (in the project folder). One command per box: type it (or paste it), press Enter.

What changed since the first try (2026-10-22 build, docs/build-plans/e02-fix-build.md): you no longer put robots
on particular spots. Put them anywhere on the open board; the runner finds each one by a small turn on its spot,
parks them itself (going round the others), and checks the camera against the robots' own wheels all the time.
New: the robots feel out the board first (mapping), which also measures the edge guard for the first time.

## Before you start (at the board)
1. Charge all three robots fully. The firmware is the same as last time (2026-10-21b): no upload needed.
2. Boards on the **top** and **left** edges of the overhead picture, cement blocks behind them. The other two
   edges stay open. Nothing else on the plywood.
3. Robots switched on, anywhere on the open part of the board, at least 30 cm (1 ft) from each other, the boards
   and the edges.

## 1. Get the new code
**GitHub Desktop:** Fetch origin, then Pull origin.

## 2. Start the recording (PI tab)
```
ssh st1maxfini@st1maxfini.local
```
```
cd ~/alvik-fleet-capstone && git pull
```
```
python3 tools/pi/camrec.py --device 0 --names overhead --fps 25 --label E02 --no-lock
```
Leave this tab alone until the end.

## 3. Start the runner (LAPTOP tab)
```
python tools\experiment\e02.py --pi st1maxfini.local --hours 1.75 --no-side --map --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253
```
Answers: **y** (the green outline sits on the board), **left** (the x-board), **top** (the y-board).
Then it works on its own; you watch:
1. **Finding** (about 20 s): each robot turns a little on its spot and back; it says where each one is.
2. **Parking** (about 2 minutes): each robot drives a short way (to see which way it faces), then to its
   corner, going round the others.
3. **Mapping** (about 6 minutes): one robot drives out from the middle of the board in 16 directions, stopping
   at each board (its front sensor) and each open edge (its edge sensors). **This is the first real test of the
   edge sensors.** Watch the open edges: if it looks like it will drive off, pick it up and press **Ctrl + C**.
   (It may only fall an inch, but tell me if it does.) It then says where it felt each side.
4. The find check, the 2-minute naming check, then the runs.

## 4. Watch the first 3 runs, then leave
If the first 3 runs go without a call for help, the rest should too. If something odd happens, take a
screenshot of the LAPTOP tab and send it. If a robot needs you, the runner says exactly what to do: put it
anywhere open on the board and press Enter; it finds it and parks it itself.

## 5. The end
**Start copying the footage straight away** (Nick, 2026-10-21): it takes 10+ minutes, so it runs while the
next steps are discussed. Claude gives the copy and upload steps first, before anything else.
1. PI tab: **Ctrl + C** stops the recording.
2. Copy the recording and the laptop's `logs\e02-...` folder to a GitHub release (steps as for e02-setup-oct21);
   if the video is over 2 GB, cut it first with `ffmpeg -ss <seconds> -i video.avi -c copy part.avi`.
