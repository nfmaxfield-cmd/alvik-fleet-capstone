# E02 accuracy session: step by step

About 2 hours of runs plus about 30 minutes of setup. Three windows on the laptop: **GitHub Desktop**,
**Arduino IDE**, and **PowerShell** with two tabs: **PI** (connected to the Raspberry Pi) and
**LAPTOP** (in the project folder). One command per box: type it, press Enter.

## Before you start (at the board)
1. Charge all three robots fully.
2. Boards along the **far x edge (top of the overhead picture)** and the **far y edge (left of the
   picture)**, meeting at the far corner; cement blocks behind them so a bump can't move them. The
   other two edges stay open.
3. Nothing else on the plywood. Robots anywhere on the board, switched on.

## 1. Get the new code (LAPTOP)
GitHub Desktop: **Fetch origin**, then **Pull origin**.

## 2. Load the new firmware on each robot (Arduino IDE)
1. File > Open: `C:\Users\nfmax\Documents\GitHub\alvik-fleet-capstone\firmware\home_lab\home_lab.ino`.
2. Tools > Port: pick **Left at ...** (the network port).
3. Click **Upload** (the right arrow). Type the upload password when asked. Wait for "Done uploading".
4. Repeat 2-3 for **Center** and **Right**.
The robot must be standing still. If a network port doesn't show, plug that robot in by USB and pick
its USB port instead.

## 3. Start the recording (PI tab)
```
ssh st1maxfini@st1maxfini.local
```
```
cd ~/alvik-fleet-capstone && git pull
```
```
v4l2-ctl -d /dev/video2 -c auto_exposure=3 -c white_balance_automatic=1
```
```
python3 tools/pi/camrec.py --device 0,2 --names overhead,side --fps 25 --label E02 --no-lock
```
You should see both cameras start and "recording". Leave this tab alone until the end.

## 4. Start the runner (LAPTOP)
In File Explorer, double-click `run-e02.bat` in the project folder. Answer its questions:
the Pi address is `st1maxfini.local`; the boards are on the **top** and **left** edges of the picture.
It then checks everything in plain words: both cameras, the floor calibration (it draws the board
outline on the picture: type **y** if it lines up with the plywood's edges), the boards, the robots,
the edge guard, then one find per robot, then the 2-minute naming check, then the runs.

## 5. Watch the first 10 minutes
The edge guard's sensitivity has never been measured on a real open edge. Stay near the board for
the first few runs; if a robot looks like it will leave the board, pick it up and press **Ctrl + C**
in the LAPTOP tab (everything is saved; `run-e02.bat` offers to carry on). After that you can leave:
if it needs you, it says exactly what to do, and after a wait it puts that robot aside and carries on.

## 6. The end
The runner says when it's done. Then in the PI tab press **Ctrl + C** to stop the recording, and copy
the recording and the logs as before (or upload them to a GitHub release); the analysis runs from them.

If something odd happens, take a screenshot of the LAPTOP tab and send it.
