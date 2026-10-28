# Running a session from the Pi (Nick, 2026-10-24)

The laptop way (`run-e02.bat`) still works exactly as before. This is the other choice: the Raspberry Pi runs
everything (camera recorder and runner), so the laptop can be closed and Nick can leave the house. At the end the
Pi uploads the logs and the video to a GitHub release by itself.

## Once: the USB drive

The Pi's card is too small for a session's video (about 5 GB a session). Plug the 256 GB USB drive into one of the
Pi's **blue** USB ports (USB 3). The Pi opens it at `/media/<user>/<drive name>`.

It must be **exFAT** (most new large drives are) or ext4, not FAT32 (a FAT32 drive can't hold a file over 4 GB).
`run-e02-pi.sh` checks and stops with a message if it is FAT32. To reformat it as exFAT on the Pi (this erases it):
in the Pi's file manager, right-click the drive, **Format**, choose **exFAT**.

## Once: the GitHub token (so the Pi can upload)

On the laptop, in a browser:
1. github.com, top right picture, **Settings**, **Developer settings** (bottom left), **Personal access tokens**,
   **Fine-grained tokens**, **Generate new token**.
2. Name: `Pi upload`. Expiration: 90 days. Repository access: **Only select repositories**, pick
   `alvik-fleet-capstone`. Permissions, Repository permissions, **Contents**: **Read and write**.
3. **Generate token**, then **copy** it (it is shown once).

On the Pi, in a terminal:
```
cd ~/alvik-fleet-capstone && git pull
```
```
python3 tools/pi/upload_session.py --set-token
```
Paste the token (Ctrl+Shift+V in the Pi's terminal; it does not show) and press Enter. It should say
`Saved. The Pi can upload sessions ...`.

## Once: the start page (2026-10-26)

A web page on the Pi that starts and stops sessions, so no PowerShell is needed for a session. Install it once,
**after any running session has finished uploading** (the install updates the code on the Pi).

1. On the laptop, open **PowerShell** (Start menu, type `PowerShell`, press Enter).
2. In the PowerShell window, type this and press Enter:
   ```
   ssh st1maxfini@100.122.222.75
   ```
   The first time it may ask `Are you sure you want to continue connecting (yes/no)?`: type `yes` and press Enter.
   Then type the Pi's password (nothing shows while you type) and press Enter. You should see a line ending in
   `st1maxfini@...:~ $`.
3. Type this and press Enter:
   ```
   cd ~/alvik-fleet-capstone
   ```
4. Type this and press Enter (it fetches the newest code; a few lines scroll by):
   ```
   git pull
   ```
5. Type this and press Enter:
   ```
   bash tools/pi/install_launcher.sh
   ```
   - It lists the cameras; you should see `OK: overhead camera: USB 2.0 Camera: USB Camera at /dev/video...` and
     `OK: side camera: EMEET SmartCam C950 at /dev/video...` (a `MISSING:` line means that camera is unplugged; the
     install still goes on).
   - It asks `Choose a PIN for the start page (4 to 12 digits; it will not show)`: type a PIN, press Enter, type it
     again, press Enter.
   - It may ask `[sudo] password for st1maxfini:`: type the Pi's password, press Enter.
   - It ends with `DONE.` and the page's addresses, e.g. `Tailscale (anywhere): http://100.122.222.75:8091`.
6. Type `exit` and press Enter. Close PowerShell.
7. On the phone, open `http://100.122.222.75:8091`, type the PIN, press **Open**. Add it to the home screen
   (Safari: Share, Add to Home Screen; Chrome: the three dots, Add to Home screen).

Running the install again is safe: it keeps the PIN (press Enter) and restarts the page with the newest code.

**Using it.** Pick the options and press **Start**; a box asks to confirm. **Stop** (with its own confirm box) does
what Ctrl+C in the runner's window does: every robot stops, everything is saved, then the logs and video upload.
The page shows: idle / starting / running (with a button to the live page) / stopping / uploading (a progress bar),
the last session (runs done, how it ended, the upload link), free space, the cameras it found and which is which,
and the robots' batteries (from the running session, or **Ask the robots now** while idle). It will not start a
second session while one runs or uploads, or without the overhead camera.

**The one address (2026-10-28).** Port **8091** is the page to bookmark: `http://100.122.222.75:8091`. It has three
tabs, in Mission Control's look, and a bar at the top saying what is running on the Pi (an E02 session, a DOE loop,
Monday's mission, the ladder, Mission Control, the cameras):
- **Live lab**: during an E02 session, its live page (questions are asked there) and, one click over, the Lab page.
  During a DOE loop, Monday's mission (`./run-fleet-pi.sh monday`) or the ladder in tmux: that run's own screen (its
  last lines, every 2 s) and a **Stop** button (Ctrl+C in its window: every robot stops, then it saves and uploads).
  With the cameras on: their pictures behind a "Show the pictures" button. Nothing running: says so.
- **Mission Control**: the Mission Control page itself. If it is not open, an **Open Mission Control** button runs
  `./run-fleet-pi.sh mc` on the Pi in tmux session `mc` (so it keeps going when the browser closes; it starts the
  camera recorder too if nothing runs it). It opens beside a DOE loop or Monday's mission, but will not start a robot
  run of its own until that has ended (the simulator still runs). To close it: `./run-fleet-pi.sh mc-stop`.
- **Start/Stop**: this start page (also at `http://100.122.222.75:8091/start`). During a DOE loop or Monday's mission
  it says "A robot run is going", locks Start and offers Stop for that run.

The old addresses all still work: port **8090** shows the tabs while nothing runs and the live page during an E02
session; **8095** is Mission Control while it is open; **8000** / **8001** are the cameras' pictures.

**After a `git pull` on the Pi**, the page runs the new code only after a restart. In the SSH window to the Pi, type
this and press Enter (it may ask for the Pi's password):
```
sudo systemctl restart mirlt-launcher
```
Nothing should print. Then reload the page in the browser.

**Access.** Only from the home WiFi and Tailscale, and only with the PIN. The PIN is stored on the Pi only (as a
salted hash in `~/.config/mirlt/launcher.json`). To change it: `python3 tools/pi/launcher.py --set-pin` in a
terminal on the Pi, then `sudo systemctl restart mirlt-launcher`.

**If the page does not answer:** in a terminal on the Pi, `sudo systemctl status mirlt-launcher` (it should say
`active (running)`), and `sudo journalctl -u mirlt-launcher -n 30` for what it said.

## Cameras by name (2026-10-26)

After a Pi restart the camera numbers can swap (on 2026-10-26 the runner opened the side camera as the overhead).
`run-e02-pi.sh` and the start page now pick each camera by its **name**. `~/.config/mirlt/cameras.json` on the Pi
says which name is which (the install writes it):
```
{"overhead": "USB 2.0 Camera", "side": "EMEET SmartCam C950"}
```
`python3 tools/pi/camnames.py` shows what the Pi sees and which camera is which. If a camera is missing, the start
page says so in red and will not start; `run-e02-pi.sh` stops with the same message. The old way still works as an
override: `OVERHEAD_DEV=2 ./run-e02-pi.sh` (and `SIDE_DEV=0` with `--with-side`). A production session now passes
the overhead's number to the tracker too (before, it always opened `/dev/video0`).

## Each session

Robots on their parking spots, turned on, the camera aimed as usual. The easy way: the start page (above). The
terminal way, unchanged: on the Pi (RealVNC, or SSH), in a terminal:
```
cd ~/alvik-fleet-capstone && ./run-e02-pi.sh
```
Extra runner options go on the end, as on the laptop, e.g. `./run-e02-pi.sh --hours 1.5 --marks`.

It checks the tools (installs any missing, asking for the Pi's password once), finds the USB drive, starts the
recorder, waits 8 s, starts the runner and shows its window. Any first questions (an unfinished session to carry
on?) are answered there.

**Watch and answer from anywhere on the home WiFi:** open `http://st1maxfini.local:8090` on a phone or the laptop.
When the runner asks something, a yellow box shows the question; type the answer (or leave it empty for the
default) and press **Send**. A question nobody answers within 15 minutes takes its default (said in the log).

**Leave it running:** press **Ctrl+B**, then **D** (the windows keep running; the terminal can be closed).
**Look again later:** `tmux attach -t e02` in a terminal on the Pi.

When the runner finishes, its window stops the recorder, uploads everything to a GitHub release named
`e02-session-<date-time>` and says `All up: <link>`. If the upload is cut off, run it again (finished parts are
skipped):
```
python3 tools/pi/upload_session.py --recordings /media/$USER/*/recordings
```

## What is different on the Pi

- The runner reads the camera on the Pi itself (no WiFi in between), so pictures arrive sooner.
- The Pi does all the picture work. The laptop handled 18-21 of the camera's 25 pictures a second with three
  robots; the Pi's tracker alone handles about 25 a second on 30 s of footage, but with the recorder and the runner
  on the same Pi it may handle fewer. perf.csv shows it, as on the laptop. **The first Pi session should be short
  and watched** (`--hours 0.5`).
- The robots answer whichever computer last sent them a command: while it waits for them, the runner on the Pi calls
  them on the WiFi so they answer the Pi.

## When the session holds for you (the virtual border, a lost robot)

Every session (Pi or laptop), from 2026-10-24:
- **The virtual border.** A robot the camera sees within 15 cm of an open edge, moving towards it, stops every
  robot. Explore's forward crawls looking for the edge, and the edge find, are allowed past it (the robot's own
  edge guard stops those); anything backwards or at speed never is.
- **A lost robot.** A robot driving a leg that the camera has not seen for 1.5 s, or whose spot did not move the
  way its wheels say, stops every robot too.

Then the session waits for you, but not for ever (2026-10-26). If nobody answers a hold or a question for 30
minutes, or for 10 minutes once the planned time (`--hours`) is up, the session ends by itself: every robot stopped,
everything saved, then the upload runs and the recorder stops. The live page shows the time under the question
("No answer by 23:05 ends the session and uploads"). At the planned time plus 30 minutes there is a hard cap: no new
run starts, and the session ends. Questions that already take their default after 15 minutes still do. The options
`--answer-timeout-min`, `--after-end-timeout-min` and `--hard-cap-min` change the three times (0 switches one off).
The question is on the live page
(http://100.122.222.75:8090 over Tailscale); type one letter in the box and press the button:

| Answer | What happens |
|---|---|
| c | carry on (the lost robot is found again by a turn on its spot) |
| a | carry on, and from now on find lost robots again without asking (the border still asks) |
| o | leave that robot out: it stays where it is, the others carry on round it |
| s | stop the session: every robot stopped; `--resume` carries on later |

Every hold is in the log (HOLD), in e02_state.json ("holds": when, which robot, why, your answer, how long it
waited) and marked on the video.
