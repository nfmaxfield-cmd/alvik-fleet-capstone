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

## Each session

Robots on their parking spots, turned on, the camera aimed as usual. On the Pi (RealVNC, or SSH), in a terminal:
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
